// SPDX-License-Identifier: GPL-2.0-only

#include "hz2w_device.hpp"

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

#if defined(__APPLE__) || defined(PX4_USERLAND)
#include <libusb.h>
#include <exe_path.h>
#endif
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#include "msg.h"
#include "util.hpp"

namespace px4 {
namespace {
	constexpr unsigned int kControlTimeoutMs = 1000;
	/* ファームウェアの送信は 4 KB 単位の転送で、機器側の書き込みを待つため長めに取る。 */
	constexpr unsigned int kFirmwareControlTimeoutMs = 3000;
	/* ベンダーのドライバーを usbmon で測った周期 */
	constexpr auto kCardStatePeriod = std::chrono::microseconds(HZ2W_KEEPALIVE_CARD_PERIOD_US);
	constexpr auto kWatchdogPeriod = std::chrono::microseconds(HZ2W_KEEPALIVE_WATCHDOG_PERIOD_US);
	/* 周期処理の連続失敗がこの回数に達したら、機器が止まったとみて利用者へ知らせる。 */
	constexpr unsigned int kKeepaliveFailureReport = 5;
}

Hz2wDevice::Hz2wDevice(const std::wstring &path, const DeviceDefinition &definition,
		       std::uintptr_t index, ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	/* 受信は未対応のため、受信機の定義は受け付けない (定義があると、TS が来ない受信機が選局できてしまう)。 */
	if (!definition.receivers.empty())
		throw DeviceError("HZ2W supports the card reader only; remove the receiver definitions.");
	const auto vendor = static_cast<unsigned long>(
		std::wcstoul(definition.configs.Get(L"VendorId", L"0").c_str(), nullptr, 0));
	const auto product = static_cast<unsigned long>(
		std::wcstoul(definition.configs.Get(L"ProductId", L"0").c_str(), nullptr, 0));
	if (vendor == HZ2W_USB_VENDOR_COLD && product == HZ2W_USB_PRODUCT_COLD)
		cold_ = true;
	else if (vendor != HZ2W_USB_VENDOR_WARM || product != HZ2W_USB_PRODUCT_WARM)
		throw DeviceError("HZ2W: unexpected USB ID in the device definition.");
}

Hz2wDevice::~Hz2wDevice()
{
	Term();
}

int Hz2wDevice::ControlCallback(void *context, u8 request_type, u8 request,
				u16 value, u16 index, void *buffer, u16 length)
{
	return static_cast<Hz2wDevice *>(context)->Control(request_type, request, value, index, buffer, length);
}

void Hz2wDevice::SleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

u64 Hz2wDevice::NowCallback(void *)
{
	return static_cast<u64>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
}

void Hz2wDevice::LogCallback(void *context, int level, const char *format, ...)
{
	auto *device = static_cast<Hz2wDevice *>(context);
	char text[256];
	va_list args;
	va_start(args, format);
	std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	if (level <= HZ2W_LOG_ERR)
		msg_err("HZ2W %s: %s\n", device->dev_.device_name, text);
	else if (level == HZ2W_LOG_INFO)
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "HZ2W %s: %s\n", device->dev_.device_name, text);
	else
		msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEVICE, "HZ2W %s: %s\n", device->dev_.device_name, text);
}

int Hz2wDevice::Control(u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length)
{
	if (!available_)
		return -ENODEV;
	if (length && !buffer)
		return -EINVAL;
	unsigned int timeout = request_type == HZ2W_REQ_OUT ? kFirmwareControlTimeoutMs : kControlTimeoutMs;
	/* カードの操作中 (bus_mutex_ を持つ呼び出し側) は、各転送を操作の残り時間で打ち切る。 */
	if (card_op_active_ && card_.deadline_ms) {
		const u64 now = NowCallback(nullptr);
		const u64 remaining = card_.deadline_ms > now ? card_.deadline_ms - now : 1;
		if (remaining < timeout)
			timeout = static_cast<unsigned int>(remaining);
	}
#if defined(__APPLE__) || defined(PX4_USERLAND)
	const int ret = libusb_control_transfer(usb_dev_.handle, request_type, request,
		value, index, static_cast<unsigned char *>(buffer), length, timeout);
	if (ret < 0) {
		if (ret == LIBUSB_ERROR_TIMEOUT)
			return -ETIMEDOUT;
		if (ret == LIBUSB_ERROR_NO_DEVICE)
			return -ENODEV;
		if (ret == LIBUSB_ERROR_PIPE)
			return -EPIPE;
		return -EIO;
	}
	return ret;
#else
	if (!control_event_ || !ResetEvent(control_event_))
		return -EIO;
	WINUSB_SETUP_PACKET setup = {};
	setup.RequestType = request_type;
	setup.Request = request;
	setup.Value = value;
	setup.Index = index;
	setup.Length = length;
	ULONG transferred = 0;
	OVERLAPPED overlapped = {};
	overlapped.hEvent = control_event_;
	if (!WinUsb_ControlTransfer(usb_dev_.winusb, setup, static_cast<PUCHAR>(buffer),
				    length, &transferred, &overlapped)) {
		DWORD error = GetLastError();
		if (error == ERROR_IO_PENDING) {
			const DWORD wait = WaitForSingleObject(overlapped.hEvent, timeout);
			if (wait == WAIT_TIMEOUT) {
				/* 転送を取り消し、完了するまで OVERLAPPED を保持する。 */
				if (!CancelIoEx(usb_dev_.dev, &overlapped) && GetLastError() != ERROR_NOT_FOUND)
					msg_warn("HZ2W: USB 制御転送の timeout 後、キャンセル完了を待っています\n");
				WaitForSingleObject(overlapped.hEvent, INFINITE);
				return -ETIMEDOUT;
			}
			if (wait != WAIT_OBJECT_0) {
				CancelIoEx(usb_dev_.dev, &overlapped);
				WaitForSingleObject(overlapped.hEvent, INFINITE);
				return -EIO;
			}
			if (!GetOverlappedResult(usb_dev_.dev, &overlapped, &transferred, FALSE))
				error = GetLastError();
			else
				error = ERROR_SUCCESS;
		}
		if (error == ERROR_SUCCESS)
			return static_cast<int>(transferred);
		if (error == ERROR_SEM_TIMEOUT)
			return -ETIMEDOUT;
		/* ファームウェアの送信後の再列挙で、機材が切断された扱いになる。 */
		if (error == ERROR_DEVICE_NOT_CONNECTED || error == ERROR_NO_SUCH_DEVICE ||
		    error == ERROR_FILE_NOT_FOUND || error == ERROR_INVALID_HANDLE)
			return -ENODEV;
		if (error == ERROR_OPERATION_ABORTED)
			return -ECANCELED;
		return -EIO;
	}
	return static_cast<int>(transferred);
#endif
}

/*
 * ファームウェアのファイルは利用者が用意する。設定の相対パスは実行ファイルの場所を基準にする。
 * 送信が済むと機器は約 4 秒後に再列挙し、新しい USB ID (04bb:0526) のデバイスとして改めて Init される。
 */
int Hz2wDevice::LoadFirmware()
{
	std::filesystem::path file(device_def_.configs.Get(L"Hz2wFirmware", L"asv5211-firmware.bin"));
	if (file.is_relative()) {
#if defined(__APPLE__) || defined(PX4_USERLAND)
		char exe_path[4096];
		if (px4::GetExecutablePath(exe_path, sizeof(exe_path)))
			file = std::filesystem::path(exe_path).parent_path() / file;
#elif defined(_WIN32) || defined(_WIN64)
		wchar_t exe_path[32768];
		const DWORD exe_len = GetModuleFileNameW(nullptr, exe_path, 32768);
		if (exe_len && exe_len < 32768)
			file = std::filesystem::path(exe_path).parent_path() / file;
#endif
	}
	std::ifstream stream(file, std::ios::binary);
	if (!stream) {
		msg_err("HZ2W: firmware file not found: %s (extract it with fwtool/asv5211fw)\n", file.string().c_str());
		return -ENOENT;
	}
	firmware_.assign(std::istreambuf_iterator<char>(stream), {});
	if (firmware_.size() != HZ2W_FIRMWARE_SIZE) {
		msg_err("HZ2W: unexpected firmware size %zu (expected %d)\n", firmware_.size(), HZ2W_FIRMWARE_SIZE);
		return -EINVAL;
	}
	/* 起動アドレスは firmware の版で決まる。主の版は 0x5583。別の版を使うときだけ設定で変える。 */
	const unsigned long boot = std::wcstoul(device_def_.configs.Get(L"Hz2wBootIndex", L"0x5583").c_str(), nullptr, 0);
	if (!boot || boot > 0xffff)
		return -EINVAL;
	const int ret = hz2w_fw_load(&bridge_, firmware_.data(), firmware_.size(), static_cast<u16>(boot));
	if (ret < 0) {
		/* 途中の転送の失敗は、再列挙とは区別して元のエラーを返す。 */
		msg_err("HZ2W: firmware transfer failed: %d\n", ret);
		return ret;
	}
	/* 全て成功したか、最後の転送の直後に機器が切り離された (HZ2W_FW_REENUMERATING)。どちらも warm の到着を待つ。 */
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "HZ2W: firmware sent, waiting for re-enumeration\n");
	return -ENODEV;
}

int Hz2wDevice::Init()
{
	if (initialized_)
		return -EALREADY;
#if defined(_WIN32) || defined(_WIN64)
	if (!control_event_) {
		control_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
		if (!control_event_)
			return -EIO;
	}
#endif
	static const struct hz2w_ops ops = {
		&Hz2wDevice::ControlCallback,
		&Hz2wDevice::SleepCallback,
		&Hz2wDevice::NowCallback,
		&Hz2wDevice::LogCallback,
	};
	hz2w_bridge_init(&bridge_, &ops, this);
	hz2w_card_init(&card_, &bridge_);
	if (cold_)
		return LoadFirmware();

	std::lock_guard<std::mutex> lock(bus_mutex_);
	int ret = hz2w_board_open(&bridge_);
	if (ret) {
		msg_err("HZ2W board setup failed: %d\n", ret);
		/* 起動の途中まで進むと、周期処理を始めない限り約 20 秒で機器が消えるため、閉じる手順で戻す。 */
		hz2w_board_close(&bridge_);
		return ret;
	}
	board_open_ = true;
	keepalive_stop_ = false;
	try {
		keepalive_thread_ = std::thread(&Hz2wDevice::KeepaliveThread, this);
	} catch (const std::system_error &) {
		hz2w_board_close(&bridge_);
		board_open_ = false;
		return -EAGAIN;
	}
	initialized_ = true;
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
		   "HZ2W ready (card reader only): host_index=%s\n", dev_.device_name);
	return 0;
}

void Hz2wDevice::KeepaliveThread()
{
	using clock = std::chrono::steady_clock;
	auto next_state = clock::now();
	auto next_watchdog = next_state;
	unsigned int failures = 0;
	for (;;) {
		{
			std::unique_lock<std::mutex> lock(keepalive_mutex_);
			keepalive_cond_.wait_until(lock, (std::min)(next_state, next_watchdog),
						   [this] { return keepalive_stop_; });
			if (keepalive_stop_)
				return;
		}
		if (!available_)
			return;
		int ret = 0;
		{
			std::lock_guard<std::mutex> lock(bus_mutex_);
			/* Term() が閉じる手順を済ませた後は何も送らない */
			if (!board_open_)
				return;
			const auto now = clock::now();
			if (now >= next_state) {
				ret = hz2w_keepalive_card_state(&bridge_);
				next_state = now + kCardStatePeriod;
			}
			/* watchdog はカード状態の成否と独立に送る (状態の要求が続けて失敗しても 0x22 を止めない) */
			if (ret != -ENODEV && now >= next_watchdog) {
				const int wd = hz2w_keepalive_watchdog(&bridge_);
				next_watchdog = now + kWatchdogPeriod;
				if (!ret || wd == -ENODEV)
					ret = wd;
			}
		}
		if (ret == -ENODEV) {
			available_ = false;
			card_open_ = false;
			return;
		}
		if (ret) {
			if (++failures == kKeepaliveFailureReport)
				msg_err("HZ2W: the keep-alive requests keep failing (%d); the device may disappear from USB\n", ret);
		} else {
			failures = 0;
		}
	}
}

void Hz2wDevice::StopKeepalive()
{
	{
		std::lock_guard<std::mutex> lock(keepalive_mutex_);
		keepalive_stop_ = true;
	}
	keepalive_cond_.notify_all();
	if (keepalive_thread_.joinable())
		keepalive_thread_.join();
}

void Hz2wDevice::Term()
{
	/* 周期処理を止める前に閉じる手順を送る (逆順だと約 20 秒で機器が USB から消える)。 */
	card_open_ = false;
	if (board_open_) {
		std::lock_guard<std::mutex> lock(bus_mutex_);
		if (available_) {
			hz2w_card_power_down(&card_);
			hz2w_board_close(&bridge_);
		}
		board_open_ = false;
	}
	StopKeepalive();
	initialized_ = false;
	available_ = false;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	hz2w_card_discard(&card_);
#if defined(_WIN32) || defined(_WIN64)
	if (control_event_) {
		CloseHandle(control_event_);
		control_event_ = nullptr;
	}
#endif
}

void Hz2wDevice::SetAvailability(bool available)
{
	available_ = available;
	if (!available) {
		card_open_ = false;
		/* 進行中のカード操作を回収してから、カードの状態を捨てる (Control は以後 -ENODEV を返す)。 */
		std::lock_guard<std::mutex> lock(bus_mutex_);
		hz2w_card_discard(&card_);
	}
}

ReceiverBase *Hz2wDevice::GetReceiver(int) const
{
	throw std::out_of_range("HZ2W has no receiver");
}

int Hz2wDevice::NoteResult(int ret)
{
	if (ret == -ENODEV) {
		available_ = false;
		card_open_ = false;
	}
	return ret;
}

int Hz2wDevice::OpenCard()
{
	if (!available_ || cold_ || !initialized_)
		return -ENODEV;
	/* CloseCard() の終了処理 (bus_mutex_ の中) が済むまでは開かない */
	std::lock_guard<std::mutex> lock(bus_mutex_);
	bool expected = false;
	return card_open_.compare_exchange_strong(expected, true) ? 0 : -EALREADY;
}

void Hz2wDevice::CloseCard()
{
	/* 電源断と状態の破棄が済むまで card_open_ を残し、次のセッションを開かせない。 */
	std::lock_guard<std::mutex> lock(bus_mutex_);
	if (!card_open_)
		return;
	if (available_) {
		const int ret = NoteResult(hz2w_card_power_down(&card_));
		if (ret)
			msg_warn("HZ2W %s: CloseCard: power down failed: %d\n", dev_.device_name, ret);
	}
	hz2w_card_discard(&card_);
	card_open_ = false;
}

int Hz2wDevice::DetectCard(bool &detected)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	const int ret = NoteResult(hz2w_card_detect(&card_, &detected));
	if (ret)
		msg_warn("HZ2W %s: DetectCard failed: %d\n", dev_.device_name, ret);
	return ret;
}

int Hz2wDevice::ResetCard()
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	card_op_active_ = true;
	const int ret = NoteResult(hz2w_card_reset(&card_));
	card_op_active_ = false;
	if (ret)
		msg_warn("HZ2W %s: ResetCard failed: %d\n", dev_.device_name, ret);
	return ret;
}

/*
 * ASIE5606B がボーレートを自動で扱う。ベンダーのドライバーも ATR の後に速度の設定を送らないため、
 * 要求は受け付けるだけで、機器へは何も送らない。
 */
int Hz2wDevice::SetCardBaudrate(std::uint32_t baudrate)
{
	if (!card_open_)
		return -ENODEV;
	if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400)
		return -EINVAL;
	return 0;
}

int Hz2wDevice::IsCardDataReady(bool &ready)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	return hz2w_card_rx_ready(&card_, &ready);
}

/* カードの送受信のバイト列をデバッグレベルで残す (B-CAS の応答には個体の情報が含まれるため、既定のレベルでは出さない)。 */
void Hz2wDevice::LogBytes(const char *what, const std::uint8_t *data, std::uint8_t length, std::uint8_t capacity)
{
	char text[3 * 255 + 1] = {};
	char *p = text;
	for (unsigned i = 0; i < length; ++i)
		p += std::snprintf(p, 4, "%02x ", data[i]);
	msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEVICE, "HZ2W %s: card %s (%u bytes, capacity %u): %s\n",
		   dev_.device_name, what, static_cast<unsigned>(length), static_cast<unsigned>(capacity), text);
}

int Hz2wDevice::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
{
	if (!card_open_)
		return -ENODEV;
	if (!buf)
		return -EINVAL;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	const std::uint8_t capacity = len;
	const int ret = hz2w_card_read(&card_, buf, &len);
	if (!ret && len)
		LogBytes("read", buf, len, capacity);
	return ret;
}

int Hz2wDevice::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
{
	if (!card_open_)
		return -ENODEV;
	if (!buf)
		return -EINVAL;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	LogBytes("write", buf, len, len);
	card_op_active_ = true;
	const int ret = NoteResult(hz2w_card_write(&card_, buf, len));
	card_op_active_ = false;
	if (ret)
		msg_warn("HZ2W %s: WriteCardData failed: %d\n", dev_.device_name, ret);
	return ret;
}

int Hz2wDevice::SetCardIoTimeout(std::uint32_t timeout_ms)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	hz2w_card_set_timeout(&card_, timeout_ms);
	return 0;
}

} // namespace px4
