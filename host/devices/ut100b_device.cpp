// SPDX-License-Identifier: GPL-2.0-only

#include "ut100b_device.hpp"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <new>
#include <stdexcept>
#include <system_error>

#if defined(__APPLE__) || defined(PX4_USERLAND)
#include <libusb.h>
#endif
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#include "msg.h"
#include "signal_stat.h"
#include "util.hpp"

namespace px4 {
namespace {
	constexpr unsigned int kControlTimeoutMs = 1000;
	/* ベンダーのドライバーを usbmon で測った周期 (HZ2W と同じ ASV5211) */
	constexpr auto kCardStatePeriod = std::chrono::microseconds(HZ2W_KEEPALIVE_CARD_PERIOD_US);
	constexpr auto kWatchdogPeriod = std::chrono::microseconds(HZ2W_KEEPALIVE_WATCHDOG_PERIOD_US);
	/* 周期処理の連続失敗がこの回数に達したら、機器が止まったとみて利用者へ知らせる。 */
	constexpr unsigned int kKeepaliveFailureReport = 5;
	constexpr unsigned int kStreamBufferSize = UT100B_URB_SIZE;
	/* 受信できる周波数 (kHz)。UHF 13〜62ch の中心は 473143〜767143。CATV を含めて、これより外は選局しない。 */
	constexpr std::uint32_t kMinFreqKhz = 111000;
	constexpr std::uint32_t kMaxFreqKhz = 770000;
}

class Ut100bDevice::Receiver final : public ReceiverBase {
public:
	explicit Receiver(Ut100bDevice &parent) : ReceiverBase(0), parent_(parent) {}
	~Receiver() override;
	int Open() override;
	void Close() override;
	int CheckLock(bool &locked) override;
	int SetLnbVoltage(std::int32_t voltage) override;
	int SetCapture(bool capture) override;
	int ReadStat(command::StatType type, std::int32_t &value) override;

protected:
	bool TuneOverride(std::uint32_t timeout, bool &result) override;
	int PrepareTune() override;
	int SetFrequency() override;
	int SetStreamId() override;

private:
	Ut100bDevice &parent_;
};

Ut100bDevice::Ut100bDevice(const std::wstring &path, const DeviceDefinition &definition,
			   std::uintptr_t index, ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	if (definition.receivers.size() != 1 || definition.receivers[0].index != 0 ||
	    definition.receivers[0].systems != SystemType::ISDB_T)
		throw DeviceError("UT100B requires exactly one ISDB-T receiver.");
	const auto vendor = static_cast<unsigned long>(
		std::wcstoul(definition.configs.Get(L"VendorId", L"0").c_str(), nullptr, 0));
	const auto product = static_cast<unsigned long>(
		std::wcstoul(definition.configs.Get(L"ProductId", L"0").c_str(), nullptr, 0));
	if (vendor != UT100B_USB_VENDOR || product != UT100B_USB_PRODUCT)
		throw DeviceError("UT100B: unexpected USB ID in the device definition.");
}

Ut100bDevice::~Ut100bDevice()
{
	Term();
}

int Ut100bDevice::ControlCallback(void *context, u8 request_type, u8 request,
				  u16 value, u16 index, void *buffer, u16 length)
{
	return static_cast<Ut100bDevice *>(context)->Control(request_type, request, value, index, buffer, length);
}

void Ut100bDevice::SleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

u64 Ut100bDevice::NowCallback(void *)
{
	return static_cast<u64>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
}

void Ut100bDevice::LogCallback(void *context, int level, const char *format, ...)
{
	auto *device = static_cast<Ut100bDevice *>(context);
	char text[256];
	va_list args;
	va_start(args, format);
	std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	if (level <= HZ2W_LOG_ERR)
		msg_err("UT100B %s: %s\n", device->dev_.device_name, text);
	else if (level == HZ2W_LOG_INFO)
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "UT100B %s: %s\n", device->dev_.device_name, text);
	else
		msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEVICE, "UT100B %s: %s\n", device->dev_.device_name, text);
}

int Ut100bDevice::Control(u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length)
{
	if (!available_)
		return -ENODEV;
	if (length && !buffer)
		return -EINVAL;
	unsigned int timeout = kControlTimeoutMs;
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
					msg_warn("UT100B: USB 制御転送の timeout 後、キャンセル完了を待っています\n");
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

int Ut100bDevice::Init()
{
	std::lock_guard<std::recursive_mutex> lock(mutex_);
	if (initialized_)
		return -EALREADY;
#if defined(_WIN32) || defined(_WIN64)
	if (!control_event_) {
		control_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
		if (!control_event_)
			return -EIO;
	}
#endif
	const auto &config = device_def_.configs;
	const unsigned int packets = util::wtoui(config.Get(L"ReceiverMaxPackets", L"16384"));
	max_urbs_ = util::wtoui(config.Get(L"Ut100bMaxUrbs", L"32"));
	lock_settle_ms_ = util::wtoui(config.Get(L"Ut100bLockSettleMs", L"300"));
	if (!packets || packets > (std::numeric_limits<std::uint32_t>::max)() / 188U ||
	    !max_urbs_ || max_urbs_ > 64)
		return -EINVAL;
	receiver_packets_ = packets;

	static const struct hz2w_ops ops = {
		&Ut100bDevice::ControlCallback,
		&Ut100bDevice::SleepCallback,
		&Ut100bDevice::NowCallback,
		&Ut100bDevice::LogCallback,
	};
	hz2w_bridge_init(&bridge_, &ops, this);
	try {
		aligner_.reset(new struct ut100b_aligner());
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
	ut100b_card_init(&card_, &bridge_);

	int ret;
	{
		std::lock_guard<std::mutex> bus_lock(bus_mutex_);
		ret = ut100b_board_open(&bridge_);
		if (ret) {
			msg_err("UT100B board setup failed: %d\n", ret);
			/* 起動の途中まで進むと、周期処理を始めない限り約 20 秒で機器が消えるため、閉じる手順で戻す。 */
			ut100b_board_close(&bridge_);
			return ret;
		}
		board_open_ = true;
	}
	keepalive_stop_ = false;
	try {
		keepalive_thread_ = std::thread(&Ut100bDevice::KeepaliveThread, this);
	} catch (const std::system_error &) {
		std::lock_guard<std::mutex> bus_lock(bus_mutex_);
		ut100b_board_close(&bridge_);
		board_open_ = false;
		return -EAGAIN;
	}

	stream_bus_.dev = &dev_;
	stream_bus_.type = ITEDTV_BUS_USB;
	stream_bus_.usb.dev = &usb_dev_;
	stream_bus_.usb.ctrl_timeout = kControlTimeoutMs;
	stream_bus_.usb.streaming.endpoint = UT100B_STREAM_ENDPOINT;
	stream_bus_.usb.streaming.urb_buffer_size = kStreamBufferSize;
	stream_bus_.usb.streaming.urb_num = max_urbs_;
	stream_bus_.usb.streaming.fail_on_error = true;
	ret = itedtv_bus_init(&stream_bus_);
	if (ret)
		return ret;
	bus_initialized_ = true;

	try {
		receiver_.reset(new Receiver(*this));
		buffer_ = receiver_->GetStreamBuffer();
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
	if (!buffer_->Alloc(static_cast<std::size_t>(receiver_packets_) * 188U))
		return -ENOMEM;
	const auto &definition = device_def_.receivers.front();
	command::ReceiverInfo info = {};
	wcscpy_s(info.device_name, device_def_.name.c_str());
	info.device_guid = device_def_.guid;
	wcscpy_s(info.receiver_name, definition.name.c_str());
	info.receiver_guid = definition.guid;
	info.systems = definition.systems;
	info.index = definition.index;
	if (!receiver_manager_.Register(info, receiver_.get()))
		return -EEXIST;
	initialized_ = true;
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
		   "UT100B ready: host_index=%s\n", dev_.device_name);
	return 0;
}

void Ut100bDevice::KeepaliveThread()
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
				msg_err("UT100B: the keep-alive requests keep failing (%d); the device may disappear from USB\n", ret);
		} else {
			failures = 0;
		}
	}
}

void Ut100bDevice::StopKeepalive()
{
	{
		std::lock_guard<std::mutex> lock(keepalive_mutex_);
		keepalive_stop_ = true;
	}
	keepalive_cond_.notify_all();
	if (keepalive_thread_.joinable())
		keepalive_thread_.join();
}

void Ut100bDevice::Term()
{
	if (initialized_.exchange(false) && receiver_)
		receiver_manager_.Unregister(receiver_.get());
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	std::unique_ptr<Receiver> receiver;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		/* TS の URB を止める (機器が残っている間に、受信を止めてから閉じる手順を送る)。 */
		StopUsbStream();
		if (buffer_started_ && buffer_)
			buffer_->Stop();
		buffer_started_ = false;
		receiver_open_ = false;
		card_open_ = false;
		failed_buffer = buffer_;
	}
	if (failed_buffer)
		failed_buffer->NotifyStreamFailure();
	/* 周期処理を止める前に閉じる手順を送る (逆順だと約 20 秒で機器が USB から消える)。 */
	if (board_open_) {
		std::lock_guard<std::mutex> lock(bus_mutex_);
		if (available_) {
			hz2w_card_power_down(&card_);
			ut100b_board_close(&bridge_);
		}
		board_open_ = false;
	}
	StopKeepalive();
	available_ = false;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		if (bus_initialized_) {
			itedtv_bus_term(&stream_bus_);
			bus_initialized_ = false;
		}
#if defined(_WIN32) || defined(_WIN64)
		if (control_event_) {
			CloseHandle(control_event_);
			control_event_ = nullptr;
		}
#endif
		/* 受信機の破棄は、クライアントの Close() を待つため、機材のロックを解いてから行う。 */
		receiver = std::move(receiver_);
		buffer_.reset();
	}
	{
		std::lock_guard<std::mutex> lock(bus_mutex_);
		hz2w_card_discard(&card_);
	}
	receiver.reset();
}

void Ut100bDevice::SetAvailability(bool available)
{
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		available_ = available;
		if (!available) {
			card_open_ = false;
			failed_buffer = buffer_;
		}
	}
	if (!available) {
		/* 進行中のカード操作を回収してから、カードの状態を捨てる (Control は以後 -ENODEV を返す)。 */
		std::lock_guard<std::mutex> lock(bus_mutex_);
		hz2w_card_discard(&card_);
	}
	if (failed_buffer)
		failed_buffer->NotifyStreamFailure();
}

ReceiverBase *Ut100bDevice::GetReceiver(int id) const
{
	if (id != 0)
		throw std::out_of_range("receiver id out of range");
	return receiver_.get();
}

int Ut100bDevice::NoteResult(int ret)
{
	if (ret == -ENODEV) {
		available_ = false;
		card_open_ = false;
	}
	return ret;
}

/* TS の受信を始める。受け始めのデータは、aligner が捨てる (機器の中に前回の終わりが残っている)。 */
int Ut100bDevice::StartUsbStream()
{
	if (!available_ || !bus_initialized_)
		return -ENODEV;
	if (streaming_)
		return stream_failed_ ? -EIO : 0;
	stream_failed_ = false;
	/* 選局の直後は、ロックから一定の時間が経つまで待つ (ロックしていない搬送波は待たない) */
	if (lock_seen_) {
		const auto until = lock_time_ + std::chrono::milliseconds(lock_settle_ms_);
		const auto now = std::chrono::steady_clock::now();
		if (now < until)
			std::this_thread::sleep_for(until - now);
	}
	/*
	 * 機器の TS は、読み出しの直前に流し始める。機器の中の FIFO は、流し始めた時点から溜まり、満杯のまま止まるため、
	 * 先に流すと、再開した直後に、前のチャンネルの TS と選局の途中の乱れた TS (TEI が付く) が先に出る。
	 */
	int ret;
	{
		std::lock_guard<std::mutex> bus_lock(bus_mutex_);
		ret = NoteResult(ut100b_stream_start(&bridge_));
	}
	if (ret)
		return ret;
	ut100b_aligner_init(aligner_.get());
	ret = itedtv_bus_start_streaming(&stream_bus_, StreamHandler, this);
	if (ret) {
		std::lock_guard<std::mutex> bus_lock(bus_mutex_);
		ut100b_stream_stop(&bridge_);
		return ret;
	}
	streaming_ = true;
	return 0;
}

/* 読み出しを止め、機器の TS も止める (止めている間に、機器の FIFO を満杯にしない)。 */
int Ut100bDevice::StopUsbStream()
{
	int ret = 0;
	if (bus_initialized_ && streaming_)
		ret = itedtv_bus_stop_streaming(&stream_bus_);
	const bool was_streaming = streaming_;
	streaming_ = false;
	if (was_streaming && available_) {
		std::lock_guard<std::mutex> bus_lock(bus_mutex_);
		const int step = NoteResult(ut100b_stream_stop(&bridge_));
		if (!ret)
			ret = step;
	}
	return ret;
}

int Ut100bDevice::StreamHandler(void *context, void *data, u32 size)
{
	auto &device = *static_cast<Ut100bDevice *>(context);
	if (!data || !size) {
		device.stream_failed_ = true;
		if (device.buffer_)
			device.buffer_->NotifyStreamFailure();
		return -EIO;
	}
	if (size > kStreamBufferSize) {
		device.stream_failed_ = true;
		if (device.buffer_)
			device.buffer_->NotifyStreamFailure();
		return -EOVERFLOW;
	}
	if (!device.buffer_ || !device.available_ || !device.aligner_)
		return -ENODEV;
	std::array<std::uint8_t, UT100B_ALIGNED_OUT_SIZE> output{};
	const std::size_t output_size = ut100b_aligner_feed(device.aligner_.get(),
		static_cast<const std::uint8_t *>(data), size, output.data());
	if (!output_size)
		return 0;
	std::size_t written = output_size;
	if (!device.buffer_->Write(output.data(), written)) {
		device.stream_failed_ = true;
		device.buffer_->NotifyStreamFailure();
		return -ENOBUFS;
	}
	device.buffer_->NotifyWrite();
	return 0;
}

/* ---- 内蔵カード (HZ2W と同じ。受信中も使える設定の hz2w_card を使う) ---- */

int Ut100bDevice::OpenCard()
{
	if (!available_ || !initialized_)
		return -ENODEV;
	/* CloseCard() の終了処理 (bus_mutex_ の中) が済むまでは開かない */
	std::lock_guard<std::mutex> lock(bus_mutex_);
	/* ロックを待つ間に、周期処理が機器の消失 (-ENODEV) を見て available_ を落とすことがある。消えた機器のセッションは開かない。 */
	if (!available_)
		return -ENODEV;
	bool expected = false;
	return card_open_.compare_exchange_strong(expected, true) ? 0 : -EALREADY;
}

void Ut100bDevice::CloseCard()
{
	/* 電源断と状態の破棄が済むまで card_open_ を残し、次のセッションを開かせない。 */
	std::lock_guard<std::mutex> lock(bus_mutex_);
	if (!card_open_)
		return;
	bool power_down_failed = false;
	if (available_) {
		const int ret = NoteResult(hz2w_card_power_down(&card_));
		if (ret) {
			msg_warn("UT100B %s: CloseCard: power down failed: %d\n", dev_.device_name, ret);
			/* 機器が消えたのでなければ、次の ResetCard が電源断をやり直せるよう powered を残す */
			power_down_failed = ret != -ENODEV;
		}
	}
	hz2w_card_discard(&card_);
	if (power_down_failed)
		card_.powered = true;
	card_open_ = false;
}

int Ut100bDevice::DetectCard(bool &detected)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	const int ret = NoteResult(hz2w_card_detect(&card_, &detected));
	if (ret)
		msg_warn("UT100B %s: DetectCard failed: %d\n", dev_.device_name, ret);
	return ret;
}

int Ut100bDevice::ResetCard()
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	card_op_active_ = true;
	const int ret = NoteResult(hz2w_card_reset(&card_));
	card_op_active_ = false;
	if (ret)
		msg_warn("UT100B %s: ResetCard failed: %d\n", dev_.device_name, ret);
	return ret;
}

/* ASIE5606 がボーレートを自動で扱う。要求は受け付けるだけで、機器へは何も送らない (HZ2W と同じ)。 */
int Ut100bDevice::SetCardBaudrate(std::uint32_t baudrate)
{
	if (!card_open_)
		return -ENODEV;
	if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400)
		return -EINVAL;
	return 0;
}

int Ut100bDevice::IsCardDataReady(bool &ready)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	return hz2w_card_rx_ready(&card_, &ready);
}

/* カードの送受信のバイト列をデバッグレベルで残す (B-CAS の応答には個体の情報が含まれるため、既定のレベルでは出さない)。 */
void Ut100bDevice::LogBytes(const char *what, const std::uint8_t *data, std::uint8_t length, std::uint8_t capacity)
{
	char text[3 * 255 + 1] = {};
	char *p = text;
	for (unsigned i = 0; i < length; ++i)
		p += std::snprintf(p, 4, "%02x ", data[i]);
	msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEVICE, "UT100B %s: card %s (%u bytes, capacity %u): %s\n",
		   dev_.device_name, what, static_cast<unsigned>(length), static_cast<unsigned>(capacity), text);
}

int Ut100bDevice::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
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

int Ut100bDevice::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
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
		msg_warn("UT100B %s: WriteCardData failed: %d\n", dev_.device_name, ret);
	return ret;
}

int Ut100bDevice::SetCardIoTimeout(std::uint32_t timeout_ms)
{
	if (!card_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(bus_mutex_);
	hz2w_card_set_timeout(&card_, timeout_ms);
	return 0;
}

/* ---- 受信機 ---- */

/*
 * クライアントの接続は、機材の抜去の後にも、受信機の Close() を呼ぶ。Close() が戻るまで受信機を破棄しないと、
 * 破棄済みのオブジェクトの仮想関数を呼ぶ。他の機種の受信機と同じ契約。
 */
Ut100bDevice::Receiver::~Receiver()
{
	std::unique_lock<std::recursive_mutex> lock(parent_.mutex_);
	parent_.close_cond_.wait(lock, [this] { return !parent_.client_open_; });
}

int Ut100bDevice::Receiver::Open()
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.board_open_)
		return -ENODEV;
	if (parent_.receiver_open_)
		return -EALREADY;
	parent_.receiver_open_ = true;
	parent_.client_open_ = true;
	return 0;
}

void Ut100bDevice::Receiver::Close()
{
	SetCapture(false);
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	parent_.receiver_open_ = false;
	parent_.client_open_ = false;
	parent_.close_cond_.notify_all();
}

int Ut100bDevice::Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return voltage == 0 ? 0 : -ENOSYS;
}

int Ut100bDevice::Receiver::PrepareTune()
{
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (params_.system != SystemType::ISDB_T)
		return -ENOSYS;
	if (params_.bandwidth && params_.bandwidth != 6)
		return -EINVAL;
	if (params_.freq < kMinFreqKhz || params_.freq > kMaxFreqKhz)
		return -EINVAL;
	return 0;
}

int Ut100bDevice::Receiver::SetFrequency()
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	const bool resume = parent_.streaming_;
	int ret = 0;
	if (resume)
		ret = parent_.StopUsbStream();
	if (!ret && parent_.buffer_ && !parent_.buffer_->Purge(parent_.deadline_))
		ret = -EIO;
	parent_.lock_seen_ = false;
	if (!ret) {
		std::lock_guard<std::mutex> bus_lock(parent_.bus_mutex_);
		ret = parent_.NoteResult(ut100b_tune(&parent_.bridge_, params_.freq));
	}
	if (resume) {
		if (parent_.defer_restart_) {
			/* TuneOverride が、ロックを確認し、復調が安定してから再開する */
			parent_.restart_pending_ = true;
		} else {
			const int restart_ret = parent_.StartUsbStream();
			if (!ret)
				ret = restart_ret;
		}
	}
	return ret;
}

int Ut100bDevice::Receiver::CheckLock(bool &locked)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	locked = false;
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	std::lock_guard<std::mutex> bus_lock(parent_.bus_mutex_);
	const int ret = parent_.NoteResult(ut100b_is_locked(&parent_.bridge_, &locked));
	if (!ret && locked && !parent_.lock_seen_) {
		parent_.lock_seen_ = true;
		parent_.lock_time_ = std::chrono::steady_clock::now();
	}
	return ret;
}

int Ut100bDevice::Receiver::SetCapture(bool capture)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (capture == parent_.buffer_started_) {
		/*
		 * 受信中の選局で USB の受信の再開に失敗すると、buffer_started_ は真のまま、streaming_ は偽になる。
		 * 停止を挟まない開始の再試行で、USB の受信を再開できるようにする。
		 */
		if (capture && !parent_.streaming_)
			return parent_.StartUsbStream();
		return 0;
	}
	if (capture) {
		parent_.buffer_->Start();
		parent_.buffer_started_ = true;
		const int ret = parent_.StartUsbStream();
		if (ret) {
			parent_.buffer_started_ = false;
			parent_.buffer_->Stop();
		}
		return ret;
	}
	const int ret = parent_.StopUsbStream();
	parent_.buffer_started_ = false;
	parent_.buffer_->Stop();
	return ret;
}

int Ut100bDevice::Receiver::ReadStat(command::StatType type, std::int32_t &value)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (type != command::StatType::CNR)
		return -ENOSYS;
	u32 cndat = 0;
	{
		std::lock_guard<std::mutex> bus_lock(parent_.bus_mutex_);
		const int ret = parent_.NoteResult(ut100b_read_cndat(&parent_.bridge_, &cndat));
		if (ret)
			return ret;
	}
	s32 mdb = 0;
	/* 未ロックのときは 0 が返り、換算できない */
	if (signal_cn_tc90522_t_mdb(cndat, &mdb))
		return -EAGAIN;
	value = mdb;
	return 0;
}

int Ut100bDevice::Receiver::SetStreamId()
{
	return -ENOSYS;
}

bool Ut100bDevice::Receiver::TuneOverride(std::uint32_t timeout, bool &result)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	result = false;
	const auto start = std::chrono::steady_clock::now();
	parent_.deadline_ = start + std::chrono::milliseconds(timeout);
	const std::uint64_t tune_start_ms = msg_now_ms();
	int ret = PrepareTune();
	parent_.defer_restart_ = true;
	parent_.restart_pending_ = false;
	if (!ret)
		ret = SetFrequency();
	parent_.defer_restart_ = false;
	const std::uint64_t freq_done_ms = msg_now_ms();
	bool locked = false;
	while (!ret && std::chrono::steady_clock::now() < parent_.deadline_) {
		ret = CheckLock(locked);
		if (ret || locked)
			break;
		const auto remaining = parent_.deadline_ - std::chrono::steady_clock::now();
		if (remaining > std::chrono::steady_clock::duration::zero())
			std::this_thread::sleep_for((std::min)(remaining,
				std::chrono::duration_cast<std::chrono::steady_clock::duration>(
					std::chrono::milliseconds(20))));
	}
	if (!ret && !locked)
		ret = -ETIMEDOUT;
	/*
	 * 選局で止めた受信を再開する。ロックを確認できたときは、StartUsbStream がロックから一定の時間、安定を待つ。
	 * ロックしなかったときも、受信中だった状態 (SetCapture) は保つ。
	 */
	if (parent_.restart_pending_) {
		parent_.restart_pending_ = false;
		const int restart_ret = parent_.StartUsbStream();
		if (!ret)
			ret = restart_ret;
	}
	result = !ret && locked;
	if (result) {
		stream_buf_->ArmTuneTiming(tune_start_ms, freq_done_ms - tune_start_ms,
					   msg_now_ms() - tune_start_ms,
					   params_.system, params_.freq);
	} else {
		stream_buf_->CancelTuneTiming();
		msg_log_at(ret == -ECANCELED ? PX4_LOG_DEBUG : PX4_LOG_WARN,
			   PX4_LOG_CAT_TS,
			   "UT100B tune failed: freq_khz=%u locked=%u result=%d\n",
			   params_.freq, locked ? 1U : 0U, ret);
	}
	parent_.deadline_ = (std::chrono::steady_clock::time_point::max)();
	return true;
}

} // namespace px4
