// SPDX-License-Identifier: GPL-2.0-only

#include "tbs5530_device.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <new>
#include <thread>

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
	constexpr unsigned int kControlTimeoutMs = 2000;
	/* ファームウェアの送信後、ブリッジが応答するまでの待ち (Linux 版と同じ 100 ms x 30)。 */
	constexpr unsigned int kFirmwareWaitMs = 100;
	constexpr unsigned int kFirmwareWaitTries = 30;
	constexpr std::uint8_t kStreamEndpoint = TBS5530_TS_ENDPOINT;
	/*
	 * 1 転送の長さは、バルク端点の最大パケット長 512 バイトの倍数 (既定 65536、Linux 版と同じ 64 KB) にする。
	 * 188 バイトの境界は TS の同期で合わせるため、188 の倍数にする必要はない。
	 */
	constexpr unsigned int kDefaultUrbBytes = 65536;
}

class Tbs5530Device::Receiver final : public ReceiverBase {
public:
	explicit Receiver(Tbs5530Device &parent) : ReceiverBase(0), parent_(parent) {}
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
	Tbs5530Device &parent_;
};

Tbs5530Device::Tbs5530Device(const std::wstring &path,
			     const DeviceDefinition &definition,
			     std::uintptr_t index,
			     ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	if (definition.receivers.size() != 1 || definition.receivers[0].index != 0 ||
	    definition.receivers[0].systems != SystemType::ISDB_T)
		throw DeviceError("TBS 5530 requires exactly one ISDB-T receiver.");
}

Tbs5530Device::~Tbs5530Device()
{
	Term();
}

int Tbs5530Device::ControlCallback(void *context, u8 request_type, u8 request,
				   u16 value, u16 index, void *buffer, u16 length)
{
	return static_cast<Tbs5530Device *>(context)->Control(request_type, request,
							      value, index, buffer, length);
}

void Tbs5530Device::SleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void Tbs5530Device::LogCallback(void *context, int level, const char *format, ...)
{
	auto *device = static_cast<Tbs5530Device *>(context);
	char text[256];
	va_list args;
	va_start(args, format);
	std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	if (level <= TBS5530_LOG_ERR)
		msg_err("TBS 5530 %s: %s", device->dev_.device_name, text);
	else if (level == TBS5530_LOG_INFO)
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "TBS 5530 %s: %s",
			   device->dev_.device_name, text);
	else
		msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEVICE, "TBS 5530 %s: %s",
			   device->dev_.device_name, text);
}

/* 選局の残り時間を超えて USB の待機をしない。期限のない操作は既定の待ち時間を使う。 */
int Tbs5530Device::Control(u8 request_type, u8 request, u16 value, u16 index,
			   void *buffer, u16 length)
{
	if (!available_)
		return -ENODEV;
	if (length && !buffer)
		return -EINVAL;
	unsigned int timeout = kControlTimeoutMs;
	if (deadline_ != (std::chrono::steady_clock::time_point::max)()) {
		const auto remaining = deadline_ - std::chrono::steady_clock::now();
		if (remaining <= std::chrono::steady_clock::duration::zero())
			return -ETIMEDOUT;
		const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
		timeout = static_cast<unsigned int>(std::min<std::int64_t>(kControlTimeoutMs,
			(std::max<std::int64_t>)(1, ms)));
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
					msg_warn("TBS 5530: USB 制御転送の timeout 後、キャンセル完了を待っています\n");
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
 * 通電直後の機材は、送信後に一度だけ再列挙して USB のハンドルが無効になる。その場合はこの Init は
 * 失敗で終わり、再列挙後に新しく見えたデバイスが改めて Init される。
 */
int Tbs5530Device::LoadFirmware()
{
	std::wstring path = device_def_.configs.Get(L"Tbs5530Firmware", L"dvb-usb-id5530.fw");
	std::filesystem::path file(path);
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
		msg_err("TBS 5530: firmware file not found: %s\n", file.string().c_str());
		return -ENOENT;
	}
	firmware_.assign(std::istreambuf_iterator<char>(stream), {});
	if (firmware_.empty() || firmware_.size() > 0x10000) {
		msg_err("TBS 5530: unexpected firmware size %zu\n", firmware_.size());
		return -EINVAL;
	}
	/* FX2 の端点の FIFO は前の利用者の終わり方で不整合のまま残ることがあるため、動作中でも毎回送り直す。 */
	int ret = tbs5530_fw_load(&bridge_, firmware_.data(), firmware_.size());
	if (ret == -ENODEV || ret == -EIO) {
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
			   "TBS 5530: firmware sent, the device is re-enumerating (ret=%d)\n", ret);
		return -ENODEV;
	}
	if (ret)
		return ret;
	for (unsigned int i = 0; i < kFirmwareWaitTries; ++i) {
		u8 mac[6];
		const int mac_ret = tbs5530_read_mac(&bridge_, mac);
		if (!mac_ret) {
			msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
				   "TBS 5530 firmware downloaded (%u ms)\n", i * kFirmwareWaitMs);
			return 0;
		}
		if (mac_ret == -ENODEV) {
			msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
				   "TBS 5530: the device re-enumerated after the firmware download\n");
			return -ENODEV;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kFirmwareWaitMs));
	}
	msg_err("TBS 5530: the bridge did not respond after the firmware download\n");
	return -ETIMEDOUT;
}

int Tbs5530Device::Init()
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
	receiver_packets_ = util::wtoui(config.Get(L"ReceiverMaxPackets", L"16384"));
	max_urbs_ = util::wtoui(config.Get(L"Tbs5530MaxUrbs", L"8"));
	urb_bytes_ = util::wtoui(config.Get(L"Tbs5530UrbBytes", std::to_wstring(kDefaultUrbBytes)));
	experimental_isdbc_ = util::wtob(config.Get(L"ExperimentalISDBC", L"false"));
	if (!receiver_packets_ || receiver_packets_ > (std::numeric_limits<std::uint32_t>::max)() / 188U ||
	    !max_urbs_ || max_urbs_ > 64 || urb_bytes_ < 4096 || urb_bytes_ > 1048576 || urb_bytes_ % 512)
		return -EINVAL;

	static const struct tbs5530_ops ops = {
		&Tbs5530Device::ControlCallback,
		&Tbs5530Device::SleepCallback,
		&Tbs5530Device::LogCallback,
	};
	tbs5530_bridge_init(&bridge_, &ops, this);
	int ret = LoadFirmware();
	if (ret)
		return ret;
#if defined(__APPLE__) || defined(PX4_USERLAND)
	/* ファームウェアが動いた後、ストリームを読む前にインターフェースを選び直す (しないとバルク読み出しが -EOVERFLOW になる)。 */
	int usb_ret = libusb_set_interface_alt_setting(usb_dev_.handle, 0, 0);
	if (usb_ret < 0) {
		msg_err("TBS 5530: set interface failed: %s\n", libusb_error_name(usb_ret));
		return -EIO;
	}
#else
	/* WinUSB の SET_INTERFACE。動作中の機材へファームウェアを送り直したときの、端点の FIFO の不整合を直す。 */
	if (!WinUsb_SetCurrentAlternateSetting(usb_dev_.winusb, 0)) {
		msg_err("TBS 5530: set interface failed: %lu\n", static_cast<unsigned long>(GetLastError()));
		return -EIO;
	}
#endif
	ret = tbs5530_demod_attach(&demod_, &bridge_);
	if (ret) {
		msg_err("TBS 5530: demodulator detection failed: %d (chip id 0x%03x)\n",
			ret, demod_.chip_id);
		return ret;
	}
	ret = tbs5530_select_stream(&bridge_, false);
	if (ret)
		return ret;
	ret = tbs5530_demod_init(&demod_);
	if (ret) {
		msg_err("TBS 5530: demodulator initialization failed: %d\n", ret);
		return ret;
	}
	demod_ready_ = true;

	stream_bus_.dev = &dev_;
	stream_bus_.type = ITEDTV_BUS_USB;
	stream_bus_.usb.dev = &usb_dev_;
	stream_bus_.usb.ctrl_timeout = kControlTimeoutMs;
	stream_bus_.usb.streaming.endpoint = kStreamEndpoint;
	stream_bus_.usb.streaming.urb_buffer_size = urb_bytes_;
	stream_bus_.usb.streaming.urb_num = max_urbs_;
	stream_bus_.usb.streaming.fail_on_error = true;
	ret = itedtv_bus_init(&stream_bus_);
	if (ret)
		return ret;
	bus_initialized_ = true;

	try {
		/* ストリームは 188 バイトの境界から始まるとは限らず、同期の確認に連続した数パケット分を保持する。 */
		synchronizer_.reset(new t230c::TsSynchronizer(static_cast<std::size_t>(urb_bytes_) * 2));
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
	/* ISDB-C は、ISDB-T の受信機の方式へ、ExperimentalISDBC の設定で加える (BonDriver の ini の System も合わせる) */
	if (experimental_isdbc_)
		info.systems |= SystemType::ISDB_C;
	info.index = definition.index;
	if (!receiver_manager_.Register(info, receiver_.get()))
		return -EEXIST;
	initialized_ = true;
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
		   "TBS 5530 ready: host_index=%s usb_serial=%ls chip=0x%03x\n",
		   dev_.device_name, usb_serial_number_.c_str(), demod_.chip_id);
	return 0;
}

void Tbs5530Device::Term()
{
	if (initialized_.exchange(false) && receiver_)
		receiver_manager_.Unregister(receiver_.get());
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	std::unique_ptr<Receiver> receiver;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		if (bus_initialized_ && streaming_)
			itedtv_bus_stop_streaming(&stream_bus_);
		streaming_ = false;
		if (available_ && demod_ready_)
			tbs5530_demod_sleep(&demod_);
		demod_ready_ = false;
		if (buffer_started_ && buffer_)
			buffer_->Stop();
		buffer_started_ = false;
		receiver_open_ = false;
		available_ = false;
		failed_buffer = buffer_;
	}
	if (failed_buffer)
		failed_buffer->NotifyStreamFailure();
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		if (bus_initialized_) {
			itedtv_bus_term(&stream_bus_);
			bus_initialized_ = false;
		}
		/* 受信機の破棄は、クライアントの Close() を待つため、機材のロックを解いてから行う。 */
		receiver = std::move(receiver_);
		buffer_.reset();
		synchronizer_.reset();
#if defined(_WIN32) || defined(_WIN64)
		if (control_event_) {
			CloseHandle(control_event_);
			control_event_ = nullptr;
		}
#endif
	}
	receiver.reset();
}

void Tbs5530Device::SetAvailability(bool available)
{
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		available_ = available;
		if (!available)
			failed_buffer = buffer_;
	}
	if (failed_buffer)
		failed_buffer->NotifyStreamFailure();
}

ReceiverBase *Tbs5530Device::GetReceiver(int id) const
{
	if (id != 0)
		throw std::out_of_range("receiver id out of range");
	return receiver_.get();
}

/* 前のチャンネルの端数と同期の状態を捨ててから、地上波の復調器の TS をバルク端点へ繋いで転送を始める。 */
int Tbs5530Device::StartUsbStream()
{
	if (!available_ || !demod_ready_ || !bus_initialized_)
		return -ENODEV;
	if (streaming_)
		return stream_failed_ ? -EIO : 0;
	stream_failed_ = false;
	synchronizer_->Reset();
	int ret = tbs5530_select_stream(&bridge_, false);
	if (ret)
		return ret;
	ret = itedtv_bus_start_streaming(&stream_bus_, StreamHandler, this);
	if (ret)
		return ret;
	streaming_ = true;
	return 0;
}

int Tbs5530Device::StopUsbStream()
{
	int ret = 0;
	if (bus_initialized_ && streaming_)
		ret = itedtv_bus_stop_streaming(&stream_bus_);
	streaming_ = false;
	if (synchronizer_)
		synchronizer_->Reset();
	return ret;
}

int Tbs5530Device::StreamHandler(void *context, void *data, std::uint32_t size)
{
	auto &device = *static_cast<Tbs5530Device *>(context);
	if (!data) {
		device.stream_failed_ = true;
		if (device.buffer_)
			device.buffer_->NotifyStreamFailure();
		return 0;
	}
	if (!size)
		return 0;
	if (!device.buffer_ || !device.available_ || !device.synchronizer_)
		return -ENODEV;
	bool failed = false;
	try {
		device.synchronizer_->Push(static_cast<std::uint8_t *>(data), size,
			[&device, &failed](const std::uint8_t *packet) {
				std::size_t length = 188;
				if (!device.buffer_->Write(packet, length))
					failed = true;
			});
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
	if (failed) {
		device.stream_failed_ = true;
		device.buffer_->NotifyStreamFailure();
		return -ENOBUFS;
	}
	device.buffer_->NotifyWrite();
	return 0;
}

/*
 * クライアントの接続は、機材の抜去の後にも、受信機の Close() を呼ぶ。Close() が戻るまで受信機を破棄しないと、
 * 破棄済みのオブジェクトの仮想関数を呼んで、DriverHost_PX4 が異常終了する。XZ、DT295 と同じ契約。
 */
Tbs5530Device::Receiver::~Receiver()
{
	std::unique_lock<std::recursive_mutex> lock(parent_.mutex_);
	parent_.close_cond_.wait(lock, [this] { return !parent_.client_open_; });
}

int Tbs5530Device::Receiver::Open()
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.demod_ready_)
		return -ENODEV;
	if (parent_.receiver_open_)
		return -EALREADY;
	parent_.receiver_open_ = true;
	parent_.client_open_ = true;
	return 0;
}

void Tbs5530Device::Receiver::Close()
{
	SetCapture(false);
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	/* 抜去や終了で receiver_open_ が先に偽になっていても、待っている受信機の破棄を必ず解除する。 */
	if (parent_.receiver_open_ && parent_.available_ && parent_.demod_ready_)
		tbs5530_demod_sleep(&parent_.demod_);
	parent_.receiver_open_ = false;
	parent_.client_open_ = false;
	parent_.close_cond_.notify_all();
}

/* 地上波専用機に LNB 給電はなく、OFF だけを受け付ける。 */
int Tbs5530Device::Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return voltage == 0 ? 0 : -ENOSYS;
}

int Tbs5530Device::Receiver::SetStreamId()
{
	return -ENOSYS;
}

int Tbs5530Device::Receiver::PrepareTune()
{
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (params_.system != SystemType::ISDB_T &&
	    !(params_.system == SystemType::ISDB_C && parent_.experimental_isdbc_))
		return -ENOSYS;
	if (params_.bandwidth && params_.bandwidth != 6)
		return -EINVAL;
	return 0;
}

/* 受信中は転送を止めてから選局し、前のチャンネルの TS を引き継がずに再開する。 */
int Tbs5530Device::Receiver::SetFrequency()
{
	const bool resume = parent_.streaming_;
	int ret = 0;
	if (resume)
		ret = parent_.StopUsbStream();
	if (!ret && parent_.buffer_ && !parent_.buffer_->Purge(parent_.deadline_))
		ret = -EIO;
	if (!ret) {
		/*
		 * ISDB-C の TSMF は共通のフィルターで選ぶ (ヘッダーのない搬送波は TS0 の指定でそのまま出力する)。
		 * 選局の前に転送を止めて配信バッファーを破棄しているため、前の搬送波の TS を捨てる数は 0 にする。
		 * ロックの後に、TuneOverride が TSMF の解析をやり直す。
		 */
		if (params_.system == SystemType::ISDB_C) {
			if (!stream_buf_->StartTsmf(params_.tsmf_stream, false, 0))
				ret = -ENOMEM;
		} else {
			stream_buf_->StopTsmf();
		}
	}
	if (!ret) {
		if (params_.system == SystemType::ISDB_C)
			ret = tbs5530_demod_tune_isdbc(&parent_.demod_, params_.freq);
		else
			ret = tbs5530_demod_tune_isdbt(&parent_.demod_, params_.freq);
	}
	if (resume) {
		const int restart_ret = parent_.StartUsbStream();
		if (!ret)
			ret = restart_ret;
	}
	return ret;
}

int Tbs5530Device::Receiver::CheckLock(bool &locked)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	locked = false;
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	u8 status = 0;
	const int ret = tbs5530_demod_read_status(&parent_.demod_, &status);
	if (!ret)
		locked = !!(status & TBS5530_STATUS_LOCK);
	return ret;
}

int Tbs5530Device::Receiver::SetCapture(bool capture)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return capture ? -ENODEV : 0;
	if (capture == parent_.buffer_started_)
		return 0;
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

/* CNR は 1/1000 dB (ロック時のみ有効)。 */
int Tbs5530Device::Receiver::ReadStat(command::StatType type, std::int32_t &value)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (type != command::StatType::CNR)
		return -ENOSYS;
	u8 status = 0;
	int ret = tbs5530_demod_read_status(&parent_.demod_, &status);
	if (ret)
		return ret;
	if (!(status & TBS5530_STATUS_LOCK))
		return -EAGAIN;
	s32 cnr_mdb = 0;
	ret = tbs5530_demod_read_cnr(&parent_.demod_, &cnr_mdb);
	if (ret)
		return ret;
	value = cnr_mdb;
	return 0;
}

/* 要求の開始から選局とロック確認まで同じ期限を使う。 */
bool Tbs5530Device::Receiver::TuneOverride(std::uint32_t timeout, bool &result)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	result = false;
	const auto start = std::chrono::steady_clock::now();
	parent_.deadline_ = start + std::chrono::milliseconds(timeout);
	const std::uint64_t tune_start_ms = msg_now_ms();
	int ret = PrepareTune();
	if (!ret)
		ret = SetFrequency();
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
	if (!ret && locked && params_.system == SystemType::ISDB_C) {
		/* ロックより前に届いた TS で判断しないよう、ロック後に TSMF の解析をやり直し、
		 * 指定した TS が見つかるか、ないと分かるまで同じ期限で待つ。 */
		stream_buf_->RestartTsmf();
		bool found = false;
		while (std::chrono::steady_clock::now() < parent_.deadline_) {
			const tsmf_filter_state state = stream_buf_->GetTsmfState();
			if (state != TSMF_FILTER_SEARCHING) {
				found = state == TSMF_FILTER_FOUND;
				if (!found)
					ret = -ENODATA;
				break;
			}
			const auto remaining = parent_.deadline_ - std::chrono::steady_clock::now();
			if (remaining > std::chrono::steady_clock::duration::zero())
				std::this_thread::sleep_for((std::min)(remaining,
					std::chrono::duration_cast<std::chrono::steady_clock::duration>(
						std::chrono::milliseconds(20))));
		}
		if (!found && !ret)
			ret = -ETIMEDOUT;
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
			   "TBS 5530 tune failed: freq_khz=%u locked=%u result=%d\n",
			   params_.freq, locked ? 1U : 0U, ret);
	}
	parent_.deadline_ = (std::chrono::steady_clock::time_point::max)();
	return true;
}

} // namespace px4
