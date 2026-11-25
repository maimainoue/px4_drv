// SPDX-License-Identifier: GPL-2.0-only

#include "xz_device.hpp"

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

#include "msg.h"
#include "util.hpp"

namespace px4 {
namespace {
	constexpr unsigned int kControlTimeoutMs = 1000;
	/* 非同期の読み出しは 8 KiB (16 パケット) の 16 本。ポンプのパケット数を 16 の倍数にして、読み出しの境界に合わせる。 */
	constexpr unsigned int kStreamBufferSize = XZ_PACKET_BYTES * XZ_PUMP_UNIT_PACKETS;
	constexpr std::uint8_t kStreamEndpoint = 0x81;
	constexpr unsigned int kDrainMaxReads = 16;
	constexpr unsigned int kDrainTimeoutMs = 20;
	constexpr int kPumpErrorLimit = 3;
	/* ロックしてから復調が安定するまでの待ち (GV-MVP/XZ2 の実機の試験で、ロック直後の 1 秒目だけに TEI が出た) */
	constexpr unsigned int kLockSettleMs = 500;
}

class XzDevice::Receiver final : public ReceiverBase {
public:
	explicit Receiver(XzDevice &parent) : ReceiverBase(0), parent_(parent) {}
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
	XzDevice &parent_;
};

XzDevice::XzDevice(const std::wstring &path,
		   const DeviceDefinition &definition,
		   std::uintptr_t index,
		   ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	if (definition.receivers.size() != 1 || definition.receivers[0].index != 0 ||
	    definition.receivers[0].systems != SystemType::ISDB_T)
		throw DeviceError("XZ requires exactly one ISDB-T receiver.");
}

XzDevice::~XzDevice()
{
	Term();
}

int XzDevice::Init()
{
	std::lock_guard<std::recursive_mutex> lock(mutex_);
	if (initialized_)
		return -EALREADY;
#if defined(_WIN32) || defined(_WIN64)
	control_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	if (!control_event_)
		return -EIO;
#endif
	const auto &config = device_def_.configs;
	const unsigned int packets = util::wtoui(config.Get(L"ReceiverMaxPackets", L"16384"));
	max_urbs_ = util::wtoui(config.Get(L"XzMaxUrbs", L"16"));
	if (!packets || packets > (std::numeric_limits<std::uint32_t>::max)() / 188U ||
	    !max_urbs_ || max_urbs_ > 64)
		return -EINVAL;
	receiver_packets_ = packets;

	static const struct xz_ops ops = {
		&XzDevice::ControlCallback,
		&XzDevice::SleepCallback,
		&XzDevice::NowCallback,
		&XzDevice::LogCallback,
	};
	static const struct torne_card_io_ops card_io_ops = {
		&XzDevice::CardRead32Callback,
		&XzDevice::CardWrite32Callback,
		&XzDevice::CardLockCallback,
		&XzDevice::CardUnlockCallback,
		&XzDevice::CardSleepCallback,
		&XzDevice::CardNowCallback,
	};
	xz_bridge_init(&bridge_, &ops, this);
	torne_card_port_init(&card_port_, &card_io_ops, this);

	/*
	 * 識別 (別の版の boot0、使用済みの機材には何も書かない) と SoC の初期化。
	 * -ENODEV (別の版) と -EBUSY (起動直後でない) は、この機材だけを使えない状態にして、DriverHost と他の機種は止めない。
	 */
	int ret = xz_board_open(&bridge_);
	if (ret) {
		msg_err("XZ board setup failed: %d%s\n", ret,
			ret == -EBUSY ? " (the device is not in the fresh state; replug it)" : "");
		return ret;
	}
	board_up_ = true;
	ret = xz_fe_init(&bridge_);
	if (ret) {
		msg_err("XZ front-end initialization failed: %d\n", ret);
		return ret;
	}
	frontend_initialized_ = true;

	stream_bus_.dev = &dev_;
	stream_bus_.type = ITEDTV_BUS_USB;
	stream_bus_.usb.dev = &usb_dev_;
	stream_bus_.usb.ctrl_timeout = kControlTimeoutMs;
	stream_bus_.usb.streaming.endpoint = kStreamEndpoint;
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
		   "XZ ready: host_index=%s usb_serial=%ls\n",
		   dev_.device_name, usb_serial_number_.c_str());
	return 0;
}

void XzDevice::Term()
{
	if (initialized_.exchange(false) && receiver_)
		receiver_manager_.Unregister(receiver_.get());
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	std::unique_ptr<Receiver> receiver;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		/* 終了処理の間はカード UART の新しいレジスター操作を受け付けない。 */
		std::lock_guard<std::mutex> card_lock(card_uart_mutex_);
		/* ポンプのスレッドと URB を止め、機材がある場合は、ライタ、hook、本体の領域の順に元へ戻す。 */
		StopUsbStream();
		/* 復調器を休止させる (機材がある場合のみ)。クロック系のレジスタは戻さない (前面が動作中の復元は、周辺のレジスタブロックを止めた)。 */
		if (available_ && frontend_initialized_)
			xz_fe_sleep(&bridge_);
		if (buffer_started_ && buffer_)
			buffer_->Stop();
		buffer_started_ = false;
		receiver_open_ = false;
		available_ = false;
		card_open_ = false;
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
#if defined(_WIN32) || defined(_WIN64)
		if (control_event_) {
			CloseHandle(control_event_);
			control_event_ = nullptr;
		}
#endif
		/* 受信機の破棄は、クライアントの Close() を待つため、機材のロックを解いてから行う。 */
		receiver = std::move(receiver_);
		buffer_.reset();
		board_up_ = false;
		frontend_initialized_ = false;
		buffer_started_ = false;
	}
	receiver.reset();
}

void XzDevice::SetAvailability(bool available)
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
		/* 切断前に進行中の UART レジスター列を回収する。以後の要求は lock callback が拒否する。 */
		std::lock_guard<std::mutex> card_lock(card_uart_mutex_);
	}
	if (failed_buffer)
		failed_buffer->NotifyStreamFailure();
}

ReceiverBase *XzDevice::GetReceiver(int id) const
{
	if (id != 0)
		throw std::out_of_range("receiver id out of range");
	return receiver_.get();
}

int XzDevice::OpenCard()
{
	/* 直前の CloseCard() の電源断が済むまで待つ */
	std::lock_guard<std::mutex> session_lock(card_session_mutex_);
	if (!available_ || !board_up_)
		return -ENODEV;
	bool expected = false;
	return card_open_.compare_exchange_strong(expected, true) ? 0 : -EALREADY;
}

void XzDevice::CloseCard()
{
	/* 電源断が済むまで、次の OpenCard() を待たせる */
	std::lock_guard<std::mutex> session_lock(card_session_mutex_);
	if (!card_open_.exchange(false) || !available_ || !board_up_)
		return;
	/* PC/SC が閉じた後はカード UART の電源を切る。受信回路の電源とは独立している。 */
	xz_card_power_down(&card_port_);
}

int XzDevice::DetectCard(bool &detected)
{
	if (!card_open_)
		return -ENODEV;
	return xz_card_detect(&card_port_, &detected);
}

int XzDevice::ResetCard()
{
	return card_open_ ? xz_card_reset(&card_port_) : -ENODEV;
}

int XzDevice::SetCardBaudrate(std::uint32_t baudrate)
{
	return card_open_ ? torne_card_set_baudrate(&card_port_, baudrate) : -ENODEV;
}

int XzDevice::IsCardDataReady(bool &ready)
{
	if (!card_open_)
		return -ENODEV;
	return torne_card_rx_ready(&card_port_, &ready);
}

int XzDevice::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
{
	return card_open_ ? torne_card_read(&card_port_, buf, &len) : -ENODEV;
}

int XzDevice::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
{
	return card_open_ ? torne_card_write(&card_port_, buf, len) : -ENODEV;
}

int XzDevice::SetCardIoTimeout(std::uint32_t timeout_ms)
{
	return card_open_ ? torne_card_port_set_timeout(&card_port_, timeout_ms) : -ENODEV;
}

unsigned int XzDevice::ControlTimeout() const noexcept
{
	if (deadline_ == (std::chrono::steady_clock::time_point::max)())
		return kControlTimeoutMs;
	const auto remaining = deadline_ - std::chrono::steady_clock::now();
	if (remaining <= std::chrono::steady_clock::duration::zero())
		return 0;
	const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
	return static_cast<unsigned int>(std::min<std::int64_t>(kControlTimeoutMs,
		(std::max<std::int64_t>)(1, milliseconds)));
}

int XzDevice::ControlCallback(void *context, u8 request_type, u16 value,
			      u16 index, void *buffer, u16 length, unsigned int timeout_ms)
{
	return static_cast<XzDevice *>(context)->Control(request_type, value,
							 index, buffer, length, timeout_ms);
}

int XzDevice::Control(u8 request_type, u16 value, u16 index,
		      void *buffer, u16 length, unsigned int timeout_ms)
{
	if (!available_)
		return -ENODEV;
	/* 読みは 64 バイトまで、書きは 4 バイトまで。共通層が守るが、ここでも USB へ出す前に確かめる。 */
	if (length > 64 || (length && !buffer))
		return -EINVAL;
	const unsigned int timeout = timeout_ms ?
		std::min<unsigned int>(timeout_ms, kControlTimeoutMs) : ControlTimeout();
	if (!timeout)
		return -ETIMEDOUT;
	const auto lock_started = std::chrono::steady_clock::now();
	std::unique_lock<std::timed_mutex> control_lock(control_mutex_, std::defer_lock);
	if (!control_lock.try_lock_for(std::chrono::milliseconds(timeout)))
		return -ETIMEDOUT;
	const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - lock_started).count();
	if (waited >= timeout)
		return -ETIMEDOUT;
	const unsigned int transfer_timeout = timeout - static_cast<unsigned int>(waited);
#if defined(__APPLE__) || defined(PX4_USERLAND)
	const int ret = libusb_control_transfer(usb_dev_.handle, request_type, 0xbc,
		value, index, static_cast<unsigned char *>(buffer), length, transfer_timeout);
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
	setup.Request = 0xbc;
	setup.Value = value;
	setup.Index = index;
	ULONG transferred = 0;
	OVERLAPPED overlapped = {};
	overlapped.hEvent = control_event_;
	/*
	 * 書き込み (OUT) のデータも、WinUSB は書き込み可能なメモリを要求する。共通層 (xz_write) は、読み取り専用のデータを含む
	 * 呼び出し側のバッファを、書き込み可能な一時バッファへコピーしてから、この関数へ渡す。
	 */
	if (!WinUsb_ControlTransfer(usb_dev_.winusb, setup,
				    static_cast<PUCHAR>(buffer), length,
				    &transferred, &overlapped)) {
		DWORD error = GetLastError();
		if (error == ERROR_IO_PENDING) {
			const DWORD wait = WaitForSingleObject(overlapped.hEvent, transfer_timeout);
			if (wait == WAIT_TIMEOUT) {
				if (!CancelIoEx(usb_dev_.dev, &overlapped) && GetLastError() != ERROR_NOT_FOUND)
					msg_warn("XZ: USB 制御転送の timeout 後、キャンセル完了を待っています\n");
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
		if (error == ERROR_DEVICE_NOT_CONNECTED || error == ERROR_NO_SUCH_DEVICE)
			return -ENODEV;
		if (error == ERROR_OPERATION_ABORTED)
			return -ECANCELED;
		return -EIO;
	}
	return static_cast<int>(transferred);
#endif
}

int XzDevice::CardRead32Callback(void *context, u32 address, u32 *value,
				 unsigned int timeout_ms)
{
	return static_cast<XzDevice *>(context)->CardRead32(address, value, timeout_ms);
}

int XzDevice::CardWrite32Callback(void *context, u32 address, u32 value,
				  unsigned int timeout_ms)
{
	return static_cast<XzDevice *>(context)->CardWrite32(address, value, timeout_ms);
}

int XzDevice::CardLockCallback(void *context)
{
	auto *device = static_cast<XzDevice *>(context);
	try {
		device->card_uart_mutex_.lock();
	} catch (...) {
		return -EDEADLK;
	}
	if (!device->available_) {
		device->card_uart_mutex_.unlock();
		return -ENODEV;
	}
	return 0;
}

void XzDevice::CardUnlockCallback(void *context)
{
	static_cast<XzDevice *>(context)->card_uart_mutex_.unlock();
}

void XzDevice::CardSleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

u64 XzDevice::CardNowCallback(void *)
{
	return msg_now_ms();
}

int XzDevice::CardRead32(u32 address, u32 *value, unsigned int timeout_ms)
{
	if (!value || !available_ || !board_up_)
		return -ENODEV;
	std::uint8_t bytes[4] = {};
	/* 24 ビットのアドレス: 上位 8 ビットが wValue の上位バイト、下位 16 ビットが wIndex */
	const u16 request_value = static_cast<u16>((address >> 8) & 0xff00);
	const u16 request_index = static_cast<u16>(address);
	const int ret = Control(0xc0, request_value, request_index, bytes,
		static_cast<u16>(sizeof(bytes)), timeout_ms);
	if (ret < 0)
		return ret;
	if (ret != static_cast<int>(sizeof(bytes)))
		return -EIO;
	*value = static_cast<u32>(bytes[0]) << 24 | static_cast<u32>(bytes[1]) << 16 |
		static_cast<u32>(bytes[2]) << 8 | bytes[3];
	return 0;
}

int XzDevice::CardWrite32(u32 address, u32 value, unsigned int timeout_ms)
{
	if (!available_ || !board_up_)
		return -ENODEV;
	std::uint8_t bytes[4] = {
		static_cast<std::uint8_t>(value >> 24),
		static_cast<std::uint8_t>(value >> 16),
		static_cast<std::uint8_t>(value >> 8),
		static_cast<std::uint8_t>(value),
	};
	const u16 request_value = static_cast<u16>((address >> 8) & 0xff00);
	const u16 request_index = static_cast<u16>(address);
	const int ret = Control(0x40, request_value, request_index, bytes,
		static_cast<u16>(sizeof(bytes)), timeout_ms);
	if (ret < 0)
		return ret;
	return ret == static_cast<int>(sizeof(bytes)) ? 0 : -EIO;
}

void XzDevice::SleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

u64 XzDevice::NowCallback(void *)
{
	return msg_now_ms();
}

void XzDevice::LogCallback(void *, int level, const char *format, ...)
{
	char message[512] = {};
	va_list args;
	va_start(args, format);
	std::vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (level <= XZ_LOG_ERR)
		msg_err("xz: %s\n", message);
	else if (level == XZ_LOG_INFO)
		msg_info("xz: %s\n", message);
	else
		msg_dbg("xz: %s\n", message);
}

/*
 * 機材の bulk IN に残った、前回の TS を読み捨てる。
 * 前回の受信が、ホストが読み出す側を先に止められて終わると、ポンプが送った最後の数パケットが機材の送信バッファに残り、
 * 次の受信の先頭に混ざって連続性の飛びになる。ライタもポンプもまだ動いていないので、新しいデータは来ない。
 * 読み出しが時間切れになったら、残りは無い。ライタの開始より前に行う。
 */
void XzDevice::DrainStream()
{
	std::array<std::uint8_t, XZ_PACKET_BYTES> buffer{};
	unsigned int total = 0;
	for (unsigned int i = 0; i < kDrainMaxReads; ++i) {
		int length = static_cast<int>(buffer.size());
		if (itedtv_bus_stream_rx(&stream_bus_, buffer.data(), &length, kDrainTimeoutMs) || length <= 0)
			break;
		total += static_cast<unsigned int>(length);
	}
	if (total)
		msg_dbg("XZ: drained %u stale bytes from bulk IN\n", total);
}

/* 取り込みを始める: 取り込みブロックの設定 → 機材内プログラムの導入 → URB → ポンプのスレッド → ライタの開始。失敗したら逆順に戻す。 */
int XzDevice::StartUsbStream()
{
	if (!available_ || !frontend_initialized_ || !bus_initialized_)
		return -ENODEV;
	if (needs_replug_)
		return -EIO;
	if (streaming_)
		return stream_failed_ ? -EIO : 0;
	stream_failed_ = false;
	/* 選局の直後は、ロックから一定の時間が経つまで待つ (ロックしていない搬送波は待たない) */
	if (lock_seen_) {
		const auto until = lock_time_ + std::chrono::milliseconds(kLockSettleMs);
		const auto now = std::chrono::steady_clock::now();
		if (now < until)
			std::this_thread::sleep_for(until - now);
	}
	torne_framer_reset(&framer_);
	DrainStream();
	int ret = xz_cap_start(&bridge_);
	if (ret)
		return ret;
	ret = xz_prog_install(&bridge_);
	if (ret) {
		xz_cap_stop(&bridge_);
		return ret;
	}
	ret = itedtv_bus_start_streaming(&stream_bus_, StreamHandler, this);
	if (ret) {
		if (xz_prog_remove(&bridge_))
			needs_replug_ = true;
		xz_cap_stop(&bridge_);
		return ret;
	}
	/* 前回のスレッドが失敗で終了していても、join するまで joinable のままなので、代入の前に回収する。 */
	if (pump_thread_.joinable())
		pump_thread_.join();
	pump_stop_ = false;
	try {
		pump_thread_ = std::thread(&XzDevice::PumpThread, this);
	} catch (const std::system_error &) {
		ret = -EAGAIN;
	}
	/* ライタは、読み出し側が動いてから開始する (先に開始すると、約 50 ms でリングが 1 周して、未読のデータを上書きする) */
	if (!ret)
		ret = xz_cap_run(&bridge_);
	if (ret) {
		pump_stop_ = true;
		if (pump_thread_.joinable())
			pump_thread_.join();
		itedtv_bus_stop_streaming(&stream_bus_);
		if (xz_prog_remove(&bridge_))
			needs_replug_ = true;
		xz_cap_stop(&bridge_);
		return ret;
	}
	streaming_ = true;
	return 0;
}

/* 取り込みを止める。切断後 (available_ が偽) は、機材へ書かず、スレッドと URB だけを止める。 */
int XzDevice::StopUsbStream()
{
	int ret = 0;
	pump_stop_ = true;
	if (pump_thread_.joinable())
		pump_thread_.join();
	if (bus_initialized_ && streaming_)
		ret = itedtv_bus_stop_streaming(&stream_bus_);
	const bool was_streaming = streaming_;
	streaming_ = false;
	if (was_streaming && available_) {
		/* ライタを止めて、hook を先に元へ戻し、次に本体の領域を戻す */
		int step = xz_cap_stop(&bridge_);
		if (!ret)
			ret = step;
		step = xz_prog_remove(&bridge_);
		if (step) {
			needs_replug_ = true;
			if (!ret)
				ret = step;
		}
	}
	torne_framer_reset(&framer_);
	return ret;
}

/*
 * ポンプのスレッド: 書き込み位置を読み、溜まった分を 16 パケットの倍数で機材内プログラムに送らせ、
 * 送り終えた位置を 192 バイトの倍数に切り下げて、読み出し位置へ書く。溜まりが無ければ短く眠る。
 * このスレッドの遅れは、SDRAM のリング (192 KiB、約 85 ms 分) が一周して、未読の TS を上書きし、連続性エラーになる。
 * 優先度が通常の Windows では、70〜90 ms 遅れる周期が実機で確認されたので、優先度を最大にする (プロセス全体は変えない)。
 * 制御転送の失敗が続いたら、受信機のバッファへ失敗を通知して終了する。
 */
void XzDevice::PumpThread()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	u32 ring_pos = 0, rptr = 0;
	int errors = 0;
	bool failed = false;

	while (!pump_stop_) {
		u32 wptr = 0;
		int ret = xz_cap_wptr(&bridge_, &wptr);
		if (ret) {
			if (!available_ || ++errors >= kPumpErrorLimit) {
				failed = true;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		const u32 avail = (wptr + XZ_RING_SIZE - ring_pos) % XZ_RING_SIZE;
		u32 packets = avail / XZ_PACKET_BYTES / XZ_PUMP_UNIT_PACKETS * XZ_PUMP_UNIT_PACKETS;
		if (!packets) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		if (packets > XZ_PUMP_MAX_PACKETS)
			packets = XZ_PUMP_MAX_PACKETS;
		ret = xz_prog_pump(&bridge_, ring_pos, packets);
		if (ret) {
			if (!available_ || ++errors >= kPumpErrorLimit) {
				failed = true;
				break;
			}
			continue;
		}
		errors = 0;
		ring_pos = (ring_pos + packets * XZ_PACKET_BYTES) % XZ_RING_SIZE;
		const u32 floor_pos = ring_pos / XZ_UNIT_SIZE * XZ_UNIT_SIZE;
		if (floor_pos != rptr) {
			ret = xz_cap_set_rptr(&bridge_, floor_pos);
			if (ret) {
				if (!available_ || ++errors >= kPumpErrorLimit) {
					failed = true;
					break;
				}
				continue;
			}
			rptr = floor_pos;
		}
	}
	if (failed && !pump_stop_) {
		stream_failed_ = true;
		std::shared_ptr<ReceiverBase::StreamBuffer> buffer = buffer_;
		if (buffer)
			buffer->NotifyStreamFailure();
		msg_err("XZ pump thread stopped on errors (device %s)\n", available_ ? "available" : "removed");
	}
}

int XzDevice::StreamHandler(void *context, void *data, u32 size)
{
	auto &device = *static_cast<XzDevice *>(context);
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
	if (!device.buffer_ || !device.available_)
		return -ENODEV;
	std::array<std::uint8_t, kStreamBufferSize + XZ_TS_PACKET_SIZE> output{};
	const std::size_t output_size = torne_framer_feed(&device.framer_,
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

/*
 * クライアントの接続は、機材の抜去の後にも、受信機の Close() を呼ぶ。Close() が戻るまで受信機を破棄しないと、
 * 破棄済みのオブジェクトの仮想関数を呼ぶ。Px4Device などの他の機種の受信機と同じ契約。
 */
XzDevice::Receiver::~Receiver()
{
	std::unique_lock<std::recursive_mutex> lock(parent_.mutex_);
	parent_.close_cond_.wait(lock, [this] { return !parent_.client_open_; });
}

int XzDevice::Receiver::Open()
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.frontend_initialized_)
		return -ENODEV;
	if (parent_.receiver_open_)
		return -EALREADY;
	parent_.receiver_open_ = true;
	parent_.client_open_ = true;
	return 0;
}

void XzDevice::Receiver::Close()
{
	SetCapture(false);
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	parent_.receiver_open_ = false;
	parent_.client_open_ = false;
	parent_.close_cond_.notify_all();
}

int XzDevice::Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return voltage == 0 ? 0 : -ENOSYS;
}

int XzDevice::Receiver::PrepareTune()
{
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (params_.system != SystemType::ISDB_T)
		return -ENOSYS;
	if (params_.bandwidth && params_.bandwidth != 6)
		return -EINVAL;
	return 0;
}

int XzDevice::Receiver::SetFrequency()
{
	const bool resume = parent_.streaming_;
	int ret = 0;
	if (resume)
		ret = parent_.StopUsbStream();
	if (!ret && parent_.buffer_ && !parent_.buffer_->Purge(parent_.deadline_))
		ret = -EIO;
	if (!ret) {
		torne_framer_reset(&parent_.framer_);
		parent_.lock_seen_ = false;
		/* 周波数は kHz (UHF 13〜62ch の範囲。範囲外は -EINVAL) */
		ret = xz_fe_tune(&parent_.bridge_, params_.freq);
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

int XzDevice::Receiver::CheckLock(bool &locked)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	locked = false;
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	const int ret = xz_fe_locked(&parent_.bridge_, &locked);
	if (!ret && locked && !parent_.lock_seen_) {
		parent_.lock_seen_ = true;
		parent_.lock_time_ = std::chrono::steady_clock::now();
	}
	return ret;
}

int XzDevice::Receiver::SetCapture(bool capture)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
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

int XzDevice::Receiver::ReadStat(command::StatType, std::int32_t &)
{
	/*
	 * 復調器の C/N の生の値 (xz_fe_cnr_raw) は読めるが、dB への換算は GV-MVP/XZ2 では確認していないため、未対応にする。
	 * 換算していない値を C/N として返さない。
	 */
	return -ENOSYS;
}

int XzDevice::Receiver::SetStreamId()
{
	return -ENOSYS;
}

bool XzDevice::Receiver::TuneOverride(std::uint32_t timeout, bool &result)
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
	 * 選局で止めた受信を再開する。ロックを確認できたときは、StartUsbStream がロックから 0.5 秒の安定を待つ。
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
			   "XZ tune failed: freq_khz=%u locked=%u result=%d\n",
			   params_.freq, locked ? 1U : 0U, ret);
	}
	parent_.deadline_ = (std::chrono::steady_clock::time_point::max)();
	return true;
}

} // namespace px4
