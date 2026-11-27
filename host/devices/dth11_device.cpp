// SPDX-License-Identifier: GPL-2.0-only

#include "dth11_device.hpp"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <new>
#include <random>
#include <stdexcept>
#include <system_error>

#if defined(__APPLE__) || defined(PX4_USERLAND)
#include <libusb.h>
#include <exe_path.h>
#endif

#include "msg.h"
#include "util.hpp"

namespace px4 {
namespace {
	constexpr unsigned int kControlTimeoutMs = 3000;
	/* 相互認証は、1 段ごとに機器が ECDSA を計算する (1 回に約 1 秒)。この間の制御転送の排他を取る待ちの上限。 */
	constexpr unsigned int kHandshakeLockMs = 10000;
	/*
	 * 非同期の読み出しは 32 KiB (128 スロット、約 12 ms 分) の 16 本。256 バイトのスロットと 512 バイトの
	 * bulk パケットの両方の倍数。
	 */
	constexpr unsigned int kStreamBufferSize = 32768;
	constexpr std::uint8_t kStreamEndpoint = 0x81;
	/*
	 * 選局の後、番組表 (PAT と PMT) がそろうまでの上限。PAT と PMT は 0.1 秒ごとに来て、通常は 0.4〜0.8 秒で
	 * そろう。弱い電波で最後までそろわないときも、プレーヤーの自分の期限 (TVTest は約 10 秒) の内に選局を
	 * 終えられるよう、短く保つ。
	 */
	constexpr std::chrono::milliseconds kPsiLearnTimeout(4000);
	constexpr std::size_t kMaxTsPerUrb = kStreamBufferSize / DTH11_SLOT_SIZE;
}

class Dth11Device::Receiver final : public ReceiverBase {
public:
	explicit Receiver(Dth11Device &parent) : ReceiverBase(0), parent_(parent) {}
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
	Dth11Device &parent_;
};

/* 制御転送の排他を、呼び出しのスレッドの間だけ保持する (相互認証の手順を、他の制御転送で分断させない)。 */
class ControlHold final {
public:
	ControlHold(std::timed_mutex &mutex, std::atomic<std::thread::id> &owner, unsigned int timeout_ms)
		: mutex_(mutex), owner_(owner), locked_(mutex.try_lock_for(std::chrono::milliseconds(timeout_ms)))
	{
		if (locked_)
			owner_.store(std::this_thread::get_id());
	}
	~ControlHold()
	{
		if (locked_) {
			owner_.store(std::thread::id());
			mutex_.unlock();
		}
	}
	ControlHold(const ControlHold &) = delete;
	ControlHold &operator=(const ControlHold &) = delete;
	bool Locked() const noexcept { return locked_; }

private:
	std::timed_mutex &mutex_;
	std::atomic<std::thread::id> &owner_;
	bool locked_;
};

Dth11Device::Dth11Device(const std::wstring &path,
		   const DeviceDefinition &definition,
		   std::uintptr_t index,
		   ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	if (definition.receivers.size() != 1 || definition.receivers[0].index != 0 ||
	    definition.receivers[0].systems != SystemType::ISDB_T)
		throw DeviceError("DTH11 requires exactly one ISDB-T receiver.");
}

Dth11Device::~Dth11Device()
{
	Term();
}

/*
 * 資格情報のファイルは利用者が用意する。設定の相対パスは実行ファイルの場所を基準にする。
 * ファイルが読めない、または純正の CoBDACom.dll でないときは、この機材を使えない状態にする
 * (鍵がないまま受信を始めると、ワンセグ以外が暗号化されたままで届くため)。
 */
int Dth11Device::LoadAppKeys()
{
	std::filesystem::path file(device_def_.configs.Get(L"Dth11AppKeys", L"CoBDACom.dll"));
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
		msg_err("DTH11: application key file not found: %s (put the CoBDACom.dll of the vendor's PCastTV3 there)\n",
			file.string().c_str());
		return -ENOENT;
	}
	const std::vector<char> image((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	const int ret = dth11_appkeys_from_image(reinterpret_cast<const u8 *>(image.data()), image.size(), &keys_);
	if (ret) {
		msg_err("DTH11: cannot read the application credentials from %s (%d%s)\n", file.string().c_str(), ret,
			ret == DTH11_APPKEYS_NOT_PE ? ": not a DLL" :
			ret == DTH11_APPKEYS_NO_CERT ? ": certificate not found, is it the CoBDACom.dll of PCastTV3?" :
			ret == DTH11_APPKEYS_NO_PRIVATE_KEY ? ": private key not found" : "");
		return ret;
	}
	keys_loaded_ = true;
	return 0;
}

int Dth11Device::Init()
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
	max_urbs_ = util::wtoui(config.Get(L"Dth11MaxUrbs", L"16"));
	if (!packets || packets > (std::numeric_limits<std::uint32_t>::max)() / 188U ||
	    !max_urbs_ || max_urbs_ > 64)
		return -EINVAL;
	receiver_packets_ = packets;

	static const struct dth11_ops ops = {
		&Dth11Device::ControlCallback,
		&Dth11Device::SleepCallback,
		&Dth11Device::NowCallback,
		&Dth11Device::LogCallback,
	};
	dth11_bridge_init(&bridge_, &ops, this);

	/* 機器情報 (キットの確認)、ファームウェアの版、資格情報、起動。失敗してもこの機材だけを使えない状態にする。 */
	int ret = dth11_read_info(&bridge_, &info_);
	if (ret) {
		msg_err("DTH11 device information failed: %d%s\n", ret,
			ret == -ENODEV ? " (not a PN2031_MxL135RF kit)" :
			ret == -ENOTSUP ? " (unsupported configuration)" : "");
		return ret;
	}
	u8 fw_major = 0, fw_minor = 0;
	u16 fw_build = 0;
	if (!dth11_fw_version(&bridge_, &fw_major, &fw_minor, &fw_build))
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "DTH11 firmware %u.%u.%u\n",
			   fw_major, fw_minor, fw_build);
	ret = LoadAppKeys();
	if (ret)
		return ret;
	try {
		psi_.reset(new struct dth11_psi());
	} catch (const std::bad_alloc &) {
		return -ENOMEM;
	}
	ret = dth11_start(&bridge_, &info_);
	if (ret) {
		msg_err("DTH11 start failed: %d\n", ret);
		return ret;
	}
	started_ = true;

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
		   "DTH11 ready: host_index=%s usb_serial=%ls\n",
		   dev_.device_name, usb_serial_number_.c_str());
	return 0;
}

void Dth11Device::Term()
{
	if (initialized_.exchange(false) && receiver_)
		receiver_manager_.Unregister(receiver_.get());
	std::shared_ptr<ReceiverBase::StreamBuffer> failed_buffer;
	std::unique_ptr<Receiver> receiver;
	{
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		StopStream();
		if (buffer_started_ && buffer_)
			buffer_->Stop();
		buffer_started_ = false;
		receiver_open_ = false;
		capture_ = false;
		locked_ = false;
		/* 受信機を止める (機材がある場合だけ)。電源は落とさない。次に開くときに起動からやり直す。 */
		if (started_ && available_)
			dth11_stop(&bridge_, &info_);
		started_ = false;
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
#if defined(_WIN32) || defined(_WIN64)
		if (control_event_) {
			CloseHandle(control_event_);
			control_event_ = nullptr;
		}
#endif
		if (keys_loaded_) {
			dth11_appkeys_free(&keys_);
			keys_loaded_ = false;
		}
		{
			std::lock_guard<std::mutex> hs_lock(handshake_mutex_);
			std::memset(&handshake_, 0, sizeof(handshake_));
			has_handshake_ = false;
		}
		/* 受信機の破棄は、クライアントの Close() を待つため、機材のロックを解いてから行う。 */
		receiver = std::move(receiver_);
		buffer_.reset();
	}
	receiver.reset();
}

void Dth11Device::SetAvailability(bool available)
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
	if (!available)
		dec_cond_.notify_all();
}

ReceiverBase *Dth11Device::GetReceiver(int id) const
{
	if (id != 0)
		throw std::out_of_range("receiver id out of range");
	return receiver_.get();
}

unsigned int Dth11Device::ControlTimeout() const noexcept
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

int Dth11Device::ControlCallback(void *context, u8 request_type, u8 request, u16 value,
			      u16 index, void *buffer, u16 length)
{
	const int ret = static_cast<Dth11Device *>(context)->Control(request_type, request, value, index, buffer, length);
	/* 機器の拒否 (STALL) は相互認証の待ちで普通に起きるので、それ以外の失敗だけを記録する */
	if (ret < 0 && ret != -EPIPE)
		msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_USB,
			   "DTH11 control transfer failed: type=0x%02x req=0x%02x value=0x%04x index=0x%04x len=%u ret=%d\n",
			   request_type, request, value, index, length, ret);
	return ret;
}

int Dth11Device::Control(u8 request_type, u8 request, u16 value, u16 index,
		      void *buffer, u16 length, unsigned int timeout_ms)
{
	if (!available_)
		return -ENODEV;
	if (length && !buffer)
		return -EINVAL;
	const unsigned int timeout = timeout_ms ?
		std::min<unsigned int>(timeout_ms, kControlTimeoutMs) : ControlTimeout();
	if (!timeout)
		return -ETIMEDOUT;
	const auto lock_started = std::chrono::steady_clock::now();
	std::unique_lock<std::timed_mutex> control_lock(control_mutex_, std::defer_lock);
	/* 相互認証の手順を保持しているスレッドは、既に排他を持っている */
	const bool owned = control_owner_.load() == std::this_thread::get_id();
	if (!owned && !control_lock.try_lock_for(std::chrono::milliseconds(timeout)))
		return -ETIMEDOUT;
	const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - lock_started).count();
	if (waited >= timeout)
		return -ETIMEDOUT;
	const unsigned int transfer_timeout = timeout - static_cast<unsigned int>(waited);
#if defined(__APPLE__) || defined(PX4_USERLAND)
	const int ret = libusb_control_transfer(usb_dev_.handle, request_type, request,
		value, index, static_cast<unsigned char *>(buffer), length, transfer_timeout);
	if (ret < 0) {
		if (ret == LIBUSB_ERROR_TIMEOUT)
			return -ETIMEDOUT;
		if (ret == LIBUSB_ERROR_NO_DEVICE)
			return -ENODEV;
		/* 機器が要求を拒否した (STALL)。相互認証の応答待ちでは、これが「まだ計算中」を表す */
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
	ULONG transferred = 0;
	OVERLAPPED overlapped = {};
	overlapped.hEvent = control_event_;
	if (!WinUsb_ControlTransfer(usb_dev_.winusb, setup,
				    static_cast<PUCHAR>(buffer), length,
				    &transferred, &overlapped)) {
		DWORD error = GetLastError();
		if (error == ERROR_IO_PENDING) {
			const DWORD wait = WaitForSingleObject(overlapped.hEvent, transfer_timeout);
			if (wait == WAIT_TIMEOUT) {
				if (!CancelIoEx(usb_dev_.dev, &overlapped) && GetLastError() != ERROR_NOT_FOUND)
					msg_warn("DTH11: USB 制御転送の timeout 後、キャンセル完了を待っています\n");
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
		/* WinUSB は、機器が要求を拒否した (STALL) ことを ERROR_GEN_FAILURE で返す */
		if (error == ERROR_GEN_FAILURE)
			return -EPIPE;
		return -EIO;
	}
	return static_cast<int>(transferred);
#endif
}

void Dth11Device::SleepCallback(void *, unsigned int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

u64 Dth11Device::NowCallback(void *)
{
	return msg_now_ms();
}

void Dth11Device::LogCallback(void *, int level, const char *format, ...)
{
	char message[512] = {};
	va_list args;
	va_start(args, format);
	std::vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (level <= DTH11_LOG_ERR)
		msg_err("dth11: %s\n", message);
	else if (level == DTH11_LOG_INFO)
		msg_info("dth11: %s\n", message);
	else
		msg_dbg("dth11: %s\n", message);
}

/* ECDSA の乱数と相互認証の乱数。OS の乱数源 (std::random_device) を使う。 */
int Dth11Device::RngCallback(void *, u8 *buffer, std::size_t length)
{
	try {
		std::random_device source;
		std::size_t i = 0;
		while (i < length) {
			const unsigned int word = source();
			for (unsigned int k = 0; k < sizeof(word) && i < length; ++k, ++i)
				buffer[i] = static_cast<u8>(word >> (8 * k));
		}
		return 0;
	} catch (...) {
		return -EIO;
	}
}

int Dth11Device::RunHandshake(dth11_handshake &out)
{
	if (!available_ || !started_)
		return -ENODEV;
	ControlHold hold(control_mutex_, control_owner_, kHandshakeLockMs);
	if (!hold.Locked())
		return -ETIMEDOUT;
	const auto started = std::chrono::steady_clock::now();
	const int ret = dth11_cepm_handshake_for_path(&bridge_, DTH11_CEPM_PATH0, &keys_,
		&Dth11Device::RngCallback, this, &out);
	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - started).count();
	if (ret)
		msg_err("DTH11: CEPM handshake failed: %d (%lld ms)\n", ret, static_cast<long long>(elapsed));
	else
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_AUTH, "DTH11: CEPM handshake done in %lld ms\n",
			   static_cast<long long>(elapsed));
	return ret;
}

int Dth11Device::EnsureHandshake()
{
	{
		std::lock_guard<std::mutex> lock(handshake_mutex_);
		if (has_handshake_)
			return 0;
	}
	dth11_handshake hs = {};
	const int ret = RunHandshake(hs);
	if (ret)
		return ret;
	std::lock_guard<std::mutex> lock(handshake_mutex_);
	handshake_ = hs;
	has_handshake_ = true;
	return 0;
}

void Dth11Device::FailStream()
{
	stream_failed_ = true;
	if (buffer_)
		buffer_->NotifyStreamFailure();
	dec_cond_.notify_all();
}

/*
 * 受信のスレッド。256 バイトのスロットから TS を取り出し、リンク暗号を復号して、番組表の収集 (学習中) か、
 * 受信機のバッファ (受信中) へ渡す。新しい相互認証が必要になったら、要求を立てるだけにする
 * (数秒かかるので、このスレッドでは行わない。待たせると USB の読み出しが滞る)。
 */
int Dth11Device::StreamHandler(void *context, void *data, u32 size)
{
	auto &device = *static_cast<Dth11Device *>(context);
	if (!data || !size) {
		device.FailStream();
		return -EIO;
	}
	if (size > kStreamBufferSize) {
		device.FailStream();
		return -EOVERFLOW;
	}
	if (!device.buffer_ || !device.available_)
		return -ENODEV;
	const auto handler_start = std::chrono::steady_clock::now();
	auto &timing = device.stream_timing_;
	if (timing.last_call != std::chrono::steady_clock::time_point{}) {
		const auto interval = std::chrono::duration_cast<std::chrono::microseconds>(handler_start - timing.last_call).count();
		timing.max_interval_us = (std::max<std::int64_t>)(timing.max_interval_us, interval);
	}
	timing.last_call = handler_start;
	timing.urbs++;
	timing.bytes += size;
	if (size < kStreamBufferSize)
		timing.short_urbs++;
	std::array<std::uint8_t, kMaxTsPerUrb * DTH11_TS_PACKET_SIZE> ts{};
	const std::size_t count = dth11_extract_ts(static_cast<const u8 *>(data), size, ts.data(), true);
	if (!count)
		return 0;
	std::size_t kept = 0;
	bool learning = false;
	{
		std::lock_guard<std::mutex> lock(device.dec_mutex_);
		const u64 now = msg_now_ms();
		learning = device.learning_;
		for (std::size_t i = 0; i + DTH11_TS_PACKET_SIZE <= count; i += DTH11_TS_PACKET_SIZE) {
			u8 *packet = ts.data() + i;
			if (!dth11_dec_process(&device.decryptor_, packet, now))
				continue;
			if (learning) {
				dth11_psi_feed(device.psi_.get(), packet);
			} else {
				if (kept != i)
					std::memmove(ts.data() + kept, packet, DTH11_TS_PACKET_SIZE);
				kept += DTH11_TS_PACKET_SIZE;
			}
		}
		if (dth11_dec_take_rehandshake_request(&device.decryptor_))
			device.rehandshake_requested_ = true;
		if (learning || device.rehandshake_requested_)
			device.dec_cond_.notify_all();
	}
	if (!learning && kept) {
		std::size_t written = kept;
		if (!device.buffer_->Write(ts.data(), written)) {
			device.FailStream();
			return -ENOBUFS;
		}
		device.buffer_->NotifyWrite();
	}
	const auto handler_end = std::chrono::steady_clock::now();
	timing.max_handler_us = (std::max<std::int64_t>)(timing.max_handler_us,
		std::chrono::duration_cast<std::chrono::microseconds>(handler_end - handler_start).count());
	/* 3 秒ごとに、読み出しの間隔と処理時間の最大値を記録する (診断用。debug のときだけ出る) */
	if (handler_end >= timing.next_log) {
		if (timing.next_log != std::chrono::steady_clock::time_point{}) {
			dth11_dec_stats stats;
			{
				std::lock_guard<std::mutex> lock(device.dec_mutex_);
				stats = device.decryptor_.stats;
			}
			msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_TS,
				   "DTH11 stream: urbs=%llu bytes=%llu short=%llu max_interval_us=%lld max_handler_us=%lld "
				   "dec: packets=%llu decrypted=%llu clear=%llu key=%llu no_key=%llu errors=%llu\n",
				   static_cast<unsigned long long>(timing.urbs), static_cast<unsigned long long>(timing.bytes),
				   static_cast<unsigned long long>(timing.short_urbs),
				   static_cast<long long>(timing.max_interval_us), static_cast<long long>(timing.max_handler_us),
				   static_cast<unsigned long long>(stats.packets), static_cast<unsigned long long>(stats.decrypted),
				   static_cast<unsigned long long>(stats.clear), static_cast<unsigned long long>(stats.key_packets),
				   static_cast<unsigned long long>(stats.dropped_no_key), static_cast<unsigned long long>(stats.errors));
		}
		timing.next_log = handler_end + std::chrono::seconds(3);
		timing.max_interval_us = 0;
		timing.max_handler_us = 0;
	}
	return 0;
}

/*
 * 番組の PID リストをデバイスへ送ったうえで、ストリームを有効にする。デバイスは、ストリームを
 * 無効にするとリストを忘れ、有効にした後にもう一度送った方が確実なため (純正の通信の再現)、
 * 有効化の後にも送る。
 */
int Dth11Device::EnableStreamWithPids()
{
	int ret = dth11_stream_enable(&bridge_, &info_, DTH11_STREAM_DEVICE);
	if (ret)
		return ret;
	if (!program_pids_.empty()) {
		ret = dth11_set_program_pids(&bridge_, program_number_, program_pids_.data(), program_pids_.size());
		if (ret) {
			/* PID リストのないストリームは、デバイスが何も復号しない。成功として返さず、呼び出し側に後始末をさせる */
			msg_warn("DTH11: set program PIDs failed: %d\n", ret);
			dth11_stream_disable(&bridge_);
			return ret;
		}
	}
	return 0;
}

/*
 * 選局したチャンネルの番組表を、リンク暗号の復号を通して読む。デバイスは番組 1 つ分の PID リストを
 * 受け取って、その番組だけを復号する。学習中に機器が新しい相互認証を要求したら (カードの抜き差しの後
 * など)、認証をやり直してストリームを入れ直し、期限を延ばす。
 */
int Dth11Device::LearnProgram(u32 &program, std::vector<u16> &pids)
{
	program = 0;
	pids.clear();
	dth11_handshake hs = {};
	{
		std::lock_guard<std::mutex> lock(handshake_mutex_);
		hs = handshake_;
	}
	{
		std::lock_guard<std::mutex> lock(dec_mutex_);
		dth11_dec_init(&decryptor_, &hs, &keys_);
		dth11_psi_init(psi_.get());
		learning_ = true;
		rehandshake_requested_ = false;
	}
	int ret = itedtv_bus_start_streaming(&stream_bus_, StreamHandler, this);
	if (ret) {
		std::lock_guard<std::mutex> lock(dec_mutex_);
		learning_ = false;
		return ret;
	}
	ret = dth11_stream_enable(&bridge_, &info_, DTH11_STREAM_DEVICE);
	const auto learn_start = std::chrono::steady_clock::now();
	auto deadline = learn_start + kPsiLearnTimeout;
	if (!ret) {
		std::unique_lock<std::mutex> lock(dec_mutex_);
		for (;;) {
			if (dth11_psi_complete(psi_.get()))
				break;
			if (stream_failed_ || !available_) {
				ret = -EIO;
				break;
			}
			if (rehandshake_requested_) {
				rehandshake_requested_ = false;
				lock.unlock();
				dth11_handshake renewed = {};
				const int hs_ret = RunHandshake(renewed);
				if (!hs_ret) {
					{
						std::lock_guard<std::mutex> hs_lock(handshake_mutex_);
						handshake_ = renewed;
						has_handshake_ = true;
					}
					{
						std::lock_guard<std::mutex> dec_lock(dec_mutex_);
						dth11_dec_install_handshake(&decryptor_, &renewed);
					}
					/* 機器は、ストリームを有効にした時点のセッションで鍵を包み続けるので、入れ直す */
					dth11_stream_disable(&bridge_);
					const int re_ret = dth11_stream_enable(&bridge_, &info_, DTH11_STREAM_DEVICE);
					if (re_ret) {
						/* 入れ直せていないのに期限を延ばすと、TS が来ないまま待ち続ける */
						lock.lock();
						ret = re_ret;
						break;
					}
					deadline = std::chrono::steady_clock::now() + kPsiLearnTimeout;
				}
				lock.lock();
				continue;
			}
			const auto now = std::chrono::steady_clock::now();
			if (now >= deadline)
				break;
			dec_cond_.wait_until(lock, (std::min)(deadline, now + std::chrono::milliseconds(200)));
		}
	}
	/* ストリームの無効化 (受信の一時停止) を先にして、その後で読み出しを止める。 */
	if (available_)
		dth11_stream_disable(&bridge_);
	itedtv_bus_stop_streaming(&stream_bus_);
	std::array<struct dth11_program, DTH11_PSI_MAX_PROGRAMS> programs{};
	std::size_t count = 0;
	bool complete = false;
	const std::wstring choice = device_def_.configs.Get(L"Dth11Program", L"first");
	const bool explicit_id = choice != L"all" && choice != L"first";
	const unsigned long wanted = explicit_id ? std::wcstoul(choice.c_str(), nullptr, 0) : 0;
	enum dth11_psi_program_state wanted_state = DTH11_PSI_PROGRAM_ABSENT;
	{
		std::lock_guard<std::mutex> lock(dec_mutex_);
		learning_ = false;
		complete = dth11_psi_complete(psi_.get());
		count = dth11_psi_programs(psi_.get(), programs.data(), programs.size());
		if (explicit_id)
			wanted_state = dth11_psi_program_state(psi_.get(), static_cast<u16>(wanted));
	}
	if (ret)
		return ret;
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE, "DTH11: PSI learned in %lld ms (complete: %d, programs: %zu)\n",
		   static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
			   std::chrono::steady_clock::now() - learn_start).count()),
		   complete ? 1 : 0, count);
	if (explicit_id && wanted_state == DTH11_PSI_PROGRAM_PMT_MISSING) {
		/* PAT にあるサービスの PMT が間に合わなかった。別の番組 (または PID リストなし) で成功にすると、
		 * 利用者が選んだ番組が映らない。他の番組の PMT が 1 件も届かず count が 0 のときも、ここで失敗にする */
		msg_warn("DTH11: PMT of program %lu was not received in time\n", wanted);
		return -ETIMEDOUT;
	}
	if (!count) {
		msg_warn("DTH11: no PAT/PMT learned, streaming without a PID list\n");
		return 0;
	}
	/* 復号する番組。設定は first (PAT の先頭)、all (最後に送った 1 つだけが有効)、サービス ID */
	std::size_t selected = 0;
	if (choice == L"all") {
		selected = count - 1;
	} else if (explicit_id) {
		bool found = false;
		for (std::size_t i = 0; i < count; ++i)
			if (programs[i].number == wanted) {
				selected = i;
				found = true;
			}
		if (!found)
			msg_warn("DTH11: program %lu is not in the PAT, using the first one\n", wanted);
	}
	std::array<u16, DTH11_PROGRAM_MAX_PIDS> list{};
	const std::size_t n = dth11_program_pid_list(&programs[selected], list.data());
	program = programs[selected].number;
	pids.assign(list.begin(), list.begin() + n);
	if (count > 1 && choice == L"first")
		msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
			   "DTH11: %zu programs in this stream, using program %u (set Dth11Program to choose)\n",
			   count, program);
	/* 無効の間に 1 回送る (有効化の後にも送る) */
	{
		const int pid_ret = dth11_set_program_pids(&bridge_, program, pids.data(), pids.size());
		if (pid_ret)
			msg_warn("DTH11: set program PIDs failed: %d\n", pid_ret);
	}
	return 0;
}

int Dth11Device::StartUsbStream()
{
	dth11_handshake hs = {};
	{
		std::lock_guard<std::mutex> lock(handshake_mutex_);
		hs = handshake_;
	}
	{
		/* 受信中の復号器は、学習のものとは別に、新しい状態から始める */
		std::lock_guard<std::mutex> lock(dec_mutex_);
		dth11_dec_init(&decryptor_, &hs, &keys_);
		learning_ = false;
		rehandshake_requested_ = false;
	}
	int ret = itedtv_bus_start_streaming(&stream_bus_, StreamHandler, this);
	if (ret)
		return ret;
	ret = EnableStreamWithPids();
	if (ret) {
		dth11_stream_disable(&bridge_);
		itedtv_bus_stop_streaming(&stream_bus_);
		return ret;
	}
	rehandshake_stop_ = false;
	try {
		rehandshake_thread_ = std::thread(&Dth11Device::RehandshakeWorker, this);
	} catch (const std::system_error &) {
		dth11_stream_disable(&bridge_);
		itedtv_bus_stop_streaming(&stream_bus_);
		return -EAGAIN;
	}
	streaming_ = true;
	return 0;
}

/*
 * 受信中に、機器が自分のものでないセッションを通知し続けたとき (電源の入れ直し、カードの抜き差し) の
 * 相互認証のやり直し。数秒かかるので、受信のスレッドとは別のスレッドで行う。
 */
void Dth11Device::RehandshakeWorker()
{
	std::unique_lock<std::mutex> lock(dec_mutex_);
	while (!rehandshake_stop_) {
		dec_cond_.wait(lock, [this] { return rehandshake_stop_ || rehandshake_requested_; });
		if (rehandshake_stop_)
			break;
		rehandshake_requested_ = false;
		lock.unlock();
		dth11_handshake renewed = {};
		const int ret = RunHandshake(renewed);
		if (!rehandshake_stop_ && !ret) {
			{
				std::lock_guard<std::mutex> hs_lock(handshake_mutex_);
				handshake_ = renewed;
				has_handshake_ = true;
			}
			{
				std::lock_guard<std::mutex> dec_lock(dec_mutex_);
				dth11_dec_install_handshake(&decryptor_, &renewed);
			}
			/* 機器は、ストリームを有効にした時点のセッションで鍵を包み続け、無効にすると PID リストを忘れる */
			const int dis_ret = dth11_stream_disable(&bridge_);
			const int en_ret = dis_ret ? dis_ret : EnableStreamWithPids();
			if (!en_ret) {
				msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_AUTH, "DTH11: stream restarted for the new session\n");
			} else if (!rehandshake_stop_) {
				/* 再開できないまま稼働中にすると、TS が来ないのに録画が続く。失敗として通知する */
				msg_err("DTH11: stream restart after re-handshake failed: %d\n", en_ret);
				FailStream();
			}
		}
		lock.lock();
	}
}

int Dth11Device::StartStream()
{
	if (!available_ || !started_ || !bus_initialized_)
		return -ENODEV;
	if (streaming_)
		return stream_failed_ ? -EIO : 0;
	stream_failed_ = false;
	int ret = EnsureHandshake();
	if (ret)
		return ret;
	ret = LearnProgram(program_number_, program_pids_);
	if (ret)
		return ret;
	return StartUsbStream();
}

int Dth11Device::StopStream()
{
	int ret = 0;
	{
		/* 待機側の述語の確認と待機の開始の間に、フラグの更新と通知が入って取りこぼされないよう、mutex の下で立てる */
		std::lock_guard<std::mutex> lock(dec_mutex_);
		rehandshake_stop_ = true;
	}
	dec_cond_.notify_all();
	if (rehandshake_thread_.joinable())
		rehandshake_thread_.join();
	if (streaming_) {
		if (available_)
			ret = dth11_stream_disable(&bridge_);
		const int stop_ret = itedtv_bus_stop_streaming(&stream_bus_);
		if (!ret)
			ret = stop_ret;
		streaming_ = false;
	}
	return ret;
}

/*
 * クライアントの接続は、機材の抜去の後にも、受信機の Close() を呼ぶ。Close() が戻るまで受信機を破棄しないと、
 * 破棄済みのオブジェクトの仮想関数を呼ぶ。他の機種の受信機と同じ契約。
 */
Dth11Device::Receiver::~Receiver()
{
	std::unique_lock<std::recursive_mutex> lock(parent_.mutex_);
	parent_.close_cond_.wait(lock, [this] { return !parent_.client_open_; });
}

int Dth11Device::Receiver::Open()
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.started_)
		return -ENODEV;
	if (parent_.receiver_open_)
		return -EALREADY;
	parent_.receiver_open_ = true;
	parent_.client_open_ = true;
	return 0;
}

void Dth11Device::Receiver::Close()
{
	SetCapture(false);
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	parent_.receiver_open_ = false;
	parent_.client_open_ = false;
	parent_.locked_ = false;
	parent_.close_cond_.notify_all();
}

int Dth11Device::Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return voltage == 0 ? 0 : -ENOSYS;
}

int Dth11Device::Receiver::PrepareTune()
{
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (params_.system != SystemType::ISDB_T)
		return -ENOSYS;
	if (params_.bandwidth && params_.bandwidth != 6)
		return -EINVAL;
	/* 周波数は kHz。UHF 13〜62ch (473.143〜767.143 MHz) の範囲だけを受け付ける */
	if (params_.freq < 470000 || params_.freq > 770000)
		return -EINVAL;
	return 0;
}

int Dth11Device::Receiver::SetFrequency()
{
	int ret = 0;
	/* 受信中の選局は、受信を止めてから行う (前のチャンネルの TS とバッファの端数を次へ持ち越さない) */
	if (parent_.streaming_)
		ret = parent_.StopStream();
	if (!ret && parent_.buffer_ && !parent_.buffer_->Purge(parent_.deadline_))
		ret = -EIO;
	if (!ret) {
		parent_.locked_ = false;
		parent_.have_cnr_ = false;
		parent_.cnr_skip_first_ = true;
		ret = dth11_tune(&parent_.bridge_, &parent_.info_, params_.freq * 1000U);
	}
	return ret;
}

int Dth11Device::Receiver::CheckLock(bool &locked)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	locked = false;
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	u8 state = 0;
	const int ret = dth11_read_sync_state(&parent_.bridge_, &parent_.info_, &state);
	if (!ret)
		locked = state >= DTH11_SYNC_LOCKED;
	return ret;
}

int Dth11Device::Receiver::SetCapture(bool capture)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	if (capture == parent_.buffer_started_)
		return 0;
	if (capture) {
		parent_.buffer_->Start();
		parent_.buffer_started_ = true;
		parent_.capture_ = true;
		/* 選局でロックを確認できていれば、すぐ始める。まだなら、選局の完了で始まる */
		if (parent_.locked_) {
			parent_.deadline_ = (std::chrono::steady_clock::time_point::max)();
			const int ret = parent_.StartStream();
			if (ret) {
				parent_.buffer_started_ = false;
				parent_.capture_ = false;
				parent_.buffer_->Stop();
			}
			return ret;
		}
		return 0;
	}
	const int ret = parent_.StopStream();
	parent_.capture_ = false;
	parent_.buffer_started_ = false;
	parent_.buffer_->Stop();
	return ret;
}

int Dth11Device::Receiver::ReadStat(command::StatType type, std::int32_t &value)
{
	std::lock_guard<std::recursive_mutex> lock(parent_.mutex_);
	if (!parent_.available_ || !parent_.receiver_open_)
		return -ENODEV;
	/* 信号強度は、純正の式の単位を確認していないため未対応にする。C/N は、純正の表による換算 */
	if (type != command::StatType::CNR)
		return -ENOSYS;
	struct dth11_signal signal = {};
	const int ret = dth11_read_signal(&parent_.bridge_, &parent_.info_, &signal);
	if (ret)
		return ret;
	if (!signal.locked)
		return -EAGAIN;
	/* 復調器は C/N を周期で測る。測定が済んだときだけ値が更新され、それまでは前の値を返す */
	if (signal.valid_cnr && parent_.cnr_skip_first_) {
		/*
		 * 選局の後、最初に完了した測定は、実機で 30.0 dB (表の上限) などの実際と懸け離れた値になった
		 * (ロック前の状態を含む)。次の測定 (約 1 秒後) から採用する。
		 */
		parent_.cnr_skip_first_ = false;
	} else if (signal.valid_cnr) {
		parent_.cnr_mdb_ = signal.cnr_tenth_db * 100;
		parent_.have_cnr_ = true;
		msg_log_at(PX4_LOG_DEBUG, PX4_LOG_CAT_DEMOD, "DTH11 C/N raw=%u -> %d mdB, sync=%u\n",
			   signal.cnr_raw, parent_.cnr_mdb_, signal.sync_state);
	}
	if (!parent_.have_cnr_)
		return -EAGAIN;
	value = parent_.cnr_mdb_;
	return 0;
}

int Dth11Device::Receiver::SetStreamId()
{
	return -ENOSYS;
}

bool Dth11Device::Receiver::TuneOverride(std::uint32_t timeout, bool &result)
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
	parent_.locked_ = !ret && locked;
	/*
	 * ストリームの開始 (相互認証、番組表の学習、PID リストの送信) は数秒かかる。ロックを待つ期限の外で行う
	 * (期限が残っていると、その間の制御転送が期限切れで失敗する)。
	 */
	parent_.deadline_ = (std::chrono::steady_clock::time_point::max)();
	if (parent_.locked_ && parent_.capture_) {
		const int start_ret = parent_.StartStream();
		if (start_ret) {
			ret = start_ret;
			parent_.locked_ = false;
		}
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
			   "DTH11 tune failed: freq_khz=%u locked=%u result=%d\n",
			   params_.freq, locked ? 1U : 0U, ret);
	}
	return true;
}

} // namespace px4
