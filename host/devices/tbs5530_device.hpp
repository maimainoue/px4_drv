#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "device_base.hpp"
#include "itedtv_bus.h"
#include "t230c_protocol.hpp"
#include "tbs5530_core.h"

namespace px4 {

/*
 * TBS 5530 (ISDB-T)。USB ブリッジ (Cypress FX2) は不揮発のファームウェアを持たず、
 * 接続のたびに RAM へ送る必要がある。ファームウェアは配布物に含めず、利用者が
 * Tbs5530Firmware で指定したファイル (既定は DriverHost_PX4 と同じ場所の dvb-usb-id5530.fw) を読む。
 * macOS は libusb、Windows は WinUSB で制御転送を行う。
 */
class Tbs5530Device final : public DeviceBase {
public:
	Tbs5530Device(const std::wstring &path, const DeviceDefinition &definition,
		      std::uintptr_t index, ReceiverManager &receiver_manager);
	~Tbs5530Device() override;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	ReceiverBase *GetReceiver(int id) const override;

private:
	class Receiver;

	static int ControlCallback(void *context, u8 request_type, u8 request,
		u16 value, u16 index, void *buffer, u16 length);
	static void SleepCallback(void *context, unsigned int milliseconds);
	static void LogCallback(void *context, int level, const char *format, ...);
	static int StreamHandler(void *context, void *data, std::uint32_t size);

	int Control(u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length);
	int LoadFirmware();
	int StartUsbStream();
	int StopUsbStream();

	mutable std::recursive_mutex mutex_;
	std::unique_ptr<Receiver> receiver_;
	std::shared_ptr<ReceiverBase::StreamBuffer> buffer_;
	std::unique_ptr<t230c::TsSynchronizer> synchronizer_;
	struct tbs5530_bridge bridge_ = {};
	struct tbs5530_demod demod_ = {};
	struct itedtv_bus stream_bus_ = {};
	t230c::Bytes firmware_;
	std::chrono::steady_clock::time_point deadline_ =
		(std::chrono::steady_clock::time_point::max)();
	unsigned int receiver_packets_ = 16384;
	unsigned int max_urbs_ = 8;
	unsigned int urb_bytes_ = 65536;
	/* ISDB-C (J.83 Annex C) は実機で未確認のため、設定で明示的に有効にしたときだけ受信機の方式へ加える */
	bool experimental_isdbc_ = false;
	std::atomic_bool available_{true};
	std::atomic_bool stream_failed_{false};
	std::atomic_bool initialized_{false};
	/* クライアントの接続が Close() を呼ぶまで、受信機を破棄しない (抜去の後にも Close() が呼ばれる)。 */
	bool client_open_ = false;
	std::condition_variable_any close_cond_;
#if defined(_WIN32) || defined(_WIN64)
	HANDLE control_event_ = nullptr;
#endif
	bool bus_initialized_ = false;
	bool demod_ready_ = false;
	bool streaming_ = false;
	bool receiver_open_ = false;
	bool buffer_started_ = false;
};

} // namespace px4
