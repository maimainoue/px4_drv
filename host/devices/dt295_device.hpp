#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "device_base.hpp"
#include "itedtv_bus.h"
#include "torne_card.h"
#include "torne_core.h"
#include "dt295_card.h"
#include "dt295_core.h"

namespace px4 {

/*
 * PIXELA PIX-DT295 (USB 06b8:103b / 06b8:105c) のホスト側の実装 (地上波 1 受信機と内蔵カード)。
 * 公式ファームウェアを使わず、共通層 (driver/common/dt295) がホストから SoC を直接操作する。
 * 設計と検証の範囲は docs/devices/dt295.md を参照。
 */
class Dt295Device final : public DeviceBase {
public:
	Dt295Device(const std::wstring &path, const DeviceDefinition &definition,
		 std::uintptr_t index, ReceiverManager &receiver_manager);
	~Dt295Device() override;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	ReceiverBase *GetReceiver(int id) const override;
	bool HasCardReader() const noexcept override { return true; }
	int OpenCard() override;
	void CloseCard() override;
	int DetectCard(bool &detected) override;
	int ResetCard() override;
	int SetCardBaudrate(std::uint32_t baudrate) override;
	int IsCardDataReady(bool &ready) override;
	int ReadCardData(std::uint8_t *buf, std::uint8_t &len) override;
	int WriteCardData(const std::uint8_t *buf, std::uint8_t len) override;
	int SetCardIoTimeout(std::uint32_t timeout_ms) override;

private:
	class Receiver;

	static int ControlCallback(void *context, u8 request_type, u16 value,
		u16 index, void *buffer, u16 length, unsigned int timeout_ms);
	static int ControlLockedCallback(void *context, u8 request_type, u16 value,
		u16 index, void *buffer, u16 length, unsigned int timeout_ms);
	static int LockCallback(void *context);
	static void UnlockCallback(void *context);
	static void SleepCallback(void *context, unsigned int milliseconds);
	static u64 NowCallback(void *context);
	static void LogCallback(void *context, int level, const char *format, ...);
	static int CardRead32Callback(void *context, u32 address, u32 *value,
		unsigned int timeout_ms);
	static int CardWrite32Callback(void *context, u32 address, u32 value,
		unsigned int timeout_ms);
	static int CardLockCallback(void *context);
	static void CardUnlockCallback(void *context);
	static void CardSleepCallback(void *context, unsigned int milliseconds);
	static u64 CardNowCallback(void *context);
	static int StreamHandler(void *context, void *data, u32 size);

	/* locked が真のときは、呼び出し側が control_mutex_ を保持している (ポンプの要求と応答の組。LockCallback で取る)。 */
	int Control(u8 request_type, u16 value, u16 index, void *buffer, u16 length,
		    unsigned int timeout_ms = 0, bool locked = false);
	int CardRead32(u32 address, u32 *value, unsigned int timeout_ms);
	int CardWrite32(u32 address, u32 value, unsigned int timeout_ms);
	unsigned int ControlTimeout() const noexcept;
	void DrainStream();
	int StartUsbStream();
	int StopUsbStream();
	void PumpThread();

	mutable std::recursive_mutex mutex_;
	/*
	 * 制御転送を 1 つずつ通す。PIX-DT295 のポンプの要求と応答は、同じ 64 バイトの場所 (0xC4620) を通るので、
	 * 組の間は、共通層が lock / unlock で保持する (組の間に別の制御転送が入ると壊れる)。
	 */
	std::timed_mutex control_mutex_;
	std::mutex card_uart_mutex_;
	std::unique_ptr<Receiver> receiver_;
	std::shared_ptr<ReceiverBase::StreamBuffer> buffer_;
	struct dt295_bridge bridge_ = {};
	struct torne_card_port card_port_ = {};
	struct torne_framer framer_ = {};
	struct itedtv_bus stream_bus_ = {};
	std::thread pump_thread_;
	std::atomic_bool pump_stop_{false};
	std::chrono::steady_clock::time_point deadline_ =
		(std::chrono::steady_clock::time_point::max)();
	/* 選局の後、最初にロックを確認した時刻。ロックの直後は、復調が安定するまで TEI 付きのパケットが出るので、受信の開始を遅らせる。 */
	std::chrono::steady_clock::time_point lock_time_ = {};
	bool lock_seen_ = false;
	/*
	 * 受信中の選局では、受信を止めて選局し、ロックを確認し、復調が安定してから再開する (選局の途中で再開すると、ロックまでの
	 * 乱れた TS がホストのバッファへ入り、チャンネルの切り替え直後に同期・TEI・連続性のエラーが出る)。
	 * defer_restart_ は、TuneOverride が選局の間だけ立てる。restart_pending_ は、選局で止めた受信を再開する必要がある印。
	 */
	bool defer_restart_ = false;
	bool restart_pending_ = false;
	unsigned int receiver_packets_ = 16384;
	unsigned int max_urbs_ = 16;
	std::atomic_bool available_{true};
	std::atomic_bool card_open_{false};
	std::atomic_bool stream_failed_{false};
	std::atomic_bool initialized_{false};
	/* hook を元へ戻せなかった。機材を挿し直すまで、受信を開始しない。 */
	std::atomic_bool needs_replug_{false};
#if defined(_WIN32) || defined(_WIN64)
	HANDLE control_event_ = nullptr;
#endif
	bool board_up_ = false;
	bool frontend_initialized_ = false;
	bool bus_initialized_ = false;
	bool streaming_ = false;
	bool receiver_open_ = false;
	/* クライアントが Open した受信機を、まだ Close していない。機材の抜去で receiver_open_ を落とした後も、Close が戻るまで受信機を破棄しない。 */
	bool client_open_ = false;
	std::condition_variable_any close_cond_;
	bool buffer_started_ = false;
};

} // namespace px4
