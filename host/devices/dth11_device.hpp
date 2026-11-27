#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_base.hpp"
#include "itedtv_bus.h"
#include "dth11_appkeys.h"
#include "dth11_cepm.h"
#include "dth11_core.h"
#include "dth11_psi.h"

namespace px4 {

/*
 * BUFFALO DT-H11/U2 (USB 0411:01da) のホスト側の実装 (地上波 1 受信機)。
 * 公式のファームウェアは要らない (機器に内蔵)。受信は device モードだけで、デバイス内蔵のカードで
 * 復号された番組の TS が、ホストとの間でだけ暗号化 (CEPM) されて届く。この暗号の相互認証に使う
 * アプリケーションの資格情報は、利用者が用意した純正の CoBDACom.dll から実行時に読み出す
 * (リポジトリには含まれない)。この機体のファームウェア (1.0.18) は、復号しない raw モードを
 * 受け付けない。内蔵カードをホストへカードリーダーとして見せる経路は見つかっていないため、
 * HasCardReader() は偽。設計と検証の範囲は docs/devices/dth11.md を参照。
 */
class Dth11Device final : public DeviceBase {
public:
	Dth11Device(const std::wstring &path, const DeviceDefinition &definition,
		    std::uintptr_t index, ReceiverManager &receiver_manager);
	~Dth11Device() override;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	ReceiverBase *GetReceiver(int id) const override;
	bool HasCardReader() const noexcept override { return false; }

private:
	class Receiver;

	static int ControlCallback(void *context, u8 request_type, u8 request, u16 value,
		u16 index, void *buffer, u16 length);
	static void SleepCallback(void *context, unsigned int milliseconds);
	static u64 NowCallback(void *context);
	static void LogCallback(void *context, int level, const char *format, ...);
	static int RngCallback(void *context, u8 *buffer, std::size_t length);
	static int StreamHandler(void *context, void *data, u32 size);

	int Control(u8 request_type, u8 request, u16 value, u16 index,
		    void *buffer, u16 length, unsigned int timeout_ms = 0);
	unsigned int ControlTimeout() const noexcept;
	int LoadAppKeys();
	/* 相互認証を 1 回行う。制御転送の排他を保持するので、他の制御転送はその間待たされる (最大数秒) */
	int RunHandshake(dth11_handshake &out);
	int EnsureHandshake();
	/* 選局済みのチャンネルのストリームを始める: PAT/PMT を読み、番組の PID リストを送って、TS の受信を始める。 */
	int StartStream();
	int StopStream();
	int LearnProgram(u32 &program, std::vector<u16> &pids);
	int StartUsbStream();
	int EnableStreamWithPids();
	void RehandshakeWorker();
	void FailStream();

	mutable std::recursive_mutex mutex_;
	/* 制御転送を 1 つずつ通す。相互認証は、手順の間に他の制御転送が入らないよう、組で保持する。 */
	std::timed_mutex control_mutex_;
	std::atomic<std::thread::id> control_owner_{};
	std::unique_ptr<Receiver> receiver_;
	std::shared_ptr<ReceiverBase::StreamBuffer> buffer_;
	struct dth11_bridge bridge_ = {};
	struct dth11_info info_ = {};
	struct dth11_appkeys keys_ = {};
	bool keys_loaded_ = false;
	struct itedtv_bus stream_bus_ = {};
	std::chrono::steady_clock::time_point deadline_ =
		(std::chrono::steady_clock::time_point::max)();

	/* 相互認証の結果。機器の電源が切られるまで有効 */
	std::mutex handshake_mutex_;
	struct dth11_handshake handshake_ = {};
	bool has_handshake_ = false;

	/*
	 * ストリームの状態。受信のスレッド (StreamHandler) と、選局の側のスレッドで共有する。
	 * decryptor_ と psi_ は dec_mutex_ で保護する。
	 */
	std::mutex dec_mutex_;
	std::condition_variable dec_cond_;
	struct dth11_decryptor decryptor_ = {};
	std::unique_ptr<struct dth11_psi> psi_;
	bool learning_ = false;
	bool rehandshake_requested_ = false;
	std::thread rehandshake_thread_;
	std::atomic_bool rehandshake_stop_{false};
	u32 program_number_ = 0;
	/* 受信のスレッドの周期の診断 (StreamHandler だけが触る)。URB の完了の間隔と、処理の所要時間を測る。 */
	struct StreamTiming {
		std::chrono::steady_clock::time_point last_call{};
		std::chrono::steady_clock::time_point next_log{};
		std::uint64_t urbs = 0;
		std::uint64_t bytes = 0;
		std::uint64_t short_urbs = 0;
		std::int64_t max_interval_us = 0;
		std::int64_t max_handler_us = 0;
	} stream_timing_;
	std::vector<u16> program_pids_;

	unsigned int receiver_packets_ = 16384;
	unsigned int max_urbs_ = 16;
	std::atomic_bool available_{true};
	std::atomic_bool stream_failed_{false};
	std::atomic_bool initialized_{false};
#if defined(_WIN32) || defined(_WIN64)
	HANDLE control_event_ = nullptr;
#endif
	bool started_ = false;
	bool bus_initialized_ = false;
	bool streaming_ = false;
	bool receiver_open_ = false;
	/* クライアントが Open した受信機を、まだ Close していない。機材の抜去の後も、Close が戻るまで受信機を破棄しない。 */
	bool client_open_ = false;
	std::condition_variable_any close_cond_;
	bool buffer_started_ = false;
	/* SetCapture(true) の状態と、選局でロックを確認できたか */
	bool capture_ = false;
	bool locked_ = false;
	std::chrono::steady_clock::time_point lock_time_ = {};
	bool have_cnr_ = false;
	/* 選局の後、最初に完了した C/N の測定は、選局の途中の状態を含む値になるため捨てる */
	bool cnr_skip_first_ = true;
	std::int32_t cnr_mdb_ = 0;
};

} // namespace px4
