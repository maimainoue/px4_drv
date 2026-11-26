// SPDX-License-Identifier: GPL-2.0-only
#pragma once

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
#include "ut100b_core.h"
#include "itedtv_bus.h"

namespace px4 {

/*
 * Quixun QRS-UT100B (ASICEN ASV5211 + ASIE5606、地デジと CATV 用のチューナー、内蔵 B-CAS スロット)。
 *
 * warm (3275:7020) だけを扱う。cold (1738:5211) への firmware の送信は、HZ2W と共通の cold の定義 (Type=HZ2W) が行う。
 * cold の USB ID は ASV5211 を使う機器で共通のため、どの機種へ送る firmware かは、利用者が置いた firmware のファイルで決まる
 * (HZ2W の firmware で UT100B も起動する。docs/devices/ut100b.md)。送った後に出直した USB ID の定義が、この機種を開く。
 *
 * 構成:
 *  - 制御転送と周期処理は、HZ2W と同じ (bus_mutex_ で 1 つずつ通す。周期処理を止めると約 20 秒で機器が USB から消える)。
 *  - TS は bulk IN (0x81) を、非同期の URB で受ける。DES を外し、188 バイトに区切って、受信機のバッファへ書く。
 *  - 内蔵カードは、HZ2W の共通層 (hz2w_card) を、受信中も使える設定 (ASIE5606 を初期値へ戻さない) で使う。
 * 設計と検証の範囲は docs/devices/ut100b.md を参照。
 */
class Ut100bDevice final : public DeviceBase {
public:
	Ut100bDevice(const std::wstring &path, const DeviceDefinition &definition,
		     std::uintptr_t index, ReceiverManager &receiver_manager);
	~Ut100bDevice() override;

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
	std::uint8_t MaxCardBlockSize() const noexcept override { return HZ2W_CARD_RX_MAX; }

private:
	class Receiver;

	static int ControlCallback(void *context, u8 request_type, u8 request,
		u16 value, u16 index, void *buffer, u16 length);
	static void SleepCallback(void *context, unsigned int milliseconds);
	static u64 NowCallback(void *context);
	static void LogCallback(void *context, int level, const char *format, ...);
	static int StreamHandler(void *context, void *data, u32 size);

	int Control(u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length);
	void KeepaliveThread();
	void StopKeepalive();
	int StartUsbStream();
	int StopUsbStream();
	/* 転送の失敗が機器の消失 (-ENODEV) なら、以後の操作を止める。 */
	int NoteResult(int ret);
	void LogBytes(const char *what, const std::uint8_t *data, std::uint8_t length, std::uint8_t capacity);

	mutable std::recursive_mutex mutex_;
	/* 制御転送の列 (ガード付きの nonce の取得から読み出しまで) を、周期処理と分けて 1 つずつ通す。 */
	std::mutex bus_mutex_;
	std::thread keepalive_thread_;
	std::mutex keepalive_mutex_;
	std::condition_variable keepalive_cond_;
	bool keepalive_stop_ = false;
	struct hz2w_bridge bridge_ = {};
	struct hz2w_card card_ = {};
	struct itedtv_bus stream_bus_ = {};
	/* DES の表を持つため大きい (約 40 KB)。受信の開始のたびに、受け始めを捨てる状態へ戻す。 */
	std::unique_ptr<struct ut100b_aligner> aligner_;
	std::shared_ptr<ReceiverBase::StreamBuffer> buffer_;
	std::unique_ptr<Receiver> receiver_;
	std::chrono::steady_clock::time_point deadline_ =
		(std::chrono::steady_clock::time_point::max)();
	unsigned int receiver_packets_ = 16384;
	unsigned int max_urbs_ = UT100B_URB_COUNT;
	/*
	 * 受信中の選局では、受信を止めて選局し、ロックを確認し、復調が安定してから再開する。選局の直後の数十ミリ秒は、
	 * 復調が安定せず、TEI が付いた乱れたパケットが続くため (ロックの前から流れ始める)、ロックの確認の前に再開すると、
	 * チャンネルの切り替えの直後に TEI、連続性のエラーが出る。
	 * defer_restart_ は、TuneOverride が選局の間だけ立てる。restart_pending_ は、選局で止めた受信を再開する必要がある印。
	 */
	bool defer_restart_ = false;
	bool restart_pending_ = false;
	unsigned int lock_settle_ms_ = 300;
	bool lock_seen_ = false;
	std::chrono::steady_clock::time_point lock_time_ = {};
	bool board_open_ = false;
	/* ResetCard / WriteCardData の実行中 (bus_mutex_ の中) だけ真。制御転送の待ちを操作の期限で打ち切る。 */
	bool card_op_active_ = false;
	std::atomic_bool available_{true};
	std::atomic_bool card_open_{false};
	std::atomic_bool initialized_{false};
	std::atomic_bool stream_failed_{false};
	bool bus_initialized_ = false;
	bool streaming_ = false;
	bool receiver_open_ = false;
	bool buffer_started_ = false;
	/* クライアントが Open した受信機を、まだ Close していない。機材の抜去で receiver_open_ を落とした後も、Close が戻るまで受信機を破棄しない。 */
	bool client_open_ = false;
	std::condition_variable_any close_cond_;
#if defined(_WIN32) || defined(_WIN64)
	HANDLE control_event_ = nullptr;
#endif
};

} // namespace px4
