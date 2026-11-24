// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_base.hpp"
#include "hz2w_core.h"

namespace px4 {

/*
 * I-O DATA GV-MVP/HZ2W (ASICEN ASV5211 + ASIE5606B) の内蔵カードリーダーだけを扱う。
 * 地上波の受信 (復調器、チューナー、TS) は未対応のため、受信機は登録しない。
 *
 * 同じクラスが 2 つの USB ID を扱う。
 *  - cold (1738:5211): 通電直後。firmware を送ると機器が再列挙するため、Init() は -ENODEV で終わる。
 *    firmware は配布物に含まれない。利用者が fwtool (asv5211fw) で自分のドライバーから取り出して置く。
 *  - warm (04bb:0526): firmware が動いている。起動の手順を済ませ、カードを開く。
 * warm の間は周期処理 (カード状態の読み出しと要求 0x22) を止めてはならない。止めると約 20 秒で機器が USB から消える。
 * 設計と検証の範囲は docs/devices/hz2w.md を参照。
 */
class Hz2wDevice final : public DeviceBase {
public:
	Hz2wDevice(const std::wstring &path, const DeviceDefinition &definition,
		   std::uintptr_t index, ReceiverManager &receiver_manager);
	~Hz2wDevice() override;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	ReceiverBase *GetReceiver(int id) const override;
	bool HasCardReader() const noexcept override { return !cold_; }
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
	static int ControlCallback(void *context, u8 request_type, u8 request,
		u16 value, u16 index, void *buffer, u16 length);
	static void SleepCallback(void *context, unsigned int milliseconds);
	static u64 NowCallback(void *context);
	static void LogCallback(void *context, int level, const char *format, ...);

	void LogBytes(const char *what, const std::uint8_t *data, std::uint8_t length, std::uint8_t capacity);
	int Control(u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length);
	int LoadFirmware();
	void KeepaliveThread();
	void StopKeepalive();
	/* 転送の失敗が機器の消失 (-ENODEV) なら、以後の操作を止める。 */
	int NoteResult(int ret);

	/* 制御転送の列 (ガード付きの nonce の取得から読み出しまで) を、周期処理と分けて 1 つずつ通す。 */
	std::mutex bus_mutex_;
	std::thread keepalive_thread_;
	std::mutex keepalive_mutex_;
	std::condition_variable keepalive_cond_;
	bool keepalive_stop_ = false;
	struct hz2w_bridge bridge_ = {};
	struct hz2w_card card_ = {};
	std::vector<std::uint8_t> firmware_;
	bool cold_ = false;
	bool board_open_ = false;
	std::atomic_bool available_{true};
	std::atomic_bool card_open_{false};
	std::atomic_bool initialized_{false};
#if defined(_WIN32) || defined(_WIN64)
	HANDLE control_event_ = nullptr;
#endif
};

} // namespace px4
