// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#if defined(__linux__) && defined(PX4_USERLAND)

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "device_base.hpp"

namespace px4 {

/*
 * VIXS XCode6 SoC搭載STB(I-O DATA HVT-4KBC、SHARP 4S-C00AS1など、同一バイナリを共有する
 * OEM機種群)の内蔵ACAS/B-CASリーダー。本機自身の /dev/vixs/xcodedrv を直接制御する
 * CardDeviceで、チューナー/受信機は提供しない(定義にReceiverを持たないカード専用機種)。
 * ioctl構造体・cmd値は C:\tmp\ch341a\HVT-4KBC-bcas-reverse-engineering-report.md の逆解析結果による。
 * DeviceManagerのUSB VID/PID探索を経由せず、固定パス(Config:DevicePath)で1回だけ登録される。
 */
class XcodeCardDevice final : public DeviceBase {
public:
	XcodeCardDevice(const std::wstring &path, const DeviceDefinition &definition,
			std::uintptr_t index, ReceiverManager &receiver_manager);
	~XcodeCardDevice() override;

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
	/* ioctl(fd, 0x25, &scard_ctrl) の271バイト構造体。レイアウトは実証済みの scard_test.c / apdu_test.c と同一。 */
	struct ScardCtrl final {
		std::uint32_t cmd;
		std::uint8_t data[258];
		std::uint32_t data_len;
		std::uint16_t timeout_ms;
		std::uint16_t reserved_10c;
		std::uint8_t cold_warm;
	} __attribute__((packed));

	static constexpr int kScardIoctl = 0x25;
	static constexpr std::uint32_t kCmdInit = 1;
	static constexpr std::uint32_t kCmdReset = 2;
	static constexpr std::uint32_t kCmdBlock = 4;
	static constexpr std::uint32_t kCmdGetAtr = 5;

	int Ioctl(ScardCtrl &ctrl) const;
	std::uint16_t ClampTimeout() const noexcept;

	std::mutex io_mutex_;
	std::wstring device_path_w_;
	std::string device_path_;
	int fd_ = -1;
	std::vector<std::uint8_t> pending_;
	std::size_t pending_pos_ = 0;
	std::uint32_t io_timeout_ms_ = 1000;
	std::atomic_bool available_{true};
	std::atomic_bool card_open_{false};
};

} // namespace px4

#endif // defined(__linux__) && defined(PX4_USERLAND)
