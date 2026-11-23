// SPDX-License-Identifier: GPL-2.0-only

#include "xcode_card_device.hpp"

#if defined(__linux__) && defined(PX4_USERLAND)

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "msg.h"

namespace px4 {

namespace {

/* パスはASCIIの/dev/...しか扱わないため、codecvt等を使わず素直に変換する。 */
std::string NarrowAscii(const std::wstring &wide)
{
	std::string out;
	out.reserve(wide.size());
	for (wchar_t c : wide)
		out.push_back(static_cast<char>(c));
	return out;
}

} // namespace

XcodeCardDevice::XcodeCardDevice(const std::wstring &path,
				  const DeviceDefinition &definition,
				  std::uintptr_t index,
				  ReceiverManager &receiver_manager)
	: DeviceBase(path, definition, index, receiver_manager)
{
	if (!definition.receivers.empty())
		throw DeviceError("XcodeCardDevice: card-only device must not define receivers.");

	device_path_w_ = definition.configs.Get(L"DevicePath", L"/dev/vixs/xcodedrv");
	device_path_ = NarrowAscii(device_path_w_);
}

XcodeCardDevice::~XcodeCardDevice()
{
	Term();
}

int XcodeCardDevice::Init()
{
	/* 経路の存在だけを確認する。実際のfdはOpenCard()が開く(PC/SCセッションの間だけ保持する)。 */
	int fd = open(device_path_.c_str(), O_RDWR);
	if (fd < 0)
		return -errno;
	close(fd);
	msg_log_at(PX4_LOG_INFO, PX4_LOG_CAT_DEVICE,
		   "xcode internal card ready: path=%s\n", device_path_.c_str());
	return 0;
}

void XcodeCardDevice::Term()
{
	CloseCard();
	available_ = false;
}

void XcodeCardDevice::SetAvailability(bool available)
{
	available_ = available;
	if (!available)
		CloseCard();
}

ReceiverBase *XcodeCardDevice::GetReceiver(int id) const
{
	(void)id;
	throw std::out_of_range("XcodeCardDevice has no receiver.");
}

int XcodeCardDevice::Ioctl(ScardCtrl &ctrl) const
{
	if (fd_ < 0)
		return -ENODEV;
	if (ioctl(fd_, kScardIoctl, &ctrl) < 0)
		return -errno;
	return 0;
}

std::uint16_t XcodeCardDevice::ClampTimeout() const noexcept
{
	return io_timeout_ms_ > 0xffffU ? static_cast<std::uint16_t>(0xffffU) :
		static_cast<std::uint16_t>(io_timeout_ms_);
}

int XcodeCardDevice::OpenCard()
{
	if (!available_)
		return -ENODEV;
	std::lock_guard<std::mutex> lock(io_mutex_);
	bool expected = false;
	if (!card_open_.compare_exchange_strong(expected, true))
		return -EALREADY;

	fd_ = open(device_path_.c_str(), O_RDWR);
	if (fd_ < 0) {
		card_open_ = false;
		return -errno;
	}

	ScardCtrl ctrl{};
	ctrl.cmd = kCmdInit;
	ctrl.timeout_ms = ClampTimeout();
	int ret = Ioctl(ctrl);
	if (ret) {
		close(fd_);
		fd_ = -1;
		card_open_ = false;
		return ret;
	}
	return 0;
}

void XcodeCardDevice::CloseCard()
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	if (!card_open_.exchange(false))
		return;
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}
	pending_.clear();
	pending_pos_ = 0;
}

int XcodeCardDevice::DetectCard(bool &detected)
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	detected = false;
	if (!card_open_)
		return -ENODEV;

	/*
	 * MMIO 0x1034 のbit 0x100直接読みはカード未挿入でも立っており信頼できないと実機で判明している
	 * (HVT-4KBC-bcas-reverse-engineering-report.md 5.1/7b)。GET_ATRの成否だけを判定材料にする。
	 */
	ScardCtrl ctrl{};
	ctrl.cmd = kCmdGetAtr;
	ctrl.timeout_ms = ClampTimeout();
	if (Ioctl(ctrl) == 0 && ctrl.data_len > 0)
		detected = true;
	return 0;
}

int XcodeCardDevice::ResetCard()
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	if (!card_open_)
		return -ENODEV;

	/*
	 * 常に物理RESETを行う(GET_ATRキャッシュの再利用による省略はしない)。T=1のN(S)/N(R)は
	 * /dev/shm/smartcard のグローバルな tx_seq/rx_seq で追跡されており、SmartCard側が想定する
	 * フレッシュな送受信シーケンス(N(S)=N(R)=0)に合わせるにはRESETでの再同期が必要。
	 * このメソッドはSmartCard::Reset()がDetectCard()==trueを確認した後にしか呼ばれないため、
	 * カード未挿入状態でのRESET(実機再起動の原因と疑われる操作)には入らない。
	 */
	ScardCtrl reset_ctrl{};
	reset_ctrl.cmd = kCmdReset;
	reset_ctrl.timeout_ms = ClampTimeout();
	reset_ctrl.cold_warm = 1; /* 7a/7bで実証済みの値 */
	int ret = Ioctl(reset_ctrl);
	if (ret)
		return ret;

	ScardCtrl atr_ctrl{};
	atr_ctrl.cmd = kCmdGetAtr;
	atr_ctrl.timeout_ms = ClampTimeout();
	ret = Ioctl(atr_ctrl);
	if (ret)
		return ret;
	if (!atr_ctrl.data_len)
		return -EIO;

	pending_.assign(atr_ctrl.data, atr_ctrl.data + atr_ctrl.data_len);
	pending_pos_ = 0;
	return 0;
}

int XcodeCardDevice::SetCardBaudrate(std::uint32_t baudrate)
{
	if (!card_open_)
		return -ENODEV;
	/* ACAS既定の38400、従来のB-CASの9600以外は、切り替え用のioctlが見つかっていない。 */
	return (baudrate == 38400 || baudrate == 9600) ? 0 : -ENOSYS;
}

int XcodeCardDevice::IsCardDataReady(bool &ready)
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	if (!card_open_)
		return -ENODEV;
	ready = pending_pos_ < pending_.size();
	return 0;
}

int XcodeCardDevice::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	if (!card_open_)
		return -ENODEV;

	const std::size_t available_bytes = pending_.size() - pending_pos_;
	const std::size_t count = (std::min)(available_bytes, static_cast<std::size_t>(len));
	if (count)
		std::memcpy(buf, pending_.data() + pending_pos_, count);
	pending_pos_ += count;
	if (pending_pos_ >= pending_.size()) {
		pending_.clear();
		pending_pos_ = 0;
	}
	len = static_cast<std::uint8_t>(count);
	return 0;
}

int XcodeCardDevice::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
{
	std::lock_guard<std::mutex> lock(io_mutex_);
	if (!card_open_)
		return -ENODEV;

	/* lenはuint8_t(最大255)で、ctrl.dataは258バイトのため上限チェックは不要。 */
	ScardCtrl ctrl{};
	ctrl.cmd = kCmdBlock;
	if (len)
		std::memcpy(ctrl.data, buf, len);
	ctrl.data_len = len;
	ctrl.timeout_ms = ClampTimeout();

	int ret = Ioctl(ctrl);
	if (ret)
		return ret;

	pending_.assign(ctrl.data, ctrl.data + ctrl.data_len);
	pending_pos_ = 0;
	return 0;
}

int XcodeCardDevice::SetCardIoTimeout(std::uint32_t timeout_ms)
{
	if (!card_open_)
		return -ENODEV;
	io_timeout_ms_ = timeout_ms;
	return 0;
}

} // namespace px4

#endif // defined(__linux__) && defined(PX4_USERLAND)
