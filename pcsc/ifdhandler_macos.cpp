// SPDX-License-Identifier: GPL-2.0-only
/* macOS の PC/SC IFD ハンドラー。カード操作は起動中の DriverHost_PX4 に委譲する。 */
extern "C" {
#include <wintypes.h>
/* Apple の wintypes.h が RESPONSECODE を定義するため、上流 IFD ヘッダー側の重複定義を止める。 */
#define RESPONSECODE_DEFINED_IN_WINTYPES_H
#include <ifdhandler.h>
}

/* PC/SC のマクロを避け、IPC 契約の名前付き定数を使う。 */
#undef MAX_ATR_SIZE

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <map>
#include <memory>
#include <set>
#include <mutex>
#include <string>
#include <syslog.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "card_command.hpp"

namespace {

constexpr unsigned char MAX_READERS = 16;
/* DriverHost_PX4 の内蔵カードリーダーの名前の前半。後ろに USB デバイスのパスから作る 16 桁の ID が続く。 */
constexpr const wchar_t *READER_PREFIXES[] = {
	L"Sony CECH-ZD1 J Smart Card Reader (ID: ",
	L"I-O DATA GV-MVP/XZ Smart Card Reader (ID: ",
	L"PIXELA PIX-DT295 Rev.01 Smart Card Reader (ID: ",
	L"PIXELA PIX-DT295 Rev.02 Smart Card Reader (ID: ",
	L"I-O DATA GV-MVP/HZ2W Smart Card Reader (ID: ",
};

struct Reader {
	std::mutex lock;
	int fd = -1;
	std::wstring name;
	std::vector<std::uint8_t> atr;
	bool connected = true;
	bool card_connected = false;
	bool powered = false;
	DWORD protocol = SCARD_PROTOCOL_UNDEFINED;

	~Reader()
	{
		if (fd >= 0)
			close(fd);
	}
};

std::mutex readers_lock;
std::map<DWORD, std::shared_ptr<Reader>> readers;
/* リーダーの選択から登録までを 1 つずつ行い、同じリーダーを 2 つのチャンネルへ割り当てない。 */
std::mutex select_lock;

std::string CardSocketPath()
{
	const char *runtime_dir = std::getenv("PX4_RUNTIME_DIR");
	std::string path = runtime_dir && runtime_dir[0] ? runtime_dir : "/tmp";
	if (path.back() != '/')
		path.push_back('/');
	path += "px4_card_pipe.sock";
	return path;
}

bool SendAll(int fd, const void *buffer, std::size_t length)
{
	const auto *data = static_cast<const std::uint8_t *>(buffer);
	while (length) {
		const ssize_t written = send(fd, data, length, 0);
		if (written <= 0)
			return false;
		data += written;
		length -= static_cast<std::size_t>(written);
	}
	return true;
}

bool ReceiveAll(int fd, void *buffer, std::size_t length)
{
	auto *data = static_cast<std::uint8_t *>(buffer);
	while (length) {
		const ssize_t received = recv(fd, data, length, 0);
		if (received <= 0)
			return false;
		data += received;
		length -= static_cast<std::size_t>(received);
	}
	return true;
}

int ConnectCardServer()
{
	const std::string path = CardSocketPath();
	if (path.size() >= sizeof(((sockaddr_un *)nullptr)->sun_path))
		return -1;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
	do {
		const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
		if (fd < 0)
			return -1;
		int no_sigpipe = 1;
		if (setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe)) < 0) {
			close(fd);
			return -1;
		}
		sockaddr_un address{};
		address.sun_family = AF_UNIX;
		std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
		if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0)
			return fd;
		const int error = errno;
		close(fd);
		if (error != ENOENT && error != ECONNREFUSED)
			return -1;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (std::chrono::steady_clock::now() < deadline);
	return -1;
}

bool Exchange(Reader &reader, px4::card_command::Command &command)
{
	if (reader.fd < 0 || !SendAll(reader.fd, &command, sizeof(command)) ||
	    !ReceiveAll(reader.fd, &command, sizeof(command))) {
		if (reader.fd >= 0)
			close(reader.fd);
		reader.fd = -1;
		reader.card_connected = false;
		reader.powered = false;
		reader.protocol = SCARD_PROTOCOL_UNDEFINED;
		reader.atr.clear();
		return false;
	}
	return command.status == px4::card_command::Status::SUCCEEDED;
}

RESPONSECODE MapError(const px4::card_command::Command &command)
{
	if (command.status == px4::card_command::Status::SUCCEEDED)
		return IFD_SUCCESS;
	switch (command.error) {
	case px4::card_command::Error::NO_READER:
	case px4::card_command::Error::REMOVED:
		return IFD_NO_SUCH_DEVICE;
	case px4::card_command::Error::NO_CARD:
		return IFD_ICC_NOT_PRESENT;
	case px4::card_command::Error::TIMEOUT:
		return IFD_RESPONSE_TIMEOUT;
	case px4::card_command::Error::INSUFFICIENT_BUFFER:
		return IFD_ERROR_INSUFFICIENT_BUFFER;
	case px4::card_command::Error::PROTOCOL:
		return IFD_COMMUNICATION_ERROR;
	case px4::card_command::Error::BUSY:
		return IFD_COMMUNICATION_ERROR;
	case px4::card_command::Error::INVALID_PARAMETER:
		return IFD_COMMUNICATION_ERROR;
	default:
		return IFD_COMMUNICATION_ERROR;
	}
}

std::wstring ReaderId(const std::wstring &name)
{
	for (const wchar_t *prefix : READER_PREFIXES) {
		const std::size_t length = std::wcslen(prefix);
		if (name.compare(0, length, prefix) == 0 && name.size() >= length + 16)
			return name.substr(length, 16);
	}
	return {};
}

bool ParseDeviceBusAddress(const char *device_name, std::wstring &path)
{
	/* IFDHCreateChannelByName の USB 名は OS ごとに異なる。末尾の BUS:ADDRESS が
	 * DriverHost_PX4 が使う BUS:ADDRESS と一致する場合だけ複数個体の対応に使う。 */
	if (!device_name)
		return false;
	const std::string value(device_name);
	for (std::size_t end = value.size(); end > 0; --end) {
		if (value[end - 1] < '0' || value[end - 1] > '9')
			continue;
		std::size_t address_begin = end - 1;
		while (address_begin && value[address_begin - 1] >= '0' && value[address_begin - 1] <= '9')
			--address_begin;
		if (!address_begin || (value[address_begin - 1] != ':' && value[address_begin - 1] != '/'))
			continue;
		std::size_t bus_end = address_begin - 1;
		std::size_t bus_begin = bus_end;
		while (bus_begin && value[bus_begin - 1] >= '0' && value[bus_begin - 1] <= '9')
			--bus_begin;
		if (bus_begin == bus_end || !bus_begin ||
		    (value[bus_begin - 1] != ':' && value[bus_begin - 1] != '/'))
			continue;
		const std::string bus_text = value.substr(bus_begin, bus_end - bus_begin);
		const std::string address_text = value.substr(address_begin, end - address_begin);
		char *parse_end = nullptr;
		errno = 0;
		const unsigned long bus = std::strtoul(bus_text.c_str(), &parse_end, 10);
		if (errno || !parse_end || *parse_end)
			continue;
		errno = 0;
		const unsigned long address = std::strtoul(address_text.c_str(), &parse_end, 10);
		if (errno || !parse_end || *parse_end)
			continue;
		path = std::to_wstring(bus) + L":" + std::to_wstring(address);
		return true;
	}
	return false;
}

std::wstring HashDevicePath(const std::wstring &path)
{
	std::uint64_t hash = 14695981039346656037ULL;
	for (wchar_t character : path) {
		hash ^= static_cast<std::uint16_t>(character);
		hash *= 1099511628211ULL;
	}
	wchar_t id[17] = {};
	swprintf(id, 17, L"%016llX", static_cast<unsigned long long>(hash));
	return id;
}

/* リーダー名の「機種名」の部分 (" (ID: " の前)。Apple の IFD ホストは、IFDHCreateChannelByName の
 * DeviceName に USB のパスではなく、Info.plist の ifdFriendlyName をそのまま渡す。 */
std::wstring ModelName(const wchar_t *prefix)
{
	std::wstring model(prefix);
	const std::wstring::size_type id = model.find(L" (ID: ");
	return id == std::wstring::npos ? model : model.substr(0, id);
}

bool IsModelName(const std::wstring &name)
{
	for (const wchar_t *prefix : READER_PREFIXES)
		if (name == ModelName(prefix))
			return true;
	return false;
}

struct UsbDevice {
	std::uint32_t location = 0;
	std::uint32_t address = 0;
	std::uint64_t entry_id = 0; /* IOKit の登録 ID。USB 機器が接続された順に増える。 */
};

bool ReadUsbNumber(io_service_t service, const char *key, std::uint32_t &value)
{
	CFStringRef name = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
	if (!name)
		return false;
	CFTypeRef property = IORegistryEntryCreateCFProperty(service, name, kCFAllocatorDefault, 0);
	CFRelease(name);
	if (!property)
		return false;
	bool ok = false;
	if (CFGetTypeID(property) == CFNumberGetTypeID())
		ok = CFNumberGetValue(static_cast<CFNumberRef>(property), kCFNumberSInt32Type, &value);
	CFRelease(property);
	return ok;
}

/* 接続中の USB 機器のうち VID/PID が一致するものを、接続された順 (IOKit の登録 ID の昇順) で返す。
 * DriverHost_PX4 のパス (libusb の BUS:ADDRESS) は、バスが locationID の上位 8 ビット、
 * アドレスが IOKit の "USB Address" と一致する。 */
std::vector<UsbDevice> EnumerateUsbDevices(unsigned vendor, unsigned product)
{
	std::vector<UsbDevice> devices;
	io_iterator_t iterator = IO_OBJECT_NULL;
	if (IOServiceGetMatchingServices(0, IOServiceMatching("IOUSBHostDevice"), &iterator) != KERN_SUCCESS)
		return devices;
	while (io_service_t service = IOIteratorNext(iterator)) {
		std::uint32_t id_vendor = 0, id_product = 0;
		UsbDevice device;
		if (ReadUsbNumber(service, "idVendor", id_vendor) && ReadUsbNumber(service, "idProduct", id_product) &&
		    id_vendor == vendor && id_product == product &&
		    ReadUsbNumber(service, "locationID", device.location) &&
		    ReadUsbNumber(service, "USB Address", device.address) &&
		    IORegistryEntryGetRegistryEntryID(service, &device.entry_id) == KERN_SUCCESS)
			devices.push_back(device);
		IOObjectRelease(service);
	}
	IOObjectRelease(iterator);
	std::sort(devices.begin(), devices.end(), [](const UsbDevice &a, const UsbDevice &b) {
		return a.entry_id < b.entry_id;
	});
	return devices;
}

struct ModelUsbId {
	const wchar_t *name;
	unsigned vendor;
	unsigned product;
};
constexpr ModelUsbId MODEL_USB_IDS[] = {
	{ L"Sony CECH-ZD1 J Smart Card Reader", 0x054c, 0x04b2 },
	{ L"I-O DATA GV-MVP/XZ Smart Card Reader", 0x04bb, 0x0530 },
	{ L"PIXELA PIX-DT295 Rev.01 Smart Card Reader", 0x06b8, 0x103b },
	{ L"PIXELA PIX-DT295 Rev.02 Smart Card Reader", 0x06b8, 0x105c },
	{ L"I-O DATA GV-MVP/HZ2W Smart Card Reader", 0x04bb, 0x0526 },
};

enum class Selection { SELECTED, NOT_YET, FAILED };

/* DriverHost_PX4 の内蔵カードリーダーの一覧から 1 台を選ぶ。NOT_YET は、該当するリーダーが
 * まだ一覧にない状態で、DriverHost_PX4 の機器の初期化が終わる前の問い合わせで起こる。 */
Selection SelectReaderOnce(Reader &reader, const char *device_name,
			   const std::set<std::wstring> &claimed)
{
	px4::card_command::Command command{};
	command.code = px4::card_command::Code::LIST_READERS;
	if (!Exchange(reader, command) || command.reader_count > px4::card_command::MAX_READERS)
		return Selection::FAILED;

	std::vector<std::wstring> candidates;
	for (std::uint32_t index = 0; index < command.reader_count; ++index) {
		if (!std::wmemchr(command.readers[index], L'\0', px4::card_command::MAX_READER_NAME))
			return Selection::FAILED;
		const std::wstring name(command.readers[index]);
		if (!ReaderId(name).empty())
			candidates.push_back(name);
	}
	if (candidates.empty())
		return Selection::NOT_YET;

	std::wstring device_path;
	if (ParseDeviceBusAddress(device_name, device_path)) {
		const std::wstring wanted_id = HashDevicePath(device_path);
		for (const auto &candidate : candidates)
			if (ReaderId(candidate) == wanted_id) {
				if (claimed.count(candidate))
					return Selection::FAILED;
				reader.name = candidate;
				return Selection::SELECTED;
		}
	}

	/* DeviceName が機種名のとき、その機種の内蔵カードリーダーが 1 台だけならそれを選ぶ。
	 * 同じ機種が複数台あると、機種名だけでは個体を区別できないため、誤ったカードへ接続せず失敗させる。
	 * 機種名なのに該当するリーダーがない場合に、別の機種のリーダーへ接続してはならない。 */
	std::wstring model;
	for (const char *c = device_name ? device_name : ""; *c; ++c)
		model.push_back(static_cast<unsigned char>(*c));
	if (IsModelName(model)) {
		const std::wstring prefix = model + L" (ID: ";
		std::vector<std::wstring> same_model;
		for (const auto &candidate : candidates)
			if (candidate.compare(0, prefix.size(), prefix) == 0)
				same_model.push_back(candidate);
		/* 接続中の同じ機種の USB 機器の数。DriverHost_PX4 の一覧がそれに揃うまで待つ。 */
		std::vector<UsbDevice> usb;
		for (const auto &entry : MODEL_USB_IDS)
			if (model == entry.name)
				usb = EnumerateUsbDevices(entry.vendor, entry.product);
		if (!usb.empty() && same_model.size() < usb.size())
			return Selection::NOT_YET;
		if (same_model.empty())
			return Selection::NOT_YET;
		if (same_model.size() == 1) {
			/* 別のチャンネルが割り当て済みなら、同じ機種のもう 1 台が一覧に載るのを待つ。 */
			if (claimed.count(same_model.front()))
				return Selection::NOT_YET;
			reader.name = same_model.front();
			return Selection::SELECTED;
		}
		/* 同じ機種が複数台のとき、Apple の IFD ホストは個体を区別できる情報を渡さない。
		 * 割り当て済みでないリーダーを持つ USB 機器のうち、最も早く接続されたものを選ぶ。
		 * 1 台ずつ順に接続すれば、未割り当ての機器は常に 1 台で、順序を仮定せずに決まる。
		 * 複数台を同時に接続したときだけ、Apple が機器を見つける順は接続順と仮定する。 */
		for (const auto &device : usb) {
			const std::wstring path = std::to_wstring(device.location >> 24) + L":" + std::to_wstring(device.address);
			const std::wstring wanted_id = HashDevicePath(path);
			for (const auto &name : same_model)
				if (ReaderId(name) == wanted_id && !claimed.count(name)) {
					syslog(LOG_NOTICE, "unclaimed usb device: location 0x%08x address %u entry %llu",
					       device.location, device.address,
					       static_cast<unsigned long long>(device.entry_id));
					reader.name = name;
					return Selection::SELECTED;
				}
		}
		syslog(LOG_ERR, "Could not map '%s': no unclaimed reader (%zu readers, %zu usb devices)",
		       device_name, same_model.size(), usb.size());
		return Selection::FAILED;
	}

	if (candidates.size() == 1 && !claimed.count(candidates.front())) {
		reader.name = candidates.front();
		return Selection::SELECTED;
	}
	syslog(LOG_ERR, "Could not map macOS USB device '%s' to one of %zu px4 readers",
		device_name ? device_name : "", candidates.size());
	return Selection::FAILED;
}

/* リーダーを選び、同じ排他の中で Lun へ登録する。登録済みのリーダーは別のチャンネルへ割り当てない。 */
bool SelectReader(DWORD lun, const std::shared_ptr<Reader> &reader_ptr, const char *device_name)
{
	Reader &reader = *reader_ptr;
	/* 機器の初期化 (XZ と PIX-DT295 は基板の起動に数秒かかる) を待つ絶対期限。 */
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
	for (;;) {
		Selection selection;
		{
			std::lock_guard<std::mutex> select_guard(select_lock);
			std::set<std::wstring> claimed;
			{
				std::lock_guard<std::mutex> lock(readers_lock);
				for (const auto &entry : readers)
					claimed.insert(entry.second->name);
			}
			selection = SelectReaderOnce(reader, device_name, claimed);
			if (selection == Selection::SELECTED) {
				std::lock_guard<std::mutex> lock(readers_lock);
				if (readers.size() >= MAX_READERS || readers.count(lun))
					return false;
				readers.emplace(lun, reader_ptr);
				return true;
			}
		}
		if (selection == Selection::FAILED || std::chrono::steady_clock::now() >= deadline) {
			if (selection == Selection::NOT_YET)
				syslog(LOG_ERR, "No px4 reader for '%s' within 15 s", device_name ? device_name : "");
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

std::shared_ptr<Reader> FindReader(DWORD lun)
{
	std::lock_guard<std::mutex> lock(readers_lock);
	auto found = readers.find(lun);
	return found == readers.end() ? nullptr : found->second;
}

template<class Function>
RESPONSECODE WithReader(DWORD lun, Function function)
{
	try {
		auto reader = FindReader(lun);
		if (!reader)
			return IFD_NO_SUCH_DEVICE;
		std::lock_guard<std::mutex> lock(reader->lock);
		if (!reader->connected)
			return IFD_NO_SUCH_DEVICE;
		try {
			return function(*reader);
		} catch (...) {
			return IFD_COMMUNICATION_ERROR;
		}
	} catch (...) {
		return IFD_COMMUNICATION_ERROR;
	}
}

px4::card_command::Command MakeCommand(px4::card_command::Code code,
					      const Reader &reader)
{
	px4::card_command::Command command{};
	command.code = code;
	const std::size_t length = std::min(reader.name.size(),
		px4::card_command::MAX_READER_NAME - 1);
	std::wmemcpy(command.reader_name, reader.name.data(), length);
	return command;
}

void SaveAtr(Reader &reader, const px4::card_command::Command &command)
{
	if (command.atr_length <= px4::card_command::MAX_ATR_SIZE)
		reader.atr.assign(command.atr, command.atr + command.atr_length);
	else
		reader.atr.clear();
}

} // namespace

extern "C" RESPONSECODE IFDHCreateChannelByName(DWORD Lun, LPSTR DeviceName)
{
	if ((Lun & 0xffff) != 0)
		return IFD_NO_SUCH_DEVICE;
	try {
		auto reader = std::make_shared<Reader>();
		reader->fd = ConnectCardServer();
		if (reader->fd < 0 || !SelectReader(Lun, reader, DeviceName)) {
			syslog(LOG_ERR, "IFDHCreateChannelByName: Lun=0x%x DeviceName='%s' was not mapped",
			       static_cast<unsigned>(Lun), DeviceName ? DeviceName : "");
			return IFD_NO_SUCH_DEVICE;
		}
		/* syslog の %ls は出力されないため、リーダー名 (ASCII) を変換して出す。 */
		std::string mapped;
		for (wchar_t character : reader->name)
			mapped.push_back(character < 0x80 ? static_cast<char>(character) : '?');
		syslog(LOG_NOTICE, "IFDHCreateChannelByName: Lun=0x%x DeviceName='%s' -> %s",
		       static_cast<unsigned>(Lun), DeviceName ? DeviceName : "", mapped.c_str());
		return IFD_SUCCESS;
	} catch (...) {
		return IFD_COMMUNICATION_ERROR;
	}
}

extern "C" RESPONSECODE IFDHCreateChannel(DWORD, DWORD)
{
	return IFD_NOT_SUPPORTED;
}

extern "C" RESPONSECODE IFDHCloseChannel(DWORD Lun)
{
	std::shared_ptr<Reader> reader;
	{
		std::lock_guard<std::mutex> lock(readers_lock);
		auto found = readers.find(Lun);
		if (found == readers.end())
			return IFD_NO_SUCH_DEVICE;
		reader = found->second;
		readers.erase(found);
	}
	std::lock_guard<std::mutex> lock(reader->lock);
	if (reader->card_connected) {
		auto command = MakeCommand(px4::card_command::Code::DISCONNECT, *reader);
		(void)Exchange(*reader, command);
	}
	reader->connected = false;
	return IFD_SUCCESS;
}

extern "C" RESPONSECODE IFDHGetCapabilities(DWORD Lun, DWORD Tag,
	PDWORD Length, PUCHAR Value)
{
	if (!Length)
		return IFD_COMMUNICATION_ERROR;
	const DWORD capacity = *Length;
	*Length = 0;
	/* 同時に扱えるリーダー数はドライバー全体の値で、リーダーの登録前にも問い合わせられる。
	 * Apple の IFD ホストは最初のチャンネルを開く前に問い合わせ、0 が返ると以後のリーダーを拒否する。 */
	if (Tag == TAG_IFD_SIMULTANEOUS_ACCESS) {
		if (capacity < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		if (!Value)
			return IFD_COMMUNICATION_ERROR;
		*Value = MAX_READERS;
		*Length = 1;
		return IFD_SUCCESS;
	}
	return WithReader(Lun, [&](Reader &reader) -> RESPONSECODE {
		unsigned char value = 1;
		const void *data = &value;
		DWORD size = 1;
		if (Tag == TAG_IFD_ATR) {
			data = reader.atr.data();
			size = static_cast<DWORD>(reader.atr.size());
		} else if (Tag != TAG_IFD_THREAD_SAFE && Tag != TAG_IFD_SLOT_THREAD_SAFE &&
			   Tag != TAG_IFD_SLOTS_NUMBER) {
			return IFD_ERROR_TAG;
		}
		*Length = size;
		if (capacity < size)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		if (size && !Value)
			return IFD_COMMUNICATION_ERROR;
		if (size)
			std::memcpy(Value, data, size);
		return IFD_SUCCESS;
	});
}

extern "C" RESPONSECODE IFDHSetCapabilities(DWORD, DWORD, DWORD, PUCHAR)
{
	return IFD_ERROR_TAG;
}

extern "C" RESPONSECODE IFDHICCPresence(DWORD Lun)
{
	return WithReader(Lun, [](Reader &reader) -> RESPONSECODE {
		auto command = MakeCommand(px4::card_command::Code::STATUS, reader);
		if (!Exchange(reader, command)) {
			reader.powered = false;
			reader.protocol = SCARD_PROTOCOL_UNDEFINED;
			reader.atr.clear();
			return MapError(command);
		}
		SaveAtr(reader, command);
		if (!command.card_present) {
			reader.powered = false;
			reader.protocol = SCARD_PROTOCOL_UNDEFINED;
		}
		return command.card_present ? IFD_ICC_PRESENT : IFD_ICC_NOT_PRESENT;
	});
}

extern "C" RESPONSECODE IFDHPowerICC(DWORD Lun, DWORD Action,
	PUCHAR Atr, PDWORD AtrLength)
{
	if (!AtrLength)
		return IFD_COMMUNICATION_ERROR;
	/* ATR のバッファは常に MAX_ATR_SIZE バイトで、*AtrLength は出力用の値。Apple の IFD ホストは入力時に
	 * 0 を渡すため、これを容量として扱うとすべてのカードのリセットが失敗する。 */
	*AtrLength = 0;
	return WithReader(Lun, [&](Reader &reader) -> RESPONSECODE {
		if (Action == IFD_POWER_DOWN) {
			/* DriverHost の CONNECT を保ち、PC/SC の停止とカード参照の寿命を分ける。 */
			reader.powered = false;
			reader.protocol = SCARD_PROTOCOL_UNDEFINED;
			reader.atr.clear();
			return IFD_SUCCESS;
		}
		if (Action != IFD_POWER_UP && Action != IFD_RESET) {
			syslog(LOG_ERR, "IFDHPowerICC: unsupported action %u", static_cast<unsigned>(Action));
			return IFD_NOT_SUPPORTED;
		}
		if (!Atr)
			return IFD_COMMUNICATION_ERROR;
		const bool newly_connected = !reader.card_connected;
		if (newly_connected) {
			auto connect = MakeCommand(px4::card_command::Code::CONNECT, reader);
			connect.share_mode = static_cast<std::uint32_t>(px4::card_command::ShareMode::SHARED);
			if (!Exchange(reader, connect)) {
				syslog(LOG_ERR, "IFDHPowerICC: CONNECT failed, error=%u", static_cast<unsigned>(connect.error));
				return MapError(connect);
			}
			reader.card_connected = true;
		}
		auto command = MakeCommand(newly_connected ?
			px4::card_command::Code::STATUS : px4::card_command::Code::RESET, reader);
		if (!Exchange(reader, command)) {
			reader.powered = false;
			reader.protocol = SCARD_PROTOCOL_UNDEFINED;
			reader.atr.clear();
			syslog(LOG_ERR, "IFDHPowerICC: %s failed, error=%u", newly_connected ? "STATUS" : "RESET",
			       static_cast<unsigned>(command.error));
			return MapError(command);
		}
		if (newly_connected && (!command.card_present || !command.card_initialized)) {
			syslog(LOG_ERR, "IFDHPowerICC: card present=%d initialized=%d", command.card_present, command.card_initialized);
			reader.powered = false;
			reader.protocol = SCARD_PROTOCOL_UNDEFINED;
			reader.atr.clear();
			return IFD_ICC_NOT_PRESENT;
		}
		if (command.atr_length > px4::card_command::MAX_ATR_SIZE)
			return IFD_COMMUNICATION_ERROR;
		SaveAtr(reader, command);
		reader.powered = true;
		reader.protocol = SCARD_PROTOCOL_UNDEFINED;
		std::memcpy(Atr, reader.atr.data(), reader.atr.size());
		*AtrLength = static_cast<DWORD>(reader.atr.size());
		return IFD_SUCCESS;
	});
}

extern "C" RESPONSECODE IFDHSetProtocolParameters(DWORD Lun, DWORD Protocol,
	UCHAR Flags, UCHAR, UCHAR, UCHAR)
{
	return WithReader(Lun, [&](Reader &reader) -> RESPONSECODE {
		if (Protocol != SCARD_PROTOCOL_T1)
			return IFD_PROTOCOL_NOT_SUPPORTED;
		if (Flags)
			return IFD_ERROR_PTS_FAILURE;
		if (!reader.powered || !reader.card_connected)
			return IFD_COMMUNICATION_ERROR;
		reader.protocol = Protocol;
		return IFD_SUCCESS;
	});
}

extern "C" RESPONSECODE IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci,
	PUCHAR TxBuffer, DWORD TxLength, PUCHAR RxBuffer, PDWORD RxLength,
	PSCARD_IO_HEADER RecvPci)
{
	if (!RxLength)
		return IFD_COMMUNICATION_ERROR;
	/* 受信バッファが DriverHost_PX4 の 1 回の応答の上限より大きいのは正常で、その分は使わない。 */
	const DWORD capacity = (std::min)(*RxLength,
		static_cast<DWORD>(px4::card_command::MAX_DATA_SIZE));
	*RxLength = 0;
	return WithReader(Lun, [&](Reader &reader) -> RESPONSECODE {
		if (SendPci.Protocol != 1 || reader.protocol != SCARD_PROTOCOL_T1)
			return IFD_PROTOCOL_NOT_SUPPORTED;
		if (!reader.powered || !reader.card_connected || !TxBuffer || !TxLength ||
		    !RxBuffer || !capacity || TxLength > px4::card_command::MAX_DATA_SIZE)
			return IFD_COMMUNICATION_ERROR;
		auto command = MakeCommand(px4::card_command::Code::TRANSMIT, reader);
		command.data_length = TxLength;
		command.data_capacity = capacity;
		std::memcpy(command.data, TxBuffer, TxLength);
		if (!Exchange(reader, command))
			return MapError(command);
		if (command.data_length > capacity || command.data_length > px4::card_command::MAX_DATA_SIZE)
			return IFD_COMMUNICATION_ERROR;
		std::memcpy(RxBuffer, command.data, command.data_length);
		*RxLength = command.data_length;
		if (RecvPci) {
			RecvPci->Protocol = SCARD_PROTOCOL_T1;
			RecvPci->Length = sizeof(*RecvPci);
		}
		return IFD_SUCCESS;
	});
}

extern "C" RESPONSECODE IFDHControl(DWORD, DWORD, PUCHAR, DWORD, PUCHAR, DWORD,
	LPDWORD BytesReturned)
{
	if (BytesReturned)
		*BytesReturned = 0;
	return IFD_NOT_SUPPORTED;
}
