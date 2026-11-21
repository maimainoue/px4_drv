// device_parity_test.cpp - OS 間で利用できる機種に差が無いことを確かめる。
//
// - macOS 版と WinUSB 版の DriverHost_PX4.ini が、同じ機種 (Type、VID、PID、
//   受信機の System) を定義していること
// - すべての Type をホスト層の device factory が扱えること
// - Linux カーネルモジュールの USB ID 表が同じ VID/PID を持つこと
//
// INI は ASCII の key だけを読むため、WinUSB 版の CP932 と macOS 版の UTF-8 の
// どちらもそのまま扱える。

#include <cctype>
#include <cstdio>
#include <tuple>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "device_factory.hpp"

namespace {

/*
 * WinUSB 版の INI は VID/PID を持たず、INF が割り当てる DeviceInterfaceGUID で機種を
 * 識別する。比較は Type、DeviceInterfaceGUID、受信機の System で行い、VID/PID は
 * macOS 版の値を Linux 版との照合に使う。
 */
struct Device {
	std::string type;
	std::string interface_guid;
	unsigned vid = 0, pid = 0;
	unsigned xfer_packets = 0;
	bool has_no_raw_io = false;
	bool no_raw_io = true;
	std::set<std::string> systems;
	bool operator<(const Device &o) const
	{
		return std::tie(type, interface_guid, systems) <
			std::tie(o.type, o.interface_guid, o.systems);
	}
};

std::string Upper(std::string s)
{
	for (auto &c : s)
		c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
	return s;
}

std::string Trim(std::string s)
{
	while (!s.empty() && (s.back() == '\r' || s.back() == ' ' || s.back() == '\t'))
		s.pop_back();
	size_t b = s.find_first_not_of(" \t");
	s = b == std::string::npos ? "" : s.substr(b);
	if (s.size() >= 2 && s.front() == '"' && s.back() == '"')
		s = s.substr(1, s.size() - 2);
	return s;
}

std::vector<Device> LoadIni(const std::string &path)
{
	std::ifstream in(path, std::ios::binary);
	std::map<std::string, Device> by_def;
	std::string line, section;
	while (std::getline(in, line)) {
		line = Trim(line);
		if (line.empty() || line[0] == ';')
			continue;
		if (line[0] == '[') {
			section = line.substr(1, line.find(']') - 1);
			continue;
		}
		const size_t eq = line.find('=');
		if (eq == std::string::npos || section.rfind("DeviceDefinition", 0) != 0)
			continue;
		const std::string key = Trim(line.substr(0, eq)), value = Trim(line.substr(eq + 1));
		const std::string def = section.substr(0, section.find('.'));
		Device &d = by_def[def];
		if (section == def && key == "Type")
			d.type = value;
		else if (section == def && key == "DeviceInterfaceGUID")
			d.interface_guid = Upper(value);
		else if (section == def + ".Config" && key == "VendorId")
			d.vid = std::stoul(value, nullptr, 0);
		else if (section == def + ".Config" && key == "ProductId")
			d.pid = std::stoul(value, nullptr, 0);
		else if (section == def + ".Config" && key == "XferPackets")
			d.xfer_packets = std::stoul(value, nullptr, 0);
		else if (section == def + ".Config" && key == "NoRawIo")
		{
			d.has_no_raw_io = true;
			d.no_raw_io = Upper(value) == "TRUE" || value == "1";
		}
		else if (section.find(".Receiver") != std::string::npos && key == "System") {
			std::stringstream ss(value);
			std::string sys;
			while (std::getline(ss, sys, ','))
				d.systems.insert(Trim(sys));
		}
	}
	std::vector<Device> out;
	for (auto &kv : by_def)
		if (!kv.second.type.empty())
			out.push_back(kv.second);
	return out;
}

std::wstring Widen(const std::string &s)
{
	return std::wstring(s.begin(), s.end());
}

/* driver/linux/px4_usb.c の USB_DEVICE(vid, pid) と、PID 定数の #define を読む。 */
std::set<std::pair<unsigned, unsigned>> LoadLinuxIds(const std::string &root)
{
	std::map<std::string, unsigned> defs;
	std::set<std::pair<unsigned, unsigned>> ids;
	for (const char *f : { "/driver/linux/px4_usb.h", "/driver/linux/px4_usb.c" }) {
		std::ifstream in(root + f);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		std::regex def_re(R"(#define\s+(USB_(?:PID|VID)_\w+)\s+(0x[0-9a-fA-F]+))");
		for (std::sregex_iterator it(text.begin(), text.end(), def_re), end; it != end; ++it)
			defs[(*it)[1]] = std::stoul((*it)[2], nullptr, 16);
		std::regex id_re(R"(USB_DEVICE\(\s*(\w+)\s*,\s*(\w+)\s*\))");
		auto value = [&](const std::string &v) -> unsigned {
			return v.rfind("0x", 0) == 0 ? std::stoul(v, nullptr, 16) : defs[v];
		};
		for (std::sregex_iterator it(text.begin(), text.end(), id_re), end; it != end; ++it)
			ids.insert({ value((*it)[1]), value((*it)[2]) });
	}
	return ids;
}

} // namespace

int main()
{
	const std::string root = PX4_REPO_ROOT;
	const auto mac = LoadIni(root + "/macos/DriverHost_PX4.ini");
	const auto win = LoadIni(root + "/winusb/pkg/DriverHost_PX4/DriverHost_PX4.ini");
	bool ok = true;

	const std::set<Device> mac_set(mac.begin(), mac.end()), win_set(win.begin(), win.end());
	for (const auto &d : mac_set)
		if (!win_set.count(d)) {
			std::fprintf(stderr, "WinUSB INI lacks %s %s\n", d.type.c_str(), d.interface_guid.c_str());
			ok = false;
		}
	for (const auto &d : win_set)
		if (!mac_set.count(d)) {
			std::fprintf(stderr, "macOS INI lacks %s %s\n", d.type.c_str(), d.interface_guid.c_str());
			ok = false;
		}
	/* DTV02A-5TS-P は共通 PXMLT 実装を使うため、macOS の USB ID 定義も必要。 */
	bool found_dtv02a_5ts_p = false;
	for (const auto &d : mac) {
		if (d.vid != 0x0511 || d.pid != 0x924e)
			continue;
		found_dtv02a_5ts_p = true;
		if (d.type != "PXMLT" || d.systems != std::set<std::string>{"ISDB-T", "ISDB-S"}) {
			std::fprintf(stderr, "DTV02A-5TS-P has an invalid macOS device definition\n");
			ok = false;
		}
	}
	if (!found_dtv02a_5ts_p) {
		std::fprintf(stderr, "macOS INI lacks DTV02A-5TS-P 0511:924e\n");
		ok = false;
	}
	for (const auto &d : mac_set)
		if (px4::ParseDeviceType(Widen(d.type)) == px4::DeviceType::UNKNOWN) {
			std::fprintf(stderr, "device factory does not know Type=%s\n", d.type.c_str());
			ok = false;
		}
	for (const auto &[os_name, devices] : {
		std::pair<const char *, const std::vector<Device> *>{"macOS", &mac},
		std::pair<const char *, const std::vector<Device> *>{"WinUSB", &win}}) {
		bool found_torne = false;
		for (const auto &d : *devices) {
			if (d.type != "TORNE")
				continue;
			found_torne = true;
			if (d.systems != std::set<std::string>{"ISDB-T"}) {
				std::fprintf(stderr, "%s TORNE must expose one ISDB-T receiver\n", os_name);
				ok = false;
			}
			if (std::string(os_name) == "macOS" &&
			    (d.vid != 0x054c || d.pid != 0x04b2)) {
				std::fprintf(stderr, "macOS TORNE has %04x:%04x, expected 054c:04b2\n",
					d.vid, d.pid);
				ok = false;
			}
			if (std::string(os_name) == "WinUSB" && d.interface_guid.empty()) {
				std::fprintf(stderr, "WinUSB TORNE lacks DeviceInterfaceGUID\n");
				ok = false;
			}
		}
		if (!found_torne) {
			std::fprintf(stderr, "%s INI lacks TORNE 054c:04b2\n", os_name);
			ok = false;
		}
	}
	if (px4::ParseDeviceType(L"TORNE") == px4::DeviceType::UNKNOWN) {
		std::fprintf(stderr, "device factory does not know Type=TORNE\n");
		ok = false;
	}
	std::string torne_interface_guid;
	for (const auto &d : win)
		if (d.type == "TORNE")
			torne_interface_guid = d.interface_guid;
	std::ifstream torne_inf(root + "/winusb/pkg/inf/Torne.inf", std::ios::binary);
	const std::string torne_inf_text((std::istreambuf_iterator<char>(torne_inf)),
					 std::istreambuf_iterator<char>());
	if (torne_interface_guid.empty() ||
	    torne_inf_text.find("USB\\VID_054C&PID_04B2") == std::string::npos ||
	    torne_inf_text.find(torne_interface_guid) == std::string::npos) {
		std::fprintf(stderr, "Torne.inf does not match the WinUSB device definition\n");
		ok = false;
	}
	/*
	 * GV-MVP/XZ: 地上波 1 受信機。macOS の機種定義が VID/PID を持ち、WinUSB の INF が同じ DeviceInterfaceGUID と
	 * ハードウェア ID を持つこと。
	 */
	for (const auto &[os_name, devices] : {
		std::pair<const char *, const std::vector<Device> *>{"macOS", &mac},
		std::pair<const char *, const std::vector<Device> *>{"WinUSB", &win}}) {
		bool found_xz = false;
		for (const auto &d : *devices) {
			if (d.type != "XZ")
				continue;
			found_xz = true;
			if (d.systems != std::set<std::string>{"ISDB-T"}) {
				std::fprintf(stderr, "%s XZ must expose one ISDB-T receiver\n", os_name);
				ok = false;
			}
			if (d.vid != 0x04bb || d.pid != 0x0530) {
				std::fprintf(stderr, "%s XZ has %04x:%04x, expected 04bb:0530\n", os_name, d.vid, d.pid);
				ok = false;
			}
			if (d.interface_guid.empty()) {
				std::fprintf(stderr, "%s XZ lacks DeviceInterfaceGUID\n", os_name);
				ok = false;
			}
		}
		if (!found_xz) {
			std::fprintf(stderr, "%s INI lacks XZ 04bb:0530\n", os_name);
			ok = false;
		}
	}
	if (px4::ParseDeviceType(L"XZ") == px4::DeviceType::UNKNOWN) {
		std::fprintf(stderr, "device factory does not know Type=XZ\n");
		ok = false;
	}
	std::string xz_interface_guid;
	for (const auto &d : win)
		if (d.type == "XZ")
			xz_interface_guid = d.interface_guid;
	std::ifstream xz_inf(root + "/winusb/pkg/inf/GV-MVP-XZ.inf", std::ios::binary);
	const std::string xz_inf_text((std::istreambuf_iterator<char>(xz_inf)), std::istreambuf_iterator<char>());
	if (xz_interface_guid.empty() || xz_inf_text.find("USB\\VID_04BB&PID_0530") == std::string::npos ||
	    xz_inf_text.find(xz_interface_guid) == std::string::npos) {
		std::fprintf(stderr, "GV-MVP-XZ.inf does not match the WinUSB device definition\n");
		ok = false;
	}
	/*
	 * TurboSight TBS 5530: 地上波 1 受信機。macOS と WinUSB の機種定義が同じ VID/PID を持ち、WinUSB の INF が同じ
	 * DeviceInterfaceGUID とハードウェア ID を持つこと。
	 */
	for (const auto &[os_name, devices] : {
		std::pair<const char *, const std::vector<Device> *>{"macOS", &mac},
		std::pair<const char *, const std::vector<Device> *>{"WinUSB", &win}}) {
		bool found_tbs = false;
		for (const auto &d : *devices) {
			if (d.type != "TBS5530")
				continue;
			found_tbs = true;
			if (d.systems != std::set<std::string>{"ISDB-T"}) {
				std::fprintf(stderr, "%s TBS5530 must expose one ISDB-T receiver\n", os_name);
				ok = false;
			}
			if (d.vid != 0x734c || d.pid != 0x5530) {
				std::fprintf(stderr, "%s TBS5530 has %04x:%04x, expected 734c:5530\n", os_name, d.vid, d.pid);
				ok = false;
			}
			if (d.interface_guid.empty()) {
				std::fprintf(stderr, "%s TBS5530 lacks DeviceInterfaceGUID\n", os_name);
				ok = false;
			}
		}
		if (!found_tbs) {
			std::fprintf(stderr, "%s INI lacks TBS5530 734c:5530\n", os_name);
			ok = false;
		}
	}
	if (px4::ParseDeviceType(L"TBS5530") == px4::DeviceType::UNKNOWN) {
		std::fprintf(stderr, "device factory does not know Type=TBS5530\n");
		ok = false;
	}
	std::string tbs_interface_guid;
	for (const auto &d : win)
		if (d.type == "TBS5530")
			tbs_interface_guid = d.interface_guid;
	std::ifstream tbs_inf(root + "/winusb/pkg/inf/TBS5530.inf", std::ios::binary);
	const std::string tbs_inf_text((std::istreambuf_iterator<char>(tbs_inf)), std::istreambuf_iterator<char>());
	if (tbs_interface_guid.empty() || tbs_inf_text.find("USB\\VID_734C&PID_5530") == std::string::npos ||
	    tbs_inf_text.find(tbs_interface_guid) == std::string::npos) {
		std::fprintf(stderr, "TBS5530.inf does not match the WinUSB device definition\n");
		ok = false;
	}
	/*
	 * PIX-DT295: Rev.01 (06b8:103b) と Rev.02 (06b8:105c)。それぞれ別の DeviceInterfaceGUID を持ち、
	 * WinUSB の INF が同じ GUID とハードウェア ID を持つこと。
	 */
	{
		const std::set<unsigned> dt295_pids = { 0x103b, 0x105c };
		std::string inf_path = root + "/winusb/pkg/inf/PIX-DT295.inf";
		std::ifstream dt295_inf(inf_path, std::ios::binary);
		const std::string dt295_inf_text((std::istreambuf_iterator<char>(dt295_inf)), std::istreambuf_iterator<char>());
		for (const auto &[os_name, devices] : {
			std::pair<const char *, const std::vector<Device> *>{"macOS", &mac},
			std::pair<const char *, const std::vector<Device> *>{"WinUSB", &win}}) {
			std::set<unsigned> found;
			for (const auto &d : *devices) {
				if (d.type != "DT295")
					continue;
				found.insert(d.pid);
				if (d.systems != std::set<std::string>{"ISDB-T"}) {
					std::fprintf(stderr, "%s DT295 must expose one ISDB-T receiver\n", os_name);
					ok = false;
				}
				if (d.vid != 0x06b8) {
					std::fprintf(stderr, "%s DT295 has vendor %04x, expected 06b8\n", os_name, d.vid);
					ok = false;
				}
				if (d.interface_guid.empty()) {
					std::fprintf(stderr, "%s DT295 lacks DeviceInterfaceGUID\n", os_name);
					ok = false;
				}
				if (std::string(os_name) == "WinUSB" && dt295_inf_text.find(d.interface_guid) == std::string::npos) {
					std::fprintf(stderr, "PIX-DT295.inf lacks the DeviceInterfaceGUID %s\n", d.interface_guid.c_str());
					ok = false;
				}
			}
			if (found != dt295_pids) {
				std::fprintf(stderr, "%s INI must define DT295 Rev.01 and Rev.02\n", os_name);
				ok = false;
			}
		}
		if (dt295_inf_text.find("USB\\VID_06B8&PID_103B") == std::string::npos ||
		    dt295_inf_text.find("USB\\VID_06B8&PID_105C") == std::string::npos) {
			std::fprintf(stderr, "PIX-DT295.inf lacks a hardware ID\n");
			ok = false;
		}
		if (px4::ParseDeviceType(L"DT295") == px4::DeviceType::UNKNOWN) {
			std::fprintf(stderr, "device factory does not know Type=DT295\n");
			ok = false;
		}
	}
	for (const auto &[os_name, devices] : {
		std::pair<const char *, const std::vector<Device> *>{"macOS", &mac},
		std::pair<const char *, const std::vector<Device> *>{"WinUSB", &win}}) {
		bool found_t230c = false;
		for (const auto &d : *devices) {
			if (d.type != "T230C") {
				if (d.has_no_raw_io) {
					std::fprintf(stderr, "%s %s has T230C-only NoRawIo config\n",
						os_name, d.type.c_str());
					ok = false;
				}
				continue;
			}
			found_t230c = true;
			if (d.xfer_packets != 22) {
				std::fprintf(stderr, "%s T230C XferPackets=%u, expected 22\n",
					os_name, d.xfer_packets);
				ok = false;
			}
			if (!d.has_no_raw_io || d.no_raw_io) {
				std::fprintf(stderr, "%s T230C NoRawIo=%s, expected false\n",
					os_name, d.has_no_raw_io ? (d.no_raw_io ? "true" : "false") : "missing");
				ok = false;
			}
		}
		if (!found_t230c) {
			std::fprintf(stderr, "%s INI lacks T230C transfer configuration\n", os_name);
			ok = false;
		}
	}

	/*
	 * Linux 版の未対応機種。ここに無い差は失敗にする。
	 */
	const std::set<std::pair<unsigned, unsigned>> linux_pending = {
	};
	const auto linux_ids = LoadLinuxIds(root);
	if (linux_ids.empty()) {
		std::fprintf(stderr, "Linux USB ID table could not be read\n");
		ok = false;
	}
	if (!linux_ids.count({0x054c, 0x04b2})) {
		std::fprintf(stderr, "Linux lacks TORNE 054c:04b2\n");
		ok = false;
	}
	if (!linux_ids.count({0x04bb, 0x0530})) {
		std::fprintf(stderr, "Linux lacks XZ 04bb:0530\n");
		ok = false;
	}
	for (const auto &d : mac_set) {
		const std::pair<unsigned, unsigned> id(d.vid, d.pid);
		const bool in_linux = linux_ids.count(id) != 0;
		if (!in_linux && !linux_pending.count(id)) {
			std::fprintf(stderr, "Linux lacks %s %04x:%04x\n", d.type.c_str(), d.vid, d.pid);
			ok = false;
		} else if (!in_linux) {
			std::printf("pending on Linux: %s %04x:%04x\n", d.type.c_str(), d.vid, d.pid);
		} else if (linux_pending.count(id)) {
			std::fprintf(stderr, "remove %04x:%04x from linux_pending\n", d.vid, d.pid);
			ok = false;
		}
	}
	std::printf("devices: %zu, Linux USB IDs: %zu\n", mac_set.size(), linux_ids.size());
	return ok ? 0 : 1;
}
