# SPDX-License-Identifier: GPL-2.0-only
"""Info.plist の登録を検査する。

pcscd は USB の VID と PID が Info.plist に載っている機器にだけ IFD ハンドラーを割り当てる。
載せ忘れた機種は、カーネルモジュールがカードリーダーを登録していても、pcscd からは見えない。
"""
import plistlib
import re
from pathlib import Path

root = Path(__file__).resolve().parents[2]
text = (root / 'pcsc/Info.plist').read_text(encoding='utf-8')

# pcsc-lite は string 要素を行単位で読むため、1 行に 2 つ以上並べると 2 つ目以降が登録されない
for number, line in enumerate(text.splitlines(), 1):
    assert line.count('<string>') <= 1, f'Info.plist:{number}: more than one <string> on a line'

info = plistlib.loads(text.encode('utf-8'))
vendors, products, names = info['ifdVendorID'], info['ifdProductID'], info['ifdFriendlyName']
assert len(vendors) == len(products) == len(names), 'ifdVendorID / ifdProductID / ifdFriendlyName differ in length'
registered = [(v.lower(), p.lower()) for v, p in zip(vendors, products)]
assert len(set(registered)) == len(registered), 'duplicate VID/PID in Info.plist'

# 内蔵カードリーダーを持ち、px4card を登録する機種 (VID 0511 の機種は check_card_guids.py が GUID の表で照合する)
header = (root / 'driver/linux/px4_usb.h').read_text(encoding='utf-8')
define = {name: value.lower() for name, value in re.findall(r'#define\s+(\w+)\s+(0x[0-9a-fA-F]+)', header)}
required = {
    'Sony CECH-ZD1 J': ('USB_VID_SONY', 'USB_PID_TORNE'),
    'PIX-DT295 Rev.01': ('USB_VID_PIXELA', 'USB_PID_PIX_DT295_REV01'),
    'PIX-DT295 Rev.02': ('USB_VID_PIXELA', 'USB_PID_PIX_DT295_REV02'),
}
for model, (vendor_name, product_name) in required.items():
    pair = (define[vendor_name], define[product_name])
    assert pair in registered, f'{model} {pair[0]}:{pair[1]} is not in Info.plist'

# macOS 用の Info-macos.plist も、同じ機種を登録していること (DriverHost_PX4 の内蔵カードリーダーを持つ機種)
mac_text = (root / 'pcsc/Info-macos.plist').read_text(encoding='utf-8')
for number, line in enumerate(mac_text.splitlines(), 1):
    assert line.count('<string>') <= 1, f'Info-macos.plist:{number}: more than one <string> on a line'
mac = plistlib.loads(mac_text.encode('utf-8'))
assert len(mac['ifdVendorID']) == len(mac['ifdProductID']) == len(mac['ifdFriendlyName']), 'Info-macos.plist arrays differ in length'
mac_registered = [(v.lower(), p.lower()) for v, p in zip(mac['ifdVendorID'], mac['ifdProductID'])]
assert len(set(mac_registered)) == len(mac_registered), 'duplicate VID/PID in Info-macos.plist'
for model, (vendor_name, product_name) in required.items():
    pair = (define[vendor_name], define[product_name])
    assert pair in mac_registered, f'{model} {pair[0]}:{pair[1]} is not in Info-macos.plist'

print(f'check_info_plist: {len(registered)} entries, {len(required)} card-capable models registered (macOS: {len(mac_registered)} entries)')
