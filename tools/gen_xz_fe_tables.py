#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# tools/xz_fe_trace.inc (mAgicVISION ファームウェアが前面へ送った I2C の通信の記録。観察した事実) の、書き込み 0〜75 番を、
# 共通層の表 (driver/common/xz/xz_fe_tables_gen.h) にする。
# 使い方: gen_xz_fe_tables.py tools/xz_fe_trace.inc > driver/common/xz/xz_fe_tables_gen.h
import re, sys

sys.stdout.reconfigure(encoding="utf-8", newline="\n")
text = open(sys.argv[1], encoding="utf-8").read().split("*/", 1)[1]
entries = re.findall(r"\{\s*(0x[0-9a-f]+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*\{([^}]*)\}\s*\}", text)
INIT_END, TUNE_END, FREQ = 58, 76, 66
assert len(entries) >= TUNE_END
rows = []
for i, (addr, wlen, rlen, data) in enumerate(entries[:TUNE_END]):
    assert int(rlen) == 0 and int(wlen) > 0 and addr in ("0x18", "0x19"), i
    b = [x.strip() for x in data.split(",") if x.strip()]
    assert len(b) == int(wlen) <= 40, i
    rows.append((addr, int(wlen), b))
assert rows[FREQ][0] == "0x18" and rows[FREQ][1] == 10 and rows[FREQ][2][0] == "0xfe" and rows[FREQ][2][6] == "0x14" and rows[FREQ][2][8] == "0x15"

print("/* SPDX-License-Identifier: GPL-2.0-only */")
print("/*")
print(" * GV-MVP/XZ2 の前面 (復調器 TC90512 とチューナー) の I2C の書き込みの表。")
print(" * mAgicVISION ファームウェア (mAgicVISION が再配布する firmware.bin) が、地上波 (25ch) の選局で前面へ送った通信を、実機の動作の")
print(" * 観察 (アドレス、バイト列、順序) で記録したもの。機械語は含まない。tools/gen_xz_fe_tables.py で tools/xz_fe_trace.inc から生成する。")
print(" * 初期化 (記録の 0〜57 番) と選局 (58〜75 番) に分ける。選局のうち、66 番の周波数の 2 バイト (チューナーの 0x14 が下位、0x15 が上位) だけが、")
print(" * 13ch、25ch、30ch の記録の比較でチャンネルにより異なった。各項目の意味は特定していない (観察した値のまま)。")
print(" * 項目を減らす最小化は、前面については成立していない (docs/devices/xz.md)。")
print(" */")
print("#ifndef XZ_FE_TABLES_GEN_H")
print("#define XZ_FE_TABLES_GEN_H")
print("")
print("struct xz_fe_write {")
print("\tunsigned char address, length;")
print("\tunsigned char data[40];")
print("};")
print("")
print("#define XZ_FE_INIT_COUNT %d" % INIT_END)
print("#define XZ_FE_TUNE_COUNT %d" % (TUNE_END - INIT_END))
print("#define XZ_FE_FREQ_INDEX %d	/* 選局の表の中の、周波数を書く項目 */" % (FREQ - INIT_END))
print("")
print("static const struct xz_fe_write xz_fe_writes[XZ_FE_INIT_COUNT + XZ_FE_TUNE_COUNT] = {")
for i, (addr, wlen, b) in enumerate(rows):
    if i == 0:
        print("\t/* 初期化 */")
    if i == INIT_END:
        print("\t/* 選局 */")
    print("\t{ %s, %d, { %s } }, /* %d */" % (addr, wlen, ", ".join(b), i))
print("};")
print("")
print("#endif /* XZ_FE_TABLES_GEN_H */")
