#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# tools/fr30/xz_e3.s を組み立てた結果 (xz_e3.bin) を、C の配列 (driver/common/xz/xz_program_gen.h) にする。
# 使い方: gen_xz_program.py xz_e3.bin > xz_program_gen.h
import sys

# Windows でも UTF-8、LF で書く (リダイレクト先のファイルの文字コードが、実行環境の既定にならないように)
sys.stdout.reconfigure(encoding="utf-8", newline="\n")

data = open(sys.argv[1], "rb").read()
assert len(data) % 4 == 0 and len(data) <= 0x180, len(data)  # 0xE1180 からの郵便受けの手前まで
print("/* SPDX-License-Identifier: GPL-2.0-only */")
print("/* tools/fr30/xz_e3.s を fr30-elf-as、fr30-elf-ld -Ttext=0xe1000、fr30-elf-objcopy -O binary で組み立てた機械語。")
print(" * tools/fr30/gen_xz_program.py で生成する。自分で書いたプログラムであり、他の実装のコードや機械語は含まない。 */")
print("#ifndef XZ_PROGRAM_GEN_H")
print("#define XZ_PROGRAM_GEN_H")
print("")
print("#define XZ_PROGRAM_BYTES %d" % len(data))
print("")
print("static const unsigned char xz_program[XZ_PROGRAM_BYTES] = {")
for i in range(0, len(data), 12):
    print("\t" + ", ".join("0x%02x" % b for b in data[i:i + 12]) + ",")
print("};")
print("")
print("#endif /* XZ_PROGRAM_GEN_H */")
