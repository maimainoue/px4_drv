# SPDX-License-Identifier: GPL-2.0-only
#
# Linux のカーネルモジュールのデバイスファイル用の確認ツール (docs/tools.md)
#   make        ptx_tei_monitor、ptx_chrdev_test、px4card_test をビルドする
#   make check  TS の集計の自己試験 (機材は不要) をビルドして実行する
#   make clean  ビルドしたファイルを消す

CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
INCLUDE  = -I ../include -I ../driver/common/include

TOOLS = ptx_tei_monitor ptx_chrdev_test px4card_test

all: $(TOOLS)

ptx_tei_monitor ptx_chrdev_test: ptx_tool.h
ptx_tool_selftest: ptx_tool.h

$(TOOLS) ptx_tool_selftest: %: %.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

check: ptx_tool_selftest
	./ptx_tool_selftest

clean:
	rm -f $(TOOLS) ptx_tool_selftest

.PHONY: all check clean
