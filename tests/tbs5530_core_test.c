/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TBS 5530 の共通層 (driver/common/tbs5530) の自動試験。
 *
 * 機材を模擬する: ブリッジの制御転送 (ファームウェア 0xa0、I2C 0x80/0x90/0x91、TS の選択 0x8a) と、
 * I2C の先のレジスタ (復調器の SLVT/SLVX、内蔵チューナー、EEPROM)。
 * 確認するのは、制御転送の符号化、ファームウェア送信の区切りと失敗時の中断、チップ ID の判定、
 * 電源投入と選局のレジスタ値と順序、ロック判定、C/N の計算、失敗の注入 (転送の失敗、短い転送、
 * チューナーの CPU が起動しない) と、失敗後に最初の誤りを引きずらないこと。
 * 実機の代用ではない (値の出どころは TurboSight の linux_media の cxd2878.c と tbs5530.c)。
 *
 * build: gcc -Wall -Wextra -I driver/common/tbs5530 tests/tbs5530_core_test.c \
 *            driver/common/tbs5530/tbs5530_core.c -o tbs5530_core_test
 */
#include "tbs5530_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SLVT 0x64
#define SLVX 0x66
#define TUNER 0x60
#define EEPROM 0x50

static int checks, fails;

#define CHECK(c)                                                                  \
	do {                                                                      \
		checks++;                                                         \
		if (!(c)) {                                                       \
			fails++;                                                  \
			printf("FAIL %s:%d: %s\n", __func__, __LINE__, #c);       \
		}                                                                 \
	} while (0)

/* ---- 模擬機材 ---- */

enum { EV_FW, EV_I2C_W, EV_I2C_PTR, EV_I2C_R, EV_STREAM, EV_SLEEP };

struct ev {
	int kind;
	u8 addr;	/* 7 ビットの I2C アドレス */
	u8 reg;
	u8 len;		/* 書き込んだバイト数 (レジスタを除く) / 読んだバイト数 */
	u8 data[48];
	u16 value;	/* ファームウェアのアドレス */
	unsigned int ms;
};

#define EV_MAX 20000

struct mock {
	struct ev ev[EV_MAX];
	int n;
	u8 regs[128][256][256];	/* [I2C アドレス][バンク][レジスタ]。バンクはレジスタ 0 への書き込みで切り替わる */
	u8 bank[128];
	u8 ptr_len, ptr_addr, ptr_reg;
	int calls;		/* 制御転送の通し番号 (1 から) */
	int fail_at;		/* この番号の制御転送を -EIO にする (0 で無効) */
	int short_at;		/* この番号の制御転送を 1 バイト短くする (0 で無効) */
	int cpu_status;		/* チューナーのレジスタ 0x1a (CPU_STT) */
	unsigned int slept_ms;
} M;

static void reset_mock(void)
{
	memset(&M, 0, sizeof(M));
	/* チップ ID: ((0xfb & 3) << 8) | 0xfd = 0x19c (CXD6822) */
	M.regs[SLVX][0][0xfb] = 0x01;
	M.regs[SLVX][0][0xfd] = 0x9c;
	M.regs[TUNER][0][0x7f] = 0xf6;
	/* EEPROM の 16〜21 バイト目: MAC アドレス */
	{
		static const u8 mac[6] = { 0x00, 0x22, 0xab, 0xc0, 0x7d, 0x24 };

		memcpy(&M.regs[EEPROM][0][16], mac, 6);
	}
}

static struct ev *add_ev(int kind)
{
	struct ev *e;

	if (M.n >= EV_MAX) {
		fprintf(stderr, "event log full\n");
		exit(2);
	}
	e = &M.ev[M.n++];
	memset(e, 0, sizeof(*e));
	e->kind = kind;
	return e;
}

static int mock_control(void *priv, u8 type, u8 request, u16 value, u16 index, void *buf, u16 len)
{
	u8 *b = buf;
	struct ev *e;
	int ret = len;

	(void)priv;
	(void)index;
	M.calls++;
	if (M.fail_at == M.calls)
		return -EIO;
	switch (request) {
	case 0xa0:
		e = add_ev(EV_FW);
		e->value = value;
		e->len = (u8)len;
		memcpy(e->data, b, len > sizeof(e->data) ? sizeof(e->data) : len);
		break;
	case 0x80: {
		/* [長さ + 1, アドレス << 1, レジスタ, データ...] */
		u8 reg, n;

		if (type != 0x40 || len < 3 || b[0] != len - 1) {
			fprintf(stderr, "bad I2C write framing\n");
			return -EINVAL;
		}
		reg = b[2];
		n = (u8)(len - 3);
		e = add_ev(EV_I2C_W);
		e->addr = b[1] >> 1;
		e->reg = reg;
		e->len = n;
		memcpy(e->data, b + 3, n);
		if (reg == 0x00 && n >= 1)
			M.bank[e->addr] = b[3];
		else
			memcpy(&M.regs[e->addr][M.bank[e->addr]][reg], b + 3, n);
		/* チューナーの CPU 状態は、ファームウェアの動作で決まる固定値 */
		M.regs[TUNER][0][0x1a] = (u8)M.cpu_status;
		break;
	}
	case 0x90:
		if (type != 0x40 || len != 3)
			return -EINVAL;
		M.ptr_len = b[0];
		M.ptr_addr = b[1] >> 1;
		M.ptr_reg = b[2];
		e = add_ev(EV_I2C_PTR);
		e->addr = M.ptr_addr;
		e->reg = M.ptr_reg;
		e->len = M.ptr_len;
		break;
	case 0x91:
		if (type != 0xc0 || len != M.ptr_len)
			return -EINVAL;
		memcpy(b, &M.regs[M.ptr_addr][M.bank[M.ptr_addr]][M.ptr_reg], len);
		/* SLVT のステータスなどは、試験が直接レジスタへ置く */
		e = add_ev(EV_I2C_R);
		e->addr = M.ptr_addr;
		e->reg = M.ptr_reg;
		e->len = (u8)len;
		break;
	case 0x8a:
		e = add_ev(EV_STREAM);
		e->len = (u8)len;
		memcpy(e->data, b, len);
		break;
	default:
		return -EINVAL;
	}
	if (M.short_at == M.calls)
		ret = len - 1;
	return ret;
}

static void mock_msleep(void *priv, unsigned int ms)
{
	struct ev *e;

	(void)priv;
	M.slept_ms += ms;
	e = add_ev(EV_SLEEP);
	e->ms = ms;
}

static void mock_log(void *priv, int level, const char *fmt, ...)
{
	(void)priv;
	(void)level;
	(void)fmt;
}

static const struct tbs5530_ops OPS = { mock_control, mock_msleep, mock_log };

static struct tbs5530_bridge B;
static struct tbs5530_demod D;

static void setup(void)
{
	reset_mock();
	tbs5530_bridge_init(&B, &OPS, NULL);
}

/* 書き込みの記録から、(アドレス, レジスタ) の最後の書き込みの先頭バイトを探す。無ければ -1 */
static int last_write(u8 addr, u8 reg)
{
	int i, r = -1;

	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == addr && M.ev[i].reg == reg)
			r = M.ev[i].data[0];
	return r;
}

/* 最初の書き込みの位置。無ければ -1 */
static int first_write_index(u8 addr, u8 reg)
{
	int i;

	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == addr && M.ev[i].reg == reg)
			return i;
	return -1;
}

static int count_writes(u8 addr, u8 reg)
{
	int i, c = 0;

	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == addr && M.ev[i].reg == reg)
			c++;
	return c;
}

/* ---- 試験 ---- */

static void test_i2c(void)
{
	u8 d[4] = { 0x12, 0xaa, 0xbb, 0xcc };
	u8 out[3];
	u8 big[49] = { 0 };

	setup();
	CHECK(tbs5530_i2c_write(&B, 0x64, d, 4) == 0);
	CHECK(M.n == 1 && M.ev[0].kind == EV_I2C_W && M.ev[0].addr == 0x64 && M.ev[0].reg == 0x12 && M.ev[0].len == 3);
	CHECK(M.ev[0].data[0] == 0xaa && M.ev[0].data[2] == 0xcc);

	M.regs[0x60][0][0x30] = 0x11;
	M.regs[0x60][0][0x31] = 0x22;
	M.regs[0x60][0][0x32] = 0x33;
	CHECK(tbs5530_i2c_read(&B, 0x60, 0x30, out, 3) == 0);
	CHECK(out[0] == 0x11 && out[1] == 0x22 && out[2] == 0x33);

	/* 長さの上限 (48) と 0 */
	setup();
	CHECK(tbs5530_i2c_write(&B, 0x64, big, 49) == -EINVAL);
	CHECK(tbs5530_i2c_write(&B, 0x64, big, 0) == -EINVAL);
	CHECK(tbs5530_i2c_write(&B, 0x64, big, 48) == 0);
	CHECK(tbs5530_i2c_read(&B, 0x64, 0, out, 0) == -EINVAL);
	CHECK(tbs5530_i2c_read(&B, 0x64, 0, big, 49) == -EINVAL);

	/* 転送の失敗と短い転送 */
	setup();
	M.fail_at = 1;
	CHECK(tbs5530_i2c_write(&B, 0x64, d, 4) == -EIO);
	setup();
	M.short_at = 1;
	CHECK(tbs5530_i2c_write(&B, 0x64, d, 4) == -EIO);
	setup();
	M.fail_at = 2; /* ポインタの書き込みは通り、読み出しが失敗 */
	CHECK(tbs5530_i2c_read(&B, 0x60, 0x30, out, 3) == -EIO);
	setup();
	M.short_at = 2;
	CHECK(tbs5530_i2c_read(&B, 0x60, 0x30, out, 3) == -EIO);
	setup();
	M.fail_at = 1; /* ポインタの書き込みが失敗 */
	CHECK(tbs5530_i2c_read(&B, 0x60, 0x30, out, 3) == -EIO);
}

static void test_bridge(void)
{
	u8 mac[6];
	u8 image[200];
	int i, chunks;

	/* TS の選択 */
	setup();
	CHECK(tbs5530_select_stream(&B, false) == 0);
	CHECK(M.n == 1 && M.ev[0].kind == EV_STREAM && M.ev[0].len == 2 && M.ev[0].data[0] == 10 && M.ev[0].data[1] == 0);
	CHECK(tbs5530_select_stream(&B, true) == 0);
	CHECK(M.ev[1].data[0] == 10 && M.ev[1].data[1] == 1);
	M.fail_at = M.calls + 1;
	CHECK(tbs5530_select_stream(&B, false) == -EIO);

	/* MAC アドレス (EEPROM の 16 バイト目から) */
	setup();
	CHECK(tbs5530_read_mac(&B, mac) == 0);
	CHECK(mac[0] == 0x00 && mac[1] == 0x22 && mac[2] == 0xab && mac[5] == 0x24);
	CHECK(M.ev[0].addr == EEPROM && M.ev[0].reg == 16);
	setup();
	M.fail_at = 3;
	CHECK(tbs5530_read_mac(&B, mac) == -EIO);

	/* ファームウェア: 200 バイトは 64 + 64 + 64 + 8 */
	for (i = 0; i < (int)sizeof(image); i++)
		image[i] = (u8)i;
	setup();
	CHECK(tbs5530_fw_load(&B, image, sizeof(image)) == 0);
	/* CPU 停止 (0x7f92、0xe600 へ 1)、4 区画、CPU 再開 (0x7f92、0xe600 へ 0)、その後に 100 ms の待ち */
	CHECK(M.n == 2 + 4 + 2 + 1);
	CHECK(M.ev[0].kind == EV_FW && M.ev[0].value == 0x7f92 && M.ev[0].data[0] == 1);
	CHECK(M.ev[1].value == 0xe600 && M.ev[1].data[0] == 1);
	chunks = 0;
	for (i = 2; i < 6; i++) {
		CHECK(M.ev[i].kind == EV_FW && M.ev[i].value == (u16)((i - 2) * 0x40));
		CHECK(M.ev[i].len == (i == 5 ? 8 : 0x40));
		CHECK(M.ev[i].data[0] == image[(i - 2) * 0x40]);
		chunks++;
	}
	CHECK(chunks == 4);
	CHECK(M.ev[6].value == 0x7f92 && M.ev[6].data[0] == 0);
	CHECK(M.ev[7].value == 0xe600 && M.ev[7].data[0] == 0);
	CHECK(M.ev[8].kind == EV_SLEEP && M.ev[8].ms == 100);

	/* 8192 バイトは 128 区画 */
	{
		static u8 big[8192];

		setup();
		CHECK(tbs5530_fw_load(&B, big, sizeof(big)) == 0);
		CHECK(M.n == 2 + 128 + 2 + 1);
	}

	/* 引数の誤り */
	setup();
	CHECK(tbs5530_fw_load(&B, NULL, 100) == -EINVAL);
	CHECK(tbs5530_fw_load(&B, image, 0) == -EINVAL);
	CHECK(tbs5530_fw_load(&B, image, 0x10001) == -EINVAL);
	CHECK(M.n == 0);

	/* 途中の失敗: CPU を再開せず、待たない */
	setup();
	M.fail_at = 5; /* CPU 停止の 2 回の後、3 区画目 */
	CHECK(tbs5530_fw_load(&B, image, sizeof(image)) == -EIO);
	CHECK(M.n == 2 + 2); /* 停止 2 回と、成功した区画 2 つ */
	CHECK(M.slept_ms == 0);
	/* CPU を止められない */
	setup();
	M.fail_at = 1;
	CHECK(tbs5530_fw_load(&B, image, sizeof(image)) == -EIO && M.n == 0);
	/* CPU の再開に失敗 */
	setup();
	M.fail_at = 2 + 4 + 1;
	CHECK(tbs5530_fw_load(&B, image, sizeof(image)) == -EIO);
	CHECK(M.slept_ms == 0);
	/* 短い書き込みは失敗として扱う */
	setup();
	M.short_at = 3;
	CHECK(tbs5530_fw_load(&B, image, sizeof(image)) == -EIO);
}

static void test_math(void)
{
	CHECK(tbs5530_log10_x100(1) == 0);
	CHECK(tbs5530_log10_x100(10) == 100);
	CHECK(tbs5530_log10_x100(100) == 200);
	/* 単調 */
	{
		u32 x, prev = 0;
		int ok = 1;

		for (x = 1; x < 70000; x += 7) {
			u32 v = tbs5530_log10_x100(x);

			if (v < prev)
				ok = 0;
			prev = v;
		}
		CHECK(ok);
	}
	/* 実数の log10 (x) * 100 との差が 3 以内 (表の精度) */
	{
		static const struct {
			u32 x;
			u32 expect;
		} t[] = { { 2, 30 }, { 5, 70 }, { 50, 170 }, { 1000, 300 }, { 4096, 361 }, { 65535, 482 } };
		int i;

		for (i = 0; i < (int)(sizeof(t) / sizeof(t[0])); i++) {
			u32 v = tbs5530_log10_x100(t[i].x);

			CHECK(v + 3 >= t[i].expect && v <= t[i].expect + 3);
		}
	}
}

static void test_attach(void)
{
	/* CXD6822 */
	setup();
	CHECK(tbs5530_demod_attach(&D, &B) == 0 && D.chip_id == 0x19c && D.state == TBS5530_DEMOD_UNKNOWN);
	/* SLVX のバンク 0 を選んでから、0xfb と 0xfd を読む */
	CHECK(M.ev[0].kind == EV_I2C_W && M.ev[0].addr == SLVX && M.ev[0].reg == 0x00 && M.ev[0].data[0] == 0x00);
	/* CXD2878A */
	setup();
	M.regs[SLVX][0][0xfb] = 0x03;
	M.regs[SLVX][0][0xfd] = 0x9a;
	CHECK(tbs5530_demod_attach(&D, &B) == 0 && D.chip_id == 0x39a);
	/* 上位 6 ビットは無視する */
	setup();
	M.regs[SLVX][0][0xfb] = 0xfd;
	CHECK(tbs5530_demod_attach(&D, &B) == 0 && D.chip_id == 0x19c);
	/* ASCOT3 型 (CXD2878、0x396) と、別のチップは対象外 */
	setup();
	M.regs[SLVX][0][0xfb] = 0x03;
	M.regs[SLVX][0][0xfd] = 0x96;
	CHECK(tbs5530_demod_attach(&D, &B) == -ENODEV && D.chip_id == 0x396);
	setup();
	M.regs[SLVX][0][0xfb] = 0x00;
	M.regs[SLVX][0][0xfd] = 0x00;
	CHECK(tbs5530_demod_attach(&D, &B) == -ENODEV);
	/* I2C の失敗 */
	setup();
	M.fail_at = 2;
	CHECK(tbs5530_demod_attach(&D, &B) == -EIO);
	/* 失敗の後は、最初の誤りを引きずらない */
	M.fail_at = 0;
	CHECK(tbs5530_demod_attach(&D, &B) == 0 && B.err == 0);
}

static void attach_ok(void)
{
	setup();
	if (tbs5530_demod_attach(&D, &B)) {
		printf("attach failed in setup\n");
		exit(2);
	}
}

static void test_init(void)
{
	int i, clk, clr;

	attach_ok();
	CHECK(tbs5530_demod_init(&D) == 0);
	CHECK(D.warm && D.state == TBS5530_DEMOD_SLEEP && D.tuner_id == 0xf6 && !D.isdbt && !D.isdbc && !D.led_locked);
	/* レジスタの初期化が最初、次に 24 MHz の設定 */
	clr = first_write_index(SLVX, 0x02);
	clk = first_write_index(SLVX, 0x14);
	CHECK(clr >= 0 && clk > clr);
	CHECK(M.ev[clr].data[0] == 0x00 && M.ev[clk].data[0] == 0x01);
	/* CXD6822 の設定 */
	CHECK(last_write(SLVT, 0x23) == 0x03);
	/* チューナーの I2C リピーターは、有効にしてからチューナーに触り、最後に無効にする */
	{
		int rep_on = -1, rep_off = -1, first_tuner = -1, last_tuner = -1;

		for (i = 0; i < M.n; i++) {
			if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVX && M.ev[i].reg == 0x08) {
				if (M.ev[i].data[0] == 1)
					rep_on = i;
				else
					rep_off = i;
			}
			if ((M.ev[i].kind == EV_I2C_W || M.ev[i].kind == EV_I2C_PTR) && M.ev[i].addr == TUNER) {
				if (first_tuner < 0)
					first_tuner = i;
				last_tuner = i;
			}
		}
		CHECK(rep_on >= 0 && rep_off > rep_on && first_tuner > rep_on && last_tuner < rep_off);
	}
	/* TS は並列、クロックマスク 1。ロック LED は消灯 */
	CHECK(last_write(SLVT, 0xc6) == 0x01);
	CHECK(last_write(SLVX, 0xa2) == 0x00);
	/* チューナーの最後の設定 */
	CHECK(last_write(TUNER, 0x1e) == 0xa0);
	/* 電源投入は 1 回だけ。選局の中で、もう一度は走らない */
	CHECK(tbs5530_demod_tune_isdbt(&D, 557143) == 0);
	CHECK(count_writes(SLVX, 0x02) == 1);

	/* チューナーの CPU が起動しない: 失敗し、warm にならない */
	attach_ok();
	M.cpu_status = 0x01;
	CHECK(tbs5530_demod_init(&D) == -EIO && !D.warm && D.state == TBS5530_DEMOD_UNKNOWN && B.err == 0);
	/* 転送の失敗 (どの位置でも、失敗が呼び出し元へ返り、状態が進まない) */
	for (i = 1; i <= 60; i += 7) {
		attach_ok();
		M.fail_at = M.calls + i;
		CHECK(tbs5530_demod_init(&D) == -EIO && !D.warm && B.err == 0);
	}
	/* 失敗の後に、もう一度試せる */
	M.fail_at = 0;
	CHECK(tbs5530_demod_init(&D) == 0 && D.warm);
}

static void test_tune(void)
{
	u32 khz = 557143; /* UHF 27ch */
	int i, blk = -1;
	u8 *t;

	attach_ok();
	CHECK(tbs5530_demod_init(&D) == 0);
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbt(&D, khz) == 0);
	CHECK(D.state == TBS5530_DEMOD_ACTIVE && D.isdbt);
	/* ISDB-T の中間周波数 3.55 MHz (0x12eeef) と 6 MHz の帯域設定 */
	CHECK(last_write(SLVT, 0xb6) == 0x12);
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xb6) {
			CHECK(M.ev[i].len == 3 && M.ev[i].data[1] == 0xee && M.ev[i].data[2] == 0xef);
		}
	CHECK(last_write(SLVT, 0xd7) == 0x00);
	/* チューナーの 0x68 からの 17 バイト: 周波数 (kHz) は 0x70 から 3 バイト */
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == TUNER && M.ev[i].reg == 0x68)
			blk = i;
	CHECK(blk >= 0 && M.ev[blk].len == 17);
	t = M.ev[blk].data;
	CHECK(t[8] == (khz & 0xff) && t[9] == ((khz >> 8) & 0xff) && t[10] == ((khz >> 16) & 0x1f));
	CHECK(t[1] == 0x86 && t[4] == 0x33 && t[5] == 0x00 && t[6] == 0x17 && t[7] == 0x1b);
	/* TS の出力は、選局の最後に有効にする (0xc3 = 0)。選局の最初は止まっている */
	{
		int c3 = -1;

		for (i = 0; i < M.n; i++)
			if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xc3)
				c3 = i;
		CHECK(c3 > blk && M.ev[c3].data[0] == 0x00);
	}
	/* 選局の後に、リピーターは無効になっている */
	CHECK(last_write(SLVX, 0x08) == 0x00);

	/* 起きている状態の再選局: 起動の手順は繰り返さず、TS を止めて、帯域設定をやり直す */
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbt(&D, 563143) == 0);
	CHECK(count_writes(SLVT, 0x2c) == 0 && count_writes(SLVT, 0x4b) == 0);
	CHECK(count_writes(SLVT, 0xb6) == 1);
	{
		int off = -1, on = -1;

		for (i = 0; i < M.n; i++)
			if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xc3) {
				if (M.ev[i].data[0] == 1 && off < 0)
					off = i;
				if (M.ev[i].data[0] == 0)
					on = i;
			}
		CHECK(off >= 0 && on > off);
	}
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == TUNER && M.ev[i].reg == 0x68)
			blk = i;
	CHECK(M.ev[blk].data[8] == (563143 & 0xff));

	/* 周波数の範囲 */
	attach_ok();
	CHECK(tbs5530_demod_tune_isdbt(&D, 41999) == -EINVAL);
	CHECK(tbs5530_demod_tune_isdbt(&D, 1002001) == -EINVAL);
	CHECK(tbs5530_demod_tune_isdbt(&D, 42000) == 0);
	CHECK(tbs5530_demod_tune_isdbt(&D, 1002000) == 0);

	/* 転送の失敗: 失敗が返り、状態は起きた扱いにならず、次の選局は通る */
	/* 選局は約 80 回の転送。どの位置の失敗でも同じ振る舞いになる */
	for (i = 1; i <= 75; i += 7) {
		attach_ok();
		CHECK(tbs5530_demod_init(&D) == 0);
		M.fail_at = M.calls + i;
		CHECK(tbs5530_demod_tune_isdbt(&D, khz) == -EIO);
		CHECK(B.err == 0);
		M.fail_at = 0;
		CHECK(tbs5530_demod_tune_isdbt(&D, khz) == 0);
		CHECK(D.state == TBS5530_DEMOD_ACTIVE);
	}
}

static void test_status(void)
{
	u8 st = 0xff;
	s32 cnr = 0;
	int led;

	attach_ok();
	/* 初期化の前と、起きていない間は読めない */
	CHECK(tbs5530_demod_read_status(&D, &st) == -ENODEV);
	CHECK(tbs5530_demod_read_cnr(&D, &cnr) == -ENODEV);
	CHECK(tbs5530_demod_init(&D) == 0);
	CHECK(tbs5530_demod_read_status(&D, &st) == -ENODEV);
	CHECK(tbs5530_demod_tune_isdbt(&D, 557143) == 0);

	/* SLVT のレジスタ 0x10: bit4 アンロック検出、bit1 同期、bit0 TS ロック */
	M.regs[SLVT][0x60][0x10] = 0x03;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0);
	CHECK(st == (TBS5530_STATUS_SIGNAL | TBS5530_STATUS_LOCK) && D.led_locked);
	M.regs[SLVT][0x60][0x10] = 0x13; /* アンロック検出が立っていればロックではない */
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == TBS5530_STATUS_SIGNAL && !D.led_locked);
	M.regs[SLVT][0x60][0x10] = 0x02; /* 同期だけ */
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == TBS5530_STATUS_SIGNAL);
	M.regs[SLVT][0x60][0x10] = 0x01; /* TS ロックだけ */
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == TBS5530_STATUS_SIGNAL);
	M.regs[SLVT][0x60][0x10] = 0x00;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == 0);

	/* LED は、状態が変わったときだけ書く */
	M.regs[SLVT][0x60][0x10] = 0x03;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0);
	M.n = 0;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0);
	led = count_writes(SLVX, 0xa2);
	CHECK(led == 0); /* 前回もロック: 書かない */
	M.regs[SLVT][0x60][0x10] = 0x00;
	M.n = 0;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0);
	CHECK(count_writes(SLVX, 0xa2) == 1 && last_write(SLVX, 0xa2) == 0x00);

	/* 失敗 */
	M.fail_at = M.calls + 1;
	CHECK(tbs5530_demod_read_status(&D, &st) == -EIO && B.err == 0);

	/* C/N: 100 * log10 (レジスタ) - 9031 - 1500 (0.001 dB) */
	M.regs[SLVT][0x60][0x28] = 0x08;
	M.regs[SLVT][0x60][0x29] = 0x00; /* 0x0800 = 2048 */
	CHECK(tbs5530_demod_read_cnr(&D, &cnr) == 0);
	CHECK(cnr == 100 * (s32)tbs5530_log10_x100(2048) - 9031 - 1500);
	/* 2048 は約 33.1 dB から 9.03 dB と 1.5 dB を引いた値 (約 22.6 dB) */
	CHECK(cnr > 22000 && cnr < 23200);
	M.regs[SLVT][0x60][0x28] = 0x00;
	M.regs[SLVT][0x60][0x29] = 0x00;
	CHECK(tbs5530_demod_read_cnr(&D, &cnr) == -EAGAIN);
	M.fail_at = M.calls + 2;
	CHECK(tbs5530_demod_read_cnr(&D, &cnr) == -EIO && B.err == 0);
}

static void test_sleep(void)
{
	int i, off = -1;

	/* 初期化の前は、何もできない */
	attach_ok();
	CHECK(tbs5530_demod_sleep(&D) == -ENODEV);
	CHECK(tbs5530_demod_init(&D) == 0);
	/* 起きていない状態の sleep: TS の出力の停止と、設定の取り消しは行わず、基本の設定だけ */
	M.n = 0;
	CHECK(tbs5530_demod_sleep(&D) == 0 && D.state == TBS5530_DEMOD_SLEEP);
	CHECK(count_writes(SLVT, 0xc3) == 0);

	CHECK(tbs5530_demod_tune_isdbt(&D, 557143) == 0);
	M.n = 0;
	CHECK(tbs5530_demod_sleep(&D) == 0);
	CHECK(D.state == TBS5530_DEMOD_SLEEP && !D.isdbt);
	/* 最初に TS の出力を止め (0xc3 = 1)、最後に復調器のクロックを止める (0x2c = 0) */
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xc3) {
			off = i;
			break;
		}
	CHECK(off >= 0 && M.ev[off].data[0] == 1);
	CHECK(last_write(SLVT, 0x2c) == 0x00 && last_write(SLVX, 0x17) == 0x01 && last_write(SLVT, 0xa9) == 0x00);
	/* sleep の後に、もう一度選局できる (起動の手順が走る) */
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbt(&D, 563143) == 0 && count_writes(SLVT, 0x2c) == 1);
	/* 失敗 */
	M.fail_at = M.calls + 3;
	CHECK(tbs5530_demod_sleep(&D) == -EIO && B.err == 0);
}

/*
 * ISDB-C の起動で、復調器 (SLVX / SLVT) へ書く列。TBS6281SEv2_drv (GPL-2.0-or-later) の SLtoAIC と
 * SLtoAIC_BandAndTSMFSetting の書き込みの順序と値に合わせる (固定の値: BB-AGC 0x48、IF 3.7 MHz)。
 * TS クロックのマスクと A9 / DE / DA の再設定は、起動と set_ts_clock で済んでいるため、含まない。
 */
struct gw {
	u8 addr, reg, len;
	u8 data[14];
};

static const struct gw ISDBC_WAKE[] = {
	{ SLVX, 0x00, 1, { 0x00 } },
	{ SLVX, 0x17, 1, { 0x04 } },
	{ SLVT, 0x00, 1, { 0x00 } },
	{ SLVT, 0xa9, 1, { 0x00 } },
	{ SLVT, 0x2c, 1, { 0x01 } },
	{ SLVT, 0x4b, 1, { 0x74 } },
	{ SLVT, 0x49, 1, { 0x00 } },
	{ SLVX, 0x18, 1, { 0x00 } },
	{ SLVT, 0x00, 1, { 0x11 } },
	{ SLVT, 0x6a, 1, { 0x48 } },
	{ SLVT, 0x00, 1, { 0x10 } },
	{ SLVT, 0xa5, 1, { 0x01 } },
	{ SLVT, 0x00, 1, { 0x11 } },
	{ SLVT, 0x33, 3, { 0x00, 0x03, 0x3b } },
	{ SLVT, 0x00, 1, { 0x48 } },
	{ SLVT, 0x2c, 1, { 0x03 } },
	{ SLVT, 0xac, 1, { 0x00 } },
	{ SLVT, 0x00, 1, { 0x00 } },
	{ SLVT, 0xce, 2, { 0x01, 0x01 } },
	{ SLVT, 0x00, 1, { 0x10 } },
	{ SLVT, 0xa6, 14, { 0x31, 0xa8, 0x29, 0x9b, 0x27, 0x9c, 0x28, 0x9e, 0x29, 0xa4, 0x29, 0xa2, 0x29, 0xa8 } },
	{ SLVT, 0xb6, 3, { 0x13, 0xbb, 0xbc } },
	{ SLVT, 0xd2, 1, { 0x09 } },
	{ SLVT, 0x00, 1, { 0x11 } },
	{ SLVT, 0xa3, 1, { 0x14 } },
	{ SLVT, 0x00, 1, { 0x40 } },
	{ SLVT, 0x26, 4, { 0x07, 0x33, 0x00, 0xee } },
	{ SLVT, 0x14, 1, { 0x1f } },
	{ SLVT, 0x1e, 1, { 0x00 } },
	{ SLVT, 0x00, 1, { 0x00 } },
	{ SLVT, 0x80, 1, { 0x08 } },
};

/* 記録の中の SLVX / SLVT の書き込みを、from 番目から、golden と順に照合する。一致した件数 (全件なら n) を返す */
static int match_writes(int from, const struct gw *g, int n)
{
	int i, k = 0;

	for (i = from; i < M.n && k < n; i++) {
		if (M.ev[i].kind != EV_I2C_W)
			continue;
		if (M.ev[i].addr != g[k].addr || M.ev[i].reg != g[k].reg || M.ev[i].len != g[k].len ||
		    memcmp(M.ev[i].data, g[k].data, g[k].len))
			break;
		k++;
	}
	return k;
}

/* ISDB-C の起動の列の先頭 (SLVX 0x17 = 0x04 の直前の SLVX 0x00 の書き込み) の位置。無ければ -1 */
static int wake_start(void)
{
	int i, j;

	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVX && M.ev[i].reg == 0x17 && M.ev[i].data[0] == 0x04)
			break;
	for (j = i - 1; j >= 0; j--)
		if (M.ev[j].kind == EV_I2C_W && M.ev[j].addr == SLVX && M.ev[j].reg == 0x00)
			return i < M.n ? j : -1;
	return -1;
}

static void test_isdbc_tune(void)
{
	u32 khz = 111000; /* C13 */
	int i, blk = -1, from;
	u8 *t;

	attach_ok();
	CHECK(tbs5530_demod_init(&D) == 0);
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbc(&D, khz) == 0);
	CHECK(D.state == TBS5530_DEMOD_ACTIVE && D.isdbc && !D.isdbt);

	/* 復調器の起動の列が、参考にする実装の順序と値に一致する */
	from = wake_start();
	CHECK(from >= 0);
	CHECK(match_writes(from, ISDBC_WAKE, (int)(sizeof(ISDBC_WAKE) / sizeof(ISDBC_WAKE[0]))) ==
	      (int)(sizeof(ISDBC_WAKE) / sizeof(ISDBC_WAKE[0])));
	/* ISDB-T の起動の値 (SLVX 0x17 = 0x06、ISDB-T の中間周波数) は書かない */
	CHECK(last_write(SLVX, 0x17) == 0x04 && count_writes(SLVT, 0xd7) == 0 && count_writes(SLVT, 0x9f) == 0);

	/* チューナー: ケーブル用の 0x3c、0x52、0x8b と、0x68 からの 17 バイト */
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == TUNER && M.ev[i].reg == 0x68)
			blk = i;
	CHECK(blk >= 0 && M.ev[blk].len == 17);
	t = M.ev[blk].data;
	CHECK(t[8] == (khz & 0xff) && t[9] == ((khz >> 8) & 0xff) && t[10] == ((khz >> 16) & 0x1f));
	CHECK(t[1] == 0x83 && t[3] == 0x0b && t[4] == 0x33 && t[5] == 0x00 && t[6] == 0x1a && t[7] == 0x1c);
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == TUNER && M.ev[i].reg == 0x3c) {
			CHECK(M.ev[i].len == 3 && M.ev[i].data[0] == 0xfc && M.ev[i].data[1] == 0x87 && M.ev[i].data[2] == 0xcf);
		}
	CHECK(last_write(TUNER, 0x52) == 0x78 && last_write(TUNER, 0x8b) == 0x05);
	/* TS の出力は、チューナーの設定の後に有効にする。リピーターは、選局の後に無効 */
	{
		int c3 = -1;

		for (i = 0; i < M.n; i++)
			if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xc3)
				c3 = i;
		CHECK(c3 > blk && M.ev[c3].data[0] == 0x00);
	}
	CHECK(last_write(SLVX, 0x08) == 0x00);

	/* 起きている状態の再選局: 起動の手順は繰り返さず、TS を止め、帯域設定をやり直す */
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbc(&D, 117000) == 0);
	CHECK(count_writes(SLVT, 0x2c) == 0 && count_writes(SLVT, 0x4b) == 0 && count_writes(SLVX, 0x17) == 0);
	CHECK(count_writes(SLVT, 0xb6) == 1 && count_writes(SLVT, 0x26) == 1 && count_writes(SLVT, 0xac) == 1);
	{
		int off = -1, on = -1;

		for (i = 0; i < M.n; i++)
			if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == SLVT && M.ev[i].reg == 0xc3) {
				if (M.ev[i].data[0] == 1 && off < 0)
					off = i;
				if (M.ev[i].data[0] == 0)
					on = i;
			}
		CHECK(off >= 0 && on > off);
	}
	for (i = 0; i < M.n; i++)
		if (M.ev[i].kind == EV_I2C_W && M.ev[i].addr == TUNER && M.ev[i].reg == 0x68)
			blk = i;
	CHECK(M.ev[blk].data[8] == (117000 & 0xff) && M.ev[blk].data[9] == ((117000 >> 8) & 0xff));

	/* 周波数の範囲 (Freia の 42〜1002 MHz) */
	attach_ok();
	CHECK(tbs5530_demod_tune_isdbc(&D, 41999) == -EINVAL);
	CHECK(tbs5530_demod_tune_isdbc(&D, 1002001) == -EINVAL);
	CHECK(tbs5530_demod_tune_isdbc(&D, 93000) == 0);
	CHECK(tbs5530_demod_tune_isdbc(&D, 767000) == 0);

	/* 転送の失敗: 失敗が返り、次の選局は通る */
	for (i = 1; i <= 80; i += 7) {
		attach_ok();
		CHECK(tbs5530_demod_init(&D) == 0);
		M.fail_at = M.calls + i;
		CHECK(tbs5530_demod_tune_isdbc(&D, khz) == -EIO);
		CHECK(B.err == 0);
		M.fail_at = 0;
		CHECK(tbs5530_demod_tune_isdbc(&D, khz) == 0);
		CHECK(D.state == TBS5530_DEMOD_ACTIVE && D.isdbc && !D.isdbt);
	}
}

/* 方式の切り替え: 前の方式の設定を取り消してから、次の方式の起動の手順を走らせる */
static void test_isdbc_switch(void)
{
	int i, k;

	attach_ok();
	CHECK(tbs5530_demod_init(&D) == 0);
	CHECK(tbs5530_demod_tune_isdbt(&D, 557143) == 0 && D.isdbt && !D.isdbc);

	/* ISDB-T -> ISDB-C: ISDB-T の設定を取り消し (0x69 = 0x05)、ISDB-C の起動の列が走る */
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbc(&D, 111000) == 0);
	CHECK(D.isdbc && !D.isdbt);
	CHECK(count_writes(SLVT, 0x69) == 1 && last_write(SLVT, 0x69) == 0x05);
	k = wake_start();
	CHECK(k >= 0 && match_writes(k, ISDBC_WAKE, (int)(sizeof(ISDBC_WAKE) / sizeof(ISDBC_WAKE[0]))) ==
			(int)(sizeof(ISDBC_WAKE) / sizeof(ISDBC_WAKE[0])));
	/* ISDB-T の起動の値は、ISDB-C の起動に残らない (ISDB-T の起動の後の 0xa8 は書かない) */
	CHECK(count_writes(SLVT, 0xa8) == 0);

	/* ISDB-C -> ISDB-T: ISDB-C の設定を取り消す (0xa3 = 0、0x14 = 0x1f、0x1e = 0)。ISDB-T の帯域設定が走る */
	M.n = 0;
	CHECK(tbs5530_demod_tune_isdbt(&D, 557143) == 0);
	CHECK(D.isdbt && !D.isdbc);
	{
		int a3 = -1, c14 = -1, wake = -1;

		for (i = 0; i < M.n; i++) {
			if (M.ev[i].kind != EV_I2C_W || M.ev[i].addr != SLVT)
				continue;
			if (M.ev[i].reg == 0xa3 && M.ev[i].data[0] == 0x00 && a3 < 0)
				a3 = i;
			if (M.ev[i].reg == 0x14 && M.ev[i].data[0] == 0x1f && c14 < 0)
				c14 = i;
			if (M.ev[i].reg == 0xb6 && M.ev[i].data[0] == 0x12)
				wake = i;
		}
		CHECK(a3 >= 0 && c14 > a3 && wake > c14);
	}
	CHECK(last_write(SLVX, 0x17) == 0x06);

	/* ISDB-C のまま sleep: 取り消しの後に、基本の設定へ戻り、方式の記録が消える */
	CHECK(tbs5530_demod_tune_isdbc(&D, 111000) == 0);
	M.n = 0;
	CHECK(tbs5530_demod_sleep(&D) == 0);
	CHECK(D.state == TBS5530_DEMOD_SLEEP && !D.isdbc && !D.isdbt);
	CHECK(count_writes(SLVT, 0x1e) == 1 && last_write(SLVT, 0x1e) == 0x00 && count_writes(SLVT, 0x69) == 0);
	CHECK(last_write(SLVT, 0x2c) == 0x00 && last_write(SLVX, 0x17) == 0x01);
}

static void test_isdbc_status(void)
{
	u8 st = 0xff;
	s32 cnr = 0;

	attach_ok();
	CHECK(tbs5530_demod_init(&D) == 0);
	CHECK(tbs5530_demod_tune_isdbc(&D, 111000) == 0);

	/* バンク 0x40 のレジスタ 0x88 の bit0 (AGC の収束) と、0x10 の bit5 (同期) の両方でロック */
	M.regs[SLVT][0x40][0x88] = 0x01;
	M.regs[SLVT][0x40][0x10] = 0x20;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0);
	CHECK(st == (TBS5530_STATUS_SIGNAL | TBS5530_STATUS_LOCK) && D.led_locked);
	M.regs[SLVT][0x40][0x10] = 0x00; /* 搬送波だけ */
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == TBS5530_STATUS_SIGNAL && !D.led_locked);
	M.regs[SLVT][0x40][0x88] = 0x00; /* 同期だけでは、ロックではない */
	M.regs[SLVT][0x40][0x10] = 0x20;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == 0);
	/* ISDB-T のロックのレジスタ (バンク 0x60) は見ない */
	M.regs[SLVT][0x60][0x10] = 0x03;
	CHECK(tbs5530_demod_read_status(&D, &st) == 0 && st == 0);

	/* 失敗 */
	M.fail_at = M.calls + 1;
	CHECK(tbs5530_demod_read_status(&D, &st) == -EIO && B.err == 0);

	/* C/N は、換算を確認していないため未対応 */
	CHECK(tbs5530_demod_read_cnr(&D, &cnr) == -ENOSYS);
}

int main(void)
{
	test_i2c();
	test_bridge();
	test_math();
	test_attach();
	test_init();
	test_tune();
	test_status();
	test_sleep();
	test_isdbc_tune();
	test_isdbc_switch();
	test_isdbc_status();
	printf("tbs5530_core_test: %d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
