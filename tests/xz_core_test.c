/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GV-MVP/XZ2 の共通層 (driver/common/xz) の自動試験。
 *
 * 機材を模擬する: ベンダー要求 0xBC のメモリ (16 MiB、取り込みブロックと SoC のレジスタを含む)、SoC の I2C コントローラ
 * (復調器 0x18 と 0x19 だけが ACK を返す)、hook から動く機材内プログラムのポンプ (郵便受けと起動の書き込み)。
 * 確認するのは、起動直後の識別 (別の版や使用済みの機材には何も書かない)、書き込みが 1 回 4 バイト・読み出しが 64 バイトを
 * 超えないこと、SoC の初期化の値、I2C の通信と失敗 (ACK なし、時間切れ、停止条件)、前面の列と周波数の差し替え、
 * 取り込みの書き込み順序、hook の導入と復元の順序、失敗の注入、ポンプの引数と結果の判定。
 * 実機の代用ではない (値の出どころと実機の結果は docs/devices/xz.md)。
 *
 * build: gcc -Wall -Wextra -fsanitize=address,undefined -I driver/common/xz tests/xz_core_test.c \
 *            driver/common/xz/xz_{link,board,frontend,program}.c -o xz_core_test
 */
#include "xz_core.h"
#include "xz_program_gen.h"
#include "xz_fe_tables_gen.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HOOK 0xe9f48u
#define BODY 0xe1000u
#define MAILBOX 0xe1180u
#define TRIGGER 0xe11fcu
#define I2C_STATUS 0xfd8fe0u
#define I2C_CONTROL 0xfd8fe4u
#define I2C_DATA 0xfd8ff0u

static const u8 hook_original[8] = { 0xc4, 0x0d, 0x02, 0xa0, 0xa8, 0x10, 0xe3, 0x0e };
static const u8 hook_program[8] = { 0x9b, 0xec, 0x10, 0x00, 0x97, 0x0c, 0xe3, 0x0e };
static const u8 signature[8] = { 0xb4, 0x27, 0x9f, 0x8d, 0x00, 0x0c, 0x46, 0x00 };

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

struct write_rec {
	u32 address;
	u32 value;	/* 先頭の 4 バイト (ビッグエンディアン) */
	u16 len;
};

struct xfer {
	u8 address;
	u8 w[48];
	unsigned int wlen, rlen;
};

#define MAX_WRITES 4096
#define MAX_XFERS 128

static struct {
	u8 mem[0x1000000];
	struct write_rec writes[MAX_WRITES];
	unsigned int nwrites;
	unsigned int calls, fail_at;	/* fail_at 番目の control を失敗させる (1 回だけ) */
	int fail_ret;
	int dead;			/* 機材の取り外し: すべて -ENODEV */
	unsigned int violations;	/* 読み 64 バイト超、書き 4 バイト超 */
	int drop_hook, drop_body, drop_block;	/* この領域への書き込みを無視する */
	int scribble;			/* 書き込みの転送の後に、転送のバッファを破壊する (WinUSB のように、バッファを書き換える USB 層の模擬) */
	u64 now;
	unsigned int slept;
	unsigned int error_logs;
	/* I2C コントローラ */
	int phase;			/* 0 空き、1 書き、2 読み */
	int nack_all, no_done;
	u8 cur_addr;
	struct xfer cur, xfers[MAX_XFERS];
	unsigned int nxfers;
	u8 status_reg, data_reg;
	u8 demod[256];
	u8 ptr;
	/* ポンプ */
	u32 pump_result;
	int pump_sent;	/* 負なら要求の個数 */
	unsigned int pumps;
} M;

static u32 be32(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

static void put32(u8 *p, u32 v)
{
	p[0] = (u8)(v >> 24);
	p[1] = (u8)(v >> 16);
	p[2] = (u8)(v >> 8);
	p[3] = (u8)v;
}

static u32 rd32(u32 a) { return be32(M.mem + a); }
static void wr32(u32 a, u32 v) { put32(M.mem + a, v); }

static void i2c_finish(void)
{
	if (M.phase && M.nxfers < MAX_XFERS)
		M.xfers[M.nxfers++] = M.cur;
	M.phase = 0;
}

static void i2c_control(u8 c)
{
	M.status_reg = 0;
	if (c == 0x42) {
		i2c_finish();
		return;
	}
	/* 読みの途中の「繰り返しの開始条件」(0x72) は、書きの通信を区切って、読みのアドレス段へ進む */
	if (M.phase == 0 || (M.phase == 1 && c == 0x72)) {
		u8 byte = M.data_reg;

		if (M.phase == 0) {
			memset(&M.cur, 0, sizeof(M.cur));
			M.cur.address = byte >> 1;
		}
		if (M.nack_all || (M.cur.address != 0x18 && M.cur.address != 0x19)) {
			M.status_reg = 0x10;
			M.phase = 1;
			return;
		}
		M.phase = (byte & 1) ? 2 : 1;
		return;
	}
	if (M.phase == 1) {
		if (M.cur.wlen < sizeof(M.cur.w))
			M.cur.w[M.cur.wlen] = M.data_reg;
		M.cur.wlen++;
		if (M.cur.wlen == 1)
			M.ptr = M.data_reg;
		return;
	}
	M.data_reg = M.demod[M.ptr++];
	M.cur.rlen++;
}

static void pump_trigger(void)
{
	u32 count = rd32(MAILBOX + 4);

	M.pumps++;
	/* 本体を通ったときだけ、結果を書き換える (hook が入っていなければ 0xFFFFFFFF のまま) */
	if (rd32(MAILBOX) == 0x11 && !memcmp(M.mem + HOOK, hook_program, 8)) {
		wr32(MAILBOX + 0x20, M.pump_result);
		wr32(MAILBOX + 0x24, M.pump_sent < 0 ? count : (u32)M.pump_sent);
		wr32(MAILBOX + 0x28, rd32(MAILBOX + 0x0c) + count * 512u);
	}
}

static int mock_control(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms)
{
	u32 address = ((u32)(value & 0xff00u) << 8) | index;
	u8 *p = buf;
	unsigned int i;

	(void)priv;
	(void)timeout_ms;
	M.calls++;
	if (M.dead)
		return -ENODEV;
	if (M.fail_at && M.calls == M.fail_at) {
		M.fail_at = 0;
		return M.fail_ret;
	}
	if (request_type == 0xc0) {
		if (len > 64)
			M.violations++;
		for (i = 0; i < len; ++i) {
			u32 a = address + i;

			if (a >= 0xfd8fe0u && a < 0xfd9000u) {
				/* I2C コントローラ: 語の最下位バイトが値 */
				u32 word = 0;

				if ((a & ~3u) == I2C_CONTROL)
					word = M.no_done ? 0 : 1;
				else if ((a & ~3u) == I2C_STATUS)
					word = M.status_reg;
				else if ((a & ~3u) == I2C_DATA)
					word = M.data_reg;
				p[i] = (u8)(word >> (8 * (3 - (a & 3u))));
			} else {
				p[i] = M.mem[a];
			}
		}
		return len;
	}
	if (len > 4)
		M.violations++;
	if (M.nwrites < MAX_WRITES) {
		M.writes[M.nwrites].address = address;
		M.writes[M.nwrites].value = len >= 4 ? be32(p) : 0;
		M.writes[M.nwrites].len = len;
		M.nwrites++;
	}
	if (len == 4 && address == I2C_CONTROL) {
		i2c_control(p[3]);
		return len;
	}
	if (len == 4 && address == I2C_DATA) {
		M.data_reg = p[3];
		return len;
	}
	if (address == TRIGGER && (value & 0xff) == 0x7d) {
		pump_trigger();
		return len;
	}
	for (i = 0; i < len; ++i) {
		u32 a = address + i;

		if ((M.drop_hook && a >= HOOK && a < HOOK + 8) || (M.drop_body && a >= BODY && a < BODY + 0x200) ||
		    (M.drop_block && a >= 0xd00000u && a < 0xd03000u))
			continue;
		M.mem[a] = p[i];
	}
	if (M.scribble)
		memset(p, 0xaa, len);
	return len;
}

static void mock_msleep(void *priv, unsigned int ms)
{
	(void)priv;
	M.now += ms;
	M.slept += ms;
}

static u64 mock_now(void *priv)
{
	(void)priv;
	return M.now++;
}

static void mock_log(void *priv, int level, const char *fmt, ...)
{
	(void)priv;
	(void)fmt;
	if (level == XZ_LOG_ERR)
		M.error_logs++;
}

static const struct xz_ops ops = { mock_control, mock_msleep, mock_now, mock_log };

static struct xz_bridge bridge;

/* 起動直後の機材。 */
static void fresh(void)
{
	memset(&M, 0, sizeof(M));
	wr32(0xf00000, 0x4d423841);
	wr32(0xf00004, 0x43303138);
	memcpy(M.mem + 0xe9ce8, signature, sizeof(signature));
	wr32(0xe9a2c, 0x00f00000);
	wr32(0xe9d78, 0x00f00000);
	memcpy(M.mem + HOOK, hook_original, 8);
	/* 起動直後の SoC のクロック系 */
	wr32(0xf10114, 0xba000000);
	wr32(0xf10118, 0x60100000);
	wr32(0xf10128, 0);
	xz_bridge_init(&bridge, &ops, NULL);
}

static int wrote_to(u32 lo, u32 hi)
{
	unsigned int i;

	for (i = 0; i < M.nwrites; ++i)
		if (M.writes[i].address >= lo && M.writes[i].address < hi)
			return 1;
	return 0;
}

/* 書き込みの記録の中で、address への最初の書き込みの番号 (無ければ -1)。 */
static int first_write(u32 address)
{
	unsigned int i;

	for (i = 0; i < M.nwrites; ++i)
		if (M.writes[i].address == address)
			return (int)i;
	return -1;
}

static int last_write(u32 address)
{
	int found = -1;
	unsigned int i;

	for (i = 0; i < M.nwrites; ++i)
		if (M.writes[i].address == address)
			found = (int)i;
	return found;
}

/* ---- 試験 ---- */

static void test_link(void)
{
	u8 buf[200], expect[200];
	unsigned int i;

	fresh();
	for (i = 0; i < sizeof(expect); ++i)
		M.mem[0x10000 + i] = expect[i] = (u8)(i * 7 + 1);
	CHECK(xz_read(&bridge, 0x10000, buf, 200) == 0 && !memcmp(buf, expect, 200));
	CHECK(M.calls == 4 && M.violations == 0);	/* 64 + 64 + 64 + 8 */
	/* 書きは 4 バイトずつ */
	M.calls = 0;
	CHECK(xz_write(&bridge, 0x20000, expect, 10) == 0 && !memcmp(M.mem + 0x20000, expect, 10));
	CHECK(M.calls == 3 && M.violations == 0);
	CHECK(M.writes[0].len == 4 && M.writes[2].len == 2 && M.writes[2].address == 0x20008);
	/* 24 ビットの上位バイトが wValue へ (0xF10118 の読み出し) */
	CHECK(xz_read32(&bridge, 0xf10118, &i) == 0 && i == 0x60100000);
	/*
	 * 書き込みの転送が、渡されたバッファを書き換えても、呼び出し側のデータは変わらない
	 * (WinUSB は書き込み可能なバッファを要求する。読み取り専用のデータ (機械語) を直接渡すと、エラー 998 で失敗する)
	 */
	{
		static const u8 readonly[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
		u8 copy[10];

		memcpy(copy, readonly, sizeof(copy));
		M.scribble = 1;
		CHECK(xz_write(&bridge, 0x30000, copy, 10) == 0);
		M.scribble = 0;
		CHECK(!memcmp(copy, readonly, sizeof(copy)));	/* 呼び出し側のバッファは無事 */
		CHECK(!memcmp(M.mem + 0x30000, readonly, sizeof(readonly)));	/* 機材には正しいデータが届く */
	}
	/* 範囲外と空 */
	CHECK(xz_read(&bridge, 0x1000000, buf, 4) == -EINVAL);
	CHECK(xz_read(&bridge, 0xfffffe, buf, 4) == -EINVAL);
	CHECK(xz_write(&bridge, 0x10000, expect, 0) == -EINVAL);
	CHECK(xz_write(&bridge, 0x10000, NULL, 4) == -EINVAL);
	/* 失敗の注入: 2 回目の読み出しが失敗したら、そのエラーを返す */
	M.calls = 0;
	M.fail_at = 2;
	M.fail_ret = -ETIMEDOUT;
	CHECK(xz_read(&bridge, 0x10000, buf, 200) == -ETIMEDOUT);
	CHECK(xz_write(&bridge, 0x20000, expect, 8) == 0);
	/* 取り外し */
	M.dead = 1;
	CHECK(xz_read32(&bridge, 0xf00000, &i) == -ENODEV);
	CHECK(xz_write32(&bridge, 0x20000, 0) == -ENODEV);
}

static void test_board_open(void)
{
	/* 起動直後: SoC の値が仕様どおりで、待ちが入る */
	fresh();
	CHECK(xz_board_open(&bridge) == 0);
	CHECK(rd32(0xf10114) == 0x80080000u);	/* 実機の初期化の後の値 */
	CHECK(rd32(0xf10118) == 0xffff0000u);	/* クロックのゲート全開 */
	CHECK(rd32(0xf10014) == 0xd0f40000u);
	CHECK(rd32(0xf10018) == 0x10e40000u);	/* 種類 5: 半語に 0x1000 を足してから */
	CHECK(rd32(0xfd8010) == 0x00000010u);
	CHECK(rd32(0xfd8fe8) == 0x0000003cu);
	CHECK(M.slept >= 10);
	CHECK(M.violations == 0 && M.error_logs == 0);
	/* 書き込みの順序: ゲート (F10118) は PLL (F10014) より先 */
	CHECK(first_write(0xf10118) >= 0 && first_write(0xf10118) < first_write(0xf10014));
	CHECK(first_write(0xf10014) < first_write(0xf10018) && first_write(0xf10018) < first_write(0xfd8fe8));
	CHECK(!wrote_to(0xe0000, 0xf00000));	/* hook と本体の領域には触らない */

	/* チップ ID の違い: 何も書かない */
	fresh();
	wr32(0xf00000, 0x12345678);
	CHECK(xz_board_open(&bridge) == -ENODEV && M.nwrites == 0 && M.error_logs == 1);
	/* boot0 の署名の違い (別の版): 何も書かない */
	fresh();
	M.mem[0xe9ce8] ^= 1;
	CHECK(xz_board_open(&bridge) == -ENODEV && M.nwrites == 0);
	/* 起動直後でない: 窓の即値、状態、hook のどれが違っても、何も書かない */
	fresh();
	wr32(0xe9a2c, 0x01900000);
	CHECK(xz_board_open(&bridge) == -EBUSY && M.nwrites == 0);
	fresh();
	wr32(0xe9d78, 0x01900000);
	CHECK(xz_board_open(&bridge) == -EBUSY && M.nwrites == 0);
	fresh();
	wr32(0x82008, 0x00010000);
	CHECK(xz_board_open(&bridge) == -EBUSY && M.nwrites == 0);
	fresh();
	M.mem[HOOK] ^= 0xff;
	CHECK(xz_board_open(&bridge) == -EBUSY && M.nwrites == 0);
	/* 識別の読み出しの失敗は、そのエラー */
	fresh();
	M.dead = 1;
	CHECK(xz_board_open(&bridge) == -ENODEV);
	/* 初期化の途中の書き込みの失敗は、そのエラーで止まる (後続を書かない) */
	fresh();
	{
		unsigned int total;

		CHECK(xz_board_open(&bridge) == 0);
		total = M.calls;
		fresh();
		M.fail_at = total / 2;
		M.fail_ret = -EIO;
		CHECK(xz_board_open(&bridge) == -EIO);
		CHECK(rd32(0xfd8fe8) == 0);	/* 最後の項目までは進んでいない */
	}
	/* 復調器が応答しないとき: ACK なしを返す */
	fresh();
	M.nack_all = 1;
	CHECK(xz_board_open(&bridge) == -ENXIO);
}

static void test_i2c(void)
{
	u8 w[3] = { 0x11, 0x22, 0x33 }, r[4], reg = 0x80;

	fresh();
	/* 書きだけ */
	CHECK(xz_i2c_transfer(&bridge, 0x18, w, 3, NULL, 0) == 0);
	CHECK(M.nxfers == 1 && M.xfers[0].address == 0x18 && M.xfers[0].wlen == 3 && !memcmp(M.xfers[0].w, w, 3));
	/* 書いて読む: 読みのアドレス段は繰り返しの開始条件 (0x72) */
	M.demod[0x80] = 0xaa;
	M.demod[0x81] = 0xbb;
	M.nxfers = 0;
	CHECK(xz_i2c_transfer(&bridge, 0x18, &reg, 1, r, 2) == 0);
	CHECK(r[0] == 0xaa && r[1] == 0xbb);
	CHECK(M.nxfers == 1 && M.xfers[0].wlen == 1 && M.xfers[0].rlen == 2);
	/* 読みだけ (繰り返しの開始条件ではなく 0x52) */
	M.nxfers = 0;
	M.ptr = 0x80;
	CHECK(xz_i2c_transfer(&bridge, 0x18, NULL, 0, r, 1) == 0 && r[0] == 0xaa);
	/* ACK なし: -ENXIO で、停止条件 (0x42) を出す */
	M.nwrites = 0;
	CHECK(xz_i2c_transfer(&bridge, 0x60, w, 3, NULL, 0) == -ENXIO);
	CHECK(M.writes[M.nwrites - 1].address == I2C_CONTROL && (M.writes[M.nwrites - 1].value & 0xff) == 0x42);
	/* 完了しない: -ETIMEDOUT。停止条件は出そうとする */
	M.no_done = 1;
	M.nwrites = 0;
	CHECK(xz_i2c_transfer(&bridge, 0x18, w, 3, NULL, 0) == -ETIMEDOUT);
	CHECK(M.nwrites > 0 && (M.writes[M.nwrites - 1].value & 0xff) == 0x42);
	M.no_done = 0;
	/* 引数 */
	CHECK(xz_i2c_transfer(&bridge, 0x18, NULL, 0, NULL, 0) == -EINVAL);
	CHECK(xz_i2c_transfer(&bridge, 0x18, w, 3, NULL, 38) == -EINVAL);
	CHECK(xz_i2c_transfer(&bridge, 0x18, w, 3, NULL, 0) == 0);
	/* USB の失敗はそのエラー */
	M.calls = 0;
	M.fail_at = 3;
	M.fail_ret = -EPIPE;
	CHECK(xz_i2c_transfer(&bridge, 0x18, w, 3, NULL, 0) == -EPIPE);
	CHECK(M.violations == 0);
}

static void test_frontend(void)
{
	bool locked = true;
	u32 raw;
	static const struct { u32 khz; u8 low, high; } freq[] = {
		{ 473143, 0x49, 0x76 },	/* 13ch: 30281 */
		{ 545143, 0x49, 0x88 },	/* 25ch: 34889 */
		{ 575143, 0xc9, 0x8f },	/* 30ch: 36809 */
	};
	unsigned int i;

	/* 初期化: 表 (記録の 0〜57 番) と全項目が、同じ順序・同じアドレス・同じバイト列で送られる */
	fresh();
	CHECK(xz_fe_init(&bridge) == 0);
	CHECK(M.nxfers == XZ_FE_INIT_COUNT && M.nxfers == 58);
	for (i = 0; i < XZ_FE_INIT_COUNT && i < M.nxfers; ++i) {
		CHECK(M.xfers[i].address == xz_fe_writes[i].address && M.xfers[i].wlen == xz_fe_writes[i].length &&
		      !memcmp(M.xfers[i].w, xz_fe_writes[i].data, xz_fe_writes[i].length));
	}
	CHECK(M.xfers[0].address == 0x18 && M.xfers[22].address == 0x19);	/* 衛星側 (0x19) の項目も含む */
	CHECK(M.slept >= 2 * XZ_FE_INIT_COUNT);
	/* 選局: 18 回。表の値のうち、周波数の 2 バイトだけが変わる */
	for (i = 0; i < sizeof(freq) / sizeof(freq[0]); ++i) {
		unsigned int k;

		fresh();
		CHECK(xz_fe_tune(&bridge, freq[i].khz) == 0);
		CHECK(M.nxfers == XZ_FE_TUNE_COUNT && M.nxfers == 18);
		for (k = 0; k < XZ_FE_TUNE_COUNT && k < M.nxfers; ++k) {
			const struct xz_fe_write *w = &xz_fe_writes[XZ_FE_INIT_COUNT + k];
			u8 expect[40];

			memcpy(expect, w->data, sizeof(expect));
			if (k == XZ_FE_FREQ_INDEX) {
				expect[7] = freq[i].low;
				expect[9] = freq[i].high;
			}
			CHECK(M.xfers[k].address == w->address && M.xfers[k].wlen == w->length && !memcmp(M.xfers[k].w, expect, w->length));
		}
		CHECK(M.xfers[XZ_FE_FREQ_INDEX].wlen == 10 && M.xfers[XZ_FE_FREQ_INDEX].w[0] == 0xfe &&
		      M.xfers[XZ_FE_FREQ_INDEX].w[6] == 0x14 && M.xfers[XZ_FE_FREQ_INDEX].w[8] == 0x15);
	}
	/* 範囲外 (UHF 以外) は何も送らない */
	fresh();
	CHECK(xz_fe_tune(&bridge, 469999) == -EINVAL && xz_fe_tune(&bridge, 770001) == -EINVAL && M.nxfers == 0);
	/* 失敗はそのエラーで止まる */
	fresh();
	M.nack_all = 1;
	CHECK(xz_fe_tune(&bridge, 545143) == -ENXIO && M.nxfers == 1);	/* 1 件目で止まる */
	fresh();
	M.nack_all = 1;
	CHECK(xz_fe_init(&bridge) == -ENXIO && M.nxfers == 1);

	/* ロックの判定: (0x80 & 0x28) == 0 かつ (0xB0 & 0x0F) >= 8 */
	fresh();
	M.demod[0x80] = 0x00;
	M.demod[0xb0] = 0xa9;
	CHECK(xz_fe_locked(&bridge, &locked) == 0 && locked);
	M.demod[0x80] = 0x08;
	CHECK(xz_fe_locked(&bridge, &locked) == 0 && !locked);
	M.demod[0x80] = 0x20;
	CHECK(xz_fe_locked(&bridge, &locked) == 0 && !locked);
	M.demod[0x80] = 0x46;	/* 0x28 のビットが立たない値 */
	M.demod[0xb0] = 0xa7;	/* 下位 4 ビットが 7 */
	CHECK(xz_fe_locked(&bridge, &locked) == 0 && !locked);
	M.demod[0xb0] = 0xa8;
	CHECK(xz_fe_locked(&bridge, &locked) == 0 && locked);
	M.nack_all = 1;
	CHECK(xz_fe_locked(&bridge, &locked) == -ENXIO);
	M.nack_all = 0;

	/* C/N: 0x8B から 3 バイト */
	M.demod[0x8b] = 0x01;
	M.demod[0x8c] = 0x23;
	M.demod[0x8d] = 0x45;
	CHECK(xz_fe_cnr_raw(&bridge, &raw) == 0 && raw == 0x012345);

	/* 休止: 地上波 0x18 へ 03 90、衛星 0x19 へ 17 01 */
	M.nxfers = 0;
	CHECK(xz_fe_sleep(&bridge) == 0 && M.nxfers == 2 && M.xfers[0].address == 0x18 && M.xfers[0].w[0] == 0x03 &&
	      M.xfers[0].w[1] == 0x90 && M.xfers[1].address == 0x19 && M.xfers[1].w[0] == 0x17 && M.xfers[1].w[1] == 0x01);
}

static void test_capture(void)
{
	u32 wptr;

	fresh();
	CHECK(xz_cap_start(&bridge) == 0);
	/* 先頭はサブブロックのリセット解除、最後は 0xD00004 */
	CHECK(M.writes[0].address == 0xd000c4 && M.writes[0].value == 0);
	CHECK(M.writes[1].address == 0xd00000 && M.writes[1].value == 0xffffffffu);
	CHECK(M.writes[M.nwrites - 1].address == 0xd00004 && M.writes[M.nwrites - 1].value == 0x00043305);
	/* 設定値 (mAgicVISION ファームウェアと同じ。torne 流 (0xD01004 = 0) ではない) */
	CHECK(rd32(0xd01004) == 0x04000000 && rd32(0xd0108c) == 0xffffffffu && rd32(0xd01000) == 0x06070000);
	CHECK(rd32(0xd01044) == 0x00bc0000 && rd32(0xd01048) == 0x47000000 && rd32(0xd01080) == 0x01000000);
	CHECK(rd32(0xd02020) == XZ_RING_BASE && rd32(0xd02024) == XZ_RING_SIZE && rd32(0xd02000) == 0x03020300);
	/* start ではライタを開始しない (読み出し側が動く前に始めると、リングが 1 周して上書きされる) */
	CHECK(rd32(0xd02010) == 0 && rd32(0xd01040) == 0);
	CHECK(M.violations == 0);

	/* サブブロックが書き込みを保持しないとき (ゲートが開いていない): 設定を書かずに -EIO */
	fresh();
	M.drop_block = 1;
	CHECK(xz_cap_start(&bridge) == -EIO);
	CHECK(first_write(0xd01004) < 0 && first_write(0xd02020) < 0);

	/* run: 止める → 位置を 0 → 入力 → ライタの順 */
	fresh();
	CHECK(xz_cap_run(&bridge) == 0);
	CHECK(M.writes[0].address == 0xd02010 && M.writes[0].value == 0x04000000);
	{
		static const u32 pointers[] = { 0xd02028, 0xd0202c, 0xd02030, 0xd02038 };
		unsigned int k;

		for (k = 0; k < sizeof(pointers) / sizeof(pointers[0]); ++k) {
			/* 位置を 0 に戻すのは、止めた後 (先頭の停止の後) で、入力 (0xD01040 = 0x03000000) とライタの開始より前 */
			CHECK(first_write(pointers[k]) > 0);
			CHECK(first_write(pointers[k]) < last_write(0xd01040));
			CHECK(last_write(pointers[k]) < (int)M.nwrites - 1);
		}
	}
	CHECK(M.writes[M.nwrites - 1].address == 0xd02010 && M.writes[M.nwrites - 1].value == 0x02000000);
	CHECK(rd32(0xd01040) == 0x03000000);

	/* stop */
	M.nwrites = 0;
	CHECK(xz_cap_stop(&bridge) == 0);
	CHECK(rd32(0xd02010) == 0x04000000 && rd32(0xd01040) == 0 && rd32(0xd01020) == 0x02000000);

	/* 書き込み位置は、リングの大きさで折り返す。読み出し位置は 192 の倍数 */
	wr32(0xd02028, XZ_RING_SIZE + 192);
	CHECK(xz_cap_wptr(&bridge, &wptr) == 0 && wptr == 192);
	CHECK(xz_cap_set_rptr(&bridge, 192 * 5) == 0 && rd32(0xd0202c) == 960);
	CHECK(xz_cap_set_rptr(&bridge, 100) == -EINVAL);
	CHECK(xz_cap_set_rptr(&bridge, XZ_RING_SIZE) == -EINVAL);
	CHECK(rd32(0xd0202c) == 960);
	M.dead = 1;
	CHECK(xz_cap_wptr(&bridge, &wptr) == -ENODEV);
}

static void test_program(void)
{
	u8 body_before[512], body_back[512];
	int front, back;
	unsigned int i;

	/* 導入: 本体 → 郵便受け → hook (前半 → 後半) */
	fresh();
	for (i = 0; i < 512; ++i)
		M.mem[BODY + i] = body_before[i] = (u8)(0x55 + i);
	CHECK(xz_prog_install(&bridge) == 0 && bridge.program_installed);
	CHECK(!memcmp(M.mem + BODY, xz_program, XZ_PROGRAM_BYTES));
	CHECK(rd32(MAILBOX) == 0 && rd32(MAILBOX + 0x10) == XZ_RING_BASE && rd32(MAILBOX + 0x14) == XZ_RING_BASE + XZ_RING_SIZE);
	CHECK(!memcmp(M.mem + HOOK, hook_program, 8));
	front = first_write(HOOK);
	back = first_write(HOOK + 4);
	CHECK(front >= 0 && back > front);
	CHECK(first_write(MAILBOX + 0x14) < front && first_write(BODY) < front);
	CHECK(M.violations == 0);
	CHECK(xz_prog_install(&bridge) == -EALREADY);

	/* 復元: hook (後半 → 前半) → 本体の領域 */
	M.nwrites = 0;
	CHECK(xz_prog_remove(&bridge) == 0 && !bridge.program_installed);
	CHECK(!memcmp(M.mem + HOOK, hook_original, 8));
	CHECK(first_write(HOOK + 4) >= 0 && first_write(HOOK + 4) < first_write(HOOK) && first_write(HOOK) < first_write(BODY));
	memcpy(body_back, M.mem + BODY, 512);
	CHECK(!memcmp(body_back, body_before, 512));
	/* 導入していないときの復元は、何も書かない (保存していない本体の領域を潰さない) */
	M.nwrites = 0;
	CHECK(xz_prog_remove(&bridge) == 0 && M.nwrites == 0);

	/* hook が元の値でない (他の使い手がいる): 何も書かない */
	fresh();
	M.mem[HOOK + 2] ^= 1;
	CHECK(xz_prog_install(&bridge) == -EBUSY && M.nwrites == 0 && !bridge.program_installed);

	/* 本体の書き込みが保持されない: -EIO で、hook に触れず、本体の領域を元へ戻す */
	fresh();
	M.drop_body = 1;
	CHECK(xz_prog_install(&bridge) == -EIO && !wrote_to(HOOK, HOOK + 8) && !bridge.program_installed);

	/* hook が保持されない: -EIO で、hook を (後半 → 前半の順に) 元へ戻し、本体も戻す */
	fresh();
	M.drop_hook = 1;
	for (i = 0; i < 512; ++i)
		M.mem[BODY + i] = body_before[i] = (u8)(i * 3);
	CHECK(xz_prog_install(&bridge) == -EIO && !bridge.program_installed);
	CHECK(!memcmp(M.mem + HOOK, hook_original, 8) && !memcmp(M.mem + BODY, body_before, 512));

	/*
	 * 導入のどの制御転送が失敗しても、導入済みにならず、hook と本体の領域が元に戻る (失敗させる呼び出しを 1 つずつずらす)。
	 * 戻す途中の書き込みも失敗させない 1 回きりの注入なので、元へ戻せる。
	 */
	{
		unsigned int total, at;

		fresh();
		CHECK(xz_prog_install(&bridge) == 0);
		total = M.calls;
		for (at = 1; at <= total; ++at) {
			fresh();
			for (i = 0; i < 512; ++i)
				M.mem[BODY + i] = body_before[i] = (u8)(i + 9);
			M.fail_at = at;
			M.fail_ret = -EIO;
			CHECK(xz_prog_install(&bridge) == -EIO);
			CHECK(!bridge.program_installed);
			CHECK(!memcmp(M.mem + HOOK, hook_original, 8));
			CHECK(!memcmp(M.mem + BODY, body_before, 512));
		}
	}

	/* 復元の失敗: 記録を残し、導入済みのまま (再試行できる) */
	fresh();
	CHECK(xz_prog_install(&bridge) == 0);
	M.dead = 1;
	CHECK(xz_prog_remove(&bridge) == -ENODEV && bridge.program_installed && M.error_logs == 0);	/* 取り外しは記録しない */
	M.dead = 0;
	M.drop_hook = 1;
	CHECK(xz_prog_remove(&bridge) == -EIO && bridge.program_installed && M.error_logs == 1);
	M.drop_hook = 0;
	CHECK(xz_prog_remove(&bridge) == 0 && !bridge.program_installed);
}

static void test_pump(void)
{
	/* 導入前は拒否 */
	fresh();
	CHECK(xz_prog_pump(&bridge, 0, 16) == -EINVAL);
	CHECK(xz_prog_install(&bridge) == 0);

	/* 正常: 引数が郵便受けへ、起動の書き込みが 4 バイトで出る。結果は 0 */
	M.pump_result = 0;
	M.pump_sent = -1;
	M.nwrites = 0;
	CHECK(xz_prog_pump(&bridge, 192 * 10, 32) == 0);
	CHECK(rd32(MAILBOX + 4) == 32 && rd32(MAILBOX + 8) == 5000000u && rd32(MAILBOX + 0x0c) == XZ_RING_BASE + 1920);
	CHECK(rd32(MAILBOX) == 0x11 && M.pumps == 1 && M.violations == 0);
	/* 起動の書き込みは、引数と操作番号の後 (結果の初期化の後) */
	CHECK(last_write(TRIGGER) > last_write(MAILBOX) && last_write(MAILBOX) > last_write(MAILBOX + 0x20));
	CHECK(M.writes[M.nwrites - 1].address == TRIGGER && M.writes[M.nwrites - 1].len == 4);

	/* 引数の検査 */
	M.pumps = 0;
	CHECK(xz_prog_pump(&bridge, 0, 0) == -EINVAL);
	CHECK(xz_prog_pump(&bridge, 0, 144) == -EINVAL);	/* 128 を超える */
	CHECK(xz_prog_pump(&bridge, 0, 20) == -EINVAL);	/* 16 の倍数でない */
	CHECK(xz_prog_pump(&bridge, XZ_RING_SIZE, 16) == -EINVAL);
	CHECK(xz_prog_pump(&bridge, 2, 16) == -EINVAL);
	CHECK(M.pumps == 0);

	/* 待ちの上限 (結果 1) は -ETIMEDOUT、送った数の食い違いと、本体を通らなかった (結果 0xFFFFFFFF) は -EIO */
	M.pump_result = 1;
	CHECK(xz_prog_pump(&bridge, 0, 16) == -ETIMEDOUT);
	M.pump_result = 0;
	M.pump_sent = 8;
	CHECK(xz_prog_pump(&bridge, 0, 16) == -EIO);
	M.pump_sent = -1;
	memcpy(M.mem + HOOK, hook_original, 8);	/* hook が外れていて、本体を通らない */
	CHECK(xz_prog_pump(&bridge, 0, 16) == -EIO);
	CHECK(M.error_logs >= 3);

	/* 起動の書き込みの失敗はそのエラー */
	memcpy(M.mem + HOOK, hook_program, 8);
	M.calls = 0;
	M.fail_at = 6;	/* 引数 5 回 (4 バイトずつ) の次が起動 */
	M.fail_ret = -ETIMEDOUT;
	CHECK(xz_prog_pump(&bridge, 0, 16) == -ETIMEDOUT);
}

int main(void)
{
	test_link();
	test_board_open();
	test_i2c();
	test_frontend();
	test_capture();
	test_program();
	test_pump();
	printf("%d checks, %d failed\n", checks, fails);
	return fails ? 1 : 0;
}
