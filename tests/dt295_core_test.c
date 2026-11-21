/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PIX-DT295 の共通層 (driver/common/dt295) の自動試験。
 *
 * 機材を模擬する: ベンダー要求 0xBC のメモリ (16 MiB) と取り込みブロック (128 KiB)、boot0 の窓 (即値を RAM から読む)、
 * I2C コントローラ (復調器 0x10 とチューナーの中継)、hook から動く機材内プログラムのポンプ。
 * 確認するのは、起動直後の判定、書き込みの順序、64 バイトの上限、窓の設定と復元、hook の導入と復元、
 * 失敗の注入 (書き込みが落ちる、確認の読み戻しが違う、I2C の ACK なし、バスが空かない、機材の取り外し)、
 * ポンプの引数と要求の組の排他。実機の代用ではない (値の出どころは dt295_init_crosscheck.md)。
 *
 * build: gcc -Wall -Wextra -I driver/common/dt295 tests/dt295_core_test.c \
 *            driver/common/dt295/dt295_{link,board,frontend,program}.c -o dt295_core_test
 */
#include "dt295_core.h"
#include "dt295_program_gen.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HOOK 0xea6fcu
#define BODY 0xe1000u
#define STAGING 0xc4620u
#define BLK_BASE 0x1900000u
#define BLK_SIZE 0x20000u

static const u8 hook_original[8] = { 0xc4, 0x6d, 0x02, 0xb0, 0xa8, 0x10, 0xe3, 0x2b };
static const u8 hook_program[8] = { 0x9f, 0x8c, 0x00, 0x0e, 0x10, 0x00, 0x97, 0x0c };

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

enum { OP_READ, OP_WRITE, OP_LOCK, OP_UNLOCK };

struct op {
	u8 kind;
	u8 masked;
	u8 locked_path;	/* control_locked で呼ばれた */
	u32 target;	/* 窓の変換後のアドレス */
	u32 value;	/* 書き込みの先頭 4 バイト (ビッグエンディアン) */
	u16 len;
};

#define MAX_OPS 16384
#define I2C_EV_MAX 512

struct ev {
	u8 tuner;	/* 0 復調器、1 チューナー */
	u8 reg;
	u8 val;
};

static struct {
	u8 mem[0x1000000];
	u8 blk[BLK_SIZE];
	struct op ops[MAX_OPS];
	unsigned int nops;
	unsigned int calls;	/* control の呼び出し回数 (注入の添字) */
	unsigned int fail_at;	/* この番号の呼び出しを失敗させる (1 回だけ) */
	int fail_ret;
	int dead;		/* 機材の取り外し: すべて -ENODEV */
	int short_len;		/* 読み書きの長さを 1 つ少なく返す */
	int scribble;		/* 書き込みの転送の後に、転送のバッファを破壊する (WinUSB のように、バッファを書き換える USB 層の模擬) */
	int drop_imm_b, drop_body, drop_hook;
	unsigned int max_len;
	unsigned int violations;	/* 64 バイト超、ロック規則違反 */
	int locked;
	int lock_ret;
	unsigned int lock_calls, unlock_calls;
	u64 now;
	unsigned int error_logs;
	/* I2C */
	int busy, no_done, nack_addr, nack_data;
	int state;	/* 0 空き、1 書き、2 受信 */
	int nack;
	u8 wbytes[64];
	unsigned int wn;
	u8 ptr;
	u8 demod[256], tuner[256];
	struct ev ev[I2C_EV_MAX];
	unsigned int nev;
	u8 last_control_write;
	/* ポンプ */
	u32 pump_result;
	int pump_sent;	/* 負なら要求の個数 */
	u8 last_req[64];
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

static u8 getb(u32 t)
{
	if (t >= BLK_BASE && t < BLK_BASE + BLK_SIZE)
		return M.blk[t - BLK_BASE];
	return t < 0x1000000u ? M.mem[t] : 0;
}

static void putb(u32 t, u8 v)
{
	if (t >= BLK_BASE && t < BLK_BASE + BLK_SIZE) {
		M.blk[t - BLK_BASE] = v;
		return;
	}
	if (t >= 0x1000000u)
		return;
	if ((M.drop_imm_b && t >= 0xe9ebc && t < 0xe9ec0) || (M.drop_body && t >= BODY && t < BODY + 0x200) ||
	    (M.drop_hook && t >= HOOK && t < HOOK + 8))
		return;
	M.mem[t] = v;
}

/* boot0 のアドレス変換: 0x80000〜0x9FFFF -> (アドレス & 0x3FFFF) + 即値。読みは 0xE9EBC、書きは 0xEA3D8 の即値。 */
static u32 xlate(u32 a, int write)
{
	if (a >= 0x80000 && a < 0xa0000)
		return (a & 0x3ffff) + rd32(write ? 0xea3d8 : 0xe9ebc);
	return a;
}

static void i2c_status(void)
{
	M.mem[0xfd8fe3] = (M.busy ? 0x80 : 0) | (M.nack ? 0x10 : 0);
}

static void i2c_commit(void)
{
	unsigned int i;

	if (M.wn == 4 && M.wbytes[0] == 0xfe && M.wbytes[1] == 0xc0) {
		M.tuner[M.wbytes[2]] = M.wbytes[3];
		if (M.nev < I2C_EV_MAX)
			M.ev[M.nev++] = (struct ev){ 1, M.wbytes[2], M.wbytes[3] };
	} else if (M.wn == 1) {
		M.ptr = M.wbytes[0];
	} else if (M.wn >= 2) {
		for (i = 1; i < M.wn; ++i)
			M.demod[(u8)(M.wbytes[0] + i - 1)] = M.wbytes[i];
		if (M.nev < I2C_EV_MAX)
			M.ev[M.nev++] = (struct ev){ 0, M.wbytes[0], M.wbytes[1] };
	}
	M.wn = 0;
}

/* 制御レジスタ (0xFD8FE4) への書き込み */
static void i2c_control(u8 v)
{
	u8 data = M.mem[0xfd8ff3];

	M.last_control_write = v;
	if (!(v & 0x10)) {	/* 停止 */
		i2c_commit();
		M.state = 0;
		M.nack = 0;
		i2c_status();
		return;
	}
	if (v == 0x72 || (v == 0x52 && M.state == 0)) {	/* アドレスの送出 (開始、再スタート) */
		if (M.state == 1)
			i2c_commit();
		M.state = (data & 1) ? 2 : 1;
		M.wn = 0;
		M.nack = ((data >> 1) != 0x10) || M.nack_addr;
	} else if (M.state == 1) {
		if (M.wn < sizeof(M.wbytes))
			M.wbytes[M.wn++] = data;
		M.nack = M.nack_data;
	} else if (M.state == 2) {
		M.mem[0xfd8ff3] = M.demod[M.ptr++];
		M.nack = 0;
	}
	i2c_status();
	if (!M.no_done)
		M.mem[0xfd8fe7] |= 1;
}

static void log_op(u8 kind, u8 masked, u8 locked_path, u32 target, u32 value, u16 len)
{
	if (M.nops < MAX_OPS)
		M.ops[M.nops++] = (struct op){ kind, masked, locked_path, target, value, len };
}

static int do_control(void *priv, u8 rt, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms, int via_locked)
{
	u32 addr = ((u32)(value & 0xff00) << 8) | index;
	int masked = (value & 0xff) == 1;
	u8 *p = buf;
	unsigned int i;
	u32 t0;

	(void)priv;
	(void)timeout_ms;
	M.calls++;
	if (M.fail_at && M.calls == M.fail_at) {
		M.fail_at = 0;
		return M.fail_ret;
	}
	if (M.dead)
		return -ENODEV;
	if (len > M.max_len)
		M.max_len = len;
	if (len > 64) {
		M.violations++;
		return -EIO;
	}
	if (via_locked != M.locked)
		M.violations++;	/* lock 中は control_locked だけ、lock 外は control だけ */
	if (rt == 0xc0) {
		t0 = xlate(addr, 0);
		for (i = 0; i < len; ++i)
			p[i] = getb(xlate(addr + i, 0));
		log_op(OP_READ, 0, (u8)via_locked, t0, len >= 4 ? be32(p) : 0, len);
		return M.short_len ? len - 1 : len;
	}
	t0 = xlate(addr, 1);
	if (masked) {
		u32 v, m, old;

		if (len != 8) {
			M.violations++;
			return -EIO;
		}
		v = be32(p);
		m = be32(p + 4);
		for (old = 0, i = 0; i < 4; ++i)
			old = old << 8 | getb(xlate(addr + i, 1));
		v = (old & ~m) | (v & m);
		for (i = 0; i < 4; ++i)
			putb(xlate(addr + i, 1), (u8)(v >> (24 - 8 * i)));
		log_op(OP_WRITE, 1, (u8)via_locked, t0, be32(p), len);
	} else {
		for (i = 0; i < len; ++i)
			putb(xlate(addr + i, 1), p[i]);
		log_op(OP_WRITE, 0, (u8)via_locked, t0, len >= 4 ? be32(p) : 0, len);
	}
	if (t0 <= 0xfd8fe7 && t0 + len > 0xfd8fe4)
		i2c_control(M.mem[0xfd8fe7]);
	if (M.scribble)
		memset(p, 0xaa, len);
	/* hook から動くプログラム: 'DNT1' の要求に、結果と送った個数で答える */
	if (t0 == STAGING && len == 64 && !memcmp(M.mem + HOOK, hook_program, 8) && !memcmp(p, "DNT1", 4)) {
		memcpy(M.last_req, p, 64);
		M.pumps++;
		wr32(STAGING + 8, M.pump_result);
		wr32(STAGING + 12, M.pump_sent < 0 ? be32(p + 8) : (u32)M.pump_sent);
	}
	return M.short_len ? len - 1 : len;
}

static int op_control(void *priv, u8 rt, u16 v, u16 i, void *buf, u16 len, unsigned int to)
{
	return do_control(priv, rt, v, i, buf, len, to, 0);
}

static int op_control_locked(void *priv, u8 rt, u16 v, u16 i, void *buf, u16 len, unsigned int to)
{
	return do_control(priv, rt, v, i, buf, len, to, 1);
}

static int op_lock(void *priv)
{
	(void)priv;
	if (M.lock_ret)
		return M.lock_ret;
	M.locked = 1;
	M.lock_calls++;
	log_op(OP_LOCK, 0, 0, 0, 0, 0);
	return 0;
}

static void op_unlock(void *priv)
{
	(void)priv;
	M.locked = 0;
	M.unlock_calls++;
	log_op(OP_UNLOCK, 0, 0, 0, 0, 0);
}

static void op_msleep(void *priv, unsigned int ms)
{
	(void)priv;
	M.now += ms;
}

static u64 op_now(void *priv)
{
	(void)priv;
	return ++M.now;	/* 呼ぶたびに進む: 待ちのループが必ず期限で終わる */
}

static void op_log(void *priv, int level, const char *fmt, ...)
{
	(void)priv;
	(void)fmt;
	if (level == DT295_LOG_ERR)
		M.error_logs++;
}

static const struct dt295_ops OPS = {
	.control = op_control,
	.lock = op_lock,
	.unlock = op_unlock,
	.control_locked = op_control_locked,
	.msleep = op_msleep,
	.now_ms = op_now,
	.log = op_log,
};

static struct dt295_bridge B;
static u8 body_pattern[512];

/* 起動直後の機材 */
static void setup(void)
{
	unsigned int i;

	memset(&M, 0, sizeof(M));
	memset(M.blk, 0xaa, sizeof(M.blk));
	M.fail_ret = -EIO;
	M.pump_sent = -1;
	memcpy(M.mem + HOOK, hook_original, 8);
	for (i = 0; i < sizeof(body_pattern); ++i)
		body_pattern[i] = (u8)(0xa5 ^ i);
	memcpy(M.mem + BODY, body_pattern, sizeof(body_pattern));
	wr32(0xea3d8, 0x00f00000);
	wr32(0xe9ebc, 0x00f00000);
	wr32(0xf10118, 0x90000000);
	wr32(0xf10014, 0x00001234);
	wr32(0xf10114, 0x80000000);
	wr32(0xfd8010, 0);
	wr32(0xfd8014, 0x10);
	wr32(0xfd80b0, 0);
	i2c_status();
	dt295_bridge_init(&B, &OPS, NULL);
}

static void clear_log(void)
{
	M.nops = 0;
	M.calls = 0;
	M.error_logs = 0;
	M.nev = 0;
}

/* target への書き込みで、値が value のものを start 以降から探す。なければ -1 */
static int find_write(u32 target, u32 value, unsigned int start)
{
	unsigned int i;

	for (i = start; i < M.nops; ++i)
		if (M.ops[i].kind == OP_WRITE && M.ops[i].target == target && M.ops[i].value == value)
			return (int)i;
	return -1;
}

static int find_last_write(u32 target, u32 value)
{
	int i, found = -1;

	for (i = 0; i < (int)M.nops; ++i)
		if (M.ops[i].kind == OP_WRITE && M.ops[i].target == target && M.ops[i].value == value)
			found = i;
	return found;
}

static unsigned int count_writes(u32 lo, u32 hi)
{
	unsigned int i, n = 0;

	for (i = 0; i < M.nops; ++i)
		if (M.ops[i].kind == OP_WRITE && M.ops[i].target >= lo && M.ops[i].target < hi)
			n++;
	return n;
}

static unsigned int total_writes(void)
{
	return count_writes(0, 0xffffffffu);
}

static int first_write_index(void)
{
	unsigned int i;

	for (i = 0; i < M.nops; ++i)
		if (M.ops[i].kind == OP_WRITE)
			return (int)i;
	return -1;
}

/* ---- 試験 ---- */

static void test_link(void)
{
	u8 in[200], out[200];
	u32 v;
	unsigned int i;

	setup();
	for (i = 0; i < sizeof(in); ++i)
		in[i] = (u8)(i * 7 + 3);
	CHECK(dt295_write(&B, 0x100000, in, 130) == 0);
	CHECK(M.max_len <= 64);
	CHECK(M.nops == 3);	/* 64 + 64 + 2 */
	CHECK(!memcmp(M.mem + 0x100000, in, 130));
	CHECK(dt295_read(&B, 0x100000, out, 130) == 0);
	CHECK(!memcmp(out, in, 130));
	CHECK(M.max_len == 64 && M.violations == 0);

	/* アドレスの符号化 (0x123456 -> wValue 0x1200、index 0x3456) は、往復で確認する */
	M.mem[0x123456] = 0x5a;
	CHECK(dt295_read(&B, 0x123456, out, 1) == 0 && out[0] == 0x5a);

	/*
	 * 書き込みの転送が、渡されたバッファを書き換えても、呼び出し側のデータは変わらない
	 * (WinUSB は書き込み可能なバッファを要求する。読み取り専用のデータ (機械語) を直接渡すと、エラー 998 で失敗する)
	 */
	{
		static const u8 readonly[130] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
		u8 copy[130];

		memcpy(copy, readonly, sizeof(copy));
		M.scribble = 1;
		CHECK(dt295_write(&B, 0x140000, copy, 130) == 0);
		M.scribble = 0;
		CHECK(!memcmp(copy, readonly, sizeof(copy)));	/* 呼び出し側のバッファは無事 */
		CHECK(!memcmp(M.mem + 0x140000, readonly, sizeof(readonly)));	/* 機材には正しいデータが届く */
	}
	CHECK(dt295_read(&B, 0x1000000, out, 1) == -EINVAL);
	CHECK(dt295_read(&B, 0xffffff, out, 2) == -EINVAL);
	CHECK(dt295_read(&B, 0xffffff, out, 1) == 0);
	CHECK(dt295_read(&B, 0x100000, out, 0) == -EINVAL);
	CHECK(dt295_write(&B, 0x100000, NULL, 4) == -EINVAL);

	/* 失敗の伝播と、長さの不一致 */
	M.short_len = 1;
	CHECK(dt295_read(&B, 0x100000, out, 4) == -EIO);
	CHECK(dt295_write(&B, 0x100000, in, 4) == -EIO);
	M.short_len = 0;
	M.dead = 1;
	CHECK(dt295_read(&B, 0x100000, out, 4) == -ENODEV);
	M.dead = 0;
	M.fail_at = M.calls + 2;
	CHECK(dt295_read(&B, 0x100000, out, 130) == -EIO);	/* 2 回目の分割で失敗 */

	/* マスク付き書き込み: *a = (*a & ~mask) | (value & mask)、wValue 下位 1 */
	wr32(0x200000, 0x12345678);
	clear_log();
	CHECK(dt295_mwrite32(&B, 0x200000, 0xaabbccdd, 0x00ff00ff) == 0);
	CHECK(rd32(0x200000) == 0x12bb56dd);
	CHECK(M.nops == 1 && M.ops[0].masked && M.ops[0].len == 8);
	CHECK(dt295_mwrite32(&B, 0x200002, 0, 0xffff) == -EINVAL);
	CHECK(dt295_write16_hi(&B, 0x200000, 0xbeef) == 0 && rd32(0x200000) == 0xbeef56dd);
	CHECK(dt295_read32(&B, 0x200000, &v) == 0 && v == 0xbeef56dd);
}

static void test_window(void)
{
	u32 v;

	setup();
	wr32(0xf00000, 0xdeadbeef);
	/* 窓の前: 0x80000 は既定の即値 0xF00000 へ届く */
	CHECK(dt295_read32(&B, 0x80000, &v) == 0 && v == 0xdeadbeef);
	CHECK(dt295_blk_read(&B, 0x1900000, &v) == -EINVAL);	/* 窓の設定前 */
	CHECK(dt295_blk_write(&B, 0x1900004, 1) == -EINVAL);

	clear_log();
	CHECK(dt295_window_set(&B, BLK_BASE) == 0);
	CHECK(B.window_set);
	CHECK(rd32(0xea3d8) == BLK_BASE && rd32(0xe9ebc) == BLK_BASE);
	/* 2 か所に書いて、2 か所を読み戻す (どちらも書き換えないと読み出しの経路に効かない) */
	CHECK(count_writes(0xea3d8, 0xea3dc) == 1 && count_writes(0xe9ebc, 0xe9ec0) == 1);

	memset(M.blk + 4, 0, 4);
	CHECK(dt295_blk_write(&B, 0x1900004, 0x11223344) == 0);
	CHECK(be32(M.blk + 4) == 0x11223344);
	CHECK(dt295_blk_read(&B, 0x1900004, &v) == 0 && v == 0x11223344);
	CHECK(dt295_blk_mask(&B, 0x1900004, 0xff000000, 0xff000000) == 0 && be32(M.blk + 4) == 0xff223344);
	CHECK(dt295_blk_write(&B, 0x1900005, 0) == -EINVAL);	/* 4 の倍数でない */
	CHECK(dt295_blk_write(&B, 0x1920000, 0) == -EINVAL);	/* ブロックの外 */
	CHECK(dt295_blk_read(&B, 0x18fffffc, &v) == -EINVAL);
	CHECK(dt295_blk_read(&B, 0x1900000 + 0x1fffc, &v) == 0);

	CHECK(dt295_window_restore(&B) == 0);
	CHECK(!B.window_set && rd32(0xea3d8) == 0x00f00000 && rd32(0xe9ebc) == 0x00f00000);
	CHECK(dt295_blk_read(&B, 0x1900004, &v) == -EINVAL);

	/* 読み戻しの不一致 (片方の即値が書けない) */
	M.drop_imm_b = 1;
	CHECK(dt295_window_set(&B, BLK_BASE) == -EIO);
	CHECK(!B.window_set);
	M.drop_imm_b = 0;
	/* 失敗の伝播 */
	M.fail_at = M.calls + 1;
	CHECK(dt295_window_set(&B, BLK_BASE) == -EIO && !B.window_set);
}

static void test_board_open(void)
{
	int first_w;

	setup();
	clear_log();
	CHECK(dt295_board_open(&B) == 0);
	/* 状態と hook の読みが、最初の書き込みより前 */
	first_w = first_write_index();
	CHECK(first_w >= 2 && M.ops[0].kind == OP_READ && M.ops[1].kind == OP_READ);
	CHECK(M.ops[0].target == 0xf02008 && M.ops[1].target == HOOK);
	CHECK(M.ops[first_w].target == 0xf10118);	/* 最初はクロックのゲート */
	CHECK((rd32(0xf10118) >> 16) == 0xffff);
	CHECK((rd32(0xf10014) >> 16) == 0xd082 && (rd32(0xf10014) & 0xffff) == 0x1234);	/* 下位半語は保たれる */
	CHECK((rd32(0xf10018) >> 16) == 0x0002);
	CHECK(rd32(0xf10114) == 0x80200000);
	CHECK((rd32(0xfd8010) & 0x10) == 0x10 && (rd32(0xfd8014) & 0x10) == 0);
	CHECK((rd32(0xfd80b0) & 1) == 1 && rd32(0xfd80c0) == 0);
	CHECK(rd32(0xfd8ff8) == 6 && rd32(0xfd8ff4) == 0x10 && rd32(0xfd8fe8) == 0x3c && rd32(0xfd8fe4) == 0x42);
	CHECK(M.ops[M.nops - 1].target == 0xfd8fe4);	/* 最後は I2C の制御 */
	CHECK(count_writes(BODY, BODY + 0x200) == 0 && count_writes(HOOK, HOOK + 8) == 0);
	CHECK(M.max_len <= 64 && M.violations == 0 && M.error_logs == 0);

	/* 起動直後でない機材 (状態 0x82008 が 0 でない、hook が元でない) は、1 つも書かずに -EBUSY */
	setup();
	M.mem[0xf02008] = 1;	/* 0x82008 は窓 (0x80000〜) を通り、即値 0xF00000 で 0xF02008 へ届く */
	clear_log();
	CHECK(dt295_board_open(&B) == -EBUSY && total_writes() == 0 && M.error_logs == 1);
	setup();
	M.mem[0xf02009] = 2;
	clear_log();
	CHECK(dt295_board_open(&B) == -EBUSY && total_writes() == 0);
	setup();
	memcpy(M.mem + HOOK, hook_program, 8);
	clear_log();
	CHECK(dt295_board_open(&B) == -EBUSY && total_writes() == 0);
	setup();
	M.mem[HOOK + 7] ^= 1;	/* 1 ビットだけ違う */
	clear_log();
	CHECK(dt295_board_open(&B) == -EBUSY && total_writes() == 0);

	/* 取り外し、途中の失敗 */
	setup();
	M.dead = 1;
	CHECK(dt295_board_open(&B) == -ENODEV);
	setup();
	M.fail_at = 5;	/* 読み 2 回の後の、チップ側の書き込みの途中 */
	CHECK(dt295_board_open(&B) == -EIO);
	CHECK(rd32(0xfd8fe4) == 0);	/* I2C の準備まで進んでいない */
}

static void i2c_ready(void)
{
	setup();
	CHECK(dt295_board_open(&B) == 0);
	clear_log();
}

static void test_i2c(void)
{
	u8 buf[8], w[2], big[41];

	i2c_ready();
	M.demod[0x80] = 0x5c;
	buf[0] = 0x80;
	CHECK(dt295_i2c_transfer(&B, 0x10, buf, 1, buf + 1, 1) == 0 && buf[1] == 0x5c);
	CHECK(M.last_control_write == 0x42);	/* 最後は停止 */
	M.demod[0x8b] = 1;
	M.demod[0x8c] = 2;
	M.demod[0x8d] = 3;
	buf[0] = 0x8b;
	CHECK(dt295_i2c_transfer(&B, 0x10, buf, 1, buf + 1, 3) == 0 && buf[1] == 1 && buf[2] == 2 && buf[3] == 3);
	w[0] = 0x0e;
	w[1] = 0x71;
	CHECK(dt295_i2c_transfer(&B, 0x10, w, 2, NULL, 0) == 0 && M.demod[0x0e] == 0x71);
	CHECK(M.error_logs == 0 && M.violations == 0 && M.max_len <= 64);

	/* 引数 */
	CHECK(dt295_i2c_transfer(&B, 0x10, NULL, 0, NULL, 0) == -EINVAL);
	CHECK(dt295_i2c_transfer(&B, 0x10, NULL, 1, buf, 1) == -EINVAL);
	CHECK(dt295_i2c_transfer(&B, 0x10, buf, 1, NULL, 1) == -EINVAL);
	memset(big, 0, sizeof(big));
	CHECK(dt295_i2c_transfer(&B, 0x10, big, 41, NULL, 0) == -EINVAL);
	CHECK(dt295_i2c_transfer(&B, 0x10, big, 20, big + 20, 21) == -EINVAL);
	CHECK(dt295_i2c_transfer(&B, 0x10, big, 20, big + 20, 20) == 0);	/* 合計 40 は通る */

	/* アドレスの ACK なし: -ENXIO、停止は送る、ログが出る */
	clear_log();
	buf[0] = 0x80;
	CHECK(dt295_i2c_transfer(&B, 0x11, buf, 1, buf + 1, 1) == -ENXIO);
	CHECK(M.last_control_write == 0x42 && M.error_logs == 1);
	/* 失敗の後も、次の転送は成立する (模擬では詰まらない) */
	CHECK(dt295_i2c_transfer(&B, 0x10, buf, 1, buf + 1, 1) == 0);

	/* データの ACK なし: -EIO */
	M.nack_data = 1;
	CHECK(dt295_i2c_transfer(&B, 0x10, w, 2, NULL, 0) == -EIO);
	M.nack_data = 0;

	/* バスが空かない: 期限で -ETIMEDOUT。操作の開始前なので何も送らない */
	clear_log();
	M.busy = 1;
	i2c_status();
	CHECK(dt295_i2c_transfer(&B, 0x10, w, 2, NULL, 0) == -ETIMEDOUT);
	CHECK(total_writes() == 0);
	M.busy = 0;
	i2c_status();

	/* 1 バイトの終了が立たない: 期限で -ETIMEDOUT、停止は送る */
	clear_log();
	M.no_done = 1;
	CHECK(dt295_i2c_transfer(&B, 0x10, w, 2, NULL, 0) == -ETIMEDOUT);
	CHECK(M.last_control_write == 0x42 && M.error_logs == 1);
	M.no_done = 0;

	/* 機材の取り外し */
	M.dead = 1;
	CHECK(dt295_i2c_transfer(&B, 0x10, w, 2, NULL, 0) == -ENODEV);
}

static void test_frontend(void)
{
	bool locked = true;
	u32 raw = 0;

	i2c_ready();
	CHECK(dt295_fe_init(&B) == 0);
	CHECK(M.nev == 5);
	CHECK(M.ev[0].tuner == 0 && M.ev[0].reg == 0x0e && M.ev[0].val == 0x71);
	CHECK(M.ev[1].tuner == 0 && M.ev[1].reg == 0x1d && M.ev[1].val == 0x02);
	CHECK(M.ev[2].tuner == 1 && M.ev[2].reg == 0x03 && M.ev[2].val == 0x01);
	CHECK(M.ev[3].tuner == 1 && M.ev[3].reg == 0x05 && M.ev[3].val == 0x1b);
	CHECK(M.ev[4].tuner == 1 && M.ev[4].reg == 0x08 && M.ev[4].val == 0x05);	/* C/N を上げる設定 */

	/* UHF 25ch (545143 kHz): 545143 * 64 / 1000 = 34889 = 0x8849、700 MHz 未満 */
	clear_log();
	CHECK(dt295_fe_tune(&B, 545143) == 0);
	CHECK(M.nev == 8);
	CHECK(M.ev[0].tuner == 0 && M.ev[0].reg == 0x01 && M.ev[0].val == 0x40);	/* 最初に復調器の同期の補助 */
	CHECK(M.ev[1].reg == 0x12 && M.ev[1].val == 0);
	CHECK(M.ev[2].reg == 0x0b && M.ev[2].val == 1);
	CHECK(M.ev[3].reg == 0x10 && M.ev[3].val == 0x49);
	CHECK(M.ev[4].reg == 0x11 && M.ev[4].val == 0x88);
	CHECK(M.ev[5].reg == 0x31 && M.ev[5].val == 0x81);
	CHECK(M.ev[6].reg == 0x7c && M.ev[6].val == 0x1f);
	CHECK(M.ev[7].reg == 0x12 && M.ev[7].val == 1);	/* 最後に調整の開始 */

	/* 700 MHz 以上は 0x91 / 0x9f。境界: 699999 は下、700000 は上 */
	clear_log();
	CHECK(dt295_fe_tune(&B, 725143) == 0);
	CHECK(M.ev[5].val == 0x91 && M.ev[6].val == 0x9f);
	clear_log();
	CHECK(dt295_fe_tune(&B, 699999) == 0 && M.ev[5].val == 0x81);
	clear_log();
	CHECK(dt295_fe_tune(&B, 700000) == 0 && M.ev[5].val == 0x91);

	/* 範囲 (90000〜770000 kHz): 外は何も送らない */
	clear_log();
	CHECK(dt295_fe_tune(&B, 89999) == -EINVAL);
	CHECK(dt295_fe_tune(&B, 770001) == -EINVAL);
	CHECK(dt295_fe_tune(&B, 0) == -EINVAL);
	CHECK(M.nops == 0 && M.nev == 0);
	CHECK(dt295_fe_tune(&B, 90000) == 0);
	CHECK(dt295_fe_tune(&B, 770000) == 0);

	/* 途中の失敗は、残りを送らずに返す */
	clear_log();
	M.nack_data = 1;
	CHECK(dt295_fe_tune(&B, 545143) == -EIO);
	CHECK(M.nev == 0);
	M.nack_data = 0;

	/* ロック判定: 0x80 の bit5 と bit3 が 0、0xB0 の下位 4 ビットが 8 以上 */
	M.demod[0x80] = 0x00;
	M.demod[0xb0] = 0x08;
	CHECK(dt295_fe_locked(&B, &locked) == 0 && locked);
	M.demod[0xb0] = 0xf8;
	CHECK(dt295_fe_locked(&B, &locked) == 0 && locked);
	M.demod[0xb0] = 0x07;
	CHECK(dt295_fe_locked(&B, &locked) == 0 && !locked);
	M.demod[0xb0] = 0x0f;
	M.demod[0x80] = 0x08;
	CHECK(dt295_fe_locked(&B, &locked) == 0 && !locked);
	M.demod[0x80] = 0x20;
	CHECK(dt295_fe_locked(&B, &locked) == 0 && !locked);
	M.demod[0x80] = 0x47;	/* 他のビットは無関係 */
	CHECK(dt295_fe_locked(&B, &locked) == 0 && locked);
	M.dead = 1;
	locked = true;
	CHECK(dt295_fe_locked(&B, &locked) == -ENODEV && !locked);
	M.dead = 0;

	M.demod[0x8b] = 0x01;
	M.demod[0x8c] = 0x02;
	M.demod[0x8d] = 0x03;
	CHECK(dt295_fe_cnr_raw(&B, &raw) == 0 && raw == 0x010203);
	M.dead = 1;
	CHECK(dt295_fe_cnr_raw(&B, &raw) == -ENODEV);
}

static void test_capture(void)
{
	u32 v, wp;
	int i_stop, i_first, i_in, i_ring, i_start, i_a, i_b, first_blk = -1;
	unsigned int i, dry_calls;

	i2c_ready();
	CHECK(dt295_cap_start(&B) == 0);
	CHECK(B.window_set && rd32(0xea3d8) == BLK_BASE && rd32(0xe9ebc) == BLK_BASE);
	for (i = 0; i < M.nops; ++i)
		if (M.ops[i].kind == OP_WRITE && M.ops[i].target >= BLK_BASE && M.ops[i].target < BLK_BASE + BLK_SIZE) {
			first_blk = (int)i;
			break;
		}
	/* 窓の設定の後、最初に取り込みブロックへ書くのは、前回の残りを止める書き込み位置の停止 */
	CHECK(first_blk > 0 && M.ops[first_blk].target == 0x1902010 && M.ops[first_blk].value == 0x04000000);
	i_stop = find_write(0x1902010, 0x04000000, 0);
	i_first = find_write(0x19000c4, 0, 0);
	i_a = find_write(0x1900000, 0x3002ffff, 0);
	i_b = find_write(0x1900000, 0x00020000, 0);
	i_ring = find_write(0x1902000, 0x03020300, 0);
	i_in = find_write(0x1901040, 0x03000000, 0);
	/* 0x1900020 は、ホストから書いても保持されず、無くても受信できるので書かない (dt295_reg_meaning.md) */
	CHECK(count_writes(0x1900020, 0x1900024) == 0);
	/* cap_start はライタを開始しない (ポンプが動いてから dt295_cap_run で開始する) */
	CHECK(find_write(0x1902010, 0x02000000, 0) < 0);
	CHECK(M.ops[M.nops - 1].target == 0x1900488 && M.ops[M.nops - 1].value == 0x206c0000);
	CHECK(dt295_cap_run(&B) == 0);
	i_start = find_last_write(0x1902010, 0x02000000);
	CHECK(i_stop >= 0 && i_first > i_stop && i_a > i_first && i_b > i_a);
	CHECK(i_ring > i_b && i_in > i_ring && i_start > i_in);
	CHECK((int)(M.nops - 1) == i_start);	/* 最後はライタの開始 */
	CHECK(M.ops[M.nops - 1].target == 0x1902010 && M.ops[M.nops - 1].value == 0x02000000);
	/* 値 */
	CHECK(be32(M.blk + 0x1044) == 0x00bc0000 && be32(M.blk + 0x1048) == 0x47000000 && be32(M.blk + 0x104c) == 0x00ffffff);
	CHECK(be32(M.blk + 0x2020) == 0x0138ec00 && be32(M.blk + 0x2024) == 0x30000);
	CHECK(be32(M.blk + 0x2028) == 0 && be32(M.blk + 0x202c) == 0);	/* 位置は 0 から */
	CHECK(be32(M.blk + 0x1040) == 0x03000000 && be32(M.blk + 0x10a4) == 0x13170400);
	CHECK(be32(M.blk + 0x1000) == 0x06070000);	/* マスク付きで、下位は 0 へ、上位は後半の値へ */
	CHECK(be32(M.blk + 0x484) == 0x01000000 && be32(M.blk + 0x488) == 0x206c0000);
	CHECK(M.violations == 0 && M.max_len <= 64 && M.error_logs == 0);
	dry_calls = M.calls;

	/* 書き込み位置と読み出し位置 */
	put32(M.blk + 0x2028, 0x1c0);
	CHECK(dt295_cap_wptr(&B, &wp) == 0 && wp == 0x1c0);
	CHECK(dt295_cap_set_rptr(&B, 0x180) == 0 && be32(M.blk + 0x202c) == 0x180);

	/* 停止 */
	clear_log();
	CHECK(dt295_cap_stop(&B) == 0);
	CHECK(M.nops == 3 && M.ops[0].target == 0x1902010 && M.ops[0].value == 0x04000000);
	CHECK(M.ops[1].target == 0x1901040 && M.ops[1].value == 0 && M.ops[2].target == 0x1901020 && M.ops[2].value == 0x02000000);
	CHECK(dt295_window_restore(&B) == 0);
	CHECK(dt295_cap_stop(&B) == -EINVAL);	/* 窓が向いていない */
	CHECK(dt295_cap_wptr(&B, &v) == -EINVAL && dt295_cap_set_rptr(&B, 0) == -EINVAL);

	/* 窓の設定で失敗: 取り込みブロックへは書かない */
	i2c_ready();
	M.fail_at = 1;
	CHECK(dt295_cap_start(&B) == -EIO && !B.window_set);
	CHECK(count_writes(BLK_BASE, BLK_BASE + BLK_SIZE) == 0);

	/* 設定の途中で失敗: 止めて窓を戻し、元のエラーを返す */
	i2c_ready();
	M.fail_at = dry_calls - 4;	/* 終わり近くの書き込み */
	CHECK(dt295_cap_start(&B) == -EIO);
	CHECK(!B.window_set && rd32(0xea3d8) == 0x00f00000 && rd32(0xe9ebc) == 0x00f00000);
	CHECK(M.error_logs >= 1);
	i_stop = find_write(0x1902010, 0x04000000, 0);
	CHECK(i_stop >= 0 && find_write(0x1902010, 0x04000000, (unsigned int)i_stop + 1) > i_stop);	/* 失敗の後に再度止めている */
	CHECK(find_last_write(0x1902010, 0x02000000) < 0 || find_last_write(0x1902010, 0x02000000) < find_last_write(0x1902010, 0x04000000));
}

static void prog_ready(void)
{
	setup();
	clear_log();
}

static void test_program(void)
{
	u8 back[512];
	unsigned int i;
	int first_w, body_w, hook_w, hook_rm, body_rm;

	/* 導入 */
	prog_ready();
	CHECK(dt295_prog_install(&B) == 0 && B.program_installed);
	CHECK(!memcmp(M.mem + BODY, dt295_program, sizeof(dt295_program)));
	CHECK(!memcmp(M.mem + BODY + sizeof(dt295_program), body_pattern + sizeof(dt295_program), 512 - sizeof(dt295_program)));
	CHECK(!memcmp(M.mem + HOOK, hook_program, 8));
	CHECK(!memcmp(B.saved_body, body_pattern, 512));
	first_w = first_write_index();
	CHECK(first_w >= 2 && M.ops[0].kind == OP_READ);	/* 保存と hook の確認が、最初の書き込みより前 */
	body_w = find_write(BODY, be32(dt295_program), 0);
	hook_w = find_write(HOOK, be32(hook_program), 0);
	CHECK(body_w >= 0 && hook_w > body_w);	/* 本体が先、hook が最後 */
	CHECK((unsigned int)hook_w > M.nops - 3);	/* hook の後は読み戻しだけ */
	CHECK(M.max_len <= 64 && M.violations == 0 && M.error_logs == 0);
	/* 二重の導入は、何もしない */
	clear_log();
	CHECK(dt295_prog_install(&B) == -EALREADY && M.nops == 0);

	/* 復元: hook が先、本体が後 */
	clear_log();
	CHECK(dt295_prog_remove(&B) == 0 && !B.program_installed);
	CHECK(!memcmp(M.mem + HOOK, hook_original, 8) && !memcmp(M.mem + BODY, body_pattern, 512));
	hook_rm = find_write(HOOK, be32(hook_original), 0);
	body_rm = find_write(BODY, be32(body_pattern), 0);
	CHECK(hook_rm >= 0 && body_rm > hook_rm);
	CHECK(M.error_logs == 0);

	/* 他のプログラムが hook を使っている (元の値でない): 導入しない */
	prog_ready();
	M.mem[HOOK] ^= 0x10;
	CHECK(dt295_prog_install(&B) == -EBUSY && !B.program_installed && total_writes() == 0);

	/* 本体の読み戻しが違う: hook は書かない */
	prog_ready();
	M.drop_body = 1;
	CHECK(dt295_prog_install(&B) == -EIO && !B.program_installed);
	CHECK(count_writes(HOOK, HOOK + 8) == 0 && !memcmp(M.mem + HOOK, hook_original, 8));
	M.drop_body = 0;

	/* hook の読み戻しが違う: hook を元へ戻す試み、本体を戻す */
	prog_ready();
	M.drop_hook = 1;
	CHECK(dt295_prog_install(&B) == -EIO && !B.program_installed);
	CHECK(count_writes(HOOK, HOOK + 8) >= 2);	/* 導入の書き込みと、元へ戻す書き込み */
	CHECK(!memcmp(M.mem + BODY, body_pattern, 512));
	M.drop_hook = 0;

	/* 保存の読みで失敗: 何も書かない */
	prog_ready();
	M.fail_at = 1;
	CHECK(dt295_prog_install(&B) == -EIO && total_writes() == 0);

	/* 本体の書き込みで失敗 (最初の書き込み = 呼び出し 3 番): hook は書かない、本体の領域は元に戻す */
	prog_ready();
	M.fail_at = 3;
	CHECK(dt295_prog_install(&B) == -EIO && count_writes(HOOK, HOOK + 8) == 0 && !memcmp(M.mem + BODY, body_pattern, 512));

	/* 復元の失敗: hook が戻らない -> -EIO、ログ、本体には触れない、導入済みのまま */
	prog_ready();
	CHECK(dt295_prog_install(&B) == 0);
	clear_log();
	M.drop_hook = 1;
	CHECK(dt295_prog_remove(&B) == -EIO && B.program_installed && M.error_logs == 1);
	CHECK(count_writes(BODY, BODY + 0x200) == 0);
	M.drop_hook = 0;
	/* 機材の取り外し: -ENODEV、ログは出さない (戻せないのが当然) */
	clear_log();
	M.dead = 1;
	CHECK(dt295_prog_remove(&B) == -ENODEV && M.error_logs == 0);
	M.dead = 0;
	CHECK(dt295_prog_remove(&B) == 0 && !memcmp(M.mem + HOOK, hook_original, 8));
	/* 導入していないときの復元は、何も書かない (空の保存で本体の領域を潰さない) */
	clear_log();
	CHECK(dt295_prog_remove(&B) == 0 && M.nops == 0 && !memcmp(M.mem + BODY, body_pattern, 512));
	(void)back;
	(void)i;
}

static void test_pump(void)
{
	u8 req[64];
	unsigned int n;

	prog_ready();
	CHECK(dt295_prog_pump(&B, 0, 16) == -EINVAL);	/* 導入前 */
	CHECK(dt295_prog_install(&B) == 0);
	clear_log();
	CHECK(dt295_prog_pump(&B, 0, 0) == -EINVAL);
	CHECK(dt295_prog_pump(&B, 0, 17) == -EINVAL);
	CHECK(dt295_prog_pump(&B, 0, 144) == -EINVAL);
	CHECK(dt295_prog_pump(&B, 0x30000, 16) == -EINVAL);
	CHECK(dt295_prog_pump(&B, 2, 16) == -EINVAL);
	CHECK(M.nops == 0 && M.lock_calls == 0);

	/* 正常: lock -> 要求の書き -> 応答の読み -> unlock の 4 つ組。他の制御転送が間に入らない */
	CHECK(dt295_prog_pump(&B, 0x1000, 128) == 0);
	CHECK(M.nops == 4);
	CHECK(M.ops[0].kind == OP_LOCK && M.ops[1].kind == OP_WRITE && M.ops[2].kind == OP_READ && M.ops[3].kind == OP_UNLOCK);
	CHECK(M.ops[1].locked_path && M.ops[2].locked_path);
	CHECK(M.ops[1].target == STAGING && M.ops[1].len == 64 && M.ops[2].target == STAGING && M.ops[2].len == 64);
	memcpy(req, M.last_req, 64);
	CHECK(!memcmp(req, "DNT1", 4) && be32(req + 4) == 0x11 && be32(req + 8) == 128);
	CHECK(be32(req + 12) == 5000000 && be32(req + 16) == 0x8138ec00u + 0x1000);
	CHECK(M.violations == 0 && M.pumps == 1 && M.lock_calls == 1 && M.unlock_calls == 1);
	CHECK(dt295_prog_pump(&B, 0x2ffc0, 16) == 0);	/* リングの末尾付近 (折り返しは機材内プログラムが行う) */
	CHECK(be32(M.last_req + 16) == 0x8138ec00u + 0x2ffc0);

	/* 結果 */
	M.pump_result = 1;
	n = M.error_logs;
	CHECK(dt295_prog_pump(&B, 0, 16) == -ETIMEDOUT && M.error_logs == n + 1);
	M.pump_result = 2;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EIO);
	M.pump_result = 0;
	M.pump_sent = 15;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EIO);
	M.pump_sent = -1;
	CHECK(M.violations == 0 && M.locked == 0 && M.lock_calls == M.unlock_calls);

	/* 制御転送の失敗でも unlock する。要求の書きで失敗したら、応答は読まない */
	clear_log();
	M.fail_at = M.calls + 1;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EIO);
	CHECK(M.locked == 0 && M.lock_calls == M.unlock_calls && M.nops == 2);	/* lock、unlock のみ (失敗した転送は記録されない) */
	/* 応答の読みで失敗 */
	clear_log();
	M.fail_at = M.calls + 2;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EIO && M.locked == 0 && M.lock_calls == M.unlock_calls);
	/* 長さの不一致 */
	M.short_len = 1;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EIO && M.locked == 0 && M.lock_calls == M.unlock_calls);
	M.short_len = 0;
	/* ロックが取れない (割り込み): 何も送らず、unlock もしない */
	clear_log();
	M.lock_ret = -EINTR;
	n = M.unlock_calls;
	CHECK(dt295_prog_pump(&B, 0, 16) == -EINTR && M.nops == 0 && M.unlock_calls == n);
	M.lock_ret = 0;
	/* 機材の取り外し */
	M.dead = 1;
	CHECK(dt295_prog_pump(&B, 0, 16) == -ENODEV && M.locked == 0);
}

int main(void)
{
	test_link();
	test_window();
	test_board_open();
	test_i2c();
	test_frontend();
	test_capture();
	test_program();
	test_pump();
	printf("dt295_core_test: %d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
