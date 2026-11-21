// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295: 起動直後の確認、チップ側の設定、I2C コントローラ、取り込みブロック。
 *
 * 取り込みブロックの設定は、px4_drv の torne の stream_setup() (driver/common/torne/torne_bridge.c) と同じ配置と値を
 * 大半で使う (DT295 は同じ TS 取り込みの回路で、ベース 0x1900000)。リセット解除の列の値と、チップ側の
 * 0xF10014 / 0xF10018 / 0xF10114 は、別の実装の動作の外からの観測で得た値。意味は実機の試験で絞り込んだ (dt295_reg_meaning.md):
 * 0xF10014 / 0xF10018 の bit1 は汎用ポートの出力許可と出力値 (復調器の許可とみられる)、0xF10114 の bit5 は I2C の動作のクロックの許可。
 * 観測された書き込みのうち 0x1900020 は、ホストから書いても保持されず、無くても受信できるので、書かない。
 */
#include "dt295_core.h"

#define BOOT_STATE 0x82008u
#define HOOK_ADDRESS 0xea6fcu

#define CHIP_CLOCK_GATES 0xf10118u
#define CHIP_PIN_A 0xf10014u
#define CHIP_PIN_B 0xf10018u
#define CHIP_UNK114 0xf10114u

/* I2C コントローラ (4 バイトの語の最下位バイトが値) */
#define I2C_ENABLE 0xfd8010u
#define I2C_RESET 0xfd8014u
#define I2C_UNK0B0 0xfd80b0u
#define I2C_UNK0C0 0xfd80c0u
#define I2C_STATUS 0xfd8fe0u	/* bit7 バス使用中、bit4 最後の ACK なし */
#define I2C_CONTROL 0xfd8fe4u	/* bit5 再スタート、bit4 開始、bit3 受信時 ACK、bit1 割り込み許可、bit0 1 バイト終了 */
#define I2C_CLOCK 0xfd8fe8u
#define I2C_DATA 0xfd8ff0u
#define I2C_DIVIDER 0xfd8ff4u
#define I2C_UNK18 0xfd8ff8u

#define I2C_TIMEOUT_MS 200U
#define I2C_MAX_TRANSFER 40U

static const u8 original_hook[8] = { 0xc4, 0x6d, 0x02, 0xb0, 0xa8, 0x10, 0xe3, 0x2b };

int dt295_board_open(struct dt295_bridge *b)
{
	u8 state[4], hook[8];
	int ret;

	ret = dt295_read(b, BOOT_STATE, state, sizeof(state));
	if (!ret)
		ret = dt295_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (ret)
		return ret;
	/* 起動直後の機材だけを対象にする。温まった機材 (他のプログラムが hook を使った後) は、挿し直しが要る。 */
	if (state[0] || state[1] || memcmp(hook, original_hook, sizeof(hook))) {
		b->ops->log(b->priv, DT295_LOG_ERR,
			    "DT295 is not in the fresh boot0 state (state=%02x%02x, hook=%02x%02x%02x%02x%02x%02x%02x%02x); replug the device",
			    state[0], state[1], hook[0], hook[1], hook[2], hook[3], hook[4], hook[5], hook[6], hook[7]);
		return -EBUSY;
	}

	/* チップ側の設定。ホストの標準のマスク付き書き込みで行う (機材内のプログラムからは書かない)。 */
	ret = dt295_write16_hi(b, CHIP_CLOCK_GATES, 0xffff);	/* 起動直後 0x9000。bit4 が取り込みブロックのクロック */
	if (!ret)
		ret = dt295_write16_hi(b, CHIP_PIN_A, 0xd082);
	if (!ret)
		ret = dt295_write16_hi(b, CHIP_PIN_B, 0x0002);
	if (!ret) {
		/* 起動直後 0x8000。この語は読まずに書く (他の初期化でも読み戻していない) */
		u8 data[4] = { 0x80, 0x20, 0x00, 0x00 };

		ret = dt295_write(b, CHIP_UNK114, data, sizeof(data));
	}
	if (ret)
		return ret;

	/* I2C コントローラの準備 */
	ret = dt295_mwrite32(b, I2C_ENABLE, 0x10, 0x10);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_RESET, 0, 0x10);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_UNK0B0, 1, 1);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_UNK0C0, 0, 0xffffffffu);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_UNK18, 6, 0xffffffffu);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_DIVIDER, 0x10, 0xffffffffu);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_CLOCK, 0x3c, 0xffffffffu);
	if (!ret)
		ret = dt295_mwrite32(b, I2C_CONTROL, 0x42, 0xffffffffu);
	return ret;
}

static int i2c_reg8(struct dt295_bridge *b, u32 address, u32 *value)
{
	int ret = dt295_read32(b, address, value);

	if (!ret)
		*value &= 0xff;
	return ret;
}

static int i2c_wr(struct dt295_bridge *b, u32 address, u32 value)
{
	return dt295_mwrite32(b, address, value, 0xffffffffu);
}

/* 制御の bit0 (1 バイト終了) が立つまで待つ。deadline は操作全体の絶対期限。 */
static int i2c_wait_done(struct dt295_bridge *b, u64 deadline)
{
	u32 control;
	int ret;

	for (;;) {
		ret = i2c_reg8(b, I2C_CONTROL, &control);
		if (ret)
			return ret;
		if (control & 1)
			return 0;
		if (b->ops->now_ms(b->priv) > deadline)
			return -ETIMEDOUT;
	}
}

static int i2c_wait_idle(struct dt295_bridge *b, u64 deadline)
{
	u32 status;
	int ret;

	for (;;) {
		ret = i2c_reg8(b, I2C_STATUS, &status);
		if (ret)
			return ret;
		if (!(status & 0x80))
			return 0;
		if (b->ops->now_ms(b->priv) > deadline)
			return -ETIMEDOUT;
	}
}

/* 書きの 1 バイト、または読みの設定の後の、ACK の確認 (状態の bit4 が 1 なら、ACK なし)。 */
static int i2c_check_ack(struct dt295_bridge *b, int nack_error)
{
	u32 status;
	int ret = i2c_reg8(b, I2C_STATUS, &status);

	if (ret)
		return ret;
	return (status & 0x10) ? nack_error : 0;
}

int dt295_i2c_transfer(struct dt295_bridge *b, u8 address, const u8 *wbuf, unsigned int wlen, u8 *rbuf, unsigned int rlen)
{
	u64 deadline;
	unsigned int i;
	int ret = 0;

	if (!b || (wlen && !wbuf) || (rlen && !rbuf) || !(wlen + rlen) || wlen + rlen > I2C_MAX_TRANSFER)
		return -EINVAL;
	deadline = b->ops->now_ms(b->priv) + I2C_TIMEOUT_MS;
	ret = i2c_wait_idle(b, deadline);
	if (ret)
		return ret;	/* 取り外し (-ENODEV) などを、期限切れに変えない */
	if (wlen) {
		ret = i2c_wr(b, I2C_DATA, (u32)address << 1);
		if (!ret)
			ret = i2c_wr(b, I2C_CONTROL, 0x52);	/* 開始 */
		if (!ret)
			ret = i2c_wait_done(b, deadline);
		if (!ret)
			ret = i2c_check_ack(b, -ENXIO);
		for (i = 0; !ret && i < wlen; ++i) {
			ret = i2c_wr(b, I2C_DATA, wbuf[i]);
			if (!ret)
				ret = i2c_wr(b, I2C_CONTROL, 0x52);
			if (!ret)
				ret = i2c_wait_done(b, deadline);
			if (!ret)
				ret = i2c_check_ack(b, -EIO);
		}
	}
	if (!ret && rlen) {
		ret = i2c_wr(b, I2C_DATA, ((u32)address << 1) | 1u);
		if (!ret)
			ret = i2c_wr(b, I2C_CONTROL, wlen ? 0x72 : 0x52);	/* 再スタート (書きの後) または開始 */
		if (!ret)
			ret = i2c_wait_done(b, deadline);
		if (!ret)
			ret = i2c_check_ack(b, -ENXIO);
		for (i = 0; !ret && i < rlen; ++i) {
			u32 value = 0;

			/* 最後の 1 バイト以外は ACK を返す */
			ret = i2c_wr(b, I2C_CONTROL, i + 1 < rlen ? 0x5a : 0x52);
			if (!ret)
				ret = i2c_wait_done(b, deadline);
			if (!ret)
				ret = i2c_reg8(b, I2C_DATA, &value);
			if (!ret)
				rbuf[i] = (u8)value;
		}
	}
	/* 停止: 開始ビットを落として、バスが空くのを待つ。ここでの失敗は、元のエラーを優先する。 */
	{
		int stop = i2c_wr(b, I2C_CONTROL, 0x42);

		if (!stop)
			stop = i2c_wait_idle(b, b->ops->now_ms(b->priv) + I2C_TIMEOUT_MS);
		if (!ret)
			ret = stop;
	}
	if (ret)
		b->ops->log(b->priv, DT295_LOG_ERR, "DT295 I2C %#x: transfer failed (%d). The bus may stay stuck for several seconds",
			    address, ret);
	return ret;
}

/* ---- 取り込みブロック ---- */

struct cap_write {
	u32 reg;
	u32 value;
	u32 mask;
};

/*
 * 窓越しの書き込み列。順序は、px4_drv の torne の stream_setup() と同じ位置関係 (0x1901000 のポートモードの
 * マスク付き書き込みを除く)。リセット解除の列 (最初の 5 つ) の値は別の実装の観測で得たもの。
 */
static const struct cap_write cap_config[] = {
	/* リセット解除 */
	{ 0x19000c4, 0x00000000, 0xffffffff },
	{ 0x1900004, 0x001f11ff, 0xffffffff },
	{ 0x1900000, 0x3002ffff, 0xffffffff },
	{ 0x1900000, 0x00020000, 0xffffffff },
	{ 0x1900004, 0x00001100, 0xffffffff },
	/* 入力を止めておく */
	{ 0x1901040, 0x00000000, 0xffffffff },
	{ 0x1901064, 0x00000000, 0xffffffff },
	{ 0x19010a0, 0x00000000, 0xffffffff },
	{ 0x19010a4, 0x00000000, 0xffffffff },
	{ 0x19010a8, 0x00000000, 0xffffffff },
	{ 0x1901000, 0x06000000, 0xffff0000 },	/* ポートモード (前半) */
	{ 0x1901020, 0x02000000, 0xffffffff },
	{ 0x1901004, 0x04000000, 0xffffffff },
	{ 0x1901000, 0x00000000, 0x0000ffff },
	/* 188 バイトのパケット、同期バイト */
	{ 0x1901044, 0x00bc0000, 0xffffffff },
	{ 0x1901048, 0x47000000, 0xffffffff },
	{ 0x190104c, 0x00ffffff, 0xffffffff },
	{ 0x1901050, 0x00000000, 0xffffffff },
	{ 0x1901060, 0x00000000, 0xffffffff },
	/* タイムスタンプ (192 バイト単位) */
	{ 0x1901080, 0x01000000, 0xffffffff },
	{ 0x1901088, 0x00000000, 0xffffffff },
	{ 0x190108c, 0xffffffff, 0xffffffff },
	{ 0x1901084, 0x00000000, 0xffffffff },
	{ 0x1901090, 0x01000000, 0xffffffff },
	/* リング (物理アドレス 0x0138EC00、0x30000 バイト) と位置 */
	{ 0x1902020, 0x0138ec00, 0xffffffff },
	{ 0x1902024, 0x00030000, 0xffffffff },
	{ 0x1902028, 0x00000000, 0xffffffff },
	{ 0x190202c, 0x00000000, 0xffffffff },
	{ 0x1902030, 0x00000000, 0xffffffff },
	{ 0x1902000, 0x03020300, 0xffffffff },
	{ 0x1901000, 0x06070000, 0xffff0000 },	/* ポートモード (後半) */
	/* 入力を開く */
	{ 0x1901040, 0x00000000, 0xffffffff },
	{ 0x1901020, 0x00000000, 0xffffffff },
	{ 0x1901040, 0x03000000, 0xffffffff },
	{ 0x19010a4, 0x13170400, 0xffffffff },
	/* タイムスタンプのクロック (ライタの開始は dt295_cap_run) */
	{ 0x1900484, 0x01000000, 0xffffffff },
	{ 0x1900488, 0x206c0000, 0xffffffff },
};

int dt295_cap_stop(struct dt295_bridge *b)
{
	int ret;

	if (!b->window_set)
		return -EINVAL;
	ret = dt295_blk_write(b, 0x1902010, 0x04000000);	/* ライタを止める */
	if (!ret)
		ret = dt295_blk_write(b, 0x1901040, 0);		/* 入力を閉じる */
	if (!ret)
		ret = dt295_blk_write(b, 0x1901020, 0x02000000);	/* 入力を保留 */
	return ret;
}

int dt295_cap_start(struct dt295_bridge *b)
{
	unsigned int i;
	int ret;

	/* 取り込みブロックはクロックのゲート (0xF10118 bit4) が開いていないと応答しない (board_open で 0xFFFF にしてある) */
	ret = dt295_window_set(b, 0x01900000u);
	if (ret)
		return ret;
	/* 前回の取り込みが残っていても、書き込み位置と入力を止めてから、設定し直す */
	ret = dt295_cap_stop(b);
	for (i = 0; !ret && i < sizeof(cap_config) / sizeof(cap_config[0]); ++i)
		ret = dt295_blk_mask(b, cap_config[i].reg, cap_config[i].value, cap_config[i].mask);
	if (ret) {
		b->ops->log(b->priv, DT295_LOG_ERR, "DT295 capture start failed at step %u (%d)", i, ret);
		/* 失敗したら止めて、窓を戻す。ここでの失敗は、元のエラーを優先する。 */
		dt295_cap_stop(b);
		dt295_window_restore(b);
	}
	return ret;
}

/*
 * ライタを開始する。ライタは 0x30000 バイトのリングを約 50 ms で 1 周するので、読み出し側 (ポンプ) が動き出す前に開始すると、
 * 読み出される前に書き込みが 1 周して、未読のデータを上書きする (1 周目の直後に連続性の飛びが出る)。
 * そのため、cap_start (設定) と分け、ポンプが動いてから呼ぶ。
 */
int dt295_cap_run(struct dt295_bridge *b)
{
	return dt295_blk_write(b, 0x1902010, 0x02000000);
}

int dt295_cap_wptr(struct dt295_bridge *b, u32 *wptr)
{
	return dt295_blk_read(b, 0x1902028, wptr);
}

int dt295_cap_set_rptr(struct dt295_bridge *b, u32 rptr)
{
	return dt295_blk_write(b, 0x190202c, rptr);
}
