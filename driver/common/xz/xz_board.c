// SPDX-License-Identifier: GPL-2.0-only
/*
 * GV-MVP/XZ2: 起動直後の識別、SoC の初期化、I2C コントローラ、取り込みブロック。
 *
 * SoC の初期化の表と、取り込みブロックの設定値の出どころは、ファームウェアの動作の観察 (レジスタのアドレス、値、順序) で、
 * 機械語は含まない。SoC の表は、実機の試験 (ddmin) で、地上波の受信に必要な部分に絞ってある (29 項目のうち 9 件)。
 * SoC のレジスタは、試験の間のリセット (復元) で起動直後の値へ戻ることを読み戻しで確認しているが、前面 (xz_frontend.c) で最小化が
 * 成立しなかった経緯があり、機材の抜き差し直後からの確認は限られる。絞った根拠と未確認の条件は、docs/devices/xz.md を参照。
 */
#include "xz_core.h"

#define CHIP_ID_ADDRESS 0xf00000u
#define CHIP_ID_WORD 0x4d423841u	/* "MB8A" (MB8AC018) */
#define SIGNATURE_ADDRESS 0xe9ce8u
#define WINDOW_READ_IMMEDIATE 0xe9a2cu
#define WINDOW_WRITE_IMMEDIATE 0xe9d78u
#define WINDOW_DEFAULT 0x00f00000u
#define STATE_ADDRESS 0x82008u
#define HOOK_ADDRESS 0xe9f48u

/* 識別に使う、この版の boot0 の署名と、hook の位置の元の 8 バイト。XZ から XZ3 は USB ID が同じなので、これで版を確かめる。 */
static const u8 boot0_signature[8] = { 0xb4, 0x27, 0x9f, 0x8d, 0x00, 0x0c, 0x46, 0x00 };
static const u8 original_hook[8] = { 0xc4, 0x0d, 0x02, 0xa0, 0xa8, 0x10, 0xe3, 0x0e };

/*
 * SoC の初期化の再生。種類 (type):
 *   0, 6: 語全体を (value | (mask & 現在の値)) にする
 *   3:    語全体を value にする
 *   1, 2, 5: 上位半語だけを ((半語 & mask) | value) にする (5 は先に半語へ 0x1000 を足す)。下位半語は保つ。
 * mAgicVISION ファームウェアの常駐プログラムの表の 16 バイトのレコードと同じ意味。
 */
struct soc_record {
	u32 address, value, mask;
	u16 delay_ms;
	u8 type;
};

static const struct soc_record soc_records[] = {
	{ 0xf10114, 0x00008008, 0x00000000, 0, 1 },
	{ 0xf10118, 0x0000ffff, 0x00000000, 0, 1 },	/* クロックのゲートを全て開く (取り込みブロックの 0xD00000 もここ) */
	{ 0xf10128, 0x00006100, 0x00000e0e, 0, 1 },
	{ 0xf10014, 0x0000d0f4, 0x0000ffff, 0, 1 },
	{ 0xf10018, 0x000000e4, 0x0000ff1b, 10, 5 },
	{ 0xfd8010, 0x00000010, 0xffffffff, 0, 0 },	/* I2C コントローラと端子 */
	{ 0xfd8014, 0x00000000, 0xffffffef, 0, 0 },
	{ 0xfd80c0, 0x00000000, 0x00000000, 0, 3 },
	{ 0xfd8fe8, 0x0000003c, 0x00000000, 0, 6 },
};

static int soc_replay(struct xz_bridge *b)
{
	unsigned int i;

	for (i = 0; i < sizeof(soc_records) / sizeof(soc_records[0]); ++i) {
		const struct soc_record *r = &soc_records[i];
		u32 old, value, half;
		int ret = xz_read32(b, r->address, &old);

		if (ret)
			return ret;
		if (r->type == 3) {
			value = r->value;
		} else if (r->type == 0 || r->type == 6) {
			value = r->value | (r->mask & old);
		} else {
			half = old >> 16;
			if (r->type == 5)
				half |= 0x1000u;
			half = ((half & r->mask) | r->value) & 0xffffu;
			value = (half << 16) | (old & 0xffffu);
		}
		ret = xz_write32(b, r->address, value);
		if (ret)
			return ret;
		if (r->delay_ms)
			b->ops->msleep(b->priv, r->delay_ms);
	}
	return 0;
}

int xz_board_open(struct xz_bridge *b)
{
	u32 chip = 0, read_imm = 0, write_imm = 0, state = 0;
	u8 signature[8], hook[8], reg = 0x80, r;
	int ret;

	/* 識別: 版の違う boot0 へ hook を書くと機材が止まるおそれがあるので、全て確かめてから何かを書く。 */
	ret = xz_read32(b, CHIP_ID_ADDRESS, &chip);
	if (ret)
		return ret;
	if (chip != CHIP_ID_WORD) {
		b->ops->log(b->priv, XZ_LOG_ERR, "XZ chip id %08x is not the expected one; nothing written", chip);
		return -ENODEV;
	}
	ret = xz_read(b, SIGNATURE_ADDRESS, signature, sizeof(signature));
	if (ret)
		return ret;
	if (memcmp(signature, boot0_signature, sizeof(signature))) {
		b->ops->log(b->priv, XZ_LOG_ERR, "XZ boot0 signature differs (another revision?); nothing written");
		return -ENODEV;
	}
	/* 起動直後の確認: 窓の即値が既定、状態が 0、hook が元の値。違えば、他の使い手が触った後なので、挿し直しを促す。 */
	ret = xz_read32(b, WINDOW_READ_IMMEDIATE, &read_imm);
	if (!ret)
		ret = xz_read32(b, WINDOW_WRITE_IMMEDIATE, &write_imm);
	if (!ret)
		ret = xz_read32(b, STATE_ADDRESS, &state);
	if (!ret)
		ret = xz_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (ret)
		return ret;
	if (read_imm != WINDOW_DEFAULT || write_imm != WINDOW_DEFAULT || (state >> 16) || memcmp(hook, original_hook, sizeof(hook))) {
		b->ops->log(b->priv, XZ_LOG_ERR, "XZ is not in the fresh boot0 state; replug the device");
		return -EBUSY;
	}

	ret = soc_replay(b);
	if (ret)
		return ret;
	/* I2C コントローラの確認: 復調器 (地上波) が応答すること。 */
	return xz_i2c_transfer(b, 0x18, &reg, 1, &r, 1);
}

/* ---- I2C コントローラ (SoC の 0xFD8FE0 台) ---- */
#define I2C_STATUS 0xfd8fe0u
#define I2C_CONTROL 0xfd8fe4u
#define I2C_DATA 0xfd8ff0u
#define I2C_TIMEOUT_MS 200U

static int reg8(struct xz_bridge *b, u32 address, u32 *value)
{
	int ret;

	*value = 0;
	ret = xz_read32(b, address, value);
	if (!ret)
		*value &= 0xff;
	return ret;
}

static int i2c_wait_done(struct xz_bridge *b, u64 deadline)
{
	u32 control = 0;

	for (;;) {
		int ret = reg8(b, I2C_CONTROL, &control);

		if (ret)
			return ret;
		if (control & 1)
			return 0;
		if (b->ops->now_ms(b->priv) > deadline)
			return -ETIMEDOUT;
	}
}

static int i2c_wait_idle(struct xz_bridge *b, u64 deadline)
{
	u32 status = 0;

	for (;;) {
		int ret = reg8(b, I2C_STATUS, &status);

		if (ret)
			return ret;
		if (!(status & 0x80))
			return 0;
		if (b->ops->now_ms(b->priv) > deadline)
			return -ETIMEDOUT;
	}
}

/* 1 バイトを送って、完了と ACK を確かめる。ACK なしは -ENXIO。 */
static int i2c_send_byte(struct xz_bridge *b, u32 byte, u32 control, u64 deadline)
{
	u32 status = 0;
	int ret = xz_write32(b, I2C_DATA, byte);

	if (!ret)
		ret = xz_write32(b, I2C_CONTROL, control);
	if (!ret)
		ret = i2c_wait_done(b, deadline);
	if (!ret)
		ret = reg8(b, I2C_STATUS, &status);
	if (ret)
		return ret;
	return (status & 0x10) ? -ENXIO : 0;
}

int xz_i2c_transfer(struct xz_bridge *b, u8 address, const u8 *wbuf, unsigned int wlen, u8 *rbuf, unsigned int rlen)
{
	u64 deadline;
	unsigned int i;
	int ret, stop;

	if (!b || (wlen && !wbuf) || (rlen && !rbuf) || !(wlen || rlen) || wlen + rlen > 40)
		return -EINVAL;
	deadline = b->ops->now_ms(b->priv) + I2C_TIMEOUT_MS;
	ret = i2c_wait_idle(b, deadline);
	if (ret)
		return ret;
	/* 書き込み: アドレス (書き) + データ。 */
	if (wlen) {
		ret = i2c_send_byte(b, (u32)address << 1, 0x52, deadline);
		for (i = 0; !ret && i < wlen; ++i)
			ret = i2c_send_byte(b, wbuf[i], 0x52, deadline);
	}
	/* 読み出し: アドレス (読み)。書きがあったときは繰り返しの開始条件 (0x72)。最後の 1 バイトだけ NACK で終える。 */
	if (!ret && rlen) {
		ret = i2c_send_byte(b, ((u32)address << 1) | 1, wlen ? 0x72 : 0x52, deadline);
		for (i = 0; !ret && i < rlen; ++i) {
			u32 value = 0;

			ret = xz_write32(b, I2C_CONTROL, i + 1 < rlen ? 0x5a : 0x52);
			if (!ret)
				ret = i2c_wait_done(b, deadline);
			if (!ret)
				ret = reg8(b, I2C_DATA, &value);
			if (!ret)
				rbuf[i] = (u8)value;
		}
	}
	/* 停止条件。途中の失敗の後も出す (バスを掴んだままにしない)。失敗の原因は最初のものを返す。 */
	stop = xz_write32(b, I2C_CONTROL, 0x42);
	if (!stop)
		stop = i2c_wait_idle(b, b->ops->now_ms(b->priv) + I2C_TIMEOUT_MS);
	return ret ? ret : stop;
}

/* ---- 取り込みブロック (0xD00000。mAgicVISION ファームウェアの設定関数と同じ値) ---- */
#define CAP_WPTR 0xd02028u
#define CAP_RPTR 0xd0202cu

struct reg_write {
	u32 address, value;
};

static int write_list(struct xz_bridge *b, const struct reg_write *list, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; ++i) {
		int ret = xz_write32(b, list[i].address, list[i].value);

		if (ret)
			return ret;
	}
	return 0;
}

int xz_cap_start(struct xz_bridge *b)
{
	/* サブブロックのリセット解除 */
	static const struct reg_write release[] = {
		{ 0xd000c4, 0 }, { 0xd00000, 0xffffffff }, { 0xd00004, 0x001f33ff }, { 0xd00000, 0x30030fff },
		{ 0xd00000, 0x00030000 }, { 0xd00004, 0x00003305 },
	};
	/* 設定。0xD01004 = 0x04000000 と 0xD0108C = 0xFFFFFFFF は、mAgicVISION ファームウェアの値 (別の機種 (torne) の流儀とは違う)。 */
	static const struct reg_write config[] = {
		{ 0xd01040, 0 }, { 0xd01064, 0 }, { 0xd010a0, 0 }, { 0xd010a4, 0 }, { 0xd010a8, 0 },
		{ 0xd01020, 0x02000000 },	/* 入力を止めておく */
		{ 0xd01004, 0x04000000 }, { 0xd01044, 0x00bc0000 },	/* 188 バイトのパケット */
		{ 0xd01048, 0x47000000 },	/* 同期バイト */
		{ 0xd0104c, 0x00ffffff }, { 0xd01050, 0 }, { 0xd01060, 0 },
		{ 0xd01000, 0x06070000 },	/* ポートモード (地上波) */
		{ 0xd01080, 0x01000000 },	/* タイムスタンプ: 192 バイト単位 */
		{ 0xd01088, 0 }, { 0xd0108c, 0xffffffff }, { 0xd01084, 0 }, { 0xd01090, 0x01000000 },
		{ 0xd02028, 0 }, { 0xd0202c, 0 }, { 0xd02030, 0 }, { 0xd02038, 0 },
		{ 0xd02000, 0x03020300 }, { 0xd02020, XZ_RING_BASE }, { 0xd02024, XZ_RING_SIZE }, { 0xd00004, 0x00043305 },
	};
	u32 check = 0;
	int ret = write_list(b, release, sizeof(release) / sizeof(release[0]));

	/* サブブロックが書き込みを保持するか (保持しなければ、ゲートが開いていない)。 */
	if (!ret)
		ret = xz_write32(b, 0xd01044, 0x00bc0000);
	if (!ret)
		ret = xz_read32(b, 0xd01044, &check);
	if (!ret && check != 0x00bc0000)
		ret = -EIO;
	if (ret)
		return ret;
	return write_list(b, config, sizeof(config) / sizeof(config[0]));
}

int xz_cap_run(struct xz_bridge *b)
{
	/* いったん止めて位置を 0 に戻してから、入力、タイムスタンプのクロック、ライタを開始する。 */
	static const struct reg_write run[] = {
		{ 0xd02010, 0x04000000 }, { 0xd01040, 0 }, { 0xd01020, 0x02000000 },
		{ 0xd02028, 0 }, { 0xd0202c, 0 }, { 0xd02030, 0 }, { 0xd02038, 0 },
		{ 0xd01020, 0 }, { 0xd01040, 0x03000000 }, { 0xd010a4, 0x13170400 },
		{ 0xd00484, 0x01000000 }, { 0xd00488, 0x206c0000 }, { 0xd02010, 0x02000000 },
	};

	return write_list(b, run, sizeof(run) / sizeof(run[0]));
}

int xz_cap_stop(struct xz_bridge *b)
{
	static const struct reg_write stop[] = {
		{ 0xd02010, 0x04000000 }, { 0xd01040, 0 }, { 0xd01020, 0x02000000 },
	};

	return write_list(b, stop, sizeof(stop) / sizeof(stop[0]));
}

int xz_cap_wptr(struct xz_bridge *b, u32 *wptr)
{
	u32 value = 0;
	int ret = xz_read32(b, CAP_WPTR, &value);

	if (ret)
		return ret;
	*wptr = value % XZ_RING_SIZE;
	return 0;
}

int xz_cap_set_rptr(struct xz_bridge *b, u32 rptr)
{
	if (rptr >= XZ_RING_SIZE || rptr % XZ_UNIT_SIZE)
		return -EINVAL;
	return xz_write32(b, CAP_RPTR, rptr);
}
