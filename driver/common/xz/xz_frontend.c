// SPDX-License-Identifier: GPL-2.0-only
/*
 * GV-MVP/XZ2: 前面 (地上波)。復調器 TC90512 (TC90522 系、I2C 0x18 が地上波、0x19 が衛星) と、その中継 (0xFE) の先のチューナー (I2C 0x60)。
 *
 * 書き込みの列は、mAgicVISION ファームウェアが前面へ送った I2C の通信の観察 (アドレス、バイト列、順序) で、xz_fe_tables_gen.h にある。
 * 機械語は含まない。各項目の意味は特定していない (観察した値のまま)。
 * 項目を減らす最小化は、前面については成立していない: 実機の ddmin で絞った集合は、受信が欠けずに流れたが、TS の出力が全再生と違った
 * (null が約 1.7 倍に増えた、または TEI が約 18% 出た)。ddmin の試験間のリセットは、復調器とチューナーの内部状態を戻さないので、
 * 前の試験が残した設定が、落とした項目を補っていた可能性が高い。衛星側 (0x19) の項目も、地上波の受信に要らないとは確かめていない。
 */
#include "xz_core.h"
#include "xz_fe_tables_gen.h"

#define DEMOD_TERRESTRIAL 0x18u
#define DEMOD_SATELLITE 0x19u
#define STEP_DELAY_MS 2U	/* mAgicVISION ファームウェアの再生と同じ、通信と通信の間隔 */
#define FREQ_DATA_LENGTH 10u

static int send_write(struct xz_bridge *b, const struct xz_fe_write *w, const u8 *data)
{
	int ret = xz_i2c_transfer(b, w->address, data, w->length, NULL, 0);

	if (!ret)
		b->ops->msleep(b->priv, STEP_DELAY_MS);
	return ret;
}

int xz_fe_init(struct xz_bridge *b)
{
	unsigned int i;

	for (i = 0; i < XZ_FE_INIT_COUNT; ++i) {
		int ret = send_write(b, &xz_fe_writes[i], xz_fe_writes[i].data);

		if (ret)
			return ret;
	}
	return 0;
}

int xz_fe_tune(struct xz_bridge *b, u32 freq_khz)
{
	unsigned int i;
	u32 value;

	/* UHF 13〜62ch の範囲 (470〜770 MHz) だけを受け付ける。範囲外の周波数で受信できるかは確認していない。 */
	if (freq_khz < 470000 || freq_khz > 770000)
		return -EINVAL;
	value = (u32)((u64)freq_khz * 64 / 1000);	/* MHz × 64 の整数部 */
	for (i = 0; i < XZ_FE_TUNE_COUNT; ++i) {
		const struct xz_fe_write *w = &xz_fe_writes[XZ_FE_INIT_COUNT + i];
		u8 data[40];
		int ret;

		memcpy(data, w->data, sizeof(data));
		if (i == XZ_FE_FREQ_INDEX) {
			/* 中継 (0xFE 0xC0) の先のチューナーのレジスタ 0x14 (下位) と 0x15 (上位)。表は 25ch の値なので、ここで差し替える。 */
			if (w->length != FREQ_DATA_LENGTH || data[0] != 0xfe || data[6] != 0x14 || data[8] != 0x15)
				return -EINVAL;
			data[7] = (u8)value;
			data[9] = (u8)(value >> 8);
		}
		ret = send_write(b, w, data);
		if (ret)
			return ret;
	}
	return 0;
}

/* 1 バイトのレジスタを読む (書き: レジスタ番号、読み: 1 バイト)。 */
static int demod_read(struct xz_bridge *b, u8 reg, u8 *value)
{
	return xz_i2c_transfer(b, DEMOD_TERRESTRIAL, &reg, 1, value, 1);
}

int xz_fe_locked(struct xz_bridge *b, bool *locked)
{
	u8 status = 0, quality = 0;
	int ret = demod_read(b, 0x80, &status);

	if (!ret)
		ret = demod_read(b, 0xb0, &quality);
	if (ret)
		return ret;
	/* px4_drv の地上波 (tc90522_is_signal_locked) と同じ判定 */
	*locked = !(status & 0x28) && (quality & 0x0f) >= 8;
	return 0;
}

int xz_fe_cnr_raw(struct xz_bridge *b, u32 *raw)
{
	u8 reg = 0x8b, data[3];
	int ret = xz_i2c_transfer(b, DEMOD_TERRESTRIAL, &reg, 1, data, sizeof(data));

	if (ret)
		return ret;
	*raw = (u32)data[0] << 16 | (u32)data[1] << 8 | data[2];
	return 0;
}

int xz_fe_sleep(struct xz_bridge *b)
{
	/* 受信を止めるときの休止 (mAgicVISION ファームウェアの終了処理の記録と同じ): 地上波 0x03 = 0x90、衛星 0x17 = 0x01 */
	static const u8 sleep_terrestrial[2] = { 0x03, 0x90 };
	static const u8 sleep_satellite[2] = { 0x17, 0x01 };
	int ret = xz_i2c_transfer(b, DEMOD_TERRESTRIAL, sleep_terrestrial, sizeof(sleep_terrestrial), NULL, 0);

	if (!ret)
		ret = xz_i2c_transfer(b, DEMOD_SATELLITE, sleep_satellite, sizeof(sleep_satellite), NULL, 0);
	return ret;
}
