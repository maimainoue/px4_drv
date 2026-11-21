// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295: 地上波の復調器 (TC90522 系、I2C 0x10) とチューナー (MaxLinear MxL603 系、復調器の中継 0xFE 0xC0 の先)。
 *
 * 最小の初期化は、実機の最小化 (復調器とチューナーの 145 個の書き込みから 4 つ) の結果で、値は次の公開資料と照合済み
 * (dt295_init_crosscheck.md): px4_drv の TC90522 ドライバと isdb2056_device.c、MIT の資料 (smpn2/pix-dt295 の
 * docs/device.md と fe1.rs)、MaxLinear の MxL603 の公開 API (レジスタ 0x03 の水晶の校正、0x05 の IF の反転と利得)。
 */
#include "dt295_core.h"

#define DEMOD_T 0x10

/* 復調器 */
#define DEMOD_TS_OUTPUT_SELECT 0x0e	/* 起動直後 0x77。T の出力をバスに流す値 0x71 */
#define DEMOD_TS_PINS 0x1d		/* 起動直後 0x55。TS の出力ピンの有効化 0x02 */
#define DEMOD_LOCK_ASSIST 0x01		/* ビット 6 を立てると、選局直後の同期が速くなる (ビット 7 は 0 のまま) */
#define DEMOD_STATUS 0x80
#define DEMOD_CNR 0x8b
#define DEMOD_AGC_STATUS 0xb0

/* チューナー (復調器の中継: 書きは [0xFE, 0xC0, レジスタ, 値]) */
#define TUNER_RELAY 0xfe
#define TUNER_ADDRESS 0xc0
#define TUNER_XTAL_CAL 0x03		/* 0x00 を書いて 0x01 を書くと、水晶の校正を実行 */
#define TUNER_IF_PATH_GAIN 0x05		/* ビット 7〜6 = スペクトラム反転、ビット 3〜0 = 利得 */
#define TUNER_CNR_BOOST 0x08		/* 起動直後 0x01。ビット 2 を立てると C/N が良くなる (ビット 0 は 1、ビット 3 は 0 のまま) */
#define TUNER_POWER 0x0b
#define TUNER_FREQ_LOW 0x10
#define TUNER_FREQ_HIGH 0x11
#define TUNER_START_TUNE 0x12
#define TUNER_BAND_A 0x31
#define TUNER_BAND_B 0x7c

#define TUNE_MIN_KHZ 90000U	/* チューナーが受け付ける範囲 (90〜770 MHz) */
#define TUNE_MAX_KHZ 770000U
#define BAND_SPLIT_KHZ 700000U	/* 700 MHz 以上 (UHF 51ch から) は 0x31/0x7C の値が変わる */

static int demod_write(struct dt295_bridge *b, u8 reg, u8 value)
{
	u8 data[2] = { reg, value };

	return dt295_i2c_transfer(b, DEMOD_T, data, sizeof(data), NULL, 0);
}

static int demod_read(struct dt295_bridge *b, u8 reg, u8 *buf, unsigned int len)
{
	return dt295_i2c_transfer(b, DEMOD_T, &reg, 1, buf, len);
}

static int tuner_write(struct dt295_bridge *b, u8 reg, u8 value)
{
	u8 data[4] = { TUNER_RELAY, TUNER_ADDRESS, reg, value };

	return dt295_i2c_transfer(b, DEMOD_T, data, sizeof(data), NULL, 0);
}

int dt295_fe_init(struct dt295_bridge *b)
{
	int ret = demod_write(b, DEMOD_TS_OUTPUT_SELECT, 0x71);

	if (!ret)
		ret = demod_write(b, DEMOD_TS_PINS, 0x02);
	if (!ret)
		ret = tuner_write(b, TUNER_XTAL_CAL, 0x01);
	if (!ret)
		ret = tuner_write(b, TUNER_IF_PATH_GAIN, 0x1b);
	/*
	 * 最小の初期化 (上の 4 つ) では、ロックと TS は得られるが、C/N が 0x08 の設定より悪い。
	 * 公開された別実装 (soralis0912/MB8AC031-CFW の fe_dt295.h) の「チューナー 0x08 = 0x05 で C/N が上がる」を、
	 * 実機 (Rev.01 と Rev.02、UHF 13/21/30ch) で確かめた: C/N の生値 (小さいほど良い) が約 5400 から約 3300 に下がる。
	 */
	if (!ret)
		ret = tuner_write(b, TUNER_CNR_BOOST, 0x05);
	return ret;
}

int dt295_fe_tune(struct dt295_bridge *b, u32 freq_khz)
{
	u32 value;
	int ret;

	if (freq_khz < TUNE_MIN_KHZ || freq_khz > TUNE_MAX_KHZ)
		return -EINVAL;
	/* チューナーの周波数は MHz の 1/64 の単位 (例: 545.142857 MHz = 0x8849) */
	value = (u32)(((u64)freq_khz * 64U + 500U) / 1000U);
	/*
	 * 選局の最初に書く。これを書くと、選局直後に出ていた TEI と連続性エラー (UHF 13/21/30ch で最初の約 0.4 秒に
	 * TEI が 3000〜18000) が 0 になった (Rev.01 と Rev.02)。代わりに、この選局の所要時間が約 80 ms から約 540 ms に伸びる。
	 */
	ret = demod_write(b, DEMOD_LOCK_ASSIST, 0x40);
	if (!ret)
		ret = tuner_write(b, TUNER_START_TUNE, 0x00);
	if (!ret)
		ret = tuner_write(b, TUNER_POWER, 0x01);
	if (!ret)
		ret = tuner_write(b, TUNER_FREQ_LOW, value & 0xff);
	if (!ret)
		ret = tuner_write(b, TUNER_FREQ_HIGH, (value >> 8) & 0xff);
	if (!ret)
		ret = tuner_write(b, TUNER_BAND_A, freq_khz < BAND_SPLIT_KHZ ? 0x81 : 0x91);
	if (!ret)
		ret = tuner_write(b, TUNER_BAND_B, freq_khz < BAND_SPLIT_KHZ ? 0x1f : 0x9f);
	if (!ret)
		ret = tuner_write(b, TUNER_START_TUNE, 0x01);
	return ret;
}

/* px4_drv の tc90522_is_signal_locked_t と同じ判定: 0x80 の bit5 と bit3 が 0、0xB0 の下位 4 ビットが 8 以上。 */
int dt295_fe_locked(struct dt295_bridge *b, bool *locked)
{
	u8 status, agc;
	int ret;

	*locked = false;
	ret = demod_read(b, DEMOD_STATUS, &status, 1);
	if (ret)
		return ret;
	if (status & 0x28)
		return 0;
	ret = demod_read(b, DEMOD_AGC_STATUS, &agc, 1);
	if (ret)
		return ret;
	*locked = (agc & 0x0f) >= 8;
	return 0;
}

int dt295_fe_cnr_raw(struct dt295_bridge *b, u32 *raw)
{
	u8 data[3];
	int ret = demod_read(b, DEMOD_CNR, data, sizeof(data));

	if (!ret)
		*raw = (u32)data[0] << 16 | (u32)data[1] << 8 | data[2];
	return ret;
}
