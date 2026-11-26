// SPDX-License-Identifier: GPL-2.0-only
/*
 * ut100b_core.c - QRS-UT100B の起動、選局、C/N、閉じる手順。
 * 手順と値の出どころは ut100b_core.h の先頭を参照。
 *
 * I2C の構成:
 *   ASV5211 (USB ブリッジ) ── I2C ─┬─ 0x20: 復調器 (TC90522 の地上波側と同じレジスタの並び)
 *                                   │         └ 中継 (0xfe) ─ 0xc0: チューナー (型番不明)
 *                                   └─ 0x4a: ASIE5606 (B-CAS カードの IC。USB 転送の TS に DES もかける)
 */
#include "ut100b_core.h"
#include "hz2w_internal.h"

#define I2C_DEMOD 0x20
#define I2C_TUNER 0xc0
#define I2C_CARD 0x4a

/* ASV5211 のベンダー要求 (recfriio の internal/asv5211 と、PLEX の libPlexLib の名前に合わせた) */
#define REQ_REG_READ 0x02
#define REQ_REG_WRITE 0x03
#define REQ_FILTER_READ 0x04
#define REQ_FILTER_WRITE 0x05
#define REQ_STREAM_START 0x06
#define REQ_STREAM_STOP 0x07
#define REQ_GPIO 0x08
#define REQ_FILTER_RESET 0x09
#define REQ_HIGH_SPEED 0x0a
#define REQ_CUSTOMER_INFO 0x0c
#define REQ_I2C_FILL 0x0d
#define REQ_I2C_SEND 0x0e
#define REQ_GPIO_EX 0x10
#define REQ_SYSCTRL_READ 0x17

/* 復調器 (TC90522 地上波側) のレジスタ */
#define TC_AGC1 0x01
#define TC_1C 0x1c	/* 制御。選局で bit 4、5 を立てる */
#define TC_1E 0x1e	/* 出力の線の設定と見られる */
#define TC_1F 0x1f	/* 0x1e と組で、選局の前後に書く (意味は未確認) */
#define TC_AGC23 0x23	/* AGC。bit 0 が自動 (0x4d) か止める (0x4c) か */
#define TC_AGC25 0x25
#define TC_CNDAT 0x8b	/* C/N の生の値 (0x8b..0x8d の 24 ビット) */
#define TC_REPEATER 0xfe	/* ここに書くと、続くバイトがチューナーへ中継される */

/* ASIE5606 のレジスタ (TS の設定) */
#define CARD_CTRL 0x00
#define CARD_04 0x04	/* 閉じるときに 0 (意味は未確認) */
#define CARD_TS_MODE 0x05	/* USB 転送の TS の暗号の方式: 0x91 = XOR、0x80 = DES */
#define CARD_IDENTITY 0x09	/* 下位 3 ビットが 6 なら DES の版 */
#define CARD_TS_KEY 0x10	/* 0x10..0x1f。普通の書き込みでは鍵は変わらないが、TS を出すのに書く必要がある */
#define CARD_80 0x80	/* 閉じるときに 0 (意味は未確認) */
#define CARD_TS_MODE_DES 0x80

/* TS フィルター (チャンネル 0) のレジスタ */
#define FILTER_CONTROL 0x40	/* bit 0、1 入力、bit 2 モード、bit 3 動かす */
#define FILTER_XPID_MIN 0x41	/* 捨てる PID の範囲 (16 ビット) */
#define FILTER_XPID_MAX 0x43

/* GPIO (どの線が何かは元のコメントからの推測) */
#define GPIO_TUNER_POWER 0x08	/* 1 で切る、0 で入れる */
#define GPIO_CARD_RESET 0x11	/* ASIE5606 のリセット (1 → 0 → 1) */
#define GPIO_D4 0xd4	/* 切るときに 0 (意味は未確認) */
#define GPIO_D5 0xd5	/* 入れるときに 0 → 1 (意味は未確認) */

/* チューナー (0xc0) のレジスタ (選局で使うもの) */
#define TUNER_CONTROL 0x11	/* 0 にしてから周波数を書き、2 で選局を始める (と見られる) */
#define TUNER_13 0x13	/* 意味は未確認 (選局のたびに 0x15) */
#define TUNER_FREQ_LO 0x14	/* 周波数 (1/64 MHz 単位) の下位 */
#define TUNER_FREQ_HI 0x15	/* 上位 */

/* 復調部の初期値 (レジスタと値の組。recfriio の Hdp.cpp の表) */
static const u8 demod_init[][2] = {
	{ 0x01, 0x00 }, { 0x02, 0x00 }, { 0x03, 0x00 }, { 0x04, 0x00 }, { 0x05, 0x00 }, { 0x06, 0x00 }, { 0x07, 0x00 }, { 0x08, 0x00 },
	{ 0x0c, 0x00 }, { 0x11, 0x21 }, { 0x12, 0x10 }, { 0x13, 0x03 }, { 0x14, 0xe0 }, { 0x15, 0x42 }, { 0x16, 0x09 }, { 0x17, 0x20 },
	{ 0x18, 0x29 }, { 0x19, 0x13 }, { 0x1a, 0x29 }, { 0x1b, 0x13 }, { 0x1c, 0x2a }, { 0x1d, 0xaa }, { 0x1e, 0xaa }, { 0x1f, 0xa8 },
	{ 0x20, 0x00 }, { 0x21, 0xff }, { 0x22, 0x80 }, { 0x23, 0x4c }, { 0x24, 0x4c }, { 0x25, 0x80 }, { 0x26, 0x00 }, { 0x27, 0x0c },
	{ 0x28, 0x60 }, { 0x29, 0x6b }, { 0x2a, 0x40 }, { 0x2b, 0x40 }, { 0x2c, 0xff }, { 0x2d, 0x00 }, { 0x2e, 0xff }, { 0x2f, 0x00 },
	{ 0x30, 0x20 }, { 0x31, 0x0f }, { 0x32, 0x84 }, { 0x34, 0x0f }, { 0x38, 0x00 }, { 0x39, 0x0f }, { 0x3a, 0x20 }, { 0x3b, 0x21 },
	{ 0x3c, 0x3f }, { 0x3d, 0x10 }, { 0x3e, 0x08 }, { 0x3f, 0x0c }, { 0x40, 0x0c }, { 0x41, 0x00 }, { 0x42, 0x00 }, { 0x43, 0x4f },
	{ 0x44, 0xff }, { 0x46, 0x20 }, { 0x47, 0x00 }, { 0x48, 0x90 }, { 0x49, 0xe6 }, { 0x4a, 0x02 }, { 0x4b, 0x54 }, { 0x4c, 0x00 },
	{ 0x50, 0x04 }, { 0x51, 0x58 }, { 0x52, 0x20 }, { 0x54, 0x57 }, { 0x55, 0xf1 }, { 0x56, 0x20 }, { 0x57, 0x70 }, { 0x58, 0x60 },
	{ 0x5c, 0x50 }, { 0x5d, 0x00 }, { 0x5e, 0x01 }, { 0x5f, 0x80 }, { 0x70, 0x18 }, { 0x71, 0x00 }, { 0x72, 0x00 }, { 0x75, 0x22 },
	{ 0x76, 0x00 }, { 0x77, 0x01 }, { 0x7c, 0x00 }, { 0x7d, 0x52 }, { 0xba, 0x00 }, { 0xbb, 0x00 }, { 0xbc, 0x00 }, { 0xc2, 0x10 },
	{ 0xc7, 0x00 }, { 0xe4, 0x1a }, { 0xe6, 0x2f }, { 0xe9, 0x08 }, { 0xec, 0x00 }, { 0xef, 0x00 },
};

/* チューナー (0xc0) の初期化。型番が分からないため、レジスタは番号のまま。 */
static const u8 tuner_init[][2] = {
	{ 0x2c, 0x44 }, { 0x4d, 0x40 }, { 0x7f, 0x02 }, { 0x9a, 0x52 }, { 0x48, 0x5a }, { 0x76, 0x1a },
	{ 0x6a, 0x48 }, { 0x64, 0x28 }, { 0x66, 0xe6 }, { 0x35, 0x11 }, { 0x7e, 0x01 }, { 0x0b, 0x99 },
	{ 0x0c, 0x00 }, { 0x10, 0x40 }, { 0x12, 0xca }, { 0x16, 0x90 }, { 0x32, 0x36 }, { 0xd8, 0x18 },
	{ 0x05, 0x01 },
};

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static void wait_ms(struct hz2w_bridge *b, unsigned int ms)
{
	b->ops->msleep(b->priv, ms);
}

/*
 * レジスターの書き込みは、状態 (応答の先頭) を見ない。元の実装 (recfriio) が見ておらず、
 * 初期化表の途中に、状態が 1 でない書き込みがあっても受信できるため。転送自体の失敗だけを返す。
 */
static int reg_write(struct hz2w_bridge *b, u8 addr, u8 reg, u8 value)
{
	u8 buf[2] = { 0, 0 };

	return hz2w_ctl_in(b, REQ_REG_WRITE, (u16)(addr | (reg << 8)), value, buf, 2);
}

static int reg_read(struct hz2w_bridge *b, u8 addr, u8 reg, u8 *value)
{
	u8 buf[2] = { 0, 0 };
	int ret = hz2w_ctl_in(b, REQ_REG_READ, (u16)(addr | (reg << 8)), 0, buf, 2);

	if (!ret)
		*value = buf[1];
	return ret;
}

static int gpio(struct hz2w_bridge *b, u8 mask, u8 value)
{
	return hz2w_gpio(b, false, value, mask, NULL);
}

/*
 * I2C の機器 addr へ bytes を送る。ASV5211 は、3 バイトずつ溜めてから (要求 0x0d)、まとめて送る (要求 0x0e)。
 * 送るバイト数は 12 以下とする (応答の入れ物の大きさ)。
 */
static int i2c_write(struct hz2w_bridge *b, u8 addr, const u8 *bytes, unsigned int n)
{
	unsigned int offset = 0;
	u8 buf[16];	/* 溜める要求の応答は offset + 1 バイト (最大 12 バイトの送信まで) */
	int ret;

	if (n > 12)
		return -EINVAL;
	while (n >= 3) {
		ret = hz2w_ctl_in(b, REQ_I2C_FILL, (u16)((bytes[0] << 8) | offset),
				  (u16)((bytes[2] << 8) | bytes[1]), buf, 4);
		if (ret)
			return ret;
		bytes += 3;
		n -= 3;
		offset += 3;
	}
	if (n == 2) {
		ret = hz2w_ctl_in(b, REQ_I2C_FILL, (u16)((bytes[0] << 8) | offset), bytes[1], buf, 3);
		if (ret)
			return ret;
		offset += 2;
	} else if (n == 1) {
		ret = hz2w_ctl_in(b, REQ_I2C_FILL, (u16)((bytes[0] << 8) | offset), 0, buf, 2);
		if (ret)
			return ret;
		offset += 1;
	}
	return hz2w_ctl_in(b, REQ_I2C_SEND, addr, 0, buf, (u16)(offset + 1));
}

/* 復調器の中継 (0xfe) を通して、チューナー (0xc0) へ reg と value を書く。 */
static int tuner_send(struct hz2w_bridge *b, u8 reg, u8 value)
{
	const u8 bytes[4] = { TC_REPEATER, I2C_TUNER, reg, value };

	return i2c_write(b, I2C_DEMOD, bytes, 4);
}

/* 復調器へ書く 1 バイト (リセットと見られる 0xff) は、中継の先へ 1 バイトだけ送る。 */
static int tuner_send_one(struct hz2w_bridge *b, u8 value)
{
	const u8 bytes[3] = { TC_REPEATER, I2C_TUNER, value };

	return i2c_write(b, I2C_DEMOD, bytes, 3);
}

static int filter_read(struct hz2w_bridge *b, u8 reg, u8 *value)
{
	u8 buf[2] = { 0, 0 };
	int ret = hz2w_ctl_in(b, REQ_FILTER_READ, reg, 0, buf, 2);

	if (!ret)
		*value = buf[1];
	return ret;
}

static int filter_write8(struct hz2w_bridge *b, u8 reg, u8 value)
{
	u8 buf[2] = { 0, 0 };

	return hz2w_ctl_in(b, REQ_FILTER_WRITE, (u16)((value << 8) | reg), 0, buf, 2);
}

/* 16 ビットのレジスタ reg (上位) と reg + 1 (下位) へ書く */
static int filter_write16(struct hz2w_bridge *b, u8 reg, u16 value)
{
	u8 buf[3] = { 0, 0, 0 };

	return hz2w_ctl_in(b, REQ_FILTER_WRITE, (u16)((value & 0xff00) | reg), (u16)(value & 0xff), buf, 3);
}

static int filter_reset(struct hz2w_bridge *b, u8 channel, u8 mode)
{
	u8 buf[1] = { 0 };

	return hz2w_ctl_in(b, REQ_FILTER_RESET, (u16)((mode << 8) | channel), 0, buf, 1);
}

static int stream_request(struct hz2w_bridge *b, u8 request, u8 channel)
{
	u8 buf[1] = { 0 };

	return hz2w_ctl_in(b, request, channel, 0, buf, 1);
}

/* 続けて送る命令の最初の失敗を覚えておく */
#define STEP(ret, expr) \
	do { \
		if (!(ret)) \
			(ret) = (expr); \
	} while (0)

int ut100b_board_open(struct hz2w_bridge *b)
{
	u8 info[0x3a];
	u8 sys[3];
	u8 tc1c = 0, identity = 0;
	unsigned int i;
	int ret;

	/* 基板の情報 (先頭の状態 1 バイトと 0x39 バイト) とシステム制御の領域の読み出し。VID/PID は warm の 3275:7020。 */
	ret = hz2w_ctl_in(b, REQ_CUSTOMER_INFO, 0, 0, info, sizeof(info));
	if (ret)
		return ret;
	if (info[0] != 1 || info[3] != 0x32 || info[4] != 0x75 || info[5] != 0x70 || info[6] != 0x20) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "unexpected board information");
		return -ENODEV;
	}
	STEP(ret, hz2w_ctl_in(b, REQ_SYSCTRL_READ, 2, 0, sys, sizeof(sys)));
	STEP(ret, gpio(b, 0xfb, 0x07));
	STEP(ret, hz2w_gpio(b, true, 0x00, 0x02, NULL));

	/* 電源を切る */
	STEP(ret, gpio(b, GPIO_D4, 0x00));
	STEP(ret, gpio(b, GPIO_TUNER_POWER, GPIO_TUNER_POWER));
	wait_ms(b, 50);
	STEP(ret, hz2w_ctl_in(b, REQ_SYSCTRL_READ, 2, 0, sys, sizeof(sys)));

	/* チューナーの電源を入れる */
	STEP(ret, gpio(b, GPIO_TUNER_POWER, GPIO_TUNER_POWER));
	wait_ms(b, 50);
	STEP(ret, gpio(b, GPIO_TUNER_POWER, 0x00));
	wait_ms(b, 150);
	STEP(ret, gpio(b, GPIO_CARD_RESET, 0xff));
	wait_ms(b, 50);
	STEP(ret, gpio(b, GPIO_D5, 0x00));
	wait_ms(b, 50);
	STEP(ret, gpio(b, GPIO_D5, 0xff));
	wait_ms(b, 100);
	if (ret)
		return ret;

	/* チューナーと復調器の初期化 */
	STEP(ret, tuner_send_one(b, 0xff));	/* 1 バイトだけ 0xff を送る (リセットと見られる) */
	for (i = 0; !ret && i < ARRAY_LEN(demod_init); i++)
		ret = reg_write(b, I2C_DEMOD, demod_init[i][0], demod_init[i][1]);
	STEP(ret, reg_read(b, I2C_DEMOD, TC_1C, &tc1c));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1C, (u8)((tc1c & 0xeb) | 0x28)));
	for (i = 0; !ret && i < ARRAY_LEN(tuner_init); i++)
		ret = tuner_send(b, tuner_init[i][0], tuner_init[i][1]);
	if (ret)
		return ret;

	/* ASIE5606 の電源とリセット */
	STEP(ret, gpio(b, GPIO_CARD_RESET, 0xff));
	wait_ms(b, 50);
	STEP(ret, gpio(b, GPIO_CARD_RESET, 0x00));
	wait_ms(b, 50);
	STEP(ret, gpio(b, GPIO_CARD_RESET, 0xff));
	wait_ms(b, 100);
	STEP(ret, reg_read(b, I2C_CARD, CARD_IDENTITY, &identity));
	if (ret) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "card interface IC does not respond (%d)", ret);
		return ret;
	}
	/* 下位 3 ビットが 6 の版は DES。それ以外の版 (XOR) の USB 転送の暗号は扱えない。 */
	if ((identity & 7) != 6) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "unexpected card interface IC version 0x%02x (not the DES version)", identity);
		return -ENODEV;
	}
	STEP(ret, hz2w_ctl_in(b, REQ_HIGH_SPEED, 0, 0, sys, 1));
	wait_ms(b, 30);

	/* USB 転送の TS にかける暗号の方式と、0x10..0x1f への 0 の書き込み。この普通の書き込みでは鍵は変わらないが、無いと TS が出てこない。 */
	STEP(ret, reg_write(b, I2C_CARD, CARD_TS_MODE, CARD_TS_MODE_DES));
	for (i = 0; !ret && i < 0x10; i++)
		ret = reg_write(b, I2C_CARD, (u8)(CARD_TS_KEY + i), 0x00);
	return ret;
}

int ut100b_tune(struct hz2w_bridge *b, unsigned int freq_khz)
{
	/* 1/64 MHz 単位。recfriio と同じ整数の MHz で指定する (実機で、この値で受信できることを確認した)。 */
	const u16 code = (u16)(((freq_khz + 500) / 1000) * 64);
	u8 tc1c = 0;
	int ret = 0;

	/* 選局の前の復調部の設定 */
	STEP(ret, reg_read(b, I2C_DEMOD, TC_1C, &tc1c));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1C, (u8)((tc1c & 0xfb) | 0x38)));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_AGC25, 0x00));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_AGC23, 0x4d));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1E, 0xaa));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1F, 0xa8));

	/* チューナーに周波数を書く */
	STEP(ret, tuner_send(b, TUNER_CONTROL, 0x00));
	STEP(ret, tuner_send(b, TUNER_13, 0x15));
	STEP(ret, tuner_send(b, TUNER_FREQ_LO, (u8)code));
	STEP(ret, tuner_send(b, TUNER_FREQ_HI, (u8)(code >> 8)));
	STEP(ret, tuner_send(b, TUNER_CONTROL, 0x02));

	/* 選局の後の復調部の設定 */
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1E, 0xa2));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_1F, 0x08));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_AGC1, 0x40));
	STEP(ret, reg_write(b, I2C_DEMOD, TC_AGC23, 0x4c));

	return ret;
}

/*
 * TS を流し始める。TS フィルター (チャンネル 0) をリセットし、入力と動作を有効にして、捨てる PID (ヌルパケットだけ) を決めてから、
 * 受信を始める。機器は、始めた時点から TS を機器の中の FIFO (約 260 KB) へ溜め、読み出し側が読まないと、FIFO が満杯のまま止まる。
 * このため、選局の途中や、読み出しを止めている間に始めてはならない (満杯の FIFO には、前のチャンネルの TS と選局の途中の乱れた
 * TS が残り、読み出しを再開した直後に、TEI が付いた約 690 パケットが出る)。ロックの確認と安定の待ちを済ませ、読み出し (URB) を
 * 始める直前に呼ぶ。
 */
int ut100b_stream_start(struct hz2w_bridge *b)
{
	u8 ctl = 0;
	int ret = 0;

	STEP(ret, filter_reset(b, 0, 1));
	STEP(ret, filter_read(b, FILTER_CONTROL, &ctl));
	STEP(ret, filter_write8(b, FILTER_CONTROL, (u8)(ctl | 0x0f)));
	STEP(ret, filter_write16(b, FILTER_XPID_MIN, 0x1fff));
	STEP(ret, filter_write16(b, FILTER_XPID_MAX, 0x1fff));
	STEP(ret, stream_request(b, REQ_STREAM_START, 0));
	/*
	 * 150 ms の待ちは、recfriio の選局 (Tune) が、受信を始めた直後に置いている待ちと同じ値。待ちの間も、機器は TS を FIFO へ溜める
	 * (約 15 Mbps では約 290 KB 分で、FIFO の容量 (約 260 KB とみられる) を超える計算になる)。
	 * それでも、読み出しの先頭から数えた連続性エラーは、次の試験で 0 件だった: Linux (libusb 版のホスト) の 7 チャンネル × 2 周の録画 (各録画の先頭から)、
	 * Linux カーネルモジュールの 7 チャンネル × 2 周 (各回の先頭から)。受信中の 10 回の切り替え (libusb 版) は、最後の区間だけ 5 件で、原因は未調査。
	 * 待ちを短くしたり、読み出しの後へ移したりした場合の動作は確認していないため、recfriio の手順のまま置いている。
	 */
	wait_ms(b, 150);
	return ret;
}

int ut100b_stream_stop(struct hz2w_bridge *b)
{
	int ret = 0;

	STEP(ret, stream_request(b, REQ_STREAM_STOP, 0));
	STEP(ret, filter_reset(b, 0, 1));
	return ret;
}

int ut100b_read_cndat(struct hz2w_bridge *b, u32 *cndat)
{
	u8 v[3];
	int ret = 0;

	STEP(ret, reg_read(b, I2C_DEMOD, TC_CNDAT, &v[0]));
	STEP(ret, reg_read(b, I2C_DEMOD, TC_CNDAT + 1, &v[1]));
	STEP(ret, reg_read(b, I2C_DEMOD, TC_CNDAT + 2, &v[2]));
	if (!ret)
		*cndat = ((u32)v[0] << 16) | ((u32)v[1] << 8) | v[2];
	return ret;
}

int ut100b_is_locked(struct hz2w_bridge *b, bool *locked)
{
	u8 v = 0;
	int ret;

	*locked = false;
	ret = reg_read(b, I2C_DEMOD, 0x80, &v);
	if (ret)
		return ret;
	if (v & 0x28)
		return 0;
	ret = reg_read(b, I2C_DEMOD, 0xb0, &v);
	if (ret)
		return ret;
	*locked = (v & 0x0f) >= 8;
	return 0;
}

void ut100b_board_close(struct hz2w_bridge *b)
{
	/*
	 * recfriio の UsbProcEnd と同じ順。この後に周期処理を止めても、機器は USB に残る (recfriio の終了後に確認)。
	 * 機器が既に無い場合の失敗は無視する。
	 */
	wait_ms(b, 100);
	stream_request(b, REQ_STREAM_STOP, 0);
	filter_reset(b, 0, 1);
	reg_write(b, I2C_CARD, CARD_80, 0x00);
	reg_write(b, I2C_CARD, CARD_04, 0x00);
	reg_write(b, I2C_CARD, CARD_CTRL, 0x00);
	wait_ms(b, 50);
	gpio(b, GPIO_D4, 0x00);
	gpio(b, GPIO_TUNER_POWER, GPIO_TUNER_POWER);
	wait_ms(b, 50);
}

void ut100b_card_init(struct hz2w_card *card, struct hz2w_bridge *bridge)
{
	hz2w_card_init(card, bridge);
	/* 受信中も使うため、ASIE5606 を初期値へ戻さない (TS の暗号の方式と鍵のレジスターが消える) */
	card->keep_ic_state = true;
}
