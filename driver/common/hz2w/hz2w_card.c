// SPDX-License-Identifier: GPL-2.0-only
/*
 * hz2w_card.c - GV-MVP/HZ2W の内蔵カード (ASIE5606B の 1 系統目、I2C アドレス 0x4a)。
 *
 * ASIE5606B は UART ではなく、ISO 7816 のブロックを丸ごと受け渡す (バッファは reg 0x40 から、
 * ビット順が反転している)。このため、host/core/smart_card.cpp から見た UART の操作を次のように写す。
 *   ResetCard      電源投入と ATR の受信。ATR は受信バッファへ入れる。
 *   WriteCardData  T=1 のブロックを 1 つ送り、応答のブロックを受信バッファへ入れるまで待つ。
 *   ReadCardData   受信バッファの取り出し。
 * 手順と待ち時間は、ベンダーのドライバーの通信記録の再現で成功した値 (docs/devices/hz2w.md) による。
 */
#include "hz2w_core.h"
#include "hz2w_internal.h"

#define CARD_DEFAULT_TIMEOUT_MS 1000
/* 命令の実行後、ベンダーは 100〜150 ms 待ってから応答の長さを読む */
#define CARD_EXEC_WAIT_MS 120
#define CARD_POLL_MS 30
#define CARD_BUFFER_REG 0x40
#define CARD_REG_PAGE 0x3a
#define CARD_REG_CMD 0x00
#define CARD_REG_STATE 0x04
#define CARD_REG_LEN 0x03

/* coax の asv5211/card.go と同じガード表 */
static const u8 guard_table[16] = {
	0x39, 0x72, 0x30, 0xc3, 0xad, 0xd4, 0x46, 0x36,
	0xba, 0xe5, 0x5f, 0x48, 0xae, 0x14, 0x06, 0xc1,
};

static u8 reverse_bits(u8 v)
{
	v = (u8)(((v & 0xf0) >> 4) | ((v & 0x0f) << 4));
	v = (u8)(((v & 0xcc) >> 2) | ((v & 0x33) << 2));
	v = (u8)(((v & 0xaa) >> 1) | ((v & 0x55) << 1));
	return v;
}

void hz2w_card_init(struct hz2w_card *card, struct hz2w_bridge *bridge)
{
	memset(card, 0, sizeof(*card));
	card->bridge = bridge;
}

void hz2w_card_discard(struct hz2w_card *card)
{
	card->powered = false;
	card->nonce_valid = false;
	card->rx_len = 0;
	card->rx_pos = 0;
	card->deadline_ms = 0;
	memset(card->nonce, 0, sizeof(card->nonce));
}

void hz2w_card_set_timeout(struct hz2w_card *card, unsigned int timeout_ms)
{
	card->deadline_ms = card->bridge->ops->now_ms(card->bridge->priv) + timeout_ms;
}

/*
 * 期限は操作 (reset、write) 単位。set_timeout で渡された絶対期限は次の 1 回の操作で使い切り、
 * 渡されていなければ操作の始めに既定の待ちを設定する (前の操作の期限を引き継がない)。
 */
static void begin_operation(struct hz2w_card *card)
{
	if (!card->deadline_ms)
		card->deadline_ms = card->bridge->ops->now_ms(card->bridge->priv) + CARD_DEFAULT_TIMEOUT_MS;
}

static void end_operation(struct hz2w_card *card)
{
	card->deadline_ms = 0;
}

/* 絶対期限を過ぎていれば真 */
static bool expired(struct hz2w_card *card)
{
	return card->bridge->ops->now_ms(card->bridge->priv) >= card->deadline_ms;
}

static void sleep_ms(struct hz2w_card *card, unsigned int ms)
{
	card->bridge->ops->msleep(card->bridge->priv, ms);
}

static int wr(struct hz2w_card *card, u8 reg, u8 value)
{
	return hz2w_reg_write(card->bridge, HZ2W_ASIE_ADDR, reg, value);
}

static int rr(struct hz2w_card *card, u8 reg, u8 *value)
{
	return hz2w_reg_read(card->bridge, HZ2W_ASIE_ADDR, reg, value);
}

/* 要求 0x1a は呼ぶたびに新しい 16 バイトを返す。ガード付きの操作の前に取り直す。 */
static int new_nonce(struct hz2w_card *card)
{
	int ret = hz2w_ctl_in(card->bridge, 0x1a, 0, 0, card->nonce, sizeof(card->nonce));

	card->nonce_valid = !ret;
	return ret;
}

/*
 * ガード付きの書き込み。e = 値 ^ guard[r & 15] ^ nonce[r >> 4]、wIndex = e | (r ^ e) << 8。
 * r は任意の 1 バイトで、ベンダーはタイマーの値 ^ 0xcd を使う。応答の長さは 2 でなければ電源が入らない。
 */
static int guarded_write(struct hz2w_card *card, u8 reg, u8 value)
{
	u8 buf[2] = { 0, 0 };
	u8 r, e;
	int ret;

	if (!card->nonce_valid)
		return -EINVAL;
	r = (u8)((card->bridge->ops->now_ms(card->bridge->priv) ^ 0xcd) + card->guard_seed++);
	e = (u8)(value ^ guard_table[r & 15] ^ card->nonce[r >> 4]);
	ret = hz2w_ctl_in(card->bridge, 0x1c, (u16)(HZ2W_ASIE_ADDR | (reg << 8)),
			  (u16)(e | ((r ^ e) << 8)), buf, 2);
	if (ret)
		return ret;
	return buf[0] == 1 ? 0 : -EIO;
}

/* 応答は状態、b1、b2。c = b1 ^ b2 とし、値 = b2 ^ guard[c & 15] ^ nonce[c >> 4]。 */
static int guarded_read(struct hz2w_card *card, u8 reg, u8 *value)
{
	u8 buf[3] = { 0, 0, 0 };
	u8 c;
	int ret;

	if (!card->nonce_valid)
		return -EINVAL;
	ret = hz2w_ctl_in(card->bridge, 0x1b, (u16)(HZ2W_ASIE_ADDR | (reg << 8)), 0, buf, 3);
	if (ret)
		return ret;
	if (buf[0] != 1)
		return -EIO;
	c = buf[1] ^ buf[2];
	*value = (u8)(buf[2] ^ guard_table[c & 15] ^ card->nonce[c >> 4]);
	return 0;
}

int hz2w_card_detect(struct hz2w_card *card, bool *present)
{
	u8 state;
	int ret = rr(card, CARD_REG_STATE, &state);

	if (ret)
		return ret;
	/* bit0 がカード有り、bit7 が電源 on */
	*present = state & 0x01;
	return 0;
}

/* 応答の長さ (reg 0x03) が min_len 以上で、2 回続けて同じになるまで待つ (受信の途中は値が増える)。 */
static int wait_length(struct hz2w_card *card, u8 min_len, u8 *length)
{
	u8 prev = 0;
	bool have_prev = false;

	for (;;) {
		u8 n;
		int ret = guarded_read(card, CARD_REG_LEN, &n);

		if (ret)
			return ret;
		if (n >= min_len && have_prev && n == prev) {
			*length = n;
			return 0;
		}
		prev = n;
		have_prev = true;
		if (expired(card))
			return -ETIMEDOUT;
		sleep_ms(card, CARD_POLL_MS);
	}
}

/*
 * ATR の構造 (TS、T0、TAi/TBi/TCi/TDi、履歴バイト、T=0 以外を含むときの TCK) が、長さ len でちょうど完結していれば真。
 * 電源投入の直後は、応答の長さ (reg 0x03) が前回の値のままだったり、受信の途中の値だったりして、
 * 長さの安定だけでは ATR の完成を判定できないため、中身で確かめる。
 */
static bool atr_complete(const u8 *atr, u8 len)
{
	unsigned int pos = 2, y, historical;
	bool need_tck = false;

	if (len < 2 || (atr[0] != 0x3b && atr[0] != 0x3f))
		return false;
	y = atr[1] >> 4;
	historical = atr[1] & 0x0f;
	for (;;) {
		pos += (y & 1) + ((y >> 1) & 1) + ((y >> 2) & 1);
		if (!(y & 8))
			break;
		if (pos >= len)
			return false;
		if (atr[pos] & 0x0f)
			need_tck = true;
		y = atr[pos] >> 4;
		pos++;
	}
	return pos + historical + (need_tck ? 1 : 0) == len;
}

/* バッファ (reg 0x40 から) を読み、ビットを反転して受信バッファへ入れる。 */
static int fetch_buffer(struct hz2w_card *card, u8 length)
{
	unsigned int i;
	int ret;

	if (length > HZ2W_CARD_RX_MAX)
		return -EMSGSIZE;
	ret = guarded_write(card, CARD_REG_PAGE, 0x00);
	if (ret)
		return ret;
	for (i = 0; i < length; i++) {
		u8 v;

		/* 64 バイトを超える分は、ページを切り替えて同じ窓から読む (LRC は 65 バイト目以降にある) */
		if (i && i % HZ2W_CARD_PAGE_SIZE == 0) {
			ret = guarded_write(card, CARD_REG_PAGE, (u8)(i / HZ2W_CARD_PAGE_SIZE));
			if (ret)
				return ret;
		}
		ret = guarded_read(card, (u8)(CARD_BUFFER_REG + i % HZ2W_CARD_PAGE_SIZE), &v);
		if (ret)
			return ret;
		card->rx[i] = reverse_bits(v);
	}
	/* 次の読み書きがページ 0 から始まるよう戻す */
	if (length > HZ2W_CARD_PAGE_SIZE) {
		ret = guarded_write(card, CARD_REG_PAGE, 0x00);
		if (ret)
			return ret;
	}
	card->rx_len = length;
	card->rx_pos = 0;
	return 0;
}

int hz2w_card_power_down(struct hz2w_card *card)
{
	int ret;

	card->rx_len = 0;
	card->rx_pos = 0;
	if (!card->powered)
		return 0;
	/*
	 * ベンダーの終了: reg0 = 0x0c (ガード付き)、100 ms 待ち、reg 0x80、0x04、0x00 を 0 にする。
	 * powered は終了が最後まで済んでから落とす (途中で失敗したら、次の呼び出しが最初からやり直す)。
	 */
	ret = new_nonce(card);
	if (ret)
		return ret;
	ret = guarded_write(card, CARD_REG_CMD, 0x0c);
	if (ret)
		return ret;
	sleep_ms(card, 100);
	ret = wr(card, 0x80, 0x00);
	if (ret)
		return ret;
	ret = wr(card, CARD_REG_STATE, 0x00);
	if (ret)
		return ret;
	ret = wr(card, CARD_REG_CMD, 0x00);
	if (ret)
		return ret;
	card->powered = false;
	return 0;
}

static int card_reset(struct hz2w_card *card)
{
	u8 state = 0, length = 0;
	bool present;
	int ret;

	ret = hz2w_card_power_down(card);
	if (ret)
		return ret;
	/*
	 * 命令を実行した後の ASIE5606B は、電源を入れ直しても ATR を正しく受け取れない (reg1 を戻しても、
	 * 先頭が欠けたり文字化けしたりする)。リセットのパルスと要求 0x23、0x21 で初期値に戻してから電源を入れる
	 * (実機で、命令の実行と電源の入れ直しを 4 回繰り返して、毎回 ATR が完全に取れた)。
	 */
	if (!card->keep_ic_state) {
		ret = hz2w_card_ic_reset(card->bridge);
		if (ret)
			return ret;
		/* reg5 = 0xa0 を先に書かないと、カードの検出 (reg4 の bit0) が立たない。 */
		ret = wr(card, 0x05, 0xa0);
		if (ret)
			return ret;
	} else {
		/*
		 * ASIE5606 を戻せない場合 (QRS-UT100B の受信中) は、前のセッションの途中のやり取りが受け取りの入れ物に
		 * 残らないよう、準備の命令 (0x80) を出して、少し待ってから止める (recfriio の UT100B の手順)。
		 */
		ret = wr(card, CARD_REG_CMD, 0x80);
		if (ret)
			return ret;
		sleep_ms(card, 10);
		ret = wr(card, CARD_REG_CMD, 0x00);
		if (ret)
			return ret;
	}
	ret = wr(card, CARD_REG_STATE, 0x00);
	if (ret)
		return ret;
	ret = wr(card, CARD_REG_CMD, 0x00);
	if (ret)
		return ret;
	sleep_ms(card, 100);
	ret = hz2w_card_detect(card, &present);
	if (ret)
		return ret;
	if (!present)
		return -ENXIO;
	hz2w_gpio(card->bridge, false, 0xff, 0x01, NULL);
	ret = wr(card, CARD_REG_CMD, 0x08);
	if (!ret)
		ret = wr(card, CARD_REG_STATE, 0x80);
	if (ret)
		return ret;
	sleep_ms(card, 200);
	/* 0x09 を送った後は、失敗しても機器の電源が入っていることがあるため、終了処理へ進める */
	card->powered = true;
	ret = wr(card, CARD_REG_CMD, 0x09);
	if (ret)
		goto fail;
	sleep_ms(card, 60);
	/* 電源が入ると reg4 が 0x81 になり、reg3 に ATR の長さが入る (受信の途中は 1 で、完了すると増える)。 */
	ret = new_nonce(card);
	if (ret)
		goto fail;
	for (;;) {
		u8 reg39;

		ret = guarded_read(card, 0x39, &reg39);
		if (!ret)
			ret = guarded_read(card, CARD_REG_STATE, &state);
		if (ret)
			goto fail;
		if (state == 0x81)
			break;
		if (expired(card)) {
			ret = -ETIMEDOUT;
			goto fail;
		}
		sleep_ms(card, CARD_POLL_MS);
	}
	for (;;) {
		ret = guarded_read(card, CARD_REG_LEN, &length);
		if (ret)
			goto fail;
		if (length >= 2 && length <= 33) {
			ret = fetch_buffer(card, length);
			if (ret)
				goto fail;
			if (atr_complete(card->rx, length))
				return 0;
			card->rx_len = 0;
		}
		if (expired(card)) {
			ret = -ETIMEDOUT;
			goto fail;
		}
		sleep_ms(card, CARD_POLL_MS);
	}
fail:
	/* 途中で失敗したら電源を落とす。落とせなければ powered を残し、次の呼び出しが落とし直す。 */
	card->powered = true;
	hz2w_card_power_down(card);
	card->rx_len = 0;
	card->rx_pos = 0;
	return ret;
}

int hz2w_card_reset(struct hz2w_card *card)
{
	int ret;

	begin_operation(card);
	ret = card_reset(card);
	end_operation(card);
	return ret;
}

int hz2w_card_rx_ready(struct hz2w_card *card, bool *ready)
{
	*ready = card->rx_pos < card->rx_len;
	return 0;
}

int hz2w_card_read(struct hz2w_card *card, u8 *data, u8 *length)
{
	u8 remaining = (u8)(card->rx_len - card->rx_pos);
	u8 n = *length < remaining ? *length : remaining;

	memcpy(data, card->rx + card->rx_pos, n);
	card->rx_pos = (u8)(card->rx_pos + n);
	*length = n;
	return 0;
}

static int card_write(struct hz2w_card *card, const u8 *data, u8 length)
{
	u8 response_length = 0;
	unsigned int i;
	int ret;

	if (!card->powered)
		return -ENODEV;
	if (!length || length > HZ2W_CARD_RX_MAX)
		return -EINVAL;
	/* 前の応答の読み残しは新しい命令で無効になる */
	card->rx_len = 0;
	card->rx_pos = 0;
	ret = new_nonce(card);
	if (ret)
		return ret;
	if (expired(card))
		return -ETIMEDOUT;
	/* 命令: reg0 = 0x0c、reg1 = 0x01 (この 1 つだけ通常の書き込み)、長さ、バッファ、reg0 = 0x0a で実行 */
	ret = guarded_write(card, CARD_REG_CMD, 0x0c);
	if (!ret)
		ret = wr(card, 0x01, 0x01);
	if (!ret)
		ret = guarded_write(card, 0x02, length);
	if (!ret)
		ret = guarded_write(card, 0x38, 0x00);
	if (!ret)
		ret = guarded_write(card, CARD_REG_PAGE, 0x00);
	for (i = 0; !ret && i < length; i++) {
		if (i && i % HZ2W_CARD_PAGE_SIZE == 0)
			ret = guarded_write(card, CARD_REG_PAGE, (u8)(i / HZ2W_CARD_PAGE_SIZE));
		if (!ret)
			ret = guarded_write(card, (u8)(CARD_BUFFER_REG + i % HZ2W_CARD_PAGE_SIZE), reverse_bits(data[i]));
	}
	/* 実行の前にページ 0 へ戻す (ベンダーは先頭のページを指した状態で実行する) */
	if (!ret && length > HZ2W_CARD_PAGE_SIZE)
		ret = guarded_write(card, CARD_REG_PAGE, 0x00);
	/* 期限を過ぎた命令はカードで実行させない */
	if (!ret && expired(card))
		ret = -ETIMEDOUT;
	if (!ret)
		ret = guarded_write(card, CARD_REG_CMD, 0x0a);
	if (ret)
		goto fail;
	sleep_ms(card, CARD_EXEC_WAIT_MS);
	/* ブロックの最小長は NAD、PCB、LEN、EDC の 4 バイト */
	ret = wait_length(card, 4, &response_length);
	if (!ret)
		ret = fetch_buffer(card, response_length);
	if (ret)
		goto fail;
	return 0;
fail:
	card->rx_len = 0;
	card->rx_pos = 0;
	return ret;
}

int hz2w_card_write(struct hz2w_card *card, const u8 *data, u8 length)
{
	int ret;

	begin_operation(card);
	ret = card_write(card, data, length);
	end_operation(card);
	return ret;
}
