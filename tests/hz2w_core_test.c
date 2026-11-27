/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GV-MVP/HZ2W の共通層 (driver/common/hz2w) の自動試験。
 *
 * ASV5211 と ASIE5606B (1 系統目、0x4a) を模擬する。確認するのは、firmware 送信の区切り、起動時の GPIO と
 * 閉じる手順の要求列、ガード付きの読み書きの符号化、カードの電源投入 (ATR)、T=1 ブロックの交換、
 * 電源投入に失敗したときの復旧と、短い転送や状態の異常を成功として扱わないこと。
 * 実機の代用ではない (値の出どころは docs/devices/hz2w.md の実機の確認)。
 *
 * build: gcc -Wall -Wextra -I driver/common/hz2w tests/hz2w_core_test.c \
 *            driver/common/hz2w/hz2w_core.c driver/common/hz2w/hz2w_card.c -o hz2w_core_test
 */
#include "hz2w_core.h"
#include "hz2w_internal.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks, fails;

#define CHECK(c)                                                                  \
	do {                                                                      \
		checks++;                                                         \
		if (!(c)) {                                                       \
			fails++;                                                  \
			printf("FAIL %s:%d: %s\n", __func__, __LINE__, #c);       \
		}                                                                 \
	} while (0)

static const u8 guard_table[16] = {
	0x39, 0x72, 0x30, 0xc3, 0xad, 0xd4, 0x46, 0x36,
	0xba, 0xe5, 0x5f, 0x48, 0xae, 0x14, 0x06, 0xc1,
};

static const u8 atr[13] = { 0x3b, 0xf0, 0x12, 0x00, 0xff, 0x91, 0x81, 0xb1, 0x7c, 0x45, 0x1f, 0x03, 0x99 };

static u8 rev(u8 v)
{
	u8 r = 0;
	int i;

	for (i = 0; i < 8; i++)
		if (v & (1 << i))
			r |= (u8)(0x80 >> i);
	return r;
}

#define EV_MAX 4096

struct ev {
	u8 type, req;
	u16 value, index, len;
	u8 data[0x1000];
};

struct mock {
	struct ev *ev;
	int n;
	u64 now;
	/* 機器 */
	u8 reg[256];		/* 0x4a の ASIE5606B のレジスタ */
	u8 nonce[16];
	u8 nonce_serial;
	u8 gpio, gpio_ext;
	bool card_present;
	bool powered;
	bool power_fail;	/* reg0 = 9 で電源が入らない */
	bool rx_stuck;		/* 応答の長さが増え続ける */
	int short_after;	/* この番号の制御転送から短い転送を返す (0 は無効) */
	int status_bad;		/* 状態が 1 でない応答を返す番号 (0 は無効) */
	int calls;
	u8 fw[HZ2W_FIRMWARE_SIZE];
	u8 buf[256];		/* ASIE5606B のカードバッファ (反転済みのバイト)。レジスタ 0x40-0x7f が 64 バイトの窓で、reg 0x3a がページ */
	u8 long_resp[130];	/* 次の命令への応答 (ISO のバイト列)。長さが 0 でなければ、exec_command がこれを返す */
	u8 long_resp_len;
	u8 last_tx[256];	/* 直近の命令 (ISO のバイト列) */
	u8 buf_len;
	bool sblock_seen;
	int exec_count;
	int fw_fail_chunk;	/* この番号 (1 から) の firmware 転送が fw_fail_err で失敗する (0 は無効) */
	int fw_fail_err;
	int out_calls;
	u64 tick_per_call;	/* 制御転送 1 回ごとに進む時刻 (ms) */
	int short_reads;	/* reg3 の読み出しがこの回数まで、本当の長さより 1 小さい値を返す (受信の途中で止まる) */
	int atr_garbage_reads;	/* 電源投入の直後、reg3 の読み出しがこの回数まで、古い 4 バイトの内容を返す */
};

static struct ev *add_ev(struct mock *m, u8 type, u8 req, u16 value, u16 index, u16 len)
{
	struct ev *e;

	if (m->n >= EV_MAX)
		abort();
	e = &m->ev[m->n++];
	e->type = type;
	e->req = req;
	e->value = value;
	e->index = index;
	e->len = len;
	return e;
}

static u8 xor_lrc(const u8 *d, int n)
{
	u8 x = 0;
	int i;

	for (i = 0; i < n; i++)
		x ^= d[i];
	return x;
}

/* 命令の実行 (reg0 = 0x0a): バッファの ISO のブロックに応答する */
static void exec_command(struct mock *m)
{
	u8 frame[64];
	/* frame は S ブロック判定にだけ使う */
	int n = m->reg[0x02], i;
	u8 resp[8];

	m->exec_count++;
	for (i = 0; i < n; i++)
		m->last_tx[i] = frame[i < 64 ? i : 63] = rev(m->buf[i]);
	if (m->long_resp_len) {
		for (i = 0; i < m->long_resp_len; i++)
			m->buf[i] = rev(m->long_resp[i]);
		m->reg[0x03] = m->long_resp_len;
		return;
	}
	/* 応答: S ブロックの応答 (IFS) か、I ブロックへ SW 90 00 */
	if (n >= 5 && frame[1] == 0xc1) {
		resp[0] = 0x00; resp[1] = 0xe1; resp[2] = 0x01; resp[3] = frame[3];
		resp[4] = xor_lrc(resp, 4);
		m->buf_len = 5;
	} else {
		resp[0] = 0x00; resp[1] = 0x00; resp[2] = 0x02; resp[3] = 0x90; resp[4] = 0x00;
		resp[5] = xor_lrc(resp, 5);
		m->buf_len = 6;
	}
	for (i = 0; i < m->buf_len; i++)
		m->buf[i] = rev(resp[i]);
	m->reg[0x03] = m->rx_stuck ? 0xff : m->buf_len;
}

static int mock_control(void *priv, u8 rt, u8 req, u16 value, u16 index, void *buf, u16 len)
{
	struct mock *m = priv;
	u8 *b = buf;
	struct ev *e;
	int i;

	m->calls++;
	m->now += m->tick_per_call;
	e = add_ev(m, rt, req, value, index, len);
	if (rt == HZ2W_REQ_OUT) {
		if (m->fw_fail_chunk && ++m->out_calls == m->fw_fail_chunk)
			return m->fw_fail_err;
		if (len > sizeof(e->data))
			return -EINVAL;
		memcpy(e->data, buf, len);
		if ((u32)value + len <= HZ2W_FIRMWARE_SIZE)
			memcpy(m->fw + value, buf, len);
		return len;
	}
	memset(b, 0, len);
	if (m->short_after && m->calls >= m->short_after)
		return len > 1 ? len - 1 : 0;
	switch (req) {
	case 0x0c:
		b[0] = 1; b[3] = 0x04; b[4] = 0xbb; b[5] = 0x05; b[6] = 0x26;
		break;
	case 0x17:
		b[0] = 1; b[1] = 0x11; b[2] = 0x52;
		break;
	case 0x08:
	case 0x10: {
		u8 *g = req == 0x08 ? &m->gpio : &m->gpio_ext;
		u8 mask = (u8)(value >> 8), v = (u8)value;

		*g = (u8)((*g & ~mask) | (v & mask));
		b[0] = *g;
		break;
	}
	case 0x21:
	case 0x22:
	case 0x23:
		b[0] = 1;
		break;
	case 0x1a:
		for (i = 0; i < 16; i++)
			m->nonce[i] = (u8)(0x10 * 3 + i * 7 + ++m->nonce_serial);
		memcpy(b, m->nonce, 16);
		break;
	case 0x03:
	case 0x1c: {
		u8 reg = (u8)(value >> 8), v;

		if ((u8)value != 0x4a && !(req == 0x03 && (u8)value == 0x4c)) {
			b[0] = 0;
			break;
		}
		if (req == 0x1c) {
			u8 e_ = (u8)index, r = (u8)(index >> 8) ^ e_;

			v = (u8)(e_ ^ guard_table[r & 15] ^ m->nonce[r >> 4]);
		} else {
			v = (u8)index;
		}
		/* 長さ 1 の書き込みは成功と応答されるが、実機では電源が入らない */
		if (len < 2)
			b[0] = 1;
		else {
			b[0] = 1; b[1] = v;
		}
		if ((u8)value == 0x4c)
			break;
		if (reg >= 0x40) {
			m->buf[(m->reg[0x3a] & 1) * 64 + (reg - 0x40)] = v;
		} else if (len >= 2) {
			m->reg[reg] = v;
			if (reg == 0x00 && v == 0x09 && m->reg[0x04] == 0x80 && m->card_present && !m->power_fail) {
				m->powered = true;
				m->reg[0x04] = 0x81;
				memcpy(m->buf, atr, 13);
				for (i = 0; i < 13; i++)
					m->buf[i] = rev(m->buf[i]);
				m->reg[0x03] = 13;
				if (m->atr_garbage_reads) {
					u8 stale[4] = { 0xff, 0xff, 0x9e, 0xf8 };

					memcpy(m->buf + 64, m->buf, 13);	/* 本物は退避 (ページ 1) */
					memcpy(m->buf, stale, 4);
					m->reg[0x03] = 4;
				}
			} else if (reg == 0x00 && v == 0x0a) {
				exec_command(m);
			} else if (reg == 0x00 && v == 0x00 && m->reg[0x04] == 0x00 && m->reg[0x05] == 0xa0) {
				m->reg[0x04] = m->card_present ? 0x01 : 0x00;
				m->powered = false;
			} else if (reg == 0x04 && v == 0x00) {
				m->reg[0x04] = (u8)(m->card_present && m->reg[0x05] == 0xa0 ? 0x01 : 0x00);
				m->powered = false;
			}
		}
		break;
	}
	case 0x02: {
		u8 reg = (u8)(value >> 8);

		if ((u8)value == 0x4a || (u8)value == 0x4c) {
			b[0] = 1;
			b[1] = reg == 0x09 ? 0x06 : m->reg[reg];
		}
		break;
	}
	case 0x1b: {
		u8 reg = (u8)(value >> 8), v = reg >= 0x40 ? m->buf[(m->reg[0x3a] & 1) * 64 + (reg - 0x40)] : m->reg[reg], c = 0x5a;

		b[0] = 1;
		/* ベンダーの復号: c = b1 ^ b2、値 = b2 ^ guard[c & 15] ^ nonce[c >> 4] */
		b[2] = (u8)(v ^ guard_table[c & 15] ^ m->nonce[c >> 4]);
		b[1] = (u8)(b[2] ^ c);
		if (reg == 0x03 && m->atr_garbage_reads && m->reg[0x03] == 4 && --m->atr_garbage_reads == 0) {
			memcpy(m->buf, m->buf + 64, 13);
			m->reg[0x03] = 13;
			v = 13;
			b[2] = (u8)(v ^ guard_table[c & 15] ^ m->nonce[c >> 4]);
			b[1] = (u8)(b[2] ^ c);
		}
		if (reg == 0x03 && m->short_reads > 0 && v > 4) {
			m->short_reads--;
			v--;
			b[2] = (u8)(v ^ guard_table[c & 15] ^ m->nonce[c >> 4]);
			b[1] = (u8)(b[2] ^ c);
		}
		if (m->rx_stuck && reg == 0x03)
			m->reg[0x03] = (u8)(m->reg[0x03] + 1);
		break;
	}
	default:
		b[0] = 1;
		break;
	}
	if (m->status_bad && m->calls == m->status_bad && len)
		b[0] = 0;
	return len;
}

static void mock_sleep(void *priv, unsigned int ms)
{
	((struct mock *)priv)->now += ms;
}

static u64 mock_now(void *priv)
{
	return ((struct mock *)priv)->now;
}

static void mock_log(void *priv, int level, const char *fmt, ...)
{
	(void)priv; (void)level; (void)fmt;
}

static const struct hz2w_ops ops = { mock_control, mock_sleep, mock_now, mock_log };

static struct mock *new_mock(void)
{
	struct mock *m = calloc(1, sizeof(*m));

	m->ev = calloc(EV_MAX, sizeof(struct ev));
	m->gpio = 0xff;
	m->gpio_ext = 0x0f;
	m->card_present = true;
	m->reg[0x05] = 0x00;
	return m;
}

static void free_mock(struct mock *m)
{
	free(m->ev);
	free(m);
}

static int count_req(struct mock *m, u8 req)
{
	int i, n = 0;

	for (i = 0; i < m->n; i++)
		if (m->ev[i].req == req)
			n++;
	return n;
}

static void test_firmware(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	u8 fw[HZ2W_FIRMWARE_SIZE];
	int i;

	for (i = 0; i < HZ2W_FIRMWARE_SIZE; i++)
		fw[i] = (u8)(i * 31 + 7);
	hz2w_bridge_init(&b, &ops, m);
	CHECK(hz2w_fw_load(&b, fw, HZ2W_FIRMWARE_SIZE, HZ2W_BOOT_INDEX_MAIN) == 0);
	CHECK(m->n == 4);
	CHECK(m->ev[0].type == 0x40 && m->ev[0].req == 0xab && m->ev[0].value == 0x0000 && m->ev[0].len == 0xc00);
	CHECK(m->ev[1].req == 0xab && m->ev[1].value == 0x2000 && m->ev[1].len == 0x400);
	CHECK(m->ev[2].req == 0xab && m->ev[2].value == 0x2800 && m->ev[2].len == 0x1000);
	CHECK(m->ev[3].req == 0xac && m->ev[3].value == 0x3800 && m->ev[3].len == 0x800);
	for (i = 0; i < 4; i++)
		CHECK(m->ev[i].index == HZ2W_BOOT_INDEX_MAIN);
	CHECK(memcmp(m->ev[2].data, fw + 0x2800, 0x1000) == 0);
	/* 大きさの違う firmware は 1 回も送らない */
	m->n = 0;
	CHECK(hz2w_fw_load(&b, fw, HZ2W_FIRMWARE_SIZE - 1, HZ2W_BOOT_INDEX_MAIN) == -EINVAL);
	CHECK(hz2w_fw_load(&b, NULL, HZ2W_FIRMWARE_SIZE, HZ2W_BOOT_INDEX_MAIN) == -EINVAL);
	CHECK(m->n == 0);
	free_mock(m);
}

static void test_board_open_close(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	int i, first_gpio = -1, first_ext = -1;

	hz2w_bridge_init(&b, &ops, m);
	CHECK(hz2w_board_open(&b) == 0);
	for (i = 0; i < m->n; i++) {
		if (first_gpio < 0 && m->ev[i].req == 0x08)
			first_gpio = i;
		if (first_ext < 0 && m->ev[i].req == 0x10)
			first_ext = i;
	}
	/* 識別 (0x0c)、0x17、起動時の GPIO の順 */
	CHECK(m->ev[0].req == 0x0c && m->ev[0].len == 58);
	CHECK(m->ev[1].req == 0x17 && m->ev[1].value == 2);
	CHECK(first_gpio == 2 && m->ev[2].value == 0xfb07);
	CHECK(first_ext == 3 && m->ev[3].value == 0x0200);
	CHECK(m->gpio_ext == 0x0d);
	CHECK(count_req(m, 0x23) == 1);
	CHECK(count_req(m, 0x21) == 2);
	/* カード検出の準備の後、reg4 の bit0 がカード有りを表す */
	CHECK(m->reg[0x05] == 0xa0);
	{
		struct hz2w_card c;
		bool present = false;

		hz2w_card_init(&c, &b);
		CHECK(hz2w_card_detect(&c, &present) == 0 && present);
		m->card_present = false;
		m->reg[0x04] = 0;
		CHECK(hz2w_card_detect(&c, &present) == 0 && !present);
		/* カードが抜かれたら、受信済みで未読の応答は捨てる。電源の状態 (powered) は、あとの電源断のために残す。 */
		c.powered = true;
		c.rx_len = 4;
		c.rx_pos = 1;
		CHECK(hz2w_card_detect(&c, &present) == 0 && !present);
		CHECK(c.rx_len == 0 && c.rx_pos == 0 && c.powered);
		/* カードが有るときは、未読の応答を残す */
		m->card_present = true;
		m->reg[0x04] = 1;
		c.rx_len = 4;
		c.rx_pos = 1;
		CHECK(hz2w_card_detect(&c, &present) == 0 && present);
		CHECK(c.rx_len == 4 && c.rx_pos == 1);
	}
	/* 閉じる手順は、ベンダーの記録の順で最後が GPIO 0xd400、0x0808 */
	m->n = 0;
	hz2w_board_close(&b);
	CHECK(m->n == 10);
	CHECK(m->ev[0].req == 0x07 && m->ev[0].value == 0 && m->ev[1].req == 0x07 && m->ev[1].value == 1);
	CHECK(m->ev[2].req == 0x03 && (m->ev[2].value & 0xff) == 0x4c && (m->ev[2].value >> 8) == 0x80);
	CHECK(m->ev[3].req == 0x21 && m->ev[3].value == 0x0001);
	CHECK(m->ev[4].req == 0x03 && (m->ev[4].value & 0xff) == 0x4a && (m->ev[4].value >> 8) == 0x80);
	CHECK(m->ev[7].req == 0x21 && m->ev[7].value == 0);
	CHECK(m->ev[8].req == 0x08 && m->ev[8].value == 0xd400);
	CHECK(m->ev[9].req == 0x08 && m->ev[9].value == 0x0808);
	free_mock(m);
}

/* firmware 送信: 最後の転送の切断だけが再列挙の始まり。途中の失敗は元のエラーで返す */
static void test_firmware_failures(void)
{
	struct hz2w_bridge b;
	static u8 fw[HZ2W_FIRMWARE_SIZE];
	struct mock *m = new_mock();

	hz2w_bridge_init(&b, &ops, m);
	m->fw_fail_chunk = 4;
	m->fw_fail_err = -ENODEV;
	CHECK(hz2w_fw_load(&b, fw, HZ2W_FIRMWARE_SIZE, HZ2W_BOOT_INDEX_MAIN) == HZ2W_FW_REENUMERATING);
	free_mock(m);
	m = new_mock();
	hz2w_bridge_init(&b, &ops, m);
	m->fw_fail_chunk = 2;
	m->fw_fail_err = -EIO;
	CHECK(hz2w_fw_load(&b, fw, HZ2W_FIRMWARE_SIZE, HZ2W_BOOT_INDEX_MAIN) == -EIO);
	CHECK(m->n == 2);	/* 残りは送らない */
	free_mock(m);
	m = new_mock();
	hz2w_bridge_init(&b, &ops, m);
	m->fw_fail_chunk = 1;
	m->fw_fail_err = -ENODEV;
	CHECK(hz2w_fw_load(&b, fw, HZ2W_FIRMWARE_SIZE, HZ2W_BOOT_INDEX_MAIN) == -ENODEV);
	free_mock(m);
}

static void test_board_open_failures(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;

	hz2w_bridge_init(&b, &ops, m);
	/* 短い転送の応答を成功として扱わない */
	m->short_after = 1;
	CHECK(hz2w_board_open(&b) == -EIO);
	free_mock(m);
	/* 状態が 1 でない書き込み応答は失敗 */
	m = new_mock();
	hz2w_bridge_init(&b, &ops, m);
	m->status_bad = 11;	/* 版を読む 0x02 */
	CHECK(hz2w_board_open(&b) != 0);
	free_mock(m);
}

static void test_keepalive(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;

	hz2w_bridge_init(&b, &ops, m);
	CHECK(hz2w_keepalive_card_state(&b) == 0);
	CHECK(m->ev[0].req == 0x02 && m->ev[0].value == 0x044a && m->ev[0].len == 2);
	m->n = 0;
	CHECK(hz2w_keepalive_watchdog(&b) == 0);
	CHECK(m->n == 2 && m->ev[0].req == 0x22 && m->ev[0].value == 0 && m->ev[1].req == 0x22 && m->ev[1].value == 1);
	free_mock(m);
}

static void power_card(struct mock *m, struct hz2w_bridge *b, struct hz2w_card *c)
{
	hz2w_bridge_init(b, &ops, m);
	hz2w_card_init(c, b);
	CHECK(hz2w_board_open(b) == 0);
}

static void test_card_atr_and_t1(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	u8 out[64], len;
	bool ready;
	const u8 sblock[5] = { 0x00, 0xc1, 0x01, 0xfe, 0x3e };
	const u8 info[9] = { 0x00, 0x00, 0x05, 0x90, 0x32, 0x00, 0x00, 0x00, 0xa7 };
	int writes_before;

	power_card(m, &b, &c);
	/* 電源が入る前は書き込めない */
	CHECK(hz2w_card_write(&c, sblock, 5) == -ENODEV);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0);
	CHECK(m->powered && c.powered);
	CHECK(hz2w_card_rx_ready(&c, &ready) == 0 && ready);
	/* 小さな読み取りを重ねても、ATR が欠けず重ならない */
	len = 5;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 5 && memcmp(out, atr, 5) == 0);
	len = 32;
	CHECK(hz2w_card_read(&c, out + 5, &len) == 0 && len == 8 && memcmp(out + 5, atr + 5, 8) == 0);
	CHECK(hz2w_card_rx_ready(&c, &ready) == 0 && !ready);
	len = 8;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 0);
	/* S(IFS) と I ブロック。バッファはビットを反転して書かれる */
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	CHECK(m->exec_count == 1);
	CHECK(m->reg[0x02] == 5);
	CHECK(m->buf_len == 5);
	len = 16;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 5);
	CHECK(out[0] == 0x00 && out[1] == 0xe1 && out[2] == 0x01 && out[3] == 0xfe);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, info, 9) == 0);
	CHECK(m->reg[0x02] == 9);
	len = 16;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 6);
	CHECK(out[3] == 0x90 && out[4] == 0x00);
	/* 新しい命令は、前の応答の読み残しを捨てる */
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, info, 9) == 0);
	CHECK(c.rx_len == 6 && c.rx_pos == 0);
	/* 長さのない、大きすぎる命令は送らない */
	writes_before = m->n;
	CHECK(hz2w_card_write(&c, info, 0) == -EINVAL);
	CHECK(m->n == writes_before);
	/* 電源を切ると、reg4 が 0x01 (カード有り、電源 off) へ戻り、以後の書き込みは拒否される */
	CHECK(hz2w_card_power_down(&c) == 0);
	CHECK(!m->powered && m->reg[0x04] == 0x01 && !c.powered);
	CHECK(hz2w_card_write(&c, sblock, 5) == -ENODEV);
	/* 続けての電源投入も成功する */
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0);
	free_mock(m);
}

static void test_card_failures(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	const u8 sblock[5] = { 0x00, 0xc1, 0x01, 0xfe, 0x3e };

	power_card(m, &b, &c);
	/* カードが無いときは電源を入れない */
	m->card_present = false;
	m->reg[0x04] = 0x00;
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == -ENXIO);
	CHECK(!c.powered && !m->powered);
	/* 電源が入らないときは、期限で止まり、電源を落とした状態へ戻る */
	m->card_present = true;
	m->power_fail = true;
	m->reg[0x04] = 0x01;
	hz2w_card_set_timeout(&c, 600);
	CHECK(hz2w_card_reset(&c) == -ETIMEDOUT);
	CHECK(!c.powered && c.rx_len == 0);
	/* 復旧後は最初から電源を入れ直せる */
	m->power_fail = false;
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0 && c.powered);
	/* 応答の長さが落ち着かないときは期限で失敗し、読み残しを返さない */
	m->rx_stuck = true;
	hz2w_card_set_timeout(&c, 800);
	CHECK(hz2w_card_write(&c, sblock, 5) == -ETIMEDOUT);
	CHECK(c.rx_len == 0);
	m->rx_stuck = false;
	/*
	 * 受信の途中 (本当の長さより 1 小さい値) で長さが 2 回続けて同じになっても、LEN と合わないブロックは
	 * 完了とみなさず、読み直す。
	 */
	m->short_reads = 6;
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	CHECK(c.rx_len == 5 && m->short_reads == 0);
	/* 連鎖したブロック (実機の応答は 2 つのブロックが 1 つのバッファに続く) は、全体が揃えば成功する */
	{
		static const u8 chained[10] = { 0x00, 0x20, 0x02, 0x11, 0x22, 0x31, 0x00, 0x00, 0x00, 0x00 };

		memcpy(m->long_resp, chained, sizeof(chained));
		m->long_resp_len = 10;
		hz2w_card_set_timeout(&c, 5000);
		CHECK(hz2w_card_write(&c, sblock, 5) == 0);
		CHECK(c.rx_len == 10 && memcmp(c.rx, chained, 10) == 0);
	}
	/* ブロックをたどって長さがちょうどにならないときは、期限で失敗し、読み残しを返さない */
	m->long_resp_len = 7;
	memset(m->long_resp, 0, 8);	/* 4 バイトのブロックの次が 3 バイトで足りない */
	hz2w_card_set_timeout(&c, 300);
	CHECK(hz2w_card_write(&c, sblock, 5) == -ETIMEDOUT);
	CHECK(c.rx_len == 0);
	m->long_resp_len = 0;
	/* USB が切れた (転送の失敗) とき、状態を破棄する */
	hz2w_card_discard(&c);
	CHECK(!c.powered && !c.nonce_valid && c.rx_len == 0);
	CHECK(hz2w_card_write(&c, sblock, 5) == -ENODEV);
	free_mock(m);
}

static void test_guard_encoding(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	const u8 sblock[5] = { 0x00, 0xc1, 0x01, 0xfe, 0x3e };
	int i, guarded = 0;

	power_card(m, &b, &c);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0);
	m->n = 0;
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	/* 先頭は nonce の取得 (16 バイト)。書き込みは 0x1c (応答 2 バイト) と、1 つだけ通常の 0x03 */
	CHECK(m->ev[0].req == 0x1a && m->ev[0].len == 16);
	for (i = 0; i < m->n; i++) {
		if (m->ev[i].req == 0x1c) {
			CHECK(m->ev[i].len == 2);
			guarded++;
		}
		if (m->ev[i].req == 0x1b)
			CHECK(m->ev[i].len == 3);
	}
	CHECK(guarded == 1 + 4 + 5 + 1 - 0);	/* reg0=0x0c、0x02、0x38、0x3a、バッファ 5 バイト、reg0=0x0a。読み出し前の 0x3a も含め +1 */
	CHECK(count_req(m, 0x03) == 1);
	/* 電源の書き込み (reg4 = 0x80、0x00) は長さ 2 で送られる */
	for (i = 0; i < m->n; i++)
		if (m->ev[i].req == 0x03)
			CHECK(m->ev[i].len == 2);
	free_mock(m);
}

static void test_atr_stale(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	u8 out[33], len = sizeof(out);

	power_card(m, &b, &c);
	/* 受信の途中や前回の内容 (長さ 4 で安定している) は、ATR として受け付けない */
	m->atr_garbage_reads = 6;
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0);
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 13 && memcmp(out, atr, 13) == 0);
	/* 本物の ATR が最後まで来ないときは期限で失敗し、電源を落とす */
	hz2w_card_discard(&c);
	m->atr_garbage_reads = 1000000;
	hz2w_card_set_timeout(&c, 800);
	CHECK(hz2w_card_reset(&c) == -ETIMEDOUT);
	CHECK(!c.powered && c.rx_len == 0);
	free_mock(m);
}

static void test_card_paging(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	u8 tx[100], out[HZ2W_CARD_RX_MAX + 8], len;
	int i;

	power_card(m, &b, &c);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_reset(&c) == 0);
	len = 32;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 13);
	/* 64 バイトを超える応答 (65 バイトの応答の LRC は 2 ページ目にある)。バッファの 1 ページ目だけを読むと LRC が欠ける。 */
	m->long_resp_len = 65;
	for (i = 0; i < 64; i++)
		m->long_resp[i] = (u8)(i * 7 + 3);
	m->long_resp[2] = 65 - 4;	/* T=1 ブロックの LEN。長さは 4 + LEN で完結する */
	{
		u8 x = 0;

		for (i = 0; i < 64; i++)
			x ^= m->long_resp[i];
		m->long_resp[64] = x;
	}
	for (i = 0; i < 100; i++)
		tx[i] = (u8)(0x11 + i * 3);
	hz2w_card_set_timeout(&c, 5000);
	CHECK(hz2w_card_write(&c, tx, 100) == 0);
	/* 100 バイトの命令が、2 ページに分けて欠けなく書かれる */
	CHECK(m->reg[0x02] == 100);
	CHECK(memcmp(m->last_tx, tx, 100) == 0);
	CHECK(m->reg[0x3a] == 0);
	CHECK(c.rx_len == 65);
	CHECK(memcmp(c.rx, m->long_resp, 65) == 0);
	len = 100;
	CHECK(hz2w_card_read(&c, out, &len) == 0 && len == 65 && memcmp(out, m->long_resp, 65) == 0);
	/* 128 バイトまでは受け付け、それを超える命令は送らない */
	{
		u8 big[129] = { 0 };
		int before;

		m->long_resp_len = 128;
		m->long_resp[2] = 128 - 4;
		hz2w_card_set_timeout(&c, 5000);
		CHECK(hz2w_card_write(&c, big, 128) == 0 && c.rx_len == 128);
		before = m->n;
		CHECK(hz2w_card_write(&c, big, 129) == -EINVAL);
		CHECK(m->n == before);
		/* 128 バイトを超える応答は失敗にする (読み取りが途中で切れたまま成功にしない) */
		m->long_resp_len = 129;
		hz2w_card_set_timeout(&c, 5000);
		CHECK(hz2w_card_write(&c, tx, 10) == -EMSGSIZE);
		CHECK(c.rx_len == 0);
	}
	free_mock(m);
}

static void power_on(struct mock *m, struct hz2w_bridge *b, struct hz2w_card *c)
{
	power_card(m, b, c);
	hz2w_card_set_timeout(c, 5000);
	CHECK(hz2w_card_reset(c) == 0 && c->powered);
}

/* 期限は操作ごと。期限を渡さない操作は、前の操作の期限を引き継がない */
static void test_card_deadline(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	const u8 sblock[5] = { 0x00, 0xc1, 0x01, 0xfe, 0x3e };
	int before;

	power_on(m, &b, &c);
	CHECK(c.deadline_ms == 0);	/* 使い切っている */
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	m->now += 5000;		/* 次の操作までに十分な時間が空く */
	CHECK(hz2w_card_write(&c, sblock, 5) == 0);
	CHECK(c.deadline_ms == 0);
	/* 期限を過ぎてから命令を実行しない (転送のたびに時刻が進む) */
	m->tick_per_call = 60;
	before = m->exec_count;
	hz2w_card_set_timeout(&c, 300);
	CHECK(hz2w_card_write(&c, sblock, 5) == -ETIMEDOUT);
	CHECK(m->exec_count == before);
	CHECK(c.deadline_ms == 0 && c.rx_len == 0);
	free_mock(m);
}

/* 電源断が途中で失敗したら powered を残し、再試行で最後まで行う */
static void test_card_power_retry(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;
	struct hz2w_card c;
	int i, base;

	power_on(m, &b, &c);
	base = m->calls;
	m->status_bad = base + 1;	/* power_down 最初の nonce の取得 */
	m->short_after = base + 1;
	CHECK(hz2w_card_power_down(&c) != 0);
	CHECK(c.powered);
	m->short_after = 0;
	CHECK(hz2w_card_power_down(&c) == 0);
	CHECK(!c.powered && !m->powered);
	free_mock(m);
	/* どの転送が 1 回失敗しても、card の powered が機器の電源より先に false にならない */
	for (i = 1; i < 80; i++) {
		m = new_mock();
		power_card(m, &b, &c);
		m->calls = 0;
		m->status_bad = i;
		hz2w_card_set_timeout(&c, 5000);
		if (hz2w_card_reset(&c) != 0) {
			if (!c.powered)
				CHECK(!m->powered);
			/* 失敗の後も、続けてやり直せる */
			m->status_bad = 0;
			hz2w_card_set_timeout(&c, 5000);
			CHECK(hz2w_card_reset(&c) == 0 && c.powered && m->powered);
		}
		free_mock(m);
	}
}

/* リセットのパルスの転送が失敗したら、リセット成功として続けない */
static void test_ic_reset_failure(void)
{
	struct mock *m = new_mock();
	struct hz2w_bridge b;

	hz2w_bridge_init(&b, &ops, m);
	m->short_after = 2;	/* パルスの 2 つ目の GPIO から失敗 */
	CHECK(hz2w_card_ic_reset(&b) != 0);
	CHECK(count_req(m, 0x23) == 0 && count_req(m, 0x21) == 0);
	free_mock(m);
}

int main(void)
{
	test_firmware();
	test_firmware_failures();
	test_board_open_close();
	test_board_open_failures();
	test_keepalive();
	test_card_atr_and_t1();
	test_card_failures();
	test_guard_encoding();
	test_card_paging();
	test_atr_stale();
	test_card_deadline();
	test_card_power_retry();
	test_ic_reset_failure();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
