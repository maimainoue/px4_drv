/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 の共通層 (driver/common/dth11/dth11_core) の自動試験。
 *
 * 機器 (EVA01 ブリッジ) を模擬して、要求の列と符号化を確認する。確認するのは、機器情報 TLV の解析、
 * 起動の手順 (チューナーの起床が機器の起動より先であること)、選局、C/N の換算、周波数ワード、MxL の
 * 表の補正、TS の取り出し、ストリームと PID フィルタの符号化、転送の失敗を成功として扱わないこと。
 * 実機の代用ではない。期待値は DT-H11/U2 の実機 (ファームウェア 1.0.18) で受信できた手順の出力による。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_core_test.c \
 *            driver/common/dth11/dth11_core.c -o dth11_core_test
 */
#include "dth11_core.h"

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

/* DT-H11/U2 (シリアル 041101DA-110525N2088) の機器情報 TLV 全体 (139 バイト) */
static const u8 devinfo[139] = {
	0x81, 0x40, 0x88, 0x82, 0x84, 0x43, 0x6f, 0x44, 0x50, 0x83, 0x81, 0x01,
	0x90, 0x81, 0x01, 0x91, 0x82, 0x04, 0x11, 0x92, 0x82, 0x01, 0xda, 0x93,
	0x81, 0x01, 0x99, 0x89, 0x44, 0x54, 0x2d, 0x48, 0x31, 0x31, 0x2f, 0x55,
	0x32, 0x9c, 0x8c, 0x42, 0x75, 0x66, 0x66, 0x61, 0x6c, 0x6f, 0x20, 0x49,
	0x6e, 0x63, 0x2e, 0xd0, 0x8f, 0x50, 0x4e, 0x32, 0x30, 0x33, 0x31, 0x5f,
	0x4d, 0x78, 0x4c, 0x31, 0x33, 0x35, 0x52, 0x46, 0xd1, 0x81, 0x00, 0xd2,
	0x81, 0x00, 0xd3, 0x84, 0x00, 0x18, 0x80, 0x00, 0xd4, 0x84, 0x00, 0x10,
	0x00, 0x00, 0xd5, 0x84, 0x00, 0x08, 0x80, 0x00, 0xd7, 0x81, 0x60, 0xd8,
	0x81, 0x00, 0xde, 0x84, 0x00, 0xf4, 0x24, 0x00, 0xdf, 0x84, 0x00, 0x3d,
	0x09, 0x00, 0xe0, 0x82, 0x01, 0x02, 0xe1, 0x81, 0x04, 0x94, 0x94, 0x30,
	0x34, 0x31, 0x31, 0x30, 0x31, 0x44, 0x41, 0x2d, 0x31, 0x31, 0x30, 0x35,
	0x32, 0x35, 0x4e, 0x32, 0x30, 0x38, 0x38,
};

#define EV_MAX 4096

struct ev {
	u8 type, req;
	u16 value, index, len;
	u8 data[0x1000];
};

struct mock {
	struct ev *ev;
	int n;
	/* この番号 (0 起点) 以降の転送を失敗させる。負なら失敗させない */
	int fail_at;
	int fail_errno;
	/* 直近の転送に渡されたバッファ (OUT のデータが、呼び出し側の読み取り専用の領域でないことの確認) */
	const void *last_buf;
	u8 regs[256]; /* 復調器のレジスタの値 (reg 読み出し) */
	u8 tuner_reg[256];
	u8 tuner_sel;
};

static int m_control(void *priv, u8 type, u8 req, u16 value, u16 index, void *buf, u16 len)
{
	struct mock *m = priv;
	struct ev *e;
	int idx = m->n;

	if (m->n >= EV_MAX)
		return -ENOMEM;
	e = &m->ev[m->n++];
	m->last_buf = buf;
	e->type = type;
	e->req = req;
	e->value = value;
	e->index = index;
	e->len = len;
	if (!(type & 0x80) && len <= sizeof(e->data) && len)
		memcpy(e->data, buf, len);
	if (m->fail_at >= 0 && idx >= m->fail_at)
		return -m->fail_errno;
	if (type & 0x80) {
		memset(buf, 0, len);
		if (value == 0 && index == 0xc0de) {
			size_t n = len < sizeof(devinfo) ? len : sizeof(devinfo);

			memcpy(buf, devinfo, n);
			return (int)n;
		}
		if (value == 0 && index == 0) {
			static const u8 fw[16] = { 'E', 'V', 'A', '1', 'K', 'E', 'R', 'N', 0x12, 0, 0, 1, 0x34, 0, 0, 0x80 };
			size_t n = len < 16 ? len : 16;

			memcpy(buf, fw, n);
			return (int)n;
		}
		if ((value & 0xff00) == 0x2d00 && len == 1)
			*(u8 *)buf = m->regs[index & 0xff];
		if ((value & 0xff00) == 0x2c00 && index == 0x33 && len == 1)
			*(u8 *)buf = m->tuner_reg[m->tuner_sel];
		return len;
	}
	/* チューナーへの書き込み {FB, reg} は、続く 1 バイトの読み出しで返すレジスタの選択 */
	if ((value & 0xff00) == 0x2c00 && index == 0x30 && len == 2 && ((u8 *)buf)[0] == 0xfb)
		m->tuner_sel = ((u8 *)buf)[1];
	return len;
}

static void m_sleep(void *priv, unsigned int ms)
{
	(void)priv;
	(void)ms;
}

static u64 m_now(void *priv)
{
	(void)priv;
	return 0;
}

static const struct dth11_ops ops = { .control = m_control, .msleep = m_sleep, .now_ms = m_now, .log = NULL };

static void mock_init(struct mock *m)
{
	memset(m, 0, sizeof(*m));
	m->ev = calloc(EV_MAX, sizeof(struct ev));
	m->fail_at = -1;
	m->fail_errno = EIO;
}

static void mock_free(struct mock *m)
{
	free(m->ev);
}

static struct dth11_info info_ok(void)
{
	struct dth11_info info;

	CHECK(dth11_parse_info(devinfo, sizeof(devinfo), &info) == 0);
	return info;
}

static int find_ev(struct mock *m, u16 value, u16 index, int from)
{
	int i;

	for (i = from; i < m->n; i++)
		if (m->ev[i].value == value && m->ev[i].index == index)
			return i;
	return -1;
}

static void test_parse_info(void)
{
	struct dth11_info info;
	u8 bad[139];

	CHECK(dth11_parse_info(devinfo, sizeof(devinfo), &info) == 0);
	CHECK(!strcmp(info.model_name, "DT-H11/U2"));
	CHECK(!strcmp(info.maker, "Buffalo Inc."));
	CHECK(!strcmp(info.serial, "041101DA-110525N2088"));
	CHECK(!strcmp(info.kit_name, "PN2031_MxL135RF"));
	CHECK(info.id == 0 && info.flags == 0 && info.stream_param == 0);
	/* 復調器のアドレスが TLV に無いときは既定の 0x10 */
	CHECK(info.demod_addr == 0x10 && info.tuner_addr == 0x60);
	CHECK(info.gpio_mask_a == 0x100000 && info.gpio_mask_b == 0x088000 && info.gpio_mask_c == 0);
	CHECK(info.tuner_xtal_hz == 16000000 && info.if_freq_hz == 4000000);
	CHECK(info.hw_version == 0x102 && info.tuner_opts == 4);

	/* 外側が tag 1 でない */
	memcpy(bad, devinfo, sizeof(bad));
	bad[0] = 0x82;
	CHECK(dth11_parse_info(bad, sizeof(bad), &info) == -EPROTO);
	/* tag の bit7 が無い */
	memcpy(bad, devinfo, sizeof(bad));
	bad[0] = 0x01;
	CHECK(dth11_parse_info(bad, sizeof(bad), &info) == -EPROTO);
	CHECK(dth11_parse_info(devinfo, 0, &info) == -EPROTO);
	CHECK(dth11_parse_info(devinfo, 1, &info) == -EPROTO);
	/* 外側の長さに足りない切り詰め: 内側の列が途中で終わり、キット名が無いので別の機種として断る */
	CHECK(dth11_parse_info(devinfo, 20, &info) == -EPROTO);

	/* 別のキット名 (キット名の本体の 1 文字を変える) は、この層で扱わない */
	memcpy(bad, devinfo, sizeof(bad));
	bad[0x38] = 'X';
	CHECK(dth11_parse_info(bad, sizeof(bad), &info) == -ENODEV);
}

static void test_parse_info_rejects_unsupported(void)
{
	struct dth11_info info;
	u8 b[139];
	size_t i;

	/* IF の値 (4,000,000 = 00 3d 09 00) を 5,000,000 (00 4c 4b 40) に書き換える: 復調器の設定が合わないため断る */
	memcpy(b, devinfo, sizeof(b));
	for (i = 0; i + 3 < sizeof(b); i++)
		if (b[i] == 0x00 && b[i + 1] == 0x3d && b[i + 2] == 0x09 && b[i + 3] == 0x00)
			break;
	CHECK(i + 3 < sizeof(b));
	b[i + 1] = 0x4c;
	b[i + 2] = 0x4b;
	b[i + 3] = 0x40;
	CHECK(dth11_parse_info(b, sizeof(b), &info) == -ENOTSUP);
}

static void test_fw_version(void)
{
	struct mock m;
	struct dth11_bridge br;
	u8 ma = 0, mi = 0;
	u16 bu = 0;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_fw_version(&br, &ma, &mi, &bu) == 0);
	/* 0x01000012 の little endian → 1.0.18 */
	CHECK(ma == 1 && mi == 0 && bu == 18);
	CHECK(m.n == 1 && m.ev[0].type == 0xc0 && m.ev[0].req == 0x01 && m.ev[0].index == 0);
	mock_free(&m);
}

static void test_read_info_over_bridge(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_read_info(&br, &info) == 0);
	CHECK(m.n == 1 && m.ev[0].value == 0 && m.ev[0].index == 0xc0de && m.ev[0].len == 0x1000);
	CHECK(!strcmp(info.kit_name, "PN2031_MxL135RF"));

	m.n = 0;
	m.fail_at = 0;
	m.fail_errno = EPIPE;
	CHECK(dth11_read_info(&br, &info) == -EPIPE);
	mock_free(&m);
}

static void test_rf_tune(void)
{
	struct dth11_regval r[DTH11_MXL_RFTUNE_COUNT];
	/* 実機で受信できた Python の移植の出力 (557.142857 MHz = 27ch) */
	static const u8 exp557[20] = { 0x0f, 0x00, 0x0c, 0x15, 0x0d, 0x49, 0x0e, 0x8b, 0x1f, 0x87,
				       0x20, 0x1f, 0x21, 0x87, 0x22, 0x1f, 0x80, 0x41, 0x0f, 0x01 };
	size_t i;

	CHECK(dth11_rf_tune_word(557142857) == 0x8b49);
	CHECK(dth11_rf_tune_word(473142857) == 0x7649);
	CHECK(dth11_rf_tune_word(767142857) == 0xbfc9);
	CHECK(dth11_uhf_freq_hz(27) == 557142857);
	CHECK(dth11_uhf_freq_hz(13) == 473142857);
	CHECK(dth11_uhf_freq_hz(62) == 767142857);
	CHECK(dth11_uhf_freq_hz(12) == 0 && dth11_uhf_freq_hz(63) == 0);

	dth11_mxl_rftune_registers(557142857, 6000000, r);
	for (i = 0; i < DTH11_MXL_RFTUNE_COUNT; i++)
		CHECK(r[i].reg == exp557[2 * i] && r[i].val == exp557[2 * i + 1]);
	/* 333 MHz 未満では 0x80 のビット 6 を立てない */
	dth11_mxl_rftune_registers(200000000, 6000000, r);
	CHECK(r[8].reg == 0x80 && r[8].val == 0x01);
	/* 8 MHz 帯域は 0x0C の下位 6 ビットを立てる */
	dth11_mxl_rftune_registers(557142857, 8000000, r);
	CHECK(r[1].reg == 0x0c && r[1].val == 0x3f);
}

static void test_mxl_init(void)
{
	struct dth11_info info = info_ok();
	struct dth11_regval r[DTH11_MXL_INIT_COUNT];
	/* 実機で受信できた Python の移植の出力 */
	static const u8 exp[44] = { 0x04, 0x00, 0x02, 0x00, 0x03, 0x00, 0x05, 0x00, 0x06, 0x10, 0x07, 0x0a,
				    0x08, 0x00, 0x09, 0x3f, 0x0a, 0x3f, 0x0b, 0x3f, 0x2e, 0x15, 0x30, 0x10,
				    0x45, 0x58, 0x48, 0x19, 0x52, 0x03, 0x53, 0x44, 0x6a, 0x4b, 0x76, 0x00,
				    0x78, 0x18, 0x7a, 0x17, 0x85, 0x06, 0x01, 0x01 };
	size_t i;

	CHECK(dth11_mxl_init_registers(&info, r) == 0);
	for (i = 0; i < DTH11_MXL_INIT_COUNT; i++)
		CHECK(r[i].reg == exp[2 * i] && r[i].val == exp[2 * i + 1]);

	/* IF が 0 は拒む */
	info.if_freq_hz = 0;
	CHECK(dth11_mxl_init_registers(&info, r) == -EINVAL);
	/* 24 MHz の水晶は 0x03 の上位 4 ビット = 4、0x05 の下位 = 4 */
	info = info_ok();
	info.tuner_xtal_hz = 24000000;
	CHECK(dth11_mxl_init_registers(&info, r) == 0);
	CHECK(r[2].reg == 0x03 && r[2].val == 0x40 && r[3].reg == 0x05 && r[3].val == 0x04);
	/* スペクトルの反転 (IF が負) は 0x02 のビット 4 を立てる */
	info = info_ok();
	info.if_freq_hz = -4000000;
	CHECK(dth11_mxl_init_registers(&info, r) == 0);
	CHECK(r[1].reg == 0x02 && r[1].val == 0x10);
}

static void test_cnr(void)
{
	int prev;
	unsigned x;

	CHECK(dth11_cnr_tenth_db_from_raw(0) == 0);
	CHECK(dth11_cnr_tenth_db_from_raw(19648) == 0);
	CHECK(dth11_cnr_tenth_db_from_raw(0xffff) == 0);
	CHECK(dth11_cnr_tenth_db_from_raw(787) == 300);
	/* 表のしきい値ちょうどは、その行の C/N に丸めの項 (step / 2 の切り捨て) だけ足した値 */
	CHECK(dth11_cnr_tenth_db_from_raw(18187) == 10 - (0 - 1461 / 2) / 1461);
	CHECK(dth11_cnr_tenth_db_from_raw(788) == 300 - (0 - 52 / 2) / 52);
	/* 生値が小さいほど C/N は高い (単調) */
	prev = dth11_cnr_tenth_db_from_raw(19647);
	for (x = 19640; x >= 800; x -= 37) {
		int v = dth11_cnr_tenth_db_from_raw((u16)x);

		CHECK(v >= prev);
		prev = v;
	}
}

static void test_signal_strength(void)
{
	/* 純正の式: 1000 * (113 - ba2 - cc)。ba が 7 を超えるときは 4 ビットの符号つき */
	CHECK(dth11_signal_strength(0x3a, 3) == 1000 * (113 - 3 - 0x3a));
	CHECK(dth11_signal_strength(0x3a, 8) == 1000 * (113 - (int)(u8)(8 + 0xf0) - 0x3a));
}

static void test_start_order(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();
	int wake = -1, start, standby = -1, rst = -1, tbl = -1, d = -1, i;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_start(&br, &info) == 0);

	/* 最初は GPIO の有効化 (A | B)。ペイロードは {0, mask, mask}。A | B = 0x188000 */
	CHECK(m.ev[0].index == 0x1004 && m.ev[0].value == 0 && m.ev[0].len == 12);
	CHECK(m.ev[0].data[4] == 0x00 && m.ev[0].data[5] == 0x80 && m.ev[0].data[6] == 0x18 && m.ev[0].data[7] == 0x00);
	CHECK(!memcmp(m.ev[0].data + 8, m.ev[0].data + 4, 4));
	/* A | B を落とし、A (0x100000) を立て、B (0x088000) を立てる */
	CHECK(m.ev[1].index == 0x1006 && m.ev[1].data[8] == 0 && m.ev[1].data[9] == 0 && m.ev[1].data[10] == 0);
	CHECK(m.ev[2].index == 0x1006 && m.ev[2].data[6] == 0x10 && m.ev[2].data[10] == 0x10);
	CHECK(m.ev[3].index == 0x1006 && m.ev[3].data[5] == 0x80 && m.ev[3].data[9] == 0x80);

	/* OUT は bmRequestType 0x40 / bRequest 0x02、IN は 0xC0 / 0x01 */
	for (i = 0; i < m.n; i++) {
		if (m.ev[i].type == 0x40)
			CHECK(m.ev[i].req == 0x02);
		else
			CHECK(m.ev[i].type == 0xc0 && m.ev[i].req == 0x01);
	}

	/* 起動 (wIndex 0x10) は最後で、データ段なし */
	start = find_ev(&m, 0x0000, 0x0010, 0);
	CHECK(start == m.n - 1 && m.ev[start].len == 0 && m.ev[start].type == 0x40);
	/* チューナー (0x60) への最後の書き込みは起床 {01, 01} で、起動より前 */
	for (i = 0; i < start; i++)
		if (m.ev[i].value == 0x2c60 && m.ev[i].index == 0x30)
			wake = i;
	CHECK(wake >= 0 && wake < start);
	CHECK(m.ev[wake].len == 2 && m.ev[wake].data[0] == 0x01 && m.ev[wake].data[1] == 0x01);
	/* 起床の直前は、復調器のゲート (0x10 のレジスタ 0xFE ← 0) */
	CHECK(m.ev[wake - 1].value == 0x2d10 && m.ev[wake - 1].index == 0xfe && m.ev[wake - 1].data[0] == 0);
	/* 起床の前に、スタンバイ {01 00 0F 00}、リセット {FF}、44 バイトの初期化表の順で書いている */
	for (i = 0; i < wake; i++) {
		if (m.ev[i].value != 0x2c60)
			continue;
		if (m.ev[i].len == 4 && m.ev[i].data[0] == 0x01 && m.ev[i].data[2] == 0x0f)
			standby = i;
		if (m.ev[i].len == 1 && m.ev[i].data[0] == 0xff)
			rst = i;
		if (m.ev[i].len == 44)
			tbl = i;
	}
	CHECK(rst >= 0 && tbl > rst && standby > tbl && wake > standby);
	/* I2C のバッチは 256 バイトを超えず、1 件 6 バイト */
	for (i = 0; i < m.n; i++)
		if (m.ev[i].value == 0x2b00) {
			CHECK(m.ev[i].len <= 256 && m.ev[i].len % 6 == 0);
			CHECK(m.ev[i].index == 0);
		}
	/* 復調器の設定の最後は、0x6D ← 0x71、0x6E ← 0xE5 の直接書き込み */
	for (i = 0; i < start; i++)
		if (m.ev[i].value == 0x2d10 && m.ev[i].index == 0x6d)
			d = i;
	CHECK(d >= 0 && m.ev[d].data[0] == 0x71);
	CHECK(m.ev[d + 1].value == 0x2d10 && m.ev[d + 1].index == 0x6e && m.ev[d + 1].data[0] == 0xe5);
	mock_free(&m);
}

static int ts_select_value_in_config(const struct mock *m)
{
	int i, found = 0, value = -1;

	for (i = 0; i < m->n; i++) {
		u16 k;

		if (m->ev[i].value != 0x2b00)
			continue;
		/* 1 件は {0, 2, 0, addr << 1, reg, val}。{0x50, 0xD5} の直後の {0x51, 値} が TS 出力形式 */
		for (k = 0; k + 12 <= m->ev[i].len; k += 6)
			if (m->ev[i].data[k + 4] == 0x50 && m->ev[i].data[k + 5] == 0xd5 && m->ev[i].data[k + 10] == 0x51) {
				value = m->ev[i].data[k + 11];
				found++;
			}
	}
	return found == 1 ? value : -1;
}

static void test_ts_select_in_config(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();

	/* stream_param のビット 0 が 0 のとき、間接 0xD5 へシリアル TS (1) を書く */
	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_start(&br, &info) == 0);
	CHECK(ts_select_value_in_config(&m) == 1);
	mock_free(&m);

	/* ビット 0 が 1 のときは並列 TS (0) */
	info.stream_param = 1;
	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_start(&br, &info) == 0);
	CHECK(ts_select_value_in_config(&m) == 0);
	mock_free(&m);
}

static void test_tune_sequence(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();
	int i, rf = -1;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_tune(&br, &info, 557142857) == 0);

	/* 最初は 0x08 ← 01、続けて復調器への 6 バイトの生の書き込み */
	CHECK(m.ev[0].value == 0x2d10 && m.ev[0].index == 0x08 && m.ev[0].data[0] == 0x01);
	CHECK(m.ev[1].value == 0x2c10 && m.ev[1].index == 0x30 && m.ev[1].len == 6);
	CHECK(!memcmp(m.ev[1].data, "\x71\x00\x39\x01\x44\x00", 6));
	/* チューナーへの周波数設定 (20 バイト) */
	for (i = 0; i < m.n; i++)
		if (m.ev[i].value == 0x2c60 && m.ev[i].len == 20)
			rf = i;
	CHECK(rf >= 2);
	CHECK(m.ev[rf].data[4] == 0x0d && m.ev[rf].data[5] == 0x49 && m.ev[rf].data[6] == 0x0e && m.ev[rf].data[7] == 0x8b);
	/* その直前はゲート。後に 0x08 ← 00 がある */
	CHECK(m.ev[rf - 1].value == 0x2d10 && m.ev[rf - 1].index == 0xfe);
	CHECK(find_ev(&m, 0x2d10, 0x08, rf) > rf);
	/* 最後の 3 つ: 0x50 ← B1、0x51 ← 07、0x51 ← 00 */
	CHECK(m.ev[m.n - 3].index == 0x50 && m.ev[m.n - 3].data[0] == 0xb1);
	CHECK(m.ev[m.n - 2].index == 0x51 && m.ev[m.n - 2].data[0] == 0x07);
	CHECK(m.ev[m.n - 1].index == 0x51 && m.ev[m.n - 1].data[0] == 0x00);
	mock_free(&m);
}

static void test_signal(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();
	struct dth11_signal sig;
	int w1 = -1, w2 = -1, i;
	u8 s = 0;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);

	/* 同期状態 9 でロック。C/N の完了ビットが無ければ C/N は無効 */
	m.regs[0x0a] = 0xf9;
	m.regs[0x45] = 0x00;
	m.tuner_reg[0xcc] = 0x3a;
	m.tuner_reg[0xba] = 3;
	CHECK(dth11_read_signal(&br, &info, &sig) == 0);
	CHECK(sig.sync_state == 9 && sig.locked && !sig.valid_cnr);
	CHECK(sig.strength == 1000 * (113 - 3 - 0x3a));

	/* 同期状態 8 は未ロック */
	m.regs[0x0a] = 0x08;
	CHECK(dth11_read_signal(&br, &info, &sig) == 0);
	CHECK(sig.sync_state == 8 && !sig.locked);

	/* 完了ビットがあれば 0x46/0x47 から C/N を読み、測定を再開する (0x45 ← v | 0x10、v & 0x6F) */
	m.regs[0x0a] = 0x09;
	m.regs[0x45] = 0x40;
	m.regs[0x46] = 0x08;
	m.regs[0x47] = 0x00;
	m.n = 0;
	CHECK(dth11_read_signal(&br, &info, &sig) == 0);
	CHECK(sig.valid_cnr && sig.cnr_raw == 0x0800 && sig.cnr_tenth_db == dth11_cnr_tenth_db_from_raw(0x0800));
	CHECK(sig.cnr_tenth_db > 190 && sig.cnr_tenth_db < 210);
	for (i = 0; i < m.n; i++)
		if (m.ev[i].type == 0x40 && m.ev[i].value == 0x2d10 && m.ev[i].index == 0x45) {
			if (w1 < 0)
				w1 = i;
			else
				w2 = i;
		}
	CHECK(w1 >= 0 && w2 > w1);
	CHECK(m.ev[w1].data[0] == (0x40 | 0x10) && m.ev[w2].data[0] == (0x40 & 0x6f));

	CHECK(dth11_read_sync_state(&br, &info, &s) == 0 && s == 9);
	mock_free(&m);
}

static void test_stream_encoding(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();
	static const u16 pids[3] = { 0x0100, 0x0110, 0x01ff };
	static const u16 many[DTH11_PID_LIST_MAX + 1] = { 0 };

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);

	CHECK(dth11_stream_enable(&br, &info, DTH11_STREAM_DEVICE) == 0);
	CHECK(m.ev[0].value == 0xcbda && m.ev[0].index == 0x0002 && m.ev[0].len == 16);
	CHECK(m.ev[0].data[0] == 0 && m.ev[0].data[1] == 1 && m.ev[0].data[2] == 1 && m.ev[0].data[3] == 0xbc);
	CHECK(m.ev[0].data[10] == 0);
	info.id = 1;
	info.stream_param = 0x11223344;
	CHECK(dth11_stream_enable(&br, &info, DTH11_STREAM_DEVICE) == 0);
	CHECK(m.ev[1].data[4] == 0x44 && m.ev[1].data[7] == 0x11 && m.ev[1].data[10] == 2);

	CHECK(dth11_stream_disable(&br) == 0);
	CHECK(m.ev[2].len == 16 && m.ev[2].data[0] == 0 && m.ev[2].data[1] == 0 && m.ev[2].data[2] == 0 &&
	      m.ev[2].data[3] == 0xbc);

	CHECK(dth11_set_program_pids(&br, 0x400, pids, 3) == 0);
	CHECK(m.ev[3].value == 0xcbda && m.ev[3].index == 0x1000 && m.ev[3].len == 24);
	CHECK(m.ev[3].data[0] == 0x00 && m.ev[3].data[1] == 0x04 && m.ev[3].data[4] == 3 && m.ev[3].data[8] == 0);
	CHECK(m.ev[3].data[12] == 0x00 && m.ev[3].data[13] == 0x01 && m.ev[3].data[16] == 0x10 &&
	      m.ev[3].data[17] == 0x01);
	CHECK(m.ev[3].data[20] == 0xff && m.ev[3].data[21] == 0x01);
	/* 上限を超える PID リストは黙って切らずに断る (転送もしない) */
	CHECK(dth11_set_program_pids(&br, 0x400, many, DTH11_PID_LIST_MAX + 1) == -EINVAL);
	CHECK(m.n == 4);
	mock_free(&m);
}

static void test_extract_ts(void)
{
	static u8 in[256 * 5 + 100];
	u8 out[188 * 5];
	size_t n, i;

	memset(in, 0, sizeof(in));
	/* 0: 通常、1: 同期バイトなし (ストリーム開始直後の 0 のスロット)、2: 詰め物、3: 通常、4: TEI つき */
	in[0] = 0x47;
	in[1] = 0x01;
	in[2] = 0x00;
	in[187] = 0xaa;
	in[256] = 0x00;
	in[512] = 0x47;
	in[513] = 0x9f;
	in[514] = 0xff;
	in[768] = 0x47;
	in[769] = 0x01;
	in[770] = 0x11;
	in[768 + 188] = 0x77; /* 詰め物の領域は出力に入らない */
	in[1024] = 0x47;
	in[1025] = 0x81;
	in[1026] = 0x22;
	/* 末尾の 100 バイトは 256 に満たないので、同期バイトがあっても捨てる */
	in[1280] = 0x47;

	n = dth11_extract_ts(in, sizeof(in), out, true);
	CHECK(n == 188 * 3);
	CHECK(out[0] == 0x47 && out[187] == 0xaa);
	CHECK(out[188 + 2] == 0x11 && out[188 + 1] == 0x01);
	CHECK(out[2 * 188 + 1] == 0x81 && out[2 * 188 + 2] == 0x22);

	/* 詰め物も残す設定 */
	n = dth11_extract_ts(in, sizeof(in), out, false);
	CHECK(n == 188 * 4);
	for (i = 0; i < n; i += 188)
		CHECK(out[i] == 0x47);
	CHECK(dth11_extract_ts(in, 255, out, true) == 0);
	CHECK(dth11_extract_ts(in, 0, out, true) == 0);
}

/*
 * WinUSB は OUT のデータにも書き込み可能なメモリを要求する。読み取り専用の表や定数を渡しても、
 * 転送へは複写したバッファが渡る。
 */
static void test_out_uses_writable_copy(void)
{
	struct mock m;
	struct dth11_bridge br;
	static const u8 ro[4] = { 1, 2, 3, 4 };
	static const u8 big[513] = { 0 };
	struct dth11_info info = info_ok();

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_vendor_out(&br, 0x80, 0xdeaf, 1, ro, sizeof(ro)) == 0);
	CHECK(m.n == 1 && m.last_buf != (const void *)ro);
	CHECK(m.ev[0].req == 0x82 && m.ev[0].len == 4 && !memcmp(m.ev[0].data, ro, 4));
	/* 起動の転送 (データなし) も、チューナーへの定数の書き込みも、読み取り専用のポインターを渡さない */
	m.n = 0;
	CHECK(dth11_stream_disable(&br) == 0);
	CHECK(m.last_buf != NULL);
	/* 大きすぎる OUT は、転送せずに断る */
	m.n = 0;
	CHECK(dth11_vendor_out(&br, 0, 0xdeaf, 1, big, sizeof(big)) == -EINVAL);
	CHECK(m.n == 0);
	/* 0 バイトの OUT は成功する */
	CHECK(dth11_vendor_out(&br, 0, 0xdeaf, 0, NULL, 0) == 0);
	CHECK(m.n == 1 && m.ev[0].len == 0);
	(void)info;
	mock_free(&m);
}

/* どの転送が失敗しても、その負のエラーを呼び出し側へ返し、成功扱いにしない */
static void test_failures_propagate(void)
{
	struct mock m;
	struct dth11_bridge br;
	struct dth11_info info = info_ok();
	int total, k;

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_start(&br, &info) == 0);
	total = m.n;
	mock_free(&m);

	for (k = 0; k < total; k++) {
		mock_init(&m);
		m.fail_at = k;
		m.fail_errno = EPIPE;
		dth11_bridge_init(&br, &ops, &m);
		CHECK(dth11_start(&br, &info) == -EPIPE);
		/* 失敗した内容を次の転送へ持ち越さない */
		CHECK(br.batch_len == 0);
		mock_free(&m);
	}

	mock_init(&m);
	dth11_bridge_init(&br, &ops, &m);
	CHECK(dth11_tune(&br, &info, 557142857) == 0);
	total = m.n;
	mock_free(&m);
	for (k = 0; k < total; k++) {
		int r;

		mock_init(&m);
		m.fail_at = k;
		dth11_bridge_init(&br, &ops, &m);
		r = dth11_tune(&br, &info, 557142857);
		/* チューナーの D8 の読み出しは純正と同じく結果を捨てるので、失敗にならない転送もある */
		CHECK(r == -EIO || r == 0);
		mock_free(&m);
	}
}

int main(void)
{
	test_parse_info();
	test_parse_info_rejects_unsupported();
	test_fw_version();
	test_read_info_over_bridge();
	test_rf_tune();
	test_mxl_init();
	test_cnr();
	test_signal_strength();
	test_start_order();
	test_ts_select_in_config();
	test_tune_sequence();
	test_signal();
	test_stream_encoding();
	test_extract_ts();
	test_out_uses_writable_copy();
	test_failures_propagate();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
