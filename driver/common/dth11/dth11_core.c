// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_core.c - BUFFALO DT-H11/U2 (EVA01 ブリッジ + PN2031 + MxL135RF) の共通層
 *
 * 手順は i4M1k0SU/DTH10-series-driver (MIT License、Copyright (c) i4M1k0SU) の
 * crates/dth10-core/src/{device,pn2031,mxl13xrf,tlv}.rs による。
 */

#include "dth11_core.h"
#include "dth11_tables.h"

/* ---- ベンダー要求の符号 (wValue の上位バイトが種別) ---- */

#define DTH11_REQ_IN_DATA 0x01 /* bRequest: IN のデータ段 */
#define DTH11_REQ_OUT_DATA 0x02 /* bRequest: OUT のデータ段 (データなしの命令も OUT) */

#define VAL_SYS 0x0000
#define IDX_DEVICE_INFO 0xc0de
#define IDX_FW_VERSION 0x0000
#define IDX_START 0x0010
#define IDX_GPIO_ENABLE 0x1004
#define IDX_GPIO_SET 0x1006

#define VAL_I2C_BATCH 0x2b00
#define VAL_I2C_RAW 0x2c00
#define IDX_I2C_RAW_WRITE 0x0030
#define IDX_I2C_RAW_READ 0x0033
#define VAL_I2C_REG 0x2d00

#define VAL_CBDA 0xcbda
#define IDX_STREAM_CTRL 0x0002
#define IDX_PID_FILTER 0x1000

#define DEMOD_ADDR_DEFAULT 0x10
/* 復調器の I2C ゲート。チューナーへの転送の直前に毎回 0 を書く */
#define DEMOD_GATE_REG 0xfe

/* 復調器は IF 4 MHz 固定の設定を持つ。機器情報の IF が異なると受信できない */
#define DTH11_CONFIG_IF_HZ 4000000

#define TLV_TAG_CONTAINER 1

#define INFO_FLAG_TUNER_DIRECT_I2C 0x08
#define INFO_FLAG_SKIP_INIT_CONFIG 0x20

/* 選局の前処理で復調器へ 1 回の書き込みで送る 3 組のレジスタ値 (0x71 ← 00、0x39 ← 01、0x44 ← 00) */
static const u8 dth11_tune_raw[6] = { 0x71, 0x00, 0x39, 0x01, 0x44, 0x00 };

/* 復調器の C/N 表: (生値のしきい値、C/N (0.1 dB)、次のしきい値との差)。生値が大きいほど C/N が低い */
static const struct {
	int threshold;
	int cn;
	int step;
} dth11_cn_table[] = {
	{ 18187, 10, 1461 }, { 16534, 20, 1653 }, { 14823, 30, 1711 }, { 13161, 40, 1662 },
	{ 11622, 50, 1539 }, { 10279, 60, 1343 }, { 9089, 70, 1190 }, { 8042, 80, 1047 },
	{ 7137, 90, 905 }, { 6342, 100, 795 }, { 5641, 110, 701 }, { 5030, 120, 611 },
	{ 4474, 130, 556 }, { 3988, 140, 486 }, { 3556, 150, 432 }, { 3180, 160, 376 },
	{ 2841, 170, 339 }, { 2541, 180, 300 }, { 2276, 190, 265 }, { 2038, 200, 238 },
	{ 1800, 210, 238 }, { 1625, 220, 175 }, { 1462, 230, 163 }, { 1324, 240, 138 },
	{ 1175, 250, 149 }, { 1063, 260, 112 }, { 980, 270, 83 }, { 907, 280, 73 },
	{ 840, 290, 67 }, { 788, 300, 52 },
};

/* 生値がこれ以上のとき C/N は 0 と報告する */
#define DTH11_CN_RAW_LIMIT 19648

#define LOG(br, lvl, ...) \
	do { \
		if ((br)->ops->log) \
			(br)->ops->log((br)->priv, (lvl), __VA_ARGS__); \
	} while (0)

/* ---- 制御転送 ---- */

void dth11_bridge_init(struct dth11_bridge *br, const struct dth11_ops *ops, void *priv)
{
	memset(br, 0, sizeof(*br));
	br->ops = ops;
	br->priv = priv;
}

static void dth11_sleep(struct dth11_bridge *br, unsigned int ms)
{
	br->ops->msleep(br->priv, ms);
}

/*
 * OUT のデータは、書き込み可能な一時バッファへ複写してから転送へ渡す。WinUSB は OUT のデータにも
 * 書き込み可能なメモリを要求し、読み取り専用のデータ (static const の表や定数) を渡すと失敗する
 * (実機の WinUSB で確認)。1 回の OUT は 512 バイトまで (最大は PID リストの 268 バイト)。
 */
#define DTH11_OUT_MAX 512

static int dth11_control_out(struct dth11_bridge *br, u8 request, u16 value, u16 index, const void *data, u16 len)
{
	u8 tmp[DTH11_OUT_MAX];
	int r;

	if (len > sizeof(tmp))
		return -EINVAL;
	if (len)
		memcpy(tmp, data, len);
	r = br->ops->control(br->priv, DTH11_REQ_OUT, request, value, index, tmp, len);
	if (r < 0)
		return r;
	return r == len ? 0 : -EIO;
}

static int dth11_out(struct dth11_bridge *br, u16 value, u16 index, const void *data, u16 len)
{
	return dth11_control_out(br, DTH11_REQ_OUT_DATA, value, index, data, len);
}

static int dth11_in(struct dth11_bridge *br, u16 value, u16 index, void *buf, u16 len)
{
	int r = br->ops->control(br->priv, DTH11_REQ_IN, DTH11_REQ_IN_DATA, value, index, buf, len);

	if (r < 0)
		return r;
	return r;
}

static int dth11_in_exact(struct dth11_bridge *br, u16 value, u16 index, void *buf, u16 len)
{
	int r = dth11_in(br, value, index, buf, len);

	if (r < 0)
		return r;
	return r == len ? 0 : -EIO;
}

int dth11_vendor_out(struct dth11_bridge *br, u8 path_flags, u16 value, u16 index, const void *data, u16 len)
{
	return dth11_control_out(br, (u8)((path_flags & 0xfc) | DTH11_REQ_OUT_DATA), value, index, data, len);
}

int dth11_vendor_in(struct dth11_bridge *br, u8 path_flags, u16 value, u16 index, void *buf, u16 len)
{
	return br->ops->control(br->priv, DTH11_REQ_IN, (u8)((path_flags & 0xfc) | DTH11_REQ_IN_DATA), value, index, buf,
				len);
}

/* ---- I2C ---- */

/*
 * バッチは、他の I2C 転送より先に送り出す。送信の失敗でも待ち行列は空にして、
 * 失敗した内容を次の転送へ持ち越さない。
 */
static int dth11_batch_flush(struct dth11_bridge *br)
{
	int r;

	if (!br->batch_len)
		return 0;
	r = dth11_out(br, VAL_I2C_BATCH, 0, br->batch, br->batch_len);
	br->batch_len = 0;
	return r;
}

static int dth11_batch_append(struct dth11_bridge *br, u8 addr, u8 reg, u8 val)
{
	u8 *p;

	/* 1 件は {0, データ長 + 1, 0, addr << 1, reg, data} の 6 バイト */
	if (br->batch_len + 6 > DTH11_BATCH_MAX)
		return -ENOSPC;
	p = br->batch + br->batch_len;
	p[0] = 0;
	p[1] = 2;
	p[2] = 0;
	p[3] = (u8)(addr << 1);
	p[4] = reg;
	p[5] = val;
	br->batch_len += 6;
	return 0;
}

static int dth11_batch_pairs(struct dth11_bridge *br, u8 addr, const struct dth11_regval *pairs, size_t count)
{
	size_t i;
	int r;

	for (i = 0; i < count; i++) {
		r = dth11_batch_append(br, addr, pairs[i].reg, pairs[i].val);
		if (r) {
			br->batch_len = 0;
			return r;
		}
	}
	return dth11_batch_flush(br);
}

static int dth11_reg_write(struct dth11_bridge *br, u8 addr, u8 reg, u8 val)
{
	int r = dth11_batch_flush(br);

	if (r)
		return r;
	return dth11_out(br, VAL_I2C_REG | addr, reg, &val, 1);
}

static int dth11_reg_read(struct dth11_bridge *br, u8 addr, u8 reg, u8 *val)
{
	int r = dth11_batch_flush(br);

	if (r)
		return r;
	return dth11_in_exact(br, VAL_I2C_REG | addr, reg, val, 1);
}

static int dth11_raw_write(struct dth11_bridge *br, u8 addr, const u8 *data, u16 len)
{
	int r = dth11_batch_flush(br);

	if (r)
		return r;
	return dth11_out(br, VAL_I2C_RAW | addr, IDX_I2C_RAW_WRITE, data, len);
}

static int dth11_raw_read(struct dth11_bridge *br, u8 addr, u8 *buf, u16 len)
{
	int r = dth11_batch_flush(br);

	if (r)
		return r;
	return dth11_in_exact(br, VAL_I2C_RAW | addr, IDX_I2C_RAW_READ, buf, len);
}

/* ---- 機器情報 ---- */

struct tlv_entry {
	u8 tag;
	const u8 *data;
	size_t len;
};

/*
 * TLV の 1 件を読む。tag は bit7 が立っていなければならず、長さは 0x80|n (n < 128) か
 * 0x40|hi, lo の形。長さ 0 の件は純正の解析器と同じく列の終わりとして扱う。
 * 戻り値は消費した長さ、列の終わりは 0、不正は -EPROTO。
 */
static int tlv_next(const u8 *p, size_t remain, struct tlv_entry *e)
{
	size_t hdr, len;

	if (remain < 2)
		return 0;
	if (!(p[0] & 0x80))
		return -EPROTO;
	e->tag = p[0] & 0x7f;
	if (p[1] & 0x80) {
		len = p[1] & 0x7f;
		hdr = 2;
	} else if (p[1] & 0x40) {
		if (remain < 3)
			return 0;
		len = ((size_t)(p[1] & 0x3f) << 8) | p[2];
		hdr = 3;
	} else {
		return -EPROTO;
	}
	if (remain < hdr + len)
		return 0;
	if (!len)
		return 0;
	e->data = p + hdr;
	e->len = len;
	return (int)(hdr + len);
}

static u32 tlv_be_int(const struct tlv_entry *e)
{
	u32 v = 0;
	size_t i;

	for (i = 0; i < e->len && i < 4; i++)
		v = (v << 8) | e->data[i];
	return v;
}

/* NUL 終端の固定長の文字列へ写す */
static void tlv_string(const struct tlv_entry *e, char *dst, size_t max)
{
	size_t n = e->len < max - 1 ? e->len : max - 1;
	size_t i;

	for (i = 0; i < n && e->data[i]; i++)
		dst[i] = (char)e->data[i];
	dst[i] = '\0';
}

int dth11_parse_info(const u8 *blob, size_t len, struct dth11_info *info)
{
	struct tlv_entry top, e;
	const u8 *p;
	size_t remain;
	int n;

	memset(info, 0, sizeof(*info));
	n = tlv_next(blob, len, &top);
	if (n <= 0 || top.tag != TLV_TAG_CONTAINER)
		return -EPROTO;

	p = top.data;
	remain = top.len;
	while ((n = tlv_next(p, remain, &e)) > 0) {
		p += n;
		remain -= n;
		switch (e.tag) {
		case 0x19:
			tlv_string(&e, info->model_name, sizeof(info->model_name));
			break;
		case 0x1c:
			tlv_string(&e, info->maker, sizeof(info->maker));
			break;
		case 0x14:
			tlv_string(&e, info->serial, sizeof(info->serial));
			break;
		case 0x50:
			tlv_string(&e, info->kit_name, sizeof(info->kit_name));
			info->present |= DTH11_PRESENT_KIT_NAME;
			break;
		case 0x51:
			info->id = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_ID;
			break;
		case 0x52:
			info->flags = tlv_be_int(&e);
			break;
		case 0x54:
			info->gpio_mask_a = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_GPIO_A;
			break;
		case 0x55:
			info->gpio_mask_b = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_GPIO_B;
			break;
		case 0x56:
			info->demod_addr = e.data[0];
			break;
		case 0x57:
			info->tuner_addr = e.data[0];
			break;
		case 0x58:
			info->stream_param = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_STREAM_PARAM;
			break;
		case 0x5e:
			info->tuner_xtal_hz = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_XTAL;
			break;
		case 0x5f:
			info->if_freq_hz = (s32)tlv_be_int(&e);
			info->present |= DTH11_PRESENT_IF;
			break;
		case 0x60:
			info->hw_version = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_HW_VERSION;
			break;
		case 0x61:
			info->tuner_opts = tlv_be_int(&e);
			info->present |= DTH11_PRESENT_OPTS;
			break;
		case 0x63:
			info->gpio_mask_c = tlv_be_int(&e);
			break;
		case 0x64:
			info->gpio_c_open = tlv_be_int(&e);
			break;
		case 0x66:
			info->gpio_c_close = tlv_be_int(&e);
			break;
		default:
			break;
		}
	}
	if (n < 0)
		return n;

	/* この層が扱うのは PN2031 + MxL135RF のキットだけ。他のキットを同じ手順で動かさない */
	if (!(info->present & DTH11_PRESENT_KIT_NAME) || strcmp(info->kit_name, "PN2031_MxL135RF"))
		return -ENODEV;
	if (!info->demod_addr)
		info->demod_addr = DEMOD_ADDR_DEFAULT;
	if (!info->tuner_addr)
		return -EPROTO;
	/* 復調器の設定は IF 4 MHz 固定。別の値では受信できないため、使い始める前に断る */
	if (info->if_freq_hz != DTH11_CONFIG_IF_HZ)
		return -ENOTSUP;
	/* チューナーを復調器のゲートの外に置く構成と、初期化を開く時に回す構成は未対応 */
	if (info->flags & (INFO_FLAG_TUNER_DIRECT_I2C | INFO_FLAG_SKIP_INIT_CONFIG))
		return -ENOTSUP;
	return 0;
}

int dth11_read_info(struct dth11_bridge *br, struct dth11_info *info)
{
	u8 blob[0x1000];
	int r = dth11_in(br, VAL_SYS, IDX_DEVICE_INFO, blob, sizeof(blob));

	if (r < 0)
		return r;
	return dth11_parse_info(blob, (size_t)r, info);
}

int dth11_fw_version(struct dth11_bridge *br, u8 *major, u8 *minor, u16 *build)
{
	u8 buf[16];
	u32 v;
	int r = dth11_in(br, VAL_SYS, IDX_FW_VERSION, buf, sizeof(buf));

	if (r < 0)
		return r;
	if (r < 12)
		return -EPROTO;
	/* 8 バイト目からの little endian の u32。major.minor.build = (v >> 24).((v >> 16) & 0xff).(v & 0xffff) */
	v = (u32)buf[8] | ((u32)buf[9] << 8) | ((u32)buf[10] << 16) | ((u32)buf[11] << 24);
	*major = (u8)(v >> 24);
	*minor = (u8)((v >> 16) & 0xff);
	*build = (u16)(v & 0xffff);
	return 0;
}

/* ---- GPIO ---- */

static int dth11_gpio(struct dth11_bridge *br, u16 index, u32 mask, u32 value)
{
	u8 p[12] = { 0 };

	p[4] = (u8)mask;
	p[5] = (u8)(mask >> 8);
	p[6] = (u8)(mask >> 16);
	p[7] = (u8)(mask >> 24);
	p[8] = (u8)value;
	p[9] = (u8)(value >> 8);
	p[10] = (u8)(value >> 16);
	p[11] = (u8)(value >> 24);
	return dth11_out(br, VAL_SYS, index, p, sizeof(p));
}

/* 電源投入: A と B を落として 2 ms、A を立てて 15 ms、B を立てて 10 ms */
static int dth11_power_up(struct dth11_bridge *br, const struct dth11_info *info)
{
	u32 a = info->gpio_mask_a, b = info->gpio_mask_b, c = info->gpio_mask_c;
	int r;

	r = dth11_gpio(br, IDX_GPIO_ENABLE, a | b | c, a | b | c);
	if (r)
		return r;
	r = dth11_gpio(br, IDX_GPIO_SET, a | b, 0);
	if (r)
		return r;
	dth11_sleep(br, 2);
	r = dth11_gpio(br, IDX_GPIO_SET, a, a);
	if (r)
		return r;
	dth11_sleep(br, 15);
	r = dth11_gpio(br, IDX_GPIO_SET, b, b);
	if (r)
		return r;
	dth11_sleep(br, 10);
	return 0;
}

/* ---- 復調器 (PN2031) ---- */

/* 間接レジスタ 0xD5 に書く TS 出力形式。stream_param のビット 0 が 0 のときシリアル (1) */
static u8 dth11_ts_select(u32 stream_param)
{
	return (u8)(~stream_param & 1);
}

static int dth11_demod_configure(struct dth11_bridge *br, const struct dth11_info *info)
{
	const u8 a = info->demod_addr;
	size_t i, k, n;
	static const size_t coef_batches[3] = { 8, 8, 9 };
	int r;

	r = dth11_reg_write(br, a, 0x70, 0x00);
	if (r)
		return r;
	dth11_sleep(br, 1);
	r = dth11_reg_write(br, a, 0x70, 0xff);
	if (r)
		return r;

#define BATCH(t) \
	do { \
		r = dth11_batch_pairs(br, a, t, sizeof(t) / sizeof((t)[0])); \
		if (r) \
			return r; \
	} while (0)

	BATCH(dth11_cfg_base);
	BATCH(dth11_cfg_window04_a);
	BATCH(dth11_cfg_window04_b);
	BATCH(dth11_cfg_ind_a7);
	BATCH(dth11_cfg_ind_dc);
	BATCH(dth11_cfg_ind_b0);

	/* TS 出力形式の値を挟む部分は、前後を 1 つのバッチで送る */
	for (i = 0; i < sizeof(dth11_cfg_ts_head) / sizeof(dth11_cfg_ts_head[0]); i++) {
		r = dth11_batch_append(br, a, dth11_cfg_ts_head[i].reg, dth11_cfg_ts_head[i].val);
		if (r)
			goto fail;
	}
	r = dth11_batch_append(br, a, 0x51, dth11_ts_select(info->stream_param));
	if (r)
		goto fail;
	BATCH(dth11_cfg_ts_tail);

	BATCH(dth11_cfg_ind_1e);
	BATCH(dth11_cfg_words_a);

	/* 24 ビットのワードは、インデックスを 0x28、値を 0x29..0x2B (上位から) へ 1 バイトずつ書く */
	k = 0;
	for (n = 0; n < 3; n++) {
		for (i = 0; i < coef_batches[n]; i++, k++) {
			u32 w = dth11_coef_words[k].value;

			if (dth11_batch_append(br, a, 0x28, dth11_coef_words[k].index) ||
			    dth11_batch_append(br, a, 0x29, (u8)(w >> 16)) ||
			    dth11_batch_append(br, a, 0x2a, (u8)(w >> 8)) ||
			    dth11_batch_append(br, a, 0x2b, (u8)w)) {
				r = -ENOSPC;
				goto fail;
			}
		}
		r = dth11_batch_flush(br);
		if (r)
			return r;
	}
#undef BATCH

	r = dth11_reg_write(br, a, 0x6d, 0x71);
	if (r)
		return r;
	return dth11_reg_write(br, a, 0x6e, 0xe5);

fail:
	br->batch_len = 0;
	return r;
}

static int dth11_demod_restart_cn(struct dth11_bridge *br, const struct dth11_info *info)
{
	u8 v;
	int r = dth11_reg_read(br, info->demod_addr, 0x45, &v);

	if (r)
		return r;
	r = dth11_reg_write(br, info->demod_addr, 0x45, v | 0x10);
	if (r)
		return r;
	return dth11_reg_write(br, info->demod_addr, 0x45, v & 0x6f);
}

/* ---- チューナー (MxL135RF)。復調器の I2C ゲートの先 ---- */

static int dth11_tuner_write(struct dth11_bridge *br, const struct dth11_info *info, const u8 *data, u16 len)
{
	int r = dth11_reg_write(br, info->demod_addr, DEMOD_GATE_REG, 0x00);

	if (r)
		return r;
	return dth11_raw_write(br, info->tuner_addr, data, len);
}

static int dth11_tuner_read_reg(struct dth11_bridge *br, const struct dth11_info *info, u8 reg, u8 *val)
{
	const u8 sel[2] = { 0xfb, reg };
	int r = dth11_tuner_write(br, info, sel, sizeof(sel));

	if (r)
		return r;
	r = dth11_reg_write(br, info->demod_addr, DEMOD_GATE_REG, 0x00);
	if (r)
		return r;
	return dth11_raw_read(br, info->tuner_addr, val, 1);
}

/* 表の (レジスタ, 値) の並びを、レジスタと値を交互に並べたバイト列にして送る */
static int dth11_tuner_write_regs(struct dth11_bridge *br, const struct dth11_info *info,
				  const struct dth11_regval *regs, size_t count)
{
	u8 buf[2 * DTH11_MXL_INIT_COUNT];
	size_t i;

	if (count > DTH11_MXL_INIT_COUNT)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		buf[2 * i] = regs[i].reg;
		buf[2 * i + 1] = regs[i].val;
	}
	return dth11_tuner_write(br, info, buf, (u16)(2 * count));
}

static void mxl_patch(struct dth11_regval *tab, size_t count, u8 reg, u8 and_mask, u8 or_bits)
{
	size_t i;

	for (i = 0; i < count; i++)
		if (tab[i].reg == reg)
			tab[i].val = (u8)((tab[i].val & and_mask) | or_bits);
}

int dth11_mxl_init_registers(const struct dth11_info *info, struct dth11_regval *out)
{
	const size_t n = DTH11_MXL_INIT_COUNT;
	u32 if_hz;
	bool inverted;
	bool opt1;

	memcpy(out, dth11_mxl_init_table, n * sizeof(*out));

	/* 1. ハードウェアの版 → レジスタ 0x06 */
	if (info->present & DTH11_PRESENT_HW_VERSION) {
		if (info->hw_version == 0x101)
			mxl_patch(out, n, 0x06, 0xf2, 0x12);
		else if (info->hw_version == 0x102)
			mxl_patch(out, n, 0x06, 0xf0, 0x10);
	}
	/* 2. IF の周波数 → レジスタ 0x02。負は周波数スペクトルの反転 */
	if (info->present & DTH11_PRESENT_IF) {
		inverted = info->if_freq_hz < 0;
		if_hz = inverted ? (u32)(-(info->if_freq_hz + 1)) + 1 : (u32)info->if_freq_hz;
		if (!if_hz)
			return -EINVAL;
		switch (if_hz) {
		case 4000000:
			mxl_patch(out, n, 0x02, 0xf0, 0x00);
			break;
		case 5000000:
			mxl_patch(out, n, 0x02, 0xf4, 0x04);
			break;
		case 6000000:
			mxl_patch(out, n, 0x02, 0xf6, 0x06);
			break;
		case 44000000:
			mxl_patch(out, n, 0x02, 0xfb, 0x0b);
			break;
		default:
			break;
		}
		if (inverted)
			mxl_patch(out, n, 0x02, 0xff, 0x10);
		else
			mxl_patch(out, n, 0x02, 0xef, 0x00);
	}
	/* 3. 水晶の周波数 → レジスタ 0x03 の上位 4 ビットと 0x05 の下位 4 ビット */
	if (info->present & DTH11_PRESENT_XTAL) {
		switch (info->tuner_xtal_hz) {
		case 16000000:
			mxl_patch(out, n, 0x03, 0x0f, 0x00);
			mxl_patch(out, n, 0x05, 0xf0, 0x00);
			break;
		case 24000000:
			mxl_patch(out, n, 0x03, 0x0f, 0x40);
			mxl_patch(out, n, 0x05, 0xf4, 0x04);
			break;
		case 27000000:
			mxl_patch(out, n, 0x03, 0x0f, 0x70);
			mxl_patch(out, n, 0x05, 0xf7, 0x07);
			break;
		case 32000000:
			mxl_patch(out, n, 0x03, 0x0f, 0x90);
			mxl_patch(out, n, 0x05, 0xf9, 0x09);
			break;
		default:
			break;
		}
	}
	/* 4. オプションのビット 0 → レジスタ 0x03 のビット 3 (クロック出力) */
	if (info->tuner_opts & 0x1)
		mxl_patch(out, n, 0x03, 0xff, 0x08);
	else
		mxl_patch(out, n, 0x03, 0xf7, 0x00);
	/* 5. オプションのビット 1 または機器の flags のビット 0 → レジスタ 0x04 のビット 0 */
	opt1 = (info->tuner_opts & 0x2) || (info->flags & 0x1);
	mxl_patch(out, n, 0x04, 0xfe, opt1 ? 0x01 : 0x00);
	/* 6. オプションのビット 2 → レジスタ 0x08 のビット 7 (立っているとき下げる) */
	if (info->tuner_opts & 0x4)
		mxl_patch(out, n, 0x08, 0x7f, 0x00);
	else
		mxl_patch(out, n, 0x08, 0xff, 0x80);
	return 0;
}

u16 dth11_rf_tune_word(u32 freq_hz)
{
	u32 dig = freq_hz / 1000000;
	u32 rem = freq_hz % 1000000;
	u32 div = 1000000;
	int i;

	for (i = 0; i < 6; i++) {
		div >>= 1;
		dig <<= 1;
		if (rem > div) {
			rem -= div;
			dig++;
		}
	}
	/* 1/64 MHz の端数を四捨五入 */
	if (rem > 7812)
		dig++;
	return (u16)dig;
}

void dth11_mxl_rftune_registers(u32 freq_hz, u32 bw_hz, struct dth11_regval *out)
{
	const size_t n = DTH11_MXL_RFTUNE_COUNT;
	u16 w;

	memcpy(out, dth11_mxl_rftune_table, n * sizeof(*out));
	if (bw_hz == 6000000) {
		mxl_patch(out, n, 0x0c, 0xd5, 0x15);
	} else if (bw_hz == 8000000) {
		mxl_patch(out, n, 0x0c, 0xff, 0x3f);
	}
	w = dth11_rf_tune_word(freq_hz);
	mxl_patch(out, n, 0x0d, 0x00, (u8)(w & 0xff));
	mxl_patch(out, n, 0x0e, 0x00, (u8)(w >> 8));
	if (freq_hz >= 333000000)
		mxl_patch(out, n, 0x80, 0xff, 0x40);
}

int dth11_signal_strength(u8 cc, u8 ba)
{
	/* ba が 7 を超えるときは 4 ビットの符号つき整数として扱う */
	int ba2 = ba > 7 ? (int)(u8)(ba + 0xf0) : (int)ba;

	return 1000 * (113 - ba2 - (int)cc);
}

int dth11_cnr_tenth_db_from_raw(u16 raw)
{
	int x = raw;
	size_t i;

	if (x == 0 || x >= DTH11_CN_RAW_LIMIT)
		return 0;
	for (i = 0; i < sizeof(dth11_cn_table) / sizeof(dth11_cn_table[0]); i++) {
		int t = dth11_cn_table[i].threshold;
		int step = dth11_cn_table[i].step;

		/* 純正の式は C の整数の除算 (0 方向への切り捨て) で、その結果と一致する */
		if (x >= t)
			return dth11_cn_table[i].cn - (10 * (x - t) - step / 2) / step;
	}
	return 300;
}

/* ---- 起動と選局 ---- */

static int dth11_tuner_init(struct dth11_bridge *br, const struct dth11_info *info)
{
	struct dth11_regval regs[DTH11_MXL_INIT_COUNT];
	static const u8 reset = 0xff;
	int r;

	r = dth11_mxl_init_registers(info, regs);
	if (r)
		return r;
	r = dth11_tuner_write(br, info, &reset, 1);
	if (r)
		return r;
	return dth11_tuner_write_regs(br, info, regs, DTH11_MXL_INIT_COUNT);
}

static int dth11_tuner_standby(struct dth11_bridge *br, const struct dth11_info *info)
{
	static const u8 data[4] = { 0x01, 0x00, 0x0f, 0x00 };

	return dth11_tuner_write(br, info, data, sizeof(data));
}

static int dth11_tuner_wake(struct dth11_bridge *br, const struct dth11_info *info)
{
	static const u8 data[2] = { 0x01, 0x01 };

	return dth11_tuner_write(br, info, data, sizeof(data));
}

int dth11_start(struct dth11_bridge *br, const struct dth11_info *info)
{
	int r;

	r = dth11_power_up(br, info);
	if (r)
		return r;
	r = dth11_demod_configure(br, info);
	if (r)
		return r;
	r = dth11_tuner_init(br, info);
	if (r)
		return r;
	r = dth11_tuner_standby(br, info);
	if (r)
		return r;

	/* 起動 (wIndex 0x0010) の後の最初のチューナー書き込みは、この機体で要求が拒否される (実機で確認)。起床を先にする */
	r = dth11_tuner_wake(br, info);
	if (r)
		return r;
	if (info->gpio_mask_c) {
		r = dth11_gpio(br, IDX_GPIO_SET, info->gpio_mask_c, info->gpio_c_open);
		if (r)
			return r;
	}
	return dth11_out(br, VAL_SYS, IDX_START, NULL, 0);
}

int dth11_stop(struct dth11_bridge *br, const struct dth11_info *info)
{
	int r = 0;

	if (info->gpio_mask_c)
		r = dth11_gpio(br, IDX_GPIO_SET, info->gpio_mask_c, info->gpio_c_close);
	if (r)
		return r;
	return dth11_tuner_standby(br, info);
}

u32 dth11_uhf_freq_hz(int channel)
{
	if (channel < 13 || channel > 62)
		return 0;
	/* ARIB STD-B31: 13ch の中心は 473 MHz + 1/7 MHz、以降は 6 MHz 間隔 */
	return 473000000u + 6000000u * (u32)(channel - 13) + 142857u;
}

int dth11_tune(struct dth11_bridge *br, const struct dth11_info *info, u32 freq_hz)
{
	struct dth11_regval regs[DTH11_MXL_RFTUNE_COUNT];
	const u8 a = info->demod_addr;
	static const struct dth11_regval post_5f[2] = { { 0x5f, 0x00 }, { 0x5f, 0x07 } };
	u8 dummy;
	int r;

	r = dth11_reg_write(br, a, 0x08, 0x01);
	if (r)
		return r;
	r = dth11_raw_write(br, a, dth11_tune_raw, sizeof(dth11_tune_raw));
	if (r)
		return r;

	dth11_mxl_rftune_registers(freq_hz, 6000000, regs);
	r = dth11_tuner_write_regs(br, info, regs, DTH11_MXL_RFTUNE_COUNT);
	if (r)
		return r;
	dth11_sleep(br, 3);
	/* 純正は同期の確認としてレジスタ 0xD8 を 2 回読んで値を捨てる。読み出しの失敗は選局の失敗にしない */
	(void)dth11_tuner_read_reg(br, info, 0xd8, &dummy);
	(void)dth11_tuner_read_reg(br, info, 0xd8, &dummy);

	r = dth11_reg_write(br, a, 0x08, 0x00);
	if (r)
		return r;
	r = dth11_demod_restart_cn(br, info);
	if (r)
		return r;
	r = dth11_reg_write(br, a, 0x53, 0x00);
	if (r)
		return r;
	r = dth11_reg_write(br, a, 0x53, 0x07);
	if (r)
		return r;
	r = dth11_batch_pairs(br, a, post_5f, 2);
	if (r)
		return r;
	r = dth11_reg_write(br, a, 0x50, 0xb1);
	if (r)
		return r;
	r = dth11_reg_write(br, a, 0x51, 0x07);
	if (r)
		return r;
	return dth11_reg_write(br, a, 0x51, 0x00);
}

int dth11_read_sync_state(struct dth11_bridge *br, const struct dth11_info *info, u8 *sync_state)
{
	u8 v;
	int r = dth11_reg_read(br, info->demod_addr, 0x0a, &v);

	if (r)
		return r;
	*sync_state = v & 0x0f;
	return 0;
}

int dth11_read_signal(struct dth11_bridge *br, const struct dth11_info *info, struct dth11_signal *sig)
{
	u8 v, hi, lo, cc, ba;
	int r;

	memset(sig, 0, sizeof(*sig));
	r = dth11_read_sync_state(br, info, &sig->sync_state);
	if (r)
		return r;
	sig->locked = sig->sync_state >= DTH11_SYNC_LOCKED;

	/* C/N は復調器が周期で測る。完了のビット (0x45 の bit6) が立っているときだけ読み、測定を再開する */
	r = dth11_reg_read(br, info->demod_addr, 0x45, &v);
	if (r)
		return r;
	if (v & 0x40) {
		r = dth11_reg_read(br, info->demod_addr, 0x46, &hi);
		if (r)
			return r;
		r = dth11_reg_read(br, info->demod_addr, 0x47, &lo);
		if (r)
			return r;
		r = dth11_demod_restart_cn(br, info);
		if (r)
			return r;
		sig->cnr_raw = (u16)(((u16)hi << 8) | lo);
		sig->cnr_tenth_db = dth11_cnr_tenth_db_from_raw(sig->cnr_raw);
		sig->valid_cnr = true;
	}

	r = dth11_tuner_read_reg(br, info, 0xcc, &cc);
	if (r)
		return r;
	r = dth11_tuner_read_reg(br, info, 0xba, &ba);
	if (r)
		return r;
	sig->strength = dth11_signal_strength(cc, ba);
	return 0;
}

/* ---- ストリーム ---- */

int dth11_stream_enable(struct dth11_bridge *br, const struct dth11_info *info, enum dth11_stream_mode mode)
{
	u8 p[16] = { 0 };

	p[0] = (u8)mode;
	p[1] = 1;
	p[2] = 1;
	p[3] = 0xbc;
	p[4] = (u8)info->stream_param;
	p[5] = (u8)(info->stream_param >> 8);
	p[6] = (u8)(info->stream_param >> 16);
	p[7] = (u8)(info->stream_param >> 24);
	p[10] = info->id ? 2 : 0;
	return dth11_out(br, VAL_CBDA, IDX_STREAM_CTRL, p, sizeof(p));
}

int dth11_stream_disable(struct dth11_bridge *br)
{
	u8 p[16] = { 0 };

	p[3] = 0xbc;
	return dth11_out(br, VAL_CBDA, IDX_STREAM_CTRL, p, sizeof(p));
}

int dth11_set_program_pids(struct dth11_bridge *br, u32 program_number, const u16 *pids, size_t count)
{
	u8 table[12 + 4 * DTH11_PID_LIST_MAX];
	size_t i;

	/* 番組 1 つの PID は数個から十数個 (dth11_psi.h の DTH11_PROGRAM_MAX_PIDS が上限)。超える要求は黙って切らずに断る */
	if (count > DTH11_PID_LIST_MAX)
		return -EINVAL;
	memset(table, 0, 12);
	table[0] = (u8)program_number;
	table[1] = (u8)(program_number >> 8);
	table[2] = (u8)(program_number >> 16);
	table[3] = (u8)(program_number >> 24);
	table[4] = (u8)count;
	table[5] = (u8)(count >> 8);
	for (i = 0; i < count; i++) {
		table[12 + 4 * i] = (u8)pids[i];
		table[12 + 4 * i + 1] = (u8)(pids[i] >> 8);
		table[12 + 4 * i + 2] = 0;
		table[12 + 4 * i + 3] = 0;
	}
	return dth11_out(br, VAL_CBDA, IDX_PID_FILTER, table, (u16)(12 + 4 * count));
}

size_t dth11_extract_ts(const u8 *in, size_t len, u8 *out, bool drop_filler)
{
	size_t slots = len / DTH11_SLOT_SIZE;
	size_t i, n = 0;

	for (i = 0; i < slots; i++) {
		const u8 *pkt = in + i * DTH11_SLOT_SIZE;

		if (pkt[0] != 0x47)
			continue;
		if (drop_filler && pkt[1] == 0x9f && pkt[2] == 0xff)
			continue;
		memcpy(out + n, pkt, DTH11_TS_PACKET_SIZE);
		n += DTH11_TS_PACKET_SIZE;
	}
	return n;
}
