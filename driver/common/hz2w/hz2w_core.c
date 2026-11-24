// SPDX-License-Identifier: GPL-2.0-only
/*
 * hz2w_core.c - GV-MVP/HZ2W の起動 (firmware、GPIO)、周期処理、閉じる手順。
 * 要求番号と値は docs/devices/hz2w.md (ベンダーのドライバーの通信記録の再現) による。
 */
#include "hz2w_core.h"
#include "hz2w_internal.h"

void hz2w_bridge_init(struct hz2w_bridge *bridge, const struct hz2w_ops *ops, void *priv)
{
	bridge->ops = ops;
	bridge->priv = priv;
}

int hz2w_ctl_in(struct hz2w_bridge *b, u8 request, u16 value, u16 index, void *buf, u16 len)
{
	int ret = b->ops->control(b->priv, HZ2W_REQ_IN, request, value, index, buf, len);

	if (ret < 0)
		return ret;
	/* 短い転送は成功として扱わない (応答の先頭は状態か値で、欠けると誤った値を使う) */
	return ret == len ? 0 : -EIO;
}

int hz2w_gpio(struct hz2w_bridge *b, bool extended, u8 value, u8 mask, u8 *readback)
{
	u8 buf[1] = { 0 };
	int ret = hz2w_ctl_in(b, extended ? 0x10 : 0x08, (u16)(value | (mask << 8)), 0, buf, 1);

	if (!ret && readback)
		*readback = buf[0];
	return ret;
}

/* 状態の 1 バイトと、書いた値の反響の 2 バイトで返る。状態が 1 でなければ失敗。 */
int hz2w_reg_write(struct hz2w_bridge *b, u8 addr, u8 reg, u8 value)
{
	u8 buf[2] = { 0, 0 };
	int ret = hz2w_ctl_in(b, 0x03, (u16)(addr | (reg << 8)), value, buf, 2);

	if (ret)
		return ret;
	return buf[0] == 1 ? 0 : -EIO;
}

int hz2w_reg_read(struct hz2w_bridge *b, u8 addr, u8 reg, u8 *value)
{
	u8 buf[2] = { 0, 0 };
	int ret = hz2w_ctl_in(b, 0x02, (u16)(addr | (reg << 8)), 0, buf, 2);

	if (ret)
		return ret;
	if (buf[0] != 1)
		return -EIO;
	*value = buf[1];
	return 0;
}

int hz2w_fw_load(struct hz2w_bridge *b, const u8 *fw, size_t size, u16 boot_index)
{
	/* ローダーと同じ区切り (0x4000 バイトのうち 0x0c00-0x2000 と 0x2400-0x2800 は 0 で、ブート ROM へ送らない) */
	static const struct {
		u8 request;
		u16 offset;
		u16 length;
	} chunks[] = {
		{ 0xab, 0x0000, 0x0c00 },
		{ 0xab, 0x2000, 0x0400 },
		{ 0xab, 0x2800, 0x1000 },
		{ 0xac, 0x3800, 0x0800 },
	};
	u8 buf[0x1000];
	unsigned int i;

	if (!fw || size != HZ2W_FIRMWARE_SIZE)
		return -EINVAL;
	for (i = 0; i < sizeof(chunks) / sizeof(chunks[0]); i++) {
		int ret;

		memcpy(buf, fw + chunks[i].offset, chunks[i].length);
		ret = b->ops->control(b->priv, HZ2W_REQ_OUT, chunks[i].request, chunks[i].offset,
				      boot_index, buf, chunks[i].length);
		if (ret < 0)
			return ret;
		if (ret != chunks[i].length)
			return -EIO;
	}
	return 0;
}

static void reset_pulse(struct hz2w_bridge *b, u8 mask)
{
	/* ベンダーと同じ 0xff → 0x00 → 0xff。待ちは 25、25、50 ms。 */
	hz2w_gpio(b, false, 0xff, mask, NULL);
	b->ops->msleep(b->priv, 25);
	hz2w_gpio(b, false, 0x00, mask, NULL);
	b->ops->msleep(b->priv, 25);
	hz2w_gpio(b, false, 0xff, mask, NULL);
	b->ops->msleep(b->priv, 50);
}

int hz2w_card_ic_reset(struct hz2w_bridge *b)
{
	u8 aux[1];
	int ret;

	reset_pulse(b, 0x11);
	ret = hz2w_ctl_in(b, 0x23, HZ2W_ASIE_ADDR, 0, aux, 1);
	if (ret)
		return ret;
	return hz2w_ctl_in(b, 0x21, 0x0100, 0, aux, 1);
}

int hz2w_board_open(struct hz2w_bridge *b)
{
	u8 info[58];
	u8 aux[3];
	u8 version = 0;
	int ret;

	ret = hz2w_ctl_in(b, 0x0c, 0, 0, info, sizeof(info));
	if (ret)
		return ret;
	/* +0x02 に VID/PID がビッグエンディアンで入る (応答の先頭 1 バイトは状態) */
	if (info[0] != 1 || info[3] != 0x04 || info[4] != 0xbb || info[5] != 0x05 || info[6] != 0x26) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "unexpected board information");
		return -ENODEV;
	}
	ret = hz2w_ctl_in(b, 0x17, 2, 0, aux, sizeof(aux));
	if (ret)
		return ret;
	/* 起動時の GPIO: 主は 0x07 (mask 0xfb)、拡張は bit1 だけ Low。これを送る前は ASIE5606B が応答しない。 */
	ret = hz2w_gpio(b, false, 0x07, 0xfb, NULL);
	if (ret)
		return ret;
	ret = hz2w_gpio(b, true, 0x00, 0x02, NULL);
	if (ret)
		return ret;
	reset_pulse(b, 0x11);
	reset_pulse(b, 0x80);
	ret = hz2w_reg_read(b, HZ2W_ASIE_ADDR, 0x09, &version);
	if (ret) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "card interface IC does not respond (%d)", ret);
		return ret;
	}
	/* ベンダーと同じ判定 ((値 >> 1) & 0x0f が 0x0b 以下) */
	if (((version >> 1) & 0x0f) > 0x0b) {
		b->ops->log(b->priv, HZ2W_LOG_ERR, "unexpected card interface IC version 0x%02x", version);
		return -ENODEV;
	}
	/* カード側の起動 (ベンダーの記録の順) */
	hz2w_gpio(b, false, 0x00, 0xd5, NULL);
	hz2w_gpio(b, false, 0xff, 0xd5, NULL);
	reset_pulse(b, 0x11);
	ret = hz2w_ctl_in(b, 0x23, HZ2W_ASIE_ADDR, 0, aux, 1);
	if (ret)
		return ret;
	ret = hz2w_ctl_in(b, 0x21, 0x0100, 0, aux, 1);
	if (ret)
		return ret;
	/*
	 * カード検出の準備。reg5 に 0xa0 を書いてから reg4/reg0 を 0 にすると、以後 reg4 の bit0 が
	 * カードの有無を表す (reg5 を書かないと 0 のままになる)。
	 */
	ret = hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x05, 0xa0);
	if (ret)
		return ret;
	hz2w_ctl_in(b, 0x21, 0x0100, 0, aux, 1);
	ret = hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x04, 0x00);
	if (ret)
		return ret;
	ret = hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x00, 0x00);
	if (ret)
		return ret;
	b->ops->msleep(b->priv, 100);
	return 0;
}

void hz2w_board_close(struct hz2w_bridge *b)
{
	u8 aux[1];

	/*
	 * ベンダーの閉じる手順。周期処理だけを止めると約 20 秒で機器が USB から消えるが、
	 * この手順の後に止めると消えない (実機で 120 秒確認)。どの要求が効くかは切り分けていない。
	 * 機器が既に無い場合の失敗は無視する。
	 */
	hz2w_ctl_in(b, 0x07, 0, 0, aux, 1);
	hz2w_ctl_in(b, 0x07, 1, 0, aux, 1);
	hz2w_reg_write(b, 0x4c, 0x80, 0x00);
	hz2w_ctl_in(b, 0x21, 0x0001, 0, aux, 1);
	hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x80, 0x00);
	hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x04, 0x00);
	hz2w_reg_write(b, HZ2W_ASIE_ADDR, 0x00, 0x00);
	hz2w_ctl_in(b, 0x21, 0x0000, 0, aux, 1);
	hz2w_gpio(b, false, 0x00, 0xd4, NULL);
	hz2w_gpio(b, false, 0x08, 0x08, NULL);
}

int hz2w_keepalive_card_state(struct hz2w_bridge *b)
{
	u8 value;

	return hz2w_reg_read(b, HZ2W_ASIE_ADDR, 0x04, &value);
}

int hz2w_keepalive_watchdog(struct hz2w_bridge *b)
{
	u8 buf[1];
	int ret = hz2w_ctl_in(b, 0x22, 0, 0, buf, 1);

	if (ret)
		return ret;
	return hz2w_ctl_in(b, 0x22, 1, 0, buf, 1);
}
