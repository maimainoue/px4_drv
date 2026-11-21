// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295 のカード UART 層。
 *
 * 電源投入・切断の順序と待ち時間は、公開されている別実装 (denpa) の記述に基づく値で、
 * 各段の必要性は、実機のレジスター読み戻しで確かめ直していない。
 * 実機 (Rev.01、Rev.02) では、ATR の取得と、T=1 の往復 (200 ms 間隔で 5 分以上、受信と同時) が成立している。
 * カードの端子の極性は 0xf00220 台の語から読む。
 */
#include "dt295_card.h"

#define CARD_ENABLE 0xfea000
#define CARD_LINES 0xfea004
#define CARD_BAUD 0xfea008
#define CARD_FIFO 0xfea014
#define CARD_IRQ_MASK 0xfea018
#define CARD_IRQ 0xfea01c
#define CARD_CWT 0xfea020
#define CARD_BWT 0xfea024
#define CARD_POWER 0xfea034
#define CARD_RECV_LRC 0xfea03c
#define CARD_NO_CARD_BIT 10

/* 端子の読み取り元。4 の倍数のアドレスの語から、仕様上のバイト位置を取り出す。 */
#define PIN_WORD_POLARITY 0xf00220	/* 下位バイト = 0xf00223 */
#define PIN_WORD_INSERT 0xf00224	/* 上位バイト = 0xf00224 */
#define PIN_WORD_OTHER 0xf0022c		/* bit15-8 = 0xf0022e */

#define POWER_STEP_WAIT_MS 20U
#define POWER_COUNT_WAIT_MS 50U
#define POWER_TOTAL_TIMEOUT_MS 3000U

struct power_step {
	u32 address;
	u32 value;
	unsigned int wait_ms;
};

static int port_valid(const struct torne_card_port *port)
{
	return port && port->ops && port->ops->read32 && port->ops->write32 &&
		port->ops->sleep_ms && port->ops->now_ms &&
		(!!port->ops->lock == !!port->ops->unlock);
}

static int read_reg(struct torne_card_port *port, u32 address, u32 *value)
{
	unsigned int timeout_ms;
	u64 now;
	int ret;

	if (port->ops->lock) {
		ret = port->ops->lock(port->context);
		if (ret)
			return ret;
	}
	now = port->ops->now_ms(port->context);
	if (now >= port->deadline_ms) {
		ret = -ETIMEDOUT;
	} else {
		u64 remaining = port->deadline_ms - now;

		timeout_ms = remaining > (u64)~0u ? ~0u : (unsigned int)remaining;
		ret = port->ops->read32(port->context, address, value, timeout_ms);
		if (!ret && port->ops->now_ms(port->context) >= port->deadline_ms)
			ret = -ETIMEDOUT;
	}
	if (port->ops->unlock)
		port->ops->unlock(port->context);
	return ret;
}

static int write_reg(struct torne_card_port *port, u32 address, u32 value)
{
	unsigned int timeout_ms;
	u64 now;
	int ret;

	if (port->ops->lock) {
		ret = port->ops->lock(port->context);
		if (ret)
			return ret;
	}
	now = port->ops->now_ms(port->context);
	if (now >= port->deadline_ms) {
		ret = -ETIMEDOUT;
	} else {
		u64 remaining = port->deadline_ms - now;

		timeout_ms = remaining > (u64)~0u ? ~0u : (unsigned int)remaining;
		ret = port->ops->write32(port->context, address, value, timeout_ms);
		if (!ret && port->ops->now_ms(port->context) >= port->deadline_ms)
			ret = -ETIMEDOUT;
	}
	if (port->ops->unlock)
		port->ops->unlock(port->context);
	return ret;
}

/* 1 段ごとに書いて待つ。待ちは USB と制御ロックを保持せずに行う。 */
static int run_steps(struct torne_card_port *port, const struct power_step *steps,
		     unsigned int count)
{
	unsigned int index;
	int ret = 0;

	for (index = 0; !ret && index < count; ++index) {
		ret = write_reg(port, steps[index].address, steps[index].value);
		if (!ret)
			port->ops->sleep_ms(port->context, steps[index].wait_ms);
	}
	return ret;
}

int dt295_card_detect(struct torne_card_port *port, bool *present)
{
	u32 irq = 0, insert = 0;
	int ret;

	if (!port_valid(port) || !present)
		return -EINVAL;
	*present = false;
	ret = read_reg(port, CARD_IRQ, &irq);
	if (!ret)
		ret = read_reg(port, PIN_WORD_INSERT, &insert);
	if (!ret)
		*present = (((irq >> CARD_NO_CARD_BIT) ^ (insert >> 24)) & 1u) != 0;
	return ret;
}

int dt295_card_reset(struct torne_card_port *port)
{
	u32 polarity_word = 0, insert_word = 0, other_word = 0, lines = 0;
	u32 polarity, insert, other, first, third;
	struct power_step steps[17];
	int ret;

	if (!port_valid(port))
		return -EINVAL;
	/* 投入は 17 段の待ちを含むので、既定の期限 (1 秒) より長い期限を、この手順だけに与える。 */
	ret = torne_card_port_set_timeout(port, POWER_TOTAL_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = read_reg(port, PIN_WORD_POLARITY, &polarity_word);
	if (!ret)
		ret = read_reg(port, PIN_WORD_INSERT, &insert_word);
	if (!ret)
		ret = read_reg(port, PIN_WORD_OTHER, &other_word);
	if (!ret)
		ret = read_reg(port, CARD_LINES, &lines);
	if (ret)
		return ret;

	polarity = polarity_word & 0xff;
	insert = insert_word >> 24;
	other = (other_word >> 8) & 0xff;
	/* 範囲外の値と、使えない組み合わせは、成功扱いにせず拒否する。 */
	if (polarity > 1 || insert > 1 || other > 1)
		return -EPROTO;
	if (polarity == 1 && insert == 1 && other == 0)
		return -EOPNOTSUPP;

	first = lines & ~4u;
	third = polarity == 0 ? 8u : 0u;
	steps[0] = (struct power_step){ CARD_IRQ, 0x3ff, POWER_STEP_WAIT_MS };
	steps[1] = (struct power_step){ CARD_LINES, first, POWER_STEP_WAIT_MS };
	steps[2] = (struct power_step){ CARD_FIFO, 0, POWER_STEP_WAIT_MS };
	steps[3] = (struct power_step){ CARD_ENABLE, 0, POWER_STEP_WAIT_MS };
	steps[4] = (struct power_step){ CARD_ENABLE, 7, POWER_STEP_WAIT_MS };
	steps[5] = (struct power_step){ CARD_LINES, third, POWER_STEP_WAIT_MS };
	steps[6] = (struct power_step){ CARD_BAUD, 0xba7f, POWER_STEP_WAIT_MS };
	steps[7] = (struct power_step){ CARD_LINES, third | 2, POWER_STEP_WAIT_MS };
	steps[8] = (struct power_step){ CARD_ENABLE, 0, POWER_STEP_WAIT_MS };
	steps[9] = (struct power_step){ CARD_ENABLE, 7, POWER_STEP_WAIT_MS };
	steps[10] = (struct power_step){ CARD_RECV_LRC, 0x3b, POWER_STEP_WAIT_MS };
	steps[11] = (struct power_step){ CARD_CWT, 9600, POWER_STEP_WAIT_MS };
	steps[12] = (struct power_step){ CARD_BWT, 0, POWER_STEP_WAIT_MS };
	steps[13] = (struct power_step){ CARD_IRQ_MASK, 0, POWER_STEP_WAIT_MS };
	steps[14] = (struct power_step){ CARD_FIFO, 0x0d, POWER_COUNT_WAIT_MS };
	steps[15] = (struct power_step){ CARD_POWER, 1, POWER_STEP_WAIT_MS };
	steps[16] = (struct power_step){ CARD_LINES, third | 6, POWER_STEP_WAIT_MS };
	return run_steps(port, steps, sizeof(steps) / sizeof(steps[0]));
}

int dt295_card_power_down(struct torne_card_port *port)
{
	static const struct power_step steps[] = {
		{ CARD_LINES, 2, POWER_STEP_WAIT_MS },
		{ CARD_FIFO, 0, POWER_STEP_WAIT_MS },
		{ CARD_ENABLE, 3, POWER_STEP_WAIT_MS },
		{ CARD_LINES, 0, POWER_STEP_WAIT_MS },
		{ CARD_LINES, 8, POWER_STEP_WAIT_MS },
	};
	int ret;

	if (!port_valid(port))
		return -EINVAL;
	ret = torne_card_port_set_timeout(port, POWER_TOTAL_TIMEOUT_MS);
	if (ret)
		return ret;
	return run_steps(port, steps, sizeof(steps) / sizeof(steps[0]));
}
