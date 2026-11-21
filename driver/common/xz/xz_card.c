// SPDX-License-Identifier: GPL-2.0-only
/*
 * GV-MVP/XZ2 のカード UART 層。
 *
 * 電源投入の 17 段の値と待ち時間は、DT295 の手順 (公開されている別実装 denpa の記述に基づく試験値) を出発点に、
 * XZ2 の実機で ATR と T=1 の往復が成立するまで合わせたもの。XZ2 では、途中の線の値 (third) が 0、最後の線の値が 6 だった。
 * 各段の必要性 (最小化) は確かめていない。
 */
#include "xz_card.h"

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
#define CARD_PRESENT_BIT 10

#define XZ_CARD_THIRD 0u	/* 途中の線の値 */

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
	return port && port->ops && port->ops->read32 && port->ops->write32 && port->ops->sleep_ms && port->ops->now_ms &&
		(!!port->ops->lock == !!port->ops->unlock);
}

/* レジスター 1 回の読み書きだけを lock で守り、期限 (操作全体の絶対期限) を残り時間として USB へ渡す。 */
static int access_reg(struct torne_card_port *port, u32 address, u32 *value, bool write)
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
		ret = write ? port->ops->write32(port->context, address, *value, timeout_ms) :
			      port->ops->read32(port->context, address, value, timeout_ms);
		if (!ret && port->ops->now_ms(port->context) >= port->deadline_ms)
			ret = -ETIMEDOUT;
	}
	if (port->ops->unlock)
		port->ops->unlock(port->context);
	return ret;
}

/* 1 段ごとに書いて待つ。待ちは USB と制御ロックを保持せずに行う。 */
static int run_steps(struct torne_card_port *port, const struct power_step *steps, unsigned int count)
{
	unsigned int index;
	int ret = 0;

	for (index = 0; !ret && index < count; ++index) {
		u32 value = steps[index].value;

		ret = access_reg(port, steps[index].address, &value, true);
		if (!ret)
			port->ops->sleep_ms(port->context, steps[index].wait_ms);
	}
	return ret;
}

int xz_card_detect(struct torne_card_port *port, bool *present)
{
	u32 irq = 0;
	int ret;

	if (!port_valid(port) || !present)
		return -EINVAL;
	*present = false;
	ret = access_reg(port, CARD_IRQ, &irq, false);
	if (!ret)
		*present = ((irq >> CARD_PRESENT_BIT) & 1u) != 0;
	return ret;
}

int xz_card_reset(struct torne_card_port *port)
{
	struct power_step steps[17];
	u32 lines = 0;
	int ret;

	if (!port_valid(port))
		return -EINVAL;
	/* 投入は 17 段の待ちを含むので、既定の期限 (1 秒) より長い期限を、この手順だけに与える。 */
	ret = torne_card_port_set_timeout(port, POWER_TOTAL_TIMEOUT_MS);
	if (ret)
		return ret;
	ret = access_reg(port, CARD_LINES, &lines, false);
	if (ret)
		return ret;

	steps[0] = (struct power_step){ CARD_IRQ, 0x3ff, POWER_STEP_WAIT_MS };
	steps[1] = (struct power_step){ CARD_LINES, lines & ~4u, POWER_STEP_WAIT_MS };
	steps[2] = (struct power_step){ CARD_FIFO, 0, POWER_STEP_WAIT_MS };
	steps[3] = (struct power_step){ CARD_ENABLE, 0, POWER_STEP_WAIT_MS };
	steps[4] = (struct power_step){ CARD_ENABLE, 7, POWER_STEP_WAIT_MS };
	steps[5] = (struct power_step){ CARD_LINES, XZ_CARD_THIRD, POWER_STEP_WAIT_MS };
	steps[6] = (struct power_step){ CARD_BAUD, 0xba7f, POWER_STEP_WAIT_MS };
	steps[7] = (struct power_step){ CARD_LINES, XZ_CARD_THIRD | 2, POWER_STEP_WAIT_MS };
	steps[8] = (struct power_step){ CARD_ENABLE, 0, POWER_STEP_WAIT_MS };
	steps[9] = (struct power_step){ CARD_ENABLE, 7, POWER_STEP_WAIT_MS };
	steps[10] = (struct power_step){ CARD_RECV_LRC, 0x3b, POWER_STEP_WAIT_MS };
	steps[11] = (struct power_step){ CARD_CWT, 9600, POWER_STEP_WAIT_MS };
	steps[12] = (struct power_step){ CARD_BWT, 0, POWER_STEP_WAIT_MS };
	steps[13] = (struct power_step){ CARD_IRQ_MASK, 0, POWER_STEP_WAIT_MS };
	steps[14] = (struct power_step){ CARD_FIFO, 0x0d, POWER_COUNT_WAIT_MS };
	steps[15] = (struct power_step){ CARD_POWER, 1, POWER_STEP_WAIT_MS };
	steps[16] = (struct power_step){ CARD_LINES, XZ_CARD_THIRD | 6, POWER_STEP_WAIT_MS };
	return run_steps(port, steps, sizeof(steps) / sizeof(steps[0]));
}

int xz_card_power_down(struct torne_card_port *port)
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
