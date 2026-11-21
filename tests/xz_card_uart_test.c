/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GV-MVP/XZ2 のカード UART 層 (driver/common/xz/xz_card.c) の自動試験。
 * 実機の代用ではない (実機の結果は docs/devices/xz.md)。
 *
 * build: gcc -Wall -Wextra -fsanitize=address,undefined -I driver/common/torne -I driver/common/xz \
 *            tests/xz_card_uart_test.c driver/common/xz/xz_card.c driver/common/torne/torne_card.c -o xz_card_uart_test
 */
#include "xz_card.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

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

struct write_log {
	u32 address;
	u32 value;
	unsigned int slept_after_ms;
};

struct mock {
	u32 irq, lines;
	struct write_log writes[64];
	unsigned int write_count;
	unsigned int fail_write_at;	/* この番号 (1 から) の書き込みを失敗させる */
	u64 now_ms;
	unsigned int now_step;		/* now_ms を呼ぶたびに進める時間 (期限の試験用) */
	unsigned int lock_depth;
	bool slept_while_locked;
	bool wrote_outside_uart;
	bool read_other;		/* UART の 2 つのレジスター以外を読んだ (端子の語など) */
	unsigned int reads;
};

static int mock_read(void *context, u32 address, u32 *value, unsigned int timeout_ms)
{
	struct mock *mock = context;

	(void)timeout_ms;
	mock->reads++;
	switch (address) {
	case CARD_IRQ:
		*value = mock->irq;
		return 0;
	case CARD_LINES:
		*value = mock->lines;
		return 0;
	default:
		mock->read_other = true;	/* DT295 のように端子の語 (0xf00220 台) を読むと、ここへ来る */
		return -EIO;
	}
}

static int mock_write(void *context, u32 address, u32 value, unsigned int timeout_ms)
{
	struct mock *mock = context;

	(void)timeout_ms;
	if (address < 0xfea000 || address > 0xfea03f)
		mock->wrote_outside_uart = true;
	if (mock->write_count < sizeof(mock->writes) / sizeof(mock->writes[0])) {
		mock->writes[mock->write_count].address = address;
		mock->writes[mock->write_count].value = value;
		mock->writes[mock->write_count].slept_after_ms = 0;
		mock->write_count++;
	}
	if (mock->fail_write_at && mock->write_count == mock->fail_write_at)
		return -EPIPE;
	return 0;
}

static int mock_lock(void *context)
{
	((struct mock *)context)->lock_depth++;
	return 0;
}

static void mock_unlock(void *context)
{
	((struct mock *)context)->lock_depth--;
}

static void mock_sleep(void *context, unsigned int milliseconds)
{
	struct mock *mock = context;

	if (mock->lock_depth)
		mock->slept_while_locked = true;
	mock->now_ms += milliseconds;
	if (mock->write_count)
		mock->writes[mock->write_count - 1].slept_after_ms += milliseconds;
}

static u64 mock_now(void *context)
{
	struct mock *mock = context;
	u64 now = mock->now_ms;

	mock->now_ms += mock->now_step;
	return now;
}

static const struct torne_card_io_ops ops = {
	.read32 = mock_read,
	.write32 = mock_write,
	.lock = mock_lock,
	.unlock = mock_unlock,
	.sleep_ms = mock_sleep,
	.now_ms = mock_now,
};

static int failures;

#define CHECK(condition) \
	do { \
		if (!(condition)) { \
			fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
			failures++; \
		} \
	} while (0)

static void start(struct mock *mock, struct torne_card_port *port)
{
	memset(mock, 0, sizeof(*mock));
	torne_card_port_init(port, &ops, mock);
}

static void test_detect(void)
{
	/* 挿入あり = 0xfea01c の bit10 が立っている。他のビットは影響しない。端子の語は読まない。 */
	static const struct { u32 irq; bool present; } cases[] = {
		{ 0, false }, { 1u << 10, true }, { ~(1u << 10), false }, { ~0u, true }, { 0x3ff, false },
	};
	unsigned int index;
	struct mock mock;
	struct torne_card_port port;
	bool present = true;

	for (index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index) {
		start(&mock, &port);
		mock.irq = cases[index].irq;
		present = !cases[index].present;
		CHECK(xz_card_detect(&port, &present) == 0);
		CHECK(present == cases[index].present);
		CHECK(!mock.read_other && mock.write_count == 0);
	}
	/* 読み出しの失敗は、存在なしとして成功扱いにせず、エラーを返す */
	start(&mock, &port);
	port.ops = &(struct torne_card_io_ops){ .read32 = mock_read, .write32 = mock_write, .sleep_ms = mock_sleep, .now_ms = mock_now };
	port.context = &mock;
	present = true;
	CHECK(xz_card_detect(&port, NULL) == -EINVAL);
	/* 不正なポート */
	CHECK(xz_card_detect(NULL, &present) == -EINVAL);
}

static void test_power_on(void)
{
	struct mock mock;
	struct torne_card_port port;
	/* 途中の線の値 third = 0、最後の線の値は 6 (DT295 の極性 0 の手順は third = 8) */
	const struct { u32 address, value; } expected[] = {
		{ CARD_IRQ, 0x3ff }, { CARD_LINES, 0x1 }, { CARD_FIFO, 0 }, { CARD_ENABLE, 0 },
		{ CARD_ENABLE, 7 }, { CARD_LINES, 0 }, { CARD_BAUD, 0xba7f },
		{ CARD_LINES, 2 }, { CARD_ENABLE, 0 }, { CARD_ENABLE, 7 },
		{ CARD_RECV_LRC, 0x3b }, { CARD_CWT, 9600 }, { CARD_BWT, 0 },
		{ CARD_IRQ_MASK, 0 }, { CARD_FIFO, 0x0d }, { CARD_POWER, 1 },
		{ CARD_LINES, 6 },
	};
	unsigned int index;

	start(&mock, &port);
	mock.lines = 0x5; /* bit2 が立っている現在値から、段 2 で bit2 だけを落とす */
	CHECK(xz_card_reset(&port) == 0);
	CHECK(mock.write_count == sizeof(expected) / sizeof(expected[0]));
	for (index = 0; index < sizeof(expected) / sizeof(expected[0]) && index < mock.write_count; ++index) {
		CHECK(mock.writes[index].address == expected[index].address);
		CHECK(mock.writes[index].value == expected[index].value);
		CHECK(mock.writes[index].slept_after_ms == (expected[index].address == CARD_FIFO &&
			expected[index].value == 0x0d ? 50u : 20u));
	}
	CHECK(!mock.slept_while_locked);
	CHECK(!mock.wrote_outside_uart);
	CHECK(!mock.read_other);	/* 端子の語を読まない */
	CHECK(mock.reads == 1);		/* 現在の線の値の 1 回だけ */
	CHECK(mock.lock_depth == 0);
	/* 現在の線の値の bit2 以外は保たれる */
	start(&mock, &port);
	mock.lines = 0xfffffffd;
	CHECK(xz_card_reset(&port) == 0 && mock.writes[1].value == 0xfffffff9);
}

static void test_failures(void)
{
	struct mock mock;
	struct torne_card_port port;
	unsigned int at;

	/* どの段の書き込みが失敗しても、そのエラーを返し、後の段を書かない */
	for (at = 1; at <= 17; ++at) {
		start(&mock, &port);
		mock.fail_write_at = at;
		CHECK(xz_card_reset(&port) == -EPIPE);
		CHECK(mock.write_count == at);
		CHECK(mock.lock_depth == 0);
	}
	/* 現在の線の値の読み出しが失敗したら、何も書かない (読み出しのアドレスは UART 内の 2 つだけなので、ここでは不正なポートで代用) */
	CHECK(xz_card_reset(NULL) == -EINVAL);
	CHECK(xz_card_power_down(NULL) == -EINVAL);
	start(&mock, &port);
	port.ops = &(struct torne_card_io_ops){ .read32 = mock_read, .sleep_ms = mock_sleep, .now_ms = mock_now };
	CHECK(xz_card_reset(&port) == -EINVAL && mock.write_count == 0);	/* write32 が無い */
	/* 電源切断の失敗 */
	start(&mock, &port);
	mock.fail_write_at = 3;
	CHECK(xz_card_power_down(&port) == -EPIPE && mock.write_count == 3);
}

static void test_power_down(void)
{
	struct mock mock;
	struct torne_card_port port;
	const struct { u32 address, value; } expected[] = {
		{ CARD_LINES, 2 }, { CARD_FIFO, 0 }, { CARD_ENABLE, 3 }, { CARD_LINES, 0 },
		{ CARD_LINES, 8 },
	};
	unsigned int index;

	start(&mock, &port);
	CHECK(xz_card_power_down(&port) == 0);
	CHECK(mock.write_count == sizeof(expected) / sizeof(expected[0]));
	for (index = 0; index < sizeof(expected) / sizeof(expected[0]) && index < mock.write_count; ++index) {
		CHECK(mock.writes[index].address == expected[index].address);
		CHECK(mock.writes[index].value == expected[index].value);
	}
	CHECK(!mock.slept_while_locked && !mock.wrote_outside_uart && mock.reads == 0);
}

static void test_timeout_is_absolute(void)
{
	struct mock mock;
	struct torne_card_port port;

	/* 待ちの合計 (約 400 ms) が期限 (3 秒) を超える前に、1 回の電源投入が完了すること。 */
	start(&mock, &port);
	CHECK(xz_card_reset(&port) == 0);
	CHECK(mock.now_ms < 3000);
	/* 期限は手順全体の絶対期限: 時計が速く進む (USB が遅い) と、途中で -ETIMEDOUT になり、残りの段を書かない */
	start(&mock, &port);
	mock.now_step = 400;
	CHECK(xz_card_reset(&port) == -ETIMEDOUT);
	CHECK(mock.write_count < 17 && mock.lock_depth == 0);
	/* 期限が過ぎているポートでは、USB の転送を発行せずに -ETIMEDOUT (期限切れの後に、時間のかかる転送を始めない) */
	start(&mock, &port);
	port.deadline_ms = 5;
	mock.now_ms = 10;
	{
		bool present = true;

		CHECK(xz_card_detect(&port, &present) == -ETIMEDOUT);
		CHECK(mock.reads == 0 && mock.lock_depth == 0 && !present);
	}
}

int main(void)
{
	test_detect();
	test_power_on();
	test_failures();
	test_power_down();
	test_timeout_is_absolute();
	if (failures) {
		fprintf(stderr, "%d failure(s)\n", failures);
		return 1;
	}
	puts("xz_card_uart_test: ok");
	return 0;
}
