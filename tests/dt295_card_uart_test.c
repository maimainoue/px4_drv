/* SPDX-License-Identifier: GPL-2.0-only */
#include "dt295_card.h"

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
#define PIN_POLARITY 0xf00220
#define PIN_INSERT 0xf00224
#define PIN_OTHER 0xf0022c

struct write_log {
	u32 address;
	u32 value;
	unsigned int slept_after_ms;
};

struct mock {
	u32 irq, lines;
	u32 polarity, insert, other;
	struct write_log writes[64];
	unsigned int write_count;
	u64 now_ms;
	unsigned int last_slept;
	unsigned int lock_depth;
	bool slept_while_locked;
	bool wrote_outside_uart;
};

static int mock_read(void *context, u32 address, u32 *value, unsigned int timeout_ms)
{
	struct mock *mock = context;

	(void)timeout_ms;
	switch (address) {
	case CARD_IRQ:
		*value = mock->irq;
		return 0;
	case CARD_LINES:
		*value = mock->lines;
		return 0;
	case PIN_POLARITY:
		*value = mock->polarity;
		return 0;
	case PIN_INSERT:
		*value = mock->insert << 24;
		return 0;
	case PIN_OTHER:
		*value = mock->other << 8;
		return 0;
	default:
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
	return ((struct mock *)context)->now_ms;
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
	/* 挿入あり = 0xfea01c の bit10 と 0xf00224 上位バイトの bit0 の XOR が 1。 */
	static const struct { u32 irq, insert; bool present; } cases[] = {
		{ 0, 0, false }, { 1u << 10, 0, true }, { 0, 1, true }, { 1u << 10, 1, false },
	};
	unsigned int index;

	for (index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index) {
		struct mock mock;
		struct torne_card_port port;
		bool present = !cases[index].present;

		start(&mock, &port);
		mock.irq = cases[index].irq;
		mock.insert = cases[index].insert;
		CHECK(dt295_card_detect(&port, &present) == 0);
		CHECK(present == cases[index].present);
	}
}

static void test_power_on(u32 polarity)
{
	struct mock mock;
	struct torne_card_port port;
	const u32 third = polarity == 0 ? 8u : 0u;
	const struct { u32 address, value; } expected[] = {
		{ CARD_IRQ, 0x3ff }, { CARD_LINES, 0x1 }, { CARD_FIFO, 0 }, { CARD_ENABLE, 0 },
		{ CARD_ENABLE, 7 }, { CARD_LINES, third }, { CARD_BAUD, 0xba7f },
		{ CARD_LINES, third | 2 }, { CARD_ENABLE, 0 }, { CARD_ENABLE, 7 },
		{ CARD_RECV_LRC, 0x3b }, { CARD_CWT, 9600 }, { CARD_BWT, 0 },
		{ CARD_IRQ_MASK, 0 }, { CARD_FIFO, 0x0d }, { CARD_POWER, 1 },
		{ CARD_LINES, third | 6 },
	};
	unsigned int index;

	start(&mock, &port);
	mock.polarity = polarity;
	mock.lines = 0x5; /* bit2 が立っている現在値から、段 2 で bit2 だけを落とす */
	CHECK(dt295_card_reset(&port) == 0);
	CHECK(mock.write_count == sizeof(expected) / sizeof(expected[0]));
	for (index = 0; index < sizeof(expected) / sizeof(expected[0]) && index < mock.write_count; ++index) {
		CHECK(mock.writes[index].address == expected[index].address);
		CHECK(mock.writes[index].value == expected[index].value);
		CHECK(mock.writes[index].slept_after_ms == (expected[index].address == CARD_FIFO &&
			expected[index].value == 0x0d ? 50u : 20u));
	}
	CHECK(!mock.slept_while_locked);
	CHECK(!mock.wrote_outside_uart);
	CHECK(mock.lock_depth == 0);
}

static void test_rejects_bad_pins(void)
{
	struct mock mock;
	struct torne_card_port port;

	/* 範囲外の値は拒否し、UART には 1 つも書かない。 */
	start(&mock, &port);
	mock.polarity = 2;
	CHECK(dt295_card_reset(&port) == -EPROTO);
	CHECK(mock.write_count == 0);
	start(&mock, &port);
	mock.insert = 2;
	CHECK(dt295_card_reset(&port) == -EPROTO);
	CHECK(mock.write_count == 0);
	/* 使えない組み合わせ (0xf00223 = 1, 0xf00224 = 1, 0xf0022e = 0) */
	start(&mock, &port);
	mock.polarity = 1;
	mock.insert = 1;
	mock.other = 0;
	CHECK(dt295_card_reset(&port) == -EOPNOTSUPP);
	CHECK(mock.write_count == 0);
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
	CHECK(dt295_card_power_down(&port) == 0);
	CHECK(mock.write_count == sizeof(expected) / sizeof(expected[0]));
	for (index = 0; index < sizeof(expected) / sizeof(expected[0]) && index < mock.write_count; ++index) {
		CHECK(mock.writes[index].address == expected[index].address);
		CHECK(mock.writes[index].value == expected[index].value);
	}
	CHECK(!mock.slept_while_locked);
}

static void test_timeout_is_absolute(void)
{
	struct mock mock;
	struct torne_card_port port;

	/* 待ちの合計が期限を超える前に、1 回の電源投入が完了すること。 */
	start(&mock, &port);
	CHECK(dt295_card_reset(&port) == 0);
	CHECK(mock.now_ms < 3000);
}

int main(void)
{
	test_detect();
	test_power_on(0);
	test_power_on(1);
	test_rejects_bad_pins();
	test_power_down();
	test_timeout_is_absolute();
	if (failures) {
		fprintf(stderr, "%d failure(s)\n", failures);
		return 1;
	}
	puts("dt295_card_uart_test: ok");
	return 0;
}
