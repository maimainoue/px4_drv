// SPDX-License-Identifier: GPL-2.0-only
/*
 * tools/ptx_tool.h の TS の集計 (ts_state_feed) の自己試験。機材は要らない。
 *
 * build: gcc -Wall -Wextra -O2 -I include -I driver/common/include tools/ptx_tool_selftest.c -o ptx_tool_selftest
 */
#include "ptx_tool.h"

static int failures;
static int checks;

#define CHECK(cond)                                                                             \
	do {                                                                                    \
		checks++;                                                                       \
		if (!(cond)) {                                                                  \
			failures++;                                                             \
			fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #cond);                 \
		}                                                                               \
	} while (0)

/* pid の TS パケットを 1 つ作る。afc は adaptation_field_control、cc は連続性カウンター */
static void make_packet(uint8_t *p, unsigned int pid, unsigned int afc, unsigned int cc, int tei)
{
	memset(p, 0xff, TS_PACKET_SIZE);
	p[0] = 0x47;
	p[1] = (uint8_t)(((tei ? 0x80 : 0) | ((pid >> 8) & 0x1f)));
	p[2] = (uint8_t)pid;
	p[3] = (uint8_t)((afc << 4) | (cc & 0x0f));
	if (afc & 2) {
		p[4] = 7;
		p[5] = 0;
	}
}

/* 連続性カウンターが正しく進む n 個のパケット列を作る (PID 0x100) */
static size_t clean_stream(uint8_t *out, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		make_packet(out + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i, 0);
	return (size_t)n * TS_PACKET_SIZE;
}

int main(void)
{
	static uint8_t stream[TS_PACKET_SIZE * 400];
	static struct ts_state s;
	size_t len, off;
	unsigned int i;

	/* 1. 正常な列を、いろいろな長さに分けて渡しても、全パケットを数え、誤りは 0 */
	ts_state_init(&s);
	len = clean_stream(stream, 100);
	for (off = 0; off < len;) {
		size_t chunk = 1 + (off * 7) % 400;

		if (chunk > len - off)
			chunk = len - off;
		ts_state_feed(&s, stream + off, chunk);
		off += chunk;
	}
	CHECK(s.counts.packets == 100);
	CHECK(ts_counts_clean(&s.counts));
	CHECK(s.counts.cc_dups == 0);

	/* 2. 1 バイトずつ渡しても同じ */
	ts_state_init(&s);
	len = clean_stream(stream, 200);
	for (off = 0; off < len; off++)
		ts_state_feed(&s, stream + off, 1);
	CHECK(s.counts.packets == 200);
	CHECK(ts_counts_clean(&s.counts));

	/* 3. 連続性の飛び (1 つ飛ばす) は 1 回、PID ごとに数える */
	ts_state_init(&s);
	len = clean_stream(stream, 50);
	for (i = 20; i < 50; i++)	/* 20 番目から、カウンターが 1 つ先へ進む (飛びは 20 番目の 1 回だけ) */
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i + 1, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.cc_jumps == 1);
	CHECK(s.jumps_by_pid[0x100] == 1);

	/* 4. 同じカウンターの値の重複は、飛びに数えず、重複として数える */
	ts_state_init(&s);
	len = clean_stream(stream, 30);
	make_packet(stream + 10 * TS_PACKET_SIZE, 0x100, 1, 9, 0);	/* 9 の次のパケットを、もう一度 9 にする */
	for (i = 11; i < 30; i++)
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i - 1, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.cc_dups == 1);
	CHECK(s.counts.cc_jumps == 0);

	/*
	 * 5. TEI のパケットは TEI に数え、連続性の検査へは使わない (中身が信頼できないため、カウンターも進めない)。
	 *    そのため、同じ PID の次のパケットで、連続性の飛びが 1 回数えられる (欠けたパケットとして扱う)。
	 *    TEI のパケットの連続性カウンターが壊れていても、その値で飛びを数えることはない。
	 */
	ts_state_init(&s);
	len = clean_stream(stream, 40);
	make_packet(stream + 15 * TS_PACKET_SIZE, 0x100, 1, 3, 1);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.tei == 1);
	CHECK(s.counts.cc_jumps == 1);

	/* 6. ヌルパケットは、連続性の検査へ使わない */
	ts_state_init(&s);
	len = clean_stream(stream, 40);
	make_packet(stream + 10 * TS_PACKET_SIZE, 0x1fff, 1, 7, 0);
	make_packet(stream + 11 * TS_PACKET_SIZE, 0x1fff, 1, 0, 0);
	for (i = 12; i < 40; i++)
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i - 2, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.nulls == 2);
	CHECK(s.counts.cc_jumps == 0);

	/* 7. ペイロードのないパケット (アダプテーションフィールドだけ) は、カウンターが進まない */
	ts_state_init(&s);
	len = clean_stream(stream, 40);
	make_packet(stream + 10 * TS_PACKET_SIZE, 0x100, 2, 0, 0);
	for (i = 11; i < 40; i++)
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i - 1, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.cc_jumps == 0);

	/* 8. discontinuity_indicator が立ったパケットは、カウンターの不連続を許す */
	ts_state_init(&s);
	len = clean_stream(stream, 40);
	make_packet(stream + 10 * TS_PACKET_SIZE, 0x100, 3, 12, 0);
	stream[10 * TS_PACKET_SIZE + 5] = 0x80;
	for (i = 11; i < 40; i++)
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i + 2, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.cc_jumps == 0);

	/* 9. アダプテーションフィールドの長さがパケットに収まらない値でも、範囲外を読まない (discontinuity と見なさない) */
	ts_state_init(&s);
	len = clean_stream(stream, 40);
	make_packet(stream + 10 * TS_PACKET_SIZE, 0x100, 3, 10, 0);
	stream[10 * TS_PACKET_SIZE + 4] = 200;
	stream[10 * TS_PACKET_SIZE + 5] = 0x80;
	for (i = 11; i < 40; i++)
		make_packet(stream + (size_t)i * TS_PACKET_SIZE, 0x100, 1, i, 0);
	ts_state_feed(&s, stream, len);
	CHECK(s.counts.packets == 40);

	/* 10. 途中でゴミのバイトが挟まると、同期を失った回数に 1 回数え、その後、再び同期して数える */
	ts_state_init(&s);
	len = clean_stream(stream, 30);
	ts_state_feed(&s, stream, 20 * TS_PACKET_SIZE);
	{
		static const uint8_t garbage[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

		ts_state_feed(&s, garbage, sizeof(garbage));
	}
	{
		static uint8_t tail[TS_PACKET_SIZE * 40];

		for (i = 0; i < 40; i++)
			make_packet(tail + (size_t)i * TS_PACKET_SIZE, 0x200, 1, i, 0);
		ts_state_feed(&s, tail, sizeof(tail));
	}
	CHECK(s.counts.sync_loss == 1);
	CHECK(s.counts.packets >= 20 + 38);
	CHECK(s.counts.cc_jumps == 0);

	/* 11. 先頭が 188 バイトの境界から外れていても、最初の位置合わせは、同期を失った回数に数えない */
	ts_state_init(&s);
	len = clean_stream(stream, 50);
	{
		static uint8_t shifted[TS_PACKET_SIZE * 50 + 15];

		memset(shifted, 0x55, 15);
		memcpy(shifted + 15, stream, len);
		ts_state_feed(&s, shifted, sizeof(shifted));
	}
	CHECK(s.counts.sync_loss == 0);
	CHECK(s.counts.packets >= 48);

	printf("ptx_tool_selftest: %d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
