/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 の PAT/PMT の収集 (driver/common/dth11/dth11_psi) の自動試験。
 *
 * 確認するのは、CRC32 の値、PAT と PMT の解析 (番組番号、PMT の PID、PCR、CA 記述子の ECM、ES)、
 * 複数のパケットにまたがるセクション、ポインターフィールドの前の続きのセクション、CRC の合わない
 * セクションと TEI のパケットの無視、全ての番組の PMT がそろうまで完了にしないこと、番組の PID リストの
 * 順序と重複の除去。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_psi_test.c \
 *            driver/common/dth11/dth11_psi.c -o dth11_psi_test
 */
#include "dth11_psi.h"

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

/* セクションの長さと CRC を埋めて、全体の長さを返す。body は 3 バイト目以降の、CRC を除く部分を含む配列 */
static size_t finish_section(u8 *sec, size_t len_without_crc)
{
	size_t slen = len_without_crc - 3 + 4;
	u32 crc;

	sec[1] = (u8)(0xb0 | ((slen >> 8) & 0x0f));
	sec[2] = (u8)slen;
	crc = dth11_mpeg_crc32(sec, len_without_crc);
	sec[len_without_crc] = (u8)(crc >> 24);
	sec[len_without_crc + 1] = (u8)(crc >> 16);
	sec[len_without_crc + 2] = (u8)(crc >> 8);
	sec[len_without_crc + 3] = (u8)crc;
	return len_without_crc + 4;
}

/* 1 パケットに収まるセクション */
static void packet(u8 *p, u16 pid, bool pusi, const u8 *payload, size_t n, u8 cc)
{
	memset(p, 0xff, 188);
	p[0] = 0x47;
	p[1] = (u8)(((pid >> 8) & 0x1f) | (pusi ? 0x40 : 0));
	p[2] = (u8)pid;
	p[3] = (u8)(0x10 | (cc & 0x0f));
	if (pusi)
		p[4] = 0; /* pointer_field */
	memcpy(p + (pusi ? 5 : 4), payload, n);
}

static size_t make_pat(u8 *sec, const u16 *numbers, const u16 *pids, size_t count)
{
	size_t n = 8, i;

	memset(sec, 0, 1024);
	sec[0] = 0x00;
	sec[3] = 0x7f;
	sec[4] = 0xe0;
	sec[5] = 0xc1;
	for (i = 0; i < count; i++) {
		sec[n++] = (u8)(numbers[i] >> 8);
		sec[n++] = (u8)numbers[i];
		sec[n++] = (u8)(0xe0 | (pids[i] >> 8));
		sec[n++] = (u8)pids[i];
	}
	return finish_section(sec, n);
}

/*
 * PMT: 番組情報の記述子に CA (ECM) を 1 つ、ES を es_count 個 (最初の ES に CA 記述子を持たせる場合は ecm_es)。
 */
static size_t make_pmt(u8 *sec, u16 number, u16 pcr, u16 ecm, const u16 *es, size_t es_count, u16 ecm_es)
{
	size_t n = 12, pi_start, i;

	memset(sec, 0, 1024);
	sec[0] = 0x02;
	sec[3] = (u8)(number >> 8);
	sec[4] = (u8)number;
	sec[5] = 0xc1;
	sec[8] = (u8)(0xe0 | (pcr >> 8));
	sec[9] = (u8)pcr;
	pi_start = n;
	if (ecm) {
		sec[n++] = 0x09;
		sec[n++] = 0x04;
		sec[n++] = 0x00;
		sec[n++] = 0x05;
		sec[n++] = (u8)(0xe0 | (ecm >> 8));
		sec[n++] = (u8)ecm;
	}
	sec[10] = (u8)(0xf0 | (((n - pi_start) >> 8) & 0x0f));
	sec[11] = (u8)(n - pi_start);
	for (i = 0; i < es_count; i++) {
		size_t es_info = 0;

		sec[n++] = 0x0f;
		sec[n++] = (u8)(0xe0 | (es[i] >> 8));
		sec[n++] = (u8)es[i];
		if (i == 0 && ecm_es) {
			es_info = 6;
			sec[n++] = 0xf0;
			sec[n++] = 6;
			sec[n++] = 0x09;
			sec[n++] = 0x04;
			sec[n++] = 0x00;
			sec[n++] = 0x05;
			sec[n++] = (u8)(0xe0 | (ecm_es >> 8));
			sec[n++] = (u8)ecm_es;
		} else {
			sec[n++] = 0xf0;
			sec[n++] = 0;
		}
		(void)es_info;
	}
	return finish_section(sec, n);
}

static void test_crc(void)
{
	/* CRC-32/MPEG-2 の標準の検査値 */
	CHECK(dth11_mpeg_crc32((const u8 *)"123456789", 9) == 0x0376e6e7);
	/* セクションの末尾に CRC を付けると、全体の CRC は 0 */
	{
		u8 sec[64] = { 0x00, 0xb0, 0, 0, 1, 0xc1, 0, 0 };
		size_t n = finish_section(sec, 8);

		CHECK(dth11_mpeg_crc32(sec, n) == 0);
	}
}

static void test_pat_pmt(void)
{
	struct dth11_psi psi;
	u8 sec[1024], pkt[188];
	struct dth11_program progs[4];
	u16 pids[DTH11_PROGRAM_MAX_PIDS];
	static const u16 es[2] = { 0x100, 0x110 };
	size_t n;

	dth11_psi_init(&psi);
	{
		static const u16 nums[1] = { 0x400 }, ppid[1] = { 0x1f0 };

		n = make_pat(sec, nums, ppid, 1);
		packet(pkt, 0, true, sec, n, 0);
		dth11_psi_feed(&psi, pkt);
	}
	CHECK(!dth11_psi_complete(&psi));
	n = make_pmt(sec, 0x400, 0x1ff, 0x901, es, 2, 0);
	packet(pkt, 0x1f0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(dth11_psi_complete(&psi));
	n = dth11_psi_programs(&psi, progs, 4);
	CHECK(n == 1);
	CHECK(progs[0].number == 0x400 && progs[0].pmt_pid == 0x1f0 && progs[0].pcr_pid == 0x1ff);
	CHECK(progs[0].ecm_count == 1 && progs[0].ecm_pids[0] == 0x901);
	CHECK(progs[0].es_count == 2 && progs[0].es_pids[0] == 0x100 && progs[0].es_pids[1] == 0x110);
	n = dth11_program_pid_list(&progs[0], pids);
	/* 参考実装の試験と同じ並び: PMT、ECM、PCR、ES */
	CHECK(n == 5 && pids[0] == 0x1f0 && pids[1] == 0x901 && pids[2] == 0x1ff && pids[3] == 0x100 && pids[4] == 0x110);
}

static void test_dedupe_and_no_pcr(void)
{
	struct dth11_program prog;
	u16 pids[DTH11_PROGRAM_MAX_PIDS];
	size_t n;

	memset(&prog, 0, sizeof(prog));
	prog.number = 1;
	prog.pmt_pid = 0x30;
	prog.pcr_pid = 0x100; /* ES と同じ PID は、PCR で 1 回入れる */
	prog.ecm_pids[0] = 0x31;
	prog.ecm_pids[1] = 0x31;
	prog.ecm_count = 2;
	prog.es_pids[0] = 0x100;
	prog.es_pids[1] = 0x101;
	prog.es_count = 2;
	n = dth11_program_pid_list(&prog, pids);
	CHECK(n == 4 && pids[0] == 0x30 && pids[1] == 0x31 && pids[2] == 0x100 && pids[3] == 0x101);
	prog.pcr_pid = 0x1fff;
	n = dth11_program_pid_list(&prog, pids);
	CHECK(n == 4 && pids[2] == 0x100 && pids[3] == 0x101);
}

static void test_incomplete_and_order(void)
{
	struct dth11_psi psi;
	u8 sec[1024], pkt[188];
	struct dth11_program progs[4];
	static const u16 nums[2] = { 0x401, 0x400 }, ppid[2] = { 0x1f1, 0x1f0 };
	static const u16 es[1] = { 0x100 };
	size_t n;

	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 2);
	packet(pkt, 0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	n = make_pmt(sec, 0x401, 0x1ff, 0, es, 1, 0);
	packet(pkt, 0x1f1, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	/* 全ての番組の PMT がそろうまで完了にしない */
	CHECK(!dth11_psi_complete(&psi));
	n = make_pmt(sec, 0x400, 0x1ff, 0, es, 1, 0);
	packet(pkt, 0x1f0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(dth11_psi_complete(&psi));
	n = dth11_psi_programs(&psi, progs, 4);
	/* 番組番号の昇順 (PAT の並びに依らない) */
	CHECK(n == 2 && progs[0].number == 0x400 && progs[1].number == 0x401);
	/* 出力先が小さいときは書ける分だけ */
	CHECK(dth11_psi_programs(&psi, progs, 1) == 1);
}

static void test_bad_input(void)
{
	struct dth11_psi psi;
	u8 sec[1024], pkt[188];
	static const u16 nums[1] = { 0x400 }, ppid[1] = { 0x1f0 };
	static const u16 es[1] = { 0x100 };
	size_t n;

	/* CRC の合わない PAT は採用しない */
	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 1);
	sec[n - 1] ^= 1;
	packet(pkt, 0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(psi.pat_count == 0);

	/* TEI の立ったパケットは無視 */
	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 1);
	packet(pkt, 0, true, sec, n, 0);
	pkt[1] |= 0x80;
	dth11_psi_feed(&psi, pkt);
	CHECK(psi.pat_count == 0);
	/* 同期バイトのないものも無視 */
	packet(pkt, 0, true, sec, n, 0);
	pkt[0] = 0x00;
	dth11_psi_feed(&psi, pkt);
	CHECK(psi.pat_count == 0);

	/* PAT が示さない PID の PMT は無視 */
	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 1);
	packet(pkt, 0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	n = make_pmt(sec, 0x400, 0x1ff, 0, es, 1, 0);
	packet(pkt, 0x1f5, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(!dth11_psi_complete(&psi));
	/* CRC の合わない PMT は採用しない */
	sec[n - 2] ^= 1;
	packet(pkt, 0x1f0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(!dth11_psi_complete(&psi));
	/* ペイロードのないパケット (アダプテーションのみ) は無視 */
	n = make_pmt(sec, 0x400, 0x1ff, 0, es, 1, 0);
	packet(pkt, 0x1f0, true, sec, n, 0);
	pkt[3] = 0x20;
	dth11_psi_feed(&psi, pkt);
	CHECK(!dth11_psi_complete(&psi));
	/* 正しい PMT で完了 */
	packet(pkt, 0x1f0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(dth11_psi_complete(&psi));
}

static void test_multi_packet_section(void)
{
	struct dth11_psi psi;
	u8 sec[1024], pkt[188];
	struct dth11_program prog;
	static const u16 nums[1] = { 0x400 }, ppid[1] = { 0x1f0 };
	u16 es[40];
	size_t n, i, off;

	/* PMT が 1 パケット (183 バイト) に収まらない: ES を 32 個 (1 つに CA 記述子つき) で 188 バイト */
	for (i = 0; i < 40; i++)
		es[i] = (u16)(0x100 + i);
	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 1);
	packet(pkt, 0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);
	n = make_pmt(sec, 0x400, 0x1ff, 0x901, es, 32, 0x902);
	CHECK(n > 183);
	/* 先頭のパケット (pointer 付き、183 バイト) と、続きのパケット */
	packet(pkt, 0x1f0, true, sec, 183, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(!dth11_psi_complete(&psi));
	for (off = 183, i = 1; off < n; i++) {
		size_t chunk = n - off > 184 ? 184 : n - off;

		packet(pkt, 0x1f0, false, sec + off, chunk, (u8)i);
		dth11_psi_feed(&psi, pkt);
		off += chunk;
	}
	CHECK(dth11_psi_complete(&psi));
	CHECK(dth11_psi_programs(&psi, &prog, 1) == 1);
	CHECK(prog.es_count == 32 && prog.es_pids[31] == 0x100 + 31);
	/* ES の CA 記述子の ECM も集める (番組の記述子の ECM と合わせて 2 つ) */
	CHECK(prog.ecm_count == 2 && prog.ecm_pids[0] == 0x901 && prog.ecm_pids[1] == 0x902);
}

static void test_pointer_field_continuation(void)
{
	struct dth11_psi psi;
	u8 sec[1024], sec2[1024], pkt[188], payload[184];
	static const u16 nums[1] = { 0x400 }, ppid[1] = { 0x1f0 };
	static const u16 es[1] = { 0x100 };
	size_t n, n2, split;

	/* 1 つ目のセクションの途中までを 1 つのパケットで送り、続きを、次のセクションの先頭の前 (pointer) に載せる */
	dth11_psi_init(&psi);
	n = make_pat(sec, nums, ppid, 1);
	packet(pkt, 0, true, sec, n, 0);
	dth11_psi_feed(&psi, pkt);

	n = make_pmt(sec, 0x400, 0x1ff, 0, es, 1, 0);
	n2 = make_pmt(sec2, 0x400, 0x1ff, 0x901, es, 1, 0); /* 次のセクション: ECM つき */
	split = 20;
	/* パケット 1: 先頭から split バイト */
	packet(pkt, 0x1f0, true, sec, split, 0);
	dth11_psi_feed(&psi, pkt);
	CHECK(!dth11_psi_complete(&psi));
	/* パケット 2: pointer = 残り、続いて次のセクション */
	memset(payload, 0xff, sizeof(payload));
	payload[0] = (u8)(n - split);
	memcpy(payload + 1, sec + split, n - split);
	memcpy(payload + 1 + (n - split), sec2, n2);
	memset(pkt, 0xff, 188);
	pkt[0] = 0x47;
	pkt[1] = 0x40 | 0x01;
	pkt[2] = 0xf0;
	pkt[3] = 0x11;
	memcpy(pkt + 4, payload, 184);
	dth11_psi_feed(&psi, pkt);
	CHECK(dth11_psi_complete(&psi));
	{
		struct dth11_program prog;

		/* 後のセクション (ECM つき) が、前のセクションを置き換える */
		CHECK(dth11_psi_programs(&psi, &prog, 1) == 1);
		CHECK(prog.ecm_count == 1 && prog.ecm_pids[0] == 0x901);
	}
}

int main(void)
{
	test_crc();
	test_pat_pmt();
	test_dedupe_and_no_pcr();
	test_incomplete_and_order();
	test_bad_input();
	test_multi_packet_section();
	test_pointer_field_continuation();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
