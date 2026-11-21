// SPDX-License-Identifier: GPL-2.0-only
/*
 * px4_drv のカーネルモジュールのデバイスファイル (/dev/px4video*、/dev/tornevideo*、/dev/dt295video* など) から、
 * 選局して指定の秒数だけ TS を受信し、品質 (同期、TEI、PID ごとの連続性の飛び、ヌル、速度) を集計する試験。
 * 受信した TS は、ファイルへ保存することもできる。
 *
 * build: gcc -Wall -Wextra -O2 -I include -I driver/common/include tools/ptx_chrdev_test.c -o ptx_chrdev_test
 * usage: ptx_chrdev_test [options] DEVICE SECONDS
 * 詳細は docs/tools.md を参照。
 */
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <unistd.h>

#include "ptx_tool.h"

#define MAX_BINS 3600	/* 秒ごとの集計の上限 (1 時間まで) */

static void usage(const char *name)
{
	fprintf(stderr,
		"usage: %s [options] DEVICE SECONDS\n"
		"  DEVICE       デバイスファイル (例: /dev/px4video2、/dev/tornevideo0、/dev/dt295video0)\n"
		"  SECONDS      受信する秒数 (1〜%d)\n"
		"options:\n"
		PTX_TARGET_USAGE
		"  -o FILE      受信した TS を FILE へ保存する (同期したパケットだけでなく、読み取ったバイト列をそのまま書く)\n"
		"  -w SEC       選局してから、受信を始めるまで待つ秒数 (選局直後の TEI と連続性エラーを調べるため。既定 0)\n"
		"  -p           連続性の飛びが多かった PID を、上位 10 個まで表示する\n"
		"  -h           この説明を表示する\n"
		"終了コード: 0 = 誤りなし、1 = 誤りがある、または選局や受信に失敗した、2 = 使い方の誤り\n",
		name, MAX_BINS);
}

/* 累積の集計 now から、前の時点 before を引いた、その間の数 */
static struct ts_counts counts_since(const struct ts_counts *now, const struct ts_counts *before)
{
	struct ts_counts d;

	d.packets = now->packets - before->packets;
	d.nulls = now->nulls - before->nulls;
	d.tei = now->tei - before->tei;
	d.cc_jumps = now->cc_jumps - before->cc_jumps;
	d.cc_dups = now->cc_dups - before->cc_dups;
	d.sync_loss = now->sync_loss - before->sync_loss;
	return d;
}

struct pid_rank {
	unsigned int pid;
	unsigned long jumps;
};

static int by_jumps(const void *a, const void *b)
{
	const struct pid_rank *x = a, *y = b;

	if (x->jumps != y->jumps)
		return x->jumps < y->jumps ? 1 : -1;
	return x->pid > y->pid ? 1 : -1;
}

int main(int argc, char **argv)
{
	static uint8_t buffer[1 << 16];
	static struct ts_state state;
	static struct ts_counts bins[MAX_BINS];
	struct ts_counts snapshot = { 0 };
	struct ptx_target target;
	char msg[128], text[96];
	const char *out_path = NULL;
	FILE *out = NULL;
	unsigned int seconds, bin_count, errored = 0;
	double wait_s = 0, t_start, t_end, first_ts = -1, elapsed;
	unsigned long long bytes = 0;
	uint32_t cn = 0;
	int opt, fd, per_pid = 0, ret, cnr_ok, i;

	ptx_target_init(&target);
	while ((opt = getopt(argc, argv, PTX_TARGET_OPTSTRING "o:w:ph")) != -1) {
		switch (opt) {
		case 'o':
			out_path = optarg;
			break;
		case 'w':
			wait_s = strtod(optarg, NULL);
			if (wait_s < 0) {
				fprintf(stderr, "-w には、0 以上の秒数を指定してください\n");
				return 2;
			}
			break;
		case 'p':
			per_pid = 1;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		case '?':
			usage(argv[0]);
			return 2;
		default:
			if (ptx_target_option(&target, opt, optarg) < 0) {
				fprintf(stderr, "-%c の値が不正です: %s\n", opt, optarg);
				return 2;
			}
			break;
		}
	}
	if (argc - optind != 2) {
		usage(argv[0]);
		return 2;
	}
	if (ptx_target_check(&target, msg, sizeof(msg)) < 0) {
		fprintf(stderr, "%s\n", msg);
		return 2;
	}
	seconds = (unsigned int)strtoul(argv[optind + 1], NULL, 0);
	if (seconds < 1 || seconds > MAX_BINS) {
		fprintf(stderr, "SECONDS は 1〜%d を指定してください\n", MAX_BINS);
		return 2;
	}
	if (out_path) {
		out = fopen(out_path, "wb");
		if (!out) {
			fprintf(stderr, "fopen %s: %s\n", out_path, strerror(errno));
			return 1;
		}
	}

	fd = open(argv[optind], O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", argv[optind], strerror(errno));
		return 1;
	}
	ptx_target_describe(&target, text, sizeof(text));
	printf("device: %s\ntarget: %s\n", argv[optind], text);
	if (ptx_tune(fd, &target, msg, sizeof(msg)) < 0) {
		printf("tune: FAILED at %s (errno %d): %s\n", msg, errno, ptx_tune_errno_text(errno));
		close(fd);
		return 1;
	}
	printf("tune: ok\n");
	if (wait_s > 0) {
		printf("wait: %.1f s\n", wait_s);
		fflush(stdout);
		poll(NULL, 0, (int)(wait_s * 1000));
	}
	cnr_ok = ioctl(fd, PTX_GET_CNR, &cn) == 0;
	if (cnr_ok)
		printf("PTX_GET_CNR: raw=%u (値の単位は機種による。docs/tools.md を参照)\n", cn);
	else
		printf("PTX_GET_CNR: not available (errno %d)\n", errno);
	ret = ioctl(fd, PTX_START_STREAMING);
	if (ret < 0) {
		printf("PTX_START_STREAMING: FAILED (errno %d)\n", errno);
		ptx_tune_release(fd, &target);
		close(fd);
		return 1;
	}

	ts_state_init(&state);
	t_start = ptx_now_s();
	bin_count = 0;
	while ((elapsed = ptx_now_s() - t_start) < (double)seconds) {
		ssize_t n = read(fd, buffer, sizeof(buffer));
		unsigned int bin;

		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			printf("read: %s\n", strerror(errno));
			break;
		}
		if (n > 0 && first_ts < 0)
			first_ts = ptx_now_s() - t_start;
		bytes += (unsigned long long)n;
		if (out && n > 0)
			fwrite(buffer, 1, (size_t)n, out);
		ts_state_feed(&state, buffer, (size_t)n);
		/* 秒ごとの集計。区間をまたいだら、直前の区間の差分を記録する */
		bin = (unsigned int)(ptx_now_s() - t_start);
		while (bin_count < bin && bin_count < MAX_BINS) {
			bins[bin_count++] = counts_since(&state.counts, &snapshot);
			snapshot = state.counts;
		}
	}
	/* 最後の区間 (1 秒に満たない端数) も記録する */
	if (bin_count < MAX_BINS)
		bins[bin_count++] = counts_since(&state.counts, &snapshot);
	t_end = ptx_now_s();
	ret = ioctl(fd, PTX_STOP_STREAMING);
	ptx_tune_release(fd, &target);
	close(fd);
	if (out)
		fclose(out);

	elapsed = t_end - t_start;
	printf("PTX_STOP_STREAMING: ret=%d\n", ret);
	printf("received %llu bytes in %.2f s (%.2f Mbps)\n", bytes, elapsed, elapsed > 0 ? (double)bytes * 8 / elapsed / 1e6 : 0.0);
	if (first_ts >= 0)
		printf("first data after START_STREAMING: %.0f ms\n", first_ts * 1000.0);
	printf("TS packets %lu (null %lu, %.1f%%), TEI %lu, continuity jumps %lu (duplicates %lu), sync loss %lu\n",
	       state.counts.packets, state.counts.nulls,
	       state.counts.packets ? 100.0 * (double)state.counts.nulls / (double)state.counts.packets : 0.0, state.counts.tei,
	       state.counts.cc_jumps, state.counts.cc_dups, state.counts.sync_loss);
	if (state.counts.packets > state.counts.nulls)
		printf("error rate (non-null packets): TEI %.4f%%, continuity jumps %.4f%%\n",
		       100.0 * (double)state.counts.tei / (double)(state.counts.packets - state.counts.nulls),
		       100.0 * (double)state.counts.cc_jumps / (double)(state.counts.packets - state.counts.nulls));

	for (i = 0; i < (int)bin_count; i++)
		if (!ts_counts_clean(&bins[i]))
			errored++;
	if (errored) {
		printf("seconds with errors: %u of %u (second: tei/cc/sync)\n", errored, bin_count);
		for (i = 0, errored = 0; i < (int)bin_count && errored < 20; i++) {
			if (ts_counts_clean(&bins[i]))
				continue;
			printf("  %ds: %lu/%lu/%lu\n", i, bins[i].tei, bins[i].cc_jumps, bins[i].sync_loss);
			errored++;
		}
	}
	if (per_pid && state.counts.cc_jumps) {
		static struct pid_rank rank[TS_PID_COUNT];
		unsigned int pid, n = 0;

		for (pid = 0; pid < TS_PID_COUNT; pid++) {
			if (!state.jumps_by_pid[pid])
				continue;
			rank[n].pid = pid;
			rank[n].jumps = state.jumps_by_pid[pid];
			n++;
		}
		qsort(rank, n, sizeof(rank[0]), by_jumps);
		printf("continuity jumps by PID (top %u):\n", n > 10 ? 10 : n);
		for (i = 0; i < (int)n && i < 10; i++)
			printf("  PID 0x%04x: %lu\n", rank[i].pid, rank[i].jumps);
	}
	if (!state.counts.packets) {
		printf("result: FAILED (TS を受信できませんでした)\n");
		return 1;
	}
	if (!ts_counts_clean(&state.counts)) {
		printf("result: ERRORS\n");
		return 1;
	}
	printf("result: OK\n");
	return 0;
}
