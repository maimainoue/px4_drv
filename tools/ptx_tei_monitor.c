// SPDX-License-Identifier: GPL-2.0-only
/*
 * 受信しながら、一定の間隔ごとに TS のパケット数、ヌル、TEI、連続性の飛び、同期ずれを数えて 1 行ずつ表示する。
 * TS は保存しない。長時間の受信で、誤りの出る時刻を調べたり、複数の受信機 (同時に起動した複数のプロセス) の
 * 誤りの出る時刻を比べたりする用途。
 *
 * build: gcc -Wall -Wextra -O2 -I include -I driver/common/include tools/ptx_tei_monitor.c -o ptx_tei_monitor
 * usage: ptx_tei_monitor [options] DEVICE TOTAL_SEC
 * 詳細は docs/tools.md を参照。
 */
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include "ptx_tool.h"

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static void usage(const char *name)
{
	fprintf(stderr,
		"usage: %s [options] DEVICE TOTAL_SEC\n"
		"  DEVICE       デバイスファイル (例: /dev/px4video2、/dev/tornevideo0、/dev/dt295video0)\n"
		"  TOTAL_SEC    受信する秒数 (0 = Ctrl+C まで)\n"
		"options:\n"
		PTX_TARGET_USAGE
		"  -i SEC       集計して表示する間隔 (秒、既定 10)\n"
		"  -r           各行に、PTX_GET_CNR の生の値を cnr_raw として加える\n"
		"  -T           各行の先頭に、時刻 (時:分:秒) を加える (複数のプロセスの出力を、時刻で突き合わせるため)\n"
		"  -e           全体に 1 つでも誤り (TEI、連続性の飛び、同期ずれ) があれば、終了コードを 1 にする\n"
		"  -h           この説明を表示する\n",
		name);
}

static void print_line(double elapsed, int wall, const struct ts_counts *c, int fd, int with_cnr)
{
	char when[16] = "";
	char cnr[32] = "";

	if (wall) {
		time_t t = time(NULL);
		struct tm tm;

		localtime_r(&t, &tm);
		strftime(when, sizeof(when), "%H:%M:%S ", &tm);
	}
	if (with_cnr) {
		uint32_t cn = 0;

		if (ioctl(fd, PTX_GET_CNR, &cn) == 0)
			snprintf(cnr, sizeof(cnr), " cnr_raw=%u", cn);
		else
			snprintf(cnr, sizeof(cnr), " cnr_raw=n/a");
	}
	printf("%s%.0fs packets=%lu null=%lu tei=%lu cc=%lu dup=%lu sync=%lu%s\n", when, elapsed, c->packets, c->nulls, c->tei,
	       c->cc_jumps, c->cc_dups, c->sync_loss, cnr);
	fflush(stdout);
}

int main(int argc, char **argv)
{
	static uint8_t buffer[1 << 16];
	static struct ts_state state;
	struct ts_counts total_counts = { 0 };
	struct ptx_target target;
	char msg[128], text[96];
	double total, interval = 10.0, t_start, next_print;
	int opt, fd, wall = 0, with_cnr = 0, fail_on_error = 0, rc = 0;
	struct sigaction sa;

	ptx_target_init(&target);
	while ((opt = getopt(argc, argv, PTX_TARGET_OPTSTRING "i:rTeh")) != -1) {
		switch (opt) {
		case 'i':
			interval = strtod(optarg, NULL);
			if (interval <= 0) {
				fprintf(stderr, "-i には、正の秒数を指定してください\n");
				return 2;
			}
			break;
		case 'r':
			with_cnr = 1;
			break;
		case 'T':
			wall = 1;
			break;
		case 'e':
			fail_on_error = 1;
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
	total = strtod(argv[optind + 1], NULL);

	fd = open(argv[optind], O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", argv[optind], strerror(errno));
		return 1;
	}
	ptx_target_describe(&target, text, sizeof(text));
	if (ptx_tune(fd, &target, msg, sizeof(msg)) < 0) {
		fprintf(stderr, "%s で失敗しました: %s\n", msg, ptx_tune_errno_text(errno));
		close(fd);
		return 1;
	}
	if (ioctl(fd, PTX_START_STREAMING) < 0) {
		fprintf(stderr, "PTX_START_STREAMING で失敗しました: %s\n", strerror(errno));
		ptx_tune_release(fd, &target);
		close(fd);
		return 1;
	}
	fprintf(stderr, "# %s: %s、間隔 %.0f 秒%s\n", argv[optind], text, interval, total > 0 ? "" : "、Ctrl+C で終了");

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	ts_state_init(&state);
	t_start = ptx_now_s();
	next_print = interval;
	while (!g_stop && (total <= 0 || ptx_now_s() - t_start < total)) {
		ssize_t n = read(fd, buffer, sizeof(buffer));

		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			fprintf(stderr, "read: %s\n", strerror(errno));
			rc = 1;
			break;
		}
		ts_state_feed(&state, buffer, (size_t)n);
		if (ptx_now_s() - t_start >= next_print) {
			print_line(next_print, wall, &state.counts, fd, with_cnr);
			ts_counts_add(&total_counts, &state.counts);
			memset(&state.counts, 0, sizeof(state.counts));
			next_print += interval;
		}
	}
	/* 最後の端数の区間も表示する */
	if (state.counts.packets || !ts_counts_clean(&state.counts)) {
		print_line(ptx_now_s() - t_start, wall, &state.counts, fd, with_cnr);
		ts_counts_add(&total_counts, &state.counts);
	}
	ioctl(fd, PTX_STOP_STREAMING);
	ptx_tune_release(fd, &target);
	close(fd);

	printf("total packets=%lu null=%lu tei=%lu cc=%lu dup=%lu sync=%lu\n", total_counts.packets, total_counts.nulls,
	       total_counts.tei, total_counts.cc_jumps, total_counts.cc_dups, total_counts.sync_loss);
	if (rc)
		return rc;
	if (!total_counts.packets) {
		fprintf(stderr, "TS を受信できませんでした\n");
		return 1;
	}
	return fail_on_error && !ts_counts_clean(&total_counts) ? 1 : 0;
}
