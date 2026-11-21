// SPDX-License-Identifier: GPL-2.0-only
/*
 * px4_drv のカーネルモジュールの内蔵カードリーダーのデバイスファイル (/dev/px4card/<ID>) の試験。
 * 電源投入、カードの検出、リセット、ATR の受信、通信速度の切り替え、T=1 の S ブロック (IFS 要求) の往復を行い、
 * 往復を指定の間隔で指定の秒数だけ繰り返して、成功と失敗と応答時間を数える。
 * 内蔵カードリーダーを持つ機種 (PX4 系、PX-MLT 系、ISDB2056 系、Sony CECH-ZD1 J、PIX-DT295、GV-MVP/XZ など) に使える。
 *
 * build: gcc -Wall -Wextra -O2 -I include tools/px4card_test.c -o px4card_test
 * usage: px4card_test [options] [DEVICE]
 * 詳細は docs/tools.md を参照。
 */
#include <dirent.h>
#include <fcntl.h>
#include <getopt.h>
#include <unistd.h>

#include "px4_card_ioctl.h"
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>

#define CARD_DIR "/dev/px4card"
#define NAME_SIZE 300	/* CARD_DIR + "/" + ファイル名 (最大 255 バイト) が収まる大きさ */

static double now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* 各操作の直前に、その操作の残り時間 (ms) を指定する (px4_card の契約。指定しないと期限が切れた状態になる) */
static int card_ioctl(int fd, unsigned long request, void *arg, uint32_t timeout_ms)
{
	if (ioctl(fd, PX4_CARD_SET_TIMEOUT, &timeout_ms) < 0 && errno != ENOTTY)
		return -1;
	return ioctl(fd, request, arg);
}

/* 最初のバイトが来てから idle_ms 途切れるまで集める。first_ms の間に 1 バイトも来なければ 0 を返す */
static size_t collect(int fd, uint8_t *buffer, size_t capacity, double first_ms, double idle_ms)
{
	size_t length = 0;
	double start = now_ms(), last = start;

	for (;;) {
		struct px4_card_frame frame;
		double now = now_ms();

		if ((!length && now - start > first_ms) || (length && now - last > idle_ms))
			break;
		memset(&frame, 0, sizeof(frame));
		frame.length = PX4_CARD_MAX_FRAME;
		if (card_ioctl(fd, PX4_CARD_READ, &frame, 200) < 0) {
			if (errno == EAGAIN || errno == ETIMEDOUT || errno == EINTR) {
				usleep(2000);
				continue;
			}
			break;
		}
		if (frame.length && length + frame.length <= capacity) {
			memcpy(buffer + length, frame.data, frame.length);
			length += frame.length;
			last = now_ms();
		} else {
			usleep(2000);
		}
	}
	return length;
}

static int list_devices(char names[][NAME_SIZE], int max)
{
	DIR *dir = opendir(CARD_DIR);
	struct dirent *entry;
	int n = 0;

	if (!dir)
		return 0;
	while ((entry = readdir(dir)) != NULL && n < max) {
		if (entry->d_name[0] == '.')
			continue;
		snprintf(names[n++], NAME_SIZE, "%s/%s", CARD_DIR, entry->d_name);
	}
	closedir(dir);
	return n;
}

static void usage(const char *name)
{
	fprintf(stderr,
		"usage: %s [options] [DEVICE]\n"
		"  DEVICE       カードリーダーのデバイスファイル (例: /dev/px4card/ED1A5DB35C10DE62)。\n"
		"               省略すると、/dev/px4card/ に 1 つだけあれば、それを使う\n"
		"options:\n"
		"  -t SEC       往復を繰り返す秒数 (既定 5。0 = 電源投入、ATR までで終える)\n"
		"  -I MS        往復の間隔 (ミリ秒、既定 200)\n"
		"  -b BAUD      ATR の後に切り替える通信速度。9600、19200、38400。0 = 切り替えない (既定 19200)\n"
		"  -l           /dev/px4card/ のデバイスファイルを一覧にして終わる\n"
		"  -h           この説明を表示する\n"
		"終了コード: 0 = すべて成功、1 = カードなし、ATR が来ない、または往復に失敗した、2 = 使い方の誤りまたは open の失敗\n",
		name);
}

int main(int argc, char **argv)
{
	static const uint8_t ifs[] = { 0x00, 0xc1, 0x01, 0xfe, 0x3e };	/* T=1 の S(IFS request)。LRC は 0x3e */
	struct px4_card_frame frame;
	uint8_t atr[64], reply[64];
	unsigned long exchanges = 0, failures = 0;
	double total = 0, max = 0, started, interval_ms = 200;
	unsigned int seconds = 5, baud = 19200;
	char found[16][NAME_SIZE];
	const char *path = NULL;
	uint32_t present = 0;
	size_t length, k;
	int opt, fd, ret, count, list = 0, rc = 0;

	while ((opt = getopt(argc, argv, "t:I:b:lh")) != -1) {
		switch (opt) {
		case 't':
			seconds = (unsigned int)strtoul(optarg, NULL, 0);
			break;
		case 'I':
			interval_ms = strtod(optarg, NULL);
			if (interval_ms < 0) {
				fprintf(stderr, "-I には、0 以上のミリ秒を指定してください\n");
				return 2;
			}
			break;
		case 'b':
			baud = (unsigned int)strtoul(optarg, NULL, 0);
			if (baud && baud != 9600 && baud != 19200 && baud != 38400) {
				fprintf(stderr, "-b には、0、9600、19200、38400 のいずれかを指定してください\n");
				return 2;
			}
			break;
		case 'l':
			list = 1;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		default:
			usage(argv[0]);
			return 2;
		}
	}
	count = list_devices(found, 16);
	if (list) {
		for (k = 0; (int)k < count; k++)
			printf("%s\n", found[k]);
		if (!count)
			fprintf(stderr, "%s にデバイスファイルがありません (px4_drv が読み込まれていないか、内蔵カードリーダーを持つ機材がありません)\n",
				CARD_DIR);
		return count ? 0 : 1;
	}
	if (optind < argc) {
		path = argv[optind];
	} else if (count == 1) {
		path = found[0];
	} else {
		if (!count)
			fprintf(stderr, "%s にデバイスファイルがありません\n", CARD_DIR);
		else {
			fprintf(stderr, "カードリーダーが複数あります。DEVICE を指定してください:\n");
			for (k = 0; (int)k < count; k++)
				fprintf(stderr, "  %s\n", found[k]);
		}
		return 2;
	}

	fd = open(path, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return 2;
	}
	printf("device: %s\n", path);
	ret = ioctl(fd, PX4_CARD_POWER_ON);
	printf("POWER_ON: ret=%d (errno %d)\n", ret, ret ? errno : 0);
	if (ret) {
		close(fd);
		return 1;
	}
	ret = card_ioctl(fd, PX4_CARD_DETECT, &present, 1000);
	printf("DETECT: ret=%d (errno %d) present=%u\n", ret, ret ? errno : 0, present);
	if (ret || !present) {
		printf("result: FAILED (カードが検出されません)\n");
		rc = 1;
		goto out;
	}
	ret = card_ioctl(fd, PX4_CARD_RESET, NULL, 3000);
	printf("RESET: ret=%d (errno %d)\n", ret, ret ? errno : 0);
	length = collect(fd, atr, sizeof(atr), 1500, 200);
	printf("ATR (%zu bytes):", length);
	for (k = 0; k < length; ++k)
		printf(" %02x", atr[k]);
	printf("\n");
	if (!length) {
		printf("result: FAILED (ATR が来ません)\n");
		rc = 1;
		goto out;
	}
	if (atr[0] == 0x3b)
		printf("  TS=0x3b (direct convention)\n");
	else if (atr[0] == 0x3f)
		printf("  TS=0x3f (inverse convention)\n");
	else
		printf("  TS=0x%02x (想定外の値)\n", atr[0]);
	if (baud) {
		ret = card_ioctl(fd, PX4_CARD_SET_BAUDRATE, &baud, 1000);
		printf("SET_BAUDRATE %u: ret=%d (errno %d)\n", baud, ret, ret ? errno : 0);
		if (ret) {
			rc = 1;
			goto out;
		}
	}
	started = now_ms();
	while (seconds && now_ms() - started < seconds * 1000.0) {
		double t0 = now_ms(), ms;

		memset(&frame, 0, sizeof(frame));
		frame.length = sizeof(ifs);
		memcpy(frame.data, ifs, sizeof(ifs));
		ret = card_ioctl(fd, PX4_CARD_WRITE, &frame, 1000);
		length = ret ? 0 : collect(fd, reply, sizeof(reply), 1000, 60);
		ms = now_ms() - t0;
		exchanges++;
		/* S(IFS response) の PCB は 0xe1 */
		if (ret || length < 5 || reply[1] != 0xe1)
			failures++;
		total += ms;
		if (ms > max)
			max = ms;
		while (now_ms() - t0 < interval_ms)
			usleep(2000);
	}
	if (seconds) {
		printf("exchanges %lu, failures %lu, mean %.1f ms, max %.1f ms\n", exchanges, failures,
		       exchanges ? total / (double)exchanges : 0.0, max);
		if (!exchanges || failures)
			rc = 1;
	}
out:
	ret = ioctl(fd, PX4_CARD_POWER_OFF);
	printf("POWER_OFF: ret=%d (errno %d)\n", ret, ret ? errno : 0);
	close(fd);
	if (!rc)
		printf("result: OK\n");
	else if (present)
		printf("result: FAILED\n");
	return rc;
}
