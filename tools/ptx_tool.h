// SPDX-License-Identifier: GPL-2.0-only
/*
 * ptx_tei_monitor と ptx_chrdev_test の共通部分 (Linux のカーネルモジュールのデバイスファイル用)。
 *
 *  - 選局: ISDB-T (地上波)、ISDB-S (BS / CS110)、ISDB-C (CATV)、J.83 Annex B を、機種に依らず同じ引数で選局する。
 *  - TS の集計: 同期ずれ、TEI、ヌル、連続性の飛び (PID ごと) を数える。
 *
 * 使い方は docs/tools.md を参照。
 * build: gcc -Wall -Wextra -I include -I driver/common/include tools/<ツール>.c -o <ツール>
 */
#ifndef PTX_TOOL_H
#define PTX_TOOL_H

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>

#include "isdbc_channel.h"
#include "ptx_ioctl.h"

#define TS_PACKET_SIZE 188
#define TS_PID_COUNT 8192

/* ---------------------------------------------------------------- 時刻 */

static inline double ptx_now_s(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* ---------------------------------------------------------------- 選局 */

/* 選局の指定。system は 'T' (ISDB-T)、'S' (ISDB-S)、'C' (ISDB-C)、'J' (J.83 Annex B) */
struct ptx_target {
	char system;
	int channel;		/* T: 物理チャンネル 13〜62、S: トランスポンダー 0〜23、C: CATV の C チャンネル 13〜63 */
	int has_channel;
	long freq_khz;		/* C と J: 中心周波数 (kHz)。指定すると channel より優先する */
	int has_freq_khz;
	int freq_no;		/* PTX_SET_CHANNEL の freq_no をそのまま渡す (T と S) */
	int has_freq_no;
	int slot;		/* S: PTX_SET_CHANNEL の slot (相対 TS 番号または TSID) */
	int tsmf;		/* C: TSMF の何番目の TS を受けるか (0〜14) */
	int has_tsmf;
	int lnb;		/* S: LNB 電源の電圧 (0 = 供給しない、11 または 15) */
};

/* getopt のオプション文字列と、選局のオプションの説明 (各ツールで共通) */
#define PTX_TARGET_OPTSTRING "s:c:f:n:S:m:L:"
#define PTX_TARGET_USAGE \
	"  -s SYSTEM    方式。T = ISDB-T (既定)、S = ISDB-S、C = ISDB-C、J = J.83 Annex B\n" \
	"  -c CHANNEL   チャンネル。T: UHF 13〜62、S: トランスポンダー 0〜23 (BS 0〜11、CS110 12〜23)、C: CATV の C13〜C63\n" \
	"  -f KHZ       C と J: 中心周波数 (kHz)。-c より優先する (J は必須)\n" \
	"  -n FREQ_NO   T と S: PTX_SET_CHANNEL の freq_no を直接指定する (-c より優先する)\n" \
	"  -S SLOT      S: PTX_SET_CHANNEL の slot (相対 TS 番号または TSID)。既定 0\n" \
	"  -m N         C: TSMF のうち、0 から数えて N 番目の TS を受ける (0〜14)。既定 0\n" \
	"  -L VOLT      S: LNB の電源を供給する。11 または 15 (V)。終了時に止める\n"

static inline void ptx_target_init(struct ptx_target *t)
{
	memset(t, 0, sizeof(*t));
	t->system = 'T';
}

/* getopt が返したオプション c を処理する。選局のオプションでなければ 0 を、処理したら 1 を、値が不正なら -1 を返す */
static inline int ptx_target_option(struct ptx_target *t, int c, const char *arg)
{
	char *end = NULL;
	long v;

	switch (c) {
	case 's':
		if (!arg[0] || arg[1] || !strchr("TSCJtscj", arg[0]))
			return -1;
		t->system = (char)(arg[0] & ~0x20);	/* 大文字へ */
		return 1;
	case 'c':
	case 'f':
	case 'n':
	case 'S':
	case 'm':
	case 'L':
		v = strtol(arg, &end, 0);
		if (end == arg || *end)
			return -1;
		break;
	default:
		return 0;
	}
	switch (c) {
	case 'c':
		t->channel = (int)v;
		t->has_channel = 1;
		break;
	case 'f':
		t->freq_khz = v;
		t->has_freq_khz = 1;
		break;
	case 'n':
		t->freq_no = (int)v;
		t->has_freq_no = 1;
		break;
	case 'S':
		t->slot = (int)v;
		break;
	case 'm':
		if (v < 0 || v > 14)
			return -1;
		t->tsmf = (int)v;
		t->has_tsmf = 1;
		break;
	case 'L':
		if (v != 0 && v != 11 && v != 15)
			return -1;
		t->lnb = (int)v;
		break;
	}
	return 1;
}

/* 指定の組み合わせを検査する。問題があれば、理由を msg へ書いて -1 を返す */
static inline int ptx_target_check(const struct ptx_target *t, char *msg, size_t size)
{
	switch (t->system) {
	case 'T':
		if (!t->has_freq_no && !(t->has_channel && t->channel >= 13 && t->channel <= 62)) {
			snprintf(msg, size, "ISDB-T は、-c に UHF の 13〜62 を指定するか、-n を指定してください");
			return -1;
		}
		break;
	case 'S':
		if (!t->has_freq_no && !(t->has_channel && t->channel >= 0 && t->channel <= 23)) {
			snprintf(msg, size, "ISDB-S は、-c にトランスポンダー 0〜23 を指定するか、-n を指定してください");
			return -1;
		}
		break;
	case 'C':
		if (!t->has_freq_khz && !(t->has_channel && t->channel >= 13 && t->channel <= 63)) {
			snprintf(msg, size, "ISDB-C は、-c に C13〜C63 を指定するか、-f に中心周波数 (kHz) を指定してください");
			return -1;
		}
		break;
	case 'J':
		if (!t->has_freq_khz || t->freq_khz < 90000 || t->freq_khz > 770000) {
			snprintf(msg, size, "J.83 Annex B は、-f に中心周波数 (90000〜770000 kHz) を指定してください");
			return -1;
		}
		break;
	default:
		snprintf(msg, size, "方式は T、S、C、J のいずれかです");
		return -1;
	}
	if (t->has_freq_khz && t->system != 'C' && t->system != 'J') {
		snprintf(msg, size, "-f は、ISDB-C (C) と J.83 Annex B (J) だけで使えます");
		return -1;
	}
	if (t->has_freq_no && t->system != 'T' && t->system != 'S') {
		snprintf(msg, size, "-n は、ISDB-T (T) と ISDB-S (S) だけで使えます");
		return -1;
	}
	if (t->has_tsmf && t->system != 'C') {
		snprintf(msg, size, "-m は、ISDB-C (C) だけで使えます");
		return -1;
	}
	if (t->lnb && t->system != 'S') {
		snprintf(msg, size, "-L は、ISDB-S (S) だけで使えます");
		return -1;
	}
	return 0;
}

/* errno を、選局の失敗の理由を示す文にする */
static inline const char *ptx_tune_errno_text(int e)
{
	switch (e) {
	case EAGAIN:
		return "ロックしませんでした (アンテナ未接続、信号なし、または受信条件の不一致)";
	case ETIMEDOUT:
		return "時間切れでした (モジュールの読み込み直後の最初の選局では、起きることがあります)";
	case EINVAL:
		return "このデバイスは、この方式またはチャンネルに対応していません";
	case EIO:
		return "機材との通信に失敗しました";
	case ENODEV:
		return "機材が取り外されました";
	default:
		return strerror(e);
	}
}

/* 選局する。成功なら 0、失敗なら -1 (errno は ioctl の失敗のまま、msg に、失敗した操作を書く) */
static inline int ptx_tune(int fd, const struct ptx_target *t, char *msg, size_t size)
{
	struct ptx_freq freq = { .freq_no = 0, .slot = 0 };
	struct ptxt_additional_param prop;
	struct ptxt_params params;
	unsigned long mode;
	int saved;

	msg[0] = '\0';
	switch (t->system) {
	case 'T':
	case 'S':
		mode = t->system == 'T' ? PTX_ISDB_T_SYSTEM : PTX_ISDB_S_SYSTEM;
		if (ioctl(fd, PTX_SET_SYSTEM_MODE, mode) < 0) {
			saved = errno;
			snprintf(msg, size, "PTX_SET_SYSTEM_MODE (%s)", t->system == 'T' ? "ISDB-T" : "ISDB-S");
			errno = saved;
			return -1;
		}
		if (t->system == 'S' && t->lnb) {
			if (ioctl(fd, PTX_ENABLE_LNB_POWER, t->lnb == 15 ? 2 : 1) < 0) {
				saved = errno;
				snprintf(msg, size, "PTX_ENABLE_LNB_POWER (%d V)", t->lnb);
				errno = saved;
				return -1;
			}
		}
		freq.freq_no = t->has_freq_no ? t->freq_no : (t->system == 'T' ? t->channel + 50 : t->channel);
		freq.slot = t->slot;
		if (ioctl(fd, PTX_SET_CHANNEL, &freq) < 0) {
			saved = errno;
			snprintf(msg, size, "PTX_SET_CHANNEL (freq_no=%d)", freq.freq_no);
			errno = saved;
			return -1;
		}
		return 0;
	case 'C':
	case 'J':
		memset(&params, 0, sizeof(params));
		params.system = t->system == 'C' ? PTX_ISDB_C_SYSTEM : PTX_J83B_SYSTEM;
		if (t->has_freq_khz) {
			params.freq = (__u32)t->freq_khz;
		} else {
			/* C13〜C22 は物理チャンネル 3〜12、C23〜C63 は 22〜62 (README の「録画 (px4rec)」と同じ対応) */
			int physical = t->channel <= 22 ? t->channel - 10 : t->channel - 1;

			params.freq = (__u32)isdbc_channel_to_khz(physical);
		}
		if (t->system == 'C' && t->has_tsmf) {
			prop.prop = PTXT_TSMF_STREAM_PARAM;
			prop.data = (__u32)t->tsmf;
			params.num_prop = 1;
			params.prop = (__u64)(uintptr_t)&prop;
		}
		if (ioctl(fd, PTXT_SET_PARAMS, &params) < 0) {
			saved = errno;
			snprintf(msg, size, "PTXT_SET_PARAMS (%u kHz)", params.freq);
			errno = saved;
			return -1;
		}
		if (ioctl(fd, PTXT_TUNE) < 0) {
			saved = errno;
			snprintf(msg, size, "PTXT_TUNE (%u kHz)", params.freq);
			errno = saved;
			return -1;
		}
		return 0;
	}
	snprintf(msg, size, "方式が不正です");
	errno = EINVAL;
	return -1;
}

/* 受信を終えるときに呼ぶ。LNB の電源を止める (供給していなければ何もしない) */
static inline void ptx_tune_release(int fd, const struct ptx_target *t)
{
	if (t->system == 'S' && t->lnb)
		ioctl(fd, PTX_DISABLE_LNB_POWER);
}

/* 選局の指定を、1 行の文にする (ログ用) */
static inline void ptx_target_describe(const struct ptx_target *t, char *out, size_t size)
{
	switch (t->system) {
	case 'T':
		if (t->has_freq_no)
			snprintf(out, size, "ISDB-T freq_no=%d", t->freq_no);
		else
			snprintf(out, size, "ISDB-T UHF %dch", t->channel);
		break;
	case 'S':
		if (t->has_freq_no)
			snprintf(out, size, "ISDB-S freq_no=%d slot=%d", t->freq_no, t->slot);
		else
			snprintf(out, size, "ISDB-S transponder %d slot=%d", t->channel, t->slot);
		break;
	case 'C':
		if (t->has_freq_khz)
			snprintf(out, size, "ISDB-C %ld kHz tsmf=%d", t->freq_khz, t->tsmf);
		else
			snprintf(out, size, "ISDB-C C%dch tsmf=%d", t->channel, t->tsmf);
		break;
	default:
		snprintf(out, size, "J.83B %ld kHz", t->freq_khz);
		break;
	}
}

/* ---------------------------------------------------------------- TS の集計 */

/* 数える項目 (区間ごとにゼロへ戻せる) */
struct ts_counts {
	unsigned long packets;		/* 同期できた TS パケット (ヌルを含む) */
	unsigned long nulls;		/* PID 0x1fff のパケット */
	unsigned long tei;		/* transport_error_indicator が立ったパケット */
	unsigned long cc_jumps;		/* 連続性の飛び (PID ごとに、前のカウンターの次でも同じでもない値) */
	unsigned long cc_dups;		/* 前と同じカウンターの値 (規格で 1 回までは許される再送。参考値) */
	unsigned long sync_loss;	/* 同期を失った回数 (0x47 が 188 バイト間隔から外れた回数) */
};

struct ts_state {
	struct ts_counts counts;
	unsigned long jumps_by_pid[TS_PID_COUNT];	/* 通算 */
	uint8_t last_cc[TS_PID_COUNT];
	uint8_t seen[TS_PID_COUNT];
	uint8_t pending[2 * TS_PACKET_SIZE];
	size_t pending_len;
	int synced;
};

static inline void ts_state_init(struct ts_state *s)
{
	memset(s, 0, sizeof(*s));
}

static inline void ts_counts_add(struct ts_counts *to, const struct ts_counts *from)
{
	to->packets += from->packets;
	to->nulls += from->nulls;
	to->tei += from->tei;
	to->cc_jumps += from->cc_jumps;
	to->cc_dups += from->cc_dups;
	to->sync_loss += from->sync_loss;
}

static inline int ts_counts_clean(const struct ts_counts *c)
{
	return !c->tei && !c->cc_jumps && !c->sync_loss;
}

/* 1 パケットを集計する (同期は済んでいる) */
static inline void ts_state_packet(struct ts_state *s, const uint8_t *p)
{
	unsigned int pid = ((unsigned int)(p[1] & 0x1f) << 8) | p[2];
	unsigned int afc = (p[3] >> 4) & 3;
	unsigned int cc = p[3] & 0x0f;
	int discontinuity = 0;

	s->counts.packets++;
	if (p[1] & 0x80)
		s->counts.tei++;
	if (pid == 0x1fff) {
		s->counts.nulls++;
		return;
	}
	/* TEI の付いたパケットは、中身が壊れている可能性があるため、連続性の検査へ使わない */
	if (p[1] & 0x80)
		return;
	/* ペイロードのないパケットは、連続性カウンターが進まない */
	if (!(afc & 1))
		return;
	/* アダプテーションフィールドの長さがパケットに収まることを確かめてから、discontinuity_indicator を読む */
	if ((afc & 2) && p[4] > 0 && p[4] <= 183 && (p[5] & 0x80))
		discontinuity = 1;
	if (discontinuity) {
		s->seen[pid] = 1;
		s->last_cc[pid] = (uint8_t)cc;
		return;
	}
	if (s->seen[pid]) {
		if (cc == s->last_cc[pid]) {
			s->counts.cc_dups++;
		} else if (cc != ((s->last_cc[pid] + 1u) & 0x0f)) {
			s->counts.cc_jumps++;
			s->jumps_by_pid[pid]++;
		}
	}
	s->seen[pid] = 1;
	s->last_cc[pid] = (uint8_t)cc;
}

/*
 * 読み取ったバイト列を集計する。188 バイトの境界を 0x47 で見つけ、外れたら 1 バイトずつ捨てて探し直す
 * (最初の位置合わせは、同期ずれに数えない)。途中のパケットは次の呼び出しへ持ち越す。
 */
static inline void ts_state_feed(struct ts_state *s, const uint8_t *data, size_t length)
{
	uint8_t work[sizeof(s->pending) + 65536];

	while (length) {
		size_t take = length > 65536 ? 65536 : length;
		size_t total, pos = 0;

		memcpy(work, s->pending, s->pending_len);
		memcpy(work + s->pending_len, data, take);
		total = s->pending_len + take;
		data += take;
		length -= take;

		while (pos + TS_PACKET_SIZE <= total) {
			if (!s->synced) {
				if (work[pos] != 0x47) {
					pos++;
					continue;
				}
				/* 次の同期バイトを確かめてから、同期したと見なす (データの中の 0x47 を誤って選ばないため) */
				if (pos + 2 * TS_PACKET_SIZE > total)
					break;
				if (work[pos + TS_PACKET_SIZE] != 0x47) {
					pos++;
					continue;
				}
				s->synced = 1;
			}
			if (work[pos] != 0x47) {
				s->counts.sync_loss++;
				s->synced = 0;
				continue;
			}
			ts_state_packet(s, work + pos);
			pos += TS_PACKET_SIZE;
		}
		s->pending_len = total - pos;
		if (s->pending_len > sizeof(s->pending))
			s->pending_len = sizeof(s->pending);	/* 同期を探している間だけ。最後の 376 バイトを残す */
		memcpy(s->pending, work + total - s->pending_len, s->pending_len);
	}
}

#endif /* PTX_TOOL_H */
