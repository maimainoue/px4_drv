/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_psi.h - DT-H11/U2 の device モードで、番組の PID リストを作るための PAT/PMT の収集
 *
 * device モードのデバイスは、番組 1 つ分の PID リスト (wValue 0xCBDA / wIndex 0x1000) を受け取って、
 * その番組だけをデバイス内蔵のカードで復号する。このためにホストが、選局したチャンネルの
 * PAT と PMT を TS から読み、番組の PID (PMT、ECM、PCR、ES) を集める。
 * PAT と PMT のセクションの CRC32 が合わないものは採用しない (弱い電波で壊れたセクションから
 * 誤った PID リストを作らないため)。
 * 手順は i4M1k0SU/DTH10-series-driver (MIT License、Copyright (c) i4M1k0SU) の
 * crates/dth10-backend/src/psi.rs による。
 */
#ifndef DTH11_PSI_H
#define DTH11_PSI_H

#include "dth11_core.h"

#define DTH11_PSI_MAX_PROGRAMS 32
#define DTH11_PSI_MAX_ECM 16
#define DTH11_PSI_MAX_ES 32
/* 番組の PID リストの最大の長さ: PMT 1、ECM、PCR 1、ES */
#define DTH11_PROGRAM_MAX_PIDS (1 + DTH11_PSI_MAX_ECM + 1 + DTH11_PSI_MAX_ES)
#define DTH11_PSI_SECTION_MAX 1024

struct dth11_program {
	u16 number;
	u16 pmt_pid;
	u16 pcr_pid;
	u16 ecm_pids[DTH11_PSI_MAX_ECM];
	u8 ecm_count;
	u16 es_pids[DTH11_PSI_MAX_ES];
	u8 es_count;
};

struct dth11_psi_section {
	u16 pid;
	u16 len;
	u8 data[DTH11_PSI_SECTION_MAX];
};

struct dth11_psi {
	/* PAT (PID 0) と、PAT が示す PMT の PID ごとのセクションの組み立て領域 */
	struct dth11_psi_section sections[1 + DTH11_PSI_MAX_PROGRAMS];
	u8 section_count;
	/* PAT: 番組番号と PMT の PID */
	u16 pat_number[DTH11_PSI_MAX_PROGRAMS];
	u16 pat_pmt_pid[DTH11_PSI_MAX_PROGRAMS];
	u8 pat_count;
	struct dth11_program pmt[DTH11_PSI_MAX_PROGRAMS];
	bool pmt_seen[DTH11_PSI_MAX_PROGRAMS];
};

#ifdef __cplusplus
extern "C" {
#endif

void dth11_psi_init(struct dth11_psi *psi);
/* 188 バイトの TS パケット 1 つを渡す。TEI の立ったもの、同期バイトのないものは無視する */
void dth11_psi_feed(struct dth11_psi *psi, const u8 *pkt);
/* PAT を受け取っていて、PAT の全ての番組の PMT がそろった */
bool dth11_psi_complete(const struct dth11_psi *psi);
/* 集めた番組を、番組番号の昇順で out へ書く。書いた数を返す */
size_t dth11_psi_programs(const struct dth11_psi *psi, struct dth11_program *out, size_t max);
/* 番組の PID リスト: PMT、ECM、PCR (0x1FFF を除く)、ES の順で、重複を除く。書いた数を返す */
size_t dth11_program_pid_list(const struct dth11_program *prog, u16 out[DTH11_PROGRAM_MAX_PIDS]);

/* MPEG-2 の CRC32 (多項式 0x04C11DB7、初期値 0xFFFFFFFF、出力の反転なし) */
u32 dth11_mpeg_crc32(const u8 *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_PSI_H */
