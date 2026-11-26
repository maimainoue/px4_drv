/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ut100b_core.h - Quixun QRS-UT100B (ASICEN ASV5211 + ASIE5606) の共通層 (地上波の受信とカード)
 *
 * OS に依存しない。USB の制御転送と時刻は、HZ2W の共通層 (hz2w_core.h) の struct hz2w_ops で受ける。
 * 同じ ASV5211 と ASIE5606 を使うため、firmware の送信、周期処理、内蔵カードの処理は HZ2W の共通層をそのまま使い、
 * この層は、復調器、チューナー、TS の設定と、TS の暗号の解除だけを持つ。
 *
 * 手順と値 (復調器とチューナーの初期化表、GPIO の順と待ち時間、周波数の式、TS フィルターの設定、DES の鍵) は、
 * 次の資料による。実機 (docs/devices/ut100b.md) では、ここの手順で TS が欠けなく取れることを確認した。
 *   - recfriio (Copyright (c) 2008 FUKAUMI Naoki、2 条項 BSD): recfriior5/Hdp.cpp。
 *     UT100B と HDP の対応は tslroom (2012)、stz2012 (2016) による追加。
 *   - waya0125/recfriio (MIT、Copyright (c) 2026 waya0125): internal/ut100b、internal/asie5606、internal/asv5211。
 *     レジスタの名前、0x1c の読み直し、C/N の式、受け始めのデータを捨てる処理、DES を外す処理。
 *   - Till0196/coax (Apache-2.0 OR GPL-2.0-or-later): ASIE5606 の手順。
 * 2 条項 BSD の条文は、このファイルの末尾にある。
 */
#ifndef UT100B_CORE_H
#define UT100B_CORE_H

#include "hz2w_core.h"

#define UT100B_USB_VENDOR 0x3275
#define UT100B_USB_PRODUCT 0x7020

/* TS が来る bulk IN のエンドポイントと、受信の設定 (recfriio の HDP の設定と同じ) */
#define UT100B_STREAM_ENDPOINT 0x81
#define UT100B_URB_COUNT 32
#define UT100B_URB_SIZE 16384

/* 受け始めに捨てるバイト数。機器の中に前回の受信の終わり (13 KB 以内) が残っていて、先に出てくる。 */
#define UT100B_STALE_DISCARD_BYTES (2 * UT100B_URB_SIZE)

#define UT100B_TS_PACKET_SIZE 188

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 機器の起動: 基板の電源、チューナーと復調器の初期化、ASIE5606 のリセットと TS の暗号の設定。
 * warm (3275:7020) の機器に対して呼ぶ。成功したら、周期処理 (hz2w_keepalive_*) を直ちに始める必要がある。
 * 内蔵カード用の hz2w_card は、ut100b_card_init() で作る。
 */
int ut100b_board_open(struct hz2w_bridge *bridge);

/*
 * 閉じる手順 (受信の停止、フィルターのリセット、カード側のレジスタを 0、チューナーの電源を切る)。
 * 周期処理を止める前に呼ぶ。失敗しても最後まで進める。
 */
void ut100b_board_close(struct hz2w_bridge *bridge);

/* 地上波の中心周波数 (kHz) を選局する。freq_khz は 473143 など (UHF 13ch の中心)。TS は流し始めない (ut100b_stream_start)。 */
int ut100b_tune(struct hz2w_bridge *bridge, unsigned int freq_khz);

/*
 * TS を流し始める (TS フィルターの設定と、受信の開始)。機器の中の FIFO (約 260 KB) は、始めた時点から溜まり、読み出されないと
 * 満杯のまま止まるため、ロックと安定の待ちが済み、読み出し (URB) を始める直前に呼ぶ。
 */
int ut100b_stream_start(struct hz2w_bridge *bridge);

/* 受信を止める (チャンネルの切り替えの前、読み出しを止めるとき、または閉じる前)。 */
int ut100b_stream_stop(struct hz2w_bridge *bridge);

/* 復調器が地上波を捕まえているか (TC90522 と同じ判定: レジスタ 0x80 の bit 3、5 が 0、かつレジスタ 0xb0 の下位 4 ビットが 8 以上)。 */
int ut100b_is_locked(struct hz2w_bridge *bridge, bool *locked);

/* 復調器の C/N の生の値 (24 ビット)。 */
int ut100b_read_cndat(struct hz2w_bridge *bridge, u32 *cndat);

/* 内蔵カードの hz2w_card を、UT100B 用に初期化する (TS の暗号の設定を壊さないよう、ASIE5606 を初期値へ戻さない)。 */
void ut100b_card_init(struct hz2w_card *card, struct hz2w_bridge *bridge);

/* ---- TS の暗号の解除 (ASIE5606 が USB 転送でかける DES。放送のスクランブルとは別のもの) ---- */

/*
 * DES の鍵と表。機器ごとに持ち (約 40 KB)、ut100b_cipher_init() で 1 度だけ作る。
 * 作った後は読み出しだけなので、複数のスレッドから同時に復号できる。
 */
struct ut100b_cipher {
	u64 ip[8][256];		/* 初期転置 */
	u64 fp[8][256];		/* 最終転置 */
	u64 e[4][256];		/* 拡張転置 */
	u32 sp[8][64];		/* S ボックスと P 転置 */
	u64 k_head[16];		/* 先頭の 128 バイトの鍵 (復号の順) */
	u64 k_tail[16];		/* 残りの 56 バイトの鍵 */
};

void ut100b_cipher_init(struct ut100b_cipher *cipher);

/* 188 バイトのパケット src の DES を外して dst に書く (dst と src は同じでもよい)。放送のスクランブルはそのまま残る。 */
void ut100b_cipher_decrypt_packet(const struct ut100b_cipher *cipher, u8 *dst, const u8 *src);

/* USB から届いたバイト列を 188 バイトのパケットに区切り、DES を外す。 */
struct ut100b_aligner {
	struct ut100b_cipher cipher;
	size_t stale;			/* まだ捨てるバイト数 */
	size_t rest_len;
	u8 rest[UT100B_TS_PACKET_SIZE];	/* 区切りに満たない残り */
	u64 resyncs;			/* 区切りを探し直した回数 */
	u64 skipped;			/* 読み飛ばしたバイト数 */
};

/* 受け始めの UT100B_STALE_DISCARD_BYTES を捨てる状態にする (DES の表も作るため、使う前に 1 度呼ぶ)。 */
void ut100b_aligner_init(struct ut100b_aligner *a);

/*
 * in (n バイト) を受け取り、DES を外した 188 バイト単位の TS を out へ書く。戻り値は書いたバイト数。
 * out の容量は ut100b_aligner_out_capacity(a, n) 以上にする。半端な残りは次の呼び出しまで持つ。
 */
size_t ut100b_aligner_feed(struct ut100b_aligner *a, const u8 *in, size_t n, u8 *out);
size_t ut100b_aligner_out_capacity(const struct ut100b_aligner *a, size_t n);

#ifdef __cplusplus
}
#endif

/*
 * recfriio の 2 条項 BSD (Copyright (c) 2008 FUKAUMI Naoki):
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 * the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#endif
