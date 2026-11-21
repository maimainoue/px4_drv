/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xz_core.h - GV-MVP/XZ2 (I-O DATA、Fujitsu MB86H58A + TOSHIBA TC90512 + チューナーモジュール) の共通層
 *
 * OS 非依存。USB の制御転送と時間は struct xz_ops の関数で受け取るので、同じコードを、ユーザー空間の試験
 * (libusb) とカーネルモジュールの両方で使える。機材の仕組みと検証の範囲は docs/devices/xz.md を参照。
 *
 * 手順は GV-MVP/XZ2 の 1 台の実機で確認したもの (USB ID 04bb:0530 は GV-MVP/XZ、XZ3 と共通で、他の版は未確認)。
 * 機材の仕組み:
 *  - ホストはベンダー要求 0xBC で、機材のメモリとレジスタを、24 ビットのアドレスで直接読み書きする。
 *    読み出しは 1 回 64 バイトまで、書き込みは **1 回 4 バイトまで** (64 バイトの書き込みは失敗した)。
 *  - 取り込みブロック (0xD00000) は 24 ビットの範囲内にあるので、DT295 のような boot0 の窓の書き換えは使わない。
 *  - 機材内の自作プログラム (boot0 の hook から動く) は、SDRAM のリングから bulk IN へ TS を送るだけにする。
 *    引数と結果は、本体の領域内の郵便受けを通る。DT295 の作業領域 (0xC4620) と違い、組の間に別の制御転送が入っても壊れない。
 *  - 前面 (復調器とチューナー) は、SoC の I2C コントローラ (0xFD8FE0 台) から操作する。
 */
#ifndef XZ_CORE_H
#define XZ_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#endif

#define XZ_USB_VENDOR 0x04bb
#define XZ_USB_PRODUCT 0x0530

#define XZ_TS_PACKET_SIZE 188
#define XZ_UNIT_SIZE 192	/* 4 バイトのタイムスタンプ + 188 バイトの TS */

/* SDRAM のリング (CPU のアドレス)。 */
#define XZ_RING_BASE 0x8138ec00u
#define XZ_RING_SIZE 0x30000u
#define XZ_PACKET_BYTES 512u
#define XZ_PUMP_MAX_PACKETS 128u
/* 受信側の非同期転送 (8 KiB) で割り切れるように、ポンプのパケット数は 16 の倍数にする。 */
#define XZ_PUMP_UNIT_PACKETS 16u

#define XZ_LOG_ERR 0
#define XZ_LOG_INFO 1
#define XZ_LOG_DEBUG 2

/* ホストから見た機材: USB の制御転送、時間、ログ。排他は呼び出し側 (1 転送ずつ完結させる) が持つ。 */
struct xz_ops {
	/* ベンダー要求 0xBC の制御転送 (request_type は 0xC0 または 0x40)。長さは読みが 64 バイト以下、書きが 4 バイト以下。長さ、または -errno を返す。 */
	int (*control)(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms);
	void (*msleep)(void *priv, unsigned int ms);
	u64 (*now_ms)(void *priv); /* 単調増加のミリ秒 */
	void (*log)(void *priv, int level, const char *fmt, ...);
};

struct xz_bridge {
	const struct xz_ops *ops;
	void *priv;
	bool program_installed;	/* hook と本体を導入している */
	u8 saved_body[512];	/* 導入前の本体の領域 (0xE1000〜) */
};

#ifdef __cplusplus
extern "C" {
#endif

void xz_bridge_init(struct xz_bridge *b, const struct xz_ops *ops, void *priv);

/* ---- リンク (xz_link.c): 24 ビットのアドレス ---- */
int xz_read(struct xz_bridge *b, u32 address, void *buf, unsigned int len);	/* 64 バイトずつに分ける */
int xz_write(struct xz_bridge *b, u32 address, const void *buf, unsigned int len);	/* 4 バイトずつに分ける */
int xz_read32(struct xz_bridge *b, u32 address, u32 *value);
int xz_write32(struct xz_bridge *b, u32 address, u32 value);

/* ---- ボード (xz_board.c) ---- */
/*
 * 起動直後の確認 (チップ ID、boot0 の署名、窓の即値、状態、hook の元の値) と、SoC の初期化の再生。
 * 確認に失敗したら何も書かずに -ENODEV (別の版の boot0)、起動直後でなければ -EBUSY。
 */
int xz_board_open(struct xz_bridge *b);
/* I2C (7 ビットアドレス)。-ETIMEDOUT (時間切れ) / -ENXIO (ACK なし) / -EIO。 */
int xz_i2c_transfer(struct xz_bridge *b, u8 address, const u8 *wbuf, unsigned int wlen, u8 *rbuf, unsigned int rlen);
/* 取り込みブロック。cap_start はリセット解除と設定、cap_run は位置を 0 に戻してライタを開始する (読み出し側が動いてから呼ぶ)。 */
int xz_cap_start(struct xz_bridge *b);
int xz_cap_run(struct xz_bridge *b);
int xz_cap_stop(struct xz_bridge *b);
/* 書き込み位置 (リングの先頭からのバイト数) と、読み出し位置の更新 (192 の倍数)。 */
int xz_cap_wptr(struct xz_bridge *b, u32 *wptr);
int xz_cap_set_rptr(struct xz_bridge *b, u32 rptr);

/* ---- 前面 (xz_frontend.c): 地上波 ---- */
int xz_fe_init(struct xz_bridge *b);
/* 周波数は kHz (ISDB-T の中心周波数。UHF 13ch = 473143)。 */
int xz_fe_tune(struct xz_bridge *b, u32 freq_khz);
int xz_fe_locked(struct xz_bridge *b, bool *locked);
/* 復調 0x8B から 3 バイト (TC90522 系の C/N の生の値)。 */
int xz_fe_cnr_raw(struct xz_bridge *b, u32 *raw);
/* 復調器を休止させる (地上波 0x18 と衛星 0x19)。 */
int xz_fe_sleep(struct xz_bridge *b);

/* ---- 機材内プログラム (xz_program.c) ---- */
/* 本体を 0xE1000 に書いて読み戻し、郵便受けを初期化して、最後に hook を (前半 → 後半の順に) 書く。 */
int xz_prog_install(struct xz_bridge *b);
/* hook を (後半 → 前半の順に) 元へ戻し、次に本体の領域を元に戻す。 */
int xz_prog_remove(struct xz_bridge *b);
/* リングの ring_pos (先頭からのバイト数) から、packets (16 の倍数、128 以下) 個の 512 バイトのパケットを bulk IN へ送らせる。 */
int xz_prog_pump(struct xz_bridge *b, u32 ring_pos, u32 packets);

#ifdef __cplusplus
}
#endif

#endif /* XZ_CORE_H */
