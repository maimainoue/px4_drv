/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dt295_core.h - PIX-DT295 (Fujitsu MB86M01 + TC90522/TC90532 + MxL603 系) の共通層
 *
 * OS 非依存。USB の制御転送と時間は struct dt295_ops の関数で受け取るので、同じコードを、ユーザー空間の試験
 * (libusb) とカーネルモジュールの両方で使える。
 *
 * 手順は実機 (Rev.01、Rev.02) で確認したもの。確認の記録と値の出どころは dt295_init_crosscheck.md と
 * dt295_e3_results.md を参照。機材の仕組み:
 *  - ホストはベンダー要求 0xBC で、機材のメモリとレジスタを、24 ビットのアドレスで直接読み書きする。
 *    1 回の要求は 64 バイトまで (超えると機材が固まる)。
 *  - boot0 のアドレス変換 (0x80000〜0x9FFFF -> +即値、即値の既定は 0xF00000) の即値を RAM 上で書き換えて、
 *    24 ビットを超える領域 (取り込みブロック 0x1900000 など) を、標準の読み書きで扱う。
 *  - 機材内の自作プログラム (boot0 の hook から動く) は、SDRAM のリングから bulk IN へ TS を送るだけにする。
 *    取り込みブロックと 0xF10118 へは、機材内のプログラムから書くと機材が止まるので、ホストの標準の経路で書く。
 *  - 失敗した I2C の試行は、バスを数秒〜十数秒詰まらせる。失敗の直後に再試行しない。
 */
#ifndef DT295_CORE_H
#define DT295_CORE_H

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

#define DT295_USB_VENDOR 0x06b8
#define DT295_USB_PRODUCT_REV01 0x103b
#define DT295_USB_PRODUCT_REV02 0x105c

#define DT295_TS_PACKET_SIZE 188
#define DT295_UNIT_SIZE 192	/* 4 バイトのタイムスタンプ + 188 バイトの TS */

/* SDRAM のリング (CPU のアドレス)。取り込みブロックの書き込み先は物理アドレス 0x0138EC00。 */
#define DT295_RING_BASE 0x8138ec00u
#define DT295_RING_SIZE 0x30000u
/* 機材内プログラムが 1 回の要求で送る最大のパケット数。1 パケットは 512 バイト (bulk IN の 1 パケット)。 */
#define DT295_PACKET_BYTES 512u
#define DT295_PUMP_MAX_PACKETS 128u
/* 受信側の非同期転送 (8 KiB) で割り切れるように、ポンプのパケット数は 16 の倍数にする。 */
#define DT295_PUMP_UNIT_PACKETS 16u

#define DT295_LOG_ERR 0
#define DT295_LOG_INFO 1
#define DT295_LOG_DEBUG 2

/* ホストから見た機材: USB の制御転送、排他、時間、ログ。 */
struct dt295_ops {
	/* ベンダー要求 0xBC の制御転送 (request_type は 0xC0 または 0x40)。長さ 64 バイト以下。長さ、または -errno を返す。 */
	int (*control)(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms);
	/*
	 * ポンプの要求と応答は、同じ 64 バイトの場所 (0xC4620) を通る。組の間に別の制御転送が入ると壊れるので、
	 * lock で他の制御転送を止め、control_locked (呼び出し側が lock を保持している) で組を実行して、unlock する。
	 */
	int (*lock)(void *priv);
	void (*unlock)(void *priv);
	int (*control_locked)(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms);
	void (*msleep)(void *priv, unsigned int ms);
	u64 (*now_ms)(void *priv); /* 単調増加のミリ秒 */
	void (*log)(void *priv, int level, const char *fmt, ...);
};

struct dt295_bridge {
	const struct dt295_ops *ops;
	void *priv;
	bool window_set;	/* 窓を 0x1900000 に向けている */
	bool program_installed;	/* hook と本体を導入している */
	u8 saved_body[512];	/* 導入前の本体の領域 (0xE1000〜) */
};

#ifdef __cplusplus
extern "C" {
#endif

void dt295_bridge_init(struct dt295_bridge *b, const struct dt295_ops *ops, void *priv);

/* ---- リンク (dt295_link.c): 24 ビットのアドレス、1 回の要求は 64 バイトまで (長い要求は 64 バイトずつに分ける) ---- */
int dt295_read(struct dt295_bridge *b, u32 address, void *buf, unsigned int len);
int dt295_write(struct dt295_bridge *b, u32 address, const void *buf, unsigned int len);
/* *address = (*address & ~mask) | (value & mask)。address は 4 の倍数。boot0 自身のマスク付き書き込み。 */
int dt295_mwrite32(struct dt295_bridge *b, u32 address, u32 value, u32 mask);
int dt295_read32(struct dt295_bridge *b, u32 address, u32 *value);
/* 16 ビットのレジスタ (4 の倍数のアドレスの上位半語) への書き込み。 */
int dt295_write16_hi(struct dt295_bridge *b, u32 address, u16 value);

/* 窓 (boot0 のアドレス変換の即値)。base は 0x01900000 など。dt295_window_restore で 0x00F00000 に戻す。 */
int dt295_window_set(struct dt295_bridge *b, u32 base);
int dt295_window_restore(struct dt295_bridge *b);
/* 取り込みブロックのレジスタ (0x1900000 台)。窓が 0x1900000 に向いていること。 */
int dt295_blk_read(struct dt295_bridge *b, u32 reg, u32 *value);
int dt295_blk_write(struct dt295_bridge *b, u32 reg, u32 value);
int dt295_blk_mask(struct dt295_bridge *b, u32 reg, u32 value, u32 mask);

/* ---- ボード (dt295_board.c) ---- */
/* 起動直後の確認 (状態 0x82008 が 0、hook が元の値) と、チップ側の設定、I2C コントローラの準備。起動直後でなければ -EBUSY。 */
int dt295_board_open(struct dt295_bridge *b);
/* I2C (7 ビットアドレス、書き長と読み長の合計 40 バイト以下)。-ETIMEDOUT / -ENXIO (ACK なし) / -EIO。 */
int dt295_i2c_transfer(struct dt295_bridge *b, u8 address, const u8 *wbuf, unsigned int wlen, u8 *rbuf, unsigned int rlen);
/* 取り込みブロック (窓越し)。cap_start は、窓を 0x1900000 に向けて、リセット解除、入力、パケット、タイムスタンプ、リングを書く (ライタは dt295_cap_run で開始)。 */
int dt295_cap_start(struct dt295_bridge *b);
int dt295_cap_stop(struct dt295_bridge *b);
/* ライタを開始する。読み出し側 (ポンプ) が動いてから呼ぶ (開始が早いと、読む前にリングが 1 周して上書きされる)。 */
int dt295_cap_run(struct dt295_bridge *b);
/* 書き込み位置 (リングの先頭からのバイト数。192 の倍数) と、読み出し位置の更新。 */
int dt295_cap_wptr(struct dt295_bridge *b, u32 *wptr);
int dt295_cap_set_rptr(struct dt295_bridge *b, u32 rptr);

/* ---- 前面 (dt295_frontend.c): 地上波 ---- */
int dt295_fe_init(struct dt295_bridge *b);
/* 周波数は kHz (ISDB-T の中心周波数。UHF 13ch = 473143)。 */
int dt295_fe_tune(struct dt295_bridge *b, u32 freq_khz);
int dt295_fe_locked(struct dt295_bridge *b, bool *locked);
/* 復調 0x8B から 3 バイト (TC90522 系の C/N の生の値)。 */
int dt295_fe_cnr_raw(struct dt295_bridge *b, u32 *raw);

/* ---- 機材内プログラム (dt295_program.c) ---- */
/* 本体を 0xE1000 に書いて読み戻し、最後に hook を書く。導入前の本体の領域は b->saved_body に保存する。 */
int dt295_prog_install(struct dt295_bridge *b);
/* hook を元に戻し、次に本体の領域を元に戻す。 */
int dt295_prog_remove(struct dt295_bridge *b);
/* リングの ring_pos (リングの先頭からのバイト数) から、packets (16 の倍数、128 以下) 個の 512 バイトのパケットを bulk IN へ送らせる。 */
int dt295_prog_pump(struct dt295_bridge *b, u32 ring_pos, u32 packets);

#ifdef __cplusplus
}
#endif

#endif /* DT295_CORE_H */
