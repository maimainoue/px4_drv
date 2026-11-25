/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * hz2w_core.h - I-O DATA GV-MVP/HZ2W (ASICEN ASV5211 + ASIE5606B) の共通層 (内蔵カードリーダーのみ)
 *
 * OS に依存しない。USB の制御転送と時刻は struct hz2w_ops のコールバックで受けるため、
 * ホストの実装と自動試験 (模擬機材) が同じコードを使う。
 * 要求の形と手順は docs/devices/hz2w.md の実機の確認 (ベンダーのドライバーの通信記録の再現) に従う。
 * 復調器、チューナー、TS の受信は扱わない (TS は 2026-11 時点で未達成)。
 *
 * 手順の由来の一部は Till0196/coax の asv5211 パッケージ (Apache-2.0 OR GPL-2.0-or-later) と、
 * waya0125/recfriio (MIT) の資料による (ガード表と firmware の鍵は docs/devices/hz2w.md を参照)。
 */
#ifndef HZ2W_CORE_H
#define HZ2W_CORE_H

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

#define HZ2W_USB_VENDOR_COLD 0x1738
#define HZ2W_USB_PRODUCT_COLD 0x5211
#define HZ2W_USB_VENDOR_WARM 0x04bb
#define HZ2W_USB_PRODUCT_WARM 0x0526

#define HZ2W_REQ_OUT 0x40 /* vendor、ホストから機器へ */
#define HZ2W_REQ_IN 0xc0 /* vendor、機器からホストへ */

#define HZ2W_FIRMWARE_SIZE 0x4000
/* ローダーが firmware の中で持つ起動アドレス。版で決まる。 (主の版 SHA-256 39428b36…) */
#define HZ2W_BOOT_INDEX_MAIN 0x5583

#define HZ2W_LOG_ERR 0
#define HZ2W_LOG_INFO 1
#define HZ2W_LOG_DEBUG 2

struct hz2w_ops {
	/* 戻り値は転送した長さ、または -errno。buf はスタック上のことがある。 */
	int (*control)(void *priv, u8 request_type, u8 request, u16 value, u16 index, void *buf, u16 len);
	void (*msleep)(void *priv, unsigned int ms);
	/* 単調増加の時刻 (ミリ秒) */
	u64 (*now_ms)(void *priv);
	void (*log)(void *priv, int level, const char *fmt, ...);
};

struct hz2w_bridge {
	const struct hz2w_ops *ops;
	void *priv;
};

#ifdef __cplusplus
extern "C" {
#endif

void hz2w_bridge_init(struct hz2w_bridge *bridge, const struct hz2w_ops *ops, void *priv);

/*
 * cold (1738:5211) へ firmware を送る。送信が成功すると機器は再列挙して warm (04bb:0526) になり、
 * 以降の制御転送は -ENODEV になる。size は HZ2W_FIRMWARE_SIZE でなければならない。
 * 全ての転送が成功したときは 0、最後の転送の直後に機器が切り離されたとき (-ENODEV、-EIO) は
 * HZ2W_FW_REENUMERATING を返す。途中の転送の失敗は、その負のエラーをそのまま返す。
 */
#define HZ2W_FW_REENUMERATING 1
int hz2w_fw_load(struct hz2w_bridge *bridge, const u8 *fw, size_t size, u16 boot_index);

/*
 * warm の機器の起動: 識別情報の確認、起動時の GPIO、1 系統目のカード IC (ASIE5606B、0x4a) のリセットと版の確認、
 * カードの検出の準備まで。成功したら、周期処理 (hz2w_keepalive_*) を直ちに始める必要がある
 * (止まると約 20 秒で機器が USB から消える)。
 */
int hz2w_board_open(struct hz2w_bridge *bridge);
/* ベンダーの閉じる手順。周期処理を止める前に呼ぶ。失敗しても続行する (機器が既に無い場合がある)。 */
void hz2w_board_close(struct hz2w_bridge *bridge);

/* 周期処理: 437.5 ms ごとに hz2w_keepalive_card_state、6.25 s ごとに hz2w_keepalive_watchdog を呼ぶ。 */
#define HZ2W_KEEPALIVE_CARD_PERIOD_US 437500
#define HZ2W_KEEPALIVE_WATCHDOG_PERIOD_US 6250000
int hz2w_keepalive_card_state(struct hz2w_bridge *bridge);
int hz2w_keepalive_watchdog(struct hz2w_bridge *bridge);

/* ---- カード (1 系統目の ASIE5606B。ISO 7816 のバイト列はブロック単位で交換する) ---- */

/*
 * カードのバッファは、レジスタ 0x40-0x7f の 64 バイトを 1 ページとして、レジスタ 0x3a でページを切り替える
 * (0 が先頭、1 が次の 64 バイト。実機で 65 バイトの応答の LRC と、74 バイトの命令で確認した)。
 * 3 ページ目以降は未確認のため、1 ブロックを 2 ページ (128 バイト) までとする。T=1 の IFSC (B-CAS は 124) の
 * ブロック (NAD、PCB、LEN、INF、EDC) がちょうど収まる。
 */
#define HZ2W_CARD_PAGE_SIZE 64
#define HZ2W_CARD_RX_MAX 128

struct hz2w_card {
	struct hz2w_bridge *bridge;
	u8 nonce[16];
	bool nonce_valid;
	bool powered;
	u8 rx[HZ2W_CARD_RX_MAX];
	u8 rx_len;
	u8 rx_pos;
	/* 直後のカード I/O の絶対期限 (0 は未設定。既定の待ちを使う) */
	u64 deadline_ms;
	u8 guard_seed;
};

void hz2w_card_init(struct hz2w_card *card, struct hz2w_bridge *bridge);
/* 受信バッファと電源の状態を捨てる (抜去、再接続) */
void hz2w_card_discard(struct hz2w_card *card);
void hz2w_card_set_timeout(struct hz2w_card *card, unsigned int timeout_ms);
int hz2w_card_detect(struct hz2w_card *card, bool *present);
/* 電源を入れ、ATR を受信バッファへ入れる。読み出しは呼び出し側 (hz2w_card_read)。 */
int hz2w_card_reset(struct hz2w_card *card);
int hz2w_card_power_down(struct hz2w_card *card);
int hz2w_card_rx_ready(struct hz2w_card *card, bool *ready);
/* *length は呼び出し前に容量、成功後に読み取ったバイト数 */
int hz2w_card_read(struct hz2w_card *card, u8 *data, u8 *length);
/* T=1 のブロック (NAD から EDC まで) を 1 つ送り、応答のブロックを受信バッファへ入れる。 */
int hz2w_card_write(struct hz2w_card *card, const u8 *data, u8 length);

#ifdef __cplusplus
}
#endif

#endif /* HZ2W_CORE_H */
