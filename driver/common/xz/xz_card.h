/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef XZ_CARD_H
#define XZ_CARD_H

/*
 * GV-MVP/XZ2 のカード UART 層。
 *
 * UART のレジスター配置 (0xfea000-0xfea03f) は DT295 (MB86H47A 系) と同じなので、ポートの型、FIFO の読み取り、送信、
 * 速度設定は torne_card.h のものをそのまま使う。XZ2 で異なるのは次の 3 点だけで、この層が持つ。
 *  - カード検出: 0xfea01c の bit10 が立っていれば挿入あり (DT295 のように端子の語 0xf00224 とは XOR しない)
 *  - 電源投入: 端子の語 (0xf00220 台) を読まず、XZ2 の値を定数で持つ (途中の線の値 third = 0、最後は 6)。
 *    DT295 の手順は、端子の極性の語が 0 のとき third = 8 になり、XZ2 の実機の結果と合わない。
 *  - 電源切断: DT295 と同じ 5 段
 * ATR と T=1 のフレームの組み立てと完了判定は、この層にも px4_card (カーネル) にも無く、フレーム単位で read / write する
 * ユーザー空間の層が担当する (XZ2 の実機の試験では、受信が途切れるまで待つ方式より、3 バイトのヘッダの長さで判定する方式の方が、
 * 受信中の取りこぼしに強く、往復も約 90 ms から約 30 ms に短くなった。設計資料の §6.6 を参照)。
 */
#include "../torne/torne_card.h"

#ifdef __cplusplus
extern "C" {
#endif

int xz_card_detect(struct torne_card_port *port, bool *present);
/* 電源を入れ、UART を 9600 bps の ATR 待ちにする。ATR の読み取りは呼び出し側が行う。 */
int xz_card_reset(struct torne_card_port *port);
int xz_card_power_down(struct torne_card_port *port);

#ifdef __cplusplus
}
#endif

#endif /* XZ_CARD_H */
