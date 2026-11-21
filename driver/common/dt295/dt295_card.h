/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef DT295_CARD_H
#define DT295_CARD_H

/*
 * PIX-DT295 のカード UART 層 (試験実装)。
 *
 * UART のレジスター配置 (0xfea000-0xfea03f) は CECH-ZD1 J の MB86H47A と同じなので、
 * ポートの型、FIFO の読み取り、送信、速度設定は torne_card.h のものをそのまま使う。
 * DT295 で異なるのは次の 3 点だけで、この層が持つ。
 *  - カード検出 (0xfea01c の bit10 と 0xf00224 の XOR)
 *  - 電源投入の手順 (端子の極性 0xf00223 で一部の値が変わる)
 *  - 電源切断の手順
 * ATR と T=1 は全機種共通の host/core/smart_card.cpp が担当する。
 */
#include "../torne/torne_card.h"

#ifdef __cplusplus
extern "C" {
#endif

int dt295_card_detect(struct torne_card_port *port, bool *present);
/* 電源を入れ、UART を 9600 bps の ATR 待ちにする。ATR の読み取りは呼び出し側が行う。 */
int dt295_card_reset(struct torne_card_port *port);
int dt295_card_power_down(struct torne_card_port *port);

#ifdef __cplusplus
}
#endif

#endif /* DT295_CARD_H */
