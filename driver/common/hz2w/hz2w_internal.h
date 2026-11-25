/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HZ2W_INTERNAL_H
#define HZ2W_INTERNAL_H

#include "hz2w_core.h"

/* 1 系統目のカードインターフェース IC (ASIE5606B) の I2C アドレス */
#define HZ2W_ASIE_ADDR 0x4a

int hz2w_ctl_in(struct hz2w_bridge *b, u8 request, u16 value, u16 index, void *buf, u16 len);
int hz2w_gpio(struct hz2w_bridge *b, bool extended, u8 value, u8 mask, u8 *readback);
int hz2w_reg_write(struct hz2w_bridge *b, u8 addr, u8 reg, u8 value);
/* 1 系統目の ASIE5606B のリセット (GPIO のパルス) と、要求 0x23、0x21 の再投入。レジスタが初期値に戻る。 */
int hz2w_card_ic_reset(struct hz2w_bridge *b);
int hz2w_reg_read(struct hz2w_bridge *b, u8 addr, u8 reg, u8 *value);

#endif
