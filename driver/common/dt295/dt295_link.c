// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295: ベンダー要求 0xBC によるメモリとレジスタの読み書き、boot0 の窓。
 */
#include "dt295_core.h"

#define LINK_TIMEOUT_MS 1000U
#define LINK_CHUNK 64U	/* 1 回の要求は 64 バイトまで。超えると機材が固まり、物理的な抜き差しが必要になる。 */

/* boot0 のアドレス変換の即値 (4 バイト、ビッグエンディアン)。どちらも書き換えないと、読み出しの経路に効かない。 */
#define WINDOW_IMM_A 0xea3d8u
#define WINDOW_IMM_B 0xe9ebcu
#define WINDOW_DEFAULT 0x00f00000u
/* 窓の先頭 (0x80000) と大きさ (128 KiB)。窓越しのアドレスは 0x80000 + (アドレス - 窓の基準)。 */
#define WINDOW_BASE 0x80000u

static void put_be32(u8 *p, u32 v)
{
	p[0] = (u8)(v >> 24);
	p[1] = (u8)(v >> 16);
	p[2] = (u8)(v >> 8);
	p[3] = (u8)v;
}

static u32 get_be32(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

void dt295_bridge_init(struct dt295_bridge *b, const struct dt295_ops *ops, void *priv)
{
	memset(b, 0, sizeof(*b));
	b->ops = ops;
	b->priv = priv;
}

int dt295_read(struct dt295_bridge *b, u32 address, void *buf, unsigned int len)
{
	unsigned int offset;

	if (!b || !buf || !len || address >= 0x1000000u || len > 0x1000000u - address)
		return -EINVAL;
	for (offset = 0; offset < len; offset += LINK_CHUNK) {
		unsigned int chunk = len - offset < LINK_CHUNK ? len - offset : LINK_CHUNK;
		u32 at = address + offset;
		int ret = b->ops->control(b->priv, 0xc0, (u16)((at >> 8) & 0xff00), (u16)at, (u8 *)buf + offset, (u16)chunk,
					  LINK_TIMEOUT_MS);

		if (ret < 0)
			return ret;
		if ((unsigned int)ret != chunk)
			return -EIO;
	}
	return 0;
}

int dt295_write(struct dt295_bridge *b, u32 address, const void *buf, unsigned int len)
{
	unsigned int offset;

	if (!b || !buf || !len || address >= 0x1000000u || len > 0x1000000u - address)
		return -EINVAL;
	for (offset = 0; offset < len; offset += LINK_CHUNK) {
		unsigned int chunk = len - offset < LINK_CHUNK ? len - offset : LINK_CHUNK;
		u32 at = address + offset;
		u8 tmp[LINK_CHUNK];
		int ret;

		/*
		 * 書き込みのデータは、呼び出し側のバッファ (機材内プログラムの機械語のような読み取り専用のデータを含む) ではなく、
		 * 書き込み可能な一時の領域から転送する。WinUSB は、書き込み (OUT) の制御転送でも、書き込み可能なメモリを要求し、
		 * 読み取り専用のデータを渡すとエラー 998 (ERROR_NOACCESS) で失敗する (同じ構成の別機種の Windows で確認)。
		 */
		memcpy(tmp, (const u8 *)buf + offset, chunk);
		ret = b->ops->control(b->priv, 0x40, (u16)((at >> 8) & 0xff00), (u16)at, tmp, (u16)chunk, LINK_TIMEOUT_MS);

		if (ret < 0)
			return ret;
		if ((unsigned int)ret != chunk)
			return -EIO;
	}
	return 0;
}

int dt295_mwrite32(struct dt295_bridge *b, u32 address, u32 value, u32 mask)
{
	u8 data[8];
	int ret;

	if (!b || (address & 3u) || address >= 0x1000000u)
		return -EINVAL;
	put_be32(data, value);
	put_be32(data + 4, mask);
	/* wValue の下位バイトが 1 のとき、boot0 は *address = (*address & ~mask) | (value & mask) を行う。 */
	ret = b->ops->control(b->priv, 0x40, (u16)(((address >> 8) & 0xff00) | 1u), (u16)address, data, sizeof(data),
			      LINK_TIMEOUT_MS);
	if (ret < 0)
		return ret;
	return ret == (int)sizeof(data) ? 0 : -EIO;
}

int dt295_read32(struct dt295_bridge *b, u32 address, u32 *value)
{
	u8 data[4];
	int ret = dt295_read(b, address, data, sizeof(data));

	if (ret)
		return ret;
	*value = get_be32(data);
	return 0;
}

int dt295_write16_hi(struct dt295_bridge *b, u32 address, u16 value)
{
	return dt295_mwrite32(b, address, (u32)value << 16, 0xffff0000u);
}

int dt295_window_set(struct dt295_bridge *b, u32 base)
{
	u8 data[4], back[4];
	int ret;

	put_be32(data, base);
	ret = dt295_write(b, WINDOW_IMM_A, data, sizeof(data));
	if (!ret)
		ret = dt295_write(b, WINDOW_IMM_B, data, sizeof(data));
	if (!ret)
		ret = dt295_read(b, WINDOW_IMM_A, back, sizeof(back));
	if (!ret && memcmp(data, back, sizeof(data)))
		ret = -EIO;
	if (!ret)
		ret = dt295_read(b, WINDOW_IMM_B, back, sizeof(back));
	if (!ret && memcmp(data, back, sizeof(data)))
		ret = -EIO;
	if (!ret)
		b->window_set = base != WINDOW_DEFAULT;
	return ret;
}

int dt295_window_restore(struct dt295_bridge *b)
{
	int ret = dt295_window_set(b, WINDOW_DEFAULT);

	if (!ret)
		b->window_set = false;
	return ret;
}

/* 取り込みブロックは 0x1900000 〜。窓が 0x1900000 を指すとき、0x80000 + (reg - 0x1900000) で届く。 */
static u32 blk_address(u32 reg)
{
	return WINDOW_BASE + (reg - 0x1900000u);
}

int dt295_blk_read(struct dt295_bridge *b, u32 reg, u32 *value)
{
	if (!b->window_set || reg < 0x1900000u || reg >= 0x1900000u + 0x20000u)
		return -EINVAL;
	return dt295_read32(b, blk_address(reg), value);
}

int dt295_blk_write(struct dt295_bridge *b, u32 reg, u32 value)
{
	return dt295_blk_mask(b, reg, value, 0xffffffffu);
}

int dt295_blk_mask(struct dt295_bridge *b, u32 reg, u32 value, u32 mask)
{
	if (!b->window_set || reg < 0x1900000u || reg >= 0x1900000u + 0x20000u || (reg & 3u))
		return -EINVAL;
	return dt295_mwrite32(b, blk_address(reg), value, mask);
}
