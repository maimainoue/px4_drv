// SPDX-License-Identifier: GPL-2.0-only
/*
 * GV-MVP/XZ2: ベンダー要求 0xBC によるメモリとレジスタの読み書き。
 */
#include "xz_core.h"

#define LINK_TIMEOUT_MS 1000U
#define LINK_READ_CHUNK 64U	/* 読みは 1 回 64 バイトまで */
#define LINK_WRITE_CHUNK 4U	/* 書きは 1 回 4 バイトまで。64 バイトの書き込みは XZ2 で失敗した */
#define LINK_ADDRESS_LIMIT 0x1000000u	/* 24 ビット */

void xz_bridge_init(struct xz_bridge *b, const struct xz_ops *ops, void *priv)
{
	memset(b, 0, sizeof(*b));
	b->ops = ops;
	b->priv = priv;
}

/* 24 ビットのアドレスの上位 8 ビットは wValue の上位バイト、下位 16 ビットは wIndex。 */
static int link_transfer(struct xz_bridge *b, u8 request_type, u32 at, u8 *buf, unsigned int chunk)
{
	int ret = b->ops->control(b->priv, request_type, (u16)((at >> 8) & 0xff00), (u16)at, buf, (u16)chunk, LINK_TIMEOUT_MS);

	if (ret < 0)
		return ret;
	return (unsigned int)ret == chunk ? 0 : -EIO;
}

int xz_read(struct xz_bridge *b, u32 address, void *buf, unsigned int len)
{
	unsigned int offset;

	if (!b || !buf || !len || address >= LINK_ADDRESS_LIMIT || len > LINK_ADDRESS_LIMIT - address)
		return -EINVAL;
	for (offset = 0; offset < len; offset += LINK_READ_CHUNK) {
		unsigned int chunk = len - offset < LINK_READ_CHUNK ? len - offset : LINK_READ_CHUNK;
		int ret = link_transfer(b, 0xc0, address + offset, (u8 *)buf + offset, chunk);

		if (ret)
			return ret;
	}
	return 0;
}

int xz_write(struct xz_bridge *b, u32 address, const void *buf, unsigned int len)
{
	unsigned int offset;

	if (!b || !buf || !len || address >= LINK_ADDRESS_LIMIT || len > LINK_ADDRESS_LIMIT - address)
		return -EINVAL;
	for (offset = 0; offset < len; offset += LINK_WRITE_CHUNK) {
		unsigned int chunk = len - offset < LINK_WRITE_CHUNK ? len - offset : LINK_WRITE_CHUNK;
		u8 tmp[LINK_WRITE_CHUNK];
		int ret;

		/*
		 * 書き込みのデータは、呼び出し側のバッファ (機材内プログラムの機械語のような読み取り専用のデータを含む) ではなく、
		 * 書き込み可能な一時の領域から転送する。WinUSB は、書き込み (OUT) の制御転送でも、書き込み可能なメモリを要求し、
		 * 読み取り専用のデータを渡すとエラー 998 (ERROR_NOACCESS) で失敗する (GV-MVP/XZ2 の Windows で確認)。
		 */
		memcpy(tmp, (const u8 *)buf + offset, chunk);
		ret = link_transfer(b, 0x40, address + offset, tmp, chunk);

		if (ret)
			return ret;
	}
	return 0;
}

int xz_read32(struct xz_bridge *b, u32 address, u32 *value)
{
	u8 data[4];
	int ret = xz_read(b, address, data, sizeof(data));

	if (ret)
		return ret;
	*value = (u32)data[0] << 24 | (u32)data[1] << 16 | (u32)data[2] << 8 | data[3];
	return 0;
}

int xz_write32(struct xz_bridge *b, u32 address, u32 value)
{
	u8 data[4] = { (u8)(value >> 24), (u8)(value >> 16), (u8)(value >> 8), (u8)value };

	return xz_write(b, address, data, sizeof(data));
}
