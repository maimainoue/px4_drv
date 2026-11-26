// SPDX-License-Identifier: GPL-2.0-only
/*
 * ut100b_stream.c - QRS-UT100B の TS の暗号 (ASIE5606 が USB 転送でかける DES) の解除と、188 バイトへの区切り。
 *
 * ASIE5606 は、復調器から受けた TS をブリッジ (ASV5211) へ渡すとき、各パケットに DES をかける。
 * 基板の中の IC どうしの転送にかかる暗号で、放送の暗号 (ARIB STD-B25、MULTI2) とは別のもの。
 * 外したあとの TS は、放送のスクランブルがかかったまま。放送のスクランブルを外すのは上位 (B-CAS カードと復号器)。
 *
 * 外し方 (waya0125/recfriio の internal/asie5606/stream_cipher.go と同じ。実機で確認):
 *   1. 4 バイトのヘッダーはそのまま。
 *   2. 残りの 184 バイトに、8 バイト周期の値 whitening を XOR する。
 *   3. 先頭の 128 バイトを DES 鍵 key_head で、残りの 56 バイトを DES 鍵 key_tail で、ECB で復号する。
 * 鍵は、ASIE5606 の 0x10..0x1f が全部 0 のときのもの (起動で 0 を書く)。DES は標準のもの (FIPS 46-3)。
 *
 * 区切りの合わせ方 (0x47 を探し、188、376 バイト先も 0x47 かを見る) は、recfriio の Hdp::getStream による。
 */
#include "ut100b_core.h"

/* ---- DES (FIPS 46-3)。表を引く実装。鍵の展開と表の生成は ut100b_cipher_init() で 1 度だけ行う ---- */

static const u8 des_ip[64] = {
	58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4,
	62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8,
	57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27, 19, 11, 3,
	61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7,
};
static const u8 des_fp[64] = {
	40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31,
	38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29,
	36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,
	34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25,
};
static const u8 des_e[48] = {
	32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9, 8, 9, 10, 11,
	12, 13, 12, 13, 14, 15, 16, 17, 16, 17, 18, 19, 20, 21, 20, 21,
	22, 23, 24, 25, 24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1,
};
static const u8 des_p[32] = {
	16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10,
	2, 8, 24, 14, 32, 27, 3, 9, 19, 13, 30, 6, 22, 11, 4, 25,
};
static const u8 des_pc1[56] = {
	57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18,
	10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60, 52, 44, 36,
	63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22,
	14, 6, 61, 53, 45, 37, 29, 21, 13, 5, 28, 20, 12, 4,
};
static const u8 des_pc2[48] = {
	14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10, 23, 19, 12, 4,
	26, 8, 16, 7, 27, 20, 13, 2, 41, 52, 31, 37, 47, 55, 30, 40,
	51, 45, 33, 48, 44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32,
};
static const u8 des_shifts[16] = { 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1 };
static const u8 des_sbox[8][64] = {
	{ 14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7, 0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8,
	  4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0, 15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13 },
	{ 15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10, 3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5,
	  0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15, 13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9 },
	{ 10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8, 13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1,
	  13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7, 1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12 },
	{ 7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15, 13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9,
	  10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4, 3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14 },
	{ 2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9, 14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6,
	  4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14, 11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3 },
	{ 12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11, 10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8,
	  9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6, 4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13 },
	{ 4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1, 13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6,
	  1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2, 6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12 },
	{ 13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7, 1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2,
	  7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8, 2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11 },
};

/* 4 バイトのヘッダーの後ろに XOR する 8 バイト周期の値と、2 つの DES 鍵 (ASIE5606 の 0x10..0x1f が 0 のとき) */
static const u8 whitening[8] = { 0x00, 0x00, 0xb1, 0xf2, 0x00, 0x04, 0xf2, 0x04 };
static const u8 key_head[8] = { 0xbc, 0x8c, 0x73, 0xe9, 0x0e, 0x68, 0x31, 0x23 };
static const u8 key_tail[8] = { 0xec, 0x15, 0x80, 0x01, 0x80, 0x7f, 0x80, 0x80 };

/* 表 tab (1 始まり、MSB が 1) に従って、in_bits ビットの in から out_bits ビットを取り出して並べる */
static u64 permute(u64 in, const u8 *tab, unsigned int out_bits, unsigned int in_bits)
{
	u64 out = 0;
	unsigned int i;

	for (i = 0; i < out_bits; i++)
		out = (out << 1) | ((in >> (in_bits - tab[i])) & 1);
	return out;
}

static u64 load_be64(const u8 *p)
{
	u64 v = 0;
	unsigned int i;

	for (i = 0; i < 8; i++)
		v = (v << 8) | p[i];
	return v;
}

static void store_be64(u8 *p, u64 v)
{
	unsigned int i;

	for (i = 0; i < 8; i++)
		p[i] = (u8)(v >> (56 - 8 * i));
}

/* 鍵 key (8 バイト) から、復号の順 (K16 → K1) の 16 個の 48 ビットの副鍵を作る */
static void des_decrypt_keys(const u8 *key, u64 *out)
{
	u64 cd = permute(load_be64(key), des_pc1, 56, 64);
	u32 c = (u32)(cd >> 28) & 0x0fffffff;
	u32 d = (u32)cd & 0x0fffffff;
	u64 subkeys[16];
	unsigned int i;

	for (i = 0; i < 16; i++) {
		unsigned int s = des_shifts[i];

		c = ((c << s) | (c >> (28 - s))) & 0x0fffffff;
		d = ((d << s) | (d >> (28 - s))) & 0x0fffffff;
		subkeys[i] = permute(((u64)c << 28) | d, des_pc2, 48, 56);
	}
	for (i = 0; i < 16; i++)
		out[i] = subkeys[15 - i];
}

void ut100b_cipher_init(struct ut100b_cipher *c)
{
	unsigned int i, v, b;

	des_decrypt_keys(key_head, c->k_head);
	des_decrypt_keys(key_tail, c->k_tail);

	/* 初期転置と最終転置: 入力の各バイト (位置 i、値 v) が出力へ寄与するビットの表 */
	for (i = 0; i < 8; i++) {
		for (v = 0; v < 256; v++) {
			u64 in = (u64)v << (56 - 8 * i);

			c->ip[i][v] = permute(in, des_ip, 64, 64);
			c->fp[i][v] = permute(in, des_fp, 64, 64);
		}
	}
	/* 拡張転置 E: R (32 ビット) の各バイトが、48 ビットへ寄与するビットの表 */
	for (i = 0; i < 4; i++) {
		for (v = 0; v < 256; v++) {
			u64 in = (u64)v << (24 - 8 * i);

			c->e[i][v] = permute(in, des_e, 48, 32);
		}
	}
	/* S ボックスと P 転置を合わせた表: 6 ビットの入力 (group i) から、P を通した 32 ビットの出力 */
	for (i = 0; i < 8; i++) {
		for (b = 0; b < 64; b++) {
			unsigned int row = ((b & 0x20) >> 4) | (b & 1);
			unsigned int col = (b >> 1) & 0xf;
			u64 s = (u64)des_sbox[i][row * 16 + col] << (28 - 4 * i);

			c->sp[i][b] = (u32)permute(s, des_p, 32, 32);
		}
	}
}

static u64 des_decrypt_block(const struct ut100b_cipher *c, const u64 *keys, u64 block)
{
	u64 x = 0;
	u32 l, r;
	unsigned int i, round;

	for (i = 0; i < 8; i++)
		x |= c->ip[i][(u8)(block >> (56 - 8 * i))];
	l = (u32)(x >> 32);
	r = (u32)x;
	for (round = 0; round < 16; round++) {
		u64 e = c->e[0][r >> 24] | c->e[1][(r >> 16) & 0xff] | c->e[2][(r >> 8) & 0xff] | c->e[3][r & 0xff];
		u64 t = e ^ keys[round];
		u32 f = 0;
		u32 next;

		for (i = 0; i < 8; i++)
			f |= c->sp[i][(unsigned int)(t >> (42 - 6 * i)) & 0x3f];
		next = l ^ f;
		l = r;
		r = next;
	}
	/* 最終の入れ替え (R16 L16) の後に最終転置 */
	x = ((u64)r << 32) | l;
	block = 0;
	for (i = 0; i < 8; i++)
		block |= c->fp[i][(u8)(x >> (56 - 8 * i))];
	return block;
}

void ut100b_cipher_decrypt_packet(const struct ut100b_cipher *c, u8 *dst, const u8 *src)
{
	u8 block[8];
	unsigned int i, j;

	/* dst と src が同じでもよいよう、先頭の 4 バイトは最後に確定させる必要はない (そのまま) */
	if (dst != src)
		memcpy(dst, src, 4);
	for (i = 4; i + 8 <= UT100B_TS_PACKET_SIZE; i += 8) {
		u64 v;

		for (j = 0; j < 8; j++)
			block[j] = src[i + j] ^ whitening[j];
		v = des_decrypt_block(c, i < 4 + 128 ? c->k_head : c->k_tail, load_be64(block));
		store_be64(&dst[i], v);
	}
}

/* ---- 188 バイトへの区切り ---- */

void ut100b_aligner_init(struct ut100b_aligner *a)
{
	memset(a, 0, sizeof(*a));
	a->stale = UT100B_STALE_DISCARD_BYTES;
	ut100b_cipher_init(&a->cipher);
}

size_t ut100b_aligner_out_capacity(const struct ut100b_aligner *a, size_t n)
{
	return (a->rest_len + n) / UT100B_TS_PACKET_SIZE * UT100B_TS_PACKET_SIZE;
}

/* 前回の残り (rest) と今回の入力 (in) をつないだ列の、位置 i の 1 バイト */
static u8 at(const struct ut100b_aligner *a, const u8 *in, size_t i)
{
	return i < a->rest_len ? a->rest[i] : in[i - a->rest_len];
}

size_t ut100b_aligner_feed(struct ut100b_aligner *a, const u8 *in, size_t n, u8 *out)
{
	size_t size, pos = 0, written = 0, k;
	u8 packet[UT100B_TS_PACKET_SIZE];
	u8 rest[UT100B_TS_PACKET_SIZE];
	size_t rest_len;

	if (a->stale) {
		size_t drop = n < a->stale ? n : a->stale;

		a->stale -= drop;
		in += drop;
		n -= drop;
		if (!n)
			return 0;
	}
	size = a->rest_len + n;
	for (;;) {
		if (pos < size && at(a, in, pos) != 0x47) {
			size_t start = pos;

			while (pos < size) {
				if (at(a, in, pos) == 0x47) {
					if (pos + UT100B_TS_PACKET_SIZE < size && at(a, in, pos + UT100B_TS_PACKET_SIZE) != 0x47) {
						pos++;
						continue;
					}
					if (pos + UT100B_TS_PACKET_SIZE * 2 < size &&
					    at(a, in, pos + UT100B_TS_PACKET_SIZE * 2) != 0x47) {
						pos++;
						continue;
					}
					break;
				}
				pos++;
			}
			if (pos != start) {
				a->resyncs++;
				a->skipped += pos - start;
			}
		}
		if (size - pos < UT100B_TS_PACKET_SIZE)
			break;
		for (k = 0; k < UT100B_TS_PACKET_SIZE; k++)
			packet[k] = at(a, in, pos + k);
		ut100b_cipher_decrypt_packet(&a->cipher, out + written, packet);
		written += UT100B_TS_PACKET_SIZE;
		pos += UT100B_TS_PACKET_SIZE;
	}
	rest_len = size - pos;
	for (k = 0; k < rest_len; k++)
		rest[k] = at(a, in, pos + k);
	memcpy(a->rest, rest, rest_len);
	a->rest_len = rest_len;
	return written;
}
