// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_crypto.c - SHA-1、AES-128、AES-CMAC (dth11_crypto.h)
 */

#include "dth11_crypto.h"

/* ---- SHA-1 (FIPS 180-1) ---- */

static u32 rol32(u32 v, unsigned int n)
{
	return (v << n) | (v >> (32 - n));
}

static void sha1_block(struct dth11_sha1 *c, const u8 *p)
{
	u32 w[80], a, b, d, e, cc, t;
	int i;

	for (i = 0; i < 16; i++)
		w[i] = ((u32)p[4 * i] << 24) | ((u32)p[4 * i + 1] << 16) | ((u32)p[4 * i + 2] << 8) | p[4 * i + 3];
	for (i = 16; i < 80; i++)
		w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

	a = c->h[0];
	b = c->h[1];
	cc = c->h[2];
	d = c->h[3];
	e = c->h[4];
	for (i = 0; i < 80; i++) {
		u32 f, k;

		if (i < 20) {
			f = (b & cc) | (~b & d);
			k = 0x5a827999;
		} else if (i < 40) {
			f = b ^ cc ^ d;
			k = 0x6ed9eba1;
		} else if (i < 60) {
			f = (b & cc) | (b & d) | (cc & d);
			k = 0x8f1bbcdc;
		} else {
			f = b ^ cc ^ d;
			k = 0xca62c1d6;
		}
		t = rol32(a, 5) + f + e + k + w[i];
		e = d;
		d = cc;
		cc = rol32(b, 30);
		b = a;
		a = t;
	}
	c->h[0] += a;
	c->h[1] += b;
	c->h[2] += cc;
	c->h[3] += d;
	c->h[4] += e;
}

void dth11_sha1_init(struct dth11_sha1 *c)
{
	c->h[0] = 0x67452301;
	c->h[1] = 0xefcdab89;
	c->h[2] = 0x98badcfe;
	c->h[3] = 0x10325476;
	c->h[4] = 0xc3d2e1f0;
	c->total = 0;
	c->buf_len = 0;
}

void dth11_sha1_update(struct dth11_sha1 *c, const void *data, size_t len)
{
	const u8 *p = data;

	c->total += len;
	while (len) {
		size_t n = 64 - c->buf_len;

		if (n > len)
			n = len;
		memcpy(c->buf + c->buf_len, p, n);
		c->buf_len += (u32)n;
		p += n;
		len -= n;
		if (c->buf_len == 64) {
			sha1_block(c, c->buf);
			c->buf_len = 0;
		}
	}
}

void dth11_sha1_final(struct dth11_sha1 *c, u8 out[DTH11_SHA1_SIZE])
{
	u64 bits = c->total * 8;
	u8 pad = 0x80;
	u8 zero = 0;
	u8 len[8];
	int i;

	dth11_sha1_update(c, &pad, 1);
	while (c->buf_len != 56)
		dth11_sha1_update(c, &zero, 1);
	for (i = 0; i < 8; i++)
		len[i] = (u8)(bits >> (56 - 8 * i));
	dth11_sha1_update(c, len, 8);
	for (i = 0; i < 5; i++) {
		out[4 * i] = (u8)(c->h[i] >> 24);
		out[4 * i + 1] = (u8)(c->h[i] >> 16);
		out[4 * i + 2] = (u8)(c->h[i] >> 8);
		out[4 * i + 3] = (u8)c->h[i];
	}
}

void dth11_sha1(const void *data, size_t len, u8 out[DTH11_SHA1_SIZE])
{
	struct dth11_sha1 c;

	dth11_sha1_init(&c);
	dth11_sha1_update(&c, data, len);
	dth11_sha1_final(&c, out);
}

/* ---- AES-128 (FIPS 197) ---- */

static u8 sbox[256];
static u8 inv_sbox[256];
static int tables_ready;

static u8 xtime(u8 x)
{
	return (u8)((x << 1) ^ ((x & 0x80) ? 0x1b : 0));
}

static u8 gmul(u8 a, u8 b)
{
	u8 r = 0;

	while (b) {
		if (b & 1)
			r ^= a;
		a = xtime(a);
		b >>= 1;
	}
	return r;
}

/*
 * S-box は GF(2^8) の乗法逆元とアフィン変換から作る。同じ値を書き込むだけなので、複数のスレッドが
 * 同時に最初の呼び出しをしても結果は変わらない (tables_ready は全ての表を書き終えた後に立てる)。
 */
static void make_tables(void)
{
	u8 p = 1, q = 1;
	int i;

	if (tables_ready)
		return;
	do {
		u8 x;

		/* p を 3 倍、q を 1/3 倍 (乗法逆元) していく */
		p = (u8)(p ^ (p << 1) ^ ((p & 0x80) ? 0x1b : 0));
		q ^= (u8)(q << 1);
		q ^= (u8)(q << 2);
		q ^= (u8)(q << 4);
		if (q & 0x80)
			q ^= 0x09;
		x = (u8)(q ^ ((q << 1) | (q >> 7)) ^ ((q << 2) | (q >> 6)) ^ ((q << 3) | (q >> 5)) ^
			 ((q << 4) | (q >> 4)));
		sbox[p] = (u8)(x ^ 0x63);
	} while (p != 1);
	sbox[0] = 0x63;
	for (i = 0; i < 256; i++)
		inv_sbox[sbox[i]] = (u8)i;
	tables_ready = 1;
}

void dth11_aes_set_key(struct dth11_aes *a, const u8 key[16])
{
	u8 rcon = 1;
	int i;

	make_tables();
	memcpy(a->rk, key, 16);
	for (i = 16; i < 176; i += 4) {
		u8 t[4];

		memcpy(t, a->rk + i - 4, 4);
		if (i % 16 == 0) {
			u8 tmp = t[0];

			t[0] = (u8)(sbox[t[1]] ^ rcon);
			t[1] = sbox[t[2]];
			t[2] = sbox[t[3]];
			t[3] = sbox[tmp];
			rcon = xtime(rcon);
		}
		a->rk[i] = (u8)(a->rk[i - 16] ^ t[0]);
		a->rk[i + 1] = (u8)(a->rk[i - 15] ^ t[1]);
		a->rk[i + 2] = (u8)(a->rk[i - 14] ^ t[2]);
		a->rk[i + 3] = (u8)(a->rk[i - 13] ^ t[3]);
	}
}

static void add_round_key(u8 s[16], const u8 *rk)
{
	int i;

	for (i = 0; i < 16; i++)
		s[i] ^= rk[i];
}

void dth11_aes_encrypt_block(const struct dth11_aes *a, const u8 in[16], u8 out[16])
{
	u8 s[16], t[16];
	int round, i;

	memcpy(s, in, 16);
	add_round_key(s, a->rk);
	for (round = 1; round <= 10; round++) {
		/* SubBytes と ShiftRows (列優先の状態: s[4 * col + row]) */
		for (i = 0; i < 16; i++) {
			int col = i / 4, row = i % 4;

			t[i] = sbox[s[4 * ((col + row) % 4) + row]];
		}
		if (round != 10) {
			for (i = 0; i < 4; i++) {
				u8 *c = t + 4 * i;
				u8 c0 = c[0], c1 = c[1], c2 = c[2], c3 = c[3];

				c[0] = (u8)(xtime(c0) ^ (xtime(c1) ^ c1) ^ c2 ^ c3);
				c[1] = (u8)(c0 ^ xtime(c1) ^ (xtime(c2) ^ c2) ^ c3);
				c[2] = (u8)(c0 ^ c1 ^ xtime(c2) ^ (xtime(c3) ^ c3));
				c[3] = (u8)((xtime(c0) ^ c0) ^ c1 ^ c2 ^ xtime(c3));
			}
		}
		memcpy(s, t, 16);
		add_round_key(s, a->rk + 16 * round);
	}
	memcpy(out, s, 16);
}

void dth11_aes_decrypt_block(const struct dth11_aes *a, const u8 in[16], u8 out[16])
{
	u8 s[16], t[16];
	int round, i;

	memcpy(s, in, 16);
	add_round_key(s, a->rk + 160);
	for (round = 9; round >= 0; round--) {
		/* InvShiftRows と InvSubBytes */
		for (i = 0; i < 16; i++) {
			int col = i / 4, row = i % 4;

			t[4 * ((col + row) % 4) + row] = inv_sbox[s[i]];
		}
		add_round_key(t, a->rk + 16 * round);
		if (round != 0) {
			for (i = 0; i < 4; i++) {
				u8 *c = t + 4 * i;
				u8 c0 = c[0], c1 = c[1], c2 = c[2], c3 = c[3];

				c[0] = (u8)(gmul(c0, 14) ^ gmul(c1, 11) ^ gmul(c2, 13) ^ gmul(c3, 9));
				c[1] = (u8)(gmul(c0, 9) ^ gmul(c1, 14) ^ gmul(c2, 11) ^ gmul(c3, 13));
				c[2] = (u8)(gmul(c0, 13) ^ gmul(c1, 9) ^ gmul(c2, 14) ^ gmul(c3, 11));
				c[3] = (u8)(gmul(c0, 11) ^ gmul(c1, 13) ^ gmul(c2, 9) ^ gmul(c3, 14));
			}
		}
		memcpy(s, t, 16);
	}
	memcpy(out, s, 16);
}

int dth11_aes_cbc_decrypt(const u8 key[16], const u8 iv[16], u8 *data, size_t len)
{
	struct dth11_aes a;
	u8 prev[16], cur[16], plain[16];
	size_t off;
	int i;

	if (len % 16)
		return -EINVAL;
	dth11_aes_set_key(&a, key);
	memcpy(prev, iv, 16);
	for (off = 0; off < len; off += 16) {
		memcpy(cur, data + off, 16);
		dth11_aes_decrypt_block(&a, cur, plain);
		for (i = 0; i < 16; i++)
			data[off + i] = (u8)(plain[i] ^ prev[i]);
		memcpy(prev, cur, 16);
	}
	return 0;
}

/* ---- AES-CTR ---- */

void dth11_ctr_init(struct dth11_ctr *c, const u8 key[16], const u8 iv[16])
{
	dth11_aes_set_key(&c->aes, key);
	memcpy(c->counter, iv, 16);
	/* 最初のブロックの鍵ストリームは iv そのものの暗号化 (カウンターは使った後で進める) */
	dth11_aes_encrypt_block(&c->aes, iv, c->ks);
	c->pos = 0;
}

static void ctr_increment(u8 counter[16])
{
	int i;

	for (i = 15; i >= 0; i--) {
		counter[i]++;
		if (counter[i])
			break;
	}
}

void dth11_ctr_apply(struct dth11_ctr *c, u8 *buf, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (c->pos == 16) {
			ctr_increment(c->counter);
			dth11_aes_encrypt_block(&c->aes, c->counter, c->ks);
			c->pos = 0;
		}
		buf[i] ^= c->ks[c->pos++];
	}
}

/* ---- AES-CMAC (RFC 4493) ---- */

static void cmac_shift_xor(const u8 in[16], u8 out[16])
{
	int i;
	u8 carry = 0;

	for (i = 15; i >= 0; i--) {
		u8 next = (u8)(in[i] >> 7);

		out[i] = (u8)((in[i] << 1) | carry);
		carry = next;
	}
	if (in[0] & 0x80)
		out[15] ^= 0x87;
}

void dth11_aes_cmac(const u8 key[16], const void *data, size_t len, u8 out[16])
{
	struct dth11_aes a;
	u8 k1[16], k2[16], l[16], x[16] = { 0 }, y[16];
	const u8 *p = data;
	size_t blocks = (len + 15) / 16;
	bool complete = len && len % 16 == 0;
	size_t i;
	int j;

	dth11_aes_set_key(&a, key);
	memset(l, 0, 16);
	dth11_aes_encrypt_block(&a, l, l);
	cmac_shift_xor(l, k1);
	cmac_shift_xor(k1, k2);

	if (!blocks)
		blocks = 1;
	for (i = 0; i + 1 < blocks; i++) {
		for (j = 0; j < 16; j++)
			y[j] = (u8)(x[j] ^ p[16 * i + j]);
		dth11_aes_encrypt_block(&a, y, x);
	}
	/* 最後のブロック: 16 バイトちょうどなら K1 を、足りなければ 0x80 0x00… で埋めて K2 を混ぜる */
	{
		u8 last[16] = { 0 };
		size_t rem = len - 16 * (blocks - 1);

		if (rem)
			memcpy(last, p + 16 * (blocks - 1), rem);
		if (complete) {
			for (j = 0; j < 16; j++)
				last[j] ^= k1[j];
		} else {
			last[rem] = 0x80;
			for (j = 0; j < 16; j++)
				last[j] ^= k2[j];
		}
		for (j = 0; j < 16; j++)
			y[j] = (u8)(x[j] ^ last[j]);
		dth11_aes_encrypt_block(&a, y, out);
	}
}
