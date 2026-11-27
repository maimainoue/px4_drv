/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_crypto.h - DT-H11/U2 のリンク暗号 (CEPM) が使う暗号部品
 *
 * SHA-1、AES-128 (ECB の 1 ブロック、CBC 復号、CTR)、AES-CMAC。OS に依存しない。
 * 性能より、短さと取り違えにくさを優先した素直な実装で、鍵の導出と TS の復号 (1 パケットあたり
 * 184 バイト) に使う。サイドチャネル耐性は扱わない (鍵は機器との通信と同じ処理系の中にしか現れない)。
 * 試験は tests/dth11_crypto_test.c (FIPS 180-1、FIPS 197、RFC 4493、NIST SP 800-38A の例)。
 */
#ifndef DTH11_CRYPTO_H
#define DTH11_CRYPTO_H

#include "dth11_core.h"

#define DTH11_SHA1_SIZE 20
#define DTH11_AES_BLOCK 16

struct dth11_sha1 {
	u32 h[5];
	u64 total;
	u8 buf[64];
	u32 buf_len;
};

struct dth11_aes {
	/* 11 個のラウンド鍵 */
	u8 rk[176];
};

/* AES-CTR の鍵ストリーム。カウンターは 128 ビットのビッグエンディアン、使い切るまでブロック内の位置を保つ */
struct dth11_ctr {
	struct dth11_aes aes;
	u8 counter[16];
	u8 ks[16];
	u32 pos;
};

#ifdef __cplusplus
extern "C" {
#endif

void dth11_sha1_init(struct dth11_sha1 *c);
void dth11_sha1_update(struct dth11_sha1 *c, const void *data, size_t len);
void dth11_sha1_final(struct dth11_sha1 *c, u8 out[DTH11_SHA1_SIZE]);
void dth11_sha1(const void *data, size_t len, u8 out[DTH11_SHA1_SIZE]);

void dth11_aes_set_key(struct dth11_aes *a, const u8 key[16]);
void dth11_aes_encrypt_block(const struct dth11_aes *a, const u8 in[16], u8 out[16]);
void dth11_aes_decrypt_block(const struct dth11_aes *a, const u8 in[16], u8 out[16]);
/* CBC 復号 (その場で)。len は 16 の倍数でなければならず、違うときは -EINVAL */
int dth11_aes_cbc_decrypt(const u8 key[16], const u8 iv[16], u8 *data, size_t len);

/* CTR の開始。iv は最初のカウンターブロックで、最初の鍵ストリームは iv そのものの暗号化 */
void dth11_ctr_init(struct dth11_ctr *c, const u8 key[16], const u8 iv[16]);
/* buf を鍵ストリームと XOR する。前回の続きから使う (パケットをまたいで鍵ストリームが続く) */
void dth11_ctr_apply(struct dth11_ctr *c, u8 *buf, size_t len);

/* AES-CMAC (RFC 4493) */
void dth11_aes_cmac(const u8 key[16], const void *data, size_t len, u8 out[16]);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_CRYPTO_H */
