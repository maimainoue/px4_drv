/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 の暗号部品 (driver/common/dth11/dth11_crypto) の自動試験。
 * 規格の例 (FIPS 180-1、FIPS 197、NIST SP 800-38A、RFC 4493) の値と一致することを確認する。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_crypto_test.c \
 *            driver/common/dth11/dth11_crypto.c -o dth11_crypto_test
 */
#include "dth11_crypto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks, fails;

#define CHECK(c)                                                                  \
	do {                                                                      \
		checks++;                                                         \
		if (!(c)) {                                                       \
			fails++;                                                  \
			printf("FAIL %s:%d: %s\n", __func__, __LINE__, #c);       \
		}                                                                 \
	} while (0)

/* 16 進の文字列を n バイトに変換する */
static void hex(const char *s, u8 *out, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		unsigned int v;

		sscanf(s + 2 * i, "%2x", &v);
		out[i] = (u8)v;
	}
}

static int same(const u8 *a, const char *want, size_t n)
{
	u8 w[64];

	hex(want, w, n);
	return !memcmp(a, w, n);
}

static void test_sha1(void)
{
	u8 d[20];
	struct dth11_sha1 c;
	static u8 million[1000];
	int i;

	dth11_sha1("abc", 3, d);
	CHECK(same(d, "a9993e364706816aba3e25717850c26c9cd0d89d", 20));
	dth11_sha1("", 0, d);
	CHECK(same(d, "da39a3ee5e6b4b0d3255bfef95601890afd80709", 20));
	dth11_sha1("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56, d);
	CHECK(same(d, "84983e441c3bd26ebaae4aa1f95129e5e54670f1", 20));
	/* 細切れの更新でも同じ値。a を 100 万個 */
	memset(million, 'a', sizeof(million));
	dth11_sha1_init(&c);
	for (i = 0; i < 1000; i++)
		dth11_sha1_update(&c, million, sizeof(million));
	dth11_sha1_final(&c, d);
	CHECK(same(d, "34aa973cd4c4daa4f61eeb2bdbad27316534016f", 20));
}

static void test_aes(void)
{
	struct dth11_aes a;
	u8 key[16], pt[16], ct[16], back[16];

	/* FIPS 197 付録 C.1 */
	hex("000102030405060708090a0b0c0d0e0f", key, 16);
	hex("00112233445566778899aabbccddeeff", pt, 16);
	dth11_aes_set_key(&a, key);
	dth11_aes_encrypt_block(&a, pt, ct);
	CHECK(same(ct, "69c4e0d86a7b0430d8cdb78070b4c55a", 16));
	dth11_aes_decrypt_block(&a, ct, back);
	CHECK(!memcmp(back, pt, 16));

	/* FIPS 197 付録 B */
	hex("2b7e151628aed2a6abf7158809cf4f3c", key, 16);
	hex("3243f6a8885a308d313198a2e0370734", pt, 16);
	dth11_aes_set_key(&a, key);
	dth11_aes_encrypt_block(&a, pt, ct);
	CHECK(same(ct, "3925841d02dc09fbdc118597196a0b32", 16));
}

static void test_cbc(void)
{
	u8 key[16], iv[16], data[32], want[32];

	/* SP 800-38A F.2.2 (CBC-AES128.Decrypt) */
	hex("2b7e151628aed2a6abf7158809cf4f3c", key, 16);
	hex("000102030405060708090a0b0c0d0e0f", iv, 16);
	hex("7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2", data, 32);
	hex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51", want, 32);
	CHECK(dth11_aes_cbc_decrypt(key, iv, data, 32) == 0);
	CHECK(!memcmp(data, want, 32));
	/* 16 の倍数でない長さは断る */
	CHECK(dth11_aes_cbc_decrypt(key, iv, data, 17) == -EINVAL);
}

static void test_ctr(void)
{
	struct dth11_ctr c;
	u8 key[16], ctr[16], data[64], want[64], one[64];
	size_t i;

	/* SP 800-38A F.5.1 (CTR-AES128.Encrypt) の最初の 2 ブロック */
	hex("2b7e151628aed2a6abf7158809cf4f3c", key, 16);
	hex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff", ctr, 16);
	hex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51", data, 32);
	hex("874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff", want, 32);
	memcpy(one, data, 32);
	dth11_ctr_init(&c, key, ctr);
	dth11_ctr_apply(&c, data, 32);
	CHECK(!memcmp(data, want, 32));

	/* 1 バイトずつ、不揃いな長さで刻んでも同じ鍵ストリーム */
	dth11_ctr_init(&c, key, ctr);
	for (i = 0; i < 32; i++)
		dth11_ctr_apply(&c, one + i, 1);
	CHECK(!memcmp(one, want, 32));
	memset(data, 0, sizeof(data));
	dth11_ctr_init(&c, key, ctr);
	dth11_ctr_apply(&c, data, 5);
	dth11_ctr_apply(&c, data + 5, 27);
	dth11_ctr_apply(&c, data + 32, 32);
	{
		u8 whole[64] = { 0 };
		struct dth11_ctr c2;

		dth11_ctr_init(&c2, key, ctr);
		dth11_ctr_apply(&c2, whole, 64);
		CHECK(!memcmp(data, whole, 64));
	}
	/* カウンターの桁上がり: 末尾が ff ff のとき、次のブロックで上の桁へ繰り上がる */
	hex("00000000000000000000000000ffffff", ctr, 16);
	{
		u8 a[32] = { 0 }, b[32] = { 0 };
		struct dth11_ctr c3;
		u8 ctr2[16];

		dth11_ctr_init(&c3, key, ctr);
		dth11_ctr_apply(&c3, a, 32);
		hex("00000000000000000000000001000000", ctr2, 16);
		dth11_ctr_init(&c3, key, ctr2);
		dth11_ctr_apply(&c3, b, 16);
		CHECK(!memcmp(a + 16, b, 16));
	}
}

static void test_cmac(void)
{
	u8 key[16], msg[64], mac[16];

	/* RFC 4493 の例 1 〜 4 */
	hex("2b7e151628aed2a6abf7158809cf4f3c", key, 16);
	hex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710",
	    msg, 64);
	dth11_aes_cmac(key, msg, 0, mac);
	CHECK(same(mac, "bb1d6929e95937287fa37d129b756746", 16));
	dth11_aes_cmac(key, msg, 16, mac);
	CHECK(same(mac, "070a16b46b4d4144f79bdd9dd04a287c", 16));
	dth11_aes_cmac(key, msg, 40, mac);
	CHECK(same(mac, "dfa66747de9ae63030ca32611497c827", 16));
	dth11_aes_cmac(key, msg, 64, mac);
	CHECK(same(mac, "51f0bebf7e3b9d92fc49741779363cfe", 16));
	/* 長さを 1 バイトずらすと別の値 */
	dth11_aes_cmac(key, msg, 63, mac);
	CHECK(!same(mac, "51f0bebf7e3b9d92fc49741779363cfe", 16));
}

int main(void)
{
	test_sha1();
	test_aes();
	test_cbc();
	test_ctr();
	test_cmac();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
