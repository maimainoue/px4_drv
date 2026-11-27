/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 の楕円曲線演算 (driver/common/dth11/dth11_ec) の自動試験。
 *
 * 機器 (DT-H11/U2) が実際に返した証明書の自己署名と、ハンドシェイクの 3 番目の応答の署名を検証できることが、
 * 加算、2 倍、スカラー倍、逆元、SHA-1 までを通した確認になる (機器の署名は外部で作られた値)。
 * そのほか、署名の往復、ECDH の対称性、範囲外の入力の拒否を確認する。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_ec_test.c \
 *            driver/common/dth11/dth11_ec.c driver/common/dth11/dth11_crypto.c -o dth11_ec_test
 */
#include "dth11_ec.h"

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

static void hex(const char *s, u8 *out, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		unsigned int v;

		sscanf(s + 2 * i, "%2x", &v);
		out[i] = (u8)v;
	}
}

/* 機器 (DT-H11/U2) が返した証明書 (ヘッダー 24、公開鍵 40、自己署名 40) */
static const char DEV_CERT[] =
	"1110ffff0411019904110199010000000b08164e00000146c8ac51a0fa7db12b11eb806c1872f9216af049619a12105a3d2584e5fd5"
	"8defee5b5edf4a8eec9238f3e6bcd22cf822154f59f2d1e17db7049b17bf9107b2d704246055ee1cc54626b045a87fd7a7f42";
/* 機器の 3 番目の応答 (一時鍵 40、FF x 8、ホストの乱数 16、署名 40) */
static const char DEV_M3[] =
	"4ce73df13fdf54dd634f48045924cf518ab3a53a1d00fd85b784e8f6ed7ab623cc6849f0a9e5283bffffffffffffffff000102030405060708090a0b0c0d0e0f"
	"74c54aa357535def92e64afb327110b1722122dd64a2fe85cf62a49cc3f76ae7469fa01d00354bf8";

/* 試験用の決定的な乱数 (線形合同法) */
static u32 lcg_state = 12345;

static int test_rng(void *priv, u8 *buf, size_t len)
{
	size_t i;

	(void)priv;
	for (i = 0; i < len; i++) {
		lcg_state = lcg_state * 1664525u + 1013904223u;
		buf[i] = (u8)(lcg_state >> 24);
	}
	return 0;
}

static int failing_rng(void *priv, u8 *buf, size_t len)
{
	(void)priv;
	(void)buf;
	(void)len;
	return -EIO;
}

static void scalar(u8 d[20], unsigned int v)
{
	memset(d, 0, 20);
	d[16] = (u8)(v >> 24);
	d[17] = (u8)(v >> 16);
	d[18] = (u8)(v >> 8);
	d[19] = (u8)v;
}

static void test_device_certificate(void)
{
	u8 cert[104], bad[64];

	hex(DEV_CERT, cert, 104);
	CHECK(dth11_ec_point_check(cert + 24) == 0);
	CHECK(dth11_ec_verify(cert + 24, cert, 64, cert + 64));
	/* メッセージを 1 バイトずらすと不一致 */
	CHECK(!dth11_ec_verify(cert + 24, cert + 1, 64, cert + 64));
	memcpy(bad, cert, 64);
	bad[10] ^= 1;
	CHECK(!dth11_ec_verify(cert + 24, bad, 64, cert + 64));
	/* 署名の 1 ビットを変えると不一致 */
	{
		u8 sig[40];

		memcpy(sig, cert + 64, 40);
		sig[30] ^= 0x10;
		CHECK(!dth11_ec_verify(cert + 24, cert, 64, sig));
	}
}

static void test_device_m3(void)
{
	u8 cert[104], m3[104];

	hex(DEV_CERT, cert, 104);
	hex(DEV_M3, m3, 104);
	CHECK(dth11_ec_point_check(m3) == 0);
	CHECK(dth11_ec_verify(cert + 24, m3, 64, m3 + 64));
	/* 別の鍵 (一時鍵) で検証すると不一致 */
	CHECK(!dth11_ec_verify(m3, m3, 64, m3 + 64));
}

static void test_generator_and_ranges(void)
{
	u8 d[20], q[40], gx[20], gy[20];

	hex("4a96b5688ef573284664698968c38bb913cbfc82", gx, 20);
	hex("23a628553168947d59dcc912042351377ac5fb32", gy, 20);
	scalar(d, 1);
	CHECK(dth11_ec_public_key(d, q) == 0);
	CHECK(!memcmp(q, gx, 20) && !memcmp(q + 20, gy, 20));
	CHECK(dth11_ec_point_check(q) == 0);
	CHECK(dth11_ec_key_matches(d, q));
	scalar(d, 2);
	CHECK(!dth11_ec_key_matches(d, q));

	/* 0 は鍵として使えない */
	memset(d, 0, 20);
	CHECK(dth11_ec_public_key(d, q) == -EINVAL);
	/* n (161 ビット) 以上は 20 バイトに収まらないが、20 バイトの最大値 (2^160 - 1) は n 未満なので受け付ける */
	memset(d, 0xff, 20);
	CHECK(dth11_ec_public_key(d, q) == 0);
	CHECK(dth11_ec_point_check(q) == 0);

	/* 曲線上にない点 */
	scalar(d, 1);
	CHECK(dth11_ec_public_key(d, q) == 0);
	q[39] ^= 1;
	CHECK(dth11_ec_point_check(q) == -EINVAL);
	{
		u8 sh[20];

		CHECK(dth11_ec_ecdh(d, q, sh) == -EINVAL);
	}
}

static void test_ecdh(void)
{
	u8 a[20], b[20], qa[40], qb[40], s1[20], s2[20];
	int i;

	for (i = 0; i < 4; i++) {
		CHECK(dth11_ec_random_scalar(test_rng, NULL, a) == 0);
		CHECK(dth11_ec_random_scalar(test_rng, NULL, b) == 0);
		CHECK(dth11_ec_public_key(a, qa) == 0);
		CHECK(dth11_ec_public_key(b, qb) == 0);
		CHECK(dth11_ec_ecdh(a, qb, s1) == 0);
		CHECK(dth11_ec_ecdh(b, qa, s2) == 0);
		CHECK(!memcmp(s1, s2, 20));
	}
	/* 小さな値でも結合則が成り立つ: (3 * 5) G の x 座標は ECDH(3, 5G) と ECDH(5, 3G) と 15 G で一致 */
	{
		u8 d3[20], d5[20], d15[20], q3[40], q5[40], q15[40], x1[20], x2[20];

		scalar(d3, 3);
		scalar(d5, 5);
		scalar(d15, 15);
		CHECK(dth11_ec_public_key(d3, q3) == 0);
		CHECK(dth11_ec_public_key(d5, q5) == 0);
		CHECK(dth11_ec_public_key(d15, q15) == 0);
		CHECK(dth11_ec_ecdh(d3, q5, x1) == 0);
		CHECK(dth11_ec_ecdh(d5, q3, x2) == 0);
		CHECK(!memcmp(x1, x2, 20) && !memcmp(x1, q15, 20));
	}
}

static void test_sign_verify(void)
{
	u8 d[20], q[40], sig[40], sig2[40];
	static const char msg[] = "hello";
	int i;

	for (i = 0; i < 6; i++) {
		CHECK(dth11_ec_random_scalar(test_rng, NULL, d) == 0);
		CHECK(dth11_ec_public_key(d, q) == 0);
		CHECK(dth11_ec_sign(d, msg, 5, test_rng, NULL, sig) == 0);
		CHECK(dth11_ec_verify(q, msg, 5, sig));
		CHECK(!dth11_ec_verify(q, "hellp", 5, sig));
		/* 署名ごとに k が違うので、同じ内容でも署名の値は変わり、どちらも検証できる */
		CHECK(dth11_ec_sign(d, msg, 5, test_rng, NULL, sig2) == 0);
		CHECK(memcmp(sig, sig2, 40) != 0);
		CHECK(dth11_ec_verify(q, msg, 5, sig2));
	}
	/* r か s が 0 の署名は拒否 */
	memset(sig, 0, 40);
	CHECK(!dth11_ec_verify(q, msg, 5, sig));
	/* 乱数の失敗はそのまま返す */
	CHECK(dth11_ec_sign(d, msg, 5, failing_rng, NULL, sig) == -EIO);
	/* 範囲外の秘密鍵 */
	memset(d, 0, 20);
	CHECK(dth11_ec_sign(d, msg, 5, test_rng, NULL, sig) == -EINVAL);
}

int main(void)
{
	test_device_certificate();
	test_device_m3();
	test_generator_and_ranges();
	test_ecdh();
	test_sign_verify();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
