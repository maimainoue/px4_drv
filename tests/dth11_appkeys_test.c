/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 の資格情報の読み出し (driver/common/dth11/dth11_appkeys) の自動試験。
 *
 * 純正の DLL はリポジトリに含められないため、試験は次の 2 つで行う。
 *  1. 合成した PE 画像: 自己署名の証明書と秘密鍵をバイトごとの即値の格納命令 (mov byte ptr [addr], imm8) で
 *     データ領域へ書くコードと、16 バイトのスタックバッファを組み立てるコードを持つ最小の PE を作り、
 *     証明書、秘密鍵、IV を取り出せること。PE でない入力、証明書が無い画像、秘密鍵が無い画像の拒否。
 *  2. 環境変数 DTH11_TEST_COBDACOM に純正の CoBDACom.dll のパスを指定したときだけ、その実物から
 *     証明書と秘密鍵を取り出せること (指定がなければ省略)。値は表示しない。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_appkeys_test.c \
 *            driver/common/dth11/dth11_appkeys.c driver/common/dth11/dth11_ec.c \
 *            driver/common/dth11/dth11_crypto.c -o dth11_appkeys_test
 */
#include "dth11_appkeys.h"
#include "dth11_crypto.h"
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

static u32 lcg_state = 4242;

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

#define IMAGE_BASE 0x10000000u
#define DATA_VA 0x2000u

static void put16(u8 *p, u16 v)
{
	p[0] = (u8)v;
	p[1] = (u8)(v >> 8);
}

static void put32(u8 *p, u32 v)
{
	put16(p, (u16)v);
	put16(p + 2, (u16)(v >> 16));
}

/* mov byte ptr [abs32], imm8 */
static size_t emit_mov_abs8(u8 *code, size_t n, u32 addr, u8 v)
{
	code[n++] = 0xc6;
	code[n++] = 0x05;
	put32(code + n, addr);
	n += 4;
	code[n++] = v;
	return n;
}

/* mov byte ptr [ebp + disp8], imm8 */
static size_t emit_mov_ebp8(u8 *code, size_t n, s32 disp, u8 v)
{
	code[n++] = 0xc6;
	code[n++] = 0x45;
	code[n++] = (u8)disp;
	code[n++] = v;
	return n;
}

struct synth {
	u8 *image;
	size_t len;
	u8 cert[DTH11_CERT_LEN];
	u8 priv[DTH11_PRIV_LEN];
	u8 iv[16];
};

/*
 * 最小の PE32 (.text と .data) を作る。cert_ok が偽なら証明書の署名を壊し、priv_ok が偽なら秘密鍵を
 * 別の値にする。コードの中身: 証明書 104 バイト、秘密鍵 20 バイト (直後のアドレス)、IV の 16 バイト (ebp の下)。
 */
static void make_pe(struct synth *s, bool cert_ok, bool priv_ok)
{
	u8 q[40], header[24], sig[40], *code, *p;
	size_t n = 0, i, code_len, raw_text = 0x400, raw_data;
	u32 cert_addr = IMAGE_BASE + DATA_VA;

	CHECK(dth11_ec_random_scalar(test_rng, NULL, s->priv) == 0);
	CHECK(dth11_ec_public_key(s->priv, q) == 0);
	memset(header, 0, sizeof(header));
	header[0] = 0x11;
	header[1] = 0x10;
	memcpy(header + 4, "TESTCERT", 8);
	/* 証明書: ヘッダー 24、公開鍵 40、ヘッダー + 公開鍵への自己署名 40 */
	memcpy(s->cert, header, 24);
	memcpy(s->cert + 24, q, 40);
	CHECK(dth11_ec_sign(s->priv, s->cert, 64, test_rng, NULL, sig) == 0);
	memcpy(s->cert + 64, sig, 40);
	if (!cert_ok)
		s->cert[70] ^= 0x01;
	test_rng(NULL, s->iv, 16);

	code = calloc(1, 4096);
	for (i = 0; i < DTH11_CERT_LEN; i++)
		n = emit_mov_abs8(code, n, cert_addr + (u32)i, s->cert[i]);
	for (i = 0; i < DTH11_PRIV_LEN; i++) {
		u8 v = s->priv[i];

		if (!priv_ok)
			v ^= 0x5a;
		n = emit_mov_abs8(code, n, cert_addr + DTH11_CERT_LEN + (u32)i, v);
	}
	/* 関数の間のゴミ */
	for (i = 0; i < 40; i++)
		code[n++] = (u8)(0x90 + (i % 7));
	/* スタックの 16 バイト: [ebp - 0x20 .. ebp - 0x11] に順に書く */
	for (i = 0; i < 16; i++)
		n = emit_mov_ebp8(code, n, -0x20 + (s32)i, s->iv[i]);
	code[n++] = 0xc3;
	code_len = n;

	raw_data = raw_text + 0x200 * ((code_len + 0x1ff) / 0x200);
	s->len = raw_data + 0x200;
	s->image = calloc(1, s->len);
	p = s->image;
	p[0] = 'M';
	p[1] = 'Z';
	put32(p + 0x3c, 0x40);
	memcpy(p + 0x40, "PE\0\0", 4);
	put16(p + 0x44, 0x14c); /* i386 */
	put16(p + 0x46, 2); /* セクション数 */
	put16(p + 0x54, 0xe0); /* オプションヘッダーの大きさ */
	put16(p + 0x58, 0x10b); /* PE32 */
	put32(p + 0x58 + 28, IMAGE_BASE);
	/* セクション表 (オプションヘッダーの後) */
	{
		u8 *t = p + 0x58 + 0xe0;

		memcpy(t, ".text", 5);
		put32(t + 8, (u32)code_len);
		put32(t + 12, 0x1000);
		put32(t + 16, (u32)(raw_data - raw_text));
		put32(t + 20, (u32)raw_text);
		put32(t + 36, 0x60000020);
		t += 40;
		memcpy(t, ".data", 5);
		put32(t + 8, 0x1000);
		put32(t + 12, DATA_VA);
		put32(t + 16, 0x200);
		put32(t + 20, (u32)raw_data);
		put32(t + 36, 0xc0000040);
	}
	memcpy(p + raw_text, code, code_len);
	/* データ領域は、証明書のアドレスに別の値 (ゴミ) を置く。実行時に書き込まれる値が採用される */
	test_rng(NULL, p + raw_data, 0x200);
	free(code);
}

static void free_synth(struct synth *s)
{
	free(s->image);
	memset(s, 0, sizeof(*s));
}

static void test_synthetic_extract(void)
{
	struct synth s;
	struct dth11_appkeys keys;
	u8 k3[16], blob[64], d[32], iv[16], mac[16], key_mac[16], content_key[16], prev[16];
	struct dth11_aes a;
	size_t i;
	int c;

	make_pe(&s, true, true);
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == 0);
	CHECK(!memcmp(keys.cert, s.cert, DTH11_CERT_LEN));
	CHECK(!memcmp(keys.priv_key, s.priv, DTH11_PRIV_LEN));

	/* IV の候補に、コードが組み立てたバッファ (と、その反転) が入っている */
	{
		bool found = false, found_rev = false;
		u8 rev[16];

		for (c = 0; c < 16; c++)
			rev[c] = s.iv[15 - c];
		for (i = 0; i < keys.iv_candidate_count; i++) {
			if (!memcmp(keys.iv_candidates[i], s.iv, 16))
				found = true;
			if (!memcmp(keys.iv_candidates[i], rev, 16))
				found_rev = true;
		}
		CHECK(found && found_rev);
	}

	/* コンテンツブロックを作る: id 16、CBC(IV) で包んだ 32 バイト (CMAC 鍵 16 + コンテンツ鍵 16)、CMAC 16 */
	test_rng(NULL, k3, 16);
	test_rng(NULL, content_key, 16);
	test_rng(NULL, key_mac, 16);
	test_rng(NULL, blob, 16);
	memcpy(d, key_mac, 16);
	memcpy(d + 16, content_key, 16);
	dth11_aes_set_key(&a, k3);
	memcpy(prev, s.iv, 16);
	for (i = 0; i < 32; i += 16) {
		u8 x[16];

		for (c = 0; c < 16; c++)
			x[c] = (u8)(d[i + c] ^ prev[c]);
		dth11_aes_encrypt_block(&a, x, blob + 16 + i);
		memcpy(prev, blob + 16 + i, 16);
	}
	dth11_aes_cmac(key_mac, blob, 48, mac);
	memcpy(blob + 48, mac, 16);

	CHECK(!keys.iv_settled);
	CHECK(dth11_appkeys_find_content_iv(&keys, k3, blob, iv) == 0);
	CHECK(!memcmp(iv, s.iv, 16));
	CHECK(keys.iv_settled);
	/* 決まった後は、ブロックが違っても同じ IV を返す */
	blob[50] ^= 1;
	CHECK(dth11_appkeys_find_content_iv(&keys, k3, blob, iv) == 0 && !memcmp(iv, s.iv, 16));
	dth11_appkeys_free(&keys);

	/* CMAC が合わないブロックでは決まらない */
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == 0);
	CHECK(dth11_appkeys_find_content_iv(&keys, k3, blob, iv) == -ENOENT);
	CHECK(!keys.iv_settled);
	dth11_appkeys_free(&keys);
	free_synth(&s);
}

static void test_rejects(void)
{
	struct synth s;
	struct dth11_appkeys keys;
	u8 junk[256];

	/* PE でない */
	memset(junk, 0x41, sizeof(junk));
	CHECK(dth11_appkeys_from_image(junk, sizeof(junk), &keys) == DTH11_APPKEYS_NOT_PE);
	CHECK(dth11_appkeys_from_image(junk, 1, &keys) == DTH11_APPKEYS_NOT_PE);
	CHECK(dth11_appkeys_from_image(junk, 0, &keys) == DTH11_APPKEYS_NOT_PE);

	/* 署名の壊れた証明書は、証明書として採用しない */
	make_pe(&s, false, true);
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == DTH11_APPKEYS_NO_CERT);
	free_synth(&s);

	/* 秘密鍵が公開鍵に対応しない */
	make_pe(&s, true, false);
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == DTH11_APPKEYS_NO_PRIVATE_KEY);
	free_synth(&s);

	/* 切り詰められた PE */
	make_pe(&s, true, true);
	CHECK(dth11_appkeys_from_image(s.image, 0x50, &keys) == DTH11_APPKEYS_NOT_PE);
	free_synth(&s);

	/* PE32+ は x86 の格納命令の走査の対象外。黙って空の画像を作らず、PE として受け付けない */
	make_pe(&s, true, true);
	put16(s.image + 0x58, 0x20b);
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == DTH11_APPKEYS_NOT_PE);
	free_synth(&s);

	/* 重なるセクション: 2 つのセクションがファイル全体を指すと、合計が画像の長さを越える (確保した領域への書き込み超過) */
	make_pe(&s, true, true);
	{
		u8 *t = s.image + 0x58 + 0xe0;

		put32(t + 16, (u32)s.len);
		put32(t + 20, 0);
		put32(t + 40 + 16, (u32)s.len);
		put32(t + 40 + 20, 0);
	}
	CHECK(dth11_appkeys_from_image(s.image, s.len, &keys) == DTH11_APPKEYS_NOT_PE);
	free_synth(&s);
}

static void test_real_dll(void)
{
	const char *path = getenv("DTH11_TEST_COBDACOM");
	FILE *f;
	u8 *buf;
	long n;
	struct dth11_appkeys keys;
	u8 q[40];

	if (!path || !*path) {
		printf("SKIP real DLL (DTH11_TEST_COBDACOM not set)\n");
		return;
	}
	f = fopen(path, "rb");
	if (!f) {
		printf("FAIL cannot open %s\n", path);
		fails++;
		return;
	}
	fseek(f, 0, SEEK_END);
	n = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf = malloc((size_t)n);
	if (fread(buf, 1, (size_t)n, f) != (size_t)n) {
		printf("FAIL read %s\n", path);
		fails++;
		fclose(f);
		free(buf);
		return;
	}
	fclose(f);
	CHECK(dth11_appkeys_from_image(buf, (size_t)n, &keys) == 0);
	CHECK(dth11_ec_verify(keys.cert + 24, keys.cert, 64, keys.cert + 64));
	CHECK(dth11_ec_public_key(keys.priv_key, q) == 0 && !memcmp(q, keys.cert + 24, 40));
	CHECK(keys.iv_candidate_count > 0);
	printf("real DLL: %zu IV candidates, %zu data bytes\n", keys.iv_candidate_count, keys.data_len);
	dth11_appkeys_free(&keys);
	free(buf);
}

int main(void)
{
	test_synthetic_extract();
	test_rejects();
	test_real_dll();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
