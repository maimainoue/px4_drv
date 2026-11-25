// SPDX-License-Identifier: GPL-2.0-only
/*
 * asv5211fw.c
 *
 * ASICEN ASV5211 の USB ブリッジ (cold 状態の USB ID 1738:5211 / 1738:5216) へ送る 16 KB の firmware を、
 * メーカー製 Windows ドライバーの firmware ローダー (AS11Loader.sys 系) から取り出す。
 *
 * ローダーは firmware を .data の中へ、各バイトに 1 を足してから AES-128-ECB で暗号化して持つ。
 * 暗号化の方式と鍵は、waya0125/recfriio (MIT、internal/firmware/firmware.go) が公開している。
 * 取り出した firmware はメーカーの著作物であり、このリポジトリには含めない。
 *
 * ローダーごとに firmware の位置が異なるため、位置の表は持たず、ファイル全体を走査する。
 * 16 KB のうち、どの firmware でも 0 で埋まっている 2 か所 (0x0c00-0x2000 と 0x2400-0x2800) が、
 * 復号後に 0 になる位置を、firmware の候補とする。
 * 候補は SHA-256 で既知の版と照合し、起動アドレス (ブート ROM へ渡す wIndex) を表示する。
 *
 * build: gcc -O2 -Wall -o asv5211fw asv5211fw.c
 * usage: asv5211fw [-l] [-n NUMBER] LOADER.sys [OUTPUT.bin]
 * 詳細は docs/devices/hz2w.md を参照。
 */
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FW_SIZE		0x4000
#define MAX_CANDIDATES	16

/* loader が firmware を暗号化する鍵 (waya0125/recfriio の公開値) */
static const uint8_t loader_key[16] = {
	0x27, 0x78, 0x83, 0xa7, 0x07, 0xd7, 0x48, 0xed, 0x8e, 0x12, 0xab, 0xfb, 0x89, 0xcc, 0xbd, 0x77
};

/* ---------------------------------------------------------------- SHA-256 */

struct sha256 {
	uint32_t h[8];
	uint8_t buf[64];
	uint64_t len;
	size_t fill;
};

static const uint32_t sha_k[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

#define ROR32(x, n)	(((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(struct sha256 *s, const uint8_t *p)
{
	uint32_t w[64], a, b, c, d, e, f, g, h, t1, t2;
	int i;

	for (i = 0; i < 16; i++)
		w[i] = (uint32_t)p[i * 4] << 24 | (uint32_t)p[i * 4 + 1] << 16 | (uint32_t)p[i * 4 + 2] << 8 | p[i * 4 + 3];
	for (i = 16; i < 64; i++) {
		uint32_t s0 = ROR32(w[i - 15], 7) ^ ROR32(w[i - 15], 18) ^ (w[i - 15] >> 3);
		uint32_t s1 = ROR32(w[i - 2], 17) ^ ROR32(w[i - 2], 19) ^ (w[i - 2] >> 10);

		w[i] = w[i - 16] + s0 + w[i - 7] + s1;
	}
	a = s->h[0]; b = s->h[1]; c = s->h[2]; d = s->h[3];
	e = s->h[4]; f = s->h[5]; g = s->h[6]; h = s->h[7];
	for (i = 0; i < 64; i++) {
		t1 = h + (ROR32(e, 6) ^ ROR32(e, 11) ^ ROR32(e, 25)) + ((e & f) ^ (~e & g)) + sha_k[i] + w[i];
		t2 = (ROR32(a, 2) ^ ROR32(a, 13) ^ ROR32(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
		h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
	}
	s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d;
	s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}

static void sha256_init(struct sha256 *s)
{
	static const uint32_t init[8] = {
		0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
	};

	memcpy(s->h, init, sizeof(init));
	s->len = 0;
	s->fill = 0;
}

static void sha256_update(struct sha256 *s, const uint8_t *p, size_t n)
{
	s->len += n;
	while (n) {
		size_t k = 64 - s->fill;

		if (k > n)
			k = n;
		memcpy(s->buf + s->fill, p, k);
		s->fill += k;
		p += k;
		n -= k;
		if (s->fill == 64) {
			sha256_block(s, s->buf);
			s->fill = 0;
		}
	}
}

static void sha256_final(struct sha256 *s, char hex[65])
{
	uint64_t bits = s->len * 8;
	uint8_t pad = 0x80, zero = 0, lenbuf[8];
	int i;

	sha256_update(s, &pad, 1);
	while (s->fill != 56)
		sha256_update(s, &zero, 1);
	for (i = 0; i < 8; i++)
		lenbuf[i] = (uint8_t)(bits >> (56 - i * 8));
	sha256_update(s, lenbuf, 8);
	for (i = 0; i < 8; i++)
		snprintf(hex + i * 8, 9, "%08x", s->h[i]);
}

static void sha256_hex(const uint8_t *p, size_t n, char hex[65])
{
	struct sha256 s;

	sha256_init(&s);
	sha256_update(&s, p, n);
	sha256_final(&s, hex);
}

/* ---------------------------------------------------------------- AES-128 (復号のみ) */

static uint8_t sbox[256], inv_sbox[256];
static uint8_t round_key[11][16];

static uint8_t xtime(uint8_t x)
{
	return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0));
}

static uint8_t gmul(uint8_t a, uint8_t b)
{
	uint8_t r = 0;

	while (b) {
		if (b & 1)
			r ^= a;
		a = xtime(a);
		b >>= 1;
	}
	return r;
}

/* FIPS-197 の定義どおり、GF(2^8) の逆元とアフィン変換から S-box を作る (表の転記による誤りを避ける) */
static void aes_init_tables(void)
{
	int i;

	for (i = 0; i < 256; i++) {
		uint8_t inv = 0, x, y;
		int j;

		if (i) {
			for (j = 1; j < 256; j++) {
				if (gmul((uint8_t)i, (uint8_t)j) == 1) {
					inv = (uint8_t)j;
					break;
				}
			}
		}
		x = inv;
		y = (uint8_t)(x ^ ((x << 1) | (x >> 7)) ^ ((x << 2) | (x >> 6)) ^ ((x << 3) | (x >> 5)) ^ ((x << 4) | (x >> 4)) ^ 0x63);
		sbox[i] = y;
		inv_sbox[y] = (uint8_t)i;
	}
}

static void aes_set_key(const uint8_t key[16])
{
	uint8_t rcon = 1;
	int i, j;

	memcpy(round_key[0], key, 16);
	for (i = 1; i < 11; i++) {
		uint8_t t[4];

		for (j = 0; j < 4; j++)
			t[j] = sbox[round_key[i - 1][12 + (j + 1) % 4]];
		t[0] ^= rcon;
		rcon = xtime(rcon);
		for (j = 0; j < 4; j++)
			round_key[i][j] = round_key[i - 1][j] ^ t[j];
		for (j = 4; j < 16; j++)
			round_key[i][j] = round_key[i - 1][j] ^ round_key[i][j - 4];
	}
}

static void aes_decrypt_block(const uint8_t in[16], uint8_t out[16])
{
	uint8_t s[16], t[16];
	int r, i;

	for (i = 0; i < 16; i++)
		s[i] = in[i] ^ round_key[10][i];
	for (r = 9; r >= 0; r--) {
		/* InvShiftRows (列優先の配置: s[col * 4 + row]) */
		for (i = 0; i < 16; i++) {
			int row = i % 4, col = i / 4;

			t[((col + row) % 4) * 4 + row] = s[i];
		}
		/* InvSubBytes と AddRoundKey */
		for (i = 0; i < 16; i++)
			s[i] = (uint8_t)(inv_sbox[t[i]] ^ round_key[r][i]);
		if (r) {
			/* InvMixColumns */
			for (i = 0; i < 4; i++) {
				uint8_t a0 = s[i * 4], a1 = s[i * 4 + 1], a2 = s[i * 4 + 2], a3 = s[i * 4 + 3];

				s[i * 4] = gmul(a0, 14) ^ gmul(a1, 11) ^ gmul(a2, 13) ^ gmul(a3, 9);
				s[i * 4 + 1] = gmul(a0, 9) ^ gmul(a1, 14) ^ gmul(a2, 11) ^ gmul(a3, 13);
				s[i * 4 + 2] = gmul(a0, 13) ^ gmul(a1, 9) ^ gmul(a2, 14) ^ gmul(a3, 11);
				s[i * 4 + 3] = gmul(a0, 11) ^ gmul(a1, 13) ^ gmul(a2, 9) ^ gmul(a3, 14);
			}
		}
	}
	memcpy(out, s, 16);
}

/* ---------------------------------------------------------------- firmware の検出 */

struct known_image {
	const char *sha256;
	uint16_t index;
	int legacy;	/* ローダーが持つが、送られない古い版 */
	const char *name;
};

/*
 * 起動アドレス (wIndex) は firmware の版で決まり、機種では決まらない。違うアドレスで起動すると、
 * USB の抜き差しが要る。HZ2W の版は、I-O DATA の GV-MVP/HZ2W 用ドライバー (1.12.04.19) から取り出して
 * 起動アドレスを確認した。SKNET の版は、SKNET_AS11Loader.sys から取り出し、ローダーが各イメージの
 * 直前に持つ起動アドレスの値 (0x55d8) と、Till0196/coax の既知の値の一致で決めた (実機での起動は未確認)。
 * ほかの 4 件は waya0125/recfriio の docs と、Till0196/coax の記述による。
 */
static const struct known_image known_images[] = {
	{ "39428b362d6be2284814d418e57ad86274749973eaf4c68d459c02b072488e41", 0x5583, 0,
	  "I-O DATA GV-MVP/HZ2W 用ドライバー 1.12.04.19 (2012-04-19) の firmware" },
	{ "d134890df91427d405ad5619588e63e3220e21f972ede18a9701fec5686b4816", 0x515f, 1,
	  "古い版 (Quixun と PLEX と I-O DATA のローダーが持つ、送られないもの)" },
	{ "1f70eb5934456b3b78b0c5ff09a1713e55449cfcd40cc3bcc6a9762f00ec08a0", 0x5399, 0,
	  "Quixun QRS-UT100B 1.08.11.17 の firmware" },
	{ "50d406d570a9357451a1911645bc3ec1bd64c4e59f9919259d3144efafcc38a4", 0x53e4, 0,
	  "Quixun QRS-UT100B 1.09.02.19 (forNetbook) の firmware" },
	{ "b45d510200a1690b3ca358d93de13f40e1d3567b663c17e773349ad96f597aa8", 0x5399, 0,
	  "PLEX の Linux ドライバー (loader.ko) の firmware" },
	{ "f4848c8c091634897f9829e50d2ff8e5dc28792c6b20cf095d38d40379518c7a", 0x55d8, 0,
	  "SKNET のローダー (SKNET_AS11Loader.sys、QRS-UT100B.zip 同梱) の firmware" }
};

#define KNOWN_NUM	(sizeof(known_images) / sizeof(known_images[0]))

struct candidate {
	size_t offset;
	uint8_t image[FW_SIZE];
	char sha256[65];
	const struct known_image *known;
};

static void decrypt_at(const uint8_t *file, size_t off, size_t n, uint8_t *out)
{
	size_t i, k;

	for (i = 0; i < n; i += 16) {
		uint8_t sealed[16];

		for (k = 0; k < 16; k++)
			sealed[k] = (uint8_t)(file[off + i + k] - 1);
		aes_decrypt_block(sealed, out + i);
	}
}

static int is_zero(const uint8_t *p, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		if (p[i])
			return 0;
	return 1;
}

/* 送らない 2 か所が 0 で、それ以外に十分な量の内容があれば、firmware の形とみなす */
static int looks_like_firmware(const uint8_t *image)
{
	size_t i, nonzero = 0;

	if (!is_zero(image + 0x0c00, 0x2000 - 0x0c00) || !is_zero(image + 0x2400, 0x2800 - 0x2400))
		return 0;
	for (i = 0; i < FW_SIZE; i++)
		if (image[i])
			nonzero++;
	return nonzero >= 0x400;
}

static int find_candidates(const uint8_t *file, size_t size, struct candidate *list, int max)
{
	size_t off;
	int n = 0;
	uint8_t probe[16];

	if (size < FW_SIZE)
		return 0;
	for (off = 0; off + FW_SIZE <= size && n < max; off++) {
		size_t i;

		/* 先に、送らない領域の 3 か所だけを復号して、候補でない位置を速く除く */
		decrypt_at(file, off + 0x0c00, 16, probe);
		if (!is_zero(probe, 16))
			continue;
		decrypt_at(file, off + 0x1ff0, 16, probe);
		if (!is_zero(probe, 16))
			continue;
		decrypt_at(file, off + 0x2400, 16, probe);
		if (!is_zero(probe, 16))
			continue;
		decrypt_at(file, off, FW_SIZE, list[n].image);
		if (!looks_like_firmware(list[n].image))
			continue;
		list[n].offset = off;
		sha256_hex(list[n].image, FW_SIZE, list[n].sha256);
		list[n].known = NULL;
		for (i = 0; i < KNOWN_NUM; i++)
			if (!strcmp(list[n].sha256, known_images[i].sha256))
				list[n].known = &known_images[i];
		n++;
		off += FW_SIZE - 1;	/* 見つけた firmware と重なる位置は、次の候補に数えない */
	}
	return n;
}

/* ---------------------------------------------------------------- 入出力と自己検査 */

static int load_file(const char *path, uint8_t **buf, size_t *size)
{
	FILE *fp = fopen(path, "rb");
	long len;

	if (!fp) {
		fprintf(stderr, "%s を開けません\n", path);
		return -1;
	}
	if (fseek(fp, 0, SEEK_END) || (len = ftell(fp)) < 0 || fseek(fp, 0, SEEK_SET)) {
		fprintf(stderr, "%s の大きさを取得できません\n", path);
		fclose(fp);
		return -1;
	}
	*buf = malloc((size_t)len ? (size_t)len : 1);
	if (!*buf || fread(*buf, 1, (size_t)len, fp) != (size_t)len) {
		fprintf(stderr, "%s を読めません\n", path);
		free(*buf);
		fclose(fp);
		return -1;
	}
	fclose(fp);
	*size = (size_t)len;
	return 0;
}

static int selftest(int verbose)
{
	/* FIPS-197 付録 C.1 の AES-128 と、FIPS 180-4 の SHA-256("abc") */
	static const uint8_t key[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	static const uint8_t cipher[16] = {
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a
	};
	static const uint8_t plain[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
	};
	uint8_t out[16];
	char hex[65];
	int ok = 1;

	aes_set_key(key);
	aes_decrypt_block(cipher, out);
	if (memcmp(out, plain, 16)) {
		fprintf(stderr, "selftest: AES-128 の復号が FIPS-197 の値と一致しません\n");
		ok = 0;
	}
	sha256_hex((const uint8_t *)"abc", 3, hex);
	if (strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")) {
		fprintf(stderr, "selftest: SHA-256 が FIPS 180-4 の値と一致しません\n");
		ok = 0;
	}
	aes_set_key(loader_key);
	if (ok && verbose)
		printf("selftest: OK\n");
	return ok ? 0 : 1;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"usage: %s [-l] [-n NUMBER] LOADER.sys [OUTPUT.bin]\n"
		"  LOADER.sys  メーカー製ドライバーの firmware ローダー (IODATA_AS11Loader.sys など)\n"
		"  OUTPUT.bin  取り出した 16 KB の firmware の書き出し先\n"
		"  -l          候補を一覧するだけで、書き出さない\n"
		"  -n NUMBER   書き出す候補の番号 (一覧の左端、0 から)。省略すると、既知で、古い版でない最初の候補\n"
		"  --selftest  AES-128 と SHA-256 の自己検査をして終わる\n",
		prog);
}

int main(int argc, char **argv)
{
	static struct candidate list[MAX_CANDIDATES];
	const char *input = NULL, *output = NULL;
	int list_only = 0, pick = -1, i, n, ret = 1;
	uint8_t *file = NULL;
	size_t size = 0;
	FILE *fp;

	aes_init_tables();
	aes_set_key(loader_key);
	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--selftest"))
			return selftest(1);
		if (!strcmp(argv[i], "-l")) {
			list_only = 1;
		} else if (!strcmp(argv[i], "-n") && i + 1 < argc) {
			char *end;
			long value;

			errno = 0;
			value = strtol(argv[++i], &end, 10);
			if (end == argv[i] || *end || errno || value < 0 || value > INT_MAX) {
				fprintf(stderr, "-n の値が不正です: %s\n", argv[i]);
				usage(argv[0]);
				return 2;
			}
			pick = (int)value;
		} else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage(argv[0]);
			return 0;
		} else if (argv[i][0] == '-' && argv[i][1]) {
			usage(argv[0]);
			return 2;
		} else if (!input) {
			input = argv[i];
		} else if (!output) {
			output = argv[i];
		} else {
			usage(argv[0]);
			return 2;
		}
	}
	if (!input || (!output && !list_only)) {
		usage(argv[0]);
		return 2;
	}
	if (selftest(0))
		return 1;
	if (load_file(input, &file, &size))
		return 1;

	n = find_candidates(file, size, list, MAX_CANDIDATES);
	if (!n) {
		fprintf(stderr, "%s に firmware が見つかりません (ASV5211 の firmware ローダーではないか、暗号化の方式が違います)\n", input);
		goto out;
	}
	for (i = 0; i < n; i++) {
		printf("[%d] offset 0x%zx  sha256 %s\n", i, list[i].offset, list[i].sha256);
		if (list[i].known)
			printf("    起動アドレス (wIndex) 0x%04x  %s%s\n", list[i].known->index, list[i].known->name,
			       list[i].known->legacy ? " [古い版]" : "");
		else
			printf("    未知の版 (起動アドレスは、このツールでは分かりません)\n");
	}
	if (list_only) {
		ret = 0;
		goto out;
	}
	if (pick < 0) {
		for (i = 0; i < n; i++) {
			if (list[i].known && !list[i].known->legacy) {
				pick = i;
				break;
			}
		}
		if (pick < 0) {
			fprintf(stderr, "既知で古い版でない firmware がありません。-n で番号を指定してください\n");
			goto out;
		}
	}
	if (pick >= n) {
		fprintf(stderr, "-n %d は候補の範囲外です (0-%d)\n", pick, n - 1);
		goto out;
	}
	fp = fopen(output, "wb");
	if (!fp || fwrite(list[pick].image, 1, FW_SIZE, fp) != FW_SIZE) {
		fprintf(stderr, "%s へ書き出せません\n", output);
		if (fp)
			fclose(fp);
		goto out;
	}
	/* 書き込みの遅延エラーは fclose で分かる */
	if (fclose(fp) != 0) {
		fprintf(stderr, "%s へ書き出せません (close に失敗)\n", output);
		goto out;
	}
	printf("[%d] を %s へ書き出しました (%d バイト)\n", pick, output, FW_SIZE);
	if (list[pick].known)
		printf("起動アドレス (wIndex): 0x%04x\n", list[pick].known->index);
	ret = 0;
out:
	free(file);
	return ret;
}
