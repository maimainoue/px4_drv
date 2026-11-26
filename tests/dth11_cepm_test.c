/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DT-H11/U2 のリンク暗号 (driver/common/dth11/dth11_cepm) の自動試験。
 *
 * 機器側 (証明書、一時鍵、署名、セッション、コンテンツ鍵のブロック、暗号化した TS) を、同じ曲線演算の
 * 部品で模擬し、ホスト側の手順と復号器を通す。確認するのは、相互認証の 6 段が互いに検証し合うこと、
 * STALL の間の再試行と絶対期限、機器が不正な応答 (証明書、署名、乱数の折り返し、短い応答) を返したときの
 * 拒否、先に #1 の経路で認証すること、鍵パケットの処理、復号、鍵が無いときの扱い、新しい認証の要求の
 * 間隔、ハンドシェイクの差し替えで状態が全て破棄されること。
 * 実機の代用ではない (実機では tests の外で、機器の応答と純正のストリームで確認する)。
 *
 * build: gcc -Wall -Wextra -I driver/common/dth11 tests/dth11_cepm_test.c \
 *            driver/common/dth11/dth11_cepm.c driver/common/dth11/dth11_appkeys.c \
 *            driver/common/dth11/dth11_ec.c driver/common/dth11/dth11_crypto.c \
 *            driver/common/dth11/dth11_core.c -o dth11_cepm_test
 */
#include "dth11_cepm.h"

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

static u32 lcg_state = 777;

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

/* 機器の模擬。要求の bRequest の bit7 が 1 の経路 (#1) と 0 の経路 (#0) は同じ機器が応える */
struct dev {
	u8 dev_priv[20];
	u8 dev_cert[104];
	u8 dev_nonce[16];
	u8 eph_priv[20], eph_pub[40];
	u8 host_nonce[16], host_cert[104];
	u8 shared[20];
	u8 session_id[16];
	u8 m5[52];
	/* 試験の操作 */
	int stall_remaining; /* 2 番目の応答 (step 2) をあと何回 STALL で返すか */
	int stall_errno;
	bool tamper_cert;
	bool tamper_m3_sig;
	bool tamper_m3_nonce;
	bool short_step3;
	bool stall_all_in;
	/* 記録 */
	u8 reqs[64];
	u16 steps[64];
	int nreq;
	int in_attempts;
	bool host_cert_ok, host_sig_ok, step6_ok;
	/* 時計 */
	u64 now;
	u64 slept;
};

static void dev_init(struct dev *d)
{
	u8 pub[40], sig[40];

	memset(d, 0, sizeof(*d));
	d->stall_errno = EPIPE;
	CHECK(dth11_ec_random_scalar(test_rng, NULL, d->dev_priv) == 0);
	CHECK(dth11_ec_public_key(d->dev_priv, pub) == 0);
	memcpy(d->dev_cert, "\x11\x10\xff\xff\x04\x11\x01\xda\x04\x11\x01\xda", 12);
	memcpy(d->dev_cert + 24, pub, 40);
	CHECK(dth11_ec_sign(d->dev_priv, d->dev_cert, 64, test_rng, NULL, sig) == 0);
	memcpy(d->dev_cert + 64, sig, 40);
	test_rng(NULL, d->dev_nonce, 16);
	CHECK(dth11_ec_random_scalar(test_rng, NULL, d->eph_priv) == 0);
	CHECK(dth11_ec_public_key(d->eph_priv, d->eph_pub) == 0);
	test_rng(NULL, d->session_id, 16);
	memset(d->m5, 0xee, sizeof(d->m5));
	memcpy(d->m5 + 4, d->session_id, 16);
	memset(d->m5 + 20, 0xff, 8);
}

static int dev_control(void *priv, u8 type, u8 req, u16 value, u16 index, void *buf, u16 len)
{
	struct dev *d = priv;
	u8 *b = buf;

	if (value != 0xdeaf)
		return -EINVAL;
	if (d->nreq < 64) {
		d->reqs[d->nreq] = req;
		d->steps[d->nreq] = index;
		d->nreq++;
	}
	if (type == 0x40) {
		if (index == 1 && len == 120) {
			memcpy(d->host_nonce, b, 16);
			memcpy(d->host_cert, b + 16, 104);
			d->host_cert_ok = dth11_ec_verify(d->host_cert + 24, d->host_cert, 64, d->host_cert + 64);
		} else if (index == 4 && len == 104) {
			d->host_sig_ok = dth11_ec_verify(d->host_cert + 24, b, 64, b + 64) &&
					 !memcmp(b + 48, d->dev_nonce, 16);
			if (dth11_ec_ecdh(d->eph_priv, b, d->shared))
				d->host_sig_ok = false;
		} else if (index == 6 && len == 52) {
			d->step6_ok = !memcmp(b, d->m5, 52);
		} else {
			return -EINVAL;
		}
		return len;
	}
	/* IN */
	d->in_attempts++;
	if (d->stall_all_in)
		return -d->stall_errno;
	if (index == 2) {
		if (d->stall_remaining > 0) {
			d->stall_remaining--;
			return -d->stall_errno;
		}
		if (len != 120)
			return -EINVAL;
		memcpy(b, d->dev_nonce, 16);
		memcpy(b + 16, d->dev_cert, 104);
		if (d->tamper_cert)
			b[16 + 30] ^= 0x01;
		return 120;
	}
	if (index == 3) {
		u8 m3[104];
		u8 sig[40];

		memcpy(m3, d->eph_pub, 40);
		memset(m3 + 40, 0xff, 8);
		memcpy(m3 + 48, d->host_nonce, 16);
		if (d->tamper_m3_nonce)
			m3[50] ^= 1;
		CHECK(dth11_ec_sign(d->dev_priv, m3, 64, test_rng, NULL, sig) == 0);
		memcpy(m3 + 64, sig, 40);
		if (d->tamper_m3_sig)
			m3[80] ^= 1;
		if (d->short_step3) {
			memcpy(b, m3, 100);
			return 100;
		}
		memcpy(b, m3, 104);
		return 104;
	}
	if (index == 5) {
		memcpy(b, d->m5, 52);
		return 52;
	}
	return -EINVAL;
}

static void dev_sleep(void *priv, unsigned int ms)
{
	struct dev *d = priv;

	d->now += ms;
	d->slept += ms;
}

static u64 dev_now(void *priv)
{
	return ((struct dev *)priv)->now;
}

static const struct dth11_ops dev_ops = { .control = dev_control, .msleep = dev_sleep, .now_ms = dev_now, .log = NULL };

/* アプリケーションの資格情報 (合成): 自己署名の証明書と、対応する秘密鍵、IV の候補 */
static void make_keys(struct dth11_appkeys *k, const u8 iv[16])
{
	u8 q[40], sig[40];

	memset(k, 0, sizeof(*k));
	CHECK(dth11_ec_random_scalar(test_rng, NULL, k->priv_key) == 0);
	CHECK(dth11_ec_public_key(k->priv_key, q) == 0);
	memcpy(k->cert, "APPCERT-HEADER-0123456789", 24);
	memcpy(k->cert + 24, q, 40);
	CHECK(dth11_ec_sign(k->priv_key, k->cert, 64, test_rng, NULL, sig) == 0);
	memcpy(k->cert + 64, sig, 40);
	k->iv_candidates = calloc(2, 16);
	test_rng(NULL, k->iv_candidates[0], 16);
	memcpy(k->iv_candidates[1], iv, 16);
	k->iv_candidate_count = 2;
}

static void test_handshake_ok(void)
{
	struct dev d;
	struct dth11_bridge br;
	struct dth11_appkeys keys;
	struct dth11_handshake hs;
	u8 iv[16] = { 1 };
	u8 ser[DTH11_HANDSHAKE_SERIALIZED_LEN];
	struct dth11_handshake back;
	int i;

	dev_init(&d);
	make_keys(&keys, iv);
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == 0);
	CHECK(d.host_cert_ok && d.host_sig_ok && d.step6_ok);
	/* 両側の共有秘密が一致し、セッション ID は 5 番目の応答の 4..20 */
	CHECK(!memcmp(hs.shared, d.shared, 20));
	CHECK(!memcmp(hs.session_id, d.session_id, 16));
	CHECK(!memcmp(hs.dev_cert, d.dev_cert, 104) && !memcmp(hs.dev_nonce, d.dev_nonce, 16));
	CHECK(!memcmp(hs.nonce, d.host_nonce, 16));
	/* 段の順序: 1 (OUT)、2 3 (IN)、4 (OUT)、5 (IN)、6 (OUT)。#0 の経路なので bRequest の bit7 は立たない */
	CHECK(d.nreq == 6);
	{
		static const u16 steps[6] = { 1, 2, 3, 4, 5, 6 };
		static const u8 reqs[6] = { 0x02, 0x01, 0x01, 0x02, 0x01, 0x02 };

		for (i = 0; i < 6; i++)
			CHECK(d.steps[i] == steps[i] && d.reqs[i] == reqs[i]);
	}
	/* 各 IN の前に 400 ms 待つ (3 回) */
	CHECK(d.slept == 3 * 400);

	dth11_handshake_to_bytes(&hs, ser);
	dth11_handshake_from_bytes(&back, ser);
	CHECK(!memcmp(&back, &hs, sizeof(hs)));
	dth11_appkeys_free(&keys);
}

static void test_handshake_stall_retry(void)
{
	struct dev d;
	struct dth11_bridge br;
	struct dth11_appkeys keys;
	struct dth11_handshake hs;
	u8 iv[16] = { 1 };

	/* STALL の間は 50 ms 間隔で再試行し、期限内に応答が出れば成功する */
	dev_init(&d);
	d.stall_remaining = 5;
	make_keys(&keys, iv);
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == 0);
	CHECK(d.slept == 3 * 400 + 5 * 50);

	/* 期限 (4 秒) を過ぎても STALL なら、最後のエラーを返す */
	dev_init(&d);
	d.stall_all_in = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EPIPE);
	CHECK(d.now >= 400 + 4000 && d.now <= 400 + 4000 + 50);
	CHECK(d.in_attempts > 10 && d.in_attempts < 200);

	/* 機器が抜かれたことを示すエラーは再試行しない */
	dev_init(&d);
	d.stall_all_in = true;
	d.stall_errno = ENODEV;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -ENODEV);
	CHECK(d.in_attempts == 1);
	dth11_appkeys_free(&keys);
}

static void test_handshake_rejects(void)
{
	struct dev d;
	struct dth11_bridge br;
	struct dth11_appkeys keys;
	struct dth11_handshake hs;
	u8 iv[16] = { 1 };

	make_keys(&keys, iv);

	dev_init(&d);
	d.tamper_cert = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EBADMSG);

	dev_init(&d);
	d.tamper_m3_sig = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EBADMSG);

	/* ホストの乱数が折り返されていない応答 (再生攻撃) は、署名が正しくても拒否する */
	dev_init(&d);
	d.tamper_m3_nonce = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EBADMSG);

	dev_init(&d);
	d.short_step3 = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EIO);

	/* 乱数の失敗はそのまま返し、機器へ何も送らない */
	dev_init(&d);
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake(&br, DTH11_CEPM_PATH0, &keys, failing_rng, NULL, &hs) == -EIO);
	CHECK(d.nreq == 0);
	dth11_appkeys_free(&keys);
}

static void test_handshake_for_path(void)
{
	struct dev d;
	struct dth11_bridge br;
	struct dth11_appkeys keys;
	struct dth11_handshake hs;
	u8 iv[16] = { 1 };

	make_keys(&keys, iv);
	dev_init(&d);
	dth11_bridge_init(&br, &dev_ops, &d);
	/* #0 の経路: 先に #1 の経路 (bRequest の bit7) で 1 回、その後 #0 の経路で 1 回。計 12 要求 */
	CHECK(dth11_cepm_handshake_for_path(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == 0);
	CHECK(d.nreq == 12);
	CHECK(d.reqs[0] == 0x82 && d.steps[0] == 1 && d.reqs[5] == 0x82 && d.steps[5] == 6);
	CHECK(d.reqs[6] == 0x02 && d.steps[6] == 1 && d.reqs[11] == 0x02 && d.steps[11] == 6);

	/* #1 の経路の指定は 1 回だけ */
	dev_init(&d);
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake_for_path(&br, DTH11_CEPM_PATH1, &keys, test_rng, NULL, &hs) == 0);
	CHECK(d.nreq == 6 && d.reqs[0] == 0x82);

	/* #1 の失敗は無視して #0 を続ける (#1 は、機器の証明書が壊れているので失敗し、#0 でも同じく失敗する) */
	dev_init(&d);
	d.tamper_cert = true;
	dth11_bridge_init(&br, &dev_ops, &d);
	CHECK(dth11_cepm_handshake_for_path(&br, DTH11_CEPM_PATH0, &keys, test_rng, NULL, &hs) == -EBADMSG);
	/* どちらの経路も、1 (OUT) と 2 (IN) の後、証明書の検証で止まる */
	CHECK(d.reqs[0] == 0x82 && d.steps[0] == 1 && d.reqs[1] == 0x81 && d.steps[1] == 2);
	CHECK(d.nreq == 4 && d.reqs[2] == 0x02 && d.steps[2] == 1 && d.reqs[3] == 0x01 && d.steps[3] == 2);
	dth11_appkeys_free(&keys);
}

/* ---- 復号器 ---- */

struct stream_ctx {
	u8 k3[16];
	u8 content_key[16];
	u8 cmac_key[16];
	u8 iv[16];
	u8 session_id[16];
};

static void put_header(u8 *p, u16 pid, u8 b3)
{
	memset(p, 0, 188);
	p[0] = 0x47;
	p[1] = (u8)((pid >> 8) & 0x1f);
	p[2] = (u8)pid;
	p[3] = b3;
}

static void make_session_packet(u8 *p, const u8 id[16])
{
	put_header(p, 0x1fc8, 0x20);
	p[4] = 0xb7;
	p[5] = 0x02;
	p[6] = 0xb5;
	p[7] = 0xec;
	memcpy(p + 24, id, 16);
	memset(p + 40, 0x11, 20);
}

/* コンテンツ鍵のブロック (id 16、CBC(IV) で包んだ [CMAC 鍵 | コンテンツ鍵] 32、CMAC 16) を持つ鍵パケット */
static void make_content_packet(u8 *p, u16 pid, const struct stream_ctx *s, const u8 content_id[16], u32 dw1, u32 dw2)
{
	struct dth11_aes a;
	u8 d[32], prev[16], blob[64], mac[16];
	int i, c;

	put_header(p, pid, 0x20);
	p[4] = 0xb7;
	p[5] = 0x02;
	p[6] = 0xb5;
	p[7] = 0xc0;
	memcpy(blob, content_id, 16);
	memcpy(d, s->cmac_key, 16);
	memcpy(d + 16, s->content_key, 16);
	dth11_aes_set_key(&a, s->k3);
	memcpy(prev, s->iv, 16);
	for (i = 0; i < 32; i += 16) {
		u8 x[16];

		for (c = 0; c < 16; c++)
			x[c] = (u8)(d[i + c] ^ prev[c]);
		dth11_aes_encrypt_block(&a, x, blob + 16 + i);
		memcpy(prev, blob + 16 + i, 16);
	}
	dth11_aes_cmac(s->cmac_key, blob, 48, mac);
	memcpy(blob + 48, mac, 16);
	memcpy(p + 24, blob, 64);
	p[88] = (u8)dw1;
	p[89] = (u8)(dw1 >> 8);
	p[90] = (u8)(dw1 >> 16);
	p[91] = (u8)(dw1 >> 24);
	p[92] = (u8)dw2;
	p[93] = (u8)(dw2 >> 8);
	p[94] = (u8)(dw2 >> 16);
	p[95] = (u8)(dw2 >> 24);
}

static void make_scrambled(u8 *p, u16 pid, const u8 *plain184, struct dth11_ctr *enc)
{
	put_header(p, pid, 0x90); /* transport_scrambling_control = 10、payload のみ */
	memcpy(p + 4, plain184, 184);
	dth11_ctr_apply(enc, p + 4, 184);
}

static void setup_stream(struct stream_ctx *s, const struct dth11_handshake *hs)
{
	u8 k[20];

	dth11_cepm_kdf(hs->shared, 3, k);
	memcpy(s->k3, k, 16);
	test_rng(NULL, s->content_key, 16);
	test_rng(NULL, s->cmac_key, 16);
	test_rng(NULL, s->iv, 16);
	memcpy(s->session_id, hs->session_id, 16);
}

static void test_kdf_and_iv(void)
{
	u8 secret[20], out[20], expect[20], iv[16];
	u8 buf[36] = { 0 };

	memset(secret, 0x5a, 20);
	dth11_cepm_kdf(secret, 3, out);
	memcpy(buf, secret, 20);
	buf[35] = 4;
	dth11_sha1(buf, 36, expect);
	CHECK(!memcmp(out, expect, 20));
	/* カウンターが違えば別の値 */
	dth11_cepm_kdf(secret, 4, expect);
	CHECK(memcmp(out, expect, 20) != 0);

	/* (dw2 & 0xF0000FFF) | pid << 12 を BE32 で、dw1 を BE32 で。残りは 0 */
	dth11_cepm_ctr_iv(0x0100, 0x01020304, 0xabcdef12, iv);
	/* (0xabcdef12 & 0xF0000FFF) = 0xA0000F12、pid << 12 = 0x00100000 */
	CHECK(iv[0] == 0xa0 && iv[1] == 0x10 && iv[2] == 0x0f && iv[3] == 0x12);
	CHECK(iv[4] == 0x01 && iv[5] == 0x02 && iv[6] == 0x03 && iv[7] == 0x04);
	CHECK(iv[8] == 0 && iv[15] == 0);
}

static void test_decryptor_flow(void)
{
	struct dth11_handshake hs;
	struct dth11_decryptor dec;
	struct dth11_appkeys keys;
	struct stream_ctx s;
	u8 pkt[188], plain[184], copy[188];
	u8 content_id[16], civ[16];
	struct dth11_ctr enc;
	int i;
	u64 t = 1000;

	memset(&hs, 0, sizeof(hs));
	test_rng(NULL, hs.shared, 20);
	test_rng(NULL, hs.session_id, 16);
	setup_stream(&s, &hs);
	make_keys(&keys, s.iv);
	dth11_dec_init(&dec, &hs, &keys);
	test_rng(NULL, content_id, 16);
	for (i = 0; i < 184; i++)
		plain[i] = (u8)(i * 7 + 3);

	/* 鍵が無いうちの暗号化パケットは捨てる */
	{
		struct dth11_ctr e0;

		dth11_cepm_ctr_iv(0x0100, 5, 0x40000000, civ);
		dth11_ctr_init(&e0, s.content_key, civ);
		make_scrambled(pkt, 0x0100, plain, &e0);
		CHECK(dth11_dec_process(&dec, pkt, t) == 0);
		CHECK(dec.stats.dropped_no_key == 1);
	}
	/* 平文と PID 0x1FFF はそのまま出力 */
	put_header(pkt, 0x0111, 0x10);
	CHECK(dth11_dec_process(&dec, pkt, t) == 1);
	put_header(pkt, 0x1fff, 0x10);
	CHECK(dth11_dec_process(&dec, pkt, t) == 1);
	CHECK(dec.stats.clear == 2);

	/* セッションが無い間のコンテンツ鍵のブロックは消費して捨てる (鍵なしの計数) */
	make_content_packet(pkt, 0x0100, &s, content_id, 5, 0x40000000);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.stats.dropped_no_key == 2 && !dec.has_content_key);

	/* セッションの通知 (ハンドシェイクの ID): 共有秘密がセッションの秘密になる */
	make_session_packet(pkt, hs.session_id);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.has_session && !memcmp(dec.session_secret, hs.shared, 20));
	CHECK(dec.stats.session_unwraps == 1);
	/* 同じ通知が繰り返されても何も起きない */
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.stats.session_unwraps == 1);

	/* コンテンツ鍵のブロックで、PID 0x100 の文脈ができる */
	make_content_packet(pkt, 0x0100, &s, content_id, 5, 0x40000000);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.has_content_key && dec.stats.content_unwraps == 1 && keys.iv_settled);
	CHECK(!memcmp(dec.content_key, s.content_key, 16));

	/* 暗号化された TS パケットが復号され、スクランブル制御のビットが落ちる */
	dth11_cepm_ctr_iv(0x0100, 5, 0x40000000, civ);
	dth11_ctr_init(&enc, s.content_key, civ);
	for (i = 0; i < 3; i++) {
		/* 鍵ストリームは、パケットをまたいで続く */
		make_scrambled(pkt, 0x0100, plain, &enc);
		memcpy(copy, pkt, 188);
		CHECK(dth11_dec_process(&dec, pkt, t) == 1);
		CHECK(!memcmp(pkt + 4, plain, 184));
		CHECK((pkt[3] & 0xc0) == 0 && pkt[3] == (copy[3] & 0x3f));
		CHECK(!memcmp(pkt, copy, 3));
	}
	CHECK(dec.stats.decrypted == 3);

	/* 文脈の無い PID は共通の文脈 (PID 0) を探す。無ければ捨てる */
	{
		struct dth11_ctr e2;

		dth11_cepm_ctr_iv(0x0200, 0, 0, civ);
		dth11_ctr_init(&e2, s.content_key, civ);
		make_scrambled(pkt, 0x0200, plain, &e2);
		CHECK(dth11_dec_process(&dec, pkt, t) == 0);
		CHECK(dec.stats.dropped_no_key == 3);
	}
	/* PID 0x1FC8 の鍵パケットは、共通の文脈 (PID 0) に入る */
	make_content_packet(pkt, 0x1fc8, &s, content_id, 9, 0x20000000);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	{
		struct dth11_ctr e3;

		dth11_cepm_ctr_iv(0, 9, 0x20000000, civ);
		dth11_ctr_init(&e3, s.content_key, civ);
		make_scrambled(pkt, 0x0200, plain, &e3);
		CHECK(dth11_dec_process(&dec, pkt, t) == 1);
		CHECK(!memcmp(pkt + 4, plain, 184));
	}
	/* 鍵ブロックの CMAC が合わないとき: 文脈は作らず、エラーを数える */
	{
		u8 other_id[16];
		u64 errors = dec.stats.errors;

		test_rng(NULL, other_id, 16);
		make_content_packet(pkt, 0x0100, &s, other_id, 1, 1);
		pkt[24 + 60] ^= 1;
		CHECK(dth11_dec_process(&dec, pkt, t) == 0);
		CHECK(dec.stats.errors == errors + 1);
		/* 以前のコンテンツ鍵は残る */
		CHECK(!memcmp(dec.content_id, content_id, 16));
	}
	dth11_appkeys_free(&keys);
}

static void test_rehandshake_request(void)
{
	struct dth11_handshake hs, hs2;
	struct dth11_decryptor dec;
	struct dth11_appkeys keys;
	struct stream_ctx s;
	u8 pkt[188], foreign[16], foreign2[16], content_id[16];
	u64 t = 0;

	memset(&hs, 0, sizeof(hs));
	test_rng(NULL, hs.shared, 20);
	test_rng(NULL, hs.session_id, 16);
	setup_stream(&s, &hs);
	make_keys(&keys, s.iv);
	dth11_dec_init(&dec, &hs, &keys);
	test_rng(NULL, foreign, 16);
	test_rng(NULL, foreign2, 16);
	test_rng(NULL, content_id, 16);

	/* 自分のものでない通知は 1 回目では要求しない。2 回続くと要求する */
	make_session_packet(pkt, foreign);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(!dth11_dec_take_rehandshake_request(&dec));
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dth11_dec_take_rehandshake_request(&dec));
	/* 要求は 1 回だけ返す */
	CHECK(!dth11_dec_take_rehandshake_request(&dec));

	/* 最小の間隔 (10 秒) の間は、続けて通知されても再要求しない */
	make_session_packet(pkt, foreign2);
	t = 9999;
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(!dth11_dec_take_rehandshake_request(&dec));
	t = 10000;
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dth11_dec_take_rehandshake_request(&dec));

	/* 自分のセッションの通知は、外来の数を戻す */
	make_session_packet(pkt, hs.session_id);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.has_session && dec.foreign_announcements == 0);
	make_content_packet(pkt, 0x0100, &s, content_id, 1, 1);
	CHECK(dth11_dec_process(&dec, pkt, t) == 0);
	CHECK(dec.has_content_key);

	/* 新しいハンドシェイクを入れると、セッション、コンテンツ鍵、文脈、要求が全て破棄される */
	memset(&hs2, 0, sizeof(hs2));
	test_rng(NULL, hs2.shared, 20);
	test_rng(NULL, hs2.session_id, 16);
	dec.rehandshake_wanted = true;
	dth11_dec_install_handshake(&dec, &hs2);
	CHECK(!dec.has_session && !dec.has_content_key && !dec.rehandshake_wanted);
	CHECK(!dec.ctx[0].used && dec.foreign_announcements == 0);
	CHECK(!memcmp(dec.hs.session_id, hs2.session_id, 16));
	/* 古いセッションの通知は、今度は外来になる */
	make_session_packet(pkt, hs.session_id);
	CHECK(dth11_dec_process(&dec, pkt, t + 1) == 0);
	CHECK(!dec.has_session && dec.foreign_announcements == 1);
	dth11_appkeys_free(&keys);
}

static void test_key_packet_detection(void)
{
	u8 p[188];

	make_session_packet(p, p);
	CHECK(dth11_is_key_packet(p));
	p[7] = 0xc5;
	CHECK(dth11_is_key_packet(p));
	p[7] = 0xd0;
	CHECK(!dth11_is_key_packet(p));
	p[7] = 0xec;
	p[3] = 0x30; /* アダプテーションと payload の両方 */
	CHECK(!dth11_is_key_packet(p));
	p[3] = 0x20;
	p[5] = 0x03;
	CHECK(!dth11_is_key_packet(p));
	p[5] = 0x02;
	p[0] = 0x46;
	CHECK(!dth11_is_key_packet(p));
}

int main(void)
{
	test_kdf_and_iv();
	test_key_packet_detection();
	test_handshake_ok();
	test_handshake_stall_retry();
	test_handshake_rejects();
	test_handshake_for_path();
	test_decryptor_flow();
	test_rehandshake_request();
	printf("%d checks, %d failures\n", checks, fails);
	return fails ? 1 : 0;
}
