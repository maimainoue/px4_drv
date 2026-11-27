// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_cepm.c - DT-H11/U2 のリンク暗号 (CEPM) (dth11_cepm.h)
 */

#include "dth11_cepm.h"

/*
 * 相互認証の応答を待つ時間。機器のブリッジ MCU が ECDSA を計算する間、IN は STALL で返る。
 * 1 回の IN の待機は、操作全体の絶対期限で打ち切る。
 */
#define RESPONSE_WAIT_MS 4000
#define RETRY_INTERVAL_MS 50
/*
 * 各ステップの IN の前の待ち。前のステップを機器が処理し終える前に問い合わせると、ブリッジは
 * 相互認証を捨てる (ECDSA の検証中)。
 */
#define STEP_DELAY_MS 400

#define LOG(br, lvl, ...) \
	do { \
		if ((br)->ops->log) \
			(br)->ops->log((br)->priv, (lvl), __VA_ARGS__); \
	} while (0)

/* ---- 鍵の導出と IV ---- */

void dth11_cepm_kdf(const u8 secret[20], u8 counter, u8 out[20])
{
	u8 buf[36] = { 0 };

	memcpy(buf, secret, 20);
	buf[35] = (u8)(counter + 1);
	dth11_sha1(buf, sizeof(buf), out);
}

void dth11_cepm_ctr_iv(u16 pid, u32 dw1, u32 dw2, u8 iv[16])
{
	u32 w = (dw2 & 0xf0000fff) | ((u32)pid << 12);

	memset(iv, 0, 16);
	iv[0] = (u8)(w >> 24);
	iv[1] = (u8)(w >> 16);
	iv[2] = (u8)(w >> 8);
	iv[3] = (u8)w;
	iv[4] = (u8)(dw1 >> 24);
	iv[5] = (u8)(dw1 >> 16);
	iv[6] = (u8)(dw1 >> 8);
	iv[7] = (u8)dw1;
}

/* ---- 相互認証 ---- */

/* 一時的な失敗 (STALL、タイムアウトなど) か。機器が抜かれたことを示すエラーは再試行しない */
static bool retryable(int err)
{
	return err == -EPIPE || err == -EIO || err == -ETIMEDOUT || err == -EAGAIN;
}

/*
 * IN の段を、STALL の間は絶対期限まで繰り返す。最初の試行の前に STEP_DELAY_MS 待つ。
 * 期限を過ぎたら最後のエラーを返す。
 */
static int cepm_in(struct dth11_bridge *br, u8 flags, u16 step, void *buf, u16 len)
{
	u64 deadline;
	int r;

	br->ops->msleep(br->priv, STEP_DELAY_MS);
	deadline = br->ops->now_ms(br->priv) + RESPONSE_WAIT_MS;
	for (;;) {
		r = dth11_vendor_in(br, flags, DTH11_CEPM_VALUE, step, buf, len);
		if (r >= 0 || !retryable(r) || br->ops->now_ms(br->priv) >= deadline)
			return r;
		br->ops->msleep(br->priv, RETRY_INTERVAL_MS);
	}
}

int dth11_cepm_handshake(struct dth11_bridge *br, u8 flags, const struct dth11_appkeys *keys, dth11_rng_fn rng,
			 void *rng_priv, struct dth11_handshake *hs)
{
	struct dth11_handshake h;
	u8 m1[120], m2[120], m3[104], m4[104], m5[52];
	u8 d_e[DTH11_EC_BYTES], q_e[DTH11_EC_POINT_BYTES];
	int r;

	memset(&h, 0, sizeof(h));

	/* 1: ホストの乱数 16 と、アプリケーションの証明書 104 */
	r = rng(rng_priv, h.nonce, sizeof(h.nonce));
	if (r)
		return r;
	memcpy(m1, h.nonce, 16);
	memcpy(m1 + 16, keys->cert, 104);
	r = dth11_vendor_out(br, flags, DTH11_CEPM_VALUE, 1, m1, sizeof(m1));
	if (r)
		return r;

	/* 2: 機器の乱数 16 と、機器の証明書 104。証明書の自己署名を確かめる */
	r = cepm_in(br, flags, 2, m2, sizeof(m2));
	if (r < 0)
		return r;
	if (r < (int)sizeof(m2))
		return -EIO;
	memcpy(h.dev_nonce, m2, 16);
	memcpy(h.dev_cert, m2 + 16, 104);
	if (dth11_ec_point_check(h.dev_cert + 24))
		return -EBADMSG;
	if (!dth11_ec_verify(h.dev_cert + 24, h.dev_cert, 64, h.dev_cert + 64))
		return -EBADMSG;

	/* 3: 機器の一時鍵 40、FF x 8、ホストの乱数 16 (折り返し)、機器の証明書の鍵による署名 40 */
	r = cepm_in(br, flags, 3, m3, sizeof(m3));
	if (r < 0)
		return r;
	if (r < (int)sizeof(m3))
		return -EIO;
	if (!dth11_ec_verify(h.dev_cert + 24, m3, 64, m3 + 64))
		return -EBADMSG;
	if (memcmp(m3 + 48, h.nonce, 16))
		return -EBADMSG;
	memcpy(h.dev_ephemeral, m3, 40);
	if (dth11_ec_point_check(h.dev_ephemeral))
		return -EBADMSG;

	/* 4: ホストの一時鍵 40、FF x 8、機器の乱数 16、アプリケーションの鍵による署名 40 */
	r = dth11_ec_random_scalar(rng, rng_priv, d_e);
	if (r)
		return r;
	r = dth11_ec_public_key(d_e, q_e);
	if (r)
		return r;
	memcpy(m4, q_e, 40);
	memset(m4 + 40, 0xff, 8);
	memcpy(m4 + 48, h.dev_nonce, 16);
	r = dth11_ec_sign(keys->priv_key, m4, 64, rng, rng_priv, m4 + 64);
	if (r)
		return r;
	r = dth11_vendor_out(br, flags, DTH11_CEPM_VALUE, 4, m4, sizeof(m4));
	if (r)
		return r;
	r = dth11_ec_ecdh(d_e, h.dev_ephemeral, h.shared);
	if (r)
		return -EBADMSG;

	/* 5 と 6: セッションの識別 (4..20) を受け取り、そのまま返す */
	r = cepm_in(br, flags, 5, m5, sizeof(m5));
	if (r < 0)
		return r;
	if (r < (int)sizeof(m5))
		return -EIO;
	memcpy(h.m5, m5, sizeof(m5));
	memcpy(h.session_id, m5 + 4, 16);
	r = dth11_vendor_out(br, flags, DTH11_CEPM_VALUE, 6, m5, sizeof(m5));
	if (r)
		return r;

	*hs = h;
	return 0;
}

int dth11_cepm_handshake_for_path(struct dth11_bridge *br, u8 flags, const struct dth11_appkeys *keys,
				  dth11_rng_fn rng, void *rng_priv, struct dth11_handshake *hs)
{
	if (!(flags & DTH11_CEPM_PATH1)) {
		struct dth11_handshake ignored;
		int r = dth11_cepm_handshake(br, DTH11_CEPM_PATH1, keys, rng, rng_priv, &ignored);

		if (r)
			LOG(br, DTH11_LOG_DEBUG, "cepm: #1 の経路の先行の相互認証に失敗 (%d)、#0 の経路で続ける", r);
	}
	return dth11_cepm_handshake(br, flags, keys, rng, rng_priv, hs);
}

void dth11_handshake_to_bytes(const struct dth11_handshake *hs, u8 out[DTH11_HANDSHAKE_SERIALIZED_LEN])
{
	size_t o = 0;

	memcpy(out + o, hs->dev_cert, 104);
	o += 104;
	memcpy(out + o, hs->dev_nonce, 16);
	o += 16;
	memcpy(out + o, hs->nonce, 16);
	o += 16;
	memcpy(out + o, hs->dev_ephemeral, 40);
	o += 40;
	memcpy(out + o, hs->shared, 20);
	o += 20;
	memcpy(out + o, hs->m5, 52);
	o += 52;
	memcpy(out + o, hs->session_id, 16);
}

void dth11_handshake_from_bytes(struct dth11_handshake *hs, const u8 in[DTH11_HANDSHAKE_SERIALIZED_LEN])
{
	size_t o = 0;

	memcpy(hs->dev_cert, in + o, 104);
	o += 104;
	memcpy(hs->dev_nonce, in + o, 16);
	o += 16;
	memcpy(hs->nonce, in + o, 16);
	o += 16;
	memcpy(hs->dev_ephemeral, in + o, 40);
	o += 40;
	memcpy(hs->shared, in + o, 20);
	o += 20;
	memcpy(hs->m5, in + o, 52);
	o += 52;
	memcpy(hs->session_id, in + o, 16);
}

/* ---- TS の復号 ---- */

#define KEYPKT_SESSION 0xec
/* 通知されたセッションが自分のものでないことを、この回数続けて見たら新しいハンドシェイクを要求する */
#define FOREIGN_ANNOUNCEMENTS_BEFORE_REHANDSHAKE 2
/* ストリームが引き起こす相互認証の最小の間隔 */
#define REHANDSHAKE_MIN_INTERVAL_MS 10000

static void dec_reset_session(struct dth11_decryptor *dec)
{
	dec->has_session = false;
	memset(dec->session_id, 0, sizeof(dec->session_id));
	memset(dec->session_secret, 0, sizeof(dec->session_secret));
	dec->has_content_key = false;
	memset(dec->content_id, 0, sizeof(dec->content_id));
	memset(dec->content_key, 0, sizeof(dec->content_key));
	memset(dec->ctx, 0, sizeof(dec->ctx));
	dec->ctx_next = 0;
}

void dth11_dec_init(struct dth11_decryptor *dec, const struct dth11_handshake *hs, struct dth11_appkeys *keys)
{
	memset(dec, 0, sizeof(*dec));
	dec->hs = *hs;
	dec->keys = keys;
}

void dth11_dec_install_handshake(struct dth11_decryptor *dec, const struct dth11_handshake *hs)
{
	dec->hs = *hs;
	dec->foreign_announcements = 0;
	dec->rehandshake_wanted = false;
	dec_reset_session(dec);
}

bool dth11_dec_take_rehandshake_request(struct dth11_decryptor *dec)
{
	bool v = dec->rehandshake_wanted;

	dec->rehandshake_wanted = false;
	return v;
}

bool dth11_is_key_packet(const u8 *p)
{
	/*
	 * アダプテーションフィールドだけのパケット (p[3] の bit5:4 が 10) で、私的データが B7 02 B5 の後に
	 * 鍵の型を持つ。純正は 8..24 バイト目の定数の目印も確かめるが、この機器の出力ではこの構造だけで
	 * 他のパケットと区別できる。
	 */
	return p[0] == 0x47 && (p[3] & 0x30) == 0x20 && p[4] == 0xb7 && p[5] == 0x02 && p[6] == 0xb5 &&
	       (p[7] == KEYPKT_SESSION || (p[7] >= 0xc0 && p[7] <= 0xcf));
}

/* PID 0x1FC8..0x1FCF は共通の文脈 (PID 0) を使う */
static u16 context_pid(u16 pid)
{
	return (pid >= 0x1fc8 && pid <= 0x1fcf) ? 0 : pid;
}

static u32 le32(const u8 *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void session_k3(const struct dth11_decryptor *dec, u8 k3[16])
{
	u8 k[20];

	dth11_cepm_kdf(dec->session_secret, 3, k);
	memcpy(k3, k, 16);
}

static void install_ctx(struct dth11_decryptor *dec, u16 pid, const u8 key[16], const u8 iv[16])
{
	u32 i;

	for (i = 0; i < DTH11_DEC_CTX_MAX; i++)
		if (dec->ctx[i].used && dec->ctx[i].pid == pid)
			goto set;
	for (i = 0; i < DTH11_DEC_CTX_MAX; i++)
		if (!dec->ctx[i].used)
			goto set;
	/* 満杯のときは古い方から入れ替える (番組の PID は十数個なので、通常は満杯にならない) */
	i = dec->ctx_next++ % DTH11_DEC_CTX_MAX;
set:
	dec->ctx[i].used = true;
	dec->ctx[i].pid = pid;
	dth11_ctr_init(&dec->ctx[i].ctr, key, iv);
}

static struct dth11_ctr *find_ctx(struct dth11_decryptor *dec, u16 pid)
{
	u32 i;

	for (i = 0; i < DTH11_DEC_CTX_MAX; i++)
		if (dec->ctx[i].used && dec->ctx[i].pid == pid)
			return &dec->ctx[i].ctr;
	return NULL;
}

/* セッション通知 (0xEC) の処理。戻り値は常に 0 (パケットは消費する) */
static void handle_session_packet(struct dth11_decryptor *dec, const u8 *p, u64 now_ms)
{
	const u8 *id = p + 24;
	const u8 *ident = p + 40;
	static const u8 zero[16] = { 0 };

	/*
	 * 現在のセッションと同じ ID の通知は処理しない (「セッションなし」は全て 0 として比べるので、
	 * 機器の最初の空の通知も飛ばす)。
	 */
	if (!memcmp(id, dec->has_session ? dec->session_id : zero, 16))
		return;
	if (!memcmp(id, dec->hs.session_id, 16)) {
		/* ハンドシェイクのセッションは、ECDH の共有秘密が秘密になる */
		memcpy(dec->session_id, id, 16);
		memcpy(dec->session_secret, dec->hs.shared, 20);
		dec->has_session = true;
		dec->stats.session_unwraps++;
		dec->foreign_announcements = 0;
		return;
	}
	/*
	 * 自分のものでないセッション: 機器が有効なハンドシェイクを持たないとき (電源を入れ直した後に保存した
	 * ハンドシェイクを使う場合、カードを抜き差しした後)、または自分の識別を持つが別のハンドシェイクに
	 * 結びついている (古い保存) とき。純正は wIndex 7 で開くが、この機器では、現在のセッションが
	 * 持たないブロックはタイムアウトして機器がセッションを捨て、再接続まで 0xDEAF を拒む。新しい
	 * ハンドシェイクで結び直すだけにする。
	 */
	(void)ident;
	dec->foreign_announcements++;
	if (dec->foreign_announcements >= FOREIGN_ANNOUNCEMENTS_BEFORE_REHANDSHAKE &&
	    (!dec->has_last_rehandshake || now_ms - dec->last_rehandshake_ms >= REHANDSHAKE_MIN_INTERVAL_MS)) {
		dec->has_last_rehandshake = true;
		dec->last_rehandshake_ms = now_ms;
		dec->foreign_announcements = 0;
		dec->rehandshake_wanted = true;
	}
}

/* コンテンツ鍵のブロック (0xC0..0xCF)。戻り値は 0 (消費) */
static void handle_content_packet(struct dth11_decryptor *dec, const u8 *p, u16 pid)
{
	const u8 *id = p + 24;
	u8 k3[16], iv[16], d[32], mac[16];
	u32 dw1, dw2;
	u16 cpid;
	u8 civ[16];

	if (!dec->has_session) {
		dec->stats.dropped_no_key++;
		return;
	}
	session_k3(dec, k3);
	if (!dec->has_content_key || memcmp(dec->content_id, id, 16)) {
		if (dth11_appkeys_find_content_iv(dec->keys, k3, p + 24, iv)) {
			/* 取り出した資格情報が、この機器のコンテンツ鍵を開けない (別の版の DLL など) */
			dec->stats.errors++;
			return;
		}
		memcpy(d, p + 24 + 16, 32);
		if (dth11_aes_cbc_decrypt(k3, iv, d, 32)) {
			dec->stats.errors++;
			return;
		}
		/* d[0..16] は CMAC の鍵で、ブロックの先頭 48 バイトの認証に使う。d[16..32] が AES-CTR の鍵 */
		dth11_aes_cmac(d, p + 24, 48, mac);
		if (memcmp(mac, p + 24 + 48, 16)) {
			dec->stats.errors++;
			return;
		}
		dec->stats.content_unwraps++;
		memcpy(dec->content_key, d + 16, 16);
		memcpy(dec->content_id, id, 16);
		dec->has_content_key = true;
		memset(dec->ctx, 0, sizeof(dec->ctx));
		dec->ctx_next = 0;
	}
	dw1 = le32(p + 88);
	dw2 = le32(p + 92);
	cpid = context_pid(pid);
	dth11_cepm_ctr_iv(cpid, dw1, dw2, civ);
	install_ctx(dec, cpid, dec->content_key, civ);
}

int dth11_dec_process(struct dth11_decryptor *dec, u8 *p, u64 now_ms)
{
	u16 pid;

	dec->stats.packets++;
	if (p[0] != 0x47)
		return 1;
	pid = (u16)(((p[1] & 0x1f) << 8) | p[2]);
	if (pid == 0x1fff) {
		dec->stats.clear++;
		return 1;
	}
	if (dth11_is_key_packet(p)) {
		dec->stats.key_packets++;
		if (p[7] == KEYPKT_SESSION)
			handle_session_packet(dec, p, now_ms);
		else
			handle_content_packet(dec, p, pid);
		return 0;
	}
	if ((p[3] & 0xc0) == 0) {
		dec->stats.clear++;
		return 1;
	}
	{
		/* PID 専用の文脈が無ければ、共通の文脈 (PID 0) を使う */
		struct dth11_ctr *c = find_ctx(dec, pid);

		if (!c)
			c = find_ctx(dec, 0);
		if (!c) {
			dec->stats.dropped_no_key++;
			return 0;
		}
		dth11_ctr_apply(c, p + 4, 184);
		p[3] &= 0x3f;
		dec->stats.decrypted++;
		return 1;
	}
}
