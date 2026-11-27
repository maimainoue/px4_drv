/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_cepm.h - DT-H11/U2 のストリームのリンク暗号 (CEPM): 相互認証と TS の復号
 *
 * device モード (ストリーム有効化の byte0 = 0) では、デバイス内蔵のカードで復号された番組の TS が、
 * ホストとの間でだけ暗号化されて届く。この層はその手順を扱う。
 *
 *  - 相互認証 (ECDSA と ECDH、secp160r1): ベンダー要求 wValue 0xDEAF、wIndex 1..6。ホストの証明書と
 *    秘密鍵は dth11_appkeys.h が利用者の CoBDACom.dll から読み出す。共有秘密 (ECDH の x 座標) が
 *    セッションの秘密になる。
 *  - TS の復号: transport_scrambling_control が 0 でないパケットの、ヘッダー以降の 184 バイトが
 *    AES-128-CTR で暗号化されている (PID ごとのカウンター)。鍵はストリームの中に、アダプテーション
 *    フィールドの私的データとして届く (鍵パケット)。0xEC はセッションの通知、0xC0..0xCF は
 *    64 バイトのコンテンツ鍵のブロックで、ホスト側で AES-CBC と AES-CMAC により開く。
 *
 * 手順は i4M1k0SU/DTH10-series-driver (MIT License、Copyright (c) i4M1k0SU) の
 * crates/dth10-core/src/cepm.rs による。復号器は、新しいハンドシェイクの必要を検出しても自分では
 * 実行しない (数秒かかり、TS の受信スレッドを止めてしまうため)。呼び出し側が別のスレッドで
 * dth11_cepm_handshake_for_path() を実行し、dth11_dec_install_handshake() で渡す。
 */
#ifndef DTH11_CEPM_H
#define DTH11_CEPM_H

#include "dth11_appkeys.h"
#include "dth11_crypto.h"
#include "dth11_ec.h"

#define DTH11_CEPM_VALUE 0xdeaf
/* チューナーの経路 (bRequest の bit7)。DT-H11/U2 の TS は #0 の経路に結びつく。#0 は #1 の後で応答する */
#define DTH11_CEPM_PATH0 0x00
#define DTH11_CEPM_PATH1 0x80

#define DTH11_HANDSHAKE_SERIALIZED_LEN (104 + 16 + 16 + 40 + 20 + 52 + 16)

struct dth11_handshake {
	u8 dev_cert[104]; /* 機器の証明書 (ヘッダー 24、公開鍵 40、自己署名 40) */
	u8 dev_nonce[16];
	u8 nonce[16]; /* ホストの乱数 */
	u8 dev_ephemeral[40];
	u8 shared[20]; /* ECDH の共有秘密 (x 座標) */
	u8 m5[52]; /* 5 番目の応答。4..20 がセッションの ID */
	u8 session_id[16];
};

#define DTH11_DEC_CTX_MAX 32

struct dth11_dec_stats {
	u64 packets;
	u64 decrypted;
	u64 clear;
	u64 key_packets;
	u64 session_unwraps;
	u64 content_unwraps;
	u64 dropped_no_key;
	u64 errors;
};

struct dth11_decryptor {
	struct dth11_handshake hs;
	struct dth11_appkeys *keys;
	bool has_session;
	u8 session_id[16];
	u8 session_secret[20];
	bool has_content_key;
	u8 content_id[16];
	u8 content_key[16];
	struct {
		bool used;
		u16 pid;
		struct dth11_ctr ctr;
	} ctx[DTH11_DEC_CTX_MAX];
	u32 ctx_next;
	/* 自分のものでないセッションの通知を数える。数回続いたら新しいハンドシェイクを要求する */
	u32 foreign_announcements;
	bool has_last_rehandshake;
	u64 last_rehandshake_ms;
	bool rehandshake_wanted;
	struct dth11_dec_stats stats;
};

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 相互認証 ---- */

/*
 * チューナーの経路 flags で相互認証 (1..6) を行う。機器の応答は ECDSA の計算を待つため 1 回に約 1 秒かかる。
 * 失敗の種類: 通信 (負の errno)、短い応答 (-EIO)、機器の証明書や署名が不正 (-EBADMSG)、
 * 乱数の失敗 (rng の値)。
 */
int dth11_cepm_handshake(struct dth11_bridge *br, u8 flags, const struct dth11_appkeys *keys, dth11_rng_fn rng,
			 void *rng_priv, struct dth11_handshake *hs);
/*
 * flags が #0 の経路のとき、先に #1 の経路で 1 回行ってから #0 の経路で行う。新しく電源が入った機器は
 * #1 の経路だけが応答し、#0 の経路は #1 の後で応答するため。#1 の失敗は無視する。
 */
int dth11_cepm_handshake_for_path(struct dth11_bridge *br, u8 flags, const struct dth11_appkeys *keys,
				  dth11_rng_fn rng, void *rng_priv, struct dth11_handshake *hs);

void dth11_handshake_to_bytes(const struct dth11_handshake *hs, u8 out[DTH11_HANDSHAKE_SERIALIZED_LEN]);
void dth11_handshake_from_bytes(struct dth11_handshake *hs, const u8 in[DTH11_HANDSHAKE_SERIALIZED_LEN]);

/* KDF(secret, c) = SHA-1(secret || 00 x 12 || 00 00 00 (c + 1)) */
void dth11_cepm_kdf(const u8 secret[20], u8 counter, u8 out[20]);
/* PID の CTR の初期カウンターブロック: BE32((dw2 & 0xF0000FFF) | pid << 12) || BE32(dw1) || 00 x 8 */
void dth11_cepm_ctr_iv(u16 pid, u32 dw1, u32 dw2, u8 iv[16]);

/* ---- TS の復号 ---- */

/* keys は復号器が使い続けるので、復号器より長く生きていなければならない。IV の決定結果を keys に覚える */
void dth11_dec_init(struct dth11_decryptor *dec, const struct dth11_handshake *hs, struct dth11_appkeys *keys);

/*
 * 188 バイトのパケット 1 つを、その場で処理する。戻り値は 1 のとき出力に含める (平文か、復号済み)、
 * 0 のとき捨てる (鍵パケットで、消費済み。または、鍵がまだ無い)。now_ms は単調増加の時刻。
 */
int dth11_dec_process(struct dth11_decryptor *dec, u8 *pkt, u64 now_ms);

/* 新しいハンドシェイクが要るかを、1 回だけ真で返す (返したら要求は取り下げる) */
bool dth11_dec_take_rehandshake_request(struct dth11_decryptor *dec);
/* 新しいハンドシェイクの結果に差し替え、セッション、コンテンツ鍵、PID の文脈を全て破棄する */
void dth11_dec_install_handshake(struct dth11_decryptor *dec, const struct dth11_handshake *hs);

/* 鍵パケットか: アダプテーションフィールドだけで、私的データが鍵の型 (0xEC または 0xC0..0xCF) で始まる */
bool dth11_is_key_packet(const u8 *pkt);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_CEPM_H */
