/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_ec.h - secp160r1 の楕円曲線演算 (ECDSA 署名と検証、ECDH)
 *
 * DT-H11/U2 の CEPM ハンドシェイク (dth11_cepm.h) が使う。ハンドシェイクは機器を開くたびに数回だけ
 * 行うため、性能とサイドチャネル耐性は扱わない (Jacobian 座標の素朴な実装)。
 * 点は 40 バイト (x と y を、それぞれ 20 バイトのビッグエンディアン) で受け渡す。
 * ECDSA は SHA-1 のダイジェストに署名し、署名は r と s を 20 バイトずつ並べた 40 バイト。
 * 試験は tests/dth11_ec_test.c (機器の証明書の自己署名の検証、署名の往復、ECDH の対称性、n * G が無限遠点)。
 */
#ifndef DTH11_EC_H
#define DTH11_EC_H

#include "dth11_core.h"

#define DTH11_EC_BYTES 20
#define DTH11_EC_POINT_BYTES 40
#define DTH11_EC_SIG_BYTES 40

/* 乱数の供給。buf を len バイトの乱数で埋め、成功なら 0、失敗なら負のエラー */
typedef int (*dth11_rng_fn)(void *priv, u8 *buf, size_t len);

#ifdef __cplusplus
extern "C" {
#endif

/* 点 (x, y) が曲線上にあれば 0、なければ -EINVAL。無限遠点は扱わない */
int dth11_ec_point_check(const u8 xy[DTH11_EC_POINT_BYTES]);

/* d * G。d が 0 か n 以上のとき -EINVAL */
int dth11_ec_public_key(const u8 d[DTH11_EC_BYTES], u8 out_xy[DTH11_EC_POINT_BYTES]);

/* 秘密鍵 d が公開鍵 q に対応していれば真 (d は 1..n-1 の範囲で、d * G == q) */
bool dth11_ec_key_matches(const u8 d[DTH11_EC_BYTES], const u8 q_xy[DTH11_EC_POINT_BYTES]);

/* ECDH: d * Q の x 座標。Q が曲線上になければ -EINVAL、結果が無限遠点なら -EINVAL */
int dth11_ec_ecdh(const u8 d[DTH11_EC_BYTES], const u8 q_xy[DTH11_EC_POINT_BYTES], u8 shared[DTH11_EC_BYTES]);

/*
 * ECDSA 署名 (SHA-1)。k は rng から取る。署名の値が 160 ビットに収まらないとき (n は 161 ビット)
 * は k を取り直す。rng の失敗はそのまま返す。
 */
int dth11_ec_sign(const u8 d[DTH11_EC_BYTES], const void *msg, size_t len, dth11_rng_fn rng, void *rng_priv,
		  u8 sig[DTH11_EC_SIG_BYTES]);

/* ECDSA 検証 (SHA-1)。公開鍵が曲線上にない、署名が範囲外、不一致のとき偽 */
bool dth11_ec_verify(const u8 q_xy[DTH11_EC_POINT_BYTES], const void *msg, size_t len,
		     const u8 sig[DTH11_EC_SIG_BYTES]);

/* 1..n-1 の乱数の秘密鍵を作る。rng の失敗はそのまま返す */
int dth11_ec_random_scalar(dth11_rng_fn rng, void *rng_priv, u8 d[DTH11_EC_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_EC_H */
