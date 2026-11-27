/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_appkeys.h - 利用者が用意する純正の CoBDACom.dll (PCastTV3) から、リンク暗号 (CEPM) の
 *                   アプリケーション側の資格情報を実行時に読み出す
 *
 * この層にも、リポジトリのどこにも、資格情報の値は含まれない。読み出すのは次の 2 つで、いずれも
 * 検証で正しさを確かめてから採用する。
 *
 *  - アプリケーションの証明書 (104 バイト) と秘密鍵 (20 バイト): DLL のコードが、データ領域へ
 *    1 バイトずつ即値で書き込む命令列 (mov byte ptr [addr], imm8 など) を再生して画像を組み立て、
 *    証明書は自己署名の検証で、秘密鍵は d * G == Q で見つける。
 *  - コンテンツ鍵を CBC で開くときの IV (16 バイト): コードがスタック上に組み立てる 16 バイトの
 *    バッファを候補とし、最初のコンテンツブロックの CMAC が一致する候補を採用する。
 *
 * x86 の逆アセンブラではなく、上の格納命令の形だけを拾うパターン走査で、他の命令は 1 バイトずつ
 * 読み飛ばす。ゴミを拾うことがあるが、上の検証で採用されない。DLL の実行はしない。
 * 値の出どころと検証の考え方は i4M1k0SU/DTH10-series-driver (MIT License、Copyright (c) i4M1k0SU) の
 * crates/dth10-core/src/cepm/appkeys.rs による。
 */
#ifndef DTH11_APPKEYS_H
#define DTH11_APPKEYS_H

#include "dth11_core.h"

#define DTH11_CERT_LEN 104
#define DTH11_PRIV_LEN 20

/* 失敗の種類。DLL でない、証明書が無い、秘密鍵が無いを区別して返す */
#define DTH11_APPKEYS_NOT_PE (-EINVAL)
#define DTH11_APPKEYS_NO_CERT (-ENOENT)
#define DTH11_APPKEYS_NO_PRIVATE_KEY (-ENODATA)

struct dth11_appkeys {
	u8 cert[DTH11_CERT_LEN];
	u8 priv_key[DTH11_PRIV_LEN];
	/* コードが組み立てた 16 バイトのスタックバッファ (格納された順と、反転した順の両方) */
	u8 (*iv_candidates)[16];
	size_t iv_candidate_count;
	/* コード以外のセクションの内容。IV が上の候補に無いときの最後の探索先 */
	u8 *data;
	size_t data_len;
	bool iv_settled;
	bool iv_scan_done; /* データ領域の全走査を一度行った (失敗した後は繰り返さない) */
	u8 iv_content[16];
};

#ifdef __cplusplus
extern "C" {
#endif

/* PE 画像 (CoBDACom.dll の内容) から資格情報を読み出す。失敗時は out に何も残さない */
int dth11_appkeys_from_image(const u8 *image, size_t len, struct dth11_appkeys *out);
void dth11_appkeys_free(struct dth11_appkeys *keys);

/*
 * コンテンツブロック (64 バイト: id 16、包んだ鍵 32、CMAC 16) とセッションの K3 から、
 * 包んだ鍵を開く IV を決める。決まった IV は keys に覚える。見つかれば 0、なければ -ENOENT。
 */
int dth11_appkeys_find_content_iv(struct dth11_appkeys *keys, const u8 k3[16], const u8 blob[64], u8 iv[16]);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_APPKEYS_H */
