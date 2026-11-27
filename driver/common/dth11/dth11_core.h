/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * dth11_core.h - BUFFALO DT-H11/U2 (EVA01 ブリッジ + PN2031 + MxL135RF) の共通層
 *
 * OS に依存しない。USB の制御転送と時刻は struct dth11_ops のコールバックで受けるため、
 * ホストの実装と自動試験 (模擬機材) が同じコードを使う。
 * 手順は i4M1k0SU/DTH10-series-driver (MIT License、Copyright (c) i4M1k0SU) の
 * crates/dth10-core の実装と、実機 (ファームウェア 1.0.18、ハードウェア 0x102) の確認による。
 * 機種の資料は docs/devices/dth11.md を参照。
 *
 * 扱う範囲は地上波 (ISDB-T) の受信まで。ストリームは device モード (byte0 = 0) だけを使う。
 * この機体のファームウェアは raw モード (byte0 = 1) の有効化を拒否する。
 * device モードのストリームのリンク暗号 (CEPM) は dth11_cepm.h が扱う。
 */
#ifndef DTH11_CORE_H
#define DTH11_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t s32;
#endif

#define DTH11_USB_VENDOR 0x0411
#define DTH11_USB_PRODUCT 0x01da

/* ベンダー要求。WinUSB で通るように、受け取り先は機器 (device) にする */
#define DTH11_REQ_OUT 0x40
#define DTH11_REQ_IN 0xc0

/* bulk IN の 1 スロットは 256 バイトで、先頭の 188 バイトが TS パケット (残りは詰め物) */
#define DTH11_SLOT_SIZE 256
#define DTH11_TS_PACKET_SIZE 188

#define DTH11_BATCH_MAX 256
/* 番組の PID リストの最大の長さ (dth11_psi.h の DTH11_PROGRAM_MAX_PIDS 以上) */
#define DTH11_PID_LIST_MAX 64

#define DTH11_LOG_ERR 0
#define DTH11_LOG_INFO 1
#define DTH11_LOG_DEBUG 2

struct dth11_ops {
	/* 戻り値は転送した長さ、または -errno。buf はスタック上のことがある。 */
	int (*control)(void *priv, u8 request_type, u8 request, u16 value, u16 index, void *buf, u16 len);
	void (*msleep)(void *priv, unsigned int ms);
	/* 単調増加の時刻 (ミリ秒) */
	u64 (*now_ms)(void *priv);
	void (*log)(void *priv, int level, const char *fmt, ...);
};

struct dth11_regval {
	u8 reg;
	u8 val;
};

struct dth11_bridge {
	const struct dth11_ops *ops;
	void *priv;
	/* I2C バッチ書き込みの待ち行列。他の I2C 転送の前に必ず送り出す */
	u8 batch[DTH11_BATCH_MAX];
	u16 batch_len;
};

/* 機器情報 TLV (wValue 0 / wIndex 0xC0DE) の内容のうち、この層が使うもの */
struct dth11_info {
	char model_name[64];
	char maker[64];
	char serial[64];
	char kit_name[32];
	u32 id;
	u32 flags;
	u32 gpio_mask_a;
	u32 gpio_mask_b;
	u32 gpio_mask_c;
	u32 gpio_c_open;
	u32 gpio_c_close;
	u32 stream_param;
	u32 hw_version;
	u32 tuner_xtal_hz;
	s32 if_freq_hz;
	u32 tuner_opts;
	u8 demod_addr;
	u8 tuner_addr;
	/* TLV に存在した項目 (struct dth11_present の和) */
	u32 present;
};

/* struct dth11_info.present のビット (純正ドライバーの解析器と同じ割り当て) */
#define DTH11_PRESENT_HW_VERSION 0x00001
#define DTH11_PRESENT_ID 0x00002
#define DTH11_PRESENT_KIT_NAME 0x00008
#define DTH11_PRESENT_GPIO_A 0x00020
#define DTH11_PRESENT_GPIO_B 0x00080
#define DTH11_PRESENT_STREAM_PARAM 0x01000
#define DTH11_PRESENT_XTAL 0x02000
#define DTH11_PRESENT_IF 0x04000
#define DTH11_PRESENT_OPTS 0x08000

/* ロックの状態。復調器のレジスタ 0x0A の下位 4 ビット (同期状態) が 9 以上で完全ロック */
#define DTH11_SYNC_LOCKED 9

struct dth11_signal {
	u8 sync_state;
	bool locked;
	/* C/N (0.1 dB)。測定が完了していないときは valid_cnr が false */
	bool valid_cnr;
	u16 cnr_raw;
	int cnr_tenth_db;
	/* チューナーの信号強度 (0.001 単位の相対値)。純正ドライバーの式による */
	int strength;
};

enum dth11_stream_mode {
	/* PSI と 1seg だけでなく、番組の PID リストに従ってデバイス内蔵カードで復号したストリーム */
	DTH11_STREAM_DEVICE = 0,
};

#ifdef __cplusplus
extern "C" {
#endif

void dth11_bridge_init(struct dth11_bridge *br, const struct dth11_ops *ops, void *priv);

/* 機器情報 TLV を読んで解析する。形式が不正なとき -EPROTO、この機種でない (キット名) とき -ENODEV */
int dth11_read_info(struct dth11_bridge *br, struct dth11_info *info);
/* 機器情報 TLV の解析だけ (試験と、別の経路で得た TLV 用) */
int dth11_parse_info(const u8 *blob, size_t len, struct dth11_info *info);

/* ファームウェアの版を (major, minor, build) で返す */
int dth11_fw_version(struct dth11_bridge *br, u8 *major, u8 *minor, u16 *build);

/*
 * 電源投入、復調器とチューナーの初期化、チューナーの起床、機器の起動まで。
 * 起動 (wIndex 0x0010) より先にチューナーを起こす。この機体は起動の後の最初のチューナー書き込みで
 * 要求を拒否することがあるため (実機で確認)、起床を先にする。
 */
int dth11_start(struct dth11_bridge *br, const struct dth11_info *info);
/* チューナーをスタンバイにして、電源を落とす */
int dth11_stop(struct dth11_bridge *br, const struct dth11_info *info);

/* 地上波の選局。freq_hz は中心周波数 (Hz)、帯域は 6 MHz 固定 */
int dth11_tune(struct dth11_bridge *br, const struct dth11_info *info, u32 freq_hz);
/* UHF のチャンネル番号 (13..62) から中心周波数 (Hz)。範囲外は 0 */
u32 dth11_uhf_freq_hz(int channel);

int dth11_read_signal(struct dth11_bridge *br, const struct dth11_info *info, struct dth11_signal *sig);
/* 復調器の同期状態だけを読む (ロック待ちの周期処理用) */
int dth11_read_sync_state(struct dth11_bridge *br, const struct dth11_info *info, u8 *sync_state);

int dth11_stream_enable(struct dth11_bridge *br, const struct dth11_info *info, enum dth11_stream_mode mode);
int dth11_stream_disable(struct dth11_bridge *br);
/*
 * 番組の PID リストをデバイスへ送る (wIndex 0x1000)。デバイスは 1 番組分だけ保持し、
 * ストリームの無効化で忘れる。
 */
int dth11_set_program_pids(struct dth11_bridge *br, u32 program_number, const u16 *pids, size_t count);

/*
 * 追加の経路 (CEPM) 用の素のベンダー要求。path_flags はチューナーの経路を選ぶ bRequest の上位ビット
 * (0x00 = チューナー #0 の経路、0x80 = #1 の経路)。データ段の向きの下位ビットは、ここで付ける。
 * 戻り値は、OUT が 0 か負のエラー、IN が受け取った長さか負のエラー。
 */
int dth11_vendor_out(struct dth11_bridge *br, u8 path_flags, u16 value, u16 index, const void *data, u16 len);
int dth11_vendor_in(struct dth11_bridge *br, u8 path_flags, u16 value, u16 index, void *buf, u16 len);

/* ---- 内部の部品。自動試験が直接呼ぶ ---- */

/* C/N の生値 (レジスタ 0x46/0x47) から 0.1 dB 単位。0 と 19648 以上は 0、788 未満は 300 (30.0 dB) */
int dth11_cnr_tenth_db_from_raw(u16 raw);
/* MxL135RF の周波数ワード (整数部と 1/64 MHz 単位の小数) */
u16 dth11_rf_tune_word(u32 freq_hz);
/* MxL135RF の初期化表を機器情報で補正して out (22 個) に書く。IF が 0 のとき -EINVAL */
#define DTH11_MXL_INIT_COUNT 22
#define DTH11_MXL_RFTUNE_COUNT 10
int dth11_mxl_init_registers(const struct dth11_info *info, struct dth11_regval *out);
void dth11_mxl_rftune_registers(u32 freq_hz, u32 bw_hz, struct dth11_regval *out);
int dth11_signal_strength(u8 cc, u8 ba);

/*
 * bulk IN のデータ (256 バイトのスロットの並び) から TS パケットを取り出す。
 * 戻り値は out に書いた長さ (188 の倍数)。out は len / 256 * 188 バイト以上。
 * 先頭が 0x47 でないスロットは捨てる (ストリーム開始直後に TS ヘッダーが 0 のスロットが来る)。
 * 256 バイトに満たない末尾は捨てる。drop_filler が真のとき、ブリッジが復調器の出力が有効に
 * なるまで出す詰め物 (47 9F FF 1F …) も捨てる。
 */
size_t dth11_extract_ts(const u8 *in, size_t len, u8 *out, bool drop_filler);

#ifdef __cplusplus
}
#endif

#endif /* DTH11_CORE_H */
