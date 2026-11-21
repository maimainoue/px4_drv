/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * tbs5530_core.h - TurboSight TBS 5530 (ISDB-T) driver core
 *
 * The core is OS independent: it talks to the USB device and to the clock
 * through the callbacks in struct tbs5530_ops, so the same code runs in a
 * user-space test program (usbdevfs) and in the kernel module.
 *
 * The TBS 5530 is a Cypress FX2 style USB bridge (firmware downloaded to RAM on
 * every start), a Sony CXD6822 / CXD2878A demodulator with its built-in Freia
 * tuner for terrestrial and cable, and a Montage M88RS6060 for satellite. Only
 * the terrestrial ISDB-T half is handled here: the M88RS6060 receives DVB-S/S2
 * only, so ISDB-S cannot be received on this board.
 *
 * The bridge protocol and the demodulator / tuner register sequences follow
 * TurboSight's linux_media sources:
 *   tbs5530.c  Copyright (c) 2022 Davin <Davin@tbsdtv.com>, GPL-2.0
 *   cxd2878.c  Copyright (c) 2021 Davin zhang <Davin@tbsdtv.com>, GPL-2.0-or-later
 */
#ifndef TBS5530_CORE_H
#define TBS5530_CORE_H

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
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;
#endif

#define TBS5530_USB_VENDOR 0x734c
#define TBS5530_USB_PRODUCT 0x5530

/* the transport stream is on bulk endpoint 0x82 (188-byte TS packets) */
#define TBS5530_TS_ENDPOINT 0x82
#define TBS5530_TS_PACKET_SIZE 188

/* USB request types for the vendor requests */
#define TBS5530_REQ_OUT 0x40 /* vendor, host to device */
#define TBS5530_REQ_IN 0xc0 /* vendor, device to host */

/* The host's view of the device: USB control transfers, time, a log. */
struct tbs5530_ops {
	/*
	 * Control transfer. buf may point to the stack, so an implementation that hands it to a DMA
	 * capable transfer (the Linux kernel) has to copy it to heap memory first.
	 * Returns the length transferred or -errno.
	 */
	int (*control)(void *priv, u8 request_type, u8 request, u16 value, u16 index, void *buf, u16 len);
	void (*msleep)(void *priv, unsigned int ms);
	void (*log)(void *priv, int level, const char *fmt, ...);
};

#define TBS5530_LOG_ERR 0
#define TBS5530_LOG_INFO 1
#define TBS5530_LOG_DEBUG 2

struct tbs5530_bridge {
	const struct tbs5530_ops *ops;
	void *priv;

	int err; /* first error of a sequence of register accesses (negative errno) */
	int log_level;
};

/* demodulator state */
enum tbs5530_demod_state {
	TBS5530_DEMOD_UNKNOWN = 0,
	TBS5530_DEMOD_SLEEP,
	TBS5530_DEMOD_ACTIVE,
};

struct tbs5530_demod {
	struct tbs5530_bridge *b;
	enum tbs5530_demod_state state;
	u32 chip_id;	/* CXD28xx chip id read at attach */
	u8 tuner_id;	/* register 0x7f of the built-in tuner, read at init */
	bool warm;	/* the power-on sequence has run */
	bool isdbt;	/* the demodulator is set to ISDB-T */
	bool isdbc;	/* the demodulator is set to ISDB-C (J.83 Annex C) */
	bool led_locked; /* last state written to the lock LED */
};

/* status flags of tbs5530_demod_read_status() */
#define TBS5530_STATUS_SIGNAL 0x01 /* a carrier was found */
#define TBS5530_STATUS_LOCK 0x02 /* synchronised and the TS is locked */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- bridge ---- */
void tbs5530_bridge_init(struct tbs5530_bridge *b, const struct tbs5530_ops *ops, void *priv);
/*
 * Downloads the bridge firmware (stop the CPU, write 0x40 bytes at a time, restart it). The device
 * disconnects and reconnects on its own afterwards; the caller waits for it and re-opens it.
 */
int tbs5530_fw_load(struct tbs5530_bridge *b, const u8 *image, size_t size);
/* I2C master. addr is the 7-bit address. data[0] is the register, followed by the bytes to write (at most 48). */
int tbs5530_i2c_write(struct tbs5530_bridge *b, u8 addr, const u8 *data, unsigned int len);
/* writes the register pointer and reads len bytes (at most 48) in one transaction */
int tbs5530_i2c_read(struct tbs5530_bridge *b, u8 addr, u8 reg, u8 *out, unsigned int len);
/* selects the demodulator that feeds the bulk endpoint: terrestrial/cable (false) or satellite (true) */
int tbs5530_select_stream(struct tbs5530_bridge *b, bool satellite);
/* MAC address stored in the EEPROM (bytes 16..21); identifies the board */
int tbs5530_read_mac(struct tbs5530_bridge *b, u8 mac[6]);

/* ---- demodulator and tuner (ISDB-T, 6 MHz channels) ---- */
/* reads the chip id; only CXD6822 and CXD2878A (Freia tuner) are supported, others return -ENODEV */
int tbs5530_demod_attach(struct tbs5530_demod *d, struct tbs5530_bridge *b);
/* power-on sequence and built-in tuner initialisation */
int tbs5530_demod_init(struct tbs5530_demod *d);
/* tunes an ISDB-T channel (centre frequency in kHz) and enables the TS output */
int tbs5530_demod_tune_isdbt(struct tbs5530_demod *d, u32 frequency_khz);
/* ISDB-C (J.83 Annex C, 6 MHz, 64QAM). The TSMF is not separated here: the TS is passed through. */
int tbs5530_demod_tune_isdbc(struct tbs5530_demod *d, u32 frequency_khz);
/* reads the lock state; status is a mask of TBS5530_STATUS_* */
int tbs5530_demod_read_status(struct tbs5530_demod *d, u8 *status);
/* carrier to noise ratio in 0.001 dB; only valid while locked */
int tbs5530_demod_read_cnr(struct tbs5530_demod *d, s32 *cnr_mdb);
/* disables the TS output and puts the demodulator to sleep */
int tbs5530_demod_sleep(struct tbs5530_demod *d);

/* fixed point log10(x) * 100 (the demodulator's own table), exported for tests */
u32 tbs5530_log10_x100(u32 x);

#ifdef __cplusplus
}
#endif
#endif /* TBS5530_CORE_H */
