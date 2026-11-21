// SPDX-License-Identifier: GPL-2.0-only
/*
 * tbs5530_core.c - TurboSight TBS 5530 (ISDB-T) driver core
 *
 * Bridge protocol: TurboSight linux_media tbs5530.c (Davin, GPL-2.0).
 * Demodulator and tuner sequences: TurboSight linux_media cxd2878.c (Davin zhang, GPL-2.0-or-later),
 * reduced to the ISDB-T and ISDB-C (6 MHz) paths of the CXD6822 / CXD2878A with its built-in Freia tuner.
 * ISDB-C is the vendor's DVB-C 6 MHz setting plus the additions of m1tore/TBS6281SEv2_drv (GPL-2.0-or-later).
 */
#include "tbs5530_core.h"

#define SLVT 0x64 /* demodulator, terrestrial side */
#define SLVX 0x66 /* demodulator, common registers (SLVT + 2) */
#define TUNER 0x60 /* built-in tuner behind the demodulator's I2C repeater */
#define EEPROM 0x50

#define REQ_I2C_WRITE 0x80
#define REQ_STREAM_SELECT 0x8a
#define REQ_I2C_PTR 0x90
#define REQ_I2C_READ 0x91
#define REQ_FIRMWARE 0xa0

#define I2C_MAX 48 /* the bridge forwards at most 50 bytes including its two byte header */

#define CHIP_ID_CXD6822 0x19c
#define CHIP_ID_CXD2878A 0x39a

/* ISDB-T intermediate frequency of the demodulator: 3.55 MHz, (3.55 / 48 * 2^24 + 0.5) */
#define IFFREQ_ISDBT_6 0x12eeef
/* ISDB-C (DVB-C 6 MHz) intermediate frequency: 3.7 MHz, (3.7 / 48 * 2^24 + 0.5) */
#define IFFREQ_ISDBC_6 0x13bbbc

/* Offset the vendor driver subtracts from the measured C/N (0.001 dB) */
#define CNR_OFFSET_MDB 1500

static void log_at(struct tbs5530_bridge *b, int level, const char *msg)
{
	if (b->ops->log && level <= b->log_level)
		b->ops->log(b->priv, level, "tbs5530: %s\n", msg);
}

static void sleep_ms(struct tbs5530_bridge *b, unsigned int ms)
{
	if (b->ops->msleep)
		b->ops->msleep(b->priv, ms);
}

void tbs5530_bridge_init(struct tbs5530_bridge *b, const struct tbs5530_ops *ops, void *priv)
{
	memset(b, 0, sizeof(*b));
	b->ops = ops;
	b->priv = priv;
	b->log_level = TBS5530_LOG_ERR;
}

/* ---- bridge ---- */

static int fx2_write(struct tbs5530_bridge *b, u16 address, const u8 *data, u16 len)
{
	u8 buf[0x40];
	int ret;

	if (len > sizeof(buf))
		return -EINVAL;
	memcpy(buf, data, len);
	ret = b->ops->control(b->priv, TBS5530_REQ_OUT, REQ_FIRMWARE, address, 0, buf, len);
	if (ret < 0)
		return ret;
	return ret == len ? 0 : -EIO;
}

/* CPUCS is written at both 0x7f92 and 0xe600, as the vendor driver does */
static int fx2_cpu(struct tbs5530_bridge *b, bool run)
{
	u8 v = run ? 0 : 1;
	int ret;

	ret = fx2_write(b, 0x7f92, &v, 1);
	if (ret)
		return ret;
	return fx2_write(b, 0xe600, &v, 1);
}

int tbs5530_fw_load(struct tbs5530_bridge *b, const u8 *image, size_t size)
{
	size_t i;
	int ret;

	if (!image || size == 0 || size > 0x10000)
		return -EINVAL;
	ret = fx2_cpu(b, false);
	if (ret) {
		log_at(b, TBS5530_LOG_ERR, "could not stop the USB controller CPU");
		return ret;
	}
	for (i = 0; i < size; i += 0x40) {
		size_t n = size - i < 0x40 ? size - i : 0x40;

		ret = fx2_write(b, (u16)i, image + i, (u16)n);
		if (ret) {
			log_at(b, TBS5530_LOG_ERR, "error while transferring firmware");
			return ret;
		}
	}
	ret = fx2_cpu(b, true);
	if (ret) {
		log_at(b, TBS5530_LOG_ERR, "could not restart the USB controller CPU");
		return ret;
	}
	sleep_ms(b, 100);
	return 0;
}

int tbs5530_i2c_write(struct tbs5530_bridge *b, u8 addr, const u8 *data, unsigned int len)
{
	u8 buf[I2C_MAX + 2];
	int ret;

	if (len == 0 || len > I2C_MAX)
		return -EINVAL;
	buf[0] = (u8)(len + 1);
	buf[1] = (u8)(addr << 1);
	memcpy(buf + 2, data, len);
	ret = b->ops->control(b->priv, TBS5530_REQ_OUT, REQ_I2C_WRITE, 0, 0, buf, (u16)(len + 2));
	if (ret < 0)
		return ret;
	return ret == (int)(len + 2) ? 0 : -EIO;
}

int tbs5530_i2c_read(struct tbs5530_bridge *b, u8 addr, u8 reg, u8 *out, unsigned int len)
{
	u8 ptr[3];
	u8 buf[I2C_MAX];
	int ret;

	if (len == 0 || len > I2C_MAX)
		return -EINVAL;
	ptr[0] = (u8)len;
	ptr[1] = (u8)(addr << 1);
	ptr[2] = reg;
	ret = b->ops->control(b->priv, TBS5530_REQ_OUT, REQ_I2C_PTR, 0, 0, ptr, sizeof(ptr));
	if (ret < 0)
		return ret;
	if (ret != (int)sizeof(ptr))
		return -EIO;
	ret = b->ops->control(b->priv, TBS5530_REQ_IN, REQ_I2C_READ, 0, 0, buf, (u16)len);
	if (ret < 0)
		return ret;
	if (ret != (int)len)
		return -EIO;
	memcpy(out, buf, len);
	return 0;
}

int tbs5530_select_stream(struct tbs5530_bridge *b, bool satellite)
{
	u8 buf[2] = { 10, satellite ? 1 : 0 };
	int ret;

	ret = b->ops->control(b->priv, TBS5530_REQ_OUT, REQ_STREAM_SELECT, 0, 0, buf, sizeof(buf));
	if (ret < 0)
		return ret;
	return ret == (int)sizeof(buf) ? 0 : -EIO;
}

int tbs5530_read_mac(struct tbs5530_bridge *b, u8 mac[6])
{
	int i, ret;

	for (i = 0; i < 6; i++) {
		ret = tbs5530_i2c_read(b, EEPROM, (u8)(16 + i), &mac[i], 1);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---- register helpers: the first failure is kept in b->err and later accesses are skipped ---- */

static void put(struct tbs5530_demod *d, u8 addr, u8 reg, const u8 *data, unsigned int n)
{
	u8 buf[I2C_MAX];

	if (d->b->err)
		return;
	if (n + 1 > sizeof(buf)) {
		d->b->err = -EINVAL;
		return;
	}
	buf[0] = reg;
	memcpy(buf + 1, data, n);
	d->b->err = tbs5530_i2c_write(d->b, addr, buf, n + 1);
}

static void wr(struct tbs5530_demod *d, u8 addr, u8 reg, u8 val)
{
	put(d, addr, reg, &val, 1);
}

static u8 rd(struct tbs5530_demod *d, u8 addr, u8 reg)
{
	u8 v = 0;

	if (d->b->err)
		return 0;
	d->b->err = tbs5530_i2c_read(d->b, addr, reg, &v, 1);
	return v;
}

static void setbits(struct tbs5530_demod *d, u8 addr, u8 reg, u8 val, u8 mask)
{
	if (mask == 0)
		return;
	if (mask != 0xff) {
		u8 cur = rd(d, addr, reg);

		val = (u8)((val & mask) | (cur & (mask ^ 0xff)));
	}
	wr(d, addr, reg, val);
}

static void bank_setbits(struct tbs5530_demod *d, u8 addr, u8 bank, u8 reg, u8 val, u8 mask)
{
	wr(d, addr, 0x00, bank);
	setbits(d, addr, reg, val, mask);
}

static void delay(struct tbs5530_demod *d, unsigned int ms)
{
	if (!d->b->err)
		sleep_ms(d->b, ms);
}

/* returns the sequence's first error and clears it */
static int finish(struct tbs5530_demod *d)
{
	int e = d->b->err;

	d->b->err = 0;
	return e;
}

/* ---- math ---- */

static const u8 log2_lookup[] = {
	0,  4,  9,  13, 17, 21, 25, 29, 32, 36, 39, 43, 46, 49, 52, 55,
	58, 61, 64, 67, 70, 73, 75, 78, 81, 83, 86, 88, 91, 93, 95, 98,
};

static u32 log2_x100(u32 x)
{
	u8 count = 0;
	u32 xval = x;
	u32 r;

	for (x >>= 1; x > 0; x >>= 1)
		count++;
	r = (u32)count * 100;
	if (count > 0) {
		u8 index;

		if (count <= 5)
			index = (u8)(xval << (5 - count)) & 0x1f;
		else
			index = (u8)(xval >> (count - 5)) & 0x1f;
		r += log2_lookup[index];
	}
	return r;
}

u32 tbs5530_log10_x100(u32 x)
{
	/* log10(x) = log2(x) / log2(10); log2(10) * 100 = 332 */
	return (100 * log2_x100(x) + 332 / 2) / 332;
}

/* ---- demodulator: power-on ---- */

static void i2c_repeater(struct tbs5530_demod *d, bool enable)
{
	wr(d, SLVX, 0x08, enable ? 1 : 0);
	delay(d, 20);
}

/* Freia (CXD6866) tuner initialisation; the I2C repeater must be enabled */
static void freia_init(struct tbs5530_demod *d)
{
	static const u8 t5e[3] = { 0x15, 0x00, 0x00 };
	static const u8 t79[3] = { 0x9e, 0x00, 0x00 };
	static const u8 t99[3] = { 0xa9, 0x01, 0x00 };
	/* regs 0x81..0x94: oscillator, REFOUT, GPIO, CPU boot, RFAGC and analog block settings */
	static const u8 t81[20] = { 0x18, 0x84, 0xb5, 0x82, 0x00, 0x00, 0x84, 0x40, 0x10, 0x00,
				    0x45, 0x75, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x0c, 0x3f, 0x00 };
	static const u8 t22[2] = { 0x00, 0x08 };
	static const u8 t17[2] = { 0x7f, 0x06 };
	static const u8 t41[2] = { 0x00, 0x00 };
	static const u8 t45[6] = { 0x0a, 0x00, 0x00, 0x11, 0x00, 0x03 };
	static const u8 ta6[2] = { 0x66, 0x08 };
	u8 status, sram;

	d->tuner_id = rd(d, TUNER, 0x7f);
	wr(d, TUNER, 0x01, 0x00);
	wr(d, TUNER, 0x67, 0x00);
	wr(d, TUNER, 0x43, 0xc0);
	put(d, TUNER, 0x5e, t5e, sizeof(t5e));
	wr(d, TUNER, 0x0c, 0x14);
	put(d, TUNER, 0x79, t79, sizeof(t79));
	put(d, TUNER, 0x99, t99, sizeof(t99));
	put(d, TUNER, 0x81, t81, sizeof(t81));
	put(d, TUNER, 0x22, t22, sizeof(t22));
	wr(d, TUNER, 0x46, 0x00);
	delay(d, 10);

	/* CPU_STT must be 0 once the tuner's CPU has booted */
	status = rd(d, TUNER, 0x1a);
	if (!d->b->err && status != 0x00) {
		log_at(d->b, TBS5530_LOG_ERR, "built-in tuner CPU did not boot");
		d->b->err = -EIO;
	}
	put(d, TUNER, 0x17, t17, sizeof(t17));
	delay(d, 1);
	sram = rd(d, TUNER, 0x19);
	wr(d, TUNER, 0x79, sram == 0 ? 0x00 : 0x9e);
	wr(d, TUNER, 0x74, 0x02);
	wr(d, TUNER, 0xa0, 0x0b);
	wr(d, TUNER, 0x88, 0x00);
	wr(d, TUNER, 0x87, 0x80);
	wr(d, TUNER, 0x80, 0x01);
	put(d, TUNER, 0x41, t41, sizeof(t41));
	put(d, TUNER, 0x45, t45, sizeof(t45));
	put(d, TUNER, 0xa6, ta6, sizeof(ta6));
	wr(d, TUNER, 0x1e, 0xa0);
}

int tbs5530_demod_attach(struct tbs5530_demod *d, struct tbs5530_bridge *b)
{
	u8 hi, lo;
	u32 id;

	memset(d, 0, sizeof(*d));
	d->b = b;
	d->state = TBS5530_DEMOD_UNKNOWN;
	wr(d, SLVX, 0x00, 0x00);
	hi = rd(d, SLVX, 0xfb);
	lo = rd(d, SLVX, 0xfd);
	if (finish(d))
		return -EIO;
	id = ((u32)(hi & 0x03) << 8) | lo;
	d->chip_id = id;
	if (id != CHIP_ID_CXD6822 && id != CHIP_ID_CXD2878A) {
		log_at(b, TBS5530_LOG_ERR, "unsupported demodulator (only CXD6822 and CXD2878A are handled)");
		return -ENODEV;
	}
	return 0;
}

int tbs5530_demod_init(struct tbs5530_demod *d)
{
	int ret;

	/* clear all registers */
	wr(d, SLVX, 0x02, 0x00);
	delay(d, 4);
	wr(d, SLVX, 0x00, 0x00);
	/* crystal oscillator: 24 MHz (clock mode 1) */
	wr(d, SLVX, 0x1d, 0x00);
	wr(d, SLVX, 0x14, 0x01);
	delay(d, 2);
	wr(d, SLVX, 0x50, 0x00);
	delay(d, 2);
	wr(d, SLVX, 0x10, 0x00);
	delay(d, 21);
	/* CXD6822 / CXD2878A */
	wr(d, SLVT, 0x00, 0x95);
	wr(d, SLVT, 0x23, 0x03);
	/* tuner I2C bus */
	bank_setbits(d, SLVX, 0x00, 0x1a, 0x01, 0xff);
	delay(d, 2);

	i2c_repeater(d, true);
	freia_init(d);
	i2c_repeater(d, false);

	/* TS output: parallel, clock mask 1, drive currents */
	bank_setbits(d, SLVT, 0x00, 0xc4, 0x00, 0x80);
	bank_setbits(d, SLVT, 0x02, 0xe4, 0x00, 0x01);
	bank_setbits(d, SLVT, 0x00, 0xc6, 0x01, 0x1f);
	bank_setbits(d, SLVT, 0x60, 0x52, 0x01, 0x1f);
	bank_setbits(d, SLVT, 0x00, 0x95, 0x01, 0x03);
	bank_setbits(d, SLVT, 0x00, 0x95, 0x14, 0x3c);
	bank_setbits(d, SLVT, 0x00, 0x96, 0x54, 0xff);
	bank_setbits(d, SLVT, 0x00, 0x97, 0x54, 0xff);

	/* GPIO2 drives the lock LED: start unlocked */
	bank_setbits(d, SLVX, 0x00, 0xa5, 0x00, 0x0f);
	bank_setbits(d, SLVX, 0x00, 0x82, 0x01, 0x04);
	bank_setbits(d, SLVX, 0x00, 0xa2, 0x00, 0x04);

	ret = finish(d);
	if (ret)
		return ret;
	d->state = TBS5530_DEMOD_SLEEP;
	d->warm = true;
	d->led_locked = false;
	return 0;
}

/* ---- demodulator: ISDB-T ---- */

static void set_stream_output(struct tbs5530_demod *d, bool enable)
{
	u8 mode;

	wr(d, SLVT, 0x00, 0x00);
	mode = rd(d, SLVT, 0xa9);
	/* only the TS output (not TLV) is used for ISDB-T */
	if (!d->b->err && (mode & 0x03) == 0x00)
		wr(d, SLVT, 0xc3, enable ? 0x00 : 0x01);
}

static void set_ts_data_hiz(struct tbs5530_demod *d, bool enable)
{
	u8 c4, mask;

	wr(d, SLVT, 0x00, 0x00);
	/* the TLV output is not used, so only the TS pin layout (register 0xc4) is looked at */
	if (rd(d, SLVT, 0xa9) & 0x01) {
		if (!d->b->err)
			d->b->err = -EOPNOTSUPP;
		return;
	}
	c4 = rd(d, SLVT, 0xc4);
	switch (c4 & 0x88) {
	case 0x80:
		mask = 0x01; /* serial, TSDATA0 */
		break;
	case 0x88:
		mask = 0x80; /* serial, TSDATA7 */
		break;
	default:
		mask = 0xff; /* parallel */
		break;
	}
	wr(d, SLVT, 0x00, 0x00);
	setbits(d, SLVT, 0x81, enable ? 0xff : 0x00, mask);
}

/* TS interface clock for ISDB-T with a parallel TS output (the board's configuration) */
static void set_ts_clock(struct tbs5530_demod *d)
{
	u8 c4;

	wr(d, SLVT, 0x00, 0x00);
	c4 = rd(d, SLVT, 0xc4);
	if (!d->b->err && (c4 & 0x80)) {
		d->b->err = -EOPNOTSUPP; /* serial TS is not used on this board */
		return;
	}
	/* ISDB-T: the TS rate control is off */
	setbits(d, SLVT, 0xd3, 0x01, 0x01);
	setbits(d, SLVT, 0xde, 0x00, 0x01);
	setbits(d, SLVT, 0xda, 0x00, 0x01);
	wr(d, SLVT, 0xd9, 0x08); /* parallel: TS clock period 8 */
	setbits(d, SLVT, 0x32, 0x00, 0x01); /* disable the TS IF clock */
	setbits(d, SLVT, 0x33, 0x01, 0x03); /* clock select */
	setbits(d, SLVT, 0x32, 0x01, 0x01); /* enable the TS IF clock */
	bank_setbits(d, SLVT, 0x10, 0x66, 0x01, 0x01); /* parity period (DVB-T) */
	bank_setbits(d, SLVT, 0x40, 0x66, 0x01, 0x01); /* parity period (DVB-C family) */
}

/* 6 MHz band setting; it is also used to retune while the demodulator is already active */
static void isdbt_band_setting(struct tbs5530_demod *d)
{
	static const u8 nominal_rate[5] = { 0x17, 0xa0, 0x00, 0x00, 0x00 };
	static const u8 itb_coef[14] = { 0x31, 0xa8, 0x29, 0x9b, 0x27, 0x9c, 0x28,
					 0x9e, 0x29, 0xa4, 0x29, 0xa2, 0x29, 0xa8 };
	static const u8 iffreq[3] = { (IFFREQ_ISDBT_6 >> 16) & 0xff, (IFFREQ_ISDBT_6 >> 8) & 0xff,
				      IFFREQ_ISDBT_6 & 0xff };
	static const u8 d9[2] = { 0x1f, 0x79 };

	wr(d, SLVT, 0x00, 0x10);
	put(d, SLVT, 0x9f, nominal_rate, sizeof(nominal_rate));
	put(d, SLVT, 0xa6, itb_coef, sizeof(itb_coef));
	put(d, SLVT, 0xb6, iffreq, sizeof(iffreq));
	wr(d, SLVT, 0xd7, 0x00);
	put(d, SLVT, 0xd9, d9, sizeof(d9));
	wr(d, SLVT, 0x00, 0x12);
	wr(d, SLVT, 0x71, 0x07);
	wr(d, SLVT, 0x00, 0x15);
	wr(d, SLVT, 0xbe, 0x02);
}

/* sleep to active (ISDB-T) */
static void wake_isdbt(struct tbs5530_demod *d)
{
	static const u8 ce[2] = { 0x00, 0x00 };
	static const u8 x38[2] = { 0x00, 0x03 };
	static const u8 x33[3] = { 0x00, 0x03, 0x3b };
	static const u8 xa8[2] = { 0xb7, 0x1b };

	set_ts_clock(d);
	wr(d, SLVX, 0x00, 0x00);
	wr(d, SLVX, 0x17, 0x06);
	wr(d, SLVT, 0x00, 0x00);
	wr(d, SLVT, 0xa9, 0x00);
	wr(d, SLVT, 0x2c, 0x01);
	wr(d, SLVT, 0x4b, 0x74);
	wr(d, SLVT, 0x49, 0x00);
	wr(d, SLVX, 0x18, 0x00);

	wr(d, SLVT, 0x00, 0x11);
	wr(d, SLVT, 0x6a, 0x50);
	wr(d, SLVT, 0x00, 0x10);
	wr(d, SLVT, 0xa5, 0x01);
	wr(d, SLVT, 0x00, 0x00);
	put(d, SLVT, 0xce, ce, sizeof(ce));
	wr(d, SLVT, 0x00, 0x10);
	wr(d, SLVT, 0x69, 0x04);
	wr(d, SLVT, 0x6b, 0x03);
	wr(d, SLVT, 0x9d, 0x50);
	wr(d, SLVT, 0xd3, 0x06);
	wr(d, SLVT, 0xed, 0x00);
	wr(d, SLVT, 0xe2, 0xce);
	wr(d, SLVT, 0xf2, 0x13);
	wr(d, SLVT, 0xde, 0x2e);
	wr(d, SLVT, 0x00, 0x15);
	wr(d, SLVT, 0xde, 0x02);
	wr(d, SLVT, 0x00, 0x17);
	put(d, SLVT, 0x38, x38, sizeof(x38));
	wr(d, SLVT, 0x00, 0x1e);
	wr(d, SLVT, 0x73, 0x68);
	wr(d, SLVT, 0x00, 0x63);
	wr(d, SLVT, 0x81, 0x00);
	wr(d, SLVT, 0x00, 0x11);
	put(d, SLVT, 0x33, x33, sizeof(x33));
	wr(d, SLVT, 0x00, 0x60);
	put(d, SLVT, 0xa8, xa8, sizeof(xa8));

	isdbt_band_setting(d);
	wr(d, SLVT, 0x00, 0x00);
	setbits(d, SLVT, 0x80, 0x08, 0x1f);
	set_ts_data_hiz(d, false);
}

/* ISDB-C: 6 MHz band setting; it is also used to retune while the demodulator is already active */
static void isdbc_band_setting(struct tbs5530_demod *d)
{
	static const u8 itb_coef[14] = { 0x31, 0xa8, 0x29, 0x9b, 0x27, 0x9c, 0x28,
					 0x9e, 0x29, 0xa4, 0x29, 0xa2, 0x29, 0xa8 };
	static const u8 iffreq[3] = { (IFFREQ_ISDBC_6 >> 16) & 0xff, (IFFREQ_ISDBC_6 >> 8) & 0xff,
				      IFFREQ_ISDBC_6 & 0xff };
	/* symbol rate search range around 5.274 Msym/s */
	static const u8 symrate[4] = { 0x07, 0x33, 0x00, 0xee };

	wr(d, SLVT, 0x00, 0x10);
	put(d, SLVT, 0xa6, itb_coef, sizeof(itb_coef));
	put(d, SLVT, 0xb6, iffreq, sizeof(iffreq));
	setbits(d, SLVT, 0xd2, 0x09, 0x1f); /* demodulator IFAGC */
	wr(d, SLVT, 0x00, 0x11);
	wr(d, SLVT, 0xa3, 0x14); /* sampling for 6 MHz */
	wr(d, SLVT, 0x00, 0x40);
	put(d, SLVT, 0x26, symrate, sizeof(symrate));
	wr(d, SLVT, 0x14, 0x1f); /* follow every QAM */
	wr(d, SLVT, 0x1e, 0x00); /* the TSMF is not separated by the demodulator: the TS is passed through */
}

/* sleep to active (ISDB-C) */
static void wake_isdbc(struct tbs5530_demod *d)
{
	static const u8 x33[3] = { 0x00, 0x03, 0x3b };
	static const u8 ce[2] = { 0x01, 0x01 };

	set_ts_clock(d);
	wr(d, SLVX, 0x00, 0x00);
	wr(d, SLVX, 0x17, 0x04);
	wr(d, SLVT, 0x00, 0x00);
	wr(d, SLVT, 0xa9, 0x00);
	wr(d, SLVT, 0x2c, 0x01);
	wr(d, SLVT, 0x4b, 0x74);
	wr(d, SLVT, 0x49, 0x00);
	wr(d, SLVX, 0x18, 0x00);

	wr(d, SLVT, 0x00, 0x11);
	wr(d, SLVT, 0x6a, 0x48);
	wr(d, SLVT, 0x00, 0x10);
	wr(d, SLVT, 0xa5, 0x01);
	wr(d, SLVT, 0x00, 0x11);
	put(d, SLVT, 0x33, x33, sizeof(x33));
	wr(d, SLVT, 0x00, 0x48);
	wr(d, SLVT, 0x2c, 0x03);
	wr(d, SLVT, 0xac, 0x00); /* a Reed-Solomon error does not mute the TS output */
	wr(d, SLVT, 0x00, 0x00);
	put(d, SLVT, 0xce, ce, sizeof(ce));

	isdbc_band_setting(d);
	wr(d, SLVT, 0x00, 0x00);
	setbits(d, SLVT, 0x80, 0x08, 0x1f);
	set_ts_data_hiz(d, false);
}

/*
 * Freia tuner settings that differ between the systems. RF_GAIN is auto, IF_BPF_F0 is 0 and the
 * bandwidth is 6 MHz for both. RFOVLD_DET_LV1 and IFOVLD_DET_LV are the same for every band.
 */
struct freia_param {
	u8 x3c[3];	/* registers 0x3c..0x3e */
	u8 r52;
	u8 r8b;
	u8 if_bpf_gc;
	u8 rfovld;
	u8 ifovld;
	u8 fif_offset;	/* 5 bit */
	u8 bw_offset;	/* 5 bit */
};

/* ISDB-T 6 MHz entry of the vendor's table: IF_BPF_GC 0x06, RFOVLD 0x0e, IFOVLD 0x03, FIF_OFFSET -9, BW_OFFSET -5 */
static const struct freia_param freia_isdbt = { { 0xfc, 0x9c, 0x8f }, 0x00, 0x75, 0x06, 0x0e, 0x03, 0x17, 0x1b };

/*
 * ISDB-C: the vendor uses its DVB-C 6 MHz entry (IF_BPF_GC 0x03, RFOVLD 0x0b, IFOVLD 0x03, FIF_OFFSET -6,
 * BW_OFFSET -4) and the cable variant of registers 0x3c, 0x52 and 0x8b.
 */
static const struct freia_param freia_isdbc = { { 0xfc, 0x87, 0xcf }, 0x78, 0x05, 0x03, 0x0b, 0x03, 0x1a, 0x1c };

/* Freia tuner tuning for a 6 MHz channel; the I2C repeater must be enabled */
static void freia_tune(struct tbs5530_demod *d, u32 khz, const struct freia_param *p)
{
	static const u8 x87[2] = { 0x84, 0x40 };
	static const u8 x91[2] = { 0x0a, 0x0c };
	static const u8 x9c[2] = { 0x8c, 0x01 };
	static const u8 x5e[3] = { 0x6e, 0x02, 0x9e };
	static const u8 x61[6] = { 0x66, 0x01, 0x38, 0x1e, 0x02, 0x24 };
	u8 t[17];

	wr(d, TUNER, 0x01, 0x00);
	wr(d, TUNER, 0x74, 0x02);
	put(d, TUNER, 0x87, x87, sizeof(x87));
	put(d, TUNER, 0x3c, p->x3c, sizeof(p->x3c));
	wr(d, TUNER, 0x52, p->r52);
	wr(d, TUNER, 0x8b, p->r8b);
	wr(d, TUNER, 0x8d, 0x01);
	put(d, TUNER, 0x91, x91, sizeof(x91));
	put(d, TUNER, 0x9c, x9c, sizeof(x9c));
	/* enable the analog block */
	wr(d, TUNER, 0x7c, 0x01);
	put(d, TUNER, 0x5e, x5e, sizeof(x5e));
	wr(d, TUNER, 0x7c, 0x00);
	wr(d, TUNER, 0x1e, 0xa4);
	wr(d, TUNER, 0x5e, 0xee);
	put(d, TUNER, 0x61, x61, sizeof(x61));
	setbits(d, TUNER, 0x67, 0x00, 0x02);

	/* registers 0x68..0x78 */
	t[0] = 0x00;
	t[1] = 0x80 | p->if_bpf_gc; /* RF gain auto | IF_BPF_GC */
	t[2] = 0x00;
	t[3] = p->rfovld; /* RFOVLD_DET_LV1, the same for every band */
	t[4] = p->ifovld | 0x30; /* IFOVLD_DET_LV | not DVB-T/T2 */
	t[5] = 0x00; /* IF_BPF_F0 | BW (6 MHz) */
	t[6] = p->fif_offset;
	t[7] = p->bw_offset;
	t[8] = (u8)(khz & 0xff);
	t[9] = (u8)((khz >> 8) & 0xff);
	t[10] = (u8)((khz >> 16) & 0x1f);
	t[11] = 0xff;
	t[12] = 0x00;
	t[13] = 0xf1;
	t[14] = 0x0f;
	t[15] = 0x06;
	t[16] = 0x03;
	put(d, TUNER, 0x68, t, sizeof(t));
	delay(d, 50);
	wr(d, TUNER, 0x88, 0x00);
	wr(d, TUNER, 0x87, 0x80);
}

/* cancel the ISDB-T setting */
static void cancel_isdbt(struct tbs5530_demod *d)
{
	static const u8 x38[2] = { 0x01, 0x02 };

	wr(d, SLVT, 0x00, 0x10);
	wr(d, SLVT, 0x69, 0x05);
	wr(d, SLVT, 0x6b, 0x07);
	wr(d, SLVT, 0x9d, 0x14);
	wr(d, SLVT, 0xd3, 0x00);
	wr(d, SLVT, 0xed, 0x01);
	wr(d, SLVT, 0xe2, 0x4e);
	wr(d, SLVT, 0xf2, 0x03);
	wr(d, SLVT, 0xde, 0x32);
	wr(d, SLVT, 0x00, 0x15);
	wr(d, SLVT, 0xde, 0x03);
	wr(d, SLVT, 0x00, 0x17);
	put(d, SLVT, 0x38, x38, sizeof(x38));
	wr(d, SLVT, 0x00, 0x1e);
	wr(d, SLVT, 0x73, 0x00);
	wr(d, SLVT, 0x00, 0x63);
	wr(d, SLVT, 0x81, 0x01);
}

/* cancel the ISDB-C setting */
static void cancel_isdbc(struct tbs5530_demod *d)
{
	wr(d, SLVT, 0x00, 0x11);
	wr(d, SLVT, 0xa3, 0x00);
	wr(d, SLVT, 0x00, 0x48);
	wr(d, SLVT, 0x2c, 0x01);
	wr(d, SLVT, 0x00, 0x40);
	wr(d, SLVT, 0x14, 0x1f);
	wr(d, SLVT, 0x1e, 0x00);
}

int tbs5530_demod_sleep(struct tbs5530_demod *d)
{
	if (!d->warm)
		return -ENODEV;
	if (d->state == TBS5530_DEMOD_ACTIVE) {
		set_stream_output(d, false);
		wr(d, SLVT, 0x00, 0x00);
		setbits(d, SLVT, 0x80, 0x1f, 0x1f);
		set_ts_data_hiz(d, true);
		if (d->isdbc)
			cancel_isdbc(d);
		else
			cancel_isdbt(d);
	}
	wr(d, SLVX, 0x00, 0x00);
	wr(d, SLVX, 0x18, 0x01); /* TADC */
	wr(d, SLVT, 0x00, 0x00);
	wr(d, SLVT, 0x49, 0x33); /* TADC */
	wr(d, SLVT, 0x4b, 0x21); /* TADC */
	wr(d, SLVT, 0xfe, 0x01); /* demodulator software reset */
	wr(d, SLVT, 0x2c, 0x00); /* demodulator clock off */
	wr(d, SLVT, 0xa9, 0x00); /* tstlv mode to default */
	wr(d, SLVX, 0x17, 0x01); /* demodulator mode to default */
	if (finish(d))
		return -EIO;
	d->state = TBS5530_DEMOD_SLEEP;
	d->isdbt = false;
	d->isdbc = false;
	return 0;
}

static int demod_tune(struct tbs5530_demod *d, u32 frequency_khz, bool isdbc)
{
	int ret;

	if (!d->warm) {
		ret = tbs5530_demod_init(d);
		if (ret)
			return ret;
	}
	/* the Freia tuner covers 42 - 1002 MHz */
	if (frequency_khz < 42000 || frequency_khz > 1002000)
		return -EINVAL;

	if (d->state == TBS5530_DEMOD_ACTIVE && (isdbc ? d->isdbc : d->isdbt)) {
		wr(d, SLVT, 0x00, 0x00);
		wr(d, SLVT, 0xc3, 0x01); /* stop the TS output while retuning */
		if (isdbc) {
			wr(d, SLVT, 0x00, 0x48);
			wr(d, SLVT, 0xac, 0x00);
			isdbc_band_setting(d);
		} else {
			isdbt_band_setting(d);
		}
	} else {
		if (d->state == TBS5530_DEMOD_ACTIVE) {
			ret = tbs5530_demod_sleep(d);
			if (ret)
				return ret;
		}
		if (isdbc)
			wake_isdbc(d);
		else
			wake_isdbt(d);
	}
	ret = finish(d);
	if (ret)
		return ret;
	d->state = TBS5530_DEMOD_ACTIVE;
	d->isdbt = !isdbc;
	d->isdbc = isdbc;

	i2c_repeater(d, true);
	freia_tune(d, frequency_khz, isdbc ? &freia_isdbc : &freia_isdbt);
	i2c_repeater(d, false);

	/* tune end: reset the demodulator and enable the TS output */
	wr(d, SLVT, 0x00, 0x00);
	wr(d, SLVT, 0xfe, 0x01);
	set_stream_output(d, true);
	ret = finish(d);
	if (ret)
		return ret;
	sleep_ms(d->b, 20);
	return 0;
}

int tbs5530_demod_tune_isdbt(struct tbs5530_demod *d, u32 frequency_khz)
{
	return demod_tune(d, frequency_khz, false);
}

int tbs5530_demod_tune_isdbc(struct tbs5530_demod *d, u32 frequency_khz)
{
	return demod_tune(d, frequency_khz, true);
}

static void lock_led(struct tbs5530_demod *d, bool locked)
{
	if (d->led_locked == locked)
		return;
	bank_setbits(d, SLVX, 0x00, 0xa5, 0x00, 0x0f);
	bank_setbits(d, SLVX, 0x00, 0x82, 0x01, 0x04);
	bank_setbits(d, SLVX, 0x00, 0xa2, locked ? 0x07 : 0x00, 0x04);
	if (!d->b->err)
		d->led_locked = locked;
}

int tbs5530_demod_read_status(struct tbs5530_demod *d, u8 *status)
{
	u8 v, st = 0;
	bool locked;

	if (!d->warm || d->state != TBS5530_DEMOD_ACTIVE || (!d->isdbt && !d->isdbc))
		return -ENODEV;
	if (d->isdbc) {
		u8 agc;

		wr(d, SLVT, 0x00, 0x40);
		agc = rd(d, SLVT, 0x88);
		v = rd(d, SLVT, 0x10);
		if (finish(d))
			return -EIO;
		/* register 0x88 bit0: the AGC has converged (a carrier), 0x10 bit5: synchronised */
		locked = (agc & 0x01) && (v & 0x20);
		if (locked)
			st = TBS5530_STATUS_SIGNAL | TBS5530_STATUS_LOCK;
		else if (agc & 0x01)
			st = TBS5530_STATUS_SIGNAL;
	} else {
		wr(d, SLVT, 0x00, 0x60);
		v = rd(d, SLVT, 0x10);
		if (finish(d))
			return -EIO;
		/* bit4: unlock detected, bit1: synchronised, bit0: TS locked */
		locked = !(v & 0x10) && (v & 0x02) && (v & 0x01);
		if (locked)
			st = TBS5530_STATUS_SIGNAL | TBS5530_STATUS_LOCK;
		else if (v & 0x03)
			st = TBS5530_STATUS_SIGNAL;
	}
	lock_led(d, locked);
	if (finish(d))
		return -EIO;
	*status = st;
	return 0;
}

int tbs5530_demod_read_cnr(struct tbs5530_demod *d, s32 *cnr_mdb)
{
	u8 buf[2];
	u16 reg;
	int ret;

	if (!d->warm || d->state != TBS5530_DEMOD_ACTIVE || (!d->isdbt && !d->isdbc))
		return -ENODEV;
	/* the ISDB-C conversion is not confirmed: no C/N is reported rather than a wrong one */
	if (d->isdbc)
		return -ENOSYS;
	wr(d, SLVT, 0x00, 0x60);
	if (finish(d))
		return -EIO;
	ret = tbs5530_i2c_read(d->b, SLVT, 0x28, buf, sizeof(buf));
	if (ret)
		return ret;
	reg = (u16)((buf[0] << 8) | buf[1]);
	if (reg == 0)
		return -EAGAIN; /* no measurement yet */
	*cnr_mdb = 100 * (s32)tbs5530_log10_x100(reg) - 9031 - CNR_OFFSET_MDB;
	return 0;
}
