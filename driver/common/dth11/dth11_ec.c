// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_ec.c - secp160r1 の楕円曲線演算 (dth11_ec.h)
 *
 * 曲線: y^2 = x^3 + a x + b (mod p)、a = p - 3 (== -3)。点の加算と 2 倍は Jacobian 座標で行い、
 * 最後に 1 回だけ逆元を取って affine に戻す。逆元は p と n が素数なので、フェルマーの小定理による。
 */

#include "dth11_ec.h"
#include "dth11_crypto.h"

#define LIMBS 6 /* 192 ビット。n は 161 ビットで、積の途中も 2 倍と加算しか行わないため収まる */

struct bn {
	u32 w[LIMBS]; /* 下位が先 */
};

/* secp160r1 のパラメーター (SEC 2) */
static const u8 P_BYTES[20] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff };
static const u8 B_BYTES[20] = { 0x1c, 0x97, 0xbe, 0xfc, 0x54, 0xbd, 0x7a, 0x8b, 0x65, 0xac,
				0xf8, 0x9f, 0x81, 0xd4, 0xd4, 0xad, 0xc5, 0x65, 0xfa, 0x45 };
static const u8 GX_BYTES[20] = { 0x4a, 0x96, 0xb5, 0x68, 0x8e, 0xf5, 0x73, 0x28, 0x46, 0x64,
				 0x69, 0x89, 0x68, 0xc3, 0x8b, 0xb9, 0x13, 0xcb, 0xfc, 0x82 };
static const u8 GY_BYTES[20] = { 0x23, 0xa6, 0x28, 0x55, 0x31, 0x68, 0x94, 0x7d, 0x59, 0xdc,
				 0xc9, 0x12, 0x04, 0x23, 0x51, 0x37, 0x7a, 0xc5, 0xfb, 0x32 };
/* 位数 n = 0x0100000000000000000001F4C8F927AED3CA752257 (21 バイト) */
static const u8 N_BYTES[21] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
				0xf4, 0xc8, 0xf9, 0x27, 0xae, 0xd3, 0xca, 0x75, 0x22, 0x57 };

static struct bn P, N, B, GX, GY;
static int params_ready;

/* ---- 多倍長整数 ---- */

static void bn_from_be(struct bn *r, const u8 *b, size_t n)
{
	size_t i;

	memset(r, 0, sizeof(*r));
	for (i = 0; i < n; i++) {
		size_t bit = 8 * (n - 1 - i);

		r->w[bit / 32] |= (u32)b[i] << (bit % 32);
	}
}

static void bn_to_be20(const struct bn *a, u8 out[20])
{
	int i;

	for (i = 0; i < 20; i++) {
		size_t bit = 8 * (size_t)(19 - i);

		out[i] = (u8)(a->w[bit / 32] >> (bit % 32));
	}
}

static void bn_set_u32(struct bn *r, u32 v)
{
	memset(r, 0, sizeof(*r));
	r->w[0] = v;
}

static bool bn_is_zero(const struct bn *a)
{
	int i;

	for (i = 0; i < LIMBS; i++)
		if (a->w[i])
			return false;
	return true;
}

static int bn_cmp(const struct bn *a, const struct bn *b)
{
	int i;

	for (i = LIMBS - 1; i >= 0; i--) {
		if (a->w[i] > b->w[i])
			return 1;
		if (a->w[i] < b->w[i])
			return -1;
	}
	return 0;
}

static u32 bn_add(struct bn *r, const struct bn *a, const struct bn *b)
{
	u64 carry = 0;
	int i;

	for (i = 0; i < LIMBS; i++) {
		u64 s = (u64)a->w[i] + b->w[i] + carry;

		r->w[i] = (u32)s;
		carry = s >> 32;
	}
	return (u32)carry;
}

static u32 bn_sub(struct bn *r, const struct bn *a, const struct bn *b)
{
	u64 borrow = 0;
	int i;

	for (i = 0; i < LIMBS; i++) {
		u64 d = (u64)a->w[i] - b->w[i] - borrow;

		r->w[i] = (u32)d;
		borrow = (d >> 32) & 1;
	}
	return (u32)borrow;
}

static int bn_bits(const struct bn *a)
{
	int i, b;

	for (i = LIMBS - 1; i >= 0; i--) {
		if (!a->w[i])
			continue;
		for (b = 31; b >= 0; b--)
			if (a->w[i] & ((u32)1 << b))
				return 32 * i + b + 1;
	}
	return 0;
}

static bool bn_bit(const struct bn *a, int i)
{
	return (a->w[i / 32] >> (i % 32)) & 1;
}

/* ---- 法 m の演算。入力は 0..m-1 の範囲 ---- */

static void mod_add(struct bn *r, const struct bn *a, const struct bn *b, const struct bn *m)
{
	u32 carry = bn_add(r, a, b);

	if (carry || bn_cmp(r, m) >= 0)
		bn_sub(r, r, m);
}

static void mod_sub(struct bn *r, const struct bn *a, const struct bn *b, const struct bn *m)
{
	if (bn_sub(r, a, b))
		bn_add(r, r, m);
}

/* 積。b の上位ビットから、2 倍して (必要なら) a を足す。値は常に 0..m-1 に保たれる */
static void mod_mul(struct bn *r, const struct bn *a, const struct bn *b, const struct bn *m)
{
	struct bn acc;
	int i;

	memset(&acc, 0, sizeof(acc));
	for (i = bn_bits(b) - 1; i >= 0; i--) {
		mod_add(&acc, &acc, &acc, m);
		if (bn_bit(b, i))
			mod_add(&acc, &acc, a, m);
	}
	*r = acc;
}

static void mod_sqr(struct bn *r, const struct bn *a, const struct bn *m)
{
	mod_mul(r, a, a, m);
}

/* a の逆元。m は素数で、フェルマーの小定理により a^(m-2)。a が 0 のときは 0 */
static void mod_inv(struct bn *r, const struct bn *a, const struct bn *m)
{
	struct bn e, two, result;
	int i;

	bn_set_u32(&two, 2);
	bn_sub(&e, m, &two);
	bn_set_u32(&result, 1);
	for (i = bn_bits(&e) - 1; i >= 0; i--) {
		mod_sqr(&result, &result, m);
		if (bn_bit(&e, i))
			mod_mul(&result, &result, a, m);
	}
	*r = result;
}

static void params_init(void)
{
	if (params_ready)
		return;
	bn_from_be(&P, P_BYTES, 20);
	bn_from_be(&N, N_BYTES, 21);
	bn_from_be(&B, B_BYTES, 20);
	bn_from_be(&GX, GX_BYTES, 20);
	bn_from_be(&GY, GY_BYTES, 20);
	params_ready = 1;
}

/* ---- 点 (Jacobian 座標。z == 0 が無限遠点) ---- */

struct jpt {
	struct bn x, y, z;
};

struct apt {
	struct bn x, y;
	bool inf;
};

static void jac_set_inf(struct jpt *r)
{
	bn_set_u32(&r->x, 1);
	bn_set_u32(&r->y, 1);
	memset(&r->z, 0, sizeof(r->z));
}

static void jac_from_affine(struct jpt *r, const struct apt *a)
{
	if (a->inf) {
		jac_set_inf(r);
		return;
	}
	r->x = a->x;
	r->y = a->y;
	bn_set_u32(&r->z, 1);
}

static void jac_to_affine(struct apt *r, const struct jpt *j)
{
	struct bn zi, zi2, zi3;

	if (bn_is_zero(&j->z)) {
		memset(r, 0, sizeof(*r));
		r->inf = true;
		return;
	}
	mod_inv(&zi, &j->z, &P);
	mod_sqr(&zi2, &zi, &P);
	mod_mul(&zi3, &zi2, &zi, &P);
	mod_mul(&r->x, &j->x, &zi2, &P);
	mod_mul(&r->y, &j->y, &zi3, &P);
	r->inf = false;
}

/* 2 倍 (a = -3 用の式) */
static void jac_double(struct jpt *r, const struct jpt *p)
{
	struct bn delta, gamma, beta, alpha, t, u, x3, y3, z3;

	if (bn_is_zero(&p->z) || bn_is_zero(&p->y)) {
		jac_set_inf(r);
		return;
	}
	mod_sqr(&delta, &p->z, &P);
	mod_sqr(&gamma, &p->y, &P);
	mod_mul(&beta, &p->x, &gamma, &P);
	/* alpha = 3 * (X - delta) * (X + delta) */
	mod_sub(&t, &p->x, &delta, &P);
	mod_add(&u, &p->x, &delta, &P);
	mod_mul(&alpha, &t, &u, &P);
	mod_add(&t, &alpha, &alpha, &P);
	mod_add(&alpha, &t, &alpha, &P);
	/* X3 = alpha^2 - 8 * beta */
	mod_sqr(&x3, &alpha, &P);
	mod_add(&t, &beta, &beta, &P);
	mod_add(&t, &t, &t, &P);
	mod_add(&u, &t, &t, &P); /* 8 * beta */
	mod_sub(&x3, &x3, &u, &P);
	/* Z3 = (Y + Z)^2 - gamma - delta */
	mod_add(&z3, &p->y, &p->z, &P);
	mod_sqr(&z3, &z3, &P);
	mod_sub(&z3, &z3, &gamma, &P);
	mod_sub(&z3, &z3, &delta, &P);
	/* Y3 = alpha * (4 * beta - X3) - 8 * gamma^2。t は 4 * beta */
	mod_sub(&y3, &t, &x3, &P);
	mod_mul(&y3, &alpha, &y3, &P);
	mod_sqr(&u, &gamma, &P);
	mod_add(&u, &u, &u, &P);
	mod_add(&u, &u, &u, &P);
	mod_add(&u, &u, &u, &P); /* 8 * gamma^2 */
	mod_sub(&y3, &y3, &u, &P);
	r->x = x3;
	r->y = y3;
	r->z = z3;
}

/* Jacobian の点 p に affine の点 q を足す */
static void jac_add_affine(struct jpt *r, const struct jpt *p, const struct apt *q)
{
	struct bn z1z1, u2, s2, h, hh, i, j, rr, v, x3, y3, z3, t;

	if (q->inf) {
		*r = *p;
		return;
	}
	if (bn_is_zero(&p->z)) {
		jac_from_affine(r, q);
		return;
	}
	mod_sqr(&z1z1, &p->z, &P);
	mod_mul(&u2, &q->x, &z1z1, &P);
	mod_mul(&s2, &q->y, &p->z, &P);
	mod_mul(&s2, &s2, &z1z1, &P);
	mod_sub(&h, &u2, &p->x, &P);
	if (bn_is_zero(&h)) {
		if (!bn_cmp(&s2, &p->y)) {
			/* 同じ点 */
			jac_double(r, p);
		} else {
			/* 逆の点 */
			jac_set_inf(r);
		}
		return;
	}
	mod_sqr(&hh, &h, &P);
	mod_add(&i, &hh, &hh, &P);
	mod_add(&i, &i, &i, &P);
	mod_mul(&j, &h, &i, &P);
	mod_sub(&rr, &s2, &p->y, &P);
	mod_add(&rr, &rr, &rr, &P);
	mod_mul(&v, &p->x, &i, &P);
	/* X3 = r^2 - J - 2V */
	mod_sqr(&x3, &rr, &P);
	mod_sub(&x3, &x3, &j, &P);
	mod_sub(&x3, &x3, &v, &P);
	mod_sub(&x3, &x3, &v, &P);
	/* Y3 = r * (V - X3) - 2 * Y1 * J */
	mod_sub(&y3, &v, &x3, &P);
	mod_mul(&y3, &rr, &y3, &P);
	mod_mul(&t, &p->y, &j, &P);
	mod_add(&t, &t, &t, &P);
	mod_sub(&y3, &y3, &t, &P);
	/* Z3 = (Z1 + H)^2 - Z1Z1 - HH */
	mod_add(&z3, &p->z, &h, &P);
	mod_sqr(&z3, &z3, &P);
	mod_sub(&z3, &z3, &z1z1, &P);
	mod_sub(&z3, &z3, &hh, &P);
	r->x = x3;
	r->y = y3;
	r->z = z3;
}

/* k * Q (affine)。k の上位ビットから 2 倍と加算を繰り返す */
static void scalar_mul(struct apt *out, const struct bn *k, const struct apt *q)
{
	struct jpt acc;
	int i;

	jac_set_inf(&acc);
	for (i = bn_bits(k) - 1; i >= 0; i--) {
		jac_double(&acc, &acc);
		if (bn_bit(k, i))
			jac_add_affine(&acc, &acc, q);
	}
	jac_to_affine(out, &acc);
}

static void apt_add(struct apt *out, const struct apt *a, const struct apt *b)
{
	struct jpt j;

	jac_from_affine(&j, a);
	jac_add_affine(&j, &j, b);
	jac_to_affine(out, &j);
}

static void generator(struct apt *g)
{
	g->x = GX;
	g->y = GY;
	g->inf = false;
}

/* y^2 == x^3 - 3x + b (mod p)、x と y は p 未満 */
static bool on_curve(const struct bn *x, const struct bn *y)
{
	struct bn lhs, rhs, x3, three_x;

	if (bn_cmp(x, &P) >= 0 || bn_cmp(y, &P) >= 0)
		return false;
	mod_sqr(&lhs, y, &P);
	mod_sqr(&x3, x, &P);
	mod_mul(&x3, &x3, x, &P);
	mod_add(&three_x, x, x, &P);
	mod_add(&three_x, &three_x, x, &P);
	mod_sub(&rhs, &x3, &three_x, &P);
	mod_add(&rhs, &rhs, &B, &P);
	return !bn_cmp(&lhs, &rhs);
}

static bool apt_from_bytes(struct apt *a, const u8 xy[DTH11_EC_POINT_BYTES])
{
	bn_from_be(&a->x, xy, 20);
	bn_from_be(&a->y, xy + 20, 20);
	a->inf = false;
	return on_curve(&a->x, &a->y);
}

static void apt_to_bytes(const struct apt *a, u8 xy[DTH11_EC_POINT_BYTES])
{
	bn_to_be20(&a->x, xy);
	bn_to_be20(&a->y, xy + 20);
}

/* ---- 公開 API ---- */

int dth11_ec_point_check(const u8 xy[DTH11_EC_POINT_BYTES])
{
	struct apt a;

	params_init();
	return apt_from_bytes(&a, xy) ? 0 : -EINVAL;
}

static bool scalar_valid(const struct bn *d)
{
	return !bn_is_zero(d) && bn_cmp(d, &N) < 0;
}

int dth11_ec_public_key(const u8 d[DTH11_EC_BYTES], u8 out_xy[DTH11_EC_POINT_BYTES])
{
	struct bn k;
	struct apt g, q;

	params_init();
	bn_from_be(&k, d, 20);
	if (!scalar_valid(&k))
		return -EINVAL;
	generator(&g);
	scalar_mul(&q, &k, &g);
	if (q.inf)
		return -EINVAL;
	apt_to_bytes(&q, out_xy);
	return 0;
}

bool dth11_ec_key_matches(const u8 d[DTH11_EC_BYTES], const u8 q_xy[DTH11_EC_POINT_BYTES])
{
	u8 q[DTH11_EC_POINT_BYTES];

	if (dth11_ec_public_key(d, q))
		return false;
	return !memcmp(q, q_xy, sizeof(q));
}

int dth11_ec_ecdh(const u8 d[DTH11_EC_BYTES], const u8 q_xy[DTH11_EC_POINT_BYTES], u8 shared[DTH11_EC_BYTES])
{
	struct bn k;
	struct apt q, r;

	params_init();
	bn_from_be(&k, d, 20);
	if (!scalar_valid(&k) || !apt_from_bytes(&q, q_xy))
		return -EINVAL;
	scalar_mul(&r, &k, &q);
	if (r.inf)
		return -EINVAL;
	bn_to_be20(&r.x, shared);
	return 0;
}

int dth11_ec_random_scalar(dth11_rng_fn rng, void *rng_priv, u8 d[DTH11_EC_BYTES])
{
	struct bn k;
	u8 buf[DTH11_EC_BYTES];
	int tries, r;

	params_init();
	/* 160 ビットの乱数を 1..n-1 に収める。n は 2^160 より少し大きいので、棄却は稀 */
	for (tries = 0; tries < 64; tries++) {
		r = rng(rng_priv, buf, sizeof(buf));
		if (r)
			return r;
		bn_from_be(&k, buf, sizeof(buf));
		if (scalar_valid(&k)) {
			memcpy(d, buf, sizeof(buf));
			return 0;
		}
	}
	return -EIO;
}

int dth11_ec_sign(const u8 d[DTH11_EC_BYTES], const void *msg, size_t len, dth11_rng_fn rng, void *rng_priv,
		  u8 sig[DTH11_EC_SIG_BYTES])
{
	struct bn dd, e, k, r, s, t, kinv;
	struct apt g, kg;
	u8 digest[DTH11_SHA1_SIZE], kb[DTH11_EC_BYTES];
	int tries, ret;

	params_init();
	bn_from_be(&dd, d, 20);
	if (!scalar_valid(&dd))
		return -EINVAL;
	dth11_sha1(msg, len, digest);
	bn_from_be(&e, digest, sizeof(digest));
	/* e は 160 ビットで n より小さい */
	generator(&g);
	for (tries = 0; tries < 64; tries++) {
		ret = dth11_ec_random_scalar(rng, rng_priv, kb);
		if (ret)
			return ret;
		bn_from_be(&k, kb, sizeof(kb));
		scalar_mul(&kg, &k, &g);
		if (kg.inf)
			continue;
		r = kg.x;
		while (bn_cmp(&r, &N) >= 0)
			bn_sub(&r, &r, &N);
		if (bn_is_zero(&r))
			continue;
		/* s = k^-1 * (e + d * r) mod n */
		mod_mul(&t, &dd, &r, &N);
		mod_add(&t, &t, &e, &N);
		mod_inv(&kinv, &k, &N);
		mod_mul(&s, &kinv, &t, &N);
		/* 署名は r と s を 20 バイトずつ並べる。s が 160 ビットに収まらないときは k を取り直す */
		if (bn_is_zero(&s) || bn_bits(&s) > 160 || bn_bits(&r) > 160)
			continue;
		bn_to_be20(&r, sig);
		bn_to_be20(&s, sig + 20);
		return 0;
	}
	return -EIO;
}

bool dth11_ec_verify(const u8 q_xy[DTH11_EC_POINT_BYTES], const void *msg, size_t len,
		     const u8 sig[DTH11_EC_SIG_BYTES])
{
	struct bn r, s, e, w, u1, u2, x;
	struct apt q, g, a, b, sum;
	u8 digest[DTH11_SHA1_SIZE];

	params_init();
	if (!apt_from_bytes(&q, q_xy))
		return false;
	bn_from_be(&r, sig, 20);
	bn_from_be(&s, sig + 20, 20);
	if (bn_is_zero(&r) || bn_is_zero(&s) || bn_cmp(&r, &N) >= 0 || bn_cmp(&s, &N) >= 0)
		return false;
	dth11_sha1(msg, len, digest);
	bn_from_be(&e, digest, sizeof(digest));
	mod_inv(&w, &s, &N);
	mod_mul(&u1, &e, &w, &N);
	mod_mul(&u2, &r, &w, &N);
	generator(&g);
	scalar_mul(&a, &u1, &g);
	scalar_mul(&b, &u2, &q);
	apt_add(&sum, &a, &b);
	if (sum.inf)
		return false;
	x = sum.x;
	while (bn_cmp(&x, &N) >= 0)
		bn_sub(&x, &x, &N);
	return !bn_cmp(&x, &r);
}
