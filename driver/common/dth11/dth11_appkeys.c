// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_appkeys.c - 純正の CoBDACom.dll からの資格情報の読み出し (dth11_appkeys.h)
 */

#include "dth11_appkeys.h"
#include "dth11_crypto.h"
#include "dth11_ec.h"

#include <stdlib.h>

/* ---- PE 画像 ---- */

struct pe_section {
	u32 va, vsize, raw, rsize, characteristics;
	char name[8];
};

struct pe_info {
	u64 image_base;
	struct pe_section *sections;
	size_t nsec;
};

static bool rd16(const u8 *b, size_t len, size_t o, u16 *v)
{
	if (o > len || len - o < 2)
		return false;
	*v = (u16)(b[o] | (b[o + 1] << 8));
	return true;
}

static bool rd32(const u8 *b, size_t len, size_t o, u32 *v)
{
	if (o > len || len - o < 4)
		return false;
	*v = (u32)b[o] | ((u32)b[o + 1] << 8) | ((u32)b[o + 2] << 16) | ((u32)b[o + 3] << 24);
	return true;
}

static bool rd64(const u8 *b, size_t len, size_t o, u64 *v)
{
	u32 lo, hi;

	if (!rd32(b, len, o, &lo) || !rd32(b, len, o + 4, &hi))
		return false;
	*v = (u64)lo | ((u64)hi << 32);
	return true;
}

static bool section_is_code(const struct pe_section *s)
{
	/* IMAGE_SCN_CNT_CODE | IMAGE_SCN_MEM_EXECUTE、または名前が .text */
	return (s->characteristics & (0x20 | 0x20000000)) || !memcmp(s->name, ".text", 5);
}

static u64 pe_image_size(const struct pe_info *pe)
{
	u64 best = 0;
	size_t i;

	for (i = 0; i < pe->nsec; i++) {
		u64 sz = pe->sections[i].vsize > pe->sections[i].rsize ? pe->sections[i].vsize : pe->sections[i].rsize;
		u64 end = (u64)pe->sections[i].va + sz;

		if (end > best)
			best = end;
	}
	return best;
}

static int parse_pe(const u8 *b, size_t len, struct pe_info *pe)
{
	u32 pe_off, v32;
	u16 nsec, opt_size, magic;
	size_t opt, i;

	memset(pe, 0, sizeof(*pe));
	if (len < 2 || b[0] != 'M' || b[1] != 'Z')
		return -EINVAL;
	if (!rd32(b, len, 0x3c, &pe_off) || pe_off > len || len - pe_off < 4 || memcmp(b + pe_off, "PE\0\0", 4))
		return -EINVAL;
	if (!rd16(b, len, pe_off + 6, &nsec) || !rd16(b, len, pe_off + 20, &opt_size))
		return -EINVAL;
	opt = (size_t)pe_off + 24;
	if (!rd16(b, len, opt, &magic))
		return -EINVAL;
	if (magic == 0x10b) {
		if (!rd32(b, len, opt + 28, &v32))
			return -EINVAL;
		pe->image_base = v32;
	} else if (magic == 0x20b) {
		if (!rd64(b, len, opt + 24, &pe->image_base))
			return -EINVAL;
	} else {
		return -EINVAL;
	}
	if (!nsec)
		return -EINVAL;
	pe->sections = calloc(nsec, sizeof(*pe->sections));
	if (!pe->sections)
		return -ENOMEM;
	for (i = 0; i < nsec; i++) {
		size_t o = opt + opt_size + i * 40;
		struct pe_section *s = &pe->sections[i];

		if (o > len || len - o < 40) {
			free(pe->sections);
			pe->sections = NULL;
			return -EINVAL;
		}
		memcpy(s->name, b + o, 8);
		rd32(b, len, o + 8, &s->vsize);
		rd32(b, len, o + 12, &s->va);
		rd32(b, len, o + 16, &s->rsize);
		rd32(b, len, o + 20, &s->raw);
		rd32(b, len, o + 36, &s->characteristics);
	}
	pe->nsec = nsec;
	return 0;
}

/* ---- x86 の格納命令の走査 ---- */

enum target_kind { TARGET_ABS, TARGET_STACK };

struct store {
	size_t pos; /* 命令のコード内の位置 */
	enum target_kind kind;
	u64 abs; /* TARGET_ABS のとき絶対アドレス */
	u8 base; /* TARGET_STACK のとき 4 (esp) か 5 (ebp) */
	s32 disp;
	u8 bytes[4];
	u8 len;
};

struct store_list {
	struct store *v;
	size_t n, cap;
};

static int store_push(struct store_list *l, const struct store *s)
{
	if (l->n == l->cap) {
		size_t cap = l->cap ? l->cap * 2 : 4096;
		struct store *nv = realloc(l->v, cap * sizeof(*nv));

		if (!nv)
			return -ENOMEM;
		l->v = nv;
		l->cap = cap;
	}
	l->v[l->n++] = *s;
	return 0;
}

/*
 * レジスタに直前に代入された即値。コードの間の命令は復号しないので、代入からこの距離
 * (コードのバイト数) までは値が保たれているとみなす。純正のコードは、繰り返し現れるバイト値を
 * レジスタに持ったまま 1000 バイト以上の格納列を並べるため、1 つの関数に収まる幅を取る。
 */
#define REG_FRESH 8192

struct reg_slot {
	bool ok;
	u32 v;
	size_t pos;
};

struct regs {
	struct reg_slot r8[8], r16[8], r32[8];
};

static void regs_set8(struct regs *r, int i, u8 v, size_t pos)
{
	r->r8[i] = (struct reg_slot){ true, v, pos };
	r->r16[i & 3].ok = false;
	r->r32[i & 3].ok = false;
}

static void regs_set16(struct regs *r, int i, u16 v, size_t pos)
{
	r->r16[i] = (struct reg_slot){ true, v, pos };
	r->r32[i].ok = false;
	if (i < 4) {
		r->r8[i] = (struct reg_slot){ true, (u8)v, pos };
		r->r8[i + 4] = (struct reg_slot){ true, (u8)(v >> 8), pos };
	}
}

static void regs_set32(struct regs *r, int i, u32 v, size_t pos)
{
	r->r32[i] = (struct reg_slot){ true, v, pos };
	r->r16[i] = (struct reg_slot){ true, (u16)v, pos };
	if (i < 4) {
		r->r8[i] = (struct reg_slot){ true, (u8)v, pos };
		r->r8[i + 4] = (struct reg_slot){ true, (u8)(v >> 8), pos };
	}
}

static bool reg_fresh(const struct reg_slot *s, size_t pos, u32 *v)
{
	if (!s->ok)
		return false;
	if (pos > s->pos && pos - s->pos > REG_FRESH)
		return false;
	*v = s->v;
	return true;
}

/* ModRM (と SIB) のメモリ参照。対象にするのは [disp32]、[ebp+disp]、SIB で index なしの [esp+disp]、[ebp+disp]、[disp32] */
static bool modrm_target(const u8 *code, size_t len, size_t i, struct store *t, size_t *consumed)
{
	u8 b, sib, base;
	u32 d32;

	if (i >= len)
		return false;
	b = code[i];
	switch ((b >> 6) * 8 + (b & 7)) {
	case 0 * 8 + 5:
		if (!rd32(code, len, i + 1, &d32))
			return false;
		t->kind = TARGET_ABS;
		t->abs = d32;
		*consumed = 5;
		return true;
	case 1 * 8 + 5:
		if (i + 1 >= len)
			return false;
		t->kind = TARGET_STACK;
		t->base = 5;
		t->disp = (s32)(signed char)code[i + 1];
		*consumed = 2;
		return true;
	case 2 * 8 + 5:
		if (!rd32(code, len, i + 1, &d32))
			return false;
		t->kind = TARGET_STACK;
		t->base = 5;
		t->disp = (s32)d32;
		*consumed = 5;
		return true;
	default:
		break;
	}
	if ((b & 7) != 4)
		return false;
	if (i + 1 >= len)
		return false;
	sib = code[i + 1];
	base = sib & 7;
	/* index が 4 (なし) でない参照は、単純なスロットではない */
	if (((sib >> 3) & 7) != 4)
		return false;
	switch (b >> 6) {
	case 0:
		if (base == 5) {
			if (!rd32(code, len, i + 2, &d32))
				return false;
			t->kind = TARGET_ABS;
			t->abs = d32;
			*consumed = 6;
			return true;
		}
		if (base == 4) {
			t->kind = TARGET_STACK;
			t->base = 4;
			t->disp = 0;
			*consumed = 2;
			return true;
		}
		return false;
	case 1:
		if ((base != 4 && base != 5) || i + 2 >= len)
			return false;
		t->kind = TARGET_STACK;
		t->base = base;
		t->disp = (s32)(signed char)code[i + 2];
		*consumed = 3;
		return true;
	case 2:
		if ((base != 4 && base != 5) || !rd32(code, len, i + 2, &d32))
			return false;
		t->kind = TARGET_STACK;
		t->base = base;
		t->disp = (s32)d32;
		*consumed = 6;
		return true;
	default:
		return false;
	}
}

static int modrm_reg(const u8 *code, size_t i)
{
	return (code[i] >> 3) & 7;
}

static bool plausible(const struct store *t)
{
	if (t->kind == TARGET_ABS)
		return true;
	return t->disp > -0x10000 && t->disp < 0x10000;
}

static int emit_store(struct store_list *l, size_t pos, const struct store *tgt, u32 v, u8 len)
{
	struct store s = *tgt;

	if (!plausible(&s))
		return 0;
	s.pos = pos;
	s.len = len;
	s.bytes[0] = (u8)v;
	s.bytes[1] = (u8)(v >> 8);
	s.bytes[2] = (u8)(v >> 16);
	s.bytes[3] = (u8)(v >> 24);
	return store_push(l, &s);
}

/* コードの中の、認識できる即値の格納命令を、コードの順に全て集める */
static int scan_stores(const u8 *code, size_t len, struct store_list *out)
{
	struct regs regs;
	size_t i = 0;
	int ret;

	memset(&regs, 0, sizeof(regs));
	while (i < len) {
		u8 op = code[i];
		size_t step = 1;
		struct store t;
		size_t n;
		u32 v, a;
		u16 v16;

		memset(&t, 0, sizeof(t));
		/* mov r8, imm8 (B0..B7)、mov r32, imm32 (B8..BF)。範囲 case は MSVC が受けないため、先に条件で処理する */
		if (op >= 0xb0 && op <= 0xb7) {
			if (i + 1 < len) {
				regs_set8(&regs, op - 0xb0, code[i + 1], i);
				step = 2;
			}
			i += step;
			continue;
		}
		if (op >= 0xb8 && op <= 0xbf) {
			if (rd32(code, len, i + 1, &v)) {
				regs_set32(&regs, op - 0xb8, v, i);
				step = 5;
			}
			i += step;
			continue;
		}
		if (op == 0x66 && i + 1 < len && code[i + 1] >= 0xb8 && code[i + 1] <= 0xbf) {
			/* mov r16, imm16 */
			if (rd16(code, len, i + 2, &v16)) {
				regs_set16(&regs, code[i + 1] - 0xb8, v16, i);
				step = 4;
			}
			i += step;
			continue;
		}
		switch (op) {
		case 0x66:
			if (i + 1 >= len)
				break;
			switch (code[i + 1]) {
			case 0xc7:
				if (modrm_target(code, len, i + 2, &t, &n) && modrm_reg(code, i + 2) == 0 &&
				    rd16(code, len, i + 2 + n, &v16)) {
					ret = emit_store(out, i, &t, v16, 2);
					if (ret)
						return ret;
					step = 2 + n + 2;
				}
				break;
			case 0x89:
				if (modrm_target(code, len, i + 2, &t, &n) &&
				    reg_fresh(&regs.r16[modrm_reg(code, i + 2)], i, &v)) {
					ret = emit_store(out, i, &t, v, 2);
					if (ret)
						return ret;
					step = 2 + n;
				}
				break;
			case 0xa3:
				if (rd32(code, len, i + 2, &a) && reg_fresh(&regs.r16[0], i, &v)) {
					t.kind = TARGET_ABS;
					t.abs = a;
					ret = emit_store(out, i, &t, v, 2);
					if (ret)
						return ret;
					step = 6;
				}
				break;
			default:
				break;
			}
			break;
		case 0xc6:
			if (modrm_target(code, len, i + 1, &t, &n) && modrm_reg(code, i + 1) == 0 && i + 1 + n < len) {
				ret = emit_store(out, i, &t, code[i + 1 + n], 1);
				if (ret)
					return ret;
				step = 1 + n + 1;
			}
			break;
		case 0xc7:
			if (modrm_target(code, len, i + 1, &t, &n) && modrm_reg(code, i + 1) == 0 &&
			    rd32(code, len, i + 1 + n, &v)) {
				ret = emit_store(out, i, &t, v, 4);
				if (ret)
					return ret;
				step = 1 + n + 4;
			}
			break;
		case 0x88:
			if (modrm_target(code, len, i + 1, &t, &n) && reg_fresh(&regs.r8[modrm_reg(code, i + 1)], i, &v)) {
				ret = emit_store(out, i, &t, v, 1);
				if (ret)
					return ret;
				step = 1 + n;
			}
			break;
		case 0x89:
			if (modrm_target(code, len, i + 1, &t, &n) && reg_fresh(&regs.r32[modrm_reg(code, i + 1)], i, &v)) {
				ret = emit_store(out, i, &t, v, 4);
				if (ret)
					return ret;
				step = 1 + n;
			}
			break;
		case 0xa2:
			if (rd32(code, len, i + 1, &a) && reg_fresh(&regs.r8[0], i, &v)) {
				t.kind = TARGET_ABS;
				t.abs = a;
				ret = emit_store(out, i, &t, v, 1);
				if (ret)
					return ret;
				step = 5;
			}
			break;
		case 0xa3:
			if (rd32(code, len, i + 1, &a) && reg_fresh(&regs.r32[0], i, &v)) {
				t.kind = TARGET_ABS;
				t.abs = a;
				ret = emit_store(out, i, &t, v, 4);
				if (ret)
					return ret;
				step = 5;
			}
			break;
		default:
			break;
		}
		i += step;
	}
	return 0;
}

/* ---- 書き込みの画像 (アドレス → バイト) ---- */

struct wbyte {
	u64 addr;
	u32 seq; /* 書かれた順 */
	u8 v;
};

struct image {
	u64 *addr;
	u8 *v;
	size_t n;
};

static int wbyte_cmp(const void *pa, const void *pb)
{
	const struct wbyte *a = pa, *b = pb;

	if (a->addr != b->addr)
		return a->addr < b->addr ? -1 : 1;
	if (a->seq != b->seq)
		return a->seq < b->seq ? -1 : 1;
	return 0;
}

/* 同じアドレスへの複数の書き込みから、最初の値 (first = true) か最後の値を採る */
static int image_build(struct image *img, struct wbyte *w, size_t nw, bool first)
{
	size_t i, n = 0;

	img->addr = malloc((nw ? nw : 1) * sizeof(u64));
	img->v = malloc(nw ? nw : 1);
	img->n = 0;
	if (!img->addr || !img->v)
		return -ENOMEM;
	for (i = 0; i < nw;) {
		size_t j = i;

		while (j + 1 < nw && w[j + 1].addr == w[i].addr)
			j++;
		img->addr[n] = w[i].addr;
		img->v[n] = first ? w[i].v : w[j].v;
		n++;
		i = j + 1;
	}
	img->n = n;
	return 0;
}

static void image_free(struct image *img)
{
	free(img->addr);
	free(img->v);
	memset(img, 0, sizeof(*img));
}

/* アドレスが連続する範囲 [start, end) の添字 */
static bool image_next_run(const struct image *img, size_t *pos, size_t *start, size_t *end)
{
	size_t i = *pos;

	if (i >= img->n)
		return false;
	*start = i;
	while (i + 1 < img->n && img->addr[i + 1] == img->addr[i] + 1)
		i++;
	*end = i + 1;
	*pos = i + 1;
	return true;
}

static bool image_get(const struct image *img, u64 addr, u8 *v)
{
	size_t lo = 0, hi = img->n;

	while (lo < hi) {
		size_t mid = lo + (hi - lo) / 2;

		if (img->addr[mid] == addr) {
			*v = img->v[mid];
			return true;
		}
		if (img->addr[mid] < addr)
			lo = mid + 1;
		else
			hi = mid;
	}
	return false;
}

/* 先頭 104 バイトのうち、24..64 が曲線上の点で、64..104 がその点による 0..64 の署名であるもの */
static bool find_certificate(const struct image *img, u64 *addr, u8 cert[DTH11_CERT_LEN])
{
	size_t pos = 0, s, e, o;

	while (image_next_run(img, &pos, &s, &e)) {
		if (e - s < DTH11_CERT_LEN)
			continue;
		for (o = 0; o + DTH11_CERT_LEN <= e - s; o++) {
			const u8 *c = img->v + s + o;

			if (dth11_ec_point_check(c + 24))
				continue;
			if (dth11_ec_verify(c + 24, c, 64, c + 64)) {
				memcpy(cert, c, DTH11_CERT_LEN);
				*addr = img->addr[s + o];
				return true;
			}
		}
	}
	return false;
}

static bool image_read20(const struct image *img, u64 addr, u8 out[DTH11_PRIV_LEN])
{
	int i;

	for (i = 0; i < DTH11_PRIV_LEN; i++)
		if (!image_get(img, addr + (u64)i, &out[i]))
			return false;
	return true;
}

/* 秘密鍵は証明書のすぐ後にある。無ければ、証明書の近く (0x10000 以内) の 20 バイトの窓を全部試す */
static bool find_private_key(const u8 q_xy[40], u64 cert_addr, const struct image *first, const struct image *last,
			     u8 key[DTH11_PRIV_LEN])
{
	const struct image *images[2] = { first, last };
	u64 after = cert_addr + DTH11_CERT_LEN;
	int k;

	for (k = 0; k < 2; k++)
		if (image_read20(images[k], after, key) && dth11_ec_key_matches(key, q_xy))
			return true;
	for (k = 0; k < 2; k++) {
		const struct image *img = images[k];
		size_t pos = 0, s, e, o;

		while (image_next_run(img, &pos, &s, &e)) {
			u64 start = img->addr[s];
			u64 dist = start > cert_addr ? start - cert_addr : cert_addr - start;

			if (dist > 0x10000 || e - s < DTH11_PRIV_LEN)
				continue;
			for (o = 0; o + DTH11_PRIV_LEN <= e - s; o++)
				if (dth11_ec_key_matches(img->v + s + o, q_xy)) {
					memcpy(key, img->v + s + o, DTH11_PRIV_LEN);
					return true;
				}
		}
	}
	return false;
}

/* ---- IV の候補 (スタック上の 16 バイトのバッファ) ---- */

struct slot {
	u8 base;
	s32 disp;
	u8 v;
};

static int slot_cmp(const void *pa, const void *pb)
{
	const struct slot *a = pa, *b = pb;

	if (a->base != b->base)
		return a->base < b->base ? -1 : 1;
	if (a->disp != b->disp)
		return a->disp < b->disp ? -1 : 1;
	return 0;
}

static int iv_cmp(const void *pa, const void *pb)
{
	return memcmp(pa, pb, 16);
}

/*
 * 近くの格納命令 (先頭から WINDOW_CODE_BYTES バイト以内) が組み立てた、16 バイトの欠けのないバッファを
 * 全て集める。同じバイトへの書き込みは最初の値を採る。格納された順と、反転した順の両方を候補にし、
 * 重複は除く。どの候補が正しいかは、最初のコンテンツブロックの CMAC で決まる。
 */
#define WINDOW_CODE_BYTES 400
#define CANDIDATE_LIMIT 50000

static int stack_buffers(const struct store_list *stores, u8 (**out)[16], size_t *out_n)
{
	struct store **stack;
	size_t nstack = 0, i, j, cap = 0, n = 0;
	u8 (*cand)[16] = NULL;
	struct slot *slots = NULL;
	size_t slot_cap = 0;

	stack = malloc((stores->n ? stores->n : 1) * sizeof(*stack));
	if (!stack)
		return -ENOMEM;
	for (i = 0; i < stores->n; i++)
		if (stores->v[i].kind == TARGET_STACK)
			stack[nstack++] = &stores->v[i];

	for (i = 0; i < nstack; i++) {
		size_t ns = 0, k;

		for (j = i; j < nstack; j++) {
			const struct store *s = stack[j];
			int b;

			if (s->pos - stack[i]->pos > WINDOW_CODE_BYTES)
				break;
			if (ns + 4 > slot_cap) {
				size_t nc = slot_cap ? slot_cap * 2 : 1024;
				struct slot *nsl = realloc(slots, nc * sizeof(*slots));

				if (!nsl) {
					free(slots);
					free(cand);
					free(stack);
					return -ENOMEM;
				}
				slots = nsl;
				slot_cap = nc;
			}
			for (b = 0; b < s->len; b++) {
				slots[ns].base = s->base;
				slots[ns].disp = s->disp + b;
				slots[ns].v = s->bytes[b];
				ns++;
			}
		}
		/*
		 * 同じ (base, disp) は最初に書かれた値を採る。安定でない qsort でも最初の値を選べるよう、
		 * 先に重複を除く (ns は小さいので二重ループで足りる)。
		 */
		{
			size_t w = 0;

			for (k = 0; k < ns; k++) {
				size_t m;
				bool dup = false;

				for (m = 0; m < w; m++)
					if (slots[m].base == slots[k].base && slots[m].disp == slots[k].disp) {
						dup = true;
						break;
					}
				if (!dup)
					slots[w++] = slots[k];
			}
			ns = w;
		}
		qsort(slots, ns, sizeof(*slots), slot_cmp);
		for (k = 0; k < ns; k++) {
			u8 buf[16];
			int c;
			bool complete = true;

			for (c = 0; c < 16 && complete; c++) {
				size_t m;
				bool found = false;

				for (m = k; m < ns && slots[m].base == slots[k].base; m++)
					if (slots[m].disp == slots[k].disp + c) {
						buf[c] = slots[m].v;
						found = true;
						break;
					}
				complete = found;
			}
			if (!complete)
				continue;
			if (n + 2 > cap) {
				size_t nc = cap ? cap * 2 : 4096;
				u8 (*nv)[16] = realloc(cand, nc * 16);

				if (!nv) {
					free(slots);
					free(cand);
					free(stack);
					return -ENOMEM;
				}
				cand = nv;
				cap = nc;
			}
			memcpy(cand[n++], buf, 16);
			for (c = 0; c < 16; c++)
				cand[n][c] = buf[15 - c];
			n++;
			if (n >= CANDIDATE_LIMIT)
				goto done;
		}
	}
done:
	free(slots);
	free(stack);
	/* 重複を除く。並べ替えるので、探索の順は変わるが、採用は CMAC の一致だけで決まる */
	if (n > 1) {
		size_t w = 1;

		qsort(cand, n, 16, iv_cmp);
		for (i = 1; i < n; i++)
			if (memcmp(cand[i], cand[w - 1], 16))
				memcpy(cand[w++], cand[i], 16);
		n = w;
	}
	*out = cand;
	*out_n = n;
	return 0;
}

/* ---- 公開 API ---- */

void dth11_appkeys_free(struct dth11_appkeys *keys)
{
	free(keys->iv_candidates);
	free(keys->data);
	memset(keys, 0, sizeof(*keys));
}

int dth11_appkeys_from_image(const u8 *image, size_t len, struct dth11_appkeys *out)
{
	struct pe_info pe;
	struct store_list stores = { 0 };
	struct wbyte *w = NULL;
	struct image first = { 0 }, last = { 0 };
	u8 *code = NULL, *data = NULL;
	size_t code_len = 0, data_len = 0, i, nw = 0, k;
	u64 image_end, cert_addr = 0;
	u8 cert[DTH11_CERT_LEN], priv_key[DTH11_PRIV_LEN];
	u8 (*cands)[16] = NULL;
	size_t ncands = 0;
	int ret;

	memset(out, 0, sizeof(*out));
	ret = parse_pe(image, len, &pe);
	if (ret)
		return ret;

	code = malloc(len ? len : 1);
	data = malloc(len ? len : 1);
	if (!code || !data) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < pe.nsec; i++) {
		const struct pe_section *s = &pe.sections[i];
		size_t end = (size_t)s->raw + (size_t)s->rsize;
		size_t raw_len;

		if (end < s->raw || end > len)
			end = len;
		raw_len = s->raw < end ? end - s->raw : 0;
		if (section_is_code(s)) {
			memcpy(code + code_len, image + s->raw, raw_len);
			code_len += raw_len;
		} else {
			memcpy(data + data_len, image + s->raw, raw_len);
			data_len += raw_len;
		}
	}
	if (!code_len) {
		ret = DTH11_APPKEYS_NOT_PE;
		goto out;
	}

	ret = scan_stores(code, code_len, &stores);
	if (ret)
		goto out;

	/* 画像の範囲 [image_base, image_end) の絶対アドレスへの格納だけを、書き込みの画像に入れる */
	image_end = pe.image_base + pe_image_size(&pe);
	w = malloc((stores.n * 4 + 1) * sizeof(*w));
	if (!w) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < stores.n; i++) {
		const struct store *s = &stores.v[i];

		if (s->kind != TARGET_ABS)
			continue;
		if (s->abs < pe.image_base || s->abs + s->len > image_end)
			continue;
		for (k = 0; k < s->len; k++) {
			w[nw].addr = s->abs + k;
			w[nw].seq = (u32)(i * 4 + k);
			w[nw].v = s->bytes[k];
			nw++;
		}
	}
	qsort(w, nw, sizeof(*w), wbyte_cmp);
	ret = image_build(&first, w, nw, true);
	if (!ret)
		ret = image_build(&last, w, nw, false);
	if (ret)
		goto out;

	if (!find_certificate(&first, &cert_addr, cert)) {
		ret = DTH11_APPKEYS_NO_CERT;
		goto out;
	}
	if (!find_private_key(cert + 24, cert_addr, &first, &last, priv_key)) {
		ret = DTH11_APPKEYS_NO_PRIVATE_KEY;
		goto out;
	}
	ret = stack_buffers(&stores, &cands, &ncands);
	if (ret)
		goto out;

	memcpy(out->cert, cert, sizeof(cert));
	memcpy(out->priv_key, priv_key, sizeof(priv_key));
	out->iv_candidates = cands;
	out->iv_candidate_count = ncands;
	out->data = data;
	out->data_len = data_len;
	data = NULL;
	cands = NULL;
	ret = 0;
out:
	free(pe.sections);
	free(stores.v);
	free(w);
	image_free(&first);
	image_free(&last);
	free(code);
	free(data);
	free(cands);
	return ret;
}

/* IV の候補を、包んだ鍵の最初のブロックの CBC 復号と CMAC で試す */
static bool iv_matches(const u8 d0[16], const u8 iv[16], const u8 blob[64])
{
	u8 k[16], mac[16];
	int i;

	for (i = 0; i < 16; i++)
		k[i] = (u8)(d0[i] ^ iv[i]);
	dth11_aes_cmac(k, blob, 48, mac);
	return !memcmp(mac, blob + 48, 16);
}

int dth11_appkeys_find_content_iv(struct dth11_appkeys *keys, const u8 k3[16], const u8 blob[64], u8 iv[16])
{
	struct dth11_aes a;
	u8 d0[16];
	size_t i;

	if (keys->iv_settled) {
		memcpy(iv, keys->iv_content, 16);
		return 0;
	}
	/* CBC では D[0..16] = AES_dec(c0) ^ IV で、D[0..16] が CMAC の鍵になる。候補 1 つの確認は CMAC 1 回 */
	dth11_aes_set_key(&a, k3);
	dth11_aes_decrypt_block(&a, blob + 16, d0);

	for (i = 0; i < keys->iv_candidate_count; i++)
		if (iv_matches(d0, keys->iv_candidates[i], blob)) {
			memcpy(keys->iv_content, keys->iv_candidates[i], 16);
			goto found;
		}
	/* スタックのバッファに無いときの最後の探索先: コード以外のセクションの全ての 16 バイトの窓 (そのままと反転) */
	for (i = 0; i + 16 <= keys->data_len; i++) {
		u8 w[16], r[16];
		int c;

		memcpy(w, keys->data + i, 16);
		if (iv_matches(d0, w, blob)) {
			memcpy(keys->iv_content, w, 16);
			goto found;
		}
		for (c = 0; c < 16; c++)
			r[c] = w[15 - c];
		if (iv_matches(d0, r, blob)) {
			memcpy(keys->iv_content, r, 16);
			goto found;
		}
	}
	return -ENOENT;
found:
	keys->iv_settled = true;
	memcpy(iv, keys->iv_content, 16);
	return 0;
}
