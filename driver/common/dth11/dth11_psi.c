// SPDX-License-Identifier: GPL-2.0-only
/*
 * dth11_psi.c - PAT/PMT の収集 (dth11_psi.h)
 */

#include "dth11_psi.h"

u32 dth11_mpeg_crc32(const u8 *data, size_t len)
{
	u32 crc = 0xffffffff;
	size_t i;
	int b;

	for (i = 0; i < len; i++) {
		crc ^= (u32)data[i] << 24;
		for (b = 0; b < 8; b++)
			crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04c11db7 : (crc << 1);
	}
	return crc;
}

void dth11_psi_init(struct dth11_psi *psi)
{
	memset(psi, 0, sizeof(*psi));
}

static bool pmt_pid_known(const struct dth11_psi *psi, u16 pid)
{
	u8 i;

	for (i = 0; i < psi->pat_count; i++)
		if (psi->pat_pmt_pid[i] == pid)
			return true;
	return false;
}

static struct dth11_psi_section *section_for(struct dth11_psi *psi, u16 pid)
{
	u8 i;

	for (i = 0; i < psi->section_count; i++)
		if (psi->sections[i].pid == pid)
			return &psi->sections[i];
	if (psi->section_count >= 1 + DTH11_PSI_MAX_PROGRAMS)
		return NULL;
	psi->sections[psi->section_count].pid = pid;
	psi->sections[psi->section_count].len = 0;
	return &psi->sections[psi->section_count++];
}

static void add_pid(u16 *list, u8 *count, u8 max, u16 pid)
{
	u8 i;

	for (i = 0; i < *count; i++)
		if (list[i] == pid)
			return;
	if (*count < max)
		list[(*count)++] = pid;
}

/* 記述子の列から、CA 記述子 (tag 0x09) の CA_PID を集める */
static void collect_ca_pids(const u8 *d, size_t len, struct dth11_program *prog)
{
	size_t i = 0;

	while (i + 2 <= len) {
		u8 tag = d[i];
		size_t dlen = d[i + 1];

		if (tag == 0x09 && dlen >= 4 && i + 2 + dlen <= len) {
			u16 pid = (u16)(((d[i + 4] & 0x1f) << 8) | d[i + 5]);

			if (pid != 0x1fff)
				add_pid(prog->ecm_pids, &prog->ecm_count, DTH11_PSI_MAX_ECM, pid);
		}
		i += 2 + dlen;
	}
}

static void handle_section(struct dth11_psi *psi, u16 pid, const u8 *sec, size_t len)
{
	size_t slen, end, i;
	u8 tid;

	if (len < 12)
		return;
	tid = sec[0];
	slen = ((size_t)(sec[1] & 0x0f) << 8) | sec[2];
	if (len < 3 + slen || slen < 9)
		return;
	/* セクションの末尾の CRC32 を含めて計算すると 0 になる。壊れたセクションは採用しない */
	if (dth11_mpeg_crc32(sec, 3 + slen))
		return;
	end = 3 + slen - 4;

	if (pid == 0 && tid == 0) {
		for (i = 8; i + 4 <= end; i += 4) {
			u16 prog = (u16)((sec[i] << 8) | sec[i + 1]);
			u16 ppid = (u16)(((sec[i + 2] & 0x1f) << 8) | sec[i + 3]);
			u8 k;
			bool found = false;

			if (!prog)
				continue;
			for (k = 0; k < psi->pat_count; k++)
				if (psi->pat_number[k] == prog) {
					psi->pat_pmt_pid[k] = ppid;
					found = true;
				}
			if (!found && psi->pat_count < DTH11_PSI_MAX_PROGRAMS) {
				psi->pat_number[psi->pat_count] = prog;
				psi->pat_pmt_pid[psi->pat_count] = ppid;
				psi->pat_count++;
			}
		}
	} else if (tid == 2 && pmt_pid_known(psi, pid)) {
		struct dth11_program prog;
		u16 number = (u16)((sec[3] << 8) | sec[4]);
		size_t pilen, j;
		u8 k;

		memset(&prog, 0, sizeof(prog));
		prog.number = number;
		prog.pmt_pid = pid;
		prog.pcr_pid = (u16)(((sec[8] & 0x1f) << 8) | sec[9]);
		pilen = ((size_t)(sec[10] & 0x0f) << 8) | sec[11];
		if (12 + pilen > end)
			return;
		collect_ca_pids(sec + 12, pilen, &prog);
		j = 12 + pilen;
		while (j + 5 <= end) {
			u16 epid = (u16)(((sec[j + 1] & 0x1f) << 8) | sec[j + 2]);
			size_t eilen = ((size_t)(sec[j + 3] & 0x0f) << 8) | sec[j + 4];

			if (prog.es_count < DTH11_PSI_MAX_ES)
				prog.es_pids[prog.es_count++] = epid;
			if (j + 5 + eilen <= end)
				collect_ca_pids(sec + j + 5, eilen, &prog);
			j += 5 + eilen;
		}
		/* 同じ番組の PMT は後のものに置き換える */
		for (k = 0; k < psi->pat_count; k++)
			if (psi->pat_number[k] == number) {
				psi->pmt[k] = prog;
				psi->pmt_seen[k] = true;
			}
	}
}

void dth11_psi_feed(struct dth11_psi *psi, const u8 *p)
{
	u16 pid;
	u8 afc;
	size_t off;
	const u8 *payload;
	size_t plen;
	struct dth11_psi_section *s;
	bool pusi;

	if (p[0] != 0x47 || (p[1] & 0x80))
		return;
	pid = (u16)(((p[1] & 0x1f) << 8) | p[2]);
	afc = (p[3] >> 4) & 3;
	if (!(afc & 1))
		return;
	if (pid != 0 && !pmt_pid_known(psi, pid))
		return;
	off = 4;
	if (afc & 2)
		off += 1 + p[4];
	if (off >= 188)
		return;
	payload = p + off;
	plen = 188 - off;
	pusi = (p[1] & 0x40) != 0;
	s = section_for(psi, pid);
	if (!s)
		return;

	if (pusi) {
		size_t ptr = payload[0];

		/* ポインターの前にあるのは、直前のセクションの続き。完成したものとして処理する */
		if (s->len) {
			size_t head = ptr < plen - 1 ? ptr : plen - 1;

			if (s->len + head <= DTH11_PSI_SECTION_MAX) {
				memcpy(s->data + s->len, payload + 1, head);
				s->len += (u16)head;
				handle_section(psi, pid, s->data, s->len);
			}
			s->len = 0;
		}
		/* ポインターの後が、新しいセクションの先頭。ポインターがパケットの外を指すものは捨てる */
		if (1 + ptr <= plen) {
			size_t n = plen - 1 - ptr;

			if (n > DTH11_PSI_SECTION_MAX)
				n = DTH11_PSI_SECTION_MAX;
			memcpy(s->data, payload + 1 + ptr, n);
			s->len = (u16)n;
		}
	} else if (s->len) {
		if (s->len + plen <= DTH11_PSI_SECTION_MAX) {
			memcpy(s->data + s->len, payload, plen);
			s->len += (u16)plen;
		} else {
			/* 収まらないセクションは捨てる (PAT と PMT は 1 KiB に収まる) */
			s->len = 0;
		}
	}
	/* 長さが分かった時点で、完成したセクションは直ちに処理する */
	if (s->len >= 3) {
		size_t slen = ((size_t)(s->data[1] & 0x0f) << 8) | s->data[2];

		if (s->len >= 3 + slen) {
			handle_section(psi, pid, s->data, 3 + slen);
			s->len = 0;
		}
	}
}

bool dth11_psi_complete(const struct dth11_psi *psi)
{
	u8 i;

	if (!psi->pat_count)
		return false;
	for (i = 0; i < psi->pat_count; i++)
		if (!psi->pmt_seen[i])
			return false;
	return true;
}

size_t dth11_psi_programs(const struct dth11_psi *psi, struct dth11_program *out, size_t max)
{
	size_t n = 0, i, j;

	for (i = 0; i < psi->pat_count && n < max; i++)
		if (psi->pmt_seen[i])
			out[n++] = psi->pmt[i];
	/* 番組番号の昇順 (件数は少ないので挿入ソート) */
	for (i = 1; i < n; i++) {
		struct dth11_program t = out[i];

		for (j = i; j > 0 && out[j - 1].number > t.number; j--)
			out[j] = out[j - 1];
		out[j] = t;
	}
	return n;
}

size_t dth11_program_pid_list(const struct dth11_program *prog, u16 out[DTH11_PROGRAM_MAX_PIDS])
{
	u8 n = 0, i;

	out[n++] = prog->pmt_pid;
	for (i = 0; i < prog->ecm_count; i++)
		add_pid(out, &n, DTH11_PROGRAM_MAX_PIDS, prog->ecm_pids[i]);
	if (prog->pcr_pid != 0x1fff)
		add_pid(out, &n, DTH11_PROGRAM_MAX_PIDS, prog->pcr_pid);
	for (i = 0; i < prog->es_count; i++)
		add_pid(out, &n, DTH11_PROGRAM_MAX_PIDS, prog->es_pids[i]);
	return n;
}
