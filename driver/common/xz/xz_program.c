// SPDX-License-Identifier: GPL-2.0-only
/*
 * GV-MVP/XZ2: 機材内プログラム (boot0 の hook から動く) の導入、復元、ポンプの要求。
 *
 * hook (0xE9F48) は 8 バイトだが、書き込みが 1 回 4 バイトまでなので 2 回に分ける。hook の位置は、次の書き込みの要求自身が
 *   通る経路なので、途中の状態 (前半だけ、または後半だけが新しい) でも要求を処理できる必要がある。実機で通った順序
 *   (導入は前半 (ldi:20 #0xE1000, r12) → 後半 (jmp @r12)、復元は後半 → 前半) だけを使う。逆順は試していない。
 * 導入の順序: 本体の領域を保存 → 本体を書いて読み戻す → 郵便受けを初期化 → hook を書いて読み戻す。
 * 復元は、hook を先に戻してから本体の領域を戻す (hook を残したまま本体を消すと、次の要求で壊れた命令を実行する)。
 */
#include "xz_core.h"
#include "xz_program_gen.h"

#define BODY_ADDRESS 0xe1000u
#define BODY_SAVE_BYTES 512u
#define HOOK_ADDRESS 0xe9f48u
#define MAILBOX 0xe1180u
#define TRIGGER_ADDRESS 0xe11fcu
#define TRIGGER_VALUE 0x007du	/* wValue の下位バイト。上位バイトはアドレス (0xE11FC) の上位 */
#define OP_PUMP 0x11u
#define PUMP_WAIT_LIMIT 5000000u	/* コマンドを受けられるまでの待ちの上限 (回)。ホストが読まないとき、制御転送を返せなくなるのを防ぐ */
#define PUMP_TIMEOUT_MS 5000U

static const u8 original_hook[8] = { 0xc4, 0x0d, 0x02, 0xa0, 0xa8, 0x10, 0xe3, 0x0e };
/* 前半: ldi:20 #0xE1000, r12。後半: jmp @r12 */
static const u8 program_hook[8] = { 0x9b, 0xec, 0x10, 0x00, 0x97, 0x0c, 0xe3, 0x0e };

static u32 get_be32(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

/* hook を元へ戻す (後半 → 前半)。 */
static int hook_restore(struct xz_bridge *b)
{
	int ret = xz_write(b, HOOK_ADDRESS + 4, original_hook + 4, 4);

	if (!ret)
		ret = xz_write(b, HOOK_ADDRESS, original_hook, 4);
	return ret;
}

int xz_prog_install(struct xz_bridge *b)
{
	u8 back[XZ_PROGRAM_BYTES], hook[8];
	int ret;

	if (b->program_installed)
		return -EALREADY;
	ret = xz_read(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	if (!ret)
		ret = xz_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (ret)
		return ret;
	if (memcmp(hook, original_hook, sizeof(hook)))
		return -EBUSY;
	ret = xz_write(b, BODY_ADDRESS, xz_program, XZ_PROGRAM_BYTES);
	if (!ret)
		ret = xz_read(b, BODY_ADDRESS, back, sizeof(back));
	if (!ret && memcmp(back, xz_program, sizeof(back)))
		ret = -EIO;
	/* 郵便受け: 操作 0、リングの先頭と終わりの次 */
	if (!ret)
		ret = xz_write32(b, MAILBOX, 0);
	if (!ret)
		ret = xz_write32(b, MAILBOX + 0x10, XZ_RING_BASE);
	if (!ret)
		ret = xz_write32(b, MAILBOX + 0x14, XZ_RING_BASE + XZ_RING_SIZE);
	if (ret)
		goto restore_body;
	ret = xz_write(b, HOOK_ADDRESS, program_hook, 4);
	if (!ret)
		ret = xz_write(b, HOOK_ADDRESS + 4, program_hook + 4, 4);
	if (!ret)
		ret = xz_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (!ret && memcmp(hook, program_hook, sizeof(hook)))
		ret = -EIO;
	if (ret) {
		/* hook を書いた可能性があるので、先に元へ戻す */
		hook_restore(b);
		goto restore_body;
	}
	b->program_installed = true;
	return 0;

restore_body:
	xz_write(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	return ret;
}

int xz_prog_remove(struct xz_bridge *b)
{
	u8 hook[8], body[BODY_SAVE_BYTES];
	int ret;

	/* 導入していないとき、saved_body は空なので、本体の領域へ書き戻すと 0 で潰してしまう */
	if (!b->program_installed)
		return 0;
	ret = hook_restore(b);
	if (!ret)
		ret = xz_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (!ret && memcmp(hook, original_hook, sizeof(hook)))
		ret = -EIO;
	if (ret) {
		/* 機材が取り外されたとき (-ENODEV) は、元へ戻せないのが当然なので、記録しない */
		if (ret != -ENODEV)
			b->ops->log(b->priv, XZ_LOG_ERR, "XZ could not restore the boot0 hook (%d); replug the device", ret);
		return ret;
	}
	b->program_installed = false;
	ret = xz_write(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	if (!ret)
		ret = xz_read(b, BODY_ADDRESS, body, sizeof(body));
	if (!ret && memcmp(body, b->saved_body, sizeof(body)))
		ret = -EIO;
	return ret;
}

int xz_prog_pump(struct xz_bridge *b, u32 ring_pos, u32 packets)
{
	u8 trigger[4] = { 0 }, reply[12];
	u32 result, sent;
	int ret;

	if (!b->program_installed || !packets || packets > XZ_PUMP_MAX_PACKETS || packets % XZ_PUMP_UNIT_PACKETS ||
	    ring_pos >= XZ_RING_SIZE || (ring_pos & 3u))
		return -EINVAL;
	/* 引数、結果の初期値 (0xFFFFFFFF = 本体を通っていない印)、操作番号の順に書いてから、起動の書き込みを出す。 */
	ret = xz_write32(b, MAILBOX + 0x04, packets);
	if (!ret)
		ret = xz_write32(b, MAILBOX + 0x08, PUMP_WAIT_LIMIT);
	if (!ret)
		ret = xz_write32(b, MAILBOX + 0x0c, XZ_RING_BASE + ring_pos);
	if (!ret)
		ret = xz_write32(b, MAILBOX + 0x20, 0xffffffffu);
	if (!ret)
		ret = xz_write32(b, MAILBOX, OP_PUMP);
	if (ret)
		return ret;
	/* 起動の書き込み (データは使われない)。本体は、送り終えてから、この要求へ応答する。 */
	ret = b->ops->control(b->priv, 0x40, (u16)(TRIGGER_VALUE | ((TRIGGER_ADDRESS >> 8) & 0xff00)), (u16)(TRIGGER_ADDRESS & 0xffffu), trigger,
			      sizeof(trigger), PUMP_TIMEOUT_MS);
	if (ret < 0)
		return ret;
	if (ret != (int)sizeof(trigger))
		return -EIO;
	ret = xz_read(b, MAILBOX + 0x20, reply, sizeof(reply));
	if (ret)
		return ret;
	result = get_be32(reply);
	sent = get_be32(reply + 4);
	/* 結果 0 = 完了、1 = 待ちの上限 (読み出し側が追いつかない)、0xFFFFFFFF = 本体を通っていない */
	if (result != 0 || sent != packets) {
		b->ops->log(b->priv, XZ_LOG_ERR, "XZ pump: result=%u sent=%u of %u", result, sent, packets);
		return result == 1 ? -ETIMEDOUT : -EIO;
	}
	return 0;
}
