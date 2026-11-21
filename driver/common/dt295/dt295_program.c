// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295: 機材内プログラム (boot0 の hook から動く) の導入、復元、ポンプの要求。
 *
 * 導入の順序 (どれも実機で確認): 本体の領域を保存 -> 本体を書いて読み戻す -> hook を最後に書いて読み戻す。
 * 復元は、hook を先に元へ戻してから、本体の領域を戻す。hook を残したまま本体を消すと、次の要求で壊れた命令を実行する。
 * DT295 の hook は 8 バイトを 1 回の要求で書ける。
 */
#include "dt295_core.h"
#include "dt295_program_gen.h"

#define BODY_ADDRESS 0xe1000u
#define BODY_SAVE_BYTES 512u
#define HOOK_ADDRESS 0xea6fcu
/* 要求と応答の場所と、本体を呼び出す要求の印 (wValue 下位 0x7D、wLength 0x40、アドレス 0xC4620、先頭 'DNT1') */
#define STAGING_ADDRESS 0xc4620u
#define REQUEST_VALUE 0x0c7du
#define REQUEST_INDEX 0x4620u
#define REQUEST_SIZE 64u
#define OP_PUMP 0x11u
#define PUMP_WAIT_LIMIT 5000000u	/* コマンドを受けられるまでの待ちの上限 (回)。ホストが読まないとき、制御転送を返せなくなるのを防ぐ */
#define PUMP_TIMEOUT_MS 3000U

static const u8 original_hook[8] = { 0xc4, 0x6d, 0x02, 0xb0, 0xa8, 0x10, 0xe3, 0x2b };
/* ldi:32 #0xE1000, r12 ; jmp @r12 */
static const u8 program_hook[8] = { 0x9f, 0x8c, 0x00, 0x0e, 0x10, 0x00, 0x97, 0x0c };

static void put_be32(u8 *p, u32 v)
{
	p[0] = (u8)(v >> 24);
	p[1] = (u8)(v >> 16);
	p[2] = (u8)(v >> 8);
	p[3] = (u8)v;
}

static u32 get_be32(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

int dt295_prog_install(struct dt295_bridge *b)
{
	u8 back[sizeof(dt295_program)], hook[8];
	int ret;

	if (b->program_installed)
		return -EALREADY;
	ret = dt295_read(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	if (!ret)
		ret = dt295_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (ret)
		return ret;
	if (memcmp(hook, original_hook, sizeof(hook)))
		return -EBUSY;
	ret = dt295_write(b, BODY_ADDRESS, dt295_program, sizeof(dt295_program));
	if (!ret)
		ret = dt295_read(b, BODY_ADDRESS, back, sizeof(back));
	if (!ret && memcmp(back, dt295_program, sizeof(back)))
		ret = -EIO;
	if (ret)
		goto restore_body;
	ret = dt295_write(b, HOOK_ADDRESS, program_hook, sizeof(program_hook));
	if (!ret)
		ret = dt295_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (!ret && memcmp(hook, program_hook, sizeof(hook)))
		ret = -EIO;
	if (ret) {
		/* hook を書いた可能性があるので、先に元へ戻す */
		dt295_write(b, HOOK_ADDRESS, original_hook, sizeof(original_hook));
		goto restore_body;
	}
	b->program_installed = true;
	return 0;

restore_body:
	dt295_write(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	return ret;
}

int dt295_prog_remove(struct dt295_bridge *b)
{
	u8 hook[8], body[BODY_SAVE_BYTES];
	int ret;

	/* 導入していないとき、saved_body は空なので、本体の領域へ書き戻すと 0 で潰してしまう */
	if (!b->program_installed)
		return 0;
	ret = dt295_write(b, HOOK_ADDRESS, original_hook, sizeof(original_hook));
	if (!ret)
		ret = dt295_read(b, HOOK_ADDRESS, hook, sizeof(hook));
	if (!ret && memcmp(hook, original_hook, sizeof(hook)))
		ret = -EIO;
	if (ret) {
		/* 機材が取り外されたとき (-ENODEV) は、元へ戻せないのが当然なので、記録しない */
		if (ret != -ENODEV)
			b->ops->log(b->priv, DT295_LOG_ERR, "DT295 could not restore the boot0 hook (%d); replug the device", ret);
		return ret;
	}
	b->program_installed = false;
	ret = dt295_write(b, BODY_ADDRESS, b->saved_body, BODY_SAVE_BYTES);
	if (!ret)
		ret = dt295_read(b, BODY_ADDRESS, body, sizeof(body));
	if (!ret && memcmp(body, b->saved_body, sizeof(body)))
		ret = -EIO;
	return ret;
}

int dt295_prog_pump(struct dt295_bridge *b, u32 ring_pos, u32 packets)
{
	u8 request[REQUEST_SIZE] = { 'D', 'N', 'T', '1' };
	u8 reply[REQUEST_SIZE];
	u32 result, sent;
	int ret;

	if (!b->program_installed || !packets || packets > DT295_PUMP_MAX_PACKETS || packets % DT295_PUMP_UNIT_PACKETS ||
	    ring_pos >= DT295_RING_SIZE || (ring_pos & 3u))
		return -EINVAL;
	put_be32(request + 4, OP_PUMP);
	put_be32(request + 8, packets);
	put_be32(request + 12, PUMP_WAIT_LIMIT);
	put_be32(request + 16, DT295_RING_BASE + ring_pos);

	/* 要求と応答は同じ場所を通るので、組の間は他の制御転送を止める。 */
	ret = b->ops->lock(b->priv);
	if (ret)
		return ret;
	ret = b->ops->control_locked(b->priv, 0x40, REQUEST_VALUE, REQUEST_INDEX, request, REQUEST_SIZE, PUMP_TIMEOUT_MS);
	if (ret >= 0 && ret != (int)REQUEST_SIZE)
		ret = -EIO;
	if (ret >= 0) {
		ret = b->ops->control_locked(b->priv, 0xc0, (u16)((STAGING_ADDRESS >> 8) & 0xff00), (u16)(STAGING_ADDRESS & 0xffffu), reply,
					     REQUEST_SIZE, PUMP_TIMEOUT_MS);
		if (ret >= 0 && ret != (int)REQUEST_SIZE)
			ret = -EIO;
	}
	b->ops->unlock(b->priv);
	if (ret < 0)
		return ret;
	result = get_be32(reply + 8);
	sent = get_be32(reply + 12);
	/* 結果 0 = 完了、1 = 待ちの上限 (読み出し側が追いつかない) */
	if (result != 0 || sent != packets) {
		b->ops->log(b->priv, DT295_LOG_ERR, "DT295 pump: result=%u sent=%u of %u", result, sent, packets);
		return result == 1 ? -ETIMEDOUT : -EIO;
	}
	return 0;
}
