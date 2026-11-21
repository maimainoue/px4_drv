/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PIX-DT295 の boot0 の hook (0xEA6FC) に置く、TS ポンプのプログラム (カーネルモジュール用の最小版)。
 * tools/fr30/dt295_e3.s から、試験用の操作 (リングの充填、任意アドレスの読み書き) を除いたもの。
 *
 * 入口の条件 (wValue 下位 0x7D、wLength 0x40、アドレス 0xC4620、先頭 'DNT1') が合えば操作を実行し、違えば元の命令へ素通しする。
 * 操作 (0xC4624 の語):
 *   0x12  SDRAM の到達確認。リングの先頭に 0xDEADBEEF を書いて読み戻し、+8 に結果を返す。
 *   0x11  リング (0x8138EC00 から 0x30000 バイト) から EP1 (bulk IN 0x81) へ、512 バイト (128 語) のパケットを指定個数だけ送る。
 *         要求: +8 個数、+12 待ちの上限 (コマンドを受けられるのを待つ回数)、+16 読み出し位置 (CPU のアドレス)
 *         応答: +8 結果 (0 完了、1 待ちの上限)、+12 送ったパケット数、+16 次の読み出し位置
 *
 * EP1 の送信 (ROM の EP3 の送信ルーチンと、ホストからの試験で確かめたもの):
 *   - 送信バッファは 2 つ。状態 0xFDBD0C の bit2 が立っている間は、FIFO (0xFDBD84) に 128 語を書いてコマンドを送れる。
 *     2 つ送信待ちになると bit2 が落ちる。落ちている間に書いたパケットは、コマンドが受理されず失われる。
 *   - コマンド: 0xFD814C = 8 にして 0xFDBD0C へ 4 を書き、0xFD814C = 0xF に戻す。
 *   - コマンドの直後は状態の更新が遅れて見えることがあるので、状態を数回読んでから、次の判定をする。
 * 取り込みブロック (0x19xxxxx) と 0xF10118 へは、このプログラムから書かない (機材が止まる)。
 * 状態段 (0xEA8C8) で r8〜r11 が復元されるので、操作を実行する側では r2〜r13 を使ってよい。r14、r15 は変えない。
 * 素通しの側 (入口の判定まで) は r0、r1、r12、r13 だけを使う。
 *
 * 組み立て: fr30-elf-as でオブジェクトにし、fr30-elf-ld -Ttext=0xe1000 でリンクして、fr30-elf-objcopy -O binary で取り出す。
 * カーネルモジュールへは、tools/fr30/gen_dt295_program.py が C の配列にして埋め込む。
 */
	.text
	.global body
body:
	ldi:8	#0x46, r13
	ldub	@(r13,r11), r0
	ldi:8	#0x7d, r1
	cmp	r1, r0
	bne	orig
	ldi:8	#0x48, r13
	lduh	@(r13,r11), r0
	ldi:8	#0x40, r1
	cmp	r1, r0
	bne	orig
	ld	@r11, r0
	ldi:20	#0xc4620, r1
	cmp	r1, r0
	bne	orig
	ldi:32	#0xc4620, r12
	ld	@r12, r0
	ldi:32	#0x444e5431, r1
	cmp	r1, r0
	bne	orig
	ldi:32	#0xc4624, r12
	ld	@r12, r0		/* 操作番号 */
	ldi:8	#0x12, r1
	cmp	r1, r0
	beq	op_probe
	ldi:8	#0x11, r1
	cmp	r1, r0
	beq	op_pump
	/* 知らない操作は、元の処理へ (下へ続く) */
orig:
	ldi:8	#0x46, r13
	ldub	@(r13,r11), r0
	cmp	#1, r0
	beq	to_mask
	ldi:32	#0xea75a, r12
	jmp	@r12
to_mask:
	ldi:32	#0xea704, r12
	jmp	@r12

finish:
	ldi:32	#0xea8c8, r12
	jmp	@r12

/* 操作 0x12 */
op_probe:
	ldi:32	#0x8138ec00, r2
	ldi:32	#0xdeadbeef, r3
	st	r3, @r2
	ld	@r2, r4
	ldi:32	#0xc4628, r12
	st	r4, @r12
	bra	finish

/* 操作 0x11 */
op_pump:
	ldi:32	#0xc4628, r12
	ld	@r12, r3		/* 個数 */
	ldi:32	#0xc4630, r12
	ld	@r12, r2		/* 読み出し位置 */
	ldi:32	#0xfdbd84, r4		/* EP1 の FIFO */
	ldi:32	#0xfdbd0c, r5		/* EP1 の状態とコマンド */
	ldi:32	#0xfd814c, r7		/* バイトレーン */
	ldi:32	#0x8138ec00, r8		/* リングの先頭 */
	ldi:32	#0x813bec00, r9		/* リングの終わり (0x8138ec00 + 0x30000) */
	ldi:8	#4, r10			/* 状態の bit2 (コマンドを受けられる) */
	ldi:8	#0, r11			/* 送ったパケット数 */
	ldi:8	#0, r13			/* 結果 */
packet:
	cmp	#0, r3
	beq	pump_done
	/* コマンドを受けられるまで待つ。待ちの上限を超えたら結果 1 で終わる (制御転送を返せなくなるのを防ぐ)。 */
	ldi:32	#0xc462c, r12
	ld	@r12, r6
wait_ready:
	ld	@r5, r1
	mov	r1, r0
	and	r10, r0
	bne	ready
	add2	#-1, r6
	bne	wait_ready
	ldi:8	#1, r13
	bra	pump_done
ready:
	ldi:8	#0x80, r1		/* 128 語 */
copy:
	ld	@r2, r0
	st	r0, @r4
	add	#4, r2
	add2	#-1, r1
	bne	copy
	/* リングの終わりで折り返す */
	cmp	r9, r2
	bne	no_wrap
	mov	r8, r2
no_wrap:
	/* コマンド: 最下位バイトのレーンだけで 4 を書く */
	ldi:8	#8, r0
	st	r0, @r7
	ldi:8	#4, r0
	st	r0, @r5
	ldi:8	#0xf, r0
	st	r0, @r7
	/* 状態の更新を待つ (コマンド直後の古い状態を見て、満杯のバッファへ書くのを防ぐ) */
	ldi:8	#16, r1
settle:
	ld	@r5, r0
	add2	#-1, r1
	bne	settle
	add	#1, r11
	add2	#-1, r3
	ldi:32	#packet, r12		/* 条件分岐の範囲を超えるので絶対ジャンプ */
	jmp	@r12
pump_done:
	ldi:32	#0xc4628, r12
	st	r13, @r12
	ldi:32	#0xc462c, r12
	st	r11, @r12
	ldi:32	#0xc4630, r12
	st	r2, @r12
	ldi:32	#finish, r12		/* finish は遠いので絶対ジャンプ */
	jmp	@r12
	.balign	4
