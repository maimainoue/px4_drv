/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GV-MVP/XZ (04bb:0530) の boot0 の hook (0xE9F48) に置く、TS ポンプの試験プログラム (E3a/b)。
 * DT295 の tools/fr30/dt295_e3.s を、次の XZ の条件に合わせたもの。
 *   - 書き込みは 1 回の要求が 4 バイトまでなので、DT295 のような 64 バイトの要求ブロックを置けない。
 *     パラメータは本体の領域内の郵便受け (MB = 0xE1180) へ 4 バイトずつ書き、起動は別の 4 バイト書き込み
 *     (wValue 下位 0x7D、アドレス 0xE11FC) で行う。起動の書き込みのデータは使わない。
 *   - 作業領域は r10 = 0xC4600 (+0x40 wValue 下位、+0x44 アドレス)。r11 はセットアップのバッファで、ここでは使わない。
 *   - リングの位置は DT295 の値が XZ で通用する保証がないので、郵便受けの +0x10 (先頭)、+0x14 (終わりの次) で指定する。
 *
 * 郵便受け (すべて 32 ビット、ビッグエンディアン):
 *   +0x00 操作  +0x04 引数0  +0x08 引数1  +0x0C 引数2  +0x10 リング先頭  +0x14 リング終わり
 *   +0x20 結果 (ホストが起動前に 0xFFFFFFFF を入れ、本体が書き換える。変わらなければ本体を通っていない)
 *   +0x24 応答1  +0x28 応答2  +0x2C〜+0x34 診断  +0x40〜+0x6C 読み出した語 (最大 12 語)
 * 操作:
 *   0x13 読み出し: 引数0 アドレス、引数1 語数 (1〜12)。結果 0、語は +0x40 から。
 *   0x14 書き込み: 引数0 アドレス、引数1 値。結果 0、応答1 に読み戻した値。
 *   0x10 リングを語の番号 (0、1、2、...) で埋める。結果 0。
 *   0x11 リングから EP1 (bulk IN 0x81) へ 512 バイトのパケットを送る。
 *        引数0 個数、引数1 待ちの上限 (コマンドを受けられるのを待つ回数)、引数2 読み出し位置 (CPU のアドレス)。
 *        結果 0 完了 / 1 待ちの上限、応答1 送ったパケット数、応答2 次の読み出し位置。
 *
 * EP1 の送信 (DT295 の ROM の送信ルーチンと実機の試験に基づく。XZ の boot0 も同じレジスタ参照):
 *   - 送信バッファは 2 つ。状態 0xFDBD0C の bit2 が立っている間だけ、FIFO (0xFDBD84) に 128 語を書いてコマンドを送れる。
 *     bit2 が落ちている間に書いたパケットは、コマンドが受理されず失われる。
 *   - コマンド: 0xFD814C = 8 にして 0xFDBD0C へ 4 を書き、0xFD814C = 0xF に戻す。
 *   - コマンド直後は状態の更新が遅れて見えることがあるので、状態を数回読んでから次の判定をする。
 *
 * 終了は 0xE9FB6 (EP0 の応答を返して関数を抜ける共通の出口) へ飛ぶ。そこで r8〜r11 が復元されるので、
 * 操作の側では r2〜r13 を使ってよい。r14、r15 は変えない。素通しの側は r0、r1、r12、r13 だけを使う。
 *
 * 組み立て: fr30-elf-as でオブジェクトにし、fr30-elf-ld -Ttext=0xe1000 -e body でリンクして、
 * fr30-elf-objcopy -O binary で取り出す (遠い分岐を絶対ジャンプにしたので、アドレスを解決するため)。
 * 本体は 0x180 バイト未満にすること (郵便受けの手前まで)。
 */
	.text
	.global body
body:
	ldi:8	#0x40, r13
	ldub	@(r13,r10), r0
	ldi:8	#0x7d, r1
	cmp	r1, r0
	bne	orig
	ldi:8	#0x44, r13
	ld	@(r13,r10), r0
	ldi:20	#0xe11fc, r1
	cmp	r1, r0
	bne	orig
	ldi:20	#0xe1180, r12
	ld	@r12, r0		/* 操作番号 */
	ldi:8	#0x13, r1
	cmp	r1, r0
	beq	op_read
	ldi:8	#0x14, r1
	cmp	r1, r0
	beq	op_write
	ldi:8	#0x10, r1
	cmp	r1, r0
	beq	op_fill
	ldi:8	#0x11, r1
	cmp	r1, r0
	bne	orig
	ldi:32	#op_pump, r12		/* 遠いので絶対ジャンプ */
	jmp	@r12
	/* 知らない操作と、起動条件に合わない書き込みは、元の 4 命令と同じ動作で元の分岐先へ (xz_e2a.s と同じ) */
orig:
	ldi:8	#0x40, r13
	ldub	@(r13,r10), r0
	cmp	#1, r0
	bne	to_copy
	ldi:32	#0xe9f50, r12
	jmp	@r12
to_copy:
	ldi:32	#0xe9f6c, r12
	jmp	@r12

finish:
	ldi:32	#0xe9fb6, r12
	jmp	@r12

/* 操作 0x13 */
op_read:
	ldi:20	#0xe1184, r12
	ld	@r12, r2		/* アドレス */
	ldi:20	#0xe1188, r12
	ld	@r12, r3		/* 語数 */
	ldi:20	#0xe11c0, r4		/* 応答の置き場 (+0x40) */
read_loop:
	cmp	#0, r3
	beq	read_done
	ld	@r2, r0
	st	r0, @r4
	add	#4, r2
	add	#4, r4
	add2	#-1, r3
	bra	read_loop
read_done:
	ldi:20	#0xe11a0, r12
	ldi:8	#0, r0
	st	r0, @r12
	bra	finish

/* 操作 0x14 */
op_write:
	ldi:20	#0xe1184, r12
	ld	@r12, r2		/* アドレス */
	ldi:20	#0xe1188, r12
	ld	@r12, r3		/* 値 */
	st	r3, @r2
	ld	@r2, r5
	ldi:20	#0xe11a4, r12
	st	r5, @r12		/* 読み戻した値 */
	ldi:20	#0xe11a0, r12
	ldi:8	#0, r0
	st	r0, @r12
	bra	finish

/* 操作 0x10 */
op_fill:
	ldi:20	#0xe1190, r12
	ld	@r12, r2		/* リング先頭 */
	ldi:20	#0xe1194, r12
	ld	@r12, r3		/* リング終わり */
	sub	r2, r3
	lsr	#2, r3			/* 語数 */
	ldi:8	#0, r1
fill_loop:
	st	r1, @r2
	add	#4, r2
	add	#1, r1
	add2	#-1, r3
	bne	fill_loop
	ldi:20	#0xe11a0, r12
	ldi:8	#0, r0
	st	r0, @r12
	bra	finish

/* 操作 0x11 */
op_pump:
	ldi:20	#0xe1184, r12
	ld	@r12, r3		/* 個数 */
	ldi:20	#0xe118c, r12
	ld	@r12, r2		/* 読み出し位置 */
	ldi:20	#0xe1190, r12
	ld	@r12, r8		/* リング先頭 */
	ldi:20	#0xe1194, r12
	ld	@r12, r9		/* リング終わり */
	ldi:32	#0xfdbd84, r4		/* EP1 の FIFO */
	ldi:32	#0xfdbd0c, r5		/* EP1 の状態とコマンド */
	ldi:32	#0xfd814c, r7		/* バイトレーン */
	ldi:8	#4, r10			/* 状態の bit2 (コマンドを受けられる) */
	ldi:8	#0, r11			/* 送ったパケット数 */
	ldi:8	#0, r13			/* 結果 */
packet:
	cmp	#0, r3
	beq	pump_done
	/* コマンドを受けられるまで待つ。待ちの上限を超えたら結果 1 で終わる (制御転送を返せなくなるのを防ぐ)。 */
	ldi:20	#0xe1188, r12
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
	ldi:20	#0xe11ac, r12		/* 診断: 書く前の生の状態 */
	st	r1, @r12
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
	ld	@r5, r0			/* 診断: コマンド直後の状態 */
	ldi:20	#0xe11b0, r12
	st	r0, @r12
	/* 状態の更新を待つ (コマンド直後の古い状態を見て、満杯のバッファへ書くのを防ぐ) */
	ldi:8	#16, r1
settle:
	ld	@r5, r0
	add2	#-1, r1
	bne	settle
	ldi:20	#0xe11b4, r12		/* 診断: 状態を数回読んだ後の状態 */
	st	r0, @r12
	add	#1, r11
	add2	#-1, r3
	bra	packet
pump_done:
	ldi:20	#0xe11a0, r12
	st	r13, @r12
	ldi:20	#0xe11a4, r12
	st	r11, @r12
	ldi:20	#0xe11a8, r12
	st	r2, @r12
	ldi:32	#finish, r12		/* finish は遠いので絶対ジャンプ */
	jmp	@r12
	.balign	4
