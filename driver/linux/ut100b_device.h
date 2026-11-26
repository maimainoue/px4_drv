// SPDX-License-Identifier: GPL-2.0-only
#ifndef __UT100B_DEVICE_H__
#define __UT100B_DEVICE_H__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/usb.h>

#include "ut100b_core.h"
#include "itedtv_bus.h"
#include "px4_card.h"
#include "ptx_chrdev.h"

#define UT100B_CHRDEV_NUM 1U

struct task_struct;

struct ut100b_device {
	struct kref kref;
	atomic_t available;
	struct device *dev;
	struct usb_device *usb_dev;
	struct completion *quit_completion;
	struct ptx_chrdev_group *chrdev_group;
	struct ptx_chrdev *chrdev;
	/* ptx_chrdev の open / release / tune / set_capture / 読み出しを直列化する */
	struct mutex operation_lock;
	/*
	 * 機器への制御転送の組 (カードの 1 命令や、起動の手順) と、周期処理を直列化する。
	 * 制御転送を 1 つずつ通すだけでは足りない。カードの命令は、複数の転送の組が 1 つの操作で、間に別の転送が入ると壊れる。
	 * operation_lock より内側で取る (取る順は operation_lock、bus_lock)。
	 */
	struct mutex bus_lock;
	struct itedtv_bus stream_bus;
	struct hz2w_bridge bridge;
	struct hz2w_card card;
	struct px4_card *card_dev;
	/* 復号の表を含み 40 KB を超えるため、ヒープに置く */
	struct ut100b_aligner *aligner;
	u8 *ts_output;
	/* 周期処理 (0x4a のレジスタ 4 の読み出しと要求 0x22)。止まると、約 20 秒で機器が USB から消える。 */
	struct task_struct *keepalive_task;
	bool board_open;
	bool bus_initialized;
	bool capture_enabled;
	bool streaming;
	/* カードの 1 命令の最中は、制御転送の期限を命令の残り時間で打ち切る (bus_lock の中で設定する) */
	bool card_op_active;
	/*
	 * 切断または unbind の処理に入った。USB コアは disconnect の前に URB を止めるので、止める最中の転送は失敗する。
	 * その失敗は想定内なので、エラーとして記録しない。
	 */
	bool stopping;
	/* 選局の後、最初にロックを確認した時刻。ロックの直後は、復調が安定するまで TEI 付きのパケットが出るので、受信の開始を遅らせる。 */
	bool lock_seen;
	unsigned long lock_time;
};

int ut100b_device_init(struct ut100b_device *device, struct device *dev, struct usb_interface *intf,
		       struct ptx_chrdev_context *chrdev_ctx, struct completion *quit_completion);
void ut100b_device_term(struct ut100b_device *device);

#endif
