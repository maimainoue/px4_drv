// SPDX-License-Identifier: GPL-2.0-only
#ifndef __HZ2W_DEVICE_H__
#define __HZ2W_DEVICE_H__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/usb.h>

#include "hz2w_core.h"
#include "px4_card.h"

struct task_struct;

struct hz2w_device {
	struct kref kref;
	atomic_t available;
	struct device *dev;
	struct usb_device *usb_dev;
	struct completion *quit_completion;
	/*
	 * 機器への制御転送の組 (カードの 1 命令や、起動の手順) と、周期処理を直列化する。
	 * 制御転送を 1 つずつ通すだけでは足りない。カードの命令は、複数の転送の組が 1 つの操作で、間に別の転送が入ると壊れる。
	 */
	struct mutex bus_lock;
	struct hz2w_bridge bridge;
	struct hz2w_card card;
	struct px4_card *card_dev;
	/* 周期処理 (0x4a のレジスタ 4 の読み出しと要求 0x22)。止まると、約 20 秒で機器が USB から消える。 */
	struct task_struct *keepalive_task;
	bool board_open;
	/* カードの 1 命令の最中は、制御転送の期限を命令の残り時間で打ち切る (bus_lock の中で設定する) */
	bool card_op_active;
	/* 切断または unbind の処理に入った。止める最中の転送の失敗は想定内なので、エラーとして記録しない。 */
	bool stopping;
};

int hz2w_device_init(struct hz2w_device *device, struct device *dev, struct usb_interface *intf,
		     struct completion *quit_completion);
void hz2w_device_term(struct hz2w_device *device);

#endif
