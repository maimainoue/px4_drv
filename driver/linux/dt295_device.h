// SPDX-License-Identifier: GPL-2.0-only
#ifndef __DT295_DEVICE_H__
#define __DT295_DEVICE_H__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/usb.h>

#include "dt295_core.h"
#include "itedtv_bus.h"
#include "px4_card.h"
#include "ptx_chrdev.h"
#include "torne_card.h"
#include "torne_core.h"

#define DT295_CHRDEV_NUM 1U

struct task_struct;

struct dt295_device {
	struct kref kref;
	atomic_t available;
	struct device *dev;
	struct usb_device *usb_dev;
	struct completion *quit_completion;
	struct ptx_chrdev_group *chrdev_group;
	struct ptx_chrdev *chrdev;
	/* ptx_chrdev の open / release / tune / set_capture / 読み出しを直列化する */
	struct mutex operation_lock;
	/* USB 制御転送を 1 つずつ通す。ポンプの要求と応答の組の間も保持する。 */
	struct mutex control_lock;
	struct mutex card_uart_lock;
	/* 制御転送の順番待ちの数。ポンプのスレッドは、待ちがいるとき次の周期を遅らせて譲る。 */
	atomic_t control_waiters;
	struct itedtv_bus stream_bus;
	struct dt295_bridge bridge;
	struct torne_card_port card_port;
	struct px4_card *card;
	struct torne_framer *framer;
	u8 *ts_output;
	spinlock_t stream_lock;
	struct task_struct *pump_task;
	bool bus_initialized;
	bool board_up;
	bool ready;
	bool capture_enabled;
	bool streaming;
	/*
	 * 切断または unbind の処理に入った。USB コアは disconnect の前に URB を止めるので、止める最中のポンプは待ち時間切れになる。
	 * その失敗は想定内なので、エラーとして記録しない。
	 */
	bool stopping;
	/* hook や窓を元へ戻せなかった。機材を挿し直すまで、受信を開始しない。 */
	bool needs_replug;
	/* 選局の後、最初にロックを確認した時刻。ロックの直後は、TMCC が確定するまで TEI 付きのパケットが出るので、受信の開始を遅らせる。 */
	bool lock_seen;
	unsigned long lock_time;
};

int dt295_device_init(struct dt295_device *device, struct device *dev, struct usb_interface *intf,
		      struct ptx_chrdev_context *chrdev_ctx, struct completion *quit_completion);
void dt295_device_term(struct dt295_device *device);

#endif
