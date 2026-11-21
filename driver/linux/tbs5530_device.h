// SPDX-License-Identifier: GPL-2.0-only
#ifndef __TBS5530_DEVICE_H__
#define __TBS5530_DEVICE_H__

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/usb.h>

#include "itedtv_bus.h"
#include "ptx_chrdev.h"
#include "t230c_ts_sync.h"
#include "tbs5530_core.h"

#define TBS5530_CHRDEV_NUM 1U
/* 同期の確認に使う、連続した TS パケットの数だけの保存領域 */
#define TBS5530_TS_SYNC_BUFFER_SIZE (T230C_TS_PACKET_SIZE * T230C_TS_SYNC_COUNT)
#define TBS5530_TS_OUTPUT_SIZE (T230C_TS_PACKET_SIZE * 16U)

struct tbs5530_device;

struct tbs5530_stream_context {
	struct tbs5530_device *device;
	struct ptx_chrdev *chrdev;
	spinlock_t lock;
	struct t230c_ts_sync synchronizer;
	u8 sync_buffer[TBS5530_TS_SYNC_BUFFER_SIZE];
	u8 output_buffer[TBS5530_TS_OUTPUT_SIZE];
	size_t output_size;
};

struct tbs5530_device {
	struct kref kref;
	atomic_t available;
	struct device *dev;
	struct usb_device *usb_dev;
	struct completion *quit_completion;
	struct ptx_chrdev_group *chrdev_group;
	struct ptx_chrdev *chrdev;
	struct mutex operation_lock;
	struct mutex control_lock;
	struct itedtv_bus stream_bus;
	struct tbs5530_bridge bridge;
	struct tbs5530_demod demod;
	struct tbs5530_stream_context stream;
	unsigned int control_timeout_ms;
	bool bus_initialized;
	bool ready;
	bool capture_enabled;
	bool streaming;
};

int tbs5530_device_init(struct tbs5530_device *device, struct device *dev,
			struct usb_interface *intf,
			struct ptx_chrdev_context *chrdev_ctx,
			struct completion *quit_completion);
void tbs5530_device_term(struct tbs5530_device *device);

#endif
