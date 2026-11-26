// SPDX-License-Identifier: GPL-2.0-only
/*
 * I-O DATA GV-MVP/HZ2W (04bb:0526) の Linux バックエンド (内蔵 B-CAS カードのみ。地上波の受信は未対応)。
 *
 * 機器の仕組みと検証の範囲は docs/devices/hz2w.md。USB の制御転送は共通層 (driver/common/hz2w/) へ ops で渡す。
 * 対象は firmware 送信後の warm の機器だけで、cold (1738:5211) への firmware の送信は、このモジュールの対象外。
 */

#include "print_format.h"
#include "hz2w_device.h"

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb.h>

#define HZ2W_CTRL_TIMEOUT_MS 1000U
#define HZ2W_CTRL_MAX_LENGTH 256U
/* 周期処理の連続失敗がこの回数に達したら、機器が止まったとみて知らせる。 */
#define HZ2W_KEEPALIVE_FAILURE_REPORT 5U

static void hz2w_device_release(struct kref *kref);

/* ---- 制御転送 ---- */

static u64 hz2w_now_ms(void)
{
	return div_u64(ktime_get_ns(), NSEC_PER_MSEC);
}

/* 呼び出し側は bus_lock を持つ */
static int hz2w_ops_control(void *priv, u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length)
{
	struct hz2w_device *device = priv;
	unsigned int timeout_ms = HZ2W_CTRL_TIMEOUT_MS;
	unsigned int pipe;
	u8 *bounce = NULL;
	int ret;

	if (!device || !device->usb_dev || !atomic_read(&device->available))
		return -ENODEV;
	if (length > HZ2W_CTRL_MAX_LENGTH || (length && !buffer))
		return -EINVAL;
	/* カードの操作中は、各転送を操作の残り時間で打ち切る (操作全体の絶対期限を守る) */
	if (device->card_op_active && device->card.deadline_ms) {
		const u64 now = hz2w_now_ms();
		const u64 remaining = device->card.deadline_ms > now ? device->card.deadline_ms - now : 1;

		if (remaining < timeout_ms)
			timeout_ms = (unsigned int)remaining;
	}
	pipe = (request_type & USB_DIR_IN) ? usb_rcvctrlpipe(device->usb_dev, 0) : usb_sndctrlpipe(device->usb_dev, 0);
	if (length) {
		/* usb_control_msg の転送バッファは DMA 対象なので、スタック上の配列は渡せない。ヒープの一時領域を経由する。 */
		bounce = kmalloc(length, GFP_KERNEL);
		if (!bounce)
			return -ENOMEM;
		if (!(request_type & USB_DIR_IN))
			memcpy(bounce, buffer, length);
	}
	ret = usb_control_msg(device->usb_dev, pipe, request, request_type, value, index, bounce, length, timeout_ms);
	if (bounce) {
		if ((request_type & USB_DIR_IN) && ret > 0)
			memcpy(buffer, bounce, min_t(int, ret, length));
		kfree(bounce);
	}
	return ret;
}

static void hz2w_ops_msleep(void *priv, unsigned int milliseconds)
{
	(void)priv;
	msleep(milliseconds);
}

static u64 hz2w_ops_now_ms(void *priv)
{
	(void)priv;
	return hz2w_now_ms();
}

static void hz2w_ops_log(void *priv, int level, const char *format, ...)
{
	struct hz2w_device *device = priv;
	struct va_format vaf;
	va_list args;

	va_start(args, format);
	vaf.fmt = format;
	vaf.va = &args;
	if (level <= HZ2W_LOG_ERR && READ_ONCE(device->stopping))
		dev_dbg(device->dev, "%pV\n", &vaf);
	else if (level <= HZ2W_LOG_ERR)
		dev_err(device->dev, "%pV\n", &vaf);
	else if (level == HZ2W_LOG_INFO)
		dev_info(device->dev, "%pV\n", &vaf);
	else
		dev_dbg(device->dev, "%pV\n", &vaf);
	va_end(args);
}

static const struct hz2w_ops hz2w_operations = {
	.control = hz2w_ops_control,
	.msleep = hz2w_ops_msleep,
	.now_ms = hz2w_ops_now_ms,
	.log = hz2w_ops_log,
};

/*
 * bus_lock を取る。眠って待ち、シグナルで中断されたときは、機器へ触れる前に -ERESTARTSYS を返す。
 * 切断の後や、機器を閉じた後は、取らずに -ENODEV を返す。
 */
static int hz2w_bus_lock(struct hz2w_device *device)
{
	if (mutex_lock_interruptible(&device->bus_lock))
		return -ERESTARTSYS;
	if (!atomic_read(&device->available) || !device->board_open) {
		mutex_unlock(&device->bus_lock);
		return -ENODEV;
	}
	return 0;
}

/* ---- 周期処理 ---- */

/*
 * 起動の手順を送った後は、周期処理を止めない。止まると約 20 秒で機器が USB から消える (docs/devices/hz2w.md)。
 * 終了のときは、閉じる手順 (hz2w_board_close) を送ってから止める (逆順だと機器が消える)。
 */
static int hz2w_keepalive_thread(void *data)
{
	struct hz2w_device *device = data;
	u64 next_state = hz2w_now_ms();
	u64 next_watchdog = next_state;
	unsigned int failures = 0;

	while (!kthread_should_stop()) {
		u64 now = hz2w_now_ms();
		const u64 wake = min(next_state, next_watchdog);
		int ret = 0;

		if (wake > now) {
			schedule_timeout_interruptible(msecs_to_jiffies(wake - now));
			continue;
		}
		mutex_lock(&device->bus_lock);
		/* 閉じる手順を済ませた後は、何も送らない */
		if (device->board_open && atomic_read(&device->available)) {
			now = hz2w_now_ms();
			if (now >= next_state) {
				ret = hz2w_keepalive_card_state(&device->bridge);
				next_state = now + HZ2W_KEEPALIVE_CARD_PERIOD_US / 1000;
			}
			/* watchdog はカード状態の成否と独立に送る (状態の要求が続けて失敗しても 0x22 を止めない) */
			if (ret != -ENODEV && now >= next_watchdog) {
				const int wd = hz2w_keepalive_watchdog(&device->bridge);

				next_watchdog = now + HZ2W_KEEPALIVE_WATCHDOG_PERIOD_US / 1000;
				if (!ret || wd == -ENODEV)
					ret = wd;
			}
		}
		mutex_unlock(&device->bus_lock);
		if (ret == -ENODEV) {
			/* kthread_stop() が来るまで待つ (自分で終了すると、kthread_stop() が解放済みの領域に触れる) */
			while (!kthread_should_stop())
				msleep(10);
			break;
		}
		if (ret) {
			if (++failures == HZ2W_KEEPALIVE_FAILURE_REPORT && !READ_ONCE(device->stopping))
				dev_err(device->dev, "HZ2W keep-alive requests keep failing (%d); the device may disappear from USB\n", ret);
		} else {
			failures = 0;
		}
	}
	return 0;
}

static void hz2w_stop_keepalive(struct hz2w_device *device)
{
	if (device->keepalive_task) {
		kthread_stop(device->keepalive_task);
		device->keepalive_task = NULL;
	}
}

/* ---- カード ---- */

static int hz2w_card_power_on(void *context)
{
	struct hz2w_device *device = context;

	return atomic_read(&device->available) && device->board_open ? 0 : -ENODEV;
}

static int hz2w_card_power_off(void *context)
{
	struct hz2w_device *device = context;
	bool power_down_failed = false;
	int ret;

	/* USB 切断後は、解放済みの機器へ転送しない。 */
	if (!atomic_read(&device->available))
		return 0;
	mutex_lock(&device->bus_lock);
	if (device->board_open) {
		ret = hz2w_card_power_down(&device->card);
		if (ret) {
			dev_warn(device->dev, "HZ2W card power down failed: %d\n", ret);
			/* 機器が消えたのでなければ、次の reset が電源断をやり直せるよう powered を残す */
			power_down_failed = ret != -ENODEV;
		}
	}
	hz2w_card_discard(&device->card);
	if (power_down_failed)
		device->card.powered = true;
	mutex_unlock(&device->bus_lock);
	return 0;
}

static int hz2w_backend_card_set_timeout(void *context, u32 timeout_ms)
{
	struct hz2w_device *device = context;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	hz2w_card_set_timeout(&device->card, timeout_ms);
	mutex_unlock(&device->bus_lock);
	return 0;
}

static int hz2w_backend_card_detect(void *context, bool *present)
{
	struct hz2w_device *device = context;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_detect(&device->card, present);
	mutex_unlock(&device->bus_lock);
	return ret;
}

static int hz2w_backend_card_reset(void *context)
{
	struct hz2w_device *device = context;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	device->card_op_active = true;
	ret = hz2w_card_reset(&device->card);
	device->card_op_active = false;
	mutex_unlock(&device->bus_lock);
	return ret;
}

/* ASIE5606 がボーレートを自動で扱う。要求は受け付けるだけで、機器へは何も送らない (HZ2W と同じ)。 */
static int hz2w_backend_card_set_baudrate(void *context, u32 baudrate)
{
	struct hz2w_device *device = context;

	if (!atomic_read(&device->available))
		return -ENODEV;
	return baudrate == 9600 || baudrate == 19200 || baudrate == 38400 ? 0 : -EINVAL;
}

static int hz2w_backend_card_rx_ready(void *context, bool *ready)
{
	struct hz2w_device *device = context;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_rx_ready(&device->card, ready);
	mutex_unlock(&device->bus_lock);
	return ret;
}

static int hz2w_backend_card_read(void *context, struct px4_card_frame *frame)
{
	struct hz2w_device *device = context;
	u8 length = frame->length;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_read(&device->card, frame->data, &length);
	mutex_unlock(&device->bus_lock);
	if (!ret)
		frame->length = length;
	return ret;
}

static int hz2w_backend_card_write(void *context, const struct px4_card_frame *frame)
{
	struct hz2w_device *device = context;
	int ret = hz2w_bus_lock(device);

	if (ret)
		return ret;
	device->card_op_active = true;
	ret = hz2w_card_write(&device->card, frame->data, frame->length);
	device->card_op_active = false;
	mutex_unlock(&device->bus_lock);
	return ret;
}

static const struct px4_card_operations hz2w_card_operations = {
	.power_on = hz2w_card_power_on,
	.power_off = hz2w_card_power_off,
	.set_timeout = hz2w_backend_card_set_timeout,
	.detect = hz2w_backend_card_detect,
	.reset = hz2w_backend_card_reset,
	.set_baudrate = hz2w_backend_card_set_baudrate,
	.rx_ready = hz2w_backend_card_rx_ready,
	.read = hz2w_backend_card_read,
	.write = hz2w_backend_card_write,
};

/* ---- 寿命 ---- */

static void hz2w_device_release(struct kref *kref)
{
	struct hz2w_device *device = container_of(kref, struct hz2w_device, kref);

	mutex_destroy(&device->bus_lock);
	put_device(device->dev);
	complete(device->quit_completion);
}

/* 閉じる手順を送る。機器が消えていれば、何も送らない。周期処理は、この後で止める。 */
static void hz2w_close_board(struct hz2w_device *device)
{
	mutex_lock(&device->bus_lock);
	if (device->board_open) {
		if (atomic_read(&device->available)) {
			hz2w_card_power_down(&device->card);
			hz2w_board_close(&device->bridge);
		}
		device->board_open = false;
	}
	hz2w_card_discard(&device->card);
	mutex_unlock(&device->bus_lock);
}

int hz2w_device_init(struct hz2w_device *device, struct device *dev, struct usb_interface *intf,
		     struct completion *quit_completion)
{
	struct usb_device *usb_dev;
	int ret;

	if (!device || !dev || !intf || !quit_completion)
		return -EINVAL;
	usb_dev = interface_to_usbdev(intf);
	ret = usb_set_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber, 0);
	if (ret)
		return ret;
	memset(device, 0, sizeof(*device));
	get_device(dev);
	kref_init(&device->kref);
	atomic_set(&device->available, 0);
	device->dev = dev;
	device->usb_dev = usb_dev;
	device->quit_completion = quit_completion;
	mutex_init(&device->bus_lock);
	hz2w_bridge_init(&device->bridge, &hz2w_operations, device);
	hz2w_card_init(&device->card, &device->bridge);
	/* 起動中も制御転送を受け付ける。失敗したら閉じる手順で戻す。 */
	atomic_set(&device->available, 1);
	mutex_lock(&device->bus_lock);
	ret = hz2w_board_open(&device->bridge);
	if (ret) {
		dev_err(dev, "HZ2W board setup failed: %d\n", ret);
		/* 起動の途中まで進むと、周期処理を始めない限り約 20 秒で機器が消えるため、閉じる手順で戻す。 */
		hz2w_board_close(&device->bridge);
		mutex_unlock(&device->bus_lock);
		goto fail;
	}
	device->board_open = true;
	mutex_unlock(&device->bus_lock);
	device->keepalive_task = kthread_run(hz2w_keepalive_thread, device, "hz2w_keepalive");
	if (IS_ERR(device->keepalive_task)) {
		ret = PTR_ERR(device->keepalive_task);
		device->keepalive_task = NULL;
		hz2w_close_board(device);
		goto fail;
	}
	{
		struct px4_card_backend card_backend = {
			.it930x = NULL,
			.operations = &hz2w_card_operations,
			.lock = &device->bus_lock,
			.available = &device->available,
			.context = device,
		};

		ret = px4_card_register(dev, &card_backend, &device->card_dev);
		if (ret)
			dev_warn(dev, "Card reader registration failed: %d\n", ret);
	}
	dev_info(dev, "HZ2W ready (card reader only)\n");
	return 0;

fail:
	atomic_set(&device->available, 0);
	mutex_destroy(&device->bus_lock);
	put_device(dev);
	return ret;
}

void hz2w_device_term(struct hz2w_device *device)
{
	/*
	 * カードを外し、機器へ閉じる手順を送ってから、周期処理を止め、受付を閉じる。
	 * 閉じる手順を送らずに周期処理を止めると、機器が USB から消え、挿し直すまで現れない。
	 */
	WRITE_ONCE(device->stopping, true);
	px4_card_unregister(device->card_dev);
	device->card_dev = NULL;
	hz2w_close_board(device);
	hz2w_stop_keepalive(device);
	atomic_xchg(&device->available, 0);
	kref_put(&device->kref, hz2w_device_release);
}
