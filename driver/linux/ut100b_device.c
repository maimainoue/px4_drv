// SPDX-License-Identifier: GPL-2.0-only
/*
 * Quixun QRS-UT100B (3275:7020) の Linux バックエンド (地上波と内蔵 B-CAS カード)。
 *
 * 機器の仕組みと検証の範囲は docs/devices/ut100b.md。USB の制御転送は共通層 (driver/common/ut100b/、driver/common/hz2w/) へ ops で渡す。
 * TS は bulk IN (0x81) から 16 KB の URB で受け、ASIE5606 が 1 パケットごとに DES でかけた暗号を、URB の完了処理で外して 188 バイトに区切る。
 * 対象は firmware 送信後の warm の機器だけで、cold (1738:5211) への firmware の送信は、このモジュールの対象外 (HZ2W と ID が同じで、区別できない)。
 */

#include "print_format.h"
#include "ut100b_device.h"

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>

#include "px4_device_params.h"
#include "px4_usb_params.h"

#define UT100B_CTRL_TIMEOUT_MS 1000U
#define UT100B_CTRL_MAX_LENGTH 256U
/* 受信できる周波数 (kHz)。UHF 13〜62ch の中心は 473143〜767143。CATV を含めて、これより外は選局しない。 */
#define UT100B_MIN_FREQ_KHZ 111000U
#define UT100B_MAX_FREQ_KHZ 770000U
/* ロックしてから復調が安定するまでの待ち。DriverHost_PX4 の既定値 (Ut100bLockSettleMs) と同じ。 */
#define UT100B_LOCK_SETTLE_MS 300U
/* 周期処理の連続失敗がこの回数に達したら、機器が止まったとみて知らせる。 */
#define UT100B_KEEPALIVE_FAILURE_REPORT 5U

static void ut100b_device_release(struct kref *kref);

/* ---- 制御転送 ---- */

static u64 ut100b_now_ms(void)
{
	return div_u64(ktime_get_ns(), NSEC_PER_MSEC);
}

/* 呼び出し側は bus_lock を持つ */
static int ut100b_ops_control(void *priv, u8 request_type, u8 request, u16 value, u16 index, void *buffer, u16 length)
{
	struct ut100b_device *device = priv;
	unsigned int timeout_ms = UT100B_CTRL_TIMEOUT_MS;
	unsigned int pipe;
	u8 *bounce = NULL;
	int ret;

	if (!device || !device->usb_dev || !atomic_read(&device->available))
		return -ENODEV;
	if (length > UT100B_CTRL_MAX_LENGTH || (length && !buffer))
		return -EINVAL;
	/* カードの操作中は、各転送を操作の残り時間で打ち切る (操作全体の絶対期限を守る) */
	if (device->card_op_active && device->card.deadline_ms) {
		const u64 now = ut100b_now_ms();
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

static void ut100b_ops_msleep(void *priv, unsigned int milliseconds)
{
	(void)priv;
	msleep(milliseconds);
}

static u64 ut100b_ops_now_ms(void *priv)
{
	(void)priv;
	return ut100b_now_ms();
}

static void ut100b_ops_log(void *priv, int level, const char *format, ...)
{
	struct ut100b_device *device = priv;
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

static const struct hz2w_ops ut100b_operations = {
	.control = ut100b_ops_control,
	.msleep = ut100b_ops_msleep,
	.now_ms = ut100b_ops_now_ms,
	.log = ut100b_ops_log,
};

/*
 * bus_lock を取る。眠って待ち、シグナルで中断されたときは、機器へ触れる前に -ERESTARTSYS を返す。
 * 切断の後や、機器を閉じた後は、取らずに -ENODEV を返す。
 */
static int ut100b_bus_lock(struct ut100b_device *device)
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
 * 終了のときは、閉じる手順 (ut100b_board_close) を送ってから止める (逆順だと機器が消える)。
 */
static int ut100b_keepalive_thread(void *data)
{
	struct ut100b_device *device = data;
	u64 next_state = ut100b_now_ms();
	u64 next_watchdog = next_state;
	unsigned int failures = 0;

	while (!kthread_should_stop()) {
		u64 now = ut100b_now_ms();
		const u64 wake = min(next_state, next_watchdog);
		int ret = 0;

		if (wake > now) {
			schedule_timeout_interruptible(msecs_to_jiffies(wake - now));
			continue;
		}
		mutex_lock(&device->bus_lock);
		/* 閉じる手順を済ませた後は、何も送らない */
		if (device->board_open && atomic_read(&device->available)) {
			now = ut100b_now_ms();
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
			if (++failures == UT100B_KEEPALIVE_FAILURE_REPORT && !READ_ONCE(device->stopping))
				dev_err(device->dev, "UT100B keep-alive requests keep failing (%d); the device may disappear from USB\n", ret);
		} else {
			failures = 0;
		}
	}
	return 0;
}

static void ut100b_stop_keepalive(struct ut100b_device *device)
{
	if (device->keepalive_task) {
		kthread_stop(device->keepalive_task);
		device->keepalive_task = NULL;
	}
}

/* ---- カード ---- */

static int ut100b_card_power_on(void *context)
{
	struct ut100b_device *device = context;

	return atomic_read(&device->available) && device->board_open ? 0 : -ENODEV;
}

static int ut100b_card_power_off(void *context)
{
	struct ut100b_device *device = context;
	bool power_down_failed = false;
	int ret;

	/* USB 切断後は、解放済みの機器へ転送しない。 */
	if (!atomic_read(&device->available))
		return 0;
	mutex_lock(&device->bus_lock);
	if (device->board_open) {
		ret = hz2w_card_power_down(&device->card);
		if (ret) {
			/* 機器が消えたとき (-ENODEV) は想定内なので、警告しない */
			if (ret != -ENODEV)
				dev_warn(device->dev, "UT100B card power down failed: %d\n", ret);
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

static int ut100b_backend_card_set_timeout(void *context, u32 timeout_ms)
{
	struct ut100b_device *device = context;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	hz2w_card_set_timeout(&device->card, timeout_ms);
	mutex_unlock(&device->bus_lock);
	return 0;
}

static int ut100b_backend_card_detect(void *context, bool *present)
{
	struct ut100b_device *device = context;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_detect(&device->card, present);
	mutex_unlock(&device->bus_lock);
	return ret;
}

static int ut100b_backend_card_reset(void *context)
{
	struct ut100b_device *device = context;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	device->card_op_active = true;
	ret = hz2w_card_reset(&device->card);
	device->card_op_active = false;
	mutex_unlock(&device->bus_lock);
	return ret;
}

/* ASIE5606 がボーレートを自動で扱う。要求は受け付けるだけで、機器へは何も送らない (HZ2W と同じ)。 */
static int ut100b_backend_card_set_baudrate(void *context, u32 baudrate)
{
	struct ut100b_device *device = context;

	if (!atomic_read(&device->available))
		return -ENODEV;
	return baudrate == 9600 || baudrate == 19200 || baudrate == 38400 ? 0 : -EINVAL;
}

static int ut100b_backend_card_rx_ready(void *context, bool *ready)
{
	struct ut100b_device *device = context;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_rx_ready(&device->card, ready);
	mutex_unlock(&device->bus_lock);
	return ret;
}

static int ut100b_backend_card_read(void *context, struct px4_card_frame *frame)
{
	struct ut100b_device *device = context;
	u8 length = frame->length;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	ret = hz2w_card_read(&device->card, frame->data, &length);
	mutex_unlock(&device->bus_lock);
	if (!ret)
		frame->length = length;
	return ret;
}

static int ut100b_backend_card_write(void *context, const struct px4_card_frame *frame)
{
	struct ut100b_device *device = context;
	int ret = ut100b_bus_lock(device);

	if (ret)
		return ret;
	device->card_op_active = true;
	ret = hz2w_card_write(&device->card, frame->data, frame->length);
	device->card_op_active = false;
	mutex_unlock(&device->bus_lock);
	return ret;
}

static const struct px4_card_operations ut100b_card_operations = {
	.power_on = ut100b_card_power_on,
	.power_off = ut100b_card_power_off,
	.set_timeout = ut100b_backend_card_set_timeout,
	.detect = ut100b_backend_card_detect,
	.reset = ut100b_backend_card_reset,
	.set_baudrate = ut100b_backend_card_set_baudrate,
	.rx_ready = ut100b_backend_card_rx_ready,
	.read = ut100b_backend_card_read,
	.write = ut100b_backend_card_write,
};

/* ---- TS の経路 ---- */

/*
 * URB の完了処理から呼ばれる。DES の復号 (1 URB あたり 87 パケット) もここで行う。
 * URB は順序どおりに 1 つずつ完了するため、aligner の状態に別のロックは要らない (止める・始めるは、URB が無いときだけ)。
 */
static int ut100b_stream_handler(void *context, void *buffer, u32 length)
{
	struct ut100b_device *device = context;
	size_t output_size;

	if (!device || !buffer || !length || !device->chrdev || !device->aligner || !device->ts_output)
		return 0;
	if (length > UT100B_URB_SIZE)
		return -EOVERFLOW;
	if (!READ_ONCE(device->capture_enabled))
		return 0;
	output_size = ut100b_aligner_feed(device->aligner, buffer, length, device->ts_output);
	if (output_size)
		ptx_chrdev_put_stream(device->chrdev, device->ts_output, output_size);
	return 0;
}

/*
 * TS を流し始め、URB の読み出しを始める。
 * 機器の中の FIFO は、流し始めた時点から溜まるため、ロックと安定の待ちが済み、読み出しを始める直前に流し始める。
 * 呼び出し側は operation_lock を持つ。
 */
static int ut100b_start_stream(struct ut100b_device *device)
{
	int ret;

	if (device->streaming)
		return 0;
	if (!atomic_read(&device->available) || !device->board_open)
		return -ENODEV;
	/* 選局の直後は、ロックから一定の時間が経つまで待つ (ロックしていない搬送波は待たない) */
	if (device->lock_seen) {
		const unsigned long until = device->lock_time + msecs_to_jiffies(UT100B_LOCK_SETTLE_MS);
		/* 判定と待ち時間の計算で、jiffies を 2 回読むと、間に進んだ分だけ引き算が符号なしでラップして、非常に長く眠る */
		const unsigned long now = jiffies;

		if (time_before(now, until))
			msleep(jiffies_to_msecs(until - now));
	}
	ret = ut100b_bus_lock(device);
	if (ret)
		return ret;
	ret = ut100b_stream_start(&device->bridge);
	mutex_unlock(&device->bus_lock);
	if (ret)
		return ret;
	ut100b_aligner_init(device->aligner);
	ret = itedtv_bus_start_streaming(&device->stream_bus, ut100b_stream_handler, device);
	if (ret) {
		if (!ut100b_bus_lock(device)) {
			ut100b_stream_stop(&device->bridge);
			mutex_unlock(&device->bus_lock);
		}
		return ret;
	}
	device->streaming = true;
	return 0;
}

/* 読み出しを止め、機器の TS も止める (止めている間に、機器の FIFO を満杯にしない)。切断後 (available が 0) は、機器へ書かない。 */
static int ut100b_stop_stream(struct ut100b_device *device)
{
	int ret = 0, step;

	if (device->streaming)
		ret = itedtv_bus_stop_streaming(&device->stream_bus);
	if (device->streaming && atomic_read(&device->available)) {
		step = ut100b_bus_lock(device);
		if (!step) {
			step = ut100b_stream_stop(&device->bridge);
			mutex_unlock(&device->bus_lock);
		}
		if (!ret)
			ret = step;
	}
	device->streaming = false;
	return ret;
}

/* ---- ptx_chrdev ---- */

static int ut100b_chrdev_init(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int ut100b_chrdev_term(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int ut100b_chrdev_open(struct ptx_chrdev *chrdev)
{
	struct ut100b_device *device = chrdev->priv;
	int ret;

	mutex_lock(&device->operation_lock);
	ret = atomic_read(&device->available) && device->board_open ? 0 : -ENODEV;
	mutex_unlock(&device->operation_lock);
	if (!ret)
		kref_get(&device->kref);
	return ret;
}

static int ut100b_chrdev_release(struct ptx_chrdev *chrdev)
{
	struct ut100b_device *device = chrdev->priv;

	mutex_lock(&device->operation_lock);
	WRITE_ONCE(device->capture_enabled, false);
	ut100b_stop_stream(device);
	mutex_unlock(&device->operation_lock);
	kref_put(&device->kref, ut100b_device_release);
	return 0;
}

static int ut100b_chrdev_tune(struct ptx_chrdev *chrdev, struct ptx_tune_params *params)
{
	struct ut100b_device *device = chrdev->priv;
	bool resume;
	int ret;

	if (!params || params->system != PTX_ISDB_T_SYSTEM)
		return -EINVAL;
	if (params->freq < UT100B_MIN_FREQ_KHZ || params->freq > UT100B_MAX_FREQ_KHZ)
		return -EINVAL;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->board_open) {
		ret = -ENODEV;
		goto out;
	}
	/* 受信中の選局は、前のチャンネルの TS を引き継がないよう、いったん止めてから行う */
	resume = device->streaming;
	if (resume)
		ut100b_stop_stream(device);
	device->lock_seen = false;
	ret = ut100b_bus_lock(device);
	if (!ret) {
		ret = ut100b_tune(&device->bridge, params->freq);
		mutex_unlock(&device->bus_lock);
	}
	if (resume) {
		const int restart_ret = ut100b_start_stream(device);

		if (!ret)
			ret = restart_ret;
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int ut100b_chrdev_check_lock(struct ptx_chrdev *chrdev, bool *locked)
{
	struct ut100b_device *device = chrdev->priv;
	int ret;

	if (!locked)
		return -EINVAL;
	*locked = false;
	mutex_lock(&device->operation_lock);
	ret = ut100b_bus_lock(device);
	if (!ret) {
		ret = ut100b_is_locked(&device->bridge, locked);
		mutex_unlock(&device->bus_lock);
	}
	if (!ret && *locked && !device->lock_seen) {
		device->lock_seen = true;
		device->lock_time = jiffies;
	}
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int ut100b_chrdev_set_lnb_voltage(struct ptx_chrdev *chrdev, int voltage)
{
	(void)chrdev;
	return voltage == 0 ? 0 : -EOPNOTSUPP;
}

static int ut100b_chrdev_set_capture(struct ptx_chrdev *chrdev, bool capture)
{
	struct ut100b_device *device = chrdev->priv;
	int ret = 0;

	mutex_lock(&device->operation_lock);
	if (capture) {
		if (!atomic_read(&device->available) || !device->board_open) {
			ret = -ENODEV;
			goto out;
		}
		WRITE_ONCE(device->capture_enabled, true);
		ret = ut100b_start_stream(device);
		if (ret)
			WRITE_ONCE(device->capture_enabled, false);
	} else {
		WRITE_ONCE(device->capture_enabled, false);
		ret = ut100b_stop_stream(device);
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int ut100b_chrdev_read_cnr_raw(struct ptx_chrdev *chrdev, u32 *value)
{
	struct ut100b_device *device = chrdev->priv;
	bool locked = false;
	int ret;

	if (!value)
		return -EINVAL;
	*value = 0;
	mutex_lock(&device->operation_lock);
	ret = ut100b_bus_lock(device);
	if (!ret) {
		ret = ut100b_is_locked(&device->bridge, &locked);
		if (!ret && !locked)
			ret = -EAGAIN;
		if (!ret)
			ret = ut100b_read_cndat(&device->bridge, value);
		mutex_unlock(&device->bus_lock);
	}
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int ut100b_chrdev_read_signal_strength(struct ptx_chrdev *chrdev, u32 *value)
{
	(void)chrdev;
	/* チューナーの強度の換算は未確認。 */
	if (!value)
		return -EINVAL;
	*value = 0;
	return -EOPNOTSUPP;
}

static const struct ptx_chrdev_operations ut100b_chrdev_ops = {
	.init = ut100b_chrdev_init,
	.term = ut100b_chrdev_term,
	.open = ut100b_chrdev_open,
	.release = ut100b_chrdev_release,
	.tune = ut100b_chrdev_tune,
	.check_lock = ut100b_chrdev_check_lock,
	.set_stream_id = NULL,
	.set_lnb_voltage = ut100b_chrdev_set_lnb_voltage,
	.set_capture = ut100b_chrdev_set_capture,
	.read_signal_strength = ut100b_chrdev_read_signal_strength,
	.read_cnr = NULL,
	.read_cnr_raw = ut100b_chrdev_read_cnr_raw,
};

/* ---- 寿命 ---- */

static void ut100b_device_release(struct kref *kref)
{
	struct ut100b_device *device = container_of(kref, struct ut100b_device, kref);

	if (device->streaming)
		itedtv_bus_stop_streaming(&device->stream_bus);
	if (device->bus_initialized)
		itedtv_bus_term(&device->stream_bus);
	kvfree(device->aligner);
	kvfree(device->ts_output);
	mutex_destroy(&device->bus_lock);
	mutex_destroy(&device->operation_lock);
	put_device(device->dev);
	complete(device->quit_completion);
}

/* 閉じる手順を送る。機器が消えていれば、何も送らない。周期処理は、この後で止める。 */
static void ut100b_close_board(struct ut100b_device *device)
{
	mutex_lock(&device->bus_lock);
	if (device->board_open) {
		if (atomic_read(&device->available)) {
			hz2w_card_power_down(&device->card);
			ut100b_board_close(&device->bridge);
		}
		device->board_open = false;
	}
	hz2w_card_discard(&device->card);
	mutex_unlock(&device->bus_lock);
}

int ut100b_device_init(struct ut100b_device *device, struct device *dev, struct usb_interface *intf,
		       struct ptx_chrdev_context *chrdev_ctx, struct completion *quit_completion)
{
	struct ptx_chrdev_config chrdev_config = { 0 };
	struct ptx_chrdev_group_config group_config = { 0 };
	struct ptx_chrdev_group *chrdev_group;
	struct usb_device *usb_dev;
	int ret;

	if (!device || !dev || !intf || !chrdev_ctx || !quit_completion)
		return -EINVAL;
	usb_dev = interface_to_usbdev(intf);
	/* 複合デバイス (MI_00: 映像、MI_01: リモコン)。受信は MI_00 だけが使う。 */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;
	ret = usb_set_interface(usb_dev, 0, 0);
	if (ret)
		return ret;
	memset(device, 0, sizeof(*device));
	get_device(dev);
	kref_init(&device->kref);
	atomic_set(&device->available, 0);
	device->dev = dev;
	device->usb_dev = usb_dev;
	device->quit_completion = quit_completion;
	mutex_init(&device->operation_lock);
	mutex_init(&device->bus_lock);
	device->aligner = kvzalloc(sizeof(*device->aligner), GFP_KERNEL);
	device->ts_output = kvzalloc(UT100B_ALIGNED_OUT_SIZE, GFP_KERNEL);
	if (!device->aligner || !device->ts_output) {
		ret = -ENOMEM;
		goto fail_device;
	}
	device->stream_bus.dev = dev;
	device->stream_bus.type = ITEDTV_BUS_USB;
	device->stream_bus.usb.dev = usb_dev;
	device->stream_bus.usb.ctrl_timeout = UT100B_CTRL_TIMEOUT_MS;
	device->stream_bus.usb.streaming.endpoint = UT100B_STREAM_ENDPOINT;
	device->stream_bus.usb.streaming.urb_buffer_size = UT100B_URB_SIZE;
	device->stream_bus.usb.streaming.urb_num = UT100B_URB_COUNT;
	device->stream_bus.usb.streaming.no_dma = px4_usb_params.no_dma;
	ret = itedtv_bus_init(&device->stream_bus);
	if (ret)
		goto fail_device;
	device->bus_initialized = true;

	hz2w_bridge_init(&device->bridge, &ut100b_operations, device);
	ut100b_card_init(&device->card, &device->bridge);
	/* 起動中も制御転送を受け付ける。失敗したら fail_board で戻す。 */
	atomic_set(&device->available, 1);
	mutex_lock(&device->bus_lock);
	ret = ut100b_board_open(&device->bridge);
	if (ret) {
		dev_err(dev, "UT100B board setup failed: %d\n", ret);
		/* 起動の途中まで進むと、周期処理を始めない限り約 20 秒で機器が消えるため、閉じる手順で戻す。 */
		ut100b_board_close(&device->bridge);
		mutex_unlock(&device->bus_lock);
		goto fail_board;
	}
	device->board_open = true;
	mutex_unlock(&device->bus_lock);
	device->keepalive_task = kthread_run(ut100b_keepalive_thread, device, "ut100b_keepalive");
	if (IS_ERR(device->keepalive_task)) {
		ret = PTR_ERR(device->keepalive_task);
		device->keepalive_task = NULL;
		goto fail_keepalive;
	}

	chrdev_config.system_cap = PTX_ISDB_T_SYSTEM;
	chrdev_config.ops = &ut100b_chrdev_ops;
	chrdev_config.ringbuf_size = 188 * px4_device_params.tsdev_max_packets;
	chrdev_config.ringbuf_threshold_size = chrdev_config.ringbuf_size / 10;
	chrdev_config.priv = device;
	group_config.owner_kref = &device->kref;
	group_config.owner_kref_release = ut100b_device_release;
	group_config.reserved = false;
	group_config.minor_base = 0;
	group_config.chrdev_num = UT100B_CHRDEV_NUM;
	group_config.chrdev_config = &chrdev_config;
	ret = ptx_chrdev_context_add_group(chrdev_ctx, dev, &group_config, &chrdev_group);
	if (ret)
		goto fail_keepalive;
	device->chrdev_group = chrdev_group;
	device->chrdev = &chrdev_group->chrdev[0];
	{
		struct px4_card_backend card_backend = {
			.it930x = NULL,
			.operations = &ut100b_card_operations,
			.lock = &device->operation_lock,
			.available = &device->available,
			.context = device,
		};

		ret = px4_card_register(dev, &card_backend, &device->card_dev);
		if (ret)
			dev_warn(dev, "Card reader registration failed: %d\n", ret);
	}
	dev_info(dev, "UT100B receiver registered (bulk IN 0x81)\n");
	return 0;

fail_keepalive:
	ut100b_close_board(device);
	ut100b_stop_keepalive(device);
fail_board:
	atomic_set(&device->available, 0);
fail_device:
	if (device->bus_initialized) {
		itedtv_bus_term(&device->stream_bus);
		device->bus_initialized = false;
	}
	kvfree(device->aligner);
	kvfree(device->ts_output);
	mutex_destroy(&device->bus_lock);
	mutex_destroy(&device->operation_lock);
	put_device(dev);
	return ret;
}

void ut100b_device_term(struct ut100b_device *device)
{
	/*
	 * カードを外し、受信を止めて、機器へ閉じる手順を送ってから、周期処理を止め、受付を閉じる。
	 * 閉じる手順を送らずに周期処理を止めると、機器が USB から消え、挿し直すまで現れない。
	 * 機器が取り外されたときは、USB コアが転送を速やかに失敗させるので、手順は短時間で終わる。
	 */
	WRITE_ONCE(device->stopping, true);
	px4_card_unregister(device->card_dev);
	device->card_dev = NULL;
	mutex_lock(&device->operation_lock);
	WRITE_ONCE(device->capture_enabled, false);
	ut100b_stop_stream(device);
	mutex_unlock(&device->operation_lock);
	ut100b_close_board(device);
	ut100b_stop_keepalive(device);
	mutex_lock(&device->operation_lock);
	atomic_xchg(&device->available, 0);
	mutex_unlock(&device->operation_lock);
	ptx_chrdev_group_destroy(device->chrdev_group);
	kref_put(&device->kref, ut100b_device_release);
}
