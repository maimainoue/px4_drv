// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIX-DT295 (06b8:103b、06b8:105c) の Linux バックエンド (地上波)。
 *
 * 設計: dt295_kernel_design.md。USB の制御転送は共通層 (driver/common/dt295/) へ ops で渡す。
 * TS は、機材内の自作プログラムが SDRAM のリングから bulk IN (0x81) へ送り、ホスト側のスレッドが
 * 書き込み位置と読み出し位置を窓越しに扱う。ストリームの整形は torne と同じ形式 (192 バイトのユニット)。
 */

#include "print_format.h"
#include "dt295_device.h"

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

#include "dt295_card.h"
#include "px4_device_params.h"
#include "px4_usb_params.h"

#define DT295_CTRL_TIMEOUT_MS 1000U
#define DT295_CTRL_TIMEOUT_MAX_MS 5000U
#define DT295_STREAM_ENDPOINT 0x81U
/* 非同期転送は 8 KiB (16 パケット) の 16 本。512 バイトずつの同期読み出しは、約 29 Mbps で頭打ちになる。 */
#define DT295_STREAM_URB_SIZE (DT295_PACKET_BYTES * DT295_PUMP_UNIT_PACKETS)
#define DT295_STREAM_URB_NUM 16U
#define DT295_DRAIN_MAX_READS 16U
#define DT295_DRAIN_TIMEOUT_MS 20U
#define DT295_OUTPUT_SIZE (DT295_STREAM_URB_SIZE + DT295_TS_PACKET_SIZE)
#define DT295_PUMP_ERROR_LIMIT 3
/* ロックしてから TMCC が確定するまでの待ち (実測: ロックの約 0.4 秒後まで TEI 付きのパケットが出る) */
#define DT295_LOCK_SETTLE_MS 500U

static void dt295_device_release(struct kref *kref);

/* ---- 制御転送 ---- */

static int dt295_usb_control(struct dt295_device *device, u8 request_type, u16 value, u16 index, void *buffer,
			     u16 length, unsigned int timeout_ms, bool locked)
{
	unsigned long deadline, remaining;
	unsigned int pipe;
	u8 *bounce = NULL;
	int ret;

	if (!device || !device->usb_dev || !atomic_read(&device->available))
		return -ENODEV;
	if (length > 64 || (length && !buffer))
		return -EINVAL;
	if (!timeout_ms)
		return -ETIMEDOUT;
	if (timeout_ms > DT295_CTRL_TIMEOUT_MAX_MS)
		timeout_ms = DT295_CTRL_TIMEOUT_MAX_MS;
	deadline = jiffies + max_t(unsigned long, 1, msecs_to_jiffies(timeout_ms));
	if (!locked) {
		/*
		 * 順番待ちの数を数えておき、ポンプのスレッドが譲れるようにする。
		 * ロックは眠って待つ。mutex_trylock と msleep(1) を繰り返すと、msleep(1) が 1 ジフィー (HZ=250 で 4 ms) 眠るので、
		 * ポンプのスレッドの 1 周期 (制御転送が 3〜4 回) がカードの転送と衝突するたびに伸び、リングが溢れる (実機で確認)。
		 * 待つ時間は、ロックを持つ側の転送の期限 (最長 DT295_CTRL_TIMEOUT_MAX_MS) で抑えられる。
		 * 転送そのものの期限は、ロックを取った後に決める。
		 */
		atomic_inc(&device->control_waiters);
		if (mutex_lock_interruptible(&device->control_lock)) {
			atomic_dec(&device->control_waiters);
			return -ERESTARTSYS;
		}
		atomic_dec(&device->control_waiters);
		if (!atomic_read(&device->available)) {
			mutex_unlock(&device->control_lock);
			return -ENODEV;
		}
		deadline = jiffies + max_t(unsigned long, 1, msecs_to_jiffies(timeout_ms));
	}
	if (time_after_eq(jiffies, deadline)) {
		ret = -ETIMEDOUT;
		goto out_unlock;
	}
	remaining = deadline - jiffies;
	timeout_ms = max_t(unsigned int, 1, jiffies_to_msecs(remaining));
	pipe = (request_type & USB_DIR_IN) ? usb_rcvctrlpipe(device->usb_dev, 0) : usb_sndctrlpipe(device->usb_dev, 0);
	if (length) {
		/* usb_control_msg の転送バッファは DMA 対象なので、スタック上の配列は渡せない。ヒープの一時領域を経由する。 */
		bounce = kmalloc(length, GFP_KERNEL);
		if (!bounce) {
			ret = -ENOMEM;
			goto out_unlock;
		}
		if (!(request_type & USB_DIR_IN))
			memcpy(bounce, buffer, length);
	}
	ret = usb_control_msg(device->usb_dev, pipe, 0xbc, request_type, value, index, bounce, length, timeout_ms);
	if (bounce) {
		if ((request_type & USB_DIR_IN) && ret > 0)
			memcpy(buffer, bounce, min_t(int, ret, length));
		kfree(bounce);
	}
out_unlock:
	if (!locked)
		mutex_unlock(&device->control_lock);
	return ret;
}

static int dt295_ops_control(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len, unsigned int timeout_ms)
{
	return dt295_usb_control(priv, request_type, value, index, buf, len, timeout_ms, false);
}

static int dt295_ops_control_locked(void *priv, u8 request_type, u16 value, u16 index, void *buf, u16 len,
				    unsigned int timeout_ms)
{
	return dt295_usb_control(priv, request_type, value, index, buf, len, timeout_ms, true);
}

static int dt295_ops_lock(void *priv)
{
	struct dt295_device *device = priv;

	atomic_inc(&device->control_waiters);
	if (mutex_lock_interruptible(&device->control_lock)) {
		atomic_dec(&device->control_waiters);
		return -ERESTARTSYS;
	}
	atomic_dec(&device->control_waiters);
	if (!atomic_read(&device->available)) {
		mutex_unlock(&device->control_lock);
		return -ENODEV;
	}
	return 0;
}

static void dt295_ops_unlock(void *priv)
{
	struct dt295_device *device = priv;

	mutex_unlock(&device->control_lock);
}

static void dt295_ops_msleep(void *priv, unsigned int milliseconds)
{
	(void)priv;
	msleep(milliseconds);
}

static u64 dt295_ops_now_ms(void *priv)
{
	(void)priv;
	return div_u64(ktime_get_ns(), NSEC_PER_MSEC);
}

static void dt295_ops_log(void *priv, int level, const char *format, ...)
{
	struct dt295_device *device = priv;
	struct va_format vaf;
	va_list args;

	va_start(args, format);
	vaf.fmt = format;
	vaf.va = &args;
	if (level <= DT295_LOG_ERR && READ_ONCE(device->stopping))
		dev_dbg(device->dev, "%pV\n", &vaf);
	else if (level <= DT295_LOG_ERR)
		dev_err(device->dev, "%pV\n", &vaf);
	else
		dev_info(device->dev, "%pV\n", &vaf);
	va_end(args);
}

static const struct dt295_ops dt295_operations = {
	.control = dt295_ops_control,
	.lock = dt295_ops_lock,
	.unlock = dt295_ops_unlock,
	.control_locked = dt295_ops_control_locked,
	.msleep = dt295_ops_msleep,
	.now_ms = dt295_ops_now_ms,
	.log = dt295_ops_log,
};

/* ---- カード (UART は torne と同じ配置。電源投入と検出は dt295_card.c) ---- */

static int dt295_card_read32(void *context, u32 address, u32 *value, unsigned int timeout_ms)
{
	struct dt295_device *device = context;
	u8 data[4];
	int ret;

	if (!device || !value || !atomic_read(&device->available) || !device->board_up)
		return -ENODEV;
	ret = dt295_usb_control(device, 0xc0, (u16)((address >> 8) & 0xff00), (u16)address, data, sizeof(data), timeout_ms,
				false);
	if (ret < 0)
		return ret;
	if (ret != (int)sizeof(data))
		return -EIO;
	*value = (u32)data[0] << 24 | (u32)data[1] << 16 | (u32)data[2] << 8 | data[3];
	return 0;
}

static int dt295_card_write32(void *context, u32 address, u32 value, unsigned int timeout_ms)
{
	struct dt295_device *device = context;
	u8 data[4] = { value >> 24, value >> 16, value >> 8, value };
	int ret;

	if (!device || !atomic_read(&device->available) || !device->board_up)
		return -ENODEV;
	ret = dt295_usb_control(device, 0x40, (u16)((address >> 8) & 0xff00), (u16)address, data, sizeof(data), timeout_ms,
				false);
	if (ret < 0)
		return ret;
	return ret == (int)sizeof(data) ? 0 : -EIO;
}

static int dt295_card_lock(void *context)
{
	struct dt295_device *device = context;

	if (mutex_lock_interruptible(&device->card_uart_lock))
		return -ERESTARTSYS;
	if (!atomic_read(&device->available) || !device->board_up) {
		mutex_unlock(&device->card_uart_lock);
		return -ENODEV;
	}
	return 0;
}

static void dt295_card_unlock(void *context)
{
	struct dt295_device *device = context;

	mutex_unlock(&device->card_uart_lock);
}

static void dt295_card_sleep(void *context, unsigned int milliseconds)
{
	(void)context;
	msleep(milliseconds);
}

static u64 dt295_card_now_ms(void *context)
{
	(void)context;
	return div_u64(ktime_get_ns(), NSEC_PER_MSEC);
}

static const struct torne_card_io_ops dt295_card_io_operations = {
	.read32 = dt295_card_read32,
	.write32 = dt295_card_write32,
	.lock = dt295_card_lock,
	.unlock = dt295_card_unlock,
	.sleep_ms = dt295_card_sleep,
	.now_ms = dt295_card_now_ms,
};

static int dt295_card_power_on(void *context)
{
	struct dt295_device *device = context;

	return atomic_read(&device->available) && device->board_up ? 0 : -ENODEV;
}

static int dt295_card_power_off(void *context)
{
	struct dt295_device *device = context;

	/* USB 切断後は、解放済みの機材へ転送しない。 */
	return !atomic_read(&device->available) ? 0 : dt295_card_power_down(&device->card_port);
}

static int dt295_backend_card_set_timeout(void *context, u32 timeout_ms)
{
	struct dt295_device *device = context;

	return torne_card_port_set_timeout(&device->card_port, timeout_ms);
}

static int dt295_backend_card_detect(void *context, bool *present)
{
	struct dt295_device *device = context;

	return dt295_card_detect(&device->card_port, present);
}

static int dt295_backend_card_reset(void *context)
{
	struct dt295_device *device = context;

	return dt295_card_reset(&device->card_port);
}

static int dt295_backend_card_set_baudrate(void *context, u32 baudrate)
{
	struct dt295_device *device = context;

	return torne_card_set_baudrate(&device->card_port, baudrate);
}

static int dt295_backend_card_rx_ready(void *context, bool *ready)
{
	struct dt295_device *device = context;

	return torne_card_rx_ready(&device->card_port, ready);
}

static int dt295_backend_card_read(void *context, struct px4_card_frame *frame)
{
	struct dt295_device *device = context;

	return torne_card_read(&device->card_port, frame->data, &frame->length);
}

static int dt295_backend_card_write(void *context, const struct px4_card_frame *frame)
{
	struct dt295_device *device = context;

	return torne_card_write(&device->card_port, frame->data, frame->length);
}

static const struct px4_card_operations dt295_card_operations = {
	.power_on = dt295_card_power_on,
	.power_off = dt295_card_power_off,
	.set_timeout = dt295_backend_card_set_timeout,
	.detect = dt295_backend_card_detect,
	.reset = dt295_backend_card_reset,
	.set_baudrate = dt295_backend_card_set_baudrate,
	.rx_ready = dt295_backend_card_rx_ready,
	.read = dt295_backend_card_read,
	.write = dt295_backend_card_write,
};

/* ---- TS の経路 ---- */

static void dt295_stream_reset(struct dt295_device *device)
{
	unsigned long flags;

	if (!device->framer)
		return;
	spin_lock_irqsave(&device->stream_lock, flags);
	torne_framer_reset(device->framer);
	spin_unlock_irqrestore(&device->stream_lock, flags);
}

static int dt295_stream_handler(void *context, void *buffer, u32 length)
{
	struct dt295_device *device = context;
	unsigned long flags;
	size_t output_size;

	if (!device || !buffer || !length || !device->chrdev || !device->framer || !device->ts_output)
		return 0;
	if (length > DT295_STREAM_URB_SIZE)
		return -EOVERFLOW;
	spin_lock_irqsave(&device->stream_lock, flags);
	if (READ_ONCE(device->capture_enabled)) {
		output_size = torne_framer_feed(device->framer, buffer, length, device->ts_output);
		if (output_size)
			ptx_chrdev_put_stream(device->chrdev, device->ts_output, output_size);
	}
	spin_unlock_irqrestore(&device->stream_lock, flags);
	return 0;
}

/*
 * ポンプのスレッド: 書き込み位置を窓越しに読み、溜まった分を 16 パケットの倍数で機材内プログラムに送らせ、
 * 送り終えた位置を 192 バイトの倍数に切り下げて、読み出し位置へ書く。
 * 溜まりが無ければ短く眠る。カードなどの制御転送が順番待ちをしているときは、次の周期を遅らせて譲る。
 * 制御転送の失敗が続いたら終了する (set_capture(false)、切断で後始末する)。
 */
static int dt295_pump_thread(void *data)
{
	struct dt295_device *device = data;
	u32 ring_pos = 0, rptr = 0;
	int errors = 0;

	while (!kthread_should_stop()) {
		u32 wptr, avail, packets, floor_pos;
		int ret;

		ret = dt295_cap_wptr(&device->bridge, &wptr);
		if (ret) {
			if (!atomic_read(&device->available) || ++errors >= DT295_PUMP_ERROR_LIMIT)
				break;
			msleep(5);
			continue;
		}
		wptr %= DT295_RING_SIZE;
		avail = (wptr + DT295_RING_SIZE - ring_pos) % DT295_RING_SIZE;
		packets = avail / DT295_PACKET_BYTES / DT295_PUMP_UNIT_PACKETS * DT295_PUMP_UNIT_PACKETS;
		if (!packets) {
			usleep_range(1000, 2000);
			continue;
		}
		if (packets > DT295_PUMP_MAX_PACKETS)
			packets = DT295_PUMP_MAX_PACKETS;
		ret = dt295_prog_pump(&device->bridge, ring_pos, packets);
		if (ret) {
			if (!atomic_read(&device->available) || ++errors >= DT295_PUMP_ERROR_LIMIT)
				break;
			continue;
		}
		errors = 0;
		ring_pos = (ring_pos + packets * DT295_PACKET_BYTES) % DT295_RING_SIZE;
		floor_pos = ring_pos / DT295_UNIT_SIZE * DT295_UNIT_SIZE;
		if (floor_pos != rptr) {
			ret = dt295_cap_set_rptr(&device->bridge, floor_pos);
			if (ret) {
				if (!atomic_read(&device->available) || ++errors >= DT295_PUMP_ERROR_LIMIT)
					break;
				continue;
			}
			rptr = floor_pos;
		}
		if (atomic_read(&device->control_waiters))
			usleep_range(300, 400);
	}
	if (!kthread_should_stop() && !READ_ONCE(device->stopping))
		dev_err(device->dev, "DT295 pump thread stopped on errors (device %s)\n",
			atomic_read(&device->available) ? "available" : "removed");
	/* kthread_stop() が来るまで待つ (自分で終了すると、kthread_stop() が解放済みの領域に触れる) */
	while (!kthread_should_stop())
		msleep(10);
	return 0;
}

/*
 * 機材の bulk IN に残った、前回の TS を読み捨てる。
 * 前回の受信が、ホストが読み出す側を先に止められて終わった (USB コアは unbind のとき、disconnect の前に URB を止める) と、
 * ポンプが送った最後の数パケットが機材の送信バッファに残り、次の受信の先頭に混ざって連続性の飛びになる。
 * ライタもポンプもまだ動いていないので、新しいデータは来ない。読み出しが時間切れになったら、残りは無い。
 * ライタの開始より前に行う (待ち時間が、ライタの開始からポンプが動くまでの時間に入ると、リングを追い越される)。
 */
static void dt295_drain_stream(struct dt295_device *device)
{
	unsigned int total = 0, i;
	u8 *buffer = kmalloc(DT295_PACKET_BYTES, GFP_KERNEL);

	if (!buffer)
		return;
	for (i = 0; i < DT295_DRAIN_MAX_READS; i++) {
		int actual = 0;

		if (usb_bulk_msg(device->usb_dev, usb_rcvbulkpipe(device->usb_dev, DT295_STREAM_ENDPOINT), buffer,
				 DT295_PACKET_BYTES, &actual, DT295_DRAIN_TIMEOUT_MS))
			break;
		total += actual;
	}
	kfree(buffer);
	if (total)
		dev_dbg(device->dev, "DT295 drained %u stale bytes from bulk IN\n", total);
}


/* 取り込みを始める: 取り込みブロックの設定と開始 -> 機材内プログラムの導入 -> URB -> ポンプのスレッド。失敗したら逆順に戻す。 */
static int dt295_start_stream(struct dt295_device *device)
{
	int ret;

	if (device->streaming)
		return 0;
	if (!atomic_read(&device->available) || !device->ready)
		return -ENODEV;
	if (device->needs_replug)
		return -EIO;
	/* 選局の直後は、ロックから一定の時間が経つまで待つ (ロックしていない搬送波は待たない) */
	if (device->lock_seen) {
		unsigned long until = device->lock_time + msecs_to_jiffies(DT295_LOCK_SETTLE_MS);

		if (time_before(jiffies, until))
			msleep(jiffies_to_msecs(until - jiffies));
	}
	if (!device->framer) {
		device->framer = kvzalloc(sizeof(*device->framer), GFP_KERNEL);
		device->ts_output = kvzalloc(DT295_OUTPUT_SIZE, GFP_KERNEL);
		if (!device->framer || !device->ts_output) {
			kvfree(device->framer);
			kvfree(device->ts_output);
			device->framer = NULL;
			device->ts_output = NULL;
			return -ENOMEM;
		}
	}
	dt295_stream_reset(device);
	dt295_drain_stream(device);
	ret = dt295_cap_start(&device->bridge);
	if (ret)
		return ret;
	ret = dt295_prog_install(&device->bridge);
	if (ret)
		goto fail_capture;
	ret = itedtv_bus_start_streaming(&device->stream_bus, dt295_stream_handler, device);
	if (ret)
		goto fail_program;
	device->pump_task = kthread_run(dt295_pump_thread, device, "dt295_pump");
	if (IS_ERR(device->pump_task)) {
		ret = PTR_ERR(device->pump_task);
		device->pump_task = NULL;
		goto fail_urb;
	}
	/* ライタは、読み出し側が動いてから開始する (先に開始すると、約 50 ms でリングが 1 周して、未読のデータを上書きする) */
	ret = dt295_cap_run(&device->bridge);
	if (ret)
		goto fail_pump;
	device->streaming = true;
	return 0;

fail_pump:
	kthread_stop(device->pump_task);
	device->pump_task = NULL;
fail_urb:
	itedtv_bus_stop_streaming(&device->stream_bus);
fail_program:
	if (dt295_prog_remove(&device->bridge))
		device->needs_replug = true;
fail_capture:
	dt295_cap_stop(&device->bridge);
	dt295_window_restore(&device->bridge);
	return ret;
}

/* 取り込みを止める。切断後 (available が 0) は、機材へ書かず、スレッドと URB だけを止める。 */
static int dt295_stop_stream(struct dt295_device *device)
{
	int ret = 0, step;

	if (device->pump_task) {
		kthread_stop(device->pump_task);
		device->pump_task = NULL;
	}
	if (device->streaming)
		ret = itedtv_bus_stop_streaming(&device->stream_bus);
	if (device->streaming && atomic_read(&device->available)) {
		/* ライタを止めて、hook を先に元へ戻し、本体の領域を戻し、最後に窓を戻す */
		step = dt295_cap_stop(&device->bridge);
		if (!ret)
			ret = step;
		step = dt295_prog_remove(&device->bridge);
		if (step) {
			device->needs_replug = true;
			if (!ret)
				ret = step;
		}
		step = dt295_window_restore(&device->bridge);
		if (step) {
			device->needs_replug = true;
			if (!ret)
				ret = step;
		}
	}
	device->streaming = false;
	dt295_stream_reset(device);
	return ret;
}

/* ---- ptx_chrdev ---- */

static int dt295_chrdev_init(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int dt295_chrdev_term(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int dt295_chrdev_open(struct ptx_chrdev *chrdev)
{
	struct dt295_device *device = chrdev->priv;
	int ret;

	mutex_lock(&device->operation_lock);
	ret = atomic_read(&device->available) && device->ready ? 0 : -ENODEV;
	mutex_unlock(&device->operation_lock);
	if (!ret)
		kref_get(&device->kref);
	return ret;
}

static int dt295_chrdev_release(struct ptx_chrdev *chrdev)
{
	struct dt295_device *device = chrdev->priv;

	mutex_lock(&device->operation_lock);
	WRITE_ONCE(device->capture_enabled, false);
	dt295_stop_stream(device);
	mutex_unlock(&device->operation_lock);
	kref_put(&device->kref, dt295_device_release);
	return 0;
}

static int dt295_chrdev_tune(struct ptx_chrdev *chrdev, struct ptx_tune_params *params)
{
	struct dt295_device *device = chrdev->priv;
	bool resume;
	int ret;

	if (!params || params->system != PTX_ISDB_T_SYSTEM)
		return -EINVAL;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready) {
		ret = -ENODEV;
		goto out;
	}
	/* 受信中の選局は、前のチャンネルの TS を引き継がないよう、いったん止めてから行う */
	resume = device->streaming;
	if (resume)
		dt295_stop_stream(device);
	device->lock_seen = false;
	ret = dt295_fe_tune(&device->bridge, params->freq);
	if (resume) {
		const int restart_ret = dt295_start_stream(device);

		if (!ret)
			ret = restart_ret;
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int dt295_chrdev_check_lock(struct ptx_chrdev *chrdev, bool *locked)
{
	struct dt295_device *device = chrdev->priv;
	int ret;

	if (!locked)
		return -EINVAL;
	*locked = false;
	mutex_lock(&device->operation_lock);
	ret = atomic_read(&device->available) && device->ready ? dt295_fe_locked(&device->bridge, locked) : -ENODEV;
	if (!ret && *locked && !device->lock_seen) {
		device->lock_seen = true;
		device->lock_time = jiffies;
	}
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int dt295_chrdev_set_lnb_voltage(struct ptx_chrdev *chrdev, int voltage)
{
	(void)chrdev;
	return voltage == 0 ? 0 : -EOPNOTSUPP;
}

static int dt295_chrdev_set_capture(struct ptx_chrdev *chrdev, bool capture)
{
	struct dt295_device *device = chrdev->priv;
	int ret = 0;

	mutex_lock(&device->operation_lock);
	if (capture) {
		if (!atomic_read(&device->available) || !device->ready) {
			ret = -ENODEV;
			goto out;
		}
		WRITE_ONCE(device->capture_enabled, true);
		ret = dt295_start_stream(device);
		if (ret)
			WRITE_ONCE(device->capture_enabled, false);
	} else {
		WRITE_ONCE(device->capture_enabled, false);
		ret = dt295_stop_stream(device);
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int dt295_chrdev_read_cnr_raw(struct ptx_chrdev *chrdev, u32 *value)
{
	struct dt295_device *device = chrdev->priv;
	bool locked = false;
	int ret;

	if (!value)
		return -EINVAL;
	*value = 0;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready) {
		ret = -ENODEV;
	} else {
		ret = dt295_fe_locked(&device->bridge, &locked);
		if (!ret && !locked)
			ret = -EAGAIN;
		if (!ret)
			ret = dt295_fe_cnr_raw(&device->bridge, value);
	}
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int dt295_chrdev_read_signal_strength(struct ptx_chrdev *chrdev, u32 *value)
{
	(void)chrdev;
	/* チューナーの強度の換算は未確認。 */
	if (!value)
		return -EINVAL;
	*value = 0;
	return -EOPNOTSUPP;
}

static const struct ptx_chrdev_operations dt295_chrdev_ops = {
	.init = dt295_chrdev_init,
	.term = dt295_chrdev_term,
	.open = dt295_chrdev_open,
	.release = dt295_chrdev_release,
	.tune = dt295_chrdev_tune,
	.check_lock = dt295_chrdev_check_lock,
	.set_stream_id = NULL,
	.set_lnb_voltage = dt295_chrdev_set_lnb_voltage,
	.set_capture = dt295_chrdev_set_capture,
	.read_signal_strength = dt295_chrdev_read_signal_strength,
	.read_cnr = NULL,
	.read_cnr_raw = dt295_chrdev_read_cnr_raw,
};

/* ---- 寿命 ---- */

static void dt295_device_release(struct kref *kref)
{
	struct dt295_device *device = container_of(kref, struct dt295_device, kref);

	if (device->pump_task) {
		kthread_stop(device->pump_task);
		device->pump_task = NULL;
	}
	if (device->streaming)
		itedtv_bus_stop_streaming(&device->stream_bus);
	if (device->bus_initialized)
		itedtv_bus_term(&device->stream_bus);
	kvfree(device->framer);
	kvfree(device->ts_output);
	mutex_destroy(&device->card_uart_lock);
	mutex_destroy(&device->control_lock);
	mutex_destroy(&device->operation_lock);
	put_device(device->dev);
	complete(device->quit_completion);
}

int dt295_device_init(struct dt295_device *device, struct device *dev, struct usb_interface *intf,
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
	ret = usb_set_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber, 0);
	if (ret)
		return ret;
	memset(device, 0, sizeof(*device));
	get_device(dev);
	kref_init(&device->kref);
	atomic_set(&device->available, 0);
	atomic_set(&device->control_waiters, 0);
	device->dev = dev;
	device->usb_dev = usb_dev;
	device->quit_completion = quit_completion;
	mutex_init(&device->operation_lock);
	mutex_init(&device->control_lock);
	mutex_init(&device->card_uart_lock);
	spin_lock_init(&device->stream_lock);
	device->stream_bus.dev = dev;
	device->stream_bus.type = ITEDTV_BUS_USB;
	device->stream_bus.usb.dev = usb_dev;
	device->stream_bus.usb.ctrl_timeout = DT295_CTRL_TIMEOUT_MS;
	device->stream_bus.usb.streaming.endpoint = DT295_STREAM_ENDPOINT;
	device->stream_bus.usb.streaming.urb_buffer_size = DT295_STREAM_URB_SIZE;
	device->stream_bus.usb.streaming.urb_num = DT295_STREAM_URB_NUM;
	device->stream_bus.usb.streaming.no_dma = px4_usb_params.no_dma;
	ret = itedtv_bus_init(&device->stream_bus);
	if (ret)
		goto fail_device;
	device->bus_initialized = true;
	/* 起動中も制御転送を受け付ける。probe 失敗時は fail_bus で戻す。 */
	atomic_set(&device->available, 1);
	dt295_bridge_init(&device->bridge, &dt295_operations, device);
	torne_card_port_init(&device->card_port, &dt295_card_io_operations, device);
	ret = dt295_board_open(&device->bridge);
	if (ret) {
		dev_err(dev, "DT295 board setup failed: %d\n", ret);
		goto fail_bus;
	}
	device->board_up = true;
	ret = dt295_fe_init(&device->bridge);
	if (ret) {
		dev_err(dev, "DT295 frontend initialization failed: %d\n", ret);
		goto fail_bus;
	}
	device->ready = true;

	chrdev_config.system_cap = PTX_ISDB_T_SYSTEM;
	chrdev_config.ops = &dt295_chrdev_ops;
	chrdev_config.ringbuf_size = 188 * px4_device_params.tsdev_max_packets;
	chrdev_config.ringbuf_threshold_size = chrdev_config.ringbuf_size / 10;
	chrdev_config.priv = device;
	group_config.owner_kref = &device->kref;
	group_config.owner_kref_release = dt295_device_release;
	group_config.reserved = false;
	group_config.minor_base = 0;
	group_config.chrdev_num = DT295_CHRDEV_NUM;
	group_config.chrdev_config = &chrdev_config;
	ret = ptx_chrdev_context_add_group(chrdev_ctx, dev, &group_config, &chrdev_group);
	if (ret) {
		atomic_set(&device->available, 0);
		goto fail_bus;
	}
	device->chrdev_group = chrdev_group;
	device->chrdev = &chrdev_group->chrdev[0];
	{
		struct px4_card_backend card_backend = {
			.it930x = NULL,
			.operations = &dt295_card_operations,
			.lock = &device->operation_lock,
			.available = &device->available,
			.context = device,
		};

		ret = px4_card_register(dev, &card_backend, &device->card);
		if (ret)
			dev_warn(dev, "Card reader registration failed: %d\n", ret);
	}
	/* USB の製品 ID が、2 つの版 (Rev.01 と Rev.02) の違いを示す唯一の情報。ログから見分けられるようにする。 */
	{
		const u16 product = le16_to_cpu(usb_dev->descriptor.idProduct);

		dev_info(dev, "DT295 %s (%04x:%04x) receiver registered (bulk IN 0x81)\n",
			 product == DT295_USB_PRODUCT_REV01 ? "Rev.01" :
			 product == DT295_USB_PRODUCT_REV02 ? "Rev.02" : "unknown revision",
			 le16_to_cpu(usb_dev->descriptor.idVendor), product);
	}
	return 0;

fail_bus:
	atomic_set(&device->available, 0);
	if (device->bus_initialized) {
		itedtv_bus_term(&device->stream_bus);
		device->bus_initialized = false;
	}
fail_device:
	mutex_destroy(&device->card_uart_lock);
	mutex_destroy(&device->control_lock);
	mutex_destroy(&device->operation_lock);
	put_device(dev);
	return ret;
}

void dt295_device_term(struct dt295_device *device)
{
	/*
	 * カードを外し、スレッドと URB を止めて、機材を元に戻してから、受付を閉じる。
	 * 機材がまだある場合 (アンバインド、rmmod) に hook と窓を戻さないと、次の probe が「起動直後でない」で断られる。
	 * 機材が取り外されたときは、USB コアが転送を速やかに失敗させるので、戻す操作は短時間で終わる。
	 * 止めた後の同じロックの中で available を 0 にするので、止めた直後に ptx_chrdev の操作が受信を再開することはない。
	 */
	WRITE_ONCE(device->stopping, true);
	px4_card_unregister(device->card);
	device->card = NULL;
	mutex_lock(&device->operation_lock);
	WRITE_ONCE(device->capture_enabled, false);
	dt295_stop_stream(device);
	atomic_xchg(&device->available, 0);
	mutex_unlock(&device->operation_lock);
	ptx_chrdev_group_destroy(device->chrdev_group);
	kref_put(&device->kref, dt295_device_release);
}
