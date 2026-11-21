// SPDX-License-Identifier: GPL-2.0-only
/*
 * TurboSight TBS 5530 の Linux バックエンド (ISDB-T)。
 * ブリッジ、復調器 (CXD6822 / CXD2878A)、内蔵チューナーの制御は共通層 (driver/common/tbs5530) が行い、
 * ここは USB の転送、TS の同期と受け渡し、ptx のキャラクタデバイス、デバイスの寿命を受け持つ。
 */

#include "print_format.h"
#include "tbs5530_device.h"

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/usb.h>

#include "px4_device_params.h"
#include "px4_usb_params.h"

#define TBS5530_CTRL_TIMEOUT_MS 2000U
/* ファームウェアの有無を確かめる間の転送の待ち。ファームウェアが無いと、読み出しはこの時間で失敗する */
#define TBS5530_PROBE_TIMEOUT_MS 500U
#define TBS5530_FIRMWARE "dvb-usb-id5530.fw"
/* ファームウェアの送信後、ブリッジが応答するまでの待ち (再列挙を含む) */
#define TBS5530_FW_WAIT_MS 100U
#define TBS5530_FW_WAIT_TRIES 30U
/* 端点の 512 バイトの倍数。仮想マシンの USB パススルーでは、転送が大きいほど取りこぼしが減る (16 KB より 64 KB) */
#define TBS5530_STREAM_URB_SIZE (4096U * 16U)
#define TBS5530_STREAM_URB_NUM 8U

static void tbs5530_device_release(struct kref *kref);

/* ---- 共通層から呼ばれる USB の入出力 ---- */

static int tbs5530_control(void *priv, u8 request_type, u8 request, u16 value,
			   u16 index, void *buffer, u16 length)
{
	struct tbs5530_device *device = priv;
	unsigned int pipe;
	u8 *bounce = NULL;
	int ret;

	if (!device || !device->usb_dev || !atomic_read(&device->available))
		return -ENODEV;
	if (length > 64 || (length && !buffer))
		return -EINVAL;
	mutex_lock(&device->control_lock);
	if (!atomic_read(&device->available)) {
		mutex_unlock(&device->control_lock);
		return -ENODEV;
	}
	pipe = (request_type & USB_DIR_IN) ? usb_rcvctrlpipe(device->usb_dev, 0) :
					      usb_sndctrlpipe(device->usb_dev, 0);
	if (length) {
		/*
		 * 共通層はスタック上の配列を渡す。転送バッファは DMA の対象のため、
		 * スタックのまま usb_control_msg へ渡すと -EAGAIN で拒否される (VMAP_STACK)。
		 */
		bounce = kmalloc(length, GFP_KERNEL);
		if (!bounce) {
			mutex_unlock(&device->control_lock);
			return -ENOMEM;
		}
		if (!(request_type & USB_DIR_IN))
			memcpy(bounce, buffer, length);
	}
	ret = usb_control_msg(device->usb_dev, pipe, request, request_type, value,
			      index, bounce, length, device->control_timeout_ms);
	if (bounce) {
		if ((request_type & USB_DIR_IN) && ret > 0)
			memcpy(buffer, bounce, min_t(int, ret, length));
		kfree(bounce);
	}
	mutex_unlock(&device->control_lock);
	return ret;
}

static void tbs5530_sleep(void *priv, unsigned int milliseconds)
{
	(void)priv;
	msleep(milliseconds);
}

static void tbs5530_log(void *priv, int level, const char *format, ...)
{
	struct tbs5530_device *device = priv;
	struct va_format vaf;
	va_list args;

	va_start(args, format);
	vaf.fmt = format;
	vaf.va = &args;
	if (level <= TBS5530_LOG_ERR)
		dev_err(device->dev, "%pV", &vaf);
	else
		dev_info(device->dev, "%pV", &vaf);
	va_end(args);
}

static const struct tbs5530_ops tbs5530_operations = {
	.control = tbs5530_control,
	.msleep = tbs5530_sleep,
	.log = tbs5530_log,
};

/* ---- TS ---- */

static void tbs5530_stream_emit(void *context, const t230c_ts_u8 *packet)
{
	struct tbs5530_stream_context *stream = context;

	memcpy(stream->output_buffer + stream->output_size, packet,
	       T230C_TS_PACKET_SIZE);
	stream->output_size += T230C_TS_PACKET_SIZE;
	if (stream->output_size == sizeof(stream->output_buffer)) {
		ptx_chrdev_put_stream(stream->chrdev, stream->output_buffer,
				      stream->output_size);
		stream->output_size = 0;
	}
}

/*
 * ストリームは途中から読み始めるため、先頭が 188 バイトの境界とは限らない。
 * 0x47 が 188 バイト間隔で並ぶ位置に同期してから、1 パケットずつ渡す。
 */
static int tbs5530_stream_handler(void *context, void *buffer, u32 length)
{
	struct tbs5530_stream_context *stream = context;
	unsigned long flags;
	int ret = 0;

	if (!buffer || !length || !stream->chrdev || !stream->device)
		return 0;
	spin_lock_irqsave(&stream->lock, flags);
	if (READ_ONCE(stream->device->capture_enabled)) {
		stream->output_size = 0;
		ret = t230c_ts_sync_push(&stream->synchronizer, buffer, length,
					 tbs5530_stream_emit, stream);
		if (stream->output_size) {
			ptx_chrdev_put_stream(stream->chrdev, stream->output_buffer,
					      stream->output_size);
			stream->output_size = 0;
		}
	}
	spin_unlock_irqrestore(&stream->lock, flags);
	return ret;
}

/* 選局やストリームの停止のたびに、前のチャンネルの端数と同期の状態を捨てる */
static void tbs5530_stream_reset(struct tbs5530_device *device)
{
	unsigned long flags;

	spin_lock_irqsave(&device->stream.lock, flags);
	t230c_ts_sync_reset(&device->stream.synchronizer);
	device->stream.output_size = 0;
	spin_unlock_irqrestore(&device->stream.lock, flags);
}

static int tbs5530_start_stream(struct tbs5530_device *device)
{
	int ret;

	if (device->streaming)
		return 0;
	if (!atomic_read(&device->available) || !device->ready)
		return -ENODEV;
	tbs5530_stream_reset(device);
	/* 地上波の復調器の TS を、バルク端点へ繋ぐ */
	ret = tbs5530_select_stream(&device->bridge, false);
	if (ret)
		return ret;
	ret = itedtv_bus_start_streaming(&device->stream_bus, tbs5530_stream_handler,
					 &device->stream);
	if (ret)
		return ret;
	device->streaming = true;
	return 0;
}

static int tbs5530_stop_stream(struct tbs5530_device *device)
{
	int ret = 0;

	if (device->streaming) {
		ret = itedtv_bus_stop_streaming(&device->stream_bus);
		device->streaming = false;
	}
	tbs5530_stream_reset(device);
	return ret;
}

/* ---- ptx のキャラクタデバイス ---- */

static int tbs5530_chrdev_init(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int tbs5530_chrdev_term(struct ptx_chrdev *chrdev)
{
	(void)chrdev;
	return 0;
}

static int tbs5530_chrdev_open(struct ptx_chrdev *chrdev)
{
	struct tbs5530_device *device = chrdev->priv;
	int ret = 0;

	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready)
		ret = -ENODEV;
	mutex_unlock(&device->operation_lock);
	if (!ret)
		kref_get(&device->kref);
	return ret;
}

static int tbs5530_chrdev_release(struct ptx_chrdev *chrdev)
{
	struct tbs5530_device *device = chrdev->priv;

	mutex_lock(&device->operation_lock);
	if (atomic_read(&device->available) && device->ready) {
		WRITE_ONCE(device->capture_enabled, false);
		tbs5530_stop_stream(device);
		tbs5530_demod_sleep(&device->demod);
	}
	mutex_unlock(&device->operation_lock);
	kref_put(&device->kref, tbs5530_device_release);
	return 0;
}

static int tbs5530_chrdev_tune(struct ptx_chrdev *chrdev,
			       struct ptx_tune_params *params)
{
	struct tbs5530_device *device = chrdev->priv;
	bool resume;
	int ret;

	/* ISDB-C は J.83 Annex C。J.83 Annex B (PTX_J83B_SYSTEM) は未対応 */
	if (!params || (params->system != PTX_ISDB_T_SYSTEM && params->system != PTX_ISDB_C_SYSTEM))
		return -EINVAL;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready) {
		ret = -ENODEV;
		goto out;
	}
	/* 受信中の選局は、止めてから選局し、前のチャンネルの TS を引き継がずに再開する */
	resume = device->streaming;
	if (resume)
		tbs5530_stop_stream(device);
	if (params->system == PTX_ISDB_C_SYSTEM)
		ret = tbs5530_demod_tune_isdbc(&device->demod, params->freq);
	else
		ret = tbs5530_demod_tune_isdbt(&device->demod, params->freq);
	if (resume) {
		const int restart_ret = tbs5530_start_stream(device);

		if (!ret)
			ret = restart_ret;
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int tbs5530_chrdev_check_lock(struct ptx_chrdev *chrdev, bool *locked)
{
	struct tbs5530_device *device = chrdev->priv;
	u8 status = 0;
	int ret;

	if (!locked)
		return -EINVAL;
	*locked = false;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready)
		ret = -ENODEV;
	else
		ret = tbs5530_demod_read_status(&device->demod, &status);
	mutex_unlock(&device->operation_lock);
	if (!ret)
		*locked = !!(status & TBS5530_STATUS_LOCK);
	return ret;
}

static int tbs5530_chrdev_set_lnb_voltage(struct ptx_chrdev *chrdev, int voltage)
{
	(void)chrdev;
	return voltage == 0 ? 0 : -EOPNOTSUPP;
}

static int tbs5530_chrdev_set_capture(struct ptx_chrdev *chrdev, bool capture)
{
	struct tbs5530_device *device = chrdev->priv;
	int ret = 0;

	mutex_lock(&device->operation_lock);
	if (capture) {
		if (!atomic_read(&device->available) || !device->ready) {
			ret = -ENODEV;
			goto out;
		}
		WRITE_ONCE(device->capture_enabled, true);
		ret = tbs5530_start_stream(device);
		if (ret)
			WRITE_ONCE(device->capture_enabled, false);
	} else {
		WRITE_ONCE(device->capture_enabled, false);
		ret = tbs5530_stop_stream(device);
	}
out:
	mutex_unlock(&device->operation_lock);
	return ret;
}

static int tbs5530_chrdev_read_cnr_raw(struct ptx_chrdev *chrdev, u32 *value)
{
	struct tbs5530_device *device = chrdev->priv;
	u8 status = 0;
	s32 cnr_mdb = 0;
	int ret;

	if (!value)
		return -EINVAL;
	*value = 0;
	mutex_lock(&device->operation_lock);
	if (!atomic_read(&device->available) || !device->ready) {
		ret = -ENODEV;
	} else {
		ret = tbs5530_demod_read_status(&device->demod, &status);
		if (!ret && !(status & TBS5530_STATUS_LOCK))
			ret = -EAGAIN;
		if (!ret)
			ret = tbs5530_demod_read_cnr(&device->demod, &cnr_mdb);
		/* ptx GET_CNR は 0.01 dB 単位 (torne と同じ)。 */
		if (!ret)
			*value = cnr_mdb > 0 ? (u32)(cnr_mdb / 10) : 0;
	}
	mutex_unlock(&device->operation_lock);
	return ret;
}

static const struct ptx_chrdev_operations tbs5530_chrdev_ops = {
	.init = tbs5530_chrdev_init,
	.term = tbs5530_chrdev_term,
	.open = tbs5530_chrdev_open,
	.release = tbs5530_chrdev_release,
	.tune = tbs5530_chrdev_tune,
	.check_lock = tbs5530_chrdev_check_lock,
	.set_stream_id = NULL,
	.set_lnb_voltage = tbs5530_chrdev_set_lnb_voltage,
	.set_capture = tbs5530_chrdev_set_capture,
	.read_signal_strength = NULL,
	.read_cnr = NULL,
	.read_cnr_raw = tbs5530_chrdev_read_cnr_raw,
};

/* ---- 寿命 ---- */

static void tbs5530_device_release(struct kref *kref)
{
	struct tbs5530_device *device = container_of(kref, struct tbs5530_device, kref);

	if (device->streaming)
		itedtv_bus_stop_streaming(&device->stream_bus);
	if (device->bus_initialized)
		itedtv_bus_term(&device->stream_bus);
	mutex_destroy(&device->control_lock);
	mutex_destroy(&device->operation_lock);
	put_device(device->dev);
	complete(device->quit_completion);
}

/* ブリッジのファームウェアが動いているか (EEPROM を読めるか) */
static bool tbs5530_bridge_is_warm(struct tbs5530_device *device)
{
	u8 mac[6];

	return tbs5530_read_mac(&device->bridge, mac) == 0;
}

/*
 * ブリッジのマイクロコントローラは、不揮発のファームウェアを持たず、接続のたびに RAM へ送る必要がある。
 * 通電直後の機材は、送信後に一度だけ再列挙する。再列挙した場合は、この probe は失敗で終わり、
 * 新しく見えたデバイスが改めて probe される。動作中の機材へ送り直したときは、再列挙しない。
 */
static int tbs5530_load_firmware(struct tbs5530_device *device)
{
	const struct firmware *fw;
	unsigned int i;
	int ret;

	ret = request_firmware(&fw, TBS5530_FIRMWARE, device->dev);
	if (ret) {
		dev_err(device->dev, "firmware %s not found (%d)\n", TBS5530_FIRMWARE, ret);
		return ret;
	}
	ret = tbs5530_fw_load(&device->bridge, fw->data, fw->size);
	release_firmware(fw);
	if (ret)
		return ret;
	for (i = 0; i < TBS5530_FW_WAIT_TRIES; i++) {
		if (tbs5530_bridge_is_warm(device)) {
			dev_info(device->dev, "TBS 5530 firmware downloaded (%u ms)\n",
				 i * TBS5530_FW_WAIT_MS);
			return 0;
		}
		msleep(TBS5530_FW_WAIT_MS);
	}
	dev_err(device->dev, "bridge did not respond after the firmware download\n");
	return -ETIMEDOUT;
}

int tbs5530_device_init(struct tbs5530_device *device, struct device *dev,
			struct usb_interface *intf,
			struct ptx_chrdev_context *chrdev_ctx,
			struct completion *quit_completion)
{
	struct ptx_chrdev_config chrdev_config = { 0 };
	struct ptx_chrdev_group_config group_config = { 0 };
	struct ptx_chrdev_group *chrdev_group;
	struct usb_device *usb_dev;
	int ret;

	if (!device || !dev || !intf || !chrdev_ctx || !quit_completion)
		return -EINVAL;
	usb_dev = interface_to_usbdev(intf);
	memset(device, 0, sizeof(*device));
	get_device(dev);
	kref_init(&device->kref);
	atomic_set(&device->available, 0);
	device->dev = dev;
	device->usb_dev = usb_dev;
	device->quit_completion = quit_completion;
	device->control_timeout_ms = TBS5530_PROBE_TIMEOUT_MS;
	mutex_init(&device->operation_lock);
	mutex_init(&device->control_lock);
	device->stream.device = device;
	spin_lock_init(&device->stream.lock);
	ret = t230c_ts_sync_init(&device->stream.synchronizer,
				 device->stream.sync_buffer,
				 sizeof(device->stream.sync_buffer));
	if (ret)
		goto fail_device;

	device->stream_bus.dev = dev;
	device->stream_bus.type = ITEDTV_BUS_USB;
	device->stream_bus.usb.dev = usb_dev;
	device->stream_bus.usb.ctrl_timeout = TBS5530_CTRL_TIMEOUT_MS;
	device->stream_bus.usb.streaming.endpoint = TBS5530_TS_ENDPOINT;
	device->stream_bus.usb.streaming.urb_buffer_size = TBS5530_STREAM_URB_SIZE;
	device->stream_bus.usb.streaming.urb_num = TBS5530_STREAM_URB_NUM;
	device->stream_bus.usb.streaming.no_dma = px4_usb_params.no_dma;
	ret = itedtv_bus_init(&device->stream_bus);
	if (ret)
		goto fail_device;
	device->bus_initialized = true;

	/* 起動の途中も制御転送を受け付ける。probe の失敗時は fail_bus で閉じる。 */
	atomic_set(&device->available, 1);
	tbs5530_bridge_init(&device->bridge, &tbs5530_operations, device);
	/*
	 * 動作中のファームウェアがある機材にも、毎回送り直す。FX2 の端点の FIFO は、前の利用者が
	 * 受信の途中で終わると不整合のまま残り、バルク端点の読み出しがすべて -EOVERFLOW / -EPROTO で
	 * 終わる。これはインターフェースを選び直しても直らず、CPU のリセットを伴うファームウェアの
	 * 送り直しで直る。再接続やモジュールの再読み込みでも、機材は通電したままのため、送り直しで
	 * 起動時の状態へ戻す (元の dvb-usb のドライバも、probe のたびに送っている)。
	 */
	ret = tbs5530_load_firmware(device);
	if (ret)
		goto fail_bus;
	device->control_timeout_ms = TBS5530_CTRL_TIMEOUT_MS;

	/*
	 * これを送らないと、バルク端点 0x82 の読み出しがすべて -EOVERFLOW (長さ 0) で終わる。
	 * ファームウェアが動いた後、ストリームを読む前に、インターフェースを選び直す。
	 */
	ret = usb_set_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber, 0);
	if (ret) {
		dev_err(dev, "TBS 5530 set interface failed: %d\n", ret);
		goto fail_bus;
	}
	ret = tbs5530_demod_attach(&device->demod, &device->bridge);
	if (ret) {
		dev_err(dev, "TBS 5530 demodulator detection failed: %d (chip id 0x%03x)\n",
			ret, device->demod.chip_id);
		goto fail_bus;
	}
	ret = tbs5530_select_stream(&device->bridge, false);
	if (ret)
		goto fail_bus;
	ret = tbs5530_demod_init(&device->demod);
	if (ret) {
		dev_err(dev, "TBS 5530 demodulator initialization failed: %d\n", ret);
		goto fail_bus;
	}
	device->ready = true;

	chrdev_config.system_cap = PTX_ISDB_T_SYSTEM | PTX_ISDB_C_SYSTEM;
	chrdev_config.ops = &tbs5530_chrdev_ops;
	chrdev_config.ringbuf_size = 188 * px4_device_params.tsdev_max_packets;
	chrdev_config.ringbuf_threshold_size = chrdev_config.ringbuf_size / 10;
	chrdev_config.priv = device;
	group_config.owner_kref = &device->kref;
	group_config.owner_kref_release = tbs5530_device_release;
	group_config.reserved = false;
	group_config.minor_base = 0;
	group_config.chrdev_num = TBS5530_CHRDEV_NUM;
	group_config.chrdev_config = &chrdev_config;
	ret = ptx_chrdev_context_add_group(chrdev_ctx, dev, &group_config, &chrdev_group);
	if (ret)
		goto fail_bus;
	device->chrdev_group = chrdev_group;
	device->chrdev = &chrdev_group->chrdev[0];
	/*
	 * ISDB-C は TSMF の分離を通る。選局し直した直後は、前の搬送波の TS が残っていることがあるため、
	 * ブリッジの転送 1 回分を、TSMF の判断に使わずに捨てる。null パケットはブリッジでは除かないので、
	 * discard_null_packets は既定の false のままにする (TSMF は、ヘッダーからの位置で TS を選ぶ)。
	 */
	device->chrdev->tsmf_stale_packets = TBS5530_STREAM_URB_SIZE / 188U;
	device->stream.chrdev = device->chrdev;
	dev_info(dev, "TBS 5530 receiver registered (ISDB-T, bulk IN 0x82, chip 0x%03x)\n",
		 device->demod.chip_id);
	return 0;

fail_bus:
	if (device->ready) {
		tbs5530_demod_sleep(&device->demod);
		device->ready = false;
	}
	atomic_set(&device->available, 0);
	if (device->bus_initialized) {
		itedtv_bus_term(&device->stream_bus);
		device->bus_initialized = false;
	}
fail_device:
	mutex_destroy(&device->control_lock);
	mutex_destroy(&device->operation_lock);
	put_device(dev);
	return ret;
}

void tbs5530_device_term(struct tbs5530_device *device)
{
	/* 受付を閉じてから、受信を止め、最後に利用可能状態を閉じる。 */
	atomic_xchg(&device->available, 0);
	mutex_lock(&device->operation_lock);
	WRITE_ONCE(device->capture_enabled, false);
	tbs5530_stop_stream(device);
	mutex_unlock(&device->operation_lock);
	ptx_chrdev_group_destroy(device->chrdev_group);
	kref_put(&device->kref, tbs5530_device_release);
}
