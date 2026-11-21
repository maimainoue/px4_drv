// xz_core.h と xz_card.h を C++ から使えること (extern "C" とリンク) を確かめる。ホスト層 (C++) が共通の C 層を呼ぶため。
#include "xz_card.h"
#include "xz_core.h"

static int control(void *, u8, u16, u16, void *, u16, unsigned int)
{
	return -1;
}

static void sleep_stub(void *, unsigned int)
{
}

static u64 now_stub(void *)
{
	return 0;
}

static void log_callback(void *, int, const char *, ...)
{
}

int main()
{
	xz_ops ops{};
	ops.control = control;
	ops.msleep = sleep_stub;
	ops.now_ms = now_stub;
	ops.log = log_callback;
	xz_bridge bridge{};
	xz_bridge_init(&bridge, &ops, nullptr);
	u32 value = 0;
	/* 制御転送が失敗する ops で、失敗がそのまま返ること */
	if (xz_read32(&bridge, 0xf00000, &value) != -1)
		return 1;
	torne_card_port port{};
	bool present = true;
	(void)xz_card_detect(&port, &present);
	return 0;
}
