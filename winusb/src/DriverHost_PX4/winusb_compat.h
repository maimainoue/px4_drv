// winusb_compat.h

#pragma once

#if defined(__APPLE__) || defined(PX4_USERLAND)
#include "libusb_compat.h"
#else

#include <windows.h>
#include <winusb.h>

struct usb_device {
	HANDLE dev;
	WINUSB_INTERFACE_HANDLE winusb;
	USB_DEVICE_DESCRIPTOR descriptor;
};

#endif /* __APPLE__ */
