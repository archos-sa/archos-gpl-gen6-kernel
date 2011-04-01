#ifndef _LINUX_USB_EHCI_H
#define _LINUX_USB_EHCI_H

struct ehci_platform_data
{
	int usb_enable_pin;
	int usb_enable_delay;
};

#endif
