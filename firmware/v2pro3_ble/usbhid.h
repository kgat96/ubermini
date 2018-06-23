/*
 * This file is part of the ubermini project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __USBHID_H
#define __USBHID_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/cm3/scb.h>

void usb_setup(void);
void usb_poll(void);
int usb_write_packet(u8* buf, int len);

#endif
