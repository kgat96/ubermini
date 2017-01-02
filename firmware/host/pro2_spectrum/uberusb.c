/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdlib.h>
#include <stdio.h>

#include <signal.h>

#include <libusb-1.0/libusb.h>

extern void UberPacket(unsigned short *buf, int len);

#define U0_VENDORID    0x1483
#define U0_PRODUCTID   0x5740

#define DATA_IN     (0x82 | LIBUSB_ENDPOINT_IN)
#define DATA_OUT    (0x01 | LIBUSB_ENDPOINT_OUT)
#define CTRL_IN     (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define TIMEOUT     10000

static void show_libusb_error(int error_code)
{
    char *error_hint = "";
    const char *error_name;

    /* Available only in libusb > 1.0.3 */
    // error_name = libusb_error_name(error_code);

    switch (error_code) {
        case LIBUSB_ERROR_TIMEOUT:
            error_name="Timeout";
            break;
        case LIBUSB_ERROR_NO_DEVICE:
            error_name="No Device";
            error_hint="Check Ubertooth is connected to host";
            break;
        case LIBUSB_ERROR_ACCESS:
            error_name="Insufficient Permissions";
            break;
        case LIBUSB_ERROR_OVERFLOW:
            error_name="Overflow";
            error_hint="Try resetting the Ubertooth";
            break;
        default:
            error_name="Command Error";
            break;
    }

    fprintf(stderr,"xlibUSB Error: %s: %s (%d) [%s]\n",
            error_name, error_hint, error_code, libusb_error_name(error_code));
}

#define PKT_LEN       (64 * 256)

uint8_t rx_buf[PKT_LEN];

static void callback_xfer(struct libusb_transfer *xfer)
{
    int ret;

    //LIBUSB_TRANSFER_COMPLETED
    //fprintf(stderr, "USB transfer callback %d\n", xfer->status);
    //fprintf(stderr, "%x %x %x %x\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    //fprintf(stderr,"*");
    ret = libusb_submit_transfer(xfer);
    if (ret < 0)
        fprintf(stderr, "Failed to submit USB transfer (%d)\n", ret);

    UberPacket((uint16_t *)rx_buf, PKT_LEN/2);
}

static int do_exit = 0;
struct libusb_device_handle *devh = NULL;
struct libusb_transfer *rx_transfer = NULL;

static void sighandler(int signum)
{
    do_exit = 1;
}

int uberopen(void)
{
    int ret;

    if (devh == NULL) return 1;

    ret = libusb_submit_transfer(rx_transfer);
    if (ret < 0) {
        fprintf(stderr, "rx_transfer submission err: %d\n", ret);
        return 1;
    }

    unsigned char data[4];
    ret = libusb_control_transfer(devh, CTRL_IN, 0x22, 0, 0, data, 4, 3000);
    if (ret < 0) {
        show_libusb_error(ret);
        return 1;
    }

    return 0;
}

void uberclose(void)
{
    int ret;

    if (devh == NULL) return;

    unsigned char data[4];
    ret = libusb_control_transfer(devh, CTRL_IN, 0x33, 0, 0, data, 4, 3000);
    if (ret < 0) {
        show_libusb_error(ret);
    }
}

void uber_handle_events(void)
{
    int ret;
    struct timeval tv = {1, 0};

    if (devh == NULL) return;

    ret = libusb_handle_events_timeout(NULL, &tv);
    if (ret < 0)
        fprintf(stderr, "libusb handle events %d\n", ret);
}

int uberusb(void)
{
    struct sigaction sigact;

    int usb_devs;
    int i;

    struct libusb_context *ctx = NULL;
    struct libusb_device **usb_list = NULL;
    struct libusb_device_descriptor desc;

    int usbindex = -1;

    int ret = libusb_init(NULL);
    if (ret < 0) {
        fprintf(stderr, "libusb_init failed (got 1.0?)\n");
        return -1;
    }

    usb_devs = libusb_get_device_list(ctx, &usb_list);

    for (i = 0; i < usb_devs; ++i) {
        ret = libusb_get_device_descriptor(usb_list[i], &desc);
        if (ret < 0)
            fprintf(stderr, "couldn't get usb descriptor for dev #%d!\n", i);
        else {
            fprintf(stderr, "usbdev %d: %x:%x\n", i, desc.idVendor, desc.idProduct);
            if (desc.idVendor == U0_VENDORID && desc.idProduct == U0_PRODUCTID) {
                usbindex = i;
                fprintf(stderr, "find usbdev %d: %x:%x\n", i, desc.idVendor, desc.idProduct);
                break;
            }
        }
    }

    if (usbindex < 0) {
        fprintf(stderr, "could not find usbdev\n");
        goto LIBUSB_STOP;
    }

    ret = libusb_open(usb_list[usbindex], &devh);
    if (ret) show_libusb_error(ret);
    if (devh == NULL) {
        fprintf(stderr, "could not open device\n");
        goto LIBUSB_STOP;
    }

    ret = libusb_detach_kernel_driver(devh, 0);
    if (ret < 0) {
        fprintf(stderr, "usb_detach_kernel_driver_np error %d\n", ret);
    }

    ret = libusb_claim_interface(devh, 0);
    if (ret < 0) {
        fprintf(stderr, "usb_claim_interface error %d\n", ret);
        goto LIBUSB_STOP;
    }

    {
        sigact.sa_handler = sighandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        sigaction(SIGINT, &sigact, NULL);
        sigaction(SIGTERM, &sigact, NULL);
        sigaction(SIGQUIT, &sigact, NULL);
    }

    rx_transfer = libusb_alloc_transfer(0);

    libusb_fill_bulk_transfer(rx_transfer, devh, DATA_IN,
            rx_buf, PKT_LEN, callback_xfer, NULL, TIMEOUT);

    fprintf(stderr, "uberusb init done\n");

    return ret;

LIBUSB_STOP:
    if(rx_transfer != NULL)
        libusb_cancel_transfer(rx_transfer);

    if(devh != NULL) {
        libusb_release_interface(devh, 0);
        libusb_attach_kernel_driver(devh, 0);
        libusb_close(devh);
    }

    libusb_exit(NULL);

    return ret;
}

