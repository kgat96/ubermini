/*
 * This file is part of the ubermini project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "config.h"
#include "usbhid.h"
#include "uart.h"

#include "cc.h"

#define usb_debug(a)         kputs(a)
#define usb_puthex(a, b)     kputhex(a, b)
#define usb_putchar(a)       kputc(a)

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = U1_VENDORID,
    .idProduct = U1_PRODUCTID,
    .bcdDevice = UBERTOOTH_API_VERSION,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = BULK_OUT_EP,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = BULK_IN_EP,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
} };

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 1, /* boot */
    .bInterfaceProtocol = 2, /* mouse */
    .iInterface = 0,

    .endpoint = comm_endp,

    //.extra = &hid_function,
    //.extralen = sizeof(hid_function)
} };

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
} };

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
} };

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0xc0,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char * usb_strings[] = {
    "kgat96@gmail.com",
    "ubertoothp",
    "usb-cdc",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int hid_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
            void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)usbd_dev;

    UNUSED(buf);
    UNUSED(len);

    //uint16_t value = req->wValue;
    //kputs("cp:");kputhex(req->bRequest, 2);

    printf("bRequest %d wLength %d wValue %d\n", req->bRequest, req->wLength, req->wValue);

    if (usb_request(req->bRequest))
        return USBD_REQ_HANDLED;

//    uint8_t da[4] = {0, 0, 0, 0};
//    usbd_ep_write_packet(usbd_dev, 0x81, da, 4);
//    return USBD_REQ_HANDLED;//USBD_REQ_NOTSUPP

    return USBD_REQ_NOTSUPP;
}

static void hci_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    kputs("rx");

    if (len) {
        //while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
        //kputs("epout:");kputhex(len,2);
    }
}

static void hci_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;
    (void)usbd_dev;

    kputs("tx\n");
    //usb_putchar('i');
}

static void hci_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, BULK_OUT_EP, USB_ENDPOINT_ATTR_BULK, 64, hci_data_rx_cb);
    usbd_ep_setup(usbd_dev, BULK_IN_EP, USB_ENDPOINT_ATTR_BULK, 64, hci_data_tx_cb);
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback( usbd_dev,
            USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                hid_control_request);

    usbd_register_control_callback( usbd_dev, USB_REQ_TYPE_VENDOR ,
            USB_REQ_TYPE_TYPE, hid_control_request);
}

usbd_device *usbd_dev;

void usb_setup(void)
{
    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3,
                usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, hci_set_config);

    usb_debug("usbd register done!\n");

}

int usb_write_packet(u8* buf, int len)
{
    return usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
}

void usb_poll(void)
{
    usbd_poll(usbd_dev);
}

