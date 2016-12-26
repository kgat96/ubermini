/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>

//#include <libopencmsis/core_cm3.h>

#include "cc.h"

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define NULL ((void *)0)

#define kputc(a) usart_send_blocking(USART1, a);

static void kputhex(unsigned int value, int digits)
{
    while (digits-- > 0) {
        unsigned int tmp = (value >> (4 * digits)) & 0xf;
        kputc(tmp > 9 ? tmp - 10 + 'a' : tmp + '0');
    }
}

static void kputs(char *s)
{
    while (*s) {
        if (*s == '\n')
            kputc('\r');
        kputc(*s++);
    }
}

static void delay(void)
{
    u32 i;
    for (i = 0; i < 1000000; i++) { /* Wait a bit. */
        __asm__("nop");
    }
}


static void usart_setup(void)
{
    /* Enable clocks for USART1. */
    rcc_periph_clock_enable(RCC_USART1);

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

#define PIN_MOSI    GPIO7       /* Pa.7  */
#define PIN_MISO    GPIO6       /* Pa.6  */
#define PIN_GIO6    GPIO3       /* Pa.3  */

#define PIN_ATEST1  GPIO1       /* Pa.1  */
#define PIN_ATEST2  GPIO2       /* Pa.2  */

#define PIN_CC3V3   GPIO12      /* Pb.12 */
#define PIN_RX      GPIO11      /* Pb.11 */
#define PIN_TX      GPIO10      /* Pb.10 */
#define PIN_BTGR    GPIO14      /* Pb.14 */

#define PIN_USRLED  GPIO8       /* Pc.8  */
#define PIN_RXLED   GPIO7       /* Pc.7  */
#define PIN_TXLED   GPIO6       /* Pc.6  */
#define PIN_CSN     GPIO5       /* Pc.5  */
#define PIN_SCLK    GPIO4       /* Pc.4  */
#define PIN_PAEN    GPIO3       /* Pc.3  */
#define PIN_HGM     GPIO2       /* Pc.2  */

//#define USRLED     (FIO1PIN & PIN_USRLED)
#define USRLED_SET gpio_set(GPIOC, PIN_USRLED)
#define USRLED_CLR gpio_clear(GPIOC, PIN_USRLED)
//#define RXLED      (FIO1PIN & PIN_RXLED)
#define RXLED_SET  gpio_set(GPIOC, PIN_RXLED)
#define RXLED_CLR  gpio_clear(GPIOC, PIN_RXLED)
//#define TXLED      (FIO1PIN & PIN_TXLED)
#define TXLED_SET  gpio_set(GPIOC, PIN_TXLED)
#define TXLED_CLR  gpio_clear(GPIOC, PIN_TXLED)

#define CC3V3_SET  gpio_set(GPIOB, PIN_CC3V3)
#define CC3V3_CLR  gpio_clear(GPIOB, PIN_CC3V3)
#define RX_SET     gpio_set(GPIOB, PIN_RX)
#define RX_CLR     gpio_clear(GPIOB, PIN_RX)
#define TX_SET     gpio_set(GPIOB, PIN_TX)
#define TX_CLR     gpio_clear(GPIOB, PIN_TX)
#define CSN_SET    gpio_set(GPIOC, PIN_CSN)
#define CSN_CLR    gpio_clear(GPIOC, PIN_CSN)
#define SCLK_SET   gpio_set(GPIOC, PIN_SCLK)
#define SCLK_CLR   gpio_clear(GPIOC, PIN_SCLK)
#define MOSI_SET   gpio_set(GPIOA, PIN_MOSI)
#define MOSI_CLR   gpio_clear(GPIOA, PIN_MOSI)
//#define GIO6       (FIO2PIN & PIN_GIO6)
#define GIO6_SET   gpio_set(GPIOA, PIN_GIO6)
#define GIO6_CLR   gpio_clear(GPIOA, PIN_GIO6)
#define BTGR_SET   gpio_set(GPIOB, PIN_BTGR)
#define BTGR_CLR   gpio_clear(GPIOB, PIN_BTGR)
#define MISO       gpio_get(GPIOA, PIN_MISO)

#define PAEN     gpio_get(GPIOC, PIN_PAEN)
#define PAEN_SET gpio_set(GPIOC, PIN_PAEN)
#define PAEN_CLR gpio_clear(GPIOC, PIN_PAEN)
#define HGM      gpio_get(GPIOC, PIN_HGM)
#define HGM_SET  gpio_set(GPIOC, PIN_HGM)
#define HGM_CLR  gpio_clear(GPIOC, PIN_HGM)

static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);


    /* Set GPIO LED. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO8); // LED1
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO7); // LED2
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO6); // LED3
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO15); // LED4

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO10);

    /* Setup USART1 TX/RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);   // tx
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);  // rx

    /* set certain pins as outputs, all others inputs */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_MOSI);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
            PIN_TX | PIN_CC3V3 | PIN_RX | PIN_BTGR);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
            PIN_USRLED | PIN_RXLED | PIN_TXLED | PIN_CSN | PIN_SCLK | PIN_PAEN | PIN_HGM);

}

/*
 * This is a single SPI transaction of variable length, usually 8 or 24 bits.
 * The CC2400 also supports longer transactions (e.g. for the FIFO), but we
 * haven't implemented anything longer than 32 bits.
 *
 * We're bit-banging because:
 *
 * 1. We're using one SPI peripheral for the CC2400's unbuffered data
 *    interface.
 * 2. We're saving the second SPI peripheral for an expansion port.
 * 3. The CC2400 needs CSN held low for the entire transaction which the
 *    LPC17xx SPI peripheral won't do without some workaround anyway.
 */
static u32 cc_spi(u8 len, u32 data)
{
    u32 msb = 1 << (len - 1);

    /* start transaction by dropping CSN */
    CSN_CLR;

    while (len--) {
        if (data & msb)
            MOSI_SET;
        else
            MOSI_CLR;
        data <<= 1;

        SCLK_SET;
        if (MISO)
            data |= 1;

        SCLK_CLR;
    }

    /* end transaction by raising CSN */
    CSN_SET;

    return data;
}

/* read 16 bit value from a register */
static u16 cc_get(u8 reg)
{
    u32 in;

    u32 out = (reg | 0x80) << 16;
    in = cc_spi(24, out);
    return in & 0xFFFF;
}

/* write 16 bit value to a register */
static void cc_set(u8 reg, u16 val)
{
    u32 out = (reg << 16) | val;
    cc_spi(24, out);
}

static void cc_init(void)
{
    /* CSN (slave select) is active low */
    CSN_SET;

    /* activate 3V3 supply for CC2400 IO */
    CC3V3_SET;

    // check chip id
    {
        u32 id = cc_get(MANFIDL);
        //id |= cc_get(MANFIDH) << 16;
        //kputhex(id, 8);

        if (id == 0x133d) {
            kputs("read cc chip id:"); kputhex(id, 4);kputs("\n");
        } else {
            kputs("Can't read cc chip id !");
        }
    }

    /* initialize various cc2400 settings - see datasheet pg63 */
    cc_set(MANAND,  0x7fff);


}

/* get the status */
static u8 cc_status(void)
{
    return cc_spi(8, 0);
}

/* strobe register, return status */
static u8 cc_strobe(u8 reg)
{
    return cc_spi(8, reg);
}

/*
 * Warning: This should only be called when running on the internal oscillator.
 * Otherwise use clock_start().
 */
static void cc_reset(void)
{
    cc_set(MAIN, 0x0000);
    while (cc_get(MAIN) != 0x0000);
    cc_set(MAIN, 0x8000);
    while (cc_get(MAIN) != 0x8000);

    cc_set(MAIN, 0x8002);
}



static void clock_init(void)
{
    /* configure CCxx00 oscillator
     * output carrier sense on GIO1,
     * output 16MHz GIO6 */

    cc_strobe(SXOSCOFF);

    cc_set(IOCFG, (GIO_CLK_16M << 9) | (GIO_CARRIER_SENSE_N << 3));

    cc_strobe(SXOSCON);

    while (!(cc_status() & XOSC16M_STABLE));

    kputs("cchip XOSC16M stable done\n");

    delay(); // why ???

    /* Enable external high-speed oscillator 16MHz. */
    rcc_osc_bypass_enable(RCC_HSE);
    rcc_clock_setup_hse_3v3(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_120MHZ]);
}

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
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
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
} };

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     }
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
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
    .bmAttributes = 0x80,
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

static int cdcacm_control_request(usbd_device *usbd_dev,
    struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    kputs("cp:");kputhex(req->bRequest, 2);kputhex(req->bmRequestType, 2);
    kputhex(req->wValue, 2);kputhex(req->wIndex, 2);
    //kputhex(*len, 2);

//    kputhex(*buf[0], 2);
//    kputhex(*buf[1], 2);
//    kputhex(*buf[2], 2);
//    kputhex(*buf[3], 2);
//    kputhex(*buf[4], 2);
//    kputhex(*buf[5], 2);
//    kputhex(*buf[6], 2);

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        /*
         * This Linux cdc_acm driver requires this to be implemented
         * even though it's optional in the CDC spec, and we don't
         * advertise it in the ACM functional descriptor.
         */
        return 1;
        }
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding)) {
            return 1;
        }

        return 1;
    }

    return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if (len) {
        //while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
        kputs("ep:");kputhex(len,2);
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
            cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback( usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}

int main(void)
{
    usbd_device *usbd_dev;

//    /* Enable internal high-speed oscillator. */
//    rcc_osc_on(RCC_HSI);
//    rcc_wait_for_osc_ready(RCC_HSI);
//
//    /* Select HSI as SYSCLK source. */
//    rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

    gpio_setup();

    usart_setup();

    kputs("\nTest start\n");

    cc_reset();

    cc_init();

    clock_init();

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 2000000);

    kputs("stm32 clock init done!\n");

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_OTGFS);

    /* USB pins */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12 );

    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12 );

    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3,
                usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

    kputs("usbd register done!\n");

    while (1) {
        usbd_poll(usbd_dev);
    }


    while (1) {
        /* Manually: */
        /* Using API functions gpio_set()/gpio_clear(): */
        /* Using API function gpio_toggle(): */

        gpio_set(GPIOC, GPIO0); /* LED on/off */
        gpio_set(GPIOB, GPIO15); /* LED on/off */
        delay();
        gpio_clear(GPIOC, GPIO0); /* LED on/off */
        gpio_clear(GPIOB, GPIO15); /* LED on/off */
        delay();

    }

    return 0;
}
