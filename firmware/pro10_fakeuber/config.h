/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdio.h>

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef unsigned long int u64;

typedef volatile unsigned int vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char vu8;
typedef volatile unsigned long int vu64;

#define NULL ((void *)0)


#define PIN_MOSI    GPIO7       /* Pa.7  */
#define PIN_MISO    GPIO6       /* Pa.6  */
#define PIN_GIO6    GPIO3       /* Pa.3  */

#define PIN_ATEST1  GPIO1       /* Pa.1  */
#define PIN_ATEST2  GPIO2       /* Pa.2  */

#define PIN_CC3V3   GPIO12      /* Pb.12 */
#define PIN_RX      GPIO11      /* Pb.11 */
#define PIN_TX      GPIO10      /* Pb.10 */
#define PIN_BTGR    GPIO14      /* Pb.14 */
#define PIN_GIO1    GPIO13      /* Pb.13 */

#define PIN_DBGLED  GPIO15       /* Pb.15 LED4 */
#define PIN_RXLED   GPIO8       /* Pc.8 LED1 */
#define PIN_TXLED   GPIO7       /* Pc.7 LED2 */
#define PIN_USRLED  GPIO6       /* Pc.6 LED3 */

#define PIN_CSN     GPIO5       /* Pc.5  */
#define PIN_SCLK    GPIO4       /* Pc.4  */
#define PIN_PAEN    GPIO3       /* Pc.3  */
#define PIN_HGM     GPIO2       /* Pc.2  */

//#define USRLED        (FIO1PIN & PIN_USRLED)
#define USRLED_SET()    gpio_set(GPIOC, PIN_USRLED)
#define USRLED_CLR()    gpio_clear(GPIOC, PIN_USRLED)
//#define RXLED         (FIO1PIN & PIN_RXLED)
#define RXLED_SET()     gpio_set(GPIOC, PIN_RXLED)
#define RXLED_CLR()     gpio_clear(GPIOC, PIN_RXLED)
//#define TXLED         (FIO1PIN & PIN_TXLED)
#define TXLED_SET()     gpio_set(GPIOC, PIN_TXLED)
#define TXLED_CLR()     gpio_clear(GPIOC, PIN_TXLED)

//#define DEBUGLED
#define DBGLED_SET()     gpio_set(GPIOB, PIN_DBGLED)
#define DBGLED_CLR()     gpio_clear(GPIOB, PIN_DBGLED)

#define CC3V3_SET()     gpio_set(GPIOB, PIN_CC3V3)
#define CC3V3_CLR()     gpio_clear(GPIOB, PIN_CC3V3)
#define RX_SET()        gpio_set(GPIOB, PIN_RX)
#define RX_CLR()        gpio_clear(GPIOB, PIN_RX)
#define TX_SET()        gpio_set(GPIOB, PIN_TX)
#define TX_CLR()        gpio_clear(GPIOB, PIN_TX)
#define CSN_SET()    gpio_set(GPIOC, PIN_CSN)
#define CSN_CLR()    gpio_clear(GPIOC, PIN_CSN)
#define SCLK_SET()   gpio_set(GPIOC, PIN_SCLK)
#define SCLK_CLR()   gpio_clear(GPIOC, PIN_SCLK)
#define MOSI_SET()   gpio_set(GPIOA, PIN_MOSI)
#define MOSI_CLR()   gpio_clear(GPIOA, PIN_MOSI)
//#define GIO6       (FIO2PIN & PIN_GIO6)
#define GIO6_SET()   gpio_set(GPIOA, PIN_GIO6)
#define GIO6_CLR()   gpio_clear(GPIOA, PIN_GIO6)
#define BTGR_SET()   gpio_set(GPIOB, PIN_BTGR)
#define BTGR_CLR()   gpio_clear(GPIOB, PIN_BTGR)
#define MISO()       gpio_get(GPIOA, PIN_MISO)

#define PAEN()      gpio_get(GPIOC, PIN_PAEN)
#define PAEN_SET()  gpio_set(GPIOC, PIN_PAEN)
#define PAEN_CLR()  gpio_clear(GPIOC, PIN_PAEN)
#define HGM()       gpio_get(GPIOC, PIN_HGM)
#define HGM_SET()   gpio_set(GPIOC, PIN_HGM)
#define HGM_CLR()   gpio_clear(GPIOC, PIN_HGM)

#include <libopencm3/stm32/usart.h>

#define nUSART USART1

#define kputc(a) usart_send_blocking(nUSART, a);
void kputhex(unsigned int value, int digits);
void kputs(char *s);
void delay(void);

#define U1_VENDORID    0x1d50
#define U1_PRODUCTID   0x6002

#define UBERTOOTH_API_VERSION 0x0102

#define BULK_IN_EP      0x82
#define BULK_OUT_EP     0x05

enum ubertooth_usb_commands {
    UBERTOOTH_PING               = 0,
    UBERTOOTH_RX_SYMBOLS         = 1,
    UBERTOOTH_TX_SYMBOLS         = 2,
    UBERTOOTH_GET_USRLED         = 3,
    UBERTOOTH_SET_USRLED         = 4,
    UBERTOOTH_GET_RXLED          = 5,
    UBERTOOTH_SET_RXLED          = 6,
    UBERTOOTH_GET_TXLED          = 7,
    UBERTOOTH_SET_TXLED          = 8,
    UBERTOOTH_GET_1V8            = 9,
    UBERTOOTH_SET_1V8            = 10,
    UBERTOOTH_GET_CHANNEL        = 11,
    UBERTOOTH_SET_CHANNEL        = 12,
    UBERTOOTH_RESET              = 13,
    UBERTOOTH_GET_SERIAL         = 14,
    UBERTOOTH_GET_PARTNUM        = 15,
    UBERTOOTH_GET_PAEN           = 16,
    UBERTOOTH_SET_PAEN           = 17,
    UBERTOOTH_GET_HGM            = 18,
    UBERTOOTH_SET_HGM            = 19,
    UBERTOOTH_TX_TEST            = 20,
    UBERTOOTH_STOP               = 21,
    UBERTOOTH_GET_MOD            = 22,
    UBERTOOTH_SET_MOD            = 23,
    UBERTOOTH_SET_ISP            = 24,
    UBERTOOTH_FLASH              = 25,
    BOOTLOADER_FLASH             = 26,
    UBERTOOTH_SPECAN             = 27,
    UBERTOOTH_GET_PALEVEL        = 28,
    UBERTOOTH_SET_PALEVEL        = 29,
    UBERTOOTH_REPEATER           = 30,
    UBERTOOTH_RANGE_TEST         = 31,
    UBERTOOTH_RANGE_CHECK        = 32,
    UBERTOOTH_GET_REV_NUM        = 33,
    UBERTOOTH_LED_SPECAN         = 34,
    UBERTOOTH_GET_BOARD_ID       = 35,
    UBERTOOTH_SET_SQUELCH        = 36,
    UBERTOOTH_GET_SQUELCH        = 37,
    UBERTOOTH_SET_BDADDR         = 38,
    UBERTOOTH_START_HOPPING      = 39,
    UBERTOOTH_SET_CLOCK          = 40,
    UBERTOOTH_GET_CLOCK          = 41,
    UBERTOOTH_BTLE_SNIFFING      = 42,
    UBERTOOTH_GET_ACCESS_ADDRESS = 43,
    UBERTOOTH_SET_ACCESS_ADDRESS = 44,
    UBERTOOTH_DO_SOMETHING       = 45,
    UBERTOOTH_DO_SOMETHING_REPLY = 46,
    UBERTOOTH_GET_CRC_VERIFY     = 47,
    UBERTOOTH_SET_CRC_VERIFY     = 48,
    UBERTOOTH_POLL               = 49,
    UBERTOOTH_BTLE_PROMISC       = 50,
    UBERTOOTH_SET_AFHMAP         = 51,
    UBERTOOTH_CLEAR_AFHMAP       = 52,
    UBERTOOTH_READ_REGISTER      = 53,
    UBERTOOTH_BTLE_SLAVE         = 54,
    UBERTOOTH_GET_COMPILE_INFO   = 55,
    UBERTOOTH_BTLE_SET_TARGET    = 56,
    UBERTOOTH_BTLE_PHY           = 57,
    UBERTOOTH_WRITE_REGISTER     = 58,
    UBERTOOTH_JAM_MODE           = 59,
    UBERTOOTH_EGO                = 60,
    UBERTOOTH_AFH                = 61,
    UBERTOOTH_HOP                = 62,
    UBERTOOTH_TRIM_CLOCK         = 63,
    // UBERTOOTH_GET_API_VERSION    = 64,
    UBERTOOTH_WRITE_REGISTERS    = 65,
    UBERTOOTH_READ_ALL_REGISTERS = 66,
    UBERTOOTH_RX_GENERIC         = 67,
    UBERTOOTH_TX_GENERIC_PACKET  = 68,
    UBERTOOTH_FIX_CLOCK_DRIFT    = 69,
};



#endif

