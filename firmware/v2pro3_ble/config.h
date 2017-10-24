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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>

#include <libopencm3/stm32/dma.h>

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef unsigned long int u64;

typedef volatile unsigned int vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char vu8;
typedef volatile unsigned long int vu64;

#define NULL ((void *)0)

#define UNUSED(x) (void)(x)

#define PIN_MOSI    GPIO11      /* Pc.11  */
#define PIN_MISO    GPIO10      /* Pc.10  */
#define PIN_SCLK    GPIO12      /* Pc.12  */
#define PIN_CSN     GPIO2       /* Pd.2  */

#define PIN_GIO6    GPIO15      /* Pa.15  */

#define PIN_RX      GPIO7      /* Pb.7 */
#define PIN_TX      GPIO6      /* Pb.6 */
#define PIN_BTGR    GPIO8      /* Pb.8 */

#define PIN_LED1   GPIO3       /* Pc.3 LED1 */
#define PIN_LED2   GPIO2       /* Pc.2 LED2 */
#define PIN_LED3   GPIO1       /* Pc.1 LED3 */
#define PIN_LED4   GPIO0       /* Pc.0 LED4 */

#define PIN_PAEN    GPIO13       /* Pc.13  */
#define PIN_HGM     GPIO14       /* Pc.14  */

#define UART_RX         GPIO10       /* Pa.10  */

#define UART_TOG()      gpio_toggle(GPIOA, UART_RX)

#define LED1_SET()      gpio_set(GPIOC, PIN_LED1)
#define LED1_CLR()      gpio_clear(GPIOC, PIN_LED1)
#define LED1_TOG()      gpio_toggle(GPIOC, PIN_LED1)

#define LED2_SET()      gpio_set(GPIOC, PIN_LED2)
#define LED2_CLR()      gpio_clear(GPIOC, PIN_LED2)
#define LED2_TOG()      gpio_toggle(GPIOC, PIN_LED2)

#define LED3_SET()      gpio_set(GPIOC, PIN_LED3)
#define LED3_CLR()      gpio_clear(GPIOC, PIN_LED3)
#define LED3_TOG()      gpio_toggle(GPIOC, PIN_LED3)

#define LED4_SET()      gpio_set(GPIOC, PIN_LED4)
#define LED4_CLR()      gpio_clear(GPIOC, PIN_LED4)
#define LED4_TOG()      gpio_toggle(GPIOC, PIN_LED4)

#define RX_SET()        gpio_set(GPIOB, PIN_RX)
#define RX_CLR()        gpio_clear(GPIOB, PIN_RX)
#define TX_SET()        gpio_set(GPIOB, PIN_TX)
#define TX_CLR()        gpio_clear(GPIOB, PIN_TX)

#define CSN_SET()    gpio_set(GPIOD, PIN_CSN)
#define CSN_CLR()    gpio_clear(GPIOD, PIN_CSN)
#define SCLK_SET()   gpio_set(GPIOC, PIN_SCLK)
#define SCLK_CLR()   gpio_clear(GPIOC, PIN_SCLK)
#define MOSI_SET()   gpio_set(GPIOC, PIN_MOSI)
#define MOSI_CLR()   gpio_clear(GPIOC, PIN_MOSI)

#define GIO6_SET()   gpio_set(GPIOA, PIN_GIO6)
#define GIO6_CLR()   gpio_clear(GPIOA, PIN_GIO6)
#define BTGR_SET()   gpio_set(GPIOB, PIN_BTGR)
#define BTGR_CLR()   gpio_clear(GPIOB, PIN_BTGR)
#define MISO()       gpio_get(GPIOC, PIN_MISO)

#define PAEN()      gpio_get(GPIOC, PIN_PAEN)
#define PAEN_SET()  gpio_set(GPIOC, PIN_PAEN)
#define PAEN_CLR()  gpio_clear(GPIOC, PIN_PAEN)
#define HGM()       gpio_get(GPIOC, PIN_HGM)
#define HGM_SET()   gpio_set(GPIOC, PIN_HGM)
#define HGM_CLR()   gpio_clear(GPIOC, PIN_HGM)

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

