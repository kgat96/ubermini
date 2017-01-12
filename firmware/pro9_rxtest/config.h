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

#endif

