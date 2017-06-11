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


#define LED1_SET()      gpio_set(GPIOC, PIN_LED1)
#define LED1_CLR()      gpio_clear(GPIOC, PIN_LED1)

#define LED2_SET()      gpio_set(GPIOC, PIN_LED2)
#define LED2_CLR()      gpio_clear(GPIOC, PIN_LED2)

#define LED3_SET()      gpio_set(GPIOC, PIN_LED3)
#define LED3_CLR()      gpio_clear(GPIOC, PIN_LED3)

#define LED4_SET()      gpio_set(GPIOC, PIN_LED4)
#define LED4_CLR()      gpio_clear(GPIOC, PIN_LED4)

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

#include <libopencm3/stm32/usart.h>

#define nUSART USART1

#define kputc(a) usart_send_blocking(nUSART, a);
void kputhex(unsigned int value, int digits);
void kputs(char *s);
void delay(void);

#endif

