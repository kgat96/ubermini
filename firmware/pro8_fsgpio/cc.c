/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "cc.h"

#define cc_debug(a)         kputs(a)
#define cc_puthex(a, b)     kputhex(a, b)
#define cc_putchar(a)         kputc(a)
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
    CSN_CLR();

    while (len--) {
        if (data & msb)
            MOSI_SET();
        else
            MOSI_CLR();
        data <<= 1;

        SCLK_SET();
        if (MISO())
            data |= 1;

        SCLK_CLR();
    }

    /* end transaction by raising CSN */
    CSN_SET();

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
void cc_reset(void)
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

    cc_set(IOCFG, (40 << 9) | (40 << 3));
    //cc_set(IOCFG, (61 << 9) | (61 << 3));
    //cc_puthex((cc_get(IOCFG) >> 3) & 0x3f, 2);cc_puthex((cc_get(IOCFG) >> 9) & 0x3f, 2);
    cc_strobe(SXOSCON);

    while (!(cc_status() & XOSC16M_STABLE));

    cc_debug("cchip XOSC16M stable done\n");
}

void cc_init(void)
{
    /* CSN (slave select) is active low */
    CSN_SET();

    /* activate 3V3 supply for CC2400 IO */
    CC3V3_SET();

    // check chip id
    {
        u32 id = cc_get(MANFIDL);

        if (id == 0x133d) {
            cc_debug("read cc chip id:"); cc_puthex(id, 4);cc_debug("\n");
        } else {
            cc_debug("Can't read cc chip id !");
        }
    }

    /* initialize various cc2400 settings - see datasheet pg63 */
    cc_set(MANAND,  0x7fff);

    if (cc_get(AGCCTRL) != 0xf700) {
        cc_debug("RF chip error!\n");
        while(1);
    }

    clock_init();
}

/* DMA buffers */
#define DMA_SIZE 50
u8 rxbuf1[DMA_SIZE];
u8 rxbuf2[DMA_SIZE];

/* rx terminal count and error interrupt counters */
volatile u32 rx_tc;
volatile u32 rx_err;

/*
 * The active buffer is the one with an active DMA transfer.
 * The idle buffer is the one we can read/write between transfers.
 */
u8 *active_rxbuf = &rxbuf1[0];
u8 *idle_rxbuf = &rxbuf2[0];

volatile u32 mode = MODE_IDLE;
volatile u32 requested_mode = MODE_IDLE;
volatile u32 modulation = MOD_BT_BASIC_RATE;
volatile u16 channel = 2441;
volatile u16 low_freq = 2400;
volatile u16 high_freq = 2483;

/* start un-buffered rx */
void cc_rx_mode(void)
{
    if (modulation == MOD_BT_BASIC_RATE) {
        cc_set(MANAND,  0x7fff);
        cc_set(LMTST,   0x2b22);
        cc_set(MDMTST0, 0x134b); // without PRNG
        cc_set(GRMDM,   0x0101); // un-buffered mode, GFSK
        cc_set(FSDIV,   channel - 1); // 1 MHz IF
        cc_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
    } else if (modulation == MOD_BT_LOW_ENERGY) {
        cc_set(MANAND,  0x7fff);
        cc_set(LMTST,   0x2b22);
        cc_set(MDMTST0, 0x134b); // without PRNG
        cc_set(GRMDM,   0x0101); // un-buffered mode, GFSK
        cc_set(FSDIV,   channel - 1); // 1 MHz IF
        cc_set(MDMCTRL, 0x0040); // 250 kHz frequency deviation
    } else {
        /* oops */
        return;
    }

    cc_strobe(SXOSCON);

    while (!(cc_status() & XOSC16M_STABLE));

    cc_debug("tt1\n");
    delay();delay();delay();delay();delay();delay();
    cc_debug("tt2\n");

    cc_strobe(SFSON);

    delay();

    while (!(cc_status() & FS_LOCK));

    cc_debug("tt3\n");

    cc_strobe(SRX);

    PAEN_SET();
    HGM_SET();
}

void cc_specan_mode(void)
{
    cc_set(MANAND,  0x7fff);
    cc_set(LMTST,   0x2b22);
    cc_set(MDMTST0, 0x134b); // without PRNG
    cc_set(GRMDM,   0x0101); // un-buffered mode, GFSK
    cc_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
    //FIXME maybe set RSSI.RSSI_FILT

    cc_strobe(SXOSCON);

    while (!(cc_status() & XOSC16M_STABLE));

    while ((cc_status() & FS_LOCK));

    PAEN_SET();
    HGM_SET();

    cc_debug("cc entry specan mode\n");
}


void get_specan_date(u8 *buf, int len)
{
    static int f = 2400;
    int i = 0;

     while(len > 0) {
        len -= 2;
        cc_set(FSDIV, f - 1);
        cc_strobe(SFSON);
        while (!(cc_status() & FS_LOCK));
        cc_strobe(SRX);

        buf[i++] = f - low_freq;
        buf[i++] = cc_get(RSSI) >> 8;

        f += 1;
        if (f > high_freq) f = low_freq;

        cc_strobe(SRFOFF);
        while ((cc_status() & FS_LOCK));
    }
}


