/*
 * This file is part of the ubermini project.
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
#include "uart.h"
#include "ble.h"

#define cc_debug(a)         kputs(a)
#define cc_puthex(a, b)     kputhex(a, b)
#define cc_putchar(a)       kputc(a)
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
        if (data & msb) {
            MOSI_SET();
        } else {
            MOSI_CLR();
        }
        data <<= 1;
        SCLK_SET();
        if (MISO()) {
            data |= 1;
        }
        SCLK_CLR();
    }
    /* end transaction by raising CSN */
    CSN_SET();
    return data;
}

/* write multiple bytes to SPI */
static void cc_fifo_write(u8 len, u8 *data)
{
    u8 msb = 1 << 7;
    u8 reg = FIFOREG;
    u8 i, j, temp;

    /* start transaction by dropping CSN */
    CSN_CLR();

    for (i = 0; i < 8; ++i) {
        if (reg & msb)
            MOSI_SET();
        else
            MOSI_CLR();
        reg <<= 1;
        SCLK_SET();
        __asm__("nop");__asm__("nop");__asm__("nop");
        SCLK_CLR();
    }

    for (i = 0; i < len; ++i) {
        temp = data[i];
        for (j = 0; j < 8; ++j) {
            if (temp & msb)
                MOSI_SET();
            else
                MOSI_CLR();
            temp <<= 1;
            SCLK_SET();
            __asm__("nop");__asm__("nop");__asm__("nop");
            SCLK_CLR();
        }
    }

    // this is necessary to clock in the last byte
    //for (i = 0; i < 8; ++i) {
    //    SCLK_SET();
    //    SCLK_CLR();
    //}

    __asm__("nop");__asm__("nop");__asm__("nop");

    /* end transaction by raising CSN */
    CSN_SET();
}

/* read 16 bit value from a register */
static u16 cc_get(u8 reg)
{
    u32 in = 0;
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

u8 cx_strobe(u8 reg)
{
    return cc_spi(8, reg);
}

static inline void wait_fslock(void)
{
    //while (!(cc_status() & FS_LOCK));
    while (!gpio_get(GPIOA, PIN_GIO6));
}

static inline void wait_fsunlock(void)
{
    //while ((cc_status() & FS_LOCK));
    while (gpio_get(GPIOA, PIN_GIO6));
}

//void cc_SRFoff(void)
//{
//    cc_strobe(SRFOFF);
//}
//
//void cc_SRFon_RX(void)
//{
//    wait_fsunlock();
//    cc_strobe(SFSON);
//    wait_fslock();
//    cc_strobe(SRX);
//}

/*
 * Warning: This should only be called when running on the internal oscillator.
 * Otherwise use clock_start().
 */
void cc_reset(void)
{
    cc_strobe(SXOSCOFF);
    delay();

    cc_set(MAIN, 0x0000);
    while (cc_get(MAIN) != 0x0000) {
        cc_puthex(cc_get(FSMSTATE), 2);
        delay();
    }

    cc_set(MAIN, 0x8000);
    while (cc_get(MAIN) != 0x8000) {
        cc_puthex(cc_get(FSMSTATE), 2);
        delay();
    }

    cc_set(MAIN, 0x8000);   // HW v2
    //cc_set(MAIN, 0x8002); // HW v1
}

static void clock_init(void)
{
    /* configure CCxx00 oscillator
     * output carrier sense on GIO1,
     * output 16MHz GIO6 */
    cc_strobe(SXOSCOFF);
    cc_set(IOCFG, (GIO_CLK_16M << 3) | (GIO_LOCK_STATUS << 9));
    cc_strobe(SXOSCON);                         // goto IDLE status
    while (!(cc_status() & XOSC16M_STABLE)) {
        //cc_puthex(cc_get(FSMSTATE), 2);
    };

    //cc_puthex(cc_get(FSMSTATE), 2);
    cc_debug("cchip XOSC16M stable done\n");
}

void cc_init(void)
{
    /* CSN (slave select) is active low */
    CSN_SET();

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

/* start un-buffered rx */
void rf_init(int m)
{
    u16 mdmctrl;
    if (m == MOD_BT_BASIC_RATE) {
        mdmctrl = 0x0029;   // 160 kHz frequency deviation
    } else if (m == MOD_BT_LOW_ENERGY) {
        mdmctrl = 0x0040;   // 250 kHz frequency deviation
        //mdmctrl |= ((-2 & 0x3f) << 7);  // automatic frequency control
    } else {
        /* oops */
        return;
    }

    cc_strobe(SXOSCON);            // set chip to IDLE status
    while ((cc_get(FSMSTATE) & 0x1f) != STATE_IDLE);

    //cc_putchar('#'); cc_puthex(cc_get(FSMSTATE) & 0x1f, 2);

    // Bluetooth-like modulation
    cc_set(MANAND,  0x7fff);
    cc_set(LMTST,   0x2b22);        // LNA and receive mixers test register

    cc_set(FREND, 0b1011);          // amplifier level (-7 dBm, picked from hat)
    cc_set(INT, 20);                // FIFO_THRESHOLD: 20 bytes
    cc_set(MDMCTRL, mdmctrl);

    PAEN_SET();
    HGM_SET();
}

static void rf_setting(u32 sync, u32 channel, u16 mdmtst0, u16 grmdm)
{
    cc_set(MDMTST0, mdmtst0);
    cc_set(GRMDM,   grmdm);

    cc_set(SYNCL,   sync & 0xffff);
    cc_set(SYNCH,   (sync >> 16) & 0xffff);

    cc_set(FSDIV,   channel);   // 1 MHz IF

    //cc_strobe(SXOSCON);
    //while (!(cc_status() & XOSC16M_STABLE));

    cc_strobe(SFSON);
    wait_fslock();

    //cc_puthex(cc_status(), 2);
    //cc_putchar('@'); cc_puthex(cc_get(FSMSTATE) & 0x1f, 2);

    while ((cc_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON) {
    }

    //cc_puthex(cc_get(FSMSTATE) & 0x1f, 2);
}

void rf_rxmode(u32 sync, u32 channel)
{
    u16 mdmtst0, grmdm;

    mdmtst0 = 0x124b;           // cc_set(MDMTST0, 0x134b);    // no PRNG
    // 1      2      4b
    // 00 0 1 0 0 10 01001011
    //    | | | | |  +---------> AFC_DELTA = ??
    //    | | | | +------------> AFC settling = 4 pairs (8 bit preamble)
    //    | | | +--------------> no AFC adjust on packet
    //    | | +----------------> do not invert data
    //    | +------------------> TX IF freq 1 0Hz
    //    +--------------------> PRNG off
    //
    // ref: CC2400 datasheet page 67
    // AFC settling explained page 41/42

    grmdm = 0x0561;
    // 0 00 00 1 010 11 0 00 0 1
    //                         +-> TX_GAUSSIAN_FILTER
    //                     +-----> NRZ
    //                       +---> FSK/GFSK
    //   |  |  | |   |  +--------> HW CRC
    //   |  |  | |   +-----------> sync word: 32 MSB bits
    //   |  |  | +---------------> 2 preamble bytes of 01010101
    //   |  |  +-----------------> packet mode
    //   |  +--------------------> 0: Un-buffered mode 1: Buffered mode
    //   +-----------------------> sync error bits: 0

    rf_setting(sync, channel-1, mdmtst0, grmdm);
}

void rf_txmode(u32 sync, u32 channel)
{
    u16 mdmtst0, grmdm;

    mdmtst0 = 0x134b;
    // 00 0 1 0 0 11 01001011
    //    | | | | |  +---------> AFC_DELTA = ??
    //    | | | | +------------> AFC settling = 8 pairs (16 bit preamble)
    //    | | | +--------------> no AFC adjust on packet
    //    | | +----------------> do not invert data
    //    | +------------------> TX IF freq 1 0Hz
    //    +--------------------> PRNG off

    grmdm = 0x0c01;
    // 0 00 01 1 000 00 0 00 0 1
    //                         +-> TX_GAUSSIAN_FILTER
    //                     +-----> NRZ
    //                       +---> FSK/GFSK
    //   |  |  | |   |  +--------> HW CRC
    //   |  |  | |   +-----------> sync word: 0 MSB bits
    //   |  |  | +---------------> 0 preamble bytes
    //   |  |  +-----------------> packet mode
    //   |  +--------------------> 0: Un-buffered mode 1: Buffered mode
    //   +-----------------------> sync error bits: 0

    if (sync & 1)
        sync = 0xaaaa0000;
    else
        sync = 0x55550000;

    rf_setting(sync, channel, mdmtst0, grmdm);
}

void rf_transfer(u32 len, u8 *txbuf)
{
    cc_set(IOCFG, (GIO_CLK_16M << 3) | (GIO_FIFO_FULL << 9));

    while ((cc_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);

    // put the packet into the FIFO
    for (u32 i = 0; i < len; i += 16) {
        while (gpio_get(GPIOA, PIN_GIO6)); // wait for the FIFO to drain (FIFO_FULL false)
        u32 ttmp = len - i;
        if (ttmp > 16)
            ttmp = 16;
        cc_fifo_write(ttmp, txbuf + i);

        if (!i) cc_strobe(STX);
    }

    //while(0) {
    //    while (gpio_get(GPIOA, PIN_GIO6));
    //    cc_fifo_write(16, txbuf);
    //}

    cc_set(IOCFG, (GIO_CLK_16M << 3) | (GIO_LOCK_STATUS << 9));

    while ((cc_get(FSMSTATE) & 0x1f) != STATE_STROBE_TX_OFF);

    cc_strobe(SFSON);
    //wait_fslock();
}

void rf_autofreq(void)
{
    printf("FREQEST %d\n", (signed char)(cc_get(FREQEST)>>8));
}

void cc_clean_fifo(void)
{
    while (SPI3_SR & SPI_SR_RXNE)
        {SPI2_DR = SPI3_DR;};
}
