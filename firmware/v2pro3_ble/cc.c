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

u8 cx_strobe(u8 reg);
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

void cc_SRFoff(void)
{
    cc_strobe(SRFOFF);
}

void cc_SRFon_RX(void)
{
    wait_fsunlock();
    cc_strobe(SFSON);
    wait_fslock();
    cc_strobe(SRX);
}

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

void cc_init_modulation(int mode, u32 sync)
{
    u16 grmdm, mdmctrl;
    if (mode == MOD_BT_BASIC_RATE) {
        mdmctrl = 0x0029;   // 160 kHz frequency deviation
        grmdm = 0x0461;     // un-buffered mode, packet w/ sync word detection
        // 0 00 00 1 000 11 0 00 0 1
        //   |  |  | |   |  +--------> CRC off
        //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
        //   |  |  | +---------------> 0 preamble bytes of 01010101
        //   |  |  +-----------------> packet mode
        //   |  +--------------------> un-buffered mode
        //   +-----------------------> sync error bits: 0

    } else if (mode == MOD_BT_LOW_ENERGY) {
        mdmctrl = 0x0040;   // 250 kHz frequency deviation
        grmdm = 0x0561;     // un-buffered mode, packet w/ sync word detection
        // 0 00 00 1 010 11 0 00 0 1
        //   |  |  | |   |  +--------> CRC off
        //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
        //   |  |  | +---------------> 2 preamble bytes of 01010101
        //   |  |  +-----------------> packet mode
        //   |  +--------------------> un-buffered mode
        //   +-----------------------> sync error bits: 0
    } else {
        /* oops */
        return;
    }

    // Bluetooth-like modulation
    cc_set(MANAND,  0x7fff);
    cc_set(LMTST,   0x2b22);    // LNA and receive mixers test register

    cc_set(MDMTST0, 0x124b);    // cc_set(MDMTST0, 0x134b);    // no PRNG
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

    cc_set(GRMDM,   grmdm);

    cc_set(SYNCL,   sync & 0xffff);
    cc_set(SYNCH,   (sync >> 16) & 0xffff);
}

void cc_set_channel(u32 channel)
{


}



/* start un-buffered rx */
void cc_rx_sync(int m, u32 sync, int channel)
{
    u16 grmdm, mdmctrl;
    if (m == MOD_BT_BASIC_RATE) {
        mdmctrl = 0x0029;   // 160 kHz frequency deviation
        grmdm = 0x0101;     // un-buffered mode
        // 0 00 00 0 010 00 0 00 0 1
        //   |  |  | |   |  +--------> CRC off
        //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
        //   |  |  | +---------------> 2 preamble bytes of 01010101
        //   |  |  +-----------------> packet mode
        //   |  +--------------------> un-buffered mode
        //   +-----------------------> sync error bits: 0

    } else if (m == MOD_BT_LOW_ENERGY) {
        mdmctrl = 0x0040;   // 250 kHz frequency deviation
        grmdm = 0x0561;     // un-buffered mode, packet w/ sync word detection
        // 0 00 00 1 010 11 0 00 0 1
        //   |  |  | |   |  +--------> CRC off
        //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
        //   |  |  | +---------------> 2 preamble bytes of 01010101
        //   |  |  +-----------------> packet mode
        //   |  +--------------------> un-buffered mode
        //   +-----------------------> sync error bits: 0
    } else {
        /* oops */
        return;
    }

    // Bluetooth-like modulation
    cc_set(MANAND,  0x7fff);
    cc_set(LMTST,   0x2b22);    // LNA and receive mixers test register

    cc_set(MDMTST0, 0x124b);
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

    cc_set(GRMDM,   grmdm);

    cc_set(SYNCL,   sync & 0xffff);
    cc_set(SYNCH,   (sync >> 16) & 0xffff);

    cc_set(FSDIV,   channel - 1); // 1 MHz IF
    cc_set(MDMCTRL, mdmctrl);

    cc_strobe(SXOSCON);
    while (!(cc_status() & XOSC16M_STABLE));

    cc_strobe(SFSON);
    wait_fslock();
    PAEN_SET();
    HGM_SET();
}

/* start un-buffered rx */
void cc_rx_mode(int m, int channel)
{
    if (m == MOD_BT_BASIC_RATE) {
        cc_set(MANAND,  0x7fff);
        cc_set(LMTST,   0x2b22);
        cc_set(MDMTST0, 0x134b); // without PRNG
        cc_set(GRMDM,   0x0101); // un-buffered mode, GFSK
        cc_set(FSDIV,   channel - 1); // 1 MHz IF
        cc_set(MDMCTRL, 0x0029); // 160 kHz frequency deviation
    } else if (m == MOD_BT_LOW_ENERGY) {
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

    cc_strobe(SFSON);
    wait_fslock();
    cc_strobe(SRX);
    PAEN_SET();
    HGM_SET();

    cc_debug("cchip rx mode\n");
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

    cc_strobe(SRFOFF);
    wait_fsunlock();
    PAEN_SET();
    HGM_SET();

    cc_debug("cchip entry specan mode\n");
}

/* specan stuff */
volatile u16 low_freq = 2400;
volatile u16 high_freq = 2483;

void cc_specan_date(u8 *buf, int len)
{
    static int f = 2400;
    int i = 0;

    //cc_debug("get_specan\n");

     while(len > 0) {
        len -= 2;
        cc_set(FSDIV, f - 1);
        cc_strobe(SFSON);
        wait_fslock();
        cc_strobe(SRX);

        {
            u32 k;
            for (k = 0; k < 30000; k++) { /* Wait a bit. */
                __asm__("nop");
            }
        }

        buf[i++] = f - low_freq;
        buf[i++] = cc_get(RSSI) >> 8;

        {
            char t = (char)buf[i-1];
            t *= -1;
            //cc_puthex(t, 2); cc_putchar('\n');cc_putchar('\r');
        }

        f += 1;
        if (f > high_freq) f = low_freq;

        cc_strobe(SRFOFF);
        wait_fsunlock();
    }
}
