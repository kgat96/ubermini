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
        //delay();
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

    cc_strobe(SXOSCON);

    while (!(cc_status() & XOSC16M_STABLE)) {
        delay();
        cc_puthex(cc_get(FSMSTATE), 2);
    };

    cc_puthex(cc_get(FSMSTATE), 2);

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

/* operation mode */
volatile u8 mode = MODE_RX_SYMBOLS;
volatile u8 requested_mode = MODE_IDLE;
volatile u8 modulation = MOD_BT_BASIC_RATE;

/* specan stuff */
volatile u16 low_freq = 2400;
volatile u16 high_freq = 2483;

/* hopping stuff */
volatile u8  hop_mode = HOP_SWEEP;
volatile u8  do_hop = 0;                  // set by timer interrupt
volatile u16 channel = 2441;
volatile u16 hop_direct_channel = 0;      // for hopping directly to a channel
volatile u16 hop_timeout = 158;
volatile u16 requested_channel = 2402;
volatile u16 saved_request = 0;

u64 syncword;
u8 afh_enabled;
u8 afh_map[10];
u8 used_channels;

volatile u32 clkn = 0;
volatile u32 last_hop = 0;

volatile u32 clkn_offset = 0;
volatile u16 clk100ns_offset = 0;

void cc_clkn_handler(void)
{
    clkn += clkn_offset + 1;
    clkn_offset = 0;

    /* NONE or SWEEP -> 25 Hz */
    if (hop_mode == HOP_NONE || hop_mode == HOP_SWEEP) {
        if ((clkn & 0x7f) == 0) {
            do_hop = 1;
        }
    }
}

void cc_hop(void)
{
    static char max_rssi = -120;

    if (!do_hop) {
        char rssi = (int8_t)(cc_get(RSSI) >> 8);
        if (rssi > max_rssi) max_rssi = rssi;
        return;
    } else {
        //cc_puthex(channel, 2);
        //cc_putchar('-');
        //cc_puthex(max_rssi*(-1), 2);
        //cc_putchar('\n');cc_putchar('\r');
        max_rssi = -120;
    }

    do_hop = 0;
    last_hop = clkn;

    // No hopping, if channel is set correctly, do nothing
    if (hop_mode == HOP_NONE) {
        if (cc_get(FSDIV) == (channel - 1))
            return;
    }
    /* only hop to currently used channels if AFH is enabled
     */
    else if (hop_mode == HOP_SWEEP) {
        do {
            channel += 32;
            if (channel > 2480)
                channel -= 79;
        } while ( used_channels != 0 && afh_enabled && !( afh_map[(channel-2402)/8] & 0x1<<((channel-2402)%8) ) );
    }

    /* IDLE mode, but leave amp on, so don't call cc2400_idle(). */
    cc_strobe(SRFOFF);
    wait_fsunlock();

    /* return */
    if(mode == MODE_TX_SYMBOLS)
        cc_set(FSDIV, channel);
    else
        cc_set(FSDIV, channel - 1);

    /* Wait for lock */
    cc_strobe(SFSON);
    wait_fslock();

    if(mode == MODE_TX_SYMBOLS)
        cc_strobe(STX);
    else
        cc_strobe(SRX);
}

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


void get_specan_date(u8 *buf, int len)
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


