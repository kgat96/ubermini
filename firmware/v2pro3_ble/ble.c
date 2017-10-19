/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2017 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */




static const u8 whitening_index[] = {
    70, 62, 120, 111, 77, 46, 15, 101, 66, 39, 31, 26, 80,
    83, 125, 89, 10, 35, 8, 54, 122, 17, 33, 0, 58, 115, 6,
    94, 86, 49, 52, 20, 40, 27, 84, 90, 63, 112, 47, 102
};

static const u8 whitening[] = {
    1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1,
    1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0,
    0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0,
    1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1
};

/* write multiple bytes to SPI */
static void cc_fifo_write(u8 len, u8 *data) {
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
            SCLK_CLR();
        }
    }

    // this is necessary to clock in the last byte
    for (i = 0; i < 8; ++i) {
        SCLK_SET();
        SCLK_CLR();
    }

    //delay();

    __asm__("nop");__asm__("nop");__asm__("nop");
    __asm__("nop");__asm__("nop");__asm__("nop");


    if (0) {
        u32 nn = 500;
        while(nn--) {
            __asm__("nop");__asm__("nop");__asm__("nop");
        }
    }

    /* end transaction by raising CSN */
    CSN_SET();
}

static u8 btle_channel_index(u8 ch)
{
    u8 idx;
    ch /= 2;
    if (ch == 0)
        idx = 37;
    else if (ch < 12)
        idx = ch - 1;
    else if (ch == 12)
        idx = 38;
    else if (ch < 39)
        idx = ch - 2;
    else
        idx = 39;
    return idx;
}

/*
 * Transmit a BTLE packet with the specified access address.
 *
 * All modulation parameters are set within this function. The data
 * should not be pre-whitened, but the CRC should be calculated and
 * included in the data length.
 */
static void le_transmit(u32 aa, u8 len, u8 *data)
{
    unsigned i, j;
    int bit;
    u8 txbuf[64];
    u8 tx_len;
    u8 byte;
    u16 gio_save;

    channel = 2402;     // 37

    // first four bytes: AA
    for (i = 0; i < 4; ++i) {
        byte = aa & 0xff;
        aa >>= 8;
        txbuf[i] = 0;
        for (j = 0; j < 8; ++j) {
            txbuf[i] |= (byte & 1) << (7 - j);
            byte >>= 1;
        }
    }

    // whiten the data and copy it into the txbuf
    int idx = whitening_index[btle_channel_index(channel-2402)];
    for (i = 0; i < len; ++i) {
        byte = data[i];
        txbuf[i+4] = 0;
        for (j = 0; j < 8; ++j) {
            bit = (byte & 1) ^ whitening[idx];
            idx = (idx + 1) % sizeof(whitening);
            byte >>= 1;
            txbuf[i+4] |= bit << (7 - j);
        }
    }

    len += 4; // include the AA in len

    // Bluetooth-like modulation
    cc_set(MANAND,  0x7fff);
    cc_set(LMTST,   0x2b22);    // LNA and receive mixers test register
    cc_set(MDMTST0, 0x134b);    // no PRNG

    cc_set(GRMDM,   0x0c01);
    // 0 00 01 1 000 00 0 00 0 1
    //      |  | |   |  +--------> CRC off
    //      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
    //      |  | +---------------> 0 preamble bytes of 01010101
    //      |  +-----------------> packet mode
    //      +--------------------> buffered mode

    cc_set(FSDIV,   channel);   // 37
    cc_set(FREND,   0b1011);    // amplifier level (-7 dBm, picked from hat)
    cc_set(MDMCTRL, 0x0040);    // 250 kHz frequency deviation
    cc_set(INT,     0x0014);    // FIFO_THRESHOLD: 20 bytes

    // sync byte depends on the first transmitted bit of the AA
    if (aa & 1)
        cc_set(SYNCH,   0xaaaa);
    else
        cc_set(SYNCH,   0x5555);

    // set GIO to FIFO_FULL
    gio_save = cc_get(IOCFG);

    cc_set(IOCFG, (GIO_CLK_16M << 3) | (GIO_FIFO_FULL << 9));

    while (!(cc_status() & XOSC16M_STABLE));
    cc_strobe(SFSON);

    while (!(cc_status() & FS_LOCK));

    PAEN_SET();

    while(1) {
        kputc('.');

        if (0) {
            for(i = 0; i < 64; i++) {
                txbuf[i] = (i & 1) ? 0xff : 0xff;
            }

            //txbuf[0] = 0xff;
            //txbuf[1] = 0xff;

            //txbuf[2] = 0;
            //txbuf[3] = 0;

            //txbuf[4] = 0xff;
            //txbuf[5] = 0xff;

            //txbuf[6] = 0;
            //txbuf[7] = 0;
        }

        while ((cc_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);
        cc_strobe(STX);

        // put the packet into the FIFO
        for (i = 0; i < len; i += 16) {
            while (gpio_get(GPIOA, PIN_GIO6)); // wait for the FIFO to drain (FIFO_FULL false)
            tx_len = len - i;
            if (tx_len > 16)
                tx_len = 16;
            cc_fifo_write(tx_len, txbuf + i);
        }

        {
            u32 nn = 50000;
            while(nn--) {
                __asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");
                __asm__("nop");__asm__("nop");__asm__("nop");
            }
        }
    }

    while ((cc_get(FSMSTATE) & 0x1f) != STATE_STROBE_FS_ON);

    cc_strobe(SRFOFF);
    while ((cc_status() & FS_LOCK));

    PAEN_CLR();

    // reset GIO
    cc_set(IOCFG, gio_save);
}

// calculate CRC
//  note 1: crc_init's bits should be in reverse order
//  note 2: output bytes are in reverse order compared to wire
//
//      example output:
//          0x6ff46e
//
//      bytes in packet will be:
//        { 0x6e, 0xf4, 0x6f }
//
static u32 btle_calc_crc(u32 crc_init, u8 *data, int len)
{
    u32 state = crc_init & 0xffffff;
    u32 lfsr_mask = 0x5a6000; // 010110100110000000000000
    int i, j;

    for (i = 0; i < len; ++i) {
        u8 cur = data[i];
        for (j = 0; j < 8; ++j) {
            int next_bit = (state ^ cur) & 1;
            cur >>= 1;
            state >>= 1;
            if (next_bit) {
                state |= 1 << 23;
                state ^= lfsr_mask;
            }
        }
    }
    return state;
}


/* le stuff */
uint8_t slave_mac_address[6] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
void ble_adv(void);

void ble_adv(void)
{
    u32 calc_crc;
    int i;

    u8 adv_ind[] = {
        // LL header
        0x00, 0x09,

        // advertising address
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

        // advertising data
        0x02, 0x01, 0x05,

        // CRC (calc)
        0xff, 0xff, 0xff,
    };

    u8 adv_ind_len = sizeof(adv_ind) - 3;

    // copy the user-specified mac address
    for (i = 0; i < 6; ++i)
        adv_ind[i+2] = slave_mac_address[5-i];

    calc_crc = btle_calc_crc(0xAAAAAA, adv_ind, adv_ind_len);
    adv_ind[adv_ind_len+0] = (calc_crc >>  0) & 0xff;
    adv_ind[adv_ind_len+1] = (calc_crc >>  8) & 0xff;
    adv_ind[adv_ind_len+2] = (calc_crc >> 16) & 0xff;

    // spam advertising packets
    //while (requested_mode == MODE_BT_SLAVE_LE) {
        le_transmit(0x8e89bed6, adv_ind_len+3, adv_ind);
        //msleep(100);
    //}
}















