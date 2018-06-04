/*
 * This file is part of the ubermini project.
 *
 * Copyright (C) 2017 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <string.h>

#include "config.h"
#include "cc.h"
#include "uart.h"
#include "ble.h"

void delay_us(u32 us);
void delay_ms(u32 us);

static const u32 whitening_word[40][12] = {
    { 0xc3bcb240, 0x5f4a371f, 0x9a9cf685, 0x44c5d6c1, 0xe1de5920, 0xafa51b8f,
      0xcd4e7b42, 0x2262eb60, 0xf0ef2c90, 0x57d28dc7, 0x66a73da1, 0x113175b0, },
    { 0xbcb24089, 0x4a371fc3, 0x9cf6855f, 0xc5d6c19a, 0xde592044, 0xa51b8fe1,
      0x4e7b42af, 0x62eb60cd, 0xef2c9022, 0xd28dc7f0, 0xa73da157, 0x3175b066, },
    { 0x3da157d2, 0x75b066a7, 0x96481131, 0x46e3f877, 0x9ed0abe9, 0xbad83353,
      0xcb240898, 0xa371fc3b, 0xcf6855f4, 0x5d6c19a9, 0xe592044c, 0x51b8fe1d, },
    { 0x42afa51b, 0x60cd4e7b, 0x902262eb, 0xc7f0ef2c, 0xa157d28d, 0xb066a73d,
      0x48113175, 0xe3f87796, 0xd0abe946, 0xd833539e, 0x240898ba, 0x71fc3bcb, },
    { 0x3f877964, 0x0abe946e, 0x833539ed, 0x40898bad, 0x1fc3bcb2, 0x855f4a37,
      0xc19a9cf6, 0x2044c5d6, 0x8fe1de59, 0x42afa51b, 0x60cd4e7b, 0x902262eb, },
    { 0x40898bad, 0x1fc3bcb2, 0x855f4a37, 0xc19a9cf6, 0x2044c5d6, 0x8fe1de59,
      0x42afa51b, 0x60cd4e7b, 0x902262eb, 0xc7f0ef2c, 0xa157d28d, 0xb066a73d, },
    { 0xc19a9cf6, 0x2044c5d6, 0x8fe1de59, 0x42afa51b, 0x60cd4e7b, 0x902262eb,
      0xc7f0ef2c, 0xa157d28d, 0xb066a73d, 0x48113175, 0xe3f87796, 0xd0abe946, },
    { 0xbe946e3f, 0x3539ed0a, 0x898bad83, 0xc3bcb240, 0x5f4a371f, 0x9a9cf685,
      0x44c5d6c1, 0xe1de5920, 0xafa51b8f, 0xcd4e7b42, 0x2262eb60, 0xf0ef2c90, },
    { 0x3bcb2408, 0xf4a371fc, 0xa9cf6855, 0x4c5d6c19, 0x1de59204, 0xfa51b8fe,
      0xd4e7b42a, 0x262eb60c, 0x0ef2c902, 0x7d28dc7f, 0x6a73da15, 0x13175b06, },
    { 0x44c5d6c1, 0xe1de5920, 0xafa51b8f, 0xcd4e7b42, 0x2262eb60, 0xf0ef2c90,
      0x57d28dc7, 0x66a73da1, 0x113175b0, 0xf8779648, 0xabe946e3, 0x33539ed0, },
    { 0xc5d6c19a, 0xde592044, 0xa51b8fe1, 0x4e7b42af, 0x62eb60cd, 0xef2c9022,
      0xd28dc7f0, 0xa73da157, 0x3175b066, 0x77964811, 0xe946e3f8, 0x539ed0ab, },
    { 0xbad83353, 0xcb240898, 0xa371fc3b, 0xcf6855f4, 0x5d6c19a9, 0xe592044c,
      0x51b8fe1d, 0xe7b42afa, 0x2eb60cd4, 0xf2c90226, 0x28dc7f0e, 0x73da157d, },
    { 0xc7f0ef2c, 0xa157d28d, 0xb066a73d, 0x48113175, 0xe3f87796, 0xd0abe946,
      0xd833539e, 0x240898ba, 0x71fc3bcb, 0x6855f4a3, 0x6c19a9cf, 0x92044c5d, },
    { 0xb8fe1de5, 0xb42afa51, 0xb60cd4e7, 0xc902262e, 0xdc7f0ef2, 0xda157d28,
      0x5b066a73, 0x64811317, 0x6e3f8779, 0xed0abe94, 0xad833539, 0xb240898b, },
    { 0x39ed0abe, 0x8bad8335, 0xbcb24089, 0x4a371fc3, 0x9cf6855f, 0xc5d6c19a,
      0xde592044, 0xa51b8fe1, 0x4e7b42af, 0x62eb60cd, 0xef2c9022, 0xd28dc7f0, },
    { 0x46e3f877, 0x9ed0abe9, 0xbad83353, 0xcb240898, 0xa371fc3b, 0xcf6855f4,
      0x5d6c19a9, 0xe592044c, 0x51b8fe1d, 0xe7b42afa, 0x2eb60cd4, 0xf2c90226, },
    { 0x33539ed0, 0x0898bad8, 0xfc3bcb24, 0x55f4a371, 0x19a9cf68, 0x044c5d6c,
      0xfe1de592, 0x2afa51b8, 0x0cd4e7b4, 0x02262eb6, 0x7f0ef2c9, 0x157d28dc, },
    { 0x4c5d6c19, 0x1de59204, 0xfa51b8fe, 0xd4e7b42a, 0x262eb60c, 0x0ef2c902,
      0x7d28dc7f, 0x6a73da15, 0x13175b06, 0x87796481, 0xbe946e3f, 0x3539ed0a, },
    { 0xcd4e7b42, 0x2262eb60, 0xf0ef2c90, 0x57d28dc7, 0x66a73da1, 0x113175b0,
      0xf8779648, 0xabe946e3, 0x33539ed0, 0x0898bad8, 0xfc3bcb24, 0x55f4a371, },
    { 0xb240898b, 0x371fc3bc, 0xf6855f4a, 0xd6c19a9c, 0x592044c5, 0x1b8fe1de,
      0x7b42afa5, 0xeb60cd4e, 0x2c902262, 0x8dc7f0ef, 0x3da157d2, 0x75b066a7, },
    { 0xcf6855f4, 0x5d6c19a9, 0xe592044c, 0x51b8fe1d, 0xe7b42afa, 0x2eb60cd4,
      0xf2c90226, 0x28dc7f0e, 0x73da157d, 0x175b066a, 0x79648113, 0x946e3f87, },
    { 0xb066a73d, 0x48113175, 0xe3f87796, 0xd0abe946, 0xd833539e, 0x240898ba,
      0x71fc3bcb, 0x6855f4a3, 0x6c19a9cf, 0x92044c5d, 0xb8fe1de5, 0xb42afa51, },
    { 0x3175b066, 0x77964811, 0xe946e3f8, 0x539ed0ab, 0x98bad833, 0x3bcb2408,
      0xf4a371fc, 0xa9cf6855, 0x4c5d6c19, 0x1de59204, 0xfa51b8fe, 0xd4e7b42a, },
    { 0x4e7b42af, 0x62eb60cd, 0xef2c9022, 0xd28dc7f0, 0xa73da157, 0x3175b066,
      0x77964811, 0xe946e3f8, 0x539ed0ab, 0x98bad833, 0x3bcb2408, 0xf4a371fc, },
    { 0xcb240898, 0xa371fc3b, 0xcf6855f4, 0x5d6c19a9, 0xe592044c, 0x51b8fe1d,
      0xe7b42afa, 0x2eb60cd4, 0xf2c90226, 0x28dc7f0e, 0x73da157d, 0x175b066a, },
    { 0xb42afa51, 0xb60cd4e7, 0xc902262e, 0xdc7f0ef2, 0xda157d28, 0x5b066a73,
      0x64811317, 0x6e3f8779, 0xed0abe94, 0xad833539, 0xb240898b, 0x371fc3bc, },
    { 0x3539ed0a, 0x898bad83, 0xc3bcb240, 0x5f4a371f, 0x9a9cf685, 0x44c5d6c1,
      0xe1de5920, 0xafa51b8f, 0xcd4e7b42, 0x2262eb60, 0xf0ef2c90, 0x57d28dc7, },
    { 0x4a371fc3, 0x9cf6855f, 0xc5d6c19a, 0xde592044, 0xa51b8fe1, 0x4e7b42af,
      0x62eb60cd, 0xef2c9022, 0xd28dc7f0, 0xa73da157, 0x3175b066, 0x77964811, },
    { 0x371fc3bc, 0xf6855f4a, 0xd6c19a9c, 0x592044c5, 0x1b8fe1de, 0x7b42afa5,
      0xeb60cd4e, 0x2c902262, 0x8dc7f0ef, 0x3da157d2, 0x75b066a7, 0x96481131, },
    { 0x48113175, 0xe3f87796, 0xd0abe946, 0xd833539e, 0x240898ba, 0x71fc3bcb,
      0x6855f4a3, 0x6c19a9cf, 0x92044c5d, 0xb8fe1de5, 0xb42afa51, 0xb60cd4e7, },
    { 0xc902262e, 0xdc7f0ef2, 0xda157d28, 0x5b066a73, 0x64811317, 0x6e3f8779,
      0xed0abe94, 0xad833539, 0xb240898b, 0x371fc3bc, 0xf6855f4a, 0xd6c19a9c, },
    { 0xb60cd4e7, 0xc902262e, 0xdc7f0ef2, 0xda157d28, 0x5b066a73, 0x64811317,
      0x6e3f8779, 0xed0abe94, 0xad833539, 0xb240898b, 0x371fc3bc, 0xf6855f4a, },
    { 0x2262eb60, 0xf0ef2c90, 0x57d28dc7, 0x66a73da1, 0x113175b0, 0xf8779648,
      0xabe946e3, 0x33539ed0, 0x0898bad8, 0xfc3bcb24, 0x55f4a371, 0x19a9cf68, },
    { 0x5d6c19a9, 0xe592044c, 0x51b8fe1d, 0xe7b42afa, 0x2eb60cd4, 0xf2c90226,
      0x28dc7f0e, 0x73da157d, 0x175b066a, 0x79648113, 0x946e3f87, 0x39ed0abe, },
    { 0xdc7f0ef2, 0xda157d28, 0x5b066a73, 0x64811317, 0x6e3f8779, 0xed0abe94,
      0xad833539, 0xb240898b, 0x371fc3bc, 0xf6855f4a, 0xd6c19a9c, 0x592044c5, },
    { 0xa371fc3b, 0xcf6855f4, 0x5d6c19a9, 0xe592044c, 0x51b8fe1d, 0xe7b42afa,
      0x2eb60cd4, 0xf2c90226, 0x28dc7f0e, 0x73da157d, 0x175b066a, 0x79648113, },
    { 0xde592044, 0xa51b8fe1, 0x4e7b42af, 0x62eb60cd, 0xef2c9022, 0xd28dc7f0,
      0xa73da157, 0x3175b066, 0x77964811, 0xe946e3f8, 0x539ed0ab, 0x98bad833, },
    { 0xa157d28d, 0xb066a73d, 0x48113175, 0xe3f87796, 0xd0abe946, 0xd833539e,
      0x240898ba, 0x71fc3bcb, 0x6855f4a3, 0x6c19a9cf, 0x92044c5d, 0xb8fe1de5, },
    { 0x2044c5d6, 0x8fe1de59, 0x42afa51b, 0x60cd4e7b, 0x902262eb, 0xc7f0ef2c,
      0xa157d28d, 0xb066a73d, 0x48113175, 0xe3f87796, 0xd0abe946, 0xd833539e, },
    { 0x5f4a371f, 0x9a9cf685, 0x44c5d6c1, 0xe1de5920, 0xafa51b8f, 0xcd4e7b42,
      0x2262eb60, 0xf0ef2c90, 0x57d28dc7, 0x66a73da1, 0x113175b0, 0xf8779648, },
};

u32 btle_crc_lut[256] = {
    0x000000, 0x01b4c0, 0x036980, 0x02dd40, 0x06d300, 0x0767c0, 0x05ba80, 0x040e40,
    0x0da600, 0x0c12c0, 0x0ecf80, 0x0f7b40, 0x0b7500, 0x0ac1c0, 0x081c80, 0x09a840,
    0x1b4c00, 0x1af8c0, 0x182580, 0x199140, 0x1d9f00, 0x1c2bc0, 0x1ef680, 0x1f4240,
    0x16ea00, 0x175ec0, 0x158380, 0x143740, 0x103900, 0x118dc0, 0x135080, 0x12e440,
    0x369800, 0x372cc0, 0x35f180, 0x344540, 0x304b00, 0x31ffc0, 0x332280, 0x329640,
    0x3b3e00, 0x3a8ac0, 0x385780, 0x39e340, 0x3ded00, 0x3c59c0, 0x3e8480, 0x3f3040,
    0x2dd400, 0x2c60c0, 0x2ebd80, 0x2f0940, 0x2b0700, 0x2ab3c0, 0x286e80, 0x29da40,
    0x207200, 0x21c6c0, 0x231b80, 0x22af40, 0x26a100, 0x2715c0, 0x25c880, 0x247c40,
    0x6d3000, 0x6c84c0, 0x6e5980, 0x6fed40, 0x6be300, 0x6a57c0, 0x688a80, 0x693e40,
    0x609600, 0x6122c0, 0x63ff80, 0x624b40, 0x664500, 0x67f1c0, 0x652c80, 0x649840,
    0x767c00, 0x77c8c0, 0x751580, 0x74a140, 0x70af00, 0x711bc0, 0x73c680, 0x727240,
    0x7bda00, 0x7a6ec0, 0x78b380, 0x790740, 0x7d0900, 0x7cbdc0, 0x7e6080, 0x7fd440,
    0x5ba800, 0x5a1cc0, 0x58c180, 0x597540, 0x5d7b00, 0x5ccfc0, 0x5e1280, 0x5fa640,
    0x560e00, 0x57bac0, 0x556780, 0x54d340, 0x50dd00, 0x5169c0, 0x53b480, 0x520040,
    0x40e400, 0x4150c0, 0x438d80, 0x423940, 0x463700, 0x4783c0, 0x455e80, 0x44ea40,
    0x4d4200, 0x4cf6c0, 0x4e2b80, 0x4f9f40, 0x4b9100, 0x4a25c0, 0x48f880, 0x494c40,
    0xda6000, 0xdbd4c0, 0xd90980, 0xd8bd40, 0xdcb300, 0xdd07c0, 0xdfda80, 0xde6e40,
    0xd7c600, 0xd672c0, 0xd4af80, 0xd51b40, 0xd11500, 0xd0a1c0, 0xd27c80, 0xd3c840,
    0xc12c00, 0xc098c0, 0xc24580, 0xc3f140, 0xc7ff00, 0xc64bc0, 0xc49680, 0xc52240,
    0xcc8a00, 0xcd3ec0, 0xcfe380, 0xce5740, 0xca5900, 0xcbedc0, 0xc93080, 0xc88440,
    0xecf800, 0xed4cc0, 0xef9180, 0xee2540, 0xea2b00, 0xeb9fc0, 0xe94280, 0xe8f640,
    0xe15e00, 0xe0eac0, 0xe23780, 0xe38340, 0xe78d00, 0xe639c0, 0xe4e480, 0xe55040,
    0xf7b400, 0xf600c0, 0xf4dd80, 0xf56940, 0xf16700, 0xf0d3c0, 0xf20e80, 0xf3ba40,
    0xfa1200, 0xfba6c0, 0xf97b80, 0xf8cf40, 0xfcc100, 0xfd75c0, 0xffa880, 0xfe1c40,
    0xb75000, 0xb6e4c0, 0xb43980, 0xb58d40, 0xb18300, 0xb037c0, 0xb2ea80, 0xb35e40,
    0xbaf600, 0xbb42c0, 0xb99f80, 0xb82b40, 0xbc2500, 0xbd91c0, 0xbf4c80, 0xbef840,
    0xac1c00, 0xada8c0, 0xaf7580, 0xaec140, 0xaacf00, 0xab7bc0, 0xa9a680, 0xa81240,
    0xa1ba00, 0xa00ec0, 0xa2d380, 0xa36740, 0xa76900, 0xa6ddc0, 0xa40080, 0xa5b440,
    0x81c800, 0x807cc0, 0x82a180, 0x831540, 0x871b00, 0x86afc0, 0x847280, 0x85c640,
    0x8c6e00, 0x8ddac0, 0x8f0780, 0x8eb340, 0x8abd00, 0x8b09c0, 0x89d480, 0x886040,
    0x9a8400, 0x9b30c0, 0x99ed80, 0x985940, 0x9c5700, 0x9de3c0, 0x9f3e80, 0x9e8a40,
    0x972200, 0x9696c0, 0x944b80, 0x95ff40, 0x91f100, 0x9045c0, 0x929880, 0x932c40
};

/*
 * Calculate a BTLE CRC one byte at a time. Thanks to Dominic Spill and
 * Michael Ossmann for writing and optimizing this.
 *
 * Arguments: CRCInit, pointer to start of packet, length of packet in
 * bytes
 * */
static u32 btle_crcgen_lut(u32 crc_init, u8 *data, int len)
{
    u32 state;
    int i;
    u8 key;

    state = crc_init & 0xffffff;
    for (i = 0; i < len; ++i) {
        key = data[i] ^ (state & 0xff);
        state = (state >> 8) ^ btle_crc_lut[key];
    }
    return state;
}

typedef enum {
    LINK_INACTIVE,
    LINK_LISTENING,
    LINK_CONN_PENDING,
    LINK_CONNECTED,
} link_state_t;

typedef struct _le_state_t {
    u32 access_address;         // Access Address to filter by
    u16 synch;                  // Access address in CC2400 syncword format
    u16 syncl;                  // lower 16 bits thereof
    u32 crc_init;               // CrcInit: used to calculate CRC
    u32 crc_init_reversed;      // bits-reversed version of the previous
    int crc_verify;             // true to reject packets with bad CRC

    link_state_t link_state;    // current link layer state

    u8 channel_idx;             // current channel index
    u8 channel_increment;       // amount to hop

    u32 conn_epoch;             // reference time for the start of the connection
    u16 volatile interval_timer;// number of intervals remaining before next hop
    u16 conn_interval;          // connection-specific hop interval
    u16 volatile conn_count;    // number of intervals since the start of the connection

    u8 win_size;                // window size (max packets per connection)
    u16 win_offset;             // offset of first window from start of connection

    int update_pending;         // whether a connection update is pending
    u16 update_instant;         // the connection count when the update takes effect
    u16 interval_update;        // the new hop_internal
    u8 win_size_update;         // the new window size
    u16 win_offset_update;      // the new window offset

    u8 target[6];               // target MAC for connection following (byte order reversed)
    int target_set;             // whether a target has been set (default: false)
    u32 last_packet;            // when was the last packet received
} le_state_t;

le_state_t le = {
    .access_address = 0x8e89bed6,           // advertising channel access address
    .synch = 0x6b7d,                        // bit-reversed adv channel AA
    .syncl = 0x9171,
    .crc_init  = 0x555555,                  // advertising channel CRCInit
    .crc_init_reversed = 0xAAAAAA,
    .crc_verify = 0,

    .link_state = LINK_INACTIVE,
    .conn_epoch = 0,
    .target_set = 0,
    .last_packet = 0,
};

u8 adv_ind[] = {
    // LL header
    0x00, 0x09,
    // advertising address
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
    // advertising data
    0xa2, 0xa1, 0xa5,
    // CRC (calc)
    0xff, 0xff, 0xff,
};

static void le_set_access_address(u32 aa)
{
    u32 aa_rev;

    le.access_address = aa;
    aa_rev = rbit(aa);
    le.syncl = aa_rev & 0xffff;
    le.synch = aa_rev >> 16;
}

/* reset le state, called by bt_generic_le and bt_follow_le() */
static void ble_reset(void)
{
    le_set_access_address(0x8e89bed6);     // 0x8e89bed6 advertising channel access address
    le.crc_init  = 0x555555;               // advertising channel CRCInit
    le.crc_init_reversed = 0xAAAAAA;
    le.crc_verify = 0;
    le.last_packet = 0;

    le.link_state = LINK_INACTIVE;

    le.channel_idx = 0;
    le.channel_increment = 0;

    le.conn_epoch = 0;
    le.interval_timer = 0;
    le.conn_interval = 0;
    le.conn_interval = 0;
    le.conn_count = 0;

    le.win_size = 0;
    le.win_offset = 0;

    le.update_pending = 0;
    le.update_instant = 0;
    le.interval_update = 0;
    le.win_size_update = 0;
    le.win_offset_update = 0;
}

void ble_follow(void);

//if (do_adv_index == 37)
//    channel = 2402;
//else if (do_adv_index == 38)
//    channel = 2426;
//else
//    channel = 2480;

static u8 btle_channel_index(u8 channel) {
    u8 idx;
    channel /= 2;
    if (channel == 0)
        idx = 37;
    else if (channel < 12)
        idx = channel - 1;
    else if (channel == 12)
        idx = 38;
    else if (channel < 39)
        idx = channel - 2;
    else
        idx = 39;
    return idx;
}

volatile u32 ble_packet_len;
volatile u32 clkn = 0;

static void packet_process(u8 *packet)
{
#define ADV_ADDRESS_IDX     0
#define HEADER_IDX          4
#define DATA_LEN_IDX        5
#define DATA_START_IDX      6

    //u8 header = packet[HEADER_IDX];
    u8 *data_len = &packet[DATA_LEN_IDX];
    //u8 *data = &packet[DATA_START_IDX];

    //printf("pkt %x %d\n", packet[4], packet[DATA_LEN_IDX] & 0x3f);

    if (le.link_state == LINK_LISTENING) {
        u8 pkt_type = packet[4] & 0x0F;
        if (pkt_type == 0x05) {
            uint16_t conn_interval;

            // ignore packets with incorrect length
            if (*data_len != 34)
                return;

            // conn interval must be [7.5 ms, 4.0s] in units of 1.25 ms
            conn_interval = (packet[29] << 8) | packet[28];
            if (conn_interval < 6 || conn_interval > 3200)
                return;

            // This is a connect packet
            // if we have a target, see if InitA or AdvA matches
            if (le.target_set &&
                memcmp(le.target, &packet[6], 6) &&  // Target address doesn't match Initiator.
                memcmp(le.target, &packet[12], 6)) {  // Target address doesn't match Advertiser.
                return;
            }

            le.link_state = LINK_CONN_PENDING;
            le.crc_verify = 0; // we will drop many packets if we attempt to filter by CRC

            int i = 0;
            u32 tmp = 0;

            for (i = 0; i < 4; ++i)
                tmp |= packet[18+i] << (i*8);

            le_set_access_address(tmp);

#define CRC_INIT (2+4+6+6+4)
            le.crc_init = (packet[CRC_INIT+2] << 16)
                        | (packet[CRC_INIT+1] << 8)
                        |  packet[CRC_INIT+0];
            le.crc_init_reversed = rbit(le.crc_init);

#define WIN_SIZE (2+4+6+6+4+3)
            le.win_size = packet[WIN_SIZE];

#define WIN_OFFSET (2+4+6+6+4+3+1)
            le.win_offset = packet[WIN_OFFSET];

#define CONN_INTERVAL (2+4+6+6+4+3+1+2)
            le.conn_interval = (packet[CONN_INTERVAL+1] << 8)
                             |  packet[CONN_INTERVAL+0];

#define CHANNEL_INC (2+4+6+6+4+3+1+2+2+2+2+5)
            le.channel_increment = packet[CHANNEL_INC] & 0x1f;
            le.channel_idx = le.channel_increment;

            // Hop to the initial channel immediately
            //do_hop = 1;
        }
    }
}

void ble_process(void)
{
    volatile u16 channel = 2426;

    static u32 count = 100;

    if (clkn == count) {
        count = clkn + 100;
    }

    if (ble_packet_len < 4) {
        return;
    }

    le.link_state = LINK_LISTENING;

    u32 packet[48/4+1] = { 0, };
    u8 *p = (u8 *)packet;
    u32 *rxp = (u32 *)rxbuf1;

    packet[0] = le.access_address;

    const u32 *whit = whitening_word[btle_channel_index(channel-2402)];
    for (int i = 0; i < 1; i+= 1) {
        //u32 v = rxbuf1[i+0] << 24
        //        | rxbuf1[i+1] << 16
        //        | rxbuf1[i+2] << 8
        //        | rxbuf1[i+3] << 0;
        u32 v = rev(rxp[i]);
        packet[i+1] = rbit(v) ^ whit[i];
    }

    // Preamble     Access Address  PDU                 CRC
    // (1 octet)    (4 octets)      (2 to 39 octets)    (3 octets)
    u32 len = (p[5] & 0x3f) + 2;    // PDU length

    printf("len %d %d %d\n", ble_packet_len, p[5] & 0x3f, len);

    // transfer the minimum number of bytes from the CC2400
    // this allows us enough time to resume RX for subsequent packets on the same channel

    while (ble_packet_len < (len+4+3)); // pdu + access addr + crc

    //cc_SRFoff();
    //cc_clean_fifo();
    //spi_disable(SPI3);

    spi_set_nss_high(SPI3);
    cx_strobe(SFSON);                   // goto FS_ON

    // unwhiten the rest of the packet
    for (int i = 1; i < 11; i += 1) {
        //uint32_t v = rxbuf1[i+0] << 24
        //           | rxbuf1[i+1] << 16
        //           | rxbuf1[i+2] << 8
        //           | rxbuf1[i+3] << 0;
        u32 v = rev(rxp[i]);
        packet[i+1] = rbit(v) ^ whit[i];
    }

    if (le.crc_verify) {
        u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, p + 4, len);
        u32 wire_crc = (p[4+len+2] << 16)
                     | (p[4+len+1] << 8)
                     | (p[4+len+0] << 0);
        //printf("crc ? %x %x\n", calc_crc, wire_crc);
        if (calc_crc != wire_crc) { // skip packets with a bad CRC
            //printf("crc err %x %x\n", calc_crc, wire_crc);
            goto CLEFIFO;
        }
    }

    for (u32 i= 0; i < (len + 4 + 3); i++) {
        printf("%x ", p[i]);
    }
    kputc('\n');

    //packet_process((u8 *)packet);

CLEFIFO:
    ble_packet_len = 0;
    cc_clean_fifo();
    spi_set_nss_low(SPI3);
    cx_strobe(SRX);
}

void ble_init(void)
{
    volatile u16 channel = 2426;

    ble_packet_len = 0;

    ble_reset();

    rf_init(MOD_BT_LOW_ENERGY, rbit(le.access_address), channel);

    kputs("bleX\n");

    cc_clean_fifo();

    spi_enable(SPI3);

    cx_strobe(SRX);

    spi_set_nss_low(SPI3);

#if 0
    while (0) {
        const u32 *whit = whitening_word[btle_channel_index(channel-2402)];
        u32 packet[48/4+1] = { 0, };
        u8 *p = (u8 *)packet;

        if (ble_packet_len > 16) {
            cx_strobe(SFSON);
            cc_clean_fifo();
            ble_packet_len = 0;
            spi_set_nss_high(SPI3);

            // unwhiten the rest of the packet
            for (int i = 0; i < 16; i += 4) {
                uint32_t v = rxbuf1[i+0] << 24
                           | rxbuf1[i+1] << 16
                           | rxbuf1[i+2] << 8
                           | rxbuf1[i+3] << 0;
                packet[i/4] = rbit(v) ^ whit[i/4];
            }

            kputc(':');
            for (int i= 0; i < 16; i++) {
                kputhex(p[i], 2);
            }
            kputc('\n');

            cx_strobe(SRX);
            spi_set_nss_low(SPI3);
        }
    }
#endif
}


