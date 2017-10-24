/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2017 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "config.h"
#include "cc.h"
#include "uart.h"

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

    link_state_t link_state;         // current link layer state

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

/* efficiently reverse the bits of a 32-bit word */
static u32 rbit(u32 value)
{
  u32 result = 0;
  __asm("rbit %0, %1" : "=r" (result) : "r" (value));
  return result;
}

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
    le_set_access_address(0x8e89bed6);     // advertising channel access address
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

void ble_follow(void)
{
    ble_reset();

    cc_rx_sync(MOD_BT_LOW_ENERGY, rbit(le.access_address), );



}






























