/*
 * This file is part of the ubermini project.
 *
 * Copyright (C) 2017 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __BLE_H
#define __BLE_H

/* operating modes */
enum operating_modes {
    MODE_IDLE          = 0,
    MODE_RX_SYMBOLS    = 1,
    MODE_TX_SYMBOLS    = 2,
    MODE_TX_TEST       = 3,
    MODE_SPECAN        = 4,
    MODE_RANGE_TEST    = 5,
    MODE_REPEATER      = 6,
    MODE_LED_SPECAN    = 7,
    MODE_BT_FOLLOW     = 8,
    MODE_BT_FOLLOW_LE  = 9,
    MODE_BT_PROMISC_LE = 10,
    MODE_RESET         = 11,
    MODE_BT_SLAVE_LE   = 12,
    MODE_EGO           = 13,
    MODE_AFH           = 14,
    MODE_RX_GENERIC    = 15,
    MODE_TX_GENERIC    = 16,
};

enum modulations {
    MOD_BT_BASIC_RATE = 0,
    MOD_BT_LOW_ENERGY = 1,
    MOD_80211_FHSS    = 2,
    MOD_NONE          = 3
};

enum hop_mode {
    HOP_NONE      = 0,
    HOP_SWEEP     = 1,
    HOP_BLUETOOTH = 2,
    HOP_BTLE      = 3,
    HOP_DIRECT    = 4,
    HOP_AFH       = 5,
};

extern volatile u32 ble_packet_len;
extern volatile u8 ble_rxpacket[1024];
extern volatile u32 clk3125n;

extern volatile u32 clkn;

void ble_init(void);
void ble_process(void);

/* efficiently reverse the bits of a 32-bit word */
__inline static u32 rbit(u32 value)
{
    u32 result = 0;
    __asm("rbit %0, %1" : "=r" (result) : "r" (value));
    return result;
}

__inline static u32 rev(u32 value)
{
    u32 result = 0;
    __asm("rev %0, %1" : "=r" (result) : "r" (value));
    return result;
}


#endif
