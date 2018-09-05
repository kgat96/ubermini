#include <stdio.h>

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef unsigned long int u64;

// CRC /////////////////////////////////////////////////////////////////////

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

static u32 btle_calc_crc(u32 crc_init, u8 *data, int len)
{
    u32 state = crc_init & 0xffffff;
    u32 lfsr_mask = 0x5a6000; // 010110100110000000000000
    int i, j;

	#if 0
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
	#endif
	
	for (i = 0; i < len; ++i) {
        u8 cur = data[i] ^ state;
        for (j = 0; j < 8; ++j) {
            int next_bit = cur & 1;
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

// whiten /////////////////////////////////////////////////////////////////////

void dump_whiten_bit(u8 idx)
{
    u8 pos = idx | 0x40;
    for (int i = 0; i < 1000; i++) {
        if (((idx | 0x40) == pos) && i) return;
        printf("%x[%3d/%3d],\n", pos & 1, pos, i);
        //printf("%x,", pos & 1, pos);
        if (pos & 1) {
            pos ^= 0x8;
            pos |= 0x80;
        }
        pos >>= 1;
    }
}

extern unsigned char txt[1056];

int main()
{
	printf("%x %x\n", btle_calc_crc(0x555555, txt, 1000), btle_crcgen_lut(0x555555, txt, 1000));
    
    dump_whiten_bit(0);
}



unsigned char txt[1056] = {
	0x65, 0x20, 0x2F, 0x4D, 0x65, 0x74, 0x61, 0x64, 0x61, 0x74, 0x61, 0x0A, 0x2F, 0x4C, 0x65, 0x6E, 
	0x67, 0x74, 0x68, 0x20, 0x33, 0x30, 0x39, 0x36, 0x0A, 0x2F, 0x53, 0x75, 0x62, 0x74, 0x79, 0x70, 
	0x65, 0x20, 0x2F, 0x58, 0x4D, 0x4C, 0x0A, 0x3E, 0x3E, 0x0A, 0x73, 0x74, 0x72, 0x65, 0x61, 0x6D, 
	0x0A, 0x3C, 0x3F, 0x78, 0x70, 0x61, 0x63, 0x6B, 0x65, 0x74, 0x20, 0x62, 0x65, 0x67, 0x69, 0x6E, 
	0x3D, 0x22, 0xEF, 0xBB, 0xBF, 0x22, 0x20, 0x69, 0x64, 0x3D, 0x22, 0x57, 0x35, 0x4D, 0x30, 0x4D, 
	0x70, 0x43, 0x65, 0x68, 0x69, 0x48, 0x7A, 0x72, 0x65, 0x53, 0x7A, 0x4E, 0x54, 0x63, 0x7A, 0x6B, 
	0x63, 0x39, 0x64, 0x22, 0x3F, 0x3E, 0x0A, 0x3C, 0x78, 0x3A, 0x78, 0x6D, 0x70, 0x6D, 0x65, 0x74, 
	0x61, 0x20, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x78, 0x3D, 0x22, 0x61, 0x64, 0x6F, 0x62, 0x65, 
	0x3A, 0x6E, 0x73, 0x3A, 0x6D, 0x65, 0x74, 0x61, 0x2F, 0x22, 0x20, 0x78, 0x3A, 0x78, 0x6D, 0x70, 
	0x74, 0x6B, 0x3D, 0x22, 0x58, 0x4D, 0x50, 0x20, 0x43, 0x6F, 0x72, 0x65, 0x20, 0x34, 0x2E, 0x31, 
	0x2E, 0x31, 0x22, 0x3E, 0x0A, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x52, 0x44, 0x46, 0x20, 0x78, 
	0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x72, 0x64, 0x66, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 
	0x2F, 0x77, 0x77, 0x77, 0x2E, 0x77, 0x33, 0x2E, 0x6F, 0x72, 0x67, 0x2F, 0x31, 0x39, 0x39, 0x39, 
	0x2F, 0x30, 0x32, 0x2F, 0x32, 0x32, 0x2D, 0x72, 0x64, 0x66, 0x2D, 0x73, 0x79, 0x6E, 0x74, 0x61, 
	0x78, 0x2D, 0x6E, 0x73, 0x23, 0x22, 0x3E, 0x0A, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x44, 
	0x65, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x72, 0x64, 0x66, 0x3A, 0x61, 
	0x62, 0x6F, 0x75, 0x74, 0x3D, 0x22, 0x22, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x78, 0x6D, 0x6C, 0x6E, 
	0x73, 0x3A, 0x70, 0x64, 0x66, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x6E, 0x73, 
	0x2E, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x70, 0x64, 0x66, 0x2F, 0x31, 
	0x2E, 0x33, 0x2F, 0x22, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x70, 0x64, 0x66, 0x3A, 0x50, 0x72, 
	0x6F, 0x64, 0x75, 0x63, 0x65, 0x72, 0x3E, 0x41, 0x63, 0x72, 0x6F, 0x62, 0x61, 0x74, 0x20, 0x44, 
	0x69, 0x73, 0x74, 0x69, 0x6C, 0x6C, 0x65, 0x72, 0x20, 0x35, 0x2E, 0x30, 0x2E, 0x35, 0x20, 0x28, 
	0x57, 0x69, 0x6E, 0x64, 0x6F, 0x77, 0x73, 0x29, 0x3C, 0x2F, 0x70, 0x64, 0x66, 0x3A, 0x50, 0x72, 
	0x6F, 0x64, 0x75, 0x63, 0x65, 0x72, 0x3E, 0x0A, 0x09, 0x09, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 
	0x44, 0x65, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x09, 0x09, 0x3C, 
	0x72, 0x64, 0x66, 0x3A, 0x44, 0x65, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 
	0x72, 0x64, 0x66, 0x3A, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x3D, 0x22, 0x22, 0x0A, 0x09, 0x09, 0x09, 
	0x09, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x78, 0x61, 0x70, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 
	0x3A, 0x2F, 0x2F, 0x6E, 0x73, 0x2E, 0x61, 0x64, 0x6F, 0x62, 0x65, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 
	0x78, 0x61, 0x70, 0x2F, 0x31, 0x2E, 0x30, 0x2F, 0x22, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x78, 
	0x61, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61, 0x74, 0x65, 0x44, 0x61, 0x74, 0x65, 0x3E, 0x32, 0x30, 
	0x31, 0x34, 0x2D, 0x30, 0x34, 0x2D, 0x32, 0x34, 0x54, 0x31, 0x33, 0x3A, 0x34, 0x35, 0x3A, 0x32, 
	0x33, 0x5A, 0x3C, 0x2F, 0x78, 0x61, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61, 0x74, 0x65, 0x44, 0x61, 
	0x74, 0x65, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x78, 0x61, 0x70, 0x3A, 0x4D, 0x6F, 0x64, 0x69, 
	0x66, 0x79, 0x44, 0x61, 0x74, 0x65, 0x3E, 0x32, 0x30, 0x31, 0x38, 0x2D, 0x30, 0x33, 0x2D, 0x32, 
	0x32, 0x54, 0x31, 0x35, 0x3A, 0x33, 0x31, 0x3A, 0x34, 0x30, 0x2B, 0x30, 0x38, 0x3A, 0x30, 0x30, 
	0x3C, 0x2F, 0x78, 0x61, 0x70, 0x3A, 0x4D, 0x6F, 0x64, 0x69, 0x66, 0x79, 0x44, 0x61, 0x74, 0x65, 
	0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x78, 0x61, 0x70, 0x3A, 0x4D, 0x65, 0x74, 0x61, 0x64, 0x61, 
	0x74, 0x61, 0x44, 0x61, 0x74, 0x65, 0x3E, 0x32, 0x30, 0x31, 0x34, 0x2D, 0x30, 0x34, 0x2D, 0x32, 
	0x34, 0x54, 0x31, 0x33, 0x3A, 0x34, 0x35, 0x3A, 0x32, 0x33, 0x5A, 0x3C, 0x2F, 0x78, 0x61, 0x70, 
	0x3A, 0x4D, 0x65, 0x74, 0x61, 0x64, 0x61, 0x74, 0x61, 0x44, 0x61, 0x74, 0x65, 0x3E, 0x0A, 0x09, 
	0x09, 0x09, 0x3C, 0x78, 0x61, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61, 0x74, 0x6F, 0x72, 0x54, 0x6F, 
	0x6F, 0x6C, 0x3E, 0x50, 0x53, 0x63, 0x72, 0x69, 0x70, 0x74, 0x35, 0x2E, 0x64, 0x6C, 0x6C, 0x20, 
	0x56, 0x65, 0x72, 0x73, 0x69, 0x6F, 0x6E, 0x20, 0x35, 0x2E, 0x32, 0x2E, 0x32, 0x3C, 0x2F, 0x78, 
	0x61, 0x70, 0x3A, 0x43, 0x72, 0x65, 0x61, 0x74, 0x6F, 0x72, 0x54, 0x6F, 0x6F, 0x6C, 0x3E, 0x0A, 
	0x09, 0x09, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x44, 0x65, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 
	0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x44, 0x65, 0x73, 0x63, 
	0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x72, 0x64, 0x66, 0x3A, 0x61, 0x62, 0x6F, 0x75, 
	0x74, 0x3D, 0x22, 0x22, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x78, 0x6D, 0x6C, 0x6E, 0x73, 0x3A, 0x64, 
	0x63, 0x3D, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x70, 0x75, 0x72, 0x6C, 0x2E, 0x6F, 
	0x72, 0x67, 0x2F, 0x64, 0x63, 0x2F, 0x65, 0x6C, 0x65, 0x6D, 0x65, 0x6E, 0x74, 0x73, 0x2F, 0x31, 
	0x2E, 0x31, 0x2F, 0x22, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x64, 0x63, 0x3A, 0x63, 0x72, 0x65, 
	0x61, 0x74, 0x6F, 0x72, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x53, 
	0x65, 0x71, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x6C, 0x69, 
	0x3E, 0x61, 0x30, 0x31, 0x39, 0x32, 0x39, 0x30, 0x31, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x6C, 
	0x69, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x53, 0x65, 0x71, 
	0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x2F, 0x64, 0x63, 0x3A, 0x63, 0x72, 0x65, 0x61, 0x74, 0x6F, 
	0x72, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x64, 0x63, 0x3A, 0x74, 0x69, 0x74, 0x6C, 0x65, 0x3E, 
	0x0A, 0x09, 0x09, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x41, 0x6C, 0x74, 0x3E, 0x0A, 0x09, 
	0x09, 0x09, 0x09, 0x09, 0x3C, 0x72, 0x64, 0x66, 0x3A, 0x6C, 0x69, 0x20, 0x78, 0x6D, 0x6C, 0x3A, 
	0x6C, 0x61, 0x6E, 0x67, 0x3D, 0x22, 0x78, 0x2D, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6C, 0x74, 0x22, 
	0x3E, 0x42, 0x45, 0x41, 0x47, 0x4C, 0x45, 0x42, 0x4F, 0x4E, 0x45, 0x42, 0x4C, 0x4B, 0x5F, 0x43, 
	0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 0x6C, 0x69, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x09, 0x3C, 0x2F, 
	0x72, 0x64, 0x66, 0x3A, 0x41, 0x6C, 0x74, 0x3E, 0x0A, 0x09, 0x09, 0x09, 0x3C, 0x2F, 0x64, 0x63, 
	0x3A, 0x74, 0x69, 0x74, 0x6C, 0x65, 0x3E, 0x0A, 0x09, 0x09, 0x3C, 0x2F, 0x72, 0x64, 0x66, 0x3A, 
	0x44, 0x65, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x69, 0x6F, 0x6E, 0x3E, 0x0A, 0x09, 0x3C, 0x2F
};







