
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <signal.h>

//#include <libusb-1.0/libusb.h>

//#include <btbb.h>

#include "uthash.h"
#include "check_tables.h"

#define BTBB_WHITENED    0
#define BTBB_NAP_VALID   1
#define BTBB_UAP_VALID   2
#define BTBB_LAP_VALID   3
#define BTBB_CLK6_VALID  4
#define BTBB_CLK27_VALID 5
#define BTBB_CRC_CORRECT 6
#define BTBB_HAS_PAYLOAD 7
#define BTBB_IS_EDR      8

#define BTBB_HOP_REVERSAL_INIT 9
#define BTBB_GOT_FIRST_PACKET  10
#define BTBB_IS_AFH            11
#define BTBB_LOOKS_LIKE_AFH    12
#define BTBB_IS_ALIASED        13
#define BTBB_FOLLOWING         14

/* Payload modulation */
#define BTBB_MOD_GFSK              0x00
#define BTBB_MOD_PI_OVER_2_DQPSK   0x01
#define BTBB_MOD_8DPSK             0x02

/* Transport types */
#define BTBB_TRANSPORT_ANY     0x00
#define BTBB_TRANSPORT_SCO     0x01
#define BTBB_TRANSPORT_ESCO    0x02
#define BTBB_TRANSPORT_ACL     0x03
#define BTBB_TRANSPORT_CSB     0x04

/* RX USB packet parameters */
#define PKT_LEN       64
#define SYM_LEN       50
#define BANK_LEN      (SYM_LEN * 8)

/* maximum number of symbols */
#define MAX_SYMBOLS 3125

/* maximum number of payload bits */
#define MAX_PAYLOAD_LENGTH 2744

/* Maximum number of AC errors supported by library. Caller may
 * specify any value <= AC_ERROR_LIMIT in btbb_init(). */
#define AC_ERROR_LIMIT 5

/* maximum number of bit errors for known syncwords */
#define MAX_SYNCWORD_ERRS 5

/* maximum number of bit errors in  */
#define MAX_BARKER_ERRORS 1

/* default codeword modified for PN sequence and barker code */
#define DEFAULT_CODEWORD 0xb0000002c7820e7eULL

/* Default access code, used for calculating syndromes */
#define DEFAULT_AC 0xcc7b7268ff614e1bULL

#define LAP_ANY 0xffffffffUL
#define UAP_ANY 0xff

static const uint64_t pn = 0x83848D96BBCC54FCULL;

typedef struct {
    uint64_t syndrome; /* key */
    uint64_t error;
    UT_hash_handle hh;
} syndrome_struct;

static syndrome_struct *syndrome_map = NULL;

struct btbb_packet {

    uint32_t refcount;
    uint32_t flags;

    uint8_t channel; /* Bluetooth channel (0-79) */
    uint8_t UAP; /* upper address part */
    uint16_t NAP; /* non-significant address part */
    uint32_t LAP; /* lower address part found in access code */

    uint8_t modulation;
    uint8_t transport;
    uint8_t packet_type;
    uint8_t packet_lt_addr; /* LLID field of payload header (2 bits) */
    uint8_t packet_flags; /* Flags - FLOW/ARQN/SQEN */
    uint8_t packet_hec; /* Flags - FLOW/ARQN/SQEN */

    /* packet header, one bit per char */
    char packet_header[18];

    /* number of payload header bytes: 0, 1, 2, or -1 for
     * unknown. payload is one bit per char. */
    int payload_header_length;
    char payload_header[16];

    /* LLID field of payload header (2 bits) */
    uint8_t payload_llid;

    /* flow field of payload header (1 bit) */
    uint8_t payload_flow;

    /* payload length: the total length of the asynchronous data
     * in bytes.  This does not include the length of synchronous
     * data, such as the voice field of a DV packet.  If there is a
     * payload header, this payload length is payload body length
     * (the length indicated in the payload header's length field)
     * plus payload_header_length plus 2 bytes CRC (if present).
     */
    int payload_length;

    /* The actual payload data in host format
     * Ready for passing to wireshark
     * 2744 is the maximum length, but most packets are shorter.
     * Dynamic allocation would probably be better in the long run but is
     * problematic in the short run.
     */
    char payload[MAX_PAYLOAD_LENGTH];

    uint16_t crc;
    uint32_t clock; /* CLK1-27 of master */
    uint32_t clkn; /* native (local) clock, CLK0-27 */
    uint8_t ac_errors; /* Number of bit errors in the AC */

    /* the raw symbol stream (less the preamble), one bit per char */
    //FIXME maybe this should be a vector so we can grow it only
    //to the size needed and later shrink it if we find we have
    //more symbols than necessary
    uint16_t length; /* number of symbols */
    char symbols[MAX_SYMBOLS];

};

typedef struct btbb_packet btbb_packet;

btbb_packet * btbb_packet_new(void)
{
    btbb_packet *pkt = (btbb_packet *) calloc(1, sizeof(btbb_packet));
    if (pkt)
        pkt->refcount = 1;
    else
        fprintf(stderr, "Unable to allocate packet");
    return pkt;
}

void btbb_packet_set_flag(btbb_packet *pkt, int flag, int val)
{
    uint32_t mask = 1L << flag;
    pkt->flags &= ~mask;
    if (val)
        pkt->flags |= mask;
}

static void init_packet(btbb_packet *pkt, uint32_t lap, uint8_t ac_errors)
{
    pkt->LAP = lap;
    pkt->ac_errors = ac_errors;

    pkt->flags = 0;
    btbb_packet_set_flag(pkt, BTBB_WHITENED, 1);
}

static void add_syndrome(uint64_t syndrome, uint64_t error)
{
    syndrome_struct *s;
    s = malloc(sizeof(syndrome_struct));
    s->syndrome = syndrome;
    s->error = error;

    HASH_ADD(hh, syndrome_map, syndrome, 8, s);
}

void unpack_symbols(const uint8_t* buf, char* unpacked)
{
    int i, j;

    for (i = 0; i < SYM_LEN; i++) {
        /* output one byte for each received symbol (0x00 or 0x01) */
        for (j = 0; j < 8; j++) {
            unpacked[i * 8 + j] = ((buf[i] << j) & 0x80) >> 7;
        }
    }
}

static syndrome_struct *find_syndrome(uint64_t syndrome)
{
    syndrome_struct *s;

    HASH_FIND(hh, syndrome_map, &syndrome, 8, s);
    return s;
}

static uint64_t gen_syndrome(uint64_t codeword)
{
    uint64_t syndrome = codeword & 0xffffffff;
    codeword >>= 32;
    syndrome ^= sw_check_table4[codeword & 0xff];
    codeword >>= 8;
    syndrome ^= sw_check_table5[codeword & 0xff];
    codeword >>= 8;
    syndrome ^= sw_check_table6[codeword & 0xff];
    codeword >>= 8;
    syndrome ^= sw_check_table7[codeword & 0xff];
    return syndrome;
}

static void cycle(uint64_t error, int start, int depth, uint64_t codeword)
{
    uint64_t new_error, syndrome, base;
    int i;
    base = 1;
    depth -= 1;
    for (i = start; i < 58; i++) {
        new_error = (base << i);
        new_error |= error;
        if (depth)
            cycle(new_error, i + 1, depth, codeword);
        else {
            syndrome = gen_syndrome(codeword ^ new_error);
            add_syndrome(syndrome, new_error);
        }
    }
}

static void gen_syndrome_map(int bit_errors)
{
    int i;
    for (i = 1; i <= bit_errors; i++)
        cycle(0, 0, i, DEFAULT_AC);
}

int btbb_init(int max_ac_errors)
{
    /* Sanity check max_ac_errors. */
    if ((max_ac_errors < 0) || (max_ac_errors > AC_ERROR_LIMIT)) {
        fprintf(stderr, "%s: max_ac_errors out of range\n", __FUNCTION__);
        return -1;
    }

    if ((syndrome_map == NULL) && (max_ac_errors))
        gen_syndrome_map(max_ac_errors);

    return 0;
}

/* Convert some number of bits of an air order array to a host order integer */
static uint8_t air_to_host8(const char *air_order, const int bits)
{
    int i;
    uint8_t host_order = 0;
    for (i = 0; i < bits; i++)
        host_order |= ((uint8_t) air_order[i] << i);
    return host_order;
}

static uint16_t air_to_host16(const char *air_order, const int bits)
{
    int i;
    uint16_t host_order = 0;
    for (i = 0; i < bits; i++)
        host_order |= ((uint16_t) air_order[i] << i);
    return host_order;
}

static uint32_t air_to_host32(const char *air_order, const int bits)
{
    int i;
    uint32_t host_order = 0;
    for (i = 0; i < bits; i++)
        host_order |= ((uint32_t) air_order[i] << i);
    return host_order;
}

static uint64_t air_to_host64(const char *air_order, const int bits)
{
    int i;
    uint64_t host_order = 0;
    for (i = 0; i < bits; i++)
        host_order |= ((uint64_t) air_order[i] << i);
    return host_order;
}

/* count the number of 1 bits in a uint64_t */
static uint8_t count_bits(uint64_t n)
{
#ifdef __GNUC__
	return (uint8_t) __builtin_popcountll (n);
#else
	uint8_t i = 0;
	for (i = 0; n != 0; i++)
		n &= n - 1;
	return i;
#endif
}


/* lookup table for barker code hamming distance */
static const uint8_t BARKER_DISTANCE[] = {
	3,3,3,2,3,2,2,1,2,3,3,3,3,3,3,2,2,3,3,3,3,3,3,2,1,2,2,3,2,3,3,3,
	3,2,2,1,2,1,1,0,3,3,3,2,3,2,2,1,3,3,3,2,3,2,2,1,2,3,3,3,3,3,3,2,
	2,3,3,3,3,3,3,2,1,2,2,3,2,3,3,3,1,2,2,3,2,3,3,3,0,1,1,2,1,2,2,3,
	3,3,3,2,3,2,2,1,2,3,3,3,3,3,3,2,2,3,3,3,3,3,3,2,1,2,2,3,2,3,3,3};

int promiscuous_packet_search(char *stream, int search_length, uint32_t *lap,
        int max_ac_errors, uint8_t *ac_errors)
{
    uint64_t syncword, codeword, syndrome, corrected_barker;
    syndrome_struct *errors;
    char *symbols;
    int count, offset = -1;

    /* Barker code at end of sync word (includes
     * MSB of LAP) is used as a rough filter.
     */
    // 57 + 6 = 64 sync word is a 64-bit code
    uint8_t barker = air_to_host8(&stream[57], 6);
    barker <<= 1;

    for (count = 0; count < search_length; count++) {
        symbols = &stream[count];
        barker >>= 1;
        barker |= (symbols[63] << 6);
        if (BARKER_DISTANCE[barker] <= MAX_BARKER_ERRORS) {
            // Error correction
            syncword = air_to_host64(symbols, 64);

            /* correct the barker code with a simple comparison */
            corrected_barker = barker_correct[(uint8_t) (syncword >> 57)];
            syncword = (syncword & 0x01ffffffffffffffULL) | corrected_barker;

            codeword = syncword ^ pn;

            /* Zero syndrome -> good codeword. */
            syndrome = gen_syndrome(codeword);
            *ac_errors = 0;

            /* Try to fix errors in bad codeword. */
            if (syndrome) {
                errors = find_syndrome(syndrome);
                if (errors != NULL) {
                    syncword ^= errors->error;
                    *ac_errors = count_bits(errors->error);
                    syndrome = 0;
                } else {
                    *ac_errors = 0xff;  // fail
                }
            }

            if (*ac_errors <= max_ac_errors) {
                *lap = (syncword >> 34) & 0xffffff;
                offset = count;
                break;
            }
        }
    }
    return offset;
}

/* Looks for an AC in the stream */
int btbb_find_ac(char *stream, int search_length, uint32_t lap,
        int max_ac_errors, btbb_packet **pkt_ptr)
{
    int offset;
    uint8_t ac_errors;

    /* Matching any LAP */
    if (lap == LAP_ANY)
        offset = promiscuous_packet_search(stream, search_length, &lap, max_ac_errors, &ac_errors);
    //else
    //	offset = find_known_lap(stream, search_length, lap,
    //                      max_ac_errors, &ac_errors);

    if (offset >= 0) {
        if (*pkt_ptr == NULL)
            *pkt_ptr = btbb_packet_new();
        init_packet(*pkt_ptr, lap, ac_errors);
    }

    return offset;
}


/* an ugly but effective way to identify a GIAC (inquiry packet) */
static int find_giac(uint8_t *buf)
{
    int i, j;
    const uint8_t giac[8][7] = {
            {0x47, 0x5c, 0x58, 0xcc, 0x73, 0x34, 0x5e},
            {0x8e, 0xb8, 0xb1, 0x98, 0xe6, 0x68, 0xbc},
            {0x11, 0xd7, 0x16, 0x33, 0x1c, 0xcd, 0x17},
            {0x23, 0xae, 0x2c, 0x66, 0x39, 0x9a, 0x2f},
            {0x75, 0xc5, 0x8c, 0xc7, 0x33, 0x45, 0xe7},
            {0xeb, 0x8b, 0x19, 0x8e, 0x66, 0x8b, 0xce},
            {0x1d, 0x71, 0x63, 0x31, 0xcc, 0xd1, 0x79},
            {0x3a, 0xe2, 0xc6, 0x63, 0x99, 0xa2, 0xf3}};

    for (i = 0; i < (SYM_LEN - 6); i++)
            for (j = 0; j < 8; j++)
                if (buf[i] == giac[j][0]
                        && buf[i + 1] == giac[j][1]
                        && buf[i + 2] == giac[j][2]
                        && buf[i + 3] == giac[j][3]
                        && buf[i + 4] == giac[j][4]
                        && buf[i + 5] == giac[j][5]
                        && buf[i + 6] == giac[j][6])
                    return 1;

    return 0;
}

int main(int argc, char *argv[])
{

    uint8_t buffer[SYM_LEN];

    char syms[BANK_LEN];

    btbb_packet* pkt = NULL;

    int offset;
    FILE *fd;

    btbb_init(2);
    //btbb_init_survey();

    /* Create input file descriptor */
    //fd = open(argv[1], O_RDONLY);
    fd = fopen(argv[1], "rb");

    if (fd < 0) {
        printf("open %s error\n", argv[1]);
        return 1;
    }

    printf("open %s done\n", argv[1]);

    static int cc = 0;

    //while (read(fd, buffer, SYM_LEN)) {
    while (fread(buffer, 1, SYM_LEN, fd) == SYM_LEN) {

        cc += SYM_LEN;

        //printf("read %d byte\n", cc);

        if (0) {
            int i;
            for (i = 0; i < 50; i++)
                printf("%2.2X", buffer[i]);
        }

        if (find_giac(buffer))
            printf("find giac ...\n");

        unpack_symbols(buffer, syms);
        offset = btbb_find_ac(syms, BANK_LEN - 64, LAP_ANY, 2, &pkt);
        if (offset >= 0)
            printf("find accccccc ..%d ..!\n", offset);

    }

    printf("done\n");

    /* Close file descriptors */
    fclose(fd);

    return 0;
}



















