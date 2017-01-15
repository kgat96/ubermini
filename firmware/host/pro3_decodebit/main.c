
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>

#include <signal.h>

#include <libusb-1.0/libusb.h>

#include <btbb.h>



/* RX USB packet parameters */
#define PKT_LEN       64
#define SYM_LEN       50
#define BANK_LEN      (SYM_LEN * 8)

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

    char buffer[SYM_LEN];

    char syms[BANK_LEN];

    btbb_packet* pkt = NULL;

    int offset;
    int fd;

    btbb_init(2);
    btbb_init_survey();

    /* Create input file descriptor */
    fd = open(argv[1], O_RDONLY);

    if (fd < 0) {
        printf("open %s error\n", argv[1]);
        return 1;
    }

    while (read(fd, buffer, SYM_LEN) == SYM_LEN) {
        if (find_giac(buffer))
            printf("find giac ...\n");

        unpack_symbols(buffer, syms);
        offset = btbb_find_ac(syms, BANK_LEN - 64, LAP_ANY, 2, &pkt);
        if (offset >= 0)
            printf("find accccccc ..%d ..!\n", offset);
    }

    printf("done\n");

    /* Close file descriptors */
    close (fd);

    return 0;
}



















