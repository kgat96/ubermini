
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <signal.h>

//#include <libusb-1.0/libusb.h>

#include <btbb.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>


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

//typedef struct {
//    uint8_t b[6];
//} __packed bdaddr_t;

#if 0
void extra_info(int dd, int dev_id, bdaddr_t* bdaddr)
{
	uint16_t handle, offset;
	uint8_t features[8], max_page = 0;
	char name[249], *tmp;
	char addr[19] = { 0 };
	uint8_t mode, afh_map[10];
	struct hci_version version;
	struct hci_dev_info di;
	struct hci_conn_info_req *cr;
	int i, cc = 0;

	if (hci_devinfo(dev_id, &di) < 0) {
		perror("Can't get device info");
		exit(1);
	}

	printf("Requesting information ...\n");

	cr = malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
	if (!cr) {
		perror("Can't get connection info");
		exit(1);
	}

	bacpy(&cr->bdaddr, bdaddr);
	cr->type = ACL_LINK;
	if (ioctl(dd, HCIGETCONNINFO, (unsigned long) cr) < 0) {
		if (hci_create_connection(dd, bdaddr,
					htobs(di.pkt_type & ACL_PTYPE_MASK),
					0, 0x01, &handle, 25000) < 0) {
			perror("Can't create connection");
			return;
		}
		sleep(1);
		cc = 1;
	} else
		handle = htobs(cr->conn_info->handle);

	ba2str(bdaddr, addr);
	printf("\tBD Address:  %s\n", addr);

	if (hci_read_remote_name(dd, bdaddr, sizeof(name), name, 25000) == 0)
		printf("\tDevice Name: %s\n", name);

	if (hci_read_remote_version(dd, handle, &version, 20000) == 0) {
		char *ver = lmp_vertostr(version.lmp_ver);
		printf("\tLMP Version: %s (0x%x) LMP Subversion: 0x%x\n"
			"\tManufacturer: %s (%d)\n",
			ver ? ver : "n/a",
			version.lmp_ver,
			version.lmp_subver,
			bt_compidtostr(version.manufacturer),
			version.manufacturer);
		if (ver)
			bt_free(ver);
	}

	memset(features, 0, sizeof(features));
	hci_read_remote_features(dd, handle, features, 20000);

	if ((di.features[7] & LMP_EXT_FEAT) && (features[7] & LMP_EXT_FEAT))
		hci_read_remote_ext_features(dd, handle, 0, &max_page,
							features, 20000);

	printf("\tFeatures%s: 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x "
				"0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x\n",
		(max_page > 0) ? " page 0" : "",
		features[0], features[1], features[2], features[3],
		features[4], features[5], features[6], features[7]);

	tmp = lmp_featurestostr(features, "\t\t", 63);
	printf("%s\n", tmp);
	bt_free(tmp);

	for (i = 1; i <= max_page; i++) {
		if (hci_read_remote_ext_features(dd, handle, i, NULL,
							features, 20000) < 0)
			continue;

		printf("\tFeatures page %d: 0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x "
					"0x%2.2x 0x%2.2x 0x%2.2x 0x%2.2x\n", i,
			features[0], features[1], features[2], features[3],
			features[4], features[5], features[6], features[7]);
	}

	if (hci_read_clock_offset(dd, handle, &offset, 1000) < 0) {
		perror("Reading clock offset failed");
		exit(1);
	}

	printf("\tClock offset: 0x%4.4x\n", btohs(offset));

	if(hci_read_afh_map(dd, handle, &mode, afh_map, 1000) < 0) {
	perror("HCI read AFH map request failed");
	}
	if(mode == 0x01) {
		// DGS: Replace with call to btbb_print_afh_map - need a piconet
		printf("\tAFH Map: 0x");
		for(i=0; i<10; i++)
			printf("%02x", afh_map[i]);
		printf("\n");
	} else {
		printf("AFH disabled.\n");
	}
	free(cr);

	if (cc) {
		usleep(10000);
		hci_disconnect(dd, handle, HCI_OE_USER_ENDED_CONNECTION, 10000);
	}
}

/* For a given BD_ADDR, print address, name and class */
void print_name_and_class(int dev_handle, int dev_id, bdaddr_t *bdaddr,
						  char* printable_addr, uint8_t extended)
{
	char name[248] = { 0 };

	if (hci_read_remote_name(dev_handle, bdaddr, sizeof(name), name, 0) < 0)
			strcpy(name, "[unknown]");

	printf("%s\t%s\n", printable_addr, name);
	if (extended)
		extra_info(dev_handle, dev_id, bdaddr);
}

#endif

void xbaswap(bdaddr_t *dst, const bdaddr_t *src)
{
       register unsigned char *d = (unsigned char *) dst;
       register const unsigned char *s = (const unsigned char *) src;
       register int i;

       for (i = 0; i < 6; i++)
              d[i] = s[5-i];
}

int xstr2ba(const char *str, bdaddr_t *ba)
{
       uint8_t b[6];
       const char *ptr = str;
       int i;

       for (i = 0; i < 6; i++) {
              b[i] = (uint8_t) strtol(ptr, NULL, 16);
              if (i != 5 && !(ptr = strchr(ptr, ':')))
                     ptr = ":00:00:00:00:00";
              ptr++;
       }

       xbaswap(ba, (bdaddr_t *) b);

       return 0;
}



int main(int argc, char *argv[])
{

    uint8_t buffer[SYM_LEN];

    char syms[BANK_LEN];


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
        btbb_packet* pkt = NULL;

        if (find_giac(buffer))
            printf("find giac ...\n");

        unpack_symbols(buffer, syms);
        offset = btbb_find_ac(syms, BANK_LEN - 64, LAP_ANY, 2, &pkt);
        if (offset >= 0) {
            btbb_packet_set_data(pkt, syms + offset, BANK_LEN-offset, 0, 0);
            btbb_process_packet(pkt, NULL);
            printf("find acc.[%d] LAP:%06x err:%u!\n", offset,
                    btbb_packet_get_lap(pkt), btbb_packet_get_ac_errors(pkt));
        }

        if(pkt) btbb_packet_unref(pkt);
    }



    printf("\nScan results:\n");

{
    btbb_piconet* pn;
    int lap;
    uint8_t uap;

    char addr[19];
    bdaddr_t bdaddr;

	while((pn=btbb_next_survey_result()) != NULL) {
		lap = btbb_piconet_get_lap(pn);
		if (btbb_piconet_get_flag(pn, BTBB_UAP_VALID)) {
			uap = btbb_piconet_get_uap(pn);
			sprintf(addr, "00:00:%02X:%02X:%02X:%02X", uap,
			        (lap >> 16) & 0xFF, (lap >> 8) & 0xFF, lap & 0xFF);
			xstr2ba(addr, &bdaddr);
			/* Printable version showing that the NAP is unknown */
			sprintf(addr, "??:??:%02X:%02X:%02X:%02X", uap,
			        (lap >> 16) & 0xFF, (lap >> 8) & 0xFF, lap & 0xFF);
			//print_name_and_class(dev_handle, dev_id, &bdaddr, addr, extended);
		} else
			printf("??:??:??:%02X:%02X:%02X\n", (lap >> 16) & 0xFF,
			       (lap >> 8) & 0xFF, lap & 0xFF);
		btbb_print_afh_map(pn);
	}
}


    printf("done\n");

    /* Close file descriptors */
    close (fd);

    return 0;
}



















