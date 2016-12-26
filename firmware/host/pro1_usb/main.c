
#include <stdlib.h>
#include <stdio.h>

#include <signal.h>

#include <libusb-1.0/libusb.h>

#define U0_VENDORID    0x1483
#define U0_PRODUCTID   0x5740

#define DATA_IN     (0x82 | LIBUSB_ENDPOINT_IN)
#define DATA_OUT    (0x01 | LIBUSB_ENDPOINT_OUT)
#define CTRL_IN     (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define TIMEOUT     10000

enum ubertooth_usb_commands {
    UBERTOOTH_PING               = 0,
    UBERTOOTH_RX_SYMBOLS         = 1,
    UBERTOOTH_TX_SYMBOLS         = 2,
    UBERTOOTH_GET_USRLED         = 3,
    UBERTOOTH_SET_USRLED         = 4,
    UBERTOOTH_GET_RXLED          = 5,
    UBERTOOTH_SET_RXLED          = 6,
    UBERTOOTH_GET_TXLED          = 7,
    UBERTOOTH_SET_TXLED          = 8,
    UBERTOOTH_GET_1V8            = 9,
    UBERTOOTH_SET_1V8            = 10,
    UBERTOOTH_GET_CHANNEL        = 11,
    UBERTOOTH_SET_CHANNEL        = 12,
    UBERTOOTH_RESET              = 13,
    UBERTOOTH_GET_SERIAL         = 14,
    UBERTOOTH_GET_PARTNUM        = 15,
    UBERTOOTH_GET_PAEN           = 16,
    UBERTOOTH_SET_PAEN           = 17,
    UBERTOOTH_GET_HGM            = 18,
    UBERTOOTH_SET_HGM            = 19,
    UBERTOOTH_TX_TEST            = 20,
    UBERTOOTH_STOP               = 21,
    UBERTOOTH_GET_MOD            = 22,
    UBERTOOTH_SET_MOD            = 23,
    UBERTOOTH_SET_ISP            = 24,
    UBERTOOTH_FLASH              = 25,
    BOOTLOADER_FLASH             = 26,
    UBERTOOTH_SPECAN             = 27,
    UBERTOOTH_GET_PALEVEL        = 28,
    UBERTOOTH_SET_PALEVEL        = 29,
    UBERTOOTH_REPEATER           = 30,
    UBERTOOTH_RANGE_TEST         = 31,
    UBERTOOTH_RANGE_CHECK        = 32,
    UBERTOOTH_GET_REV_NUM        = 33,
    UBERTOOTH_LED_SPECAN         = 34,
    UBERTOOTH_GET_BOARD_ID       = 35,
    UBERTOOTH_SET_SQUELCH        = 36,
    UBERTOOTH_GET_SQUELCH        = 37,
    UBERTOOTH_SET_BDADDR         = 38,
    UBERTOOTH_START_HOPPING      = 39,
    UBERTOOTH_SET_CLOCK          = 40,
    UBERTOOTH_GET_CLOCK          = 41,
    UBERTOOTH_BTLE_SNIFFING      = 42,
    UBERTOOTH_GET_ACCESS_ADDRESS = 43,
    UBERTOOTH_SET_ACCESS_ADDRESS = 44,
    UBERTOOTH_DO_SOMETHING       = 45,
    UBERTOOTH_DO_SOMETHING_REPLY = 46,
    UBERTOOTH_GET_CRC_VERIFY     = 47,
    UBERTOOTH_SET_CRC_VERIFY     = 48,
    UBERTOOTH_POLL               = 49,
    UBERTOOTH_BTLE_PROMISC       = 50,
    UBERTOOTH_SET_AFHMAP         = 51,
    UBERTOOTH_CLEAR_AFHMAP       = 52,
    UBERTOOTH_READ_REGISTER      = 53,
    UBERTOOTH_BTLE_SLAVE         = 54,
    UBERTOOTH_GET_COMPILE_INFO   = 55,
    UBERTOOTH_BTLE_SET_TARGET    = 56,
    UBERTOOTH_BTLE_PHY           = 57,
    UBERTOOTH_WRITE_REGISTER     = 58,
    UBERTOOTH_JAM_MODE           = 59,
    UBERTOOTH_EGO                = 60,
    UBERTOOTH_AFH                = 61,
    UBERTOOTH_HOP                = 62,
    UBERTOOTH_TRIM_CLOCK         = 63,
    UBERTOOTH_GET_API_VERSION    = 64,
    UBERTOOTH_WRITE_REGISTERS    = 65,
    UBERTOOTH_READ_ALL_REGISTERS = 66,
    UBERTOOTH_RX_GENERIC         = 67,
    UBERTOOTH_TX_GENERIC_PACKET  = 68,
    UBERTOOTH_FIX_CLOCK_DRIFT    = 69,
};

static void show_libusb_error(int error_code)
{
    char *error_hint = "";
    const char *error_name;

    /* Available only in libusb > 1.0.3 */
    // error_name = libusb_error_name(error_code);

    switch (error_code) {
        case LIBUSB_ERROR_TIMEOUT:
            error_name="Timeout";
            break;
        case LIBUSB_ERROR_NO_DEVICE:
            error_name="No Device";
            error_hint="Check Ubertooth is connected to host";
            break;
        case LIBUSB_ERROR_ACCESS:
            error_name="Insufficient Permissions";
            break;
        case LIBUSB_ERROR_OVERFLOW:
            error_name="Overflow";
            error_hint="Try resetting the Ubertooth";
            break;
        default:
            error_name="Command Error";
            break;
    }

    fprintf(stderr,"xlibUSB Error: %s: %s (%d) [%s]\n",
            error_name, error_hint, error_code, libusb_error_name(error_code));
}

#define PKT_LEN       64

uint8_t rx_buf[1024];

static void callback_xfer(struct libusb_transfer *xfer)
{
    int ret;

    //LIBUSB_TRANSFER_COMPLETED
    fprintf(stderr, "USB transfer callback %d\n", xfer->status);
    fprintf(stderr, "%x %x %x %x\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

    ret = libusb_submit_transfer(xfer);
    if (ret < 0)
        fprintf(stderr, "Failed to submit USB transfer (%d)\n", ret);
}

static int do_exit = 0;

static void sighandler(int signum)
{
    do_exit = 1;
}


int main(int argc, char *argv[])
{
    struct sigaction sigact;

    int usb_devs;
    int i;

    struct libusb_context *ctx = NULL;
    struct libusb_device **usb_list = NULL;
    struct libusb_device_handle *devh = NULL;
    struct libusb_device_descriptor desc;
    struct libusb_transfer* rx_transfer;

    int usbindex= 0;

    int ret = libusb_init(NULL);
    if (ret < 0) {
        fprintf(stderr, "libusb_init failed (got 1.0?)\n");
        return -1;
    }

    usb_devs = libusb_get_device_list(ctx, &usb_list);

    for (i = 0; i < usb_devs; ++i) {
        ret = libusb_get_device_descriptor(usb_list[i], &desc);
        if (ret < 0)
            fprintf(stderr, "couldn't get usb descriptor for dev #%d!\n", i);
        else {
            //fprintf(stderr, "usbdev %d: %x:%x\n", i, desc.idVendor, desc.idProduct);
            if (desc.idVendor == U0_VENDORID && desc.idProduct == U0_PRODUCTID) {
                usbindex = i;
                fprintf(stderr, "find usbdev %d: %x:%x\n", i, desc.idVendor, desc.idProduct);
                break;
            }
        }
    }

    ret = libusb_open(usb_list[usbindex], &devh);
    if (ret) show_libusb_error(ret);
    if (devh == NULL) {
        fprintf(stderr, "could not open device\n");
        goto LIBUSB_STOP;
    }

    ret = libusb_detach_kernel_driver(devh, 0);
    if (ret < 0) {
        fprintf(stderr, "usb_detach_kernel_driver_np error %d\n", ret);
    }

    ret = libusb_claim_interface(devh, 0);
    if (ret < 0) {
        fprintf(stderr, "usb_claim_interface error %d\n", ret);
        goto LIBUSB_STOP;
    }

    {
        sigact.sa_handler = sighandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        sigaction(SIGINT, &sigact, NULL);
        sigaction(SIGTERM, &sigact, NULL);
        sigaction(SIGQUIT, &sigact, NULL);

        unsigned char data[4];
        ret = libusb_control_transfer(devh, CTRL_IN, 0x33,
                0, 0, data, 4, 3000);
        if (ret < 0) {
            show_libusb_error(ret);
            goto LIBUSB_STOP;
        }
    }

    rx_transfer = libusb_alloc_transfer(0);

    libusb_fill_bulk_transfer(rx_transfer, devh, DATA_IN,
            rx_buf, PKT_LEN, callback_xfer, NULL, TIMEOUT);

    ret = libusb_submit_transfer(rx_transfer);
    if (ret < 0) {
        fprintf(stderr, "rx_transfer submission err: %d\n", ret);
        goto LIBUSB_STOP;
    }

    {
        unsigned char data[4];
        ret = libusb_control_transfer(devh, CTRL_IN, 0x22,
                0, 0, data, 4, 3000);
        if (ret < 0) {
            show_libusb_error(ret);
            goto LIBUSB_STOP;
        }
    }

    {
        while (!do_exit) {
            ret = libusb_handle_events(NULL);
            if (ret < 0)
                fprintf(stderr, "libusb handle events %d\n", ret);
        }
    }

LIBUSB_STOP:
    if(rx_transfer != NULL)
        libusb_cancel_transfer(rx_transfer);

    if(devh != NULL) {
        libusb_release_interface(devh, 0);
        libusb_attach_kernel_driver(devh, 0);
    }

    libusb_close(devh);
    libusb_exit(NULL);

    return ret;
}



















