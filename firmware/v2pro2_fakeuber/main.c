/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>

#include <libopencm3/stm32/dma.h>

//#include <errno.h>
//#include <stdio.h>
//#include <unistd.h>

#include <string.h>

#include "config.h"
#include "cc.h"
#include "usbhid.h"
#include "uart.h"

void delay(void)
{
    u32 i;
    for (i = 0; i < 1000000; i++) { /* Wait a bit. */
        __asm__("nop");
    }
}


static void usart_setup(void)
{
    /* Enable clocks for USART1. */
    rcc_periph_clock_enable(RCC_USART1);

    /* Setup USART1 parameters. */
    usart_set_baudrate(nUSART, 115200);
    usart_set_databits(nUSART, 8);
    usart_set_stopbits(nUSART, USART_STOPBITS_1);
    usart_set_mode(nUSART, USART_MODE_TX);
    usart_set_parity(nUSART, USART_PARITY_NONE);
    usart_set_flow_control(nUSART, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(nUSART);
}

static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);

    /* Set GPIO LED. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, PIN_LED1); // LED1
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, PIN_LED2); // LED2
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, PIN_LED3); // LED3
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, PIN_LED4); // LED4

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO10);

    /* Setup USART1 TX/RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);   // tx
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);  // rx

    /* GPIO spi pins */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_MOSI | PIN_SCLK);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT,  GPIO_PUPD_PULLUP, PIN_MISO);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_CSN);

    /* other pins */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, \
                                        PIN_RX | PIN_TX | PIN_BTGR);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, \
                                        PIN_PAEN | PIN_HGM);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,  GPIO_PUPD_PULLUP, 1 << 13); // ?????

    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_GIO6);

    BTGR_CLR(); TX_CLR(); RX_CLR();

    LED1_SET();LED2_SET();LED3_SET();LED4_SET();

    /* USB pins */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12 );
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12 );

}

static void spi_setup(void)
{
    /*
      *       NSS          SCK                   MISO                    MOSI
      *   --------------   -------------------   -------------           ---------------
      * SPI3  PA15*, PA4*  PB3*, PC10*           PB4*, PC11*             PB5*, PD6, PC12*
      */

     // set spi sck PB3
     gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3);
     gpio_set_af(GPIOB, GPIO_AF6, GPIO3);

     // set spi MISO PB4
     gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4);
     gpio_set_af(GPIOB, GPIO_AF6, GPIO4);
 
     // set spi MoSI PB5
     //gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
     //gpio_set_af(GPIOB, GPIO_AF6, GPIO5);

     rcc_periph_clock_enable(RCC_SPI3);

     // slave mode, 8 bit, one line only,
     SPI3_CR1 = SPI_CR1_BIDIMODE_1LINE_BIDIR | SPI_CR1_SSM
             | SPI_CR1_CPHA | SPI_CR1_CPOL; // SPI_CR1_BIDIOE
     //SPI3_CR1 =  SPI_CR1_SSM | SPI_CR1_CPHA | SPI_CR1_CPOL;
     
     // RX dma mode
     SPI3_CR2 |= 1;
}

#define CLK100NS (3125*(clkn & 0xfffff) + timer_get_counter(TIM2))

extern volatile u32 clkn;
extern volatile u32 clkn_offset;
extern volatile u16 clk100ns_offset;

extern volatile uint8_t  idle_buf_clkn_high;
extern volatile uint32_t idle_buf_clk100ns;
extern volatile uint16_t idle_buf_channel;

extern volatile u16 channel;

static void tim_setup(void)
{
    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(RCC_TIM2);

    /* Enable TIM2 interrupt. */
    nvic_enable_irq(NVIC_TIM2_IRQ);

    /* Reset TIM2 peripheral. */
    timer_reset(TIM2);

    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    //timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 100000) - 0);
    timer_set_prescaler(TIM2, 5); // rcc_apb1_frequency(42MHz) * 2 / (5 + 1) = 14MHz

    /* Enable preload. */
    timer_disable_preload(TIM2);

    /* Continous mode. */
    timer_continuous_mode(TIM2);

    /* Period. */
    timer_set_period(TIM2, 4374); // 312.5us / (1/14MHz) = 14 * 312.5

    /* Disable outputs. */
    timer_disable_oc_output(TIM2, TIM_OC1);
    timer_disable_oc_output(TIM2, TIM_OC2);
    timer_disable_oc_output(TIM2, TIM_OC3);
    timer_disable_oc_output(TIM2, TIM_OC4);

    /* -- OC1 configuration -- */
#if 0
    /* Configure global mode of line 1. */
    timer_disable_oc_clear(TIM2, TIM_OC1);
    timer_disable_oc_preload(TIM2, TIM_OC1);
    timer_set_oc_slow_mode(TIM2, TIM_OC1);
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

    /* Set the capture compare value for OC1. */
    timer_set_oc_value(TIM2, TIM_OC1, 1000);

    /* Enable commutation interrupt. */
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
#endif
    /* ---- */

    /* ARR reload enable. */
    timer_disable_preload(TIM2);

    /* Counter enable. */
    timer_enable_counter(TIM2);

    /* Enable commutation interrupt. */
    timer_enable_irq(TIM2, TIM_DIER_UIE);
}

#define UES_DMA_STREAM DMA_STREAM0
#define UES_DMA_CONUR  DMA1

static void dma_setup(void)
{
    /* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
    /* Enable DMA1 clock and IRQ */
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STREAM0_IRQ);

    dma_stream_reset(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_set_priority(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_PL_LOW);
    dma_set_memory_size(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_enable_circular_mode(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_set_transfer_mode(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

    /* The register to target is the DAC1 8-bit right justified data register */
    dma_set_peripheral_address(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) &SPI3_DR);
    /* The array v[] is filled with the waveform data to be output */
    dma_set_memory_address(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) rxbuf1);
    dma_set_memory_address_1(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) rxbuf2);
    dma_set_number_of_data(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SIZE);

    dma_channel_select(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_CHSEL_0);
    dma_enable_double_buffer_mode(UES_DMA_CONUR, UES_DMA_STREAM);

    dma_enable_transfer_complete_interrupt(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_enable_transfer_error_interrupt(UES_DMA_CONUR, UES_DMA_STREAM);

    dma_enable_stream(UES_DMA_CONUR, UES_DMA_STREAM);
}


static volatile u32 slotn = 0;
static volatile uint8_t tx_status = 0;

/* operation mode */
volatile u8 requested_mode = MODE_IDLE;
volatile u8 modulation = MOD_BT_BASIC_RATE;

enum usb_pkt_status {
    DMA_OVERFLOW  = 0x01,
    DMA_ERROR     = 0x02,
    FIFO_OVERFLOW = 0x04,
    CS_TRIGGER    = 0x08,
    RSSI_TRIGGER  = 0x10,
    DISCARD       = 0x20,
};

enum usb_pkt_types {
    BR_PACKET  = 0,
    LE_PACKET  = 1,
    MESSAGE    = 2,
    KEEP_ALIVE = 3,
    SPECAN     = 4,
    LE_PROMISC = 5,
    EGO_PACKET = 6,
};

/*
 * USB packet for Bluetooth RX (64 total bytes)
 */
typedef struct {
    u8     pkt_type;
    u8     status;
    u8     channel;
    u8     clkn_high;
    u32    clk100ns;
    char   rssi_max;   // Max RSSI seen while collecting symbols in this packet
    char   rssi_min;   // Min ...
    char   rssi_avg;   // Average ...
    u8     rssi_count; // Number of ... (0 means RSSI stats are invalid)
    u8     reserved[2];
    u8     data[DMA_SIZE];
} usb_pkt_rx;

usb_pkt_rx fifo[128];

volatile u32 head = 0;
volatile u32 tail = 0;

extern volatile char max_rssi;

static void queue_init(void)
{
    head = 0;
    tail = 0;
    memset(fifo, 0, sizeof(fifo));
}

static usb_pkt_rx *fifo_enqueue(void)
{
    u8 h = head & 0x7F;
    u8 t = tail & 0x7F;
    u8 n = (t + 1) & 0x7F;

    /* fail if queue is full */
    if (h == n) {
        return NULL;
    }

    ++tail;
    return &fifo[t];
}

static usb_pkt_rx *fifo_dequeue(void)
{
    u8 h = head & 0x7F;
    u8 t = tail & 0x7F;

    /* fail if queue is empty */
    if (h == t) {
        return NULL;
    }

    ++head;
    return &fifo[h];
}

static int usb_enqueue(uint8_t type, uint8_t* buf)
{
    usb_pkt_rx* f = fifo_enqueue();

    /* fail if queue is full */
    if (f == NULL) {
        tx_status |= FIFO_OVERFLOW;
        kputc('F');
        return 0;
    }

    f->pkt_type = type;
    if(type == SPECAN) {
        f->clkn_high = 0;
        f->clk100ns = 0;
    } else {
        f->clkn_high = idle_buf_clkn_high;
        f->clk100ns = idle_buf_clk100ns;
        f->channel = (uint8_t)((idle_buf_channel - 2402) & 0xff);
        f->rssi_min = -100;
        f->rssi_max = max_rssi;
        f->rssi_avg = 0;
        f->rssi_count = 0;
    }

    memcpy(f->data, buf, DMA_SIZE);

    f->status = tx_status;
    tx_status = 0;

    return 1;
}

static void usb_dequeue(void)
{
    static usb_pkt_rx* f = NULL;

    if (f == NULL) {
        f = fifo_dequeue();
    }

    if (f != NULL) {
        if (usb_write_packet((u8 *)f, sizeof(usb_pkt_rx))) {
            f = NULL;
            kputc('S');
        } else {
            kputc('E');
        }
    }
}

int usb_request(u8 request)
{
    switch (request) {
    case UBERTOOTH_SET_CHANNEL:
        break;
    case UBERTOOTH_RX_SYMBOLS:
        requested_mode = MODE_RX_SYMBOLS;
        break;
    case UBERTOOTH_STOP:
        requested_mode = MODE_IDLE;
        break;

    default:
        return USBD_REQ_NOTSUPP;
    }

    return USBD_REQ_HANDLED;
}


int main(void)
{
    gpio_setup();

    usart_setup();

    kputs("\nTx test start\n");

    printf("system uart output\n");

    delay();
    
    while(0) {
        delay();delay();delay();
        delay();delay();delay();
        LED1_TOG();
    }

    cc_reset();

    cc_init();

    delay(); // why ???

    /* Enable external high-speed oscillator 16MHz. */
    rcc_osc_bypass_enable(RCC_HSE);
    rcc_clock_setup_hse_3v3(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_OTGFS);

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 2000000);
    
    LED1_CLR();LED2_CLR();LED3_CLR();LED4_CLR();

    kputs("system clock init done!\n");

    tim_setup();

    spi_setup();

    dma_setup();

    usb_setup();

    SPI_CR1(SPI3) |= SPI_CR1_SPE;

    cc_rx_mode();
    //cc_specan_mode();

    queue_init();

    while(1) {

        while(!slotn) {
            cc_hop();
            usb_poll();
        }

        switch (requested_mode) {
        case MODE_IDLE:
            break;
        case MODE_RX_SYMBOLS:
            usb_enqueue(BR_PACKET, (uint8_t*)idle_rxbuf);
            usb_dequeue();
            break;
        }

        //if (slotn > 1) // Missed a DMA trasfer
        //    gpio_toggle(GPIOC, PIN_USRLED);

        slotn = 0;
    }

    return 0;
}

/* clkn_high is incremented each time CLK100NS rolls over */
void tim2_isr(void)
{
    if (TIM_SR(TIM2) & TIM_SR_UIF) {
        /* Clear compare interrupt flag. */
        TIM_SR(TIM2) = ~TIM_SR_UIF;
        cc_clkn_handler();
        LED4_TOG();
    }
}

/*--------------------------------------------------------------------*/
/* The ISR simply provides a test output for a CRO trigger */

void dma1_stream0_isr(void)
{
    if (dma_get_interrupt_flag(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TCIF)) {
        dma_clear_interrupt_flags(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TCIF);
        if (DMA_SCR(UES_DMA_CONUR, UES_DMA_STREAM) & DMA_SxCR_CT) {
            active_rxbuf = &rxbuf2[0];
            idle_rxbuf = &rxbuf1[0];
        } else {
            active_rxbuf = &rxbuf1[0];
            idle_rxbuf = &rxbuf2[0];
        }

        idle_buf_clk100ns  = CLK100NS;
        idle_buf_clkn_high = (clkn >> 20) & 0xff;
        idle_buf_channel   = channel;

        slotn++;
    } else if (dma_get_interrupt_flag(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TEIF)) {
        dma_clear_interrupt_flags(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TEIF);
    }
}


