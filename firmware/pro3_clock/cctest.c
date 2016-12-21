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

//#include <libopencmsis/core_cm3.h>

#include "cc.h"

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define kputc(a) usart_send_blocking(USART1, a);

static void kputhex(unsigned int value, int digits)
{
    while (digits-- > 0) {
        unsigned int tmp = (value >> (4 * digits)) & 0xf;
        kputc(tmp > 9 ? tmp - 10 + 'a' : tmp + '0');
    }
}

static void kputs(char *s)
{
    while (*s) {
        if (*s == '\n')
            kputc('\r');
        kputc(*s++);
    }
}

static void delay(void)
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
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);

    if (0) {

        u32 r1, r2, r3, r4, i;

        for (i = 0; i < 1000000; i++) { /* Wait a bit. */
            __asm__("nop");
        }


        USART_SR(USART1) = 0;

        r1 = USART_SR(USART1);
        while ((USART_SR(USART1) & USART_SR_TXE) == 0);
        r2 = USART_SR(USART1);
        USART_DR(USART1) = 'x';
        r3 = USART_SR(USART1);
        while ((USART_SR(USART1) & USART_SR_TC) == 0);

        r4 = USART_SR(USART1);
        while ((USART_SR(USART1) & USART_SR_TXE) == 0);

        USART_DR(USART1) = 'x';

        kputhex(r1, 4);kputhex(r2, 4);kputhex(r3, 4);kputhex(r4, 4);
    }
}


//#define PIN_VBUS   (1 << 30) /* P1.30 */
//#define PIN_CC1V8  (1 << 9 ) /* P1.9  */
//#define PIN_SSEL1   (1 << 28) /* P4.28 */

#define PIN_MOSI    GPIO7       /* Pa.7  */
#define PIN_MISO    GPIO6       /* Pa.6  */
#define PIN_GIO6    GPIO3       /* Pa.3  */

#define PIN_ATEST1  GPIO1       /* Pa.1  */
#define PIN_ATEST2  GPIO2       /* Pa.2  */

#define PIN_CC3V3   GPIO12      /* Pb.12 */
#define PIN_RX      GPIO11      /* Pb.11 */
#define PIN_TX      GPIO10      /* Pb.10 */
#define PIN_BTGR    GPIO14      /* Pb.14 */

#define PIN_USRLED  GPIO8       /* Pc.8  */
#define PIN_RXLED   GPIO7       /* Pc.7  */
#define PIN_TXLED   GPIO6       /* Pc.6  */
#define PIN_CSN     GPIO5       /* Pc.5  */
#define PIN_SCLK    GPIO4       /* Pc.4  */
#define PIN_PAEN    GPIO3       /* Pc.3  */
#define PIN_HGM     GPIO2       /* Pc.2  */

//#define USRLED     (FIO1PIN & PIN_USRLED)
#define USRLED_SET gpio_set(GPIOC, PIN_USRLED)
#define USRLED_CLR gpio_clear(GPIOC, PIN_USRLED)
//#define RXLED      (FIO1PIN & PIN_RXLED)
#define RXLED_SET  gpio_set(GPIOC, PIN_RXLED)
#define RXLED_CLR  gpio_clear(GPIOC, PIN_RXLED)
//#define TXLED      (FIO1PIN & PIN_TXLED)
#define TXLED_SET  gpio_set(GPIOC, PIN_TXLED)
#define TXLED_CLR  gpio_clear(GPIOC, PIN_TXLED)

#define CC3V3_SET  gpio_set(GPIOB, PIN_CC3V3)
#define CC3V3_CLR  gpio_clear(GPIOB, PIN_CC3V3)
#define RX_SET     gpio_set(GPIOB, PIN_RX)
#define RX_CLR     gpio_clear(GPIOB, PIN_RX)
#define TX_SET     gpio_set(GPIOB, PIN_TX)
#define TX_CLR     gpio_clear(GPIOB, PIN_TX)
#define CSN_SET    gpio_set(GPIOC, PIN_CSN)
#define CSN_CLR    gpio_clear(GPIOC, PIN_CSN)
#define SCLK_SET   gpio_set(GPIOC, PIN_SCLK)
#define SCLK_CLR   gpio_clear(GPIOC, PIN_SCLK)
#define MOSI_SET   gpio_set(GPIOA, PIN_MOSI)
#define MOSI_CLR   gpio_clear(GPIOA, PIN_MOSI)
//#define GIO6       (FIO2PIN & PIN_GIO6)
#define GIO6_SET   gpio_set(GPIOA, PIN_GIO6)
#define GIO6_CLR   gpio_clear(GPIOA, PIN_GIO6)
#define BTGR_SET   gpio_set(GPIOB, PIN_BTGR)
#define BTGR_CLR   gpio_clear(GPIOB, PIN_BTGR)
#define MISO       gpio_get(GPIOA, PIN_MISO)

#define PAEN     gpio_get(GPIOC, PIN_PAEN)
#define PAEN_SET gpio_set(GPIOC, PIN_PAEN)
#define PAEN_CLR gpio_clear(GPIOC, PIN_PAEN)
#define HGM      gpio_get(GPIOC, PIN_HGM)
#define HGM_SET  gpio_set(GPIOC, PIN_HGM)
#define HGM_CLR  gpio_clear(GPIOC, PIN_HGM)

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



static void atest_init(void)
{
    /*
     * ADC can optionally be configured for ATEST1 and ATEST2, but for now we
     * set them as floating inputs.
     */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_ATEST1 | PIN_ATEST2);
}


static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

#if 0
    // don't set swd
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO9 | GPIO10);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_ALL);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO_ALL);

    gpio_clear(GPIOA, GPIO9 | GPIO10);
    gpio_clear(GPIOB, GPIO_ALL);
    gpio_clear(GPIOC, GPIO_ALL);
#endif

    /* Set GPIO5 (in GPIO port A) to 'output push-pull'. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO8);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO0);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO15);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO10);

    /* Setup USART1 TX/RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);   // tx
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);  // rx

    // define in board

    /* set certain pins as outputs, all others inputs */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_MOSI);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
            PIN_TX | PIN_CC3V3 | PIN_RX | PIN_BTGR);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
            PIN_USRLED | PIN_RXLED | PIN_TXLED | PIN_CSN | PIN_SCLK | PIN_PAEN | PIN_HGM);

    // debug gpio 4
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO15);





}

/*
 * This is a single SPI transaction of variable length, usually 8 or 24 bits.
 * The CC2400 also supports longer transactions (e.g. for the FIFO), but we
 * haven't implemented anything longer than 32 bits.
 *
 * We're bit-banging because:
 *
 * 1. We're using one SPI peripheral for the CC2400's unbuffered data
 *    interface.
 * 2. We're saving the second SPI peripheral for an expansion port.
 * 3. The CC2400 needs CSN held low for the entire transaction which the
 *    LPC17xx SPI peripheral won't do without some workaround anyway.
 */
static u32 cc_spi(u8 len, u32 data)
{
    u32 msb = 1 << (len - 1);

    /* start transaction by dropping CSN */
    CSN_CLR;

    while (len--) {
        if (data & msb)
            MOSI_SET;
        else
            MOSI_CLR;
        data <<= 1;

        SCLK_SET;
        if (MISO)
            data |= 1;

        SCLK_CLR;
    }

    /* end transaction by raising CSN */
    CSN_SET;

    return data;
}

/* read 16 bit value from a register */
static u16 cc_get(u8 reg)
{
    u32 in;

    u32 out = (reg | 0x80) << 16;
    in = cc_spi(24, out);
    return in & 0xFFFF;
}

/* write 16 bit value to a register */
static void cc_set(u8 reg, u16 val)
{
    u32 out = (reg << 16) | val;
    cc_spi(24, out);
}

static void cc_init(void)
{
    atest_init();

    /* CSN (slave select) is active low */
    CSN_SET;

    /* activate 3V3 supply for CC2400 IO */
    CC3V3_SET;

    // check chip id
    {
        u32 id = cc_get(MANFIDL);
        //id |= cc_get(MANFIDH) << 16;
        //kputhex(id, 8);

        if (id == 0x133d) {
            kputs("read cc chip id:"); kputhex(id, 4);kputs("\n");
        } else {
            kputs("Can't read cc chip id !");
        }
    }

    /* initialize various cc2400 settings - see datasheet pg63 */
    cc_set(MANAND,  0x7fff);


}

/* get the status */
static u8 cc_status(void)
{
    return cc_spi(8, 0);
}

/* strobe register, return status */
static u8 cc_strobe(u8 reg)
{
    return cc_spi(8, reg);
}

/*
 * Warning: This should only be called when running on the internal oscillator.
 * Otherwise use clock_start().
 */
static void cc_reset(void)
{
    cc_set(MAIN, 0x0000);
    while (cc_get(MAIN) != 0x0000);
    cc_set(MAIN, 0x8000);
    while (cc_get(MAIN) != 0x8000);
}

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

//const struct rcc_clock_scale rcc_160mhz = {
//    /* 160MHz */
//    .pllm = 16,
//    .plln = 320,
//    .pllp = 2,
//    .pllq = 7,
//    .pllr = 0,
//    .hpre = RCC_CFGR_HPRE_DIV_NONE,
//    .ppre1 = RCC_CFGR_PPRE_DIV_4,
//    .ppre2 = RCC_CFGR_PPRE_DIV_2,
//    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
//            FLASH_ACR_LATENCY_5WS,
//    .ahb_frequency  = 320000000,
//    .apb1_frequency = 40000000,
//    .apb2_frequency = 80000000,
//};


static void clock_init(void)
{
    /* Enable internal high-speed oscillator. */
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);

    /* Select HSI as SYSCLK source. */
    rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

    kputs("RCC HSI STABLE\n");

    /* configure CCxx00 oscillator
     * output carrier sense on GIO1,
     * output 16MHz GIO6 */
    cc_reset();
    //cc_set(IOCFG, (GIO_CLK_16M << 9) | (GIO_CARRIER_SENSE_N << 3));
    cc_strobe(SXOSCON);
    while (!(cc_status() & XOSC16M_STABLE));

    kputs("XOSC16M STABLE\n");

    /* Enable external high-speed oscillator 16MHz. */
    rcc_clock_setup_hse_3v3(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

}

static void tim_setup(void)
{
    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(RCC_TIM2);

    /* Enable TIM2 interrupt. */
    nvic_enable_irq(NVIC_TIM2_IRQ);

    /* Reset TIM2 peripheral. */
    timer_reset(TIM2);

    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
               TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    /* Reset prescaler value.
     * Running the clock at 5kHz.
     */
    /*
     * On STM32F4 the timers are not running directly from pure APB1 or
     * APB2 clock busses.  The APB1 and APB2 clocks used for timers might
     * be the double of the APB1 and APB2 clocks.  This depends on the
     * setting in DCKCFGR register. By default the behaviour is the
     * following: If the Prescaler APBx is greater than 1 the derived timer
     * APBx clocks will be double of the original APBx frequencies. Only if
     * the APBx prescaler is set to 1 the derived timer APBx will equal the
     * original APBx frequencies.
     *
     * In our case here the APB1 is devided by 4 system frequency and APB2
     * divided by 2. This means APB1 timer will be 2 x APB1 and APB2 will
     * be 2 x APB2. So when we try to calculate the prescaler value we have
     * to use rcc_apb1_freqency * 2!!!
     *
     * For additional information see reference manual for the stm32f4
     * familiy of chips. Page 204 and 213
     */
    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 10000000) - 1);

    /* Enable preload. */
    timer_disable_preload(TIM2);

    /* Continous mode. */
    timer_continuous_mode(TIM2);

    /* Period. */
    timer_set_period(TIM2, 3276800000 - 1);

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

int main(void)
{
    /* Enable internal high-speed oscillator. */
    //rcc_osc_on(RCC_HSI);
    //rcc_wait_for_osc_ready(RCC_HSI);


    if (0) {

        gpio_setup();


        /* Enable external high-speed oscillator 16MHz. */
        rcc_clock_setup_hse_3v3(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

        rcc_periph_clock_enable(RCC_GPIOC);

        gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO8);

        while (1) {
            /* Manually: */
            /* Using API functions gpio_set()/gpio_clear(): */
            /* Using API function gpio_toggle(): */

            usart_setup();

            kputs("\nTest start\n");

            gpio_set(GPIOC, GPIO0); /* LED on/off */
            gpio_set(GPIOC, GPIO8); /* LED on/off */
            delay();
            gpio_clear(GPIOC, GPIO0); /* LED on/off */
            gpio_clear(GPIOC, GPIO8); /* LED on/off */
            delay();

        }
    }

    gpio_setup();

    usart_setup();

    kputs("\nTest start\n");

    RXLED_SET;

    cc_init();

    TXLED_SET;

    clock_init();

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 115200);

    kputs("cc clock init done!\n");

    USRLED_SET;

    tim_setup();

    /* Blink the LED (PC8) on the board. */
    while (1) {
        //kputc('a');
        /* Manually: */
        /* Using API functions gpio_set()/gpio_clear(): */
        /* Using API function gpio_toggle(): */

        gpio_set(GPIOC, GPIO0); /* LED on/off */
        gpio_set(GPIOB, GPIO15); /* LED on/off */
        delay();
        gpio_clear(GPIOC, GPIO0); /* LED on/off */
        gpio_clear(GPIOB, GPIO15); /* LED on/off */
        delay();

    }

    return 0;
}

#define CLK100NS TIM_CNT(TIM2)
volatile u8 clkn_high;
#define CLKN ((clkn_high << 20) | (CLK100NS / 3125))

/* clkn_high is incremented each time CLK100NS rolls over */
void tim2_isr(void)
{
    //kputs("tim2_isr\n");

    if (TIM_SR(TIM2) & TIM_SR_UIF) {
        /* Clear compare interrupt flag. */
        TIM_SR(TIM2) = ~TIM_SR_UIF;
        ++clkn_high;
        gpio_toggle(GPIOC, GPIO8);
    }

}

