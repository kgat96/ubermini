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

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

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

    usart_enable_rx_dma(USART1);

    /* Finally enable the USART. */
    usart_enable(USART1);
}


static void gpio_setup(void)
{
    /* Enable GPIO clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);


    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO10);

    /* Setup USART1 TX/RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);   // tx
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);  // rx

    /* Set GPIO LED. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO8); // LED1
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO7); // LED2
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO6); // LED3
}

#include <libopencm3/stm32/dma.h>

/* Globals */
uint8_t waveform1[8];
uint8_t waveform2[8];

#define UES_DMA_STREAM DMA_STREAM5
#define UES_DMA_CONUR  DMA2

/*--------------------------------------------------------------------*/
static void dma_setup(void)
{
    /* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
    /* Enable DMA1 clock and IRQ */
    rcc_periph_clock_enable(RCC_DMA2);
    nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);

    dma_stream_reset(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_set_priority(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_PL_LOW);
    dma_set_memory_size(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_enable_circular_mode(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_set_transfer_mode(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

    /* The register to target is the DAC1 8-bit right justified data register */
    dma_set_peripheral_address(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) &USART_DR(USART1));
    /* The array v[] is filled with the waveform data to be output */
    dma_set_memory_address(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) waveform1);
    dma_set_memory_address_1(UES_DMA_CONUR, UES_DMA_STREAM, (uint32_t) waveform2);
    dma_set_number_of_data(UES_DMA_CONUR, UES_DMA_STREAM, 4);

    dma_channel_select(UES_DMA_CONUR, UES_DMA_STREAM, DMA_SxCR_CHSEL_4);
    dma_enable_double_buffer_mode(UES_DMA_CONUR, UES_DMA_STREAM);

    dma_enable_transfer_complete_interrupt(UES_DMA_CONUR, UES_DMA_STREAM);
    dma_enable_transfer_error_interrupt(UES_DMA_CONUR, UES_DMA_STREAM);

    dma_enable_stream(UES_DMA_CONUR, UES_DMA_STREAM);


}

/*--------------------------------------------------------------------*/
/* The ISR simply provides a test output for a CRO trigger */

void dma2_stream5_isr(void)
{
    if (dma_get_interrupt_flag(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TCIF)) {
        dma_clear_interrupt_flags(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TCIF);

        if (DMA_SCR(UES_DMA_CONUR, UES_DMA_STREAM) & DMA_SxCR_CT) {
            kputhex(waveform1[0], 2); kputc(' ');
            kputhex(waveform1[1], 2); kputc(' ');
            kputhex(waveform1[2], 2); kputc(' ');
            kputhex(waveform1[3], 2); kputc(' ');
        } else {
            kputhex(waveform2[0], 2); kputc(' ');
            kputhex(waveform2[1], 2); kputc(' ');
            kputhex(waveform2[2], 2); kputc(' ');
            kputhex(waveform2[3], 2); kputc(' ');
        }

        gpio_toggle(GPIOC, GPIO7);
    } else if (dma_get_interrupt_flag(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TEIF)) {
        dma_clear_interrupt_flags(UES_DMA_CONUR, UES_DMA_STREAM, DMA_TEIF);
        gpio_toggle(GPIOC, GPIO6);
    }
}



int main(void)
{
    /* Enable internal high-speed oscillator. */
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);

    /* Select HSI as SYSCLK source. */
    rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

    gpio_setup();

    dma_setup();

    usart_setup();

    kputs("\nTest start\n");

    /* Blink the LED (PC8) on the board. */
    while (1) {
        //kputc('a');
        /* Manually: */
        /* Using API functions gpio_set()/gpio_clear(): */
        /* Using API function gpio_toggle(): */

        gpio_toggle(GPIOC, GPIO0);
        gpio_toggle(GPIOC, GPIO8);
        delay();
    }

    return 0;
}
