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

static void usart_setup(void)
{
    /* Enable clocks for USART1. */
    rcc_periph_clock_enable(RCC_USART1);

    /* A9 / A10 */
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

    /* Setup USART1 TX/RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);   // tx
    gpio_set_af(GPIOA, GPIO_AF7, GPIO10);  // rx
}

static void gpio_setup(void)
{
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Set GPIO5 (in GPIO port A) to 'output push-pull'. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    //gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
}

int main(void)
{
	int i;

	usart_setup();

	gpio_setup();

	//gpio_clear(GPIOC, GPIO7);

    /* Blink the LED (PC8) on the board. */
    while (1) {
        /* Manually: */
        /* Using API functions gpio_set()/gpio_clear(): */
        /* gpio_set(GPIOA, GPIO5); *//* LED off */

	    usart_send_blocking(USART1, 'D');

		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOC, GPIO8);	/* LED on/off */
		gpio_toggle(GPIOC, GPIO0);	/* LED on/off */

		for (i = 0; i < 1000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}

 
 
 
 
 
 
 
 
 
 
 
 
