/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Linker script for ST STM32F4 (STM32F406VG, 1024K flash, 128K RAM). */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN =  0x08000000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20010000, LENGTH = 64K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f4.ld
