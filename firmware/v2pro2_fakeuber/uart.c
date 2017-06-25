/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2017 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdarg.h>
#include "uart.h"

void kputhex(unsigned int value, int digits)
{
    while (digits-- > 0) {
        unsigned int tmp = (value >> (4 * digits)) & 0xf;
        kputc(tmp > 9 ? tmp - 10 + 'a' : tmp + '0');
    }
}

void kputs(char *s)
{
    while (*s) {
        if (*s == '\n')
            kputc('\r');
        kputc(*s++);
    }
}

static void xvprintf(const char* fmt,  /* Pointer to the format string */
                             va_list arp) /* Pointer to arguments */
{
    unsigned int r, i, j, w, f;
    unsigned long v;
    char s[16], c, d, *p;

    for (;;) {
        c = *fmt++;                 /* Get a char */
        if (!c) break;              /* End of format? */
        if (c != '%') {             /* Pass through it if not a % sequense */
            kputc(c); continue;
        }
        f = 0;
        c = *fmt++;                 /* Get first char of the sequense */
        if (c == '0') {             /* Flag: '0' padded */
            f = 1; c = *fmt++;
        } else {
            if (c == '-') {         /* Flag: left justified */
                f = 2; c = *fmt++;
            }
        }
        for (w = 0; c >= '0' && c <= '9'; c = *fmt++)   /* Minimum width */
            w = w * 10 + c - '0';
        if (c == 'l' || c == 'L') { /* Prefix: Size is long int */
            f |= 4; c = *fmt++;
        }
        if (!c) break;              /* End of format? */
        d = c;
        if (d >= 'a') d -= 0x20;
        switch (d) {                /* Type is... */
        case 'S' :                  /* String */
            p = va_arg(arp, char*);
            for (j = 0; p[j]; j++) ;
            while (!(f & 2) && j++ < w) kputc(' ');
            kputs(p);
            while (j++ < w) kputc(' ');
            continue;
        case 'C' :                  /* Character */
            kputc((char)va_arg(arp, int)); continue;
        case 'B' :                  /* Binary */
            r = 2; break;
        case 'O' :                  /* Octal */
            r = 8; break;
        case 'D' :                  /* Signed decimal */
        case 'U' :                  /* Unsigned decimal */
            r = 10; break;
        case 'X' :                  /* Hexdecimal */
            r = 16; break;
        default:                    /* Unknown type (passthrough) */
            kputc(c); continue;
        }

        /* Get an argument and put it in numeral */
        v = (f & 4) ? va_arg(arp, long) : ((d == 'D') ? (long)va_arg(arp, int) : (long)va_arg(arp, unsigned int));
        if (d == 'D' && (v & 0x80000000)) {
            v = 0 - v;
            f |= 8;
        }
        i = 0;
        do {
            d = (char)(v % r); v /= r;
            if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
            s[i++] = d + '0';
        } while (v && i < sizeof(s));
        if (f & 8) s[i++] = '-';
        j = i; d = (f & 1) ? '0' : ' ';
        while (!(f & 2) && j++ < w) kputc(d);
        do {kputc(s[--i]);} while(i);
        while (j++ < w) kputc(' ');
    }
}

/* Optional arguments */
/* Put a formatted string to the default device */
/* Pointer to the format string */
void xprintf(const char* fmt, ...)
{
    va_list arp;
    va_start(arp, fmt);
    xvprintf(fmt, arp);
    va_end(arp);
}
 
