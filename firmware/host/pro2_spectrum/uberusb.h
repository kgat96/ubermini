/*
 * This file is part of the ubertoothp project.
 *
 * Copyright (C) 2016 Kage Shen <kgat96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __UBERUSB_H
#define __UBERUSB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int uberusb(void);
int uberopen(void);
void uberclose(void);
void uber_handle_events(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __UBERUSB_H */
