/* Copyright (c) 2013, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __HTC_BOOTLOADER_LOG_H__
#define __HTC_BOOTLOADER_LOG_H__

#include <linux/types.h>

extern char *bl_last_log_buf;
extern char *bl_cur_log_buf;
extern unsigned long bl_last_log_buf_size;
extern unsigned long bl_cur_log_buf_size;

ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size, char __user *userbuf,
		size_t count, loff_t *ppos);
int bldr_log_init(void);
void bldr_log_release(void);

#endif 
