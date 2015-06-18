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
#ifndef __HTC_RECEIVER_H__
#define __HTC_RECEIVER_H__

#include <linux/types.h>

int htc_receiver_init(void);
void htc_receiver_release(void);
void setReceiverSelect(bool enable);
#endif 
