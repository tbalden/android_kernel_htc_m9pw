/*
 *  Copyright (C) 2012-2013 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#ifndef _EXFAT_OAL_H
#define _EXFAT_OAL_H

#include <linux/semaphore.h>
#include "exfat_config.h"
#include <linux/version.h>




typedef struct {
	u16      sec;        
	u16      min;        
	u16      hour;       
	u16      day;        
	u16      mon;        
	u16      year;       
} TIMESTAMP_T;


s32 sm_init(struct semaphore *sm);
s32 sm_P(struct semaphore *sm);
void  sm_V(struct semaphore *sm);

TIMESTAMP_T *tm_current(TIMESTAMP_T *tm);

#endif 
