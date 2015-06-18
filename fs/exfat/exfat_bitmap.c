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


#include "exfat_config.h"
#include "exfat_bitmap.h"


#define BITMAP_LOC(v)           ((v) >> 3)
#define BITMAP_SHIFT(v)         ((v) & 0x07)

s32 exfat_bitmap_test(u8 *bitmap, int i)
{
	u8 data;

	data = bitmap[BITMAP_LOC(i)];
	if ((data >> BITMAP_SHIFT(i)) & 0x01)
		return 1;
	return 0;
} 

void exfat_bitmap_set(u8 *bitmap, int i)
{
	bitmap[BITMAP_LOC(i)] |= (0x01 << BITMAP_SHIFT(i));
} 

void exfat_bitmap_clear(u8 *bitmap, int i)
{
	bitmap[BITMAP_LOC(i)] &= ~(0x01 << BITMAP_SHIFT(i));
} 
