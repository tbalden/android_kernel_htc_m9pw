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


#ifndef _EXFAT_CONFIG_H
#define _EXFAT_CONFIG_H


#ifndef CONFIG_EXFAT_DISCARD
#define CONFIG_EXFAT_DISCARD		1	
#endif

#ifndef CONFIG_EXFAT_DELAYED_SYNC
#define CONFIG_EXFAT_DELAYED_SYNC 0
#endif

#ifndef CONFIG_EXFAT_KERNEL_DEBUG
#define CONFIG_EXFAT_KERNEL_DEBUG	1	
#endif

#ifndef CONFIG_EXFAT_DEBUG_MSG
#define CONFIG_EXFAT_DEBUG_MSG		0	
#endif

#ifndef CONFIG_EXFAT_DEFAULT_CODEPAGE
#define CONFIG_EXFAT_DEFAULT_CODEPAGE	437
#define CONFIG_EXFAT_DEFAULT_IOCHARSET	"utf8"
#endif

#endif 
