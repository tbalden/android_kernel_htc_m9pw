/* include/linux/i2c/tca6418_ioexpander.h
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define CAM_I2C_ADDR	0
#define CAM2_ID		1
#define MAIN_CAM_ID	2
#define FRONT_CAM_ID	3
#define LCD_ID1		4
#define LCD_ID0		5
#define FLASH_EN	6
#define TORCH_FLASH	7
#define CIR_3V3_EN	8
#define CIR_LS_EN	9
#define CIR_RST		10
#define CAM2_A2V9_EN	11
#define CAM2_D1V2_EN	12
#define CAM2_RST	13

enum {
	IOEXP_INPUT,
	IOEXP_OUTPUT,
};

enum {
	IOEXP_PULLDOWN,
	IOEXP_NOPULL,
};

#ifndef _LINUX_ATMEGA_MICROP_H
#define _LINUX_ATMEGA_MICROP_H

int ioexp_gpio_set_value(uint8_t gpio, uint8_t value);
int ioexp_gpio_get_value(uint8_t gpio);
int ioexp_gpio_get_direction(uint8_t gpio);
int ioexp_read_gpio_status(uint8_t *data);
int ioexp_gpio_set_cfg(uint8_t gpio, uint8_t direction, uint8_t pulldown_disable, uint8_t level);
void ioexp_print_gpio_status(void);

#endif 

