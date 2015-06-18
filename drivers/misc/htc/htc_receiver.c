/* Copyright (c) 2014, HTC Corporation. All rights reserved.
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
#include <linux/of.h>
#include <mach/mt_gpio.h>

#define AUD_RECEIVER_NAME		"/htc_receiver"
#define AUD_BOARD_INFO_PATH    "/htc_aud_board_info"

static int select_gpio = -1;
static int enable_gpio = -1;

static void setReceiverPower(bool enable) {
	unsigned long local_enable_gpio = 0;
	if (enable_gpio < 0) {
		pr_err("setReceiverPower Error, cannot find the gpio, enable %d \n ", enable);
		return;
	}
	local_enable_gpio = (enable_gpio | 0x80000000);
	printk("setReceiverPower %d, gpio %d \n ", enable, enable_gpio);

	mt_set_gpio_mode(local_enable_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(local_enable_gpio, GPIO_DIR_OUT);
	if (enable)
		mt_set_gpio_out(local_enable_gpio, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(local_enable_gpio, GPIO_OUT_ZERO);
}

void setReceiverSelect(bool enable) {
	unsigned long local_select_gpio = 0;
	if (select_gpio < 0) {
		pr_err("setReceiverPower Error, cannot find the gpio, enable %d \n ", enable);
		return;
	}
	
	if ((enable_gpio >= 0) && enable)
		setReceiverPower(enable);

	local_select_gpio = (select_gpio | 0x80000000);
	printk("setReceiverSelect %d, gpio %d \n ", enable, select_gpio);
	mt_set_gpio_mode(local_select_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(local_select_gpio, GPIO_DIR_OUT);
	if (enable)
		mt_set_gpio_out(local_select_gpio, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(local_select_gpio, GPIO_OUT_ZERO);

	
	if ((enable_gpio >= 0) && !enable)
		setReceiverPower(enable);
}

int htc_receiver_init(void) {
	struct device_node *receiver_node;
	struct property *prop;

	receiver_node = of_find_node_by_path(AUD_RECEIVER_NAME);
	prop = of_find_property(receiver_node, "htc_aud,select-gpio", NULL);
	if (prop) {
		of_property_read_u32(receiver_node, "htc_aud,select-gpio", (u32*)&select_gpio);
		printk("%s: reciver_select = %d\n", __func__, select_gpio);
	} else {
		printk("%s: reciver_select not found", __func__);
		select_gpio = -1;
	}

	prop = of_find_property(receiver_node, "htc_aud,enable-gpio", NULL);
	if (prop) {
		of_property_read_u32(receiver_node, "htc_aud,enable-gpio", (u32*)&enable_gpio);
		printk("%s: reciver_enable = %d\n", __func__, enable_gpio);
	} else {
		printk("%s: reciver_enable not found", __func__);
		enable_gpio = -1;
	}
	return 0;
}

void htc_receiver_release(void) {
	select_gpio = -1;
	enable_gpio = -1;
}
