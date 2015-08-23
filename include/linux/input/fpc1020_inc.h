/* include/linux/fpr1020_inc.h
 *
 * Copyright (c)2015 Illes Pal Zoltan
 *
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

#ifndef __FPR1020_INC_H
#define __FPR1020_INC_H

//#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_SWEEP2WAKE

/* power on/off input device */
// set device for the fingerprint capture part
extern void power_onoff_setdev_capture(struct input_dev * input_device);
// set device for the fingerprint input part
extern void power_onoff_setdev_input(struct input_dev * input_device);

//#endif

#endif 

