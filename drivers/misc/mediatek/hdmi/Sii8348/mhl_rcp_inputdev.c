/*

SiI8348 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See 
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.             

*/

#include "sii_hal.h"

#ifdef RCP_INPUTDEV_SUPPORT


#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include "si_fw_macros.h"
#include "si_mhl_defs.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "platform.h"

static u16 rcp_def_keymap[] = {
	KEY_ENTER,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_HOMEPAGE,
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_BACK,
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_0,
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_6,
	KEY_7,
	KEY_8,
	KEY_9,
	KEY_DOT,
	KEY_ENTER,
	KEY_CLEAR,
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_SOUND,	    
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_RESERVED,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_PLAY,
	KEY_STOP,
	KEY_PAUSECD,
	KEY_UNKNOWN,	
	KEY_REWIND,
	KEY_FASTFORWARD,
	KEY_EJECTCD,	
	KEY_NEXTSONG,
	KEY_PREVIOUSSONG,
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_PLAY,
	KEY_PAUSE,
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_STOP,
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_UNKNOWN,	
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_F1,	
	KEY_F2,	
	KEY_F3,	
	KEY_F4,	
	KEY_F5,	
	KEY_RESERVED,	
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	
	KEY_VENDOR,
	KEY_RESERVED,	
};

int generate_rcp_input_event(struct mhl_dev_context *dev_context,
							 uint8_t rcp_keycode)
{
	int	status = -EINVAL;

	if (dev_context->rcp_input_dev == NULL)
	{
	    MHL_TX_DBG_ERR(dev_context, "RCP input device not exists!\n");
	    goto exit;
    }

	if (rcp_keycode < ARRAY_SIZE(rcp_def_keymap) &&
			rcp_def_keymap[rcp_keycode] != KEY_UNKNOWN &&
			rcp_def_keymap[rcp_keycode] != KEY_RESERVED) {

		input_report_key(dev_context->rcp_input_dev, rcp_def_keymap[rcp_keycode], 1);
		input_report_key(dev_context->rcp_input_dev, rcp_def_keymap[rcp_keycode], 0);
		input_sync(dev_context->rcp_input_dev);

		status = 0;
	}

exit:
	return status;
}

uint8_t init_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	int i;
	uint8_t error;
	struct input_dev	*rcp_input_dev;

	if (dev_context->rcp_input_dev != NULL) {
		MHL_TX_DBG_INFO(dev_context, "RCP input device already exists!\n");
		return 0;
	}

	rcp_input_dev = input_allocate_device();
	if (!rcp_input_dev) {
		MHL_TX_DBG_ERR(dev_context, "Failed to allocate RCP input device\n");
		return -ENOMEM;
	}

	set_bit(EV_KEY, rcp_input_dev->evbit);

	rcp_input_dev->name			= "MHL Remote Control";
	rcp_input_dev->keycode		= rcp_def_keymap;
	rcp_input_dev->keycodesize	= sizeof(u16);
	rcp_input_dev->keycodemax	= ARRAY_SIZE(rcp_def_keymap);

	for (i = 1; i < ARRAY_SIZE(rcp_def_keymap); i++) {
		u16	keycode = rcp_def_keymap[i];
		if (keycode != KEY_UNKNOWN && keycode != KEY_RESERVED)
			set_bit(keycode, rcp_input_dev->keybit);
	}

	rcp_input_dev->id.bustype = BUS_VIRTUAL;

	error = input_register_device(rcp_input_dev);
	if (error) {
		MHL_TX_DBG_ERR(dev_context, "Failed to register device\n");
		input_free_device(rcp_input_dev);
		return error;
	}

	MHL_TX_DBG_INFO(dev_context, "device created\n");

	dev_context->rcp_input_dev = rcp_input_dev;

	return 0;
}

void destroy_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	if (dev_context->rcp_input_dev) {
		input_unregister_device(dev_context->rcp_input_dev);
		dev_context->rcp_input_dev = NULL;
	}
}

#endif 
