/*
 * u_serial.h - interface to USB gadget "serial port"/TTY utilities
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#ifndef __U_SERIAL_H
#define __U_SERIAL_H

#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>
#define MAX_U_SERIAL_PORTS	16

struct f_serial_opts {
	struct usb_function_instance func_inst;
	u8 port_num;
};

struct gserial {
	struct usb_function		func;

	
	struct gs_port			*ioport;

	struct usb_ep			*in;
	struct usb_ep			*out;

	
	struct usb_cdc_line_coding port_line_coding;	

	
	void (*connect)(struct gserial *p);
	void (*disconnect)(struct gserial *p);
	int (*send_break)(struct gserial *p, int duration);
};

struct usb_request *gs_alloc_req(struct usb_ep *ep, unsigned len, gfp_t flags);
void gs_free_req(struct usb_ep *, struct usb_request *req);

int gserial_alloc_line(unsigned char *port_line);
void gserial_free_line(unsigned char port_line);

int gserial_connect(struct gserial *, u8 port_num);
void gserial_disconnect(struct gserial *);

int gser_bind_config(struct usb_configuration *c, u8 port_num);
int obex_bind_config(struct usb_configuration *c, u8 port_num);

#endif 
