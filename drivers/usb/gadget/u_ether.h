/*
 * u_ether.h -- interface to USB gadget "ethernet link" utilities
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __U_ETHER_H
#define __U_ETHER_H

#include <linux/err.h>
#include <linux/if_ether.h>
#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>

#include "gadget_chips.h"

struct eth_dev;

struct gether {
	struct usb_function		func;

	
	struct eth_dev			*ioport;

	
	struct usb_ep			*in_ep;
	struct usb_ep			*out_ep;

	bool				is_zlp_ok;

	u16				cdc_filter;

	
	u32				header_len;
	
	bool				is_fixed;
	u32				fixed_out_len;
	u32				fixed_in_len;

	unsigned			ul_max_pkts_per_xfer;
	unsigned			dl_max_pkts_per_xfer;
	unsigned			dl_max_transfer_len;
	bool				multi_pkt_xfer;
	struct sk_buff			*(*wrap)(struct gether *port,
						struct sk_buff *skb);
	int				(*unwrap)(struct gether *port,
						struct sk_buff *skb,
						struct sk_buff_head *list);

	
	void				(*open)(struct gether *);
	void				(*close)(struct gether *);
	struct rndis_packet_msg_type	*header;
};

#define	DEFAULT_FILTER	(USB_CDC_PACKET_TYPE_BROADCAST \
			|USB_CDC_PACKET_TYPE_ALL_MULTICAST \
			|USB_CDC_PACKET_TYPE_PROMISCUOUS \
			|USB_CDC_PACKET_TYPE_DIRECTED)

struct eth_dev *gether_setup_name(struct usb_gadget *g, u8 ethaddr[ETH_ALEN],
		const char *netname);

static inline struct eth_dev *gether_setup(struct usb_gadget *g,
		u8 ethaddr[ETH_ALEN])
{
	return gether_setup_name(g, ethaddr, "usb");
}

void gether_cleanup(struct eth_dev *dev);

struct net_device *gether_connect(struct gether *);
void gether_disconnect(struct gether *);

static inline bool can_support_ecm(struct usb_gadget *gadget)
{
	if (!gadget_supports_altsettings(gadget))
		return false;

	return true;
}

int geth_bind_config(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		struct eth_dev *dev);
int ecm_bind_config(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		struct eth_dev *dev);
int ncm_bind_config(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		struct eth_dev *dev);
int eem_bind_config(struct usb_configuration *c, struct eth_dev *dev);

#ifdef USB_ETH_RNDIS

int rndis_bind_config_vendor(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		u32 vendorID, const char *manufacturer, struct eth_dev *dev);

#else

static inline int
rndis_bind_config_vendor(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		u32 vendorID, const char *manufacturer, struct eth_dev *dev)
{
	return 0;
}

#endif

static inline int rndis_bind_config(struct usb_configuration *c,
		u8 ethaddr[ETH_ALEN], struct eth_dev *dev)
{
	return rndis_bind_config_vendor(c, ethaddr, 0, NULL, dev);
}


#endif 
