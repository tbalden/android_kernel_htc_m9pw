/*
 * MUSB OTG driver peripheral support
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 * Copyright (C) 2009 MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "musb_core.h"
#include <linux/mu3d/hal/mu3d_hal_osal.h>
#include <linux/mu3d/hal/mu3d_hal_qmu_drv.h>
#include <linux/mu3d/hal/mu3d_hal_usb_drv.h>
#include <linux/mu3d/hal/mu3d_hal_hw.h>
#include "ssusb_qmu.h"



#define is_buffer_mapped(req) (is_dma_capable() && \
					(req->map_state != UN_MAPPED))


static inline void map_dma_buffer(struct musb_request *request,
			struct musb *musb, struct musb_ep *musb_ep)
{
#ifndef USE_SSUSB_QMU
	int compatible = true;
	struct dma_controller *dma = musb->dma_controller;
#endif

	request->map_state = UN_MAPPED;

#ifndef USE_SSUSB_QMU
	if (!is_dma_capable() || !musb_ep->dma)
		return;

	if (dma->is_compatible)
		compatible = dma->is_compatible(musb_ep->dma,
				musb_ep->packet_sz, request->request.buf,
				request->request.length);
	if (!compatible)
		return;

#endif
	if (request->request.dma == DMA_ADDR_INVALID) {
		request->request.dma = dma_map_single(
				musb->controller,
				request->request.buf,
				request->request.length,
				request->tx
					? DMA_TO_DEVICE
					: DMA_FROM_DEVICE);
		request->map_state = MUSB_MAPPED;
	} else {
		dma_sync_single_for_device(musb->controller,
			request->request.dma,
			request->request.length,
			request->tx
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		request->map_state = PRE_MAPPED;
	}
}

static inline void unmap_dma_buffer(struct musb_request *request,
				struct musb *musb)
{
	if (!is_buffer_mapped(request))
		return;

	if (request->request.dma == DMA_ADDR_INVALID) {
		dev_vdbg(musb->controller,
				"not unmapping a never mapped buffer\n");
		return;
	}
	if (request->map_state == MUSB_MAPPED) {
		dma_unmap_single(musb->controller,
			request->request.dma,
			request->request.length,
			request->tx
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		request->request.dma = DMA_ADDR_INVALID;
	} else { 
		dma_sync_single_for_cpu(musb->controller,
			request->request.dma,
			request->request.length,
			request->tx
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
	}
	request->map_state = UN_MAPPED;
}

void musb_g_giveback(
	struct musb_ep		*ep,
	struct usb_request	*request,
	int			status)
__releases(ep->musb->lock)
__acquires(ep->musb->lock)
{
	struct musb_request	*req;
	struct musb		*musb;
	int			busy = ep->busy;

	if (unlikely(list_empty(&ep->req_list))) {
		os_printk(K_ERR, "ep->req_list is empty:%s\n", ep->end_point.name);
		return;
	}

	req = to_musb_request(request);

	list_del(&req->list);
	if (req->request.status == -EINPROGRESS)
		req->request.status = status;
	musb = req->musb;

	ep->busy = 1;
	spin_unlock(&musb->lock);
	unmap_dma_buffer(req, musb);
	if (request->status == 0)
		
		os_printk(K_DEBUG, "%s done request %p,  %d/%d\n",
				ep->end_point.name, request,
				req->request.actual, req->request.length);
	else
		
		os_printk(K_DEBUG, "%s request %p, %d/%d fault %d\n",
				ep->end_point.name, request,
				req->request.actual, req->request.length,
				request->status);

	os_printk(K_DEBUG, "*************** musb_g_giveback : %p, #%d\n", request, req->request.actual);
	req->request.complete(&req->ep->end_point, &req->request);
	spin_lock(&musb->lock);
	ep->busy = busy;
}


static void nuke(struct musb_ep *ep, const int status)
{
	struct musb_request	*req = NULL;

	os_printk(K_INFO, "%s status=%d %s-%s\n", __func__, status, ep->end_point.name,
					(ep->is_in ? "IN": "OUT"));

	ep->busy = 1;

#ifdef USE_SSUSB_QMU
	_ex_mu3d_hal_flush_qmu(ep->hw_ep->epnum, (ep->is_in? USB_TX: USB_RX));
	
#endif

	while (!list_empty(&ep->req_list)) {
		req = list_first_entry(&ep->req_list, struct musb_request, list);
		musb_g_giveback(ep, &req->request, status);
		os_printk(K_INFO, "%s call musb_g_giveback() EP is %s\n", __func__, ep->end_point.name);
	}
}




static inline int max_ep_writesize(struct musb *musb, struct musb_ep *ep)
{
	if (can_bulk_split(musb, ep->type))
		return ep->hw_ep->max_packet_sz_tx;
	else
		return ep->packet_sz;
}


#ifdef CONFIG_USB_INVENTRA_DMA


#endif

#ifndef USE_SSUSB_QMU

static void txstate(struct musb *musb, struct musb_request *req)
{
	u8			epnum = req->epnum;
	struct musb_ep		*musb_ep;
	struct usb_request	*request;
	u16			fifo_count = 0;
	u32 txcsr0 = 0, maxp;
	int			use_dma = 0;

	os_printk(K_DEBUG, "%s\n", __func__);

	musb_ep = req->ep;

	
	if (dma_channel_status(musb_ep->dma) == MUSB_DMA_STATUS_BUSY) {
		dev_dbg(musb->controller, "dma pending...\n");
		return;
	}

	
	txcsr0 = os_readl(musb->endpoints[epnum].addr_txcsr0);
	request = &req->request;
	fifo_count = min(max_ep_writesize(musb, musb_ep),
			(int)(request->length - request->actual));

	if (txcsr0 & TX_TXPKTRDY) {
		return;
	}

	if (txcsr0 & TX_SENDSTALL) {
		return;
	}


	if (!use_dma) {
		unmap_dma_buffer(req, musb);

		maxp = musb_ep->packet_sz;

		mu3d_hal_write_fifo(epnum,fifo_count,(u8 *) (request->buf + request->actual), maxp);
		request->actual += fifo_count;

		os_printk(K_DEBUG, "%s actual=%d, fifo_count=%d\n", __func__, request->actual, fifo_count);

#if 0
		musb_write_fifo(musb_ep->hw_ep, fifo_count,
				(u8 *) (request->buf + request->actual));
		request->actual += fifo_count;
		txcsr0 &= TX_W1C_BITS;
		txcsr0 |= TX_TXPKTRDY;
		os_writel(musb->endpoints[epnum].addr_txcsr0, txcsr0);
#endif
	}
}

#endif
void musb_g_tx(struct musb *musb, u8 epnum)
{
	u32			txcsr0;
	struct musb_request	*req;
	struct usb_request	*request;
	struct musb_ep		*musb_ep = &musb->endpoints[epnum].ep_in;
	struct dma_channel	*dma;

	req = next_request(musb_ep);
	request = &req->request;

	txcsr0 = os_readl(musb->endpoints[epnum].addr_txcsr0);

	dma = is_dma_capable() ? musb_ep->dma : NULL;
	if (dma_channel_status(dma) == MUSB_DMA_STATUS_BUSY) {
		dev_dbg(musb->controller, "%s dma still busy?\n", musb_ep->end_point.name);
		return;
	}

	if (request) {
		if ((request->zero && request->length
			&& (request->length % musb_ep->packet_sz == 0)
			&& (request->actual == request->length))
		) {
			if(txcsr0 & TX_TXPKTRDY)
				return;

			dev_dbg(musb->controller, "sending zero pkt\n");
			
			writel((txcsr0 & TX_W1C_BITS) | TX_TXPKTRDY, musb->endpoints[epnum].addr_txcsr0);
			request->zero = 0;
		}

		if (request->actual == request->length) {
			musb_g_giveback(musb_ep, request, 0);
			req = musb_ep->desc ? next_request(musb_ep) : NULL;
			if (!req) {
				dev_dbg(musb->controller, "%s idle now\n",
					musb_ep->end_point.name);
				return;
			}
		}

#ifdef USE_SSUSB_QMU
		
#else
		txstate(musb, req);
#endif
	}
}


#ifdef CONFIG_USB_INVENTRA_DMA


#endif

static void rxstate(struct musb *musb, struct musb_request *req)
{
	const u8		epnum = req->epnum;
	struct usb_request	*request = &req->request;
	struct musb_ep		*musb_ep;
	u16			fifo_count;
	unsigned		len = 0;
	u32 rxcsr0 = os_readl(musb->endpoints[epnum].addr_rxcsr0);
	struct musb_hw_ep	*hw_ep = &musb->endpoints[epnum];

	musb_ep = &hw_ep->ep_out;

	fifo_count = musb_ep->packet_sz;

	
	if (dma_channel_status(musb_ep->dma) == MUSB_DMA_STATUS_BUSY) {
		dev_dbg(musb->controller, "DMA pending...\n");
		return;
	}

	os_printk(K_DEBUG, "epnum=%d, rxcsr addr=%lX, rxcsr0=%X\n", epnum, (uintptr_t)(musb->endpoints[epnum].addr_rxcsr0), rxcsr0);

	if(rxcsr0 & RX_SENDSTALL) {
		return;
	}

	if (rxcsr0 & RX_RXPKTRDY) {
		fifo_count = (USB_ReadCsr32(U3D_RX1CSR3, epnum)>>EP_RX_COUNT_OFST);
		if (request->actual < request->length) {
			len = request->length - request->actual;
			fifo_count = min_t(unsigned, len, fifo_count);
			
			musb_read_fifo( &musb->endpoints[epnum], fifo_count, (request->buf + request->actual));
			request->actual += fifo_count;

			
#ifdef AUTOCLEAR
			if(!fifo_count){
				USB_WriteCsr32(U3D_RX1CSR0, epnum, USB_ReadCsr32(U3D_RX1CSR0, epnum) | RX_RXPKTRDY);
			}
#else
			USB_WriteCsr32(U3D_RX1CSR0, epnum, USB_ReadCsr32(U3D_RX1CSR0, epnum) | RX_RXPKTRDY);
#endif
		}
	}

	os_printk(K_DEBUG, "%s length=%d, actual=%d, fifo_count=%d\n", __func__, request->length, request->actual, fifo_count);
	os_printk(K_DEBUG, "%s len=%d, packet_sz=%d\n", __func__, len, musb_ep->packet_sz);

	
	if (request->actual == request->length || fifo_count < musb_ep->packet_sz)
		musb_g_giveback(musb_ep, request, 0);


#ifdef NEVER
	
	rxcsr0 = os_readl(musb->endpoints[epnum].addr_rxcsr0);
	os_printk(K_DEBUG, "rxcsr0 fifoempty=0x%x\n", rxcsr0 & RX_FIFOEMPTY);
	if (!(rxcsr0 & RX_FIFOEMPTY)) {
		printk("==READ AGAIN!!!==\n");
		musb_g_rx(musb, epnum);
	}
#endif 
}

void musb_g_rx(struct musb *musb, u8 epnum)
{
	u32			rxcsr0;
	struct musb_request	*req;
	struct usb_request	*request;
	struct musb_ep		*musb_ep;
	struct dma_channel	*dma;
	struct musb_hw_ep	*hw_ep = &musb->endpoints[epnum];

	musb_ep = &hw_ep->ep_out;

	req = next_request(musb_ep);
	if (!req)
		return;

	request = &req->request;

	rxcsr0 = os_readl(musb->endpoints[epnum].addr_rxcsr0);

	if (rxcsr0 & RX_SENTSTALL) {

		

		
		
		writel((rxcsr0 & RX_W1C_BITS) | RX_SENTSTALL, musb->endpoints[epnum].addr_rxcsr0);

		return;
	}


	dma = is_dma_capable() ? musb_ep->dma : NULL;

	
	rxstate(musb, req);
}


static int musb_gadget_enable(struct usb_ep *ep,
			const struct usb_endpoint_descriptor *desc)
{
	unsigned long		flags;
	struct musb_ep		*musb_ep;
	struct musb_hw_ep	*hw_ep;
	struct musb		*musb;
	void __iomem	*mbase;
	u8		epnum = 0;
	unsigned	maxp = 0;
	int		status = -EINVAL;
	TRANSFER_TYPE type = USB_CTRL;
	USB_DIR dir = USB_TX;


	if (!ep || !desc)
		return -EINVAL;

	os_printk(K_INFO, "musb_gadget_enable %s\n", ep->name);

	musb_ep = to_musb_ep(ep);
	hw_ep = musb_ep->hw_ep;
	musb = musb_ep->musb;
	mbase = musb->mregs;
	epnum = hw_ep->epnum;

	if (!musb->is_active)
		return -EINVAL;

	spin_lock_irqsave(&musb->lock, flags);

	if (musb_ep->desc) {
		status = -EBUSY;
		goto fail;
	}
	musb_ep->type = usb_endpoint_type(desc);

	
	if (usb_endpoint_num(desc) != epnum)
		goto fail;

		
	maxp = le16_to_cpu(desc->wMaxPacketSize);
	if (maxp & ~0x07ff)
		goto fail;

	musb_ep->packet_sz = maxp;

	os_printk(K_DEBUG, "U3D_EPIER=0x%X\n", os_readl(U3D_EPIER));

	if (usb_endpoint_dir_in(desc)) {


#ifndef USE_SSUSB_QMU
		u32 int_txe = os_readl(U3D_EPIER);
#endif
		if (hw_ep->is_shared_fifo)
			musb_ep->is_in = 1;
		if (!musb_ep->is_in)
			goto fail;

		if (maxp > hw_ep->max_packet_sz_tx)
			goto fail;

#ifndef USE_SSUSB_QMU
		os_printk(K_DEBUG, "epnum=%d, int_txe=0x%x, EPIER=0x%x+\n", epnum, int_txe, os_readl(U3D_EPIER));
		int_txe |= (1 << epnum);
		os_writel(U3D_EPIESR, int_txe);
		os_printk(K_DEBUG, "epnum=%d, int_txe=0x%x, EPIER=0x%x-\n", epnum, int_txe, os_readl(U3D_EPIER));
#endif
		dir = USB_TX;

		switch(musb_ep->type)
		{
			case USB_ENDPOINT_XFER_BULK:
				type = USB_BULK;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				type = USB_ISO;
				break;
			case USB_ENDPOINT_XFER_INT:
				type = USB_INTR;
				break;
		}
	} else {
#ifndef USE_SSUSB_QMU
		u32 int_rxe = os_readl(U3D_EPIER);
#endif
		if (hw_ep->is_shared_fifo)
			musb_ep->is_in = 0;
		if (musb_ep->is_in)
			goto fail;

		if (maxp > hw_ep->max_packet_sz_rx) {
			dev_dbg(musb->controller, "packet size beyond hardware FIFO size\n");
			goto fail;
		}

#ifndef USE_SSUSB_QMU
		os_printk(K_DEBUG, "int_rxe=0x%x, EPIER=0x%x+\n", int_rxe, os_readl(U3D_EPIER));
		int_rxe |= (BIT16 << epnum);
		os_writel(U3D_EPIESR, int_rxe);
		os_printk(K_DEBUG, "int_rxe=0x%x, EPIER=0x%x-\n", int_rxe, os_readl(U3D_EPIER));
#endif
		dir = USB_RX;

		switch(musb_ep->type)
		{
			case USB_ENDPOINT_XFER_BULK:
				type = USB_BULK;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				type = USB_ISO;
				break;
			case USB_ENDPOINT_XFER_INT:
				type = USB_INTR;
				break;
		}

	}

#ifdef USE_SSUSB_QMU
	_ex_mu3d_hal_ep_enable(epnum, dir, type, maxp, 0, MAX_SLOT, 0, 0);
#else
	
	_ex_mu3d_hal_ep_enable(epnum, dir, type, maxp, 0, 0, 0, 0);
#endif

#ifdef USE_SSUSB_QMU
	mu3d_hal_start_qmu(epnum, dir);
#endif
	if (is_dma_capable() && musb->dma_controller) {
		struct dma_controller	*c = musb->dma_controller;

		musb_ep->dma = c->channel_alloc(c, hw_ep,
				(desc->bEndpointAddress & USB_DIR_IN));
	} else
		musb_ep->dma = NULL;

	musb_ep->desc = desc;
	musb_ep->busy = 0;
	musb_ep->wedged = 0;
	status = 0;

	musb->active_ep++;
	os_printk(K_INFO, "[U3D]%s active_ep=%d\n", __func__, musb->active_ep);

	
	os_printk(K_INFO, "[U3D]%s periph: enabled %s for %s %s, %smaxpacket %d\n",
			musb_driver_name, musb_ep->end_point.name,
			({ char *s; switch (musb_ep->type) {
			case USB_ENDPOINT_XFER_BULK:	s = "bulk"; break;
			case USB_ENDPOINT_XFER_INT: s = "int"; break;
			default:			s = "iso"; break;
			}; s; }),
			musb_ep->is_in ? "IN" : "OUT",
			musb_ep->dma ? "dma, " : "",
			musb_ep->packet_sz);

	schedule_work(&musb->irq_work);

	fail:
		spin_unlock_irqrestore(&musb->lock, flags);

	return status;
}

static int musb_gadget_disable(struct usb_ep *ep)
{
	unsigned long	flags;
	struct musb	*musb;
	u8		epnum;
	struct musb_ep	*musb_ep;
	int		status = 0;

	os_printk(K_INFO, "%s %s\n", __func__, ep->name);

	musb_ep = to_musb_ep(ep);
	musb = musb_ep->musb;
	epnum = musb_ep->current_epnum;

	spin_lock_irqsave(&musb->lock, flags);

#ifdef USE_SSUSB_QMU
	
	if (musb_ep->is_in) {
		os_writelmskumsk(musb->endpoints[epnum].addr_rxcsr0, 0, TX_TXMAXPKTSZ, TX_W1C_BITS);
	} else {
		os_writelmskumsk(musb->endpoints[epnum].addr_rxcsr0, 0, RX_RXMAXPKTSZ, RX_W1C_BITS);
	}
#else
	
	if (musb_ep->is_in) { 
		u32 int_txe = os_readl(U3D_EPIER);
		int_txe &= ~(1 << epnum);
        os_writel(U3D_EPIESR, int_txe);
		os_writelmskumsk(musb->endpoints[epnum].addr_rxcsr0, 0, TX_TXMAXPKTSZ, TX_W1C_BITS);
	} else {
		u32 int_rxe = os_readl(U3D_EPIER);
		int_rxe &= ~(BIT16 << epnum);
		os_writel(U3D_EPIESR, int_rxe);
		os_writelmskumsk(musb->endpoints[epnum].addr_rxcsr0, 0, RX_RXMAXPKTSZ, RX_W1C_BITS);
	}
#endif
	musb_ep->desc = NULL;

	
	nuke(musb_ep, -ESHUTDOWN);

	mu3d_hal_unfigured_ep_num(epnum, (musb_ep->is_in? USB_TX: USB_RX));

	schedule_work(&musb->irq_work);

	musb->active_ep--;
	os_printk(K_INFO, "[U3D]%s active_ep=%d\n", __func__, musb->active_ep);

	if (musb->active_ep == 0 && musb->is_active == 0)
		schedule_work(&musb->suspend_work);

	spin_unlock_irqrestore(&(musb->lock), flags);

	dev_dbg(musb->controller, "%s\n", musb_ep->end_point.name);

	return status;
}

struct usb_request *musb_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	struct musb		*musb = musb_ep->musb;
	struct musb_request	*request = NULL;

	request = kzalloc(sizeof *request, gfp_flags);
	if (!request) {
		dev_dbg(musb->controller, "not enough memory\n");
		return NULL;
	}

	request->request.dma = DMA_ADDR_INVALID;
	request->epnum = musb_ep->current_epnum;
	request->ep = musb_ep;

	return &request->request;
}

void musb_free_request(struct usb_ep *ep, struct usb_request *req)
{
	kfree(to_musb_request(req));
}

static LIST_HEAD(buffers);

struct free_record {
	struct list_head	list;
	struct device		*dev;
	unsigned		bytes;
	dma_addr_t		dma;
};

void musb_ep_restart(struct musb *musb, struct musb_request *req)
{
#ifndef USE_SSUSB_QMU
	
	os_printk(K_DEBUG, "%s %s request %p len %u on hw_ep%d\n",
		__func__, req->tx ? "TX/IN" : "RX/OUT",
		&req->request, req->request.length, req->epnum);

	if (req->tx)
		txstate(musb, req);
	else
		rxstate(musb, req);
#endif
}

#ifdef EP_PROFILING
extern u32 is_prof;
extern unsigned int ep_prof[8][2];
#endif

static int musb_gadget_queue(struct usb_ep *ep, struct usb_request *req,
			gfp_t gfp_flags)
{
	struct musb_ep		*musb_ep;
	struct musb_request	*request;
	struct musb		*musb;
	int			status = 0;
	unsigned long		lockflags;

	if (!ep || !req)
		return -EINVAL;
	if (!req->buf)
		return -ENODATA;

	musb_ep = to_musb_ep(ep);
	musb = musb_ep->musb;

	request = to_musb_request(req);
	request->musb = musb;

	if (request->ep != musb_ep)
		return -EINVAL;

	os_printk(K_DEBUG, "%s %s, req=%p, len#%d\n", __func__, ep->name, req, request->request.length);

	dev_dbg(musb->controller, "<== to %s request=%p\n", ep->name, req);

	
	request->request.actual = 0;
	request->request.status = -EINPROGRESS;
	request->epnum = musb_ep->current_epnum;
	request->tx = musb_ep->is_in;

#ifdef CONFIG_MTK_LM_MODE
	if ( request->tx && (request->request.length == 0) )
		request->map_state = MUSB_MAPPED;
	else
		map_dma_buffer(request, musb, musb_ep);
#else
	map_dma_buffer(request, musb, musb_ep);
#endif


	spin_lock_irqsave(&musb->lock, lockflags);

	
	if (!musb_ep->desc) {
		dev_dbg(musb->controller, "req %p queued to %s while ep %s\n",
				req, ep->name, "disabled");
		status = -ESHUTDOWN;
		goto cleanup;
	}

	
	list_add_tail(&request->list, &musb_ep->req_list);

#ifdef EP_PROFILING
	if (is_prof!=0) ep_prof[request->epnum-1][request->tx?0:1] += request->request.length;
#endif

#ifdef USE_SSUSB_QMU
#ifdef CONFIG_MTK_LM_MODE
	if ((request->request.dma != DMA_ADDR_INVALID) || (request->map_state != UN_MAPPED))
#else
	if (request->request.dma != DMA_ADDR_INVALID)
#endif
	{
		qmu_printk(K_DEBUG, "%s %s EP%d, len=%d, maxp=%d request=%p\n",
			request->tx?"[TX]":"[RX]", __func__, request->epnum,
			request->request.length, ep->maxpacket, request);
		if(request->tx)
		{
			request->request.actual = request->request.length;
			if(request->request.length > 0) {
				u32 txcsr;

				_ex_mu3d_hal_insert_transfer_gpd(request->epnum, USB_TX,
					request->request.dma, request->request.length,
					true, true, false, ((request->request.zero==1)?1:0),
					ep->maxpacket);

				
				txcsr = USB_ReadCsr32(U3D_TX1CSR0, request->epnum) | TX_DMAREQEN;

				mb();

				USB_WriteCsr32(U3D_TX1CSR0, request->epnum, txcsr);

				mu3d_hal_resume_qmu(request->epnum, USB_TX);

			} else if(request->request.length == 0) {
			
				int utime = 1000000; 
				
				u32 val = 0;

				qmu_printk(K_DEBUG, "[TX]""Send ZLP\n");
				if (wait_for_value_us(USB_END_OFFSET(request->epnum, U3D_TX1CSR0), TX_FIFOEMPTY, TX_FIFOEMPTY, 1, utime) == RET_SUCCESS) {
					qmu_printk(K_DEBUG, "Tx[%d] 0x%x\n", request->epnum, USB_ReadCsr32(U3D_TX1CSR0, request->epnum));
				} else {
					qmu_printk(K_CRIT, "Tx[%d] Fail to send ZLP 0x%x, utime %d, skip and wait until QMU Done\n", request->epnum, USB_ReadCsr32(U3D_TX1CSR0, request->epnum), utime);
					goto cleanup;
				}

				
				val = USB_ReadCsr32(U3D_TX1CSR0, request->epnum) & ~TX_DMAREQEN;

				mb();

				USB_WriteCsr32(U3D_TX1CSR0, request->epnum, val);

				val = USB_ReadCsr32(U3D_TX1CSR0, request->epnum) | TX_TXPKTRDY;

				mb();

				USB_WriteCsr32(U3D_TX1CSR0, request->epnum, val);

				qmu_printk(K_DEBUG, "[TX]""Give back ZLP of EP%d. actual:%d, length:%d %p\n", \
					request->epnum, request->request.actual, \
					request->request.length, request);

				
				musb_g_giveback(musb_ep, &(request->request), 0);
			}
		} else {
			_ex_mu3d_hal_insert_transfer_gpd(request->epnum, USB_RX, request->request.dma, request->request.length, true,true,false,(musb_ep->type==USB_ENDPOINT_XFER_ISOC?0:1), ep->maxpacket);
			mu3d_hal_resume_qmu(request->epnum, USB_RX);
		}
	}
#endif
	
	if (!musb_ep->busy && &request->list == musb_ep->req_list.next)
		musb_ep_restart(musb, request);

cleanup:
	spin_unlock_irqrestore(&musb->lock, lockflags);
	return status;
}

static int musb_gadget_dequeue(struct usb_ep *ep, struct usb_request *request)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	struct musb_request	*req = to_musb_request(request);
	struct musb_request	*r;
	unsigned long		flags;
	int			status = 0;
	struct musb		*musb = musb_ep->musb;

	if (!ep || !request || to_musb_request(request)->ep != musb_ep)
		return -EINVAL;

	os_printk(K_INFO, "%s : request 0x%p\n", __func__, request);

	spin_lock_irqsave(&musb->lock, flags);

	list_for_each_entry(r, &musb_ep->req_list, list) {
		if (r == req)
			break;
	}
	if (r != req) {
		dev_dbg(musb->controller, "request %p not queued to %s\n", request, ep->name);
		status = -EINVAL;
		goto done;
	}

	
	if (musb_ep->req_list.next != &req->list || musb_ep->busy)
		musb_g_giveback(musb_ep, request, -ECONNRESET);

	

#ifdef USE_SSUSB_QMU
	else {
		_ex_mu3d_hal_flush_qmu(musb_ep->hw_ep->epnum, (musb_ep->is_in? USB_TX: USB_RX));

		musb_g_giveback(musb_ep, request, -ECONNRESET);

		
		
		mu3d_hal_start_qmu(musb_ep->hw_ep->epnum, (musb_ep->is_in? USB_TX: USB_RX));

		status = 0;
	}
#else
	else if (is_dma_capable() && musb_ep->dma) {
		struct dma_controller	*c = musb->dma_controller;

		if (c->channel_abort)
			status = c->channel_abort(musb_ep->dma);
		else
			status = -EBUSY;
		if (status == 0)
			musb_g_giveback(musb_ep, request, -ECONNRESET);
	} else {
		musb_g_giveback(musb_ep, request, -ECONNRESET);
	}
#endif
done:
	spin_unlock_irqrestore(&musb->lock, flags);
	return status;
}

static int musb_gadget_set_halt(struct usb_ep *ep, int value)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	u8			epnum = musb_ep->current_epnum;
	struct musb		*musb = musb_ep->musb;
	void __iomem		*mbase;
	unsigned long		flags;
	u32 txcsr0 = 0, rxcsr0 = 0;
	struct musb_request	*request;
	int			status = 0;

	if (!ep)
		return -EINVAL;
	mbase = musb->mregs;

	os_printk(K_DEBUG, "musb_gadget_set_halt : %s...", ep->name);
	spin_lock_irqsave(&musb->lock, flags);

	if ((USB_ENDPOINT_XFER_ISOC == musb_ep->type)) {
		status = -EINVAL;
		goto done;
	}


	request = next_request(musb_ep);
	if (value) {
		if (request) {
			dev_dbg(musb->controller, "request in progress, cannot halt %s\n",
			    ep->name);
			status = -EAGAIN;
			goto done;
		}
		
		if (musb_ep->is_in) {
            txcsr0 = os_readl(musb->endpoints[epnum].addr_txcsr0);
			if(!(txcsr0 & TX_FIFOEMPTY)) {
				dev_dbg(musb->controller, "FIFO busy, cannot halt %s\n", ep->name);
				status = -EAGAIN;
				goto done;
			}
		}
	} else
		musb_ep->wedged = 0;

	
	dev_dbg(musb->controller, "%s: %s stall\n", ep->name, value ? "set" : "clear");
	if (musb_ep->is_in) {
		txcsr0 = os_readl(musb->endpoints[epnum].addr_txcsr0) & TX_W1C_BITS;

 		if (value) 
		{
			txcsr0 |= TX_SENDSTALL;
			
			writel(txcsr0, musb->endpoints[epnum].addr_txcsr0);
		}
		else 
		{
			txcsr0 = (txcsr0 & (~TX_SENDSTALL)) | TX_SENTSTALL; 
			
			writel(txcsr0, musb->endpoints[epnum].addr_txcsr0);

			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) | (BIT16<<epnum));
			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) & ~(BIT16<<epnum));

			
			
			
 		}

	} else {
		rxcsr0 = os_readl(musb->endpoints[epnum].addr_rxcsr0) & RX_W1C_BITS;

		if (value) 
		{
			rxcsr0 &= RX_W1C_BITS;
			rxcsr0 |= RX_SENDSTALL;
			
			writel(rxcsr0, musb->endpoints[epnum].addr_rxcsr0);
		}
		else 
		{
			rxcsr0 = (rxcsr0 & (~RX_SENDSTALL)) | RX_SENTSTALL; 
			
			writel(rxcsr0, musb->endpoints[epnum].addr_rxcsr0);

			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) | (1 << epnum));
			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) & (~(1 << epnum)));
			
			
			
		}
	}


	
	if (!musb_ep->busy && !value && request) {
		dev_dbg(musb->controller, "restarting the request\n");
		musb_ep_restart(musb, request);
	}

done:
	spin_unlock_irqrestore(&musb->lock, flags);
	return status;
}

static int musb_gadget_set_wedge(struct usb_ep *ep)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);

	if (!ep)
		return -EINVAL;

	musb_ep->wedged = 1;

	return usb_ep_set_halt(ep);
}

static int musb_gadget_fifo_status(struct usb_ep *ep)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	int			retval = -EINVAL;

	if (musb_ep->desc && !musb_ep->is_in) {
		struct musb		*musb = musb_ep->musb;
		int			epnum = musb_ep->current_epnum;
		unsigned long		flags;

		spin_lock_irqsave(&musb->lock, flags);

		
		retval = os_readl(musb->endpoints[epnum].addr_rxcsr3) >> 16;

		spin_unlock_irqrestore(&musb->lock, flags);
	}
	return retval;
}

static void musb_gadget_fifo_flush(struct usb_ep *ep)
{
	struct musb_ep	*musb_ep = to_musb_ep(ep);
	struct musb	*musb = musb_ep->musb;
	u8		epnum = musb_ep->current_epnum;
	unsigned long	flags;
#ifndef USE_SSUSB_QMU	
	u32	int_txe;
#endif
    u32	txcsr0 = 0;

	spin_lock_irqsave(&musb->lock, flags);

	
#ifndef USE_SSUSB_QMU	
	int_txe = os_readl(U3D_EPIER);
	os_writel(U3D_EPIECR, ~(int_txe) | (1 << epnum));
#endif

	if (musb_ep->is_in) {

#ifdef USE_SSUSB_QMU
		_ex_mu3d_hal_flush_qmu(epnum, USB_TX);
		mu3d_hal_restart_qmu(epnum, USB_TX);

#endif
		txcsr0 = os_readl(musb->endpoints[epnum].addr_txcsr0);

		if (!(txcsr0 & TX_FIFOEMPTY)) {
			os_printk(K_DEBUG, "%s RESET\n", ep->name);
			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) | (BIT16<<epnum));
			os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) & ~(BIT16<<epnum));

			
		}

	} else {
#ifdef USE_SSUSB_QMU
		_ex_mu3d_hal_flush_qmu(epnum, USB_RX);
		mu3d_hal_restart_qmu(epnum, USB_RX);
#endif
		os_printk(K_DEBUG, "%s RESET\n", ep->name);
		
		
		os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) | (1 << epnum));
		os_writel(U3D_EP_RST, os_readl(U3D_EP_RST) & (~(1 << epnum)));

	}

	
#ifndef USE_SSUSB_QMU
	os_writel(U3D_EPIESR, int_txe);
#endif
	spin_unlock_irqrestore(&musb->lock, flags);
}

static const struct usb_ep_ops musb_ep_ops = {
	.enable		= musb_gadget_enable,
	.disable	= musb_gadget_disable,
	.alloc_request	= musb_alloc_request,
	.free_request	= musb_free_request,
	.queue		= musb_gadget_queue,
	.dequeue	= musb_gadget_dequeue,
	.set_halt	= musb_gadget_set_halt,
	.set_wedge	= musb_gadget_set_wedge,
	.fifo_status	= musb_gadget_fifo_status,
	.fifo_flush	= musb_gadget_fifo_flush
};


static int musb_gadget_get_frame(struct usb_gadget *gadget)
{
	return (int)os_readl(U3D_USB20_FRAME_NUM);
}

static int musb_gadget_wakeup(struct usb_gadget *gadget)
{
	struct musb	*musb = gadget_to_musb(gadget);
	unsigned long	flags;
	int		status = -EINVAL;
	u8		devctl;
	int		retries;

	spin_lock_irqsave(&musb->lock, flags);
        os_printk(K_DEBUG, "musb_gadget_wakeup\n");

	switch (musb->xceiv->state) {
	case OTG_STATE_B_PERIPHERAL:
		if (musb->may_wakeup && musb->is_suspended)
			break;
		goto done;
	case OTG_STATE_B_IDLE:
		
		devctl = os_readl(U3D_DEVICE_CONTROL);
		dev_dbg(musb->controller, "Sending SRP: devctl: %02x\n", devctl);
		devctl |= USB_DEVCTL_SESSION;
		
		devctl = os_readl(U3D_DEVICE_CONTROL);
		retries = 100;
		while (!(devctl & USB_DEVCTL_SESSION)) {
			devctl = os_readl(U3D_DEVICE_CONTROL);
			if (retries-- < 1)
				break;
		}
		retries = 10000;
		while (devctl & USB_DEVCTL_SESSION) {
			devctl = os_readl(U3D_DEVICE_CONTROL);
			if (retries-- < 1)
				break;
		}

		spin_unlock_irqrestore(&musb->lock, flags);
		otg_start_srp(musb->xceiv->otg);
		spin_lock_irqsave(&musb->lock, flags);

		
		musb_platform_try_idle(musb,
			jiffies + msecs_to_jiffies(1 * HZ));

		status = 0;
		goto done;
	default:
		dev_dbg(musb->controller, "Unhandled wake: %s\n",
			usb_otg_state_string(musb->xceiv->state));
		goto done;
	}

	status = 0;
	os_printk(K_DEBUG, "****************** musb_gadget_wakeup......\n");
done:
	spin_unlock_irqrestore(&musb->lock, flags);
	return status;
}

static int
musb_gadget_set_self_powered(struct usb_gadget *gadget, int is_selfpowered)
{
	struct musb	*musb = gadget_to_musb(gadget);

	musb->is_self_powered = !!is_selfpowered;
	return 0;
}

static void musb_pullup(struct musb *musb, int is_on)
{
	if (musb->is_active == 0) {
		os_printk(K_INFO, "power and clk is not ready!\n");
		return;
	}

	if (is_on) {
		#ifdef SUPPORT_U3
		mu3d_hal_u3dev_en();
		#else
		mu3d_hal_u2dev_connect();
		#endif
	} else {
		#ifdef SUPPORT_U3
		mu3d_hal_u3dev_dis();
		#endif
		mu3d_hal_u2dev_disconn();
	}

	dev_notice(musb->controller, "gadget D+ pullup %s\n",
		is_on ? "on" : "off");
}

#if 0
static int musb_gadget_vbus_session(struct usb_gadget *gadget, int is_active)
{
	dev_dbg(musb->controller, "<= %s =>\n", __func__);


	return -EINVAL;
}
#endif

static int musb_gadget_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	struct musb	*musb = gadget_to_musb(gadget);

	if (!musb->xceiv->set_power)
		return -EOPNOTSUPP;
	return usb_phy_set_power(musb->xceiv, mA);
}

static int musb_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct musb	*musb = gadget_to_musb(gadget);
	unsigned long	flags;

	is_on = !!is_on;

	pm_runtime_get_sync(musb->controller);

	spin_lock_irqsave(&musb->lock, flags);
	if (is_on != musb->softconnect) {
		musb->softconnect = is_on;
		musb_pullup(musb, is_on);
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	pm_runtime_put(musb->controller);

	return 0;
}

static int musb_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver);
static int musb_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver);

static const struct usb_gadget_ops musb_gadget_operations = {
	.get_frame		= musb_gadget_get_frame,
	.wakeup			= musb_gadget_wakeup,
	.set_selfpowered	= musb_gadget_set_self_powered,
	
	.vbus_draw		= musb_gadget_vbus_draw,
	.pullup			= musb_gadget_pullup,
	.udc_start		= musb_gadget_start,
	.udc_stop		= musb_gadget_stop,
	
};




#ifdef NEVER
static void musb_gadget_release(struct device *dev)
{
	
	dev_dbg(dev, "%s\n", __func__);
}
#endif 


static void
init_peripheral_ep(struct musb *musb, struct musb_ep *ep, u8 epnum, int is_in)
{
	struct musb_hw_ep	*hw_ep = musb->endpoints + epnum;

	memset(ep, 0, sizeof *ep);

	ep->current_epnum = epnum;
	ep->musb = musb;
	ep->hw_ep = hw_ep;
	ep->is_in = is_in;

	INIT_LIST_HEAD(&ep->req_list);

	sprintf(ep->name, "ep%d%s", epnum,
			(!epnum || hw_ep->is_shared_fifo) ? "" : (
				is_in ? "in" : "out"));

	os_printk(K_INFO, "%s, name=%s\n", __func__, ep->name);

	ep->end_point.name = ep->name;
	INIT_LIST_HEAD(&ep->end_point.ep_list);
	if (!epnum) {
		ep->end_point.maxpacket = hw_ep->max_packet_sz_tx;
		ep->end_point.ops = &musb_g_ep0_ops;
		musb->g.ep0 = &ep->end_point;
	} else {
		if (is_in)
			ep->end_point.maxpacket = hw_ep->max_packet_sz_tx;
		else
			ep->end_point.maxpacket = hw_ep->max_packet_sz_rx;
		ep->end_point.ops = &musb_ep_ops;
		list_add_tail(&ep->end_point.ep_list, &musb->g.ep_list);
	}

	os_printk(K_INFO, "%s, name=%s, maxp=%d\n", __func__, ep->end_point.name, ep->end_point.maxpacket);
}

static inline void musb_g_init_endpoints(struct musb *musb)
{
	u8			epnum;
	struct musb_hw_ep	*hw_ep;
	unsigned		count = 0;

	
	INIT_LIST_HEAD(&(musb->g.ep_list));

	os_printk(K_INFO, "%s nr_endpoints=%d\n", __func__,  musb->nr_endpoints);

	for (epnum = 0, hw_ep = musb->endpoints;
			epnum < musb->nr_endpoints;
			epnum++, hw_ep++) {
		os_printk(K_INFO, "%s epnum=%d shared_fifo=%d rx_maxp=%d tx_maxp=%d\n", \
			__func__, epnum, hw_ep->is_shared_fifo, hw_ep->max_packet_sz_rx?:0, hw_ep->max_packet_sz_tx?:0);
		if (hw_ep->is_shared_fifo ) {
			init_peripheral_ep(musb, &hw_ep->ep_in, epnum, 0);
			count++;
		} else {
			if (hw_ep->max_packet_sz_tx) {
				init_peripheral_ep(musb, &hw_ep->ep_in,
							epnum, 1);
				count++;
			}
			if (hw_ep->max_packet_sz_rx) {
				init_peripheral_ep(musb, &hw_ep->ep_out,
							epnum, 0);
				count++;
			}
		}
	}
}

int musb_gadget_setup(struct musb *musb)
{
	int status;


	musb->g.ops = &musb_gadget_operations;
	musb->g.max_speed = USB_SPEED_SUPER;
	musb->g.speed = USB_SPEED_UNKNOWN;

	
	
	
	
	
	musb->g.name = musb_driver_name;

	if (is_otg_enabled(musb))
		musb->g.is_otg = 1;

	musb_g_init_endpoints(musb);

	musb->is_active = 0;
	musb_platform_try_idle(musb, 0);

	
	
	
	
	

	status = usb_add_gadget_udc(musb->controller, &musb->g);
	if (status)
		goto err;

	return 0;
err:
	musb->g.dev.parent = NULL;
	device_unregister(&musb->g.dev);
	return status;
}

void musb_gadget_cleanup(struct musb *musb)
{
	usb_del_gadget_udc(&musb->g);
	
	
}

static int musb_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct musb		*musb = gadget_to_musb(g);
	struct usb_otg		*otg = musb->xceiv->otg;
	struct usb_hcd		*hcd = musb_to_hcd(musb);
	unsigned long		flags;
	int			retval = 0;

	if (driver->max_speed < USB_SPEED_HIGH) {
		retval = -EINVAL;
		goto err;
	}

	pm_runtime_get_sync(musb->controller);

	dev_dbg(musb->controller, "registering driver %s\n", driver->function);

	musb->softconnect = 0;
	musb->gadget_driver = driver;

	spin_lock_irqsave(&musb->lock, flags);
	musb->is_active = 1;

	otg_set_peripheral(otg, &musb->g);
	musb->xceiv->state = OTG_STATE_B_IDLE;


	if (!is_otg_enabled(musb))
		schedule_delayed_work_on(0, &musb->connection_work, 0);

	spin_unlock_irqrestore(&musb->lock, flags);

	if (is_otg_enabled(musb)) {
		dev_dbg(musb->controller, "OTG startup...\n");

		retval = usb_add_hcd(hcd, 0, 0);
		if (retval < 0) {
			dev_dbg(musb->controller, "add_hcd failed, %d\n", retval);
			goto err2;
		}

		if (musb->xceiv->last_event == USB_EVENT_ID)
			musb_platform_set_vbus(musb, 1);

		hcd->self.uses_pio_for_control = 1;
	}
	if (musb->xceiv->last_event == USB_EVENT_NONE)
		pm_runtime_put(musb->controller);

	return 0;

err2:
	if (!is_otg_enabled(musb))
		musb_stop(musb);
err:
	return retval;
}

static void stop_activity(struct musb *musb, struct usb_gadget_driver *driver)
{
	int			i;
	struct musb_hw_ep	*hw_ep;

	
	if (musb->g.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	else
		musb->g.speed = USB_SPEED_UNKNOWN;

	
	if (musb->softconnect) {
		musb->softconnect = 0;
		musb_pullup(musb, 0);
	}
	musb_stop(musb);

	if (driver) {
		for (i = 0, hw_ep = musb->endpoints;
				i < musb->nr_endpoints;
				i++, hw_ep++) {
			if (hw_ep->is_shared_fifo ) {
				nuke(&hw_ep->ep_in, -ESHUTDOWN);
			} else {
				if (hw_ep->max_packet_sz_tx)
					nuke(&hw_ep->ep_in, -ESHUTDOWN);
				if (hw_ep->max_packet_sz_rx)
					nuke(&hw_ep->ep_out, -ESHUTDOWN);
			}
		}
	}
}

static int musb_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct musb	*musb = gadget_to_musb(g);
	unsigned long	flags;

	if (musb->xceiv->last_event == USB_EVENT_NONE)
		pm_runtime_get_sync(musb->controller);


	spin_lock_irqsave(&musb->lock, flags);

	musb_hnp_stop(musb);

	(void) musb_gadget_vbus_draw(&musb->g, 0);

	musb->xceiv->state = OTG_STATE_UNDEFINED;
	stop_activity(musb, driver);
	otg_set_peripheral(musb->xceiv->otg, NULL);

	dev_dbg(musb->controller, "unregistering driver %s\n", driver->function);

	musb->is_active = 0;
	musb_platform_try_idle(musb, 0);
	spin_unlock_irqrestore(&musb->lock, flags);

	if (is_otg_enabled(musb)) {
		usb_remove_hcd(musb_to_hcd(musb));
	}

	if (!is_otg_enabled(musb))
		musb_stop(musb);

	pm_runtime_put(musb->controller);

	return 0;
}



void musb_g_resume(struct musb *musb)
{
	os_printk(K_INFO, "%s ++\n", __func__);
	musb->is_suspended = 0;
	switch (musb->xceiv->state) {
	case OTG_STATE_B_IDLE:
		break;
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_PERIPHERAL:
		musb->is_active = 1;
		if (musb->gadget_driver && musb->gadget_driver->resume) {
			spin_unlock(&musb->lock);
			musb->gadget_driver->resume(&musb->g);
			spin_lock(&musb->lock);
		}
		break;
	default:
		WARNING("unhandled RESUME transition (%s)\n",
				usb_otg_state_string(musb->xceiv->state));
	}
}

void musb_g_suspend(struct musb *musb)
{
	u32	devctl;

	os_printk(K_INFO, "%s ++\n", __func__);
	devctl = os_readl(U3D_DEVICE_CONTROL);
	dev_notice(musb->controller, "[U3D][%s] devctl %02x\n", __func__, devctl);

	switch (musb->xceiv->state) {
	case OTG_STATE_B_IDLE:
		if ((devctl & USB_DEVCTL_VBUSMASK) == USB_DEVCTL_VBUSVALID)
			musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
		break;
	case OTG_STATE_B_PERIPHERAL:
		musb->is_suspended = 1;
		if (musb->gadget_driver && musb->gadget_driver->suspend) {
			spin_unlock(&musb->lock);
			musb->gadget_driver->suspend(&musb->g);
			spin_lock(&musb->lock);
		}
		musb_sync_with_bat(musb,USB_SUSPEND); 
		break;
	default:
		WARNING("unhandled SUSPEND transition (%s)\n",
				usb_otg_state_string(musb->xceiv->state));
	}
}

void musb_g_wakeup(struct musb *musb)
{
	os_printk(K_INFO, "%s ++\n", __func__);
	musb_gadget_wakeup(&musb->g);
}

void musb_g_disconnect(struct musb *musb)
{
	u32 devctl = os_readl(U3D_DEVICE_CONTROL);
	os_printk(K_INFO, "%s ++\n", __func__);
	dev_notice(musb->controller, "[U3D][%s] devctl %02x\n", __func__, devctl);

	
	
	

	
	(void) musb_gadget_vbus_draw(&musb->g, 0);

	musb->g.speed = USB_SPEED_UNKNOWN;
	if (musb->gadget_driver && musb->gadget_driver->disconnect) {
		spin_unlock(&musb->lock);
		musb->gadget_driver->disconnect(&musb->g);
		spin_lock(&musb->lock);
	}

	switch (musb->xceiv->state) {
	default:
		dev_dbg(musb->controller, "Unhandled disconnect %s, setting a_idle\n",
			usb_otg_state_string(musb->xceiv->state));
		musb->xceiv->state = OTG_STATE_A_IDLE;
		MUSB_HST_MODE(musb);
		break;
	case OTG_STATE_A_PERIPHERAL:
		musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
		MUSB_HST_MODE(musb);
		break;
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_HOST:
	case OTG_STATE_B_PERIPHERAL:
	case OTG_STATE_B_IDLE:
		musb->xceiv->state = OTG_STATE_B_IDLE;
		break;
	case OTG_STATE_B_SRP_INIT:
		break;
	}

	musb->is_active = 0;
}

void musb_g_mute_disconnect(struct musb *musb)
{
	u32 devctl = os_readl(U3D_DEVICE_CONTROL);
	os_printk(K_INFO, "%s ++\n", __func__);
	dev_notice(musb->controller, "[U3D][%s] devctl %02x\n", __func__, devctl);
	
	
	

	
	(void) musb_gadget_vbus_draw(&musb->g, 0);

	musb->g.speed = USB_SPEED_UNKNOWN;

	switch (musb->xceiv->state) {
	default:
		dev_dbg(musb->controller, "Unhandled disconnect %s, setting a_idle\n",
			usb_otg_state_string(musb->xceiv->state));
		musb->xceiv->state = OTG_STATE_A_IDLE;
		MUSB_HST_MODE(musb);
		break;
	case OTG_STATE_A_PERIPHERAL:
		musb->xceiv->state = OTG_STATE_A_WAIT_BCON;
		MUSB_HST_MODE(musb);
		break;
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_HOST:
	case OTG_STATE_B_PERIPHERAL:
	case OTG_STATE_B_IDLE:
		musb->xceiv->state = OTG_STATE_B_IDLE;
		break;
	case OTG_STATE_B_SRP_INIT:
		break;
	}

	musb->is_active = 0;
}

extern struct musb_fifo_cfg ep0_cfg_u3;
extern struct musb_fifo_cfg ep0_cfg_u2;
extern void ep0_setup(struct musb *musb, struct musb_hw_ep  *hw_ep0, const struct musb_fifo_cfg *cfg);

void musb_conifg_ep0(struct musb *musb)
{
	os_printk(K_DEBUG, "[%s] U3D_DEVICE_CONF: %x\n", __func__, os_readl(U3D_DEVICE_CONF));

	if (os_readl(U3D_DEVICE_CONF) & HW_USB2_3_SEL) { 
		musb->g.speed = USB_SPEED_SUPER;
		musb->g.ep0->maxpacket = 512;

		os_printk(K_INFO, "musb_g_reset musb->g.speed: super\n");
		ep0_setup(musb, musb->endpoints, &ep0_cfg_u3);
	} else { 
		musb->g.speed = (u8)(os_readl(U3D_POWER_MANAGEMENT) & HS_MODE)
				? USB_SPEED_HIGH : USB_SPEED_FULL;
		musb->g.ep0->maxpacket = 64;

		os_printk(K_INFO, "musb_g_reset musb->g.speed: %s\n", (musb->g.speed == USB_SPEED_HIGH) ? "high" : "full");
		ep0_setup(musb, musb->endpoints, &ep0_cfg_u2);
	}

	os_printk(K_DEBUG, "U3D_EP0CSR: %x\n", os_readl(U3D_EP0CSR));
	os_printk(K_DEBUG, "U3D_RXCOUNT0: %x\n", os_readl(U3D_RXCOUNT0));
}

void musb_g_reset(struct musb *musb)
__releases(musb->lock)
__acquires(musb->lock)
{
	u32 devctl = os_readl(U3D_DEVICE_CONTROL);

	os_printk(K_INFO, "%s ++\n", __func__);
	
	if(musb->test_mode == 0)
		musb_sync_with_bat(musb,USB_UNCONFIGURED);

	mu3d_hal_unfigured_ep();

	
	if (musb->g.speed != USB_SPEED_UNKNOWN) {
		
		musb_g_mute_disconnect(musb);
	}

	
	else if(devctl & USB_DEVCTL_HOSTREQUEST)
        {
          
          

        }

	musb_conifg_ep0(musb);
	
	musb->is_active = 1;
	musb->is_suspended = 0;
	MUSB_DEV_MODE(musb);
	musb->address = 0;
	musb->ep0_state = MUSB_EP0_STAGE_SETUP;

	musb->may_wakeup = 0;
	musb->g.b_hnp_enable = 0;
	musb->g.a_alt_hnp_support = 0;
	musb->g.a_hnp_support = 0;

	if (devctl & USB_DEVCTL_BDEVICE) {
		musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
		musb->g.is_a_peripheral = 0;
	} else if (is_otg_enabled(musb)) {
		musb->xceiv->state = OTG_STATE_A_PERIPHERAL;
		musb->g.is_a_peripheral = 1;
	} else
		WARN_ON(1);

	
	(void) musb_gadget_vbus_draw(&musb->g,
			is_otg_enabled(musb) ? 8 : 100);
}
