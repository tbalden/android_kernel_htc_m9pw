/*
 * MUSB OTG driver core code
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/prefetch.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#ifdef CONFIG_USBIF_COMPLIANCE
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <mach/system.h>
#endif

#include <mach/mt_chip.h>
#include "musb_core.h"
#include <linux/mu3d/hal/mu3d_hal_osal.h>
#include <linux/mu3d/hal/mu3d_hal_usb_drv.h>
#include <linux/mu3d/hal/mu3d_hal_hw.h>
#include "ssusb_qmu.h"

#ifdef CONFIG_MTK_UART_USB_SWITCH
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#define AP_UART0_COMPATIBLE_NAME "mediatek,AP_UART0"
#endif

#define TA_WAIT_BCON(m) max_t(int, (m)->a_wait_bcon, OTG_TIME_A_WAIT_BCON)


#define DRIVER_AUTHOR "Mentor Graphics, Texas Instruments, Nokia"
#define DRIVER_DESC "Inventra Dual-Role USB Controller Driver"

#define MUSB_VERSION "6.0"

#define DRIVER_INFO DRIVER_DESC ", v" MUSB_VERSION

const char musb_driver_name[] = MUSB_DRIVER_NAME;

struct musb	*_mu3d_musb = NULL;

u32 debug_level = K_ALET | K_CRIT | K_ERR | K_WARNIN | K_NOTICE | K_INFO;

module_param(debug_level , int, 0644);
MODULE_PARM_DESC(debug_level, "Debug Print Log Lvl");

#ifdef EP_PROFILING
u32 is_prof = 1;

module_param(is_prof , int, 0644);
MODULE_PARM_DESC(is_prof, "profiling each EP");
#endif

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MUSB_DRIVER_NAME);

#define U3D_FIFO_START_ADDRESS 0

#ifdef CONFIG_MTK_UART_USB_SWITCH
	extern bool in_uart_mode;
#endif

void __iomem *u3_base;
void __iomem *u3_sif_base;
void __iomem *u3_sif2_base;
void __iomem *ap_uart0_base;

#ifdef CONFIG_MTK_FPGA
void __iomem *i2c1_base;
#endif


static inline struct musb *dev_to_musb(struct device *dev)
{
	return dev_get_drvdata(dev);
}


#if 0
static int musb_ulpi_read(struct otg_transceiver *otg, u32 offset)
{
	void __iomem *addr = otg->io_priv;
	int	i = 0;
	u8	r;
	u8	power;

	
	power = musb_readb(addr, MUSB_POWER);
	power &= ~MUSB_POWER_SUSPENDM;
	musb_writeb(addr, MUSB_POWER, power);


	musb_writeb(addr, MUSB_ULPI_REG_ADDR, (u8)offset);
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL,
			MUSB_ULPI_REG_REQ | MUSB_ULPI_RDN_WR);

	while (!(musb_readb(addr, MUSB_ULPI_REG_CONTROL)
				& MUSB_ULPI_REG_CMPLT)) {
		i++;
		if (i == 10000)
			return -ETIMEDOUT;

	}
	r = musb_readb(addr, MUSB_ULPI_REG_CONTROL);
	r &= ~MUSB_ULPI_REG_CMPLT;
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, r);

	return musb_readb(addr, MUSB_ULPI_REG_DATA);
}

static int musb_ulpi_write(struct otg_transceiver *otg,
		u32 offset, u32 data)
{
	void __iomem *addr = otg->io_priv;
	int	i = 0;
	u8	r = 0;
	u8	power;

	
	power = musb_readb(addr, MUSB_POWER);
	power &= ~MUSB_POWER_SUSPENDM;
	musb_writeb(addr, MUSB_POWER, power);

	musb_writeb(addr, MUSB_ULPI_REG_ADDR, (u8)offset);
	musb_writeb(addr, MUSB_ULPI_REG_DATA, (u8)data);
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, MUSB_ULPI_REG_REQ);

	while (!(musb_readb(addr, MUSB_ULPI_REG_CONTROL)
				& MUSB_ULPI_REG_CMPLT)) {
		i++;
		if (i == 10000)
			return -ETIMEDOUT;
	}

	r = musb_readb(addr, MUSB_ULPI_REG_CONTROL);
	r &= ~MUSB_ULPI_REG_CMPLT;
	musb_writeb(addr, MUSB_ULPI_REG_CONTROL, r);

	return 0;
}
#else
#define musb_ulpi_read		NULL
#define musb_ulpi_write		NULL
#endif

void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *src)
{
	unsigned int residue;
	unsigned int temp;
	void __iomem *fifo = (void __iomem *)(uintptr_t)USB_FIFO(hw_ep->epnum); 

	os_printk(K_DEBUG, "%s epnum=%d, len=%d, buf=%p\n", __func__, hw_ep->epnum, len, src);

	residue = len;

	while(residue > 0) {

		if(residue==1) {
			temp = ((*src)&0xFF);
			
			writeb(temp, fifo);
			src += 1;
			residue -= 1;
		} else if(residue==2) {
			temp = ((*src)&0xFF) + (((*(src+1))<<8)&0xFF00);
			
			writew(temp, fifo);
			src += 2;
			residue -= 2;
		} else if(residue==3) {
			temp = ((*src)&0xFF) + (((*(src+1))<<8)&0xFF00);
			
			writew(temp, fifo);
			src += 2;

			temp = ((*src)&0xFF);
			
			writeb(temp, fifo);
			src += 1;
			residue -= 3;
		} else {
			temp = ((*src)&0xFF) + (((*(src+1))<<8)&0xFF00) + \
				(((*(src+2))<<16)&0xFF0000) + (((*(src+3))<<24)&0xFF000000);
			
			writel(temp, fifo);
			src += 4;
			residue -= 4;
		}
	}
}

void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	u16 residue;
	unsigned int temp;
	void __iomem *fifo = (void __iomem *)(uintptr_t)USB_FIFO(hw_ep->epnum); 

	os_printk(K_DEBUG, "%s %cX ep%d fifo %p count %d buf %p\n",
			__func__, 'R', hw_ep->epnum, fifo, len, dst);

	residue = len;

	while(residue > 0) {

		temp = os_readl(fifo);

		
		*dst = temp & 0xFF;

		
		if(residue>1)
			*(dst+1) = (temp>>8) & 0xFF;

		
		if(residue>2)
			*(dst+2) = (temp>>16) & 0xFF;

		
		if(residue>3)
			*(dst+3) = (temp>>24) & 0xFF;

		if(residue>4) {
			dst = dst + 4;
			residue = residue - 4;
		} else {
			residue = 0;
		}
	}

}




static const u8 musb_test_packet[53] = {
	

	
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	
	0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	
	0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
	
	0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	
	0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd,
	
	0xfc, 0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0x7e

	
};

void musb_load_testpacket(struct musb *musb)
{
	u32 maxp;

	maxp = musb->endpoints->max_packet_sz_tx;
	mu3d_hal_write_fifo( 0, sizeof(musb_test_packet), (u8 *)musb_test_packet, maxp);
}


void musb_otg_timer_func(unsigned long data)
{
	struct musb	*musb = (struct musb *)data;
	unsigned long	flags;

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->state) {
	case OTG_STATE_B_WAIT_ACON:
		dev_dbg(musb->controller, "HNP: b_wait_acon timeout; back to b_peripheral\n");
		musb_g_disconnect(musb);
		musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
		musb->is_active = 0;
		break;
	case OTG_STATE_A_SUSPEND:
	case OTG_STATE_A_WAIT_BCON:
		dev_dbg(musb->controller, "HNP: %s timeout\n",
			usb_otg_state_string(musb->xceiv->state));
		musb_platform_set_vbus(musb, 0);
		musb->xceiv->state = OTG_STATE_A_WAIT_VFALL;
		break;
	default:
		dev_dbg(musb->controller, "HNP: Unhandled mode %s\n",
			usb_otg_state_string(musb->xceiv->state));
	}
	musb->ignore_disconnect = 0;
	spin_unlock_irqrestore(&musb->lock, flags);
}

void musb_hnp_stop(struct musb *musb)
{
	struct usb_hcd	*hcd = musb_to_hcd(musb);
	u32	reg;

	dev_dbg(musb->controller, "HNP: stop from %s\n", usb_otg_state_string(musb->xceiv->state));

	switch (musb->xceiv->state) {
	case OTG_STATE_A_PERIPHERAL:
		musb_g_disconnect(musb);
		dev_dbg(musb->controller, "HNP: back to %s\n",
			usb_otg_state_string(musb->xceiv->state));
		break;
	case OTG_STATE_B_HOST:
		dev_dbg(musb->controller, "HNP: Disabling HR\n");
		hcd->self.is_b_host = 0;
		musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
		MUSB_DEV_MODE(musb);
		
		reg = os_readl(U3D_POWER_MANAGEMENT);
		reg |= SUSPENDM_ENABLE;
		os_writel(U3D_POWER_MANAGEMENT, reg);
		
		break;
	default:
		dev_dbg(musb->controller, "HNP: Stopping in unknown state %s\n",
			usb_otg_state_string(musb->xceiv->state));
	}

	musb->port1_status &= ~(USB_PORT_STAT_C_CONNECTION << 16);
}


static irqreturn_t musb_stage0_irq(struct musb *musb, u32 int_usb,
				u8 devctl, u8 power)
{
	struct usb_otg *otg = musb->xceiv->otg;
	irqreturn_t handled = IRQ_NONE;

	dev_notice(musb->controller, "<== Power=%02x, DevCtl=%02x, int_usb=0x%x\n", power, devctl,
		int_usb);

	if (int_usb & RESUME_INTR) {
		handled = IRQ_HANDLED;
		dev_notice(musb->controller, "RESUME (%s)\n", usb_otg_state_string(musb->xceiv->state));

		
		switch (musb->xceiv->state) {
			case OTG_STATE_A_SUSPEND:
				
				musb->xceiv->state = OTG_STATE_A_HOST;
				usb_hcd_resume_root_hub(musb_to_hcd(musb));
				break;
			case OTG_STATE_B_WAIT_ACON:
			case OTG_STATE_B_PERIPHERAL:
				if ((devctl & USB_DEVCTL_VBUSVALID)
						!= (3 << USB_DEVCTL_VBUS_OFFSET)
						) {
					musb->int_usb |= DISCONN_INTR;
					musb->int_usb &= ~SUSPEND_INTR;
					break;
				}
				musb_g_resume(musb);
				break;
			case OTG_STATE_B_IDLE:
				musb->int_usb &= ~SUSPEND_INTR;
				break;
			default:
				WARNING("bogus %s RESUME (%s)\n",
					"peripheral",
					usb_otg_state_string(musb->xceiv->state));
		}
	}

	
	if (int_usb & SESSION_REQ_INTR) {
		if ((devctl & USB_DEVCTL_VBUSMASK) == USB_DEVCTL_VBUSVALID
				&& (devctl & USB_DEVCTL_BDEVICE)) {
			dev_dbg(musb->controller, "SessReq while on B state\n");
			return IRQ_HANDLED;
		}

		dev_notice(musb->controller, "SESSION_REQUEST (%s)\n",
			usb_otg_state_string(musb->xceiv->state));

		
		musb->ep0_stage = MUSB_EP0_START;
		musb->xceiv->state = OTG_STATE_A_IDLE;
		MUSB_HST_MODE(musb);
		musb_platform_set_vbus(musb, 1);

		handled = IRQ_HANDLED;
	}

	if (int_usb & VBUSERR_INTR) {
		int	ignore = 0;

		switch (musb->xceiv->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_A_WAIT_BCON:
		case OTG_STATE_A_WAIT_VRISE:
			if (musb->vbuserr_retry) {
				musb->vbuserr_retry--;
				ignore = 1;
				devctl |= USB_DEVCTL_SESSION;
				
			} else {
				musb->port1_status |=
					  USB_PORT_STAT_OVERCURRENT
					| (USB_PORT_STAT_C_OVERCURRENT << 16);
			}
			break;
		default:
			break;
		}

		dev_notice(musb->controller, "VBUS_ERROR in %s (%02x, %s), retry #%d, port1 %08x\n",
				usb_otg_state_string(musb->xceiv->state),
				devctl,
				({ char *s;
				switch (devctl & USB_DEVCTL_VBUSMASK) {
				case 0 << USB_DEVCTL_VBUS_OFFSET:
					s = "<SessEnd"; break;
				case 1 << USB_DEVCTL_VBUS_OFFSET:
					s = "<AValid"; break;
				case 2 << USB_DEVCTL_VBUS_OFFSET:
					s = "<VBusValid"; break;
				
				default:
					s = "VALID"; break;
				}; s; }),
				VBUSERR_RETRY_COUNT - musb->vbuserr_retry,
				musb->port1_status);

		
		if (!ignore)
			musb_platform_set_vbus(musb, 0);
		handled = IRQ_HANDLED;
	}

	if (int_usb & SUSPEND_INTR) {
		dev_notice(musb->controller, "SUSPEND (%s) devctl %02x power %02x\n",
			usb_otg_state_string(musb->xceiv->state), devctl, power);
		handled = IRQ_HANDLED;

		switch (musb->xceiv->state) {
		case OTG_STATE_A_PERIPHERAL:
			musb_hnp_stop(musb);
			usb_hcd_resume_root_hub(musb_to_hcd(musb));
			
			musb_platform_try_idle(musb, jiffies
					+ msecs_to_jiffies(musb->a_wait_bcon
						? : OTG_TIME_A_WAIT_BCON));

			break;
		case OTG_STATE_B_IDLE:
			if (!musb->is_active)
				break;
		case OTG_STATE_B_PERIPHERAL:
			musb_g_suspend(musb);
			musb->is_active = is_otg_enabled(musb)
					&& otg->gadget->b_hnp_enable;
			if (musb->is_active) {
				musb->xceiv->state = OTG_STATE_B_WAIT_ACON;
				dev_dbg(musb->controller, "HNP: Setting timer for b_ase0_brst\n");
				mod_timer(&musb->otg_timer, jiffies
					+ msecs_to_jiffies(
							OTG_TIME_B_ASE0_BRST));
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (musb->a_wait_bcon != 0)
				musb_platform_try_idle(musb, jiffies
					+ msecs_to_jiffies(musb->a_wait_bcon));
			break;
		case OTG_STATE_A_HOST:
			musb->xceiv->state = OTG_STATE_A_SUSPEND;
			musb->is_active = is_otg_enabled(musb)
					&& otg->host->b_hnp_enable;
			break;
		case OTG_STATE_B_HOST:
			
			dev_dbg(musb->controller, "REVISIT: SUSPEND as B_HOST\n");
			break;
		default:
			
			musb->is_active = 0;
			break;
		}
	}

	if (int_usb & CONN_INTR) {
		struct usb_hcd *hcd = musb_to_hcd(musb);
		u32 int_en = 0;

		handled = IRQ_HANDLED;
		musb->is_active = 1;

		musb->ep0_stage = MUSB_EP0_START;
		os_printk(K_DEBUG, "----- ep0 state: MUSB_EP0_START\n");

		
		if (is_peripheral_active(musb)) {
			
		}
		
		
#ifdef USE_SSUSB_QMU
		
		os_writel(U3D_EPIESR, os_readl(U3D_EPIESR)|EP0ISR);
#else
		
		os_printk(K_DEBUG, "Enable EP0 & EPn interrupt =%x\n", musb->epmask | ((musb->epmask << 16) & EPRISR));
		os_writel(U3D_EPIESR, musb->epmask | ((musb->epmask << 16) & EPRISR));
#endif
		int_en = SUSPEND_INTR_EN|RESUME_INTR_EN|RESET_INTR_EN|CONN_INTR_EN|DISCONN_INTR_EN;

		os_writel(U3D_COMMON_USB_INTR_ENABLE, int_en);

		

		musb->port1_status &= ~(USB_PORT_STAT_LOW_SPEED
					|USB_PORT_STAT_HIGH_SPEED
					|USB_PORT_STAT_ENABLE
					);
		musb->port1_status |= USB_PORT_STAT_CONNECTION
					|(USB_PORT_STAT_C_CONNECTION << 16);

		
		if (devctl & USB_DEVCTL_LS_DEV)
			musb->port1_status |= USB_PORT_STAT_LOW_SPEED;

		
		switch (musb->xceiv->state) {
		case OTG_STATE_B_PERIPHERAL:
			if (int_usb & SUSPEND_INTR) {
				dev_dbg(musb->controller, "HNP: SUSPEND+CONNECT, now b_host\n");
				int_usb &= ~SUSPEND_INTR;
				goto b_host;
			} else
				dev_dbg(musb->controller, "CONNECT as b_peripheral???\n");
			break;
		case OTG_STATE_B_WAIT_ACON:
			dev_dbg(musb->controller, "HNP: CONNECT, now b_host\n");
b_host:
			musb->xceiv->state = OTG_STATE_B_HOST;
			hcd->self.is_b_host = 1;
			musb->ignore_disconnect = 0;
			del_timer(&musb->otg_timer);
			break;
		default:
			if ((devctl & USB_DEVCTL_VBUSVALID)
					== (3 << USB_DEVCTL_VBUS_OFFSET)) {
				musb->xceiv->state = OTG_STATE_A_HOST;
				hcd->self.is_b_host = 0;
			}
			break;
		}

		
		MUSB_HST_MODE(musb);
		if (hcd->status_urb)
			usb_hcd_poll_rh_status(hcd);
		else
			usb_hcd_resume_root_hub(hcd);

		dev_notice(musb->controller, "CONNECT (%s) devctl %02x\n",
				usb_otg_state_string(musb->xceiv->state), devctl);
	}

	if ((int_usb & DISCONN_INTR) && !musb->ignore_disconnect) {
		dev_notice(musb->controller, "DISCONNECT (%s) as %s, devctl %02x\n",
				usb_otg_state_string(musb->xceiv->state),
				MUSB_MODE(musb), devctl);
		handled = IRQ_HANDLED;

		switch (musb->xceiv->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_A_SUSPEND:
			usb_hcd_resume_root_hub(musb_to_hcd(musb));
			if (musb->a_wait_bcon != 0 && is_otg_enabled(musb))
				musb_platform_try_idle(musb, jiffies
					+ msecs_to_jiffies(musb->a_wait_bcon));
			break;
		case OTG_STATE_B_HOST:
			
			musb_to_hcd(musb)->self.is_b_host = 0;
			musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
			MUSB_DEV_MODE(musb);
			musb_g_disconnect(musb);
			break;
		case OTG_STATE_A_PERIPHERAL:
			musb_hnp_stop(musb);
			
			
		case OTG_STATE_B_WAIT_ACON:
			
		case OTG_STATE_B_PERIPHERAL:
		case OTG_STATE_B_IDLE:
			musb_g_disconnect(musb);
			break;
		default:
			WARNING("unhandled DISCONNECT transition (%s)\n",
				usb_otg_state_string(musb->xceiv->state));
			break;
		}
	}

	if (int_usb & RESET_INTR) {
		handled = IRQ_HANDLED;

		mu3d_hal_pdn_ip_port(1, 0, 1, 1);

		if(1)
		{
			dev_notice(musb->controller, "BUS RESET as %s\n",
				usb_otg_state_string(musb->xceiv->state));
			os_printk(K_INFO, "BUS RESET\n");
			switch (musb->xceiv->state) {
			case OTG_STATE_A_SUSPEND:
				musb->ignore_disconnect = 1;
				musb_g_reset(musb);
				
			case OTG_STATE_A_WAIT_BCON:	
				
				dev_dbg(musb->controller, "HNP: in %s, %d msec timeout\n",
					usb_otg_state_string(musb->xceiv->state),
					TA_WAIT_BCON(musb));
				mod_timer(&musb->otg_timer, jiffies
					+ msecs_to_jiffies(TA_WAIT_BCON(musb)));
				break;
			case OTG_STATE_A_PERIPHERAL:
				musb->ignore_disconnect = 0;
				del_timer(&musb->otg_timer);
				musb_g_reset(musb);
				break;
			case OTG_STATE_B_WAIT_ACON:
				dev_dbg(musb->controller, "HNP: RESET (%s), to b_peripheral\n",
					usb_otg_state_string(musb->xceiv->state));
				musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
				musb_g_reset(musb);
				break;
			case OTG_STATE_B_IDLE:
				musb->xceiv->state = OTG_STATE_B_PERIPHERAL;
				
			case OTG_STATE_B_PERIPHERAL:
				musb_g_reset(musb);
				break;
			default:
				dev_dbg(musb->controller, "Unhandled BUS RESET as %s\n",
					usb_otg_state_string(musb->xceiv->state));
			}
		}
	}

	schedule_work(&musb->irq_work);

	return handled;
}

#ifdef EP_PROFILING
#define POLL_INTERVAL 10

unsigned int ep_prof[8][2];

static void ep_prof_work(struct work_struct *data)
{
	struct musb *musb = container_of(to_delayed_work(data), struct musb, ep_prof_work);

	int i;
	int tx = 0;
	int rx = 0;
	bool is_print = false;

	for ( i=1; i<9; i++) {
		if( (ep_prof[i-1][0] != 0) || (ep_prof[i-1][1] != 0)) {
			os_printk(K_INFO, "[%d]T%d,R%d", i, ep_prof[i-1][0], ep_prof[i-1][1]);
			tx += ep_prof[i-1][0];
			rx += ep_prof[i-1][1];
			is_print = true;
		}
		ep_prof[i-1][0] = ep_prof[i-1][1] = 0;
	}

	if (is_print) os_printk(K_INFO, "T%d,R%d\n", tx, rx);

	schedule_delayed_work(&musb->ep_prof_work, msecs_to_jiffies(POLL_INTERVAL*1000));
}
#endif

static void musb_restore_context(struct musb *musb);
static void musb_save_context(struct musb *musb);
extern void usb20_pll_settings(bool host, bool forceOn);

void musb_start(struct musb *musb)
{
	u8 devctl = (u8)os_readl(U3D_DEVICE_CONTROL);

	dev_dbg(musb->controller, "<== devctl %02x\n", devctl);

	os_printk(K_INFO, "%s\n", __func__);

	if (musb->is_clk_on == 0) {
#ifndef CONFIG_MTK_FPGA
		
		usb_phy_recover(musb->is_clk_on);
		musb->is_clk_on = 1;

		
		u3phy_ops->u2_slew_rate_calibration(u3phy);
#endif

		
		_ex_mu3d_hal_ssusb_en();

		
		usb20_pll_settings(false, false);

		
		mu3d_hal_rst_dev();

		udelay(20);

		musb_restore_context(musb);
	}

	
	os_writel(U3D_LV1IESR, 0xFFFFFFFF);

	
	_ex_mu3d_hal_system_intr_en();

#ifdef USB_GADGET_SUPERSPEED
	
	
	os_writel(U3D_POWER_MANAGEMENT, (os_readl(U3D_POWER_MANAGEMENT) | HS_ENABLE));

	
	
	os_writel(U3D_POWER_MANAGEMENT, (os_readl(U3D_POWER_MANAGEMENT) | LPM_HRWE));
	os_writel(U3D_USB2_EPCTL_LPM, (L1_EXIT_EP0_CHK | L1_EXIT_EP_IN_CHK | L1_EXIT_EP_OUT_CHK));
	os_writel(U3D_USB2_EPCTL_LPM_FC_CHK, (L1_EXIT_EP0_FC_CHK | L1_EXIT_EP_IN_FC_CHK | L1_EXIT_EP_OUT_FC_CHK));
	

#ifdef CONFIG_USBIF_COMPLIANCE
	
	os_writel(U3D_LINK_POWER_CONTROL, os_readl(U3D_LINK_POWER_CONTROL) | SW_U1_ACCEPT_ENABLE | SW_U2_ACCEPT_ENABLE);

	os_writel(U3D_LINK_HP_TIMER, (os_readl(U3D_LINK_HP_TIMER) & ~(PHP_TIMEOUT_VALUE)) | 0x6); 

	
	os_writel(U3D_POWER_MANAGEMENT, (os_readl(U3D_POWER_MANAGEMENT) |  LPM_HRWE));
	os_writel(U3D_USB2_EPCTL_LPM, (L1_EXIT_EP0_CHK | L1_EXIT_EP_IN_CHK | L1_EXIT_EP_OUT_CHK));
	os_writel(U3D_USB2_EPCTL_LPM_FC_CHK, (L1_EXIT_EP0_FC_CHK | L1_EXIT_EP_IN_FC_CHK | L1_EXIT_EP_OUT_FC_CHK));

	
	os_setmsk(U3D_MISC_CTRL, (VBUS_FRC_EN|VBUS_ON));
#endif

	
	os_writel(U3D_LTSSM_CTRL, os_readl(U3D_LTSSM_CTRL) &~ SOFT_U3_EXIT_EN);

#else
#ifdef USB_GADGET_DUALSPEED
	
	os_writel(U3D_POWER_MANAGEMENT, os_readl(U3D_POWER_MANAGEMENT) | HS_ENABLE);
#else
	
	os_writel(U3D_POWER_MANAGEMENT, os_readl(U3D_POWER_MANAGEMENT) & ~HS_ENABLE);
#endif
	
	mu3d_hal_u3dev_dis();
#endif

#ifndef CONFIG_MTK_FPGA
	if (mt_get_chip_hw_code() == 0x6595) {
		os_printk(K_INFO, "%s Set Clock to 62.4MHz+\n", __func__);
		
		os_setmsk(U3D_SSUSB_SYS_CK_CTRL, SSUSB_SYS_CK_DIV2_EN);
		
		os_writelmsk(U3D_USB20_TIMING_PARAMETER, 63, TIME_VALUE_1US);
#ifdef SUPPORT_U3
		
		os_writelmsk(U3D_TIMING_PULSE_CTRL, 63, CNT_1US_VALUE);
#endif
		os_printk(K_INFO, "%s Set Clock to 62.4MHz-\n", __func__);
	}
#endif

	os_writel(U3D_LINK_RESET_INFO, os_readl(U3D_LINK_RESET_INFO) & ~WTCHRP);

	
	os_writel(U3D_DEVICE_CONF, 0);

	musb->is_active = 1;

	musb_platform_enable(musb);

#ifdef EP_PROFILING
	if (is_prof!=0) schedule_delayed_work(&musb->ep_prof_work, msecs_to_jiffies(POLL_INTERVAL*1000));
#endif

	if (musb->softconnect)
		mu3d_hal_u3dev_en();
}


static void musb_generic_disable(void)
{
	
	mu3d_hal_initr_dis();

	
	mu3d_hal_clear_intr();
}

static void gadget_stop(struct musb *musb)
{
	
	mu3d_hal_u3dev_dis();
	mu3d_hal_u2dev_disconn();

	
	if (musb->g.speed != USB_SPEED_UNKNOWN) {
		if (musb->gadget_driver && musb->gadget_driver->disconnect) {
			musb->gadget_driver->disconnect(&musb->g);
		}
		musb->g.speed = USB_SPEED_UNKNOWN;
	}
}

void musb_stop(struct musb *musb)
{
	os_printk(K_INFO, "musb_stop\n");

	
	musb_platform_disable(musb);
	musb_generic_disable();

	
	gadget_stop(musb);
	musb->is_active = 0;
	
#ifndef CONFIG_USBIF_COMPLIANCE
	cancel_delayed_work_sync(&musb->check_ltssm_work);
#endif

	dev_dbg(musb->controller, "HDRC disabled\n");

	if (musb->active_ep == 0)
		schedule_work(&musb->suspend_work);

	
#ifdef NEVER
	musb_save_context(musb);

	
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!in_uart_mode)
		os_setmsk(U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);
#else
		os_setmsk(U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);
#endif

	#ifndef CONFIG_MTK_FPGA
	
	usb_phy_savecurrent(musb->is_clk_on);
	musb->is_clk_on = 0;
	#endif
#endif 

	musb_platform_try_idle(musb, 0);
}

static void musb_shutdown(struct platform_device *pdev)
{
	struct musb	*musb = dev_to_musb(&pdev->dev);
	unsigned long	flags;

	pm_runtime_get_sync(musb->controller);
	spin_lock_irqsave(&musb->lock, flags);
	musb_platform_disable(musb);
	musb_generic_disable();
	spin_unlock_irqrestore(&musb->lock, flags);

#ifndef CONFIG_USBIF_COMPLIANCE
	if (!is_otg_enabled(musb) && is_host_enabled(musb))
		usb_remove_hcd(musb_to_hcd(musb));
#endif

	os_writel(U3D_DEVICE_CONTROL, 0);
	musb_platform_exit(musb);

	pm_runtime_put(musb->controller);
	
}



#if defined(CONFIG_USB_MUSB_TUSB6010)			\
	|| defined(CONFIG_USB_MUSB_TUSB6010_MODULE)	\
	|| defined(CONFIG_USB_MUSB_OMAP2PLUS)		\
	|| defined(CONFIG_USB_MUSB_OMAP2PLUS_MODULE)	\
	|| defined(CONFIG_USB_MUSB_AM35X)		\
	|| defined(CONFIG_USB_MUSB_AM35X_MODULE)

#ifdef CONFIG_USBIF_COMPLIANCE
static ushort fifo_mode = 4;
#else
static ushort __initdata fifo_mode = 4;
#endif

#elif defined(CONFIG_USB_MUSB_UX500)			\
	|| defined(CONFIG_USB_MUSB_UX500_MODULE)

#ifdef CONFIG_USBIF_COMPLIANCE
static ushort fifo_mode = 5;
#else
static ushort __initdata fifo_mode = 5;
#endif

#else

#ifdef CONFIG_USBIF_COMPLIANCE
static ushort fifo_mode = 2;
#else
static ushort __initdata fifo_mode = 2;
#endif

#endif

module_param(fifo_mode, ushort, 0);
MODULE_PARM_DESC(fifo_mode, "initial endpoint configuration");


#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_0_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_0_cfg[] = {
#endif
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RXTX, .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_1_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_1_cfg[] = {
#endif
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_RXTX, .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_2_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_2_cfg[] = {
#endif
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_3_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_3_cfg[] = {
#endif
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_4_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_4_cfg[] = {
#endif
{ .hw_ep_num =  1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 10, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 10, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 11, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 11, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 12, .style = FIFO_TX,   .maxpacket = 256, },
{ .hw_ep_num = 12, .style = FIFO_RX,   .maxpacket = 64, },
{ .hw_ep_num = 13, .style = FIFO_RXTX, .maxpacket = 4096, },
{ .hw_ep_num = 14, .style = FIFO_RXTX, .maxpacket = 1024, },
{ .hw_ep_num = 15, .style = FIFO_RXTX, .maxpacket = 1024, },
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb_fifo_cfg mode_5_cfg[] = {
#else
static struct musb_fifo_cfg __initdata mode_5_cfg[] = {
#endif
{ .hw_ep_num =  1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  6, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  7, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  7, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  8, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  8, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num =  9, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num =  9, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 10, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 10, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 11, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 11, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 12, .style = FIFO_TX,   .maxpacket = 32, },
{ .hw_ep_num = 12, .style = FIFO_RX,   .maxpacket = 32, },
{ .hw_ep_num = 13, .style = FIFO_RXTX, .maxpacket = 512, },
{ .hw_ep_num = 14, .style = FIFO_RXTX, .maxpacket = 1024, },
{ .hw_ep_num = 15, .style = FIFO_RXTX, .maxpacket = 1024, },
};

void ep0_setup(struct musb *musb, struct musb_hw_ep  *hw_ep0, const struct musb_fifo_cfg *cfg)
{
    os_printk(K_INFO, "ep0_setup maxpacket: %d\n", cfg->maxpacket);

	hw_ep0->fifoaddr_rx = 0;
	hw_ep0->fifoaddr_tx = 0;
	hw_ep0->is_shared_fifo = true;
	hw_ep0->fifo = (void __iomem *)(uintptr_t)MUSB_FIFO_OFFSET(0); 

	
	hw_ep0->max_packet_sz_tx = cfg->maxpacket;
	hw_ep0->max_packet_sz_rx = cfg->maxpacket;

	
	os_writelmskumsk(U3D_EP0CSR, hw_ep0->max_packet_sz_tx, EP0_MAXPKTSZ0, EP0_W1C_BITS);

	
	os_writel(U3D_EPIESR, os_readl(U3D_EPIESR)|EP0ISR);

	return;
}

#ifdef CONFIG_USBIF_COMPLIANCE
static int fifo_setup(struct musb *musb, struct musb_hw_ep  *hw_ep, const struct musb_fifo_cfg *cfg, u16 offset)
#else
static int __init
fifo_setup(struct musb *musb, struct musb_hw_ep  *hw_ep,
		const struct musb_fifo_cfg *cfg, u16 offset)
#endif
{
	u16	maxpacket = cfg->maxpacket;
	
	u16 ret_offset = 0;
	u32 maxpreg = 0;
	u8	mult = 0;

    
	if (maxpacket > 1024) {
		maxpreg = 1024;
		mult = (maxpacket/1024) - 1;
	} else {
		maxpreg = maxpacket;
		
		
		
		
	}

	
	if (hw_ep->epnum == 1)
		musb->bulk_ep = hw_ep;
	
	if((cfg->style == FIFO_TX) || (cfg->style == FIFO_RXTX))
	{
		hw_ep->max_packet_sz_tx = maxpreg;
		hw_ep->mult_tx = mult;

		hw_ep->fifoaddr_tx = musb->txfifoadd_offset;
		if(maxpacket == 1023){
			musb->txfifoadd_offset += (1024 * (hw_ep->mult_tx + 1));
		}
		else{
			musb->txfifoadd_offset += (maxpacket * (hw_ep->mult_tx + 1));
		}
		ret_offset = musb->txfifoadd_offset;
	}

	if((cfg->style == FIFO_RX) || (cfg->style == FIFO_RXTX))
	{
		hw_ep->max_packet_sz_rx = maxpreg;
		hw_ep->mult_rx = mult;

		hw_ep->fifoaddr_rx = musb->rxfifoadd_offset;
		if(maxpacket == 1023){
			musb->rxfifoadd_offset += (1024 * (hw_ep->mult_rx + 1));
		}
		else{
			musb->rxfifoadd_offset += (maxpacket * (hw_ep->mult_rx + 1));
		}
		ret_offset = musb->rxfifoadd_offset;
	}

	musb->epmask |= (1 << hw_ep->epnum);

   return ret_offset;

}


struct musb_fifo_cfg ep0_cfg_u3 = {
	.style = FIFO_RXTX, .maxpacket = 512,
};

struct musb_fifo_cfg ep0_cfg_u2 = {
	.style = FIFO_RXTX, .maxpacket = 64,
};

#ifdef CONFIG_USBIF_COMPLIANCE
static int ep_config_from_table(struct musb *musb)
#else
static int __init ep_config_from_table(struct musb *musb)
#endif
{
	const struct musb_fifo_cfg	*cfg;
	unsigned		i, n;
	int			offset = 0;
	struct musb_hw_ep	*hw_ep = musb->endpoints;

	if (musb->config->fifo_cfg) {
		cfg = musb->config->fifo_cfg;
		n = musb->config->fifo_cfg_size;
		os_printk(K_DEBUG, "%s: usb pre-cfg fifo_mode cfg=%p sz=%d\n", musb_driver_name, cfg, n);
		goto done;
	}

	switch (fifo_mode) {
	default:
		fifo_mode = 0;
		
	case 0:
		cfg = mode_0_cfg;
		n = ARRAY_SIZE(mode_0_cfg);
		break;
	case 1:
		cfg = mode_1_cfg;
		n = ARRAY_SIZE(mode_1_cfg);
		break;
	case 2:
		cfg = mode_2_cfg;
		n = ARRAY_SIZE(mode_2_cfg);
		break;
	case 3:
		cfg = mode_3_cfg;
		n = ARRAY_SIZE(mode_3_cfg);
		break;
	case 4:
		cfg = mode_4_cfg;
		n = ARRAY_SIZE(mode_4_cfg);
		break;
	case 5:
		cfg = mode_5_cfg;
		n = ARRAY_SIZE(mode_5_cfg);
		break;
	}

	os_printk(K_INFO,  "%s: setup fifo_mode %d\n",
			musb_driver_name, fifo_mode);


done:
	#ifdef USB_GADGET_SUPERSPEED 
	
	os_printk(K_INFO, "%s ep_config_from_table ep0_cfg_u3\n", __func__);
	ep0_setup(musb, hw_ep, &ep0_cfg_u3);
	#else 
	os_printk(K_INFO, "%s ep_config_from_table ep0_cfg_u2\n", __func__);
	ep0_setup(musb, hw_ep, &ep0_cfg_u2);
	#endif
	


	for (i = 0; i < n; i++) {
		u8	epn = cfg->hw_ep_num;

		if (epn >= musb->config->num_eps) {
			os_printk(K_ERR, "%s: invalid ep %d\n",
					musb_driver_name, epn);
			return -EINVAL;
		}
		offset = fifo_setup(musb, hw_ep + epn, cfg++, offset);
		if (offset < 0) {
			os_printk(K_ERR, "%s: mem overrun, ep %d\n",
					musb_driver_name, epn);
			return -EINVAL;
		}
		epn++;
		musb->nr_endpoints = max(epn, musb->nr_endpoints);
	}

	os_printk(K_INFO, "%s: %d/%d max ep, %d/%d memory\n",
			musb_driver_name,
			n + 1, musb->config->num_eps * 2 - 1,
			offset, (1 << (musb->config->ram_bits + 2)));

	if (!musb->bulk_ep) {
		pr_debug("%s: missing bulk\n", musb_driver_name);
		return -EINVAL;
	}

	return 0;
}


static int ep_config_from_hw(struct musb *musb)
{
	u8 epnum = 0;
	struct musb_hw_ep *hw_ep;
	void __iomem *mbase = musb->mregs;
	int ret = 0;

	dev_dbg(musb->controller, "<== static silicon ep config\n");

	

	for (epnum = 1; epnum < musb->config->num_eps; epnum++) {
		musb_ep_select(mbase, epnum);
		hw_ep = musb->endpoints + epnum;

		ret = musb_read_fifosize(musb, hw_ep, epnum);
		if (ret < 0)
			break;

		

		
		if (hw_ep->max_packet_sz_tx < 512
				|| hw_ep->max_packet_sz_rx < 512)
			continue;

		if (musb->bulk_ep)
			continue;
		musb->bulk_ep = hw_ep;
	}

	if (!musb->bulk_ep) {
		pr_debug("%s: missing bulk\n", musb_driver_name);
		return -EINVAL;
	}

	return 0;
}


enum { MUSB_CONTROLLER_MHDRC, MUSB_CONTROLLER_HDRC, };

#ifdef CONFIG_USBIF_COMPLIANCE
static int musb_core_init(u16 musb_type, struct musb *musb)
#else
static int __init musb_core_init(u16 musb_type, struct musb *musb)
#endif
{
	
	
	
	void __iomem	*mbase = musb->mregs;
	int		status = 0;
	int		i;

	
#ifdef NEVER
	
	reg = musb_read_configdata(mbase);

	strcpy(aInfo, (reg & MUSB_CONFIGDATA_UTMIDW) ? "UTMI-16" : "UTMI-8");
	if (reg & MUSB_CONFIGDATA_DYNFIFO) {
		strcat(aInfo, ", dyn FIFOs");
		musb->dyn_fifo = true;
	}
	if (reg & MUSB_CONFIGDATA_MPRXE) {
		strcat(aInfo, ", bulk combine");
		musb->bulk_combine = true;
	}
	if (reg & MUSB_CONFIGDATA_MPTXE) {
		strcat(aInfo, ", bulk split");
		musb->bulk_split = true;
	}
	if (reg & MUSB_CONFIGDATA_HBRXE) {
		strcat(aInfo, ", HB-ISO Rx");
		musb->hb_iso_rx = true;
	}
	if (reg & MUSB_CONFIGDATA_HBTXE) {
		strcat(aInfo, ", HB-ISO Tx");
		musb->hb_iso_tx = true;
	}
	if (reg & MUSB_CONFIGDATA_SOFTCONE)
		strcat(aInfo, ", SoftConn");

	printk(KERN_DEBUG "%s: ConfigData=0x%02x (%s)\n",
			musb_driver_name, reg, aInfo);

	aDate[0] = 0;
	if (MUSB_CONTROLLER_MHDRC == musb_type) {
		musb->is_multipoint = 1;
		type = "M";
	} else {
		musb->is_multipoint = 0;
		type = "";
#ifndef	CONFIG_USB_OTG_BLACKLIST_HUB
		printk(KERN_ERR
			"%s: kernel must blacklist external hubs\n",
			musb_driver_name);
#endif
	}

	
	musb->hwvers = musb_read_hwvers(mbase);
	snprintf(aRevision, 32, "%d.%d%s", MUSB_HWVERS_MAJOR(musb->hwvers),
		MUSB_HWVERS_MINOR(musb->hwvers),
		(musb->hwvers & MUSB_HWVERS_RC) ? "RC" : "");
	printk(KERN_DEBUG "%s: %sHDRC RTL version %s %s\n",
			musb_driver_name, type, aRevision, aDate);

#endif 
	musb->hwvers = os_readl(U3D_SSUSB_HW_ID);

	os_printk(K_INFO, "%s: HDC version %d\n",
			musb_driver_name, musb->hwvers);

	
	musb->txfifoadd_offset = U3D_FIFO_START_ADDRESS;
	musb->rxfifoadd_offset = U3D_FIFO_START_ADDRESS;

	os_printk(K_INFO, "%s EPnFIFOSz Tx=%x, Rx=%x\n", __func__, os_readl(U3D_CAP_EPNTXFFSZ), os_readl(U3D_CAP_EPNRXFFSZ));
	os_printk(K_INFO, "%s EPnNum Tx=%x, Rx=%d\n", __func__, os_readl(U3D_CAP_EPINFO) & 0x1F, (os_readl(U3D_CAP_EPINFO) >> 8) & 0x1F);

	if(os_readl(U3D_CAP_EPNTXFFSZ) && os_readl(U3D_CAP_EPNRXFFSZ))
		musb->dyn_fifo = true;
	else
#ifdef CONFIG_MTK_UART_USB_SWITCH
		musb->dyn_fifo = true;
#else
		musb->dyn_fifo = false;
#endif

	
	musb->nr_endpoints = 1;
	musb->epmask = 1;

	
	if (musb->dyn_fifo)
		status = ep_config_from_table(musb);
	else
		status = ep_config_from_hw(musb);

	if (status < 0)
		return status;

	
	for (i = 0; i < musb->nr_endpoints; i++) {
		struct musb_hw_ep	*hw_ep = musb->endpoints + i;

		hw_ep->fifo = (void __iomem *)(uintptr_t)MUSB_FIFO_OFFSET(i);
#ifdef CONFIG_USB_MUSB_TUSB6010
		hw_ep->fifo_async = musb->async + 0x400 + MUSB_FIFO_OFFSET(i);
		hw_ep->fifo_sync = musb->sync + 0x400 + MUSB_FIFO_OFFSET(i);
		hw_ep->fifo_sync_va =
			musb->sync_va + 0x400 + MUSB_FIFO_OFFSET(i);

		if (i == 0)
			hw_ep->conf = mbase - 0x400 + TUSB_EP0_CONF;
		else
			hw_ep->conf = mbase + 0x400 + (((i - 1) & 0xf) << 2);
#endif

		
		hw_ep->addr_txcsr0 = (void __iomem *)(uintptr_t)SSUSB_EP_TXCR0_OFFSET(i, 0);
		hw_ep->addr_txcsr1 = (void __iomem *)(uintptr_t)SSUSB_EP_TXCR1_OFFSET(i, 0);
		hw_ep->addr_txcsr2 = (void __iomem *)(uintptr_t)SSUSB_EP_TXCR2_OFFSET(i, 0);
		hw_ep->addr_rxcsr0 = (void __iomem *)(uintptr_t)SSUSB_EP_RXCR0_OFFSET(i, 0);
		hw_ep->addr_rxcsr1 = (void __iomem *)(uintptr_t)SSUSB_EP_RXCR1_OFFSET(i, 0);
		hw_ep->addr_rxcsr2 = (void __iomem *)(uintptr_t)SSUSB_EP_RXCR2_OFFSET(i, 0);
		hw_ep->addr_rxcsr3 = (void __iomem *)(uintptr_t)SSUSB_EP_RXCR3_OFFSET(i, 0);

		hw_ep->target_regs = musb_read_target_reg_base(i, mbase);
		hw_ep->rx_reinit = 1;
		hw_ep->tx_reinit = 1;

		if (hw_ep->max_packet_sz_tx) {
			dev_dbg(musb->controller,
				"%s: hw_ep %d%s, %smax %d\n",
				musb_driver_name, i,
				hw_ep->is_shared_fifo ? "shared" : "tx",
				"",
				hw_ep->max_packet_sz_tx);
		}
		if (hw_ep->max_packet_sz_rx && !hw_ep->is_shared_fifo) {
			dev_dbg(musb->controller,
				"%s: hw_ep %d%s, %smax %d\n",
				musb_driver_name, i,
				"rx",
				"",
				hw_ep->max_packet_sz_rx);
		}
		if (!(hw_ep->max_packet_sz_tx || hw_ep->max_packet_sz_rx))
			dev_dbg(musb->controller, "hw_ep %d not configured\n", i);
	}

#ifdef USE_SSUSB_QMU
	
	_ex_mu3d_hal_alloc_qmu_mem(musb->controller);
	
	_ex_mu3d_hal_init_qmu();

	musb_save_context(musb);
#endif

	return 0;
}

irqreturn_t musb_interrupt(struct musb *musb)
{
	irqreturn_t	retval = IRQ_NONE;
	u8		devctl, power = 0;
#ifndef USE_SSUSB_QMU
	u32 	reg = 0, ep_num = 0;
#endif

#ifdef POWER_SAVING_MODE
	if(!(os_readl(U3D_SSUSB_U2_CTRL_0P) & SSUSB_U2_PORT_PDN)){
		devctl = (u8)os_readl(U3D_DEVICE_CONTROL);
		power = (u8)os_readl(U3D_POWER_MANAGEMENT);
	}else{
		devctl = 0;
		power = 0;
		musb->int_usb = 0;
	}
#else
	devctl = (u8)os_readl(U3D_DEVICE_CONTROL);
	power = (u8)os_readl(U3D_POWER_MANAGEMENT);
#endif

	
	os_printk(K_DEBUG, "IRQ %s usb%04x tx%04x rx%04x\n",
		(devctl & USB_DEVCTL_HOSTMODE) ? "host" : "peripheral",
		musb->int_usb, musb->int_tx, musb->int_rx);

	if (musb->int_usb)
		retval |= musb_stage0_irq(musb, musb->int_usb,
				devctl, power);

	

	
	if (musb->int_tx & 1) {
		retval |= musb_g_ep0_irq(musb);
	}

#ifndef USE_SSUSB_QMU
	
	reg = musb->int_rx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			
			
			retval = IRQ_HANDLED;
			musb_g_rx(musb, ep_num);
		}

		reg >>= 1;
		ep_num++;
	}

	
	reg = musb->int_tx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			
			
			retval = IRQ_HANDLED;
			musb_g_tx(musb, ep_num);
		}
		reg >>= 1;
		ep_num++;
	}
#endif

	return retval;
}
EXPORT_SYMBOL_GPL(musb_interrupt);

#ifndef CONFIG_USB_MU3D_PIO_ONLY
static bool use_dma = 1;

module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");

void musb_dma_completion(struct musb *musb, u8 epnum, u8 transmit)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	

	if (!epnum) {
#ifndef CONFIG_USB_TUSB_OMAP_DMA
		if (!is_cppi_enabled()) {
			
			if (devctl & MUSB_DEVCTL_HM)
				
				
				os_printk(K_DEBUG, "Call musb_h_ep0_irq(), AYKM???!!!\n");
			else
				musb_g_ep0_irq(musb);
		}
#endif
	} else {
		
		if (transmit) {
			if (devctl & MUSB_DEVCTL_HM) {
				if (is_host_capable())
					
					
					os_printk(K_DEBUG, "Call musb_host_tx(), AYKM???!!!\n");
			} else {
				if (is_peripheral_capable())
					musb_g_tx(musb, epnum);
			}
		} else {
			
			if (devctl & MUSB_DEVCTL_HM) {
				if (is_host_capable())
					
					
					os_printk(K_DEBUG, "Call musb_host_tx(), AYKM???!!!\n");
			} else {
				if (is_peripheral_capable())
					musb_g_rx(musb, epnum);
			}
		}
	}
}

#else
#define use_dma			0
#endif


#ifdef CONFIG_SYSFS

static ssize_t
musb_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct musb *musb = dev_to_musb(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&musb->lock, flags);
	ret = sprintf(buf, "%s\n", usb_otg_state_string(musb->xceiv->state));
	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;
}

static ssize_t
musb_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	int		status;

	spin_lock_irqsave(&musb->lock, flags);
	if (sysfs_streq(buf, "host"))
		status = musb_platform_set_mode(musb, MUSB_HOST);
	else if (sysfs_streq(buf, "peripheral"))
		status = musb_platform_set_mode(musb, MUSB_PERIPHERAL);
	else if (sysfs_streq(buf, "otg"))
		status = musb_platform_set_mode(musb, MUSB_OTG);
	else
		status = -EINVAL;
	spin_unlock_irqrestore(&musb->lock, flags);

	return (status == 0) ? n : status;
}
static DEVICE_ATTR(mode, 0644, musb_mode_show, musb_mode_store);

static ssize_t
musb_vbus_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;

	if (sscanf(buf, "%lu", &val) < 1) {
		dev_err(dev, "Invalid VBUS timeout ms value\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&musb->lock, flags);
	
	musb->a_wait_bcon = val ? max_t(int, val, OTG_TIME_A_WAIT_BCON) : 0 ;
	if (musb->xceiv->state == OTG_STATE_A_WAIT_BCON)
		musb->is_active = 0;
	musb_platform_try_idle(musb, jiffies + msecs_to_jiffies(val));
	spin_unlock_irqrestore(&musb->lock, flags);

	return n;
}

static ssize_t
musb_vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;
	int		vbus;

	spin_lock_irqsave(&musb->lock, flags);
	val = musb->a_wait_bcon;
	vbus = musb_platform_get_vbus_status(musb);
	spin_unlock_irqrestore(&musb->lock, flags);

	return sprintf(buf, "Vbus %s, timeout %lu msec\n",
			vbus ? "on" : "off", val);
}
static DEVICE_ATTR(vbus, 0644, musb_vbus_show, musb_vbus_store);

static ssize_t
musb_srp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned short	srp;

	if (sscanf(buf, "%hu", &srp) != 1
			|| (srp != 1)) {
		dev_err(dev, "SRP: Value must be 1\n");
		return -EINVAL;
	}

	if (srp == 1)
		musb_g_wakeup(musb);

	return n;
}
static DEVICE_ATTR(srp, 0644, NULL, musb_srp_store);

extern ssize_t musb_cmode_show(struct device* dev, struct device_attribute *attr, char *buf);
extern ssize_t musb_cmode_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count);

DEVICE_ATTR(cmode, 0664, musb_cmode_show, musb_cmode_store);

#ifdef CONFIG_MTK_UART_USB_SWITCH
extern ssize_t musb_portmode_show(struct device* dev, struct device_attribute *attr, char *buf);
extern ssize_t musb_portmode_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t musb_tx_show(struct device* dev, struct device_attribute *attr, char *buf);
extern ssize_t musb_tx_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t musb_rx_show(struct device* dev, struct device_attribute *attr, char *buf);
extern ssize_t musb_uart_path_show(struct device* dev, struct device_attribute *attr, char *buf);

DEVICE_ATTR(portmode, 0664, musb_portmode_show, musb_portmode_store);
DEVICE_ATTR(tx,  0664, musb_tx_show, musb_tx_store);
DEVICE_ATTR(rx,  0444, musb_rx_show, NULL);
DEVICE_ATTR(uartpath,  0444, musb_uart_path_show, NULL);
#endif

static struct attribute *musb_attributes[] = {
	&dev_attr_mode.attr,
	&dev_attr_vbus.attr,
	&dev_attr_srp.attr,
	&dev_attr_cmode.attr,
	#ifdef CONFIG_MTK_UART_USB_SWITCH
    &dev_attr_portmode.attr,
    &dev_attr_tx.attr,
    &dev_attr_rx.attr,
    &dev_attr_uartpath.attr,
    #endif
	NULL
};

static const struct attribute_group musb_attr_group = {
	.attrs = musb_attributes,
};

#endif	

static void musb_save_context(struct musb *musb)
{
	int i;

	for (i = 0; i < musb->config->num_eps; ++i) {
		os_printk(K_DEBUG, "%s EP%d\n", __func__, i);
#ifdef USE_SSUSB_QMU
		
		musb->context.index_regs[i].txqmuaddr = os_readl(USB_QMU_TQSAR(i+1));
		os_printk(K_DEBUG, "%s TQSAR[%d]=%x\n", __func__, i, musb->context.index_regs[i].txqmuaddr);
		musb->context.index_regs[i].rxqmuaddr = os_readl(USB_QMU_RQSAR(i+1));
		os_printk(K_DEBUG, "%s RQSAR[%d]=%x\n", __func__, i, musb->context.index_regs[i].rxqmuaddr);
#endif
	}
}

static void musb_restore_context(struct musb *musb)
{
	int i;

	for (i = 0; i < musb->config->num_eps; ++i) {
#ifdef USE_SSUSB_QMU
		os_writel(USB_QMU_TQSAR(i+1), musb->context.index_regs[i].txqmuaddr);
		os_writel(USB_QMU_RQSAR(i+1), musb->context.index_regs[i].rxqmuaddr);
		os_printk(K_DEBUG, "%s TQSAR[%d]=%x\n", __func__, i, os_readl(USB_QMU_TQSAR(i+1)));
		os_printk(K_DEBUG, "%s TQSAR[%d]=%x\n", __func__, i, os_readl(USB_QMU_RQSAR(i+1)));
#endif
	}
}

static void musb_suspend_work(struct work_struct *data)
{
	struct musb *musb = container_of(data, struct musb, suspend_work);

	os_printk(K_INFO, "%s active_ep=%d, clk_on=%d\n", __func__, musb->active_ep, musb->is_clk_on);

	if (musb->is_clk_on == 1 && (!usb_cable_connected() || (musb->usb_mode != CABLE_MODE_NORMAL))) {

#ifdef EP_PROFILING
		cancel_delayed_work_sync(&musb->ep_prof_work);
#endif
		musb_save_context(musb);

		
		os_setmsk(U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);

#ifndef CONFIG_MTK_FPGA
		
		usb_phy_savecurrent(musb->is_clk_on);
		musb->is_clk_on = 0;
#endif
	}
}

static void musb_irq_work(struct work_struct *data)
{
	struct musb *musb = container_of(data, struct musb, irq_work);
	static int old_state;

	os_printk(K_INFO, "%s [%d]=[%d]\n", __func__, musb->xceiv->state, old_state);

	if (musb->xceiv->state != old_state) {
		old_state = musb->xceiv->state;
		sysfs_notify(&musb->controller->kobj, NULL, "mode");
	}

}
const struct hc_driver musb_hc_driver = {
	.description		= "musb-hcd",
	.product_desc		= "MUSB HDRC host driver",
	.hcd_priv_size		= sizeof(struct musb),
	.flags			= HCD_USB2 | HCD_MEMORY,
};

#ifdef CONFIG_USBIF_COMPLIANCE
static struct musb * allocate_instance(struct device *dev,
		struct musb_hdrc_config *config, void __iomem *mbase)
#else
static struct musb *__init
allocate_instance(struct device *dev,
		struct musb_hdrc_config *config, void __iomem *mbase)
#endif
{
	struct musb		*musb;
	struct musb_hw_ep	*ep;
	int			epnum;
	struct usb_hcd	*hcd;

	hcd = usb_create_hcd(&musb_hc_driver, dev, dev_name(dev));
	if (!hcd)
		return NULL;
	

	musb = hcd_to_musb(hcd);
	INIT_LIST_HEAD(&musb->control);
	INIT_LIST_HEAD(&musb->in_bulk);
	INIT_LIST_HEAD(&musb->out_bulk);

	hcd->uses_new_polling = 1;
	hcd->has_tt = 1;

	musb->vbuserr_retry = VBUSERR_RETRY_COUNT;
	musb->a_wait_bcon = OTG_TIME_A_WAIT_BCON;
	dev_set_drvdata(dev, musb);
	musb->mregs = mbase;
	musb->ctrl_base = mbase;
	musb->nIrq = -ENODEV;
	musb->config = config;
	BUG_ON(musb->config->num_eps > MUSB_C_NUM_EPS);
	for (epnum = 0, ep = musb->endpoints;
			epnum < musb->config->num_eps;
			epnum++, ep++) {
		ep->musb = musb;
		ep->epnum = epnum;
	}

	musb->controller = dev;

	
	
	
	

	return musb;
}

static void musb_free(struct musb *musb)
{

#ifdef CONFIG_SYSFS
	sysfs_remove_group(&musb->controller->kobj, &musb_attr_group);
#endif

	musb_gadget_cleanup(musb);

	if (musb->nIrq >= 0) {
		if (musb->irq_wake)
			disable_irq_wake(musb->nIrq);
		free_irq(musb->nIrq, musb);
	}

#ifdef USE_SSUSB_QMU
	tasklet_kill(&musb->qmu_done);
	tasklet_kill(&musb->error_recovery);
#endif

	cancel_work_sync(&musb->irq_work);
	cancel_delayed_work_sync(&musb->connection_work);
	
	cancel_work_sync(&musb->suspend_work);

#ifdef USE_SSUSB_QMU
	_ex_mu3d_hal_free_qmu_mem(musb->controller);
#endif
	wake_lock_destroy(&musb->usb_wakelock);

	
#ifdef CONFIG_USBIF_COMPLIANCE
	
	
	
#else
	
	usb_put_hcd(musb_to_hcd(musb));

	kfree(musb->xceiv); 
	musb->xceiv = NULL;

	kfree(musb);
#endif

}

#ifdef CONFIG_USBIF_COMPLIANCE
static int musb_init_controller(struct device *dev, int nIrq, void __iomem *ctrl)
#else
static int __init
musb_init_controller(struct device *dev, int nIrq, void __iomem *ctrl)
#endif
{
	int			status;
	struct musb		*musb;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct usb_hcd		*hcd;


	os_printk(K_INFO, "[MU3D]%s\n", __func__);

	if (!plat) {
		dev_dbg(dev, "no platform_data?\n");
		status = -ENODEV;
		goto fail0;
	}

	
	musb = allocate_instance(dev, plat->config, ctrl);
	if (!musb) {
		status = -ENOMEM;
		goto fail0;
	}

	
	
	

	spin_lock_init(&musb->lock);
	musb->board_mode = plat->mode;
	musb->board_set_power = plat->set_power;
	musb->min_power = plat->min_power;
	musb->ops = plat->platform_ops;
	musb->usb_mode = CABLE_MODE_NORMAL;

	_mu3d_musb = musb;

	wake_lock_init(&musb->usb_wakelock, WAKE_LOCK_SUSPEND, "USB.lock");

	INIT_DELAYED_WORK(&musb->connection_work, connection_work);

	INIT_DELAYED_WORK(&musb->check_ltssm_work, check_ltssm_work);

#ifndef CONFIG_USBIF_COMPLIANCE
	INIT_DELAYED_WORK(&musb->reconnect_work, reconnect_work);
#endif

#ifdef EP_PROFILING
	INIT_DELAYED_WORK(&musb->ep_prof_work, ep_prof_work);
#endif

	
	
	status = musb_platform_init(musb);
	if (status < 0)
		goto fail1;

	if (!musb->isr) {
		status = -ENODEV;
		goto fail3;
	}

	

	
#ifdef USE_SSUSB_QMU
	if (!is_dma_capable())
#else
	if (!is_dma_capable() || !musb->dma_controller)
#endif
		dev->dma_mask = NULL;

	
	musb_platform_disable(musb);
	musb_generic_disable();

	
	status = musb_core_init(plat->config->multipoint
			? MUSB_CONTROLLER_MHDRC
			: MUSB_CONTROLLER_HDRC, musb);
	if (status < 0)
		goto fail3;

	
	

	
	INIT_WORK(&musb->irq_work, musb_irq_work);

	INIT_WORK(&musb->suspend_work, musb_suspend_work);

#ifdef USE_SSUSB_QMU
	tasklet_init(&musb->qmu_done, qmu_done_tasklet, (unsigned long)musb);
	tasklet_init(&musb->error_recovery, qmu_error_recovery, (unsigned long)musb);
#endif

	
	if (request_irq(nIrq, musb->isr, IRQF_TRIGGER_LOW, dev_name(dev), musb)) {
		dev_err(dev, "request_irq %d failed!\n", nIrq);
		status = -ENODEV;
		goto fail3;
	}
	musb->nIrq = nIrq;
	
	if (enable_irq_wake(nIrq) == 0) {
		musb->irq_wake = 1;
		device_init_wakeup(dev, 1);
	} else {
		musb->irq_wake = 0;
	}

	
#ifndef CONFIG_USBIF_COMPLIANCE
	if (is_host_enabled(musb)) {
		hcd = musb_to_hcd(musb);
		otg_set_host(musb->xceiv->otg, &hcd->self);

		if (is_otg_enabled(musb))
			hcd->self.otg_port = 1;

		musb->xceiv->otg->host = &hcd->self;
		hcd->power_budget = 2 * (plat->power ? : 250);

		
		if (plat->extvbus) {
			u8 busctl = musb_read_ulpi_buscontrol(musb->mregs);
			busctl |= MUSB_ULPI_USE_EXTVBUS;
			musb_write_ulpi_buscontrol(musb->mregs, busctl);
		}
	}
#endif

	MUSB_DEV_MODE(musb);
	musb->xceiv->otg->default_a = 0;
	musb->xceiv->state = OTG_STATE_B_IDLE;

	status = musb_gadget_setup(musb);

	if (status < 0)
		goto fail3;

	status = musb_init_debugfs(musb);
	if (status < 0)
		goto fail4;

#ifdef CONFIG_SYSFS
	status = sysfs_create_group(&musb->controller->kobj, &musb_attr_group);
	if (status)
		goto fail5;
#endif

	pm_runtime_put(musb->controller);

	dev_info(dev, "USB %s mode controller at %p using %s, IRQ %d\n",
			({char *s;
			 switch (musb->board_mode) {
			 case MUSB_HOST:		s = "Host"; break;
			 case MUSB_PERIPHERAL:	s = "Peripheral"; break;
			 default:		s = "OTG"; break;
			 }; s; }),
			ctrl,
			(is_dma_capable() && musb->dma_controller)
			? "DMA" : "PIO",
			musb->nIrq);

	return 0;

fail5:
	musb_exit_debugfs(musb);

fail4:
	if (!is_otg_enabled(musb) && is_host_enabled(musb))
		usb_remove_hcd(musb_to_hcd(musb));
	else
		musb_gadget_cleanup(musb);

fail3:
	if (musb->irq_wake)
		device_init_wakeup(dev, 0);
	musb_platform_exit(musb);

fail1:
	dev_err(musb->controller,
		"musb_init_controller failed with status %d\n", status);

	musb_free(musb);

fail0:

	return status;

}

#define USB3_BASE_REGS_ADDR_RES_NAME "ssusb_base"
#define USB3_SIF_REGS_ADDR_RES_NAME "ssusb_sif"
#define USB3_SIF2_REGS_ADDR_RES_NAME "ssusb_sif2"

static void __iomem *acquire_reg_base(struct platform_device *pdev, const char *res_name)
{
	struct resource	*iomem;
	void __iomem *base = NULL;

	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!iomem) {
		pr_err("Can't get resource for %s\n", res_name);
		goto end;
	}

	base = ioremap(iomem->start, resource_size(iomem));
	if (!(uintptr_t)base) {
		pr_err("Can't remap %s\n", res_name);
		goto end;
	}
	os_printk(K_INFO, "%s=0x%lx\n", res_name, (uintptr_t)(base));
end:
	return base;
}


#ifdef CONFIG_USBIF_COMPLIANCE
static int mu3d_normal_driver_on = 0 ;

static int musb_probe(struct platform_device *pdev)
#else
static int __init musb_probe(struct platform_device *pdev)
#endif
{
	struct device	*dev = &pdev->dev;
	int		irq = platform_get_irq_byname(pdev, MUSB_DRIVER_NAME);
	int		status = 0;
#ifdef CONFIG_MTK_UART_USB_SWITCH
	struct device_node *ap_uart0_node = NULL;
#endif
	if (irq <= 0)
		return -ENODEV;

	os_printk(K_INFO, "[MU3D]musb_probe irq=%d\n", irq);

	u3_base = acquire_reg_base(pdev, USB3_BASE_REGS_ADDR_RES_NAME);
	if (!u3_base)
		goto exit_regs;

	u3_sif_base = acquire_reg_base(pdev, USB3_SIF_REGS_ADDR_RES_NAME);
	if (!u3_sif_base)
		goto exit_regs;

	u3_sif2_base = acquire_reg_base(pdev, USB3_SIF2_REGS_ADDR_RES_NAME);
	if (!u3_sif2_base)
		goto exit_regs;

#ifdef CONFIG_MTK_UART_USB_SWITCH
	ap_uart0_node = of_find_compatible_node(NULL, NULL, AP_UART0_COMPATIBLE_NAME);

	if(ap_uart0_node == NULL) {
		os_printk(K_ERR, "USB get ap_uart0_node failed\n");
		if (ap_uart0_base)
			iounmap(ap_uart0_base);
		ap_uart0_base = 0;
	} else {
		ap_uart0_base = of_iomap(ap_uart0_node, 0);
	}
#endif

#ifdef CONFIG_MTK_FPGA
	i2c1_base = ioremap(0x11008000, 0x1000);
	if (!(i2c1_base)) {
		pr_err("Can't remap I2C1 BASE\n");
		status = -ENOMEM;
	}
	os_printk(K_INFO, "I2C1 BASE=0x%lx\n", (uintptr_t)(i2c1_base));
#endif

	status = musb_init_controller(dev, irq, u3_base);
	if (status < 0)
		goto exit_regs;

	return status;

exit_regs:
	if (u3_base)
		iounmap(u3_base);
	if (u3_sif_base)
		iounmap(u3_sif_base);
	if (u3_sif2_base)
		iounmap(u3_sif2_base);
	u3_base = 0;
	u3_sif_base = 0;
	u3_sif2_base = 0;

	return status;
}

static int musb_remove(struct platform_device *pdev)
{
	struct musb	*musb = dev_to_musb(&pdev->dev);
	void __iomem	*ctrl_base = musb->ctrl_base;


#ifdef CONFIG_SYSFS 
	sysfs_remove_group(&musb->controller->kobj, &musb_attr_group);
#endif
	pm_runtime_get_sync(musb->controller);
	musb_exit_debugfs(musb);
	musb_shutdown(pdev);

	pm_runtime_put(musb->controller);
	musb_free(musb);
	_mu3d_musb = NULL;
#ifndef CONFIG_USBIF_COMPLIANCE
	
	iounmap(ctrl_base);
#endif
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

#ifdef NEVER

#ifdef NEVER
static void musb_save_context(struct musb *musb)
{
	int i;
	void __iomem *musb_base = musb->mregs;
	void __iomem *epio;

	if (is_host_enabled(musb)) {
		musb->context.frame = musb_readw(musb_base, MUSB_FRAME);
		musb->context.testmode = musb_readb(musb_base, MUSB_TESTMODE);
		musb->context.busctl = musb_read_ulpi_buscontrol(musb->mregs);
	}
	musb->context.power = musb_readb(musb_base, MUSB_POWER);
	musb->context.intrtxe = musb_readw(musb_base, MUSB_INTRTXE);
	musb->context.intrrxe = musb_readw(musb_base, MUSB_INTRRXE);
	musb->context.intrusbe = musb_readb(musb_base, MUSB_INTRUSBE);
	musb->context.index = musb_readb(musb_base, MUSB_INDEX);
	musb->context.devctl = musb_readb(musb_base, MUSB_DEVCTL);

	for (i = 0; i < musb->config->num_eps; ++i) {
		struct musb_hw_ep	*hw_ep;

		hw_ep = &musb->endpoints[i];
		if (!hw_ep)
			continue;

		epio = hw_ep->regs;
		if (!epio)
			continue;

		musb->context.index_regs[i].txmaxp =
			musb_readw(epio, MUSB_TXMAXP);
		musb->context.index_regs[i].txcsr =
			musb_readw(epio, MUSB_TXCSR);
		musb->context.index_regs[i].rxmaxp =
			musb_readw(epio, MUSB_RXMAXP);
		musb->context.index_regs[i].rxcsr =
			musb_readw(epio, MUSB_RXCSR);

		if (musb->dyn_fifo) {
			musb->context.index_regs[i].txfifoadd =
					musb_read_txfifoadd(musb_base);
			musb->context.index_regs[i].rxfifoadd =
					musb_read_rxfifoadd(musb_base);
			musb->context.index_regs[i].txfifosz =
					musb_read_txfifosz(musb_base);
			musb->context.index_regs[i].rxfifosz =
					musb_read_rxfifosz(musb_base);
		}
		if (is_host_enabled(musb)) {
			musb->context.index_regs[i].txtype =
				musb_readb(epio, MUSB_TXTYPE);
			musb->context.index_regs[i].txinterval =
				musb_readb(epio, MUSB_TXINTERVAL);
			musb->context.index_regs[i].rxtype =
				musb_readb(epio, MUSB_RXTYPE);
			musb->context.index_regs[i].rxinterval =
				musb_readb(epio, MUSB_RXINTERVAL);

			musb->context.index_regs[i].txfunaddr =
				musb_read_txfunaddr(musb_base, i);
			musb->context.index_regs[i].txhubaddr =
				musb_read_txhubaddr(musb_base, i);
			musb->context.index_regs[i].txhubport =
				musb_read_txhubport(musb_base, i);

			musb->context.index_regs[i].rxfunaddr =
				musb_read_rxfunaddr(musb_base, i);
			musb->context.index_regs[i].rxhubaddr =
				musb_read_rxhubaddr(musb_base, i);
			musb->context.index_regs[i].rxhubport =
				musb_read_rxhubport(musb_base, i);
		}
	}
}

static void musb_restore_context(struct musb *musb)
{
	int i;
	void __iomem *musb_base = musb->mregs;
	void __iomem *ep_target_regs;
	void __iomem *epio;

	if (is_host_enabled(musb)) {
		musb_writew(musb_base, MUSB_FRAME, musb->context.frame);
		musb_writeb(musb_base, MUSB_TESTMODE, musb->context.testmode);
		musb_write_ulpi_buscontrol(musb->mregs, musb->context.busctl);
	}
	musb_writeb(musb_base, MUSB_POWER, musb->context.power);
	musb_writew(musb_base, MUSB_INTRTXE, musb->context.intrtxe);
	musb_writew(musb_base, MUSB_INTRRXE, musb->context.intrrxe);
	musb_writeb(musb_base, MUSB_INTRUSBE, musb->context.intrusbe);
	musb_writeb(musb_base, MUSB_DEVCTL, musb->context.devctl);

	for (i = 0; i < musb->config->num_eps; ++i) {
		struct musb_hw_ep	*hw_ep;

		hw_ep = &musb->endpoints[i];
		if (!hw_ep)
			continue;

		epio = hw_ep->regs;
		if (!epio)
			continue;

		musb_writew(epio, MUSB_TXMAXP,
			musb->context.index_regs[i].txmaxp);
		musb_writew(epio, MUSB_TXCSR,
			musb->context.index_regs[i].txcsr);
		musb_writew(epio, MUSB_RXMAXP,
			musb->context.index_regs[i].rxmaxp);
		musb_writew(epio, MUSB_RXCSR,
			musb->context.index_regs[i].rxcsr);

		if (musb->dyn_fifo) {
			musb_write_txfifosz(musb_base,
				musb->context.index_regs[i].txfifosz);
			musb_write_rxfifosz(musb_base,
				musb->context.index_regs[i].rxfifosz);
			musb_write_txfifoadd(musb_base,
				musb->context.index_regs[i].txfifoadd);
			musb_write_rxfifoadd(musb_base,
				musb->context.index_regs[i].rxfifoadd);
		}

		if (is_host_enabled(musb)) {
			musb_writeb(epio, MUSB_TXTYPE,
				musb->context.index_regs[i].txtype);
			musb_writeb(epio, MUSB_TXINTERVAL,
				musb->context.index_regs[i].txinterval);
			musb_writeb(epio, MUSB_RXTYPE,
				musb->context.index_regs[i].rxtype);
			musb_writeb(epio, MUSB_RXINTERVAL,

			musb->context.index_regs[i].rxinterval);
			musb_write_txfunaddr(musb_base, i,
				musb->context.index_regs[i].txfunaddr);
			musb_write_txhubaddr(musb_base, i,
				musb->context.index_regs[i].txhubaddr);
			musb_write_txhubport(musb_base, i,
				musb->context.index_regs[i].txhubport);

			ep_target_regs =
				musb_read_target_reg_base(i, musb_base);

			musb_write_rxfunaddr(ep_target_regs,
				musb->context.index_regs[i].rxfunaddr);
			musb_write_rxhubaddr(ep_target_regs,
				musb->context.index_regs[i].rxhubaddr);
			musb_write_rxhubport(ep_target_regs,
				musb->context.index_regs[i].rxhubport);
		}
	}
	musb_writeb(musb_base, MUSB_INDEX, musb->context.index);
}
#endif 

static void musb_save_context(struct musb *musb)
{
	int i;

#ifdef CONFIG_USB_MU3D_DRV
#ifdef USE_SSUSB_QMU
#endif
#endif 
	for (i = 0; i < musb->config->num_eps; ++i) {
		os_printk(K_DEBUG, "%s EP%d\n", __func__, i);
#ifdef CONFIG_USB_MU3D_DRV
#ifdef USE_SSUSB_QMU
		
		musb->context.index_regs[i].txqmuaddr = os_readl(USB_QMU_TQSAR(i+1));
		os_printk(K_DEBUG, "%s TQSAR[%d]=%x\n", __func__, i, musb->context.index_regs[i].txqmuaddr);
		musb->context.index_regs[i].rxqmuaddr = os_readl(USB_QMU_RQSAR(i+1));
		os_printk(K_DEBUG, "%s RQSAR[%d]=%x\n", __func__, i, musb->context.index_regs[i].rxqmuaddr);
#endif
#endif 
	}
}

static void musb_restore_context(struct musb *musb)
{
	int i;

#ifdef CONFIG_USB_MU3D_DRV
#ifdef USE_SSUSB_QMU
#endif
#endif 

	for (i = 0; i < musb->config->num_eps; ++i) {
#ifdef CONFIG_USB_MU3D_DRV
#ifdef USE_SSUSB_QMU
		os_writel(USB_QMU_TQSAR(i+1), musb->context.index_regs[i].txqmuaddr);
		os_writel(USB_QMU_RQSAR(i+1), musb->context.index_regs[i].rxqmuaddr);
		os_printk(K_INFO, "%s TQSAR[%d]=%x\n", __func__, i, os_readl(USB_QMU_TQSAR(i+1)));
		os_printk(K_INFO, "%s TQSAR[%d]=%x\n", __func__, i, os_readl(USB_QMU_RQSAR(i+1)));
#endif
#endif 
	}
}

static int musb_suspend_noirq(struct device *dev)
{
	struct musb	*musb = dev_to_musb(dev);

	os_printk(K_INFO, "%s\n", __func__);
	musb_save_context(musb);

	
	os_setmsk(U3D_SSUSB_IP_PW_CTRL0, SSUSB_IP_SW_RST);

#ifndef CONFIG_MTK_FPGA
	
	usb_phy_savecurrent(musb->is_clk_on);
	musb->is_clk_on = 0;
#endif


	return 0;
}

static int musb_resume_noirq(struct device *dev)
{
	struct musb	*musb = dev_to_musb(dev);

	os_printk(K_INFO, "%s\n", __func__);

#ifndef CONFIG_MTK_FPGA
	
	usb_phy_recover(musb->is_clk_on);
	musb->is_clk_on = 1;

	
	u3phy_ops->u2_slew_rate_calibration(u3phy);
#endif

	
	_ex_mu3d_hal_ssusb_en();

	
	mu3d_hal_rst_dev();

	musb_restore_context(musb);

	return 0;
}

static const struct dev_pm_ops musb_dev_pm_ops = {
	.suspend_noirq	= musb_suspend_noirq,
	.resume_noirq	= musb_resume_noirq,
};

#define MUSB_DEV_PM_OPS (&musb_dev_pm_ops)

#else 

#ifdef CONFIG_MTK_UART_USB_SWITCH
#define MUSB_DEV_PM_OPS (&musb_dev_pm_ops)
extern void usb_phy_switch_to_uart(void);

static int musb_suspend_noirq(struct device *dev)
{
	os_printk(K_INFO, "%s: for CONFIG_MTK_UART_USB_SWITCH: in_uart_mode: %d\n", __func__, in_uart_mode);

	return 0;
}

static int musb_resume_noirq(struct device *dev)
{
	os_printk(K_INFO, "%s: for CONFIG_MTK_UART_USB_SWITCH: in_uart_mode: %d\n", __func__, in_uart_mode);

	if (in_uart_mode == true)
		usb_phy_switch_to_uart();

	return 0;
}

static const struct dev_pm_ops musb_dev_pm_ops = {
	.suspend_noirq	= musb_suspend_noirq,
	.resume_noirq	= musb_resume_noirq,
};
#else
#define	MUSB_DEV_PM_OPS	NULL
#endif 
#endif 

static struct platform_driver musb_driver = {
	.driver = {
		.name		= (char *)musb_driver_name,
		.bus		= &platform_bus_type,
		.owner		= THIS_MODULE,
		.pm		= MUSB_DEV_PM_OPS,
	},
	.probe		= musb_probe,
	.remove		= musb_remove,
	.shutdown	= musb_shutdown,
};

#ifdef CONFIG_USBIF_COMPLIANCE
extern void init_connection_work(void) ;
extern void init_check_ltssm_work(void) ;
extern void Charger_Detect_En(bool enable) ;

static int musb_mu3d_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "musb_mu3d_proc_show, mu3d is %d (on:1, off:0)\n", mu3d_normal_driver_on);
	return 0;
}

static int musb_mu3d_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, musb_mu3d_proc_show, inode->i_private);
}

static ssize_t musb_mu3d_proc_write(struct file *file, const char __user *buf, size_t length, loff_t *ppos)
{
	int ret ;
	char msg[32] ;
	int result;
	int status;
	struct device	*dev ;
	int		irq ;
	struct resource	*iomem;
	void __iomem	*base;
	struct musb *musb ;
	void __iomem	*ctrl_base;

	if (length >= sizeof(msg)) {
		os_printk(K_ERR, "musb_mu3d_proc_write length error, the error len is %d\n", (unsigned int)length);
		return -EINVAL;
	}
	if (copy_from_user(msg, buf, length))
		return -EFAULT;

	msg[length] = 0 ;

	os_printk(K_DEBUG, "musb_mu3d_proc_write: %s, current driver on/off: %d\n", msg, mu3d_normal_driver_on);

	if ((msg[0] == '1') && (mu3d_normal_driver_on == 0)){
		os_printk(K_DEBUG, "registe mu3d driver ===> \n");
		init_connection_work() ;
		init_check_ltssm_work() ;
		platform_driver_register(&musb_driver);
		mu3d_normal_driver_on = 1 ;
		Charger_Detect_En(true) ;
		os_printk(K_DEBUG, "registe mu3d driver <=== \n");
	}else if ((msg[0] == '0') && (mu3d_normal_driver_on == 1)){
		os_printk(K_DEBUG, "unregiste mu3d driver ===> \n");
		mu3d_normal_driver_on = 0 ;
		Charger_Detect_En(false) ;
		platform_driver_unregister(&musb_driver);
		os_printk(K_DEBUG, "unregiste mu3d driver <=== \n");
	}else{
		
		
		os_printk(K_ERR, "musb_mu3d_proc_write , set reboot !\n");
		
	}
	return length;
}

static const struct file_operations mu3d_proc_fops = {
	.owner = THIS_MODULE,
	.open = musb_mu3d_proc_open,
	.write = musb_mu3d_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,

};

static int __init musb_init(void)
{
	struct proc_dir_entry *prEntry;
	int ret = 0;

	if (usb_disabled())
		return 0;

	pr_info("%s: version " MUSB_VERSION ", ?dma?, otg (peripheral+host)\n",
		musb_driver_name);

	
	prEntry = proc_create("mu3d_driver_init", 0666, NULL, &mu3d_proc_fops);

	if (prEntry)
	{
		os_printk(K_ERR,"create the mu3d init proc OK!\n") ;
	}else{
		os_printk(K_ERR,"[ERROR] create the mu3d init proc FAIL\n") ;
	}

	
	ret= platform_driver_register(&musb_driver);
	mu3d_normal_driver_on = 1 ;
	Charger_Detect_En(true) ;

	return ret ;
}
module_init(musb_init);

static void __exit musb_cleanup(void)
{
	os_printk(K_ERR,"musb_cleanup\n") ;
	if (mu3d_normal_driver_on == 1){
		platform_driver_unregister(&musb_driver);
	}
	return 0 ;
}
module_exit(musb_cleanup);
#else

static int __init musb_init(void)
{
	if (usb_disabled())
		return 0;

	pr_info("%s: version " MUSB_VERSION ", ?dma?, otg (peripheral+host)\n",
		musb_driver_name);
	return platform_driver_register(&musb_driver);
}
module_init(musb_init);

static void __exit musb_cleanup(void)
{
	platform_driver_unregister(&musb_driver);
}
module_exit(musb_cleanup);
#endif
