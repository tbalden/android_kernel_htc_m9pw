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


#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>

#include "sii_hal.h"
#include "si_fw_macros.h"
#include "si_app_devcap.h"
#include "si_mhl_defs.h"  
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_8348_internal_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "si_8348_drv.h"
#include "mhl_linux_tx.h"
#include "platform.h"
#include "si_tpi_regs.h"
#include "si_8348_regs.h"

#include "si_timing_defs.h"

#ifdef CONFIG_MTK_MT6306_SUPPORT
#include<mach/dcl_sim_gpio.h>
#endif

#include <mach/irqs.h>
#include "mach/eint.h"
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include "hdmi_drv.h"
#include "mhl_supp.h"
#include "smartbook.h"
#include "hdmi_cust.h"

#define HDCP_ENABLE		

extern int debug_msgs;
extern struct mhl_dev_context *si_dev_context;
extern HDMI_CABLE_TYPE MHL_Connect_type;
extern bool HDCP_Supported_Info;

int si_mhl_tx_chip_initialize(struct drv_hw_context *hw_context);

static	int	int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status);
#ifdef HDCP_ENABLE
static	int	int_hdcp2_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status);
#endif 

static int to_be_deleted(struct drv_hw_context *hw_context, uint8_t int_status);	

static	int	int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status);
#ifdef HDCP_ENABLE
static	int	hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status);
#endif 
static	int	int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status);
static	int	g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static	int	mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int);
static	int	mhl_cbus_err_isr(struct drv_hw_context *hw_context, uint8_t cbus_err_int);

static	void	board_reset(struct drv_hw_context *hw_context, uint16_t hwResetPeriod, uint16_t hwResetDelay);
static	int	get_device_rev(struct drv_hw_context *hw_context);
static	void	enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num, uint8_t intr_mask);
static	void	switch_to_d3(struct drv_hw_context *hw_context,bool do_interrupt_clear);
static	void	disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear);
#ifdef HDCP_ENABLE
static	void	start_hdcp(struct drv_hw_context *hw_context);
#endif 
static	void	stop_video(struct drv_hw_context *hw_context);
static	void	unmute_video(struct drv_hw_context *hw_context);
static	int	set_hdmi_params(struct mhl_dev_context *dev_context);
static	int	start_video(struct drv_hw_context *hw_context, void *edid_parser_context);
static	int	get_cbus_connection_status(struct drv_hw_context *hw_context);

#define VMD_ASPECT_RATIO_4x3			0x01
#define VMD_ASPECT_RATIO_16x9			0x02


typedef struct
{
    uint8_t inputColorSpace;
    uint8_t outputColorSpace;
    uint8_t inputVideoCode;
    uint8_t inputcolorimetryAspectRatio;
    uint8_t outputcolorimetryAspectRatio;
    uint8_t input_AR;   
    uint8_t output_AR;  
} video_data_t;
video_data_t video_data;

static	int Audio_mode_fs=AUDIO_44K_2CH;
uint8_t current_audio_info_frame[14] = {0x84, 0x01, 0x0A, 0x00, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void siHdmiTx_VideoSel (int vmode);
void siHdmiTx_AudioSel (int AduioMode);

#ifdef HDCP_ENABLE
#define	HDCP_RPTR_CTS_DELAY_MS	2875
#define	HDCP_ERROR_THRESHOLD	5
static	int		hdcp_bksv_err_count = 0;
static	int		hdcp_reneg_err_count = 0;
static	int		hdcp_link_err_count = 0;
static	int		hdcp_suspend_err_count = 0;
#endif 

#define	DDC_ABORT_THRESHOLD		10
static	int		ddc_abort_count = 0;

#define	MSC_ABORT_THRESHOLD		10	
static	int		msc_abort_count = 0;

struct	intr_tbl	{
	uint8_t	mask;
	uint8_t	mask_page;
	uint8_t	mask_offset;
	uint8_t	stat_page;
	uint8_t	stat_offset;
	int	(*isr)(struct drv_hw_context *, uint8_t int_5_status);
	char	name[5];
};

#define REG_TIMING_INTR_MASK	0x66, 0x00	
#define REG_TIMING_INTR		0x66, 0x01	

struct	intr_tbl	g_intr_tbl[] = {
		 {0, REG_INTR4_MASK, REG_INTR4, int_4_isr, "DISC"}
		,{0, REG_CBUS_MDT_INT_0_MASK, REG_CBUS_MDT_INT_0, g2wb_isr, "G2WB"}
		,{0, REG_CBUS_INT_0_MASK, REG_CBUS_INT_0, mhl_cbus_isr, "MSC "}
		,{0, REG_CBUS_INT_1_MASK, REG_CBUS_INT_1, mhl_cbus_err_isr, "MERR"}
		
#ifdef HDCP_ENABLE
		,{0, REG_TPI_INTR_ST0_ENABLE, REG_TPI_INTR_ST0, hdcp_isr, "HDCP"}
#endif 
		,{0, REG_INTR3_MASK, REG_INTR3, int_3_isr, "EDID"}
#ifdef HDCP_ENABLE
		,{0, REG_TPI_INTR_ST1_ENABLE, REG_TPI_INTR_ST1, int_hdcp2_isr, "HDCP2"}
#endif 
		,{0, REG_INTR1_MASK, REG_INTR1, int_1_isr, "INTR1"}
};

typedef enum
{
	 INTR_DISC	= 0
	,INTR_G2WB	= 1
	,INTR_MSC	= 2
	,INTR_MERR	= 3
	
#ifdef HDCP_ENABLE
	,INTR_HDCP	= 4
	,INTR_EDID	= 5
	,INTR_HDCP2	= 6
	,INTR_INTR1	= 7
	,MAX_INTR	= 8
#else
	,INTR_EDID	= 4
	,INTR_INTR1	= 5
	,MAX_INTR	= 6
#endif 
}intr_nums_t;

#define SILICON_IMAGE_ADOPTER_ID		322

#define TX_HW_RESET_PERIOD			5	 
#define TX_HW_RESET_DELAY			5	 
#define TX_EDID_POLL_MAX			256

static uint8_t colorSpaceTranslateInfoFrameToHw[] = {
		BIT_TPI_INPUT_FORMAT_RGB,
		BIT_TPI_INPUT_FORMAT_YCbCr422,
		BIT_TPI_INPUT_FORMAT_YCbCr444,
		BIT_TPI_INPUT_FORMAT_INTERNAL_RGB	
};

#ifdef ENABLE_GEN2 
static	void	enable_gen2_write_burst(struct drv_hw_context *hw_context)
{
	

	if(hw_context->ready_for_mdt) {
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_TIMEOUT, 100);	
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_CONTROL, BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_ENABLE);
		enable_intr(hw_context, INTR_G2WB, BIT_MDT_RXFIFO_DATA_RDY);

		hw_context->gen2_write_burst = true;
	}
}

static void disable_gen2_write_burst(struct drv_hw_context *hw_context)
{
	
	mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_CONTROL, BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_DISABLE);
	enable_intr(hw_context, INTR_G2WB, 0);
	hw_context->gen2_write_burst = false;
}
#endif 

static void si_mhl_tx_drv_reset_ddc_fifo(struct drv_hw_context *hw_context)
{
	uint8_t	ddc_status;

	ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);

	mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
					  BIT_TPI_SEL_SW_TPI_EN_MASK,
					  BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI);
	if (BIT_DDC_STATUS_DDC_NO_ACK & ddc_status) {
		MHL_TX_DBG_ERR(hw_context, "Clearing DDC ack status\n");
		mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
						 ddc_status & ~BIT_DDC_STATUS_DDC_NO_ACK);
	}
	mhl_tx_modify_reg(hw_context, REG_DDC_CMD,
					  BIT_DDC_CMD_COMMAND_MASK,
					  BIT_DDC_CMD_COMMAND_CLEAR_FIFO);

	mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
					  BIT_TPI_SEL_SW_TPI_EN_MASK,
					  BIT_TPI_SEL_SW_TPI_EN_HW_TPI);
}

static void si_mhl_tx_drv_issue_edid_block_batch_read(struct drv_hw_context *hw_context, uint8_t block_number, uint8_t batch_number)
{
	uint8_t	ddc_status;
	uint8_t	offset = 0;

	MHL_TX_DBG_INFO(hw_context, "called.\n");

	ddc_status = mhl_tx_read_reg(hw_context, REG_DDC_STATUS);

	
	mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
					  BIT_TPI_SEL_SW_TPI_EN_MASK,
					  BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI);

	if (BIT_DDC_STATUS_DDC_NO_ACK & ddc_status) {
		MHL_TX_DBG_ERR(hw_context, "Clearing DDC ack status\n");
		mhl_tx_write_reg(hw_context, REG_DDC_STATUS,
						 ddc_status & ~BIT_DDC_STATUS_DDC_NO_ACK);
	}

	
	mhl_tx_modify_reg(hw_context, REG_DDC_CMD,
					  BIT_DDC_CMD_COMMAND_MASK,
					  BIT_DDC_CMD_COMMAND_CLEAR_FIFO);

	
	mhl_tx_write_reg(hw_context, REG_DDC_SEGM, block_number / 2);

	
	mhl_tx_write_reg(hw_context, REG_DDC_ADDR, 0xA0);

	
	offset = (0 == (block_number % 2)) ? 0x00 : 0x80;
	offset += batch_number * 16;
	mhl_tx_write_reg(hw_context, REG_DDC_OFFSET, offset);

	
	mhl_tx_write_reg(hw_context, REG_DDC_DIN_CNT1, 0x10);
	mhl_tx_write_reg(hw_context, REG_DDC_DIN_CNT2, 0x00);

	
	mhl_tx_write_reg(hw_context, REG_DDC_CMD, BIT_DDC_CMD_COMMAND_ENHANCED_READ_NO_ACK);

	
}

bool si_mhl_tx_drv_issue_edid_read_request(struct drv_hw_context *hw_context, uint8_t block_number, uint8_t batch_number)
{
	uint8_t reg_val;
			
	reg_val = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if ( BIT_CBUS_HPD & reg_val ) {
		MHL_TX_EDID_READ(hw_context,
				"\n\tRequesting EDID block:%d\n"	\
				"\tRequesting EDID block batch:%d\n"	\
				"\tcurrentEdidRequestBlock:%d\n"	\
				"\tcurrentEdidRequestBlockBatch:%d\n"	\
				"\tedidFifoBlockNumber:%d\n",
				block_number,
				batch_number,
				hw_context->current_edid_request_block,
				hw_context->current_edid_request_block_batch,
				hw_context->edid_fifo_block_number);
        
    	si_mhl_tx_drv_reset_ddc_fifo(hw_context);
		si_mhl_tx_drv_issue_edid_block_batch_read(hw_context, block_number, batch_number);

		return true;
	} else {
		MHL_TX_DBG_INFO(hw_context,
				"\n\tNo HPD for EDID block request:%d\n"	\
				"\tcurrentEdidRequestBlock:%d\n"			\
				"\tRequesting EDID block batch:%d\n"	\
				"\tcurrentEdidRequestBlockBatch:%d\n"			\
				"\tedidFifoBlockNumber:%d\n",
				block_number,
				batch_number,
				hw_context->current_edid_request_block,
				hw_context->current_edid_request_block_batch,
				hw_context->edid_fifo_block_number);
		
		return false;
	}
}

bool si_mhl_tx_drv_send_cbus_command(struct drv_hw_context *hw_context,
									 struct cbus_req *req)
{
	bool	success = true;
	uint8_t	block_write_buffer [3];	

#ifdef ENABLE_GEN2 
	
	disable_gen2_write_burst(hw_context);
#endif 
    switch (req->command ) {
	case MHL_SET_INT:
		MHL_TX_DBG_INFO(hw_context, "SET_INT reg: 0x%02x data: 0x%02x\n",
						req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
		break;

    case MHL_WRITE_STAT:
		MHL_TX_DBG_INFO(hw_context,
				"WRITE_STAT (0x%02x, 0x%02x)\n",
				req->reg,
				req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
		break;

	case MHL_READ_DEVCAP:
		MHL_TX_DBG_INFO(hw_context,
				"READ_DEVCAP (0x%02x, 0x%02x)\n",
				req->reg,
				req->reg_data);

			

		
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_READ_DEVCAP);
		break;

	case MHL_READ_EDID_BLOCK:
		hw_context->current_edid_request_block = 0;
		hw_context->current_edid_request_block_batch = 0;
		hw_context->edid_fifo_block_number = 0;
		MHL_TX_DBG_INFO(hw_context, "before si_mhl_tx_drv_issue_edid_read_request: to read block 0\n");
		success = si_mhl_tx_drv_issue_edid_read_request(hw_context,
				hw_context->current_edid_request_block,
				hw_context->current_edid_request_block_batch);
        break;

	case MHL_GET_STATE:			
	case MHL_GET_VENDOR_ID:		
	case MHL_SET_HPD:			
	case MHL_CLR_HPD:			
	case MHL_GET_SC1_ERRORCODE:	
	case MHL_GET_DDC_ERRORCODE:	
	case MHL_GET_MSC_ERRORCODE:	
	case MHL_GET_SC3_ERRORCODE:	
		MHL_TX_DBG_INFO(hw_context, "Sending MSC command %02x, %02x, %02x\n",
						req->command, req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_CMD_OR_OFFSET, req->command);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_PEER_CMD);
		break;

	case MHL_MSC_MSG:
		MHL_TX_DBG_INFO(hw_context,
						"MHL_MSC_MSG sub cmd: 0x%02x data: 0x%02x\n",
						req->msg_data[0], req->msg_data[1]);
		block_write_buffer[0] = req->command;
		block_write_buffer[1] = req->msg_data[0];
		block_write_buffer[2] = req->msg_data[1];

		mhl_tx_write_reg_block(hw_context,REG_CBUS_MSC_CMD_OR_OFFSET, 3, block_write_buffer);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_MSG);
		break;

	case MHL_WRITE_BURST:
		MHL_TX_DBG_INFO(hw_context, "MHL_WRITE_BURST offset: 0x%02x "\
						"length: 0x%02x\n",
						req->offset, req->length);

		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_CMD_OR_OFFSET,
						 req->offset + REG_CBUS_MHL_SCRPAD_BASE);

		mhl_tx_write_reg(hw_context, REG_CBUS_MSC_WRITE_BURST_DATA_LEN,
						 req->length -1);

		
		mhl_tx_write_reg_block(hw_context, REG_CBUS_WB_XMIT_DATA_0,
							   req->length, req->msg_data);
		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_COMMAND_START,
						 BIT_CBUS_MSC_WRITE_BURST);
		break;

	default:
		MHL_TX_DBG_ERR(hw_context, "Unsupported command 0x%02x detected!\n",
					   req->command);
		success = false;
		break;
    }

    return (success);
}

uint16_t si_mhl_tx_drv_get_incoming_horizontal_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_HRESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_HRESL);
	return ret_val;
}

uint16_t si_mhl_tx_drv_get_incoming_vertical_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_VRESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_VRESL);
    return ret_val;
}

int si_mhl_tx_drv_get_edid_fifo_next_block(struct drv_hw_context *hw_context, uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	uint8_t i;
	uint8_t j;

    MHL_TX_DBG_INFO(hw_context, "called.\n");

	offset = EDID_BLOCK_SIZE * hw_context->edid_fifo_block_number;

	MHL_TX_DBG_INFO(hw_context, "%x %x\n",(unsigned long)hw_context,(unsigned long)edid_buf);
	hw_context->edid_fifo_block_number++;

	memcpy(edid_buf, hw_context->current_edid_block_data + offset , EDID_BLOCK_SIZE);
	{
		MHL_TX_EDID_READ(hw_context, "current edid data: %d.\n", hw_context->edid_fifo_block_number - 1);
#if 0		
		for ( i = 0; i < 8; i ++)
		{
			printk("ROW %d:\t\t", i);
			for (j=0; j<16; j++)
			{
				printk("%02X:%02X\t", j, edid_buf[i * 16 + j]);		
				
			}
			printk("\n");
		}
		printk("\n");
	
	DUMP_EDID_BLOCK(0,edid_buf, EDID_BLOCK_SIZE);
#endif
    }
	
	ret_val = mhl_tx_read_reg(hw_context,REG_CBUS_STATUS);
	if (ret_val < 0)
	{
		MHL_TX_DBG_ERR(hw_context, "%d", ret_val);
		return ne_NO_HPD;
	}
	else if (BIT_CBUS_HPD & ret_val)
	{
		MHL_TX_DBG_INFO(hw_context, "Done reading EDID from FIFO using Manual DDC read, ret_val:0x%02x\n",ret_val);
		return 0;
	}
	else
	{
		MHL_TX_DBG_INFO(hw_context, "No HPD ret_val:0x%02x\n",ret_val);
		return ne_NO_HPD;
	}
}

int si_mhl_tx_drv_get_scratch_pad(struct drv_hw_context *hw_context,
		uint8_t start_reg, uint8_t *data,
		uint8_t length)
{
	if ((start_reg + length) > (int)MHL_SCRATCHPAD_SIZE)
		return -1;

	memcpy(data, &hw_context->write_burst_data[start_reg], length);

	return 0;
}

bool packed_pixel_available(struct mhl_dev_context *dev_context)
{
	if ((MHL_DEV_VID_LINK_SUPP_PPIXEL & DEVCAP_VAL_VID_LINK_MODE) &&
		(dev_context->dev_cap_cache.mdc.vid_link_mode &
		 MHL_DEV_VID_LINK_SUPP_PPIXEL)) {

		return true;
	}
	return false;
}

#define SIZE_AVI_INFOFRAME				14
static uint8_t calculate_avi_info_frame_checksum(hw_avi_payload_t *payload)
{
	uint8_t checksum;

	checksum = 0x82 + 0x02 + 0x0D;	
	return calculate_generic_checksum(payload->ifData, checksum, SIZE_AVI_INFOFRAME);
}
#if 0
#define SIZE_AUDIO_IF					14
static uint8_t calculate_audio_if_checksum(uint8_t *audio_if)
{
	uint8_t checksum = 0;

	return calculate_generic_checksum(audio_if, checksum, SIZE_AUDIO_IF);
}

#define SIZE_AVIF					9
static uint8_t calculate_avif_checksum(uint8_t *avif)
{
	uint8_t checksum = 0;

	return calculate_generic_checksum(avif, checksum, SIZE_AVIF);
}
#define SIZE_VSIF					8
static uint8_t calculate_vsif_checksum(uint8_t *vsif)
{
	uint8_t checksum = 0;

	return calculate_generic_checksum(vsif, checksum, SIZE_VSIF);
}

static int is_valid_avi_info_frame(struct mhl_dev_context *dev_context, avi_info_frame_t *avif)
{
	uint8_t	checksum;

	checksum = calculate_generic_checksum((uint8_t *)avif, 0, sizeof(*avif));
	if (0 != checksum) {
		MHL_TX_DBG_ERR(dev_context, "AVI info frame checksum is: 0x%02x should be 0\n", checksum);
		return 0;

	} else if (0x82 != avif->header.type_code) {
		MHL_TX_DBG_ERR(dev_context, "Invalid AVI type code: 0x%02x\n", avif->header.type_code);
		return 0;

	} else if (0x02 != avif->header.version_number) {
		MHL_TX_DBG_ERR(dev_context, "Invalid AVI version: 0x%02x\n", avif->header.version_number);
		return 0;

	} else if (0x0D != avif->header.length) {
		return 0;

	} else {
		return 1;
	}
}

static int is_valid_vsif(struct mhl_dev_context *dev_context, vendor_specific_info_frame_t *vsif)
{
	uint8_t	checksum;

	checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length );
	if (0 != checksum) {
		MHL_TX_DBG_WARN(dev_context, "VSIF info frame checksum is: 0x%02x should be 0\n", checksum);
		checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length
			+ sizeof(vsif->payLoad.checksum));
		if (0 != checksum){
			MHL_TX_DBG_ERR(dev_context, "VSIF info frame checksum "
					"(adjusted for checksum itself) is: 0x%02x "
						   "should be 0\n", checksum);
			return 0;
		}
	} 
	if (0x81 != vsif->header.type_code) {
		MHL_TX_DBG_ERR(dev_context, "Invalid VSIF type code: 0x%02x\n",
					   vsif->header.type_code);
		return 0;

	} else if (0x01 != vsif->header.version_number) {
		MHL_TX_DBG_ERR(dev_context, "Invalid VSIF version: 0x%02x\n",
					   vsif->header.version_number);
		return 0;

	} else {
		return 1;
	}
}
#endif
static	void	print_vic_modes(struct drv_hw_context *hw_context,uint8_t vic)
{
	int	i;
	struct	vic_name {
		uint8_t		vic;
		char		name[10];
	} vic_name_table[] = {
				  {2, "480P"}
				 ,{4, "720P60"}
				 ,{5, "1080i60"}
				 ,{6, "480i"}
				 ,{16,"1080P60"}
				 ,{17,"576P50"}
				 ,{19,"720P50"}
				 ,{20,"1080i50"}
				 ,{21,"576i50"}
				 ,{31,"1080P50"}
				 ,{32,"1080P24"}
				 ,{33,"1080P25"}
				 ,{34,"1080P30"}
				 ,{0,""} 
	};
#define	NUM_VIC_NAMES	(sizeof(vic_name_table)/sizeof(vic_name_table[0]) )
	
	for(i = 0; i < (NUM_VIC_NAMES - 1); i++) {
		if(vic == vic_name_table[i].vic) {
			break;
		}
	}
	MHL_TX_DBG_ERR(hw_context, "VIC = %d (%s)\n", vic, vic_name_table[i].name);
}

static void set_mhl_zone_settings(struct mhl_dev_context *dev_context , uint32_t pixel_clock_frequency)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	MHL_TX_DBG_INFO(hw_context,"pixel clock:%d %04x rev %02x\n",
					pixel_clock_frequency,
					hw_context->chip_device_id,
					hw_context->chip_rev_id);

#if 0	
	if (hw_context->chip_rev_id > 0) {
		if (pixel_clock_frequency < 30000000)
			mhl_tx_write_reg(hw_context, REG_TXMZ_CTRL2, 0x01);
		else
			mhl_tx_write_reg(hw_context, REG_TXMZ_CTRL2, 0x00);
	}
#endif

	si_mhl_tx_set_pp_link(dev_context, dev_context->link_mode);
}

static int set_hdmi_params(struct mhl_dev_context *dev_context)
{
	uint32_t			pixel_clock_frequency;
	uint32_t			threeDPixelClockRatio;
	uint8_t				packedPixelNeeded=0;
	AviColorSpace_e			input_clr_spc =acsRGB;
	uint8_t				output_clr_spc =acsRGB;
	avi_info_frame_data_byte_4_t	input_video_code;
	struct drv_hw_context	*hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	enum {
		 use_avi_vic
		,use_hardware_totals
	}timing_info_basis=use_avi_vic;

	MHL_TX_DBG_ERR(hw_context,"packed_pixel_available :%d \n",packed_pixel_available(dev_context));
	uint16_t h_total = si_mhl_tx_drv_get_incoming_horizontal_total(dev_context);
	uint16_t v_total = si_mhl_tx_drv_get_incoming_vertical_total(dev_context);
	MHL_TX_DBG_INFO(hw_context, "h_total :%u,v_total:%u\n", h_total,v_total );

	
	input_video_code.VIC = video_data.inputVideoCode;
	

	threeDPixelClockRatio = 1;
	si_mhl_tx_drv_get_incoming_horizontal_total(hw_context);
#if 0
	if (hw_context->valid_vsif && hw_context->valid_3d)
	{
		MHL_TX_DBG_WARN(, "valid HDMI VSIF\n");
		print_vic_modes(hw_context, (uint8_t) input_video_code.VIC);

		if (0 == input_video_code.VIC) {
			MHL_TX_DBG_ERR(,"AVI VIC is zero!!!\n");
			return false;
		}
		
	}
	#endif
	#if 0
	else
	{ 	
		if (0 == input_video_code.VIC) {
			timing_info_basis = use_hardware_totals;
			MHL_TX_DBG_WARN(,"no VSIF and AVI VIC is zero!!! trying HTOTAL/VTOTAL\n");
		}else{
			print_vic_modes(hw_context, (uint8_t) input_video_code.VIC);
		}
	}
	#endif
	
	

	
	switch(timing_info_basis){
		case use_avi_vic:
			pixel_clock_frequency = si_edid_find_pixel_clock_from_AVI_VIC(
					dev_context->edid_parser_context,
					input_video_code.VIC);
			break;
		case use_hardware_totals:
			pixel_clock_frequency = si_mhl_tx_find_timings_from_totals(
					dev_context->edid_parser_context);
			if (0 == pixel_clock_frequency){
				MHL_TX_DBG_ERR(,"VIC was zero and totals not supported\n");
				
			}
			break;
	}

	
	input_clr_spc = video_data.outputColorSpace;
	
	
	
	MHL_TX_DBG_INFO(dev_context, "input_clr_spc = %02X input_video_code.VIC:%02X\n",
			input_clr_spc, input_video_code.VIC);

	pixel_clock_frequency *= threeDPixelClockRatio;
	MHL_TX_DBG_INFO(hw_context, "pixel clock:%u\n", pixel_clock_frequency);

	if (qualify_pixel_clock_for_mhl(dev_context->edid_parser_context,
		pixel_clock_frequency, 24)) {
		MHL_TX_DBG_INFO(hw_context, "OK for 24 bit pixels\n");
	} else {
		
		
		{
			MHL_TX_DBG_INFO(hw_context, "Sink supports YCbCr422\n");

			if (qualify_pixel_clock_for_mhl(
				dev_context->edid_parser_context, pixel_clock_frequency, 16)) {
				
				packedPixelNeeded = 1;
			} else {
				MHL_TX_DBG_ERR(hw_context,"unsupported video mode."
						"pixel clock too high %s\n"
						,si_peer_supports_packed_pixel(dev_context)
							? "" :"(peer does not support packed pixel)."
						);
				return	false;
			}
		} 
	}

	output_clr_spc = input_clr_spc;

	if (packedPixelNeeded){
		if (packed_pixel_available(dev_context)) {
			MHL_TX_DBG_INFO(hw_context, "setting packed pixel mode\n");

			dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_PACKED_PIXEL;

			
			output_clr_spc = BIT_EDID_FIELD_FORMAT_YCbCr422;

			mhl_tx_write_reg(hw_context
					, REG_VID_MODE
					, REG_VID_MODE_DEFVAL | BIT_VID_MODE_m1080p_ENABLE);

			mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL4,
					BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK,
					BIT_MHLTX_CTL4_MHL_CLK_RATIO_2X);

			mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL6,
					BIT_MHLTX_CTL6_CLK_MASK,
					BIT_MHLTX_CTL6_CLK_PP);

		} else {
			MHL_TX_DBG_ERR(hw_context,
				"unsupported video mode. Packed Pixel not available on sink."
				"Sink's link mode = 0x%02x\n",
				dev_context->dev_cap_cache.mdc.vid_link_mode);
			return false;
		}
	} else {
		MHL_TX_DBG_INFO(hw_context, "normal Mode ,Packed Pixel mode disabled \n");

		dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_NORMAL;

		mhl_tx_write_reg(hw_context
				, REG_VID_MODE
				, REG_VID_MODE_DEFVAL | BIT_VID_MODE_m1080p_DISABLE);

		mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL4,
				BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK,
				BIT_MHLTX_CTL4_MHL_CLK_RATIO_3X);

		mhl_tx_modify_reg(hw_context, REG_MHLTX_CTL6,
				BIT_MHLTX_CTL6_CLK_MASK,
				BIT_MHLTX_CTL6_CLK_NPP);
	}

	
	
	mhl_tx_write_reg(hw_context , REG_TPI_INPUT , colorSpaceTranslateInfoFrameToHw[input_clr_spc] | BIT_TPI_OUTPUT_QUAN_RANGE_LIMITED);
	
	
	mhl_tx_write_reg(hw_context , REG_TPI_OUTPUT , colorSpaceTranslateInfoFrameToHw[output_clr_spc] | BIT_TPI_OUTPUT_QUAN_RANGE_LIMITED);

	set_mhl_zone_settings(dev_context,pixel_clock_frequency);

	{
	
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[0] = output_clr_spc << 5|0x02; 
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[1] = video_data.outputcolorimetryAspectRatio; 

	{
		hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[2] = 0x04;  
		TX_DEBUG_PRINT(("VIDEO_CAPABILITY_D_BLOCK_found= false. defult range\n"));
	}
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[3] = video_data.inputVideoCode; 
	TX_DEBUG_PRINT(("video_data.inputVideoCode:0x%02x\n",(int)video_data.inputVideoCode));

	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[4] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[5] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[6] = 0x00; 
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[7] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[8] = 0x00; 
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[9] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[10] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[11] = 0x00;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.infoFrameData[12] = 0x00;

	}
	
	hw_context->outgoingAviPayLoad.namedIfData.checksum = 0;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.pb1.colorSpace
		= output_clr_spc;
	hw_context->outgoingAviPayLoad.namedIfData.checksum =
		calculate_avi_info_frame_checksum(&hw_context->outgoingAviPayLoad);
	TX_DEBUG_PRINT(("hw_context->outgoingAviPayLoad.namedIfData.checksum:0x%02x\n",(int)hw_context->outgoingAviPayLoad.namedIfData.checksum));
	DumpIncomingInfoFrame(&(hw_context->outgoingAviPayLoad),sizeof(hw_context->outgoingAviPayLoad));
	return true;
}

#define dump_edid_fifo(hw_context, block_number) 

int si_mhl_tx_drv_set_upstream_edid(struct drv_hw_context *hw_context, uint8_t *edid, uint16_t length)
{
	uint8_t					reg_val;

	reg_val = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
	if (!(BIT_CBUS_HPD & reg_val)){
		return -1;
	}

#ifdef NEVER 
	if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_HDMI
				);
	} else {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_DVI
				);
	}
#endif 

	

	

	

	
	enable_intr(hw_context, INTR_EDID, 0);	

	
	hw_context->ready_for_mdt = true;

#ifdef ENABLE_GEN2 
	enable_gen2_write_burst(hw_context);
#endif 

	

	

	
	

	return 0;
}

static void tmds_configure(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "called\n");

	mhl_tx_write_reg(hw_context, REG_SRST, 0x9F);
	mhl_tx_write_reg(hw_context, REG_SRST, BIT_MHL_FIFO_AUTO_RST);

	mhl_tx_write_reg(hw_context, REG_DPD, 0x1F);		

	mhl_tx_write_reg(hw_context, REG_TMDS_CCTRL, BIT_TMDS_CCTRL_TMDS_OE | BIT_TMDS_CCTRL_SEL_BGR);
	mhl_tx_write_reg(hw_context, REG_USB_CHARGE_PUMP_MHL, BIT_USE_CHARGE_PUMP_MHL_DEFAULT);
	mhl_tx_write_reg(hw_context, REG_USB_CHARGE_PUMP, BIT_USE_CHARGE_PUMP_DEFAULT);

	mhl_tx_write_reg(hw_context, REG_DISC_CTRL3, BIT_DC3_DEFAULT);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1, BIT_MHLTX_CTL1_DISC_OVRIDE_ON);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL2, REG_MHLTX_CTL2_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL3, REG_MHLTX_CTL3_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL4, REG_MHLTX_CTL4_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL6, REG_MHLTX_CTL6_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL7, REG_MHLTX_CTL7_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL8, REG_MHLTX_CTL8_DEFVAL);
}

static void power_up(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "called\n");

	
	
	
	
	mhl_tx_write_reg(hw_context, REG_SYS_CTRL1, 0x31);

	
	mhl_tx_modify_reg(hw_context, REG_DISC_CTRL1, BIT_DISC_CTRL1_STROBE_OFF, 0);	
}

#define	MHL_LOGICAL_DEVICE_MAP		MHL_DEV_LD_GUI 
#define DEVCAP_REG(x) REG_CBUS_DEVICE_CAP_0 | DEVCAP_OFFSET_##x

uint8_t	dev_cap_values[] = {
			DEVCAP_VAL_DEV_STATE
		,	DEVCAP_VAL_MHL_VERSION
		,	DEVCAP_VAL_DEV_CAT
		,	DEVCAP_VAL_ADOPTER_ID_H
		,	DEVCAP_VAL_ADOPTER_ID_L
		,	DEVCAP_VAL_VID_LINK_MODE
		,	DEVCAP_VAL_AUD_LINK_MODE
		,	DEVCAP_VAL_VIDEO_TYPE
		,	DEVCAP_VAL_LOG_DEV_MAP
		,	DEVCAP_VAL_BANDWIDTH
		,	DEVCAP_VAL_FEATURE_FLAG
		,	0
		,	0
		,	DEVCAP_VAL_SCRATCHPAD_SIZE
		,	DEVCAP_VAL_INT_STAT_SIZE
		,	DEVCAP_VAL_RESERVED
};

static int init_regs(struct drv_hw_context *hw_context)
{
	int ret_val = 0;

	MHL_TX_DBG_INFO(hw_context, "called\n");

	

	
	hw_context->video_ready = true;

	
	hw_context->video_path = 0;

	hw_context->ready_for_mdt = false;
	hw_context->audio_poll_enabled = false;

	


	{
		
		ret_val = mhl_tx_read_reg(hw_context, REG_TPI_SEL);
		ret_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
		ret_val |= BIT_TPI_SEL_SW_TPI_EN_HW_TPI;
		mhl_tx_write_reg(hw_context, REG_TPI_SEL, ret_val);

#ifdef HDCP_ENABLE
		mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);
#endif 

		mhl_tx_write_reg(hw_context, REG_TPI_HW_OPT3, 0x76);	

		
		mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_100DIFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

		
		mhl_tx_write_reg(hw_context, REG_DISC_CTRL8,0x03);
		


		

#if 0 
		
		mhl_tx_write_reg(hw_context , REG_TPI_HDCP_TIMER_1_SEC , 79);	
#endif 
	}
	
	{
		dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_L] =
			(uint8_t)hw_context->chip_device_id;
		dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_H] =
			(uint8_t)(hw_context->chip_device_id >> 8);

		
		mhl_tx_write_reg_block(hw_context, DEVCAP_REG(DEV_STATE),
				ARRAY_SIZE(dev_cap_values), dev_cap_values);

		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XMIT_TIMEOUT, 100);

		
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XMIT_CONTROL, 0x03);

		
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_XFIFO_STAT, 0x00);

		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_RCV_TIMEOUT, 100);

		mhl_tx_write_reg(hw_context, REG_CBUS_LINK_CHECK_HIGH_LIMIT, REG_CBUS_LINK_CHECK_HIGH_LIMIT_DEFVAL);
		mhl_tx_write_reg(hw_context, REG_CBUS_LINK_XMIT_BIT_TIME, REG_CBUS_LINK_XMIT_BIT_TIME_DEFVAL);
	}
#ifdef ENABLE_GEN2 
	disable_gen2_write_burst(hw_context);
#endif 
	hw_context->ready_for_mdt = false;

	
	
	mhl_tx_write_reg(hw_context, REG_DPD, 0x17);		


	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x0B, 0x00);	


	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x1F, 0x80);	

	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x20, 0x80);	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x21, 0x00);	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x22, 0x00);	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x23, 0x00);	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x25, 0x0B);	

	mhl_tx_write_reg(hw_context, TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_D0); 
	
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x1A, 0x00);	
	
#ifdef HDCP_ENABLE
	mhl_tx_write_reg(hw_context, TX_PAGE_TPI, 0x2A, 0x01);	
#endif 

	return ret_val;
}

void si_mhl_tx_drv_set_hw_tpi_mode(struct drv_hw_context *hw_context, bool hw_tpi_mode)
{
	if ( hw_tpi_mode )
	{
		
		mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
				BIT_TPI_SEL_SW_TPI_EN_MASK,
				BIT_TPI_SEL_SW_TPI_EN_HW_TPI);
	}
	else
	{
		
		mhl_tx_modify_reg(hw_context, REG_TPI_SEL,
				BIT_TPI_SEL_SW_TPI_EN_MASK,
				BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI);
	}
}

void si_mhl_tx_drv_disable_video_path(struct drv_hw_context *hw_context)
{
	
	if(hw_context->video_ready && (0 == (AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG)))) {

		
		stop_video(hw_context);
		hw_context->video_path = 0;
	}
}

void si_mhl_tx_drv_enable_video_path(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK);
	uint8_t	reg;

	MHL_TX_DBG_INFO(dev_context, "called\n");

	
	{
		
		hw_context->video_path = 1;

		
		
			
		
	}
}

void si_mhl_tx_drv_content_off(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(dev_context, "RAP CONTENT_OFF video %sready\n",hw_context->video_ready?"":"NOT ");
	
	if(hw_context->video_ready && (0 == (AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG)))) {

		MHL_TX_DBG_INFO(dev_context, "RAP CONTENT_OFF\n");
		
		stop_video(hw_context);
	}
}

void si_mhl_tx_drv_content_on(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK);
	uint8_t	reg;

	
	if(hw_context->video_ready) {

		reg  = mhl_tx_read_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG);

		if(mask == (mask & reg)) {
			start_video(hw_context,hw_context->intr_info->edid_parser_context);
		}
	}
}


void dump_audio_register(struct drv_hw_context *hw_context, int audio)
{
	int value = 0;
	int offset = 0;

	{
		value = mhl_tx_read_reg(hw_context, REG_TPI_CONFIG3);
		MHL_TX_DBG_ERR(hw_context, "REG_TPI_CONFIG3: 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, TX_PAGE_TPI, 0x0020);
		MHL_TX_DBG_ERR(hw_context, "TX_PAGE_TPI:0x0020: 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, TX_PAGE_TPI, 0x001F);
		MHL_TX_DBG_ERR(hw_context, "TX_PAGE_TPI:0x001F : 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, REG_TPI_CONFIG1);
		MHL_TX_DBG_ERR(hw_context, "REG_TPI_CONFIG1 : 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, REG_TPI_CONFIG2);
		MHL_TX_DBG_ERR(hw_context, "REG_TPI_CONFIG2 : 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, TX_PAGE_L1, 0x24);
		MHL_TX_DBG_ERR(hw_context, "TX_PAGE_L1:0x24 : 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, REG_TPI_CONFIG4);
		MHL_TX_DBG_ERR(hw_context, "REG_TPI_CONFIG4 : 0x%x\n", value);

		value = mhl_tx_read_reg(hw_context, TX_PAGE_TPI, 0x0028);
		MHL_TX_DBG_ERR(hw_context, "TX_PAGE_TPI:0x0028 : 0x%x\n", value);

		printk("/********************************Start*********************************/\n");
		printk("Dump TX_PAGE_TPI 0x001F -> 0x0027 Registers\n");
		offset = 0x001F;
		for(offset=0x001F; offset<=0x0028; offset++)
		{
			value = mhl_tx_read_reg(hw_context, TX_PAGE_TPI, offset);
			MHL_TX_DBG_ERR(hw_context, "TX_PAGE_TPI:0x%x : 0x%x\n", offset, value);
		}
		printk("/********************************End***********************************/\n");

		printk("/********************************Start*********************************/\n");
		printk("Dump TX_PAGE_L1 0x0000 -> 0x0034 Registers\n");
		offset = 0x0000;
		for(offset=0x0000; offset<=0x0034; offset++)
		{
			value = mhl_tx_read_reg(hw_context, TX_PAGE_L1, offset);
			MHL_TX_DBG_ERR(hw_context, "TX_PAGE_L1:0x%x : 0x%x\n", offset, value);
		}
		printk("/********************************End***********************************/\n");
	}
}

extern uint8_t CA;
extern uint8_t  Samplebit;

extern uint8_t  MAX_channel;

extern uint8_t  Cap_MAX_channel;
extern uint8_t  Cap_Samplebit;
extern uint16_t Cap_SampleRate;

void set_platform_bitwidth(int bitWidth)
{
	Samplebit = bitWidth;
	return ;
}
void check_TV_capability(struct drv_hw_context *hw_context, int audio, int bitWidth)
{
	MHL_TX_DBG_INFO(hw_context, "MAX_channel: %d, Cap_Samplebit: %d, Cap_SampleRate: %d\n", MAX_channel, Cap_Samplebit, Cap_SampleRate);

	if(bitWidth == 24)
	{
		if(Cap_Samplebit != HDMI_BITWIDTH_24)
		{
			MHL_TX_DBG_INFO(hw_context, "Samplebit is not supported\n");
			return ;
		}
	}

	if(audio == AUDIO_32K_8CH || audio == AUDIO_44K_8CH || audio == AUDIO_48K_8CH || audio == AUDIO_96K_8CH || audio == AUDIO_192K_8CH)
	{
		if(Cap_MAX_channel != HDMI_CHANNEL_8)
		{
			MHL_TX_DBG_INFO(hw_context, "MAX_channel is not supported\n");
			return ;
		}
	}

	if(audio == AUDIO_192K_2CH || audio == AUDIO_192K_8CH)
	{
		if(Cap_SampleRate < HDMI_SAMPLERATE_192)
		{
			MHL_TX_DBG_INFO(hw_context, "Cap_SampleRate is not supported\n");
			return ;
		}
	}

	return ;
}
static void configure_audio(struct drv_hw_context *hw_context, int audio)
{
	int BIT_TPI_CONFIG4_AUDIO_WIDTH = BIT_TPI_CONFIG4_AUDIO_WIDTH_16_BIT;
	int BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH= BIT_TPI_CONFIG3_AUDIO_TDM_32B_CH;
	MHL_TX_DBG_INFO(hw_context, "Audio Update to default: Audio_mode_fs=0x%x,CA=0x%x, Samplebit=%d\n",audio, CA, Samplebit);

	
	current_audio_info_frame[5]=0x00;

	check_TV_capability(hw_context, audio, Samplebit);
	
	if(Samplebit==16){
		BIT_TPI_CONFIG4_AUDIO_WIDTH = BIT_TPI_CONFIG4_AUDIO_WIDTH_16_BIT;
		BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH= BIT_TPI_CONFIG3_AUDIO_TDM_16B_CH;
		}
	else if(Samplebit==20){
		BIT_TPI_CONFIG4_AUDIO_WIDTH = BIT_TPI_CONFIG4_AUDIO_WIDTH_20_BIT;
		BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH= BIT_TPI_CONFIG3_AUDIO_TDM_32B_CH;
		}
	else if(Samplebit==24){
		BIT_TPI_CONFIG4_AUDIO_WIDTH = BIT_TPI_CONFIG4_AUDIO_WIDTH_24_BIT;
		BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH= BIT_TPI_CONFIG3_AUDIO_TDM_32B_CH;
		}
	if ( AUDIO_32K_2CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 32K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, 0x03);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4, 0x09|BIT_TPI_CONFIG4_AUDIO_WIDTH);	
	}
	else if ( AUDIO_32K_8CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 32K / 8CH / TDM\n");

		current_audio_info_frame[4] = 0x07; 
		current_audio_info_frame[7] = CA; 

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_HD_AUDIO
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_8CH_MAX
				| BIT_TPI_CONFIG3_AUDIO_TDM_MSB_1CLK_DEALY
				| BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_32K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_32K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_8CH);	

		mhl_tx_write_reg(hw_context, TX_PAGE_L1 , 0x0024, 0x02);

	}
	else if ( AUDIO_44K_2CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 44K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		
        mhl_tx_write_reg(hw_context, TX_PAGE_TPI,0x0020, 0x90);    
        mhl_tx_write_reg(hw_context, TX_PAGE_TPI,0x001F, 0x91);    

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, 0x00);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG2, 0x02);	
		mhl_tx_write_reg(hw_context, TX_PAGE_L1,0x24, 0x02);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4, 0x11|BIT_TPI_CONFIG4_AUDIO_WIDTH);	
		mhl_tx_write_reg(hw_context, TX_PAGE_TPI,0x0028, 0x80);	
	}
	else if ( AUDIO_44K_8CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 44.1K / 8CH / TDM \n");

		current_audio_info_frame[4] = 0x07; 
		current_audio_info_frame[7] = CA; 

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_HD_AUDIO
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_8CH_MAX
				| BIT_TPI_CONFIG3_AUDIO_TDM_MSB_1CLK_DEALY
				| BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_44K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_44K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_8CH); 

		mhl_tx_write_reg(hw_context, TX_PAGE_L1 , 0x0024, 0x02);

	}
	else if ( AUDIO_48K_2CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 48K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_48K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_48K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_2CH);	
	}
	else if ( AUDIO_48K_8CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 48K / 8CH / TDM\n");

		current_audio_info_frame[4] = 0x07; 
		current_audio_info_frame[7] = CA; 

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_HD_AUDIO
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_8CH_MAX
				| BIT_TPI_CONFIG3_AUDIO_TDM_MSB_1CLK_DEALY
				| BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_48K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_48K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_8CH);	

		mhl_tx_write_reg(hw_context, TX_PAGE_L1 , 0x0024, 0x02);

	}
	else if ( AUDIO_96K_2CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 48K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_96K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_96K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_2CH);	
	}
	else if ( AUDIO_96K_8CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 48K / 8CH / TDM\n");

		current_audio_info_frame[4] = 0x07; 
		current_audio_info_frame[7] = CA; 

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_HD_AUDIO
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_8CH_MAX
				| BIT_TPI_CONFIG3_AUDIO_TDM_MSB_1CLK_DEALY
				| BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_96K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_96K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_8CH);	

		mhl_tx_write_reg(hw_context, TX_PAGE_L1 , 0x0024, 0x02);

	}
	else if ( AUDIO_192K_2CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 192K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_192K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_192K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_2CH);	
	}
	else if ( AUDIO_192K_8CH == audio )
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update: 192K / 8CH / TDM\n");

		current_audio_info_frame[4] = 0x07; 
		current_audio_info_frame[7] = CA; 

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_HD_AUDIO
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_8CH_MAX
				| BIT_TPI_CONFIG3_AUDIO_TDM_MSB_1CLK_DEALY
				| BIT_TPI_CONFIG3_AUDIO_TDM_BIT_CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_192K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_192K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_8CH);	

		mhl_tx_write_reg(hw_context, TX_PAGE_L1 , 0x0024, 0x02);

	}
	else	
	{
		MHL_TX_DBG_INFO(hw_context, "Audio Update to default: 48K / 2CH\n");

		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_ENABLE | BIT_TPI_AUDIO_FIFO_MAP_0 | BIT_TPI_AUDIO_SD_SEL_0);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_1 | BIT_TPI_AUDIO_SD_SEL_1);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_2 | BIT_TPI_AUDIO_SD_SEL_2);	
		mhl_tx_write_reg(hw_context, REG_TPI_AUDIO_MAPPING_CONFIG, BIT_TPI_AUDIO_SD_DISABLE | BIT_TPI_AUDIO_FIFO_MAP_3 | BIT_TPI_AUDIO_SD_SEL_3);	

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG3,
				BIT_TPI_CONFIG_3_AUDIO_INTERFACE_I2S
				| BIT_TPI_CONFIG3_AUDIO_PACKET_HEADER_LAYOUT_2CH
				| BIT_TPI_CONFIG3_MUTE_MUTED);		

		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG1, BIT_TPI_CONFIG1_AUDIO_FREQUENCY_48K);	
		mhl_tx_write_reg(hw_context, REG_TPI_CONFIG4,
				  BIT_TPI_CONFIG4_AUDIO_WIDTH
				| BIT_TPI_CONFIG4_AUDIO_FREQUENCY_48K
				| BIT_TPI_CONFIG4_AUDIO_CHANNEL_2CH);	
	}

	current_audio_info_frame[3]= 0;
	current_audio_info_frame[3]=calculate_generic_checksum(current_audio_info_frame, 0, 14);
}

void configure_and_send_audio_info(struct mhl_dev_context *dev_context, int audio_format)
{
    struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	MHL_TX_DBG_INFO(hw_context, "Send Audio IF out again....\n");

	configure_audio(hw_context, audio_format);
	DumpIncomingInfoFrame(&(current_audio_info_frame), 14);

	
	mhl_tx_modify_reg(hw_context, REG_TPI_CONFIG3, BIT_TPI_CONFIG3_MUTE_MASK, BIT_TPI_CONFIG3_MUTE_NORMAL);

	mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL, 0xC2);  
	mhl_tx_write_reg_block(hw_context, REG_TPI_INFO_BYTE00, AUDIO_IF_SIZE, current_audio_info_frame);

	return ;
}

#ifdef	EXAMPLE_ONLY	
static	void	mute_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "AV muted\n");
	mhl_tx_modify_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG, AV_MUTE_MASK, AV_MUTE_MUTED);
}
#endif	

static void unmute_video(struct drv_hw_context *hw_context)
{
			
	MHL_TX_DBG_INFO(hw_context, "AV unmuted.\n");

	
	{

		MHL_TX_DBG_INFO(hw_context, "It's an HDMI sink.\n");
		
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_MODE_HDMI);
		
		{
			MHL_TX_DBG_INFO(hw_context, "Send AVIF out...\n");
			DumpIncomingInfoFrame(&(hw_context->outgoingAviPayLoad),sizeof(hw_context->outgoingAviPayLoad));
			mhl_tx_write_reg_block(hw_context, TX_PAGE_TPI , 0x000C,
					sizeof(hw_context->outgoingAviPayLoad.ifData),
					(uint8_t*)&hw_context->outgoingAviPayLoad.namedIfData);
			mhl_tx_write_reg(hw_context, REG_TPI_AVI_BYTE13, 0x00);
		}

		
		

		if ( 1 == hw_context->valid_vsif && 1 == hw_context->valid_3d )
		{
			MHL_TX_DBG_INFO(hw_context, "Send VSIF out...\n");
			mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL, BIT_TPI_INFO_EN | BIT_TPI_INFO_RPT | BIT_TPI_INFO_SEL_3D_VSIF);	
			DumpIncomingInfoFrame(&(hw_context->current_vs_info_frame),sizeof(hw_context->current_vs_info_frame));
			mhl_tx_write_reg_block(hw_context, REG_TPI_INFO_BYTE00, 8, (u8 *)(&(hw_context->current_vs_info_frame)) );	
			mhl_tx_write_reg(hw_context, REG_TPI_INFO_BYTE30, 0x00);
		}

		
		{
			MHL_TX_DBG_INFO(hw_context, "Send Audio IF out...\n");

			configure_audio(hw_context, Audio_mode_fs);
			DumpIncomingInfoFrame(&(current_audio_info_frame),14);
			
			mhl_tx_modify_reg(hw_context, REG_TPI_CONFIG3, BIT_TPI_CONFIG3_MUTE_MASK, BIT_TPI_CONFIG3_MUTE_NORMAL);

			mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL, 0xC2);	
			mhl_tx_write_reg_block(hw_context, REG_TPI_INFO_BYTE00, AUDIO_IF_SIZE, current_audio_info_frame);
		}

		
		mhl_tx_modify_reg(hw_context, REG_SRST, BIT_AUDIO_FIFO_RST_MASK, BIT_AUDIO_FIFO_RST_SET);
		mhl_tx_modify_reg(hw_context, REG_SRST, BIT_AUDIO_FIFO_RST_MASK, BIT_AUDIO_FIFO_RST_CLR);
		
		
	}
	#if 0
	else
	{
		MHL_TX_DBG_INFO(hw_context, "It's a DVI sink.\n");
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_MODE_DVI);
	}
    #endif
	
	hw_context->video_ready = 1;
}

static void stop_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "stop video.\n");

#ifdef HDCP_ENABLE
	
	enable_intr(hw_context, INTR_HDCP, (0x00));
	enable_intr(hw_context, INTR_HDCP2, (0x00));
#endif 

	if(si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_HDMI
				);
	} else {
		mhl_tx_write_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG
				, TMDS_OUTPUT_CONTROL_POWER_DOWN
				| AV_MUTE_MUTED
				| TMDS_OUTPUT_MODE_DVI
				);
	}

	
	
	mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL, BIT_TPI_INFO_SEL_3D_VSIF);	
	mhl_tx_write_reg(hw_context, REG_TPI_INFO_FSEL, BIT_TPI_INFO_SEL_Audio);	
#ifdef HDCP_ENABLE
	
	mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);

	
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP2].stat_page, g_intr_tbl[INTR_HDCP2].stat_offset, 0xff);
#endif 
}

#ifdef HDCP_ENABLE
static void	start_hdcp(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO(hw_context, "start_hdcp");

	struct mhl_dev_context *dev_context;
	dev_context = container_of((void *)hw_context, struct mhl_dev_context, drv_context);
	if (dev_context->hdcp_status == 0) {
		pr_info("Disable HDCP by command\n");
		unmute_video(hw_context);
		return;
	}

	
	stop_video(hw_context);

#if 0 
	
	if( (hdcp_bksv_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_link_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_reneg_err_count > HDCP_ERROR_THRESHOLD) ||
			(hdcp_suspend_err_count > HDCP_ERROR_THRESHOLD))
	{
		MHL_TX_DBG_ERR(hw_context, "Too many HDCP Errors: bksv_err= %d, reneg_err= %d, link_err= %d, suspend_err= %d\n",
				hdcp_bksv_err_count, hdcp_reneg_err_count, hdcp_link_err_count, hdcp_suspend_err_count);
		hdcp_bksv_err_count = hdcp_reneg_err_count = hdcp_link_err_count = hdcp_suspend_err_count = 0;

		

		return;
	}
#endif 

	
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);
	mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP2].stat_page, g_intr_tbl[INTR_HDCP2].stat_offset, 0xff);

	if(get_cbus_connection_status(hw_context)) {
		
		enable_intr(hw_context, INTR_HDCP,
				 (  BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT
			  	  | BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT
				 ));

		
		enable_intr(hw_context, INTR_HDCP2,
				 (  BIT_TPI_INTR_ST1_BKSV_DONE
			  	  | BIT_TPI_INTR_ST1_BKSV_ERR
				 ));

		msleep(250);
		mhl_tx_modify_reg(hw_context, TPI_SYSTEM_CONTROL_DATA_REG,
					TMDS_OUTPUT_CONTROL_MASK,
					TMDS_OUTPUT_CONTROL_ACTIVE);

	}
}
#endif 

static	int start_video(struct drv_hw_context *hw_context, void *edid_parser_context)
{
	struct mhl_dev_context	*dev_context;

	MHL_TX_DBG_INFO(hw_context,"called.\n");

	dev_context = get_mhl_device_context(hw_context);

	stop_video(hw_context);


	
		if(false == set_hdmi_params(dev_context)) {
			
			return false;
		}

#ifdef HDCP_ENABLE
	MHL_TX_DBG_INFO(hw_context,"Start HDCP Authentication\n");
	start_hdcp(hw_context);
#else
	unmute_video(hw_context);
#endif 

	return true;
}

bool si_mhl_tx_set_path_en_I(struct mhl_dev_context *dev_context)
{
	MHL_TX_DBG_INFO(dev_context, "called si_mhl_tx_set_path_en_I\n");
	
	return start_video((struct drv_hw_context *) (&dev_context->drv_context), NULL);
}

#ifdef HDCP_ENABLE
static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status)
{
	uint8_t		query_data;

	MHL_TX_DBG_INFO(hw_context, "hdcp interrupt handling...\n");

	query_data = mhl_tx_read_reg(hw_context, TPI_HDCP_QUERY_DATA_REG);
	MHL_TX_DBG_INFO(hw_context, "R3D= %02x R29= %02x\n", tpi_int_status, query_data);

	if (BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT & tpi_int_status) {
		int link_status;

		link_status = query_data & LINK_STATUS_MASK;

		switch (link_status) {
			case LINK_STATUS_NORMAL:
				unmute_video(hw_context);
				break;

			case LINK_STATUS_LINK_LOST:
				hdcp_link_err_count++;
				start_hdcp(hw_context);
				break;
			case LINK_STATUS_RENEGOTIATION_REQ:
				MHL_TX_DBG_INFO(hw_context
						, "tpi BSTATUS2: 0x%x\n"
						, mhl_tx_read_reg(hw_context,REG_TPI_BSTATUS2)
						);
				hdcp_reneg_err_count++;
				
				mhl_tx_modify_reg(hw_context
						, TPI_SYSTEM_CONTROL_DATA_REG
						, AV_MUTE_MASK
						, AV_MUTE_MUTED);

				
				mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG, 0);
				break;
			case LINK_STATUS_LINK_SUSPENDED:	
				MHL_TX_DBG_INFO(hw_context , "######### ATTENTION!!! ######### a rsvd bit in Drake is used.\n");

				hdcp_suspend_err_count++;
				start_hdcp(hw_context);
				break;
		}
	}
	
	else if (BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT & tpi_int_status) {
		uint8_t new_link_prot_level;

		new_link_prot_level = (uint8_t)
				(query_data & (EXTENDED_LINK_PROTECTION_MASK |
							LOCAL_LINK_PROTECTION_MASK));

		switch (new_link_prot_level) {
			case (EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE):
				hdcp_link_err_count++;
				start_hdcp(hw_context);
				break;

			case EXTENDED_LINK_PROTECTION_SECURE:
			case LOCAL_LINK_PROTECTION_SECURE:
			case (EXTENDED_LINK_PROTECTION_SECURE | LOCAL_LINK_PROTECTION_SECURE):
				unmute_video(hw_context);
				break;
		}
	}
	return 0;
}
#endif 

static int to_be_deleted(struct drv_hw_context *hw_context, uint8_t int_status)
{
	hw_context = hw_context;
	int_status = int_status;
	return 0;
}

extern void hdmi_state_callback(HDMI_STATE state);
extern unsigned int sii_mhl_connected;
static unsigned int int3Number = 0;
static int int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status)
{
#if 1
	uint8_t i;
#endif
	uint8_t block_offset;

	MHL_TX_DBG_INFO(hw_context, "interrupt handling...\n");

	mhl_tx_write_reg(hw_context, REG_INTR3, int_3_status);
	if (!int_3_status)
	{
		return -1;	
	}

	if (BIT_INTR3_DDC_CMD_DONE & int_3_status) {
	   
		int ddcStatus;
		ddcStatus = mhl_tx_read_reg(hw_context,REG_DDC_STATUS);
		MHL_TX_EDID_READ(hw_context, "mhl_tx_read_reg(hw_context,REG_DDC_STATUS)=)x%x\n",ddcStatus);
		if (BIT_DDC_STATUS_DDC_NO_ACK & ddcStatus) {

			
			MHL_TX_EDID_READ(hw_context, "before si_mhl_tx_drv_issue_edid_read_request: to read block 0\n");
			if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
						hw_context->current_edid_request_block=0,
						hw_context->current_edid_request_block_batch=0)) {
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				hw_context->intr_info->msc_done_data =1;
			}
		}
		else {
			

			MHL_TX_EDID_READ(hw_context, "This batch of EDID read is complete:\n");
			MHL_TX_EDID_READ(hw_context,
					"\tcurrentEdidRequestBlock:%d\t "	\
					"\tcurrentEdidRequestBlockBatch:%d \t"	\
					"\tedidFifoBlockNumber:%d \n",
					hw_context->current_edid_request_block,
					hw_context->current_edid_request_block_batch,
					hw_context->edid_fifo_block_number);

			
			MHL_TX_EDID_READ(hw_context, "This batch:%d of EDID read:\n",hw_context->current_edid_request_block_batch);
			block_offset = 128 * hw_context->current_edid_request_block + 16 * hw_context->current_edid_request_block_batch;

			
            for (i = 0; i < 16; i++)
			{
				* (hw_context->current_edid_block_data + block_offset+ i) = mhl_tx_read_reg(hw_context, REG_DDC_DATA);
			}

			
			mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_EDID].stat_page, g_intr_tbl[INTR_EDID].stat_offset, BIT_INTR3_DDC_FIFO_FULL);
            MHL_TX_EDID_READ(hw_context, " %d \n", hw_context->current_edid_request_block_batch );
            i=0;
            
				    
            while((i++<30)&&(mhl_tx_read_reg(hw_context,REG_DDC_STATUS)&0x0B)){
                    MHL_TX_EDID_READ(hw_context, " mhl_tx_read_reg(hw_context,REG_DDC_STATUS)=0x%x 111111111\n", mhl_tx_read_reg(hw_context,REG_DDC_STATUS));
				    si_mhl_tx_drv_reset_ddc_fifo(hw_context);
				    mdelay(5);
                    MHL_TX_EDID_READ(hw_context, " mhl_tx_read_reg(hw_context,REG_DDC_STATUS)=0x%x,i=%d 22222222\n", mhl_tx_read_reg(hw_context,REG_DDC_STATUS),i);
				}
			
			if ( 7 > hw_context->current_edid_request_block_batch ) 
			{
				si_mhl_tx_drv_reset_ddc_fifo(hw_context);
				si_mhl_tx_drv_issue_edid_block_batch_read(hw_context,
						hw_context->current_edid_request_block,
						++hw_context->current_edid_request_block_batch);

			}
			else if ( 7 == hw_context->current_edid_request_block_batch ) 
			{
				int num_extensions;

				MHL_TX_EDID_READ(hw_context, "EDID block read complete\n");
				num_extensions =si_mhl_tx_get_num_cea_861_extensions(
						hw_context->intr_info->edid_parser_context,
						hw_context->current_edid_request_block);
				if (num_extensions < 0)	{
					MHL_TX_DBG_ERR(hw_context,"edid problem:%d\n",num_extensions);

					hw_context->current_edid_request_block = 0;
					hw_context->current_edid_request_block_batch = 0;
					hw_context->edid_fifo_block_number = 0;

					si_mhl_tx_drv_reset_ddc_fifo(hw_context);

					if (ne_NO_HPD == num_extensions) {

						
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
					} else {
						
						MHL_TX_EDID_READ(hw_context, "before si_mhl_tx_drv_issue_edid_read_request: to read block 0\n");
						if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
									hw_context->current_edid_request_block=0,
									hw_context->current_edid_request_block_batch=0)) {
							
							hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
							hw_context->intr_info->msc_done_data =1;
						}
					}
				} else if (hw_context->current_edid_request_block < num_extensions) {
					
					MHL_TX_EDID_READ(hw_context, "before si_mhl_tx_drv_issue_edid_read_request: to read block 0+\n");
					if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
								++hw_context->current_edid_request_block,
								hw_context->current_edid_request_block_batch=0)) {
						
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
					}
				} else {
				    mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_EDID].stat_page, g_intr_tbl[INTR_EDID].stat_offset, 0x0F);
                    enable_intr(hw_context, INTR_EDID, 0);
					MHL_TX_EDID_READ(hw_context, "All 0+ block(s) is/are read.\n");
					MHL_TX_DBG_INFO(hw_context, "All EDID have been read.\n");

					
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =0;
					hw_context->ready_for_mdt = true;

					{
						struct mhl_dev_context	*dev_context;
						struct cbus_req 		*req;

						dev_context = get_mhl_device_context(hw_context);
	                    req = dev_context->current_cbus_req;
						si_mhl_tx_handle_atomic_hw_edid_read_complete(dev_context->edid_parser_context, req);
						dev_context->edid_parse_done = true;		
						dev_context->misc_flags.flags.edid_loop_active = 0;
						mhl_event_notify(dev_context, MHL_TX_EVENT_EDID_DONE, NULL, NULL);
					    int3Number = 0;
					    mhl_tx_stop_timer(dev_context, dev_context->cbus_dpi_timer);
					}
				}
			} 
		}
	}

	return int_3_status;
}

static	int		get_cbus_connection_status(struct drv_hw_context *hw_context)
{
	return (BIT_CBUS_HPD & mhl_tx_read_reg(hw_context, REG_CBUS_STATUS));
}

static int mhl_cbus_err_isr(struct drv_hw_context *hw_context,
							uint8_t cbus_err_int)
{
	int	ret_val = 0;
	uint8_t ddc_abort_reason = 0;
	uint8_t msc_abort_reason = 0;

	MHL_TX_DBG_INFO(hw_context, "interrupt handling...\n");

	if(cbus_err_int & BIT_CBUS_DDC_ABRT) {

		ddc_abort_reason = mhl_tx_read_reg(hw_context, REG_CBUS_DDC_ABORT_INT);

		MHL_TX_DBG_ERR(hw_context, "CBUS DDC ABORT. Reason = %02X\n", ddc_abort_reason);

		if(DDC_ABORT_THRESHOLD < ++ddc_abort_count) {
			si_mhl_tx_drv_reset_ddc_fifo(hw_context);
			ddc_abort_count = 0;
			MHL_TX_DBG_ERR(hw_context, "DDC fifo has been reset.%s\n"
						,((BIT_CBUS_DDC_PEER_ABORT & ddc_abort_reason)
							&&
						  (ddc_abort_count >= DDC_ABORT_THRESHOLD)
						 ) ? "  Please reset sink device!!!"
						   : ""
						);
		}
	}
	if(cbus_err_int & BIT_CBUS_MSC_ABORT_RCVD) {
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context, REG_MSC_RCV_ERROR);

		++msc_abort_count;	

		MHL_TX_DBG_ERR(hw_context, "#%d: ABORT during MSC RCV. Reason = %02X\n",
				msc_abort_count, msc_abort_reason);

		
	}
	if(cbus_err_int & BIT_CBUS_CMD_ABORT) {
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context, REG_CBUS_MSC_MT_ABORT_INT);

		MHL_TX_DBG_ERR(hw_context, "CBUS ABORT during MSC SEND. Reason = %02X\n",
						msc_abort_reason);

		mhl_tx_write_reg(hw_context,REG_CBUS_MSC_MT_ABORT_INT,msc_abort_reason);
	}
	if (msc_abort_reason) {
		if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Retry threshold exceeded\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Protocol Error\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Translation layer timeout\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "Undefined opcode\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT & msc_abort_reason) {
				MHL_TX_DBG_ERR(hw_context, "MSC Peer sent an ABORT\n");
		}
	}
	return(ret_val);
}

 extern void reset_av_info();
static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int)
{
	MHL_TX_DBG_INFO(hw_context, "interrupt handling..., cbus_int=0x%02x\n",cbus_int);
	
    if (cbus_int & BIT_CBUS_MSC_MR_WRITE_BURST){ 
            mhl_tx_write_reg(hw_context, REG_CBUS_INT_0,cbus_int & BIT_CBUS_MSC_MR_WRITE_BURST);
            mhl_tx_read_reg_block(hw_context, REG_CBUS_MHL_SCRPAD_0,
						ARRAY_SIZE(hw_context->write_burst_data),
						hw_context->write_burst_data);
			
			struct mhl_dev_context	*dev_context;
			dev_context = get_mhl_device_context(hw_context);
			mhl_event_notify(dev_context, MHL_TX_EVENT_SMB_DATA, NULL, (void *)(hw_context->write_burst_data+8));
        }   

	if (cbus_int & ~BIT_CBUS_HPD_RCVD){
		
		mhl_tx_write_reg(hw_context, REG_CBUS_INT_0,cbus_int & ~BIT_CBUS_HPD_RCVD);
	}	

	if (BIT_CBUS_HPD_RCVD & cbus_int) {
		uint8_t cbus_status;
		uint8_t status;

		
		cbus_status = mhl_tx_read_reg(hw_context, REG_CBUS_STATUS);
		status = cbus_status & BIT_CBUS_HPD;

		if(BIT_CBUS_HPD & (hw_context->cbus_status ^ cbus_status)) {

			
			mhl_tx_write_reg(hw_context, REG_CBUS_INT_0, BIT_CBUS_HPD_RCVD);
			MHL_TX_DBG_INFO(hw_context, "HPD change\n");
		}
		else{
			MHL_TX_DBG_ERR(hw_context, "missed HPD change\n");

			
			status ^= BIT_CBUS_HPD;
			cbus_status ^= BIT_CBUS_HPD;
		}

		MHL_TX_DBG_INFO(hw_context, "DS HPD changed to %02X\n", status);

		hw_context->intr_info->flags |= DRV_INTR_FLAG_HPD_CHANGE;
		hw_context->intr_info->hpd_status = status;

		if ( 0 == status) {
			struct mhl_dev_context	*dev_context;
			dev_context = get_mhl_device_context(hw_context);
			MHL_TX_DBG_ERR(hw_context, "got CLR_HPD\n\n");

			

			hw_context->current_edid_request_block = 0;

#ifdef ENABLE_GEN2 
			disable_gen2_write_burst(hw_context);
#endif 
			hw_context->ready_for_mdt = false;

			
			hw_context->video_ready = true;	
		
			
			si_edid_reset(dev_context->edid_parser_context);
		}
		else {
			MHL_TX_DBG_INFO(hw_context, "\n\nGot SET_HPD\n\n");
			hw_context->video_ready = true;	
		
		}
		
		stop_video(hw_context);

		hw_context->cbus_status = cbus_status;
	}

	if (BIT_CBUS_MSC_MT_DONE & cbus_int) {
		MHL_TX_DBG_INFO(hw_context, "MSC_REQ_DONE\n");

		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
		hw_context->intr_info->msc_done_data =
			mhl_tx_read_reg(hw_context, REG_CBUS_PRI_RD_DATA_1ST);

#ifdef ENABLE_GEN2 
		
		enable_gen2_write_burst(hw_context);
#endif 
	}
	if (BIT_CBUS_MSC_MT_DONE_NACK & cbus_int){
		MHL_TX_DBG_ERR(hw_context,"MSC_MT_DONE_NACK\n");
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_NAK;
	}

	if (BIT_CBUS_MSC_MR_WRITE_STAT & cbus_int) {

		
		mhl_tx_read_reg_block(hw_context, REG_CBUS_WRITE_STAT_0,
				ARRAY_SIZE(hw_context->intr_info->write_stat),
				hw_context->intr_info->write_stat);

		if(MHL_STATUS_DCAP_RDY & hw_context->intr_info->write_stat[0]) {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot DCAP_RDY\n\n");

			
			
			
			
		}
		hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_STAT;
	}

	if ((BIT_CBUS_MSC_MR_MSC_MSG & cbus_int)) {
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_RECVD;
		mhl_tx_read_reg_block(hw_context,
				REG_CBUS_MSC_MR_MSC_MSG_RCVD_1ST_DATA,
				ARRAY_SIZE(hw_context->intr_info->msc_msg),
				hw_context->intr_info->msc_msg);

		MHL_TX_DBG_INFO(hw_context, "MSC MSG: %02X %02X\n",
				hw_context->intr_info->msc_msg[0],
				hw_context->intr_info->msc_msg[1]);
	}

	if(BIT_CBUS_MSC_MR_SET_INT & cbus_int) {
		MHL_TX_DBG_INFO(hw_context, "MHL INTR Received\n");
		hw_context->intr_info->flags |= DRV_INTR_FLAG_SET_INT;
		mhl_tx_read_reg_block(hw_context,
				REG_CBUS_SET_INT_0,
				ARRAY_SIZE(hw_context->intr_info->int_msg),
				hw_context->intr_info->int_msg);

		mhl_tx_write_reg_block(hw_context,
				REG_CBUS_SET_INT_0,
				ARRAY_SIZE(hw_context->intr_info->int_msg),
				hw_context->intr_info->int_msg);

		if (MHL_INT_EDID_CHG & hw_context->intr_info->int_msg[1]) {
			int reg_val;

			MHL_TX_DBG_INFO(hw_context, "\n\ngot EDID_CHG\n\n");

			
			mhl_tx_modify_reg(hw_context,REG_TPI_INFO_FSEL
					,BIT_TPI_INFO_RPT 
					,0
					);

			
			

			stop_video(hw_context);

			
			reg_val = mhl_tx_read_reg(hw_context, REG_TPI_SEL);
			MHL_TX_DBG_INFO(hw_context, "REG_TPI_SEL:%02x\n", reg_val);
			reg_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
			reg_val |= BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI;
			mhl_tx_write_reg(hw_context, REG_TPI_SEL, reg_val);

			
			MHL_TX_DBG_INFO(hw_context, "REG_TPI_SEL:%02x\n", reg_val);
			reg_val &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;
			reg_val |= BIT_TPI_SEL_SW_TPI_EN_HW_TPI;
			mhl_tx_write_reg(hw_context, REG_TPI_SEL, reg_val);

#ifdef HDCP_ENABLE
			mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);
			mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP2].stat_page, g_intr_tbl[INTR_HDCP2].stat_offset, 0xff);

#endif 
		} else if (MHL_INT_DSCR_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_INFO(hw_context, "got DSCR_CHG\n");
			if(hw_context->gen2_write_burst) {
				MHL_TX_DBG_INFO(hw_context,
						"Ignored DSCR_CHG since MDT is enabled\n");
			} else {
    			struct mhl_dev_context	*dev_context;
    			dev_context = get_mhl_device_context(hw_context);
				mhl_tx_read_reg_block(hw_context, REG_CBUS_MHL_SCRPAD_0,
						ARRAY_SIZE(hw_context->write_burst_data),
						hw_context->write_burst_data);
                si_mhl_tx_process_write_burst_data(dev_context);
			}
		} else if (MHL_INT_DCAP_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_INFO(hw_context, "\n\ngot DCAP_CHG\n\n");
		}
	}
	return -1;
}

#ifdef HDCP_ENABLE
static int int_hdcp2_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status)
{
	uint8_t		query_data;

	MHL_TX_DBG_INFO(hw_context, "hdcp2 interrupt handling...\n");

	query_data = mhl_tx_read_reg(hw_context, TPI_HDCP_QUERY_DATA_REG);
	MHL_TX_DBG_INFO(hw_context, "R3E= %02x R29= %02x\n", tpi_int_status, query_data);

	if (BIT_TPI_INTR_ST1_BKSV_DONE & tpi_int_status) {
		MHL_TX_DBG_INFO(hw_context, "BIT_TPI_INTR_ST1_BKSV_DONE handling...\n");
		if (PROTECTION_TYPE_MASK & query_data) {

			
			int temp;
			do{
				temp = mhl_tx_read_reg(hw_context,REG_TPI_HW_DBG6) & 0x1F;
			} while (temp == 2);
			if (temp < 2)
				return 0; 

			query_data = mhl_tx_read_reg(hw_context, TPI_HDCP_QUERY_DATA_REG);


#if 0			
			if (HDCP_REPEATER_YES== (HDCP_REPEATER_MASK & query_data)){
				msleep(HDCP_RPTR_CTS_DELAY_MS);
			}
#endif
			
			mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG,
					BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX);
#if 0
			mhl_tx_write_reg(hw_context, TPI_HDCP_CONTROL_DATA_REG,
					  BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
					| BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_ENABLE);
#endif
		}
	}
	else if ( BIT_TPI_INTR_ST1_BKSV_ERR & tpi_int_status) {
		MHL_TX_DBG_INFO(hw_context, "BIT_TPI_INTR_ST1_BKSV_ERR handling...\n");
		hdcp_bksv_err_count++;
		start_hdcp(hw_context);
	}

	return 0;
}
#endif 

static int int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status)
{
	int ret_val = 0;

	MHL_TX_DBG_INFO(hw_context, "interrupt handling...\n");

	if (int_1_status)
	{
		if (BIT_INTR1_RSEN_CHG & int_1_status) {
			uint8_t rsen = 0;

			MHL_TX_DBG_INFO(hw_context, "Got RSEN CHG...\n");
			
			rsen =  mhl_tx_read_reg(hw_context, REG_SYS_STAT);
			if (rsen & BIT_STAT_RSEN)
			{
				MHL_TX_DBG_INFO(hw_context, "Got RSEN_CHG: RSEN_ON\n");
			}
			else
			{
				MHL_TX_DBG_INFO(hw_context, "Got RSEN_CHG: RSEN_OFF,mhl_tx_read_reg(hw_context, REG_INTR4)=0x%x\n",mhl_tx_read_reg(hw_context, REG_INTR4));
			}
		}
	}

	return ret_val;
}

int get_device_id(struct drv_hw_context *hw_context)
{
	int ret_val = 0;
	uint16_t number = 0;

	ret_val =  mhl_tx_read_reg(hw_context, REG_DEV_IDH);
	if (ret_val < 0)
	{
		MHL_TX_DBG_ERR(hw_context, "I2C error 0x%x\n", ret_val);
	}
	else
	{
		number = ret_val << 8;

		ret_val = mhl_tx_read_reg(hw_context, REG_DEV_IDL);
		if (ret_val < 0)
		{
			MHL_TX_DBG_ERR(hw_context, "I2C error 0x%x\n", ret_val);
		}
		else
		{
			ret_val |= number;
			MHL_TX_DBG_ERR(hw_context, "Device ID is: %04X\n", ret_val);
		}
	}

	if ( 0x8348 == ret_val )
	{
		return DEVICE_ID_8348;
	}
	else if ( 0x8346 == ret_val )
	{
		return DEVICE_ID_8346;
	}
	else
	{
		return 0;
	}
}
static int get_device_rev(struct drv_hw_context *hw_context)
{
	int ret_val;

	#if MHL_PRODUCT_NUM==8348 
		ret_val = mhl_tx_read_reg(hw_context, REG_DEV_REV);
	#else 
  		
  		af
  		MHL_TX_DBG_ERR(hw_context, "error MHL_PRODUCT_NUM not defeind \n");
	#endif 

	if (ret_val < 0) {
		MHL_TX_DBG_ERR(hw_context, "I2C error\n");
		ret_val = -1;
	}

    return ret_val;
}

static void board_reset(struct drv_hw_context *hw_context,
		uint16_t hwResetPeriod,
		uint16_t hwResetDelay)
{
#if defined(GPIO_MHL_RST_B_PIN)
    printk("%s,%d+\n", __func__, __LINE__);
    mt_set_gpio_mode(GPIO_MHL_RST_B_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_MHL_RST_B_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ONE);
    mdelay(hwResetPeriod);
    mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ZERO);
    mdelay(hwResetPeriod);
    mt_set_gpio_out(GPIO_MHL_RST_B_PIN, GPIO_OUT_ONE);
    mdelay(hwResetDelay);
#elif defined(CONFIG_MTK_MT6306_SUPPORT)
    printk(":MT6306 %s,%d\n", __func__, __LINE__);
    mt6306_set_gpio_dir(GPIO4, GPIO_DIR_OUT);
    mt6306_set_gpio_out(GPIO4, GPIO_OUT_ONE);
    mdelay(hwResetPeriod);
    mt6306_set_gpio_out(GPIO4, GPIO_OUT_ZERO);
    mdelay(hwResetPeriod);
    mt6306_set_gpio_out(GPIO4, GPIO_OUT_ONE);
    mdelay(hwResetDelay);
#else
    printk("%s,%d Error: GPIO_MHL_RST_B_PIN is not defined\n", __func__, __LINE__);
#endif

}

static void clear_and_disable_on_disconnect(struct drv_hw_context *hw_context)
{
	uint8_t	intr_num;
	MHL_TX_DBG_INFO(hw_context, "called.\n");
    
	
	for(intr_num = 0; intr_num < MAX_INTR; intr_num++)
	{
		
		if(INTR_DISC == intr_num) {
			
			mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xBF);

			
			enable_intr(hw_context, INTR_DISC, BIT_INTR4_RGND_DETECTION);
		}
		
		else if(INTR_INTR1 == intr_num) {
			
			mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xFF);

			
                        enable_intr(hw_context, INTR_INTR1, 0x00);
		} else {
			
			mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xFF);
			enable_intr(hw_context, intr_num, 0x00);
		}
	}
}

static void switch_to_d3(struct drv_hw_context *hw_context,bool do_interrupt_clear)
{
    
    MHL_TX_DBG_INFO(hw_context, "switch_to_d3 called, do_interrupt_clear=%d\n",do_interrupt_clear);
    mhl_tx_vbus_control(VBUS_OFF);

    
   
    
    mdelay(50);

    if (do_interrupt_clear){
               clear_and_disable_on_disconnect(hw_context);
    }
    mhl_tx_modify_reg(hw_context, REG_DISC_CTRL4,
                                 BIT_DC6_USB_OVERRIDE_VALUE,
                                 BIT_DC6_USB_OVERRIDE_VALUE);
    mhl_tx_modify_reg(hw_context, REG_DISC_CTRL6,
                                 BIT_DC6_USB_D_OVERRIDE_ON,
                                 BIT_DC6_USB_D_OVERRIDE_ON);
    
    mhl_tx_modify_reg(hw_context, REG_DPD,
                                 BIT_MASTER_POWER_CTRL,
                                 0x00);
	

	
}


void	ForceSwitchToD3( struct mhl_dev_context *dev_context)
{
    MHL_TX_DBG_INFO((struct drv_hw_context *)(&dev_context->drv_context), "ForceSwitchToD3..............\n");
    
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
    mdt_destroy(dev_context);
#endif
    
    
	return;
}

void ForceNotSwitchToD3( )
{    
    board_reset(NULL,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
}


static void disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear)
{
	MHL_TX_DBG_INFO(hw_context, "called.\n");

	mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_OFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

	mhl_tx_write_reg(hw_context, REG_DISC_CTRL2, REG_DISC_CTRL2_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL4, REG_DISC_CTRL4_DEFVAL);
	mhl_tx_write_reg(hw_context, REG_DISC_CTRL5, REG_DISC_CTRL5_DEFVAL);

	

	
		clear_and_disable_on_disconnect(hw_context);
	

	
	hw_context->cbus_status = 0;

}


static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status)
{
	int	ret_val = 0; 

	MHL_TX_DBG_INFO(hw_context, "interrupt handling...\n");

	if(BIT_INTR4_CBUS_LKOUT & int_4_status){
		mhl_tx_write_reg(hw_context, REG_INTR4, 0x30);
		int_4_status = int_4_status & 0xCF;
		MHL_TX_DBG_INFO(hw_context, "BIT_INTR4_CBUS_LKOUT...\n");
		si_mhl_tx_chip_initialize(hw_context);
	}

	if ((BIT_INTR4_CBUS_DISCONNECT  & int_4_status) ||
		(BIT_INTR4_NON_MHL_EST & int_4_status)) {
		struct mhl_dev_context	*dev_context;
    	dev_context = get_mhl_device_context(hw_context);
		MHL_TX_DBG_ERR(hw_context, "got CBUS_DIS. MHL disconnection or USB\n");

		
               
               
               hw_context->intr_info->flags |= DRV_INTR_FLAG_DISCONNECT;
#if 0
               if (BIT_INTR4_CBUS_DISCONNECT & int_4_status){
                        disconnect_mhl(hw_context,true);
                        switch_to_d3(hw_context,false);
               }else{ 
                        disconnect_mhl(hw_context,false);
                        switch_to_d3(hw_context,true);
               }
#else
                mhl_tx_stop_timer(dev_context, dev_context->cbus_dpi_timer);
                mhl_tx_stop_timer(dev_context, dev_context->cbus_abort_timer);
                disconnect_mhl(hw_context,true);
                switch_to_d3(hw_context,false);
#endif
#ifdef CONFIG_HTC_MHL_DETECTION
		mhl_connect_notify(false);
#endif
		

		ret_val = 0xFF;	

		MHL_TX_DBG_INFO(hw_context, "CBUS disconnection or USB handling done.\n");
	}

	else if (int_4_status & BIT_INTR4_RGND_DETECTION) {
		mhl_tx_modify_reg(hw_context, REG_DISC_STAT2,
					0x80, 0x80);
		if(0x02 == (mhl_tx_read_reg(hw_context, REG_DISC_STAT2) & 0x03)) {
			MHL_TX_DBG_INFO(hw_context, "Cable impedance = 1k (MHL Device)\n");
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL1, 0x27);
			
			
            

            tmds_configure(hw_context);

            
            power_up(hw_context);

            mhl_tx_write_reg(hw_context, REG_INTR4_MASK, 0x00); 

            
            mhl_tx_write_reg(hw_context, REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_OFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

            mhl_tx_write_reg(hw_context, REG_DISC_CTRL2, REG_DISC_CTRL2_DEFVAL);
            mhl_tx_write_reg(hw_context, REG_DISC_CTRL4, REG_DISC_CTRL4_DEFVAL);
            mhl_tx_write_reg(hw_context, REG_DISC_CTRL5, REG_DISC_CTRL5_DEFVAL);
			

			
			mhl_tx_write_reg(hw_context, REG_DISC_CTRL9
					, BIT_DC9_WAKE_DRVFLT
					| BIT_DC9_DISC_PULSE_PROCEED
					);

			
			mhl_tx_vbus_control(VBUS_ON);
			msleep(100);
		}
		
		enable_intr(hw_context, INTR_DISC,
				  BIT_INTR4_MHL_EST			
				| BIT_INTR4_NON_MHL_EST		
				| BIT_INTR4_CBUS_LKOUT		
				| BIT_INTR4_CBUS_DISCONNECT	
				| BIT_INTR4_RGND_DETECTION
				| BIT_INTR4_VBUS_CHG 
				);
		
#ifdef CONFIG_HTC_MHL_DETECTION
		mhl_connect_notify(true);
#endif
	}

	else if(int_4_status & BIT_INTR4_MHL_EST) {
		struct mhl_dev_context	*dev_context;
    	dev_context = get_mhl_device_context(hw_context);
		MHL_TX_DBG_ERR(hw_context, "got MHL_EST.  MHL connection\n");

		
		
		

		

		
		init_regs(hw_context);
		
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CONNECT;

		
		enable_intr(hw_context, INTR_MERR,
				 ( BIT_CBUS_DDC_ABRT
				 | BIT_CBUS_MSC_ABORT_RCVD
				 | BIT_CBUS_CMD_ABORT
				 ));
		enable_intr(hw_context, INTR_MSC,
				(  BIT_CBUS_MSC_MT_DONE
				 | BIT_CBUS_HPD_RCVD
				 | BIT_CBUS_MSC_MR_WRITE_STAT
				 | BIT_CBUS_MSC_MR_MSC_MSG
				 | BIT_CBUS_MSC_MR_WRITE_BURST
				 | BIT_CBUS_MSC_MR_SET_INT
				 | BIT_CBUS_MSC_MT_DONE_NACK
				  | BIT_CBUS_CNX_CHG
				));
		
		
	}

	return ret_val;
}

void SiiMhlTxDrvGetScratchPad(struct drv_hw_context *hw_context,uint8_t startReg,uint8_t *pData,uint8_t length)
{
    int i;
    for (i = 0; i < length;++i,++startReg)
    {
        
    }
}

static	int	g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	uint8_t	ret_val = 0,i;
	uint8_t	mdt_buffer[20];

	MHL_TX_DBG_INFO(hw_context, "interrupt handling...\n");

	
	ret_val  = mhl_tx_read_reg(hw_context, REG_CBUS_MDT_INT_1);

	if(ret_val) {
		mhl_tx_write_reg(hw_context, REG_CBUS_MDT_INT_1, ret_val);
		MHL_TX_DBG_INFO(hw_context, "\n\ngot MDT Error = %02X\n", ret_val);
	} else {
		uint8_t	length;

		
         for (i = 0; i < 16; i++)
			{
				mdt_buffer[i] = mhl_tx_read_reg(hw_context, REG_CBUS_MDT_RCV_READ_PORT);
			}
		
		length = mdt_buffer[0];
		/*
		 * There isn't any way to know how much of the scratch pad
		 * was written so we have to read it all.  The app. will have
		 * to parse the data to know how much of it is valid.
		 */
		memcpy(hw_context->write_burst_data, &mdt_buffer[1], 16);

		
		hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_BURST;

		mhl_tx_write_reg(hw_context,
				  REG_CBUS_MDT_RCV_CONTROL,
				  BIT_CBUS_MDT_RCV_CONTROL_RFIFO_CLR_CUR_CLEAR
				| BIT_CBUS_MDT_RCV_CONTROL_RCV_EN_ENABLE
				);
	}
	return 0;
}

static void enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num, uint8_t intr_mask)
{
	g_intr_tbl[intr_num].mask = intr_mask;
	mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].mask_page, g_intr_tbl[intr_num].mask_offset, intr_mask);
}

void enable_intr3(struct drv_hw_context *hw_context)
{
	enable_intr(hw_context, INTR_EDID, BIT_INTR3_DDC_CMD_DONE);
		
}

void si_mhl_tx_drv_device_isr(struct drv_hw_context *hw_context, struct interrupt_info *intr_info)
{
	uint8_t	intr_num;

	hw_context->intr_info = intr_info;

	

	
	for(intr_num = 0; (intr_num < MAX_INTR) ; intr_num++) {
		if(g_intr_tbl[intr_num].mask)
		{
			int reg_value;
			uint8_t	intr_stat;

			reg_value = mhl_tx_read_reg(hw_context,
					g_intr_tbl[intr_num].stat_page,
					g_intr_tbl[intr_num].stat_offset);
            

			if(reg_value < 0) {
				return;
			}

			intr_stat = (uint8_t)reg_value;
			if(intr_stat) {
					
					mhl_tx_write_reg(hw_context,
							g_intr_tbl[intr_num].stat_page,
							g_intr_tbl[intr_num].stat_offset,
							intr_stat);
					
			}
            
			
			
			
			
			{
				int already_cleared;

#ifdef	PRINT_ALL_INTR
				MHL_TX_DBG_ERR(hw_context, "INTR-%s = %02X\n", g_intr_tbl[intr_num].name, intr_stat);
#else	
				MHL_TX_DBG_INFO(hw_context, "INTR-%s = %02X\n", g_intr_tbl[intr_num].name, intr_stat);
#endif	

				already_cleared = g_intr_tbl[intr_num].isr(hw_context, intr_stat);
				
					
			}

		} 
#ifdef	PRINT_ALL_INTR
		
		else {
			uint8_t	intr_stat;
			
			intr_stat = mhl_tx_read_reg(hw_context,
					g_intr_tbl[intr_num].stat_page,
					g_intr_tbl[intr_num].stat_offset);
			MHL_TX_DBG_ERR(hw_context, "INTN-%s = %02X\n", g_intr_tbl[intr_num].name, intr_stat);

		}
#endif	
	}
}

extern int	chip_device_id;
extern bool need_reset_usb_switch ;
int si_mhl_tx_chip_initialize(struct drv_hw_context *hw_context)
{
	int ret_val;
	int status = -1;

	siHdmiTx_VideoSel(HDMI_1080P60);
	siHdmiTx_AudioSel(AUDIO_44K_2CH);
	board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
	if(true ==need_reset_usb_switch)
	{
    	mhl_tx_modify_reg(hw_context, REG_DISC_CTRL7, 0xC0,0x40);
    	
    	mdelay(5);    	
    	board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
    	
    	mhl_tx_modify_reg(hw_context, REG_DISC_CTRL7, 0xC0,0x20);
    }
	mhl_tx_write_reg(hw_context, REG_INT_CTRL, BIT_INT_CTRL_POLARITY_LEVEL_LOW | BIT_INT_CTRL_OPEN_DRAIN);	

	tmds_configure(hw_context);

	
	power_up(hw_context);

	mhl_tx_write_reg(hw_context, REG_INTR4_MASK, 0x00);	

	mhl_tx_write_reg(hw_context, REG_DISC_CTRL1, VAL_DISC_CTRL1_DEFAULT | BIT_DISC_CTRL1_MHL_DISCOVERY_ENABLE);	

	ret_val = get_device_rev(hw_context);
	hw_context->chip_rev_id = (uint8_t)ret_val;

	ret_val = get_device_id(hw_context);
	hw_context->chip_device_id = (uint16_t)ret_val;
    chip_device_id = ret_val;
	if (ret_val > 0)
	{
		MHL_TX_DBG_ERR(hw_context, "Found SiI%04X rev: %01X.%01X\n",
				hw_context->chip_device_id,
				hw_context->chip_rev_id >> 4,
				(hw_context->chip_rev_id & 0x0f));

		
		disconnect_mhl(hw_context, true);
		
		switch_to_d3(hw_context, false);

		status = 0;

		
		
		hw_context->valid_vsif = 0;
		
		hw_context->valid_3d = 0;

		
		
		memset(&hw_context->current_vs_info_frame, 0, sizeof(hw_context->current_vs_info_frame));
		
	}
	else
	{
		MHL_TX_DBG_ERR(hw_context, "SiI8348 is not found!\n");
		MHL_TX_DBG_ERR(hw_context, "Found %04X rev: %01X.%01X\n",
				hw_context->chip_device_id,
				hw_context->chip_rev_id >> 4,
				(hw_context->chip_rev_id & 0x0f));
	}

	return status;
}


void si_mhl_tx_drv_shutdown(struct drv_hw_context *hw_context)
{
	if( is_reset_on_exit_requested() )
	{
		board_reset(hw_context,TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);
		
		MHL_TX_DBG_ERR(hw_context, "MHL hardware was reset\n");
	}
}
void siHdmiTx_VideoSel (int vmode)
{
	int AspectRatio = 0;
	video_data.inputColorSpace	= acsRGB;
	video_data.outputColorSpace = acsRGB;
	video_data.inputVideoCode	= vmode;
	
	TX_DEBUG_PRINT(("video_data.inputVideoCode:0x%02x\n",(int)video_data.inputVideoCode));
	
	
		
	switch (vmode)
	{
		case HDMI_480I60_4X3:
		case HDMI_576I50_4X3:
			AspectRatio = VMD_ASPECT_RATIO_4x3;
			break;
			
		case HDMI_480I60_16X9:
		case HDMI_576I50_16X9:
			AspectRatio  	= VMD_ASPECT_RATIO_16x9;
			break;
			
		case HDMI_480P60_4X3:
		case HDMI_576P50_4X3:
		case HDMI_640X480P:
			AspectRatio  	= VMD_ASPECT_RATIO_4x3;
			break;

		case HDMI_480P60_16X9:
		case HDMI_576P50_16X9:
			AspectRatio  	= VMD_ASPECT_RATIO_16x9;
			break;
			
		case HDMI_720P60:
		case HDMI_720P50:
		case HDMI_1080I60:
		case HDMI_1080I50:
		case HDMI_1080P24:
		case HDMI_1080P25:
		case HDMI_1080P30:	
		case HDMI_1080P50:
		case HDMI_1080P60:
			AspectRatio  	= VMD_ASPECT_RATIO_16x9;
			break;
				
		default:
			break;
		}
	if (AspectRatio == VMD_ASPECT_RATIO_4x3)
        video_data.inputcolorimetryAspectRatio = 0x18;
    else video_data.inputcolorimetryAspectRatio = 0x28;
   		video_data.input_AR = AspectRatio;

}

void siHdmiTx_AudioSel (int AduioMode)
{
	Audio_mode_fs			= AduioMode;
}
