#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include "tps65132_iic.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif

#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#define LCM_DSI_CMD_MODE					0
#define FRAME_WIDTH  						(1080)
#define FRAME_HEIGHT 						(1920)
#define GPIO_65132_EN						GPIO_LCD_BIAS_ENP_PIN

#define REGFLAG_PORT_SWAP					0xFFFA
#define REGFLAG_DELAY						0xFFFC
#define REGFLAG_END_OF_TABLE				0xFFFD   

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define MDELAY(n) (lcm_util.mdelay(n))

#define dsi_set_cmd_by_cmdq_dual(handle,cmd,count,ppara,force_update)    lcm_util.dsi_set_cmdq_V23(handle,cmd,count,ppara,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap) lcm_util.dsi_swap_port(swap)

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0xB0,  1, {0x04}},
	{0x3A,  1, {0x70}},
	{0xD6,  1, {0x01}},
	{0xB3,  6, {0x14, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0xB4,  2, {0x0C, 0x00}},
	{0xB6,  2, {0x3A, 0xD3}},
	{0xB7,  1, {0x00}},
	{0xB8,  6, {0x07, 0x90, 0x1E, 0x10, 0x1E, 0x32}},
	{0xB9,  6, {0x07, 0x82, 0x3C, 0x10, 0x3C, 0x87}},
	{0xBA,  6, {0x07, 0x78, 0x64, 0x10, 0x64, 0xB4}},
	{0xBE,  2, {0x00, 0x04}},
	{0xC0,  1, {0x11}},
	{0xC1, 34, {0x84, 0x60, 0x00, 0xFF, 0x8F,
		    0xF2, 0xD1, 0x31, 0xE1, 0x47,
		    0xF8, 0x5C, 0x63, 0xAC, 0xB9,
		    0x07, 0xE3, 0x07, 0xE6, 0xFC,
		    0x4F, 0xC4, 0xFF, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x68, 0x01,
		    0x00, 0x22, 0x00, 0x01}},
	{0xC2,  7, {0x31, 0xF7, 0x82, 0x06, 0x06, 0x00, 0x00}},
	{0xC3,  3, {0x00, 0x00, 0x00}},
	{0xC4, 22, {0x70, 0x01, 0x00, 0x01, 0x01,
		    0x01, 0x01, 0x01, 0x01, 0x01,
		    0x03, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x00, 0x00,
		    0x01, 0x03}},
	{0xC5,  1, {0x00}},
	{0xC6, 40, {0x71, 0x00, 0x00, 0x08, 0x66,
		    0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x14, 0x16, 0x07,
		    0x71, 0x00, 0x00, 0x08, 0x66,
		    0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x14, 0x16, 0x07}},
	{0xCA, 32, {0x01, 0x01, 0x97, 0x75, 0x97,
		    0xA8, 0xBB, 0xA8, 0x08, 0x2F,
		    0x05, 0x78, 0x0A, 0x4A, 0x37,
		    0xA0, 0x01, 0xA0, 0x08, 0x08,
		    0x20, 0x10, 0x08, 0x08, 0xCB,
		    0xF2, 0x10, 0x10, 0x3F, 0x3F,
		    0x3F, 0x3F}},

	{0xCB,  9, {0xEC, 0xFD, 0xBF, 0x37, 0x20,
		    0x00, 0x00, 0x04, 0xC0}},
	{0xCC,  1, {0x0E}},
	{0xCE, 23, {0x75, 0x00, 0x01, 0x02, 0x03,
		    0x45, 0xC5, 0xDC, 0xE3, 0xED,
		    0xF4, 0xF7, 0xF9, 0xFB, 0xFD,
		    0xFE, 0xFF, 0x01, 0x5F, 0x04,
		    0x04, 0x44, 0x20}},
	{0xCF,  5, {0x00, 0x00, 0xC1, 0x05, 0x3F}},
	{0xD0, 10, {0x11, 0x85, 0xBB, 0x54, 0xCF,
		    0x4C, 0x19, 0x19, 0x0C, 0x00}},
	{0xD1, 26, {0x20, 0x00, 0x00, 0x04, 0x08,
		    0x0C, 0x10, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x3C, 0x04, 0x20,
		    0x00, 0x00, 0x04, 0x08, 0x0C,
		    0x10, 0x00, 0x00, 0x3C, 0xC6,
		    0xF0}},



	{0xD3, 25, {0x1B, 0x33, 0xBB, 0xBB, 0xB3,
		    0x33, 0x33, 0x33, 0x00, 0x01,
		    0x00, 0xA0, 0xC8, 0xA0, 0x0D,
		    0x4D, 0x4D, 0x33, 0x3B, 0x37,
		    0x72, 0x57, 0x3D, 0xBF, 0x33}},
	{0xD5,  0, {}},
	{0xD7, 22, {0x84, 0xE0, 0x7F, 0xA8, 0xCE,
		    0x38, 0xFC, 0xC1, 0x18, 0xE7,
		    0x8F, 0x1F, 0x3C, 0x10, 0xFA,
		    0xC3, 0x0F, 0x04, 0x41, 0x00,
		    0x00, 0x00}},
	{0xDD,  2, {0x10, 0x8C}},
	{0xDE,  4, {0x00, 0xFF, 0x07, 0x10}},
	{0x53,  1, {0x24}},
	{0x55,  1, {0x02}},
	{0xB9,  6, {0x07, 0x7A, 0x45, 0x20, 0x13, 0x4B}},
	{0xBA,  6, {0x07, 0x7C, 0x57, 0x20, 0x13, 0x99}},
	{0x29,  0, {}},
	{REGFLAG_DELAY, 10, {}},
	{0x11,  0, {}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,  0, {}},
	{0xD3, 25, {0x13, 0x33, 0xBB, 0xB3, 0xB3,
		    0x33, 0x33, 0x33, 0x00, 0x01,
		    0x00, 0xA0, 0xE8, 0xA0, 0x0D,
		    0x4D, 0x4D, 0x33, 0x3B, 0x37,
		    0x72, 0x57, 0x3D, 0xBF, 0x33}},
	{REGFLAG_DELAY, 50, {}},
	{0x10,  0, {}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
			if(table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE :
			break;

		case REGFLAG_PORT_SWAP:
			dsi_swap_port(1);
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
#if defined(CONFIG_CUSTOM_KERNEL_LCM_PHY_WIDTH) && defined(CONFIG_CUSTOM_KERNEL_LCM_PHY_HEIGHT)
	params->physical_width = CONFIG_CUSTOM_KERNEL_LCM_PHY_WIDTH;
	params->physical_height = CONFIG_CUSTOM_KERNEL_LCM_PHY_HEIGHT;
#endif

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif
	
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	
	params->dsi.packet_size = 256;
	params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 4;

	
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 4;
	params->dsi.vertical_frontporch	= 74;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 44;
	params->dsi.horizontal_frontporch = 72;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 460; 

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[2].cmd = 0xb0;
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x04;
	params->dsi.lcm_esd_check_table[1].cmd = 0x36;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x40;
	params->dsi.lcm_esd_check_table[0].cmd = 0xd6;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x01;
	params->dsi.clk_lp_per_line_enable = 0;

	
	params->dsi.lane_swap_en = 0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_2;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_CK;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_1;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_3;
	params->dsi.lane_swap[MIPITX_PHY_PORT_1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_2;

	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_3;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_2;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_1;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0;

	params->pwm_min = 6;
	params->pwm_default = 88;
	params->pwm_max = 255;
	params->camera_blk = 189;
	params->camera_dua_blk = 189;
	params->dim_cmd = 0x2c;
}

static void lcm_power_on(void) {
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;

	LCD_LOG("[DISP] %s: \n", __FUNCTION__);

	MDELAY(10);

	
        mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_01);
	SET_RESET_PIN(0);
	MDELAY(10);

#ifdef BUILD_LK
	mt6331_upmu_set_rg_vgp1_en(1);
	mt6331_upmu_set_rg_vcam_io_en(1);
#endif
	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(1);
	
	cmd = 0x00;
	data = 0x0f;
	tps65132_write_bytes(cmd, data);
	MDELAY(10);

	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(1);
	
	tps65132_write_bytes(0x3, 0x43);
	
	cmd = 0x01;
	data = 0x0f;
	tps65132_write_bytes(cmd, data);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
}

static void lcm_power_off(void) {

	LCD_LOG("[DISP] %s: \n", __FUNCTION__);

	SET_RESET_PIN(0);
	MDELAY(10);

	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

	
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

	
	mt_set_gpio_mode((GPIO62 | 0x80000000), GPIO_MODE_00);
	mt_set_gpio_dir((GPIO62 | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((GPIO62 | 0x80000000), GPIO_OUT_ZERO);
}

static void lcm_init_power(void)
{
	lcm_power_on();
}

static void lcm_resume_power(void)
{
	lcm_power_on();
}

static void lcm_suspend_power(void)
{
	lcm_power_off();
}

static void lcm_init(void)
{
	LCD_LOG("[DISP] %s: \n", __FUNCTION__);
	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	LCD_LOG("[DISP] %s: \n", __FUNCTION__);
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	LCD_LOG("[DISP] %s: \n", __FUNCTION__);
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y,
			unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

static unsigned int lcm_check_id(void)
{
	unsigned int retval = (which_lcd_module_triple() == 0) ? 1 : 0;

	
	return retval;
}

static void lcm_setbacklight_cmdq(void* handle,unsigned int level)
{
	LCD_LOG("%s, R63423 backlight: level = %d\n", __func__, level);
	

	unsigned int cmd = 0x51;
	unsigned int count =1;
	unsigned int value = level;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &value, 1);
}

static void lcm_set_lcm_cmd(void* handle,unsigned int *lcm_cmd,unsigned int *lcm_count,unsigned int *lcm_value)
{
	LCD_LOG("%s, lcm cmd: \n", __func__);

	unsigned int cmd = lcm_cmd[0];
	unsigned int count = lcm_count[0];
	unsigned int *ppara = lcm_value;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, ppara, 1);
}

LCM_DRIVER r63315_fhd_tianma_lcm_drv =
{
	.name		= "r63315_fhd_tianma",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
	.check_id	= lcm_check_id,
	.init_power	= lcm_init_power,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_cmdq  = lcm_setbacklight_cmdq,
	.set_lcm_cmd    = lcm_set_lcm_cmd,
#if (LCM_DSI_CMD_MODE)
	.update		= lcm_update,
#endif
};
