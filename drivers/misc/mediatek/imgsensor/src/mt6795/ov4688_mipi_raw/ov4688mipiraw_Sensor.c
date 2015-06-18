
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov4688mipiraw_Sensor.h"
#include "ov4688_otp.h"
#define PFX "OV4688_camera_sensor"
#define LOG_1 LOG_INF("OV4688,MIPI 4LANE\n")

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV4688MIPI_SENSOR_ID,        

    .checksum_value = 0xf7375923,        

    .pre = {
        .pclk = 120000000,
        .linelength = 0x67c,                
        .framelength = 0x700,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 0xa80,        
        .grabwindow_height = 0x5f0,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 400,
    },
    .cap = {
        .pclk = 120000000,
        .linelength = 0x67c,                
        .framelength = 0x700,            
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0xa80,
        .grabwindow_height = 0x5f0,
        .mipi_data_lp2hs_settle_dc = 90,
        .max_framerate = 400,
    },
    .cap1 = {                            
        .pclk = 120000000,
        .linelength = 0x67c,                
        .framelength = 0x700,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 0xa80,        
        .grabwindow_height = 0x5f0,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 400,
        },
    .normal_video = {
        .pclk = 120000000,
        .linelength = 2576,                
        .framelength = 1554,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 2688,        
        .grabwindow_height = 1520,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 400,
        },
    .hs_video = {
        .pclk = 120000000,
        .linelength = 2576,                
        .framelength = 1554,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 2688,        
        .grabwindow_height = 1520,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 400,
        },
    .slim_video = {
        .pclk = 120000000,
        .linelength = 2576,                
        .framelength = 1554,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 2688,        
        .grabwindow_height = 1520,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 400,
        },
         .custom1 = {
          
         .pclk = 120000000,
        .linelength = 0x67c,                
        .framelength = 0x700,            
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0xa80,
        .grabwindow_height = 0x5f0,
        .mipi_data_lp2hs_settle_dc = 90,
        .max_framerate = 400,
        },
        .custom3 = {
          
        .pclk = 120000000,
        .linelength = 0xae0,                
        .framelength = 0x700,            
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 0xa80,        
        .grabwindow_height = 0x5f0,       
        
        .mipi_data_lp2hs_settle_dc = 90,
        
        .max_framerate = 240,
        },
        
    .margin = 4,            
    .min_shutter = 1,        
    .max_frame_length = 0x7fff,
    .ae_shut_delay_frame = 0,    
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,      
    .ihdr_le_firstline = 0,  
    .sensor_mode_num = 8,      

    .cap_delay_frame = 3,        
    .pre_delay_frame = 2,         
    .video_delay_frame = 2,        
    .hs_video_delay_frame = 2,    
    .slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .custom3_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_8MA, 
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, 
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = { 0x20,0xff}, 
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                
    .sensor_mode = IMGSENSOR_MODE_INIT, 
    .shutter = 0x600,                    
    .gain = 0x80,                        
    .dummy_pixel = 0,                    
    .dummy_line = 0,                    
    .current_fps = 300,  
    .autoflicker_en = KAL_FALSE,  
    .test_pattern = KAL_FALSE,        
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0, 
    .i2c_write_id = 0x20,
};


static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] =
{
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
	 { 2688, 1520,      0,              0,            2688, 1520,   2688, 1520,   0, 0,    2688, 1520,      0,    0, 2688, 1520}, 
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy()
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	
    
    write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);

}    

static kal_uint32 return_sensor_id()
{
    return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    

    LOG_INF("framerate = %d, min framelength should enable = %d \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
	    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	    {
	        imgsensor.frame_length = imgsensor_info.max_frame_length;
	        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	    }
	    if (min_framelength_en)
	        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}    


static void set_shutter(kal_uint16 shutter)
{
    LOG_INF("enter set_shutter, shutter =%d\n", shutter);

    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    
    
	if(!shutter) shutter = 1; 
    spin_lock(&imgsensor_drv_lock);
	    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
	        imgsensor.frame_length = shutter + imgsensor_info.margin;
	    else
	        imgsensor.frame_length = imgsensor.min_frame_length;
		
	    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
	
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) 
	{
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else 
		{
		
		        LOG_INF("a. OV4688 set frame_length,  imgsensor.frame_length =%d\n",  imgsensor.frame_length);
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} 
	else 
	{
		
		
		LOG_INF("b. OV4688 set frame_length,  imgsensor.frame_length =%d\n",  imgsensor.frame_length);
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

		int temp;
		shutter = shutter & 0xffff;
		temp = shutter & 0x000f;
		temp = temp<<4;
		write_cmos_sensor(0x3502, temp);
		
		temp = shutter & 0x0fff;
		temp = temp>>4;
		write_cmos_sensor(0x3501, temp);
		
		temp = shutter>>12;
		write_cmos_sensor(0x3500, temp);

    
#if 0
		write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	
#endif
    LOG_INF("Exit!JEFF shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    



static kal_uint16 set_gain(kal_uint16 gain)
{
	
	LOG_INF("Jeff: enter set_gain,from hal  gain (%d) \n ", gain );

	kal_uint16 reg_gain_h;
	kal_uint16 reg_gain_l;
	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain >= 16 * BASEGAIN)
	{
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;		 
	}

	reg_gain = gain << 1;
	LOG_INF("Jeff: enter set_gain,to reg gain (%d) \n ", reg_gain );

	 
	reg_gain =  reg_gain & 0x7FF;
	if(reg_gain < 256) 
	{
		reg_gain_h = 0;
		reg_gain_l = reg_gain;
	}
	else if(reg_gain < 512) 
	{
		reg_gain_h = 1;
		reg_gain_l = (reg_gain >> 1) - 8;
	}
	else if(reg_gain < 1024) 
	{
		reg_gain_h = 3;
		reg_gain_l = (reg_gain >> 2) - 12;
	}
	else 
	{
		reg_gain_h = 7;
		reg_gain_l = (reg_gain >> 3) - 8;
	}
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("reg_gain = %d , reg_gain_h = 0x%x,reg_gain_l = 0x%x,\n ", reg_gain, reg_gain_l, reg_gain_h);

	write_cmos_sensor(0x3508, reg_gain_h);
	write_cmos_sensor(0x3509, reg_gain_l);

	return gain;
}    

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);


	switch (image_mirror) 
	{
		case IMAGE_NORMAL:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
			write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

static void night_mode(kal_bool enable)
{
}    

static void sensor_init(void)
{
LOG_INF("Enter ov4688 sensor_init.\n");

write_cmos_sensor(0x0103, 0x01);     

 
write_cmos_sensor(0x3638, 0x00);
write_cmos_sensor(0x0300, 0x00);
write_cmos_sensor(0x0302, 0x2f);
write_cmos_sensor(0x0303, 0x01);
write_cmos_sensor(0x0304, 0x03);
write_cmos_sensor(0x0305, 0x03);
write_cmos_sensor(0x0306, 0x01);
write_cmos_sensor(0x030a, 0x00);
write_cmos_sensor(0x030b, 0x00);
write_cmos_sensor(0x030d, 0x1e);
write_cmos_sensor(0x030e, 0x04);
write_cmos_sensor(0x030f, 0x01);
write_cmos_sensor(0x0312, 0x01);
write_cmos_sensor(0x031e, 0x00);
write_cmos_sensor(0x3000, 0x20);
write_cmos_sensor(0x3002, 0x00);
write_cmos_sensor(0x3018, 0x72);
write_cmos_sensor(0x3020, 0x93);
write_cmos_sensor(0x3021, 0x03);
write_cmos_sensor(0x3022, 0x01);
write_cmos_sensor(0x3031, 0x0a);
write_cmos_sensor(0x3305, 0xf1);
write_cmos_sensor(0x3307, 0x04);
write_cmos_sensor(0x3309, 0x29);
write_cmos_sensor(0x3500, 0x00);
write_cmos_sensor(0x3501, 0x60);
write_cmos_sensor(0x3502, 0x00);
write_cmos_sensor(0x3503, 0x04);
write_cmos_sensor(0x3504, 0x00);
write_cmos_sensor(0x3505, 0x00);
write_cmos_sensor(0x3506, 0x00);
write_cmos_sensor(0x3507, 0x00);
write_cmos_sensor(0x3508, 0x00);
write_cmos_sensor(0x3509, 0x80);
write_cmos_sensor(0x350a, 0x00);
write_cmos_sensor(0x350b, 0x00);
write_cmos_sensor(0x350c, 0x00);
write_cmos_sensor(0x350d, 0x00);
write_cmos_sensor(0x350e, 0x00);
write_cmos_sensor(0x350f, 0x80);
write_cmos_sensor(0x3510, 0x00);
write_cmos_sensor(0x3511, 0x00);
write_cmos_sensor(0x3512, 0x00);
write_cmos_sensor(0x3513, 0x00);
write_cmos_sensor(0x3514, 0x00);
write_cmos_sensor(0x3515, 0x80);
write_cmos_sensor(0x3516, 0x00);
write_cmos_sensor(0x3517, 0x00);
write_cmos_sensor(0x3518, 0x00);
write_cmos_sensor(0x3519, 0x00);
write_cmos_sensor(0x351a, 0x00);
write_cmos_sensor(0x351b, 0x80);
write_cmos_sensor(0x351c, 0x00);
write_cmos_sensor(0x351d, 0x00);
write_cmos_sensor(0x351e, 0x00);
write_cmos_sensor(0x351f, 0x00);
write_cmos_sensor(0x3520, 0x00);
write_cmos_sensor(0x3521, 0x80);
write_cmos_sensor(0x3522, 0x08);
write_cmos_sensor(0x3524, 0x08);
write_cmos_sensor(0x3526, 0x08);
write_cmos_sensor(0x3528, 0x08);
write_cmos_sensor(0x352a, 0x08);
write_cmos_sensor(0x3602, 0x00);
write_cmos_sensor(0x3604, 0x02);
write_cmos_sensor(0x3605, 0x00);
write_cmos_sensor(0x3606, 0x00);
write_cmos_sensor(0x3607, 0x00);
write_cmos_sensor(0x3609, 0x12);
write_cmos_sensor(0x360a, 0x40);
write_cmos_sensor(0x360c, 0x08);
write_cmos_sensor(0x360f, 0xe5);
write_cmos_sensor(0x3608, 0x8f);
write_cmos_sensor(0x3611, 0x00);
write_cmos_sensor(0x3613, 0xf7);
write_cmos_sensor(0x3616, 0x58);
write_cmos_sensor(0x3619, 0x99);
write_cmos_sensor(0x361b, 0x60);
write_cmos_sensor(0x361c, 0x7a);
write_cmos_sensor(0x361e, 0x79);
write_cmos_sensor(0x361f, 0x02);
write_cmos_sensor(0x3632, 0x00);
write_cmos_sensor(0x3633, 0x10);
write_cmos_sensor(0x3634, 0x10);
write_cmos_sensor(0x3635, 0x10);
write_cmos_sensor(0x3636, 0x15);
write_cmos_sensor(0x3646, 0x86);
write_cmos_sensor(0x364a, 0x0b);
write_cmos_sensor(0x3700, 0x17);
write_cmos_sensor(0x3701, 0x22);
write_cmos_sensor(0x3703, 0x10);
write_cmos_sensor(0x370a, 0x37);
write_cmos_sensor(0x3705, 0x00);
write_cmos_sensor(0x3706, 0x63);
write_cmos_sensor(0x3709, 0x3c);
write_cmos_sensor(0x370b, 0x01);
write_cmos_sensor(0x370c, 0x30);
write_cmos_sensor(0x3710, 0x24);
write_cmos_sensor(0x3711, 0x0c);
write_cmos_sensor(0x3716, 0x00);
write_cmos_sensor(0x3720, 0x28);
write_cmos_sensor(0x3729, 0x7b);
write_cmos_sensor(0x372a, 0x84);
write_cmos_sensor(0x372b, 0xbd);
write_cmos_sensor(0x372c, 0xbc);
write_cmos_sensor(0x372e, 0x52);
write_cmos_sensor(0x373c, 0x0e);
write_cmos_sensor(0x373e, 0x33);
write_cmos_sensor(0x3743, 0x10);
write_cmos_sensor(0x3744, 0x88);
write_cmos_sensor(0x374a, 0x43);
write_cmos_sensor(0x374c, 0x00);
write_cmos_sensor(0x374e, 0x23);
write_cmos_sensor(0x3751, 0x7b);
write_cmos_sensor(0x3752, 0x84);
write_cmos_sensor(0x3753, 0xbd);
write_cmos_sensor(0x3754, 0xbc);
write_cmos_sensor(0x3756, 0x52);
write_cmos_sensor(0x375c, 0x00);
write_cmos_sensor(0x3760, 0x00);
write_cmos_sensor(0x3761, 0x00);
write_cmos_sensor(0x3762, 0x00);
write_cmos_sensor(0x3763, 0x00);
write_cmos_sensor(0x3764, 0x00);
write_cmos_sensor(0x3767, 0x04);
write_cmos_sensor(0x3768, 0x04);
write_cmos_sensor(0x3769, 0x08);
write_cmos_sensor(0x376a, 0x08);
write_cmos_sensor(0x376b, 0x20);
write_cmos_sensor(0x376c, 0x00);
write_cmos_sensor(0x376d, 0x00);
write_cmos_sensor(0x376e, 0x00);
write_cmos_sensor(0x3773, 0x00);
write_cmos_sensor(0x3774, 0x51);
write_cmos_sensor(0x3776, 0xbd);
write_cmos_sensor(0x3777, 0xbd);
write_cmos_sensor(0x3781, 0x18);
write_cmos_sensor(0x3783, 0x25);
write_cmos_sensor(0x3800, 0x00);
write_cmos_sensor(0x3801, 0x08);
write_cmos_sensor(0x3802, 0x00);
write_cmos_sensor(0x3803, 0x04);
write_cmos_sensor(0x3804, 0x0a);
write_cmos_sensor(0x3805, 0x97);
write_cmos_sensor(0x3806, 0x05);
write_cmos_sensor(0x3807, 0xfb);
write_cmos_sensor(0x3808, 0x0a);
write_cmos_sensor(0x3809, 0x80);
write_cmos_sensor(0x380a, 0x05);
write_cmos_sensor(0x380b, 0xf0);
write_cmos_sensor(0x380c, 0x0a);
write_cmos_sensor(0x380d, 0x0c);
write_cmos_sensor(0x380e, 0x06);
write_cmos_sensor(0x380f, 0x12);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x08);
write_cmos_sensor(0x3812, 0x00);
write_cmos_sensor(0x3813, 0x04);
write_cmos_sensor(0x3814, 0x01);
write_cmos_sensor(0x3815, 0x01);
write_cmos_sensor(0x3819, 0x01);
write_cmos_sensor(0x3820, 0x06);
write_cmos_sensor(0x3821, 0x00);
write_cmos_sensor(0x3829, 0x00);
write_cmos_sensor(0x382a, 0x01);
write_cmos_sensor(0x382b, 0x01);
write_cmos_sensor(0x382d, 0x7f);
write_cmos_sensor(0x3830, 0x04);
write_cmos_sensor(0x3836, 0x01);
write_cmos_sensor(0x3841, 0x02);
write_cmos_sensor(0x3846, 0x08);
write_cmos_sensor(0x3847, 0x07);
write_cmos_sensor(0x3d85, 0x36);
write_cmos_sensor(0x3d8c, 0x71);
write_cmos_sensor(0x3d8d, 0xcb);
write_cmos_sensor(0x3f0a, 0x00);
write_cmos_sensor(0x4000, 0x71);
write_cmos_sensor(0x4001, 0x40);
write_cmos_sensor(0x4002, 0x04);
write_cmos_sensor(0x4003, 0x14);
write_cmos_sensor(0x400e, 0x00);
write_cmos_sensor(0x4011, 0x00);
write_cmos_sensor(0x401a, 0x00);
write_cmos_sensor(0x401b, 0x00);
write_cmos_sensor(0x401c, 0x00);
write_cmos_sensor(0x401d, 0x00);
write_cmos_sensor(0x401f, 0x00);
write_cmos_sensor(0x4020, 0x00);
write_cmos_sensor(0x4021, 0x10);
write_cmos_sensor(0x4022, 0x07);
write_cmos_sensor(0x4023, 0xcf);
write_cmos_sensor(0x4024, 0x09);
write_cmos_sensor(0x4025, 0x60);
write_cmos_sensor(0x4026, 0x09);
write_cmos_sensor(0x4027, 0x6f);
write_cmos_sensor(0x4028, 0x00);
write_cmos_sensor(0x4029, 0x02);
write_cmos_sensor(0x402a, 0x06);
write_cmos_sensor(0x402b, 0x04);
write_cmos_sensor(0x402c, 0x02);
write_cmos_sensor(0x402d, 0x02);
write_cmos_sensor(0x402e, 0x0e);
write_cmos_sensor(0x402f, 0x04);
write_cmos_sensor(0x4302, 0xff);
write_cmos_sensor(0x4303, 0xff);
write_cmos_sensor(0x4304, 0x00);
write_cmos_sensor(0x4305, 0x00);
write_cmos_sensor(0x4306, 0x00);
write_cmos_sensor(0x4308, 0x02);
write_cmos_sensor(0x4500, 0x6c);
write_cmos_sensor(0x4501, 0xc4);
write_cmos_sensor(0x4502, 0x40);
write_cmos_sensor(0x4503, 0x02);
write_cmos_sensor(0x4601, 0xa7);
write_cmos_sensor(0x4800, 0x04);
write_cmos_sensor(0x4813, 0x08);
write_cmos_sensor(0x481f, 0x40);
write_cmos_sensor(0x4829, 0x78);
write_cmos_sensor(0x4837, 0x1c);
write_cmos_sensor(0x4b00, 0x2a);
write_cmos_sensor(0x4b0d, 0x00);
write_cmos_sensor(0x4d00, 0x04);
write_cmos_sensor(0x4d01, 0x42);
write_cmos_sensor(0x4d02, 0xd1);
write_cmos_sensor(0x4d03, 0x93);
write_cmos_sensor(0x4d04, 0xf5);
write_cmos_sensor(0x4d05, 0xc1);
write_cmos_sensor(0x5000, 0xf3);
write_cmos_sensor(0x5001, 0x11);
write_cmos_sensor(0x5004, 0x00);
write_cmos_sensor(0x500a, 0x00);
write_cmos_sensor(0x500b, 0x00);
write_cmos_sensor(0x5032, 0x00);
write_cmos_sensor(0x5040, 0x00);
write_cmos_sensor(0x5050, 0x0c);
write_cmos_sensor(0x5500, 0x00);
write_cmos_sensor(0x5501, 0x10);
write_cmos_sensor(0x5502, 0x01);
write_cmos_sensor(0x5503, 0x0f);
write_cmos_sensor(0x8000, 0x00);
write_cmos_sensor(0x8001, 0x00);
write_cmos_sensor(0x8002, 0x00);
write_cmos_sensor(0x8003, 0x00);
write_cmos_sensor(0x8004, 0x00);
write_cmos_sensor(0x8005, 0x00);
write_cmos_sensor(0x8006, 0x00);
write_cmos_sensor(0x8007, 0x00);
write_cmos_sensor(0x8008, 0x00);
write_cmos_sensor(0x3638, 0x00);
write_cmos_sensor(0x3105, 0x31);
write_cmos_sensor(0x301a, 0xf9);
write_cmos_sensor(0x3508, 0x07);
write_cmos_sensor(0x484b, 0x05);
write_cmos_sensor(0x4805, 0x03);
write_cmos_sensor(0x3601, 0x01);
write_cmos_sensor(0x0100,0x01);   
mDELAY(10);
write_cmos_sensor(0x3105, 0x11);
write_cmos_sensor(0x301a, 0xf1);
write_cmos_sensor(0x4805, 0x00);
write_cmos_sensor(0x301a, 0xf0);
write_cmos_sensor(0x3208, 0x00);
write_cmos_sensor(0x302a, 0x00);
write_cmos_sensor(0x302a, 0x00);
write_cmos_sensor(0x302a, 0x00);
write_cmos_sensor(0x302a, 0x00);
write_cmos_sensor(0x302a, 0x00);
write_cmos_sensor(0x3601, 0x00);
write_cmos_sensor(0x3638, 0x00);
write_cmos_sensor(0x3208, 0x10);
write_cmos_sensor(0x3208, 0xa0);

}    


static void preview_setting(void)
{
LOG_INF("OV4688 Load preview_setting\n");
write_cmos_sensor(0x0100,0x00);    
mDELAY(66);
write_cmos_sensor(0x301a,0xf9);
write_cmos_sensor(0x4805,0x03);

write_cmos_sensor(0x0300, 0x00);
write_cmos_sensor(0x0302, 0x2f);
write_cmos_sensor(0x0303, 0x01);
write_cmos_sensor(0x0304, 0x03);
write_cmos_sensor(0x0305, 0x03);
write_cmos_sensor(0x0306, 0x01);
write_cmos_sensor(0x030a, 0x00);
write_cmos_sensor(0x030b, 0x00);
write_cmos_sensor(0x030d, 0x1e);
write_cmos_sensor(0x030e, 0x04);
write_cmos_sensor(0x030f, 0x01);
write_cmos_sensor(0x0312, 0x01);
write_cmos_sensor(0x031e, 0x00);

write_cmos_sensor(0x3632, 0x00);
write_cmos_sensor(0x376b, 0x20);
write_cmos_sensor(0x3841, 0x02);
write_cmos_sensor(0x3846, 0x08);
write_cmos_sensor(0x3847, 0x07);
write_cmos_sensor(0x4800, 0x04);
write_cmos_sensor(0x4837, 0x1c);
write_cmos_sensor(0x376e, 0x00);
write_cmos_sensor(0x3800, 0x00);
write_cmos_sensor(0x3801, 0x08);
write_cmos_sensor(0x3802, 0x00);
write_cmos_sensor(0x3803, 0x04);
write_cmos_sensor(0x3804, 0x0a);
write_cmos_sensor(0x3805, 0x97);
write_cmos_sensor(0x3806, 0x05);
write_cmos_sensor(0x3807, 0xfb);
write_cmos_sensor(0x3808, 0x0a);
write_cmos_sensor(0x3809, 0x80);
write_cmos_sensor(0x380a, 0x05);
write_cmos_sensor(0x380b, 0xf0);
write_cmos_sensor(0x380c, 0x06);
write_cmos_sensor(0x380d, 0x7c);
write_cmos_sensor(0x380e, 0x07);
write_cmos_sensor(0x380f, 0x00);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x08);
write_cmos_sensor(0x3812, 0x00);
write_cmos_sensor(0x3813, 0x04);
write_cmos_sensor(0x3814, 0x01);
write_cmos_sensor(0x3815, 0x01);
write_cmos_sensor(0x3819, 0x01);
write_cmos_sensor(0x3820, 0x06);
write_cmos_sensor(0x3821, 0x00);
write_cmos_sensor(0x3829, 0x00);
write_cmos_sensor(0x382a, 0x01);
write_cmos_sensor(0x382b, 0x01);
write_cmos_sensor(0x382d, 0x7f);
write_cmos_sensor(0x3830, 0x04);
write_cmos_sensor(0x3836, 0x01);
write_cmos_sensor(0x4001, 0x40);
write_cmos_sensor(0x4020, 0x00);
write_cmos_sensor(0x4021, 0x10);
write_cmos_sensor(0x4022, 0x07);
write_cmos_sensor(0x4023, 0xcf);
write_cmos_sensor(0x4024, 0x09);
write_cmos_sensor(0x4025, 0x60);
write_cmos_sensor(0x4026, 0x09);
write_cmos_sensor(0x4027, 0x6f);
write_cmos_sensor(0x4502, 0x40);
write_cmos_sensor(0x4600, 0x00);
write_cmos_sensor(0x4601, 0xa7);
write_cmos_sensor(0x5050, 0x0c);

write_cmos_sensor(0x0100,0x01);   
mDELAY(10);
write_cmos_sensor(0x301a, 0xf9);
write_cmos_sensor(0x301a, 0xf1);
write_cmos_sensor(0x4805, 0x00);
write_cmos_sensor(0x301a, 0xf0);

mDELAY(40);
}    


static void preview_24fps_setting(void)
{
LOG_INF("OV4688 Load preview_24fps_setting\n");
write_cmos_sensor(0x0100,0x00);    
mDELAY(66);
write_cmos_sensor(0x301a,0xf9);
write_cmos_sensor(0x4805,0x03);

write_cmos_sensor(0x0300, 0x00);
write_cmos_sensor(0x0302, 0x1c);
write_cmos_sensor(0x0303, 0x01);
write_cmos_sensor(0x0304, 0x03);
write_cmos_sensor(0x0305, 0x03);
write_cmos_sensor(0x0306, 0x01);
write_cmos_sensor(0x030a, 0x00);
write_cmos_sensor(0x030b, 0x00);
write_cmos_sensor(0x030d, 0x1e);
write_cmos_sensor(0x030e, 0x04);
write_cmos_sensor(0x030f, 0x01);
write_cmos_sensor(0x0312, 0x01);
write_cmos_sensor(0x031e, 0x00);

write_cmos_sensor(0x3632, 0x00);
write_cmos_sensor(0x376b, 0x20);
write_cmos_sensor(0x3841, 0x02);
write_cmos_sensor(0x3846, 0x08);
write_cmos_sensor(0x3847, 0x07);
write_cmos_sensor(0x4800, 0x04);
write_cmos_sensor(0x4837, 0x30);
write_cmos_sensor(0x376e, 0x00);
write_cmos_sensor(0x3800, 0x00);
write_cmos_sensor(0x3801, 0x08);
write_cmos_sensor(0x3802, 0x00);
write_cmos_sensor(0x3803, 0x04);
write_cmos_sensor(0x3804, 0x0a);
write_cmos_sensor(0x3805, 0x97);
write_cmos_sensor(0x3806, 0x05);
write_cmos_sensor(0x3807, 0xfb);
write_cmos_sensor(0x3808, 0x0a);
write_cmos_sensor(0x3809, 0x80);
write_cmos_sensor(0x380a, 0x05);
write_cmos_sensor(0x380b, 0xf0);
write_cmos_sensor(0x380c, 0x0a);
write_cmos_sensor(0x380d, 0xe0);
write_cmos_sensor(0x380e, 0x07);
write_cmos_sensor(0x380f, 0x00);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x08);
write_cmos_sensor(0x3812, 0x00);
write_cmos_sensor(0x3813, 0x04);
write_cmos_sensor(0x3814, 0x01);
write_cmos_sensor(0x3815, 0x01);
write_cmos_sensor(0x3819, 0x01);
write_cmos_sensor(0x3820, 0x06);
write_cmos_sensor(0x3821, 0x00);
write_cmos_sensor(0x3829, 0x00);
write_cmos_sensor(0x382a, 0x01);
write_cmos_sensor(0x382b, 0x01);
write_cmos_sensor(0x382d, 0x7f);
write_cmos_sensor(0x3830, 0x04);
write_cmos_sensor(0x3836, 0x01);
write_cmos_sensor(0x4001, 0x40);
write_cmos_sensor(0x4020, 0x00);
write_cmos_sensor(0x4021, 0x10);
write_cmos_sensor(0x4022, 0x07);
write_cmos_sensor(0x4023, 0xcf);
write_cmos_sensor(0x4024, 0x09);
write_cmos_sensor(0x4025, 0x60);
write_cmos_sensor(0x4026, 0x09);
write_cmos_sensor(0x4027, 0x6f);
write_cmos_sensor(0x4502, 0x40);
write_cmos_sensor(0x4600, 0x00);
write_cmos_sensor(0x4601, 0xa7);
write_cmos_sensor(0x5050, 0x0c);

write_cmos_sensor(0x0100,0x01);   
mDELAY(10);
write_cmos_sensor(0x301a, 0xf9);
write_cmos_sensor(0x301a, 0xf1);
write_cmos_sensor(0x4805, 0x00);
write_cmos_sensor(0x301a, 0xf0);

mDELAY(40);
}    

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("OV4688 Load Capture_setting\n");
#if 0
	if (currefps == 240) 
	{   
	    
	write_cmos_sensor(0x0100,0x00);    


	} 
	else 
	{   
		write_cmos_sensor(0x0100,0x00);    
		
        
	
	if (imgsensor.ihdr_en)
	{

	} 
	else 
	{	

	}

        write_cmos_sensor(0x0100,0x01);  

    }
#endif
	write_cmos_sensor(0x0100,0x00);    
	mDELAY(66);
	write_cmos_sensor(0x301a,0xf9);
	write_cmos_sensor(0x4805,0x03);
	
write_cmos_sensor(0x0300, 0x00);
write_cmos_sensor(0x0302, 0x2f);
write_cmos_sensor(0x0303, 0x01);
write_cmos_sensor(0x0304, 0x03);
write_cmos_sensor(0x0305, 0x03);
write_cmos_sensor(0x0306, 0x01);
write_cmos_sensor(0x030a, 0x00);
write_cmos_sensor(0x030b, 0x00);
write_cmos_sensor(0x030d, 0x1e);
write_cmos_sensor(0x030e, 0x04);
write_cmos_sensor(0x030f, 0x01);
write_cmos_sensor(0x0312, 0x01);
write_cmos_sensor(0x031e, 0x00);

write_cmos_sensor(0x3632, 0x00);
write_cmos_sensor(0x376b, 0x20);
write_cmos_sensor(0x3841, 0x02);
write_cmos_sensor(0x3846, 0x08);
write_cmos_sensor(0x3847, 0x07);
write_cmos_sensor(0x4800, 0x04);
write_cmos_sensor(0x4837, 0x1c);
write_cmos_sensor(0x376e, 0x00);
write_cmos_sensor(0x3800, 0x00);
write_cmos_sensor(0x3801, 0x08);
write_cmos_sensor(0x3802, 0x00);
write_cmos_sensor(0x3803, 0x04);
write_cmos_sensor(0x3804, 0x0a);
write_cmos_sensor(0x3805, 0x97);
write_cmos_sensor(0x3806, 0x05);
write_cmos_sensor(0x3807, 0xfb);
write_cmos_sensor(0x3808, 0x0a);
write_cmos_sensor(0x3809, 0x80);
write_cmos_sensor(0x380a, 0x05);
write_cmos_sensor(0x380b, 0xf0);
write_cmos_sensor(0x380c, 0x06);
write_cmos_sensor(0x380d, 0x7c);
write_cmos_sensor(0x380e, 0x07);
write_cmos_sensor(0x380f, 0x00);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x08);
write_cmos_sensor(0x3812, 0x00);
write_cmos_sensor(0x3813, 0x04);
write_cmos_sensor(0x3814, 0x01);
write_cmos_sensor(0x3815, 0x01);
write_cmos_sensor(0x3819, 0x01);
write_cmos_sensor(0x3820, 0x06);
write_cmos_sensor(0x3821, 0x00);
write_cmos_sensor(0x3829, 0x00);
write_cmos_sensor(0x382a, 0x01);
write_cmos_sensor(0x382b, 0x01);
write_cmos_sensor(0x382d, 0x7f);
write_cmos_sensor(0x3830, 0x04);
write_cmos_sensor(0x3836, 0x01);
write_cmos_sensor(0x4001, 0x40);
write_cmos_sensor(0x4020, 0x00);
write_cmos_sensor(0x4021, 0x10);
write_cmos_sensor(0x4022, 0x07);
write_cmos_sensor(0x4023, 0xcf);
write_cmos_sensor(0x4024, 0x09);
write_cmos_sensor(0x4025, 0x60);
write_cmos_sensor(0x4026, 0x09);
write_cmos_sensor(0x4027, 0x6f);
write_cmos_sensor(0x4502, 0x40);
write_cmos_sensor(0x4600, 0x00);
write_cmos_sensor(0x4601, 0xa7);
write_cmos_sensor(0x5050, 0x0c);

write_cmos_sensor(0x0100,0x01);   
mDELAY(10);
write_cmos_sensor(0x301a, 0xf9);
write_cmos_sensor(0x301a, 0xf1);
write_cmos_sensor(0x4805, 0x00);
write_cmos_sensor(0x301a, 0xf0);
mDELAY(40);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
	LOG_INF("OV4688 Load normal_video_setting\n");
	write_cmos_sensor(0x0100,0x00);    
	mDELAY(100);

         
	write_cmos_sensor(0x0302, 0x1c);      
	write_cmos_sensor(0x3501, 0x60);      
	write_cmos_sensor(0x3632, 0x00);      
	write_cmos_sensor(0x376b, 0x20);      
	write_cmos_sensor(0x3800, 0x00);      
	write_cmos_sensor(0x3801, 0x08);      
	write_cmos_sensor(0x3802, 0x00);      
	write_cmos_sensor(0x3803, 0x04);      
	write_cmos_sensor(0x3804, 0x0a);      
	write_cmos_sensor(0x3805, 0x97);      
	write_cmos_sensor(0x3806, 0x05);      
	write_cmos_sensor(0x3807, 0xfb);;      
	write_cmos_sensor(0x3808, 0x0a);      
	write_cmos_sensor(0x3809, 0x80);      
	write_cmos_sensor(0x380a, 0x05);      
	write_cmos_sensor(0x380b, 0xf0);;      
	write_cmos_sensor(0x380c, 0x0a);      
	write_cmos_sensor(0x380d, 0x10);      
	write_cmos_sensor(0x380e, 0x06);      
	write_cmos_sensor(0x380f , 0x12);     
	write_cmos_sensor(0x3811, 0x08);      
	write_cmos_sensor(0x3813, 0x04);      
	write_cmos_sensor(0x3814, 0x01);      
	write_cmos_sensor(0x3820, 0x06);      
	write_cmos_sensor(0x3821, 0x00);      
	write_cmos_sensor(0x382a, 0x01);      
	write_cmos_sensor(0x3830, 0x04);      
	write_cmos_sensor(0x3836, 0x01);      
	write_cmos_sensor(0x4001, 0x40);      
	write_cmos_sensor(0x4022, 0x07);      
	write_cmos_sensor(0x4023, 0xcf );      
	write_cmos_sensor(0x4024, 0x09);      
	write_cmos_sensor(0x4025, 0x60);      
	write_cmos_sensor(0x4026, 0x09);      
	write_cmos_sensor(0x4027, 0x6f );      
	write_cmos_sensor(0x4502, 0x40);      
	write_cmos_sensor(0x4601, 0x04);      
	write_cmos_sensor(0x4837, 0x18);      
   
	write_cmos_sensor(0x0100,0x01);   
	mDELAY(40);
}
static void hs_video_setting()
{
    LOG_INF("E\n");
	LOG_INF("OV4688 Load HS_video_setting\n");
	write_cmos_sensor(0x0100,0x00);    
	
	write_cmos_sensor(0x0100,0x01);  

}

static void slim_video_setting()
{
    LOG_INF("E\n");
    LOG_INF("OV4688 Load Slim_video_setting\n");
	write_cmos_sensor(0x0100,0x00);    

	    
	write_cmos_sensor(0x0100,0x01);  
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	


	if (enable) 
	{      
		write_cmos_sensor(0x5040, 0x80);
	} 
	else 
	{
		write_cmos_sensor(0x5040, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static const char *ov4688Vendor = "OmniVision";
static const char *ov4688NAME = "ov4688_front";
static const char *ov4688Size = "4.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov4688Vendor, ov4688NAME, ov4688Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static struct kobject *android_ov4688;
static int first = true;

static int ov4688_sysfs_init(void)
{
	int ret ;
	if(first){
		LOG_INF("kobject creat and add\n");
		android_ov4688 = kobject_create_and_add("android_camera2", NULL);
		if (android_ov4688 == NULL) {
			LOG_INF("subsystem_register " \
			"failed\n");
			ret = -ENOMEM;
			return ret ;
		}
		LOG_INF("sysfs_create_file\n");
		ret = sysfs_create_file(android_ov4688, &dev_attr_sensor.attr);
		if (ret) {
			LOG_INF("sysfs_create_file " \
			"failed\n");
			kobject_del(android_ov4688);
		}else
			first = false;
	}

	return 0 ;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                ov4688_sysfs_init();
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    
    sensor_init();
#if 0	
	ov4688_otp_cali(imgsensor_info.sensor_id);
#endif

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    



static kal_uint32 close(void)
{
    LOG_INF("E\n");

    

    return ERROR_NONE;
}    


static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
	    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	    imgsensor.pclk = imgsensor_info.pre.pclk;
	    
	    imgsensor.line_length = imgsensor_info.pre.linelength;
	    imgsensor.frame_length = imgsensor_info.pre.framelength;
	    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}    

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) 
	{
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 
	else 
	{
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	
	return ERROR_NONE;
	}    
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    return ERROR_NONE;
}   

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
     preview_24fps_setting();
    return ERROR_NONE;
}   

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
	    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	    imgsensor.pclk = imgsensor_info.normal_video.pclk;
	    imgsensor.line_length = imgsensor_info.normal_video.linelength;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	    
	    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);	
	
    return ERROR_NONE;
}    

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
	    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	    imgsensor.pclk = imgsensor_info.hs_video.pclk;
	    
	    imgsensor.line_length = imgsensor_info.hs_video.linelength;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	    imgsensor.dummy_line = 0;
	    imgsensor.dummy_pixel = 0;
	    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	
    return ERROR_NONE;
}    

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
	    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	    imgsensor.pclk = imgsensor_info.slim_video.pclk;
	    imgsensor.line_length = imgsensor_info.slim_video.linelength;
	    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	    imgsensor.dummy_line = 0;
	    imgsensor.dummy_pixel = 0;
	    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	

    return ERROR_NONE;
}    



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");

    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
    
    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;
    
    return ERROR_NONE;
}    

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    
    
    

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; 
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; 
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; 
    sensor_info->SensorResetActiveHigh = FALSE; 
    sensor_info->SensorResetDelayCount = 5; 

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; 
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; 
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; 
    sensor_info->SensorPixelClockCount = 3; 
    sensor_info->SensorDataLatchCount = 2; 

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  
    sensor_info->SensorHightSampling = 0;    
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
          case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
          case MSDK_SCENARIO_ID_CUSTOM1:
            LOG_INF("ov4688_scenario_id = MSDK_SCENARIO_ID_CUSTOM1\n");
            Custom1(image_window, sensor_config_data); 
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            LOG_INF("ov4688_scenario_id = MSDK_SCENARIO_ID_CUSTOM3\n");
            Custom3(image_window, sensor_config_data); 
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    
    if (framerate == 0)
        
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) 
        imgsensor.autoflicker_en = KAL_TRUE;
    else 
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
         case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            framerate=240;
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            LOG_INF("ov4688 set framerate=240 ,  frame_length = 0x%x (should be  0x700 ).  \n", frame_length);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
        default:  
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
         case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            
            
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: 
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 OV4688_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    

