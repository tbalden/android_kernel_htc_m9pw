
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

#include "ov13850main_mipiraw_Sensor.h"

#define PFX "OV13850MAIN_camera_sensor"
#define LOG_1 LOG_INF("OV13850MAIN,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 4192*3104@30fps,1.2Gbps/lane; capture 13M@30fps,1.2Gbps/lane\n")

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

int ov13850main_chip_ver = OV13850main_R2A;
extern void otp_cali(unsigned char writeid);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV13850MAIN_SENSOR_ID,        

    .checksum_value = 0xbde6b5f8,

    .pre = {
        
        .pclk =  240000000, 
        .linelength = 0x12C0, 
        .framelength = 0x0680, 
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 2112,        
        .grabwindow_height = 1568,        
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 448000000, 
        .linelength = 0x11A0, 
        .framelength = 0xD00,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4224, 
        .grabwindow_height = 3136,
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 300,

    },
    .cap1 = {                            
        
        .pclk = 384000000, 
        .linelength = 0x12c0, 
        .framelength =0x0d06, 
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width =4224 ,        
        .grabwindow_height =3136 ,        
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 240,
    },
    .normal_video = {
        .pclk = 480000000,
        .linelength = 0x12C0,
        .framelength = 0xD00,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3840,
        .grabwindow_height = 2176,
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 240000000, 
        .linelength = 0x960,
        .framelength = 0x680,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2112,
        .grabwindow_height = 1188,
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 600,
    },
    .slim_video = {
        .pclk = 240000000, 
        .linelength = 0x0960, 
        .framelength = 0x0340, 
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0x500,  
        .grabwindow_height = 0x2d0,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 1200,

    },
      .custom1 = {
        
        .pclk = 448000000, 
        .linelength = 0x11A0, 
        .framelength = 0x0CEE, 
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0x1080,  
        .grabwindow_height = 0x0948,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 300,
      },
      .custom2= {
        
        .pclk = 384000000, 
        .linelength = 0x12c0, 
        .framelength =0x0d06, 
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width =4224 ,        
        .grabwindow_height =3136 ,        
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 240,
    },
      .custom3= {
        
        .pclk =  240000000, 
        .linelength =  0x1778, 
        .framelength = 0x0680, 
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width = 2112,        
        .grabwindow_height = 1568,        
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 240,
    },
    .margin = 8,            
    .min_shutter = 0x3,        
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
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_8MA, 
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, 
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x6c, 0xff},
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_V_MIRROR,                
    .sensor_mode = IMGSENSOR_MODE_INIT, 
    .shutter = 0x3D0,                    
    .gain = 0x100,                        
    .dummy_pixel = 0,                    
    .dummy_line = 0,                    
    .current_fps = 0,  
    .autoflicker_en = KAL_FALSE,  
    .test_pattern = KAL_FALSE,        
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0, 
    .i2c_write_id = 0x6c,
};


static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] =
{{ 4256, 3152, 0, 4, 4256, 3152, 2128, 1572, 8, 2, 2112, 1568, 0, 0, 2096, 1552}, 
 { 4256, 3152, 12, 4, 4232, 3144, 4232, 3144, 4, 4, 4224, 3136, 0, 0, 4192, 3104}, 
 {  4256, 3152, 20, 12,  4216, 3128, 4216, 3128, 4, 4, 4208, 3120, 0, 0, 4192, 3104}, 
 { 4256, 3152, 0, 376, 4256, 2392, 1064, 598, 8, 2, 1056, 594, 208, 56, 640, 480}, 
 { 4256, 3152, 824, 842, 2608, 1468, 1304, 734, 4, 2, 1296, 730, 0, 0, 1280, 720},
 { 4256, 3152, 12, 4, 4232, 3144, 4232, 3144, 4, 4, 4224, 3136, 0, 0, 4192, 3104}, 
 { 4256, 3152, 12, 4, 4232, 3144, 4232, 3144, 4, 4, 4224, 3136, 0, 0, 4192, 3104}, 
 { 4256, 3152, 0, 4, 4256, 3152, 2128, 1572, 8, 2, 2112, 1568, 0, 0, 2096, 1552}, 
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
    kal_uint32 cmos_id = 0;

    cmos_id = ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));

    if((cmos_id == 0xD850) && (mt_get_gpio_in(GPIO3) == 1))
    {
        LOG_INF(" ov13850MAIN : mt_get_gpio_in(GPIO3) == 1\n");
        cmos_id = 0xD851;
    }
    else
        cmos_id = 0x0000;
    
    return cmos_id ;
}

static int read_cmos_sensor_otp(kal_uint32 addr, u8* get_byte )
{
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    return iReadRegI2C(pu_send_cmd, 2, get_byte, 1, imgsensor.i2c_write_id);
}

static int write_cmos_sensor_otp(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    return iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

#define OV13850_LITEON_OTP_SIZE 21
char ov13850main_otp_data[OV13850_LITEON_OTP_SIZE];

static int read_ov13850main_otp(void){
    LOG_INF("==========ov13850main read_otp readed=======\n");

    const short addr[3][OV13850_LITEON_OTP_SIZE] = {
        
        {0x7220,0x7221,0x7222,0x7223,0x7224,0x7225,0x7226,0x7227,0x722D,0x7228,0x7229,0x722A,0x722B,0x722C,0x722E,0x724D,0x724E,0x724F,0x7250,0x7251,0x7252}, 
        {0x722F,0x7230,0x7231,0x7232,0x7233,0x7234,0x7235,0x7236,0x723C,0x7237,0x7238,0x7239,0x723A,0x723B,0x723D,0x7253,0x7254,0x7255,0x7256,0x7257,0x7258}, 
        {0x723E,0x723F,0x7240,0x7241,0x7242,0x7243,0x7244,0x7245,0x724B,0x7246,0x7247,0x7248,0x7249,0x724A,0x724C,0x7259,0x725A,0x725B,0x725C,0x725D,0x725E}, 
    };

    static int first= true;
    uint16_t read_data = 0;

    int32_t i=0;
    int32_t page=0;
    int32_t ret = 0;
    const int32_t offset = 0x00;
    static int32_t valid_layer=-1;

    if (first) {
		first = false;

		write_cmos_sensor_otp(0x0103,0x01);
		write_cmos_sensor_otp(0x0100,0x00);
		write_cmos_sensor_otp(0x3611,0x10);
		write_cmos_sensor_otp(0x3612,0x07);
		write_cmos_sensor_otp(0x3613,0x33);
		write_cmos_sensor_otp(0x3d84,0x00);
		write_cmos_sensor_otp(0x3d85,0x17);
		write_cmos_sensor_otp(0x3d8c,0x73);
		write_cmos_sensor_otp(0x3d8d,0xbf);
		write_cmos_sensor_otp(0x0100,0x01);
		write_cmos_sensor_otp(0x5002,0x05);
		write_cmos_sensor_otp(0x3d84,0x40);
		write_cmos_sensor_otp(0x3d88,0x7000);
		write_cmos_sensor_otp(0x3d8a,0x73ff);
		write_cmos_sensor_otp(0x3d81,0x01);

		msleep(10);

		
		for (page=2; page>=0; page--) {
			for (i=0; i<OV13850_LITEON_OTP_SIZE; i++) {
				ret = read_cmos_sensor_otp(addr[page][i]+offset, &read_data);
				if (ret < 0){
					LOG_INF(" i2c_read 0x%x failed\n", (addr[page][i]+offset));
				}
				else{
					if (read_data)
						valid_layer = page;
					if (valid_layer!=-1){
						if (i==10)
							ov13850main_otp_data[11]= read_data;
						else if (i==11)
							ov13850main_otp_data[10]= read_data;
						else if (i==12)
							ov13850main_otp_data[13]= read_data;
						else if (i==13)
							ov13850main_otp_data[12]= read_data;
						else
							ov13850main_otp_data[i]= read_data;
					}
					LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(addr[page][i]+offset), read_data);
					read_data = 0;
				}
			}
			if (valid_layer!=-1)
				break;
		}
		LOG_INF("OTP valid layer = %d\n",  valid_layer);

		write_cmos_sensor_otp(0x0100,0x00);
		write_cmos_sensor_otp(0x5002,0x07);
    }

#if 1
    LOG_INF("OTP Module vendor = 0x%x\n",               ov13850main_otp_data[0]);
    LOG_INF("OTP LENS = 0x%x\n",                        ov13850main_otp_data[1]);
    LOG_INF("OTP Sensor Version = 0x%x\n",              ov13850main_otp_data[2]);
    LOG_INF("OTP Driver IC Vendor & Version = 0x%x\n",  ov13850main_otp_data[3]);
    LOG_INF("OTP Actuator vender ID & Version = 0x%x\n",ov13850main_otp_data[4]);

    LOG_INF("OTP AF Infinity position code (MSByte) = 0x%x\n", ov13850main_otp_data[0xB]);
    LOG_INF("OTP AF Infinity position code (LSByte) = 0x%x\n", ov13850main_otp_data[0xA]);
    LOG_INF("OTP AF Macro position code (MSByte) = 0x%x\n",    ov13850main_otp_data[0xD]);
    LOG_INF("OTP AF Macro position code (LSByte) = 0x%x\n",    ov13850main_otp_data[0xC]);

    LOG_INF("OTP BAIS Calibration data = 0x%x\n",   ov13850main_otp_data[0xF]);
    LOG_INF("OTP OFFSET Calibration data = 0x%x\n", ov13850main_otp_data[0x10]);
#endif

    LOG_INF("==========exit ov13850main read_otp=======\n");
    return ret;
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



static uint16_t adjusted_line_length_pclk = 0;
static uint16_t original_line_length_pclk = 0;
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    
    uint16_t line_length_pclk = 0;
    uint32_t Read_length_pclk = 0;
    static uint16_t pre_line_length_pclk = 0;
    
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter! shutter =%d \n", shutter);

    
    
    

    
    
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;

      
     original_line_length_pclk = imgsensor.line_length;
     if(shutter > imgsensor_info.max_frame_length)
     {
         adjusted_line_length_pclk = ( shutter * original_line_length_pclk ) / (imgsensor_info.max_frame_length);
         LOG_INF("ov13850MAIN set shutter : long exposure , adjusted_line_length_pclk=0x%x . \n" , adjusted_line_length_pclk );
     }
     else
     {
	 LOG_INF("ov13850MAIN set shutter : adjusted_line_length_pclk=0 \n");
         adjusted_line_length_pclk = 0;
     }
       

    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;

    
    if (adjusted_line_length_pclk > 0)
        line_length_pclk = adjusted_line_length_pclk;
    else if (pre_line_length_pclk > 0)
        line_length_pclk = original_line_length_pclk;  
    else
        line_length_pclk = 0; 
    


    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        
        write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        }
    } else {
        
        
        
         write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

     
    if (line_length_pclk > 0)
    {
        LOG_INF("ov13850MAIN Set line_length_pclk = 0x%x\n ", line_length_pclk);
        write_cmos_sensor(0x380c, (line_length_pclk >> 8)&0xff);
        write_cmos_sensor(0x380d, line_length_pclk & 0xFF);
    }

    
    Read_length_pclk =((read_cmos_sensor(0x380c) << 8) | read_cmos_sensor(0x380d));
    LOG_INF("ov13850MAIN read sensor line_length_pclk = 0x%x. ( M-mode : preview = 0x960 , normal capture = 0x12c0 )\n ", Read_length_pclk);

    pre_line_length_pclk = adjusted_line_length_pclk;
     

    
   
   
   
		write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=gain;

	if (ov13850main_chip_ver == OV13850main_R1A)
	{
		iReg = gain*32/BASEGAIN;
		if(iReg < 0x20)
		{
			iReg = 0x20;
		}
		if(iReg > 0xfc)
		{
			iReg = 0xfc;
		}
		
	}
	else if(ov13850main_chip_ver == OV13850main_R2A)
	{
		iReg = gain*16/BASEGAIN;
		if(iReg < 0x10)
		{
			iReg = 0x10;
		}
		if(iReg > 0xf8)
		{
			iReg = 0xf8;
		}
	}
	return iReg;
}

static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
 
		reg_gain = gain2reg(gain);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain; 
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
		write_cmos_sensor(0x350a, reg_gain >> 8);
		write_cmos_sensor(0x350b, reg_gain & 0xFF);    
		
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


    switch (image_mirror) {
				case IMAGE_NORMAL:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));
					break;
				case IMAGE_H_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
					break;
				case IMAGE_V_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));		
					break;
				case IMAGE_HV_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
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
    LOG_INF("E\n");
    write_cmos_sensor(0x0103,0x01);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x30);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x030a,0x00);
    write_cmos_sensor(0x300f,0x11);
    write_cmos_sensor(0x3010,0x01);
    write_cmos_sensor(0x3011,0x76);
    write_cmos_sensor(0x3012,0x41);
    write_cmos_sensor(0x3013,0x12);
    write_cmos_sensor(0x3014,0x11);
    write_cmos_sensor(0x301f,0x03);
    write_cmos_sensor(0x3106,0x00);
    write_cmos_sensor(0x3210,0x47);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xc0);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3506,0x00);
    write_cmos_sensor(0x3507,0x02);
    write_cmos_sensor(0x3508,0x00);
    write_cmos_sensor(0x350a,0x00);
    write_cmos_sensor(0x350b,0x80);
    write_cmos_sensor(0x350e,0x00);
    write_cmos_sensor(0x350f,0x10);
    write_cmos_sensor(0x351a,0x00);
    write_cmos_sensor(0x351b,0x10);
    write_cmos_sensor(0x351c,0x00);
    write_cmos_sensor(0x351d,0x20);
    write_cmos_sensor(0x351e,0x00);
    write_cmos_sensor(0x351f,0x40);
    write_cmos_sensor(0x3520,0x00);
    write_cmos_sensor(0x3521,0x80);
    write_cmos_sensor(0x3600,0xc0);
    write_cmos_sensor(0x3601,0xfc);
    write_cmos_sensor(0x3602,0x02);
    write_cmos_sensor(0x3603,0x78);
    write_cmos_sensor(0x3604,0xb1);
    write_cmos_sensor(0x3605,0x95);
    write_cmos_sensor(0x3606,0x73);
    write_cmos_sensor(0x3607,0x07);
    write_cmos_sensor(0x3609,0x40);
    write_cmos_sensor(0x360a,0x30);
    write_cmos_sensor(0x360b,0x91);
    write_cmos_sensor(0x360C,0x09);
    write_cmos_sensor(0x360f,0x02);
    write_cmos_sensor(0x3611,0x10);
    write_cmos_sensor(0x3612,0x08);
    write_cmos_sensor(0x3613,0x33);
    write_cmos_sensor(0x3614,0x2a);
    write_cmos_sensor(0x3615,0x0c);
    write_cmos_sensor(0x3616,0x0e);
    write_cmos_sensor(0x3641,0x02);
    write_cmos_sensor(0x3660,0x82);
    write_cmos_sensor(0x3668,0x54);
    write_cmos_sensor(0x3669,0x00);
    write_cmos_sensor(0x366a,0x3f);
    write_cmos_sensor(0x3667,0xa0);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x3703,0x44);
    write_cmos_sensor(0x3704,0x2c);
    write_cmos_sensor(0x3705,0x01);
    write_cmos_sensor(0x3706,0x15);
    write_cmos_sensor(0x3707,0x44);
    write_cmos_sensor(0x3708,0x3c);
    write_cmos_sensor(0x3709,0x1f);
    write_cmos_sensor(0x370a,0x24);
    write_cmos_sensor(0x370b,0x3c);
    write_cmos_sensor(0x3710,0x28);
    write_cmos_sensor(0x3716,0x03);
    write_cmos_sensor(0x3718,0x10);
    write_cmos_sensor(0x3719,0x0c);
    write_cmos_sensor(0x371a,0x08);
    write_cmos_sensor(0x371b,0x01);
    write_cmos_sensor(0x371c,0xfc);
    write_cmos_sensor(0x3720,0x55);
    write_cmos_sensor(0x3722,0x84);
    write_cmos_sensor(0x3728,0x40);
    write_cmos_sensor(0x372a,0x05);
    write_cmos_sensor(0x372b,0x02);
    write_cmos_sensor(0x372e,0x22);
    write_cmos_sensor(0x372f,0xa0);
    write_cmos_sensor(0x3730,0x02);
    write_cmos_sensor(0x3731,0x5c);
    write_cmos_sensor(0x3732,0x02);
    write_cmos_sensor(0x3733,0x70);
    write_cmos_sensor(0x3738,0x02);
    write_cmos_sensor(0x3739,0x72);
    write_cmos_sensor(0x373a,0x02);
    write_cmos_sensor(0x373b,0x74);
	#if 0
    write_cmos_sensor(0x3740,0x01);
    write_cmos_sensor(0x3741,0xd0);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x01);
	#endif
    write_cmos_sensor(0x3748,0x21);
    write_cmos_sensor(0x3749,0x22);
    
    write_cmos_sensor(0x3760,0x13);
    write_cmos_sensor(0x3761,0x33);
    write_cmos_sensor(0x3762,0x86);
    write_cmos_sensor(0x3763,0x16);
    write_cmos_sensor(0x3767,0x24);
    write_cmos_sensor(0x3768,0x06);
    write_cmos_sensor(0x3769,0x45);
    write_cmos_sensor(0x376c,0x23);
    write_cmos_sensor(0x376f,0x80);
    write_cmos_sensor(0x3773,0x06);
    write_cmos_sensor(0x3780,0x90);
    write_cmos_sensor(0x3781,0x00);
    write_cmos_sensor(0x3782,0x01);
    write_cmos_sensor(0x3d84,0x00);
    write_cmos_sensor(0x3d85,0x17);
    write_cmos_sensor(0x3d8c,0x73);
    write_cmos_sensor(0x3d8d,0xbf);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x0C);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x93);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4B);
    write_cmos_sensor(0x3808,0x10);
    write_cmos_sensor(0x3809,0x80);
    write_cmos_sensor(0x380a,0x0c);
    write_cmos_sensor(0x380b,0x40);
    write_cmos_sensor(0x380c,0x11);
    write_cmos_sensor(0x380d,0xa0);
    write_cmos_sensor(0x380e,0x0d);
    write_cmos_sensor(0x380f,0x00);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x04);
    write_cmos_sensor(0x3814,0x11);
    write_cmos_sensor(0x3815,0x11);
    write_cmos_sensor(0x3820,0x00);
    write_cmos_sensor(0x3821,0x04);
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x04);
    write_cmos_sensor(0x3837,0x01);
    write_cmos_sensor(0x4000,0xf1);
    write_cmos_sensor(0x4001,0x00);
    write_cmos_sensor(0x400b,0x0c);
    write_cmos_sensor(0x4011,0x00);
    write_cmos_sensor(0x401a,0x00);
    write_cmos_sensor(0x401b,0x00);
    write_cmos_sensor(0x401c,0x00);
    write_cmos_sensor(0x401d,0x00);
    write_cmos_sensor(0x4020,0x03);
    write_cmos_sensor(0x4021,0x6C);
    write_cmos_sensor(0x4022,0x0D);
    write_cmos_sensor(0x4023,0x17);
    write_cmos_sensor(0x4024,0x0D);
    write_cmos_sensor(0x4025,0xFC);
    write_cmos_sensor(0x4026,0x0D);
    write_cmos_sensor(0x4027,0xFF);
    write_cmos_sensor(0x4028,0x00);
    write_cmos_sensor(0x4029,0x02);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x403d,0x2c);
    write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4041,0x07);
    write_cmos_sensor(0x4500,0x82);
    write_cmos_sensor(0x4501,0x38);
    write_cmos_sensor(0x458b,0x00);
    write_cmos_sensor(0x459c,0x00);
    write_cmos_sensor(0x459d,0x00);
    write_cmos_sensor(0x459e,0x00);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4602,0x22);
    write_cmos_sensor(0x4603,0x00);
    write_cmos_sensor(0x4837,0x0e);
    write_cmos_sensor(0x4d00,0x04);
    write_cmos_sensor(0x4d01,0x42);
    write_cmos_sensor(0x4d02,0xd1);
    write_cmos_sensor(0x4d03,0x90);
    write_cmos_sensor(0x4d04,0x66);
    write_cmos_sensor(0x4d05,0x65);
    write_cmos_sensor(0x4d0b,0x00);
    write_cmos_sensor(0x5000,0x0e);
    write_cmos_sensor(0x5001,0x01);
    write_cmos_sensor(0x5002,0x07);
    write_cmos_sensor(0x5003,0x4f);
    write_cmos_sensor(0x5013,0x40);
    write_cmos_sensor(0x501c,0x00);
    write_cmos_sensor(0x501d,0x10);
    write_cmos_sensor(0x5100,0x30);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x510f,0xfc);
    write_cmos_sensor(0x5110,0xf0);
    write_cmos_sensor(0x5111,0x10);
    write_cmos_sensor(0x536d,0x02);
    write_cmos_sensor(0x536e,0x67);
    write_cmos_sensor(0x536f,0x01);
    write_cmos_sensor(0x5370,0x4c);
    write_cmos_sensor(0x5400,0x00);
    write_cmos_sensor(0x5400,0x00);
    write_cmos_sensor(0x5401,0x71);
    write_cmos_sensor(0x5402,0x00);
    write_cmos_sensor(0x5403,0x00);
    write_cmos_sensor(0x5404,0x00);
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x540c,0x05);
    write_cmos_sensor(0x5501,0x00);
    write_cmos_sensor(0x5b00,0x00);
    write_cmos_sensor(0x5b01,0x00);
    write_cmos_sensor(0x5b02,0x01);
    write_cmos_sensor(0x5b03,0xff);
    write_cmos_sensor(0x5b04,0x02);
    write_cmos_sensor(0x5b05,0x6c);
    write_cmos_sensor(0x5b09,0x02);
    write_cmos_sensor(0x5e00,0x00);
    write_cmos_sensor(0x5e10,0x1c);
    write_cmos_sensor(0x0100,0x00);
    #if 0
    
     write_cmos_sensor(0x0103,0x01);
     write_cmos_sensor(0x0300,0x00);
     write_cmos_sensor(0x0301,0x00);
     write_cmos_sensor(0x0302,0x30);
     write_cmos_sensor(0x0303,0x00);
     write_cmos_sensor(0x030a,0x00);
     write_cmos_sensor(0x300f,0x11);
     write_cmos_sensor(0x3010,0x01);
     write_cmos_sensor(0x3011,0x76);
     write_cmos_sensor(0x3012,0x41);
     write_cmos_sensor(0x3013,0x12);
     write_cmos_sensor(0x3014,0x11);
     write_cmos_sensor(0x301f,0x03);
     write_cmos_sensor(0x3106,0x00);
     write_cmos_sensor(0x3210,0x47);
     write_cmos_sensor(0x3500,0x00);
     write_cmos_sensor(0x3501,0xc0);
     write_cmos_sensor(0x3502,0x00);
     write_cmos_sensor(0x3506,0x00);
     write_cmos_sensor(0x3507,0x02);
     write_cmos_sensor(0x3508,0x00);
     write_cmos_sensor(0x350a,0x00);
     write_cmos_sensor(0x350b,0x80);
     write_cmos_sensor(0x350e,0x00);
     write_cmos_sensor(0x350f,0x10);
     write_cmos_sensor(0x351a,0x00);
     write_cmos_sensor(0x351b,0x10);
     write_cmos_sensor(0x351c,0x00);
     write_cmos_sensor(0x351d,0x20);
     write_cmos_sensor(0x351e,0x00);
     write_cmos_sensor(0x351f,0x40);
     write_cmos_sensor(0x3520,0x00);
     write_cmos_sensor(0x3521,0x80);
     write_cmos_sensor(0x3600,0xc0);
     write_cmos_sensor(0x3601,0xfc );
     write_cmos_sensor(0x3602,0x02);
     write_cmos_sensor(0x3603,0x78);
     write_cmos_sensor(0x3604,0xb1);
     write_cmos_sensor(0x3605,0x95);
     write_cmos_sensor(0x3606,0x73);
     write_cmos_sensor(0x3607,0x07);
     write_cmos_sensor(0x3609,0x40);
     write_cmos_sensor(0x360a,0x30);
     write_cmos_sensor(0x360b,0x91);
     write_cmos_sensor(0x360C,0x09);
     write_cmos_sensor(0x360f,0x02);
     write_cmos_sensor(0x3611,0x10);
     write_cmos_sensor(0x3612,0x07);
     write_cmos_sensor(0x3613,0x33);
     write_cmos_sensor(0x3615,0x0c);
     write_cmos_sensor(0x3616,0x0e);
     write_cmos_sensor(0x3641,0x02);
     write_cmos_sensor(0x3660,0x82);
     write_cmos_sensor(0x3668,0x54);
     write_cmos_sensor(0x3669,0x00);
     write_cmos_sensor(0x366a,0x3f );
     write_cmos_sensor(0x3667,0xa0);
     write_cmos_sensor(0x3702,0x40);
     write_cmos_sensor(0x3703,0x44);
     write_cmos_sensor(0x3704,0x2c);
     write_cmos_sensor(0x3705,0x01);
     write_cmos_sensor(0x3706,0x15);
     write_cmos_sensor(0x3707,0x44);
     write_cmos_sensor(0x3708,0x3c);
     write_cmos_sensor(0x3709,0x1f );
     write_cmos_sensor(0x370a,0x24);
     write_cmos_sensor(0x370b,0x3c);
     write_cmos_sensor(0x3720,0x55);
     write_cmos_sensor(0x3722,0x84);
     write_cmos_sensor(0x3728,0x40);
     write_cmos_sensor(0x372a,0x04);
     write_cmos_sensor(0x372b,0x02);
     write_cmos_sensor(0x372e,0x22);
     write_cmos_sensor(0x372f,0xa0);
     write_cmos_sensor(0x3730,0x00);
     write_cmos_sensor(0x3731,0x00);
     write_cmos_sensor(0x3732,0x00);
     write_cmos_sensor(0x3733,0x00);
     write_cmos_sensor(0x3710,0x28);
     write_cmos_sensor(0x3716,0x03);
     write_cmos_sensor(0x3718,0x10);
     write_cmos_sensor(0x3719,0x0c);
     write_cmos_sensor(0x371a,0x08);
     write_cmos_sensor(0x371c,0xfc);
     write_cmos_sensor(0x3748,0x00);
     write_cmos_sensor(0x3760,0x13);
     write_cmos_sensor(0x3761,0x33);
     write_cmos_sensor(0x3762,0x86);
     write_cmos_sensor(0x3763,0x16);
     write_cmos_sensor(0x3767,0x24);
     write_cmos_sensor(0x3768,0x06);
     write_cmos_sensor(0x3769,0x45);
     write_cmos_sensor(0x376c,0x23);
     write_cmos_sensor(0x376f,0x80);
     write_cmos_sensor(0x3773,0x06);
     write_cmos_sensor(0x3d84,0x00);
     write_cmos_sensor(0x3d85,0x17);
     write_cmos_sensor(0x3d8c,0x73);
     write_cmos_sensor(0x3d8d,0xbf);
     write_cmos_sensor(0x3800,0x00);
     write_cmos_sensor(0x3801,0x0C);
     write_cmos_sensor(0x3802,0x00);
     write_cmos_sensor(0x3803,0x04);
     write_cmos_sensor(0x3804,0x10);
     write_cmos_sensor(0x3805,0x93);
     write_cmos_sensor(0x3806,0x0c);
     write_cmos_sensor(0x3807,0x4B);
     write_cmos_sensor(0x3808,0x10);
     write_cmos_sensor(0x3809,0x80);
     write_cmos_sensor(0x380a,0x0c);
     write_cmos_sensor(0x380b,0x40);
     write_cmos_sensor(0x380c,0x12);
     write_cmos_sensor(0x380d,0xc0);
     write_cmos_sensor(0x380e,0x0d);
     write_cmos_sensor(0x380f,0x00);
     write_cmos_sensor(0x3810,0x00);
     write_cmos_sensor(0x3811,0x04);
     write_cmos_sensor(0x3812,0x00);
     write_cmos_sensor(0x3813,0x04);
     write_cmos_sensor(0x3814,0x11);
     write_cmos_sensor(0x3815,0x11);
     write_cmos_sensor(0x3820,0x00);
     write_cmos_sensor(0x3821,0x04);
     write_cmos_sensor(0x3823,0x00);
     write_cmos_sensor(0x3826,0x00);
     write_cmos_sensor(0x3827,0x02);
     write_cmos_sensor(0x3834,0x00);
     write_cmos_sensor(0x3835,0x1c);
     write_cmos_sensor(0x3836,0x04);
     write_cmos_sensor(0x3837,0x01);
     write_cmos_sensor(0x4000,0xf1);
     write_cmos_sensor(0x4001,0x00);
     write_cmos_sensor(0x400b,0x0c);
     write_cmos_sensor(0x4011,0x00);
     write_cmos_sensor(0x401a,0x00);
     write_cmos_sensor(0x401b,0x00);
     write_cmos_sensor(0x401c,0x00);
     write_cmos_sensor(0x401d,0x00);
     write_cmos_sensor(0x4020,0x02);
     write_cmos_sensor(0x4021,0x4C);
     write_cmos_sensor(0x4022,0x0E);
     write_cmos_sensor(0x4023,0x37);
     write_cmos_sensor(0x4024,0x0F);
     write_cmos_sensor(0x4025,0x1C);
     write_cmos_sensor(0x4026,0x0F);
     write_cmos_sensor(0x4027,0x1F);
     write_cmos_sensor(0x4028,0x00);
     write_cmos_sensor(0x4029,0x02);
     write_cmos_sensor(0x402a,0x04);
     write_cmos_sensor(0x402b,0x08);
     write_cmos_sensor(0x402c,0x02);
     write_cmos_sensor(0x402d,0x02);
     write_cmos_sensor(0x402e,0x0c);
     write_cmos_sensor(0x402f,0x08);
     write_cmos_sensor(0x403d,0x2c);
     write_cmos_sensor(0x403f,0x7f);
     write_cmos_sensor(0x4041,0x07);
     write_cmos_sensor(0x4500,0x82);
     write_cmos_sensor(0x4501,0x38);
     write_cmos_sensor(0x458b,0x00);
     write_cmos_sensor(0x459c,0x00);
     write_cmos_sensor(0x459d,0x00);
     write_cmos_sensor(0x459e,0x00);
     write_cmos_sensor(0x4601,0x04);
     write_cmos_sensor(0x4602,0x22);
     write_cmos_sensor(0x4603,0x01);
     write_cmos_sensor(0x4837,0x0e);
     write_cmos_sensor(0x4d00,0x04);
     write_cmos_sensor(0x4d01,0x42);
     write_cmos_sensor(0x4d02,0xd1);
     write_cmos_sensor(0x4d03,0x90);
     write_cmos_sensor(0x4d04,0x66);
     write_cmos_sensor(0x4d05,0x65);
     write_cmos_sensor(0x4d0b,0x00);
     write_cmos_sensor(0x5000,0x0e);
     write_cmos_sensor(0x5001,0x01);
     write_cmos_sensor(0x5002,0x07);
     write_cmos_sensor(0x5013,0x40);
     write_cmos_sensor(0x501c,0x00);
     write_cmos_sensor(0x501d,0x10);
     write_cmos_sensor(0x510f ,0xfc);
     write_cmos_sensor(0x5110,0xf0);
     write_cmos_sensor(0x5111,0x10);
     write_cmos_sensor(0x536d,0x02);
     write_cmos_sensor(0x536e,0x67);
     write_cmos_sensor(0x536f,0x01);
     write_cmos_sensor(0x5370,0x4c);
     write_cmos_sensor(0x5400,0x00);
     write_cmos_sensor(0x5400,0x00);
     write_cmos_sensor(0x5401,0x71);
     write_cmos_sensor(0x5402,0x00);
     write_cmos_sensor(0x5403,0x00);
     write_cmos_sensor(0x5404,0x00);
     write_cmos_sensor(0x5405,0x80);
     write_cmos_sensor(0x540c,0x05);
     write_cmos_sensor(0x5501,0x00);
     write_cmos_sensor(0x5b00,0x00);
     write_cmos_sensor(0x5b01,0x00);
     write_cmos_sensor(0x5b02,0x01);
     write_cmos_sensor(0x5b03,0xff);
     write_cmos_sensor(0x5b04,0x02);
     write_cmos_sensor(0x5b05,0x6c);
     write_cmos_sensor(0x5b09,0x02);
     write_cmos_sensor(0x5e00,0x00);
     write_cmos_sensor(0x5e10,0x1c);
     write_cmos_sensor(0x0100,0x00);
     
     #endif
     }    


 static void preview_setting(void)
 {
    LOG_INF("OV13850MAIN Load preview setting.\n");
    
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x19);
    write_cmos_sensor(0x0303,0x01);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3614,0x28);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x67);
    write_cmos_sensor(0x3502,0x80);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x371b,0x00);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f,0x90);
    write_cmos_sensor(0x3730,0x00);
    write_cmos_sensor(0x3731,0x00);
    write_cmos_sensor(0x3732,0x00);
    write_cmos_sensor(0x3733,0x00);
    write_cmos_sensor(0x3738,0x00);
    write_cmos_sensor(0x3739,0x00);
    write_cmos_sensor(0x373a,0x00);
    write_cmos_sensor(0x373b,0x00);
	#if 0
    write_cmos_sensor(0x3740,0x00);
    write_cmos_sensor(0x3741,0x00);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x00);
	#endif
    write_cmos_sensor(0x3748,0x00);
    write_cmos_sensor(0x3749,0x00);
    
    write_cmos_sensor(0x3780,0x10);
    write_cmos_sensor(0x3782,0x00);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4B);
    write_cmos_sensor(0x3808,0x08);
    write_cmos_sensor(0x3809,0x40);
    write_cmos_sensor(0x380a,0x06);
    write_cmos_sensor(0x380b,0x20);
    write_cmos_sensor(0x380c,0x12);
    write_cmos_sensor(0x380d,0xc0);
    write_cmos_sensor(0x380e,0x06);
    write_cmos_sensor(0x380f,0x80);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
	#if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
	#endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
	write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x35);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
    #if 0
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x01);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x28);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x60);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f ,0x90); 
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4b);
    write_cmos_sensor(0x3808,0x08);
    write_cmos_sensor(0x3809,0x40);
    write_cmos_sensor(0x380a,0x06);
    write_cmos_sensor(0x380b,0x20);
    write_cmos_sensor(0x380c,0x09);
    write_cmos_sensor(0x380d,0x60);
    write_cmos_sensor(0x380e,0x0d);
    write_cmos_sensor(0x380f ,0x04); 
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f ,0x08); 
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x83);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x19);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
    #endif
     
     
     		mdelay(10);	
 }    

static void preview_24fps_setting(void)
{
    LOG_INF("OV13850MAIN Load preview 24fps setting.\n");
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x1e);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3614,0x28);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x67);
    write_cmos_sensor(0x3502,0x80);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x371b,0x00);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f,0x90);
    write_cmos_sensor(0x3730,0x00);
    write_cmos_sensor(0x3731,0x00);
    write_cmos_sensor(0x3732,0x00);
    write_cmos_sensor(0x3733,0x00);
    write_cmos_sensor(0x3738,0x00);
    write_cmos_sensor(0x3739,0x00);
    write_cmos_sensor(0x373a,0x00);
    write_cmos_sensor(0x373b,0x00);
	#if 0
    write_cmos_sensor(0x3740,0x00);
    write_cmos_sensor(0x3741,0x00);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x00);
	#endif
    write_cmos_sensor(0x3748,0x00);
    write_cmos_sensor(0x3749,0x00);
    
    write_cmos_sensor(0x3780,0x10);
    write_cmos_sensor(0x3782,0x00);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4B);
    write_cmos_sensor(0x3808,0x08);
    write_cmos_sensor(0x3809,0x40);
    write_cmos_sensor(0x380a,0x06);
    write_cmos_sensor(0x380b,0x20);
    write_cmos_sensor(0x380c,0x17);
    write_cmos_sensor(0x380d,0x78);
    write_cmos_sensor(0x380e,0x06);
    write_cmos_sensor(0x380f,0x80);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
	#if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
	#endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
	write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x43);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
    mdelay(10);

}

int capture_first_flag_main = 0;
int pre_currefps_main = 0;
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
	
if (pre_currefps_main != currefps)
{
	capture_first_flag_main = 0;
	pre_currefps_main = currefps;
}
else
{
	capture_first_flag_main = 1;
}
if (capture_first_flag_main == 0)
{

   if (currefps == 240) {
    LOG_INF("OV13850MAIN Load Capture 24FPS setting. \n");
    
    write_cmos_sensor(0x0100, 0x00);
    
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x28);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x33);
    write_cmos_sensor(0x3614,0x28);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xc0);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x24);
    write_cmos_sensor(0x371b,0x00);
    write_cmos_sensor(0x372a,0x04);
    write_cmos_sensor(0x372f,0xa0);
    write_cmos_sensor(0x3730,0x00);
    write_cmos_sensor(0x3731,0x00);
    write_cmos_sensor(0x3732,0x00);
    write_cmos_sensor(0x3733,0x00);
    write_cmos_sensor(0x3738,0x00);
    write_cmos_sensor(0x3739,0x00);
    write_cmos_sensor(0x373a,0x00);
    write_cmos_sensor(0x373b,0x00);
	#if 0
    write_cmos_sensor(0x3740,0x00);
    write_cmos_sensor(0x3741,0x00);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x00);
	#endif
    write_cmos_sensor(0x3748,0x00);
    write_cmos_sensor(0x3749,0x00);
    
    write_cmos_sensor(0x3780,0x10);
    write_cmos_sensor(0x3782,0x00);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x0C);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x93);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4B);
    write_cmos_sensor(0x3808,0x10);
    write_cmos_sensor(0x3809,0x80);
    write_cmos_sensor(0x380a,0x0c);
    write_cmos_sensor(0x380b,0x40);
    write_cmos_sensor(0x380c,0x12);
    write_cmos_sensor(0x380d,0xc0);
    write_cmos_sensor(0x380e,0x0d);
    write_cmos_sensor(0x380f,0x06);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x04);
    write_cmos_sensor(0x3814,0x11);
    write_cmos_sensor(0x3815,0x11);
    write_cmos_sensor(0x3820,0x00);
    write_cmos_sensor(0x3821,0x04);
	#if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
	#endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x04);
    write_cmos_sensor(0x3837,0x01);
    write_cmos_sensor(0x4020,0x02);
    write_cmos_sensor(0x4021,0x4c);
    write_cmos_sensor(0x4022,0x0e);
    write_cmos_sensor(0x4023,0x37);
    write_cmos_sensor(0x4024,0x0f);
    write_cmos_sensor(0x4025,0x1c);
    write_cmos_sensor(0x4026,0x0f);
    write_cmos_sensor(0x4027,0x1f);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
	write_cmos_sensor(0x403f,0x7f);
    write_cmos_sensor(0x4501,0x38);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x00);
    write_cmos_sensor(0x4837,0x11);
    write_cmos_sensor(0x5101,0x08);
    write_cmos_sensor(0x5102,0x04);
    write_cmos_sensor(0x5103,0x02);
    write_cmos_sensor(0x5104,0x08);
    write_cmos_sensor(0x5105,0x04);
    write_cmos_sensor(0x5106,0x02);
    write_cmos_sensor(0x5107,0x03);
    write_cmos_sensor(0x5108,0x03);
    write_cmos_sensor(0x5109,0x03);
    write_cmos_sensor(0x5401,0x71);
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x0100,0x01);
    #if 0
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0302,0x26);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x07);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xcf );
    write_cmos_sensor(0x3502,0x80);
    write_cmos_sensor(0x370a,0x24); 
    write_cmos_sensor(0x372a,0x04); 
    write_cmos_sensor(0x372f ,0xa0); 
    write_cmos_sensor(0x3800,0x00); 
    write_cmos_sensor(0x3801,0x0C); 
    write_cmos_sensor(0x3802,0x00); 
    write_cmos_sensor(0x3803,0x04); 
    write_cmos_sensor(0x3804,0x10); 
    write_cmos_sensor(0x3805,0x93); 
    write_cmos_sensor(0x3806,0x0c); 
    write_cmos_sensor(0x3807,0x4B); 
    write_cmos_sensor(0x3808,0x10); 
    write_cmos_sensor(0x3809,0x80); 
    write_cmos_sensor(0x380a,0x0c); 
    write_cmos_sensor(0x380b,0x40); 
    write_cmos_sensor(0x380c,0x17); 
    write_cmos_sensor(0x380d,0x7a);	 
    write_cmos_sensor(0x380e,0x0d); 
    write_cmos_sensor(0x380f ,0x00); 
    write_cmos_sensor(0x3810,0x00); 
    write_cmos_sensor(0x3811,0x04); 
    write_cmos_sensor(0x3812,0x00); 
    write_cmos_sensor(0x3813,0x04); 
    write_cmos_sensor(0x3814,0x11); 
    write_cmos_sensor(0x3815,0x11); 
    write_cmos_sensor(0x3820,0x00); 
    write_cmos_sensor(0x3821,0x04); 
    write_cmos_sensor(0x3834,0x00); 
    write_cmos_sensor(0x3835,0x1c); 
    write_cmos_sensor(0x3836,0x04); 
    write_cmos_sensor(0x3837,0x01); 
    write_cmos_sensor(0x4020,0x02); 
    write_cmos_sensor(0x4021,0x4C); 
    write_cmos_sensor(0x4022,0x0E); 
    write_cmos_sensor(0x4023,0x37); 
    write_cmos_sensor(0x4024,0x0F); 
    write_cmos_sensor(0x4025,0x1C); 
    write_cmos_sensor(0x4026,0x0F); 
    write_cmos_sensor(0x4027,0x1F); 
    write_cmos_sensor(0x402a,0x04); 
    write_cmos_sensor(0x402b,0x08); 
    write_cmos_sensor(0x402c,0x02); 
    write_cmos_sensor(0x402d,0x02); 
    write_cmos_sensor(0x402e,0x0c); 
    write_cmos_sensor(0x402f ,0x08); 
   write_cmos_sensor(0x4501,0x38); 
    write_cmos_sensor(0x4601,0x04); 
    write_cmos_sensor(0x4603,0x01); 
    write_cmos_sensor(0x4837,0x12);
    write_cmos_sensor(0x5401,0x71); 
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x0100,0x01);
#endif
      } else {   
    LOG_INF("OV13850MAIN Load Capture 30FPS setting. \n");
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x30);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x08);
    write_cmos_sensor(0x3614,0x2a);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xc0);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x24);
    write_cmos_sensor(0x371b,0x01);
    write_cmos_sensor(0x372a,0x05);
    write_cmos_sensor(0x372f,0xa0);
    write_cmos_sensor(0x3730,0x02);
    write_cmos_sensor(0x3731,0x5c);
    write_cmos_sensor(0x3732,0x02);
    write_cmos_sensor(0x3733,0x70);
    write_cmos_sensor(0x3738,0x02);
    write_cmos_sensor(0x3739,0x72);
    write_cmos_sensor(0x373a,0x02);
    write_cmos_sensor(0x373b,0x74);
	#if 0
    write_cmos_sensor(0x3740,0x01);
    write_cmos_sensor(0x3741,0xd0);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x01);
	#endif
    write_cmos_sensor(0x3748,0x21);
    write_cmos_sensor(0x3749,0x22);
    
    write_cmos_sensor(0x3780,0x90);
    write_cmos_sensor(0x3782,0x01);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x0C);
    write_cmos_sensor(0x3802,0x00);
    write_cmos_sensor(0x3803,0x04);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x93);
    write_cmos_sensor(0x3806,0x0c);
    write_cmos_sensor(0x3807,0x4B);
    write_cmos_sensor(0x3808,0x10);
    write_cmos_sensor(0x3809,0x80);
    write_cmos_sensor(0x380a,0x0c);
    write_cmos_sensor(0x380b,0x40);
    write_cmos_sensor(0x380c,0x11);
    write_cmos_sensor(0x380d,0xa0);
    write_cmos_sensor(0x380e,0x0d);
    write_cmos_sensor(0x380f,0x00);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x04);
    write_cmos_sensor(0x3814,0x11);
    write_cmos_sensor(0x3815,0x11);
    write_cmos_sensor(0x3820,0x00);
    write_cmos_sensor(0x3821,0x04);
	#if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
	#endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x04);
    write_cmos_sensor(0x3837,0x01);
    write_cmos_sensor(0x4020,0x03);
    write_cmos_sensor(0x4021,0x6C);
    write_cmos_sensor(0x4022,0x0D);
    write_cmos_sensor(0x4023,0x17);
    write_cmos_sensor(0x4024,0x0D);
    write_cmos_sensor(0x4025,0xFC);
    write_cmos_sensor(0x4026,0x0D);
    write_cmos_sensor(0x4027,0xFF);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
	write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x38);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x00);
    write_cmos_sensor(0x4837,0x0e);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x71);
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x0100,0x01);
    #if 0
     
     write_cmos_sensor(0x0100,0x00);
     write_cmos_sensor(0x0300,0x00);
     write_cmos_sensor(0x0302,0x30);
     write_cmos_sensor(0x0303,0x00);
     write_cmos_sensor(0x3612,0x07);
     write_cmos_sensor(0x3500,0x00);
     write_cmos_sensor(0x3501,0xcf);
     write_cmos_sensor(0x3502,0x80);
     write_cmos_sensor(0x370a,0x24);
     write_cmos_sensor(0x372a,0x04);
     write_cmos_sensor(0x372f,0xa0);
     write_cmos_sensor(0x3800,0x00);
     write_cmos_sensor(0x3801,0x0C);
     write_cmos_sensor(0x3802,0x00);
     write_cmos_sensor(0x3803,0x04);
     write_cmos_sensor(0x3804,0x10);
     write_cmos_sensor(0x3805,0x93);
     write_cmos_sensor(0x3806,0x0c);
     write_cmos_sensor(0x3807,0x4B);
     write_cmos_sensor(0x3808,0x10);
     write_cmos_sensor(0x3809,0x80);
     write_cmos_sensor(0x380a,0x0c);
     write_cmos_sensor(0x380b,0x40);
     write_cmos_sensor(0x380c,0x12);
     write_cmos_sensor(0x380d,0xc0);
     write_cmos_sensor(0x380e,0x0d);
     write_cmos_sensor(0x380f,0x00);
     write_cmos_sensor(0x3810,0x00);
     write_cmos_sensor(0x3811,0x04);
     write_cmos_sensor(0x3812,0x00);
     write_cmos_sensor(0x3813,0x04);
     write_cmos_sensor(0x3814,0x11);
     write_cmos_sensor(0x3815,0x11);
     write_cmos_sensor(0x3820,0x00);
     write_cmos_sensor(0x3821,0x04);
     write_cmos_sensor(0x3834,0x00);
     write_cmos_sensor(0x3835,0x1c);
     write_cmos_sensor(0x3836,0x04);
     write_cmos_sensor(0x3837,0x01);
     write_cmos_sensor(0x4020,0x02);
     write_cmos_sensor(0x4021,0x4C);
     write_cmos_sensor(0x4022,0x0E);
     write_cmos_sensor(0x4023,0x37);
     write_cmos_sensor(0x4024,0x0F);
     write_cmos_sensor(0x4025,0x1C);
     write_cmos_sensor(0x4026,0x0F);
     write_cmos_sensor(0x4027,0x1F);
     write_cmos_sensor(0x402a,0x04);
     write_cmos_sensor(0x402b,0x08);
     write_cmos_sensor(0x402c,0x02);
     write_cmos_sensor(0x402d,0x02);
     write_cmos_sensor(0x402e,0x0c);
     write_cmos_sensor(0x402f,0x08);
     write_cmos_sensor(0x4501,0x38);
     write_cmos_sensor(0x4601,0x04);
     write_cmos_sensor(0x4603,0x01);
     write_cmos_sensor(0x4837,0x0e);
     write_cmos_sensor(0x5401,0x71);
     write_cmos_sensor(0x5405,0x80);
     write_cmos_sensor(0x0100,0x01);
     #endif
     



        if (imgsensor.ihdr_en) {

    } else {

    }

    }
	mdelay(20);	
	capture_first_flag_main = 1;
}

}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("OV13850main Load normal_video:4K sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0302,0x30);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x07);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xcf);
    write_cmos_sensor(0x3502,0x80);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x24);
    write_cmos_sensor(0x372a,0x04);
    write_cmos_sensor(0x372f,0xa0);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0xCC);
    write_cmos_sensor(0x3802,0x01);
    write_cmos_sensor(0x3803,0xE4);
    write_cmos_sensor(0x3804,0x0F);
    write_cmos_sensor(0x3805,0xD3);
    write_cmos_sensor(0x3806,0x0A);
    write_cmos_sensor(0x3807,0x6B);
    write_cmos_sensor(0x3808,0x0F);
    write_cmos_sensor(0x3809,0x00);
    write_cmos_sensor(0x380a,0x08);
    write_cmos_sensor(0x380b,0x80);
    write_cmos_sensor(0x380c,0x12);
    write_cmos_sensor(0x380d,0xc0);
    write_cmos_sensor(0x380e,0x0d);
    write_cmos_sensor(0x380f,0x00);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x04);
    write_cmos_sensor(0x3814,0x11);
    write_cmos_sensor(0x3815,0x11);
    write_cmos_sensor(0x3820,0x00);
    write_cmos_sensor(0x3821,0x04);
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x04);
    write_cmos_sensor(0x3837,0x01);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xE6);
    write_cmos_sensor(0x4022,0x0E);
    write_cmos_sensor(0x4023,0x1E);
    write_cmos_sensor(0x4024,0x0F);
    write_cmos_sensor(0x4025,0x00);
    write_cmos_sensor(0x4026,0x0F);
    write_cmos_sensor(0x4027,0x06);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x4501,0x38);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x0e);
    write_cmos_sensor(0x5401,0x71);
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x0100,0x01);

    mdelay(20);

}
static void hs_video_setting()
{
    LOG_INF("OV13850main Load HS_VIDEO sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x01);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x28);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3614,0x28);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x67);
    write_cmos_sensor(0x3502,0x80);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x371b,0x00);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f,0x90);
    write_cmos_sensor(0x3730,0x00);
    write_cmos_sensor(0x3731,0x00);
    write_cmos_sensor(0x3732,0x00);
    write_cmos_sensor(0x3733,0x00);
    write_cmos_sensor(0x3738,0x00);
    write_cmos_sensor(0x3739,0x00);
    write_cmos_sensor(0x373a,0x00);
    write_cmos_sensor(0x373b,0x00);
    #if 0
    write_cmos_sensor(0x3740,0x00);
    write_cmos_sensor(0x3741,0x00);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x00);
    #endif
    write_cmos_sensor(0x3748,0x00);
    write_cmos_sensor(0x3749,0x00);
    
    write_cmos_sensor(0x3780,0x10);
    write_cmos_sensor(0x3782,0x00);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x01);
    write_cmos_sensor(0x3803,0x80);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x0a);
    write_cmos_sensor(0x3807,0xcf);
    write_cmos_sensor(0x3808,0x08);
    write_cmos_sensor(0x3809,0x40);
    write_cmos_sensor(0x380a,0x04);
    write_cmos_sensor(0x380b,0xa4);
    write_cmos_sensor(0x380c,0x09);
    write_cmos_sensor(0x380d,0x60);
    write_cmos_sensor(0x380e,0x06);
    write_cmos_sensor(0x380f,0x80);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
    #if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
    #endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x83);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x19);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
    #if 0
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x01);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x28);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x60);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f,0x90);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x01);
    write_cmos_sensor(0x3803,0x80);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x0a);
    write_cmos_sensor(0x3807,0xcf);
    write_cmos_sensor(0x3808,0x08);
    write_cmos_sensor(0x3809,0x40);
    write_cmos_sensor(0x380a,0x04);
    write_cmos_sensor(0x380b,0xa4);
    write_cmos_sensor(0x380c,0x09);
    write_cmos_sensor(0x380d,0x60);
    write_cmos_sensor(0x380e,0x06);
    write_cmos_sensor(0x380f,0x80);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x83);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x19);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
#endif
    mdelay(10);
}

static void slim_video_setting()
{
    LOG_INF("OV13850main Load Slim sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x01);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x28);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x27);
    write_cmos_sensor(0x3614,0x28);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0x30);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x27);
    write_cmos_sensor(0x371b,0x00);
    write_cmos_sensor(0x372a,0x00);
    write_cmos_sensor(0x372f,0x90);
    write_cmos_sensor(0x3730,0x00);
    write_cmos_sensor(0x3731,0x00);
    write_cmos_sensor(0x3732,0x00);
    write_cmos_sensor(0x3733,0x00);
    write_cmos_sensor(0x3738,0x00);
    write_cmos_sensor(0x3739,0x00);
    write_cmos_sensor(0x373a,0x00);
    write_cmos_sensor(0x373b,0x00);
    write_cmos_sensor(0x3748,0x00);
    write_cmos_sensor(0x3749,0x00);
    write_cmos_sensor(0x3780,0x10);
    write_cmos_sensor(0x3782,0x00);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x08);
    write_cmos_sensor(0x3802,0x03);
    write_cmos_sensor(0x3803,0x54);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x97);
    write_cmos_sensor(0x3806,0x08);
    write_cmos_sensor(0x3807,0xfb);
    write_cmos_sensor(0x3808,0x05);
    write_cmos_sensor(0x3809,0x00);
    write_cmos_sensor(0x380a,0x02);
    write_cmos_sensor(0x380b,0xd0);
    write_cmos_sensor(0x380c,0x09);
    write_cmos_sensor(0x380d,0x60);
    write_cmos_sensor(0x380e,0x03);
    write_cmos_sensor(0x380f,0x40);
    write_cmos_sensor(0x3810,0x01);
    write_cmos_sensor(0x3811,0xa4);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x02);
    write_cmos_sensor(0x3814,0x31);
    write_cmos_sensor(0x3815,0x31);
    write_cmos_sensor(0x3820,0x02);
    write_cmos_sensor(0x3821,0x06);
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x08);
    write_cmos_sensor(0x3837,0x02);
    write_cmos_sensor(0x4020,0x00);
    write_cmos_sensor(0x4021,0xe4);
    write_cmos_sensor(0x4022,0x04);
    write_cmos_sensor(0x4023,0xd7);
    write_cmos_sensor(0x4024,0x05);
    write_cmos_sensor(0x4025,0xbc);
    write_cmos_sensor(0x4026,0x05);
    write_cmos_sensor(0x4027,0xbf);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x3c);
    write_cmos_sensor(0x4601,0x4f);
    write_cmos_sensor(0x4603,0x01);
    write_cmos_sensor(0x4837,0x19);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x61);
    write_cmos_sensor(0x5405,0x40);
    write_cmos_sensor(0x0100,0x01);
    mdelay(10);

}

static void full_16_9_setting()
{
    LOG_INF("OV13850main Load Full 16:9 sensor setting \n");
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0300,0x00);
    write_cmos_sensor(0x0301,0x00);
    write_cmos_sensor(0x0302,0x30);
    write_cmos_sensor(0x0303,0x00);
    write_cmos_sensor(0x3612,0x08);
    write_cmos_sensor(0x3614,0x2a);
    write_cmos_sensor(0x3500,0x00);
    write_cmos_sensor(0x3501,0xc0);
    write_cmos_sensor(0x3502,0x00);
    write_cmos_sensor(0x3702,0x40);
    write_cmos_sensor(0x370a,0x24);
    write_cmos_sensor(0x371b,0x01);
    write_cmos_sensor(0x372a,0x05);
    write_cmos_sensor(0x372f,0xa0);
    write_cmos_sensor(0x3730,0x02);
    write_cmos_sensor(0x3731,0x5c);
    write_cmos_sensor(0x3732,0x02);
    write_cmos_sensor(0x3733,0x70);
    write_cmos_sensor(0x3738,0x02);
    write_cmos_sensor(0x3739,0x72);
    write_cmos_sensor(0x373a,0x02);
    write_cmos_sensor(0x373b,0x74);
    #if 0
    write_cmos_sensor(0x3740,0x01);
    write_cmos_sensor(0x3741,0xd0);
    write_cmos_sensor(0x3742,0x00);
    write_cmos_sensor(0x3743,0x01);
    #endif
    write_cmos_sensor(0x3748,0x21);
    write_cmos_sensor(0x3749,0x22);
    
    write_cmos_sensor(0x3780,0x90);
    write_cmos_sensor(0x3782,0x01);
    write_cmos_sensor(0x3800,0x00);
    write_cmos_sensor(0x3801,0x0C);
    write_cmos_sensor(0x3802,0x01);
    write_cmos_sensor(0x3803,0x80);
    write_cmos_sensor(0x3804,0x10);
    write_cmos_sensor(0x3805,0x93);
    write_cmos_sensor(0x3806,0x0a);
    write_cmos_sensor(0x3807,0xCF);
    write_cmos_sensor(0x3808,0x10);
    write_cmos_sensor(0x3809,0x80);
    write_cmos_sensor(0x380a,0x09);
    write_cmos_sensor(0x380b,0x48);
    write_cmos_sensor(0x380c,0x11);
    write_cmos_sensor(0x380d,0xa0);
    write_cmos_sensor(0x380e,0x0c);
    write_cmos_sensor(0x380f,0xee);
    write_cmos_sensor(0x3810,0x00);
    write_cmos_sensor(0x3811,0x04);
    write_cmos_sensor(0x3812,0x00);
    write_cmos_sensor(0x3813,0x04);
    write_cmos_sensor(0x3814,0x11);
    write_cmos_sensor(0x3815,0x11);
    write_cmos_sensor(0x3820,0x00);
    write_cmos_sensor(0x3821,0x04);
    #if 0
    write_cmos_sensor(0x3823,0x00);
    write_cmos_sensor(0x3826,0x00);
    write_cmos_sensor(0x3827,0x02);
    #endif
    write_cmos_sensor(0x3834,0x00);
    write_cmos_sensor(0x3835,0x1c);
    write_cmos_sensor(0x3836,0x04);
    write_cmos_sensor(0x3837,0x01);
    write_cmos_sensor(0x4020,0x03);
    write_cmos_sensor(0x4021,0x6C);
    write_cmos_sensor(0x4022,0x0D);
    write_cmos_sensor(0x4023,0x17);
    write_cmos_sensor(0x4024,0x0D);
    write_cmos_sensor(0x4025,0xFC);
    write_cmos_sensor(0x4026,0x0D);
    write_cmos_sensor(0x4027,0xFF);
    write_cmos_sensor(0x402a,0x04);
    write_cmos_sensor(0x402b,0x08);
    write_cmos_sensor(0x402c,0x02);
    write_cmos_sensor(0x402d,0x02);
    write_cmos_sensor(0x402e,0x0c);
    write_cmos_sensor(0x402f,0x08);
    write_cmos_sensor(0x403f,0x40);
    write_cmos_sensor(0x4501,0x38);
    write_cmos_sensor(0x4601,0x04);
    write_cmos_sensor(0x4603,0x00);
    write_cmos_sensor(0x4837,0x0e);
    write_cmos_sensor(0x5101,0x02);
    write_cmos_sensor(0x5102,0x01);
    write_cmos_sensor(0x5103,0x01);
    write_cmos_sensor(0x5104,0x02);
    write_cmos_sensor(0x5105,0x01);
    write_cmos_sensor(0x5106,0x01);
    write_cmos_sensor(0x5107,0x00);
    write_cmos_sensor(0x5108,0x00);
    write_cmos_sensor(0x5109,0x00);
    write_cmos_sensor(0x5401,0x71);
    write_cmos_sensor(0x5405,0x80);
    write_cmos_sensor(0x0100,0x01);
    mdelay(10);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        
        
        write_cmos_sensor(0x5E00, 0x80);
    } else {
        
        
        write_cmos_sensor(0x5E00, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static const char *ov13850mainVendor = "OmniVision";
static const char *ov13850mainNAME = "ov13850_main";
static const char *ov13850mainSize = "13.0M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", ov13850mainVendor, ov13850mainNAME, ov13850mainSize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static struct kobject *android_ov13850main;
static int first = true;

static int ov13850main_sysfs_init(void)
{
	int ret ;
	if(first){
		LOG_INF("kobject creat and add\n");
		android_ov13850main = kobject_create_and_add("android_camera", NULL);
		if (android_ov13850main == NULL) {
			LOG_INF(" subsystem_register " \
			"failed\n");
			ret = -ENOMEM;
			return ret ;
		}
		LOG_INF("sysfs_create_file\n");
		ret = sysfs_create_file(android_ov13850main, &dev_attr_sensor.attr);
		if (ret) {
			LOG_INF("sysfs_create_file " \
			"failed\n");
			kobject_del(android_ov13850main);
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
                ov13850main_sysfs_init();
                read_ov13850main_otp();
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
    LOG_2;
    
    kdSetI2CSpeed(400);
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
        
		if ((read_cmos_sensor(0x302a))==0xb1)
		{
				LOG_INF("----R1A---- \n");
				ov13850main_chip_ver = OV13850main_R1A;
		}else if((read_cmos_sensor(0x302a))==0xb2)
		{
				LOG_INF("----R2A---- \n");
				ov13850main_chip_ver = OV13850main_R2A;
		}
    
    sensor_init();
	  
	  
	  write_cmos_sensor(0x0100, 0x00);
	  
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
    
	capture_first_flag_main = 0;
	pre_currefps_main = 0;
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
    set_mirror_flip(imgsensor.mirror);
	
    return ERROR_NONE;
}    

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
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
	  set_mirror_flip(imgsensor.mirror);
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
    
    full_16_9_setting();
    set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}   

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(240);
    set_mirror_flip(imgsensor.mirror);
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
    set_mirror_flip(imgsensor.mirror);
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
	
		set_mirror_flip(imgsensor.mirror);
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
	
		set_mirror_flip(imgsensor.mirror);
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
	
		set_mirror_flip(imgsensor.mirror);
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
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

     sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
     sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;
    
    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;
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
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
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
         case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

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
			capture_first_flag_main = 0;
			pre_currefps_main = 0;
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			capture_first_flag_main = 0;
			pre_currefps_main = 0;
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			capture_first_flag_main = 0;
			pre_currefps_main = 0;
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
			capture_first_flag_main = 0;
			pre_currefps_main = 0;
            slim_video(image_window, sensor_config_data);
            break;
         case MSDK_SCENARIO_ID_CUSTOM1:
            LOG_INF("scenario_id = MSDK_SCENARIO_ID_CUSTOM1\n");
            Custom1(image_window, sensor_config_data); 
            break;
          case MSDK_SCENARIO_ID_CUSTOM2:
            LOG_INF("scenario_id = MSDK_SCENARIO_ID_CUSTOM2\n");
            Custom2(image_window, sensor_config_data); 
            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            LOG_INF("scenario_id = MSDK_SCENARIO_ID_CUSTOM3\n");
            Custom3(image_window, sensor_config_data); 
            break;
        default:
            LOG_INF("Error ScenarioId setting");
			capture_first_flag_main = 0;
			pre_currefps_main = 0;
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
            set_dummy();
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
            set_dummy();
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
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            framerate=600;
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            LOG_INF("ov13850main set framerate=600 ,  frame_length = 0x%x .  \n", frame_length);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            framerate=1200;
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            LOG_INF("ov13850main set framerate=1200 ,  frame_length = 0x%x .  \n", frame_length);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
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
          case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            framerate=240;
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            LOG_INF("ov13850main set framerate=240 ,  frame_length = 0x%x .  \n", frame_length);
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
            set_dummy();
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
         case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
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
            LOG_INF("current fps :%d\n", *feature_data);
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
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);
			
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

UINT32 OV13850MAIN_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    
