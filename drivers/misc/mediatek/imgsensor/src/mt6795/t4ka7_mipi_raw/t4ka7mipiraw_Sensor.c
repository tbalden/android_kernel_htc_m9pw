
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

#include "t4ka7mipiraw_Sensor.h"

#define PFX "T4KA7_camera_sensor"
#define LOG_1 LOG_INF("T4KA7,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 5000*3750@30fps,1.2Gbps/lane; capture 18.7M@30fps,1.2Gbps/lane\n")

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

extern void otp_cali(unsigned char writeid);

#define Capture_15FPS
static imgsensor_info_struct imgsensor_info = {
    .sensor_id = T4KA7_SENSOR_ID,        

    .checksum_value = 0xbde6b5f8,

    .pre = {
	 
        .pclk = 199200000,                
        .linelength =0x0c58,  
        .framelength =0x0834,  
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width =2688,
        .grabwindow_height =1872,
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 300,
    },
    .cap = {
         .pclk = 480000000,
        .linelength =0x16EC,  
        .framelength =0x0EC2,  
        .startx =0,
        .starty =0,
        .grabwindow_width =5376,
        .grabwindow_height =3752,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate =210,
    },
    
    .cap1 = {  
         
         .pclk =336000000,
        .linelength =0x1728,  
        .framelength =0x0EC2,  
        .startx =0,
        .starty =0,
        .grabwindow_width =5312,
        .grabwindow_height =3752,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate =150,
    },
    .normal_video = {
        .pclk = 288000000,
        .linelength = 0x1170,  
        .framelength = 0x088A,  
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0x0F00,  
        .grabwindow_height = 2160,
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 316800000,
        .linelength = 0x11AE,  
        .framelength = 0x048E,  
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0x0780,  
        .grabwindow_height = 0x0438,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 600,
    },
    .slim_video = {
        .pclk = 403200000,
        .linelength = 0x11AE,
        .framelength = 0x02E6,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 0x0500,
        .grabwindow_height = 0x02D0,
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate = 1200,

    },
    .custom1 = {
        
        .pclk = 480000000,                
        .linelength =0x16D8,  
        .framelength =0x0BEE,  
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width =5376,
        .grabwindow_height =0x0BD4,
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 267,
    },
      .custom2 = {
        
         .pclk =336000000,
        .linelength =0x1728,  
        .framelength =0x0EC2,  
        .startx =0,
        .starty =0,
        .grabwindow_width =5312,
        .grabwindow_height =3752,  
        .mipi_data_lp2hs_settle_dc = 100,
        .max_framerate =150,
    },
      .custom3 = {
        
       .pclk = 144000000,                
        .linelength =0x0c58,  
        .framelength =0x076E,  
        .startx = 0,                    
        .starty = 0,                    
        .grabwindow_width =2688,
        .grabwindow_height =1872,
        
        .mipi_data_lp2hs_settle_dc = 100,
        
        .max_framerate = 240,
    },
    .margin = 8,            
    .min_shutter = 0x2,        
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,    
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 3,
    .ihdr_support = 0,      
    .ihdr_le_firstline = 1,  
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
    .mipi_settle_delay_mode = 1,
    .sensor_output_dataformat =  SENSOR_OUTPUT_FORMAT_RAW_Gr,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = { 0x20, 0x6C, 0xff},
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                
    .sensor_mode = IMGSENSOR_MODE_INIT, 
    .shutter = 0x3D0,                    
    .gain = 0x100,                        
    .dummy_pixel = 0,                    
    .dummy_line = 0,                    
    .current_fps = 0,  
    .autoflicker_en = KAL_FALSE,  
    .test_pattern = KAL_FALSE,        
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 1, 
    .i2c_write_id = 0x20,
};


static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] =
{{ 5384, 3752, 192, 0, 5000, 3752, 2500, 1876, 0, 0, 2500, 1876, 0, 0, 2688, 1876},  
{ 5384, 3752, 192, 0, 5000, 3752, 5000, 3752, 0, 0, 5000, 3752, 8, 4, 5312, 3752},  
{ 5384, 3752, 192, 0, 5000, 3752, 3500, 2160, 0, 0, 3500, 2160, 0, 0, 3840, 2176}, 
{ 5384, 3752, 128, 436, 5128, 2880, 1282, 720, 2, 0, 1280, 720, 0, 0, 1280, 720}, 
{ 5384, 3752, 128, 436, 5152, 2880, 1282, 720, 2, 0, 1280, 720, 0, 0, 1280, 720},
{ 5384, 3752, 192, 0, 5000, 3752, 5000, 3752, 0, 0, 5000, 3752, 8, 4, 5376, 3028},  
{ 5384, 3752, 192, 0, 5000, 3752, 5000, 3752, 0, 0, 5000, 3752, 8, 4, 5312, 3752},  
{ 5384, 3752, 192, 0, 5000, 3752, 2500, 1876, 0, 0, 2500, 1876, 0, 0, 2688, 1876},  
};



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

#if 0
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
#else
    if (iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id) != 0)
    {
        get_byte = 0xFF;
    }
#endif

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
    
   
   
   
   
   

}    
#define CONFIG_MTK_TC7_FEATURE 1
#ifdef CONFIG_MTK_TC7_FEATURE
static kal_uint32 return_sensor_id_ov4688()
{
	imgsensor.i2c_write_id=0x20;
    return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static kal_uint32 return_sensor_id_ov13850()
{
	imgsensor.i2c_write_id=0x6C;
    return  ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static kal_uint32 return_sensor_id_ov2722()
{
	imgsensor.i2c_write_id=0x6C;
    return  ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static kal_uint32 return_sensor_id()
{
    LOG_INF("==========t4k get id=======\n");
    kal_uint32 cmos_id = 0;
    kal_uint8 ori_i2c_write_id = 0;

    ori_i2c_write_id = imgsensor.i2c_write_id;

    kal_uint32 sensor_id_ov4688 = 0;
    kal_uint32 sensor_id_ov13850= 0;
    kal_uint32 sensor_id_ov2722= 0;

    cmos_id =((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));

    if(cmos_id == 0x0)
    {
        sensor_id_ov4688 = return_sensor_id_ov4688();
        sensor_id_ov13850= return_sensor_id_ov13850();
        sensor_id_ov2722= return_sensor_id_ov2722();

        if((sensor_id_ov4688 == 0x4688)||(sensor_id_ov13850 == 0xD850)||(sensor_id_ov2722 == 0x2722))
        {
            cmos_id = 0x0;
        }
        else
        {
            LOG_INF(" htc workround for T4KA7 no sensor_id module. \n");
            cmos_id =T4KA7_SENSOR_ID;
        }
    }
     
    imgsensor.i2c_write_id=ori_i2c_write_id;

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

static int read_t4ka7_otp_flag = 0;
int t4ka7_vcm_driver_vendor_flag;
char t4ka7_otp_data[23];


static int read_otp(void){
    LOG_INF("==========t4k read_otp read_t4ka7_otp_flag =%d=======\n",read_t4ka7_otp_flag);
	
    
    
	
	int page = 0;
    int	valid_layer = -1;
	int i = 0;
	u8 read_data = 0;
	int ret = 0;
	LOG_INF("==========t4k start read_otp=======\n");

	

	write_cmos_sensor_otp(0x2A00, 0x01); 
#if 1
			for(page = 2; page >= 0; page--)   
			{
				write_cmos_sensor_otp(0x2A02, page);

				mdelay(10);
				for(i = 0; i < 13; i++)
				{
					ret = read_cmos_sensor_otp(0x2A04 + i, &read_data);
					if (ret < 0)
						LOG_INF(" i2c_read 0x%x failed\n", (0x2A04 + i));
					else
					{
						
						if(read_data)
							valid_layer = page;
						if(valid_layer !=-1)
							t4ka7_otp_data[i]=read_data;

#if 0
						if(valid_layer !=-1) {
							if(i==0)
								LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);
							else if (i==1)
								LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);
							else if (i==2)
								LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);
							else if (i==3)
								LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);
							else if (i==4)
								LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);
						}
#else
						LOG_INF("page = %d, addr = 0x%x data = 0x%x \n",page,(0x2A04 + i), read_data);


#endif
						read_data = 0;
					}
				}
				if(valid_layer != -1)
				{
					if(t4ka7_otp_data[4]==0x72)
						t4ka7_vcm_driver_vendor_flag = 0;
					else
						t4ka7_vcm_driver_vendor_flag = 1;

					LOG_INF("valid_layer of module info:%d \n", valid_layer);
					break;
				}
			}
#endif
			valid_layer = -1;
			LOG_INF(" ### t4ka7_vcm_driver_vendor_flag = %d (0=TDK , 1=MTM).\n", t4ka7_vcm_driver_vendor_flag);
			for(page = 5; page >= 3; page--)   
			{
				ret = write_cmos_sensor_otp(0x2A02, page);
				mdelay(10);
				for(i = 0; i < 10; i++)
				{
                    ret = read_cmos_sensor_otp(0x2A04 + i, &read_data);
					if (ret < 0)
						LOG_INF(" i2c_read 0x%x failed\n", (0x2A04 + i));
					else
					{
						
						if(read_data)
							valid_layer = page;
						if(valid_layer !=-1) {
							if(i==4)
								t4ka7_otp_data[0x12]=read_data;
							else if (i==5)
								t4ka7_otp_data[0x11]=read_data;
							else if (i==6)
								t4ka7_otp_data[0x14]=read_data;
							else if (i==7)
								t4ka7_otp_data[0x13]=read_data;
                            else
                                t4ka7_otp_data[13+i]=read_data;
						}
						LOG_INF("page = %d, addr = 0x%x data = 0x%x\n",page,(0x2A04 + i), read_data);
						read_data = 0;
					}
				}

				if(valid_layer != -1)
				{
					LOG_INF("valid_layer of module info:%d \n", valid_layer);
					break;
				}
			}

	write_cmos_sensor(0x2A00, 0x00);
	if(valid_layer!=-1) {
	    spin_lock(&imgsensor_drv_lock);
		read_t4ka7_otp_flag = 1;
		spin_unlock(&imgsensor_drv_lock);
	}
	LOG_INF("==========exit t4k read_otp=======\n");
	return ret;
}
#else
static kal_uint32 return_sensor_id()
{
    return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}
#endif
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
    
}    



static uint16_t adjusted_line_length_pclk = 0;
static uint16_t original_line_length_pclk = 0;
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
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
         LOG_INF("T4KA7 set shutter : long exposure , adjusted_line_length_pclk=0x%x . \n" , adjusted_line_length_pclk );
     }
     else
     {
	 LOG_INF("T4KA7 set shutter : adjusted_line_length_pclk=0 \n");
         adjusted_line_length_pclk = 0;
     }
     

    spin_unlock(&imgsensor_drv_lock);
    imgsensor.shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    imgsensor.shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        
        
        
        }
    } else {
        
        
        
    }
    
    
    
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=gain;
	iReg = gain*32/BASEGAIN;
	if(iReg < 0x20)
	{
		iReg = 0x20;
	}
	if(iReg > 0x180)
	{
		iReg = 0x180;
	}
	return iReg;
}

static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
    
    uint16_t line_length_pclk = 0;
    uint32_t Read_length_pclk = 0;
    static uint16_t pre_line_length_pclk = 0;
    

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);

    
    if (adjusted_line_length_pclk > 0)
        line_length_pclk = adjusted_line_length_pclk;
    else if (pre_line_length_pclk > 0)
        line_length_pclk = original_line_length_pclk;  
    else
        line_length_pclk = 0; 
	 

    write_cmos_sensor(0x0104, 0x01);
    
    LOG_INF("T4KA7 Set imgsensor.frame_length = 0x%x\n ", imgsensor.frame_length);
    write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8)&0xff);
    write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);

    
    
    if (line_length_pclk > 0)
    {
        LOG_INF("T4KA7 Set line_length_pclk = 0x%x\n ", line_length_pclk);
        write_cmos_sensor(0x0342, (line_length_pclk >> 8)&0xff);
        write_cmos_sensor(0x0343, line_length_pclk & 0xFF);
    }
    write_cmos_sensor(0x0202, (imgsensor.shutter >> 8)&0xff);
    write_cmos_sensor(0x0203, imgsensor.shutter & 0xFF);

    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
    write_cmos_sensor(0x0204, (reg_gain >> 8) & 0x0F);
    write_cmos_sensor(0x0205, reg_gain & 0xFF);

    
    Read_length_pclk =((read_cmos_sensor(0x0342) << 8) | read_cmos_sensor(0x0343));
    LOG_INF("T4KA7 read sensor line_length_pclk = 0x%x. ( M-mode : preview = 0xC58 , normal capture = 0x16EC )\n ", Read_length_pclk);

    write_cmos_sensor(0x0104, 0x00);

    
    pre_line_length_pclk = adjusted_line_length_pclk;
    

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


        
        write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8)&0xff);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
				
        write_cmos_sensor(0x0203, le & 0xFF);
        write_cmos_sensor(0x0202, (le >> 8) & 0xFF);

        write_cmos_sensor(0x0225, se  & 0xFF);
        write_cmos_sensor(0x0224, (se >> 8) & 0xFF);

        set_gain(gain);
    }

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);


    switch (image_mirror) {
				case IMAGE_NORMAL:
					write_cmos_sensor(0x0101,((read_cmos_sensor(0x0101) & 0xFC) | 0x00));
					break;
				case IMAGE_H_MIRROR:
					write_cmos_sensor(0x0101,((read_cmos_sensor(0x0101) & 0xFC) | 0x01));
					break;
				case IMAGE_V_MIRROR:
					write_cmos_sensor(0x0101,((read_cmos_sensor(0x0101) & 0xFC) | 0x02));
					break;
				case IMAGE_HV_MIRROR:
					write_cmos_sensor(0x0101,((read_cmos_sensor(0x0101) & 0xFC) | 0x03));
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
   
   write_cmos_sensor(0x0136,0x18);
   write_cmos_sensor(0x0137,0x00);
   write_cmos_sensor(0x0101,0x00);
   write_cmos_sensor(0x3424,0x00);
   write_cmos_sensor(0x3000,0x1D);
   write_cmos_sensor(0x3004,0x50);
   write_cmos_sensor(0x300C,0x07);
   write_cmos_sensor(0x300F,0x09);
   write_cmos_sensor(0x3011,0x06);
   write_cmos_sensor(0x3012,0x06);
   write_cmos_sensor(0x3013,0x18);
   write_cmos_sensor(0x3017,0xC1);
   write_cmos_sensor(0x301B,0x40);
   write_cmos_sensor(0x303A,0x0E);
   write_cmos_sensor(0x303B,0x0E);
   write_cmos_sensor(0x304B,0x31);
   write_cmos_sensor(0x304D,0x13);
   write_cmos_sensor(0x3063,0x0E);
   write_cmos_sensor(0x306B,0xF9);
   write_cmos_sensor(0x306C,0xF6);
   write_cmos_sensor(0x306D,0xF0);
   write_cmos_sensor(0x306E,0xCC);
   write_cmos_sensor(0x3087,0x1F);
   write_cmos_sensor(0x308B,0x3F);
   write_cmos_sensor(0x308E,0x09);
   write_cmos_sensor(0x3092,0x13);
   write_cmos_sensor(0x309F,0x3D);
   write_cmos_sensor(0x30B3,0x06);
   write_cmos_sensor(0x30B5,0xB0);
   write_cmos_sensor(0x30BA,0x10);
   write_cmos_sensor(0x30BB,0x10);
   write_cmos_sensor(0x30DE,0x13);
   write_cmos_sensor(0x30E7,0x19);
   write_cmos_sensor(0x30E8,0x06);
   write_cmos_sensor(0x30E9,0x04);
   write_cmos_sensor(0x30EA,0x02);
   write_cmos_sensor(0x30EB,0x01);
   write_cmos_sensor(0x3105,0x30);
   write_cmos_sensor(0x3113,0x40);
   write_cmos_sensor(0x3121,0x20);
   write_cmos_sensor(0x312B,0xC0);
   write_cmos_sensor(0x312C,0x40);
   write_cmos_sensor(0x312F,0x30);
   write_cmos_sensor(0x3137,0x12);
   write_cmos_sensor(0x313C,0x10);
   write_cmos_sensor(0x3157,0x02);
   write_cmos_sensor(0x3226,0x30);
   write_cmos_sensor(0x3280,0x06);
   write_cmos_sensor(0x3281,0x03);
   write_cmos_sensor(0x3282,0x02);
   write_cmos_sensor(0x3286,0x03);
   write_cmos_sensor(0x3287,0x02);
   write_cmos_sensor(0x328C,0x20);
   write_cmos_sensor(0x3307,0x20);
   write_cmos_sensor(0x3308,0x18);
   write_cmos_sensor(0x3309,0x0D);
   write_cmos_sensor(0x338B,0x01);
   write_cmos_sensor(0x338E,0x60);
   write_cmos_sensor(0x3390,0x10);
   write_cmos_sensor(0x3399,0x30);
   write_cmos_sensor(0x33B0,0x11);
   write_cmos_sensor(0x33B1,0x50);
   write_cmos_sensor(0x33C0,0x5A);
   write_cmos_sensor(0x33C1,0x14);
   write_cmos_sensor(0x33C3,0x6D);
   write_cmos_sensor(0x33FF,0x10);
   write_cmos_sensor(0x3424,0x00);
   write_cmos_sensor(0x342A,0x40);
   write_cmos_sensor(0x342B,0x22);
   write_cmos_sensor(0x342C,0x33);
   write_cmos_sensor(0x3480,0x00);
   write_cmos_sensor(0x3500,0x00);
   write_cmos_sensor(0x3144,0x02);
   write_cmos_sensor(0x2B05,0x01);
   write_cmos_sensor(0x2B06,0x01);
   write_cmos_sensor(0x2B0A,0x01);

   write_cmos_sensor(0x2B0A,0x01);
   write_cmos_sensor(0x0100, 0x00); 

		
}    


static void preview_setting(void)
{
    LOG_INF("T4KA7 Load Preview sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);
    
    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x04);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x02);
    write_cmos_sensor(0x0307,0x98);
    write_cmos_sensor(0x030B,0x04);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x08);
    write_cmos_sensor(0x0341,0x34);
    write_cmos_sensor(0x0342,0x0C);
    write_cmos_sensor(0x0343,0x58);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x15);
    write_cmos_sensor(0x0349,0x07);
    write_cmos_sensor(0x034A,0x0E);
    write_cmos_sensor(0x034B,0xA7);
    write_cmos_sensor(0x034C,0x0A);
    write_cmos_sensor(0x034D,0x84);
    write_cmos_sensor(0x034E,0x07);
    write_cmos_sensor(0x034F,0x50);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x0A);
    write_cmos_sensor(0x040D,0x84);
    write_cmos_sensor(0x040E,0x07);
    write_cmos_sensor(0x040F,0x54);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x22);
    write_cmos_sensor(0x2902,0x00);
    write_cmos_sensor(0x3407,0x00);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x30);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x07);
    write_cmos_sensor(0x2821,0xC8);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);
   
   
   
   
   write_cmos_sensor(0x0100,0x01); 
    mdelay(20);
}    

static void preview_24fps_setting(void)
{
    LOG_INF("T4KA7 Load Preview 24fps binning sensor setting \n");
    write_cmos_sensor(0x0100,0x00);
    
    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x04);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x01);
    write_cmos_sensor(0x0307,0xE0);
    write_cmos_sensor(0x030B,0x04);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x07);
    write_cmos_sensor(0x0341,0x6E);
    write_cmos_sensor(0x0342,0x0C);
    write_cmos_sensor(0x0343,0x58);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x15);
    write_cmos_sensor(0x0349,0x07);
    write_cmos_sensor(0x034A,0x0E);
    write_cmos_sensor(0x034B,0xA7);
    write_cmos_sensor(0x034C,0x0A);
    write_cmos_sensor(0x034D,0x84);
    write_cmos_sensor(0x034E,0x07);
    write_cmos_sensor(0x034F,0x50);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x0A);
    write_cmos_sensor(0x040D,0x84);
    write_cmos_sensor(0x040E,0x07);
    write_cmos_sensor(0x040F,0x54);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x22);
    write_cmos_sensor(0x2902,0x01);
    write_cmos_sensor(0x3407,0x00);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x28);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x05);
    write_cmos_sensor(0x2821,0xA0);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);
    write_cmos_sensor(0x0100,0x01); 
    mdelay(20);
}    

static void Capture_16_9_setting(void)
{
    LOG_INF("T4KA7 Load Full_16_9 sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x03);
    write_cmos_sensor(0x0307,0x20);
    write_cmos_sensor(0x030B,0x02);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x0B);
    write_cmos_sensor(0x0341,0xEE);
    write_cmos_sensor(0x0342,0x16);
    write_cmos_sensor(0x0343,0xEC);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x01);
    write_cmos_sensor(0x0347,0x68);
    write_cmos_sensor(0x0348,0x15);
    write_cmos_sensor(0x0349,0x07);
    write_cmos_sensor(0x034A,0x0D);
    write_cmos_sensor(0x034B,0x3F);
    write_cmos_sensor(0x034C,0x15);
    write_cmos_sensor(0x034D,0x08);
    write_cmos_sensor(0x034E,0x0B);
    write_cmos_sensor(0x034F,0xD0);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x15);
    write_cmos_sensor(0x040D,0x08);
    write_cmos_sensor(0x040E,0x0B);
    write_cmos_sensor(0x040F,0xD8);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x11);
    write_cmos_sensor(0x2902,0x00);
    write_cmos_sensor(0x3407,0x00);
   write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x70);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
   write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x12);
    write_cmos_sensor(0x2821,0xC0);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);
    write_cmos_sensor(0x0100,0x01); 
    mdelay(20);
}    

static void capture_setting(kal_uint16 currefps)
{
    if(currefps == 150){
    
    LOG_INF("T4KA7 Load 150FPS Capture sensor setting currefps:%d\n",currefps);

    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x02);
    write_cmos_sensor(0x0307,0x2B);
    write_cmos_sensor(0x030B,0x02);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x0E);
    write_cmos_sensor(0x0341,0xC2);
    write_cmos_sensor(0x0342,0x16);
    write_cmos_sensor(0x0343,0xD8);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x15);
    write_cmos_sensor(0x0349,0x07);
    write_cmos_sensor(0x034A,0x0E);
    write_cmos_sensor(0x034B,0xA7);
    write_cmos_sensor(0x034C,0x14);
    write_cmos_sensor(0x034D,0xC0);
    write_cmos_sensor(0x034E,0x0E);
    write_cmos_sensor(0x034F,0xA4);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x24);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x14);
    write_cmos_sensor(0x040D,0xC0);
    write_cmos_sensor(0x040E,0x0E);
    write_cmos_sensor(0x040F,0xA8);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x11);
    write_cmos_sensor(0x2902,0x00);
    write_cmos_sensor(0x3407,0x00);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x50);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x0C);
    write_cmos_sensor(0x2821,0xF0);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);
    write_cmos_sensor(0x0100,0x01);
    }
   else{
   LOG_INF("T4KA7 Load 300FPS Capture sensor setting currefps:%d\n",currefps);
   write_cmos_sensor(0x0100,0x00);
   
   write_cmos_sensor(0x0301,0x0A);
   write_cmos_sensor(0x0303,0x02);
   write_cmos_sensor(0x0305,0x02);
   write_cmos_sensor(0x0306,0x03);
   write_cmos_sensor(0x0307,0x20);
   write_cmos_sensor(0x030B,0x02);
   write_cmos_sensor(0x030D,0x02);
   write_cmos_sensor(0x030E,0x03);
   write_cmos_sensor(0x030F,0x30);
   write_cmos_sensor(0x0340,0x0E);
   write_cmos_sensor(0x0341,0xC2);
   write_cmos_sensor(0x0342,0x16);
   write_cmos_sensor(0x0343,0xD8);
   write_cmos_sensor(0x0344,0x00);
   write_cmos_sensor(0x0345,0x00);
   write_cmos_sensor(0x0346,0x00);
   write_cmos_sensor(0x0347,0x00);
   write_cmos_sensor(0x0348,0x15);
   write_cmos_sensor(0x0349,0x07);
   write_cmos_sensor(0x034A,0x0E);
   write_cmos_sensor(0x034B,0xA7);
   write_cmos_sensor(0x034C,0x15);
   write_cmos_sensor(0x034D,0x08);
   write_cmos_sensor(0x034E,0x0E);
   write_cmos_sensor(0x034F,0xA4);
   write_cmos_sensor(0x0408,0x00);
   write_cmos_sensor(0x0409,0x00);
   write_cmos_sensor(0x040A,0x00);
   write_cmos_sensor(0x040B,0x04);
   write_cmos_sensor(0x040C,0x15);
   write_cmos_sensor(0x040D,0x08);
   write_cmos_sensor(0x040E,0x0E);
   write_cmos_sensor(0x040F,0xA8);
   write_cmos_sensor(0x0111,0x02);
   write_cmos_sensor(0x0112,0x0A);
   write_cmos_sensor(0x0113,0x0A);
   write_cmos_sensor(0x0114,0x03);
   write_cmos_sensor(0x2900,0x01);
   write_cmos_sensor(0x2901,0x11);
   write_cmos_sensor(0x2902,0x00);
   write_cmos_sensor(0x3407,0x00);
   write_cmos_sensor(0x2800,0x88);
   write_cmos_sensor(0x2801,0x70);
   write_cmos_sensor(0x2802,0x78);
   write_cmos_sensor(0x2803,0x48);
   write_cmos_sensor(0x2804,0x48);
   write_cmos_sensor(0x2805,0x40);
   write_cmos_sensor(0x2806,0x00);
   write_cmos_sensor(0x2807,0x48);
   write_cmos_sensor(0x2808,0x01);
   write_cmos_sensor(0x3439,0x01);
   write_cmos_sensor(0x0202,0x00);
   write_cmos_sensor(0x0203,0x19);
   write_cmos_sensor(0x0204,0x00);
   write_cmos_sensor(0x0205,0x20);
   write_cmos_sensor(0x020E,0x01);
   write_cmos_sensor(0x020F,0x00);
   write_cmos_sensor(0x0210,0x01);
   write_cmos_sensor(0x0211,0x00);
   write_cmos_sensor(0x0212,0x01);
   write_cmos_sensor(0x0213,0x00);
   write_cmos_sensor(0x0214,0x01);
   write_cmos_sensor(0x0215,0x00);
   write_cmos_sensor(0x2B05,0x01);
   write_cmos_sensor(0x2B06,0x01);
   write_cmos_sensor(0x2B0A,0x01);
   write_cmos_sensor(0x2820,0x12);
   write_cmos_sensor(0x2821,0xC0);
   write_cmos_sensor(0x2822,0x00);
   write_cmos_sensor(0x2823,0x00);
   
   
   
   
   write_cmos_sensor(0x0100,0x01);
    }
   
    mdelay(20);
     if (imgsensor.ihdr_en) {

    } else {

    }
    

    mdelay(20);
}

static void normal_video_setting(kal_uint16 currefps)
{

    LOG_INF("T4KA7 Load normal_video:4K sensor setting \n");

    
    write_cmos_sensor(0x0100,0x00);

    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x01);
    write_cmos_sensor(0x0307,0xE0);
    write_cmos_sensor(0x030B,0x02);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x08);
    write_cmos_sensor(0x0341,0x9A);
    write_cmos_sensor(0x0342,0x10);
    write_cmos_sensor(0x0343,0xF8);
    write_cmos_sensor(0x0344,0x03);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x03);
    write_cmos_sensor(0x0347,0x14);
    write_cmos_sensor(0x0348,0x12);
    write_cmos_sensor(0x0349,0x1F);
    write_cmos_sensor(0x034A,0x0B);
    write_cmos_sensor(0x034B,0x93);
    write_cmos_sensor(0x034C,0x0F);
    write_cmos_sensor(0x034D,0x00);
    write_cmos_sensor(0x034E,0x08);
    write_cmos_sensor(0x034F,0x7C);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x04);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x0F);
    write_cmos_sensor(0x040D,0x00);
    write_cmos_sensor(0x040E,0x08);
    write_cmos_sensor(0x040F,0x80);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x11);
    write_cmos_sensor(0x2902,0x00);
    write_cmos_sensor(0x3407,0x00);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x48);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x0B);
    write_cmos_sensor(0x2821,0x40);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);

    write_cmos_sensor(0x0100,0x01);

    mdelay(20);
}

static void hs_video_setting()
{
   LOG_INF("T4KA7 Load HS_VIDEO sensor setting \n");

    
    write_cmos_sensor(0x0100,0x00); 

    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x02);
    write_cmos_sensor(0x0307,0x10);
    write_cmos_sensor(0x030B,0x02);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x04);
    write_cmos_sensor(0x0341,0x8E);
    write_cmos_sensor(0x0342,0x11);
    write_cmos_sensor(0x0343,0xAE);
    write_cmos_sensor(0x0344,0x03);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x03);
    write_cmos_sensor(0x0347,0x1C);
    write_cmos_sensor(0x0348,0x12);
    write_cmos_sensor(0x0349,0x1F);
    write_cmos_sensor(0x034A,0x0B);
    write_cmos_sensor(0x034B,0x8B);
    write_cmos_sensor(0x034C,0x07);
    write_cmos_sensor(0x034D,0x80);
    write_cmos_sensor(0x034E,0x04);
    write_cmos_sensor(0x034F,0x34);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x08);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x07);
    write_cmos_sensor(0x040D,0x80);
    write_cmos_sensor(0x040E,0x04);
    write_cmos_sensor(0x040F,0x38);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x22);
    write_cmos_sensor(0x2902,0x01);
    write_cmos_sensor(0x3407,0x00);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x48);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x0C);
    write_cmos_sensor(0x2821,0x60);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);

    write_cmos_sensor(0x0100,0x01); 
    mdelay(20);
}

static void slim_video_setting()
{
     LOG_INF("T4KA7 Load Slim sensor setting \n");
    
    write_cmos_sensor(0x0100,0x00);

    write_cmos_sensor(0x0301,0x0A);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x02);
    write_cmos_sensor(0x0306,0x02);
    write_cmos_sensor(0x0307,0xA8);
    write_cmos_sensor(0x030B,0x02);
    write_cmos_sensor(0x030D,0x02);
    write_cmos_sensor(0x030E,0x03);
    write_cmos_sensor(0x030F,0x30);
    write_cmos_sensor(0x0340,0x02);
    write_cmos_sensor(0x0341,0xEA);
    write_cmos_sensor(0x0342,0x11);
    write_cmos_sensor(0x0343,0xAE);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x80);
    write_cmos_sensor(0x0346,0x01);
    write_cmos_sensor(0x0347,0xB4);
    write_cmos_sensor(0x0348,0x14);
    write_cmos_sensor(0x0349,0x9F);
    write_cmos_sensor(0x034A,0x0C);
    write_cmos_sensor(0x034B,0xF3);
    write_cmos_sensor(0x034C,0x05);
    write_cmos_sensor(0x034D,0x00);
    write_cmos_sensor(0x034E,0x02);
    write_cmos_sensor(0x034F,0xCC);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x04);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x04);
    write_cmos_sensor(0x040C,0x05);
    write_cmos_sensor(0x040D,0x00);
    write_cmos_sensor(0x040E,0x02);
    write_cmos_sensor(0x040F,0xD0);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x2900,0x01);
    write_cmos_sensor(0x2901,0x24);
    write_cmos_sensor(0x2902,0x01);
    write_cmos_sensor(0x3407,0x01);
    write_cmos_sensor(0x2800,0x88);
    write_cmos_sensor(0x2801,0x60);
    write_cmos_sensor(0x2802,0x78);
    write_cmos_sensor(0x2803,0x48);
    write_cmos_sensor(0x2804,0x48);
    write_cmos_sensor(0x2805,0x40);
    write_cmos_sensor(0x2806,0x00);
    write_cmos_sensor(0x2807,0x48);
    write_cmos_sensor(0x2808,0x01);
    write_cmos_sensor(0x3439,0x01);
    write_cmos_sensor(0x0202,0x00);
    write_cmos_sensor(0x0203,0x19);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x20);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x2B05,0x01);
    write_cmos_sensor(0x2B06,0x01);
    write_cmos_sensor(0x2B0A,0x01);
    write_cmos_sensor(0x2820,0x0F);
    write_cmos_sensor(0x2821,0xF0);
    write_cmos_sensor(0x2822,0x00);
    write_cmos_sensor(0x2823,0x00);

		write_cmos_sensor(0x0100, 0x01);
		mdelay(20);	
		
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        write_cmos_sensor(0x601, 0x02);
    } else {
  
        write_cmos_sensor(0x0601, 0x0);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static const char *t4ka7Vendor = "Toshiba";
static const char *t4ka7NAME = "t4ka7";
static const char *t4ka7Size = "20M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", t4ka7Vendor, t4ka7NAME, t4ka7Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static struct kobject *android_t4ka7;
static int first = true;
static int t4ka7_sysfs_init(void)
{
	int ret ;
	if(first)
	{
		LOG_INF("kobject creat and add\n");
		android_t4ka7 = kobject_create_and_add("android_camera", NULL);
		if (android_t4ka7 == NULL) {
			LOG_INF("subsystem_register failed\n");
			ret = -ENOMEM;
			return ret ;
		}
		LOG_INF("sysfs_create_file\n");
		ret = sysfs_create_file(android_t4ka7, &dev_attr_sensor.attr);
		if (ret) {
			LOG_INF("sysfs_create_file failed\n");
			kobject_del(android_t4ka7);
		}
		else
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
                t4ka7_sysfs_init();
                read_otp();
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
    kal_uint8 retry = 1;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    
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
        retry = 1;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;


    
    sensor_init();
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = imgsensor_info.ihdr_support;
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
    Capture_16_9_setting();
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
     capture_setting(150);
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
    LOG_INF("t4ka7_scenario_id = %d ; preview=0 , capture=1 ,  stereo_mode=11(custim2) ; 16:9_id=10(custim1).\n", scenario_id);
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
            LOG_INF("t4ka7_scenario_id = MSDK_SCENARIO_ID_CUSTOM1\n");
            Custom1(image_window, sensor_config_data); 
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            LOG_INF("t4ka7_scenario_id = MSDK_SCENARIO_ID_CUSTOM2\n");
            Custom2(image_window, sensor_config_data); 
            break;
         case MSDK_SCENARIO_ID_CUSTOM3:
            LOG_INF("t4ka7_scenario_id = MSDK_SCENARIO_ID_CUSTOM3\n");
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
            framerate=600;
            LOG_INF("T4KA7 HS video mode , set  framerate=600 .  \n");
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            framerate=1200;
            LOG_INF("T4KA7 SLIM_VIDEO mode , set  framerate=1200 .  \n");
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
            LOG_INF("T4KA7 set framerate=240 ,  frame_length = 0x%x (should be 0x076C ).  \n", frame_length);
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

UINT32 T4KA7_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    
