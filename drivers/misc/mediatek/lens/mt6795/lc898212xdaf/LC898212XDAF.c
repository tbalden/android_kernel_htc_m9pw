#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "LC898212XDAF.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define LENS_I2C_BUSNUM 0

#define AF_DRVNAME "LC898212XDAF"
#define I2C_SLAVE_ADDRESS        0xE4
#define I2C_REGISTER_ID            0x22
#define PLATFORM_DRIVER_NAME "lens_actuator_lc898212xdaf"
#define AF_DRIVER_CLASS_NAME "actuatordrv_lc898212xdaf"

static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO(AF_DRVNAME, I2C_REGISTER_ID)};

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO,    AF_DRVNAME, "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif

extern char ov13850main_otp_data[21];

static spinlock_t g_AF_SpinLock;

static struct i2c_client * g_pstAF_I2Cclient = NULL;

static dev_t g_AF_devno;
static struct cdev * g_pAF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int    g_s4AF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4AF_INF = 0;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition    = 0;

static int g_sr = 3;

static int ReadEEPROM(u16 addr, u16 *data)
{
    u8 u8data=0;
    u8 pu_send_cmd[2] = {(u8)(addr >> 8) , (u8)(addr & 0xFF) };
    g_pstAF_I2Cclient->addr = (0xA0) >> 1;
    if (i2c_master_send(g_pstAF_I2Cclient, pu_send_cmd, 2) < 0 )
    {
        LOG_INF("[ReadEEPROM] read I2C send failed!!\n");
        return -1;
    }
    if (i2c_master_recv(g_pstAF_I2Cclient, &u8data, 1) < 0)
    {
        LOG_INF("ReadI2C failed!! \n");
        return -1;
    }
    *data = u8data;
    LOG_INF("ReadEEPROM2 0x%x, 0x%x \n", addr, *data);

    return 0;
}

static int ReadI2C(u8 length, u8 addr, u16 *data)
{
    u8 pBuff[2];
    u8 u8data=0;

    g_pstAF_I2Cclient->addr = (I2C_SLAVE_ADDRESS) >> 1;
    if (i2c_master_send(g_pstAF_I2Cclient, &addr, 1) < 0 )
    {
        LOG_INF("[CAMERA SENSOR] read I2C send failed!!\n");
        return -1;
    }

    if(length==0)
    {
        if (i2c_master_recv(g_pstAF_I2Cclient, &u8data, 1) < 0)
        {
            LOG_INF("ReadI2C failed!! \n");
            return -1;
        }
        *data = u8data;
    }
    else if(length==1)
    {
        if (i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2) < 0)
        {
            LOG_INF("ReadI2C 2 failed!! \n");
            return -1;
        }

        *data = (((u16)pBuff[0]) << 8) + ((u16)pBuff[1]);
    }
    LOG_INF("ReadI2C 0x%x, 0x%x, 0x%x \n", length, addr, *data);

    return 0;
}

static int WriteI2C(u8 length, u8 addr, u16 data)
{
    u8 puSendCmd[2] = {addr, (u8)(data&0xFF)};
    u8 puSendCmd2[3] = {addr, (u8)((data>>8)&0xFF), (u8)(data&0xFF)};
    LOG_INF("WriteI2C 0x%x, 0x%x, 0x%x\n", length, addr, data);
	
    g_pstAF_I2Cclient->addr = (I2C_SLAVE_ADDRESS) >> 1;
    g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    if(length==0)
    {
        if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2) < 0)
        {
            LOG_INF("WriteI2C failed!! \n");
            return -1;
        }
    }
    else if(length==1)
    {
        if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 3) < 0)
        {
            LOG_INF("WriteI2C 2 failed!! \n");
            return -1;
        }
    }
    
    return 0;
}

inline static int getAFInfo(__user stLC898212XDAF_MotorInfo * pstMotorInfo)
{
    stLC898212XDAF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4AF_MACRO;
    stMotorInfo.u4InfPosition      = g_u4AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

    if (g_i4MotorStatus == 1)    {stMotorInfo.bIsMotorMoving = 1;}
    else                        {stMotorInfo.bIsMotorMoving = 0;}

    if (g_s4AF_Opened >= 1)    {stMotorInfo.bIsMotorOpen = 1;}
    else                        {stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stLC898212XDAF_MotorInfo)))
    {
        LOG_INF("copy to user failed when getting motor information \n");
    }

    return 0;
}

void LC898212XDAF_init_drv(void)
{
     LOG_INF("Load  LC898212XDAF_MTM_init_drv\n");
     u16 Reg_MTM_0x3C;
     u16 Reg_MTM_0x3C_2;
     
     WriteI2C(0,0x80, 0x34);
     WriteI2C(0,0x81, 0x20);
     WriteI2C(0,0x84, 0xE0);
     WriteI2C(0,0x87, 0x05);
     WriteI2C(0,0xA4, 0x24);
     WriteI2C(0,0x8B, 0x80);

     WriteI2C(1,0x3A,0x0000);
     WriteI2C(1,0x04,0x0000);
     WriteI2C(1,0x02,0x0000);
     WriteI2C(1,0x18,0x0000);
     WriteI2C(1,0x28,0x8080);

     WriteI2C(0,0x83, 0x2C);
     WriteI2C(0,0x84, 0xE3);
     WriteI2C(0,0x97, 0x00);
     WriteI2C(0,0x98, 0x42);
     WriteI2C(0,0x99, 0x00);
     WriteI2C(0,0x9A, 0x00);
     
     WriteI2C(0,0x88, 0x70);
     WriteI2C(0,0x92, 0x00);
     WriteI2C(0,0xA0, 0x01);
     WriteI2C(1,0x7A, 0x6800);
     WriteI2C(1,0x7E, 0x7800);
     WriteI2C(1,0x7C, 0x0100);
     WriteI2C(0,0x93, 0xC0);
     WriteI2C(0,0x86, 0x60);
     WriteI2C(1,0x40, 0x8010);
     WriteI2C(1,0x42, 0x7250);
     WriteI2C(1,0x44, 0x8DF0);
     WriteI2C(1,0x46, 0x6470);
     WriteI2C(1,0x48, 0x5A90);
     WriteI2C(1,0x76, 0x0C50);
     WriteI2C(1,0x4A, 0x2030);
     WriteI2C(1,0x50, 0x04F0);
     WriteI2C(1,0x52, 0x7610);
     WriteI2C(1,0x54, 0x1210);
     WriteI2C(1,0x56, 0x0000);
     WriteI2C(1,0x58, 0x7FF0);
     WriteI2C(1,0x4C, 0x2030);
     WriteI2C(1,0x78, 0x2000);
     WriteI2C(1,0x4E, 0x7FF0);
     WriteI2C(1,0x6E, 0x0000);
     WriteI2C(1,0x72, 0x18E0);
     WriteI2C(1,0x74, 0x4E30);
     WriteI2C(1,0x30, 0x0000);
     WriteI2C(1,0x5A, 0x0680);
     WriteI2C(1,0x5C, 0x72F0);
     WriteI2C(1,0x5E, 0x7F70);
     WriteI2C(1,0x60, 0x7ED0);
     WriteI2C(1,0x62, 0x7FF0);
     WriteI2C(1,0x64, 0x0000);
     WriteI2C(1,0x66, 0x0000);
     WriteI2C(1,0x68, 0x5130);
     WriteI2C(1,0x6A, 0x72F0);
     WriteI2C(1,0x70, 0x0000);
     WriteI2C(1,0x6C, 0x8010);
     WriteI2C(1,0x76, 0x0C50);
     WriteI2C(1,0x78, 0x2000);
     WriteI2C(0,0x28, ov13850main_otp_data[16]); 
     WriteI2C(0,0x29, ov13850main_otp_data[15]); 
     WriteI2C(1,0x30, 0x0000);

     
     WriteI2C(1,0x3A, 0x0000);
     WriteI2C(1,0x04, 0x0000);
     WriteI2C(1,0x02, 0x0000);
     WriteI2C(0,0x85, 0xC0);
     msleep(5);

     
     ReadI2C( 1, 0x3C, &Reg_MTM_0x3C);
     WriteI2C(1, 0x04, Reg_MTM_0x3C);
     WriteI2C(1, 0x18, Reg_MTM_0x3C);
     WriteI2C(0,0x87, 0x85);
     
     WriteI2C(0,0x5A, 0x08);
     WriteI2C(0,0x5B, 0x00);
     WriteI2C(0,0x83, 0xac);
     WriteI2C(0,0xA0, 0x01);

     
     ReadI2C( 1, 0x3C, &Reg_MTM_0x3C_2);
     WriteI2C(1, 0xA1, Reg_MTM_0x3C_2);
     WriteI2C(1, 0x16, 0x0180);
     WriteI2C(0,0x8A, 0x0D);

     msleep(30);
}

int SetVCMPos(u16 _wData)
{
    u16 TargetPos;
    
    u16 ExistentPos = 0;
    int i2cret=0;
    _wData = 1023 - _wData;
    _wData = _wData<<2 ;

    
    if( _wData < 0x800 ) TargetPos =  0x800  - _wData; 
    else                 TargetPos = (0x1800 - _wData)&0xFFF; 
    TargetPos = TargetPos<<4;

    ReadI2C(1, 0x3C, &ExistentPos);
    LOG_INF("SetVCMPos 0x%x 0x%x  \n", TargetPos, ExistentPos);
    if ((signed short)TargetPos > (signed short)ExistentPos)
    {
        WriteI2C(1, 0xA1, TargetPos&0xfff0);
        WriteI2C(1, 0x16, 0x0180);
        
        i2cret = WriteI2C(0, 0x8A, 0x0D);
    }
	else if ((signed short)TargetPos < (signed short)ExistentPos)
    {
        WriteI2C(1, 0xA1, TargetPos&0xfff0);
        WriteI2C(1, 0x16, 0xFE80);
        
        i2cret = WriteI2C(0, 0x8A, 0x0D);
    }
    return i2cret;
}

inline static int moveAF(unsigned long a_u4Position)
{
    int ret = 0;
    if((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF))
    {
        LOG_INF("out of range \n");
        return -EINVAL;
    }

    if (g_s4AF_Opened == 1)
    {
        u16 InitPos;
		LC898212XDAF_init_drv();
		
        ret = ReadI2C(1, 0x3C, &InitPos);

        if(ret == 0)
        {
            LOG_INF("Init Pos %6d \n", InitPos);

            spin_lock(&g_AF_SpinLock);
            g_u4CurrPosition = (unsigned long)InitPos;
            spin_unlock(&g_AF_SpinLock);

        }
        else
        {
            spin_lock(&g_AF_SpinLock);
            g_u4CurrPosition = 0;
            spin_unlock(&g_AF_SpinLock);
        }

        spin_lock(&g_AF_SpinLock);
        g_s4AF_Opened = 2;
        spin_unlock(&g_AF_SpinLock);
		return 0;
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_AF_SpinLock);
        g_i4Dir = 1;
        spin_unlock(&g_AF_SpinLock);
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_AF_SpinLock);
        g_i4Dir = -1;
        spin_unlock(&g_AF_SpinLock);
    }
    else {return 0;}

    spin_lock(&g_AF_SpinLock);
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_AF_SpinLock);

    

    spin_lock(&g_AF_SpinLock);
    g_sr = 3;
    g_i4MotorStatus = 0;
    spin_unlock(&g_AF_SpinLock);

    if(SetVCMPos((u16)g_u4TargetPosition) == 0)
    {
        spin_lock(&g_AF_SpinLock);
        g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
        spin_unlock(&g_AF_SpinLock);
    }
    else
    {
        LOG_INF("set I2C failed when moving the motor \n");

        spin_lock(&g_AF_SpinLock);
        g_i4MotorStatus = -1;
        spin_unlock(&g_AF_SpinLock);
    }

    return 0;
}

inline static int setAFInf(unsigned long a_u4Position)
{
    spin_lock(&g_AF_SpinLock);
    g_u4AF_INF = a_u4Position;
    spin_unlock(&g_AF_SpinLock);
    return 0;
}

inline static int setAFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_AF_SpinLock);
    g_u4AF_MACRO = a_u4Position;
    spin_unlock(&g_AF_SpinLock);
    return 0;
}

static long AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case LC898212XDAFIOC_G_MOTORINFO :
            i4RetValue = getAFInfo((__user stLC898212XDAF_MotorInfo *)(a_u4Param));
        break;
        #ifdef LensdrvCM3
        case LC898212XDAFIOC_G_MOTORMETAINFO :
            i4RetValue = getAFMETA((__user stLC898212XDAF_MotorMETAInfo *)(a_u4Param));
        break;
        #endif
        case LC898212XDAFIOC_T_MOVETO :
            i4RetValue = moveAF(a_u4Param);
        break;

        case LC898212XDAFIOC_T_SETINFPOS :
            i4RetValue = setAFInf(a_u4Param);
        break;

        case LC898212XDAFIOC_T_SETMACROPOS :
            i4RetValue = setAFMacro(a_u4Param);
        break;

        default :
            LOG_INF("No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

static int AF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{


    LOG_INF("Start \n");
    if(g_s4AF_Opened)
    {
        LOG_INF("The device is opened \n");
        return -EBUSY;
    }

    spin_lock(&g_AF_SpinLock);
    g_s4AF_Opened = 1;
    spin_unlock(&g_AF_SpinLock);
    LOG_INF("End \n");

    return 0;
}

static int AF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("Start \n");

    if (g_s4AF_Opened)
    {
        LOG_INF("Free \n");
        g_sr = 5;
        spin_lock(&g_AF_SpinLock);
        g_s4AF_Opened = 0;
        spin_unlock(&g_AF_SpinLock);
    }

    LOG_INF("End \n");

    return 0;
}

static const struct file_operations g_stAF_fops =
{
    .owner = THIS_MODULE,
    .open = AF_Open,
    .release = AF_Release,
    .unlocked_ioctl = AF_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = AF_Ioctl,
#endif
};

inline static int Register_AF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    LOG_INF("Start\n");

    
    if( alloc_chrdev_region(&g_AF_devno, 0, 1,AF_DRVNAME) )
    {
        LOG_INF("Allocate device no failed\n");

        return -EAGAIN;
    }

    
    g_pAF_CharDrv = cdev_alloc();

    if(NULL == g_pAF_CharDrv)
    {
        unregister_chrdev_region(g_AF_devno, 1);

        LOG_INF("Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    
    cdev_init(g_pAF_CharDrv, &g_stAF_fops);

    g_pAF_CharDrv->owner = THIS_MODULE;

    
    if(cdev_add(g_pAF_CharDrv, g_AF_devno, 1))
    {
        LOG_INF("Attatch file operation failed\n");

        unregister_chrdev_region(g_AF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;
    }

    vcm_device = device_create(actuator_class, NULL, g_AF_devno, NULL, AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }

    LOG_INF("End\n");
    return 0;
}

inline static void Unregister_AF_CharDrv(void)
{
    LOG_INF("Start\n");

    
    cdev_del(g_pAF_CharDrv);

    unregister_chrdev_region(g_AF_devno, 1);

    device_destroy(actuator_class, g_AF_devno);

    class_destroy(actuator_class);

    LOG_INF("End\n");
}


static int AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id AF_i2c_id[] = {{AF_DRVNAME, 0},{}};
static struct i2c_driver AF_i2c_driver = {
    .probe = AF_i2c_probe,
    .remove = AF_i2c_remove,
    .driver.name = AF_DRVNAME,
    .id_table = AF_i2c_id,
};

#if 0
static int AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, AF_DRVNAME);
    return 0;
}
#endif
static int AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

static int AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    LOG_INF("Start\n");

    
    g_pstAF_I2Cclient = client;

    g_pstAF_I2Cclient->addr = I2C_SLAVE_ADDRESS;

    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    
    i4RetValue = Register_AF_CharDrv();

    if(i4RetValue){

        LOG_INF(" register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_AF_SpinLock);

    LOG_INF("Attached!! \n");

    return 0;
}

static int AF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&AF_i2c_driver);
}

static int AF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&AF_i2c_driver);
    return 0;
}

static int AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int AF_resume(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver g_stAF_Driver = {
    .probe        = AF_probe,
    .remove    = AF_remove,
    .suspend    = AF_suspend,
    .resume    = AF_resume,
    .driver        = {
        .name    = PLATFORM_DRIVER_NAME,
        .owner    = THIS_MODULE,
    }
};
static struct platform_device g_stAF_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init LC898212XDAF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

    if(platform_device_register(&g_stAF_device)){
        LOG_INF("failed to register AF driver\n");
        return -ENODEV;
    }

    if(platform_driver_register(&g_stAF_Driver)){
        LOG_INF("Failed to register AF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit LC898212XDAF_i2C_exit(void)
{
    platform_driver_unregister(&g_stAF_Driver);
}

module_init(LC898212XDAF_i2C_init);
module_exit(LC898212XDAF_i2C_exit);

MODULE_DESCRIPTION("LC898212XDAF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");


