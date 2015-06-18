#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include <mach/upmu_common.h>
#include <linux/htc_flashlight.h>
#include "kd_flashlight.h"
#define TAG_NAME "strobe_main_sid1_part1"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        printk(KERN_WARNING TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      printk(KERN_NOTICE TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        printk(KERN_INFO TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              printk(TAG_NAME "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) printk(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       printk(KERN_ERR TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR PK_ERROR
#endif

static DEFINE_SPINLOCK(g_strobeSMPLock); 
static struct work_struct workTimeOut;
static int g_timeOutTimeMs=0;
static u32 strobe_Res = 0;
static void work_timeOutFunc(struct work_struct *data);

int led1_ma = 0;
int led2_ma = 0;
static int htc_flash_ind[10] =  {50,125,200,275,350,425,500,575,650,750};
static int current_duty = 0;

int htc_led(void)
{
#define FLASH_MODE_START_MA 176  

	PK_DBG("%d %d\n",led1_ma, led2_ma);

#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
	if ((led1_ma < FLASH_MODE_START_MA) && (led2_ma < FLASH_MODE_START_MA))
		htc_torch_main(led1_ma,led2_ma);
	else
		htc_flash_main(led1_ma,led2_ma);
#endif

	return 0;
}


static int FL_Enable(void)
{

	
	led1_ma =  htc_flash_ind[current_duty];
	htc_led();
	
	return 0;
}

static int FL_Disable(void)
{
	
	if (led1_ma ==0)
		return 0;
	led1_ma = 0;
	led2_ma = 0;
	htc_led();
	
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
    
    current_duty = duty;
    
    return 0;
}


static int FL_getPreOnTime(int duty)
{
     return -1;
}

static int FL_preOn(void)
{
    
     return 0;
}


static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    INIT_WORK(&workTimeOut, work_timeOutFunc);
  
   



    return 0;
}
static int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{

    return 0;

}


static int detLowPowerEnd(void)
{

	return 0;
}

static struct hrtimer g_timeOutTimer;

static int g_b1stInit=1;

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static void timerInit(void)
{



	

    
    

    

	if(g_b1stInit==1)
	{
		g_b1stInit=0;


	  	INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; 
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}



}

static int gGetPreOnDuty=0;
static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	kdStrobeDrvArg kdArg;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%lu\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;

		case FLASH_IOC_PRE_ON:
    		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
    		break;

		 case FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY:
            PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY: %d\n",(int)arg);
            gGetPreOnDuty = arg;
            break;

        case FLASH_IOC_GET_PRE_ON_TIME_MS:
    		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
    		temp = FL_getPreOnTime(gGetPreOnDuty);
    		kdArg.arg = temp;
            if(copy_to_user((void __user *) arg , (void*)&kdArg , sizeof(kdStrobeDrvArg)))
            {
                PK_DBG(" ioctl copy to user failed\n");
				PK_DBG(" arg = %lx",arg);
                return -1;
            }
    		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;




    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
        case FLASH_IOC_SET_REG_ADR:
            PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n",(int)arg);
            
            break;
        case FLASH_IOC_SET_REG_VAL:
            PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n",(int)arg);
            
            break;
        case FLASH_IOC_SET_REG:
          

            break;

        case FLASH_IOC_GET_REG:
            PK_DBG("FLASH_IOC_GET_REG: %d\n",(int)arg);

            
            break;

        case FLASH_IOC_HAS_LOW_POWER_DETECT:
    		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
    		temp=FL_hasLowPowerDetect();
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_START:
    		detLowPowerStart();
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_END:
    		i4RetValue = detLowPowerEnd();
    		break;

        default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;


        spin_unlock_irq(&g_strobeSMPLock);
    	FL_Uninit();
    }
    PK_DBG(" Done\n");
    return 0;
}


static FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid1_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}


