#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include <linux/string.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>
#include <mach/reg_accdet.h>
#include <accdet_custom.h>
#include <accdet_custom_def.h>

#define ACCDET_DEVNAME "accdet"
#define ACCDET_IOC_MAGIC 'A'
#define ACCDET_INIT _IO(ACCDET_IOC_MAGIC,0)
#define SET_CALL_STATE _IO(ACCDET_IOC_MAGIC,1)
#define GET_BUTTON_STATUS _IO(ACCDET_IOC_MAGIC,2)


#define CALL_IDLE 0
#define CALL_RINGING 1
#define CALL_ACTIVE 2
#define KEY_CALL	KEY_SEND
#define KEY_ENDCALL	KEY_HANGEUL

#define ACCDET_TIME_OUT 0x61A80 

enum accdet_report_state
{
    NO_DEVICE =0,
    HEADSET_MIC = 1,
    HEADSET_NO_MIC = 2,
    
    
};

enum accdet_status
{
    PLUG_OUT = 0,
    MIC_BIAS = 1,
    
    HOOK_SWITCH = 2,
    
    
    STAND_BY =4
};

char *accdet_status_string[5]=
{
    "Plug_out",
    "Headset_plug_in",
    
    "Hook_switch",
    
    "Stand_by"
};

char *accdet_report_string[4]=
{
    "No_device",
    "Headset_mic",
    "Headset_no_mic",
    
   
};

enum hook_switch_result
{
    DO_NOTHING =0,
    ANSWER_CALL = 1,
    REJECT_CALL = 2
};

char *button_string[6]=
{
    "NO_key",
    "UP_key",
    "MD_key",
    "no_define_key",
    "DW_key"
};
