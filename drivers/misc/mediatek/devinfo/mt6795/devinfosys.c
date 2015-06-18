#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/xlog.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include "devinfo.h"

#define DEVINFO_TAG "DEVINFO"

extern u32 get_devinfo_with_index(u32 index);
extern u32 g_devinfo_data_size;

 

struct devinfo_driver {    
    struct device_driver driver;    
    const struct platform_device_id *id_table;
};


static struct devinfo_driver dev_info ={
    .driver  = {        
        .name = "dev_info",        
        .bus = &platform_bus_type,        
        .owner = THIS_MODULE,    
   },   
   .id_table = NULL,
};

static ssize_t devinfo_show(struct device_driver *driver, char *buf)
{  
	  unsigned int i;
	  unsigned int *output = (unsigned int *)buf;
	  
	  output[0] = g_devinfo_data_size;

	  for(i = 0; i < g_devinfo_data_size; i++)
	  {
        output[i+1] = get_devinfo_with_index(i);
	  }
    return (g_devinfo_data_size + 1) * sizeof(unsigned int);
}

DRIVER_ATTR(dev_info, 0444, devinfo_show, NULL);

static ssize_t htc_devinfo_show(struct device_driver *driver, char *buf)
{
	unsigned int i;
	ssize_t len, ret = 0;

	for(i = 0; i < g_devinfo_data_size; i++)
	{
		len = snprintf(buf + ret, PAGE_SIZE - ret, "[%d] 0x%x\n",
				i, get_devinfo_with_index(i));
		ret += len;
	}

	return ret;
}

DRIVER_ATTR(htc_dev_info, 0444, htc_devinfo_show, NULL);

static ssize_t htc_speed_bin_show(struct device_driver *driver, char *buf)
{
	unsigned int seg_code = get_devinfo_with_index(24);
	unsigned int speed_bin = (seg_code & (0xF << 24)) >> 24;
	ssize_t size = 0;

	size += snprintf(buf, PAGE_SIZE, "0x%x\n", speed_bin);

	return size;
}

DRIVER_ATTR(htc_speed_bin, 0444, htc_speed_bin_show, NULL);

#define _BITMASK_(_bits_)               (((unsigned) -1 >> (31 - ((1) ? _bits_))) & ~((1U << ((0) ? _bits_)) - 1))
#define _GET_BITS_VAL_(_bits_, _val_)   (((_val_) & (_BITMASK_(_bits_))) >> ((0) ? _bits_))

static ssize_t htc_date_code_show(struct device_driver *driver, char *buf)
{
	unsigned int date_code = get_devinfo_with_index(5);
	ssize_t size = 0;

	date_code = _GET_BITS_VAL_(30 : 30, date_code);
	size += snprintf(buf, PAGE_SIZE, "0x%x\n", date_code);

	return size;
}

DRIVER_ATTR(htc_date_code, 0444, htc_date_code_show, NULL);


static int __init devinfo_init(void)
{
    int ret = 0;

            
    ret = driver_register(&dev_info.driver);        
    if (ret)         
    {            
        printk("fail to register devinfo driver\n");        
    }        

    ret = driver_create_file(&dev_info.driver, &driver_attr_dev_info);        
    if (ret)         
    {           
        printk("[BOOT INIT] Fail to create devinfo sysfs file\n");       
    }
    ret = driver_create_file(&dev_info.driver, &driver_attr_htc_dev_info);
    if (ret)
    {
        printk("[BOOT INIT] Fail to create devinfo sysfs file\n");
    }

    ret = driver_create_file(&dev_info.driver, &driver_attr_htc_speed_bin);
    if (ret)
    {
        printk("[BOOT INIT] Fail to create speed_bin sysfs file\n");
    }

	ret = driver_create_file(&dev_info.driver, &driver_attr_htc_date_code);
    if (ret)
    {
        printk("[BOOT INIT] Fail to create date_code sysfs file\n");
    }
    return 0;
}
module_init(devinfo_init);

