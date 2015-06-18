/******************************************************************************
 * mt6575_vibrator.c - MT6575 Android Linux Vibrator Device Driver
 *
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers vibrator relative functions
 *
 ******************************************************************************/
#ifdef CONFIG_VIB_TRIGGERS
#include <linux/vibtrig.h>
#include <linux/slab.h>
#endif
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include "timed_output.h"

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>

#include <mach/mt_typedefs.h>

#include <cust_vibrator.h>
#include <vibrator_hal.h>

#define VERSION					        "v 0.1"
#define VIB_DEVICE				"mtk_vibrator"


#define RSUCCESS        0


#define DBG_EVT_NONE		0x00000000	
#define DBG_EVT_INT			0x00000001	
#define DBG_EVT_TASKLET		0x00000002	

#define DBG_EVT_ALL			0xffffffff

#define DBG_EVT_MASK		(DBG_EVT_TASKLET)

#if 1
#define MSG(evt, fmt, args...) \
do {	\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		printk(fmt, ##args); \
	} \
} while (0)

#define MSG_FUNC_ENTRY(f)	MSG(FUC, "<FUN_ENT>: %s\n", __func__)
#else
#define MSG(evt, fmt, args...) do {} while (0)
#define MSG_FUNC_ENTRY(f)	   do {} while (0)
#endif

#ifdef CONFIG_VIB_TRIGGERS
struct mtk_vib {
	struct timed_output_dev timed_dev;
	struct vib_trigger_enabler enabler;
};
#endif

static struct workqueue_struct *vibrator_queue;
static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int ldo_state;
static int shutdown_flag;


static int vibr_Enable(void)
{
	if (!ldo_state) {
		vibr_Enable_HW();
		printk(KERN_DEBUG "[VIB] %s\n", __func__);
		ldo_state = 1;
	}
	return 0;
}

static int vibr_Disable(void)
{
	if (ldo_state) {
		vibr_Disable_HW();
		printk(KERN_DEBUG "[VIB] %s\n", __func__);
		ldo_state = 0;
	}
	return 0;
}


static void update_vibrator(struct work_struct *work)
{
	if (!vibe_state)
		vibr_Disable();
	else
		vibr_Enable();
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return ktime_to_ms(r);
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

#if 1
	
	struct vibrator_hw *hw = mt_get_cust_vibrator_hw();
#endif

	spin_lock_irqsave(&vibe_lock, flags);
	while (hrtimer_cancel(&vibe_timer)) {
		printk("[VIB] %s: try to cancel hrtimer\n", __func__);
	}

	if (value == 0 || shutdown_flag == 1) {
		if(shutdown_flag)
			printk("[VIB] %s: shutdown_flag = %d\n", __func__, shutdown_flag);
		vibe_state = 0;
	} else {
		if (value < hw->vib_timer)
		{
			printk("[VIB] %s: vibrating time(%d) too short, set to %d\n", __func__, value, hw->vib_timer);
			value = hw->vib_timer;
		}

		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000), HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
	printk("[VIB] %s: %d\n", __func__, value);
	queue_work(vibrator_queue, &vibrator_work);
}

#ifdef CONFIG_VIB_TRIGGERS
static void mtk_vib_trigger_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct mtk_vib *vib;
	struct timed_output_dev *dev;

	vib = enabler->trigger_data;
	dev = &vib->timed_dev;

	printk(KERN_INFO "[VIB] vib_trigger=%d.\r\n", value);

	vibrator_enable(dev, value);
}
#endif

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	queue_work(vibrator_queue, &vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev mtk_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int vib_probe(struct platform_device *pdev)
{
	return 0;
}

static int vib_remove(struct platform_device *pdev)
{
	return 0;
}

static void vib_shutdown(struct platform_device *pdev)
{
	unsigned long flags;
	printk("[VIB] %s: enter!\n", __func__);
	spin_lock_irqsave(&vibe_lock, flags);
	shutdown_flag = 1;
	if (vibe_state) {
		printk("[VIB] %s: vibrator will disable\n", __func__);
		vibe_state = 0;
		vibr_Disable();
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
}

static struct platform_driver vibrator_driver = {
	.probe = vib_probe,
	.remove = vib_remove,
	.shutdown = vib_shutdown,
	.driver = {
		   .name = VIB_DEVICE,
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device vibrator_device = {
	.name = "mtk_vibrator",
	.id = -1,
};

static ssize_t store_vibr_on(struct device *dev, struct device_attribute *attr, const char *buf,
			     size_t size)
{
	if (buf != NULL && size != 0) {
		
		if (buf[0] == '0') {
			vibr_Disable();
		} else {
			vibr_Enable();
		}
	}
	return size;
}

static DEVICE_ATTR(vibr_on, 0220, NULL, store_vibr_on);


static int vib_mod_init(void)
{
	s32 ret;
#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler *enabler;
#endif

	printk("[VIB] %s: MediaTek MTK vibrator driver register, version %s\n", __func__, VERSION);
#ifdef CONFIG_VIB_TRIGGERS
	enabler = kzalloc(sizeof(*enabler), GFP_KERNEL);
	if(!enabler)
		return -ENOMEM;
#endif
	vibr_power_set();	
	ret = platform_device_register(&vibrator_device);
	if (ret != 0) {
		printk("[VIB] %s: Unable to register vibrator device (%d)\n", __func__, ret);
		return ret;
	}

	vibrator_queue = create_singlethread_workqueue(VIB_DEVICE);
	if (!vibrator_queue) {
		printk("[VIB] %s: Unable to create workqueue\n", __func__);
		return -ENODATA;
	}
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	shutdown_flag = 0;
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&mtk_vibrator);

	ret = platform_driver_register(&vibrator_driver);

	if (ret) {
		printk("[VIB] %s: Unable to register vibrator driver (%d)\n", __func__, ret);
		return ret;
	}

	ret = device_create_file(mtk_vibrator.dev, &dev_attr_vibr_on);
	if (ret) {
		printk("[VIB] %s: device_create_file vibr_on fail!\n", __func__);
	}

#ifdef CONFIG_VIB_TRIGGERS
	enabler->name = "mtk-vibrator";
	enabler->default_trigger = "vibrator";
	enabler->enable = mtk_vib_trigger_enable;
	vib_trigger_enabler_register(enabler);
#endif

	printk("[VIB] %s: Done\n", __func__);

	return RSUCCESS;
}


static void vib_mod_exit(void)
{
	printk("[VIB] %s: MediaTek MTK vibrator driver unregister, version %s\n", __func__, VERSION);
	if (vibrator_queue) {
		destroy_workqueue(vibrator_queue);
	}
	printk("[VIB] %s: Done\n", __func__);
}
module_init(vib_mod_init);
module_exit(vib_mod_exit);
MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MTK Vibrator Driver (VIB)");
MODULE_LICENSE("GPL");
