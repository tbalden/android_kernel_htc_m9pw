/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/printk.h>
#include <linux/usb/htc_attr.h>

#include <linux/musb/mtk_musb.h>

#include "gadget_chips.h"

#include<linux/htc_devices_dtb.h>

enum {
	OS_NOT_YET,
	OS_MAC,
	OS_LINUX,
	OS_WINDOWS,
};

static int mac_mtp_mode;
static int fsg_mode = 0;
static int is_mtp_enable;
static int os_type;
static bool connect2pc;


#include "composite.c"

#include "f_fs.c"
#include "f_audio_source.c"
#include "f_mass_storage.c"
#include "f_adb.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "f_ecm.c"
#include "f_eem.c"
#include "u_ether.c"
#include "f_ncm.c"
#include <linux/usb/htc_info.h>
#include "f_projector.c"
#include "f_projector2.c"

#ifdef CONFIG_EVDO_DT_SUPPORT
#include <mach/viatel_rawbulk.h>
int rawbulk_bind_config(struct usb_configuration *c, int transfer_id);
int rawbulk_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl);
#endif

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

#define VENDOR_ID		0x0BB4
#define PRODUCT_ID		0x0001

#define MANUFACTURER_STRING "MediaTek"
#define PRODUCT_STRING "MT65xx Android Phone"



struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	
	struct list_head enabled_list;

	
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	
	void (*cleanup)(struct android_usb_function *);
	void (*enable)(struct android_usb_function *);
	
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	struct work_struct work;
	char ffs_aliases[256];
	int rezero_cmd;
	bool suspended;
	bool sw_suspended;
	int autobot_mode;
	atomic_t	adb_ready;
	atomic_t	delay_enable_store;

};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);
static int android_setup_config(struct usb_configuration *c, const struct usb_ctrlrequest *ctrl);

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
#ifdef CONFIG_USB_MU3D_DRV
	.bcdUSB               = __constant_cpu_to_le16(0x0300),
#else
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
#endif
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
#ifdef CONFIG_USBIF_COMPLIANCE
	
	.bmAttributes	= USB_CONFIG_ATT_ONE,
	
#else
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
#endif
#ifdef CONFIG_USB_MU3D_DRV
	.MaxPower	= 192, 
#else
	.MaxPower	= 500, 
#endif
};

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };

	
	char *hwdisconnected[2] = { "USB_STATE=HWDISCONNECTED", NULL };
	

	char *rezero_event[2] = { "USB_STATE=REZEROCMD", NULL };
	char *showcdrom_event[2] = { "USB_STATE=SHOWCDROMCMD", NULL };

	char **uevent_envp = NULL;
	char **uevent_envp_cdrom = NULL;
	unsigned long flags;
	
	bool is_hwconnected = true;

	printk("[USB] android_work : sw_suspended %d  suspended %d config %d,connect2pc %d\n",
			dev->sw_suspended, dev->suspended, cdev->config?1:0, connect2pc);
	printk("[USB] android_work : sw_connected %d  connected %d\n", dev->sw_connected, dev->connected);

	if (!cdev){
		return ;
	}

	if(usb_cable_connected())
			is_hwconnected = true;
	else
			is_hwconnected = false;

	pr_debug("[XLOG_INFO][USB]%s: is_hwconnected=%d \n", __func__, is_hwconnected);
	

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
	}

	dev->sw_connected = dev->connected;
	dev->sw_suspended = dev->suspended;

	if (dev->rezero_cmd == 1) {
		uevent_envp_cdrom = rezero_event;
		dev->rezero_cmd = 0;
	} else if (dev->rezero_cmd == 2) {
		uevent_envp_cdrom = showcdrom_event;
		dev->rezero_cmd = 0;
	}

	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_debug("[XLOG_INFO][USB]%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_debug("[XLOG_INFO][USB]%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}

	if (!is_hwconnected) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, hwdisconnected);
		pr_debug("[XLOG_INFO][USB]%s: sent uevent %s\n", __func__, hwdisconnected[0]);
	}

	if (uevent_envp_cdrom) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp_cdrom);
		pr_debug("[XLOG_INFO][USB]%s: sent uevent %s\n", __func__, uevent_envp_cdrom[0]);
	} else {
		pr_debug("[XLOG_INFO][USB]%s: did not send zero uevent\n", __func__);
	}

	if (connect2pc != dev->sw_connected) {
		connect2pc = dev->sw_connected;
		switch_set_state(&cdev->sw_connect2pc, connect2pc ? 1 : 0);
		pr_info("set usb_connect2pc = %d\n", connect2pc);
		if (!connect2pc) {
			pr_info("[USB]%s: unplug cable, so OS_NOT_YET\n", __func__);
			os_type = OS_NOT_YET;
			mtp_update_mode(1);
			fsg_update_mode(0);
			fsg_mode = 0;
		}
	}
	if (dev->connected == 0 && check_htc_mode_status() != NOT_ON_AUTOBOT) {
		htc_mode_enable(0);
		setup_usb_denied(0);
	}
}

static void android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (WARN_ON(!dev->disable_depth))
		return;
	
	pr_debug("%s: disable_depth %d -> %d\n", __func__, dev->disable_depth, dev->disable_depth-1);
	

	if (--dev->disable_depth == 0) {
		usb_add_config(cdev, &android_config_driver,
					android_bind_config);
		usb_gadget_connect(cdev->gadget);
	}
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	
	pr_debug("%s: disable_depth %d -> %d\n", __func__, dev->disable_depth, dev->disable_depth+1);
	

	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &android_config_driver);
	}
}


struct functionfs_config {
	bool opened;
	bool enabled;
	struct ffs_data *data;
};

static int ffs_function_init(struct android_usb_function *f,
			     struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct functionfs_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return functionfs_init();
}

static void ffs_function_cleanup(struct android_usb_function *f)
{
	functionfs_cleanup();
	kfree(f->config);
}

static void ffs_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = true;

	
	if (!config->opened)
		android_disable(dev);
}

static void ffs_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = false;

	
	if (!config->opened)
		android_enable(dev);
}

static int ffs_function_bind_config(struct android_usb_function *f,
				    struct usb_configuration *c)
{
	struct functionfs_config *config = f->config;
	return functionfs_bind_config(c->cdev, c, config->data);
}

static ssize_t
ffs_aliases_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = _android_dev;
	int ret;

	mutex_lock(&dev->mutex);
	ret = sprintf(buf, "%s\n", dev->ffs_aliases);
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t
ffs_aliases_store(struct device *pdev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct android_dev *dev = _android_dev;
	char buff[256];

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	strlcpy(buff, buf, sizeof(buff));
	strlcpy(dev->ffs_aliases, strim(buff), sizeof(dev->ffs_aliases));

	mutex_unlock(&dev->mutex);

	return size;
}

static DEVICE_ATTR(aliases, S_IRUGO | S_IWUSR, ffs_aliases_show,
					       ffs_aliases_store);
static struct device_attribute *ffs_function_attributes[] = {
	&dev_attr_aliases,
	NULL
};

static struct android_usb_function ffs_function = {
	.name		= "ffs",
	.init		= ffs_function_init,
	.enable		= ffs_function_enable,
	.disable	= ffs_function_disable,
	.cleanup	= ffs_function_cleanup,
	.bind_config	= ffs_function_bind_config,
	.attributes	= ffs_function_attributes,
};

static int functionfs_ready_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;
	int ret = 0;

	mutex_lock(&dev->mutex);

	ret = functionfs_bind(ffs, dev->cdev);
	if (ret)
		goto err;

	config->data = ffs;
	config->opened = true;

	if (config->enabled)
		android_enable(dev);
	
	atomic_set(&dev->adb_ready, 1);
	while (atomic_read(&dev->delay_enable_store) != 0) {
		android_enable(dev);
		dev->enabled = true;
		atomic_sub(1, &dev->delay_enable_store);
	}
	

err:
	mutex_unlock(&dev->mutex);
	return ret;
}

static void functionfs_closed_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;

	mutex_lock(&dev->mutex);

	if (config->enabled)
		android_disable(dev);

	config->opened = false;
	config->data = NULL;

	functionfs_unbind(ffs);

	mutex_unlock(&dev->mutex);
}

static void *functionfs_acquire_dev_callback(const char *dev_name)
{
	return 0;
}

static void functionfs_release_dev_callback(struct ffs_data *ffs_data)
{
}

struct adb_data {
	bool opened;
	bool enabled;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

	
	if (!data->opened)
		android_disable(dev);
#endif
}

static void adb_android_function_disable(struct android_usb_function *f)
{
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	
	if (!data->opened)
		android_enable(dev);
#endif
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

static void adb_ready_callback(void)
{
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
#endif
}

static void adb_closed_callback(void)
{
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
#endif
}

static unsigned hsm_newpid = 1;
module_param(hsm_newpid, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hsm_newpid, "Use New PID for HSM ACM");

#define MAX_ACM_INSTANCES 5
int acm_port_num = MAX_ACM_INSTANCES;

struct acm_function_config {
	int instances;
	int instances_on;
	struct usb_function *f_acm[MAX_ACM_INSTANCES];
	struct usb_function_instance *f_acm_inst[MAX_ACM_INSTANCES];
	int port_index[MAX_ACM_INSTANCES];
	int port_index_on[MAX_ACM_INSTANCES];
};

static int
acm_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct acm_function_config *config;
#ifdef CONFIG_USBIF_COMPLIANCE
	
	return 0;
#endif

	config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		config->f_acm_inst[i] = usb_get_function_instance("acm");
		if (IS_ERR(config->f_acm_inst[i])) {
			ret = PTR_ERR(config->f_acm_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_acm[i] = usb_get_function(config->f_acm_inst[i]);
		if (IS_ERR(config->f_acm[i])) {
			ret = PTR_ERR(config->f_acm[i]);
			goto err_usb_get_function;
		}
	}
	return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_acm[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_acm_inst[i]);
	}
	return ret;
}

static void acm_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct acm_function_config *config = f->config;
#ifdef CONFIG_USBIF_COMPLIANCE
	
	return 0;
#endif

	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		usb_put_function(config->f_acm[i]);
		usb_put_function_instance(config->f_acm_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
acm_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	
	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		if(config->port_index[i] != 0) {
			ret = usb_add_function(c, config->f_acm[i]);
			if (ret) {
				pr_err("Could not bind acm%u config\n", i);
				goto err_usb_add_function;
			}
			pr_info("%s Open /dev/ttyGS%d\n", __func__, i);
			
			if (i != 4)
				config->port_index[i] = 0;
			config->port_index_on[i] = 1;
			config->instances = 0;
		}
	}

	config->instances_on = config->instances;
	for (i = 0; i < config->instances_on; i++) {
		ret = usb_add_function(c, config->f_acm[i]);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			goto err_usb_add_function;
		}
		pr_info("%s Open /dev/ttyGS%d\n", __func__, i);
	}

	return 0;

err_usb_add_function:
	while (i-- > 0)
		usb_remove_function(c, config->f_acm[i]);
	return ret;
}

static void acm_function_unbind_config(struct android_usb_function *f,
				       struct usb_configuration *c)
{

	
	


	return;

}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show,
						 acm_instances_store);

static ssize_t acm_port_index_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d,%d,%d,%d,%d\n", config->port_index[0], config->port_index[1],
		config->port_index[2], config->port_index[3], config->port_index[4]);
}

static ssize_t acm_port_index_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int val[MAX_ACM_INSTANCES]={0};
	int num = 0;
	int tmp = 0;
	num = sscanf(buf, "%d,%d,%d,%d,%d", &(val[0]), &(val[1]), &(val[2]), &(val[3]), &(val[4]));

	pr_debug("%s [0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d, num=%d\n", __func__, val[0], val[1], \
								val[2], val[3], val[4], num);
	
	for (tmp = 0; tmp < MAX_ACM_INSTANCES; tmp ++)
		config->port_index[tmp] = 0;

	for (tmp = 0; tmp < num; tmp++) {
		int port = (val[tmp] > MAX_ACM_INSTANCES || val[tmp] < 1) ? 0 : val[tmp]-1;
		config->port_index[port] = 1;
	}

	return size;
}

static DEVICE_ATTR(port_index, S_IRUGO | S_IWUSR, acm_port_index_show,
						 acm_port_index_store);

static struct device_attribute *acm_function_attributes[] = {
	&dev_attr_instances,
	&dev_attr_port_index, 
	NULL
};

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.unbind_config	= acm_function_unbind_config,
	.attributes	= acm_function_attributes,
};

struct ncm_function_config {
	u8      ethaddr[ETH_ALEN];
	struct eth_dev *dev;
};
static int
ncm_function_init(struct android_usb_function *f, struct usb_composite_dev *c)
{
	f->config = kzalloc(sizeof(struct ncm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return 0;
}

static void ncm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
ncm_function_bind_config(struct android_usb_function *f,
				struct usb_configuration *c)
{
	struct ncm_function_config *ncm = f->config;
	int ret;
	struct eth_dev *dev;

	if (!ncm) {
		pr_err("%s: ncm config is null\n", __func__);
		return -EINVAL;
	}
	if (c->cdev->gadget)
		c->cdev->gadget->miMaxMtu = ETH_FRAME_LEN_MAX - ETH_HLEN;

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);

	if (ncm->ethaddr[0])
		dev = gether_setup_name(c->cdev->gadget, NULL, "ncm");
	else
		dev = gether_setup_name(c->cdev->gadget, ncm->ethaddr, "ncm");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether setup failed err:%d\n", __func__, ret);
		return ret;
	}
	ncm->dev = dev;

	ret = ncm_bind_config(c, ncm->ethaddr, dev);
	if (ret) {
		pr_err("%s: ncm bind config failed err:%d", __func__, ret);
		gether_cleanup(dev);
		return ret;
	}

	return ret;
}

static void ncm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct ncm_function_config *ncm = f->config;
	if (c->cdev->gadget)
		c->cdev->gadget->miMaxMtu = 0;
	gether_cleanup(ncm->dev);
}

static ssize_t ncm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);
}

static ssize_t ncm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ncm->ethaddr[0], (int *)&ncm->ethaddr[1],
		    (int *)&ncm->ethaddr[2], (int *)&ncm->ethaddr[3],
		    (int *)&ncm->ethaddr[4], (int *)&ncm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ncm_ethaddr, S_IRUGO | S_IWUSR, ncm_ethaddr_show,
					       ncm_ethaddr_store);
static struct device_attribute *ncm_function_attributes[] = {
	&dev_attr_ncm_ethaddr,
	NULL
};

static struct android_usb_function ncm_function = {
	.name		= "cdc_network",
	.init		= ncm_function_init,
	.cleanup	= ncm_function_cleanup,
	.bind_config	= ncm_function_bind_config,
	.unbind_config	= ncm_function_unbind_config,
	.attributes	= ncm_function_attributes,
};

#ifdef CONFIG_USB_F_LOOPBACK

#define MAX_LOOPBACK_INSTANCES 1

struct loopback_function_config {
	int port_num;
	struct usb_function *f_lp[MAX_LOOPBACK_INSTANCES];
	struct usb_function_instance *f_lp_inst[MAX_LOOPBACK_INSTANCES];
};

static int loopback_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct loopback_function_config *config;

	config = kzalloc(sizeof(struct loopback_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	for (i = 0; i < MAX_LOOPBACK_INSTANCES; i++) {
		config->f_lp_inst[i] = usb_get_function_instance("loopback");
		if (IS_ERR(config->f_lp_inst[i])) {
			ret = PTR_ERR(config->f_lp_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_lp[i] = usb_get_function(config->f_lp_inst[i]);
		if (IS_ERR(config->f_lp[i])) {
			ret = PTR_ERR(config->f_lp[i]);
			goto err_usb_get_function;
		}
	}
	return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_lp[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_lp_inst[i]);
	}
	return ret;
}

static void loopback_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct loopback_function_config *config = f->config;

	for (i = 0; i < MAX_LOOPBACK_INSTANCES; i++) {
		usb_put_function(config->f_lp[i]);
		usb_put_function_instance(config->f_lp_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
loopback_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	int i = 0;
	int ret = 0;
	struct loopback_function_config *config = f->config;

	ret = usb_add_function(c, config->f_lp[config->port_num]);
	if (ret) {
		pr_err("Could not bind loopback%u config\n", config->port_num);
		goto err_usb_add_function;
	}
	pr_info("%s Open loopback\n", __func__);

	return 0;

err_usb_add_function:
	while (i-- > 0)
		usb_remove_function(c, config->f_lp[i]);
	return ret;
}

static struct android_usb_function loopback_function = {
	.name		= "loopback",
	.init		= loopback_function_init,
	.cleanup	= loopback_function_cleanup,
	.bind_config	= loopback_function_bind_config,
};
#endif

#define MAX_SERIAL_INSTANCES 10

enum fserial_func_type {
	USB_FSER_FUNC_NONE,
	USB_FSER_FUNC_SERIAL,
	USB_FSER_FUNC_MODEM,
	USB_FSER_FUNC_MODEM_MDM,
	USB_FSER_FUNC_ACM,
	USB_FSER_FUNC_AUTOBOT,
};

enum fserial_func_type serial_str_to_func_type(const char *name)
{
	if (!name)
		return USB_FSER_FUNC_NONE;

	if (!strcasecmp("MODEM", name))
		return USB_FSER_FUNC_MODEM;
	if (!strcasecmp("MODEM_MDM", name))
		return USB_FSER_FUNC_MODEM_MDM;
	if (!strcasecmp("SERIAL", name))
		return USB_FSER_FUNC_SERIAL;
	if (!strcasecmp("ACM", name))
		return USB_FSER_FUNC_ACM;
	if (!strcasecmp("AUTOBOT", name))
		return USB_FSER_FUNC_AUTOBOT;

	return USB_FSER_FUNC_NONE;
}

static struct serial_pool {
	struct usb_function *usb_serial_function;
	enum fserial_func_type serial_func_type;
} usb_serial_pool[MAX_SERIAL_INSTANCES];


struct serial_function_config {
	int port_num;
	struct usb_function *f_ser[MAX_SERIAL_INSTANCES];
	struct usb_function_instance *f_ser_inst[MAX_SERIAL_INSTANCES];
};

static char serial_transports[108];	
static int no_ports = 0;

static int serial_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct serial_function_config *config;
#ifdef CONFIG_USBIF_COMPLIANCE
	
	return 0;
#endif
	struct android_dev *dev = _android_dev;
	char buf[128], *b, *name, *pname, *temp;
	int err = -1;

	config = kzalloc(sizeof(struct serial_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	strcpy(serial_transports, "tty:modem,tty:autobot,tty:serial,tty:autobot");
	strlcpy(buf, serial_transports, sizeof(buf));
	b = strim(buf);
	printk("[USB] %s : serial_init_string %s\n", __func__, b);
	while (b) {
		temp = strsep(&b, ",");
		if (temp) {
			name = strsep(&temp, ":");
			if (name)
				pname = strsep(&temp, ":");
		}
		printk("[USB] %s : loop %d %s %s\n", __func__, no_ports, name, pname);
				
		if (pname && !strcmp("acm",pname))
			continue;
		if (name) {
			usb_serial_pool[no_ports].serial_func_type = serial_str_to_func_type(pname);
			no_ports++;
			if (no_ports >= MAX_SERIAL_INSTANCES) {
				pr_err("serial: max no_ports reached '%s'", name);
				goto out;
			}
		}
	}

	for (i = 0; i < no_ports; i++) {
		config->f_ser_inst[i] = usb_get_function_instance("gser");
		if (IS_ERR(config->f_ser_inst[i])) {
		err = PTR_ERR(config->f_ser_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_ser[i] = usb_get_function(config->f_ser_inst[i]);

		usb_serial_pool[i].usb_serial_function = config->f_ser[i];
		if (IS_ERR(config->f_ser[i])) {
			err = PTR_ERR(config->f_ser[i]);
			goto err_usb_get_function;
		}
	}

		return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_ser[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_ser_inst[i]);
	}
	return ret;
out:
	return err;
}

static void serial_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct serial_function_config *config = f->config;
#ifdef CONFIG_USBIF_COMPLIANCE
	
	return 0;
#endif

	for (i = 0; i < MAX_SERIAL_INSTANCES; i++) {
		usb_put_function(config->f_ser[i]);
		usb_put_function_instance(config->f_ser_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
serial_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{

	int ret = 0, i = 0, car_mode = _android_dev->autobot_mode;
	struct serial_function_config *config = f->config;

	for(i = 0; i < no_ports; i++) {
		if ((usb_serial_pool[i].serial_func_type == USB_FSER_FUNC_SERIAL) ||
			(car_mode && usb_serial_pool[i].serial_func_type == USB_FSER_FUNC_AUTOBOT)) {
			ret = usb_add_function(c, config->f_ser[i]);
			if (ret) {
				pr_err("Could not bind ser%u config\n", i);
				goto err_usb_add_function;
			}
		}
	}
	pr_info("%s Open /dev/ttyGS%d\n", __func__, config->port_num);

	return 0;

err_usb_add_function:
	if (usb_serial_pool[i].serial_func_type == USB_FSER_FUNC_SERIAL)
	while (i-- > 0)
		usb_remove_function(c, config->f_ser[i]);
	return ret;
}

static ssize_t serial_port_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct serial_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->port_num);
}

static ssize_t serial_port_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct serial_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_SERIAL_INSTANCES)
		value = MAX_SERIAL_INSTANCES;
	config->port_num = value;
	return size;
}

static DEVICE_ATTR(port, S_IRUGO | S_IWUSR, serial_port_show, serial_port_store);
static struct device_attribute *serial_function_attributes[] = { &dev_attr_port, NULL };

static struct android_usb_function serial_function = {
	.name		= "serial",
	.init		= serial_function_init,
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

static int modem_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int  i, err;

	for (i = 0; i < no_ports; i++) {
		if (usb_serial_pool[i].serial_func_type == USB_FSER_FUNC_MODEM) {
			err = usb_add_function(c, usb_serial_pool[i].usb_serial_function);
			if (err) {
				pr_err("Could not bind gser%u config\n", i);
				while (i-- > 0) {
					if (usb_serial_pool[i].serial_func_type == USB_FSER_FUNC_MODEM)
						usb_remove_function(c, usb_serial_pool[i].usb_serial_function);
				}
				return err;
			}
		}
	}
	return 0;

}

static struct android_usb_function modem_function = {
	.name       = "modem",
	.cleanup    = serial_function_cleanup,
	.bind_config    = modem_function_bind_config,
};



static int
mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int
mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int
ptp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	
}

static int
ptp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	
	struct android_dev		*dev = _android_dev;
	struct android_usb_function	*f_count;
	int	   functions_no=0;
	char   usb_function_string[32];
	char * buff = usb_function_string;

	list_for_each_entry(f_count, &dev->enabled_functions, enabled_list)
	{
		functions_no++;
		buff += sprintf(buff, "%s,", f_count->name);
	}
	*(buff-1) = '\n';

	mtp_read_usb_functions(functions_no, usb_function_string);
	return mtp_ctrlrequest(cdev, c);
}

static ssize_t mtp_open_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", htc_mtp_open_state);
}

static DEVICE_ATTR(mtp_open_state, S_IRUGO, mtp_open_state_show, NULL);
static struct device_attribute *mtp_function_attributes[] = {
	&dev_attr_mtp_open_state,
	NULL
};
#if(0)
static int ptp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return ptp_ctrlrequest(cdev, c);
}
#endif
static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
	.attributes	= mtp_function_attributes,
};

static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
#if(0)
	.ctrlrequest	= ptp_function_ctrlrequest, 
#endif
};

struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
	struct eth_dev *dev;
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -1;
	}


	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
	dev = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	ecm->dev = dev;

	return ecm_bind_config(c, ecm->ethaddr, ecm->dev);
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct ecm_function_config *ecm = f->config;
	gether_cleanup(ecm->dev);
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ecm_ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ecm_ethaddr,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "cdc_ethernet",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
};

struct eem_function_config {
	u8      ethaddr[ETH_ALEN];
	char	manufacturer[256];
	struct eth_dev *dev;
};

static int
eem_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct eem_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void eem_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
eem_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct eem_function_config *eem = f->config;

	pr_debug("[XLOG_DEBUG][USB]%s: \n", __func__);

	if (!eem) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		eem->ethaddr[0], eem->ethaddr[1], eem->ethaddr[2],
		eem->ethaddr[3], eem->ethaddr[4], eem->ethaddr[5]);

	dev = gether_setup_name(c->cdev->gadget, eem->ethaddr, "rndis");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	eem->dev = dev;

	return eem_bind_config(c, eem->dev);
}

static void eem_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct eem_function_config *eem = f->config;
	gether_cleanup(eem->dev);
}

static ssize_t eem_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct eem_function_config *eem = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		eem->ethaddr[0], eem->ethaddr[1], eem->ethaddr[2],
		eem->ethaddr[3], eem->ethaddr[4], eem->ethaddr[5]);
}

static ssize_t eem_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct eem_function_config *eem = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&eem->ethaddr[0], (int *)&eem->ethaddr[1],
		    (int *)&eem->ethaddr[2], (int *)&eem->ethaddr[3],
		    (int *)&eem->ethaddr[4], (int *)&eem->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(eem_ethaddr, S_IRUGO | S_IWUSR, eem_ethaddr_show,
					       eem_ethaddr_store);

static struct device_attribute *eem_function_attributes[] = {
	&dev_attr_eem_ethaddr,
	NULL
};

static struct android_usb_function eem_function = {
	.name		= "eem",
	.init		= eem_function_init,
	.cleanup	= eem_function_cleanup,
	.bind_config	= eem_function_bind_config,
	.unbind_config	= eem_function_unbind_config,
	.attributes	= eem_function_attributes,
};

struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	
	bool	wceis;
	struct eth_dev *dev;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct rndis_function_config *rndis = f->config;

	pr_debug("[XLOG_DEBUG][USB]%s: \n", __func__);

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	dev = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	rndis->dev = dev;

	if (rndis->wceis) {
		
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer, rndis->dev);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct rndis_function_config *rndis = f->config;
	gether_cleanup(rndis->dev);
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

#ifdef CONFIG_MTK_MULTI_STORAGE_SUPPORT
#define LUN_MULTI (1)
#else
#define LUN_MULTI (0)
#endif

#ifdef CONFIG_MTK_SHARED_SDCARD
#define LUN_SHARED_SD (-1)
#else
#define LUN_SHARED_SD (0)
#endif

#ifdef CONFIG_MTK_ICUSB_SUPPORT
#define LUN_ICUSB (1)
#else
#define LUN_ICUSB (0)
#endif

#define LUN_NUM LUN_MULTI + LUN_SHARED_SD + LUN_ICUSB + 1
		
	config->fsg.product_name = "Android Phone";
	config->fsg.vendor_name = "HTC";

	config->fsg.nluns = LUN_NUM;
	for(i = 0; i < config->fsg.nluns; i++) {
		if (i == config->fsg.nluns - 1) {
			config->fsg.luns[i].cdrom = 1;
			config->fsg.luns[i].removable = 1;
			config->fsg.luns[i].nofua = 1;
		} else {
			config->fsg.luns[i].cdrom = 0;
			config->fsg.luns[i].removable = 1;
			config->fsg.luns[i].nofua = 1;
		}
	}
	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}
	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun0");
	if (err) {
		kfree(config);
		return err;
	}

	for(i = 1; i < config->fsg.nluns; i++) {
		char string_lun[5]={0};

		sprintf(string_lun, "lun%d",i);

		err = sysfs_create_link(&f->dev->kobj,
				&common->luns[i].dev.kobj,
				string_lun);
		if (err) {
			kfree(config);
			return err;
		}
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

#ifdef CONFIG_MTK_BICR_SUPPORT

static ssize_t mass_storage_bicr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->common->bicr);
}

static ssize_t mass_storage_bicr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->bicr))
		return -EINVAL;
	if (sscanf(buf, "%d", &config->common->bicr) != 1)
		return -EINVAL;

	
	if (!strcmp(buf, "1"))
		config->common->luns[0].cdrom = 1;
	else {
		
		config->common->luns[0].cdrom = 0;
		config->common->luns[0].blkbits = 0;
		config->common->luns[0].blksize = 0;
		config->common->luns[0].num_sectors = 0;
	}

	return size;
}

static DEVICE_ATTR(bicr, S_IRUGO | S_IWUSR,
					mass_storage_bicr_show,
					mass_storage_bicr_store);

#endif

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
#ifdef CONFIG_MTK_BICR_SUPPORT
	&dev_attr_bicr,
#endif
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

#ifdef CONFIG_EVDO_DT_SUPPORT
static int rawbulk_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return 0;
}

static void rawbulk_function_cleanup(struct android_usb_function *f)
{
	;
}

static int rawbulk_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
    char *i = f->name + strlen("via_");
    if (!strncmp(i, "modem", 5))
        return rawbulk_bind_config(c, RAWBULK_TID_MODEM);
    else if (!strncmp(i, "ets", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_ETS);
    else if (!strncmp(i, "atc", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_AT);
    else if (!strncmp(i, "pcv", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_PCV);
    else if (!strncmp(i, "gps", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_GPS);
    return -EINVAL;
}

static int rawbulk_function_modem_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
    if ((c->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE &&
            (c->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
        struct rawbulk_function *fn = rawbulk_lookup_function(RAWBULK_TID_MODEM);
        return rawbulk_function_setup(&fn->function, c);
    }
    return -1;
}

static struct android_usb_function rawbulk_modem_function = {
	.name		= "via_modem",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
	.ctrlrequest	= rawbulk_function_modem_ctrlrequest,
};

static struct android_usb_function rawbulk_ets_function = {
	.name		= "via_ets",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_atc_function = {
	.name		= "via_atc",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_pcv_function = {
	.name		= "via_pcv",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_gps_function = {
	.name		= "via_gps",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};
#endif

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

static int projector_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct htcmode_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector_setup(f->config);
}

static void projector_function_cleanup(struct android_usb_function *f)
{
	projector_cleanup();
	kfree(f->config);
}

static int projector_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector_bind_config(c);
}


static ssize_t projector_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.width);
}

static DEVICE_ATTR(width, S_IRUGO | S_IWUSR, projector_width_show,
						    NULL);

static ssize_t projector_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.height);
}

static DEVICE_ATTR(height, S_IRUGO | S_IWUSR, projector_height_show,
						    NULL);

static ssize_t projector_rotation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", (config->client_info.display_conf & CLIENT_INFO_SERVER_ROTATE_USED));
}

static DEVICE_ATTR(rotation, S_IRUGO | S_IWUSR, projector_rotation_show,
						    NULL);

static ssize_t projector_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->version);
}

static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, projector_version_show,
						    NULL);

static ssize_t projector_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->vendor);
}

static DEVICE_ATTR(vendor, S_IRUGO | S_IWUSR, projector_vendor_show,
						    NULL);

static ssize_t projector_server_nonce_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->nonce, HSML_SERVER_NONCE_SIZE);
	return HSML_SERVER_NONCE_SIZE;
}

static DEVICE_ATTR(server_nonce, S_IRUGO | S_IWUSR, projector_server_nonce_show,
						    NULL);

static ssize_t projector_client_sig_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->client_sig, HSML_CLIENT_SIG_SIZE);
	return HSML_CLIENT_SIG_SIZE;
}

static DEVICE_ATTR(client_sig, S_IRUGO | S_IWUSR, projector_client_sig_show,
						    NULL);

static ssize_t projector_server_sig_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(config->server_sig, buff, HSML_SERVER_SIG_SIZE);
	return HSML_SERVER_SIG_SIZE;
}

static DEVICE_ATTR(server_sig, S_IWUSR, NULL,
		projector_server_sig_store);

static ssize_t projector_auth_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(&config->auth_result, buff, sizeof(config->auth_result));
	config->auth_in_progress = 0;
	return sizeof(config->auth_result);
}

static DEVICE_ATTR(auth, S_IWUSR, NULL,
		projector_auth_store);

static ssize_t projector_debug_mode_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	int value, i;
	int framesize = DEFAULT_PROJ_HEIGHT * DEFAULT_PROJ_WIDTH;

	if (sscanf(buff, "%d", &value) == 1) {

		if (!test_frame)
			test_frame = kzalloc(framesize * 2, GFP_KERNEL);

		if (test_frame)
			for (i = 0 ; i < framesize ; i++)
				if (i < framesize/4)
					test_frame[i] = 0xF800;
				else if (i < framesize*2/4)
					test_frame[i] = 0x7E0;
				else if (i < framesize*3/4)
					test_frame[i] = 0x1F;
				else
					test_frame[i] = 0xFFFF;

		config->debug_mode = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(debug_mode, S_IWUSR, NULL,
		projector_debug_mode_store);

static struct device_attribute *projector_function_attributes[] = {
	&dev_attr_width,
	&dev_attr_height,
	&dev_attr_rotation,
	&dev_attr_version,
	&dev_attr_vendor,
	&dev_attr_server_nonce,
	&dev_attr_client_sig,
	&dev_attr_server_sig,
	&dev_attr_auth,
	&dev_attr_debug_mode,
	NULL
};


struct android_usb_function projector_function = {
	.name		= "projector",
	.init		= projector_function_init,
	.cleanup	= projector_function_cleanup,
	.bind_config	= projector_function_bind_config,
	.attributes = projector_function_attributes
};

static int projector2_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct hsml_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector2_setup(f->config);
}

static void projector2_function_cleanup(struct android_usb_function *f)
{

	projector2_cleanup();

	if (f->config) {
		kfree(f->config);
		f->config = NULL;
	}
}

static int projector2_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector2_bind_config(c);
}

static ssize_t projector2_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.wWidth);
}

#if HSML_VERSION_12
#define cHSML_WIDTH_SIZE        2

static ssize_t projector2_width_store(struct device *dev,
        struct device_attribute *attr, const char *buff, size_t size)
{
struct android_usb_function *f = dev_get_drvdata(dev);
struct hsml_protocol *config = f->config;
u16 uValue;
u8 aucWidth[cHSML_WIDTH_SIZE];

    if (size <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, buff, size);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wWidth = uValue;
        return size;
    } else {
        printk(KERN_ERR "%s: size is invalid %d/%d\n", __func__, (int)size, cHSML_WIDTH_SIZE);
    }
    return -EINVAL;
}
#endif

static DEVICE_ATTR(client_width, S_IRUGO | S_IWUSR, projector2_width_show,
#if !HSML_VERSION_12
                            NULL
#else
                            projector2_width_store
#endif
        );

static ssize_t projector2_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.wHeight);
}

#if HSML_VERSION_12
static ssize_t projector2_height_store(struct device *dev,
        struct device_attribute *attr, const char *buff, size_t size)
{
struct android_usb_function *f = dev_get_drvdata(dev);
struct hsml_protocol *config = f->config;
u16 uValue;
u8 aucWidth[cHSML_WIDTH_SIZE];

    if (size <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, buff, size);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wHeight = uValue;
        return size;
    } else {
        printk(KERN_ERR "%s: size is invalid %d/%d\n", __func__, (int)size, cHSML_WIDTH_SIZE);
    }

    return -EINVAL;
}
#endif

static DEVICE_ATTR(client_height, S_IRUGO | S_IWUSR, projector2_height_show,
#if !HSML_VERSION_12
                            NULL
#else
                            projector2_height_store
#endif
                            );

static ssize_t projector2_maxfps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->MaxFPS);
}

static DEVICE_ATTR(client_maxfps, S_IRUGO | S_IWUSR, projector2_maxfps_show,
						    NULL);

static ssize_t projector2_pixel_format_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.bPixelFormat);
}

static DEVICE_ATTR(client_pixel_format, S_IRUGO | S_IWUSR, projector2_pixel_format_show,
						    NULL);

static DEVICE_ATTR(client_context_info, S_IRUGO | S_IWUSR, NULL, context_info_store);
#if HSML_VERSION_12
static DEVICE_ATTR(client_ver, S_IRUGO | S_IWUSR, projector2_ver_show, NULL);
static DEVICE_ATTR(client_cap, S_IRUGO | S_IWUSR, projector2_cap_show, NULL);
static DEVICE_ATTR(client_uuid, S_IRUGO | S_IWUSR, NULL, projector2_uuid_store);
#endif

static struct device_attribute *projector2_function_attributes[] = {
	&dev_attr_client_width,
	&dev_attr_client_height,
	&dev_attr_client_maxfps,
	&dev_attr_client_pixel_format,
	&dev_attr_client_context_info,
#if HSML_VERSION_12
	&dev_attr_client_ver,
	&dev_attr_client_cap,
	&dev_attr_client_uuid,
#endif
	NULL
};

struct android_usb_function projector2_function = {
	.name		= "projector2",
	.init		= projector2_function_init,
	.cleanup	= projector2_function_cleanup,
	.bind_config	= projector2_function_bind_config,
	.attributes = projector2_function_attributes
};

static struct android_usb_function *supported_functions[] = {
	&ffs_function,
	&adb_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
#ifndef CONFIG_USBIF_COMPLIANCE
	&ecm_function,
	&eem_function,
#endif
	&ncm_function,
	&modem_function,
	&serial_function,
	&projector_function,
	&projector2_function,
	&rndis_function,
	&mass_storage_function,
	&accessory_function,
	&audio_source_function,
#ifdef CONFIG_EVDO_DT_SUPPORT
	&rawbulk_modem_function,
	&rawbulk_ets_function,
	&rawbulk_atc_function,
	&rawbulk_pcv_function,
	&rawbulk_gps_function,
#endif
#ifdef CONFIG_USB_F_LOOPBACK
	&loopback_function,
#endif
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;

#ifdef CONFIG_USBIF_COMPLIANCE
	int index = 1;
#else
	int index = 0;
#endif

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		
		pr_debug("[XLOG_INFO][USB]%s: f->dev_name = %s, f->name = %s\n", __func__, f->dev_name, f->name);
		
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			} else {
				pr_debug("[XLOG_INFO][USB]%s: init %s success!!\n", __func__, f->name);
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	
	pr_debug("[XLOG_INFO][USB]%s: ", __func__);
	

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_debug("[XLOG_INFO][USB]bind_config function '%s'/%p\n", f->name, f);
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_debug("[XLOG_INFO][USB]unbind_config function '%s'/%p\n", f->name, f);
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {

		
		pr_debug("[XLOG_INFO][USB]%s: name = %s, f->name=%s \n", __func__, name, f->name);
		
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

#include "htc_attr.c"


static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	
	pr_debug("[XLOG_INFO][USB]%s: ", __func__);
	

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += sprintf(buff, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	char aliases[256], *a;
	int err;
	int is_ffs;
	int ffs_enabled = 0;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&dev->enabled_functions);

	
	pr_debug("[XLOG_INFO][USB]%s: \n", __func__);
	

	is_mtp_enable = 0;
	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		
		pr_debug("[XLOG_INFO][USB]%s: name = %s \n", __func__, name);
		

		if (!name)
			continue;

		is_ffs = 0;
		strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
		a = aliases;

		while (a) {
			char *alias = strsep(&a, ",");
			if (alias && !strcmp(name, alias)) {
				is_ffs = 1;
				break;
			}
		}

		if (is_ffs) {
			if (ffs_enabled)
				continue;
			err = android_enable_function(dev, "ffs");
			if (err)
				pr_err("android_usb: Cannot enable ffs (%d)",
									err);
			else
				ffs_enabled = 1;
			continue;
		}

		if (!strcmp(name, "mtp"))
			is_mtp_enable = 1;

		err = android_enable_function(dev, name);
		if (err)
			pr_err("android_usb: Cannot enable '%s' (%d)",
							   name, err);
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static int spcf_flag = 0;
static int usb_sim_trigger =1;
static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	bool ffs_enabled = false;	
	int enabled = 0;

	
	if (spcf_flag == 1 && usb_sim_trigger == 0 ) {
		printk(KERN_WARNING "[USB][%s] Block enable_store() since spcf_flag(%d) and usb_sim_trigger(%d)\n", __func__, spcf_flag, usb_sim_trigger);
		return -EPERM;
	}

	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	
	pr_debug("[XLOG_INFO][USB]%s: device_attr->attr.name: %s\n", __func__, attr->attr.name);
	

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		cdev->next_string_id = 0x10;

		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		check_usb_project_pid(cdev);

		
		if (serial_string[0] == 0x0) {
			cdev->desc.iSerialNumber = 0;
		} else {
			cdev->desc.iSerialNumber = device_desc.iSerialNumber;
		}

		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			pr_debug("[XLOG_INFO][USB]enable function '%s'/%p\n", f->name, f);
			if (f->enable)
				f->enable(f);
			
			if (!strncmp(f->name, "ffs", 3))
				ffs_enabled = true;
			
		}
		
		if ((ffs_enabled && atomic_read(&dev->adb_ready)) || !ffs_enabled) {
			android_enable(dev);
			dev->enabled = true;

			while (atomic_read(&dev->delay_enable_store) != 0) {
				android_enable(dev);
				dev->enabled = true;
				atomic_sub(1, &dev->delay_enable_store);
			}

		} else {
			atomic_add(1, &dev->delay_enable_store);
		}
		

		
		pr_debug("[XLOG_INFO][USB]%s: enable 0->1 case, device_desc.idVendor = 0x%x, device_desc.idProduct = 0x%x\n", __func__, device_desc.idVendor, device_desc.idProduct);
		

	} else if (!enabled && dev->enabled) {

		
		pr_debug("[XLOG_INFO][USB]%s: enable 1->0 case, device_desc.idVendor = 0x%x, device_desc.idProduct = 0x%x\n", __func__, device_desc.idVendor, device_desc.idProduct);
		

		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			pr_debug("[XLOG_INFO][USB]disable function '%s'/%p\n", f->name, f);
			if (f->disable) {
				f->disable(f);
			}
		}
		
		atomic_set(&dev->delay_enable_store, 0);
		atomic_set(&dev->adb_ready, 0);
		
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
		
		if (!usb_cable_connected()) {
			schedule_work(&dev->work);
			pr_debug("[XLOG_VERBOSE][USB]%s: enable 0->0 case - no usb cable", __func__);
		}
		
	}

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}

static ssize_t sim_trigger_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d\n", usb_sim_trigger);

	return length;
}
static ssize_t sim_trigger_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	int value;
	sscanf(buff, "%d", &value);
	
	if (value==1 || value==0) {
	
		usb_sim_trigger = value;
		printk(KERN_WARNING "[USB][%s] set usb_sim_trigger = %d\n", __func__, usb_sim_trigger);
	} else {
		printk(KERN_WARNING "[USB][%s] INVALID value = %d, Do not set usb_sim_trigger(still %d)\n", __func__, value, usb_sim_trigger);
	}
	return size;
}


#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	if (sscanf(buf, "%s", buffer) == 1) {				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(usb_sim_trigger, S_IRUGO | S_IWUSR, sim_trigger_show, sim_trigger_store);

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	&dev_attr_usb_sim_trigger,
	NULL
};


static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_setup_config(struct usb_configuration *c, const struct usb_ctrlrequest *ctrl)
{
	int handled = -EINVAL;
	const u8 recip = ctrl->bRequestType & USB_RECIP_MASK;

	pr_debug("%s bRequestType=%x, bRequest=%x, recip=%x\n", __func__, ctrl->bRequestType, ctrl->bRequest, recip);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest)
		{
			case USB_REQ_CLEAR_FEATURE:
				switch (recip)
				{
					case USB_RECIP_DEVICE:
						switch (ctrl->wValue)
						{
							case USB_DEVICE_U1_ENABLE:
								handled = 1;
								pr_debug("Clear Feature->U1 Enable\n");
							break;

							case USB_DEVICE_U2_ENABLE:
								handled = 1;
								pr_debug("Clear Feature->U2 Enable\n");
							break;

							default:
								handled = -EINVAL;
							break;
						}
						break;
					default:
						handled = -EINVAL;
					break;
				}
			break;

			case USB_REQ_SET_FEATURE:
				switch (recip)
				{
					case USB_RECIP_DEVICE:
						switch (ctrl->wValue)
						{
							case USB_DEVICE_U1_ENABLE:
								pr_debug("Set Feature->U1 Enable\n");
								handled = 1;
							break;
							case USB_DEVICE_U2_ENABLE:
								pr_debug("Set Feature->U2 Enable\n");
								handled = 1;
							break;
							default:
								handled = -EINVAL;
							break;
						}
					break;

					default:
						handled = -EINVAL;
					break;
				}
			break;

			default:
				handled = -EINVAL;
			break;
		}
	}

	return handled;
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			id, ret;

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	
	strncpy(manufacturer_string, MANUFACTURER_STRING, sizeof(manufacturer_string) - 1);
	strncpy(product_string, PRODUCT_STRING, sizeof(product_string) - 1);
	strncpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

#ifdef CONFIG_USBIF_COMPLIANCE
	usb_gadget_clear_selfpowered(gadget);
#else
	usb_gadget_set_selfpowered(gadget);
#endif
	dev->cdev = cdev;

	cdev->sw_connect2pc.name = "usb_connect2pc";
	ret = switch_dev_register(&cdev->sw_connect2pc);
	if (ret < 0)
		pr_err("switch_dev_register fail:usb_connect2pc\n");

	cdev->sw_function_switch_on.name = "function_switch_on";
	ret = switch_dev_register(&cdev->sw_function_switch_on);
	cdev->sw_function_switch_off.name = "function_switch_off";
	ret = switch_dev_register(&cdev->sw_function_switch_off);

	init_mfg_serialno();

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	switch_dev_unregister(&cdev->sw_connect2pc);
	switch_dev_unregister(&cdev->sw_function_switch_on);
	switch_dev_unregister(&cdev->sw_function_switch_off);
	return 0;
}

static int (*composite_setup_func)(struct usb_gadget *gadget, const struct usb_ctrlrequest *c);
extern void composite_setup_complete(struct usb_ep *ep, struct usb_request *req);

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	pr_debug("[XLOG_VERBOSE][USB]%s\n", __func__);

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = projector_ctrlrequest(cdev, c);

	if (value < 0)
		value = projector2_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup_func(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	
	pr_debug("[XLOG_VERBOSE][USB]%s: \n", __func__);
	

	acc_disconnect();

	dev->connected = 0;
	schedule_work(&dev->work);

	
	if (switch_get_state(&ml_switch)) {
		switch_set_state(&ml_switch, 0);
		printk("[MIRROR_LINK]%s : ml_switch set 0\n", __func__);
	}
	

	
	pr_debug("[XLOG_VERBOSE][USB]%s: dev->connected = %d \n", __func__, dev->connected);
	
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_usb_unbind,
	.disconnect	= android_disconnect,
#ifdef CONFIG_USB_MU3D_DRV
	.max_speed	= USB_SPEED_SUPER
#else
	.max_speed	= USB_SPEED_HIGH
#endif
};

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

#ifdef CONFIG_USBIF_COMPLIANCE

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>


static int andoid_usbif_driver_on = 0 ;


static int android_start(void)
{
	struct android_dev *dev;
	int err;

	pr_debug("android_start ===> \n");

	err = usb_composite_probe(&android_usb_driver);
	if (err) {
		pr_err("%s: failed to probe driver %d", __func__, err);
	}

	
	composite_setup_func = android_usb_driver.gadget_driver.setup;
	android_usb_driver.gadget_driver.setup = android_setup;

	pr_debug("android_start <=== \n");

	return err;
}

static int android_stop(void)
{
	pr_debug("android_stop ===> \n");

	usb_composite_unregister(&android_usb_driver);

	pr_debug("android_stop <=== \n");
}

static int andoid_usbif_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "andoid_usbif_proc_show, andoid_usbif_driver_on is %d (on:1, off:0)\n", andoid_usbif_driver_on);
	return 0;
}

static int andoid_usbif_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, andoid_usbif_proc_show, inode->i_private);
}

static ssize_t andoid_usbif_proc_write(struct file *file, const char __user *buf, size_t length, loff_t *ppos)
{
	int ret ;
	char msg[32] ;
	int result;
	int status;
	struct device	*dev ;
	int		irq ;
	struct resource	*iomem;
	void __iomem	*base;
	struct musb *musb ;
	void __iomem	*ctrl_base;

	if (length >= sizeof(msg)) {
		pr_debug("andoid_usbif_proc_write length error, the error len is %d\n", (unsigned int)length);
		return -EINVAL;
	}
	if (copy_from_user(msg, buf, length))
		return -EFAULT;

	msg[length] = 0 ;

	pr_debug("andoid_usbif_proc_write: %s, current driver on/off: %d\n", msg, andoid_usbif_driver_on);

	if ((msg[0] == '1') && (andoid_usbif_driver_on == 0)){
		pr_debug("start usb android driver ===> \n");
		printk("start usb android driver ===> \n");
		android_start() ;
		andoid_usbif_driver_on = 1 ;
		pr_debug("start usb android driver <=== \n");
	}else if ((msg[0] == '0') && (andoid_usbif_driver_on == 1)){
		pr_debug("stop usb android driver ===> \n");
		printk("stop usb android driver ===> \n");
		andoid_usbif_driver_on = 0 ;
		android_stop() ;

		pr_debug("stop usb android driver <=== \n");
	}

	return length;
}

static const struct file_operations andoid_usbif_proc_fops = {
	.owner = THIS_MODULE,
	.open = andoid_usbif_proc_open,
	.write = andoid_usbif_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,

};

static int __init init(void)
{
	struct android_dev *dev;
	int err;
	struct proc_dir_entry *prEntry;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto err_dev;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		pr_err("%s: failed to create android device %d", __func__, err);
		goto err_create;
	}

	_android_dev = dev;

	prEntry = proc_create("android_usbif_init", 0666, NULL, &andoid_usbif_proc_fops);

	if (prEntry)
	{
		printk("create the android_usbif_init proc OK!\n") ;
	}else{
		printk("[ERROR] create the android_usbif_init proc FAIL\n") ;
	}

	
	android_start() ;
	andoid_usbif_driver_on = 1 ;

	return 0;

err_create:
	kfree(dev);
err_dev:
	class_destroy(android_class);
	return err;
}

late_initcall(init);



static void __exit cleanup(void)
{
	printk("[U3D] android cleanup ===> \n") ;
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
	printk("[U3D] android cleanup <=== \n") ;
}
module_exit(cleanup);

#else
static int __init init(void)
{
	struct android_dev *dev;
	int err;
	connect2pc = false;
	static char cid[64] = "";

	strcpy(cid, htc_get_cid());
	if (!strcmp(htc_get_bootmode(),"factory2") || !strcmp(htc_get_bootmode(),"meta") || !strcmp(htc_get_bootmode(),"ftm") || !strcmp(htc_get_bootmode(),"RUU")) {
		spcf_flag = 2;
		usb_sim_trigger =1;
		printk(KERN_WARNING "[USB][%s] got Internal Mode, set usb_sim_trigger=%d, spcf_flag=%d\n", __func__, usb_sim_trigger, spcf_flag);
	} else {
		if (cid[1]>='O' && cid[3]<='E' && cid[4]=='N' && cid[5]>='6' && cid[7]=='4') {
		
			spcf_flag = 1;
			usb_sim_trigger =0;
			printk(KERN_WARNING "[USB][%s] got specific project, set usb_sim_trigger=%d, spcf_flag=%d\n", __func__, usb_sim_trigger, spcf_flag);
		} else {
			spcf_flag = 0;
			usb_sim_trigger =1;
			printk(KERN_WARNING "[USB][%s] got common, set usb_sim_trigger=%d, spcf_flag=%d\n", __func__, usb_sim_trigger, spcf_flag);
		}
	}

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto err_dev;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		pr_err("%s: failed to create android device %d", __func__, err);
		goto err_create;
	}

	_android_dev = dev;

	err = usb_composite_probe(&android_usb_driver);
	if (err) {
		pr_err("%s: failed to probe driver %d", __func__, err);
		goto err_probe;
	}

	setup_vendor_info(_android_dev);
	printk("[USB] %s: probe complete\n",__func__);

	
	composite_setup_func = android_usb_driver.gadget_driver.setup;
	android_usb_driver.gadget_driver.setup = android_setup;

	return 0;

err_probe:
	device_destroy(android_class, dev->dev->devt);
err_create:
	kfree(dev);
err_dev:
	class_destroy(android_class);
	return err;
}
late_initcall(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
#endif
