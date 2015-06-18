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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include <linux/async.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/dma-mapping.h>

#include <linux/htc_flashlight.h>

#include <linux/htc_devices_dtb.h>


#define GPIO_TS3A225E_I2C_SCL GPIO_FLT_I2C_SCL
#define GPIO_TS3A225E_I2C_SCL_M_GPIO 0
#define GPIO_TS3A225E_I2C_SDA GPIO_FLT_I2C_SDA
#define GPIO_TS3A225E_I2C_SDA_M_GPIO 0

#define GPIO_FLT_I2C_SDA		 GPIO45
#define GPIO_FLT_I2C_SCL		 GPIO46

#define GPIO_FLT_CE_PIN GPIO95
#define GPIO_TORCH_EN_PIN GPIO132
#define GPIO_FLT_EN_PIN GPIO194

#define FLASHLIGHT_NAME "flashlight"

static struct led_classdev *this_sy7802_cdev;
struct delayed_work sy7802_delayed_work;
static struct workqueue_struct *sy7802_work_queue;


#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT]TPS " fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT]TPS " fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT][ERR]TPS " fmt, ##__VA_ARGS__)



#define GTP_DMA_MAX_TRANSACTION_LENGTH 255
#define GTP_DMA_MAX_I2C_TRANSFER_SIZE 255

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
static u8 *gpDMABuf_va1 = NULL;
static dma_addr_t gpDMABuf_pa1 = 0;
static DEFINE_MUTEX(dma_i2c_lock);




static struct i2c_client *SY7802_i2c_client = NULL;

struct SY7802_platform_data {
	u8 torch_pin_enable;	
	u8 pam_sync_pin_enable; 
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;	
	u8 vout_mode_enable;	
};

struct SY7802_chip_data {
	struct i2c_client *client;
	struct led_classdev		cdev;
	struct SY7802_platform_data	*pdata;
	struct mutex lock;
};
#if 0
static void reg_dump(void);
#endif

static int SY7802_read_reg(struct i2c_client *client, uint8_t addr,
		uint8_t *data)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[2] =
	{
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.flags = 0,
		.buf = buffer,
		.len = 1,
		.timing = 100
	},
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = I2C_M_RD,
		.buf = (uint8_t *)gpDMABuf_pa1,
		.len = 1,
		.timing = 100
	},
	};
	mutex_lock(&dma_i2c_lock);
	buffer[0] = addr;

	if (data == NULL){
		mutex_unlock(&dma_i2c_lock);
		return -1;
	}

	for (retry = 0; retry < 5; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(data, gpDMABuf_va1, 1);
		mutex_unlock(&dma_i2c_lock);
		return 0;
	}

	FLT_ERR_LOG("%s: 0x%04X, err-code: %d", __func__, addr, ret);
	mutex_unlock(&dma_i2c_lock);
	return ret;
}

static int SY7802_write_reg(struct i2c_client *client, uint8_t addr,
		uint8_t data)
{
	int ret;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va1;

	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (uint8_t *)gpDMABuf_pa1,
		.len = 1 + 1,
		.timing = 100
	};

	mutex_lock(&dma_i2c_lock);
	wr_buf[0] = addr;
	wr_buf[1] = data;

	for (retry = 0; retry < 5; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			continue;
		}
		mutex_unlock(&dma_i2c_lock);
		return 0;
	}

	FLT_ERR_LOG("%s: 0x%04X, err-code: %d", __func__, addr, ret);
	mutex_unlock(&dma_i2c_lock);
	return ret;
}

static int sy7802_turn_off(void)
{
	printk("[FLT] %s\n", __func__);

	SY7802_write_reg(SY7802_i2c_client, 0x10, 0x00);

	return 0;
}

static void sy7802_turn_off_work(struct work_struct *work)
{
	printk("[FLT] %s\n", __func__);
	sy7802_turn_off();
}
#if 0
static void reg_dump(void)
{
	u8 ret;
	printk("[FLT] 0x10: 0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0x10, &ret));
	printk("[FLT] 0xA0: 0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xA0, &ret));
	printk("[FLT] 0xB0:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xB0, &ret));
	printk("[FLT] 0xC0:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xC0, &ret));
	printk("[FLT] 0xD0:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xF0, &ret));
	printk("[FLT] 0xE0:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xE0, &ret));
	printk("[FLT] 0xF0:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xF0, &ret));
	printk("[FLT] 0x81:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0x81, &ret));
	printk("[FLT] 0x30:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0x30, &ret));
	printk("[FLT] 0x31:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0x31, &ret));
	printk("[FLT] 0x80:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0x80, &ret));
	printk("[FLT] 0xFF:	0x%x\n",SY7802_read_reg(SY7802_i2c_client, 0xFF, &ret));
}
#endif


static void led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct SY7802_chip_data *chip = container_of(led_cdev,
					struct SY7802_chip_data, cdev);

	FLT_INFO_LOG("%s: mode: %d\n", __func__, value);

	mutex_lock(&chip->lock);
	

	SY7802_write_reg(SY7802_i2c_client, 0x10, 0x00); 
	

 	if(value > 0 && value <= (LED_HALF+1)) {
 		switch(value) {
 		
 		case (LED_HALF - 2):
			SY7802_write_reg(SY7802_i2c_client, 0xA0, 0x00); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0A); 
			break;
		
		case (LED_HALF - 1):
			SY7802_write_reg(SY7802_i2c_client, 0xA0, 0x01); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0A); 
			break;
		
		case LED_HALF:
			SY7802_write_reg(SY7802_i2c_client, 0xA0, 0x02); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0A); 
			break;
		
		case (LED_HALF + 1):
			SY7802_write_reg(SY7802_i2c_client, 0xA0, 0x03); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0A); 
			break;
		default:
			FLT_INFO_LOG("%s: Not supported level\n", __func__);
			break;
 		}
 	} else if (value > (LED_HALF+1) && value <= LED_FULL) {
		switch(value) {
		
		case (LED_HALF + 3):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x01); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 4):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x03); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 5):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x05); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 6):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x07); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 7):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x09); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 8):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x0B); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case (LED_HALF + 9):
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x0D); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		
		case LED_FULL:
			SY7802_write_reg(SY7802_i2c_client, 0xB0, 0x0D); 
			SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
			queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
			break;
		default:
			FLT_INFO_LOG("%s: Not supported level\n", __func__);
			break;
 		}
		led_cdev->brightness = value;
 	}else if(value == LED_OFF) {
		led_cdev->brightness = value;
	 	sy7802_turn_off();
	} else {
		FLT_INFO_LOG("%s: unsupport value: %d\n", __func__, value);
	}
	mutex_unlock(&chip->lock);
}


static enum led_brightness led_get(struct led_classdev *led_cdev)
{
	printk("%s: %d\n", __func__, led_cdev->brightness);

	return led_cdev->brightness;
}


int sy7802_flt_flash(struct led_classdev *led_cdev, uint32_t mA)
{
	struct SY7802_chip_data *chip = container_of(led_cdev,
					struct SY7802_chip_data, cdev);
	u8 lv = 0x0;
	int ret = 0;

	mutex_lock(&chip->lock);
	lv = (u8)((int)mA/56);
	FLT_INFO_LOG("%s: %d mA, level: 0x%x.\n", __func__, mA, lv);

	if (mA == 0) {
		SY7802_write_reg(SY7802_i2c_client, 0x10, 0x00); 
	} else if(mA > 0 && mA <= 784) {
		SY7802_write_reg(SY7802_i2c_client, 0xB0, lv); 
		SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0B); 
		queue_delayed_work(sy7802_work_queue, &sy7802_delayed_work, msecs_to_jiffies(600));
	} else {
		FLT_INFO_LOG("%s: unsupport value.\n", __func__);
	}
	mutex_unlock(&chip->lock);

	return ret;
}


int sy7802_flt_torch(struct led_classdev *led_cdev, uint32_t mA)
{
	struct SY7802_chip_data *chip = container_of(led_cdev,
					struct SY7802_chip_data, cdev);
	u8 lv = 0x0;
	int ret = 0;

	mutex_lock(&chip->lock);
	lv = (u8)((int)mA/28);
	FLT_INFO_LOG("%s: %d mA, level: 0x%x.\n", __func__, mA, lv);

	if (mA == 0) {
		SY7802_write_reg(SY7802_i2c_client, 0x10, 0x00); 
	} else if(mA > 0 && mA <= 224) {
		SY7802_write_reg(SY7802_i2c_client, 0xA0, lv); 
		SY7802_write_reg(SY7802_i2c_client, 0x10, 0x0A); 
	} else {
		FLT_INFO_LOG("%s: unsupport value.\n", __func__);
	}
	mutex_unlock(&chip->lock);

	return ret;
}


#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
static int sy7802_flt_flash_adapter(int mA1, int mA2) {
	return sy7802_flt_flash(this_sy7802_cdev, mA1);
}

static int sy7802_flt_torch_adapter(int mA1, int mA2) {
	return sy7802_flt_torch(this_sy7802_cdev, mA1);
}
#endif


static void SY7802_probe_async(void *data, async_cookie_t cookie)
{
	struct i2c_client *client = (struct i2c_client *)data;
	struct SY7802_chip_data *chip = i2c_get_clientdata(client);
	int err = -1;
	uint8_t ret = 0;

	FLT_INFO_LOG("%s: +++\n", __func__);

	chip->cdev.name		= FLASHLIGHT_NAME;
	chip->cdev.brightness_set	= led_set;
	chip->cdev.brightness_get	= led_get;
	this_sy7802_cdev		= &chip->cdev;
	SY7802_i2c_client		= client;


	
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		FLT_ERR_LOG("%s: Allocate DMA I2C Buffer failed!\n", __func__);
		goto err_dma_alloc;
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
	
	gpDMABuf_va1 = (u8 *)dma_alloc_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa1, GFP_KERNEL);
	if(!gpDMABuf_va1){
		FLT_ERR_LOG("%s: create DMA I2C Buffer failed!\n", __func__);
		goto err_dma_alloc;
	}
	memset(gpDMABuf_va1, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);

	
	err = SY7802_read_reg(client, 0xFF, &ret);
	if(err >= 0) {
		FLT_INFO_LOG("%s: SY7802_chip_init end ret = %d.\n", __func__, ret);
	} else {
		FLT_ERR_LOG("%s: Check chip existence fail\n", __func__);
		goto err_chip_init;
	}

	
	SY7802_write_reg(SY7802_i2c_client, 0xC0, 0x11); 

	INIT_DELAYED_WORK(&sy7802_delayed_work, sy7802_turn_off_work);
	sy7802_work_queue = create_singlethread_workqueue("sy7802_wq");
	if (!sy7802_work_queue) {
		FLT_ERR_LOG("%s: cannot create workqueue\n", __func__);
		goto err_workqueue_init;
	}

	mutex_init(&chip->lock);
	err = led_classdev_register(&client->dev, &chip->cdev);
	if (err != 0) {
		FLT_ERR_LOG("%s: unable to register led class dev, err=%d\n",
			__func__, err);
		goto err_class_dev_reg;
	}

#if defined(CONFIG_HTC_FLASHLIGHT_COMMON)
	FLT_INFO_LOG("%s: (CONFIG_HTC_FLASHLIGHT_COMMON) is defined.\n", __func__);
	htc_flash_main			= &sy7802_flt_flash_adapter;
	htc_torch_main			= &sy7802_flt_torch_adapter;
#endif

	FLT_INFO_LOG("%s: ---\n", __func__);
	return;


err_class_dev_reg:
	mutex_destroy(&chip->lock);
	destroy_workqueue(sy7802_work_queue);
err_workqueue_init:
err_chip_init:
err_dma_alloc:
	if (gpDMABuf_va) {
		dma_free_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);					 
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
	}
	if (gpDMABuf_va1) {
		dma_free_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va1, gpDMABuf_pa1);
		gpDMABuf_va1 = NULL;
		gpDMABuf_pa1 = 0;
	}

	i2c_set_clientdata(client, NULL);
	if(client->dev.of_node)
		kfree(chip->pdata);
	kfree(chip);
	FLT_ERR_LOG("SY7802 probe is failed \n");
return ;

}


static int SY7802_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct SY7802_platform_data *pdata;
	struct SY7802_chip_data *chip;

	FLT_INFO_LOG("%s: +++\n", __func__);

	if(strcmp(htc_get_bootmode(), "charger") == 0) {
		FLT_INFO_LOG("%s: offmode_charging, do not probe sy7802_flashlight\n", __func__);
		return -EACCES;
	}

	
	mt_set_gpio_mode(GPIO_FLT_CE_PIN, 0);
	mt_set_gpio_dir(GPIO_FLT_CE_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FLT_CE_PIN, GPIO_OUT_ONE);
	
	mt_set_gpio_mode(GPIO_FLT_EN_PIN, 0);
	mt_set_gpio_dir(GPIO_FLT_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FLT_EN_PIN, GPIO_OUT_ZERO);
	
	mt_set_gpio_mode(GPIO_TORCH_EN_PIN, 0);
	mt_set_gpio_dir(GPIO_TORCH_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_TORCH_EN_PIN, GPIO_OUT_ZERO);


	if(client->dev.of_node){ 
		pdata = kzalloc(sizeof(struct SY7802_platform_data), GFP_KERNEL);
	} else {
		pdata = client->dev.platform_data;
	}

	if (pdata == NULL){
		FLT_ERR_LOG("SY7802_platform_data does not exist\n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FLT_ERR_LOG("%s: SY7802 i2c functionality check fail.\n", __func__);
		if(client->dev.of_node)
			kfree(pdata);
		return -ENODEV;
	} else {
		FLT_INFO_LOG("%s: I2C function is OK\n", __func__);
	}

	chip = kzalloc(sizeof(struct SY7802_chip_data), GFP_KERNEL);
	if (!chip) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		if(client->dev.of_node)
			kfree(pdata);
		return -ENOMEM;
	}

	chip->client		= client;
	chip->pdata		= pdata;
	i2c_set_clientdata(client, chip);


	async_schedule(SY7802_probe_async, client);

	FLT_INFO_LOG("%s: ---\n", __func__);
	return 0;
}


static int SY7802_remove(struct i2c_client *client)
{
 	struct SY7802_chip_data *chip = i2c_get_clientdata(client);

	FLT_INFO_LOG("%s: +++\n", __func__);

	
	cancel_delayed_work_sync(&sy7802_delayed_work);
	sy7802_turn_off();
	mt_set_gpio_out(GPIO_FLT_CE_PIN, GPIO_OUT_ZERO);


	led_classdev_unregister(&chip->cdev);

	mutex_destroy(&chip->lock);
	destroy_workqueue(sy7802_work_queue);

	if (gpDMABuf_va) {
		dma_free_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);					 
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
	}
	if (gpDMABuf_va1) {
		dma_free_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va1, gpDMABuf_pa1);
		gpDMABuf_va1 = NULL;
		gpDMABuf_pa1 = 0;
	}

	i2c_set_clientdata(client, NULL);
	if(client->dev.of_node)
		kfree(chip->pdata);
	kfree(chip);

	FLT_INFO_LOG("%s: ---\n", __func__);
	return 0;
}


static int SY7802_resume(struct i2c_client *client)
{
	FLT_INFO_LOG("%s:\n", __func__);
	mt_set_gpio_out(GPIO_FLT_CE_PIN, GPIO_OUT_ONE);
	return 0;
}


static int SY7802_suspend(struct i2c_client *client, pm_message_t state)
{
	FLT_INFO_LOG("%s:\n", __func__);
	sy7802_turn_off();
	mt_set_gpio_out(GPIO_FLT_CE_PIN, GPIO_OUT_ZERO);
	return 0;
}


static const struct i2c_device_id SY7802_id[] = {
	{ "SY7802_FLASHLIGHT", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, SY7802_id);

static const struct of_device_id SY7802_mttable[] = {
	{ .compatible = "SY7802_FLASHLIGHT"},
	{ },
};

static struct i2c_driver SY7802_i2c_driver = {
	.driver = {
		.name	= "SY7802_FLASHLIGHT",
		.owner = THIS_MODULE,
		.of_match_table = SY7802_mttable,
	},
	.probe		= SY7802_probe,
	.remove		= SY7802_remove,
	.resume		= SY7802_resume,
	.suspend	= SY7802_suspend,
	.id_table	= SY7802_id,
};

static int __init SY7802_init(void)
{
	FLT_INFO_LOG("%s\n", __func__);
	return i2c_add_driver(&SY7802_i2c_driver);;
}

static void __exit SY7802_exit(void)
{
	i2c_del_driver(&SY7802_i2c_driver);
}

late_initcall(SY7802_init);
module_exit(SY7802_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("terry_yen@htc.com>");
MODULE_AUTHOR("tai_kuo@htc.com>");
MODULE_DESCRIPTION("SY7802 Flashlight Driver");
