#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/pn544.h>
#include <linux/types.h>
#include "pn544_mfg.h"
#include "pn544_htc.h"


#include <mach/mt_gpio.h>
#include <linux/dma-mapping.h>
#include <mach/eint.h>
#include <linux/of_irq.h>

static u8*		I2CDMABuf_va_read = NULL;
static u32*		I2CDMABuf_pa_read = NULL;
static u8*		I2CDMABuf_va_write = NULL;
static u32*		I2CDMABuf_pa_write = NULL;
#define DMA_SIZE					255

#define FTM_MODE 1
#if FTM_MODE
int mfc_nfc_cmd_result = 0;

static   unsigned long watchdog_counter;
static   unsigned int watchdogEn;
static   unsigned int watchdog_timeout;
#define WATCHDOG_FTM_TIMEOUT_SEC 20
#endif  

int nfc_is_debug = 0;
int s_wdcmd_cnt = 0;
int is_alive = 1;
int is_uicc_swp = 1;

#define DBUF(buff,count) \
	if (nfc_is_debug) \
		for (i = 0; i < count; i++) \
			printk(KERN_DEBUG "[NFC] %s : [%d] = 0x%x\n", \
				__func__, i, buff[i]);

#define D(x...)	\
	if (nfc_is_debug)	\
		printk(KERN_INFO "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)

#define MAX_BUFFER_SIZE	512
#define PN544_RESET_CMD 	0
#define PN544_DOWNLOAD_CMD	1

#define I2C_RETRY_COUNT 10

struct pn544_dev	{
	struct class		*pn544_class;
	struct device		*pn_dev;
	struct device		*comn_dev;
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct wake_lock io_wake_lock;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	unsigned int 		ven_gpio;
	unsigned int		ven_value;
	unsigned int 		firm_gpio;
	void (*gpio_init) (void);
	unsigned int 		ven_enable;
	int boot_mode;
	bool                     isReadBlock;
};

struct pn544_dev *pn_info;
int ignoreI2C = 0;

#define RETRY_TIMES 9
static int pn544_RxData2(uint8_t *buf, int len)
{
    int ret;
    int i;
	struct pn544_dev *pni = pn_info;

    if (!pni->client) return -ENXIO;

    for (i = 0; i < RETRY_TIMES; i++) {
        D("%s(%d) len:%d\n", __func__, __LINE__, len);
        ret = i2c_master_recv(pni->client, (char *)I2CDMABuf_pa_read, (int)len);
        memcpy(buf, I2CDMABuf_va_read, len);
        if (unlikely(ret < 0)) {
            E("%s(%d) fail addr:0x%x ret:%d retry:%d\n", __func__, __LINE__, pni->client->addr, ret, i);
            msleep(1);
        }
        else {
            break;
        }
    }

    return ret;
}

static int pn544_TxData2(uint8_t *buf, int len)
{
    int ret;
    int i;
	uint8_t loop_i;
	struct pn544_dev *pni = pn_info;

    if (!pni->client) return -ENXIO;

    memcpy(I2CDMABuf_va_write, buf, len);

    for (i = 0; i < RETRY_TIMES; i++) {
        D("%s(%d) len:%d\n", __func__, __LINE__, len);
        ret = i2c_master_send(pni->client, (char *)I2CDMABuf_pa_write, (int)len);
        if (unlikely(ret < 0)) {
            E("%s(%d) fail addr:0x%x ret:%d retry:%d\n", __func__, __LINE__, pni->client->addr, ret, i);
            msleep(1);
        }
        else {
            break;
        }
    }

    return ret;
}


static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;
	static unsigned long orig_jiffies = 0;

	if (gpio_get_value(pn544_dev->irq_gpio) == 0) {
		I("%s: irq_workaround\n", __func__);
		return IRQ_HANDLED;
	}

	pn544_disable_irq(pn544_dev);

	
	wake_up(&pn544_dev->read_wq);

	if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(1000)))
		I("%s: irq=%d\n", __func__, irq);
	orig_jiffies = jiffies;

	return IRQ_HANDLED;
}

static void pn544_Enable(void)
{
	struct pn544_dev *pni = pn_info;
	unsigned int set_value = pni->ven_enable;
	I("%s: gpio=%d set_value=%d\n", __func__, pni->ven_gpio, set_value);

	mt_set_gpio_out(pni->ven_gpio, set_value);

	pni->ven_value = 1;
}

static void pn544_Disable(void)
{
	struct pn544_dev *pni = pn_info;
	unsigned int set_value = !pni->ven_enable;
	I("%s: gpio=%d set_value=%d\n", __func__, pni->ven_gpio, set_value);

	mt_set_gpio_out(pni->ven_gpio, set_value);

	pni->ven_value = 0;
}


static int pn544_isEn(void)
{
	struct pn544_dev *pni = pn_info;
	
	return pni->ven_value;
}

uint8_t read_buffer[MAX_BUFFER_SIZE];

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pni = pn_info;
	int ret;
	int val;
	int i;
	i = 0;

	D("%s: start count = %zu\n", __func__, count);

	if (count > MAX_BUFFER_SIZE) {
		E("%s : count =%zu> MAX_BUFFER_SIZE\n", __func__, count);
		count = MAX_BUFFER_SIZE;
	}

	val = gpio_get_value(pni->irq_gpio);

	D("%s: reading %zu bytes, irq_gpio = %d\n",
		__func__, count, val);

	mutex_lock(&pni->read_mutex);

	if (!gpio_get_value(pni->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			I("%s : f_flags & O_NONBLOCK read again\n", __func__);
			ret = -EAGAIN;
			goto fail;
		}

		pni->irq_enabled = true;
		enable_irq(pni->client->irq);
		D("%s: waiting read-event INT, because "
			"irq_gpio = 0\n", __func__);
		pni->isReadBlock = true;
		ret = wait_event_interruptible(pni->read_wq, gpio_get_value(pni->irq_gpio));

		pn544_disable_irq(pni);

		D("%s : wait_event_interruptible done\n", __func__);

		if (ret) {
			I("pn544_dev_read wait_event_interruptible breaked ret=%d\n", ret);
			goto fail;
		}

	}

	pni->isReadBlock = false;
    wake_lock_timeout(&pni ->io_wake_lock, IO_WAKE_LOCK_TIMEOUT);
	
	memset(read_buffer, 0, MAX_BUFFER_SIZE);
	ret = pn544_RxData2(read_buffer, count);
	mutex_unlock(&pni->read_mutex);

	if (ret < 0) {
		E("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		E("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}

	DBUF(read_buffer, count);

	if (copy_to_user(buf, read_buffer, count)) {
		E("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	D("%s done count = %zu\n", __func__, count);
	return count;

fail:
	mutex_unlock(&pni->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pni = pn_info;
	char buffer[MAX_BUFFER_SIZE];
	int ret;
	int i;
	i = 0;

	D("%s: start count = %zu\n", __func__, count);
	wake_lock_timeout(&pni ->io_wake_lock, IO_WAKE_LOCK_TIMEOUT);

	if (count > MAX_BUFFER_SIZE) {
		E("%s : count =%zu> MAX_BUFFER_SIZE\n", __func__, count);
		count = MAX_BUFFER_SIZE;
	}

	if ( nfc_is_debug && (s_wdcmd_cnt++ < 3))
		I("%s: writing %zu bytes\n",__func__, count);
	else {
		nfc_is_debug = 0;
		s_wdcmd_cnt = 4;
	}

	if (copy_from_user(buffer, buf, count)) {
		E("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	DBUF(buffer, count);
	
	ret = pn544_TxData2(buffer, count);
	if (ret < 0) {
		E("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	} else {
		D("%s done count = %zu\n", __func__, count);
		return count;
	}

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;
	I("%s : major=%d, minor=%d\n", \
		__func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pni = pn_info;
	uint8_t buffer[] = {0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5};

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 3) {
			
			I("%s Software reset\n", __func__);
			if (pn544_TxData2(buffer, 6) < 0)
				E("%s, SW-Reset TxData error!\n", __func__);
		} else if (arg == 2) {
			I("%s power on with firmware\n", __func__);
			pn544_Enable();
			mt_set_gpio_out(pni->firm_gpio, 1);

			msleep(50);
			pn544_Disable();
			msleep(50);
			pn544_Enable();
			msleep(50);
		} else if (arg == 1) {
			
			I("%s power on (delay50)\n", __func__);
			mt_set_gpio_out(pni->firm_gpio, 0);

			pn544_Enable();
			msleep(50);
			nfc_is_debug = 1;
			s_wdcmd_cnt = 0;
			I("%s pn544_Enable, set nfc_is_debug = %d, s_wdcmd_cnt : %d\n", __func__, nfc_is_debug, s_wdcmd_cnt);
		} else  if (arg == 0) {
			
			I("%s power off (delay50)\n", __func__);
			mt_set_gpio_out(pni->firm_gpio, 0);

			pn544_Disable();
			msleep(50);
			nfc_is_debug = 0;
			I("%s pn544_Disable, set nfc_is_debug = %d, s_wdcmd_cnt = %d\n", __func__, nfc_is_debug, s_wdcmd_cnt);
		} else {
			E("%s bad arg %lu\n", __func__, arg);
			goto fail;
		}
		break;
	default:
		E("%s bad ioctl %u\n", __func__, cmd);
		goto fail;
	}

	return 0;
fail:
	return -EINVAL;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
	.compat_ioctl  = pn544_dev_ioctl,
};

static ssize_t pn_temp1_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int val = -1;
	struct pn544_dev *pni = pn_info;
	uint8_t buffer[MAX_BUFFER_SIZE];
	int i = 0;


	I("%s:\n", __func__);
	val = gpio_get_value(pni->irq_gpio);

	memset(buffer, 0, MAX_BUFFER_SIZE);
#if 1
	if (val == 1) {
		ret = pn544_RxData2(buffer, 33);
		if (ret < 0) {
			E("%s, i2c Rx error!\n", __func__);
		} else {
			for (i = 0; i < 10; i++)
			I("%s : [%d] = 0x%x\n", __func__, i, buffer[i]);
		}
	} else {
		E("%s, data not ready\n", __func__);
	}
#else
	if (val != 1)
		E("%s, ####### data not ready -> force to read!#########\n", __func__);

	ret = pn544_RxData(buffer, 33);
	if (ret < 0) {
		E("%s, i2c Rx error!\n", __func__);
	} else {
		for (i = 0; i < 10; i++)
		D("%s : [%d] = 0x%x\n", __func__, i, buffer[i]);
	}
#endif

	ret = sprintf(buf, "GPIO INT = %d "
		"Rx:ret=%d [0x%x, 0x%x, 0x%x, 0x%x]\n", val, ret, buffer[0], buffer[1],
		buffer[2], buffer[3]);

	return ret;
}


#define i2cw_size (20)
static ssize_t pn_temp1_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code = -1;
	int ret = -1;
	struct pn544_dev *pni = pn_info;
	uint8_t buffer[] = {0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5};
	
	uint8_t i2cw[i2cw_size];
	uint32_t scan_data = 0;
	int i2cw_len = 0;
	int i = 0;
	char *ptr;

	sscanf(buf, "%d", &code);

	I("%s: irq = %d,  ven_gpio = %d,  firm_gpio = %d +\n", __func__, \
		gpio_get_value(pni->irq_gpio), pn544_isEn(), \
		gpio_get_value(pni->firm_gpio));
	I("%s: store value = %d\n", __func__, code);

	switch (code) {
	case 1:
			I("%s: case 1\n", __func__);
			ret = pn544_TxData2(buffer, 6);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
	case 2:
			I("%s: case 2 %d\n", __func__, pni->ven_gpio);
			pn544_Disable();
			break;
	case 3:
			I("%s: case 3 %d\n", __func__, pni->ven_gpio);
			pn544_Enable();
			break;
	case 4:
			I("%s: case 4 %d\n", __func__, pni->firm_gpio);
			mt_set_gpio_out(pni->firm_gpio, 0);
			break;
	case 5:
			I("%s: case 5 %d\n", __func__, pni->firm_gpio);
			mt_set_gpio_out(pni->firm_gpio, 1);
			break;
	case 6:
			memset(i2cw, 0, i2cw_size);
			sscanf(buf, "%d %d", &code, &i2cw_len);
			I("%s: case 6 i2cw_len=%u\n", __func__, i2cw_len);

			ptr = strpbrk(buf, " ");	
			if (ptr != NULL) {
				for (i = 0 ; i <= i2cw_len ; i++) {
					sscanf(ptr, "%x", &scan_data);
					i2cw[i] = (uint8_t)scan_data;
					I("%s: i2cw[%d]=%x\n", \
						__func__, i, i2cw[i]);
					ptr = strpbrk(++ptr, " ");
					if (ptr == NULL)
						break;
				}

				ret = pn544_TxData2(i2cw, i2cw_len+1);
				

				if (ret < 0)
					E("%s, i2c Tx error!\n", __func__);
			} else {
				I("%s: skip no data found\n", __func__);
			}
			break;
	case 7:
			I("%s: case 7 disable i2c\n", __func__);
			ignoreI2C = 1;
			break;
	case 8:
			I("%s: case 8 enable i2c\n", __func__);
			ignoreI2C = 0;
			break;
	default:
			E("%s: case default\n", __func__);
			break;
	}

	I("%s: irq = %d,  ven_gpio = %d,  firm_gpio = %d -\n", __func__, \
		gpio_get_value(pni->irq_gpio), pn544_isEn(), gpio_get_value(pni->firm_gpio));
	return count;
}

static DEVICE_ATTR(pn_temp1, 0664, pn_temp1_show, pn_temp1_store);


static ssize_t debug_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("debug_enable_show\n");

	ret = sprintf(buf, "nfc_is_debug=%d\n", nfc_is_debug);
	return ret;
}

static ssize_t debug_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &nfc_is_debug);
	return count;
}

static DEVICE_ATTR(debug_enable, 0664, debug_enable_show, debug_enable_store);

static ssize_t nxp_chip_alive_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s is %d\n", __func__, is_alive);
	ret = sprintf(buf, "%d\n", is_alive);
	return ret;
}

static DEVICE_ATTR(nxp_chip_alive, 0444, nxp_chip_alive_show, NULL);

static ssize_t nxp_uicc_swp_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s is %d\n", __func__, is_uicc_swp);
	ret = sprintf(buf, "%d\n", is_uicc_swp);
	return ret;
}

static ssize_t nxp_uicc_swp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &is_uicc_swp);
	return count;
}

static DEVICE_ATTR(nxp_uicc_swp, 0664, nxp_uicc_swp_show, nxp_uicc_swp_store);

#if FTM_MODE

static void pn544_hw_reset(void)
{
	pn544_Enable();
	msleep(50);
	pn544_Disable();
	msleep(50);
	pn544_Enable();
	msleep(50);
}

void nfc_nci_dump_data(unsigned char *data, int len) {
	int i = 0, j = 0;
	char temp[len*7];
	memset(temp, 0x00, len*7);
	for (i = 0, j = 0; i < len; i++)
		j += sprintf(temp + j, " 0x%02X", data[i]);
	I("%s\r\n", temp);
}




int nci_Reader(control_msg_pack *script, unsigned int scriptSize) {
	static control_msg_pack *previous = 0;
	static int res_achieved = 0;
	static int ntf_achieved = 0;
	static char expect_resp_header[2] = {0};
	static char expect_ntf_header[2] = {0};
	uint8_t receiverBuffer[MAX_NFC_DATA_SIZE] ={0};
	char nci_read_header[2] = {0};
	unsigned char nci_data_len = 0;
	unsigned int GOID = 0;
	int rf_support_len = 0;


	if (previous != script) {
		I("new command, reset flags.\r\n");
		previous = script;
		res_achieved = 0;
		ntf_achieved = 0;
		if (script->exp_resp_content != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_resp_header[0] = script->cmd[1] + 0x20;
			else
				expect_resp_header[0] = script->cmd[1];
			expect_resp_header[1] = script->cmd[2];
			I(": 0x%02X, 0x%02X\r\n", expect_resp_header[0], expect_resp_header[1]);
		}

		if (*(script->exp_ntf) != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = script->cmd[1] + 0x40;
			else if (0x00 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = 0x60;
			I("Expected NTF Header: 0x%02X\r\n", expect_ntf_header[0]);
		}
	}


	if ( pn544_RxData2(nci_read_header, 2) < 0) {
		I("I2C error while read out the NCI header.\r\n");
		return -255;
	} else {
		I("1. [R] NCI header read: 0x%02X, 0x%02X\r\n", nci_read_header[0], nci_read_header[1]);
		mdelay(NFC_READ_DELAY);
		if ( pn544_RxData2(&nci_data_len, 1) < 0) {
			I("I2C error while read out the NCI data length.\r\n");
			return -255;
		} else {
			I("2. [R] NCI data length read: %d\r\n", (int)nci_data_len);
			mdelay(NFC_READ_DELAY);
			if ( pn544_RxData2(receiverBuffer, nci_data_len) < 0) {
				I("I2C error while read out the NCI data.\r\n");
				return -255;
			} else {
				I("3. [R] NCI data: ");
				nfc_nci_dump_data(receiverBuffer, (int)nci_data_len);
			}
		}
	}

	
	
	if (0x40 == (nci_read_header[0] & 0xF0)) {
		GOID = nci_read_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_read_header[1];
		I("GOID: 0x%08X\r\n", GOID);

		switch (GOID) {
		case 0x0000: 
			I("Response CORE_RESET_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command. exp_len:%x\r\n", script->exp_resp_content[0]);
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			gDevice_info.NCI_version = receiverBuffer[1];
			break;
		case 0x0001: 
			I("Response CORE_INIT_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			rf_support_len = receiverBuffer[5];
			gDevice_info.NFCC_Features = ((unsigned int)receiverBuffer[4]) << 24 | ((unsigned int)receiverBuffer[3]) << 16 | ((unsigned int)receiverBuffer[2]) << 8 | receiverBuffer[1];
			gDevice_info.manufactor = receiverBuffer[12 + rf_support_len];
			gDevice_info.fwVersion = ((unsigned int)receiverBuffer[15 + rf_support_len]) << 8 | ((unsigned int)receiverBuffer[16 + rf_support_len]);
			I("FW Version 0x%07lX\r\n", gDevice_info.fwVersion);
			mfc_nfc_cmd_result = (int)gDevice_info.fwVersion;
			break;
		case 0x0103: 
			I("Response RF_DISCOVER_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			if (script->cmd[5] < 0x80)
				I("Start to detect Cards.\r\n");
			else
				I("Start to listen Reader.\r\n");
			
			expect_ntf_header[1] = 0x05;
			break;
		case 0x0200: 
			I("Response NFCEE_DISCOVER_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			
			expect_ntf_header[1] = 0x00;
			break;
		case 0x0201: 
			I("Response NFCEE_MODE_SET_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			
			expect_ntf_header[1] = 0x00;
			break;

#if FTM_NFC_CPLC
#endif 
		default:
			I("Response not defined.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			} else
				I("No response requirement.\r\n");
		}
	}

	
	if (0x00 == (nci_read_header[0] & 0xF0)) {
		I("Data Packet, Connection ID:0x%02X\r\n", (nci_read_header[0] & 0x0F));
		if (*(script->exp_resp_content)) {
			if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
				I("Response type matched with command.\r\n");
				if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
					I("Response matched with expected response, res_achieved set.\r\n");
					res_achieved = 1;
				} else {
					I("Not expected response! Quit now.\r\n");
					return -255;
				}
			} else {
				I("Command-Response type not matched, ignore.\r\n");
			}
		} else
			I("No response requirement.\r\n");
		if (0x00 == (nci_read_header[0] & 0xF0))
			expect_ntf_header[1] = 0x06;
	}

	
	if (0x60 == (nci_read_header[0] & 0xF0)) {
		GOID = nci_read_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_read_header[1];
		I("GOID: 0x%08X\r\n", GOID);

		switch (GOID) {
		case 0x0103:
			I("Notification RF_DISCOVER_NTF received.\r\n");
			if (*(script->exp_ntf)) { 
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					
					
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID = receiverBuffer[0];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol = receiverBuffer[1];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Technology = receiverBuffer[2];
					if (gDevice_info.target_rf_id == 255 &&
					 gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol == gDevice_info.protocol_set)
						gDevice_info.target_rf_id = gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID;

					if (receiverBuffer[nci_data_len - 1] == 0) {
						I("Last INTF_NTF reached.\r\n");
						I("Card detected!\r\n");
						select_rf_target[0].cmd[4] = gDevice_info.target_rf_id;
						select_rf_target[0].cmd[5] = gDevice_info.protocol_set;
						select_rf_target[0].cmd[6] = gDevice_info.intf_set;
					}
				}
			}
			gDevice_info.NTF_count++;
			break;
		case 0x0105: 
			I("Notification RF_INTF_ACTIVATED_NTF received.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						gDevice_info.activated_INTF = receiverBuffer[0];
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			}
			if (receiverBuffer[3] < 0x80)
				I("Card detected!\r\n");
			else
				I("Reader detected!\r\n");
			break;
		case 0x0200: 
			I("Notification NFCEE_DISCOVER_NTF received.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			}
			gDevice_info.HW_model = receiverBuffer[1];
			break;
#if FTM_NFC_CPLC
#endif 
		default:
			I("Notification not defined.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 1) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			} else
				I("No Notification requirement.\r\n");
		}
	}

	if (*(script->exp_resp_content) != 0) {
		if (res_achieved) {
			if (*(script->exp_ntf) != 0) {
				if (ntf_achieved) {
					return 1;
				} else {
					I("Notification requirement not achieve, stay at current command.\r\n");
					if (watchdogEn == 1)
						watchdog_counter = 0;

					return -1;
				}
			} else {
				I("No NTF requirement, step to next command.\r\n");
				return 1;
			}
		} else {
			I("Response requirement not achieve, stay at current command.\r\n");

			if (watchdogEn == 1)
				watchdog_counter = 0;

			return -1;
		}
	} else if (*(script->exp_ntf) != 0) {
		if (ntf_achieved) {
			return 1;
		} else {
			I("Notification requirement not achieve, stay at current command.\r\n");

			if (watchdogEn == 1)
				watchdog_counter = 0;

			return -1;
		}
	} else {
		I("No requirement, step to next command.\r\n");
		return 1;
	}
}

#define CHECK_READER(void) \
do { \
	if (watchdogEn == 1) {\
		if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {\
			I("watchdog timeout, command fail.\r\n");\
			goto TIMEOUT_FAIL; \
		} \
	} \
	if (gpio_get_value(pni->irq_gpio)) { \
		reader_resp = nci_Reader(&script[scriptIndex], scriptSize); \
		 \
		switch(reader_resp) { \
		case -255: \
			 \
			goto I2C_FAIL; \
			break; \
		case -1: \
			 \
			break; \
		case 0: \
			 \
			scriptIndex = 0; \
			break; \
		case 1: \
			 \
			scriptIndex++; \
			break; \
		default: \
			scriptIndex = reader_resp; \
		} \
	} \
} while(0)

int script_processor(control_msg_pack *script, unsigned int scriptSize) {
	int ret;
	int scriptIndex, reader_resp;
	int last_scriptIndex;
	struct pn544_dev *pni = pn_info;

	scriptSize = scriptSize/sizeof(control_msg_pack);

	I("script_processor script size: %d.\r\n", scriptSize);

	scriptIndex = 0;
	last_scriptIndex = -1;
	reader_resp = 1;

	do {
		if (reader_resp == -1) {
			CHECK_READER();
			mdelay(NFC_READ_DELAY);

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\r\n");
					goto TIMEOUT_FAIL;
				}

			continue;
		}

		if ( last_scriptIndex != scriptIndex) {
			I("script_processor pn544_TxData2()+\r\n");
			ret = pn544_TxData2(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
			I("script_processor pn544_TxData2()-\r\n");
			if (ret < 0) {
				E("%s, i2c Tx error!\n", __func__);
				nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
				goto I2C_FAIL;
				break;
			}
			else {
					I("[W] i2c wrote: ");
					nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
					mdelay(NFC_READ_DELAY + 20);
					last_scriptIndex = scriptIndex;
					I("script_processor CHECK_IRQ value :%d\r\n", gpio_get_value(pni->irq_gpio));
					CHECK_READER();
				}
		} else {
			CHECK_READER();

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\r\n");
					goto TIMEOUT_FAIL;
				}

		}
		mdelay(NFC_READ_DELAY);
	} while(scriptIndex < scriptSize);

	return 0;
I2C_FAIL:
	E("%s, I2C_FAIL!\n", __func__);
	mfc_nfc_cmd_result = -2;
	return 1;
TIMEOUT_FAIL:
	mfc_nfc_cmd_result = 0;
	return 1;
}

static ssize_t mfg_nfc_ctrl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s mfc_nfc_cmd_result is %d\n", __func__, mfc_nfc_cmd_result);
	ret = sprintf(buf, "%d\n\n", mfc_nfc_cmd_result);
	return ret;
}


static int mfg_nfc_test(int code)
{
	gDevice_info.NTF_count = 0;
	memset(gDevice_info.NTF_queue, 0x00, sizeof(gDevice_info.NTF_queue));
	gDevice_info.protocol_set = 4;
	gDevice_info.intf_set = 2;
	gDevice_info.target_rf_id = 255;
	mfc_nfc_cmd_result = -1;
	watchdog_counter = 0;
	watchdogEn = 1;
	I("%s: store value = %d\n", __func__, code);

	switch (code) {
	case 0:  
		I("%s: get nfcversion :\n", __func__);
		pn544_hw_reset();
		if (script_processor(nfc_version_script, sizeof(nfc_version_script)) == 0) {
			I("%s: store value = %d\n", __func__, code);
		}
		break;
	case 1:  
		I("%s: nfcreader test :\n", __func__);
		pn544_hw_reset();
		if (script_processor(nfc_reader_script, sizeof(nfc_reader_script)) == 0) {
			I("%s: store value = %d\n", __func__, code);
			mfc_nfc_cmd_result = 1;
		}
		break;
	case 2:  
		I("%s: nfccard test :\n", __func__);
		pn544_hw_reset();
		if (script_processor(nfc_card_script, sizeof(nfc_card_script)) == 0) {
			I("%s: store value = %d\n", __func__, code);
			mfc_nfc_cmd_result = 1;
		}
		break;
	case 3:
		I("%s: nfccard test 2 :\n", __func__);
		pn544_hw_reset();
		if (script_processor(nfc_card_script_plm, sizeof(nfc_card_script_plm)) == 0) {
			I("%s: store value = %d\n", __func__, code);
			mfc_nfc_cmd_result = 1;
		}
		break;
#if FTM_NFC_CPLC
#endif  
	default:
		E("%s: case default\n", __func__);
		break;
	}
	pn544_hw_reset();
	I("%s: END\n", __func__);
	return 0;
}

static ssize_t mfg_nfc_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	int code = -1;
	sscanf(buf, "%d", &code);
	ret = mfg_nfc_test(code);
	return count;
}

static ssize_t mfg_nfc_timeout_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
        int input_timeout = -1;
        sscanf(buf, "%d", &input_timeout);
	if (input_timeout >= 1) {
		watchdog_timeout = input_timeout;
	}
	else {
		watchdog_timeout = WATCHDOG_FTM_TIMEOUT_SEC;
	}
        return count;
}

static ssize_t mfg_nfc_timeout_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,"%d\n", watchdog_timeout);
}



static ssize_t mfg_nfcversion(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	watchdogEn = 1;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	ret = mfg_nfc_test(0);
	if (mfc_nfc_cmd_result > 0) {
		return scnprintf(buf, PAGE_SIZE,
			"NFC firmware version: 0x%07x\n", mfc_nfc_cmd_result);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}
}


static ssize_t mfg_nfcreader(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	watchdogEn = 1;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	ret = mfg_nfc_test(1);
	if (mfc_nfc_cmd_result == 1) {
		return scnprintf(buf, PAGE_SIZE,
			"%s Succeed\n",__func__);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}
}



static ssize_t mfg_nfccard(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	watchdogEn = 1;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	ret = mfg_nfc_test(2);
	if (mfc_nfc_cmd_result == 1) {
		return scnprintf(buf, PAGE_SIZE,
			"%s Succeed\n",__func__);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}
}


static DEVICE_ATTR(mfg_nfc_ctrl, 0660, mfg_nfc_ctrl_show, mfg_nfc_ctrl_store);
static DEVICE_ATTR(mfg_nfcversion, 0440, mfg_nfcversion, NULL);
static DEVICE_ATTR(mfg_nfcreader, 0440, mfg_nfcreader, NULL);
static DEVICE_ATTR(mfg_nfccard, 0440, mfg_nfccard, NULL);
static DEVICE_ATTR(mfg_nfc_timeout, 0660, mfg_nfc_timeout_show, mfg_nfc_timeout_store);
#endif  

static int pn544_parse_dt(struct device *dev, struct pn544_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	I("%s: Start\n", __func__);
	pn544_htc_parse_dt(dev);
	prop = of_find_property(dt, "nxp,ven_isinvert", NULL);
	if (prop) {
		of_property_read_u32(dt, "nxp,ven_isinvert", &pdata->ven_isinvert);
		printk(KERN_INFO "[NFC] %s:ven_isinvert = %d\n", __func__, pdata->ven_isinvert);
	}

	prop = of_find_property(dt, "nxp,isalive", NULL);
	if (prop) {
		of_property_read_u32(dt, "nxp,isalive", &is_alive);
		is_alive = pn544_htc_check_rfskuid(is_alive);
		printk(KERN_INFO "[NFC] %s:is_alive = %d\n", __func__, is_alive);
	}

	
	prop = of_find_property(dt, "nxp,irq-gpio", NULL);																						
	if (prop) {
		of_property_read_u32(dt, "nxp,irq-gpio", &pdata->irq_gpio);
		D("%s: irq_gpio = %d\n", __func__, pdata->irq_gpio);
	} else
		I("%s: irq_gpio not found", __func__);
	if (!gpio_is_valid(pdata->irq_gpio))
		E("DT:irq_gpio is not valid\n");
	else
		I("DT:irq_gpio=%d\n", pdata->irq_gpio);

	prop = of_find_property(dt, "nxp,ven-gpio", NULL);																						
	if (prop) {
		of_property_read_u32(dt, "nxp,ven-gpio", &pdata->ven_gpio);
		D("%s: ven_gpio = %d\n", __func__, pdata->ven_gpio);
	} else
		I("%s: ven_gpio not found", __func__);
	if (!gpio_is_valid(pdata->ven_gpio))
		E("DT:ven_gpio is not valid\n");
	else
		I("DT:ven_gpio=%d\n", pdata->ven_gpio);
	prop = of_find_property(dt, "nxp,fwdl-gpio", NULL);																						
	if (prop) {
		of_property_read_u32(dt, "nxp,fwdl-gpio", &pdata->firm_gpio);
		D("%s: firm_gpio = %d\n", __func__, pdata->firm_gpio);
	} else
		I("%s: firm_gpio not found", __func__);
	if (!gpio_is_valid(pdata->firm_gpio))
		E("DT:firm_gpio is not valid\n");
	else
		I("DT:firm_gpio=%d\n", pdata->firm_gpio);

	I("%s: End, irq_gpio:%d, ven_gpio:%d, firm_gpio:%d\n", __func__, pdata->irq_gpio, pdata->ven_gpio,pdata->firm_gpio);

	return 0;
}

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pni;
	struct device_node *node = NULL;
	u32 ints[2] = {0,0};

	I("%s() Start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	if (client->dev.of_node) {
		 platform_data = kzalloc(sizeof(*platform_data), GFP_KERNEL);
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
		 ret = pn544_parse_dt(&client->dev, platform_data);
		 if (ret) {
	                 E("%s : pn544_parse_dt fail\n", __func__);
	                 ret = -ENODEV;
	                 goto err_exit;
	         }
	} else {
		 platform_data = client->dev.platform_data;
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
	}
	
	if (!pn544_htc_pvdd_on ()) {
		E("%s : Turn On NFC_PVDD fail\n", __func__);
		ret = -ENODEV;
		goto err_exit;
	}

	
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret) {
		E("%s : request_1 gpio%d fail\n",
			__func__, platform_data->irq_gpio);
		ret = -ENODEV;
		goto err_exit;
	}

	
	ret = gpio_request(platform_data->ven_gpio, "nfc_en");
	if (ret) {
		E("%s : request_2 gpio %d fail\n",
			__func__, platform_data->ven_gpio);
		ret = -ENODEV;
		goto err_request_gpio_ven;
	}
	

	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret) {
		E("%s : request_3 gpio %d fail\n",
			__func__, platform_data->firm_gpio);
		ret = -ENODEV;
		goto err_request_gpio_firm;
	}

	pni = kzalloc(sizeof(struct pn544_dev), GFP_KERNEL);
	if (pni == NULL) {
		dev_err(&client->dev, \
				"pn544_probe : failed to allocate \
				memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn_info = pni;

	if (platform_data->gpio_init != NULL) {
		I("%s: gpio_init\n", __func__);
		platform_data->gpio_init();
	}

	pni->irq_gpio = platform_data->irq_gpio;
	pni->ven_gpio  = platform_data->ven_gpio;
	pni->firm_gpio  = platform_data->firm_gpio;
	pni->client   = client;
	pni->client->addr = (pni->client->addr & I2C_MASK_FLAG)| I2C_DMA_FLAG;
	pni->gpio_init = platform_data->gpio_init;
	pni->ven_enable = !platform_data->ven_isinvert;
	pni->boot_mode = pn544_htc_get_bootmode();
	pni->isReadBlock = false;
	I("%s : irq_gpio:%d, ven_gpio:%d, firm_gpio:%d, ven_enable:%d\n", __func__, pni->irq_gpio, pni->ven_gpio, pni->firm_gpio, pni->ven_enable);

	I("%s(), dma_alloc_coherent()\n", __func__);

	I2CDMABuf_va_read = (u8 *)dma_alloc_coherent(&pni->client->adapter->dev, DMA_SIZE, (dma_addr_t *)&I2CDMABuf_pa_read, GFP_KERNEL);
	if (I2CDMABuf_va_read == NULL) {
		I2CDMABuf_pa_read = NULL;
		pr_err("%s: allocate phsical memory error\n", __func__);
		return -ENOMEM;
	}

	I2CDMABuf_va_write = (u8 *)dma_alloc_coherent(&pni->client->adapter->dev, DMA_SIZE, (dma_addr_t *)&I2CDMABuf_pa_write, GFP_KERNEL);
	if (I2CDMABuf_va_write == NULL) {
		I2CDMABuf_pa_write = NULL;
		pr_err("%s: allocate phsical memory error\n", __func__);
		return -ENOMEM;
	}

	

	
	init_waitqueue_head(&pni->read_wq);
	mutex_init(&pni->read_mutex);
	spin_lock_init(&pni->irq_enabled_lock);

	I("%s: init io_wake_lock\n", __func__);
	wake_lock_init(&pni->io_wake_lock, WAKE_LOCK_SUSPEND, PN544_I2C_NAME);

	pni->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pni->pn544_device.name = "pn544";
	pni->pn544_device.fops = &pn544_dev_fops;

#if 1
	ret = misc_register(&pni->pn544_device);
#else
	ret = 0;
#endif
	if (ret) {
		E("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	I("%s: config NFC_VEN pin\n", __func__); 
	mt_set_gpio_mode(pni->ven_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(pni->ven_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(pni->ven_gpio, GPIO_OUT_ZERO);

	I("%s: config NFC_IRQ pin\n", __func__); 
	mt_set_gpio_mode(pni->irq_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(pni->irq_gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(pni->irq_gpio, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(pni->irq_gpio, GPIO_PULL_DOWN);

	I("%s: config NFC_FWDL pin\n", __func__); 
	mt_set_gpio_mode(pni->firm_gpio, GPIO_MODE_00);
	mt_set_gpio_dir(pni->firm_gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(pni->firm_gpio, GPIO_OUT_ZERO);


	pni->irq_enabled = true;

	I("%s: request_irq!!!!!\n", __func__);

	node = of_find_compatible_node(NULL, NULL, "mediatek, IRQ_NFC-eint");
	if(node){
	  of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
	  gpio_set_debounce(ints[0], ints[1]);
	
	  client->irq = irq_of_parse_and_map(node, 0);
	
	  ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, "IRQ_NFC-eint", pni);
	  if(ret > 0){
		ret = -1;
		E("request_irq IRQ LINE NOT AVAILABLE!.\n");
	  }
	}else{
	  E("request_irq can not find touch eint device node!.\n");
	  ret = -1;
	}

	I("%s: pn544_disable_irq(), irq:%d \n", __func__,pni->client->irq);
	pn544_disable_irq(pni);

	i2c_set_clientdata(client, pni);

	pni->pn544_class = class_create(THIS_MODULE, "NFC_sensor");
	if (IS_ERR(pni->pn544_class)) {
		ret = PTR_ERR(pni->pn544_class);
		pni->pn544_class = NULL;
		E("%s : class_create failed\n", __func__);
		goto err_create_class;
	}

	pni->pn_dev = device_create(pni->pn544_class, NULL, 0, "%s", "pn544");
	if (unlikely(IS_ERR(pni->pn_dev))) {
		ret = PTR_ERR(pni->pn_dev);
		pni->pn_dev = NULL;
		E("%s : device_create failed\n", __func__);
		goto err_create_pn_device;
	}

	
	ret = device_create_file(pni->pn_dev, &dev_attr_pn_temp1);
	if (ret) {
		E("%s : device_create_file dev_attr_pn_temp1 failed\n", __func__);
		goto err_create_pn_file;
	}

	ret = device_create_file(pni->pn_dev, &dev_attr_debug_enable);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_debug_enable failed\n");
		goto err_create_pn_file;
	}

	pni->comn_dev = device_create(pni->pn544_class, NULL, 0, "%s", "comn");
	if (unlikely(IS_ERR(pni->comn_dev))) {
		ret = PTR_ERR(pni->comn_dev);
		pni->comn_dev = NULL;
		E("%s : device_create failed\n", __func__);
		goto err_create_pn_device;
	}

	ret = device_create_file(pni->comn_dev, &dev_attr_nxp_uicc_swp);
	if (ret) {
		E("pn544_probe device_create_file dev_attrnxp_uicc_swp failed\n");
	}
#if FTM_MODE
	watchdog_timeout = WATCHDOG_FTM_TIMEOUT_SEC; 
	I("%s: device_create_file for FTM mode+\n", __func__);
	ret = device_create_file(pni->comn_dev, &dev_attr_mfg_nfc_ctrl);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_mfg_nfc_ctrl failed\n");
	}

	ret = device_create_file(pni->comn_dev, &dev_attr_mfg_nfcversion);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_mfg_nfcversion failed\n");
	}

	ret = device_create_file(pni->comn_dev, &dev_attr_mfg_nfcreader);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_mfg_nfcreader\n");
	}

	ret = device_create_file(pni->comn_dev, &dev_attr_mfg_nfccard);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_mfg_nfccard failed\n");
	}

    ret = device_create_file(pni->comn_dev, &dev_attr_mfg_nfc_timeout);
    if (ret) {
        E("pn544_probe device_create_file dev_attr_mfg_nfc_timeout failed\n");
    }
    I("%s: device_create_file for FTM mode done -\n", __func__);
#endif  

	if (is_alive == 0) {
		I("%s: Without NFC, device_create_file dev_attr_nxp_chip_alive \n", __func__);
		ret = device_create_file(pni->pn_dev, &dev_attr_nxp_chip_alive);
		if (ret) {
			E("pn544_probe device_create_file dev_attr_nxp_chip_alive failed\n");
			goto err_create_pn_file;
		}
	} else {
		if(pni->boot_mode == NFC_BOOT_MODE_OFF_MODE_CHARGING ) {
			pn544_htc_off_mode_charging();
		}
	}
	I("%s: Probe success! is_alive : %d, is_uicc_swp : %d\n", __func__, is_alive, is_uicc_swp);
	return 0;

err_create_pn_file:
	device_unregister(pni->pn_dev);
err_create_pn_device:
	class_destroy(pni->pn544_class);
err_create_class:
err_request_irq_failed:
	misc_deregister(&pni->pn544_device);
err_misc_register:
	mutex_destroy(&pni->read_mutex);
	wake_lock_destroy(&pni->io_wake_lock);
	kfree(pni);
	pn_info = NULL;
	gpio_free(platform_data->firm_gpio);
err_request_gpio_firm:
	gpio_free(platform_data->ven_gpio);
err_request_gpio_ven:
	gpio_free(platform_data->irq_gpio);
err_exit:
	E("%s: prob fail\n", __func__);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;
	I("%s:\n", __func__);

	if (I2CDMABuf_va_read != NULL) {
		dma_free_coherent(&client->adapter->dev, DMA_SIZE, I2CDMABuf_va_read, (dma_addr_t)I2CDMABuf_pa_read);
		I2CDMABuf_va_read = NULL;
		I2CDMABuf_pa_read = NULL;
	}
	if (I2CDMABuf_va_write != NULL) {
		dma_free_coherent(&client->adapter->dev, DMA_SIZE, I2CDMABuf_va_write, (dma_addr_t)I2CDMABuf_pa_write);
		I2CDMABuf_va_write = NULL;
		I2CDMABuf_pa_write = NULL;
	}
	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	wake_lock_destroy(&pn544_dev->io_wake_lock);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);
	pn_info = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int pn544_suspend(struct i2c_client *client, pm_message_t state)
{
	struct pn544_dev *pni = pn_info;

        I("%s: irq = %d, ven_gpio = %d, isEn = %d, isReadBlock =%d\n", __func__, \
                gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), pn544_isEn(), pni->isReadBlock);

	if (pni->ven_value && pni->isReadBlock && is_alive) {
		pni->irq_enabled = true;
		enable_irq(pni->client->irq);
		irq_set_irq_wake(pni->client->irq, 1);
	}

	return 0;
}

static int pn544_resume(struct i2c_client *client)
{
	struct pn544_dev *pni = pn_info;

        I("%s: irq = %d, ven_gpio = %d, isEn = %d, isReadBlock =%d\n", __func__, \
                gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), pn544_isEn(), pni->isReadBlock);

	if (pni->ven_value && pni->isReadBlock && is_alive) {
		pn544_disable_irq(pni);
		irq_set_irq_wake(pni->client->irq, 0);
	}

	return 0;
}
#endif

static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

static struct of_device_id pn544_match_table[] = {
	{ .compatible = "nxp,pn544-nfc",},
	{ },
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn544",
		.of_match_table = pn544_match_table,
	},
#if CONFIG_PM
	.suspend	= pn544_suspend,
	.resume		= pn544_resume,
#endif
};


static int __init pn544_dev_init(void)
{
	I("%s: Loading pn544 driver\n", __func__);
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	I("%s: Unloading pn544 driver\n", __func__);
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
