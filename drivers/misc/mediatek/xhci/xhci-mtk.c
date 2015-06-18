#include <xhci.h>
#include <linux/xhci/xhci-mtk.h>
#include <linux/xhci/xhci-mtk-power.h>
#include <linux/xhci/xhci-mtk-scheduler.h>
#include <linux/kernel.h>	
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#ifdef CONFIG_USB_MTK_DUALMODE
#include <mach/eint.h>
#include <linux/irq.h>
#include <linux/switch.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <mach/mt_boot.h>
#undef DRV_Reg32
#undef DRV_WriteReg32
#include <mach/battery_meter.h>
#ifdef CONFIG_USBIF_COMPLIANCE
#define CONFIG_MTK_OTG_PMIC_BOOST_5V
#endif


#undef DRV_Reg32
#undef DRV_WriteReg32
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_adc.h>
#include <linux/usb/htc_attr.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#ifdef CONFIG_PROJECT_PHY
#include <linux/mu3phy/mtk-phy-asic.h>
#endif
#endif

#include <hub.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <asm/atomic.h>
#include <linux/usb/hcd.h>
#include <mach/mt_chip.h>

#ifdef CONFIG_HTC_MHL_DETECTION
#include "hdmi_cust.h"
#include <mach/battery_common.h>
#endif

#ifdef CONFIG_MTK_FPGA
#include <linux/mu3phy/mtk-phy.h>
#endif

#define mtk_xhci_mtk_log(fmt, args...) \
    printk("%s(%d): " fmt, __func__, __LINE__, ##args)

#define RET_SUCCESS 0
#define RET_FAIL 1

struct xhci_hcd *mtk_xhci;
static spinlock_t *mtk_hub_event_lock;
static struct list_head* mtk_hub_event_list;
static int mtk_ep_count;

static struct wake_lock mtk_xhci_wakelock;

#ifdef CONFIG_MTK_FPGA
void __iomem *u3_base;
void __iomem *u3_sif_base;
void __iomem *u3_sif2_base;
void __iomem *i2c1_base;
#endif


#ifdef CONFIG_USB_MTK_DUALMODE
enum idpin_state {
	IDPIN_OUT,
	IDPIN_IN_HOST,
	IDPIN_IN_DEVICE,
};

static DEFINE_MUTEX(otg_power_sem); 

static int mtk_idpin_irqnum;
static enum idpin_state mtk_idpin_cur_stat = IDPIN_OUT;
static struct switch_dev mtk_otg_state;

static struct switch_dev dock_switch = {
	.name = "dock",
};

static struct delayed_work mtk_xhci_delaywork;
int mtk_iddig_debounce = 100;
module_param(mtk_iddig_debounce, int, 0644);

static void mtk_set_iddig_out_detect(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_HIGH);
	enable_irq(mtk_idpin_irqnum);
}

static void mtk_set_iddig_in_detect(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_LOW);
	enable_irq(mtk_idpin_irqnum);
}

static bool mtk_is_charger_4_vol(void)
{
	int vol = battery_meter_get_charger_voltage();
	mtk_xhci_mtk_log("voltage(%d)\n", vol);

#if defined(CONFIG_USBIF_COMPLIANCE) || defined(CONFIG_MTK_USB_EVB_BOARD)
	return false ;
#else
	return (vol > 4400) ? true : false;
#endif
}

bool mtk_is_usb_id_pin_short_gnd(void)
{
	return (mtk_idpin_cur_stat != IDPIN_OUT) ? true : false;
}

#ifdef CONFIG_MTK_OTG_PMIC_BOOST_5V
#define PMIC_REG_BAK_NUM (10)

extern U32 pmic_read_interface(U32 RegNum, U32 * val, U32 MASK, U32 SHIFT);
extern U32 pmic_config_interface(U32 RegNum, U32 val, U32 MASK, U32 SHIFT);

U32 pmic_bak_regs[PMIC_REG_BAK_NUM][2] = {
		{0x8D22, 0}, {0x8D14, 0}, {0x803C, 0}, {0x8036, 0}, {0x8D24, 0},
		{0x8D16, 0}, {0x803A, 0}, {0x8046, 0}, {0x803E, 0}, {0x8044, 0}
	};

static void pmic_save_regs(void){
	int i;

	for(i = 0; i < PMIC_REG_BAK_NUM; i++){
		pmic_read_interface(pmic_bak_regs[i][0], &pmic_bak_regs[i][1], 0xffffffff, 0);
	}
}

static void pmic_restore_regs(void){
	int i;

	for(i = 0; i < PMIC_REG_BAK_NUM; i++){
		pmic_config_interface(pmic_bak_regs[i][0], pmic_bak_regs[i][1], 0xffffffff, 0);
	}
}

static void mtk_enable_pmic_otg_mode(void)
{
	int val;
	mutex_lock(&otg_power_sem);

	
	pmic_save_regs();

	pmic_config_interface(0x8D22, 0x1, 0x1, 12);
	pmic_config_interface(0x8D14, 0x1, 0x1, 12);
	pmic_config_interface(0x803C, 0x3, 0x3, 0);
	pmic_config_interface(0x803C, 0x2, 0x3, 2);
	pmic_config_interface(0x803C, 0x1, 0x1, 14);
	pmic_config_interface(0x8036, 0x0, 0x0, 0);
	pmic_config_interface(0x8D24, 0xf, 0xf, 12);
	pmic_config_interface(0x8D16, 0x1, 0x1, 15);
	pmic_config_interface(0x803A, 0x1, 0x1, 6);
	pmic_config_interface(0x8046, 0x00A0, 0xffff, 0);
	pmic_config_interface(0x803E, 0x1, 0x1, 2);
	pmic_config_interface(0x803E, 0x1, 0x1, 3);
	pmic_config_interface(0x803E, 0x3, 0x3, 8);
	pmic_config_interface(0x803E, 0x1, 0x1, 10);
	pmic_config_interface(0x8044, 0x3, 0x3, 0);
	pmic_config_interface(0x8044, 0x3, 0x7, 8);
	pmic_config_interface(0x8044, 0x1, 0x1, 11);

	pmic_config_interface(0x809C, 0x8000, 0xFFFF, 0);

	val = 0;
	while (val == 0) {
		pmic_read_interface(0x809A, &val, 0x1, 15);
	}

	pmic_config_interface(0x8084, 0x1, 0x1, 0);
	mdelay(50);

	val = 0;
	while (val == 0) {
		pmic_read_interface(0x8060, &val, 0x1, 14);
	}

	mtk_xhci_mtk_log("set pmic power on, done\n");
	mutex_unlock(&otg_power_sem);
}

static void mtk_disable_pmic_otg_mode(void)
{
	int val;
	mutex_lock(&otg_power_sem);

	pmic_config_interface(0x8068, 0x0, 0x1, 0);
	pmic_config_interface(0x8084, 0x0, 0x1, 0);
	mdelay(50);
	pmic_config_interface(0x8068, 0x0, 0x1, 1);

	val = 1;
	while (val == 1) {
		pmic_read_interface(0x805E, &val, 0x1, 4);
	}

	#if 0
	pmic_config_interface(0x809E, 0x8000, 0xFFFF, 0);

	val = 1;
	while (val == 1) {
		pmic_read_interface(0x809A, &val, 0x1, 15);
	}
	#endif

	
	pmic_restore_regs();

	mtk_xhci_mtk_log("set pimc power off, done\n");
	mutex_unlock(&otg_power_sem);
}

#endif

void mtk_hub_event_steal(spinlock_t *lock, struct list_head* list){
	mtk_hub_event_lock = lock;
	mtk_hub_event_list = list;
}

void mtk_ep_count_inc(void){
	mtk_ep_count++;
}

void mtk_ep_count_dec(void){
	mtk_ep_count--;
}

int mtk_is_hub_active(void){
	struct usb_hcd *hcd = xhci_to_hcd(mtk_xhci);
	struct usb_device *rhdev = hcd->self.root_hub;
	struct usb_hub *hub = usb_hub_to_struct_hub(rhdev);

	bool ret = true;

	spin_lock_irq(mtk_hub_event_lock);
	if((mtk_ep_count == 0) && (list_empty(&hub->event_list) == 1) && (atomic_read(&(hub->kref.refcount)) == 1)){
		ret = false;
	}
	spin_unlock_irq(mtk_hub_event_lock);

	return ret;
}


static int mtk_xhci_hcd_init(void)
{
	int retval;

	retval = xhci_register_plat();
	if (retval < 0) {
		printk(KERN_ERR "Problem registering platform driver.\n");
		return retval;
	}
	retval = xhci_attrs_init();
	if (retval < 0) {
		printk(KERN_ERR "Problem creating xhci attributes.\n");
		goto unreg_plat;
	}
	BUILD_BUG_ON(sizeof(struct xhci_doorbell_array) != 256 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_slot_ctx) != 8 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_ep_ctx) != 8 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_stream_ctx) != 4 * 32 / 8);
	BUILD_BUG_ON(sizeof(union xhci_trb) != 4 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_erst_entry) != 4 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_cap_regs) != 7 * 32 / 8);
	BUILD_BUG_ON(sizeof(struct xhci_intr_reg) != 8 * 32 / 8);
	
	BUILD_BUG_ON(sizeof(struct xhci_run_regs) != (8 + 8 * 128) * 32 / 8);
	return 0;

unreg_plat:
	xhci_unregister_plat();
	return retval;
}

static void mtk_xhci_hcd_cleanup(void)
{
	xhci_unregister_plat();
	xhci_attrs_exit();
}

static void mtk_xhci_imod_set(u32 imod)
{
	u32 temp;

	temp = xhci_readl(mtk_xhci, &mtk_xhci->ir_set->irq_control);
	temp &= ~0xFFFF;
	temp |= imod;
	xhci_writel(mtk_xhci, temp, &mtk_xhci->ir_set->irq_control);
}

extern void usb20_pll_settings(bool host, bool forceOn);

static int mtk_xhci_driver_load(void)
{
	int ret = 0;

	
	usb_phy_recover(0);
	ret = mtk_xhci_hcd_init();
	if (ret || !mtk_xhci) {
		ret = -ENXIO;
		goto _err;
	}

	
	mtk_xhci_imod_set(0x30);

#ifdef CONFIG_USBIF_COMPLIANCE
	mtk_enable_pmic_otg_mode();
	enableXhciAllPortPower(mtk_xhci);
#else
#ifdef CONFIG_MTK_OTG_PMIC_BOOST_5V
	mtk_enable_pmic_otg_mode();

	
	usb20_pll_settings(true, true);
#else
	enableXhciAllPortPower(mtk_xhci);
#endif
#endif

	return 0;

_err:
	mtk_xhci_mtk_log("ret(%d), mtk_xhci(0x%p)\n", ret, mtk_xhci);
	usb_phy_savecurrent(1);
	return ret;
}

static void mtk_xhci_disPortPower(void)
{
#ifdef CONFIG_USBIF_COMPLIANCE
	mtk_disable_pmic_otg_mode();
	disableXhciAllPortPower(mtk_xhci);
#else
	#ifdef CONFIG_MTK_OTG_PMIC_BOOST_5V
	mtk_disable_pmic_otg_mode();
	#else
	disableXhciAllPortPower(mtk_xhci);
	#endif
#endif
}

static void mtk_xhci_driver_unload(void)
{
	mtk_xhci_hcd_cleanup();
	
	usb_phy_savecurrent(1);
}

void otg_oc_handler(void)
{
	kal_uint32 otg_mode;
	pmic_read_interface(0x8084, &otg_mode, 0x1, 0);
	if(otg_mode) {
#ifdef CONFIG_USBIF_COMPLIANCE
		mtk_disable_pmic_otg_mode();
		disableXhciAllPortPower(mtk_xhci);
#else
	#ifdef CONFIG_MTK_OTG_PMIC_BOOST_5V
		mtk_disable_pmic_otg_mode();
	#else
		disableXhciAllPortPower(mtk_xhci);
	#endif
#endif
	}
	return;
}

static int htc_get_usbid_adc(int channel);

static ssize_t adc_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int adc;

	adc = htc_get_usbid_adc(AUXADC_USBID_ADC_CHANNEL);
	printk("[Cable][%s] ADC = %d\n", __func__, adc);
	return sprintf(buf, "%d\n", adc);
}
static DEVICE_ATTR(adc, S_IRUGO | S_IWUSR, adc_status_show, NULL);

u32 ints[2] = {0,0};
static ssize_t USB_ID_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int id;

	id =  mt_get_gpio_in(ints[0]);

	printk("[Cable][%s] USB_ID_status = %d\n", __func__, id);
	return sprintf(buf, "%d\n", id);
}
static DEVICE_ATTR(USB_ID_status, S_IRUGO, USB_ID_status_show, NULL);

void mtk_xhci_switch_init(void)
{
	mtk_otg_state.name = "otg_state";
	mtk_otg_state.index = 0;
	mtk_otg_state.state = 0;
	int ret;

#ifndef CONFIG_USBIF_COMPLIANCE
	if (switch_dev_register(&mtk_otg_state))
		mtk_xhci_mtk_log("switch_dev_register fail\n");
	else
		mtk_xhci_mtk_log("switch_dev register success\n");
#endif

	if (switch_dev_register(&dock_switch) < 0) {
		mtk_xhci_mtk_log("[Cable][%s] fail to register dock switch!\n", __func__);
		return;
	}
	ret = device_create_file(dock_switch.dev, &dev_attr_adc);
	if (ret != 0)
		mtk_xhci_mtk_log("[Cable][%s] dev_attr_adc failed\n", __func__);
	ret = device_create_file(dock_switch.dev, &dev_attr_USB_ID_status);
	if (ret != 0)
		mtk_xhci_mtk_log("[Cable][%s] dev_attr_USB_ID_status failed\n", __func__);

}

#ifdef CONFIG_HTC_MHL_DETECTION
void mhl_status_notifier_func(bool isMHL, int charging_type)
{
	if (isMHL) {
		if ((mtk_idpin_cur_stat == IDPIN_IN_HOST) && charging_type) {
			mtk_disable_pmic_otg_mode();
			mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
			mtk_xhci_mtk_log("[Cable][%s] Disable PortPower during MHL connection\n", __func__);
		}

		
		if (charging_type)
			htc_batt_mhl_charge_set(charging_type);

	} else {
		if ((mtk_idpin_cur_stat == IDPIN_IN_HOST)) {
			mtk_disable_pmic_otg_mode();
			mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
			mtk_xhci_mtk_log("[Cable][%s] Disable PortPower when MHL disconnection\n", __func__);
		}

		
		switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);

		mtk_idpin_cur_stat = IDPIN_OUT;
		mtk_set_iddig_in_detect();
		mtk_xhci_mtk_log("[Cable][%s] MHL disconnect, enable IN detect\n", __func__);

		
		htc_batt_mhl_charge_set(charging_type);
	}
}
static struct t_mhl_status_notifier mhl_status_notifier = {
	.func = mhl_status_notifier_func,
	.isMHL = false,
	.charging_type = 0,
};
#endif

extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
extern int mfg_check_white_accessory(int accessory_type);
#define ADC_DELAY HZ/4
#define ADC_RETRY 3

static int htc_get_usbid_adc(int channel)
{
	int ret;
	int id_volt = 0;
	IMM_GetOneChannelValue_Cali(channel, &id_volt);
	return id_volt/1000;
}

static int htc_second_detect()
{
	int adc_value, acce_type, id_status = 0;
	mt_set_gpio_dir(ints[0], GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(ints[0], 1);
	mt_set_gpio_out(ints[0], 1);
	id_status = mt_get_gpio_in(ints[0]);
	printk("[2nd] id_status = %d\n", id_status);

	adc_value = htc_get_usbid_adc(AUXADC_USBID_ADC_CHANNEL);
	printk("[2nd] accessory adc = %d\n", adc_value);
	if ((adc_value >= 645 && adc_value <= 1010))
#ifdef CONFIG_HTC_MHL_DETECTION
		acce_type = DOCK_STATE_MHL; 
#else
		acce_type = DOCK_STATE_UNDEFINED; 
#endif
	else if (adc_value <= 200)
		acce_type = DOCK_STATE_USB_HOST; 
	else
		acce_type = DOCK_STATE_UNDEFINED; 
	mt_set_gpio_out(ints[0], 0);
	mt_set_gpio_pull_enable(ints[0], 0);
	mt_set_gpio_dir(ints[0], GPIO_DIR_IN);

	id_status = mt_get_gpio_in(ints[0]);
	printk("[2nd] id_status = %d\n", id_status);

	return acce_type;
}

static int htc_cable_detect_get_type(void)
{
	int id_status = 0, adc_value, acce_type, ret;
	static int prev_type, stable_count;
	id_status = mt_get_gpio_in(ints[0]);
	if (stable_count >= ADC_RETRY)
		stable_count = 0;
	if (id_status == 0) {
		printk("%s: id pin low\n", __func__);
		adc_value = htc_get_usbid_adc(AUXADC_USBID_ADC_CHANNEL);
		printk("[1st] accessory adc = %d\n", adc_value);
		if (adc_value > -100 && adc_value < 90)
					acce_type = htc_second_detect();
		else {
			if (adc_value >= 90 && adc_value < 150)
				acce_type = DOCK_STATE_CAR; 
			else if (adc_value > 250 && adc_value < 350)
				acce_type = DOCK_STATE_USB_HEADSET;
			else if (adc_value > 370 && adc_value < 450)
				acce_type = DOCK_STATE_DMB;
			else if (adc_value > 550 && adc_value < 900)
				acce_type = DOCK_STATE_DESK;
			else
				acce_type = DOCK_STATE_UNDEFINED; 
		}

	} else {
		printk("%s: id pin high\n", __func__);
		acce_type = DOCK_STATE_UNDOCKED; 
	}
	if (prev_type == acce_type)
		stable_count++;
	else
		stable_count = 0;
	printk("%s prev_type %d, acce_type %d, stable_count %d\n",
				__func__, prev_type, acce_type, stable_count);

	prev_type = acce_type;

	ret = (stable_count >= ADC_RETRY) ? acce_type : -2;

#ifdef CONFIG_HTC_MHL_DETECTION
	
	if (ret == DOCK_STATE_MHL)
		prev_type = 0;
#endif
	return ret;

#if (0)
	int id_pin, adc, type;
	static int prev_type, stable_count;

	if (stable_count >= ADC_RETRY)
		stable_count = 0;

	id_pin = gpio_get_value_cansleep(pInfo->usb_id_pin_gpio);
	if (id_pin == 0 || pInfo->cable_redetect) {
		CABLE_INFO("%s: id pin low\n", __func__);
		adc = cable_detect_get_adc();
		CABLE_INFO("[1st] accessory adc = %d\n", adc);
		if (adc > -100 && adc < 100)
			type = second_detect(pInfo);
		else {
			if (adc > 120 && adc < 250)
				type = DOCK_STATE_CAR;
			else if (adc > 370 && adc < 440)
				type = DOCK_STATE_USB_HEADSET;
			else if (adc > 440 && adc < 550)
				type = DOCK_STATE_DMB;
			else if (adc > 550 && adc < 900)
				type = DOCK_STATE_DESK;
			else
				type = DOCK_STATE_UNDEFINED;
		}
	} else {
		CABLE_INFO("%s: id pin high\n", __func__);
		type = DOCK_STATE_UNDOCKED;
	}

	if (prev_type == type)
		stable_count++;
	else
		stable_count = 0;

	CABLE_INFO("%s prev_type %d, type %d, stable_count %d\n",
				__func__, prev_type, type, stable_count);

	prev_type = type;
	return (stable_count >= ADC_RETRY) ? type : -2;
#endif
}

void mtk_xhci_mode_switch(struct work_struct *work)
{
	static bool is_load = false;
	static bool is_pwoff = false;
	int ret = 0;
	int id_volt = 0, acce_type = 0;
	static int pre_acce_type = 0;
	int value = 0;


	if (mtk_idpin_cur_stat == IDPIN_OUT) {
#ifdef CONFIG_HTC_MHL_DETECTION
		
		si_reset_mhl(0);
#endif

		acce_type = htc_cable_detect_get_type();
		if (acce_type == -2) {
			schedule_delayed_work_on(0, &mtk_xhci_delaywork, ADC_DELAY);
			return;
		}
#ifdef CONFIG_HTC_MHL_DETECTION
		
		si_reset_mhl(1);
#endif
		switch (acce_type) {
			case DOCK_STATE_CAR:
				
				printk("[Cable][%s] Car kit inserted\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_CAR);
				pre_acce_type = acce_type;
				mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
				
				mtk_set_iddig_out_detect();
				goto done;
				break;
#ifdef CONFIG_HTC_MHL_DETECTION
			case DOCK_STATE_MHL:
				
				msleep(40);
				
				printk("[Cable][%s] MHL inserted\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_MHL);
				pre_acce_type = 0;
				mtk_idpin_cur_stat = (mtk_is_charger_4_vol())? IDPIN_IN_DEVICE : IDPIN_IN_HOST;
				if (si_wakeup_mhl() != 0) {
					pre_acce_type = acce_type;
					mtk_set_iddig_out_detect();
					printk("[Cable][%s] MHL init fail\n", __func__);
				} else if (mtk_idpin_cur_stat == IDPIN_IN_HOST) {
					mtk_enable_pmic_otg_mode();
					printk("[Cable][%s] Output power to MHL Vbus\n", __func__);
				}
				goto done;
				break;
#endif
			case DOCK_STATE_USB_HEADSET:
				printk("[Cable][%s] USB headset inserted\n", __func__);
				pre_acce_type = acce_type;
				mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
				
				mtk_set_iddig_out_detect();
				goto done;
				break;
			case DOCK_STATE_DMB:
				printk("[Cable][%s] DMB inserted\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_DMB);
				pre_acce_type = acce_type;
				mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
				
				mtk_set_iddig_out_detect();
				goto done;
				break;
			case DOCK_STATE_DESK:
				printk("[Cable][%s] cradle inserted\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_DESK);
				pre_acce_type = acce_type;
				mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
				
				mtk_set_iddig_out_detect();
				goto done;
				break;
			case DOCK_STATE_USB_HOST:
				printk("[Cable][%s] USB Host inserted\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_USB_HOST);
				pre_acce_type = acce_type;
				is_load = false;
				
				mtk_idpin_cur_stat = (mtk_is_charger_4_vol())? IDPIN_IN_DEVICE : IDPIN_IN_HOST;
				
				mtk_set_iddig_out_detect();

				if (mtk_idpin_cur_stat == IDPIN_IN_DEVICE)
					goto done;

				ret = mtk_xhci_driver_load();
				if (!ret) {
					is_load = true;
					mtk_xhci_wakelock_lock();
#ifndef CONFIG_USBIF_COMPLIANCE
					switch_set_state(&mtk_otg_state, 1);
#endif
				}
				break;
			default:
				printk("[Cable][%s] Unknown cable inserted\n", __func__);
				pre_acce_type = acce_type;
				mtk_idpin_cur_stat = IDPIN_IN_DEVICE;
				
				mtk_set_iddig_out_detect();
				goto done;
				break;
		}
#if 0
		is_load = false;
		
		mtk_idpin_cur_stat = (mtk_is_charger_4_vol())? IDPIN_IN_DEVICE : IDPIN_IN_HOST;
		
		mtk_set_iddig_out_detect();

		if (mtk_idpin_cur_stat == IDPIN_IN_DEVICE)
			goto done;

		ret = mtk_xhci_driver_load();
		if (!ret) {
			is_load = true;
			mtk_xhci_wakelock_lock();
#ifndef CONFIG_USBIF_COMPLIANCE
			switch_set_state(&mtk_otg_state, 1);
#endif
		}
#endif

	} else {		
		if (is_load) {
			if(!is_pwoff)
				mtk_xhci_disPortPower();
			
			
			usb20_pll_settings(true, false);

			mtk_xhci_driver_unload();
			is_pwoff = false;
			is_load = false;
#ifndef CONFIG_USBIF_COMPLIANCE
			switch_set_state(&mtk_otg_state, 0);
#endif
			mtk_xhci_wakelock_unlock();
		}

		switch (pre_acce_type) {
			case DOCK_STATE_CAR:
				printk("[Cable][%s] Car kit removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
#ifdef CONFIG_HTC_MHL_DETECTION
			case DOCK_STATE_MHL:
				printk("[Cable][%s] MHL removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
#endif
			case DOCK_STATE_USB_HEADSET:
				printk("[Cable][%s] USB headset removed\n", __func__);
				pre_acce_type = 0;
				break;
			case DOCK_STATE_DMB:
				printk("[Cable][%s] DMB removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
			case DOCK_STATE_DESK:
				printk("[Cable][%s] cradle removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
			case DOCK_STATE_USB_HOST:
				printk("[Cable][%s] USB Host removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
			default:
				printk("[Cable][%s] Unknown cable removed\n", __func__);
				switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
				pre_acce_type = 0;
				break;
		}

		
		mtk_idpin_cur_stat = IDPIN_OUT;
		
		printk("%s(%d): type= %d\n", __func__, __LINE__, htc_cable_detect_get_type());
		mtk_set_iddig_in_detect();
	}

done:
	
	while((value = mfg_check_white_accessory(pre_acce_type)) == -EAGAIN) {
		printk("[CABLE] android_dev is not ready. Try again. value = %d\n", value);
		msleep(1000);
	}
	mtk_xhci_mtk_log("current mode is %s, ret(%d), switch(%d)\n",
			 (mtk_idpin_cur_stat == IDPIN_IN_HOST) ? "host" :
			 (mtk_idpin_cur_stat == IDPIN_IN_DEVICE) ? "id_device" : "device",
			 ret, mtk_otg_state.state);
}

static irqreturn_t xhci_eint_iddig_isr(int irqnum, void *data)
{
	int ret;
	
	
	ret = schedule_delayed_work_on(0, &mtk_xhci_delaywork, msecs_to_jiffies(mtk_iddig_debounce));
	mtk_xhci_mtk_log("schedule to delayed work, ret(%d), gpio_mode(%d), gpio_pull_enable(%d), gpio_pull_select(%d)\n",
		ret, mt_get_gpio_mode(GPIO_OTG_IDDIG_EINT_PIN), mt_get_gpio_pull_enable(GPIO_OTG_IDDIG_EINT_PIN), mt_get_gpio_pull_select(GPIO_OTG_IDDIG_EINT_PIN));

	disable_irq_nosync(irqnum);
	return IRQ_HANDLED;
}

int mtk_xhci_eint_iddig_init(void)
{
	int retval;
	struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek, OTG_IDDIG-eint");
	if(node){
		of_property_read_u32_array(node , "interrupts", ints, ARRAY_SIZE(ints));
		mtk_xhci_mtk_log("iddig gpio num, %d\n", ints[0]);
	}
	else{
		mtk_xhci_mtk_log("cannot get the node\n");
		return -ENODEV;
	}

	INIT_DELAYED_WORK(&mtk_xhci_delaywork, mtk_xhci_mode_switch);
	mtk_idpin_irqnum = mt_gpio_to_irq(ints[0]);

	
	mt_gpio_set_debounce(ints[0], 50);

	retval =
	    request_irq(mtk_idpin_irqnum, xhci_eint_iddig_isr, ints[1], "iddig_eint",
			NULL);
	if (retval != 0) {
		mtk_xhci_mtk_log("request_irq fail, ret %d, irqnum %d!!!\n", retval, mtk_idpin_irqnum);
		return retval;
	}
	mtk_xhci_mtk_log("external iddig register done, irqnum = %d, gpio_mode(%d), gpio_pull_enable(%d), gpio_pull_select(%d)\n", 
		mtk_idpin_irqnum, mt_get_gpio_mode(GPIO_OTG_IDDIG_EINT_PIN), mt_get_gpio_pull_enable(GPIO_OTG_IDDIG_EINT_PIN), mt_get_gpio_pull_select(GPIO_OTG_IDDIG_EINT_PIN));

	
	

	mt_set_gpio_mode(GPIO_OTG_DRVVBUS_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_pull_select(GPIO_OTG_DRVVBUS_PIN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_OTG_DRVVBUS_PIN, GPIO_PULL_ENABLE);
#ifdef CONFIG_HTC_MHL_DETECTION
	mhl_detect_register_notifier(&mhl_status_notifier);
#endif
	return retval;
}

void mtk_xhci_eint_iddig_deinit(void)
{
	
	disable_irq_nosync(mtk_idpin_irqnum);

	free_irq(mtk_idpin_irqnum, NULL);

	cancel_delayed_work(&mtk_xhci_delaywork);

	mtk_idpin_cur_stat = IDPIN_OUT;

	mtk_xhci_mtk_log("external iddig unregister done.\n");
}

void mtk_set_host_mode_in_host(void)
{
	mtk_idpin_cur_stat = IDPIN_IN_HOST;
}

void mtk_set_host_mode_out(void)
{
	mtk_idpin_cur_stat = IDPIN_OUT;
}

bool mtk_is_host_mode(void)
{
	return (mtk_idpin_cur_stat == IDPIN_IN_HOST) ? true : false;
}

#endif

void mtk_xhci_wakelock_init(void)
{
	wake_lock_init(&mtk_xhci_wakelock, WAKE_LOCK_SUSPEND, "xhci.wakelock");
}

void mtk_xhci_wakelock_lock(void)
{
	if (!wake_lock_active(&mtk_xhci_wakelock))
		wake_lock(&mtk_xhci_wakelock);
	mtk_xhci_mtk_log("done\n");
}

void mtk_xhci_wakelock_unlock(void)
{
	if (wake_lock_active(&mtk_xhci_wakelock))
		wake_unlock(&mtk_xhci_wakelock);
	mtk_xhci_mtk_log("done\n");
}

void mtk_xhci_set(struct usb_hcd *hcd, struct xhci_hcd *xhci)
{
	struct platform_device *pdev = to_platform_device(hcd->self.controller);
	struct resource *sif_res, *sif2_res;

	xhci->base_regs = (unsigned long)hcd->regs;

	sif_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, XHCI_SIF_REGS_ADDR_RES_NAME);
	if (!sif_res){
		printk("%s(%d): cannot get sif resources\n", __func__, __LINE__);
		return -ENODEV;
	}

	xhci->sif_regs = (unsigned long)ioremap(sif_res->start, resource_size(sif_res));

	printk("%s(%d): sif_base, logic 0x%p, phys 0x%p\n", __func__, __LINE__,
		(void *)(unsigned long)sif_res->start, (void *)xhci->sif_regs);

	sif2_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, XHCI_SIF2_REGS_ADDR_RES_NAME);
	if (!sif2_res){
		printk("%s(%d): cannot get sif2 resources\n", __func__, __LINE__);
		return -ENODEV;
	}

	xhci->sif2_regs = (unsigned long)ioremap(sif2_res->start, resource_size(sif2_res));

	printk("%s(%d): sif2_base, logic 0x%p, phys 0x%p\n", __func__, __LINE__,
		(void *)(unsigned long)sif2_res->start, (void *)xhci->sif2_regs);

	mtk_xhci_mtk_log("mtk_xhci = 0x%p\n", xhci);
	mtk_xhci = xhci;
}

void mtk_xhci_reset(struct xhci_hcd *xhci){
	iounmap(xhci->sif_regs);
	mtk_xhci_mtk_log("iounmap, sif_reg, 0x%p\n", (void *)xhci->sif_regs);
	iounmap(xhci->sif2_regs);
	mtk_xhci_mtk_log("iounmap, sif2_reg, 0x%p\n", (void *)xhci->sif2_regs);
	mtk_xhci = NULL;
}

void mtk_xhci_ck_timer_init(struct xhci_hcd *xhci)
{
	void __iomem *addr;
	u32 temp = 0;
	int num_u3_port;
	unsigned int hw_code = mt_get_chip_hw_code();
	CHIP_SW_VER sw_code = mt_get_chip_sw_ver();

	mtk_xhci_mtk_log("hw code(0x%x), sw_code(0x%x)\n", hw_code, sw_code);

	if (0x6595 == hw_code) {
		
		addr = (void __iomem *)_SSUSB_SYS_CK_CTRL(xhci->sif_regs);
		temp = readl(addr);
		temp |= SSUSB_SYS_CK_DIV2_EN;
		writel(temp, addr);
		mtk_xhci_mtk_log("mu3d sys_clk, addr 0x%p, value 0x%x\n",
				(void *)_SSUSB_SYS_CK_CTRL(xhci->sif_regs), readl((__u32 __iomem *)_SSUSB_SYS_CK_CTRL(xhci->sif_regs)));
		
		num_u3_port = SSUSB_U3_PORT_NUM(readl((void __iomem *)_SSUSB_IP_CAP(xhci->sif_regs)));
		if (num_u3_port) {
			#if 0
			
			addr = (void __iomem *) (_SSUSB_U3_MAC_BASE(xhci->base_regs) + U3_UX_EXIT_LFPS_TIMING_PAR);
			temp = readl(addr);
			temp &= ~(0xff << U3_RX_UX_EXIT_LFPS_REF_OFFSET);
			temp |= (U3_RX_UX_EXIT_LFPS_REF << U3_RX_UX_EXIT_LFPS_REF_OFFSET);
			writel(temp, addr);
			addr = (void __iomem *)(_SSUSB_U3_MAC_BASE(xhci->base_regs) + U3_REF_CK_PAR);
			temp = readl(addr);
			temp &= ~(0xff);
			temp |= U3_REF_CK_VAL;
			writel(temp, addr);
			#endif

			
			addr = (void __iomem *)(_SSUSB_U3_SYS_BASE(xhci->base_regs) + U3_TIMING_PULSE_CTRL);
			temp = readl(addr);
			temp &= ~(0xff);
			temp |= MTK_CNT_1US_VALUE;
			writel(temp, addr);
		}
		
		
		addr = (void __iomem *)(_SSUSB_U2_SYS_BASE(xhci->base_regs) + USB20_TIMING_PARAMETER);
		temp &= ~(0xff);
		temp |= MTK_TIME_VALUE_1US;
		writel(temp, addr);

		mtk_xhci_mtk_log("mu3d u2 mac sys_clk, addr 0x%p, value 0x%x\n",
							(void *)(_SSUSB_U2_SYS_BASE(xhci->base_regs) + USB20_TIMING_PARAMETER),
							readl((void __iomem *)(_SSUSB_U2_SYS_BASE(xhci->base_regs) + (unsigned long)USB20_TIMING_PARAMETER)));

		#if 0
		if (num_u3_port) {
			
			addr = (void __iomem *)(_SSUSB_U3_SYS_BASE(xhci->base_regs) + LINK_PM_TIMER);
			temp = readl(addr);
			temp &= ~(0xf);
			temp |= MTK_PM_LC_TIMEOUT_VALUE;
			writel(temp, addr);
		}
		#endif
	}
}

static void setLatchSel(struct xhci_hcd *xhci)
{
	void __iomem *latch_sel_addr;
	u32 latch_sel_value;
	int num_u3_port;

	num_u3_port = SSUSB_U3_PORT_NUM(readl((void __iomem *)_SSUSB_IP_CAP(xhci->sif_regs)));
	if (num_u3_port <= 0)
		return;

	latch_sel_addr = (void __iomem *) _U3_PIPE_LATCH_SEL_ADD(xhci->base_regs);
	latch_sel_value = ((U3_PIPE_LATCH_TX) << 2) | (U3_PIPE_LATCH_RX);
	writel(latch_sel_value, latch_sel_addr);
}

#ifndef CONFIG_USB_MTK_DUALMODE
static int mtk_xhci_phy_init(int argc, char **argv)
{
	
	if (!u3phy_ops)
		u3phy_init();

	
	if (u3phy_ops->u2_slew_rate_calibration)
		u3phy_ops->u2_slew_rate_calibration(u3phy);
	else
		printk(KERN_ERR "WARN: PHY doesn't implement u2 slew rate calibration function\n");

	
	if (u3phy_ops->init(u3phy) != PHY_TRUE)
		return RET_FAIL;

	printk(KERN_ERR "phy registers and operations initial done\n");
	return RET_SUCCESS;
}
#endif

int mtk_xhci_ip_init(struct usb_hcd *hcd, struct xhci_hcd *xhci)
{
	mtk_xhci_set(hcd, xhci);
	
#ifdef CONFIG_MTK_FPGA
	u3_base = xhci->base_regs;
	u3_sif_base = xhci->sif_regs;
	u3_sif2_base = xhci->sif2_regs;
	i2c1_base = ioremap(0x11008000, 0x1000);
	if (!(i2c1_base)) {
		pr_err("Can't remap I2C1 BASE\n");
	}
	printk("%s(%d): i2c1_base, logic x%x, phys 0x%p\n", __func__, __LINE__, 0x11008000, (void *)i2c1_base);
#endif

#ifdef CONFIG_MTK_LDVT
	mt_set_gpio_mode(121, 4);
#endif

	
#ifndef CONFIG_USB_MTK_DUALMODE
	mtk_xhci_phy_init(0, NULL);
	enableAllClockPower(xhci, 1);	
#else
	enableAllClockPower(xhci, 1);	
#endif

	setLatchSel(xhci);
	mtk_xhci_ck_timer_init(xhci);
	mtk_xhci_scheduler_init();

	return 0;
}

#if 0
int mtk_xhci_get_port_num(void)
{
	return SSUSB_U3_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP))
	    + SSUSB_U2_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP));
}
#endif

