#include <linux/init.h>		
#include <linux/module.h>	
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>

#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_sleep.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>

#include <cust_charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/charging.h>
#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include "cust_battery_meter.h"
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include "mach/mtk_rtc.h"
#include <linux/htc_devices_dtb.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/mt_gpio.h>

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
#include <mach/diso.h>
#endif

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
#include "cust_pe.h"
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

int Enable_BATDRV_LOG = BAT_LOG_CRTI;
char proc_bat_data[32];

PMU_ChargerStruct BMT_status;
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
DISO_ChargerStruct DISO_data;
static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif

int g_battery_thermal_throttling_flag = 1;	
int battery_cmd_thermal_test_mode = 0;
int battery_cmd_thermal_test_mode_value = 0;
int g_battery_tt_check_flag = 0;	


struct wake_lock battery_suspend_lock;
CHARGING_CONTROL battery_charging_control;
static unsigned int g_BatteryNotifyCode = 0x0000;
unsigned int g_BN_TestMode = 0x0000;
kal_bool g_bat_init_flag = 0;
unsigned int g_call_state = CALL_IDLE;
kal_bool g_charging_full_reset_bat_meter = KAL_FALSE;
int g_platform_boot_mode = 0;
int g_car_tune_value = 0;
struct timespec g_bat_time_before_sleep;
int g_smartbook_update = 0;
kal_bool g_by_pass_batt_exist_check = KAL_TRUE;
kal_bool g_force_reverse_boost_wa = KAL_FALSE;
kal_bool g_batt_curr_full = KAL_FALSE;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
kal_bool g_vcdt_irq_delay_flag = 0;
#endif

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
kal_uint32 g_batt_temp_status = TEMP_POS_NORMAL;
#endif

kal_bool battery_suspended = KAL_FALSE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
extern U32 suspend_time;
#endif

#if defined(CUST_SYSTEM_OFF_VOLTAGE)
#define SYSTEM_OFF_VOLTAGE CUST_SYSTEM_OFF_VOLTAGE
#endif

#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"
#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal _IOW('k', 3, int)
#define ADC_CHANNEL_READ _IOW('k', 4, int)
#define BAT_STATUS_READ _IOW('k', 5, int)
#define Set_Charger_Current _IOW('k', 6, int)
#define Get_META_BAT_VOL _IOW('k', 10, int)
#define Get_META_BAT_SOC _IOW('k', 11, int)
#define Set_META_BAT_Chg _IOW('k', 12, int)

static struct class *adc_cali_class;
static int adc_cali_major;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;

int adc_cali_slop[14] =
    { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
int adc_cali_offset[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int adc_cali_cal[1] = { 0 };
int battery_in_data[1] = { 0 };
int battery_out_data[1] = { 0 };
int charging_level_data[1] = { 0 };
int meta_charging_enable = -1;

kal_bool g_ADC_Cali = KAL_FALSE;
kal_bool g_ftm_battery_flag = KAL_FALSE;
int g_ftm_charger_ctrl_stat = 0;
#if !defined(CONFIG_POWER_EXT)
static int g_wireless_state;
#endif

#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static kal_bool bat_thread_timeout = KAL_FALSE;
static kal_bool chr_wake_up_bat = KAL_FALSE;	
static kal_bool bat_meter_timeout = KAL_FALSE;
static DEFINE_MUTEX(bat_mutex);
static DEFINE_MUTEX(charger_type_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread = NULL;
static kal_bool charger_hv_detect_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);
static struct hrtimer battery_kthread_timer;
static kal_bool g_battery_soc_ready = KAL_FALSE;
extern BOOL bat_spm_timeout;
extern U32 _g_bat_sleep_total_time;

int g_status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING;
int g_capacity_smb = 50;
int g_present_smb = 0;
static int cmd_discharging = -1;
static int adjust_power = -1;
static int suspend_discharging = -1;
static int charger_ctrl_stat = -1;
static int g_charger_ctrl_internal = -1;
#ifdef CONFIG_HAS_EARLYSUSPEND
static kal_bool g_display_on = KAL_TRUE;
#endif
int g_chg_limit_reason;
int g_chg_in_mhl;


struct wireless_data {
	struct power_supply psy;
	int WIRELESS_ONLINE;
};

struct ac_data {
	struct power_supply psy;
	int AC_ONLINE;
};

struct usb_data {
	struct power_supply psy;
	int USB_ONLINE;
};

struct battery_data {
	struct power_supply psy;
	int BAT_STATUS;
	int BAT_HEALTH;
	int BAT_PRESENT;
	int BAT_TECHNOLOGY;
	int BAT_CAPACITY;
	
	int BAT_batt_vol;
	int BAT_batt_temp;
	
	int BAT_TemperatureR;
	int BAT_TempBattVoltage;
	int BAT_InstatVolt;
	int BAT_BatteryAverageCurrent;
	int BAT_BatterySenseVoltage;
	int BAT_ISenseVoltage;
	int BAT_ChargerVoltage;
	
	int status_smb;
	int capacity_smb;
	int present_smb;
	int adjust_power;
};

static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	
	POWER_SUPPLY_PROP_batt_vol,
	POWER_SUPPLY_PROP_batt_temp,
	
	POWER_SUPPLY_PROP_TemperatureR,
	POWER_SUPPLY_PROP_TempBattVoltage,
	POWER_SUPPLY_PROP_InstatVolt,
	POWER_SUPPLY_PROP_BatteryAverageCurrent,
	POWER_SUPPLY_PROP_BatterySenseVoltage,
	POWER_SUPPLY_PROP_ISenseVoltage,
	POWER_SUPPLY_PROP_ChargerVoltage,
	
	POWER_SUPPLY_PROP_status_smb,
	POWER_SUPPLY_PROP_capacity_smb,
	POWER_SUPPLY_PROP_present_smb,
	
	POWER_SUPPLY_PROP_adjust_power,
};

#ifdef HTC_ENABLE_AICL
const kal_uint32 CHR_CUR[]=
{
    CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1100_00_MA, CHARGE_CURRENT_1200_00_MA,
    CHARGE_CURRENT_1300_00_MA, CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1500_00_MA,
};
#define CHR_CUR_SIZE    ( sizeof(CHR_CUR)/sizeof(kal_uint32) )
#endif

extern bool mt_usb_is_device(void);
#if defined(CONFIG_USB_MTK_HDRC) || defined(CONFIG_USB_MU3D_DRV)
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#else
#define mt_usb_connect() do { } while (0)
#define mt_usb_disconnect() do { } while (0)
#endif

void check_battery_exist(void);
void charging_suspend_enable(void)
{
    U32 charging_enable = true;

    suspend_discharging = 0;
    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

void charging_suspend_disable(void)
{
    U32 charging_enable = false;

    suspend_discharging = 1;
    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

int read_tbat_value(void)
{
	return BMT_status.temperature;
}

int get_charger_detect_status(void)
{
	kal_bool chr_status;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
	return chr_status;
}

#if defined(CONFIG_MTK_POWER_EXT_DETECT)
kal_bool bat_is_ext_power(void)
{
	kal_bool pwr_src = 0;

	battery_charging_control(CHARGING_CMD_GET_POWER_SOURCE, &pwr_src);
	battery_xlog_printk(BAT_LOG_FULL, "[BAT_IS_EXT_POWER] is_ext_power = %d\n", pwr_src);
	return pwr_src;
}
#endif
kal_bool upmu_is_chr_det(void)
{
#if !defined(CONFIG_POWER_EXT)
	kal_uint32 tmp32;
#endif	

    if(battery_charging_control == NULL)
        battery_charging_control = chr_control_interface;

#if defined(CONFIG_POWER_EXT)
	
	return get_charger_detect_status();
#else
        if (suspend_discharging==1)
        return KAL_FALSE;

	tmp32 = get_charger_detect_status();

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return tmp32;
#endif

	if (tmp32 == 0) {
		return KAL_FALSE;
	} else {
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (mt_usb_is_device()) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[upmu_is_chr_det] Charger exist and USB is not host\n");

			return KAL_TRUE;
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[upmu_is_chr_det] Charger exist but USB is host\n");

			return KAL_FALSE;
		}
		#else
		return KAL_TRUE;
		#endif
	}
#endif
}
EXPORT_SYMBOL(upmu_is_chr_det);


void wake_up_bat(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] wake_up_bat. \r\n");

	chr_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
    _g_bat_sleep_total_time = 0;
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);

static void _get_value_by_pcbid_probe()
{
	int result = 68;
	switch(of_machine_hwid()){
		case 0:
		case 1:
			result = 88;
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 128:
		default:
			break;
		}
	g_car_tune_value = result;
}

int g_get_car_tune_value_by_pcbid()
{
	if(!g_car_tune_value)
		_get_value_by_pcbid_probe();
	return g_car_tune_value;
}
EXPORT_SYMBOL(g_get_car_tune_value_by_pcbid);

static ssize_t bat_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	if (copy_from_user(&proc_bat_data, buff, len)) {
		battery_xlog_printk(BAT_LOG_FULL, "bat_log_write error.\n");
		return -EFAULT;
	}

	if (proc_bat_data[0] == '1') {
		battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system\n");
		Enable_BATDRV_LOG = 1;
	} else if (proc_bat_data[0] == '2') {
		battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system:2\n");
		Enable_BATDRV_LOG = 2;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "Disable battery driver log system\n");
		Enable_BATDRV_LOG = 0;
	}

	return len;
}

static const struct file_operations bat_proc_fops = {
	.write = bat_log_write,
};

int init_proc_log(void)
{
	int ret = 0;

#if 1
	proc_create("batdrv_log", 0644, NULL, &bat_proc_fops);
	battery_xlog_printk(BAT_LOG_CRTI, "proc_create bat_proc_fops\n");
#else
	proc_entry = create_proc_entry("batdrv_log", 0644, NULL);

	if (proc_entry == NULL) {
		ret = -ENOMEM;
		battery_xlog_printk(BAT_LOG_FULL, "init_proc_log: Couldn't create proc entry\n");
	} else {
		proc_entry->write_proc = bat_log_write;
		battery_xlog_printk(BAT_LOG_CRTI, "init_proc_log loaded.\n");
	}
#endif

	return ret;
}


static int wireless_get_property(struct power_supply *psy,
				 enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct wireless_data *data = container_of(psy, struct wireless_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->WIRELESS_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ac_get_property(struct power_supply *psy,
			   enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct ac_data *data = container_of(psy, struct ac_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->AC_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_get_property(struct power_supply *psy,
			    enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct usb_data *data = container_of(psy, struct usb_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_POWER_EXT)
		
		data->USB_ONLINE = 1;
		val->intval = data->USB_ONLINE;
#else
#if defined(CONFIG_MTK_POWER_EXT_DETECT)
		if (KAL_TRUE == bat_is_ext_power())
			data->USB_ONLINE = 1;
#endif
		val->intval = data->USB_ONLINE;
#endif
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int battery_get_property(struct power_supply *psy,
				enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct battery_data *data = container_of(psy, struct battery_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = data->BAT_STATUS;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = data->BAT_HEALTH;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = data->BAT_PRESENT;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = data->BAT_TECHNOLOGY;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = data->BAT_CAPACITY;
		break;
	case POWER_SUPPLY_PROP_batt_vol:
		val->intval = data->BAT_batt_vol;
		break;
	case POWER_SUPPLY_PROP_batt_temp:
		val->intval = data->BAT_batt_temp;
		break;
	case POWER_SUPPLY_PROP_TemperatureR:
		val->intval = data->BAT_TemperatureR;
		break;
	case POWER_SUPPLY_PROP_TempBattVoltage:
		val->intval = data->BAT_TempBattVoltage;
		break;
	case POWER_SUPPLY_PROP_InstatVolt:
		val->intval = data->BAT_InstatVolt;
		break;
	case POWER_SUPPLY_PROP_BatteryAverageCurrent:
		val->intval = data->BAT_BatteryAverageCurrent;
		break;
	case POWER_SUPPLY_PROP_BatterySenseVoltage:
		val->intval = data->BAT_BatterySenseVoltage;
		break;
	case POWER_SUPPLY_PROP_ISenseVoltage:
		val->intval = data->BAT_ISenseVoltage;
		break;
	case POWER_SUPPLY_PROP_ChargerVoltage:
		val->intval = data->BAT_ChargerVoltage;
		break;
		
	case POWER_SUPPLY_PROP_status_smb:
		val->intval = data->status_smb;
		break;
	case POWER_SUPPLY_PROP_capacity_smb:
		val->intval = data->capacity_smb;
		break;
	case POWER_SUPPLY_PROP_present_smb:
		val->intval = data->present_smb;
		break;
	case POWER_SUPPLY_PROP_adjust_power :
		val->intval = data->adjust_power;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct wireless_data wireless_main = {
	.psy = {
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.properties = wireless_props,
		.num_properties = ARRAY_SIZE(wireless_props),
		.get_property = wireless_get_property,
		},
	.WIRELESS_ONLINE = 0,
};

static struct ac_data ac_main = {
	.psy = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = ac_props,
		.num_properties = ARRAY_SIZE(ac_props),
		.get_property = ac_get_property,
		},
	.AC_ONLINE = 0,
};

static struct usb_data usb_main = {
	.psy = {
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = usb_props,
		.num_properties = ARRAY_SIZE(usb_props),
		.get_property = usb_get_property,
		},
	.USB_ONLINE = 0,
};

static struct battery_data battery_main = {
	.psy = {
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = battery_props,
		.num_properties = ARRAY_SIZE(battery_props),
		.get_property = battery_get_property,
		},
#if defined(CONFIG_POWER_EXT)
	.BAT_STATUS = POWER_SUPPLY_STATUS_FULL,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
	.BAT_CAPACITY = 100,
	.BAT_batt_vol = 4200,
	.BAT_batt_temp = 22,
	
	.status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.capacity_smb = 50,
	.present_smb = 0,
	
	.adjust_power = -1,
#else
	.BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
	.BAT_CAPACITY = 50,
	.BAT_batt_vol = 0,
	.BAT_batt_temp = 0,
	
	.status_smb = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.capacity_smb = 50,
	.present_smb = 0,
	
	.adjust_power = -1,
#endif
};


#if !defined(CONFIG_POWER_EXT)
static ssize_t show_ADC_Charger_Voltage(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] show_ADC_Charger_Voltage : %d\n",
			    BMT_status.charger_vol);
	return sprintf(buf, "%d\n", BMT_status.charger_vol);
}

static ssize_t store_ADC_Charger_Voltage(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Charger_Voltage, 0664, show_ADC_Charger_Voltage, store_ADC_Charger_Voltage);

static ssize_t show_ADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 0));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_0_Slope, 0664, show_ADC_Channel_0_Slope, store_ADC_Channel_0_Slope);

static ssize_t show_ADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 1));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_1_Slope, 0664, show_ADC_Channel_1_Slope, store_ADC_Channel_1_Slope);

static ssize_t show_ADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 2));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_2_Slope, 0664, show_ADC_Channel_2_Slope, store_ADC_Channel_2_Slope);

static ssize_t show_ADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 3));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_3_Slope, 0664, show_ADC_Channel_3_Slope, store_ADC_Channel_3_Slope);

static ssize_t show_ADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 4));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_4_Slope, 0664, show_ADC_Channel_4_Slope, store_ADC_Channel_4_Slope);

static ssize_t show_ADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 5));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_5_Slope, 0664, show_ADC_Channel_5_Slope, store_ADC_Channel_5_Slope);

static ssize_t show_ADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 6));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_6_Slope, 0664, show_ADC_Channel_6_Slope, store_ADC_Channel_6_Slope);

static ssize_t show_ADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 7));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_7_Slope, 0664, show_ADC_Channel_7_Slope, store_ADC_Channel_7_Slope);

static ssize_t show_ADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 8));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_8_Slope, 0664, show_ADC_Channel_8_Slope, store_ADC_Channel_8_Slope);

static ssize_t show_ADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 9));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_9_Slope, 0664, show_ADC_Channel_9_Slope, store_ADC_Channel_9_Slope);

static ssize_t show_ADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 10));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_10_Slope, 0664, show_ADC_Channel_10_Slope,
		   store_ADC_Channel_10_Slope);

static ssize_t show_ADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 11));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_11_Slope, 0664, show_ADC_Channel_11_Slope,
		   store_ADC_Channel_11_Slope);

static ssize_t show_ADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 12));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_12_Slope, 0664, show_ADC_Channel_12_Slope,
		   store_ADC_Channel_12_Slope);

static ssize_t show_ADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_slop + 13));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_13_Slope, 0664, show_ADC_Channel_13_Slope,
		   store_ADC_Channel_13_Slope);


static ssize_t show_ADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 0));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_0_Offset, 0664, show_ADC_Channel_0_Offset,
		   store_ADC_Channel_0_Offset);

static ssize_t show_ADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 1));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_1_Offset, 0664, show_ADC_Channel_1_Offset,
		   store_ADC_Channel_1_Offset);

static ssize_t show_ADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 2));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_2_Offset, 0664, show_ADC_Channel_2_Offset,
		   store_ADC_Channel_2_Offset);

static ssize_t show_ADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 3));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_3_Offset, 0664, show_ADC_Channel_3_Offset,
		   store_ADC_Channel_3_Offset);

static ssize_t show_ADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 4));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_4_Offset, 0664, show_ADC_Channel_4_Offset,
		   store_ADC_Channel_4_Offset);

static ssize_t show_ADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 5));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_5_Offset, 0664, show_ADC_Channel_5_Offset,
		   store_ADC_Channel_5_Offset);

static ssize_t show_ADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 6));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_6_Offset, 0664, show_ADC_Channel_6_Offset,
		   store_ADC_Channel_6_Offset);

static ssize_t show_ADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 7));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_7_Offset, 0664, show_ADC_Channel_7_Offset,
		   store_ADC_Channel_7_Offset);

static ssize_t show_ADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 8));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_8_Offset, 0664, show_ADC_Channel_8_Offset,
		   store_ADC_Channel_8_Offset);

static ssize_t show_ADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 9));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_9_Offset, 0664, show_ADC_Channel_9_Offset,
		   store_ADC_Channel_9_Offset);

static ssize_t show_ADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 10));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_10_Offset, 0664, show_ADC_Channel_10_Offset,
		   store_ADC_Channel_10_Offset);

static ssize_t show_ADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 11));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_11_Offset, 0664, show_ADC_Channel_11_Offset,
		   store_ADC_Channel_11_Offset);

static ssize_t show_ADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 12));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_12_Offset, 0664, show_ADC_Channel_12_Offset,
		   store_ADC_Channel_12_Offset);

static ssize_t show_ADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	int ret_value = 1;
	ret_value = (*(adc_cali_offset + 13));
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_13_Offset, 0664, show_ADC_Channel_13_Offset,
		   store_ADC_Channel_13_Offset);

static ssize_t show_ADC_Channel_Is_Calibration(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	int ret_value = 2;
	ret_value = g_ADC_Cali;
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_Is_Calibration : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_ADC_Channel_Is_Calibration(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(ADC_Channel_Is_Calibration, 0664, show_ADC_Channel_Is_Calibration,
		   store_ADC_Channel_Is_Calibration);

static ssize_t show_Power_On_Voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;
	ret_value = 3400;
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Power_On_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Power_On_Voltage(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Power_On_Voltage, 0664, show_Power_On_Voltage, store_Power_On_Voltage);

static ssize_t show_Power_Off_Voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;
	ret_value = 3400;
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Power_Off_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Power_Off_Voltage(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Power_Off_Voltage, 0664, show_Power_Off_Voltage, store_Power_Off_Voltage);

static ssize_t show_Charger_TopOff_Value(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	int ret_value = 1;
	ret_value = 4110;
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Charger_TopOff_Value : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_Charger_TopOff_Value(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(Charger_TopOff_Value, 0664, show_Charger_TopOff_Value,
		   store_Charger_TopOff_Value);

static ssize_t show_FG_Battery_CurrentConsumption(struct device *dev, struct device_attribute *attr,
						  char *buf)
{
	int ret_value = 8888;
	ret_value = battery_meter_get_battery_current();
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] FG_Battery_CurrentConsumption : %d/10 mA\n",
			    ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_FG_Battery_CurrentConsumption(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(FG_Battery_CurrentConsumption, 0664, show_FG_Battery_CurrentConsumption,
		   store_FG_Battery_CurrentConsumption);

static ssize_t show_FG_SW_CoulombCounter(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	kal_int32 ret_value = 7777;
	ret_value = battery_meter_get_car();
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] FG_SW_CoulombCounter : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_FG_SW_CoulombCounter(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(FG_SW_CoulombCounter, 0664, show_FG_SW_CoulombCounter,
		   store_FG_SW_CoulombCounter);


static ssize_t show_Charging_CallState(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_xlog_printk(BAT_LOG_CRTI, "call state = %d\n", g_call_state);
	return sprintf(buf, "%u\n", g_call_state);
}

static ssize_t store_Charging_CallState(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	sscanf(buf, "%u", &g_call_state);
	battery_xlog_printk(BAT_LOG_CRTI, "call state = %d\n", g_call_state);
	return size;
}

static DEVICE_ATTR(Charging_CallState, 0664, show_Charging_CallState, store_Charging_CallState);

static ssize_t show_Charger_Type(struct device *dev,struct device_attribute *attr,
					char *buf)
{
    UINT32 chr_ype = CHARGER_UNKNOWN;
    chr_ype = BMT_status.charger_exist ? BMT_status.charger_type : CHARGER_UNKNOWN;

    battery_xlog_printk(BAT_LOG_CRTI, "CHARGER_TYPE = %d\n",chr_ype);
    return sprintf(buf, "%u\n", chr_ype);
}
static ssize_t store_Charger_Type(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(Charger_Type, 0664, show_Charger_Type, store_Charger_Type);

#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
static ssize_t show_Pump_Express(struct device *dev,struct device_attribute *attr,
					char *buf)
{
    #if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
    int icount = 20;  

    if (KAL_TRUE == ta_check_chr_type &&
      STANDARD_CHARGER == BMT_status.charger_type &&
      BMT_status.SOC >= TA_START_BATTERY_SOC &&
      BMT_status.SOC < TA_STOP_BATTERY_SOC)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[%s]Wait for PE detection\n", __func__);
        do
        {
            icount--;
            msleep(200);
        }while(icount && ta_check_chr_type);
    }
    #endif

    battery_xlog_printk(BAT_LOG_CRTI, "Pump express = %d\n",is_ta_connect);    
    return sprintf(buf, "%u\n", is_ta_connect);
}
static ssize_t store_Pump_Express(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%u", &is_ta_connect);
    battery_xlog_printk(BAT_LOG_CRTI, "Pump express= %d\n",is_ta_connect);    
    return size;
}
static DEVICE_ATTR(Pump_Express, 0664, show_Pump_Express, store_Pump_Express);
#endif

static void mt_battery_update_EM(struct battery_data *bat_data)
{
	bat_data->BAT_CAPACITY = BMT_status.UI_SOC;
	bat_data->BAT_TemperatureR = BMT_status.temperatureR;	
	bat_data->BAT_TempBattVoltage = BMT_status.temperatureV;	
	bat_data->BAT_InstatVolt = BMT_status.bat_vol;	
	bat_data->BAT_BatteryAverageCurrent = BMT_status.ICharging;
	bat_data->BAT_BatterySenseVoltage = BMT_status.bat_vol;
	bat_data->BAT_ISenseVoltage = BMT_status.Vsense;	
	bat_data->BAT_ChargerVoltage = BMT_status.charger_vol;
	
	bat_data->status_smb = g_status_smb;
	bat_data->capacity_smb = g_capacity_smb;
	bat_data->present_smb = g_present_smb;
	battery_xlog_printk(BAT_LOG_FULL, "status_smb = %d, capacity_smb = %d, present_smb = %d\n",
			    bat_data->status_smb, bat_data->capacity_smb, bat_data->present_smb);

	if ((bat_data->BAT_batt_temp > 650) && (BMT_status.keep_charger_on || BMT_status.flag_pa_fake_batt_temp))
		bat_data->BAT_batt_temp = 650;

	if (BMT_status.test_power_monitor) {
		bat_data->BAT_CAPACITY = 77;
		bat_data->BAT_batt_temp = 330;
	}

	if((BMT_status.bat_vol > 3200) && (bat_data->BAT_CAPACITY <= 0))
		bat_data->BAT_CAPACITY = 1;

#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
	if (bat_data->BAT_CAPACITY <= 0)
		bat_data->BAT_CAPACITY = 1;

	battery_xlog_printk(BAT_LOG_CRTI,
			    "BAT_CAPACITY=1, due to define CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION\r\n");
#endif
}

extern void otg_oc_handler(void);
void htc_battery_check_vchr(void)
{
    static int vol_check = 0;
    if(BMT_status.charger_vol < 4200)
        vol_check++;
    else
        vol_check = 0;

    if(vol_check > 1) {
        vol_check = 0;
        otg_oc_handler();
    }
}

void htc_battery_update_gap(kal_int32 iGap)
{
    BMT_status.gap = iGap;
	battery_xlog_printk(BAT_LOG_CRTI, "[%s] gap = %d\n", __FUNCTION__, BMT_status.gap);
}

kal_int32 htc_battery_adjust_gap(kal_int32 gap, kal_int32 pre_soc, kal_int32 soc)
{
    kal_int32 adjust_base_tbl[] = {5, 10};
    kal_int32 adjust_map_tbl[] = {90};
    kal_int32 adjust_base;
	static kal_int32 last_gap = 0;

    if( soc < adjust_map_tbl[0] )
        adjust_base = adjust_base_tbl[0];
    else
        adjust_base = adjust_base_tbl[1];

    if( BMT_status.charger_exist == KAL_FALSE ){
		if( gap > last_gap && last_gap != 0) {
			gap--;
		}
        else if( gap > 0 ){
            if( pre_soc != -1 && pre_soc/adjust_base > soc/adjust_base )
                gap--;
        }
        else if( gap < 0 ){
            if( pre_soc != -1 && pre_soc/adjust_base > soc/adjust_base )
                gap++;
        }
    }else
        gap = 0;

	if (soc == BMT_status.UI_SOC) gap = 0;

	last_gap = gap;
    return gap;
}

void htc_battery_check_overload(void)
{
    static kal_int32 pre_state = CHR_PRE;

    if( pre_state ^ BMT_status.bat_charging_state )
        htc_battery_meter_overload(TRUE, &BMT_status.is_overload);

    if( BMT_status.bat_charging_state == CHR_CC )
        htc_battery_meter_overload(FALSE, &BMT_status.is_overload);

	battery_xlog_printk(BAT_LOG_FULL, "[%s] overload = %d\n", __FUNCTION__, BMT_status.is_overload);
    pre_state = BMT_status.bat_charging_state;
}

static kal_uint32 htc_battery_sync_speed(kal_int32 temp)
{
    kal_int32 iGap_temp_map_tbl[] = {0, 10};
    kal_uint32 uiSpeed_map_tbl[] = {0, 20, 60};
    int i;

    for( i=0; i< sizeof(iGap_temp_map_tbl)/sizeof(kal_int32); i++){
        if( temp < iGap_temp_map_tbl[i] )
            break;
    }

    if( temp >= iGap_temp_map_tbl[i] )
        i = sizeof(uiSpeed_map_tbl)/sizeof(kal_uint32) - 1;

    return uiSpeed_map_tbl[i];
}

static void htc_battery_ui_soc_sync(pSync_uisoc pUisoc_data)
{
    static kal_uint32 uiCount = 0;
    static bool start_raise_UI = false;
    static int start_raw = 0;
    kal_uint32 uiTtl_cnt = pUisoc_data->uiTotal_time / BAT_TASK_PERIOD;
	static struct timespec last_ts;
	struct timespec ts;
	getnstimeofday(&ts);

	battery_xlog_printk(BAT_LOG_FULL, "[htc_battery_ui_soc_sync] time now: %lu. time last: %lu. uiCount: %d\n",
						ts.tv_sec, last_ts.tv_sec, uiCount);

	if ((ts.tv_sec - last_ts.tv_sec) > pUisoc_data->uiTotal_time) {
		getnstimeofday(&last_ts);
		uiCount = uiTtl_cnt;
	}

    if (pUisoc_data->bIs_cnt_rst)
        uiCount = 0;

    if( pUisoc_data->bIs_charging ){

        if( uiCount >= uiTtl_cnt ){

            if( pUisoc_data->iUI_SOC < (pUisoc_data->iSOC + pUisoc_data->iGap) ){
                start_raise_UI = false;
                pUisoc_data->iUI_SOC++;
                uiCount = 0;
            }else if( (pUisoc_data->iUI_SOC < 99) && (pUisoc_data->iUI_SOC > pUisoc_data->iSOC) ){
		        if(start_raise_UI){
                    if(pUisoc_data->iSOC - start_raw >= 2){
			                battery_xlog_printk(BAT_LOG_CRTI,"Force to add UI level: iSOC:%d Raw:%d. \n", pUisoc_data->iSOC, start_raw);
			                pUisoc_data->iUI_SOC++;
                            uiCount = 0;
			                start_raw = pUisoc_data->iSOC;
                     }
		        }else{
			        start_raw = pUisoc_data->iSOC;
			        start_raise_UI = true;
                }
		    }
        }else
            uiCount++;

    }else if( !pUisoc_data->bIs_charging){
	start_raise_UI = false;
        if( uiCount >= uiTtl_cnt ){

            if( (pUisoc_data->iUI_SOC > (pUisoc_data->iSOC + pUisoc_data->iGap)) && (pUisoc_data->iGap >= 0) ){
				if (pUisoc_data->iUI_SOC > 1 || pUisoc_data->iSOC == 0)
					pUisoc_data->iUI_SOC--;

                uiCount = 0;
            }
        }else
            uiCount++;
    }
}

static kal_bool htc_battery_100percent_early(kal_bool bIs_reset, kal_bool bIs_charging)
{
    static kal_uint32 uiCount = 0;
    kal_uint32 uiEarly_complete_vol = 4330;
    kal_int32 iBatt_cur = htc_battery_meter_get_battery_current_imm(FALSE)/10;
    int ret = FALSE;

    if( bIs_reset )
        uiCount = 0;

    if( bIs_charging ){

        if( BMT_status.bat_vol >= uiEarly_complete_vol){
			battery_xlog_printk(BAT_LOG_CRTI, "[%s] i_bat=(%d), uiCount=(%d)\n",
								__FUNCTION__, iBatt_cur, uiCount);
            
			if( iBatt_cur > 0 && iBatt_cur < (kal_int32)(Q_MAX_SPEC/10) )  
                uiCount++;
            else
                uiCount = 0;
        }else
            uiCount =0;
        uiCount = min(uiCount, (kal_uint32)3);

        ret = uiCount>=3? TRUE:FALSE;
    }

    return ret;
}

static void htc_battery_100Percent_tracking_check(pSync_uisoc pUisoc_data)
{
    kal_int32 iSoc_bak = pUisoc_data->iSOC;

    pUisoc_data->iGap = 0;

    if( pUisoc_data->iUI_SOC < 100 ){

        pUisoc_data->uiTotal_time = 0;
        if( !BMT_status.is_warm )
            pUisoc_data->iSOC = 100;

        htc_battery_ui_soc_sync(pUisoc_data);

        pUisoc_data->iSOC = iSoc_bak;
    }
}

static kal_uint32 htc_battery_0percent_volt(kal_int32 temp)
{
    kal_int32 iGap_temp_map_tbl[] = {0, 10};
    kal_uint32 ui0percent_volt_map_tbl[] = {2950, 3200, 3200};
    int i;

    for( i=0; i< sizeof(iGap_temp_map_tbl)/sizeof(kal_int32); i++){
        if( temp < iGap_temp_map_tbl[i] )
            break;
    }

    if( temp >= iGap_temp_map_tbl[i] )
        i = sizeof(ui0percent_volt_map_tbl)/sizeof(kal_uint32) - 1;

    return ui0percent_volt_map_tbl[i];
}

static void htc_battery_0Percent_tracking_check(pSync_uisoc pUisoc_data)
{
    kal_uint32 uiVolt_slow_sync =
        htc_battery_0percent_volt(BMT_status.temperature_now);
    kal_int32 iSoc_bak = pUisoc_data->iSOC;

    pUisoc_data->iGap = 0;

    if( pUisoc_data->iUI_SOC > 0 ){

        if( BMT_status.bat_vol >= uiVolt_slow_sync ){

            if( BMT_status.temperature_now < 0 )
                pUisoc_data->iSOC = 0;

            pUisoc_data->uiTotal_time = htc_battery_sync_speed(BMT_status.temperature_now);
            htc_battery_ui_soc_sync(pUisoc_data);

            if( BMT_status.temperature_now < 0 )
                pUisoc_data->iSOC = iSoc_bak;

        }else{
            pUisoc_data->uiTotal_time = 0;
            if (BMT_status.bat_vol > 3200)
                pUisoc_data->iSOC = 1;
            else {
                pUisoc_data->iSOC = 0;
                iSoc_bak = 0;
            }

            htc_battery_ui_soc_sync(pUisoc_data);
            

            pUisoc_data->iSOC = iSoc_bak;
        }
    }
}

static void htc_battery_sync_ui_soc(struct battery_data *bat_data)
{
    static kal_bool pre_is_charging = KAL_FALSE;
    static kal_bool pre_is_charger_exist = KAL_FALSE;
    kal_bool bIs_charging = (BMT_status.charger_exist == KAL_TRUE? (!BMT_status.is_overload): FALSE) &&
            (BMT_status.bat_charging_state == CHR_CC || BMT_status.bat_charging_state == CHR_BATFULL) &&
            BMT_status.bat_exist;
    kal_bool bIs_chr_chg = pre_is_charging != bIs_charging;
    sSync_uisoc sUisoc_data;

    sUisoc_data.bIs_cnt_rst = bIs_chr_chg;
    sUisoc_data.bIs_charging = bIs_charging;
    sUisoc_data.iUI_SOC = BMT_status.UI_SOC;
    sUisoc_data.iSOC = BMT_status.SOC;
    sUisoc_data.iGap = BMT_status.gap;

    if (!BMT_status.charger_exist && pre_is_charger_exist) {
		htc_battery_update_gap( bat_get_ui_percentage() - BMT_status.SOC );
		if (BMT_status.UI_SOC == 100)
			battery_meter_reset(BMT_status.SOC);
    }

    if( bIs_charging ){                     
        if( BMT_status.bat_full )
            htc_battery_100Percent_tracking_check(&sUisoc_data);

        else if( htc_battery_100percent_early(bIs_chr_chg, bIs_charging) ){

            kal_int32 iSOC_bak = sUisoc_data.iSOC;

            if( !BMT_status.is_warm )
                sUisoc_data.iSOC = 100;
            sUisoc_data.iGap = 0;
            sUisoc_data.uiTotal_time = htc_battery_sync_speed(BMT_status.temperature_now);
            htc_battery_ui_soc_sync(&sUisoc_data);
            sUisoc_data.iSOC = iSOC_bak;
        }else{
            if( sUisoc_data.iUI_SOC < 100 ){    

                sUisoc_data.uiTotal_time = BAT_TASK_PERIOD;
                htc_battery_ui_soc_sync(&sUisoc_data);
                sUisoc_data.iUI_SOC = min(sUisoc_data.iUI_SOC, 99);
            }
        }

    }else if( BMT_status.bat_exist ){       
        if( BMT_status.bat_vol >= htc_battery_0percent_volt(BMT_status.temperature_now) ){

            if( sUisoc_data.iUI_SOC > 0 ){  

                sUisoc_data.uiTotal_time = htc_battery_sync_speed(BMT_status.temperature_now);
                htc_battery_ui_soc_sync(&sUisoc_data);

                if( BMT_status.temperature_now >= 0 )   
                    sUisoc_data.iUI_SOC = max(sUisoc_data.iUI_SOC, 1);
            }
        }
        else
            htc_battery_0Percent_tracking_check(&sUisoc_data);

        if( BMT_status.bat_full == KAL_TRUE && BMT_status.bat_vol > V_100PERCENT_TRACKING && !BMT_status.is_warm )
            if( sUisoc_data.iUI_SOC < 100 )
                sUisoc_data.iUI_SOC ++;

    }else{                                  
        sUisoc_data.iUI_SOC = 0;
    }
    bat_set_ui_percentage(sUisoc_data.iUI_SOC);

    battery_xlog_printk(BAT_LOG_FULL, "[%s] UI_SOC=(%d)\n", __FUNCTION__, BMT_status.UI_SOC);

    pre_is_charging = bIs_charging;
    pre_is_charger_exist = BMT_status.charger_exist;
}

static void htc_battery_update(struct battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;
#if 0
	static kal_uint32 unknown_chag_check = 0;
#endif
    bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;

	if (BMT_status.temperature >= 48)
	    bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (BMT_status.temperature <= 0)
		bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_COLD;
	else
	    bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;

    bat_data->BAT_batt_vol = BMT_status.bat_vol;
    bat_data->BAT_batt_temp = BMT_status.temperature * 10;
    bat_data->BAT_PRESENT = BMT_status.bat_exist;

    mt_battery_update_EM(bat_data);

	
	if ((BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR)) {
		if (BMT_status.bat_exist) {	
			if (fgauge_get_battery_id() == FG_ERROR_BATTERY)
				bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
			else if (BMT_status.UI_SOC == 100)
	            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
			else
				bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
		}
	} else {
		if (BMT_status.bat_charging_state == CHR_ERROR)
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

    if (BMT_status.is_overload ||
		((g_charger_ctrl_internal == DISABLE_PWRSRC_FINGERPRINT) && BMT_status.charger_exist)) {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
    }

    if (cmd_discharging == 1) {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CMD_DISCHARGING;
    }

	

    if (adjust_power != -1) {
        bat_data->adjust_power = adjust_power;
        battery_xlog_printk(BAT_LOG_CRTI, "adjust_power=(%d)\n", adjust_power);
    }
	if (BMT_status.bat_full && (BMT_status.SOC == 100))
		BMT_status.htc_extension |= HTC_EXT_CHG_FULL_EOC_STOP;
	else
		BMT_status.htc_extension &= ~HTC_EXT_CHG_FULL_EOC_STOP;
#if 0
	if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
		if (g_chg_in_mhl > CONNECT_TYPE_MHL_NONE) {
			unknown_chag_check = 0;
			BMT_status.htc_extension &= ~HTC_EXT_UNKNOWN_USB_CHARGER;
		} else if (unknown_chag_check < 3) {
			unknown_chag_check++;
		} else if (unknown_chag_check >= 3) {
			BMT_status.htc_extension |= HTC_EXT_UNKNOWN_USB_CHARGER;
		}
	} else {
		BMT_status.htc_extension &= ~HTC_EXT_UNKNOWN_USB_CHARGER;
		unknown_chag_check = 0;
	}
#endif
    power_supply_changed(bat_psy);
}

#if 0
static kal_bool mt_battery_100Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	kal_uint32 cust_sync_time = CUST_SOC_JEITA_SYNC_TIME;
	static kal_uint32 timer_counter = (CUST_SOC_JEITA_SYNC_TIME / BAT_TASK_PERIOD);
#else
	kal_uint32 cust_sync_time = ONEHUNDRED_PERCENT_TRACKING_TIME;
	static kal_uint32 timer_counter = (ONEHUNDRED_PERCENT_TRACKING_TIME / BAT_TASK_PERIOD);
#endif

	if (BMT_status.bat_full == KAL_TRUE)	
	{
		if (BMT_status.UI_SOC >= 100) {
			BMT_status.UI_SOC = 100;

			if ((g_charging_full_reset_bat_meter == KAL_TRUE)
			    && (BMT_status.bat_charging_state == CHR_BATFULL)) {
				resetBatteryMeter = KAL_TRUE;
				g_charging_full_reset_bat_meter = KAL_FALSE;
			} else {
				resetBatteryMeter = KAL_FALSE;
			}
		} else {
			
			if (timer_counter >= (cust_sync_time / BAT_TASK_PERIOD)) {
				timer_counter = 1;
				BMT_status.UI_SOC++;
			} else {
				timer_counter++;

				return resetBatteryMeter;
			}

			resetBatteryMeter = KAL_TRUE;
		}

		battery_xlog_printk(BAT_LOG_CRTI, "[100percent], UI_SOC(%d), reset(%d)\n",
				    BMT_status.UI_SOC, resetBatteryMeter);
	} else {
		

		if (BMT_status.UI_SOC >= 99) {
			BMT_status.UI_SOC = 99;
			resetBatteryMeter = KAL_FALSE;

			battery_xlog_printk(BAT_LOG_CRTI, "[100percent],UI_SOC = %d\n",
					    BMT_status.UI_SOC);
		}

		timer_counter = (cust_sync_time / BAT_TASK_PERIOD);

	}

	return resetBatteryMeter;
}


static kal_bool mt_battery_nPercent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;
#if defined(SOC_BY_HW_FG)
	static kal_uint32 timer_counter = (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD);

	if (BMT_status.nPrecent_UI_SOC_check_point == 0)
		return KAL_FALSE;

	
	if ((BMT_status.ZCV <= BMT_status.nPercent_ZCV)
	    && (BMT_status.UI_SOC > BMT_status.nPrecent_UI_SOC_check_point)) {
		if (timer_counter == (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD))	
		{
			BMT_status.UI_SOC--;
			timer_counter = 1;
		} else {
			timer_counter++;
			return resetBatteryMeter;
		}

		resetBatteryMeter = KAL_TRUE;

		battery_xlog_printk(BAT_LOG_CRTI,
				    "[nPercent] ZCV %d <= nPercent_ZCV %d, UI_SOC=%d., tracking UI_SOC=%d\n",
				    BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC,
				    BMT_status.nPrecent_UI_SOC_check_point);
	} else if ((BMT_status.ZCV > BMT_status.nPercent_ZCV)
		   && (BMT_status.UI_SOC == BMT_status.nPrecent_UI_SOC_check_point)) {
		
		timer_counter = (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD);
		resetBatteryMeter = KAL_TRUE;

		battery_xlog_printk(BAT_LOG_CRTI,
				    "[nPercent] ZCV %d > BMT_status.nPercent_ZCV %d and UI SOC=%d, then keep %d.\n",
				    BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC,
				    BMT_status.nPrecent_UI_SOC_check_point);
	} else {
		timer_counter = (NPERCENT_TRACKING_TIME / BAT_TASK_PERIOD);
	}
#endif
	return resetBatteryMeter;

}

static kal_bool mt_battery_0Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_TRUE;

	if (BMT_status.UI_SOC <= 0) {
		BMT_status.UI_SOC = 0;
	} else {
		if (BMT_status.bat_vol > SYSTEM_OFF_VOLTAGE && BMT_status.UI_SOC > 1) {
			BMT_status.UI_SOC--;
		} else if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE) {
			BMT_status.UI_SOC--;
		}
	}

	battery_xlog_printk(BAT_LOG_CRTI, "0Percent, VBAT < %d UI_SOC=%d\r\n", SYSTEM_OFF_VOLTAGE,
			    BMT_status.UI_SOC);

	return resetBatteryMeter;
}


static void mt_battery_Sync_UI_Percentage_to_Real(void)
{
	static kal_uint32 timer_counter;

	if ((BMT_status.UI_SOC > BMT_status.SOC) && ((BMT_status.UI_SOC != 1))) {
#if !defined (SYNC_UI_SOC_IMM)
		
		if (timer_counter == (SYNC_TO_REAL_TRACKING_TIME / BAT_TASK_PERIOD)) {
			BMT_status.UI_SOC--;
			timer_counter = 0;
		} else {
			timer_counter++;
		}
#else
		BMT_status.UI_SOC--;
#endif
		battery_xlog_printk(BAT_LOG_CRTI, "[Sync_Real] UI_SOC=%d, SOC=%d, counter = %d\n",
				    BMT_status.UI_SOC, BMT_status.SOC, timer_counter);
	} else {
		timer_counter = 0;
		BMT_status.UI_SOC = BMT_status.SOC;
	}

	if (BMT_status.UI_SOC <= 0) {
		BMT_status.UI_SOC = 1;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery]UI_SOC get 0 first (%d)\r\n",
				    BMT_status.UI_SOC);
	}
}

static void battery_update(struct battery_data *bat_data)
{
	struct power_supply *bat_psy = &bat_data->psy;
	kal_bool resetBatteryMeter = KAL_FALSE;

	bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
	bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
	bat_data->BAT_batt_vol = BMT_status.bat_vol;
	bat_data->BAT_batt_temp = BMT_status.temperature * 10;
	bat_data->BAT_PRESENT = BMT_status.bat_exist;

	if ((BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR)) {
		if (BMT_status.bat_exist) {	
			if (BMT_status.bat_vol <= V_0PERCENT_TRACKING) {
				resetBatteryMeter = mt_battery_0Percent_tracking_check();
			} else {
				resetBatteryMeter = mt_battery_100Percent_tracking_check();
			}

			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
		} else {	

			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
			BMT_status.UI_SOC = 0;
		}

	} else {		

		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
		if (BMT_status.bat_vol <= V_0PERCENT_TRACKING)
			resetBatteryMeter = mt_battery_0Percent_tracking_check();
		else
			resetBatteryMeter = mt_battery_nPercent_tracking_check();
	}

	if (resetBatteryMeter == KAL_TRUE) {
		battery_meter_reset();
	} else {
		if (bat_is_recharging_phase() == KAL_TRUE) {
			BMT_status.UI_SOC = 100;
			battery_xlog_printk(BAT_LOG_CRTI, "[recharging] UI_SOC=%d, SOC=%d\n",
					    BMT_status.UI_SOC, BMT_status.SOC);
		} else {
			mt_battery_Sync_UI_Percentage_to_Real();
		}
	}

	battery_xlog_printk(BAT_LOG_CRTI, "UI_SOC=(%d), resetBatteryMeter=(%d)\n",
			    BMT_status.UI_SOC, resetBatteryMeter);

	
	if (BMT_status.UI_SOC <= 1) {
		set_rtc_spare_fg_value(1);
	} else {
		set_rtc_spare_fg_value(BMT_status.UI_SOC);
	}

	mt_battery_update_EM(bat_data);

	if (cmd_discharging == 1) {
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CMD_DISCHARGING;
	}
	if (adjust_power != -1) {
			bat_data->adjust_power = adjust_power;
			battery_xlog_printk(BAT_LOG_CRTI, "adjust_power=(%d)\n", adjust_power);
	}

	power_supply_changed(bat_psy);
}
#endif

void update_charger_info(int wireless_state)
{
#if defined(CONFIG_POWER_VERIFY)
	battery_xlog_printk(BAT_LOG_CRTI, "[update_charger_info] no support\n");
#else
	g_wireless_state = wireless_state;
	battery_xlog_printk(BAT_LOG_CRTI, "[update_charger_info] get wireless_state=%d\n",
			    wireless_state);

	wake_up_bat();
#endif
}

static void wireless_update(struct wireless_data *wireless_data)
{
	struct power_supply *wireless_psy = &wireless_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE || g_wireless_state) {
		if ((BMT_status.charger_type == WIRELESS_CHARGER) || g_wireless_state) {
			wireless_data->WIRELESS_ONLINE = 1;
			wireless_psy->type = POWER_SUPPLY_TYPE_WIRELESS;
		} else {
			wireless_data->WIRELESS_ONLINE = 0;
		}
	} else {
		wireless_data->WIRELESS_ONLINE = 0;
	}

	power_supply_changed(wireless_psy);
}

static void ac_update(struct ac_data *ac_data)
{
	struct power_supply *ac_psy = &ac_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if ((BMT_status.charger_type == NONSTANDARD_CHARGER) ||
		    (BMT_status.charger_type == STANDARD_CHARGER)    ||
			(BMT_status.charger_type == APPLE_2_1A_CHARGER) ||
			(BMT_status.charger_type == APPLE_1_0A_CHARGER) ||
			(BMT_status.charger_type == APPLE_0_5A_CHARGER)) {
		#else
		if ((BMT_status.charger_type == NONSTANDARD_CHARGER) ||
		    (BMT_status.charger_type == STANDARD_CHARGER)    ||
			(BMT_status.charger_type == APPLE_2_1A_CHARGER) ||
			(BMT_status.charger_type == APPLE_1_0A_CHARGER) ||
			(BMT_status.charger_type == APPLE_0_5A_CHARGER) ||
			(DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
		#endif
			ac_data->AC_ONLINE = 1;
			ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
		} else {
			ac_data->AC_ONLINE = 0;
		}
	} else {
		ac_data->AC_ONLINE = 0;
	}

	power_supply_changed(ac_psy);
}

static void usb_update(struct usb_data *usb_data)
{
	struct power_supply *usb_psy = &usb_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if ((BMT_status.charger_type == STANDARD_HOST) ||
		    (BMT_status.charger_type == CHARGING_HOST)) {
			usb_data->USB_ONLINE = 1;
			usb_psy->type = POWER_SUPPLY_TYPE_USB;
		} else {
			usb_data->USB_ONLINE = 0;
		}
	} else {
		usb_data->USB_ONLINE = 0;
	}

	power_supply_changed(usb_psy);
}

#endif

kal_bool pmic_chrdet_status(void)
{
	if (upmu_is_chr_det() == KAL_TRUE) {
		return KAL_TRUE;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger\r\n");
		return KAL_FALSE;
	}
}

kal_bool bat_is_charger_exist(void)
{
	return get_charger_detect_status();
}


kal_bool bat_is_charging_full(void)
{
	if ((BMT_status.bat_full == KAL_TRUE) && (BMT_status.bat_in_recharging_state == KAL_FALSE))
		return KAL_TRUE;
	else
		return KAL_FALSE;
}

void bat_set_ui_percentage(kal_uint32 uiSoc)
{
	int batt_low_vol = htc_battery_0percent_volt(BMT_status.temperature_now);

    BMT_status.UI_SOC = uiSoc;

    
    if( BMT_status.UI_SOC < 1 || BMT_status.bat_vol < batt_low_vol){
        set_rtc_spare_fg_value(1);
    } else {
        set_rtc_spare_fg_value(BMT_status.UI_SOC);
    }
}

kal_uint32 bat_get_ui_percentage(void)
{
	
		return BMT_status.UI_SOC;
}

kal_uint32 bat_is_recharging_phase(void)
{
	return (BMT_status.bat_in_recharging_state || BMT_status.bat_full == KAL_TRUE);
}


int get_bat_charging_current_level(void)
{
	CHR_CURRENT_ENUM charging_current;

	battery_charging_control(CHARGING_CMD_GET_CURRENT, &charging_current);

	return charging_current;
}

int htc_batt_temp_cv_set(INT32 temperature)
{
	int cv_sel_value;
	static bool is_batt_warm = false;
	static bool first = true;

	if (temperature > 48 && !is_batt_warm)
		is_batt_warm = true;
	else if (is_batt_warm && (temperature <= 45))
		is_batt_warm = false;

	if (is_batt_warm ^ BMT_status.is_warm || first) {

		battery_xlog_printk(BAT_LOG_CRTI,"is_warm changed: %d\n", is_batt_warm);
		BMT_status.is_warm = is_batt_warm;
		first = false;

		if (BMT_status.is_warm)
			cv_sel_value = BATTERY_VOLT_04_000000_V;
		else
			cv_sel_value = BATTERY_VOLT_04_387500_V;

		battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_sel_value);
	}
}

void htc_batt_mhl_charge_set(kal_uint32 mhl_chg_type)
{
	battery_xlog_printk(BAT_LOG_CRTI,"%s: mhl_chg_type(%d)\n", __func__, mhl_chg_type);
	g_chg_in_mhl = mhl_chg_type;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
int htc_batt_update_limited_charge(INT32 temperature)
{
	kal_uint32 ichr;
	int prev_chg_limit_reason;

	prev_chg_limit_reason = g_chg_limit_reason;

	if (g_display_on == KAL_FALSE) {
		
		g_chg_limit_reason &= ~HTC_BATT_CHG_LIMIT_BIT_THRML;
	} else {
		
		if ((!(g_chg_limit_reason & HTC_BATT_CHG_LIMIT_BIT_THRML)) && temperature > 39
			&& BMT_status.SOC > 30) {
			g_chg_limit_reason |= HTC_BATT_CHG_LIMIT_BIT_THRML;
		} else if ((g_chg_limit_reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && temperature <= 37) {
			g_chg_limit_reason &= ~HTC_BATT_CHG_LIMIT_BIT_THRML;
		} else {
			
		}
	}

	if (prev_chg_limit_reason ^ g_chg_limit_reason) {
		battery_xlog_printk(BAT_LOG_CRTI,"chg_limit_reason:0x%x->0x%d, temp(%d)",
								prev_chg_limit_reason, g_chg_limit_reason, temperature);
	}
}
#endif

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
PMU_STATUS do_batt_temp_state_machine(INT32 temperature)
{
	if (temperature == ERR_CHARGE_TEMPERATURE) {
		return PMU_STATUS_FAIL;
	}

	htc_batt_temp_cv_set(temperature);

#ifdef CONFIG_HAS_EARLYSUSPEND
	htc_batt_update_limited_charge(temperature);
#endif

#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	if (temperature <= MIN_CHARGE_TEMPERATURE) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
		g_batt_temp_status = TEMP_POS_LOW;
		return PMU_STATUS_FAIL;
	} else if (g_batt_temp_status == TEMP_POS_LOW) {
		if (temperature >= MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature raise from %d to %d(%d), allow charging!!\n\r",
					    MIN_CHARGE_TEMPERATURE, temperature,
					    MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE);
			g_batt_temp_status = TEMP_POS_NORMAL;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else {
			return PMU_STATUS_FAIL;
		}
	} else
#endif
	if (temperature >= MAX_CHARGE_TEMPERATURE) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");
		g_batt_temp_status = TEMP_POS_HIGH;
		return PMU_STATUS_FAIL;
	} else if (g_batt_temp_status == TEMP_POS_HIGH) {
		if (temperature <= MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature down from %d to %d(%d), allow charging!!\n\r",
					    MAX_CHARGE_TEMPERATURE, temperature,
					    MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE);
			g_batt_temp_status = TEMP_POS_NORMAL;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else {
			return PMU_STATUS_FAIL;
		}
	} else {
		g_batt_temp_status = TEMP_POS_NORMAL;
	}
	return PMU_STATUS_OK;
}
#endif

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
	unsigned long ret_val = 0;

#if defined(CONFIG_POWER_EXT)
	ret_val = 4000;
#else
    ret_val=battery_meter_get_battery_voltage(KAL_FALSE);
#endif

	return ret_val;
}


static void mt_battery_average_method_init(BATTERY_AVG_ENUM type, kal_uint32 *bufferdata, kal_uint32 data,
					   kal_int32 *sum)
{
	kal_uint32 i;
	static kal_bool batteryBufferFirst = KAL_TRUE;
	static kal_bool previous_charger_exist = KAL_FALSE;
	static kal_bool previous_in_recharge_state = KAL_FALSE;
	static kal_uint8 index;

	
    if (type == BATTERY_AVG_CURRENT) {
	if (BMT_status.charger_exist == KAL_TRUE) {
		if (previous_charger_exist == KAL_FALSE) {
			batteryBufferFirst = KAL_TRUE;
			previous_charger_exist = KAL_TRUE;
			#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			if (BMT_status.charger_type == STANDARD_CHARGER) {
			#else
			if ((BMT_status.charger_type == STANDARD_CHARGER) || 
			    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
			#endif
				data = AC_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == CHARGING_HOST) {
				data = CHARGING_HOST_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
				data = NON_STD_AC_CHARGER_CURRENT / 100;	
			else	
				data = USB_CHARGER_CURRENT / 100;	
		} else if ((previous_in_recharge_state == KAL_FALSE)
			   && (BMT_status.bat_in_recharging_state == KAL_TRUE)) {
			batteryBufferFirst = KAL_TRUE;
			#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			if (BMT_status.charger_type == STANDARD_CHARGER) {
			#else
			if ((BMT_status.charger_type == STANDARD_CHARGER) || 
			    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
			#endif			
				data = AC_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == CHARGING_HOST) {
				data = CHARGING_HOST_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
				data = NON_STD_AC_CHARGER_CURRENT / 100;	
			else	
				data = USB_CHARGER_CURRENT / 100;	
		}

		previous_in_recharge_state = BMT_status.bat_in_recharging_state;
	} else {
		if (previous_charger_exist == KAL_TRUE) {
			batteryBufferFirst = KAL_TRUE;
			previous_charger_exist = KAL_FALSE;
			data = 0;
		}
	}
    }
	

	battery_xlog_printk(BAT_LOG_FULL, "batteryBufferFirst =%d, data= (%d)\n",
			    batteryBufferFirst, data);

	if (batteryBufferFirst == KAL_TRUE) {
		for (i = 0; i < BATTERY_AVERAGE_SIZE; i++) {
			bufferdata[i] = data;
		}

		*sum = data * BATTERY_AVERAGE_SIZE;
	}

	index++;
	if (index >= BATTERY_AVERAGE_DATA_NUMBER) {
		index = BATTERY_AVERAGE_DATA_NUMBER;
		batteryBufferFirst = KAL_FALSE;
	}
}


static kal_uint32 mt_battery_average_method(BATTERY_AVG_ENUM type, kal_uint32 *bufferdata, kal_uint32 data,
					    kal_int32 *sum, kal_uint8 batteryIndex)
{
	kal_uint32 avgdata;

	mt_battery_average_method_init(type, bufferdata, data, sum);

	*sum -= bufferdata[batteryIndex];
	*sum += data;
	bufferdata[batteryIndex] = data;
	avgdata = (*sum) / BATTERY_AVERAGE_SIZE;

	battery_xlog_printk(BAT_LOG_FULL, "bufferdata[%d]= (%d)\n", batteryIndex,
			    bufferdata[batteryIndex]);
	return avgdata;
}

void mt_battery_GetBatteryData(void)
{
	kal_uint32 bat_vol, charger_vol, Vsense, ZCV;
	kal_int32 ICharging, temperature, temperatureR, temperatureV, SOC;
	static kal_int32 bat_sum, icharging_sum, temperature_sum;
	static kal_int32 batteryVoltageBuffer[BATTERY_AVERAGE_SIZE];
	static kal_int32 batteryCurrentBuffer[BATTERY_AVERAGE_SIZE];
	static kal_int32 batteryTempBuffer[BATTERY_AVERAGE_SIZE];
	static kal_uint8 batteryIndex = 0;
	kal_int32 previous_SOC = BMT_status.SOC;

	bat_vol = battery_meter_get_battery_voltage(KAL_TRUE);
	Vsense = battery_meter_get_VSense();
	
	charger_vol = battery_meter_get_charger_voltage();
	if( upmu_is_chr_det() == KAL_TRUE ) {
		ICharging = battery_meter_get_charging_current();
	} else {
		ICharging = 0;
	}

	if (bat_meter_timeout == KAL_TRUE || bat_spm_timeout == TRUE) {
		SOC = battery_meter_get_battery_percentage();
		
			

		bat_meter_timeout = KAL_FALSE;
		bat_spm_timeout = FALSE;
	} else {
		if (previous_SOC == -1)
			SOC = battery_meter_get_battery_percentage();
		else
			SOC = previous_SOC;
	}

    BMT_status.gap = htc_battery_adjust_gap(BMT_status.gap, previous_SOC, SOC);

	temperature = battery_meter_get_battery_temperature(FALSE);
	temperatureV = battery_meter_get_tempV();
	temperatureR = battery_meter_get_tempR(temperatureV);
	ZCV = battery_meter_get_battery_zcv();

	BMT_status.ICharging =
	    mt_battery_average_method(BATTERY_AVG_CURRENT, &batteryCurrentBuffer[0], ICharging, &icharging_sum,
				      batteryIndex);

    
	if (previous_SOC == -1 && bat_vol <= V_0PERCENT_TRACKING) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "battery voltage too low, use ZCV to init average data.\n");
		BMT_status.bat_vol =
		    mt_battery_average_method(BATTERY_AVG_VOLT, &batteryVoltageBuffer[0], ZCV, &bat_sum,
					      batteryIndex);
	} else {
		if (bat_vol < 2850) {
			battery_xlog_printk(BAT_LOG_CRTI,"low voltage(%d)!! Ignore it.\n",bat_vol);
			bat_vol = batteryVoltageBuffer[(batteryIndex+BATTERY_AVERAGE_SIZE-1)%BATTERY_AVERAGE_SIZE];
		}

		BMT_status.bat_vol =
		    mt_battery_average_method(BATTERY_AVG_VOLT, &batteryVoltageBuffer[0], bat_vol, &bat_sum,
					      batteryIndex);
	}
	BMT_status.temperature =
	    mt_battery_average_method(BATTERY_AVG_TEMP, &batteryTempBuffer[0], temperature, &temperature_sum,
				      batteryIndex);
	BMT_status.temperature_now = temperature;
	BMT_status.Vsense = Vsense;
	BMT_status.charger_vol = charger_vol;
	BMT_status.temperatureV = temperatureV;
	BMT_status.temperatureR = temperatureR;
	BMT_status.SOC = SOC;
	BMT_status.ZCV = ZCV;


	batteryIndex++;
	if (batteryIndex >= BATTERY_AVERAGE_SIZE)
		batteryIndex = 0;


	if (g_battery_soc_ready == KAL_FALSE)
		g_battery_soc_ready = KAL_TRUE;

	battery_xlog_printk(BAT_LOG_CRTI,
			    "AvgVbat=(%d),bat_vol=(%d),AvgI=(%d),I=(%d),VChr=(%d),AvgT=(%d),T=(%d),is_warm=(%d),pre_SOC=(%d),SOC=(%d),UI_SOC=(%d),ZCV=(%d),"
			    "bat_charging_state=(0x%x),ftm_chg_ctrl=(%d),chg_ctrl=(%d),overload=(%d),mhl=(%d),chg_internal=(%d),gap=(%d)\n",
			    BMT_status.bat_vol, bat_vol, BMT_status.ICharging, ICharging,
			    BMT_status.charger_vol, BMT_status.temperature, temperature, BMT_status.is_warm,
			    previous_SOC, BMT_status.SOC, BMT_status.UI_SOC, BMT_status.ZCV, BMT_status.bat_charging_state,
			    g_ftm_charger_ctrl_stat, charger_ctrl_stat, BMT_status.is_overload,g_chg_in_mhl,g_charger_ctrl_internal,BMT_status.gap);



}


static PMU_STATUS mt_battery_CheckBatteryTemp(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if (BMT_status.keep_charger_on || BMT_status.flag_disable_temp_protection)
		return PMU_STATUS_OK;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] support JEITA, temperature=%d\n",
			    BMT_status.temperature);

	if (do_jeita_state_machine() == PMU_STATUS_FAIL) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] JEITA : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
	
	
	if (do_batt_temp_state_machine(BMT_status.temperature_now) == PMU_STATUS_FAIL) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Batt temp check : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
	    || (BMT_status.temperature == ERR_CHARGE_TEMPERATURE)) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
		status = PMU_STATUS_FAIL;
	}
#endif
	if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");
		status = PMU_STATUS_FAIL;
	}
#endif

#endif

	return status;
}


static PMU_STATUS mt_battery_CheckChargerVoltage(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	kal_uint32 v_charger_max = DISO_data.hv_voltage;
	#endif

	if (BMT_status.charger_exist == KAL_TRUE) {
#if (V_CHARGER_ENABLE == 1)
		if (BMT_status.charger_vol <= V_CHARGER_MIN) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger under voltage!!\r\n");
			BMT_status.bat_charging_state = CHR_ERROR;
			status = PMU_STATUS_FAIL;
		}
#endif
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (BMT_status.charger_vol >= V_CHARGER_MAX) {
		#else
		if (BMT_status.charger_vol >= v_charger_max) {
		#endif
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger over voltage !!\r\n");
			BMT_status.charger_protect_status = charger_OVER_VOL;
			BMT_status.bat_charging_state = CHR_ERROR;
			status = PMU_STATUS_FAIL;
		}
	}

	return status;
}


static PMU_STATUS mt_battery_CheckChargingTime(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)
		|| BMT_status.keep_charger_on || BMT_status.force_ac_charger
		|| BMT_status.flag_disable_safety_timer) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[TestMode] Disable Safty Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
				    g_battery_thermal_throttling_flag,
				    battery_cmd_thermal_test_mode,
				    battery_cmd_thermal_test_mode_value);

	} else {
		
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME && BMT_status.charger_type == STANDARD_CHARGER) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time.\n");

			status = PMU_STATUS_FAIL;
		}
	}

	return status;

}

#if defined(STOP_CHARGING_IN_TAKLING)
static PMU_STATUS mt_battery_CheckCallState(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_call_state == CALL_ACTIVE) && (BMT_status.bat_vol > V_CC2TOPOFF_THRES))
		status = PMU_STATUS_FAIL;

	return status;
}
#endif

static PMU_STATUS htc_battery_FullLevelCheck(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
	static kal_bool is_input_chg_off_by_bounding = KAL_FALSE;

	if (!BMT_status.charger_exist) {
		is_input_chg_off_by_bounding = KAL_FALSE;
		return status;
	}

	if ((0 < BMT_status.full_level) && (BMT_status.full_level < 100)) {

		battery_xlog_printk(BAT_LOG_CRTI,"[htc_battery_FullLevelCheck] "
			"is_input_chg_off_by_bounding:%d\n",is_input_chg_off_by_bounding);

		if (BMT_status.UI_SOC >= BMT_status.full_level) {
			status = PMU_STATUS_FAIL;
			is_input_chg_off_by_bounding = KAL_TRUE;
		} else if (is_input_chg_off_by_bounding) {
			if (BMT_status.UI_SOC <= (BMT_status.full_level - 5)) { 
				BMT_status.bat_charging_state = CHR_PRE;
				is_input_chg_off_by_bounding = KAL_FALSE;
			}
		}
	}

	return status;
}

static PMU_STATUS htc_battery_CharegerControlCheck(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
	static int pre_charger_ctrl_stat = -1;
	static int pre_g_ftm_charger_ctrl_stat = -1;
	static int pre_meta_charging_enable = -1;
	static int pre_g_charger_ctrl_internal = -1;

	if ((pre_charger_ctrl_stat != charger_ctrl_stat)
		|| (pre_g_ftm_charger_ctrl_stat != g_ftm_charger_ctrl_stat)
		|| (pre_meta_charging_enable != meta_charging_enable)
		|| (pre_g_charger_ctrl_internal != g_charger_ctrl_internal)) {

		BMT_status.bat_charging_state = CHR_PRE;

		battery_xlog_printk(BAT_LOG_CRTI, "charger_ctrl_stat changes (%d -> %d), "
			"g_ftm_charger_ctrl_stat changes (%d->%d), pre_meta_charging_enable (%d->%d), "
			"g_charger_ctrl_internal changes (%d->%d)\n",
			pre_charger_ctrl_stat,charger_ctrl_stat,pre_g_ftm_charger_ctrl_stat,
			g_ftm_charger_ctrl_stat,pre_meta_charging_enable,meta_charging_enable,
			pre_g_charger_ctrl_internal, g_charger_ctrl_internal);

		pre_charger_ctrl_stat = charger_ctrl_stat;
		pre_g_ftm_charger_ctrl_stat = g_ftm_charger_ctrl_stat;
		pre_meta_charging_enable = meta_charging_enable;
		pre_g_charger_ctrl_internal = g_charger_ctrl_internal;
	}

	if (charger_ctrl_stat == STOP_CHARGER
		|| charger_ctrl_stat == DISABLE_PWRSRC
		|| g_ftm_charger_ctrl_stat == FTM_STOP_CHARGER
		|| meta_charging_enable == STOP_CHARGER
		|| g_charger_ctrl_internal == DISABLE_PWRSRC_FINGERPRINT) {
		status = PMU_STATUS_FAIL;
	}

	return status;
}

static void mt_battery_CheckBatteryStatus(void)
{
	battery_xlog_printk(BAT_LOG_FULL, "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
			    cmd_discharging);
	if (cmd_discharging == 1) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
				    cmd_discharging);
		BMT_status.bat_charging_state = CHR_ERROR;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);	
		return;
	} else if (cmd_discharging == 0) {
		BMT_status.bat_charging_state = CHR_PRE;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);
		cmd_discharging = -1;
	}
	if (mt_battery_CheckBatteryTemp() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (mt_battery_CheckChargerVoltage() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
#if defined(STOP_CHARGING_IN_TAKLING)
	if (mt_battery_CheckCallState() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_HOLD;
		return;
	}
#endif

	if (mt_battery_CheckChargingTime() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (htc_battery_FullLevelCheck() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (htc_battery_CharegerControlCheck() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (fgauge_get_battery_id() == FG_ERROR_BATTERY) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
}


static void mt_battery_notify_TotalChargingTime_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME)
	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[TestMode] Disable Safty Timer : no UI display\n");
	} else {
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME)
			
		{
			g_BatteryNotifyCode |= 0x0010;
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time\n");
		} else {
			g_BatteryNotifyCode &= ~(0x0010);
		}
	}

	battery_xlog_printk(BAT_LOG_CRTI,
			    "[BATTERY] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME (%x)\n",
			    g_BatteryNotifyCode);
#endif
}


static void mt_battery_notify_VBat_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0004_VBAT)
	if (BMT_status.bat_vol > 4350)
		
	{
		g_BatteryNotifyCode |= 0x0008;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_vlot(%ld) > 4350mV\n",
				    BMT_status.bat_vol);
	} else {
		g_BatteryNotifyCode &= ~(0x0008);
	}

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0004_VBAT (%x)\n",
			    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_ICharging_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0003_ICHARGING)
	if ((BMT_status.ICharging > 1000) && (BMT_status.total_charging_time > 300)) {
		g_BatteryNotifyCode |= 0x0004;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] I_charging(%ld) > 1000mA\n",
				    BMT_status.ICharging);
	} else {
		g_BatteryNotifyCode &= ~(0x0004);
	}

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0003_ICHARGING (%x)\n",
			    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_VBatTemp_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)
    static kal_uint32 batt_temp_status = TEMP_POS_NORMAL;

	if (BMT_status.temperature_now >= MAX_CHARGE_TEMPERATURE) {
		g_BatteryNotifyCode |= 0x0002;
        batt_temp_status = TEMP_POS_HIGH;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too high)\n",
				    BMT_status.temperature_now);
	}
    else if( batt_temp_status == TEMP_POS_HIGH ){
        if( BMT_status.temperature_now > MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE ){
            g_BatteryNotifyCode |= 0x0002;
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too high)\n",
                        BMT_status.temperature_now);
        }else
            batt_temp_status = TEMP_POS_NORMAL;
    }
    if( BMT_status.is_warm ){
        g_BatteryNotifyCode |= 0x0002;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) is high\n",
                    BMT_status.temperature_now);
    }
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	else if (BMT_status.temperature < TEMP_NEG_10_THRESHOLD) {
		g_BatteryNotifyCode |= 0x0020;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
				    BMT_status.temperature);
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	else if (BMT_status.temperature_now <= MIN_CHARGE_TEMPERATURE) {
		g_BatteryNotifyCode |= 0x0020;
        batt_temp_status = TEMP_POS_LOW;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
				    BMT_status.temperature_now);
	}
    else if( batt_temp_status == TEMP_POS_LOW ){
        if( BMT_status.temperature_now < MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE ){
            g_BatteryNotifyCode |= 0x0002;
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
                        BMT_status.temperature_now);
        }else
            batt_temp_status = TEMP_POS_NORMAL;
    }
#endif
#endif

	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] BATTERY_NOTIFY_CASE_0002_VBATTEMP (%x)\n",
			    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_VCharger_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	kal_uint32 v_charger_max = DISO_data.hv_voltage;
	#endif

	#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if (BMT_status.charger_vol > V_CHARGER_MAX) {
	#else
	if (BMT_status.charger_vol > v_charger_max) {
	#endif
		g_BatteryNotifyCode |= 0x0001;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BMT_status.charger_vol(%d) > %d mV\n",
				    BMT_status.charger_vol, V_CHARGER_MAX);
	} else {
		g_BatteryNotifyCode &= ~(0x0001);
	}
	if (g_BatteryNotifyCode != 0x0000)
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] BATTERY_NOTIFY_CASE_0001_VCHARGER (%x)\n",
				    g_BatteryNotifyCode);
#endif
}

static void mt_battery_notify_BattId_check(void)
{
    if( fgauge_get_battery_id() == FG_ERROR_BATTERY ){
        g_BatteryNotifyCode |= 0x0040;
        battery_xlog_printk(BAT_LOG_CRTI,
                    "[BATTERY] BATTERY_NOTIFY_CASE_BATTID\n");
    }
}

static void mt_battery_notify_UI_test(void)
{
	if (g_BN_TestMode == 0x0001) {
		g_BatteryNotifyCode = 0x0001;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001_VCHARGER\n");
	} else if (g_BN_TestMode == 0x0002) {
		g_BatteryNotifyCode = 0x0002;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002_VBATTEMP\n");
	} else if (g_BN_TestMode == 0x0003) {
		g_BatteryNotifyCode = 0x0004;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003_ICHARGING\n");
	} else if (g_BN_TestMode == 0x0004) {
		g_BatteryNotifyCode = 0x0008;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004_VBAT\n");
	} else if (g_BN_TestMode == 0x0005) {
		g_BatteryNotifyCode = 0x0010;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME\n");
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Unknown BN_TestMode Code : %x\n",
				    g_BN_TestMode);
	}
}


void mt_battery_notify_check(void)
{
	g_BatteryNotifyCode &= 0x0001;

	if (g_BN_TestMode == 0x0000) {	
		battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] mt_battery_notify_check\n");

		mt_battery_notify_VCharger_check();

		mt_battery_notify_VBatTemp_check();

		mt_battery_notify_ICharging_check();

		mt_battery_notify_VBat_check();

		mt_battery_notify_TotalChargingTime_check();

        mt_battery_notify_BattId_check();
	} else {		

		mt_battery_notify_UI_test();
	}
}

static void mt_battery_thermal_check(void)
{
	if ((g_battery_thermal_throttling_flag == 1) || (g_battery_thermal_throttling_flag == 3)) {
		if (battery_cmd_thermal_test_mode == 1) {
			BMT_status.temperature = battery_cmd_thermal_test_mode_value;
			battery_xlog_printk(BAT_LOG_FULL,
					    "[Battery] In thermal_test_mode , Tbat=%d\n",
					    BMT_status.temperature);
		}
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		
#else
		if (BMT_status.temperature >= 68) {
#if defined(CONFIG_POWER_EXT)
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] CONFIG_POWER_EXT, no update battery update power down.\n");
#else
			{
				if ((g_platform_boot_mode == META_BOOT)
				    || (g_platform_boot_mode == ADVMETA_BOOT)
				    || (g_platform_boot_mode == ATE_FACTORY_BOOT)
				    || BMT_status.keep_charger_on
				    || BMT_status.flag_disable_temp_protection) {
					battery_xlog_printk(BAT_LOG_FULL,
							    "[BATTERY] boot mode = %d, bypass temperature check\n",
							    g_platform_boot_mode);
				} else {
					struct battery_data *bat_data = &battery_main;
					struct power_supply *bat_psy = &bat_data->psy;

					battery_xlog_printk(BAT_LOG_CRTI,
							    "[Battery] Tbat(%d)>=68, system need power down.\n",
							    BMT_status.temperature);

					bat_data->BAT_CAPACITY = 0;

					power_supply_changed(bat_psy);

					if (BMT_status.charger_exist == KAL_TRUE) {
						
						battery_charging_control
						    (CHARGING_CMD_SET_PLATFORM_RESET, NULL);
					}
					
					battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
					
				}
			}
#endif
		}
#endif

	}

}


static void mt_battery_update_status(void)
{
	static int last_src = -1;
	static int last_UI_SOC = -1;
	static int last_temperature = -1;
	static int last_bat_charging_state = -1;
	static struct timespec last_ts;
	struct timespec ts;
	getnstimeofday(&ts);

#if defined(CONFIG_POWER_EXT)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
#else
	{
		battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] time now: %lu. time last: %lu\n", ts.tv_sec, last_ts.tv_sec);
		if(BMT_status.charger_type != last_src ||
			BMT_status.UI_SOC != last_UI_SOC ||
			BMT_status.temperature != last_temperature ||
			BMT_status.bat_charging_state != last_bat_charging_state ||
			(ts.tv_sec - last_ts.tv_sec >= 60)) {
			battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] Update uevent.\n");
			getnstimeofday(&last_ts);
			htc_battery_update(&battery_main);

			if(BMT_status.charger_type != last_src){
				wireless_update(&wireless_main);
				ac_update(&ac_main);
				usb_update(&usb_main);
			}
			last_src = BMT_status.charger_type;
			last_UI_SOC = BMT_status.UI_SOC;
			last_temperature = BMT_status.temperature;
			last_bat_charging_state = BMT_status.bat_charging_state;
		}
	}
#endif
}


CHARGER_TYPE mt_charger_type_detection(void)
{
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

	mutex_lock(&charger_type_mutex);

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
	battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
	BMT_status.charger_type = CHR_Type_num;
#else
	#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if (BMT_status.charger_type == CHARGER_UNKNOWN) {
	#else
	if ((BMT_status.charger_type == CHARGER_UNKNOWN) &&
	    (DISO_data.diso_state.cur_vusb_state == DISO_ONLINE)) {
	#endif
		battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
		BMT_status.charger_type = CHR_Type_num;

#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)&&(defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT))
 	    if (BMT_status.UI_SOC == 100)
		{
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
			g_charging_full_reset_bat_meter = KAL_TRUE;
		}	

		 if(g_battery_soc_ready == KAL_FALSE) {
			if(BMT_status.nPercent_ZCV == 0)
				battery_meter_initial();
					
			BMT_status.SOC = battery_meter_get_battery_percentage();
		}

		if (BMT_status.bat_vol > 0)
		{
        	mt_battery_update_status();
		}
		
#endif
	}
#endif
	mutex_unlock(&charger_type_mutex);

	return BMT_status.charger_type;
}

CHARGER_TYPE mt_get_charger_type(void)
{
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    return STANDARD_HOST;
#else
	return BMT_status.charger_type;
#endif
}

static void mt_battery_charger_detect_check(void)
{
	if (upmu_is_chr_det() == KAL_TRUE) {
		wake_lock(&battery_suspend_lock);

		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		BMT_status.charger_exist = KAL_TRUE;
		#endif

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
		mt_charger_type_detection();

		if ((BMT_status.charger_type == STANDARD_HOST)
		    || (BMT_status.charger_type == CHARGING_HOST)) {
			mt_usb_connect();
		}
#else
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (BMT_status.charger_type == CHARGER_UNKNOWN) {
		#else
		if ((BMT_status.charger_type == CHARGER_UNKNOWN) &&
		    (DISO_data.diso_state.cur_vusb_state == DISO_ONLINE)) {
		#endif
			mt_charger_type_detection();

			if ((BMT_status.charger_type == STANDARD_HOST)
			    || (BMT_status.charger_type == CHARGING_HOST)) {
				mt_usb_connect();
			}
		}
#endif

		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_thread]Cable in, CHR_Type_num=%d\r\n",
				    BMT_status.charger_type);

	} else {
		wake_unlock(&battery_suspend_lock);

		BMT_status.charger_exist = KAL_FALSE;
		BMT_status.charger_type = CHARGER_UNKNOWN;
		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_in_recharging_state = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_PRE;
		BMT_status.total_charging_time = 0;
		BMT_status.PRE_charging_time = 0;
		BMT_status.CC_charging_time = 0;
		BMT_status.TOPOFF_charging_time = 0;
		BMT_status.POSTFULL_charging_time = 0;

		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_thread]Cable out \r\n");

		mt_usb_disconnect();
	}
}

static void mt_kpoc_power_off_check(void)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	battery_xlog_printk(BAT_LOG_FULL,
			    "[mt_kpoc_power_off_check] , chr_vol=%d, boot_mode=%d\r\n", BMT_status.charger_vol,
			    g_platform_boot_mode);

	if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		if ((upmu_is_chr_det() == KAL_FALSE) && (BMT_status.charger_vol < 2500))	
		{
#ifdef HTC_ENABLE_AICL
			if( delayed_work_pending(&BMT_status.vin_collapse_check_work) )
				return;
#endif
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[bat_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}

void update_battery_2nd_info(int status_smb, int capacity_smb, int present_smb)
{
#if defined(CONFIG_POWER_VERIFY)
	battery_xlog_printk(BAT_LOG_CRTI, "[update_battery_smb_info] no support\n");
#else
	g_status_smb = status_smb;
	g_capacity_smb = capacity_smb;
	g_present_smb = present_smb;
	battery_xlog_printk(BAT_LOG_CRTI,
			    "[update_battery_smb_info] get status_smb=%d,capacity_smb=%d,present_smb=%d\n",
			    status_smb, capacity_smb, present_smb);

	wake_up_bat();
	g_smartbook_update = 1;
#endif
}

void do_chrdet_int_task(void)
{
	if (g_bat_init_flag == KAL_TRUE) {
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (upmu_is_chr_det() == KAL_TRUE) {
		#else
		battery_charging_control(CHARGING_CMD_GET_DISO_STATE, &DISO_data);
		if ((DISO_data.diso_state.cur_vusb_state == DISO_ONLINE) ||
		    (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE)) {
		#endif
			battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] charger exist!\n");
			BMT_status.charger_exist = KAL_TRUE;

#ifdef HTC_ENABLE_AICL
            if( delayed_work_pending(&BMT_status.vin_collapse_check_work) )
                return;
            spin_lock(&BMT_status.aicl_lock);
            BMT_status.aicl_state = AICL_STOP;
            spin_unlock(&BMT_status.aicl_lock);
#endif
			wake_lock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
			mt_usb_connect();
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_connect();
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
				return;
			}
#endif
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] charger NOT exist!\n");
			BMT_status.charger_exist = KAL_FALSE;

			#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			battery_xlog_printk(BAT_LOG_CRTI, 
					    "turn off charging for no avaliable charging source\n");
			battery_charging_control(CHARGING_CMD_ENABLE,&BMT_status.charger_exist); 
			#endif

#ifdef HTC_ENABLE_AICL
			spin_lock(&BMT_status.aicl_lock);
			BMT_status.aicl_state = AICL_STOP;
			spin_unlock(&BMT_status.aicl_lock);
			schedule_delayed_work(&BMT_status.vin_collapse_check_work, msecs_to_jiffies(VIN_MIN_COLLAPSE_CHECK_MS));
			return;
#endif
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
				battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
				
			}
#endif
			wake_unlock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
			mt_usb_disconnect();
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_disconnect();
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
				return;
			}
#endif
			#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
				 is_ta_connect = KAL_FALSE;    
				 ta_check_chr_type = KAL_TRUE;
				 ta_cable_out_occur = KAL_TRUE;
			#endif

		}

		

		mt_battery_charger_detect_check();

		if (g_battery_soc_ready == KAL_FALSE) {
			if (BMT_status.nPercent_ZCV == 0)
				battery_meter_initial();

			BMT_status.SOC = battery_meter_get_battery_percentage();
		}

		if (BMT_status.bat_vol > 0) {
			mt_battery_update_status();
		}

		#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		DISO_data.chr_get_diso_state = KAL_TRUE;
		#endif

		wake_up_bat();
	} else {
		#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		g_vcdt_irq_delay_flag  = KAL_TRUE;
		#endif
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");
	}

}

#ifdef HTC_ENABLE_AICL
static void vin_collapse_check_worker(struct work_struct *work)
{
    kal_uint32 ichr;
    unsigned long flags;

    battery_xlog_printk(BAT_LOG_CRTI, "[%s] charger_exist = %d\n", __FUNCTION__, BMT_status.charger_exist);

    if (delayed_work_pending(&BMT_status.aicl_check_work))
        cancel_delayed_work_sync(&BMT_status.aicl_check_work);

    if( BMT_status.charger_exist == KAL_TRUE ){
        if( BMT_status.charger_type == STANDARD_CHARGER ){
            ichr = CHR_CUR[0];
            battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &ichr);
            battery_charging_control(CHARGING_CMD_SET_CURRENT, &ichr);
            battery_charging_control(CHARGING_CMD_HTC_AICL_COMPLETE, NULL);
            spin_lock_irqsave(&BMT_status.aicl_lock, flags);
            BMT_status.aicl_state = AICL_DONE;
            spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
        }
    }else{
       wake_unlock(&battery_suspend_lock);

       mt_battery_charger_detect_check();

       if (g_battery_soc_ready == KAL_FALSE) {
           if (BMT_status.nPercent_ZCV == 0)
               battery_meter_initial();

           BMT_status.SOC = battery_meter_get_battery_percentage();
       }

       if (BMT_status.bat_vol > 0) {
           mt_battery_update_status();
       }

       wake_up_bat();
    }

}

static void aicl_check_worker(struct work_struct *work)
{
    static kal_int32 index = 0;
    kal_uint32 ichr, vchrin;
    unsigned long flags;

    battery_xlog_printk(BAT_LOG_FULL, "[%s] aicl_state = %d\n", __FUNCTION__, BMT_status.aicl_state);

    switch( BMT_status.aicl_state )
    {
        case AICL_START:
            battery_xlog_printk(BAT_LOG_FULL, "[%s] AICL initial\n", __FUNCTION__);

            index = 0;
            spin_lock_irqsave(&BMT_status.aicl_lock, flags);
            if( BMT_status.aicl_state != AICL_STOP )
                BMT_status.aicl_state = AICL_LOOP;
            spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
        case AICL_LOOP:
            if( battery_charging_control(CHARGING_CMD_HTC_AICL_CHECK_STATUS, &vchrin) == 0 ){
                battery_xlog_printk(BAT_LOG_FULL,
                    "[%s] [OK] index = %d, i-chr = %d, vchrin = %d\n", __FUNCTION__, index, CHR_CUR[index]/100, vchrin);

                if( index+1 < CHR_CUR_SIZE ){
                    ichr = CHR_CUR[++index];
                    battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &ichr);
                    battery_charging_control(CHARGING_CMD_SET_CURRENT, &ichr);

                }else{
                    ichr = CHR_CUR[index];
                    spin_lock_irqsave(&BMT_status.aicl_lock, flags);
                    if( BMT_status.aicl_state != AICL_STOP )
                        BMT_status.aicl_state = AICL_DONE;
                    spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
                }
            }else{
                battery_xlog_printk(BAT_LOG_FULL,
                    "[%s] [FAILED] index = %d, i-chr = %d, vchrin = %d\n", __FUNCTION__, index, CHR_CUR[index]/100, vchrin);

                ichr = CHR_CUR[index];
                
                
                
                    index = 0;
                ichr = CHR_CUR[index];
                battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &ichr);
                battery_charging_control(CHARGING_CMD_SET_CURRENT, &ichr);

                spin_lock_irqsave(&BMT_status.aicl_lock, flags);
                if( BMT_status.aicl_state != AICL_STOP )
                    BMT_status.aicl_state = AICL_DONE;
                spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
            }

            spin_lock_irqsave(&BMT_status.aicl_lock, flags);
            if( BMT_status.aicl_state == AICL_LOOP ){
                queue_delayed_work(BMT_status.aicl_check_wq, &BMT_status.aicl_check_work, msecs_to_jiffies(AICL_CHECK_MS));
                spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
                break;
            }
            spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);

        case AICL_DONE:
            battery_xlog_printk(BAT_LOG_CRTI, "[%s] AICL done. i-chr = %dmA\n", __FUNCTION__, ichr/100);
            battery_charging_control(CHARGING_CMD_HTC_AICL_COMPLETE, NULL);
            break;

        case AICL_STOP:
        default:
            break;
    }
}
#endif

#ifdef CONFIG_HTC_LIMIT_POWER
#define LIMITED_POWER_STAGE_ONE_TEMPERATURE  0
#define LIMITED_POWER_STAGE_TWO_TEMPERATURE  -10
#define LIMITED_POWER_RELEASE_TEMPERATURE  2
#define LIMITED_POWER_STAGE_ONE_POWER_6795  1799
#define LIMITED_POWER_STAGE_ONE_POWER_6796T  1977
#define LIMITED_POWER_STAGE_TWO_POWER  668

extern int mt_cpufreq_thermal_protect(unsigned int limited_power);
static int batt_low_temp_protect_en = 0;

void htc_battery_limited_power(void)
{
	kal_int32 temperature;

	temperature = battery_meter_get_battery_temperature(FALSE);
	battery_xlog_printk(BAT_LOG_CRTI, "htc_battery_limited_power = %d degC+++\n", temperature);

	if (temperature <= LIMITED_POWER_STAGE_TWO_TEMPERATURE) {
		mt_cpufreq_thermal_protect(LIMITED_POWER_STAGE_TWO_POWER);
		batt_low_temp_protect_en = 1;
	} else if (temperature <= LIMITED_POWER_STAGE_ONE_TEMPERATURE) {
		if (mt_get_chip_info(CHIP_INFO_FUNCTION_CODE) >= 0xB) 
			mt_cpufreq_thermal_protect(LIMITED_POWER_STAGE_ONE_POWER_6796T);
		else
			mt_cpufreq_thermal_protect(LIMITED_POWER_STAGE_ONE_POWER_6795);
		batt_low_temp_protect_en = 1;
	} else if (batt_low_temp_protect_en && (temperature >= LIMITED_POWER_RELEASE_TEMPERATURE)) {
		mt_cpufreq_thermal_protect(0);
		batt_low_temp_protect_en = 0;
		battery_xlog_printk(BAT_LOG_CRTI, "release cpufreq limit\n");
	}

	battery_xlog_printk(BAT_LOG_CRTI, "htc_battery_limited_power---\n");

	return;
}
#endif

void BAT_thread(void)
{
	static kal_bool battery_meter_initilized = KAL_FALSE;
#ifdef HTC_ENABLE_AICL
    unsigned long flags;
#endif
	if (battery_meter_initilized == KAL_FALSE) {
		battery_meter_initial();	
		BMT_status.nPercent_ZCV = battery_meter_get_battery_nPercent_zcv();
		battery_meter_initilized = KAL_TRUE;
	}

	mt_battery_charger_detect_check();
#ifdef HTC_ENABLE_AICL
	if (BMT_status.charger_exist == KAL_FALSE){
        spin_lock_irqsave(&BMT_status.aicl_lock, flags);
        BMT_status.aicl_state = AICL_STOP;
        spin_unlock_irqrestore(&BMT_status.aicl_lock, flags);
        if (delayed_work_pending(&BMT_status.aicl_check_work))
            cancel_delayed_work_sync(&BMT_status.aicl_check_work);
    }
#endif

	mt_battery_GetBatteryData();
    htc_battery_check_vchr();
	if (BMT_status.charger_exist == KAL_TRUE) {
		check_battery_exist();
	}
	mt_battery_thermal_check();
	mt_battery_notify_check();

	if (BMT_status.charger_exist == KAL_TRUE) {	
		mt_battery_CheckBatteryStatus();
		mt_battery_charging_algorithm();
	}
    htc_battery_check_overload();

    htc_battery_sync_ui_soc(&battery_main);
	mt_battery_update_status();
	mt_kpoc_power_off_check();
#ifdef CONFIG_HTC_LIMIT_POWER
	htc_battery_limited_power();
#endif
}

int bat_thread_kthread(void *x)
{
	ktime_t ktime = ktime_set(3, 0);	

	
	while (1) {
		mutex_lock(&bat_mutex);
          
		if ((chargin_hw_init_done == KAL_TRUE) && (battery_suspended == KAL_FALSE))
			BAT_thread();

		mutex_unlock(&bat_mutex);

		battery_xlog_printk(BAT_LOG_FULL, "wait event \n" );

		wait_event(bat_thread_wq, (bat_thread_timeout == KAL_TRUE));

		bat_thread_timeout = KAL_FALSE;
		hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
		ktime = ktime_set(BAT_TASK_PERIOD, 0);	
		if (chr_wake_up_bat == KAL_TRUE && g_smartbook_update != 1)	
		{
			#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
			if (DISO_data.chr_get_diso_state) {
				DISO_data.chr_get_diso_state = KAL_FALSE;
				battery_charging_control(CHARGING_CMD_GET_DISO_STATE, &DISO_data);
			}
			#endif

			g_smartbook_update = 0;
			
			chr_wake_up_bat = KAL_FALSE;

			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Charger plug in/out, Call battery_meter_reset. (%d)\n",
					    BMT_status.UI_SOC);
		}

	}

	return 0;
}

void bat_thread_wakeup(void)
{
	battery_xlog_printk(BAT_LOG_FULL, "******** battery : bat_thread_wakeup  ********\n");

	bat_thread_timeout = KAL_TRUE;
	bat_meter_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
    suspend_time = 0;
#endif
    _g_bat_sleep_total_time = 0;
	wake_up(&bat_thread_wq);
}

static long adc_cali_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int *user_data_addr;
	int *naram_data_addr;
	int i = 0;
	int ret = 0;
	int adc_in_data[2] = { 1, 1 };
	int adc_out_data[2] = { 1, 1 };

	mutex_lock(&bat_mutex);

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
		g_ADC_Cali = KAL_FALSE;
		break;

	case SET_ADC_CALI_Slop:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_slop, naram_data_addr, 36);
		g_ADC_Cali = KAL_FALSE;	
		
		for (i = 0; i < 14; i++) {
			if ((*(adc_cali_slop + i) == 0) || (*(adc_cali_slop + i) == 1)) {
				*(adc_cali_slop + i) = 1000;
			}
		}
		for (i = 0; i < 14; i++)
			battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_slop[%d] = %d\n", i,
					    *(adc_cali_slop + i));
		battery_xlog_printk(BAT_LOG_FULL,
				    "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");
		break;

	case SET_ADC_CALI_Offset:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_offset, naram_data_addr, 36);
		g_ADC_Cali = KAL_FALSE;	
		for (i = 0; i < 14; i++)
			battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_offset[%d] = %d\n", i,
					    *(adc_cali_offset + i));
		battery_xlog_printk(BAT_LOG_FULL,
				    "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");
		break;

	case SET_ADC_CALI_Cal:
		naram_data_addr = (int *)arg;
		ret = copy_from_user(adc_cali_cal, naram_data_addr, 4);
		g_ADC_Cali = KAL_TRUE;
		if (adc_cali_cal[0] == 1) {
			g_ADC_Cali = KAL_TRUE;
		} else {
			g_ADC_Cali = KAL_FALSE;
		}
		for (i = 0; i < 1; i++)
			battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_cal[%d] = %d\n", i,
					    *(adc_cali_cal + i));
		battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");
		break;

	case ADC_CHANNEL_READ:
		 
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);	

		if (adc_in_data[0] == 0)	
		{
			adc_out_data[0] = battery_meter_get_VSense() * adc_in_data[1];
		} else if (adc_in_data[0] == 1)	
		{
			adc_out_data[0] = battery_meter_get_battery_voltage(KAL_TRUE) * adc_in_data[1];
		} else if (adc_in_data[0] == 3)	
		{
			adc_out_data[0] = battery_meter_get_charger_voltage() * adc_in_data[1];
			
		} else if (adc_in_data[0] == 30)	
		{
			adc_out_data[0] = battery_meter_get_battery_temperature(TRUE) * adc_in_data[1];
		} else if (adc_in_data[0] == 66) {
			adc_out_data[0] = (battery_meter_get_battery_current()) / 10;

			if (battery_meter_get_battery_current_sign() == KAL_TRUE) {
				adc_out_data[0] = 0 - adc_out_data[0];	
			}
		} else {
			battery_xlog_printk(BAT_LOG_FULL, "unknown channel(%d,%d)\n",
					    adc_in_data[0], adc_in_data[1]);
		}

		if (adc_out_data[0] < 0)
			adc_out_data[1] = 1;	
		else
			adc_out_data[1] = 0;	

		if (adc_in_data[0] == 30)
			adc_out_data[1] = 0;	

		if (adc_in_data[0] == 66)
			adc_out_data[1] = 0;	

		ret = copy_to_user(user_data_addr, adc_out_data, 8);
		battery_xlog_printk(BAT_LOG_CRTI,
				    "**** unlocked_ioctl : Channel %d * %d times = %d\n",
				    adc_in_data[0], adc_in_data[1], adc_out_data[0]);
		break;

	case BAT_STATUS_READ:
		user_data_addr = (int *)arg;
		ret = copy_from_user(battery_in_data, user_data_addr, 4);
		
		if (g_ADC_Cali) {
			battery_out_data[0] = 1;
		} else {
			battery_out_data[0] = 0;
		}
		ret = copy_to_user(user_data_addr, battery_out_data, 4);
		battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : CAL:%d\n",
				    battery_out_data[0]);
		break;

	case Set_Charger_Current:	
		user_data_addr = (int *)arg;
		ret = copy_from_user(charging_level_data, user_data_addr, 4);
		g_ftm_battery_flag = KAL_TRUE;
		if (charging_level_data[0] == 0) {
			charging_level_data[0] = CHARGE_CURRENT_70_00_MA;
		} else if (charging_level_data[0] == 1) {
			charging_level_data[0] = CHARGE_CURRENT_200_00_MA;
		} else if (charging_level_data[0] == 2) {
			charging_level_data[0] = CHARGE_CURRENT_400_00_MA;
		} else if (charging_level_data[0] == 3) {
			charging_level_data[0] = CHARGE_CURRENT_450_00_MA;
		} else if (charging_level_data[0] == 4) {
			charging_level_data[0] = CHARGE_CURRENT_550_00_MA;
		} else if (charging_level_data[0] == 5) {
			charging_level_data[0] = CHARGE_CURRENT_650_00_MA;
		} else if (charging_level_data[0] == 6) {
			charging_level_data[0] = CHARGE_CURRENT_700_00_MA;
		} else if (charging_level_data[0] == 7) {
			charging_level_data[0] = CHARGE_CURRENT_800_00_MA;
		} else if (charging_level_data[0] == 8) {
			charging_level_data[0] = CHARGE_CURRENT_900_00_MA;
		} else if (charging_level_data[0] == 9) {
			charging_level_data[0] = CHARGE_CURRENT_1000_00_MA;
		} else if (charging_level_data[0] == 10) {
			charging_level_data[0] = CHARGE_CURRENT_1100_00_MA;
		} else if (charging_level_data[0] == 11) {
			charging_level_data[0] = CHARGE_CURRENT_1200_00_MA;
		} else if (charging_level_data[0] == 12) {
			charging_level_data[0] = CHARGE_CURRENT_1300_00_MA;
		} else if (charging_level_data[0] == 13) {
			charging_level_data[0] = CHARGE_CURRENT_1400_00_MA;
		} else if (charging_level_data[0] == 14) {
			charging_level_data[0] = CHARGE_CURRENT_1500_00_MA;
		} else if (charging_level_data[0] == 15) {
			charging_level_data[0] = CHARGE_CURRENT_1600_00_MA;
		} else {
			charging_level_data[0] = CHARGE_CURRENT_450_00_MA;
		}
		wake_up_bat();
		battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : set_Charger_Current:%d\n",
				    charging_level_data[0]);
		break;
		
	case Get_META_BAT_VOL:
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);
		adc_out_data[0] = BMT_status.bat_vol;
		ret = copy_to_user(user_data_addr, adc_out_data, 8);

		break;
	case Get_META_BAT_SOC:
		user_data_addr = (int *)arg;
		ret = copy_from_user(adc_in_data, user_data_addr, 8);
		adc_out_data[0] = BMT_status.UI_SOC;
		ret = copy_to_user(user_data_addr, adc_out_data, 8);

		break;
	case Set_META_BAT_Chg:
		user_data_addr = (int *)arg;
		ret = copy_from_user(&meta_charging_enable, user_data_addr, 4);
		battery_xlog_printk(BAT_LOG_CRTI, "Recieve meta_charging_enable:%d\n",
				    meta_charging_enable);
		wake_up_bat();
		break;
		

	default :
		g_ADC_Cali = KAL_FALSE;
		break;
	}

	mutex_unlock(&bat_mutex);

	return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
	return 0;
}


static struct file_operations adc_cali_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = adc_cali_ioctl,
	.open = adc_cali_open,
	.release = adc_cali_release,
};


void check_battery_exist(void)
{
#if defined(CONFIG_DIS_CHECK_BATTERY) || defined(CONFIG_MTK_BATT_TEMP_NOT_READY)
	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] Disable check battery exist.\n");
#else
	kal_uint32 baton_count = 0;
	kal_uint32 charging_enable = KAL_FALSE;
	kal_uint32 battery_status;
	kal_uint32 i;

	for (i = 0; i < 3; i++) {
		battery_charging_control(CHARGING_CMD_GET_BATTERY_STATUS, &battery_status);
		baton_count += battery_status;

	}

	if (baton_count >= 3) {
		if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)
		    || (g_platform_boot_mode == ATE_FACTORY_BOOT) || (g_by_pass_batt_exist_check == KAL_TRUE)) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[BATTERY] boot mode = %d, bypass battery check\n",
					    g_platform_boot_mode);
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery is not exist, power off FAN5405 and system (%d)\n",
					    baton_count);

			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}


int charger_hv_detect_sw_thread_handler(void *unused)
{
	ktime_t ktime;
	kal_uint32 charging_enable;
	kal_uint32 hv_voltage = V_CHARGER_MAX*1000;
	kal_bool hv_status;
	kal_bool charging_up = KAL_FALSE;
	kal_int32 bat_curr = 0, bat_vol = 0;
	static int full_check = 0;

	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	hv_voltage = DISO_data.hv_voltage;
	#endif

	do {
		ktime = ktime_set(0, BAT_MS_TO_NS(1000));

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_SET_HV_THRESHOLD, &hv_voltage);

		wait_event_interruptible(charger_hv_detect_waiter,
					 (charger_hv_detect_flag == KAL_TRUE));

		if ((upmu_is_chr_det() == KAL_TRUE)) {
			check_battery_exist();
			bat_curr = htc_battery_meter_get_battery_current_imm(TRUE);
			bat_vol = battery_meter_get_battery_voltage(KAL_TRUE);
			g_force_reverse_boost_wa = KAL_FALSE;
			battery_xlog_printk(BAT_LOG_FULL,
					    "[charger_hv_detect_sw_thread_handler] bat_vol=%d, bat_curr=%d\n",bat_vol,bat_curr);
			if ((bat_vol > 4000) && (bat_curr < (-150)) && (htc_battery_CharegerControlCheck() == PMU_STATUS_OK)) {
				g_force_reverse_boost_wa = KAL_TRUE;
			}
			if ((bat_curr < 700) && (bat_curr > 0) && !BMT_status.is_warm && (bat_vol > 4330)) {
				full_check++;
			} else {
				full_check = 0;
				g_batt_curr_full = KAL_FALSE;
			}
			if (full_check > 30) {
				full_check = 0;
				g_batt_curr_full = KAL_TRUE;
			}
		}

		charger_hv_detect_flag = KAL_FALSE;

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_GET_HV_STATUS, &hv_status);

		if (hv_status == KAL_TRUE) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[charger_hv_detect_sw_thread_handler] charger hv\n");

			charging_enable = KAL_FALSE;
			if (chargin_hw_init_done)
				battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		} else {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");
		}

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);
#ifdef CONFIG_MTK_SWCHR_SUPPORT
		battery_charging_control(CHARGING_CMD_CURRENT_LEVEL_UP, &charging_up);
#endif
		hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
	charger_hv_detect_flag = KAL_TRUE;
	wake_up_interruptible(&charger_hv_detect_waiter);

	battery_xlog_printk(BAT_LOG_FULL, "[charger_hv_detect_sw_workaround]\n");

	return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(0, BAT_MS_TO_NS(2000));
	hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;
	hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	charger_hv_detect_thread =
	    kthread_run(charger_hv_detect_sw_thread_handler, 0,
			"mtk charger_hv_detect_sw_workaround");
	if (IS_ERR(charger_hv_detect_thread)) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[%s]: failed to create charger_hv_detect_sw_workaround thread\n",
				    __func__);
	}
	check_battery_exist();
	battery_xlog_printk(BAT_LOG_CRTI, "charger_hv_detect_sw_workaround_init : done\n");
}

void htc_battery_para_init(void)
{
	BMT_status.test_power_monitor =
		(get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY) ? KAL_TRUE : KAL_FALSE;

	BMT_status.keep_charger_on =
		(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ? KAL_TRUE  : KAL_FALSE;

	BMT_status.force_ac_charger =
		(get_kernel_flag() &KERNEL_FLAG_ENABLE_FAST_CHARGE) ? KAL_TRUE : KAL_FALSE;

	BMT_status.flag_enable_bms_charger_log =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? KAL_TRUE : KAL_FALSE;

	BMT_status.flag_disable_safety_timer=
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER) ? KAL_TRUE : KAL_FALSE;

	BMT_status.flag_disable_temp_protection=
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_TBATT_PROTECT) ? KAL_TRUE : KAL_FALSE;

	BMT_status.flag_pa_fake_batt_temp=
		(get_kernel_flag() & KERNEL_FLAG_FOR_PA_TEST) ? KAL_TRUE : KAL_FALSE;
}

static ssize_t htc_battery_show_batt_attr(struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
	int res, len = 0;
    char *cChr_type[] = {"CHARGER_UNKNOWN", "STANDARD_HOST", "CHARGING_HOST", "NONSTANDARD_CHARGER",
        "STANDARD_CHARGER", "APPLE_2_1A_CHARGER", "APPLE_1_0A_CHARGER", "APPLE_0_5A_CHARGER", "WIRELESS_CHARGER"};
    char *cBat_chr_state[] = {"CHR_PRE", "CHR_CC", "CHR_TOP_OFF",
                "CHR_POST_FULL", "CHR_BATFULL", "CHR_ERROR", "CHR_HOLD"};
    sCHR_REG_DUMP dump_reg;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"percentage:\t%d;\n"
			"percentage_raw:\t%d;\n"
			"batt_id:\t%d;\n"
			"batt_full:\t%d;\n"
			"batt_in_recharging_state:\t%d;\n"
			"overload:\t%d;\n"
			"batt_vol:\t%d;\n"
			"batt_curr:\t%d;\n"
			"batt_temp:\t%d;\n",
			BMT_status.UI_SOC,
			BMT_status.SOC,
			fgauge_get_battery_id(),
			BMT_status.bat_full,
			BMT_status.bat_in_recharging_state,
			BMT_status.is_overload,
			htc_battery_meter_get_battery_voltage_imm(1),
			htc_battery_meter_get_battery_current_imm(TRUE),
			htc_battery_meter_get_battery_temperature_imm(1)
		);

    len += htc_battery_meter_show_attr(buf + len, PAGE_SIZE - len);

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"charging_source:\t%s;\n"
			"batt_charging_state:\t%s;\n"
			"charger_vol:\t%d;\n"
			"charging_curr:\t%d;\n"
			"total_charging_time:\t%d;\n"
			"pre_charging_time:\t%d;\n"
			"cc_charging_time:\t%d;\n"
			"topoff_charging_time:\t%d;\n"
			"postfull_charging_time:\t%d;\n",
			cChr_type[BMT_status.charger_type < 9? BMT_status.charger_type : CHARGER_UNKNOWN],
			cBat_chr_state[(BMT_status.bat_charging_state & 0x7) < 7? (BMT_status.bat_charging_state & 0x7) : (CHR_ERROR & 0x7)],
			battery_meter_get_charger_voltage(),
			battery_meter_get_charging_current(),
			BMT_status.total_charging_time,
			BMT_status.PRE_charging_time,
			BMT_status.CC_charging_time,
			BMT_status.TOPOFF_charging_time,
			BMT_status.POSTFULL_charging_time
		);

    dump_reg.buf = buf + len;
    dump_reg.size = PAGE_SIZE - len;
    res = battery_charging_control(CHARGING_CMD_HTC_DUMP_REGISTER, (void*)&dump_reg);
    len += dump_reg.size;

	return len;
}

static ssize_t htc_battery_set_full_level(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int rc = 0;
	unsigned long percent = 100;

	rc = strict_strtoul(buf, 10, &percent);
	if (rc)
		return rc;

	if (percent > 100 || percent == 0)
		return -EINVAL;

	battery_xlog_printk(BAT_LOG_CRTI, "htc_battery_set_full_level\n");
	BMT_status.full_level = (int)percent;

    return count;
}

static ssize_t htc_battery_charger_stat(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", charger_ctrl_stat);

    return i;
}

void charger_switch_internal(int enable)
{
	kal_bool charging_enable;

	battery_xlog_printk(BAT_LOG_CRTI, "Set charger_switch_internal:%d", enable);

	if (enable >= END_CHARGER_INTERNAL)
		return;

	if (enable == DISABLE_PWRSRC_FINGERPRINT) {
		charging_enable = KAL_FALSE;
	} else if (enable == ENABLE_PWRSRC_FINGERPRINT) {
		charging_enable = KAL_TRUE;
	}

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
	g_charger_ctrl_internal = enable;
}

static ssize_t htc_battery_charger_switch(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	unsigned long enable = 0;
	kal_uint32 charging_enable;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	battery_xlog_printk(BAT_LOG_CRTI, "Set charger_control:%lu", enable);
	if (enable >= END_CHARGER)
		return -EINVAL;

	
	if (enable == STOP_CHARGER || enable == DISABLE_PWRSRC) {
		charging_enable = KAL_FALSE;
	} else if (enable == ENABLE_CHARGER || enable == ENABLE_PWRSRC) {
		charging_enable = KAL_TRUE;
	}

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
	charger_ctrl_stat = enable;
    return count;
}

static ssize_t htc_battery_set_phone_call(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	unsigned long enable = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	battery_xlog_printk(BAT_LOG_CRTI, "htc_battery_set_phone_call: %lu\n", enable);

	if (enable)
		g_chg_limit_reason |= HTC_BATT_CHG_LIMIT_BIT_TALK;
	else
		g_chg_limit_reason &= ~HTC_BATT_CHG_LIMIT_BIT_TALK;

    return count;
}

static ssize_t htc_battery_set_play_music(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	battery_xlog_printk(BAT_LOG_CRTI, "htc_battery_set_play_music\n");
    return count;
}

static ssize_t htc_battery_rt_vol(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", battery_meter_get_battery_voltage(KAL_TRUE));

    return i;
}

static ssize_t htc_battery_rt_current(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", (htc_battery_meter_get_battery_current_imm(TRUE)/10));

    return i;
}

static ssize_t htc_battery_rt_temp(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", battery_meter_get_battery_temperature(TRUE));

    return i;
}

static ssize_t htc_battery_rt_id(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", fgauge_get_battery_id());

    return i;
}

static ssize_t htc_battery_rt_id_mv(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", fgauge_get_battery_id_mv());

    return i;
}

static ssize_t htc_battery_ftm_charger_stat(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", g_ftm_charger_ctrl_stat);

    return i;
}

static ssize_t htc_battery_ftm_charger_switch(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	unsigned long enable = 0;
	int rc = 0;

	rc = strict_strtoul(buf, 10, &enable);
	if (rc)
		return rc;

	battery_xlog_printk(BAT_LOG_CRTI, "Set ftm_charger_control:%lu", enable);
	if (enable >= FTM_END_CHARGER)
		return -EINVAL;

	g_ftm_charger_ctrl_stat = enable;
	wake_up_bat();

    return count;
}

static int _ovp_det_gpio = 90;
static ssize_t htc_battery_over_vchg(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int i = 0, is_ovp = 0;

		if(of_machine_hwid() < 2){
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", is_ovp);
			return i;
		}

	int value = __gpio_get_value(_ovp_det_gpio);

	if(!value)
		is_ovp = 1;
	else {
		
		if (BMT_status.charger_protect_status == charger_OVER_VOL) {
			BMT_status.charger_protect_status = 0;
			BMT_status.bat_charging_state = CHR_PRE;
		}
	}

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", is_ovp);

    return i;
}

static ssize_t htc_battery_overload(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", BMT_status.is_overload);

    return i;
}

static ssize_t htc_battery_show_htc_extension_attr(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i,"%d\n", BMT_status.htc_extension);

	return i;
}

static ssize_t htc_battery_show_cc_attr(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "cc:%d\n", htc_battery_meter_get_car());

	return i;
}

static ssize_t htc_battery_show_capacity_raw(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", BMT_status.SOC);

	return i;
}

static ssize_t htc_battery_show_gap(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", BMT_status.gap);

	return i;
}

static struct device_attribute htc_battery_attrs[] = {
	__ATTR(batt_attr_text, S_IRUGO, htc_battery_show_batt_attr, NULL),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL, htc_battery_set_full_level),
	__ATTR(charger_control, S_IWUSR | S_IWGRP, htc_battery_charger_stat, htc_battery_charger_switch),
	__ATTR(phone_call, S_IWUSR | S_IWGRP, NULL, htc_battery_set_phone_call),
	__ATTR(play_music, S_IWUSR | S_IWGRP, NULL, htc_battery_set_play_music),
	__ATTR(batt_vol_now, S_IRUGO, htc_battery_rt_vol, NULL),
	__ATTR(batt_current_now, S_IRUGO, htc_battery_rt_current, NULL),
	__ATTR(batt_temp_now, S_IRUGO, htc_battery_rt_temp, NULL),
	__ATTR(batt_id, S_IRUGO, htc_battery_rt_id, NULL),
	__ATTR(batt_id_now, S_IRUGO, htc_battery_rt_id_mv, NULL),
	__ATTR(ftm_charger_control, S_IWUSR | S_IWGRP | S_IRUGO, htc_battery_ftm_charger_stat,
		htc_battery_ftm_charger_switch),
	__ATTR(over_vchg, S_IRUGO, htc_battery_over_vchg, NULL),
	__ATTR(overload, S_IRUGO, htc_battery_overload, NULL),
	__ATTR(htc_extension, S_IRUGO, htc_battery_show_htc_extension_attr, NULL),
	__ATTR(batt_power_meter, S_IRUGO, htc_battery_show_cc_attr, NULL),
	__ATTR(capacity_raw, S_IRUGO, htc_battery_show_capacity_raw, NULL),
	__ATTR(capacity_gap, S_IRUGO, htc_battery_show_gap, NULL),
};

static int htc_battery_create_attrs(struct device *dev)
{
    int i = 0, rc = 0;

    for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
        rc = device_create_file(dev, &htc_battery_attrs[i]);
        if (rc)
            goto htc_attrs_failed;
    }

    goto succeed;

htc_attrs_failed:
    while (i--)
        device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
    return rc;
}


enum hrtimer_restart battery_kthread_hrtimer_func(struct hrtimer *timer)
{
	bat_thread_wakeup();

	return HRTIMER_NORESTART;
}

void battery_kthread_hrtimer_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(1, 0);	
	hrtimer_init(&battery_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	battery_kthread_timer.function = battery_kthread_hrtimer_func;
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);

	battery_xlog_printk(BAT_LOG_CRTI, "battery_kthread_hrtimer_init : done\n");
}


static void get_charging_control(void)
{
	battery_charging_control = chr_control_interface;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
static irqreturn_t diso_auxadc_irq_thread(int irq, void *dev_id)
{
	int pre_diso_state = (DISO_data.diso_state.pre_otg_state |
		(DISO_data.diso_state.pre_vusb_state << 1) |
		(DISO_data.diso_state.pre_vdc_state << 2)) & 0x7;

	battery_xlog_printk(BAT_LOG_CRTI,
			    "[DISO]auxadc IRQ threaded handler triggered, pre_diso_state is %s\n",
			    DISO_state_s[pre_diso_state]);

	switch (pre_diso_state) {
	#ifdef MTK_DISCRETE_SWITCH 
	case USB_ONLY:
	#endif
	case OTG_ONLY:
		BMT_status.charger_exist = KAL_TRUE;
		wake_lock(&battery_suspend_lock);
		wake_up_bat();
		break;
	case DC_WITH_OTG:
		BMT_status.charger_exist = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE,&BMT_status.charger_exist); 
		BMT_status.charger_exist = KAL_FALSE; 
		BMT_status.charger_type = CHARGER_UNKNOWN;
		wake_unlock(&battery_suspend_lock);
		wake_up_bat();
		break;
	case DC_WITH_USB:
		
		if((BMT_status.charger_type==STANDARD_HOST) || (BMT_status.charger_type==CHARGING_HOST))
			mt_usb_disconnect(); 
		BMT_status.charger_type = CHARGER_UNKNOWN;
		wake_up_bat();
		break;
	case DC_ONLY:
		BMT_status.charger_type = CHARGER_UNKNOWN;
		mt_battery_charger_detect_check(); 
		break;
	default:
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[DISO]VUSB auxadc threaded handler triggered ERROR OR TEST\n");
		break;
	}
	return IRQ_HANDLED;
}

static void dual_input_init(void)
{
	DISO_data.irq_callback_func = diso_auxadc_irq_thread;
	battery_charging_control(CHARGING_CMD_DISO_INIT, &DISO_data);
}
#endif

static int _ovp_irq = -1;
static irqreturn_t ovp_det_handler(int irq, void *dev)
{
	int value = __gpio_get_value(_ovp_det_gpio);

	if(!value){
		BMT_status.charger_protect_status = charger_OVER_VOL;
		BMT_status.bat_charging_state = CHR_ERROR;
		irq_set_irq_type(_ovp_irq, IRQF_TRIGGER_RISING);
	}else {
		BMT_status.charger_protect_status = 0;
		BMT_status.bat_charging_state = CHR_PRE;
		irq_set_irq_type(_ovp_irq, IRQF_TRIGGER_FALLING);
	}
	wake_up_bat();
	battery_xlog_printk(BAT_LOG_CRTI, "%s, gpio(%d) is %s\n",
		__func__, _ovp_det_gpio, value?"high":"low");
	return IRQ_HANDLED;
}
static int htc_ovp_det_init(struct platform_device *dev)
{
	struct device_node *eint_node;
	const char * eint_str = "mediatek, OVP_DET-eint";
	int rc = -1;

	if(of_machine_hwid() < 2){
		battery_xlog_printk(BAT_LOG_CRTI, "XA/XB not support OVP.\n");
		return 0;
	}

	if(gpio_is_valid(_ovp_det_gpio)) {
		rc = gpio_request_one(_ovp_det_gpio, GPIOF_IN, "ovp_det");
		if(unlikely(rc)){
			battery_xlog_printk(BAT_LOG_CRTI, "%s, Failed to request GPIO %d, error %d\n", __func__, _ovp_det_gpio, rc);
			return rc;
		}

		eint_node = of_find_compatible_node(NULL, NULL, eint_str);
		if(eint_node){
			_ovp_irq = irq_of_parse_and_map(eint_node, 0);
		} else {
			battery_xlog_printk(BAT_LOG_CRTI, "eint_node is NULL.\n");
			_ovp_irq = -EINVAL;
		}
		battery_xlog_printk(BAT_LOG_CRTI, "%s, gpio(%d), irq(%d)\n", __func__, _ovp_det_gpio, _ovp_irq);

		irq_set_irq_type(_ovp_irq, IRQF_TRIGGER_FALLING);
		irq_set_chained_handler(_ovp_irq, ovp_det_handler);

	}
	else battery_xlog_printk(BAT_LOG_CRTI, "%s, gpio (%d) id invalid.\n", __func__, _ovp_det_gpio);

	return 0;
fail:
	gpio_free(_ovp_det_gpio);
	return rc;
}

static void htc_ovp_det_uninit(void)
{
	if(gpio_is_valid(_ovp_det_gpio))
		gpio_free(_ovp_det_gpio);
	free_irq(_ovp_irq, "ovp_det");
}

static int battery_probe(struct platform_device *dev)
{
	struct class_device *class_dev = NULL;
	int ret = 0;

	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver probe!! ********\n");

	
	ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
	if (ret)
		battery_xlog_printk(BAT_LOG_CRTI, "Error: Can't Get Major number for adc_cali\n");
	adc_cali_cdev = cdev_alloc();
	adc_cali_cdev->owner = THIS_MODULE;
	adc_cali_cdev->ops = &adc_cali_fops;
	ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
	if (ret)
		battery_xlog_printk(BAT_LOG_CRTI, "adc_cali Error: cdev_add\n");
	adc_cali_major = MAJOR(adc_cali_devno);
	adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
	class_dev = (struct class_device *)device_create(adc_cali_class,
							 NULL,
							 adc_cali_devno, NULL, ADC_CALI_DEVNAME);
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] adc_cali prepare : done !!\n ");

	get_charging_control();

	battery_charging_control(CHARGING_CMD_GET_PLATFORM_BOOT_MODE, &g_platform_boot_mode);
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] g_platform_boot_mode = %d\n ",
			    g_platform_boot_mode);

	wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");
	#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	wake_lock_init(&TA_charger_suspend_lock, WAKE_LOCK_SUSPEND, "TA charger suspend wakelock");  
	#endif

	
	ret = power_supply_register(&(dev->dev), &ac_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Success !!\n");

	ret = power_supply_register(&(dev->dev), &usb_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register USB Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Success !!\n");

	ret = power_supply_register(&(dev->dev), &wireless_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register WIRELESS Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI,
			    "[BAT_probe] power_supply_register WIRELESS Success !!\n");

	ret = power_supply_register(&(dev->dev), &battery_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register Battery Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Success !!\n");

#if !defined(CONFIG_POWER_EXT)

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power()) {
		battery_main.BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		battery_main.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
		battery_main.BAT_PRESENT = 1;
		battery_main.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
		battery_main.BAT_CAPACITY = 100;
		battery_main.BAT_batt_vol = 4200;
		battery_main.BAT_batt_temp = 220;

		g_bat_init_flag = KAL_TRUE;
		return 0;
	}
#endif
	
	{
		int ret_device_file = 0;

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Charger_Voltage);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Slope);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Slope);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Offset);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Offset);

		ret_device_file =
		    device_create_file(&(dev->dev), &dev_attr_ADC_Channel_Is_Calibration);

		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_On_Voltage);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_Off_Voltage);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_TopOff_Value);

		ret_device_file =
		    device_create_file(&(dev->dev), &dev_attr_FG_Battery_CurrentConsumption);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_SW_CoulombCounter);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charging_CallState);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_Type);
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Pump_Express);
#endif
	}

	

	
	BMT_status.bat_exist = KAL_TRUE;	
	BMT_status.charger_exist = KAL_FALSE;	
	BMT_status.bat_vol = 0;
	BMT_status.ICharging = 0;
	BMT_status.temperature = 0;
	BMT_status.charger_vol = 0;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.SOC = -1;
	BMT_status.UI_SOC = 0;
	BMT_status.full_level = 100;

	BMT_status.bat_charging_state = CHR_PRE;
	BMT_status.bat_in_recharging_state = KAL_FALSE;
	BMT_status.bat_full = KAL_FALSE;
	BMT_status.nPercent_ZCV = 0;
	BMT_status.nPrecent_UI_SOC_check_point = battery_meter_get_battery_nPercent_UI_SOC();
#ifdef HTC_ENABLE_AICL
    BMT_status.aicl_state = AICL_STOP;
    spin_lock_init(&BMT_status.aicl_lock);
#endif

	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	dual_input_init();
	#endif

	
	
	battery_kthread_hrtimer_init();

	kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread");
	battery_xlog_printk(BAT_LOG_CRTI, "[battery_probe] bat_thread_kthread Done\n");

	charger_hv_detect_sw_workaround_init();
	
	init_proc_log();

	
#ifdef HTC_ENABLE_AICL
    INIT_DELAYED_WORK(&BMT_status.vin_collapse_check_work, vin_collapse_check_worker);
    INIT_DELAYED_WORK(&BMT_status.aicl_check_work, aicl_check_worker);
    BMT_status.aicl_check_wq = create_singlethread_workqueue("aicl_check_wq");
#endif

	if (BMT_status.force_ac_charger)
		set_usb_current_unlimited(true);

	
	htc_battery_create_attrs(battery_main.psy.dev);
	

#else
	
	charger_hv_detect_sw_workaround_init();
#endif
	g_bat_init_flag = KAL_TRUE;
	htc_ovp_det_init(dev);

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
    if (g_vcdt_irq_delay_flag == KAL_TRUE)
        do_chrdet_int_task();
#endif

	return 0;

}

static void battery_timer_pause(void)
{
	struct timespec xts, tom;

    
#ifdef CONFIG_POWER_EXT
#else

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif
	mutex_lock(&bat_mutex);
	
	hrtimer_cancel(&battery_kthread_timer);
	hrtimer_cancel(&charger_hv_detect_timer);

	battery_suspended = KAL_TRUE;
	mutex_unlock(&bat_mutex);

	battery_xlog_printk(BAT_LOG_CRTI, "@bs=1@\n" );
#endif

    get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &g_bat_time_before_sleep);
}

static void battery_timer_resume(void)
{
#ifdef CONFIG_POWER_EXT
#else
	kal_bool is_pcm_timer_trigger = KAL_FALSE;
	struct timespec xts, tom, bat_time_after_sleep;
    ktime_t ktime, hvtime;

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif

    ktime = ktime_set(BAT_TASK_PERIOD, 0);  
    hvtime = ktime_set(0, BAT_MS_TO_NS(2000));

	get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &bat_time_after_sleep);
	battery_charging_control(CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER,&is_pcm_timer_trigger);

	if(is_pcm_timer_trigger == KAL_TRUE || bat_spm_timeout)
	{	
		mutex_lock(&bat_mutex);
		BAT_thread();
		mutex_unlock(&bat_mutex);
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "battery resume NOT by pcm timer!!\n" );
	}
#if 0 
	if(g_call_state == CALL_ACTIVE && (bat_time_after_sleep.tv_sec - g_bat_time_before_sleep.tv_sec >= TALKING_SYNC_TIME))	
	{
		BMT_status.UI_SOC = battery_meter_get_battery_percentage();
		battery_xlog_printk(BAT_LOG_CRTI, "Sync UI SOC to SOC immediately\n" );
	}	
#endif
	mutex_lock(&bat_mutex);
    
	
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
	hrtimer_start(&charger_hv_detect_timer, hvtime, HRTIMER_MODE_REL);
        
	battery_suspended = KAL_FALSE;
	battery_xlog_printk(BAT_LOG_CRTI, "@bs=0@\n");
	mutex_unlock(&bat_mutex);
	
#endif
}

static int battery_remove(struct platform_device *dev)
{
	htc_ovp_det_uninit();
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver remove!! ********\n");

#ifdef HTC_ENABLE_AICL
    cancel_delayed_work_sync(&BMT_status.vin_collapse_check_work);
    cancel_delayed_work_sync(&BMT_status.aicl_check_work);
    destroy_workqueue(BMT_status.aicl_check_wq);
    BMT_status.aicl_check_wq = NULL;
#endif
	return 0;
}

static void battery_shutdown(struct platform_device *dev)
{
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver shutdown!! ********\n");

}

static ssize_t show_BatteryNotify(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_xlog_printk(BAT_LOG_FULL, "[Battery] show_BatteryNotify : %x\n",
			    g_BatteryNotifyCode);

	return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}

static ssize_t store_BatteryNotify(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_BatteryNotifyCode = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BatteryNotify\n");
	if (buf != NULL && size != 0) {
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf,
				    size);
		reg_BatteryNotifyCode = simple_strtoul(buf, &pvalue, 16);
		g_BatteryNotifyCode = reg_BatteryNotifyCode;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store code : %x\n",
				    g_BatteryNotifyCode);
	}
	return size;
}

static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
	return sprintf(buf, "%u\n", g_BN_TestMode);
}

static ssize_t store_BN_TestMode(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_BN_TestMode = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BN_TestMode\n");
	if (buf != NULL && size != 0) {
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf,
				    size);
		reg_BN_TestMode = simple_strtoul(buf, &pvalue, 16);
		g_BN_TestMode = reg_BN_TestMode;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store g_BN_TestMode : %x\n",
				    g_BN_TestMode);
	}
	return size;
}

static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);


#if 0
static int battery_cmd_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;
	char *p = buf;

	p += sprintf(p,
		     "g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
		     g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode,
		     battery_cmd_thermal_test_mode_value);

	*start = buf + off;

	len = p - buf;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len : count;
}
#endif

static ssize_t battery_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0, bat_tt_enable = 0, bat_thr_test_mode = 0, bat_thr_test_value = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3) {
		g_battery_thermal_throttling_flag = bat_tt_enable;
		battery_cmd_thermal_test_mode = bat_thr_test_mode;
		battery_cmd_thermal_test_mode_value = bat_thr_test_value;

		battery_xlog_printk(BAT_LOG_CRTI,
				    "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
				    g_battery_thermal_throttling_flag,
				    battery_cmd_thermal_test_mode,
				    battery_cmd_thermal_test_mode_value);

		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
	}

	return -EINVAL;
}

static int proc_utilization_show(struct seq_file *m, void *v)
{
	seq_printf(m,
		   "=> g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
		   g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode,
		   battery_cmd_thermal_test_mode_value);

	seq_printf(m, "=> get_usb_current_unlimited=%d,\ncmd_discharging = %d\n",
		   get_usb_current_unlimited(), cmd_discharging);
	return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations battery_cmd_proc_fops = {
	.open = proc_utilization_open,
	.read = seq_read,
	.write = battery_cmd_write,
};

static ssize_t current_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32];
	int cmd_current_unlimited = false;
	U32 charging_enable = false;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &cmd_current_unlimited, &cmd_discharging) == 2) {
		set_usb_current_unlimited(cmd_current_unlimited);
		if (cmd_discharging == 1) {
			charging_enable = false;
			adjust_power = -1;
		} else if (cmd_discharging == 0) {
			charging_enable = true;
			adjust_power = -1;
		}
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

		battery_xlog_printk(BAT_LOG_CRTI,
				    "[current_cmd_write] cmd_current_unlimited=%d, cmd_discharging=%d\n",
				    cmd_current_unlimited, cmd_discharging);
		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	}

	return -EINVAL;
}

static int current_cmd_read(struct seq_file *m, void *v)
{
	U32 charging_enable = false;

	cmd_discharging = 1;
	charging_enable = false;
	adjust_power = -1;

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_xlog_printk(BAT_LOG_CRTI,
			    "[current_cmd_write] cmd_discharging=%d\n", cmd_discharging);

	return 0;
}

static int proc_utilization_open_cur_stop(struct inode *inode, struct file *file)
{
	return single_open(file, current_cmd_read, NULL);
}
static ssize_t discharging_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32];
	U32 charging_enable = false;
    
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';
    
	if (sscanf(desc, "%d %d", &charging_enable, &adjust_power) == 2) {
		battery_xlog_printk(BAT_LOG_CRTI, "[current_cmd_write] adjust_power = %d\n", adjust_power);
		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	}

    	return -EINVAL;
}

static const struct file_operations discharging_cmd_proc_fops = { 
	.open  = proc_utilization_open, 
	.read  = seq_read,
	.write = discharging_cmd_write,
};

static const struct file_operations current_cmd_proc_fops = {
	.open = proc_utilization_open_cur_stop,
	.read = seq_read,
	.write = current_cmd_write,
};

static int mt_batteryNotify_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
	
	struct proc_dir_entry *battery_dir = NULL;

	battery_xlog_printk(BAT_LOG_CRTI, "******** mt_batteryNotify_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __func__);
	} else {
#if 1
		proc_create("battery_cmd", S_IRUGO | S_IWUSR, battery_dir, &battery_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create battery_cmd_proc_fops\n");

		proc_create("current_cmd", S_IRUGO | S_IWUSR, battery_dir, &current_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create current_cmd_proc_fops\n");
		proc_create("discharging_cmd", S_IRUGO | S_IWUSR, battery_dir, &discharging_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create discharging_cmd_proc_fops\n");
            

#else
		entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
		if (entry) {
			entry->read_proc = battery_cmd_read;
			entry->write_proc = battery_cmd_write;
		}
#endif
	}

	battery_xlog_printk(BAT_LOG_CRTI, "******** mtk_battery_cmd!! ********\n");

	return 0;

}
#ifdef CONFIG_OF
static const struct of_device_id mt_battery_of_match[] = {
	{ .compatible = "mediatek,battery", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_battery_of_match);
#endif

static int battery_pm_suspend(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_resume(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_freeze(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore_noirq(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void htc_batt_early_suspend(struct early_suspend *h)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[%s]\n", __FUNCTION__);
	g_display_on = KAL_FALSE;
	return;
}

static void htc_batt_late_resume(struct early_suspend *h)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[%s]\n", __FUNCTION__);
	g_display_on = KAL_TRUE;
#ifdef CONFIG_HTC_LIMIT_POWER
	htc_battery_limited_power();
#endif
	return;
}
#endif

struct dev_pm_ops battery_pm_ops = {
	.suspend = battery_pm_suspend,
	.resume = battery_pm_resume,
	.freeze = battery_pm_freeze,
	.thaw = battery_pm_restore,
	.restore = battery_pm_restore,
	.restore_noirq = battery_pm_restore_noirq,
};

#if defined(CONFIG_OF) || defined(BATTERY_MODULE_INIT)
struct platform_device battery_device = {
    .name   = "battery",
    .id        = -1,
};
#endif

static struct platform_driver battery_driver = {
	.probe = battery_probe,
	.remove = battery_remove,
	.shutdown = battery_shutdown,
	.driver = {
		.name = "battery",
		.pm = &battery_pm_ops,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend htc_batt_early_suspend_handler =
{
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = htc_batt_early_suspend,
	.resume = htc_batt_late_resume,
};
#endif

#ifdef CONFIG_OF
static int battery_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery_dts_probe!! ********\n");

	battery_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&battery_device);
    if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_dts_probe] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}

static struct platform_driver battery_dts_driver = {
	.probe = battery_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "battery-dts",
        #ifdef CONFIG_OF 
 		.of_match_table = mt_battery_of_match,
        #endif
	},
};


static const struct of_device_id mt_bat_notify_of_match[] = {
	{ .compatible = "mediatek,bat_notify", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_bat_notify_of_match);
#endif

struct platform_device MT_batteryNotify_device = {
	.name = "mt-battery",
	.id = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
	.probe = mt_batteryNotify_probe,
	.driver = {
		   .name = "mt-battery",
	},
};

#ifdef CONFIG_OF
static int mt_batteryNotify_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	
	struct proc_dir_entry *battery_dir = NULL;

	battery_xlog_printk(BAT_LOG_CRTI, "******** mt_batteryNotify_dts_probe!! ********\n");

	MT_batteryNotify_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&MT_batteryNotify_device);
    if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify_dts] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}


static struct platform_driver mt_batteryNotify_dts_driver = {
	.probe = mt_batteryNotify_dts_probe,
	.driver = {
		   .name = "mt-dts-battery",
        #ifdef CONFIG_OF
		.of_match_table = mt_bat_notify_of_match,    
        #endif
	},
};
#endif

static int battery_pm_event(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch(pm_event) {
	case PM_HIBERNATION_PREPARE: 
	case PM_RESTORE_PREPARE: 
	case PM_SUSPEND_PREPARE: 
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_pause();
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION: 
	case PM_POST_SUSPEND: 
	case PM_POST_RESTORE: 
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block battery_pm_notifier_block = {
    .notifier_call = battery_pm_event,
    .priority = 0,
};

static int __init battery_init(void)
{
	int ret;

	printk("battery_init\n");

	if (BMT_status.flag_enable_bms_charger_log)
		Enable_BATDRV_LOG = BAT_LOG_FULL;

#ifdef CONFIG_OF
	
#else
    
#ifdef BATTERY_MODULE_INIT
	ret = platform_device_register(&battery_device);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_device] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
#endif

	ret = platform_driver_register(&battery_driver);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_driver] Unable to register driver (%d)\n", ret);
		return ret;
	}
	
#ifdef CONFIG_OF
    
#else
	ret = platform_device_register(&MT_batteryNotify_device);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
	ret = platform_driver_register(&mt_batteryNotify_driver);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
		return ret;
	}
#ifdef CONFIG_OF
	ret = platform_driver_register(&battery_dts_driver);
	ret = platform_driver_register(&mt_batteryNotify_dts_driver);
#endif	
	ret = register_pm_notifier(&battery_pm_notifier_block);
	if (ret)
		printk("[%s] failed to register PM notifier %d\n", __func__, ret);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&htc_batt_early_suspend_handler);
#endif

	battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Initialization : DONE !!\n");
	return 0;
}

#ifdef BATTERY_MODULE_INIT
late_initcall(battery_init);
#else
static void __exit battery_exit(void)
{
}
module_init(battery_init);
module_exit(battery_exit);
#endif

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");
