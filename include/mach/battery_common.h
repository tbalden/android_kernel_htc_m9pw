#ifndef BATTERY_COMMON_H
#define BATTERY_COMMON_H

#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/mt_typedefs.h>
#include "charging.h"


#define PRE_CHARGE_VOLTAGE                  3200
#define SYSTEM_OFF_VOLTAGE                  3400
#define CONSTANT_CURRENT_CHARGE_VOLTAGE     4100
#define CONSTANT_VOLTAGE_CHARGE_VOLTAGE     4200
#define CV_DROPDOWN_VOLTAGE                 4000
#define CHARGER_THRESH_HOLD                 4300
#define BATTERY_UVLO_VOLTAGE                2700

#define MAX_CHARGING_TIME                   16*60*60	

#define MAX_POSTFULL_SAFETY_TIME		1*30*60	
#define MAX_PreCC_CHARGING_TIME		1*30*60	

#define MAX_CV_CHARGING_TIME			3*60*60	


#define MUTEX_TIMEOUT                       5000
#define BAT_TASK_PERIOD                     10	
#define g_free_bat_temp					1000	

#ifdef HTC_ENABLE_AICL
#define VIN_MIN_COLLAPSE_CHECK_MS  400
#define AICL_CHECK_MS              1500
#endif

#define Battery_Percent_100    100
#define charger_OVER_VOL	    1
#define BATTERY_UNDER_VOL		2
#define BATTERY_OVER_TEMP		3
#define ADC_SAMPLE_TIMES        5

#define  CHR_PRE                        0x1000
#define  CHR_CC                         0x1001
#define  CHR_TOP_OFF                    0x1002
#define  CHR_POST_FULL                  0x1003
#define  CHR_BATFULL                    0x1004
#define  CHR_ERROR                      0x1005
#define  CHR_HOLD						0x1006

#define CALL_IDLE 0
#define CALL_ACTIVE 1

typedef unsigned int WORD;


typedef enum {
	PMU_STATUS_OK = 0,
	PMU_STATUS_FAIL = 1,
} PMU_STATUS;


typedef enum {
	USB_SUSPEND = 0,
	USB_UNCONFIGURED,
	USB_CONFIGURED
} usb_state_enum;

typedef enum {
	BATTERY_AVG_CURRENT = 0,
	BATTERY_AVG_VOLT = 1,
	BATTERY_AVG_TEMP = 2,
	BATTERY_AVG_MAX
} BATTERY_AVG_ENUM;

#ifdef HTC_ENABLE_AICL
typedef enum {
    AICL_START = 0,
    AICL_STOP,
    AICL_LOOP,
    AICL_DONE,
    AICL_MAX
} AICL_STATUS;
#endif

typedef enum charger_control_internal {
	DISABLE_PWRSRC_FINGERPRINT = 0,
	ENABLE_PWRSRC_FINGERPRINT,
	END_CHARGER_INTERNAL
} CHARGER_CONTROL_INTERNAL;

typedef enum {
    CONNECT_TYPE_MHL_NONE = 0,
    CONNECT_TYPE_MHL_500MA,
    CONNECT_TYPE_MHL_1000MA,
    CONNECT_TYPE_MHL_MAX
} MHL_CHG_STATUS;

#define HTC_EXT_UNKNOWN_USB_CHARGER		(1<<0)
#define HTC_EXT_CHG_UNDER_RATING		(1<<1)
#define HTC_EXT_CHG_SAFTY_TIMEOUT		(1<<2)
#define HTC_EXT_CHG_FULL_EOC_STOP		(1<<3)

typedef enum {
	TEMP_BELOW_NEG_10 = 0,
	TEMP_NEG_10_TO_POS_0,
	TEMP_POS_0_TO_POS_10,
	TEMP_POS_10_TO_POS_45,
	TEMP_POS_45_TO_POS_60,
	TEMP_ABOVE_POS_60
} temp_state_enum;


#define TEMP_POS_60_THRESHOLD  50
#define TEMP_POS_60_THRES_MINUS_X_DEGREE 47

#define TEMP_POS_45_THRESHOLD  45
#define TEMP_POS_45_THRES_MINUS_X_DEGREE 39

#define TEMP_POS_10_THRESHOLD  10
#define TEMP_POS_10_THRES_PLUS_X_DEGREE 16

#define TEMP_POS_0_THRESHOLD  0
#define TEMP_POS_0_THRES_PLUS_X_DEGREE 6

#ifdef CONFIG_MTK_FAN5405_SUPPORT
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#elif defined(CONFIG_MTK_BQ24158_SUPPORT)
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#else
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#endif

#define HTC_BATT_CHG_LIMIT_BIT_TALK				(1)
#define HTC_BATT_CHG_LIMIT_BIT_NAVI				(1<<1)
#define HTC_BATT_CHG_LIMIT_BIT_THRML			(1<<2)
#define HTC_BATT_CHG_LIMIT_BIT_KDDI				(1<<3)

typedef enum {
	TEMP_POS_LOW = 0,
	TEMP_POS_NORMAL,
	TEMP_POS_HIGH
} batt_temp_state_enum;

typedef struct {
	kal_bool bat_exist;
	kal_bool bat_full;
	INT32 bat_charging_state;
	UINT32 bat_vol;
	kal_bool bat_in_recharging_state;
	kal_uint32 Vsense;
	kal_bool charger_exist;
	UINT32 charger_vol;
	INT32 charger_protect_status;
	INT32 ICharging;
	INT32 IBattery;
	INT32 temperature;
	INT32 temperature_now;
	INT32 temperatureR;
	INT32 temperatureV;
	UINT32 total_charging_time;
	UINT32 PRE_charging_time;
	UINT32 CC_charging_time;
	UINT32 TOPOFF_charging_time;
	UINT32 POSTFULL_charging_time;
	UINT32 charger_type;
	INT32 SOC;
	INT32 UI_SOC;
	UINT32 nPercent_ZCV;
	UINT32 nPrecent_UI_SOC_check_point;
	UINT32 ZCV;
	UINT32 full_level;
	
	kal_bool keep_charger_on;
	kal_bool test_power_monitor;
	kal_bool force_ac_charger;
	kal_bool flag_enable_bms_charger_log;
	kal_bool is_warm;
    kal_bool is_overload;
	kal_bool flag_disable_safety_timer;
	kal_bool flag_disable_temp_protection;
	kal_bool flag_pa_fake_batt_temp;
	kal_int32 htc_extension;
    kal_int32 gap;
#ifdef HTC_ENABLE_AICL
    struct delayed_work vin_collapse_check_work;
    struct delayed_work aicl_check_work;
    struct workqueue_struct *aicl_check_wq;
    spinlock_t aicl_lock;
    kal_int32 aicl_state;
#endif
} PMU_ChargerStruct;

typedef struct {
    kal_bool bIs_charging;
    kal_bool bIs_cnt_rst;
    kal_uint32 uiTotal_time;
    kal_int32 iUI_SOC;
    kal_int32 iSOC;
    kal_int32 iGap;
}sSync_uisoc, *pSync_uisoc;

extern PMU_ChargerStruct BMT_status;
extern CHARGING_CONTROL battery_charging_control;
extern kal_bool g_ftm_battery_flag;
extern int g_ftm_charger_ctrl_stat;
extern int charging_level_data[1];
extern kal_bool g_call_state;
extern kal_bool g_charging_full_reset_bat_meter;
extern kal_bool g_force_reverse_boost_wa;
extern kal_bool g_batt_curr_full;
extern int g_chg_limit_reason;
extern int g_chg_in_mhl;
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
extern kal_bool ta_check_chr_type;
extern kal_bool ta_cable_out_occur;
extern kal_bool is_ta_connect;
extern struct wake_lock TA_charger_suspend_lock;
#endif


extern void charging_suspend_enable(void);
extern void charging_suspend_disable(void);
extern kal_bool bat_is_charger_exist(void);
extern kal_bool bat_is_charging_full(void);
extern void bat_set_ui_percentage(kal_uint32 uiSoc);
extern kal_uint32 bat_get_ui_percentage(void);
extern kal_uint32 get_charging_setting_current(void);
extern kal_uint32 bat_is_recharging_phase(void);
extern void do_chrdet_int_task(void);
extern void set_usb_current_unlimited(bool enable);
extern bool get_usb_current_unlimited(void);
extern CHARGER_TYPE mt_get_charger_type(void);
extern void htc_batt_mhl_charge_set(kal_uint32 mhl_chg_type);
extern void charger_switch_internal(int enable);
#ifdef CONFIG_MTK_SMART_BATTERY
extern void wake_up_bat(void);
extern unsigned long BAT_Get_Battery_Voltage(int polling_mode);
extern void mt_battery_charging_algorithm(void);
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
extern PMU_STATUS do_jeita_state_machine(void);
#endif

#else

#define wake_up_bat()			do {} while (0)
#define BAT_Get_Battery_Voltage(polling_mode)	({ 0; })

#endif

#ifdef CONFIG_MTK_POWER_EXT_DETECT
extern kal_bool bat_is_ext_power(void);
#endif

extern void htc_battery_para_init(void);
extern void htc_battery_update_gap(kal_int32 iGap);

#endif				
