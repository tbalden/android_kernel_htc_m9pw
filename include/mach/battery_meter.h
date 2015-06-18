#ifndef _BATTERY_METER_H
#define _BATTERY_METER_H

#include <mach/mt_typedefs.h>
#include "cust_battery_meter.h"
#define FG_CURRENT_AVERAGE_SIZE 30
#define FG_ERROR_BATTERY 255



typedef struct {
	INT32 BatteryTemp;
	INT32 TemperatureR;
} BATT_TEMPERATURE;

extern int fg_qmax_update_for_aging_flag;

extern kal_int32 battery_meter_get_battery_voltage(kal_bool update);
extern kal_int32 battery_meter_get_charging_current_imm(void);
extern kal_int32 battery_meter_get_charging_current(void);
extern kal_int32 battery_meter_get_battery_current(void);
extern kal_bool battery_meter_get_battery_current_sign(void);
extern kal_int32 battery_meter_get_car(void);
extern kal_int32 battery_meter_get_battery_temperature(kal_bool update);
extern kal_int32 battery_meter_get_charger_voltage(void);
extern kal_int32 battery_meter_get_battery_percentage(void);
extern kal_int32 battery_meter_initial(void);
extern kal_int32 battery_meter_reset(kal_uint32 uiUI_soc);
extern kal_int32 battery_meter_sync(kal_int32 bat_i_sense_offset);

extern kal_int32 battery_meter_get_battery_zcv(void);
extern kal_int32 battery_meter_get_battery_nPercent_zcv(void);	
extern kal_int32 battery_meter_get_battery_nPercent_UI_SOC(void);	

extern kal_int32 battery_meter_get_tempR(kal_int32 dwVolt);
extern kal_int32 battery_meter_get_tempV(void);
extern kal_int32 battery_meter_get_VSense(void);	
extern int fgauge_get_battery_id(void);
extern int fgauge_get_battery_id_mv(void);
extern kal_int32 fgauge_read_v_by_d(int d_val);

extern kal_int32 htc_battery_meter_get_battery_voltage_imm(int times);
extern kal_int32 htc_battery_meter_get_battery_current_imm(kal_bool bImmediately);
extern kal_int32 htc_battery_meter_get_battery_temperature_imm(int times);
extern kal_uint32 htc_battery_meter_show_attr(char *buf, kal_uint32 size);
extern void htc_battery_meter_overload(kal_bool is_reset, kal_bool *overload);
extern kal_uint32 htc_battery_meter_get_battery_c_aging(void);
extern kal_int32 htc_battery_meter_get_car(void);

#endif				
