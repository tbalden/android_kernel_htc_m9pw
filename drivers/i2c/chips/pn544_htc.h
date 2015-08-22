






#ifndef NFC_READ_RFSKUID
#define NFC_READ_RFSKUID 0
#endif

#ifndef NFC_GET_BOOTMODE
#define NFC_GET_BOOTMODE 0
#endif

#ifndef NFC_OFF_MODE_CHARGING_ENABLE
#define NFC_OFF_MODE_CHARGING_ENABLE 0
#endif

#ifndef NFC_POWER_OFF_SEQUENCE
#define NFC_POWER_OFF_SEQUENCE 0
#endif

#ifndef NFC_PVDD_GPIO_DT
#define NFC_PVDD_GPIO_DT 0
#endif

#ifndef NFC_PVDD_LOAD_SWITCH
#define NFC_PVDD_LOAD_SWITCH 1
#endif

#ifndef NFC_READ_RFSKUID_MTK6795
#define NFC_READ_RFSKUID_MTK6795 0
#endif 



#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5



int pn544_htc_check_rfskuid(int in_is_alive);

int pn544_htc_get_bootmode(void);


void pn544_htc_parse_dt(struct device *dev);

void pn544_htc_off_mode_charging (void);


int pn544_htc_pvdd_on (void);
