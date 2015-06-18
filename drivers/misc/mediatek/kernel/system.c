#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/mtk_rtc.h>
#include <mach/wd_api.h>
extern void wdt_arch_reset(char);

#ifdef CONFIG_HTC_REBOOT_REASON_INFORMATION
#include <linux/htc_reboot_info.h>
#endif

#include <linux/mrdump.h>
#include <linux/aee.h>
#include <linux/htc_devices_dtb.h>

#include <linux/delay.h>

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
extern void mtk_watchdog_bark(void);
#endif

void arch_reset(char mode, const char *cmd)
{
	char reboot = NO_AUTOREBOOT_RESET;
	int res = 0;
	struct wd_api *wd_api = NULL;
	int ramdump_debug_en = (get_radio_flag() & 0x8?1:0);
	res = get_wd_api(&wd_api);
	pr_notice("arch_reset: cmd = %s\n", cmd ? : "NULL");
	pr_notice("arch_reset: config[8] = 0x%.8X\n", get_radio_flag());

#ifdef CONFIG_HTC_REBOOT_REASON_INFORMATION
#ifdef CONFIG_MTK_AEE_MRDUMP
	switch(mrdump_get_ramdump_reason()) {
	case AEE_REBOOT_MODE_KERNEL_OOPS:
	case AEE_REBOOT_MODE_KERNEL_PANIC:
	case AEE_REBOOT_MODE_NESTED_EXCEPTION:
	case AEE_REBOOT_MODE_WDT:
	case AEE_REBOOT_MODE_MANUAL_KDUMP:
	case AEE_REBOOT_MODE_MODEM_FATAL:
		reboot = (ramdump_debug_en?SW_RESET:HW_RESET);
		goto do_arch_reboot;
		break;
	default:
		
		htc_reboot_reason_update_by_cmd(cmd);
		mrdump_set_ramdump_reason(AEE_REBOOT_MODE_NORMAL ,cmd);
		break;
	}
#else
	htc_reboot_reason_update_by_cmd(cmd);
	mrdump_set_ramdump_reason(AEE_REBOOT_MODE_NORMAL ,cmd);
#endif 

#endif 
	if (cmd && !strcmp(cmd, "charger")) {
		
	} else if (cmd && !strcmp(cmd, "recovery")) {
		rtc_mark_recovery();
	} else if (cmd && !strcmp(cmd, "bootloader")) {
		reboot = HW_RESET;
		rtc_mark_fast();
	}
#if defined(CONFIG_HTC_DEBUG_WATCHDOG) 
	else if (cmd && !strcmp(cmd, "force-dog-bark")){
		pr_notice("%s: Force dog bark!\r\n", __func__);
		
		
                reboot = SW_RESET;  
		mrdump_set_ramdump_reason(AEE_REBOOT_MODE_MANUAL_KDUMP, cmd);
		mtk_watchdog_bark();
		mdelay(10000);
		pr_info("%s: Force Watchdog bark does not work, falling back to normal process.\r\n", __func__);
	}
#endif
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	else if (cmd && !strcmp(cmd, "kpoc")) {
		rtc_mark_kpoc();
	}
#endif
	else if(cmd && !strncmp(cmd, "oem-", 4)){
		unsigned long code;
		code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
		
		if ((code >= 0x93) && (code <= 0x99)){
#ifdef CONFIG_MTK_AEE_MRDUMP
			mrdump_set_ramdump_reason(AEE_REBOOT_MODE_MODEM_FATAL, cmd);
#endif
			reboot = SW_RESET; 
		} else {
			reboot = (htc_is_need_sw_reboot() == 1?SW_RESET:HW_RESET);
		}
	}
	else if (cmd && !strncmp(cmd, "force-hard", 10)){
#ifdef CONFIG_MTK_AEE_MRDUMP
		mrdump_set_ramdump_reason(AEE_REBOOT_MODE_MANUAL_KDUMP, cmd);
#endif
		reboot = (ramdump_debug_en?SW_RESET:HW_RESET); 
	}
	else if(reboot != SW_RESET) {
		reboot = (htc_is_need_sw_reboot() == 1?SW_RESET:HW_RESET);
	}

do_arch_reboot:

	if (res) {
		pr_notice("arch_reset, get wd api error %d\n", res);
	} else {
		wd_api->wd_sw_reset(reboot);
	}
}
