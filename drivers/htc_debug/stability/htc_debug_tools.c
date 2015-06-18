/* Copyright (c) 2013, HTC Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/htc_debug_tools.h>
#include <linux/kernel_stat.h>


#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
#define PET_CHECK_THRESHOLD    24
#define NSEC_PER_THRESHOLD     PET_CHECK_THRESHOLD * NSEC_PER_SEC

static int pet_check_counter = PET_CHECK_THRESHOLD;
static unsigned long long htc_debug_watchdog_last_pet;
static unsigned long long last_pet_check;

extern htc_debug_watchdog_enabled(void);

void htc_debug_watchdog_check_pet(unsigned long long timestamp)
{
	 if (!htc_debug_watchdog_enabled() || !htc_debug_watchdog_last_pet)
		return;

	if (timestamp - htc_debug_watchdog_last_pet > (unsigned long long)NSEC_PER_THRESHOLD) {
		if (timestamp - last_pet_check > (unsigned long long)NSEC_PER_SEC) { 
			last_pet_check = timestamp;
			pr_notice("\n%s: MTK watchdog was blocked for more than %d seconds!\n", __func__, pet_check_counter++);
			pr_notice("%s: Prepare to dump stack...\n", __func__);
			dump_stack();
			pr_info("\n ### Show Blocked State ###\n");
			show_state_filter(TASK_UNINTERRUPTIBLE);
		}
	}
}

void htc_debug_watchdog_update_last_pet(unsigned long long last_pet)
{
        htc_debug_watchdog_last_pet = last_pet;
        
        pet_check_counter = PET_CHECK_THRESHOLD;
}

#endif

unsigned long htc_debug_get_sched_clock_ms(void)
{
        unsigned long long timestamp;
        timestamp = sched_clock();
        do_div(timestamp, NSEC_PER_MSEC);
        return ((unsigned long) timestamp);
}



