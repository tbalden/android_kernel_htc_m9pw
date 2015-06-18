#ifndef DDP_HTC_UTIL_H
#define DDP_HTC_UTIL_H

#include "lcm_drv.h"

#define DCS_MAX_CNT 128

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[DCS_MAX_CNT];
};

void htc_debugfs_init(void);

#endif
