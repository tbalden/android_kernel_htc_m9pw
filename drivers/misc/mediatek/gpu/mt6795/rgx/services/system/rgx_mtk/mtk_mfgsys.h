#ifndef MTK_MFGSYS_H
#define MTK_MFGSYS_H

#include "servicesext.h"
#include "rgxdevice.h"

#define MTK_PM_SUPPORT 1

PVRSRV_ERROR MTKMFGSystemInit(void);

IMG_VOID MTKMFGSystemDeInit(void);

void MTKMFGEnableDVFSTimer(bool bEnable);

PVRSRV_ERROR MTKDevPrePowerState(PVRSRV_DEV_POWER_STATE eNewPowerState,
                                         PVRSRV_DEV_POWER_STATE eCurrentPowerState,
									     IMG_BOOL bForced);

PVRSRV_ERROR MTKDevPostPowerState(PVRSRV_DEV_POWER_STATE eNewPowerState,
                                          PVRSRV_DEV_POWER_STATE eCurrentPowerState,
									      IMG_BOOL bForced);

PVRSRV_ERROR MTKSystemPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState);

PVRSRV_ERROR MTKSystemPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState);

#endif

