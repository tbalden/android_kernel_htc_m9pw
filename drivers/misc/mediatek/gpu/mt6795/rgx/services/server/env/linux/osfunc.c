 /*!
@File
@Title          Environment related functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ 

#include <linux/version.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/hugetlb.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/genalloc.h>

#include <linux/string.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/hardirq.h>
#include <asm/tlbflush.h>
#include <asm/page.h>
#include <linux/timer.h>
#include <linux/capability.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE) || \
	defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) || \
	defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || \
	defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE) || \
	defined(PVR_LINUX_USING_WORKQUEUES)
#include <linux/workqueue.h>
#endif
#include <linux/kthread.h>
#include <asm/atomic.h>

#include "osfunc.h"
#include "img_types.h"
#include "mm.h"
#include "allocmem.h"
#include "mmap.h"
#include "env_data.h"
#include "pvr_debugfs.h"
#include "event.h"
#include "linkage.h"
#include "pvr_uaccess.h"
#include "pvr_debug.h"
#include "driverlock.h"
#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#include "process_stats.h"
#endif
#if defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
#include "syscommon.h"
#endif

#if defined(EMULATOR) || defined(VIRTUAL_PLATFORM)
#define EVENT_OBJECT_TIMEOUT_MS		(2000)
#else
#define EVENT_OBJECT_TIMEOUT_MS		(100)
#endif 

ENV_DATA *gpsEnvData = IMG_NULL;


#if defined(CONFIG_GENERIC_ALLOCATOR) && defined(CONFIG_X86) && (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))
#define POOL_SIZE	(4*1024*1024)
static struct gen_pool *pvrsrv_pool_writecombine = NULL;
static char *pool_start;

static void deinit_pvr_pool(void)
{
	gen_pool_destroy(pvrsrv_pool_writecombine);
	pvrsrv_pool_writecombine = NULL;
	vfree(pool_start);
	pool_start = NULL;
}

static void init_pvr_pool(void)
{
	struct vm_struct *tmp_area;
	int ret = -1;

	
	pvrsrv_pool_writecombine = gen_pool_create(PAGE_SHIFT, -1);
	if (!pvrsrv_pool_writecombine) {
		printk(KERN_ERR "%s: create pvrsrv_pool failed\n", __func__);
		return;
	}

	
	tmp_area = __get_vm_area(POOL_SIZE, VM_ALLOC,
			VMALLOC_START, VMALLOC_END);
	if (!tmp_area) {
		printk(KERN_ERR "%s: __get_vm_area failed\n", __func__);
		gen_pool_destroy(pvrsrv_pool_writecombine);
		pvrsrv_pool_writecombine = NULL;
		return;
	}

	pool_start = tmp_area->addr;

	if (!pool_start) {
		printk(KERN_ERR "%s:No vm space to create POOL\n",
				__func__);
		gen_pool_destroy(pvrsrv_pool_writecombine);
		pvrsrv_pool_writecombine = NULL;
		return;
	} else {
		
		ret = gen_pool_add(pvrsrv_pool_writecombine,
			(unsigned long) pool_start, POOL_SIZE, -1);
		if (ret) {
			printk(KERN_ERR "%s:could not remainder pool\n",
					__func__);
			deinit_pvr_pool();
			return;
		}
	}
	return;
}

static inline IMG_BOOL vmap_from_pool(void *pvCPUVAddr)
{
	IMG_CHAR *pcTmp = pvCPUVAddr;
	if ((pcTmp >= pool_start) && (pcTmp <= (pool_start + POOL_SIZE)))
	{
		return IMG_TRUE;
	}
	return IMG_FALSE;
}
#endif	

PVRSRV_ERROR OSMMUPxAlloc(PVRSRV_DEVICE_NODE *psDevNode, IMG_SIZE_T uiSize,
							Px_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr)
{
	IMG_CPU_PHYADDR sCpuPAddr;
	struct page *psPage;

	PVR_ASSERT(uiSize == PAGE_SIZE);

	psPage = alloc_page(GFP_KERNEL);
	if (psPage == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

#if defined (CONFIG_X86)
	{
		IMG_PVOID pvPageVAddr = page_address(psPage);
		int ret;
		ret = set_memory_wc((unsigned long)pvPageVAddr, 1);

		if (ret)
		{
			__free_page(psPage);
			return PVRSRV_ERROR_UNABLE_TO_SET_CACHE_MODE;
		}
	}
#endif
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64) || defined (CONFIG_METAG)
	{
		IMG_CPU_PHYADDR sCPUPhysAddrStart, sCPUPhysAddrEnd;
		IMG_PVOID pvPageVAddr = kmap(psPage);

		sCPUPhysAddrStart.uiAddr = page_to_phys(psPage);
		sCPUPhysAddrEnd.uiAddr = sCPUPhysAddrStart.uiAddr + PAGE_SIZE;

		OSInvalidateCPUCacheRangeKM(pvPageVAddr,
									pvPageVAddr + PAGE_SIZE,
									sCPUPhysAddrStart,
									sCPUPhysAddrEnd);
	}
#endif

	psMemHandle->u.pvHandle = psPage;
	sCpuPAddr.uiAddr = page_to_phys(psPage);

	PhysHeapCpuPAddrToDevPAddr(psDevNode->apsPhysHeap[PVRSRV_DEVICE_PHYS_HEAP_CPU_LOCAL], psDevPAddr, &sCpuPAddr);

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	    PVRSRVStatsIncrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, PAGE_SIZE);
#else
	PVRSRVStatsAddMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA,
								 psPage,
								 sCpuPAddr,
								 PAGE_SIZE,
								 IMG_NULL);
#endif
#endif

	return PVRSRV_OK;
}

void OSMMUPxFree(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle)
{
	struct page *psPage = (struct page*) psMemHandle->u.pvHandle;

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	    PVRSRVStatsDecrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, PAGE_SIZE);
#else
	PVRSRVStatsRemoveMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_ALLOC_PAGES_PT_UMA, (IMG_UINT64)(IMG_UINTPTR_T)psPage);
#endif
#endif

#if defined (CONFIG_X86)
	{
		IMG_PVOID pvPageVAddr;
		int ret;

		pvPageVAddr = page_address(psPage);
		ret = set_memory_wb((unsigned long) pvPageVAddr, 1);
		if (ret)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to reset page attribute", __FUNCTION__));
		}
	}
#endif
	__free_page(psPage);
}

PVRSRV_ERROR OSMMUPxMap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
						IMG_SIZE_T uiSize, IMG_DEV_PHYADDR *psDevPAddr,
						void **pvPtr)
{
	struct page **ppsPage = (struct page **) &psMemHandle->u.pvHandle;
	IMG_UINTPTR_T uiCPUVAddr;
	pgprot_t prot = PAGE_KERNEL;
	PVR_UNREFERENCED_PARAMETER(psDevNode);

	prot = pgprot_writecombine(prot);

#if defined(CONFIG_GENERIC_ALLOCATOR) && defined(CONFIG_X86) && (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))
	uiCPUVAddr = gen_pool_alloc(pvrsrv_pool_writecombine, PAGE_SIZE);

	if (uiCPUVAddr) {
		int ret = 0;
		struct vm_struct tmp_area;

		
		tmp_area.addr = (void *)uiCPUVAddr;
		tmp_area.size =  2 * PAGE_SIZE;
		ret = map_vm_area(&tmp_area, prot, &ppsPage);
		if (ret) {
			gen_pool_free(pvrsrv_pool_writecombine, uiCPUVAddr, PAGE_SIZE);
			PVR_DPF((PVR_DBG_ERROR,
					 "%s: Cannot map page to pool",
					 __func__));
			
			uiCPUVAddr = 0;
		}
	}

	
	if (uiCPUVAddr == 0)
#endif	
	{
		uiCPUVAddr = (IMG_UINTPTR_T) vm_map_ram(ppsPage,
												1,
												-1,
												prot);
	}

	
	if (((void *)uiCPUVAddr) == IMG_NULL)
	{
		return PVRSRV_ERROR_FAILED_TO_MAP_KERNELVIRTUAL;
	}

	*pvPtr = (void *) ((uiCPUVAddr & (~OSGetPageMask())) |
							((IMG_UINTPTR_T) (psDevPAddr->uiAddr & OSGetPageMask())));

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	
	PVRSRVStatsIncrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, PAGE_SIZE);
#else
	{
		IMG_CPU_PHYADDR sCpuPAddr;
		sCpuPAddr.uiAddr = 0;

		PVRSRVStatsAddMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA,
									 (void *)uiCPUVAddr,
									 sCpuPAddr,
									 uiSize,
									 IMG_NULL);
	}
#endif
#endif

	return PVRSRV_OK;
}

void OSMMUPxUnmap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle, void *pvPtr)
{
	PVR_UNREFERENCED_PARAMETER(psDevNode);
	PVR_UNREFERENCED_PARAMETER(psMemHandle);

#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#if !defined(PVRSRV_ENABLE_MEMORY_STATS)
	
	PVRSRVStatsDecrMemAllocStat(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, PAGE_SIZE);
#else
	PVRSRVStatsRemoveMemAllocRecord(PVRSRV_MEM_ALLOC_TYPE_VMAP_PT_UMA, (IMG_UINT64)(IMG_UINTPTR_T)pvPtr);
#endif
#endif

#if defined(CONFIG_GENERIC_ALLOCATOR) && defined(CONFIG_X86) && (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))
	if (vmap_from_pool(pvPtr))
	{
		unsigned long addr = (unsigned long)pvPtr;

		
		flush_cache_vunmap(addr, addr + PAGE_SIZE);
		
		unmap_kernel_range_noflush(addr, PAGE_SIZE);
		
		__flush_tlb_single(addr);
		
		gen_pool_free(pvrsrv_pool_writecombine, addr, PAGE_SIZE);
	}
	else
#endif	
	{
		vm_unmap_ram(pvPtr, 1);
	}
}

 
void OSMemCopy(void *pvDst, const void *pvSrc, IMG_SIZE_T ui32Size)
{
	memcpy(pvDst, pvSrc, ui32Size);
}
void OSMemCopyMTK(void *pvDst, const void *pvSrc, IMG_SIZE_T ui32Size)
{
#if defined(CONFIG_ARM64) || defined(__arm64__) || defined(__aarch64__)
	volatile IMG_UINT8 * pbDst = pvDst;
	volatile const IMG_UINT8 * pbSrc = pvSrc;
	for (; ui32Size > 0; --ui32Size)
	{
		*pbDst++ = *pbSrc++;
	}
#else
	memcpy(pvDst, pvSrc, ui32Size);
#endif
}


 
void OSMemSet(void *pvDest, IMG_UINT8 ui8Value, IMG_SIZE_T ui32Size)
{
	memset(pvDest, (char) ui8Value, (size_t) ui32Size);
}

IMG_INT OSMemCmp(void *pvBufA, void *pvBufB, IMG_SIZE_T uiLen)
{
	return (IMG_INT) memcmp(pvBufA, pvBufB, uiLen);
}

 
IMG_CHAR *OSStringNCopy(IMG_CHAR *pszDest, const IMG_CHAR *pszSrc, IMG_SIZE_T uSize)
{
	return strncpy(pszDest, pszSrc, uSize);
}

 /*!
@Function       OSSNPrintf
@Description    snprintf
@Return         the chars written or -1 on error
*/ 
IMG_INT32 OSSNPrintf(IMG_CHAR *pStr, IMG_SIZE_T ui32Size, const IMG_CHAR *pszFormat, ...)
{
	va_list argList;
	IMG_INT32 iCount;

	va_start(argList, pszFormat);
	iCount = vsnprintf(pStr, (size_t)ui32Size, pszFormat, argList);
	va_end(argList);

	return iCount;
}

IMG_SIZE_T OSStringLength(const IMG_CHAR *pStr)
{
	return strlen(pStr);
}

IMG_SIZE_T OSStringNLength(const IMG_CHAR *pStr, IMG_SIZE_T uiCount)
{
	return strnlen(pStr, uiCount);
}

IMG_INT32 OSStringCompare(const IMG_CHAR *pStr1, const IMG_CHAR *pStr2)
{
	return strcmp(pStr1, pStr2);
}

 
PVRSRV_ERROR OSInitEnvData(void)
{
	
	gpsEnvData = OSAllocMem(sizeof(ENV_DATA));
	if (gpsEnvData == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	gpsEnvData->pvBridgeData = OSAllocMem(PVRSRV_MAX_BRIDGE_IN_SIZE + PVRSRV_MAX_BRIDGE_OUT_SIZE);
	if (gpsEnvData->pvBridgeData == IMG_NULL)
	{
		OSFreeMem(gpsEnvData);
		
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

#if defined(CONFIG_GENERIC_ALLOCATOR) && defined(CONFIG_X86) && (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))
	if (!pvrsrv_pool_writecombine)
	{
		init_pvr_pool();
	}
#endif	

	return PVRSRV_OK;
}


 
void OSDeInitEnvData(void)
{
	ENV_DATA *psEnvData = gpsEnvData;

#if defined(CONFIG_GENERIC_ALLOCATOR) && defined(CONFIG_X86) && (LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0))
	if (pvrsrv_pool_writecombine)
	{
		deinit_pvr_pool();
	}
#endif

	OSFreeMem(psEnvData->pvBridgeData);
	psEnvData->pvBridgeData = IMG_NULL;

	OSFreeMem(psEnvData);
}

ENV_DATA *OSGetEnvData(void)
{
	return gpsEnvData;
}

 
void OSReleaseThreadQuanta(void)
{
	schedule();
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static inline IMG_UINT32 Clockus(void)
{
	return (jiffies * (1000000 / HZ));
}
#else
static inline IMG_UINT64 Clockns64(void)
{
	IMG_UINT64 timenow;

	preempt_disable();

	timenow = sched_clock();

	preempt_enable();

	return timenow;
}
#endif

 
IMG_UINT64 OSClockns64(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	return Clockns64();
#else
	return ((IMG_UINT64)Clockus()) * 1000ULL;
#endif
}

 
IMG_UINT64 OSClockus64(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	IMG_UINT64 timenow = Clockns64();
	IMG_UINT32 remainder;
	return OSDivide64r64(timenow, 1000, &remainder);
#else
	return ((IMG_UINT64)Clockus());
#endif
}


 
IMG_UINT32 OSClockus(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	return (IMG_UINT32) OSClockus64();
#else
	return Clockus();
#endif
}


 
IMG_UINT32 OSClockms(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	IMG_UINT64 timenow = Clockns64();
	IMG_UINT32 remainder;

	return OSDivide64(timenow, 1000000, &remainder);
#else
	IMG_UINT64 time, j = (IMG_UINT32)jiffies;

	time = j * (((1 << 16) * 1000) / HZ);
	time >>= 16;

	return (IMG_UINT32)time;
#endif
}


void OSWaitus(IMG_UINT32 ui32Timeus)
{
	udelay(ui32Timeus);
}


void OSSleepms(IMG_UINT32 ui32Timems)
{
	msleep(ui32Timems);
}


IMG_PID OSGetCurrentProcessIDKM(void)
{
	if (in_interrupt())
	{
		return KERNEL_ID;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
	return (IMG_PID)current->pgrp;
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24))
	return (IMG_PID)task_tgid_nr(current);
#else
	return (IMG_PID)current->tgid;
#endif
#endif
}

IMG_CHAR *OSGetCurrentProcessNameKM(void)
{
	return current->comm;
}

IMG_UINTPTR_T OSGetCurrentThreadIDKM(void)
{
	if (in_interrupt())
	{
		return KERNEL_ID;
	}

	return current->pid;
}

 
IMG_SIZE_T OSGetPageSize(void)
{
	return PAGE_SIZE;
}

 
IMG_SIZE_T OSGetPageShift(void)
{
	return PAGE_SHIFT;
}

 
IMG_SIZE_T OSGetPageMask(void)
{
	return (OSGetPageSize()-1);
}

#if !defined (SUPPORT_SYSTEM_INTERRUPT_HANDLING)
typedef struct _LISR_DATA_ {
	PFN_LISR pfnLISR;
	void *pvData;
	IMG_UINT32 ui32IRQ;
} LISR_DATA;

static irqreturn_t DeviceISRWrapper(int irq, void *dev_id)
{
	LISR_DATA *psLISRData = (LISR_DATA *) dev_id;
	IMG_BOOL bStatus = IMG_FALSE;

	PVR_UNREFERENCED_PARAMETER(irq);

	bStatus = psLISRData->pfnLISR(psLISRData->pvData);

	return bStatus ? IRQ_HANDLED : IRQ_NONE;
}
#endif

PVRSRV_ERROR OSInstallDeviceLISR(PVRSRV_DEVICE_CONFIG *psDevConfig,
				 IMG_HANDLE *hLISRData,
				 PFN_LISR pfnLISR,
				 void *pvData)
{
#if defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
	return SysInstallDeviceLISR(psDevConfig->ui32IRQ,
					psDevConfig->pszName,
					pfnLISR,
					pvData,
					hLISRData);
#else
	LISR_DATA *psLISRData;
	unsigned long flags = IRQF_TRIGGER_LOW;

	psLISRData = kmalloc(sizeof(LISR_DATA), GFP_KERNEL);

	psLISRData->pfnLISR = pfnLISR;
	psLISRData->pvData = pvData;
	psLISRData->ui32IRQ = psDevConfig->ui32IRQ;

	if (psDevConfig->bIRQIsShared)
	{
		flags |= IRQF_SHARED;
	}

	if (psDevConfig->eIRQActiveLevel == PVRSRV_DEVICE_IRQ_ACTIVE_HIGH)
	{
		flags |= IRQF_TRIGGER_HIGH;
	}
	else if (psDevConfig->eIRQActiveLevel == PVRSRV_DEVICE_IRQ_ACTIVE_LOW)
	{
		flags |= IRQF_TRIGGER_LOW;
	}

	PVR_TRACE(("Installing device LISR %s on IRQ %d with cookie %p", psDevConfig->pszName, psDevConfig->ui32IRQ, pvData));

	if(request_irq(psDevConfig->ui32IRQ, DeviceISRWrapper,
		flags, psDevConfig->pszName, psLISRData))
	{
		PVR_DPF((PVR_DBG_ERROR,"OSInstallDeviceLISR: Couldn't install device LISR on IRQ %d", psDevConfig->ui32IRQ));

		return PVRSRV_ERROR_UNABLE_TO_INSTALL_ISR;
	}

	*hLISRData = (IMG_HANDLE) psLISRData;

	return PVRSRV_OK;
#endif
}

PVRSRV_ERROR OSUninstallDeviceLISR(IMG_HANDLE hLISRData)
{
#if defined (SUPPORT_SYSTEM_INTERRUPT_HANDLING)
	return SysUninstallDeviceLISR(hLISRData);
#else
	LISR_DATA *psLISRData = (LISR_DATA *) hLISRData;

	PVR_TRACE(("Uninstalling device LISR on IRQ %d with cookie %p", psLISRData->ui32IRQ,  psLISRData->pvData));

	free_irq(psLISRData->ui32IRQ, psLISRData);
	kfree(psLISRData);

	return PVRSRV_OK;
#endif
}

#if defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct workqueue_struct *psWorkQueue;
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR,
							void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	psMISRData->psWorkQueue = create_singlethread_workqueue("pvr_workqueue");

	if (psMISRData->psWorkQueue == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSInstallMISR: create_singlethreaded_workqueue failed"));
		kfree(psMISRData);
		return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;
	}

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}

PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	PVR_TRACE(("Uninstalling MISR"));

	destroy_workqueue(psMISRData->psWorkQueue);
	kfree(psMISRData);

	return PVRSRV_OK;
}

PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	queue_work(psMISRData->psWorkQueue, &psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}
#else	
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}


PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	PVR_TRACE(("Uninstalling MISR"));

	flush_scheduled_work();

	kfree(hMISRData);

	return PVRSRV_OK;
}

PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = hMISRData;
#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	schedule_work(&psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}

#else	
typedef struct _MISR_DATA_ {
	struct tasklet_struct sMISRTasklet;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

static void MISRWrapper(unsigned long data)
{
	MISR_DATA *psMISRData = (MISR_DATA *) data;

	psMISRData->pfnMISR(psMISRData->hData);
}

PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, void *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	tasklet_init(&psMISRData->sMISRTasklet, MISRWrapper, (unsigned long)psMISRData);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}

PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	PVR_TRACE(("Uninstalling MISR"));

	tasklet_kill(&psMISRData->sMISRTasklet);

	return PVRSRV_OK;
}

PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	tasklet_schedule(&psMISRData->sMISRTasklet);
#endif
	return PVRSRV_OK;
}

#endif 
#endif 

const IMG_INT32 ai32OSPriorityValues[LAST_PRIORITY] = { -20, 
                                                        -10, 
                                                          0, 
                                                          9, 
                                                         19, 
                                                        -22};

typedef struct {
	struct task_struct *kthread;
	PFN_THREAD pfnThread;
	void *hData;
	OS_THREAD_LEVEL eThreadPriority;
} OSThreadData;

static int OSThreadRun(void *data)
{
	OSThreadData *psOSThreadData = data;

	
	if (psOSThreadData->eThreadPriority != NOSET_PRIORITY &&
			psOSThreadData->eThreadPriority < LAST_PRIORITY)
		set_user_nice(current, ai32OSPriorityValues[psOSThreadData->eThreadPriority]);

	
	psOSThreadData->pfnThread(psOSThreadData->hData);

	
	while (!kthread_should_stop())
	{
		 schedule();
	}

	return 0;
}

PVRSRV_ERROR OSThreadCreate(IMG_HANDLE *phThread,
							IMG_CHAR *pszThreadName,
							PFN_THREAD pfnThread,
							void *hData)
{
	
	return OSThreadCreatePriority(phThread, pszThreadName, pfnThread, hData, NOSET_PRIORITY);
}

PVRSRV_ERROR OSThreadCreatePriority(IMG_HANDLE *phThread,
							IMG_CHAR *pszThreadName,
							PFN_THREAD pfnThread,
							void *hData,
							OS_THREAD_LEVEL eThreadPriority)
{
	OSThreadData *psOSThreadData;
	PVRSRV_ERROR eError;

	psOSThreadData = kmalloc(sizeof(OSThreadData), GFP_KERNEL);
	if (psOSThreadData == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	psOSThreadData->pfnThread = pfnThread;
	psOSThreadData->hData = hData;
	psOSThreadData->eThreadPriority= eThreadPriority;
	psOSThreadData->kthread = kthread_run(OSThreadRun, psOSThreadData, pszThreadName);

	if (IS_ERR(psOSThreadData->kthread))
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_kthread;
	}

	*phThread = psOSThreadData;

	return PVRSRV_OK;

fail_kthread:
	kfree(psOSThreadData);
fail_alloc:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR OSThreadDestroy(IMG_HANDLE hThread)
{
	OSThreadData *psOSThreadData = hThread;
	int ret;

	
	ret = kthread_stop(psOSThreadData->kthread);
	PVR_ASSERT(ret == 0);
	kfree(psOSThreadData);

	return PVRSRV_OK;
}

void OSPanic(void)
{
	BUG();

#if defined(__KLOCWORK__)
	
	abort();
#endif
}

 
void *
OSMapPhysToLin(IMG_CPU_PHYADDR BasePAddr,
               IMG_SIZE_T ui32Bytes,
               IMG_UINT32 ui32MappingFlags)
{
	void *pvIORemapCookie;

	pvIORemapCookie = IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags);
	if(pvIORemapCookie == IMG_NULL)
	{
		PVR_ASSERT(0);
		return IMG_NULL;
	}

	return pvIORemapCookie;
}


 
IMG_BOOL
OSUnMapPhysToLin(void *pvLinAddr, IMG_SIZE_T ui32Bytes, IMG_UINT32 ui32MappingFlags)
{
	PVR_UNREFERENCED_PARAMETER(ui32Bytes);

	IOUnmapWrapper(pvLinAddr);

	return IMG_TRUE;
}

IMG_UINT8 OSReadHWReg8(IMG_PVOID	pvLinRegBaseAddr,
						IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT8) readb((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x4e;
#endif
}

IMG_UINT16 OSReadHWReg16(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT16) readw((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x3a4e;
#endif
}

IMG_UINT32 OSReadHWReg32(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
	return (IMG_UINT32) readl((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x30f73a4e;
#endif
}


IMG_UINT64 OSReadHWReg64(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
	IMG_UINT64	ui64Result;

	ui64Result = OSReadHWReg32(pvLinRegBaseAddr, ui32Offset + 4);
	ui64Result <<= 32;
	ui64Result |= (IMG_UINT64)OSReadHWReg32(pvLinRegBaseAddr, ui32Offset);

	return ui64Result;
}

IMG_DEVMEM_SIZE_T OSReadHWRegBank(IMG_PVOID pvLinRegBaseAddr,
                                  IMG_UINT32 ui32Offset,
                                  IMG_UINT8 *pui8DstBuf,
                                  IMG_DEVMEM_SIZE_T uiDstBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;



	for(uiCounter = 0; uiCounter < uiDstBufLen; uiCounter++) {
		*(pui8DstBuf + uiCounter) =
		  readb(pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiDstBufLen;
#endif
}

void OSWriteHWReg8(void			*pvLinRegBaseAddr,
				   IMG_UINT32	ui32Offset,
				   IMG_UINT8	ui8Value)
{
#if !defined(NO_HARDWARE)
	writeb(ui8Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

void OSWriteHWReg16(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT16	ui16Value)
{
#if !defined(NO_HARDWARE)
	writew(ui16Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

void OSWriteHWReg32(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT32	ui32Value)
{
#if !defined(NO_HARDWARE)
	writel(ui32Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}


void OSWriteHWReg64(void		*pvLinRegBaseAddr,
					IMG_UINT32	ui32Offset,
					IMG_UINT64	ui64Value)
{
#if !defined(NO_HARDWARE)
	IMG_UINT32 ui32ValueLow, ui32ValueHigh;

	ui32ValueLow = ui64Value & 0xffffffff;
	ui32ValueHigh = ((IMG_UINT64) (ui64Value >> 32)) & 0xffffffff;

	writel(ui32ValueLow, pvLinRegBaseAddr + ui32Offset);
	writel(ui32ValueHigh, pvLinRegBaseAddr + ui32Offset + 4);
#endif
}

IMG_DEVMEM_SIZE_T OSWriteHWRegBank(void *pvLinRegBaseAddr,
								   IMG_UINT32 ui32Offset,
								   IMG_UINT8 *pui8SrcBuf,
								   IMG_DEVMEM_SIZE_T uiSrcBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;



	for(uiCounter = 0; uiCounter < uiSrcBufLen; uiCounter++) {
		writeb(*(pui8SrcBuf + uiCounter),
		       pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiSrcBufLen;
#endif
}

#define	OS_MAX_TIMERS	8

typedef struct TIMER_CALLBACK_DATA_TAG
{
	IMG_BOOL			bInUse;
	PFN_TIMER_FUNC		pfnTimerFunc;
	void				*pvData;
	struct timer_list	sTimer;
	IMG_UINT32			ui32Delay;
	IMG_BOOL			bActive;
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	struct work_struct	sWork;
#endif
}TIMER_CALLBACK_DATA;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
static struct workqueue_struct	*psTimerWorkQueue;
#endif

static TIMER_CALLBACK_DATA sTimers[OS_MAX_TIMERS];

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
DEFINE_MUTEX(sTimerStructLock);
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static spinlock_t sTimerStructLock = SPIN_LOCK_UNLOCKED;
#else
static DEFINE_SPINLOCK(sTimerStructLock);
#endif
#endif

static void OSTimerCallbackBody(TIMER_CALLBACK_DATA *psTimerCBData)
{
	if (!psTimerCBData->bActive)
		return;

	
	psTimerCBData->pfnTimerFunc(psTimerCBData->pvData);

	
	mod_timer(&psTimerCBData->sTimer, psTimerCBData->ui32Delay + jiffies);
}


 
static void OSTimerCallbackWrapper(IMG_UINTPTR_T uData)
{
	TIMER_CALLBACK_DATA	*psTimerCBData = (TIMER_CALLBACK_DATA*)uData;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	int res;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	res = queue_work(psTimerWorkQueue, &psTimerCBData->sWork);
#else
	res = schedule_work(&psTimerCBData->sWork);
#endif
	if (res == 0)
	{
		PVR_DPF((PVR_DBG_WARNING, "OSTimerCallbackWrapper: work already queued"));
	}
#else
	OSTimerCallbackBody(psTimerCBData);
#endif
}


#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
static void OSTimerWorkQueueCallBack(struct work_struct *psWork)
{
	TIMER_CALLBACK_DATA *psTimerCBData = container_of(psWork, TIMER_CALLBACK_DATA, sWork);

	OSTimerCallbackBody(psTimerCBData);
}
#endif

 
IMG_HANDLE OSAddTimer(PFN_TIMER_FUNC pfnTimerFunc, void *pvData, IMG_UINT32 ui32MsTimeout)
{
	TIMER_CALLBACK_DATA	*psTimerCBData;
	IMG_UINT32		ui32i;
#if !(defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE))
	unsigned long		ulLockFlags;
#endif

	
	if(!pfnTimerFunc)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: passed invalid callback"));
		return IMG_NULL;
	}

	
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	mutex_lock(&sTimerStructLock);
#else
	spin_lock_irqsave(&sTimerStructLock, ulLockFlags);
#endif
	for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
	{
		psTimerCBData = &sTimers[ui32i];
		if (!psTimerCBData->bInUse)
		{
			psTimerCBData->bInUse = IMG_TRUE;
			break;
		}
	}
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	mutex_unlock(&sTimerStructLock);
#else
	spin_unlock_irqrestore(&sTimerStructLock, ulLockFlags);
#endif
	if (ui32i >= OS_MAX_TIMERS)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: all timers are in use"));
		return IMG_NULL;
	}

	psTimerCBData->pfnTimerFunc = pfnTimerFunc;
	psTimerCBData->pvData = pvData;
	psTimerCBData->bActive = IMG_FALSE;

	psTimerCBData->ui32Delay = ((HZ * ui32MsTimeout) < 1000)
								?	1
								:	((HZ * ui32MsTimeout) / 1000);
	
	init_timer(&psTimerCBData->sTimer);

	
	psTimerCBData->sTimer.function = (void *)OSTimerCallbackWrapper;
	psTimerCBData->sTimer.data = (IMG_UINTPTR_T)psTimerCBData;

	return (IMG_HANDLE)(IMG_UINTPTR_T)(ui32i + 1);
}


static inline TIMER_CALLBACK_DATA *GetTimerStructure(IMG_HANDLE hTimer)
{
	IMG_UINT32 ui32i = (IMG_UINT32)((IMG_UINTPTR_T)hTimer) - 1;

	PVR_ASSERT(ui32i < OS_MAX_TIMERS);

	return &sTimers[ui32i];
}

 
PVRSRV_ERROR OSRemoveTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(!psTimerCBData->bActive);

	
	psTimerCBData->bInUse = IMG_FALSE;

	return PVRSRV_OK;
}


 
PVRSRV_ERROR OSEnableTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(!psTimerCBData->bActive);

	
	psTimerCBData->bActive = IMG_TRUE;

	
	psTimerCBData->sTimer.expires = psTimerCBData->ui32Delay + jiffies;

	
	add_timer(&psTimerCBData->sTimer);

	return PVRSRV_OK;
}


 
PVRSRV_ERROR OSDisableTimer (IMG_HANDLE hTimer)
{
	TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

	PVR_ASSERT(psTimerCBData->bInUse);
	PVR_ASSERT(psTimerCBData->bActive);

	
	psTimerCBData->bActive = IMG_FALSE;
	smp_mb();

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	flush_scheduled_work();
#endif

	
	del_timer_sync(&psTimerCBData->sTimer);

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	flush_scheduled_work();
#endif

	return PVRSRV_OK;
}


 
PVRSRV_ERROR OSEventObjectCreate(const IMG_CHAR *pszName, IMG_HANDLE *hEventObject)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVR_UNREFERENCED_PARAMETER(pszName);

	if(hEventObject)
	{
		if(LinuxEventObjectListCreate(hEventObject) != PVRSRV_OK)
		{
			 eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		}

	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectCreate: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_UNABLE_TO_CREATE_EVENT;
	}

	return eError;
}


 
PVRSRV_ERROR OSEventObjectDestroy(IMG_HANDLE hEventObject)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hEventObject)
	{
		LinuxEventObjectListDestroy(hEventObject);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

 
PVRSRV_ERROR OSEventObjectWaitTimeout(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs)
{
	PVRSRV_ERROR eError;

	if(hOSEventKM && uiTimeoutMs > 0)
	{
		eError = LinuxEventObjectWait(hOSEventKM, uiTimeoutMs);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectWait: invalid arguments %p, %d", hOSEventKM, uiTimeoutMs ));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

 
PVRSRV_ERROR OSEventObjectWait(IMG_HANDLE hOSEventKM)
{
	return OSEventObjectWaitTimeout(hOSEventKM, EVENT_OBJECT_TIMEOUT_MS);
}

 
PVRSRV_ERROR OSEventObjectOpen(IMG_HANDLE hEventObject,
											IMG_HANDLE *phOSEvent)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hEventObject)
	{
		if(LinuxEventObjectAdd(hEventObject, phOSEvent) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectAdd: failed"));
			eError = PVRSRV_ERROR_INVALID_PARAMS;
		}
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectOpen: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

 
PVRSRV_ERROR OSEventObjectClose(IMG_HANDLE hOSEventKM)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if(hOSEventKM)
	{
		if(LinuxEventObjectDelete(hOSEventKM) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectDelete: failed"));
			eError = PVRSRV_ERROR_INVALID_PARAMS;
		}

	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

 
PVRSRV_ERROR OSEventObjectSignal(IMG_HANDLE hEventObject)
{
	PVRSRV_ERROR eError;

	if(hEventObject)
	{
		eError = LinuxEventObjectSignal(hEventObject);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "OSEventObjectSignal: hOSEventKM is not a valid handle"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
	}

	return eError;
}

 
IMG_BOOL OSProcHasPrivSrvInit(void)
{
	return capable(CAP_SYS_ADMIN) != 0;
}

 
PVRSRV_ERROR OSCopyToUser(IMG_PVOID pvProcess,
                          void *pvDest,
                          const void *pvSrc,
                          IMG_SIZE_T ui32Bytes)
{
	PVR_UNREFERENCED_PARAMETER(pvProcess);

	if(pvr_copy_to_user(pvDest, pvSrc, ui32Bytes)==0)
		return PVRSRV_OK;
	else
		return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

 
PVRSRV_ERROR OSCopyFromUser(IMG_PVOID pvProcess,
                            void *pvDest,
                            const void *pvSrc,
                            IMG_SIZE_T ui32Bytes)
{
	PVR_UNREFERENCED_PARAMETER(pvProcess);

	if(pvr_copy_from_user(pvDest, pvSrc, ui32Bytes)==0)
		return PVRSRV_OK;
	else
		return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

 
IMG_BOOL OSAccessOK(IMG_VERIFY_TEST eVerification, void *pvUserPtr, IMG_SIZE_T ui32Bytes)
{
	IMG_INT linuxType;

	if (eVerification == PVR_VERIFY_READ)
	{
		linuxType = VERIFY_READ;
	}
	else
	{
		PVR_ASSERT(eVerification == PVR_VERIFY_WRITE);
		linuxType = VERIFY_WRITE;
	}

	return access_ok(linuxType, pvUserPtr, ui32Bytes);
}


void OSWriteMemoryBarrier(void)
{
	wmb();
}


void OSMemoryBarrier(void)
{
	mb();
}

PVRSRV_ERROR OSWRLockCreate(POSWR_LOCK *ppsLock)
{
	POSWR_LOCK psLock;

	psLock = kmalloc(sizeof(*psLock), GFP_KERNEL);
	if (psLock == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	init_rwsem(&psLock->sRWLock);

	*ppsLock = psLock;

	return PVRSRV_OK;
}

void OSWRLockDestroy(POSWR_LOCK psLock)
{
	kfree(psLock);
}

IMG_UINT64 OSDivide64r64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder)
{
	*pui32Remainder = do_div(ui64Divident, ui32Divisor);

	return ui64Divident;
}

IMG_UINT32 OSDivide64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder)
{
	*pui32Remainder = do_div(ui64Divident, ui32Divisor);

	return (IMG_UINT32) ui64Divident;
}

PVRSRV_ERROR PVROSFuncInit(void)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	{
		psTimerWorkQueue = create_workqueue("pvr_timer");
		if (psTimerWorkQueue == NULL)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: couldn't create timer workqueue", __FUNCTION__));
			return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;
		}
	}
#endif

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
	{
		IMG_UINT32 ui32i;

		for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
		{
			TIMER_CALLBACK_DATA *psTimerCBData = &sTimers[ui32i];

			INIT_WORK(&psTimerCBData->sWork, OSTimerWorkQueueCallBack);
		}
	}
#endif
	return PVRSRV_OK;
}

void PVROSFuncDeInit(void)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
	if (psTimerWorkQueue != NULL)
	{
		destroy_workqueue(psTimerWorkQueue);
	}
#endif
}

static IMG_BOOL gbDoRelease = IMG_TRUE;
void OSSetReleasePVRLock(void){ gbDoRelease = IMG_TRUE; }
void OSSetKeepPVRLock(void) { gbDoRelease = IMG_FALSE;}
IMG_BOOL OSGetReleasePVRLock(void){ return gbDoRelease;}

void OSDumpStack(void)
{
	dump_stack();
}

void OSAcquireBridgeLock(void)
{
	mutex_lock(&gPVRSRVLock);
}

void OSReleaseBridgeLock(void)
{
	mutex_unlock(&gPVRSRVLock);
}


 
IMG_PVOID OSCreateStatisticEntry(IMG_CHAR* pszName, IMG_PVOID pvFolder,
                                 OS_GET_STATS_ELEMENT_FUNC* pfnGetElement,
								 OS_INC_STATS_MEM_REFCOUNT_FUNC* pfnIncMemRefCt,
								 OS_DEC_STATS_MEM_REFCOUNT_FUNC* pfnDecMemRefCt,
                                 IMG_PVOID pvData)
{
	return PVRDebugFSCreateStatisticEntry(pszName, pvFolder, pfnGetElement, pfnIncMemRefCt, pfnDecMemRefCt, pvData);
} 


 
void OSRemoveStatisticEntry(IMG_PVOID pvEntry)
{
	PVRDebugFSRemoveStatisticEntry(pvEntry);
} 


 
IMG_PVOID OSCreateStatisticFolder(IMG_CHAR *pszName, IMG_PVOID pvFolder)
{
	void *pvFolderHandle = IMG_NULL;
	int iResult;

	iResult = PVRDebugFSCreateEntryDir(pszName, pvFolder, &pvFolderHandle);
	return (iResult == 0) ? pvFolderHandle : IMG_NULL;
} 


 
void OSRemoveStatisticFolder(IMG_PVOID pvFolder)
{
	PVRDebugFSRemoveEntryDir(pvFolder);
} 
