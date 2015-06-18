 /*!
@File           pvr_gputrace.c
@Title          PVR GPU Trace module Linux implementation
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

#include "pvrsrv_error.h"
#include "srvkm.h"
#include "pvr_debug.h"
#include "pvr_debugfs.h"
#include "pvr_uaccess.h"

#include "pvr_gputrace.h"

#if defined(CONFIG_TRACING) && defined(CONFIG_MTK_SCHED_TRACERS) && defined(MTK_GPU_DVFS)
#include <mach/mt_gpufreq.h>
#include <trace/events/mtk_events.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/gpu.h>

#define KM_FTRACE_NO_PRIORITY (0)



#define PVRSRV_KM_FTRACE_JOB_MAX       (512)
#define PVRSRV_KM_FTRACE_CTX_MAX        (16)

#define PVRSRV_FTRACE_JOB_FLAG_MASK     (0xFF000000)
#define PVRSRV_FTRACE_JOB_ID_MASK       (0x00FFFFFF)
#define PVRSRV_FTRACE_JOB_FLAG_ENQUEUED (0x80000000)

#define PVRSRV_FTRACE_JOB_GET_ID(pa)               ((pa)->ui32FlagsAndID & PVRSRV_FTRACE_JOB_ID_MASK)
#define PVRSRV_FTRACE_JOB_SET_ID_CLR_FLAGS(pa, id) ((pa)->ui32FlagsAndID = PVRSRV_FTRACE_JOB_ID_MASK & (id))

#define PVRSRV_FTRACE_JOB_GET_FLAGS(pa)     ((pa)->ui32FlagsAndID & PVRSRV_FTRACE_JOB_FLAG_MASK)
#define PVRSRV_FTRACE_JOB_SET_FLAGS(pa, fl) ((pa)->ui32FlagsAndID |= PVRSRV_FTRACE_JOB_FLAG_MASK & (fl))

typedef struct _PVRSRV_FTRACE_JOB_
{
	
	IMG_UINT32 ui32FlagsAndID;
	IMG_UINT32 ui32ExtJobRef;
	IMG_UINT32 ui32IntJobRef;
} PVRSRV_FTRACE_GPU_JOB;


typedef struct _PVRSRV_FTRACE_GPU_CTX_
{
	
	IMG_UINT32            ui32PID;

	
	IMG_UINT16            ui16JobWrite;		
	PVRSRV_FTRACE_GPU_JOB asJobs[PVRSRV_KM_FTRACE_JOB_MAX];
} PVRSRV_FTRACE_GPU_CTX;


typedef struct _PVRSRV_FTRACE_GPU_DATA_
{
	IMG_UINT16 ui16CtxWrite;				
	PVRSRV_FTRACE_GPU_CTX asFTraceContext[PVRSRV_KM_FTRACE_CTX_MAX];
} PVRSRV_FTRACE_GPU_DATA;

PVRSRV_FTRACE_GPU_DATA gsFTraceGPUData;

static void CreateJob(IMG_UINT32 ui32PID, IMG_UINT32 ui32ExtJobRef,
		IMG_UINT32 ui32IntJobRef)
{
	PVRSRV_FTRACE_GPU_CTX* psContext = IMG_NULL;
	PVRSRV_FTRACE_GPU_JOB* psJob = IMG_NULL;
	IMG_UINT32 i;

	
	for (i = 0; i < PVRSRV_KM_FTRACE_CTX_MAX; ++i)
	{
		if(gsFTraceGPUData.asFTraceContext[i].ui32PID == ui32PID)
		{
			psContext = &(gsFTraceGPUData.asFTraceContext[i]);
			break;
		} 
	}

	
	if (psContext == NULL)
	{
		i = gsFTraceGPUData.ui16CtxWrite;

		gsFTraceGPUData.asFTraceContext[i].ui32PID = ui32PID;
		gsFTraceGPUData.asFTraceContext[i].ui16JobWrite = 0;
		psContext = &(gsFTraceGPUData.asFTraceContext[i]);

		
		gsFTraceGPUData.ui16CtxWrite = (i+1) & (PVRSRV_KM_FTRACE_CTX_MAX-1);
	}

	psJob = &(psContext->asJobs[psContext->ui16JobWrite]);
	PVRSRV_FTRACE_JOB_SET_ID_CLR_FLAGS(psJob, 1001+psContext->ui16JobWrite);
	psJob->ui32ExtJobRef = ui32ExtJobRef;
	psJob->ui32IntJobRef = ui32IntJobRef;

	psContext->ui16JobWrite = (psContext->ui16JobWrite + 1) & (PVRSRV_KM_FTRACE_JOB_MAX-1);
}


static PVRSRV_ERROR GetCtxAndJobID(IMG_UINT32 ui32PID,
	IMG_UINT32 ui32ExtJobRef, IMG_UINT32 ui32IntJobRef,
	IMG_UINT32 *pui32CtxID, PVRSRV_FTRACE_GPU_JOB** ppsJob)
{
	PVRSRV_FTRACE_GPU_CTX* psContext = IMG_NULL;
	IMG_UINT32 i;

	
	for (i = 0; i < PVRSRV_KM_FTRACE_CTX_MAX; ++i)
	{
		if(gsFTraceGPUData.asFTraceContext[i].ui32PID == ui32PID)
		{
			psContext = &(gsFTraceGPUData.asFTraceContext[i]);
			
			*pui32CtxID = 101+i;
			break;
		}
	}

	
	if (psContext == NULL)
	{
		PVR_DPF((PVR_DBG_MESSAGE,"GetCtxAndJobID: Failed to find context ID for PID %d", ui32PID));
		return PVRSRV_ERROR_PROCESS_NOT_FOUND;
	}

	
	for(i = 0; i < PVRSRV_KM_FTRACE_JOB_MAX; ++i)
	{
		if((psContext->asJobs[i].ui32ExtJobRef == ui32ExtJobRef) &&
			(psContext->asJobs[i].ui32IntJobRef == ui32IntJobRef))
		{
			
			*ppsJob = &psContext->asJobs[i];
			return PVRSRV_OK;
		}
	}

	PVR_DPF((PVR_DBG_MESSAGE,"GetCtxAndJobID: Failed to find job ID for extJobRef %d, intJobRef %x", ui32ExtJobRef, ui32IntJobRef));
	return PVRSRV_ERROR_NOT_FOUND;
}


static struct dentry* gpsPVRDebugFSGpuTracingOnEntry = NULL;



static void *GpuTracingSeqStart(struct seq_file *psSeqFile, loff_t *puiPosition)
{
	if (*puiPosition == 0)
	{
		
		return (void*)1;
	}

	return NULL;
}


static void GpuTracingSeqStop(struct seq_file *psSeqFile, void *pvData)
{
	PVR_UNREFERENCED_PARAMETER(psSeqFile);
}


static void *GpuTracingSeqNext(struct seq_file *psSeqFile, void *pvData, loff_t *puiPosition)
{
	PVR_UNREFERENCED_PARAMETER(psSeqFile);
	return NULL;
}


static int GpuTracingSeqShow(struct seq_file *psSeqFile, void *pvData)
{
	IMG_BOOL bValue = PVRGpuTraceEnabled();

	PVR_UNREFERENCED_PARAMETER(pvData);

	seq_puts(psSeqFile, (bValue ? "Y\n" : "N\n"));
	return 0;
}


static struct seq_operations gsGpuTracingReadOps =
{
	.start = GpuTracingSeqStart,
	.stop  = GpuTracingSeqStop,
	.next  = GpuTracingSeqNext,
	.show  = GpuTracingSeqShow,
};


static IMG_INT GpuTracingSet(const IMG_CHAR *buffer, size_t count, loff_t uiPosition, void *data)
{
	IMG_CHAR cFirstChar;

	PVR_UNREFERENCED_PARAMETER(uiPosition);
	PVR_UNREFERENCED_PARAMETER(data);

	if (!count)
	{
		return -EINVAL;
	}

	if (pvr_copy_from_user(&cFirstChar, buffer, 1))
	{
		return -EFAULT;
	}

	switch (cFirstChar)
	{
		case '0':
		case 'n':
		case 'N':
		{
			PVRGpuTraceEnabledSet(IMG_FALSE);
			PVR_TRACE(("DISABLED GPU FTrace"));
			break;
		}
		case '1':
		case 'y':
		case 'Y':
		{
			PVRGpuTraceEnabledSet(IMG_TRUE);
			PVR_TRACE(("ENABLED GPU FTrace"));
			break;
		}
	}

	return count;
}




void PVRGpuTraceClientWork(
		const IMG_UINT32 ui32Pid,
		const IMG_UINT32 ui32ExtJobRef,
		const IMG_UINT32 ui32IntJobRef,
		const IMG_CHAR* pszKickType)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_FTRACE_GPU_JOB* psJob;
	IMG_UINT32   ui32CtxId = 0;

	PVR_ASSERT(pszKickType);

	PVR_DPF((PVR_DBG_VERBOSE, "PVRGpuTraceClientKick(%s): PID %u, extJobRef %u, intJobRef %u", pszKickType, ui32Pid, ui32ExtJobRef, ui32IntJobRef));

	CreateJob(ui32Pid, ui32ExtJobRef, ui32IntJobRef);

	if (PVRGpuTraceEnabled())
	{
		eError = GetCtxAndJobID(ui32Pid, ui32ExtJobRef, ui32IntJobRef, &ui32CtxId,  &psJob);
		PVR_LOGRN_IF_ERROR(eError, "GetCtxAndJobID");

#if defined(CONFIG_TRACING) && defined(CONFIG_MTK_SCHED_TRACERS) && defined(MTK_GPU_DVFS)
		{
			unsigned int ui32CurFreqID = mt_gpufreq_get_cur_freq_index();
			unsigned int ui32GPUFreq = mt_gpufreq_get_frequency_by_level(ui32CurFreqID);
			trace_gpu_freq(ui32GPUFreq);
		}
#endif

		trace_gpu_job_enqueue(ui32CtxId, PVRSRV_FTRACE_JOB_GET_ID(psJob), pszKickType);

		PVRSRV_FTRACE_JOB_SET_FLAGS(psJob, PVRSRV_FTRACE_JOB_FLAG_ENQUEUED);
	}
}


void PVRGpuTraceWorkSwitch(
		IMG_UINT64 ui64HWTimestampInOSTime,
		const IMG_UINT32 ui32Pid,
		const IMG_UINT32 ui32ExtJobRef,
		const IMG_UINT32 ui32IntJobRef,
		const IMG_CHAR* pszWorkType,
		PVR_GPUTRACE_SWITCH_TYPE eSwType)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
    PVRSRV_FTRACE_GPU_JOB* psJob = IMG_NULL;
	IMG_UINT32 ui32CtxId;

	PVR_ASSERT(pszWorkType);

	eError = GetCtxAndJobID(ui32Pid, ui32ExtJobRef, ui32IntJobRef,	&ui32CtxId,  &psJob);
	PVR_LOGRN_IF_ERROR(eError, "GetCtxAndJobID");

	PVR_ASSERT(psJob);

	if (PVRSRV_FTRACE_JOB_GET_FLAGS(psJob) & PVRSRV_FTRACE_JOB_FLAG_ENQUEUED)
	{
		if (eSwType == PVR_GPUTRACE_SWITCH_TYPE_END)
		{
			ui32CtxId = 0;
		}

		trace_gpu_sched_switch(pszWorkType, ui64HWTimestampInOSTime,
				ui32CtxId, KM_FTRACE_NO_PRIORITY, PVRSRV_FTRACE_JOB_GET_ID(psJob));
	}
}


PVRSRV_ERROR PVRGpuTraceInit(void)
{
	void * psPVRDebugFSGpuTracingOnEntry = (void *)gpsPVRDebugFSGpuTracingOnEntry;
	return PVRDebugFSCreateEntry("gpu_tracing_on",
				      NULL,
				      &gsGpuTracingReadOps,
				      (PVRSRV_ENTRY_WRITE_FUNC *)GpuTracingSet,
				      NULL,
				      &psPVRDebugFSGpuTracingOnEntry);
}


void PVRGpuTraceDeInit(void)
{
	
	if (gpsPVRDebugFSGpuTracingOnEntry)
	{
		PVRDebugFSRemoveEntry(gpsPVRDebugFSGpuTracingOnEntry);
		gpsPVRDebugFSGpuTracingOnEntry = NULL;
	}
}


