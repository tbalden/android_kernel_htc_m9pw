#ifndef __CMDQ_CORE_H__
#define __CMDQ_CORE_H__

#include <linux/list.h>
#include <linux/time.h>
#include <linux/xlog.h>
#include <linux/aee.h>
#include <linux/device.h>
#include <linux/printk.h>
#include "cmdq_def.h"

#define CMDQ_LOG(string, args...) \
    if (1) \
    { \
	pr_err("[CMDQ]"string, ##args); \
    }

#define CMDQ_MSG(string, args...) \
    if (cmdq_core_should_print_msg()) \
    { \
	pr_warn("[CMDQ]"string, ##args); \
    }

#define CMDQ_VERBOSE(string, args...) \
    if (cmdq_core_should_print_msg()) \
    { \
	pr_info("[CMDQ]"string, ##args); \
    }

#define CMDQ_ERR(string, args...) \
    if (1) \
    { \
	pr_err("[CMDQ][ERR]"string, ##args); \
    }

#define CMDQ_AEE(tag, string, args...) \
    do { \
	char dispatchedTag[50];\
	snprintf(dispatchedTag, 50, "CRDISPATCH_KEY:%s", tag); \
	pr_err("[CMDQ][AEE]"string, ##args); \
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT | DB_OPT_PROC_CMDQ_INFO | DB_OPT_MMPROFILE_BUFFER, dispatchedTag, "error: "string, ##args); \
    } while (0)

#define CMDQ_PROFILE

typedef unsigned long long CMDQ_TIME;

#ifdef CMDQ_PROFILE
#define CMDQ_PROF_INIT() do {if (0 < cmdq_core_profile_enabled()) {met_tag_init(); } } while (0)
#define CMDQ_PROF_START(args...) do {if (0 < cmdq_core_profile_enabled()) {met_tag_start(args); } } while (0)
#define CMDQ_PROF_END(args...) do {if (0 < cmdq_core_profile_enabled()) {met_tag_end(args); } } while (0)
#define CMDQ_PROF_ONESHOT(args...) do {if (0 < cmdq_core_profile_enabled()) {met_tag_oneshot(args); } } while (0)
#define CMDQ_PROF_MMP(args...) do {if (1) {MMProfileLogEx(args); } } while (0)
#else
#define CMDQ_PROF_INIT()
#define CMDQ_PROF_START(args...)
#define CMDQ_PROF_END(args...)
#define CMDQ_PROF_ONESHOT(args...)
#define CMDQ_PROF_MMP(args...)
#endif

#define CMDQ_GET_TIME_IN_MS(start,                                                  \
			    end,                                                    \
			    duration)                                               \
{                                                                                   \
    CMDQ_TIME _duration = end - start;						    \
    do_div(_duration, 1000000);							    \
    duration = (int32_t)_duration;						    \
}

#define CMDQ_GET_TIME_IN_US_PART(start,                                             \
			    end,                                                    \
			    duration)                                               \
{                                                                                   \
    CMDQ_TIME _duration = end - start;						    \
    do_div(_duration, 1000);							    \
    duration = (int32_t)_duration;						    \
}

#define CMDQ_PHYS_TO_AREG(addr) ((addr) & 0xFFFFFFFF)	
#define CMDQ_AREG_TO_PHYS(addr) ((addr) | 0L)

#define CMDQ_ENG_ISP_GROUP_BITS                 ((1LL << CMDQ_ENG_ISP_IMGI) |       \
						 (1LL << CMDQ_ENG_ISP_IMGO) |       \
						 (1LL << CMDQ_ENG_ISP_IMG2O))

#define CMDQ_ENG_MDP_GROUP_BITS                 ((1LL << CMDQ_ENG_MDP_CAMIN) |      \
						 (1LL << CMDQ_ENG_MDP_RDMA0) |      \
						 (1LL << CMDQ_ENG_MDP_RDMA1) |      \
						 (1LL << CMDQ_ENG_MDP_RSZ0) |       \
						 (1LL << CMDQ_ENG_MDP_RSZ1) |       \
						 (1LL << CMDQ_ENG_MDP_RSZ2) |       \
						 (1LL << CMDQ_ENG_MDP_TDSHP0) |     \
						 (1LL << CMDQ_ENG_MDP_TDSHP1) |     \
						 (1LL << CMDQ_ENG_MDP_WROT0) |      \
						 (1LL << CMDQ_ENG_MDP_WROT1) |      \
						 (1LL << CMDQ_ENG_MDP_WDMA))

#define CMDQ_ENG_DISP_GROUP_BITS                ((1LL << CMDQ_ENG_DISP_UFOE) |      \
						 (1LL << CMDQ_ENG_DISP_AAL) |       \
						 (1LL << CMDQ_ENG_DISP_COLOR0) |    \
						 (1LL << CMDQ_ENG_DISP_COLOR1) |    \
						 (1LL << CMDQ_ENG_DISP_RDMA0) |     \
						 (1LL << CMDQ_ENG_DISP_RDMA1) |     \
						 (1LL << CMDQ_ENG_DISP_RDMA2) |     \
						 (1LL << CMDQ_ENG_DISP_WDMA0) |     \
						 (1LL << CMDQ_ENG_DISP_WDMA1) |     \
						 (1LL << CMDQ_ENG_DISP_OVL0) |      \
						 (1LL << CMDQ_ENG_DISP_OVL1) |      \
						 (1LL << CMDQ_ENG_DISP_GAMMA) |     \
						 (1LL << CMDQ_ENG_DISP_MERGE) |     \
						 (1LL << CMDQ_ENG_DISP_SPLIT0) |    \
						 (1LL << CMDQ_ENG_DISP_SPLIT1) |    \
						 (1LL << CMDQ_ENG_DISP_DSI0_VDO) |  \
						 (1LL << CMDQ_ENG_DISP_DSI1_VDO) |  \
						 (1LL << CMDQ_ENG_DISP_DSI0_CMD) |  \
						 (1LL << CMDQ_ENG_DISP_DSI1_CMD) |  \
						 (1LL << CMDQ_ENG_DISP_DSI0) |      \
						 (1LL << CMDQ_ENG_DISP_DSI1) |      \
						 (1LL << CMDQ_ENG_DISP_DPI))

#define CMDQ_ENG_VENC_GROUP_BITS                ((1LL << CMDQ_ENG_VIDEO_ENC))

#define CMDQ_ENG_JPEG_GROUP_BITS                ((1LL << CMDQ_ENG_JPEG_ENC) | \
						 (1LL << CMDQ_ENG_JPEG_REMDC) | \
						 (1LL << CMDQ_ENG_JPEG_DEC))

#define CMDQ_ENG_ISP_GROUP_FLAG(flag)   ((flag) & (CMDQ_ENG_ISP_GROUP_BITS))

#define CMDQ_ENG_MDP_GROUP_FLAG(flag)   ((flag) & (CMDQ_ENG_MDP_GROUP_BITS))

#define CMDQ_ENG_DISP_GROUP_FLAG(flag)  ((flag) & (CMDQ_ENG_DISP_GROUP_BITS))

#define CMDQ_ENG_JPEG_GROUP_FLAG(flag)  ((flag) & (CMDQ_ENG_JPEG_GROUP_BITS))

#define CMDQ_ENG_VENC_GROUP_FLAG(flag)  ((flag) & (CMDQ_ENG_VENC_GROUP_BITS))

#define GENERATE_ENUM(_enum, _string) _enum,
#define GENERATE_STRING(_enum, _string) #_string,

#define CMDQ_FOREACH_GROUP(ACTION)\
    ACTION(CMDQ_GROUP_ISP, ISP) \
    ACTION(CMDQ_GROUP_MDP, MDP) \
    ACTION(CMDQ_GROUP_DISP, DISP) \
    ACTION(CMDQ_GROUP_JPEG, JPEG) \
    ACTION(CMDQ_GROUP_VENC, VENC)

typedef enum CMDQ_GROUP_ENUM {
	CMDQ_FOREACH_GROUP(GENERATE_ENUM)
	    CMDQ_MAX_GROUP_COUNT,	
} CMDQ_GROUP_ENUM;

typedef int32_t(*CmdqClockOnCB) (uint64_t engineFlag);

typedef int32_t(*CmdqDumpInfoCB) (uint64_t engineFlag, int level);

typedef int32_t(*CmdqResetEngCB) (uint64_t engineFlag);

typedef int32_t(*CmdqClockOffCB) (uint64_t engineFlag);

typedef int32_t(*CmdqInterruptCB) (unsigned long data);

typedef int32_t(*CmdqAsyncFlushCB) (unsigned long data);

typedef int32_t(*CmdqDebugRegDumpBeginCB) (uint32_t taskID, uint32_t *regCount,
					   uint32_t **regAddress);
typedef int32_t(*CmdqDebugRegDumpEndCB) (uint32_t taskID, uint32_t regCount, uint32_t *regValues);

typedef struct CmdqCBkStruct {
	CmdqClockOnCB clockOn;
	CmdqDumpInfoCB dumpInfo;
	CmdqResetEngCB resetEng;
	CmdqClockOffCB clockOff;
} CmdqCBkStruct;

typedef struct CmdqDebugCBkStruct {
	
	CmdqDebugRegDumpBeginCB beginDebugRegDump;
	CmdqDebugRegDumpEndCB endDebugRegDump;
} CmdqDebugCBkStruct;

typedef enum CMDQ_CODE_ENUM {
	
	CMDQ_CODE_READ = 0x01,
	CMDQ_CODE_MOVE = 0x02,
	CMDQ_CODE_WRITE = 0x04,
	CMDQ_CODE_POLL = 0x08,
	CMDQ_CODE_JUMP = 0x10,
	CMDQ_CODE_WFE = 0x20,	
	CMDQ_CODE_EOC = 0x40,	

	
	
	CMDQ_CODE_SET_TOKEN = 0x21,	
	CMDQ_CODE_WAIT_NO_CLEAR = 0x22,	
	CMDQ_CODE_CLEAR_TOKEN = 0x23,	
	CMDQ_CODE_RAW = 0x24,	
} CMDQ_CODE_ENUM;

typedef enum TASK_STATE_ENUM {
	TASK_STATE_IDLE,	
	TASK_STATE_BUSY,	
	TASK_STATE_KILLED,	
	TASK_STATE_ERROR,	
	TASK_STATE_DONE,	
	TASK_STATE_WAITING,	
} TASK_STATE_ENUM;

typedef struct TaskStruct {
	struct list_head listEntry;

	
	TASK_STATE_ENUM taskState;	
	uint32_t *pVABase;	
	dma_addr_t MVABase;	
	uint32_t bufferSize;	
	bool useEmergencyBuf;	

	
	int32_t scenario;
	int32_t priority;
	uint64_t engineFlag;
	int32_t commandSize;
	uint32_t *pCMDEnd;
	int32_t reorder;
	int32_t thread;		
	int32_t irqFlag;	
	CmdqInterruptCB loopCallback;	
	unsigned long loopData;	
	CmdqAsyncFlushCB flushCallback;	
	unsigned long flushData;	
	struct work_struct autoReleaseWork;	

	
	uint32_t regCount;
	uint32_t *regResults;
	dma_addr_t regResultsMVA;

	
	uint32_t regCountUserSpace;	
	uint32_t regUserToken;	

	
	cmdqSecDataStruct secData;

	
	CMDQ_TIME submit;	
	CMDQ_TIME trigger;
	CMDQ_TIME beginWait;
	CMDQ_TIME gotIRQ;
	CMDQ_TIME wakedUp;
	CMDQ_TIME entrySec; 
	CMDQ_TIME exitSec; 

	uint32_t *profileData;	
	dma_addr_t profileDataPA;

	void *privateData;	

	pid_t callerPid;
	char callerName[TASK_COMM_LEN];

	
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	cmdqProfileMarkerStruct profileMarker;
#endif
} TaskStruct;

typedef struct EngineStruct {
	int32_t userCount;
	int32_t currOwner;
	int32_t resetCount;
	int32_t failCount;
} EngineStruct;

typedef struct ThreadStruct {
	uint32_t taskCount;
	uint32_t waitCookie;
	uint32_t nextCookie;
	CmdqInterruptCB loopCallback;	
	unsigned long loopData;	
	TaskStruct * pCurTask[CMDQ_MAX_TASK_IN_THREAD];

	
	
	
	
	uint32_t allowDispatching;
} ThreadStruct;

typedef struct RecordStruct {
	pid_t user;		
	int32_t scenario;	
	int32_t priority;	
	int32_t thread;		
	int32_t reorder;
	int32_t size;
	uint32_t writeTimeNS;	
	uint64_t engineFlag;	

	bool isSecure; 

	CMDQ_TIME submit;	
	CMDQ_TIME trigger;	
	CMDQ_TIME beginWait;	
	CMDQ_TIME gotIRQ;	
	CMDQ_TIME wakedUp;	
	CMDQ_TIME done;	

	unsigned long long writeTimeNSBegin;
	unsigned long long writeTimeNSEnd;

	
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	uint32_t profileMarkerCount;
	unsigned long long profileMarkerTimeNS[CMDQ_MAX_PROFILE_MARKER_IN_TASK];
	const char *profileMarkerTag[CMDQ_MAX_PROFILE_MARKER_IN_TASK];
#endif
} RecordStruct;

typedef struct ErrorStruct {
	RecordStruct errorRec;	
	u64 ts_nsec;		
} ErrorStruct;

typedef struct WriteAddrStruct {
	struct list_head list_node;
	uint32_t count;
	void *va;
	dma_addr_t pa;
	pid_t user;
} WriteAddrStruct;

typedef struct EmergencyBufferStruct {
	bool used;
	uint32_t size;
	void *va;
	dma_addr_t pa;
} EmergencyBufferStruct;

typedef struct cmdqSecSharedMemoryStruct {
	void *pVABase;	
	dma_addr_t MVABase;	
	uint32_t size; 
} cmdqSecSharedMemoryStruct, *cmdqSecSharedMemoryHandle;

typedef struct ContextStruct {
	
	struct kmem_cache *taskCache;	
	struct list_head taskFreeList;	
	struct list_head taskActiveList;	
	struct list_head taskWaitList;	
	struct work_struct taskConsumeWaitQueueItem;
	struct workqueue_struct *taskAutoReleaseWQ;	
	struct workqueue_struct *taskConsumeWQ;	

	
	struct list_head writeAddrList;

	
	EngineStruct engine[CMDQ_MAX_ENGINE_COUNT];
	ThreadStruct thread[CMDQ_MAX_THREAD_COUNT];

	
	cmdqSecSharedMemoryHandle hSecSharedMem;
	void* hNotifyLoop;

	
	int32_t enableProfile;
	int32_t lastID;
	int32_t recNum;
	RecordStruct record[CMDQ_MAX_RECORD_COUNT];

	
	int32_t logLevel;
	int32_t errNum;
	ErrorStruct error[CMDQ_MAX_ERROR_COUNT];
} ContextStruct;


#ifdef __cplusplus
extern "C" {
#endif


	int32_t cmdqCoreInitialize(void);

	void cmdqCoreInitGroupCB(void);
	void cmdqCoreDeinitGroupCB(void);

	int32_t cmdqCoreRegisterCB(CMDQ_GROUP_ENUM engGroup,
				   CmdqClockOnCB clockOn,
				   CmdqDumpInfoCB dumpInfo,
				   CmdqResetEngCB resetEng, CmdqClockOffCB clockOff);

	int32_t cmdqCoreRegisterDebugRegDumpCB(CmdqDebugRegDumpBeginCB beginCB,
					       CmdqDebugRegDumpEndCB endCB);

	int32_t cmdqCoreSuspend(void);

	int32_t cmdqCoreResume(void);

	int32_t cmdqCoreResumedNotifier(void);

	void cmdqCoreHandleIRQ(int32_t index);

	int32_t cmdqCoreSubmitTaskAsync(cmdqCommandStruct *pCommandDesc,
					CmdqInterruptCB loopCB,
					unsigned long loopData, TaskStruct **ppTaskOut);

	int32_t cmdqCoreWaitAndReleaseTask(TaskStruct *pTask, long timeout_jiffies);


	int32_t cmdqCoreWaitResultAndReleaseTask(TaskStruct *pTask, cmdqRegValueStruct *pResult,
						 long timeout_jiffies);


	int32_t cmdqCoreReleaseTask(TaskStruct *pTask);

	int32_t cmdqCoreAutoReleaseTask(TaskStruct *pTask);

	int32_t cmdqCoreSubmitTask(cmdqCommandStruct *pCommandDesc);


	bool cmdqIsValidTaskPtr(void *pTask);


void cmdqCoreClearEvent(CMDQ_EVENT_ENUM event);

	void cmdqCoreSetEvent(CMDQ_EVENT_ENUM event);

	uint32_t cmdqCoreGetEvent(CMDQ_EVENT_ENUM event);

	int cmdqCoreAllocWriteAddress(uint32_t count, dma_addr_t *paStart);
	int cmdqCoreFreeWriteAddress(dma_addr_t paStart);
	uint32_t cmdqCoreReadWriteAddress(dma_addr_t pa);
	uint32_t cmdqCoreWriteWriteAddress(dma_addr_t pa, uint32_t value);

	void cmdqCoreDeInitialize(void);

	void *cmdq_core_alloc_hw_buffer(struct device *dev, size_t size, dma_addr_t *dma_handle, const gfp_t flag);
	void cmdq_core_free_hw_buffer(struct device *dev, size_t size, void *cpu_addr, dma_addr_t dma_handle);

	int32_t cmdq_core_set_secure_thread_exec_counter(const int32_t thread, const uint32_t cookie);

	int32_t cmdq_core_get_secure_thread_exec_counter(const int32_t thread);

	cmdqSecSharedMemoryHandle cmdq_core_get_secure_shared_memory(void);

	ssize_t cmdqCorePrintRecord(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCorePrintError(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCorePrintStatus(struct device *dev, struct device_attribute *attr, char *buf);

	ssize_t cmdqCorePrintLogLevel(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t cmdqCoreWriteLogLevel(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t size);

	ssize_t cmdqCorePrintProfileEnable(struct device *dev, struct device_attribute *attr,
					   char *buf);
	ssize_t cmdqCoreWriteProfileEnable(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size);

	int32_t cmdqCoreDebugRegDumpBegin(uint32_t taskID, uint32_t *regCount,
					  uint32_t **regAddress);
	int32_t cmdqCoreDebugRegDumpEnd(uint32_t taskID, uint32_t regCount, uint32_t *regValues);
	int32_t cmdqCoreDebugDumpCommand(TaskStruct *pTask);
	int32_t cmdqCoreQueryUsage(int32_t *pCount);

	int cmdqCorePrintRecordSeq(struct seq_file *m, void *v);
	int cmdqCorePrintErrorSeq(struct seq_file *m, void *v);
	int cmdqCorePrintStatusSeq(struct seq_file *m, void *v);

	int32_t cmdq_subsys_from_phys_addr(uint32_t physAddr);
	unsigned long long cmdq_core_get_GPR64(const CMDQ_DATA_REGISTER_ENUM regID);
	void cmdq_core_set_GPR64(const CMDQ_DATA_REGISTER_ENUM regID, const unsigned long long value);
	uint32_t cmdqCoreReadDataRegister(CMDQ_DATA_REGISTER_ENUM regID);
	void cmdq_core_set_log_level(const int32_t value);
	bool cmdq_core_should_print_msg(void);
	int32_t cmdq_core_profile_enabled(void);

	int32_t cmdq_core_enable_emergency_buffer_test(const bool enable);

	int32_t cmdq_core_parse_instruction(const uint32_t *pCmd, char *textBuf, int bufLen);
	void cmdq_core_add_consume_task(void);

	void cmdq_core_release_task_by_file_node(void *file_node);

	void cmdq_core_dump_GIC(void);

#ifdef __cplusplus
}
#endif
#endif				
