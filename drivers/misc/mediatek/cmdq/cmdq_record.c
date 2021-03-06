#include <linux/slab.h>
#include <linux/errno.h>
#include <mach/memory.h>

#include "cmdq_record.h"
#include "cmdq_core.h"
#include "cmdq_reg.h"
#include "cmdq_platform.h"
#include "cmdq_prof.h"

#ifdef _MTK_USER_
#define DISABLE_LOOP_IRQ
#endif

int32_t cmdq_rec_realloc_addr_metadata_buffer(cmdqRecHandle handle, const uint32_t size)
{
	void *pNewBuf = NULL;
	void *pOriginalBuf = (void*)CMDQ_U32_PTR(handle->secData.addrMetadatas);
	const uint32_t originalSize = sizeof(cmdqSecAddrMetadataStruct) * (handle->secData.addrMetadataMaxCount);

	if (size <= originalSize) {
		return 0;
	}

	pNewBuf = kzalloc(size, GFP_KERNEL);
	if (NULL == pNewBuf) {
		CMDQ_ERR("REC: secAddrMetadata, kzalloc %d bytes addr_metadata buffer failed\n", size);
		return -ENOMEM;
	}

	if (pOriginalBuf && originalSize > 0) {
		memcpy(pNewBuf, pOriginalBuf, originalSize);
	}

	CMDQ_VERBOSE("REC: secAddrMetadata, realloc size from %d to %d bytes\n", originalSize, size);
	kfree(pOriginalBuf);
	handle->secData.addrMetadatas = (cmdqU32Ptr_t)(unsigned long)(pNewBuf);
	handle->secData.addrMetadataMaxCount = size / sizeof(cmdqSecAddrMetadataStruct);

	return 0;
}

int cmdq_rec_realloc_cmd_buffer(cmdqRecHandle handle, uint32_t size)
{
	void *pNewBuf = NULL;

	if (size <= handle->bufferSize) {
		return 0;
	}

	pNewBuf = kzalloc(size, GFP_KERNEL);

	if (NULL == pNewBuf) {
		CMDQ_ERR("REC: kzalloc %d bytes cmd_buffer failed\n", size);
		return -ENOMEM;
	}

	memset(pNewBuf, 0, size);

	if (handle->pBuffer && handle->blockSize > 0) {
		memcpy(pNewBuf, handle->pBuffer, handle->blockSize);
	}

	CMDQ_VERBOSE("REC: realloc size from %d to %d bytes\n", handle->bufferSize, size);

	kfree(handle->pBuffer);
	handle->pBuffer = pNewBuf;
	handle->bufferSize = size;

	return 0;
}

static int32_t cmdq_rec_reset_profile_maker_data(cmdqRecHandle handle)
{
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	int32_t i = 0;

	if (NULL == handle) {
		return -EFAULT;
	}

	handle->profileMarker.count = 0;
	handle->profileMarker.hSlot = 0LL;

	for (i = 0; i < CMDQ_MAX_PROFILE_MARKER_IN_TASK; i++) {
		handle->profileMarker.tag[i] = NULL;
	}

	return 0;
#endif
	return 0;
}

int32_t cmdqRecCreate(CMDQ_SCENARIO_ENUM scenario, cmdqRecHandle *pHandle)
{
	cmdqRecHandle handle = NULL;

	if (scenario < 0 || scenario >= CMDQ_MAX_SCENARIO_COUNT) {
		CMDQ_ERR("Unknown scenario type %d\n", scenario);
		return -EINVAL;
	}

	handle = kzalloc(sizeof(uint8_t *) * sizeof(cmdqRecStruct), GFP_KERNEL);
	if (NULL == handle) {
		return -ENOMEM;
	}

	handle->scenario = scenario;
	handle->pBuffer = NULL;
	handle->bufferSize = 0;
	handle->blockSize = 0;
	handle->engineFlag = cmdq_rec_flag_from_scenario(scenario);
	handle->priority = CMDQ_THR_PRIO_NORMAL;
	handle->prefetchCount = 0;
	handle->finalized = false;
	handle->pRunningTask = NULL;

	
	handle->secData.isSecure = false;
	handle->secData.enginesNeedDAPC = 0LL;
	handle->secData.enginesNeedPortSecurity = 0LL;
	handle->secData.addrMetadatas = (cmdqU32Ptr_t)(unsigned long)NULL;
	handle->secData.addrMetadataMaxCount = 0;
	handle->secData.addrMetadataCount = 0;

	
	cmdq_rec_reset_profile_maker_data(handle);

	
	if (0 != cmdq_rec_realloc_cmd_buffer(handle, CMDQ_INITIAL_CMD_BLOCK_SIZE)) {
		kfree(handle);
		return -ENOMEM;
	}

	*pHandle = handle;

	return 0;
}

int32_t cmdq_append_addr_metadata(cmdqRecHandle handle, const cmdqSecAddrMetadataStruct *pMetadata)
{
	cmdqSecAddrMetadataStruct *pAddrs;
	int32_t status;
	
	const uint32_t index = handle->secData.addrMetadataCount;

	pAddrs = NULL;
	status = 0;

	if (0 >= handle->secData.addrMetadataMaxCount) {
		
		status = cmdq_rec_realloc_addr_metadata_buffer(handle,
					sizeof(cmdqSecAddrMetadataStruct) * 8);
	}
	else if (handle->secData.addrMetadataCount >= (handle->secData.addrMetadataMaxCount)) {
		
		status = cmdq_rec_realloc_addr_metadata_buffer(handle,
					sizeof(cmdqSecAddrMetadataStruct) * (handle->secData.addrMetadataMaxCount) * 2);
	}

	if(0 > status) {
		return -ENOMEM;
	}

	pAddrs = (cmdqSecAddrMetadataStruct*)(CMDQ_U32_PTR(handle->secData.addrMetadatas));
	
	pAddrs[index].instrIndex = pMetadata->instrIndex;
	pAddrs[index].baseHandle= pMetadata->baseHandle;
	pAddrs[index].offset = pMetadata->offset;
	pAddrs[index].size = pMetadata->size;
	pAddrs[index].port = pMetadata->port;
	pAddrs[index].type = pMetadata->type;

	
	handle->secData.addrMetadataCount += 1;
	return 0;
}

int32_t cmdq_append_command(cmdqRecHandle handle, CMDQ_CODE_ENUM code, uint32_t argA, uint32_t argB)
{
	int32_t subsys;
	uint32_t *pCommand;

	
	const uint32_t subsysBit = cmdq_core_get_subsys_LSB_in_argA();

	pCommand = (uint32_t *) ((uint8_t *) handle->pBuffer + handle->blockSize);

	if (handle->finalized) {
		CMDQ_ERR("Already finalized record 0x%p, cannot add more command", handle);
		return -EBUSY;
	}

	
	
	if ((handle->blockSize + 32) >= handle->bufferSize) {
		if (0 != cmdq_rec_realloc_cmd_buffer(handle, handle->bufferSize * 2)) {
			return -ENOMEM;
		}
	}

	
	
	if (CMDQ_CODE_EOC != code && cmdq_core_should_enable_prefetch(handle->scenario)) {
		if (handle->prefetchCount >= CMDQ_MAX_PREFETCH_INSTUCTION) {
			CMDQ_ERR("prefetchCount(%d) > MAX_PREFETCH_INSTUCTION, force insert disable prefetch marker\n",
						handle->prefetchCount);
			
			cmdqRecDisablePrefetch(handle);
			
			cmdqRecMark(handle);
		} else {
			
			if (1 <= handle->prefetchCount) {
				++handle->prefetchCount;
				CMDQ_VERBOSE("handle->prefetchCount: %d, %s, %d\n", handle->prefetchCount, __func__, __LINE__);
			}
		}
	}

	
	pCommand = (uint32_t *) ((uint8_t *) handle->pBuffer + handle->blockSize);

	CMDQ_VERBOSE("REC: 0x%p CMD: 0x%p, op: 0x%02x, argA: 0x%08x, argB: 0x%08x\n", handle,
		     pCommand, code, argA, argB);

	switch (code) {
	case CMDQ_CODE_READ:
		
		subsys = cmdq_subsys_from_phys_addr(argA);
		
		
		*pCommand++ = argB;
		*pCommand++ =
		    (CMDQ_CODE_READ << 24) | (argA & 0xffff) | ((subsys & 0x1f) << subsysBit) | (2 << 21);
		break;
	case CMDQ_CODE_MOVE:
		*pCommand++ = argB;
		*pCommand++ = CMDQ_CODE_MOVE << 24 | (argA & 0xffffff);
		break;
	case CMDQ_CODE_WRITE:
		subsys = cmdq_subsys_from_phys_addr(argA);
		if (-1 == subsys) {
			CMDQ_ERR("REC: Unsupported memory base address 0x%08x\n", argA);
			return -EFAULT;
		}

		*pCommand++ = argB;
		*pCommand++ = (CMDQ_CODE_WRITE << 24) | (argA & 0x0FFFF) | ((subsys & 0x01F) << subsysBit);
		break;
	case CMDQ_CODE_POLL:
		subsys = cmdq_subsys_from_phys_addr(argA);
		if (-1 == subsys) {
			CMDQ_ERR("REC: Unsupported memory base address 0x%08x\n", argA);
			return -EFAULT;
		}
		*pCommand++ = argB;
		*pCommand++ = (CMDQ_CODE_POLL << 24) | (argA & 0x0FFFF) | ((subsys & 0x01F) << subsysBit);
		break;
	case CMDQ_CODE_JUMP:
		*pCommand++ = argB;
		*pCommand++ = (CMDQ_CODE_JUMP << 24) | (argA & 0x0FFFFFF);
		break;
	case CMDQ_CODE_WFE:
		
		
		
		
		*pCommand++ = ((1 << 31) | (1 << 15) | 1);
		*pCommand++ = (CMDQ_CODE_WFE << 24) | argA;
		break;

	case CMDQ_CODE_SET_TOKEN:
		
		
		
		
		
		*pCommand++ = ((1 << 31) | (1 << 16));
		*pCommand++ = (CMDQ_CODE_WFE << 24) | argA;
		break;

	case CMDQ_CODE_WAIT_NO_CLEAR:
		
		
		
		*pCommand++ = ((0 << 31) | (1 << 15) | 1);
		*pCommand++ = (CMDQ_CODE_WFE << 24) | argA;
		break;

	case CMDQ_CODE_CLEAR_TOKEN:
		
		
		
		
		
		*pCommand++ = ((1 << 31) | (0 << 16));
		*pCommand++ = (CMDQ_CODE_WFE << 24) | argA;
		break;

	case CMDQ_CODE_EOC:
		*pCommand++ = argB;
		*pCommand++ = (CMDQ_CODE_EOC << 24) | (argA & 0x0FFFFFF);
		break;

	case CMDQ_CODE_RAW:
		*pCommand++ = argB;
		*pCommand++ = argA;
		break;

	default:
		return -EFAULT;
	}

	handle->blockSize += CMDQ_INST_SIZE;

	return 0;
}

int32_t cmdqRecReset(cmdqRecHandle handle)
{
	if (NULL == handle) {
		return -EFAULT;
	}

	if (NULL != handle->pRunningTask) {
		cmdqRecStopLoop(handle);
	}

	handle->blockSize = 0;
	handle->prefetchCount = 0;
	handle->finalized = false;

	
	handle->secData.isSecure = false;
	handle->secData.enginesNeedDAPC = 0LL;
	handle->secData.enginesNeedPortSecurity = 0LL;
	if (handle->secData.addrMetadatas) {
		kfree(CMDQ_U32_PTR(handle->secData.addrMetadatas));
		handle->secData.addrMetadatas = (cmdqU32Ptr_t)(unsigned long)NULL;
		handle->secData.addrMetadataMaxCount = 0;
		handle->secData.addrMetadataCount = 0;
	}

	
	cmdq_rec_reset_profile_maker_data(handle);

	return 0;
}

int32_t cmdqRecSetSecure(cmdqRecHandle handle, const bool isSecure)
{
	if (false == isSecure) {
		handle->secData.isSecure= isSecure;
		return 0;
	}

#ifdef CMDQ_SECURE_PATH_SUPPORT
	CMDQ_VERBOSE("REC: %p secure:%d\n", handle, isSecure);
	handle->secData.isSecure= isSecure;
	return 0;
#else
	CMDQ_ERR("%s failed since not support secure path\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecSecureEnableDAPC(cmdqRecHandle handle, const uint64_t engineFlag)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	handle->secData.enginesNeedDAPC |= engineFlag;
	return 0;
#else
	CMDQ_ERR("%s failed since not support secure path\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecSecureEnablePortSecurity(cmdqRecHandle handle, const uint64_t engineFlag)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	handle->secData.enginesNeedPortSecurity |= engineFlag;
	return 0;
#else
	CMDQ_ERR("%s failed since not support secure path\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecMark(cmdqRecHandle handle)
{
	int32_t status;

	
	

	
	
	
	
	
	
	
	
	
	status = cmdq_append_command(handle,
				     CMDQ_CODE_EOC,
				     (0x1 << (53 - 32)) | (0x1 << (48 - 32)), 0x00130000);

	
	
	
	handle->prefetchCount = 1;

	if (0 != status) {
		return -EFAULT;
	}

	return 0;
}

int32_t cmdqRecWrite(cmdqRecHandle handle, uint32_t addr, uint32_t value, uint32_t mask)
{
	int32_t status;

	if (0xFFFFFFFF != mask) {
		status = cmdq_append_command(handle, CMDQ_CODE_MOVE, 0, ~mask);
		if (0 != status) {
			return status;
		}

		addr = addr | 0x1;
	}

	status = cmdq_append_command(handle, CMDQ_CODE_WRITE, addr, value);
	if (0 != status) {
		return status;
	}

	return 0;
}

int32_t cmdqRecWriteSecure(cmdqRecHandle handle, uint32_t addr,
			CMDQ_SEC_ADDR_METADATA_TYPE type,
			uint32_t baseHandle,
			uint32_t offset,
			uint32_t size,
			uint32_t port)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	int32_t status;
	int32_t writeInstrIndex;
	cmdqSecAddrMetadataStruct metadata;
	const uint32_t mask = 0xFFFFFFFF;

	
	status = cmdqRecWrite(handle, addr, baseHandle, mask);
	if (0 != status) {
		return status;
	}

	
	writeInstrIndex = (handle->blockSize) / CMDQ_INST_SIZE - 1; 

	memset(&metadata, 0, sizeof(cmdqSecAddrMetadataStruct));
	metadata.instrIndex = writeInstrIndex;
	metadata.type = type;
	metadata.baseHandle = baseHandle;
	metadata.offset = offset;
	metadata.size = size;
	metadata.port = port;

	status = cmdq_append_addr_metadata(handle, &metadata);

	return 0;
#else
	CMDQ_ERR("%s failed since not support secure path\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecPoll(cmdqRecHandle handle, uint32_t addr, uint32_t value, uint32_t mask)
{
	int32_t status;

	status = cmdq_append_command(handle, CMDQ_CODE_MOVE, 0, ~mask);
	if (0 != status) {
		return status;
	}

	status = cmdq_append_command(handle, CMDQ_CODE_POLL, (addr | 0x1), value);
	if (0 != status) {
		return status;
	}

	return 0;
}


int32_t cmdqRecWait(cmdqRecHandle handle, CMDQ_EVENT_ENUM event)
{
	if (CMDQ_SYNC_TOKEN_INVALID == event || CMDQ_SYNC_TOKEN_MAX <= event || 0 > event) {
		return -EINVAL;
	}

	return cmdq_append_command(handle, CMDQ_CODE_WFE, event, 0);
}


int32_t cmdqRecWaitNoClear(cmdqRecHandle handle, CMDQ_EVENT_ENUM event)
{
	if (CMDQ_SYNC_TOKEN_INVALID == event || CMDQ_SYNC_TOKEN_MAX <= event || 0 > event) {
		return -EINVAL;
	}

	return cmdq_append_command(handle, CMDQ_CODE_WAIT_NO_CLEAR, event, 0);
}

int32_t cmdqRecClearEventToken(cmdqRecHandle handle, CMDQ_EVENT_ENUM event)
{
	if (CMDQ_SYNC_TOKEN_INVALID == event || CMDQ_SYNC_TOKEN_MAX <= event || 0 > event) {
		return -EINVAL;
	}

	return cmdq_append_command(handle, CMDQ_CODE_CLEAR_TOKEN, event, 1	
	    );
}


int32_t cmdqRecSetEventToken(cmdqRecHandle handle, CMDQ_EVENT_ENUM event)
{
	if (CMDQ_SYNC_TOKEN_INVALID == event || CMDQ_SYNC_TOKEN_MAX <= event || 0 > event) {
		return -EINVAL;
	}

	return cmdq_append_command(handle, CMDQ_CODE_SET_TOKEN, event, 1	
	    );
}

int32_t cmdqRecReadToDataRegister(cmdqRecHandle handle, uint32_t hwRegAddr,
				  CMDQ_DATA_REGISTER_ENUM dstDataReg)
{
#ifdef CMDQ_GPR_SUPPORT
	
	return cmdq_append_command(handle, CMDQ_CODE_READ, hwRegAddr, dstDataReg);

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecWriteFromDataRegister(cmdqRecHandle handle,
				  CMDQ_DATA_REGISTER_ENUM srcDataReg, uint32_t hwRegAddr)
{
#ifdef CMDQ_GPR_SUPPORT
	const uint32_t subsys = cmdq_subsys_from_phys_addr(hwRegAddr);
	const uint32_t subsysBit = cmdq_core_get_subsys_LSB_in_argA();

	
	return cmdq_append_command(
				handle,
				CMDQ_CODE_RAW,
				((CMDQ_CODE_WRITE << 24) | (hwRegAddr & 0x0FFFF) | ((subsys & 0x01F) << subsysBit) | (2 << 21)),
				srcDataReg);
#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqBackupAllocateSlot(cmdqBackupSlotHandle *phBackupSlot, uint32_t slotCount)
{
#ifdef CMDQ_GPR_SUPPORT

	dma_addr_t paStart = 0;
	int status = 0;

	if (NULL == phBackupSlot) {
		return -EINVAL;
	}

	status = cmdqCoreAllocWriteAddress(slotCount, &paStart);
	*phBackupSlot = paStart;

	return status;

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqBackupReadSlot(cmdqBackupSlotHandle hBackupSlot, uint32_t slotIndex, uint32_t *value)
{
#ifdef CMDQ_GPR_SUPPORT

	if (NULL == value) {
		return -EINVAL;
	}

	if (0 == hBackupSlot){
		CMDQ_ERR("%s, hBackupSlot is NULL\n", __func__);
		return -EINVAL;
	}

	*value = cmdqCoreReadWriteAddress(hBackupSlot + slotIndex * sizeof(uint32_t));

	return 0;

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqBackupWriteSlot(cmdqBackupSlotHandle hBackupSlot, uint32_t slotIndex, uint32_t value)
{
#ifdef CMDQ_GPR_SUPPORT

	int status = 0;
	
	status = cmdqCoreWriteWriteAddress(hBackupSlot + slotIndex * sizeof(uint32_t), value);
	return status;

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}


int32_t cmdqBackupFreeSlot(cmdqBackupSlotHandle hBackupSlot)
{
#ifdef CMDQ_GPR_SUPPORT
	return cmdqCoreFreeWriteAddress(hBackupSlot);
#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqRecBackupRegisterToSlot(cmdqRecHandle handle,
				    cmdqBackupSlotHandle hBackupSlot,
				    uint32_t slotIndex, uint32_t regAddr)
{
#ifdef CMDQ_GPR_SUPPORT
	const CMDQ_DATA_REGISTER_ENUM valueRegId = CMDQ_DATA_REG_DEBUG;
	const CMDQ_DATA_REGISTER_ENUM destRegId = CMDQ_DATA_REG_DEBUG_DST;
	const CMDQ_EVENT_ENUM regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_4;
	const dma_addr_t dramAddr = hBackupSlot + slotIndex * sizeof(uint32_t);

	
	cmdqRecWait(handle, regAccessToken);

	
	cmdq_append_command(handle, CMDQ_CODE_READ, regAddr, valueRegId);

	
	
	
	cmdq_append_command(handle, CMDQ_CODE_MOVE,
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
			    ((dramAddr >> 32) & 0xffff) |
#endif
			    ((destRegId & 0x1f) << 16) | (4 << 21), (uint32_t) dramAddr);

	
	cmdq_append_command(handle,
			    CMDQ_CODE_RAW,
			    (CMDQ_CODE_WRITE << 24) | (0 & 0xffff) | ((destRegId & 0x1f) << 16) | (6
												   <<
												   21),
			    valueRegId);
	
	cmdqRecSetEventToken(handle, regAccessToken);

	return 0;

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqRecBackupWriteRegisterFromSlot(cmdqRecHandle handle,
					cmdqBackupSlotHandle hBackupSlot,
					uint32_t slotIndex, uint32_t addr)
{
#ifdef CMDQ_GPR_SUPPORT
	const CMDQ_DATA_REGISTER_ENUM valueRegId = CMDQ_DATA_REG_DEBUG;
	const CMDQ_DATA_REGISTER_ENUM addrRegId = CMDQ_DATA_REG_DEBUG_DST;
	const CMDQ_EVENT_ENUM regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_4;
	const dma_addr_t dramAddr = hBackupSlot + slotIndex * sizeof(uint32_t);

	
	cmdqRecWait(handle, regAccessToken);

	

	
	
	
	cmdq_append_command(handle, CMDQ_CODE_MOVE,
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		((dramAddr >> 32) & 0xffff) |
#endif
		((addrRegId & 0x1f) << 16) | (4 << 21), (uint32_t) dramAddr); 

	
	cmdq_append_command(handle, CMDQ_CODE_RAW,
		(CMDQ_CODE_READ << 24) | (0 & 0xffff) | ((addrRegId & 0x1f) << 16) | (6 << 21),
		valueRegId);

	
	cmdqRecWriteFromDataRegister(handle, valueRegId, addr);

	
	cmdqRecSetEventToken(handle, regAccessToken);

	return 0;
#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 
}

int32_t cmdqRecBackupUpdateSlot(cmdqRecHandle handle,
					cmdqBackupSlotHandle hBackupSlot,
					uint32_t slotIndex, uint32_t value)
{
#ifdef CMDQ_GPR_SUPPORT
	const CMDQ_DATA_REGISTER_ENUM valueRegId = CMDQ_DATA_REG_DEBUG;
	const CMDQ_DATA_REGISTER_ENUM destRegId = CMDQ_DATA_REG_DEBUG_DST;
	const CMDQ_EVENT_ENUM regAccessToken = CMDQ_SYNC_TOKEN_GPR_SET_4;
	const dma_addr_t dramAddr = hBackupSlot + slotIndex * sizeof(uint32_t);

	
	cmdqRecWait(handle, regAccessToken);

	
	cmdq_append_command(
			handle,
			CMDQ_CODE_RAW,
			(CMDQ_CODE_MOVE << 24) | (valueRegId << 16) | (4 << 21), 
			value);

	
	
	
	cmdq_append_command(handle, CMDQ_CODE_MOVE,
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
			    ((dramAddr >> 32) & 0xffff) |
#endif
			    ((destRegId & 0x1f) << 16) | (4 << 21), (uint32_t) dramAddr);

	
	cmdq_append_command(
			handle,
			CMDQ_CODE_RAW,
			(CMDQ_CODE_WRITE << 24) | (0 & 0xffff) | ((destRegId & 0x1f) << 16) | (6 << 21),
			valueRegId);

	
	cmdqRecSetEventToken(handle, regAccessToken);

	return 0;

#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't support GPR\n", __func__);
	return -EFAULT;
#endif 

}

int32_t cmdqRecEnablePrefetch(cmdqRecHandle handle)
{
	if (cmdq_core_should_enable_prefetch(handle->scenario)) {
		
        CMDQ_VERBOSE("REC: enable prefetch\n");
		cmdqRecMark(handle);
		return true;
	}
	CMDQ_ERR("not allow enable prefetch, scenario: %d\n", handle->scenario);
	return -EFAULT;
}

int32_t cmdqRecDisablePrefetch(cmdqRecHandle handle)
{
	uint32_t argB = 0;
	uint32_t argA = 0;
	int32_t status = 0;

	if (!handle->finalized) {

        if (handle->prefetchCount > 0) {
        
        
        
        
        
            argB = 0x00100000;
            argA = (0x1 << 16); 
            
            handle->prefetchCount = 0;
            status = cmdq_append_command(handle, CMDQ_CODE_EOC, argA, argB);
        }

        if (0 != status) {
            return status;
        }
    }

    CMDQ_MSG("cmdqRecDisablePrefetch, status:%d\n", status);
    return status;
}

int32_t cmdq_rec_finalize_command(cmdqRecHandle handle, bool loop)
{
	int32_t status = 0;
	uint32_t argB = 0;

	if (!handle->finalized) {
		if ((handle->prefetchCount > 0) && cmdq_core_should_enable_prefetch(handle->scenario)) {
			CMDQ_ERR("not insert prefetch disble marker when prefetch enabled, prefetchCount:%d\n", handle->prefetchCount);
			cmdqRecDumpCommand(handle);

			status = -EFAULT;
			return status;
        }

		
		argB = 0x1;	
#ifdef DISABLE_LOOP_IRQ
		if (loop && !cmdq_platform_force_loop_irq_from_scenario(handle->scenario)){
			argB = 0x0; 
		}
#endif
		status = cmdq_append_command(handle, CMDQ_CODE_EOC, 0, argB);

		if (0 != status) {
			return status;
		}

		
		status = cmdq_append_command(handle, CMDQ_CODE_JUMP, 0,	
					     loop ? -handle->blockSize : 8);
		if (0 != status) {
			return status;
		}

		handle->finalized = true;
	}

	return status;
}

int32_t cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(
			cmdqCommandStruct *pDesc, cmdqRecHandle handle)
{
	
	pDesc->secData.isSecure = handle->secData.isSecure;
	pDesc->secData.enginesNeedDAPC = handle->secData.enginesNeedDAPC;
	pDesc->secData.enginesNeedPortSecurity = handle->secData.enginesNeedPortSecurity;

	pDesc->secData.addrMetadataCount = handle->secData.addrMetadataCount;
	pDesc->secData.addrMetadatas = handle->secData.addrMetadatas;
	pDesc->secData.addrMetadataMaxCount = handle->secData.addrMetadataMaxCount;

	
	pDesc->secData.resetExecCnt = false;
	pDesc->secData.waitCookie = 0;

	return 0;
}

int32_t cmdq_rec_setup_profile_marker_data(cmdqCommandStruct *pDesc, cmdqRecHandle handle)
{
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	uint32_t i;

	pDesc->profileMarker.count = handle->profileMarker.count;
	pDesc->profileMarker.hSlot = handle->profileMarker.hSlot;

	for (i = 0; i < CMDQ_MAX_PROFILE_MARKER_IN_TASK; i++) {
		pDesc->profileMarker.tag[i] = handle->profileMarker.tag[i];
	}
#endif
	return 0;
}

int32_t cmdqRecFlush(cmdqRecHandle handle)
{
	int32_t status;
	cmdqCommandStruct desc = { 0 };

	status = cmdq_rec_finalize_command(handle, false);
	if (status < 0) {
		return status;
	}

	CMDQ_MSG("Submit task scenario: %d, priority: %d, engine: 0x%llx, buffer: 0x%p, size: %d\n",
		 handle->scenario, handle->priority, handle->engineFlag, handle->pBuffer,
		 handle->blockSize);

	desc.scenario = handle->scenario;
	desc.priority = handle->priority;
	desc.engineFlag = handle->engineFlag;
	desc.pVABase = (cmdqU32Ptr_t)(unsigned long)handle->pBuffer;
	desc.blockSize = handle->blockSize;
	
	cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(&desc, handle);
	
	cmdq_rec_setup_profile_marker_data(&desc, handle);

	return cmdqCoreSubmitTask(&desc);
}

int32_t cmdqRecFlushAndReadRegister(cmdqRecHandle handle, uint32_t regCount, uint32_t *addrArray,
				    uint32_t *valueArray)
{
	int32_t status;
	cmdqCommandStruct desc = { 0 };

	status = cmdq_rec_finalize_command(handle, false);
	if (status < 0) {
		return status;
	}

	CMDQ_MSG("Submit task scenario: %d, priority: %d, engine: 0x%llx, buffer: 0x%p, size: %d\n",
		 handle->scenario, handle->priority, handle->engineFlag, handle->pBuffer,
		 handle->blockSize);

	desc.scenario = handle->scenario;
	desc.priority = handle->priority;
	desc.engineFlag = handle->engineFlag;
	desc.pVABase = (cmdqU32Ptr_t)(unsigned long)handle->pBuffer;
	desc.blockSize = handle->blockSize;
	desc.regRequest.count = regCount;
	desc.regRequest.regAddresses = (cmdqU32Ptr_t)(unsigned long)addrArray;
	desc.regValue.count = regCount;
	desc.regValue.regValues = (cmdqU32Ptr_t)(unsigned long)valueArray;
	
	cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(&desc, handle);
	
	cmdq_rec_setup_profile_marker_data(&desc, handle);

	return cmdqCoreSubmitTask(&desc);
}


int32_t cmdqRecFlushAsync(cmdqRecHandle handle)
{
	int32_t status = 0;
	cmdqCommandStruct desc = { 0 };
	TaskStruct *pTask = NULL;

	status = cmdq_rec_finalize_command(handle, false);
	if (status < 0) {
		return status;
	}

	desc.scenario = handle->scenario;
	desc.priority = handle->priority;
	desc.engineFlag = handle->engineFlag;
	desc.pVABase = (cmdqU32Ptr_t)(unsigned long)handle->pBuffer;
	desc.blockSize = handle->blockSize;
	desc.regRequest.count = 0;
	desc.regRequest.regAddresses = (cmdqU32Ptr_t)(unsigned long)NULL;
	desc.regValue.count = 0;
	desc.regValue.regValues = (cmdqU32Ptr_t)(unsigned long)NULL;
	
	cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(&desc, handle);
	
	cmdq_rec_setup_profile_marker_data(&desc, handle);

	status = cmdqCoreSubmitTaskAsync(&desc, NULL, 0, &pTask);

	CMDQ_MSG
	    ("[Auto Release] Submit ASYNC task scenario: %d, priority: %d, engine: 0x%llx, buffer: 0x%p, size: %d\n",
	     handle->scenario, handle->priority, handle->engineFlag, handle->pBuffer,
	     handle->blockSize);

	if (pTask) {
		pTask->flushCallback = NULL;
		pTask->flushData = 0;
	}

	
	if (pTask) {
		status = cmdqCoreAutoReleaseTask(pTask);
	} else {
		status = -ENOMEM;
	}
	return status;
}


int32_t cmdqRecFlushAsyncCallback(cmdqRecHandle handle, CmdqAsyncFlushCB callback,
				  uint32_t userData)
{
	int32_t status = 0;
	cmdqCommandStruct desc = { 0 };
	TaskStruct *pTask = NULL;

	status = cmdq_rec_finalize_command(handle, false);
	if (status < 0) {
		return status;
	}

	desc.scenario = handle->scenario;
	desc.priority = handle->priority;
	desc.engineFlag = handle->engineFlag;
	desc.pVABase = (cmdqU32Ptr_t)(unsigned long)handle->pBuffer;
	desc.blockSize = handle->blockSize;
	desc.regRequest.count = 0;
	desc.regRequest.regAddresses = (cmdqU32Ptr_t)(unsigned long)NULL;
	desc.regValue.count = 0;
	desc.regValue.regValues = (cmdqU32Ptr_t)(unsigned long)NULL;
	
	cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(&desc, handle);
	
	cmdq_rec_setup_profile_marker_data(&desc, handle);

	status = cmdqCoreSubmitTaskAsync(&desc, NULL, 0, &pTask);

	
	
	if (pTask) {
		pTask->flushCallback = callback;
		pTask->flushData = userData;
	}

	CMDQ_MSG
	    ("[Auto Release] Submit ASYNC task scenario: %d, priority: %d, engine: 0x%llx, buffer: 0x%p, size: %d\n",
	     handle->scenario, handle->priority, handle->engineFlag, handle->pBuffer,
	     handle->blockSize);

	
	if (pTask) {
		status = cmdqCoreAutoReleaseTask(pTask);
	} else {
		status = -ENOMEM;
	}

	return status;

}


static int32_t cmdqRecIRQCallback(unsigned long data)
{
	return 0;
}

int32_t cmdqRecStartLoop(cmdqRecHandle handle)
{
	int32_t status = 0;
	cmdqCommandStruct desc = { 0 };

	if (NULL != handle->pRunningTask) {
		return -EBUSY;
	}

	status = cmdq_rec_finalize_command(handle, true);
	if (status < 0) {
		return status;
	}

	CMDQ_MSG
	    ("Submit task loop: scenario: %d, priority: %d, engine: 0x%llx, buffer: 0x%p, size: %d, callback: 0x%p, data: %d\n",
	     handle->scenario, handle->priority, handle->engineFlag, handle->pBuffer,
	     handle->blockSize, &cmdqRecIRQCallback, 0);

	desc.scenario = handle->scenario;
	desc.priority = handle->priority;
	desc.engineFlag = handle->engineFlag;
	desc.pVABase = (cmdqU32Ptr_t)(unsigned long)handle->pBuffer;
	desc.blockSize = handle->blockSize;
	
	cmdq_rec_setup_sec_data_of_command_desc_by_rec_handle(&desc, handle);
	
	cmdq_rec_setup_profile_marker_data(&desc, handle);

	status = cmdqCoreSubmitTaskAsync(&desc, &cmdqRecIRQCallback, 0, &handle->pRunningTask);
	return status;
}

int32_t cmdqRecStopLoop(cmdqRecHandle handle)
{
	int32_t status = 0;
	struct TaskStruct *pTask = handle->pRunningTask;

	if (NULL == pTask) {
		return -EFAULT;
	}

	status = cmdqCoreReleaseTask(pTask);
	handle->pRunningTask = NULL;
	return status;
}

int32_t cmdqRecGetInstructionCount(cmdqRecHandle handle)
{
	if (NULL == handle) {
		return 0;
	}

	return (handle->blockSize / CMDQ_INST_SIZE);
}

int32_t cmdqRecProfileMarker(cmdqRecHandle handle, const char *tag)
{
#ifdef CMDQ_PROFILE_MARKER_SUPPORT
	int32_t status;
	int32_t index;
	cmdqBackupSlotHandle hSlot;
	dma_addr_t allocatedStartPA;

	do {
		allocatedStartPA = 0;
		status = 0;

		
		
		if ((0 == handle->profileMarker.count) && (0 == handle->profileMarker.hSlot)) {
			status = cmdqCoreAllocWriteAddress(CMDQ_MAX_PROFILE_MARKER_IN_TASK, &allocatedStartPA);
			if(0 > status) {
				CMDQ_ERR("[REC][PROF_MARKER]allocate failed, status:%d\n", status);
				break;
			}

			handle->profileMarker.hSlot = 0LL | (allocatedStartPA);

			CMDQ_VERBOSE("[REC][PROF_MARKER]update handle(%p) slot start PA:%pa(0x%llx)\n",
				handle, &allocatedStartPA, handle->profileMarker.hSlot);
		}

		
		index = handle->profileMarker.count;
		hSlot = (cmdqBackupSlotHandle)(handle->profileMarker.hSlot);

		if (index >= CMDQ_MAX_PROFILE_MARKER_IN_TASK) {
			CMDQ_ERR("[REC][PROF_MARKER]insert profile maker failed since already reach max count\n");
			status = -EFAULT;
			break;
		}

		CMDQ_VERBOSE("[REC][PROF_MARKER]inserting profile instr, handle:%p, slot:%pa(0x%llx), index:%d, tag:%s\n",
			handle, &hSlot, handle->profileMarker.hSlot, index, tag);

		cmdqRecBackupRegisterToSlot(handle, hSlot, index, CMDQ_APXGPT2_COUNT);

		handle->profileMarker.tag[index] = tag;
		handle->profileMarker.count += 1;
	} while(0);

	return status;
#else
	CMDQ_ERR("func:%s failed since CMDQ dosen't enable profile marker\n", __func__);
	return -EFAULT;
#endif
}

int32_t cmdqRecDumpCommand(cmdqRecHandle handle)
{
	int32_t status = 0;
	struct TaskStruct *pTask = handle->pRunningTask;

	if (pTask) {
		
		status = cmdqCoreDebugDumpCommand(pTask);
	} else {
		
		const uint32_t *pCmd = NULL;
		static char textBuf[128] = { 0 };
		int i = 0;

		CMDQ_LOG("======REC 0x%p command buffer:\n", handle->pBuffer);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 4,
			       handle->pBuffer, handle->blockSize, false);

		CMDQ_LOG("======REC 0x%p command buffer END\n", handle->pBuffer);
		CMDQ_LOG("REC 0x%p command buffer TRANSLATED:\n", handle->pBuffer);
		for (i = 0, pCmd = handle->pBuffer; i < handle->blockSize; i += 8, pCmd += 2) {
			cmdq_core_parse_instruction(pCmd, textBuf, 128);
			CMDQ_LOG("%s", textBuf);
		}
		CMDQ_LOG("======REC 0x%p command END\n", handle->pBuffer);

		return 0;
	}

	return status;
}

int32_t cmdqRecEstimateEommandExecTime(const cmdqRecHandle handle)
{
	int32_t time = 0;

	if (NULL == handle) {
		return -EFAULT;
	}

	CMDQ_LOG("======REC 0x%p command execution time ESTIMATE:\n", handle);
	time = cmdq_prof_estimate_command_exe_time(handle->pBuffer, handle->blockSize);
	CMDQ_LOG("======REC 0x%p  END\n", handle);

	return time;
}

void cmdqRecDestroy(cmdqRecHandle handle)
{
	if (NULL == handle) {
		return;
	}

	if (NULL != handle->pRunningTask) {
		cmdqRecStopLoop(handle);
	}

	
	kfree(handle->pBuffer);
	handle->pBuffer = NULL;

	
	kfree(handle);
}
