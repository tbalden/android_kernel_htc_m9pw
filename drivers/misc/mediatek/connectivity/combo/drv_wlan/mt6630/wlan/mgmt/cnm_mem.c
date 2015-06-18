





#include "precomp.h"





static PUINT_8 apucStaRecType[STA_TYPE_INDEX_NUM] = {
	(PUINT_8) "LEGACY",
	(PUINT_8) "P2P",
	(PUINT_8) "BOW"
};

static PUINT_8 apucStaRecRole[STA_ROLE_INDEX_NUM] = {
	(PUINT_8) "ADHOC",
	(PUINT_8) "CLIENT",
	(PUINT_8) "AP",
	(PUINT_8) "DLS"
};

#if CFG_SUPPORT_TDLS
const UINT_8 aucValidDataRate[] = {
    RATE_1M,            
    RATE_2M,            
    RATE_5_5M,          
    RATE_11M,           
    RATE_22M,           
    RATE_33M,           
    RATE_6M,            
    RATE_9M,            
    RATE_12M,           
    RATE_18M,           
    RATE_24M,           
    RATE_36M,           
    RATE_48M,           
    RATE_54M,           
    RATE_VHT_PHY,		
    RATE_HT_PHY         
};
#endif


static VOID cnmStaRoutinesForAbort(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec);

static VOID
cnmStaRecHandleEventPkt(P_ADAPTER_T prAdapter, P_CMD_INFO_T prCmdInfo, PUINT_8 pucEventBuf);

static VOID cnmStaSendUpdateCmd(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec, BOOLEAN fgNeedResp);

static VOID
cnmStaSendRemoveCmd(P_ADAPTER_T prAdapter,
		    ENUM_STA_REC_CMD_ACTION_T eActionType, UINT_8 ucStaRecIndex, UINT_8 ucBssIndex);


P_MSDU_INFO_T cnmPktAllocWrapper(P_ADAPTER_T prAdapter, UINT_32 u4Length, PUINT_8 pucStr)
{
    P_MSDU_INFO_T   prMsduInfo;

    prMsduInfo = cnmPktAlloc(prAdapter, u4Length);
    DBGLOG(MEM, INFO, ("Alloc MSDU_INFO[0x%p] by [%s]\n", prMsduInfo, pucStr));

    return prMsduInfo;
}

VOID cnmPktFreeWrapper(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo, PUINT_8 pucStr)
{
    DBGLOG(MEM, INFO, ("Free MSDU_INFO[0x%p] by [%s]\n", prMsduInfo, pucStr));

    cnmPktFree(prAdapter, prMsduInfo);
}

P_MSDU_INFO_T cnmPktAlloc(P_ADAPTER_T prAdapter, UINT_32 u4Length)
{
    P_MSDU_INFO_T   prMsduInfo;
    P_QUE_T         prQueList;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    prQueList = &prAdapter->rTxCtrl.rFreeMsduInfoList;

    
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
    QUEUE_REMOVE_HEAD(prQueList, prMsduInfo, P_MSDU_INFO_T);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);

    if (prMsduInfo) {
        prMsduInfo->prPacket = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4Length);
        prMsduInfo->eSrc = TX_PACKET_MGMT;

        if (prMsduInfo->prPacket == NULL) {
            KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
            QUEUE_INSERT_TAIL(prQueList, &prMsduInfo->rQueEntry);
            KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
            prMsduInfo = NULL;
        }
    }
#if DBG
    if (prMsduInfo == NULL) {
        DBGLOG(MEM, WARN, ("\n"));
        DBGLOG(MEM, WARN, ("MgtDesc#=%ld\n", prQueList->u4NumElem));

#if CFG_DBG_MGT_BUF
        DBGLOG(MEM, WARN, ("rMgtBufInfo: alloc#=%ld, free#=%ld, null#=%ld\n",
            prAdapter->rMgtBufInfo.u4AllocCount,
            prAdapter->rMgtBufInfo.u4FreeCount,
            prAdapter->rMgtBufInfo.u4AllocNullCount));
#endif

        DBGLOG(MEM, WARN, ("\n"));
    }
#endif

    return prMsduInfo;
}

VOID cnmPktFree(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
    P_QUE_T         prQueList;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    
	if (!prMsduInfo) {
        return;
    }

    prQueList = &prAdapter->rTxCtrl.rFreeMsduInfoList;

	
    if (prMsduInfo->prPacket) {
        cnmMemFree(prAdapter, prMsduInfo->prPacket);
        prMsduInfo->prPacket = NULL;
    }

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
    QUEUE_INSERT_TAIL(prQueList, &prMsduInfo->rQueEntry);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
}

VOID cnmMemInit(P_ADAPTER_T prAdapter)
{
    P_BUF_INFO_T    prBufInfo;

    
    prBufInfo = &prAdapter->rMgtBufInfo;
    kalMemZero(prBufInfo, sizeof(prAdapter->rMgtBufInfo));
    prBufInfo->pucBuf = prAdapter->pucMgtBufCached;

    
	prBufInfo->rFreeBlocksBitmap = (BUF_BITMAP) BITS(0, MAX_NUM_OF_BUF_BLOCKS - 1);


    
    prBufInfo = &prAdapter->rMsgBufInfo;
    kalMemZero(prBufInfo, sizeof(prAdapter->rMsgBufInfo));
    prBufInfo->pucBuf = &prAdapter->aucMsgBuf[0];

    
	prBufInfo->rFreeBlocksBitmap = (BUF_BITMAP) BITS(0, MAX_NUM_OF_BUF_BLOCKS - 1);

    return;

} 


PVOID cnmMemAlloc(IN P_ADAPTER_T prAdapter, IN ENUM_RAM_TYPE_T eRamType, IN UINT_32 u4Length)
{
    P_BUF_INFO_T        prBufInfo;
    BUF_BITMAP          rRequiredBitmap;
    UINT_32             u4BlockNum;
    UINT_32             i, u4BlkSzInPower;
    PVOID               pvMemory;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

	if (u4Length == 0) {
		DBGLOG(MEM, WARN, ("%s: Length to be allocated is ZERO, skip!\n", __func__));
        return NULL;
    }

    if (eRamType == RAM_TYPE_MSG && u4Length <= 256) {
        prBufInfo = &prAdapter->rMsgBufInfo;
        u4BlkSzInPower = MSG_BUF_BLOCK_SIZE_IN_POWER_OF_2;

        u4BlockNum = (u4Length + MSG_BUF_BLOCK_SIZE - 1)
                        >> MSG_BUF_BLOCK_SIZE_IN_POWER_OF_2;

        ASSERT(u4BlockNum <= MAX_NUM_OF_BUF_BLOCKS);
	} else {
        eRamType = RAM_TYPE_BUF;

        prBufInfo = &prAdapter->rMgtBufInfo;
        u4BlkSzInPower = MGT_BUF_BLOCK_SIZE_IN_POWER_OF_2;

        u4BlockNum = (u4Length + MGT_BUF_BLOCK_SIZE - 1)
                        >> MGT_BUF_BLOCK_SIZE_IN_POWER_OF_2;

        ASSERT(u4BlockNum <= MAX_NUM_OF_BUF_BLOCKS);
    }

#if CFG_DBG_MGT_BUF
    prBufInfo->u4AllocCount++;
#endif

    KAL_ACQUIRE_SPIN_LOCK(prAdapter,
        eRamType == RAM_TYPE_MSG ? SPIN_LOCK_MSG_BUF : SPIN_LOCK_MGT_BUF);

    if ((u4BlockNum > 0) && (u4BlockNum <= MAX_NUM_OF_BUF_BLOCKS)) {

        
		rRequiredBitmap = BITS(0, u4BlockNum - 1);

        for (i = 0; i <= (MAX_NUM_OF_BUF_BLOCKS - u4BlockNum); i++) {

            
            if ((prBufInfo->rFreeBlocksBitmap & rRequiredBitmap)
                == rRequiredBitmap) {

                
                prBufInfo->rFreeBlocksBitmap &= ~rRequiredBitmap;

                
                prBufInfo->aucAllocatedBlockNum[i] = (UINT_8) u4BlockNum;

                KAL_RELEASE_SPIN_LOCK(prAdapter,
                        eRamType == RAM_TYPE_MSG ?
                        SPIN_LOCK_MSG_BUF : SPIN_LOCK_MGT_BUF);

                
				return (PVOID) (prBufInfo->pucBuf + (i << u4BlkSzInPower));

            }

            rRequiredBitmap <<= 1;
        }
    }

    
    KAL_RELEASE_SPIN_LOCK(prAdapter,
        eRamType == RAM_TYPE_MSG ? SPIN_LOCK_MSG_BUF : SPIN_LOCK_MGT_BUF);

#ifdef LINUX
	pvMemory = (PVOID) kalMemAlloc(u4Length, PHY_MEM_TYPE);
#else
	pvMemory = (PVOID) NULL;
#endif

#if CFG_DBG_MGT_BUF
    prBufInfo->u4AllocNullCount++;

    if (pvMemory) {
        prAdapter->u4MemAllocDynamicCount++;
    }
#endif

    return pvMemory;

} 


VOID cnmMemFree(IN P_ADAPTER_T prAdapter, IN PVOID pvMemory)
{
    P_BUF_INFO_T        prBufInfo;
    UINT_32             u4BlockIndex;
    BUF_BITMAP          rAllocatedBlocksBitmap;
    ENUM_RAM_TYPE_T     eRamType;
    KAL_SPIN_LOCK_DECLARATION();


    ASSERT(prAdapter);
    
    if (!pvMemory) {
        return;
    }

    
	if (((ULONG) pvMemory >= (ULONG) &prAdapter->aucMsgBuf[0]) &&
	    ((ULONG) pvMemory <= (ULONG) &prAdapter->aucMsgBuf[MSG_BUFFER_SIZE - 1])) {

        prBufInfo = &prAdapter->rMsgBufInfo;
		u4BlockIndex = ((ULONG) pvMemory - (ULONG) prBufInfo->pucBuf)
                       >> MSG_BUF_BLOCK_SIZE_IN_POWER_OF_2;
        ASSERT(u4BlockIndex < MAX_NUM_OF_BUF_BLOCKS);
        eRamType = RAM_TYPE_MSG;
	} else if (((ULONG) pvMemory >= (ULONG) prAdapter->pucMgtBufCached) &&
		   ((ULONG) pvMemory <=
		    ((ULONG) prAdapter->pucMgtBufCached + MGT_BUFFER_SIZE - 1))) {
        prBufInfo = &prAdapter->rMgtBufInfo;
		u4BlockIndex = ((ULONG) pvMemory - (ULONG) prBufInfo->pucBuf)
                       >> MGT_BUF_BLOCK_SIZE_IN_POWER_OF_2;
        ASSERT(u4BlockIndex < MAX_NUM_OF_BUF_BLOCKS);
        eRamType = RAM_TYPE_BUF;
	} else {
#ifdef LINUX
        
        kalMemFree(pvMemory, PHY_MEM_TYPE, 0);
#else
        
        ASSERT(0);
#endif

#if CFG_DBG_MGT_BUF
        prAdapter->u4MemFreeDynamicCount++;
#endif
        return;
    }

    KAL_ACQUIRE_SPIN_LOCK(prAdapter,
        eRamType == RAM_TYPE_MSG ? SPIN_LOCK_MSG_BUF : SPIN_LOCK_MGT_BUF);

#if CFG_DBG_MGT_BUF
    prBufInfo->u4FreeCount++;
#endif

    
    ASSERT(prBufInfo->aucAllocatedBlockNum[u4BlockIndex] > 0);

	rAllocatedBlocksBitmap = BITS(0, prBufInfo->aucAllocatedBlockNum[u4BlockIndex] - 1);
    rAllocatedBlocksBitmap <<= u4BlockIndex;

    
    prBufInfo->aucAllocatedBlockNum[u4BlockIndex] = 0;

    
    prBufInfo->rFreeBlocksBitmap |= rAllocatedBlocksBitmap;

    KAL_RELEASE_SPIN_LOCK(prAdapter,
        eRamType == RAM_TYPE_MSG ? SPIN_LOCK_MSG_BUF : SPIN_LOCK_MGT_BUF);

    return;

} 


VOID cnmStaRecInit(P_ADAPTER_T prAdapter)
{
    P_STA_RECORD_T  prStaRec;
    UINT_16         i;

    for (i = 0; i < CFG_STA_REC_NUM; i++) {
        prStaRec = &prAdapter->arStaRec[i];

        prStaRec->ucIndex = (UINT_8) i;
        prStaRec->fgIsInUse = FALSE;
    }
}

P_STA_RECORD_T
cnmStaRecAlloc(P_ADAPTER_T prAdapter,
	       ENUM_STA_TYPE_T eStaType, UINT_8 ucBssIndex, PUINT_8 pucMacAddr)
{
    P_STA_RECORD_T  prStaRec;
    UINT_16         i, k;

    ASSERT(prAdapter);

    for (i = 0; i < CFG_STA_REC_NUM; i++) {
        prStaRec = &prAdapter->arStaRec[i];

        if (!prStaRec->fgIsInUse) {
            kalMemZero(prStaRec, sizeof(STA_RECORD_T));
            prStaRec->ucIndex = (UINT_8) i;
            prStaRec->ucBssIndex = ucBssIndex;
            prStaRec->fgIsInUse = TRUE;

            prStaRec->eStaType = eStaType;
            prStaRec->ucBssIndex = ucBssIndex;
            prStaRec->ucCurrentGtkId = 255;

            
            for (k = 0; k < TID_NUM + 1; k++) {
                prStaRec->au2CachedSeqCtrl[k] = 0xFFFF;
                prStaRec->afgIsIgnoreAmsduDuplicate[k] = FALSE;
            }

            
            for (k = 0; k < STA_WAIT_QUEUE_NUM; k++) {
                LINK_INITIALIZE(&prStaRec->arStaWaitQueue[k]);
            }
		
#if CFG_ENABLE_PER_STA_STATISTICS && CFG_ENABLE_PKT_LIFETIME_PROFILE
            prStaRec->u4TotalTxPktsNumber = 0;
            prStaRec->u4TotalTxPktsTime = 0;
            prStaRec->u4TotalRxPktsNumber = 0;
			prStaRec->u4MaxTxPktsTime = 0;
#endif

            for (k = 0; k < NUM_OF_PER_STA_TX_QUEUES; k++) {
                QUEUE_INITIALIZE(&prStaRec->arTxQueue[k]);
            }

            break;
        }
    }

    
    if (i < CFG_STA_REC_NUM) {
        COPY_MAC_ADDR(prStaRec->aucMacAddr, pucMacAddr);
        prStaRec->ucBMCWlanIndex = WTBL_RESERVED_ENTRY;
        if (secPrivacySeekForEntry(prAdapter, prStaRec)) {
            cnmStaSendUpdateCmd(prAdapter, prStaRec, FALSE);
        }
#if DBG
        else {
            prStaRec->fgIsInUse = FALSE;
            prStaRec = NULL;
            ASSERT(FALSE);
        }
#endif
	} else {
        prStaRec = NULL;
    }

    return prStaRec;
}

VOID cnmStaRecFree(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec)
{
    UINT_8      ucStaRecIndex, ucBssIndex;

    ASSERT(prAdapter);

	if (!prStaRec) {
        return;
    }

    ucStaRecIndex = prStaRec->ucIndex;
    ucBssIndex = prStaRec->ucBssIndex;

    cnmStaRoutinesForAbort(prAdapter, prStaRec);

	cnmStaSendRemoveCmd(prAdapter, STA_REC_CMD_ACTION_STA, ucStaRecIndex, ucBssIndex);
    return;
}

static VOID cnmStaRoutinesForAbort(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec)
{
    ASSERT(prAdapter);

	if (!prStaRec) {
        return;
    }

    
    cnmTimerStopTimer(prAdapter, &prStaRec->rTxReqDoneOrRxRespTimer);
    prStaRec->fgTransmitKeyExist = FALSE;
    prStaRec->ucCurrentGtkId = 0;

    prStaRec->fgSetPwrMgtBit = FALSE;

    if (prStaRec->pucAssocReqIe) {
		kalMemFree(prStaRec->pucAssocReqIe, VIR_MEM_TYPE, prStaRec->u2AssocReqIeLen);
        prStaRec->pucAssocReqIe = NULL;
        prStaRec->u2AssocReqIeLen = 0;
    }

    qmDeactivateStaRec(prAdapter, prStaRec);

    
    secPrivacyFreeSta(prAdapter, prStaRec);

    prStaRec->fgIsInUse = FALSE;
}

VOID
cnmStaFreeAllStaByNetwork(P_ADAPTER_T prAdapter, UINT_8 ucBssIndex, UINT_8 ucStaRecIndexExcluded)
{
#if CFG_ENABLE_WIFI_DIRECT
    P_BSS_INFO_T    prBssInfo;
#endif
    P_STA_RECORD_T  prStaRec;
    UINT_16         i;

	if (ucBssIndex > MAX_BSS_INDEX) {
        return;
    }

    for (i = 0; i < CFG_STA_REC_NUM; i++) {
        prStaRec = (P_STA_RECORD_T) &prAdapter->arStaRec[i];

        if (prStaRec->fgIsInUse && prStaRec->ucBssIndex == ucBssIndex &&
            i != ucStaRecIndexExcluded) {
            cnmStaRoutinesForAbort(prAdapter, prStaRec);
        }
    } 

    cnmStaSendRemoveCmd(prAdapter,
            (ucStaRecIndexExcluded < CFG_STA_REC_NUM) ?
                STA_REC_CMD_ACTION_BSS_EXCLUDE_STA : STA_REC_CMD_ACTION_BSS,
            ucStaRecIndexExcluded, ucBssIndex);

#if CFG_ENABLE_WIFI_DIRECT
    prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);
	if (prAdapter->fgIsP2PRegistered && prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {

        rlmUpdateParamsForAP(prAdapter, prBssInfo, FALSE);
    }
#endif
}

P_STA_RECORD_T cnmGetStaRecByIndex(P_ADAPTER_T prAdapter, UINT_8 ucIndex)
{
    P_STA_RECORD_T  prStaRec;

    ASSERT(prAdapter);

	prStaRec = (ucIndex < CFG_STA_REC_NUM) ? &prAdapter->arStaRec[ucIndex] : NULL;

    if (prStaRec && prStaRec->fgIsInUse == FALSE) {
        prStaRec = NULL;
    }

    return prStaRec;
}

P_STA_RECORD_T
cnmGetStaRecByAddress(P_ADAPTER_T prAdapter, UINT_8 ucBssIndex, PUINT_8 pucPeerMacAddr)
{
    P_STA_RECORD_T  prStaRec;
    UINT_16         i;

    ASSERT(prAdapter);

	if (!pucPeerMacAddr) {
        return NULL;
    }

    for (i = 0; i < CFG_STA_REC_NUM; i++) {
        prStaRec = &prAdapter->arStaRec[i];

        if (prStaRec->fgIsInUse &&
            prStaRec->ucBssIndex == ucBssIndex &&
            EQUAL_MAC_ADDR(prStaRec->aucMacAddr, pucPeerMacAddr)) {
            break;
        }
    }

    return (i < CFG_STA_REC_NUM) ? prStaRec : NULL;
}

VOID cnmStaRecChangeState(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec, UINT_8 ucNewState)
{
    BOOLEAN         fgNeedResp;

	if (!prAdapter) {
        return;
    }

	if (!prStaRec) {
		DBGLOG(MEM, WARN, ("%s: StaRec is NULL, skip!\n", __func__));
        return;
    }

	if (!prStaRec->fgIsInUse) {
		DBGLOG(MEM, WARN,
		       ("%s: StaRec[%u] is not in use, skip!\n", __func__, prStaRec->ucIndex));
        return;
    }

    if ((ucNewState == STA_STATE_2 && prStaRec->ucStaState != STA_STATE_3) ||
        (ucNewState == STA_STATE_1 && prStaRec->ucStaState == STA_STATE_2)) {
        prStaRec->ucStaState = ucNewState;
        return;
    }

    fgNeedResp = FALSE;
    if (ucNewState == STA_STATE_3) {
		
        if (ucNewState != prStaRec->ucStaState) {
            fgNeedResp = TRUE;
            cnmDumpStaRec(prAdapter, prStaRec->ucIndex);
        }
	} else {
		if (ucNewState != prStaRec->ucStaState && prStaRec->ucStaState == STA_STATE_3) {
            qmDeactivateStaRec(prAdapter, prStaRec);
        }
        fgNeedResp = FALSE;
    }
    prStaRec->ucStaState = ucNewState;

    cnmStaSendUpdateCmd(prAdapter, prStaRec, fgNeedResp);

#if 1 
#if CFG_ENABLE_WIFI_DIRECT
    if (prAdapter->fgIsP2PRegistered && (IS_STA_IN_P2P(prStaRec))) {
        P_BSS_INFO_T    prBssInfo;

        prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);

        if (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {
            rlmUpdateParamsForAP(prAdapter, prBssInfo, FALSE);
        }
    }
#endif
#endif

    return;
}

static VOID
cnmStaRecHandleEventPkt(P_ADAPTER_T prAdapter, P_CMD_INFO_T prCmdInfo, PUINT_8 pucEventBuf)
{
    P_EVENT_ACTIVATE_STA_REC_T  prEventContent;
    P_STA_RECORD_T              prStaRec;

    prEventContent = (P_EVENT_ACTIVATE_STA_REC_T) pucEventBuf;
    prStaRec = cnmGetStaRecByIndex(prAdapter, prEventContent->ucStaRecIdx);

    if (prStaRec && prStaRec->ucStaState == STA_STATE_3 &&
	    !kalMemCmp(&prStaRec->aucMacAddr[0], &prEventContent->aucMacAddr[0], MAC_ADDR_LEN)) {

        qmActivateStaRec(prAdapter, prStaRec);
    }

}

static VOID cnmStaSendUpdateCmd(P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec, BOOLEAN fgNeedResp)
{
    P_CMD_UPDATE_STA_RECORD_T   prCmdContent;
    WLAN_STATUS                 rStatus;

	if (!prAdapter) {
        return;
    }

	if (!prStaRec) {
		DBGLOG(MEM, WARN, ("%s: StaRec is NULL, skip!\n", __func__));
        return;
    }

	if (!prStaRec->fgIsInUse) {
		DBGLOG(MEM, WARN,
		       ("%s: StaRec[%u] is not in use, skip!\n", __func__, prStaRec->ucIndex));
        return;
    }


	prCmdContent = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, sizeof(CMD_UPDATE_STA_RECORD_T));

    
    if (!prCmdContent) {
		DBGLOG(MEM, WARN,
		       ("%s: CMD_ID_UPDATE_STA_RECORD command allocation failed\n", __func__));
        return;
    }

    
    kalMemZero(prCmdContent, sizeof(CMD_UPDATE_STA_RECORD_T));

    prCmdContent->ucStaIndex = prStaRec->ucIndex;
    prCmdContent->ucStaType = (UINT_8) prStaRec->eStaType;
	kalMemCopy(&prCmdContent->aucMacAddr[0], &prStaRec->aucMacAddr[0], MAC_ADDR_LEN);
    prCmdContent->u2AssocId = prStaRec->u2AssocId;
    prCmdContent->u2ListenInterval = prStaRec->u2ListenInterval;
    prCmdContent->ucBssIndex = prStaRec->ucBssIndex;

    prCmdContent->ucDesiredPhyTypeSet = prStaRec->ucDesiredPhyTypeSet;
    prCmdContent->u2DesiredNonHTRateSet = prStaRec->u2DesiredNonHTRateSet;
    prCmdContent->u2BSSBasicRateSet = prStaRec->u2BSSBasicRateSet;
    prCmdContent->ucMcsSet = prStaRec->ucMcsSet;
    prCmdContent->ucSupMcs32 = (UINT_8) prStaRec->fgSupMcs32;
    prCmdContent->u2HwDefaultFixedRateCode = prStaRec->u2HwDefaultFixedRateCode;

	kalMemCopy(prCmdContent->aucRxMcsBitmask, prStaRec->aucRxMcsBitmask,
		   sizeof(prCmdContent->aucRxMcsBitmask) );
    prCmdContent->u2RxHighestSupportedRate = prStaRec->u2RxHighestSupportedRate;
    prCmdContent->u4TxRateInfo = prStaRec->u4TxRateInfo;

    
    prCmdContent->u2HtCapInfo = prStaRec->u2HtCapInfo;
    prCmdContent->ucNeedResp = (UINT_8) fgNeedResp;

#if !CFG_SLT_SUPPORT
	if (prAdapter->rWifiVar.eRateSetting != FIXED_RATE_NONE) {
        
        nicUpdateRateParams(prAdapter,
                prAdapter->rWifiVar.eRateSetting,
                &(prCmdContent->ucDesiredPhyTypeSet),
                &(prCmdContent->u2DesiredNonHTRateSet),
                &(prCmdContent->u2BSSBasicRateSet),
                &(prCmdContent->ucMcsSet),
				    &(prCmdContent->ucSupMcs32), &(prCmdContent->u2HtCapInfo));
    }
#endif

    prCmdContent->ucIsQoS = prStaRec->fgIsQoS;
    prCmdContent->ucIsUapsdSupported = prStaRec->fgIsUapsdSupported;
    prCmdContent->ucStaState = prStaRec->ucStaState;

    prCmdContent->ucAmpduParam = prStaRec->ucAmpduParam;
    prCmdContent->u2HtExtendedCap = prStaRec->u2HtExtendedCap;
    prCmdContent->u4TxBeamformingCap = prStaRec->u4TxBeamformingCap;
    prCmdContent->ucAselCap = prStaRec->ucAselCap;
    prCmdContent->ucRCPI = prStaRec->ucRCPI;

    prCmdContent->u4VhtCapInfo = prStaRec->u4VhtCapInfo;
    prCmdContent->u2VhtRxMcsMap = prStaRec->u2VhtRxMcsMap;
    prCmdContent->u2VhtRxHighestSupportedDataRate = prStaRec->u2VhtRxHighestSupportedDataRate;
	prCmdContent->u2VhtTxMcsMap = prStaRec->u2VhtTxMcsMap;
    prCmdContent->u2VhtTxHighestSupportedDataRate = prStaRec->u2VhtTxHighestSupportedDataRate;

    prCmdContent->ucUapsdAc = prStaRec->ucBmpTriggerAC | (prStaRec->ucBmpDeliveryAC << 4);
    prCmdContent->ucUapsdSp = prStaRec->ucUapsdSp;

    prCmdContent->ucWlanIndex = prStaRec->ucWlanIndex;
    prCmdContent->ucBMCWlanIndex = prStaRec->ucBMCWlanIndex;


	prCmdContent->ucTrafficDataType = prStaRec->ucTrafficDataType;
    prCmdContent->ucTxGfMode = prStaRec->ucTxGfMode;
    prCmdContent->ucTxSgiMode = prStaRec->ucTxSgiMode;    
    prCmdContent->ucTxStbcMode = prStaRec->ucTxStbcMode;
    prCmdContent->u4FixedPhyRate = prStaRec->u4FixedPhyRate;
    prCmdContent->u2MaxLinkSpeed = prStaRec->u2MaxLinkSpeed;
    prCmdContent->u2MinLinkSpeed = prStaRec->u2MinLinkSpeed;
    prCmdContent->u4Flags = prStaRec->u4Flags;

    prCmdContent->ucTxAmpdu = FEATURE_OPT_IN_COMMAND(prAdapter->rWifiVar.ucAmpduTx);
    prCmdContent->ucRxAmpdu = FEATURE_OPT_IN_COMMAND(prAdapter->rWifiVar.ucAmpduRx);
    
    prCmdContent->ucTxBaSize = prAdapter->rWifiVar.ucTxBaSize;

    if(prStaRec->ucDesiredPhyTypeSet & PHY_TYPE_SET_802_11AC)
        prCmdContent->ucRxBaSize = prAdapter->rWifiVar.ucRxVhtBaSize;
    else
        prCmdContent->ucRxBaSize = prAdapter->rWifiVar.ucRxHtBaSize;
    
    
	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucSigTaRts)) {
		if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucDynBwRts)) {
            prCmdContent->ucRtsPolicy = RTS_POLICY_DYNAMIC_BW;
		} else {
            prCmdContent->ucRtsPolicy = RTS_POLICY_STATIC_BW;
        }
	} else {
        prCmdContent->ucRtsPolicy = RTS_POLICY_LEGACY;
    }

    DBGLOG(REQ, INFO, ("Update StaRec[%u] WIDX[%u] State[%u] Type[%u] BssIdx[%u] AID[%u]\n", 
        prCmdContent->ucStaIndex, 
        prCmdContent->ucWlanIndex,
        prCmdContent->ucStaState,
        prCmdContent->ucStaType,
			   prCmdContent->ucBssIndex, prCmdContent->u2AssocId));
    
     DBGLOG(REQ, INFO, ("Update StaRec[%u] QoS[%u] UAPSD[%u] BMCWIDX[%u]\n", 
        prCmdContent->ucStaIndex, 
        prCmdContent->ucIsQoS,
			   prCmdContent->ucIsUapsdSupported, prCmdContent->ucBMCWlanIndex));
     
	rStatus = wlanSendSetQueryCmd(prAdapter,	
                CMD_ID_UPDATE_STA_RECORD,   
                TRUE,                       
                fgNeedResp,                
                FALSE,                      
				      fgNeedResp ? cnmStaRecHandleEventPkt : NULL, NULL,	
                sizeof(CMD_UPDATE_STA_RECORD_T),    
                (PUINT_8) prCmdContent,     
                NULL,                       
                0                           
                );

    cnmMemFree(prAdapter, prCmdContent);

	if (rStatus != WLAN_STATUS_PENDING) {
        DBGLOG(MEM, WARN, ("%s: CMD_ID_UPDATE_STA_RECORD result 0x%08x\n", __func__, rStatus));
    }
}

static VOID
cnmStaSendRemoveCmd(P_ADAPTER_T prAdapter,
		    ENUM_STA_REC_CMD_ACTION_T eActionType, UINT_8 ucStaRecIndex, UINT_8 ucBssIndex)
{
    CMD_REMOVE_STA_RECORD_T     rCmdContent;
    WLAN_STATUS                 rStatus;

    ASSERT(prAdapter);

    rCmdContent.ucActionType = (UINT_8) eActionType;
    rCmdContent.ucStaIndex = ucStaRecIndex;
    rCmdContent.ucBssIndex = ucBssIndex;
    rCmdContent.ucReserved = 0;

	rStatus = wlanSendSetQueryCmd(prAdapter,	
                CMD_ID_REMOVE_STA_RECORD,   
                TRUE,                       
                FALSE,                      
                FALSE,                      
                NULL,                       
                NULL,                       
                sizeof(CMD_REMOVE_STA_RECORD_T),    
                (PUINT_8) &rCmdContent,     
                NULL,                       
                0                           
                );

	if (rStatus != WLAN_STATUS_PENDING) {
        DBGLOG(MEM, WARN, ("%s: CMD_ID_UPDATE_STA_RECORD result 0x%08x\n", __func__, rStatus));
    }
}

PUINT_8 cnmStaRecGetTypeString(ENUM_STA_TYPE_T eStaType)
{
    PUINT_8 pucTypeString = NULL;

	if (eStaType & STA_TYPE_LEGACY_MASK) {
        pucTypeString = apucStaRecType[STA_TYPE_LEGACY_INDEX];
    }
	if (eStaType & STA_TYPE_P2P_MASK) {
        pucTypeString = apucStaRecType[STA_TYPE_P2P_INDEX];
    }
	if (eStaType & STA_TYPE_BOW_MASK) {
        pucTypeString = apucStaRecType[STA_TYPE_BOW_INDEX];
    }

    return pucTypeString;
}

PUINT_8 cnmStaRecGetRoleString(ENUM_STA_TYPE_T eStaType)
{
    PUINT_8 pucRoleString = NULL;

	if (eStaType & STA_TYPE_ADHOC_MASK) {
        pucRoleString = apucStaRecRole[STA_ROLE_ADHOC_INDEX - STA_ROLE_BASE_INDEX];
    }
	if (eStaType & STA_TYPE_CLIENT_MASK) {
        pucRoleString = apucStaRecRole[STA_ROLE_CLIENT_INDEX - STA_ROLE_BASE_INDEX];
    }
	if (eStaType & STA_TYPE_AP_MASK) {
        pucRoleString = apucStaRecRole[STA_ROLE_AP_INDEX - STA_ROLE_BASE_INDEX];
    }
	if (eStaType & STA_TYPE_DLS_MASK) {
        pucRoleString = apucStaRecRole[STA_ROLE_DLS_INDEX - STA_ROLE_BASE_INDEX];
    }   

    return pucRoleString;
}

VOID cnmDumpStaRec(IN P_ADAPTER_T prAdapter, IN UINT_8 ucStaRecIdx)
{
    UINT_8 ucWTEntry;
    UINT_32 i;
    P_BSS_INFO_T prBssInfo;
    P_STA_RECORD_T prStaRec;

    DEBUGFUNC("cnmDumpStaRec");

    prStaRec = cnmGetStaRecByIndex(prAdapter, ucStaRecIdx);

	if (!prStaRec) {
        DBGLOG(SW4, INFO, ("Invalid StaRec index[%u], skip dump!\n", ucStaRecIdx));
        return;
    }
    
    ucWTEntry = prStaRec->ucWlanIndex;
    prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);
    
    ASSERT(prBssInfo);

    DBGLOG(SW4, INFO, ("============= DUMP STA[%u] ===========\n", ucStaRecIdx));

	DBGLOG(SW4, INFO,
	       ("STA_IDX[%u] BSS_IDX[%u] MAC[" MACSTR "] TYPE[%s %s] WTBL[%u] USED[%u] State[%u]\n",
		prStaRec->ucIndex, prStaRec->ucBssIndex, MAC2STR(prStaRec->aucMacAddr),
        cnmStaRecGetTypeString(prStaRec->eStaType),
		cnmStaRecGetRoleString(prStaRec->eStaType), ucWTEntry, prStaRec->fgIsInUse,
        prStaRec->ucStaState));

    DBGLOG(SW4, INFO, ("QoS[%u] HT/VHT[%u/%u] AID[%u] WMM[%u] UAPSD[%u] SEC[%u]\n",
        prStaRec->fgIsQoS,
			   (prStaRec->ucDesiredPhyTypeSet & PHY_TYPE_SET_802_11N) ? TRUE : FALSE,
			   (prStaRec->ucDesiredPhyTypeSet & PHY_TYPE_SET_802_11AC) ? TRUE : FALSE,
        prStaRec->u2AssocId,
        prStaRec->fgIsWmmSupported,
			   prStaRec->fgIsUapsdSupported, secIsProtectedBss(prAdapter, prBssInfo)));
    
    DBGLOG(SW4, INFO, ("PhyTypeSet: BSS[0x%02x] Desired[0x%02x] NonHtBasic[0x%02x]\n",
        prBssInfo->ucPhyTypeSet, 
			   prStaRec->ucDesiredPhyTypeSet, prStaRec->ucNonHTBasicPhyType));

	DBGLOG(SW4, INFO,
	       ("RateSet: BssBasic[0x%04x] Operational[0x%04x] DesiredNonHT[0x%02x] DeafultFixedRate[0x%02x]\n",
		prBssInfo->u2BSSBasicRateSet, prStaRec->u2OperationalRateSet,
		prStaRec->u2DesiredNonHTRateSet, prStaRec->u2HwDefaultFixedRateCode));

    DBGLOG(SW4, INFO, ("HT Cap[0x%04x] ExtCap[0x%04x] BeemCap[0x%08x] MCS[0x%02x] MCS32[%u]\n",
        prStaRec->u2HtCapInfo,
        prStaRec->u2HtExtendedCap,
			   prStaRec->u4TxBeamformingCap, prStaRec->ucMcsSet, prStaRec->fgSupMcs32));
    
    DBGLOG(SW4, INFO, ("VHT Cap[0x%08x] TxMCS[0x%04x] RxMCS[0x%04x]\n",
        prStaRec->u4VhtCapInfo,
			   prStaRec->u2VhtTxMcsMap, prStaRec->u2VhtRxMcsMap));

    DBGLOG(SW4, INFO, ("RCPI[%u] InPS[%u] TxAllowed[%u] KeyRdy[%u] AMPDU T/R[%u/%u]\n",
        prStaRec->ucRCPI,
        prStaRec->fgIsInPS,
        prStaRec->fgIsTxAllowed,
			   prStaRec->fgIsTxKeyReady, prStaRec->fgTxAmpduEn, prStaRec->fgRxAmpduEn));

    DBGLOG(SW4, INFO, ("TxQ LEN TC[0~5] [%03u:%03u:%03u:%03u:%03u:%03u]\n", 
        prStaRec->arTxQueue[0].u4NumElem,
        prStaRec->arTxQueue[1].u4NumElem,
        prStaRec->arTxQueue[2].u4NumElem,
        prStaRec->arTxQueue[3].u4NumElem,
			   prStaRec->arTxQueue[4].u4NumElem, prStaRec->arTxQueue[5].u4NumElem));

    DBGLOG(SW4, INFO, ("BMP AC Delivery/Trigger[%02x/%02x]\n", 
			   prStaRec->ucBmpDeliveryAC, prStaRec->ucBmpTriggerAC));
    
    DBGLOG(SW4, INFO, ("FreeQuota: Total[%u] Delivery/NonDelivery[%u/%u]\n", 
        prStaRec->ucFreeQuota,
			   prStaRec->ucFreeQuotaForDelivery, prStaRec->ucFreeQuotaForNonDelivery));

	for (i = 0; i < CFG_RX_MAX_BA_TID_NUM; i++) {
		if (prStaRec->aprRxReorderParamRefTbl[i]) {
			DBGLOG(SW4, INFO,
			       ("<Rx BA Entry TID[%u]>\n",
				prStaRec->aprRxReorderParamRefTbl[i]->ucTid));
			DBGLOG(SW4, INFO,
			       (" Valid[%u] WinStart/End[%u/%u] WinSize[%u] ReOrderQueLen[%u]\n",
                prStaRec->aprRxReorderParamRefTbl[i]->fgIsValid, 
                prStaRec->aprRxReorderParamRefTbl[i]->u2WinStart,
                prStaRec->aprRxReorderParamRefTbl[i]->u2WinEnd,
                prStaRec->aprRxReorderParamRefTbl[i]->u2WinSize,
                prStaRec->aprRxReorderParamRefTbl[i]->rReOrderQue.u4NumElem));
			DBGLOG(SW4, INFO,
			       (" Bubble Exist[%u] SN[%u]\n",
                prStaRec->aprRxReorderParamRefTbl[i]->fgHasBubble,
                prStaRec->aprRxReorderParamRefTbl[i]->u2FirstBubbleSn));           
        }
    }

    DBGLOG(SW4, INFO, ("============= DUMP END ===========\n"));

}

VOID cnmDumpMemoryStatus(IN P_ADAPTER_T prAdapter)
{
    P_BUF_INFO_T        prBufInfo;

#if CFG_DBG_MGT_BUF
    DBGLOG(SW4, INFO, ("============= DUMP Memory Status =============\n"));

    DBGLOG(SW4, INFO, ("Dynamic alloc OS memory count: alloc[%u] free[%u]\n", 
        prAdapter->u4MemAllocDynamicCount, prAdapter->u4MemFreeDynamicCount));


    prBufInfo = &prAdapter->rMsgBufInfo;
    DBGLOG(SW4, INFO, ("MSG memory count: alloc[%u] free[%u] null[%u] bitmap[0x%08x]\n", 
			   prBufInfo->u4AllocCount, prBufInfo->u4FreeCount,
			   prBufInfo->u4AllocNullCount, (UINT_32) prBufInfo->rFreeBlocksBitmap));

    prBufInfo = &prAdapter->rMgtBufInfo;
    DBGLOG(SW4, INFO, ("MGT memory count: alloc[%u] free[%u] null[%u] bitmap[0x%08x]\n", 
			   prBufInfo->u4AllocCount, prBufInfo->u4FreeCount,
			   prBufInfo->u4AllocNullCount, (UINT_32) prBufInfo->rFreeBlocksBitmap));

    DBGLOG(SW4, INFO, ("============= DUMP END =============\n"));

#endif    
}

#if CFG_SUPPORT_TDLS
/*!
* \brief This routine is called to add a peer record.
*
* \param[in] pvAdapter Pointer to the Adapter structure.
* \param[out] pvQueryBuf A pointer to the buffer that holds the result of
*                           the query.
* \param[in] u4QueryBufLen The length of the query buffer.
* \param[out] pu4QueryInfoLen If the call is successful, returns the number of
*                            bytes written into the query buffer. If the call
*                            failed due to invalid length of the query buffer,
*                            returns the amount of storage needed.
*
* \retval WLAN_STATUS_SUCCESS
* \retval WLAN_STATUS_INVALID_LENGTH
*/
WLAN_STATUS			
cnmPeerAdd(P_ADAPTER_T prAdapter, PVOID pvSetBuffer, UINT_32 u4SetBufferLen, PUINT_32 pu4SetInfoLen)
{
	CMD_PEER_ADD_T *prCmd;
	BSS_INFO_T *prAisBssInfo;
	STA_RECORD_T *prStaRec;


 

	

	if ((prAdapter == NULL) || (pvSetBuffer == NULL) || (pu4SetInfoLen == NULL)) {
		return TDLS_STATUS_FAIL;
	}

	
	*pu4SetInfoLen = sizeof(CMD_PEER_ADD_T);
	prCmd = (CMD_PEER_ADD_T *) pvSetBuffer;

	prAisBssInfo = prAdapter->prAisBssInfo;	
	prStaRec = cnmGetStaRecByAddress(prAdapter,
								(UINT_8) prAdapter->prAisBssInfo->ucBssIndex,
								prCmd->aucPeerMac);

	if (prStaRec == NULL) {
		prStaRec =
		    cnmStaRecAlloc(prAdapter, STA_TYPE_DLS_PEER,
				   (UINT_8) prAdapter->prAisBssInfo->ucBssIndex, prCmd->aucPeerMac);
		
		
		if (prStaRec == NULL) {
			return TDLS_STATUS_RESOURCES;
		}

		
		if (prAisBssInfo) {
			if (prAisBssInfo->ucBssIndex) {
				prStaRec->ucBssIndex = prAisBssInfo->ucBssIndex; 
			}
		}		

		
		
		COPY_MAC_ADDR(prStaRec->aucMacAddr, prCmd->aucPeerMac);

		prStaRec->u2BSSBasicRateSet = prAisBssInfo->u2BSSBasicRateSet;
		
		prStaRec->u2DesiredNonHTRateSet = prAdapter->rWifiVar.ucAvailablePhyTypeSet;
		
		prStaRec->u2OperationalRateSet = prAisBssInfo->u2OperationalRateSet;
		prStaRec->ucPhyTypeSet = prAisBssInfo->ucPhyTypeSet;
		prStaRec->eStaType = prCmd->eStaType;

		
		cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);
		
	} else {
		if ((prStaRec->ucStaState > STA_STATE_1) && (IS_DLS_STA(prStaRec))) {
			
			cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);
		}
	}
	return TDLS_STATUS_SUCCESS;
}


/*!
* \brief This routine is called to update a peer record.
*
* \param[in] pvAdapter Pointer to the Adapter structure.
* \param[out] pvQueryBuf A pointer to the buffer that holds the result of
*                           the query.
* \param[in] u4QueryBufLen The length of the query buffer.
* \param[out] pu4QueryInfoLen If the call is successful, returns the number of
*                            bytes written into the query buffer. If the call
*                            failed due to invalid length of the query buffer,
*                            returns the amount of storage needed.
*
* \retval WLAN_STATUS_SUCCESS
* \retval WLAN_STATUS_INVALID_LENGTH
*/
WLAN_STATUS			
cnmPeerUpdate(P_ADAPTER_T prAdapter,
	      PVOID pvSetBuffer, UINT_32 u4SetBufferLen, PUINT_32 pu4SetInfoLen)
{

	CMD_PEER_UPDATE_T *prCmd;
	BSS_INFO_T *prAisBssInfo;
	STA_RECORD_T *prStaRec;
	UINT_8 ucNonHTPhyTypeSet;			
	
	UINT_16 u2OperationalRateSet = 0;
	
	UINT_8 ucRate;
	UINT_16 i, j;


 
	
	if ((!prAdapter) || (!pvSetBuffer) || (!pu4SetInfoLen)) {
  
		return TDLS_STATUS_FAIL;
	}

	
	*pu4SetInfoLen = sizeof(CMD_PEER_ADD_T);
	prCmd = (CMD_PEER_UPDATE_T *) pvSetBuffer;

	prAisBssInfo = prAdapter->prAisBssInfo;

	prStaRec = cnmGetStaRecByAddress(prAdapter,
								(UINT_8) prAdapter->prAisBssInfo->ucBssIndex,
								prCmd->aucPeerMac);


	if ((!prStaRec) || !(prStaRec->fgIsInUse)) {
 		return TDLS_STATUS_FAIL;
	}

	if (!IS_DLS_STA(prStaRec)) {
 		return TDLS_STATUS_FAIL;
	}

	if (prAisBssInfo) {
		if (prAisBssInfo->ucBssIndex) {
			prStaRec->ucBssIndex = prAisBssInfo->ucBssIndex; 
		}
	}
	

	
	GET_CURRENT_SYSTIME(&prStaRec->rUpdateTime);

	
	prStaRec->u2StatusCode = prCmd->u2StatusCode;
	prStaRec->u2AssocId = 0; 
	prStaRec->u2ListenInterval = 0; 
	prStaRec->fgIsQoS = TRUE;
	prStaRec->fgIsUapsdSupported = (prCmd->UapsdBitmap == 0) ? FALSE : TRUE;
	prStaRec->u4TxBeamformingCap = 0; 
	prStaRec->ucAselCap = 0; 
	prStaRec->ucRCPI = 0;
	prStaRec->ucBmpTriggerAC = prCmd->UapsdBitmap;
	prStaRec->ucBmpDeliveryAC = prCmd->UapsdBitmap;
	prStaRec->ucUapsdSp = prCmd->UapsdMaxSp;
	prStaRec->eStaType = prCmd->eStaType;


	
	if (prCmd->aucSupRate) {
		for (i = 0; i < prCmd->u2SupRateLen; i++) {	
			if (prCmd->aucSupRate[i]) {
				ucRate = prCmd->aucSupRate[i] & RATE_MASK;
				
				for (j = 0; j < sizeof(aucValidDataRate) / sizeof(UINT_8); j++) {
					if (ucRate == aucValidDataRate[j]) {
						u2OperationalRateSet |= BIT(j);
						break;
					}
				}
			}
				
		}
		
		prStaRec->u2OperationalRateSet = u2OperationalRateSet;
		prStaRec->u2BSSBasicRateSet = prAisBssInfo->u2BSSBasicRateSet;

		
		
		prStaRec->ucPhyTypeSet = 0;
				
		if (BAND_2G4 == prAisBssInfo->eBand) {
			if (prCmd->rHtCap.rMCS.arRxMask) {
				prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_HT;
			}
			
			
			if (!(prStaRec->u2BSSBasicRateSet & RATE_SET_BIT_HT_PHY)) {
				
				if ((prStaRec->u2OperationalRateSet & RATE_SET_OFDM)) {
					prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_ERP;
				}
			
				
				if (!(prStaRec->u2BSSBasicRateSet & RATE_SET_OFDM)) {
					
					if ((prStaRec->u2OperationalRateSet & RATE_SET_HR_DSSS)) {
						prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_HR_DSSS;
					}
				}
			}
		} else {
			if (prCmd->rVHtCap.u2CapInfo) {
				prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_VHT;
			} 

			if (prCmd->rHtCap.rMCS.arRxMask) {
				prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_HT;
			}
			
			
			if (!(prStaRec->u2BSSBasicRateSet & RATE_SET_BIT_HT_PHY)) {
				
				prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_OFDM;	
			}
		}

		if (IS_STA_IN_AIS(prStaRec)) {
			if (!
			    ((prAdapter->rWifiVar.rConnSettings.eEncStatus ==
			      ENUM_ENCRYPTION3_ENABLED)
			     || (prAdapter->rWifiVar.rConnSettings.eEncStatus ==
				 ENUM_ENCRYPTION3_KEY_ABSENT)
			     || (prAdapter->rWifiVar.rConnSettings.eEncStatus ==
				 ENUM_ENCRYPTION_DISABLED)
			     || (prAdapter->prGlueInfo->u2WSCAssocInfoIELen)
#if CFG_SUPPORT_WAPI
              	|| (prAdapter->prGlueInfo->u2WapiAssocInfoIESz)
#endif
)) {
 
                		prStaRec->ucPhyTypeSet &= ~PHY_TYPE_BIT_HT;
           			}
    	}
	
		prStaRec->ucDesiredPhyTypeSet =
		    prStaRec->ucPhyTypeSet & prAdapter->rWifiVar.ucAvailablePhyTypeSet;
    	ucNonHTPhyTypeSet = prStaRec->ucDesiredPhyTypeSet & PHY_TYPE_SET_802_11ABG;

   		
    	if (ucNonHTPhyTypeSet) {
    	    if (ucNonHTPhyTypeSet & PHY_TYPE_BIT_ERP) {
    	       	prStaRec->ucNonHTBasicPhyType = PHY_TYPE_ERP_INDEX;
			} else if (ucNonHTPhyTypeSet & PHY_TYPE_BIT_OFDM) {
    	      	prStaRec->ucNonHTBasicPhyType = PHY_TYPE_OFDM_INDEX;
			} else {
    	       	prStaRec->ucNonHTBasicPhyType = PHY_TYPE_HR_DSSS_INDEX;
    	    }
	
			prStaRec->fgHasBasicPhyType = TRUE;
		} else {
       		
       		ASSERT(prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11N);
        	{
        		
       			prStaRec->ucNonHTBasicPhyType = PHY_TYPE_HR_DSSS_INDEX;
   			}
	
			prStaRec->fgHasBasicPhyType = FALSE;
    	}

	} 

		
	
	
	if (prCmd->rHtCap.rMCS.arRxMask) {
		prAdapter->rWifiVar.eRateSetting = FIXED_RATE_NONE;	
		prStaRec->ucDesiredPhyTypeSet |= PHY_TYPE_BIT_HT;
		prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_HT;
		prStaRec->u2HtCapInfo = prCmd->rHtCap.u2CapInfo;
		prStaRec->ucAmpduParam = prCmd->rHtCap.ucAmpduParamsInfo;
		prStaRec->u2HtExtendedCap = prCmd->rHtCap.u2ExtHtCapInfo;
		prStaRec->u4TxBeamformingCap = prCmd->rHtCap.u4TxBfCapInfo;
		prStaRec->ucAselCap = prCmd->rHtCap.ucAntennaSelInfo;
		prStaRec->ucMcsSet = prCmd->rHtCap.rMCS.arRxMask[0];		
		prStaRec->fgSupMcs32 =
		    (prCmd->rHtCap.rMCS.arRxMask[32 / 8] & BIT(0)) ? TRUE : FALSE;
		kalMemCopy(prStaRec->aucRxMcsBitmask, prCmd->rHtCap.rMCS.arRxMask,
			   sizeof(prStaRec->aucRxMcsBitmask));
	}
	

	cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);

	return TDLS_STATUS_SUCCESS;
}

P_STA_RECORD_T
cnmGetTdlsPeerByAddress(P_ADAPTER_T prAdapter, UINT_8 ucBssIndex, UINT_8 aucPeerMACAddress[])
{
    P_STA_RECORD_T  prStaRec;
    UINT_16         i;

    ASSERT(prAdapter);
    ASSERT(aucPeerMACAddress);
 
	for (i = 0; i < CFG_STA_REC_NUM; i++) {
		prStaRec = &prAdapter->arStaRec[i];
		if (prStaRec) {
			if (prStaRec->fgIsInUse &&
			    prStaRec->eStaType == STA_TYPE_DLS_PEER &&
				EQUAL_MAC_ADDR(prStaRec->aucMacAddr, aucPeerMACAddress)) {
					break;
			}
		}
	}

	return prStaRec;
}

#endif
