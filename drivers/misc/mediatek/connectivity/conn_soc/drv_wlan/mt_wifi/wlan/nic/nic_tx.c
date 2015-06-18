





#include "precomp.h"







VOID
nicTxInitialize (
    IN P_ADAPTER_T  prAdapter
    )
{
    P_TX_CTRL_T prTxCtrl;
    PUINT_8 pucMemHandle;
    P_MSDU_INFO_T prMsduInfo;
    UINT_32 i;
    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicTxInitialize");

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    
    nicTxResetResource(prAdapter);

#if CFG_SDIO_TX_AGG
    prTxCtrl->pucTxCoalescingBufPtr = prAdapter->pucCoalescingBufCached;
#endif 

    
    QUEUE_INITIALIZE(&prTxCtrl->rFreeMsduInfoList);

    pucMemHandle = prTxCtrl->pucTxCached;
    for (i = 0 ; i < CFG_TX_MAX_PKT_NUM ; i++) {
        prMsduInfo = (P_MSDU_INFO_T)pucMemHandle;
        kalMemZero(prMsduInfo, sizeof(MSDU_INFO_T));

        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
        QUEUE_INSERT_TAIL(&prTxCtrl->rFreeMsduInfoList, (P_QUE_ENTRY_T)prMsduInfo);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);

        pucMemHandle += ALIGN_4(sizeof(MSDU_INFO_T));
    }

    ASSERT(prTxCtrl->rFreeMsduInfoList.u4NumElem == CFG_TX_MAX_PKT_NUM);
    
    ASSERT((UINT_32)(pucMemHandle - prTxCtrl->pucTxCached) == prTxCtrl->u4TxCachedSize);

    QUEUE_INITIALIZE(&prTxCtrl->rTxMgmtTxingQueue);
    prTxCtrl->i4TxMgmtPendingNum = 0;

#if CFG_HIF_STATISTICS
    prTxCtrl->u4TotalTxAccessNum = 0;
    prTxCtrl->u4TotalTxPacketNum = 0;
#endif

    prTxCtrl->i4PendingFwdFrameCount = 0;

    qmInit(prAdapter);

    TX_RESET_ALL_CNTS(prTxCtrl);

    return;
} 


WLAN_STATUS
nicTxAcquireResource (
    IN P_ADAPTER_T  prAdapter,
    IN UINT_8       ucTC
    )
{
    P_TX_CTRL_T prTxCtrl;
    WLAN_STATUS u4Status = WLAN_STATUS_RESOURCES;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;


    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);


    if (prTxCtrl->rTc.aucFreeBufferCount[ucTC]) {

        
        prTxCtrl->rTc.aucFreeBufferCount[ucTC]--;

        DBGLOG(TX, EVENT, ("Acquire: TC = %d aucFreeBufferCount = %d\n",
            ucTC, prTxCtrl->rTc.aucFreeBufferCount[ucTC]));

        u4Status = WLAN_STATUS_SUCCESS;
    }
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);

    return u4Status;

}


WLAN_STATUS
nicTxPollingResource (
    IN P_ADAPTER_T  prAdapter,
    IN UINT_8       ucTC
    )
{
    P_TX_CTRL_T prTxCtrl;
    WLAN_STATUS u4Status = WLAN_STATUS_FAILURE;
    INT_32 i = NIC_TX_RESOURCE_POLLING_TIMEOUT;
    UINT_32 au4WTSR[2];

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    if (ucTC >= TC_NUM) {
        return WLAN_STATUS_FAILURE;
    }

    if (prTxCtrl->rTc.aucFreeBufferCount[ucTC] > 0) {
        return WLAN_STATUS_SUCCESS;
    }

    while (i-- > 0) {
        HAL_READ_TX_RELEASED_COUNT(prAdapter, au4WTSR);

        if(kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE
                || fgIsBusAccessFailed == TRUE) {
            u4Status = WLAN_STATUS_FAILURE;
            break;
        }
        else if (nicTxReleaseResource(prAdapter, (PUINT_8)au4WTSR)) {
            if (prTxCtrl->rTc.aucFreeBufferCount[ucTC] > 0) {
                u4Status = WLAN_STATUS_SUCCESS;
                break;
            }
            else {
                kalMsleep(NIC_TX_RESOURCE_POLLING_DELAY_MSEC);
            }
        }
        else {
            kalMsleep(NIC_TX_RESOURCE_POLLING_DELAY_MSEC);
        }
    }

	if (i <= 0 && ucTC == TC4_INDEX) {
		DBGLOG(TX, ERROR, ("polling Tx resource for Tc4 timeout\n"));
        glDumpConnSysCpuInfo(prAdapter->prGlueInfo);
	}
	
#if DBG
    {
        INT_32 i4Times = NIC_TX_RESOURCE_POLLING_TIMEOUT - (i+1);

        if (i4Times) {
            DBGLOG(TX, TRACE, ("Polling MCR_WTSR delay %d times, %d msec\n",
                i4Times, (i4Times * NIC_TX_RESOURCE_POLLING_DELAY_MSEC)));
        }
    }
#endif 

    return u4Status;

} 


BOOLEAN
nicTxReleaseResource (
    IN P_ADAPTER_T  prAdapter,
    IN UINT_8*      aucTxRlsCnt
    )
{
    PUINT_32 pu4Tmp = (PUINT_32)aucTxRlsCnt;
    P_TX_CTRL_T prTxCtrl;
    BOOLEAN bStatus = FALSE;
    UINT_32 i;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    if (pu4Tmp[0] | pu4Tmp[1]) {

        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);
        for (i = 0; i < TC_NUM; i++) {
            prTxCtrl->rTc.aucFreeBufferCount[i] += aucTxRlsCnt[i];
        }
		if (aucTxRlsCnt[TC4_INDEX] != 0 || aucTxRlsCnt[TC5_INDEX] != 0){
            DBGLOG(TX, STATE, ("Release: TC4 count %d, Free=%d; TC5 count %d, Free=%d\n",
                aucTxRlsCnt[TC4_INDEX], prTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX],
                aucTxRlsCnt[TC5_INDEX], prTxCtrl->rTc.aucFreeBufferCount[TC5_INDEX]));
        }
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);
#if 0
        for (i = 0; i < TC_NUM; i++) {
            DBGLOG(INIT, TRACE, ("aucFreeBufferCount[%d]: %d, aucMaxNumOfBuffer[%d]: %d\n",
                i, prTxCtrl->rTc.aucFreeBufferCount[i], i, prTxCtrl->rTc.aucMaxNumOfBuffer[i]));
        }
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[0]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[0]);
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[1]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[1]);
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[2]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[2]);
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[3]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[3]);
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[4]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[4]);
        DbgPrint("prTxCtrl->rTc.aucFreeBufferCount[5]=%d\n", prTxCtrl->rTc.aucFreeBufferCount[5]);
#endif
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC0_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC0_INDEX]);
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC1_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC1_INDEX]);
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC2_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC2_INDEX]);
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC3_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC3_INDEX]);
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC4_INDEX]);
        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[TC5_INDEX] <= prTxCtrl->rTc.aucMaxNumOfBuffer[TC5_INDEX]);
        bStatus = TRUE;
    }

    return bStatus;
} 


WLAN_STATUS
nicTxResetResource (
    IN P_ADAPTER_T  prAdapter
    )
{
    P_TX_CTRL_T prTxCtrl;

    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicTxResetResource");

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC0_INDEX] = NIC_TX_BUFF_COUNT_TC0;
    prTxCtrl->rTc.aucFreeBufferCount[TC0_INDEX] = NIC_TX_BUFF_COUNT_TC0;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC1_INDEX] = NIC_TX_BUFF_COUNT_TC1;
    prTxCtrl->rTc.aucFreeBufferCount[TC1_INDEX] = NIC_TX_BUFF_COUNT_TC1;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC2_INDEX] = NIC_TX_BUFF_COUNT_TC2;
    prTxCtrl->rTc.aucFreeBufferCount[TC2_INDEX] = NIC_TX_BUFF_COUNT_TC2;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC3_INDEX] = NIC_TX_BUFF_COUNT_TC3;
    prTxCtrl->rTc.aucFreeBufferCount[TC3_INDEX] = NIC_TX_BUFF_COUNT_TC3;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC4_INDEX] = NIC_TX_BUFF_COUNT_TC4;
    prTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX] = NIC_TX_BUFF_COUNT_TC4;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC5_INDEX] = NIC_TX_BUFF_COUNT_TC5;
    prTxCtrl->rTc.aucFreeBufferCount[TC5_INDEX] = NIC_TX_BUFF_COUNT_TC5;

    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);

    return WLAN_STATUS_SUCCESS;
}


UINT_8
nicTxGetResource (
    IN P_ADAPTER_T  prAdapter,
    IN UINT_8       ucTC
    )
{
    P_TX_CTRL_T prTxCtrl;

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    ASSERT(prTxCtrl);

    if (ucTC >= TC_NUM) {
        return 0;
    }
    else {
        return prTxCtrl->rTc.aucFreeBufferCount[ucTC];
    }
}


WLAN_STATUS
nicTxMsduInfoList (
    IN P_ADAPTER_T      prAdapter,
    IN P_MSDU_INFO_T    prMsduInfoListHead
    )
{
    P_MSDU_INFO_T prMsduInfo, prNextMsduInfo;
    QUE_T qDataPort0, qDataPort1;
    WLAN_STATUS status;

    ASSERT(prAdapter);
    ASSERT(prMsduInfoListHead);

    prMsduInfo = prMsduInfoListHead;

    QUEUE_INITIALIZE(&qDataPort0);
    QUEUE_INITIALIZE(&qDataPort1);

    
    while(prMsduInfo) {
        prNextMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);
#if DBG && 0
        LOG_FUNC("nicTxMsduInfoList Acquire TC %d net %u mac len %u len %u Type %u 1x %u 11 %u\n",
            prMsduInfo->ucTC,
                prMsduInfo->ucNetworkType,
                prMsduInfo->ucMacHeaderLength,
            prMsduInfo->u2FrameLength,
            prMsduInfo->ucPacketType,
            prMsduInfo->fgIs802_1x,
            prMsduInfo->fgIs802_11 );

        LOG_FUNC("Dest Mac: " MACSTR "\n",
                MAC2STR(prMsduInfo->aucEthDestAddr));
#endif

        
        
        switch(prMsduInfo->ucTC) {
        case TC0_INDEX:
        case TC1_INDEX:
        case TC2_INDEX:
        case TC3_INDEX:
        case TC5_INDEX: 
            QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo) = NULL;
            QUEUE_INSERT_TAIL(&qDataPort0, (P_QUE_ENTRY_T)prMsduInfo);
            status = nicTxAcquireResource(prAdapter, prMsduInfo->ucTC);
            ASSERT(status == WLAN_STATUS_SUCCESS)

            break;

        case TC4_INDEX: 
            QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo) = NULL;
            QUEUE_INSERT_TAIL(&qDataPort1, (P_QUE_ENTRY_T)prMsduInfo);

            status = nicTxAcquireResource(prAdapter, prMsduInfo->ucTC);
            ASSERT(status == WLAN_STATUS_SUCCESS)

            break;

        default:
            ASSERT(0);
            break;
        }

        prMsduInfo = prNextMsduInfo;
    }

    
    if(qDataPort0.u4NumElem > 0) {
        nicTxMsduQueue(prAdapter, 0, &qDataPort0);
    }

    if(qDataPort1.u4NumElem > 0) {
        nicTxMsduQueue(prAdapter, 1, &qDataPort1);
    }

    return WLAN_STATUS_SUCCESS;
}

#if CFG_ENABLE_PKT_LIFETIME_PROFILE

#if CFG_PRINT_RTP_PROFILE
PKT_PROFILE_T rPrevRoundLastPkt;

BOOLEAN
nicTxLifetimePrintCheckRTP (
    IN P_MSDU_INFO_T        prPrevProfileMsduInfo,
    IN P_PKT_PROFILE_T      prPrevRoundLastPkt,
    IN P_PKT_PROFILE_T      prPktProfile,
    IN OUT PBOOLEAN         pfgGotFirst,
    IN UINT_32              u4MaxDeltaTime,
    IN UINT_8               ucSnToBePrinted
    )
{
    BOOLEAN fgPrintCurPkt = FALSE;

    if(u4MaxDeltaTime) {
        
        if(!*pfgGotFirst) {
            *pfgGotFirst = TRUE;

            if(prPrevRoundLastPkt->fgIsValid) {
                if(CHK_PROFILES_DELTA(prPktProfile, prPrevRoundLastPkt, u4MaxDeltaTime)) {
                    PRINT_PKT_PROFILE(prPrevRoundLastPkt, "PR");
                    fgPrintCurPkt = TRUE;
                }
            }
        }

        
        if(prPrevProfileMsduInfo) {
            if(CHK_PROFILES_DELTA(prPktProfile, &prPrevProfileMsduInfo->rPktProfile, u4MaxDeltaTime)) {
                PRINT_PKT_PROFILE(&prPrevProfileMsduInfo->rPktProfile, "P");
                fgPrintCurPkt = TRUE;
            }
        }

        
        if(CHK_PROFILE_DELTA(prPktProfile, u4MaxDeltaTime)) {
            fgPrintCurPkt = TRUE;
        }
    }

    
    #if CFG_SUPPORT_WFD
    if((ucSnToBePrinted != 0) &&
       (prPktProfile->u2RtpSn % ucSnToBePrinted) == 0) {
        fgPrintCurPkt = TRUE;
    }
    #endif

    return fgPrintCurPkt;
}

BOOLEAN
nicTxLifetimePrintCheckSnOrder (
    IN P_MSDU_INFO_T        prPrevProfileMsduInfo,
    IN P_PKT_PROFILE_T      prPrevRoundLastPkt,
    IN P_PKT_PROFILE_T      prPktProfile,
    IN OUT PBOOLEAN         pfgGotFirst,
    IN UINT_8               ucLayer
    )
{
    BOOLEAN fgPrintCurPkt = FALSE;
    P_PKT_PROFILE_T prTarPktProfile = NULL;
    UINT_16 u2PredictSn = 0;
    UINT_16 u2CurrentSn = 0;
    UINT_8  aucNote[8];
    
    

    
    if((!*pfgGotFirst) && (prPrevRoundLastPkt->fgIsValid)) {
        *pfgGotFirst = TRUE;
        prTarPktProfile = prPrevRoundLastPkt;
        kalMemCopy(aucNote, "PR\0", 3);
            }
    
    else if (prPrevProfileMsduInfo) {
        prTarPktProfile = &prPrevProfileMsduInfo->rPktProfile;
        kalMemCopy(aucNote, "P\0", 2);
        }

    if(!prTarPktProfile) {
        return FALSE;
    }

    
    switch(ucLayer) {
         
        case 0:
            u2PredictSn = prTarPktProfile->u2IpSn + 1;
            u2CurrentSn = prPktProfile->u2IpSn;
            break;
        
        case 1:
        default:
            u2PredictSn = prTarPktProfile->u2RtpSn + 1;
            u2CurrentSn = prPktProfile->u2RtpSn;            
            break;

        }
    
    
    if(u2CurrentSn != u2PredictSn) {
        PRINT_PKT_PROFILE(prTarPktProfile, aucNote);
            fgPrintCurPkt = TRUE;
    }

    return fgPrintCurPkt;
}
#endif


VOID
nicTxReturnMsduInfoProfiling (
    IN P_ADAPTER_T    prAdapter,
    IN P_MSDU_INFO_T  prMsduInfoListHead
    )
{
    P_MSDU_INFO_T prMsduInfo = prMsduInfoListHead, prNextMsduInfo;
    P_PKT_PROFILE_T prPktProfile;
    UINT_16 u2MagicCode = 0;

    UINT_8 	ucDebugtMode=0;
    #if CFG_PRINT_RTP_PROFILE
    P_MSDU_INFO_T prPrevProfileMsduInfo = NULL;
    P_PKT_PROFILE_T prPrevRoundLastPkt = &rPrevRoundLastPkt;

    BOOLEAN fgPrintCurPkt = FALSE;
    BOOLEAN fgGotFirst = FALSE;
    UINT_8 ucSnToBePrinted = 0;

    UINT_32 u4MaxDeltaTime = 50; 
    #endif

    #if CFG_ENABLE_PER_STA_STATISTICS
    UINT_32 u4PktPrintPeriod = 0;
    #endif
    
    #if CFG_SUPPORT_WFD
    P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T)NULL;
    
    if(prAdapter->fgIsP2PRegistered) {        
        prWfdCfgSettings = &prAdapter->rWifiVar.prP2pFsmInfo->rWfdConfigureSettings;
        u2MagicCode = prWfdCfgSettings->u2WfdMaximumTp;
		ucDebugtMode=prAdapter->rWifiVar.prP2pFsmInfo->rWfdDebugSetting.ucWfdDebugMode;
        
            
        
    }
    #endif  

    #if CFG_PRINT_RTP_PROFILE
    if((u2MagicCode >= 0xF000)) {
        ucSnToBePrinted = (UINT_8)(u2MagicCode & BITS(0, 7));
        u4MaxDeltaTime = (UINT_8)(((u2MagicCode & BITS(8, 11)) >> 8) * 10);  
    }  
    else {
        ucSnToBePrinted = 0;
        u4MaxDeltaTime = 0;
    }    
	
    #endif

    #if CFG_ENABLE_PER_STA_STATISTICS
    if((u2MagicCode >= 0xE000) && (u2MagicCode < 0xF000)) {
        u4PktPrintPeriod = (UINT_32)((u2MagicCode & BITS(0, 7)) * 32);
    }  
    else {
        u4PktPrintPeriod = 0;
    }    
    #endif

    while(prMsduInfo) {
        prNextMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);
        prPktProfile = &prMsduInfo->rPktProfile;
            
        if(prPktProfile->fgIsValid) {

            prPktProfile->rHifTxDoneTimestamp = kalGetTimeTick();
            if (ucDebugtMode > 1) {
            
            
            #if CFG_PRINT_RTP_PROFILE
            #if CFG_PRINT_RTP_SN_SKIP
            fgPrintCurPkt = nicTxLifetimePrintCheckSnOrder(
                                prPrevProfileMsduInfo, 
                                prPrevRoundLastPkt, 
                                prPktProfile, 
                                &fgGotFirst,
                                0);
            #else
            fgPrintCurPkt = nicTxLifetimePrintCheckRTP(
                                prPrevProfileMsduInfo,
                                prPrevRoundLastPkt,
                                prPktProfile,
                                &fgGotFirst,
                                u4MaxDeltaTime,
                                ucSnToBePrinted);
            #endif

            
            if(fgPrintCurPkt && ucDebugtMode > 1) {
                PRINT_PKT_PROFILE(prPktProfile, "C");
            }

            prPrevProfileMsduInfo = prMsduInfo;   
            fgPrintCurPkt = FALSE;
            #endif
            }
            #if CFG_ENABLE_PER_STA_STATISTICS
            {
                P_STA_RECORD_T prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);
                UINT_32 u4DeltaTime;
                P_QUE_MGT_T prQM = &prAdapter->rQM;

                if(prStaRec) {
                    u4DeltaTime = (UINT_32)(prPktProfile->rHifTxDoneTimestamp - prPktProfile->rHardXmitArrivalTimestamp);
                        
                    prStaRec->u4TotalTxPktsNumber++;
                    prStaRec->u4TotalTxPktsTime += u4DeltaTime;
					if(u4DeltaTime > prStaRec->u4MaxTxPktsTime) {
						prStaRec->u4MaxTxPktsTime = u4DeltaTime;
					}
                    if(u4DeltaTime >= NIC_TX_TIME_THRESHOLD) {
                        prStaRec->u4ThresholdCounter++;
                    }
                                                         
                    if(u4PktPrintPeriod && (prStaRec->u4TotalTxPktsNumber >= u4PktPrintPeriod)) {
                        
                        DBGLOG(TX, TRACE, ("[%u]N[%4u]A[%5u]M[%4u]T[%4u]E[%4u]\n",
                            prStaRec->ucIndex,
                            prStaRec->u4TotalTxPktsNumber, 
                            (prStaRec->u4TotalTxPktsTime/prStaRec->u4TotalTxPktsNumber),
							prStaRec->u4MaxTxPktsTime,
							prStaRec->u4ThresholdCounter,
							prQM->au4QmTcResourceEmptyCounter[prStaRec->ucNetTypeIndex][TC2_INDEX]));
                        
                        prStaRec->u4TotalTxPktsNumber = 0;
                        prStaRec->u4TotalTxPktsTime = 0;
						prStaRec->u4MaxTxPktsTime = 0;
                        prStaRec->u4ThresholdCounter = 0;
                        prQM->au4QmTcResourceEmptyCounter[prStaRec->ucNetTypeIndex][TC2_INDEX] = 0;    
                    }                    
                }
            
            }
            #endif            
        }
            
        prMsduInfo = prNextMsduInfo;
    };

#if CFG_PRINT_RTP_PROFILE
    
    if(prPrevProfileMsduInfo) {
        prPktProfile = &prPrevProfileMsduInfo->rPktProfile;
        prPrevRoundLastPkt->u2IpSn = prPktProfile->u2IpSn;
        prPrevRoundLastPkt->u2RtpSn = prPktProfile->u2RtpSn;
        prPrevRoundLastPkt->rHardXmitArrivalTimestamp = prPktProfile->rHardXmitArrivalTimestamp;
        prPrevRoundLastPkt->rEnqueueTimestamp = prPktProfile->rEnqueueTimestamp;
        prPrevRoundLastPkt->rDequeueTimestamp = prPktProfile->rDequeueTimestamp;
        prPrevRoundLastPkt->rHifTxDoneTimestamp = prPktProfile->rHifTxDoneTimestamp;
        prPrevRoundLastPkt->ucTcxFreeCount = prPktProfile->ucTcxFreeCount;
        prPrevRoundLastPkt->fgIsPrinted = prPktProfile->fgIsPrinted;
        prPrevRoundLastPkt->fgIsValid = TRUE;
    }
#endif

    nicTxReturnMsduInfo(prAdapter, prMsduInfoListHead);

    return;
}


VOID
nicTxLifetimeRecordEn (
    IN P_ADAPTER_T     prAdapter,
    IN P_MSDU_INFO_T   prMsduInfo,
    IN P_NATIVE_PACKET prPacket
    )
{
    P_PKT_PROFILE_T prPktProfile = &prMsduInfo->rPktProfile;
    
    
    prPktProfile->fgIsValid = TRUE;

    
    prPktProfile->rHardXmitArrivalTimestamp = GLUE_GET_PKT_ARRIVAL_TIME(prPacket);

    
    prPktProfile->rEnqueueTimestamp = (OS_SYSTIME)kalGetTimeTick();

}

#if CFG_PRINT_RTP_PROFILE
UINT8 checkRtpAV(PUINT_8 data, UINT_32 size)
{
    PUINT_8 buf = data+12;
    while (buf+188 <= data+size) {
        int pid = ((buf[1] << 8 ) & 0x1F00) | (buf[2] & 0xFF);
        if (pid == 0 || pid == 0x100 || pid == 0x1000) {
            buf += 188;
        } 
        else if (pid == 0x1100){
            return 0;
        }
        else if (pid == 0x1011){
            return 1;
        }
    }
    return -1;
}

VOID
nicTxLifetimeCheckRTP (
    IN P_ADAPTER_T     prAdapter,
    IN P_MSDU_INFO_T   prMsduInfo,
    IN P_NATIVE_PACKET prPacket,
    IN UINT_32         u4PacketLen,
    IN UINT_8          ucNetworkType
    )
{
    struct sk_buff *prSkb = (struct sk_buff *) prPacket;
    UINT_16 u2EtherTypeLen;        
    PUINT_8 aucLookAheadBuf = NULL;
    P_PKT_PROFILE_T prPktProfile = &prMsduInfo->rPktProfile;

    
    UINT_8 ucRtpSnOffset = 30;
    
    P_TX_CTRL_T prTxCtrl;
#if CFG_SUPPORT_WFD
    P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T)NULL;
	P_WFD_DBG_CFG_SETTINGS_T prWfdDbgSettings = (P_WFD_DBG_CFG_SETTINGS_T)NULL;

    BOOLEAN fgEnProfiling = FALSE;

    if(prAdapter->fgIsP2PRegistered) {    
        prWfdCfgSettings = &prAdapter->rWifiVar.prP2pFsmInfo->rWfdConfigureSettings;
		prWfdDbgSettings = &prAdapter->rWifiVar.prP2pFsmInfo->rWfdDebugSetting;
        #if CFG_PRINT_RTP_SN_SKIP
        if(ucNetworkType == NETWORK_TYPE_P2P_INDEX) {
            fgEnProfiling = TRUE;
        }
        else
        #endif
        if(((prWfdCfgSettings->u2WfdMaximumTp >= 0xF000) || 
			(prWfdDbgSettings->ucWfdDebugMode>0))&&
              (ucNetworkType == NETWORK_TYPE_P2P_INDEX)) {
            fgEnProfiling = TRUE;
        }
    }

    if(fgEnProfiling == FALSE) {
        
        return;
    }
#endif
    
    prTxCtrl = &prAdapter->rTxCtrl;
    

    aucLookAheadBuf = prSkb->data;

    u2EtherTypeLen = (aucLookAheadBuf[ETH_TYPE_LEN_OFFSET] << 8) | (aucLookAheadBuf[ETH_TYPE_LEN_OFFSET + 1]);

    if ((u2EtherTypeLen == ETH_P_IP) &&
        (u4PacketLen >= LOOK_AHEAD_LEN)) {
        PUINT_8 pucIpHdr = &aucLookAheadBuf[ETH_HLEN];
		UINT_16 u2tmpIpSN=0;
        UINT_8 ucIpVersion;
    
        ucIpVersion = (pucIpHdr[0] & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
        if (ucIpVersion == IPVERSION) {
            if(pucIpHdr[IPV4_HDR_IP_PROTOCOL_OFFSET] == IP_PROTOCOL_UDP) {
                
               

                if(prPktProfile->fgIsValid == FALSE) {
                    nicTxLifetimeRecordEn(prAdapter, prMsduInfo, prPacket);
                }
                
                prPktProfile->fgIsPrinted = FALSE;

                prPktProfile->ucTcxFreeCount = prTxCtrl->rTc.aucFreeBufferCount[TC2_INDEX];

                
                prPktProfile->u2RtpSn = pucIpHdr[ucRtpSnOffset] << 8 | pucIpHdr[ucRtpSnOffset + 1];

                
                prPktProfile->u2IpSn = pucIpHdr[IPV4_HDR_IP_IDENTIFICATION_OFFSET] << 8 |
					                    pucIpHdr[IPV4_HDR_IP_IDENTIFICATION_OFFSET + 1];
				u2tmpIpSN=prPktProfile->u2IpSn;
				if(prWfdDbgSettings->ucWfdDebugMode==1) {	
					if((u2tmpIpSN & (prWfdDbgSettings->u2WfdSNShowPeiroid))==0)
						printk("RtpSn=%d IPId=%d j=%lu\n",prPktProfile->u2RtpSn,prPktProfile->u2IpSn,jiffies);
				}

                
            }
        }
    }

}
#endif
#if CFG_ENABLE_PER_STA_STATISTICS
VOID
nicTxLifetimeCheckByAC (
    IN P_ADAPTER_T     prAdapter,
    IN P_MSDU_INFO_T   prMsduInfo,
    IN P_NATIVE_PACKET prPacket,
    IN UINT_8          ucPriorityParam
    )
{
    switch(ucPriorityParam){
        
        
        
        
        
        
        
        
        
        case 4:
        case 5:
            
        
        case 6:
        case 7:
            nicTxLifetimeRecordEn(prAdapter, prMsduInfo, prPacket);
            break;
        default:
            break;
    }
}

#endif

VOID
nicTxLifetimeCheck (
    IN P_ADAPTER_T     prAdapter,
    IN P_MSDU_INFO_T   prMsduInfo,
    IN P_NATIVE_PACKET prPacket,
    IN UINT_8          ucPriorityParam,
    IN UINT_32         u4PacketLen,
    IN UINT_8          ucNetworkType
    )
{
    P_PKT_PROFILE_T prPktProfile = &prMsduInfo->rPktProfile;

    
    prPktProfile->fgIsValid = FALSE;

    #if CFG_ENABLE_PER_STA_STATISTICS
    nicTxLifetimeCheckByAC(prAdapter, prMsduInfo, prPacket, ucPriorityParam);
    #endif

    #if CFG_PRINT_RTP_PROFILE
    nicTxLifetimeCheckRTP(prAdapter, prMsduInfo, prPacket, u4PacketLen, ucNetworkType);
    #endif

}


#endif

WLAN_STATUS
nicTxMsduQueue (
    IN P_ADAPTER_T  prAdapter,
    UINT_8          ucPortIdx,
    P_QUE_T         prQue
    )
{
    P_MSDU_INFO_T prMsduInfo, prNextMsduInfo;
    HIF_TX_HEADER_T rHwTxHeader;
    P_NATIVE_PACKET prNativePacket;
    UINT_16 u2OverallBufferLength;
    UINT_8 ucEtherTypeOffsetInWord;
    PUINT_8 pucOutputBuf = (PUINT_8)NULL; 
    UINT_32 u4TxHdrSize;
    UINT_32 u4ValidBufSize;
    UINT_32 u4TotalLength;
    P_TX_CTRL_T prTxCtrl;
    QUE_T rFreeQueue;
#if CFG_TCP_IP_CHKSUM_OFFLOAD
    UINT_8 ucChksumFlag;
#endif

    ASSERT(prAdapter);
    ASSERT(ucPortIdx < 2);
    ASSERT(prQue);

    prTxCtrl = &prAdapter->rTxCtrl;
    u4ValidBufSize = prAdapter->u4CoalescingBufCachedSize;

#if CFG_HIF_STATISTICS
    prTxCtrl->u4TotalTxAccessNum++;
    prTxCtrl->u4TotalTxPacketNum += prQue->u4NumElem;
#endif

    QUEUE_INITIALIZE(&rFreeQueue);

    if(prQue->u4NumElem > 0) {
        prMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_HEAD(prQue);
        pucOutputBuf = prTxCtrl->pucTxCoalescingBufPtr;
        u4TotalLength = 0;

        while(prMsduInfo) {

#if (CFG_SUPPORT_TDLS_DBG == 1)
{
	struct sk_buff *prSkb = (struct sk_buff *)prMsduInfo->prPacket;
	UINT8 *pkt = prSkb->data;
	UINT16 u2Identifier;
	if ((*(pkt+12) == 0x08) && (*(pkt+13) == 0x00))
	{
		
		u2Identifier = ((*(pkt+18)) << 8) | (*(pkt+19));
		printk("<hif> %d\n", u2Identifier);
	}
}
#endif
#if (CFG_SUPPORT_MET_PROFILING == 1)
            kalMetProfilingFinish(prAdapter, prMsduInfo);
#endif
            kalMemZero(&rHwTxHeader, sizeof(rHwTxHeader));

            prNativePacket = prMsduInfo->prPacket;

            ASSERT(prNativePacket);

            u4TxHdrSize = TX_HDR_SIZE;

            u2OverallBufferLength = ((prMsduInfo->u2FrameLength + TX_HDR_SIZE) &
                    (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);

            
            rHwTxHeader.u2TxByteCount_UserPriority = u2OverallBufferLength;
            rHwTxHeader.u2TxByteCount_UserPriority |=
                ((UINT_16)prMsduInfo->ucUserPriority << HIF_TX_HDR_USER_PRIORITY_OFFSET);

            if (prMsduInfo->fgIs802_11) {
                ucEtherTypeOffsetInWord =
                    (TX_HDR_SIZE + prMsduInfo->ucMacHeaderLength + prMsduInfo->ucLlcLength) >> 1;
            }
            else {
                ucEtherTypeOffsetInWord =
                    ((ETHER_HEADER_LEN - ETHER_TYPE_LEN) + TX_HDR_SIZE) >> 1;
            }

            rHwTxHeader.ucEtherTypeOffset =
                ucEtherTypeOffsetInWord & HIF_TX_HDR_ETHER_TYPE_OFFSET_MASK;

            rHwTxHeader.ucResource_PktType_CSflags = (prMsduInfo->ucTC) << HIF_TX_HDR_RESOURCE_OFFSET;
            rHwTxHeader.ucResource_PktType_CSflags |=
                (UINT_8)(((prMsduInfo->ucPacketType) << HIF_TX_HDR_PACKET_TYPE_OFFSET) &
                        (HIF_TX_HDR_PACKET_TYPE_MASK));

#if CFG_TCP_IP_CHKSUM_OFFLOAD
            if (prMsduInfo->eSrc == TX_PACKET_OS
                    || prMsduInfo->eSrc == TX_PACKET_FORWARDING) {
                if (prAdapter->u4CSUMFlags &
                        (CSUM_OFFLOAD_EN_TX_TCP |
                         CSUM_OFFLOAD_EN_TX_UDP |
                         CSUM_OFFLOAD_EN_TX_IP)) {
                    kalQueryTxChksumOffloadParam(prNativePacket, &ucChksumFlag);

                    if (ucChksumFlag & TX_CS_IP_GEN) {
                        rHwTxHeader.ucResource_PktType_CSflags |= (UINT_8)HIF_TX_HDR_IP_CSUM;
                    }

                    if (ucChksumFlag & TX_CS_TCP_UDP_GEN) {
                        rHwTxHeader.ucResource_PktType_CSflags |= (UINT_8)HIF_TX_HDR_TCP_CSUM;
                    }
                }
            }
#endif 

            rHwTxHeader.u2LLH = prMsduInfo->u2PalLLH;
            rHwTxHeader.ucStaRecIdx = prMsduInfo->ucStaRecIndex;
            rHwTxHeader.ucForwardingType_SessionID_Reserved =
                (prMsduInfo->ucPsForwardingType) | ((prMsduInfo->ucPsSessionID) << HIF_TX_HDR_PS_SESSION_ID_OFFSET)
                | ((prMsduInfo->fgIsBurstEnd)? HIF_TX_HDR_BURST_END_MASK : 0);

            rHwTxHeader.ucWlanHeaderLength = (prMsduInfo->ucMacHeaderLength & HIF_TX_HDR_WLAN_HEADER_LEN_MASK);
            rHwTxHeader.ucPktFormtId_Flags =
                (prMsduInfo->ucFormatID & HIF_TX_HDR_FORMAT_ID_MASK)
                | ((prMsduInfo->ucNetworkType << HIF_TX_HDR_NETWORK_TYPE_OFFSET) & HIF_TX_HDR_NETWORK_TYPE_MASK)
                | ((prMsduInfo->fgIs802_1x << HIF_TX_HDR_FLAG_1X_FRAME_OFFSET) & HIF_TX_HDR_FLAG_1X_FRAME_MASK)
                | ((prMsduInfo->fgIs802_11 << HIF_TX_HDR_FLAG_802_11_FORMAT_OFFSET) & HIF_TX_HDR_FLAG_802_11_FORMAT_MASK);

            rHwTxHeader.u2SeqNo = prMsduInfo->u2AclSN;

            if(prMsduInfo->pfTxDoneHandler) {
                rHwTxHeader.ucPacketSeqNo = prMsduInfo->ucTxSeqNum;
                rHwTxHeader.ucAck_BIP_BasicRate = HIF_TX_HDR_NEED_ACK;
            }
            else {
                rHwTxHeader.ucPacketSeqNo = 0;
                rHwTxHeader.ucAck_BIP_BasicRate = 0;
            }

            if(prMsduInfo->fgIsBIP) {
                rHwTxHeader.ucAck_BIP_BasicRate |= HIF_TX_HDR_BIP;
            }

            if(prMsduInfo->fgIsBasicRate) {
                rHwTxHeader.ucAck_BIP_BasicRate |= HIF_TX_HDR_BASIC_RATE;
            }

#if CFG_ENABLE_PKT_LIFETIME_PROFILE
            if(prMsduInfo->rPktProfile.fgIsValid) {
                prMsduInfo->rPktProfile.rDequeueTimestamp = kalGetTimeTick();
            }
#endif            

			
			STATS_TX_TIME_TO_HIF(prMsduInfo, &rHwTxHeader);

#if CFG_SDIO_TX_AGG
            
            kalMemCopy(pucOutputBuf + u4TotalLength, &rHwTxHeader, u4TxHdrSize);
            u4TotalLength += u4TxHdrSize;

            if (prMsduInfo->eSrc == TX_PACKET_OS
                    || prMsduInfo->eSrc == TX_PACKET_FORWARDING) {
                kalCopyFrame(prAdapter->prGlueInfo,
                        prNativePacket,
                        pucOutputBuf + u4TotalLength);
            }
            else if(prMsduInfo->eSrc == TX_PACKET_MGMT) {
                kalMemCopy(pucOutputBuf + u4TotalLength,
                        prNativePacket,
                        prMsduInfo->u2FrameLength);
            }
            else {
                ASSERT(0);
            }

            u4TotalLength += ALIGN_4(prMsduInfo->u2FrameLength);

#else
            kalMemCopy(pucOutputBuf, &rHwTxHeader, u4TxHdrSize);

            
            if (prMsduInfo->eSrc == TX_PACKET_OS
                    || prMsduInfo->eSrc == TX_PACKET_FORWARDING) {
                kalCopyFrame(prAdapter->prGlueInfo,
                        prNativePacket,
                        pucOutputBuf + u4TxHdrSize);
            }
            else if(prMsduInfo->eSrc == TX_PACKET_MGMT) {
                kalMemCopy(pucOutputBuf + u4TxHdrSize,
                        prNativePacket,
                        prMsduInfo->u2FrameLength);
            }
            else {
                ASSERT(0);
            }

            ASSERT(u2OverallBufferLength <= u4ValidBufSize);

            HAL_WRITE_TX_PORT(prAdapter,
                    ucPortIdx,
                    (UINT_32)u2OverallBufferLength,
                    (PUINT_8)pucOutputBuf,
                    u4ValidBufSize);

            
#endif
            prNextMsduInfo = (P_MSDU_INFO_T)
                        QUEUE_GET_NEXT_ENTRY(&prMsduInfo->rQueEntry);

            if (prMsduInfo->eSrc == TX_PACKET_MGMT) {
                GLUE_DEC_REF_CNT(prTxCtrl->i4TxMgmtPendingNum);

                if (prMsduInfo->pfTxDoneHandler == NULL) {
                    cnmMgtPktFree(prAdapter, prMsduInfo);
                }
                else {
                    KAL_SPIN_LOCK_DECLARATION();
                    DBGLOG(INIT, TRACE,("Wait TxSeqNum:%d\n", prMsduInfo->ucTxSeqNum));
                    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);
                    QUEUE_INSERT_TAIL(&(prTxCtrl->rTxMgmtTxingQueue), (P_QUE_ENTRY_T)prMsduInfo);
                    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);
                }
            }
            else {
                
                QUEUE_INSERT_TAIL(&rFreeQueue, (P_QUE_ENTRY_T)prMsduInfo);

                if (prMsduInfo->eSrc == TX_PACKET_OS) {
                    kalSendComplete(prAdapter->prGlueInfo,
                            prNativePacket,
                            WLAN_STATUS_SUCCESS);
                }
                else if(prMsduInfo->eSrc == TX_PACKET_FORWARDING) {
                    GLUE_DEC_REF_CNT(prTxCtrl->i4PendingFwdFrameCount);
                }
            }

            prMsduInfo = prNextMsduInfo;
        }

#if CFG_SDIO_TX_AGG
        ASSERT(u4TotalLength <= u4ValidBufSize);

    #if CFG_DBG_GPIO_PINS
        {
            
            mtk_wcn_stp_debug_gpio_assert(IDX_TX_PORT_WRITE, DBG_TIE_LOW);
            kalUdelay(1);
            mtk_wcn_stp_debug_gpio_assert(IDX_TX_PORT_WRITE, DBG_TIE_HIGH);
        }
    #endif

        
        HAL_WRITE_TX_PORT(prAdapter,
                ucPortIdx,
                u4TotalLength,
                (PUINT_8)pucOutputBuf,
                u4ValidBufSize);
#endif

#if CFG_ENABLE_PKT_LIFETIME_PROFILE
        #if CFG_SUPPORT_WFD && CFG_PRINT_RTP_PROFILE && !CFG_ENABLE_PER_STA_STATISTICS
        do {          
            P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T)NULL;
            
            prWfdCfgSettings = &prAdapter->rWifiVar.prP2pFsmInfo->rWfdConfigureSettings;

            if((prWfdCfgSettings->u2WfdMaximumTp >= 0xF000)) {
                
                nicTxReturnMsduInfoProfiling(prAdapter, (P_MSDU_INFO_T)QUEUE_GET_HEAD(&rFreeQueue));
            }
            else {
                
        nicTxReturnMsduInfo(prAdapter, (P_MSDU_INFO_T)QUEUE_GET_HEAD(&rFreeQueue));
    }
        }while(FALSE);
        #else
            nicTxReturnMsduInfoProfiling(prAdapter, (P_MSDU_INFO_T)QUEUE_GET_HEAD(&rFreeQueue));
        #endif
#else
        
        nicTxReturnMsduInfo(prAdapter, (P_MSDU_INFO_T)QUEUE_GET_HEAD(&rFreeQueue));
#endif
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS
nicTxCmd (
    IN P_ADAPTER_T      prAdapter,
    IN P_CMD_INFO_T     prCmdInfo,
    IN UINT_8           ucTC
    )
{
    P_WIFI_CMD_T prWifiCmd;
    UINT_16 u2OverallBufferLength;
    PUINT_8 pucOutputBuf = (PUINT_8)NULL; 
    UINT_8 ucPortIdx;
    HIF_TX_HEADER_T rHwTxHeader;
    P_NATIVE_PACKET prNativePacket;
    UINT_8 ucEtherTypeOffsetInWord;
    P_MSDU_INFO_T prMsduInfo;
    P_TX_CTRL_T prTxCtrl;

    KAL_SPIN_LOCK_DECLARATION();


    ASSERT(prAdapter);
    ASSERT(prCmdInfo);

    prTxCtrl = &prAdapter->rTxCtrl;
    pucOutputBuf = prTxCtrl->pucTxCoalescingBufPtr;

    
    if (ucTC != TC4_INDEX) {
        ucPortIdx = 0;
    }
    else {
        
        ucPortIdx = 1;
    }

    if(prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME) {
        
        kalMemZero(&rHwTxHeader, sizeof(rHwTxHeader));

        prNativePacket = prCmdInfo->prPacket;

        ASSERT(prNativePacket);

        u2OverallBufferLength = TFCB_FRAME_PAD_TO_DW((prCmdInfo->u2InfoBufLen + TX_HDR_SIZE)
                & (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);

        rHwTxHeader.u2TxByteCount_UserPriority = ((prCmdInfo->u2InfoBufLen + TX_HDR_SIZE)
                & (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);
        ucEtherTypeOffsetInWord = ((ETHER_HEADER_LEN - ETHER_TYPE_LEN) + TX_HDR_SIZE) >> 1;

        rHwTxHeader.ucEtherTypeOffset =
            ucEtherTypeOffsetInWord & HIF_TX_HDR_ETHER_TYPE_OFFSET_MASK;

        rHwTxHeader.ucResource_PktType_CSflags = (ucTC << HIF_TX_HDR_RESOURCE_OFFSET);

        rHwTxHeader.ucStaRecIdx = prCmdInfo->ucStaRecIndex;
        rHwTxHeader.ucForwardingType_SessionID_Reserved = HIF_TX_HDR_BURST_END_MASK;

        rHwTxHeader.ucWlanHeaderLength = (ETH_HLEN & HIF_TX_HDR_WLAN_HEADER_LEN_MASK);
        rHwTxHeader.ucPktFormtId_Flags =
            (((UINT_8)(prCmdInfo->eNetworkType) << HIF_TX_HDR_NETWORK_TYPE_OFFSET) & HIF_TX_HDR_NETWORK_TYPE_MASK)
            | ((1 << HIF_TX_HDR_FLAG_1X_FRAME_OFFSET) & HIF_TX_HDR_FLAG_1X_FRAME_MASK);

        rHwTxHeader.u2SeqNo = 0;
        rHwTxHeader.ucPacketSeqNo = 0;
        rHwTxHeader.ucAck_BIP_BasicRate = 0;

        
        kalMemCopy((PVOID)&pucOutputBuf[0], (PVOID)&rHwTxHeader, TX_HDR_SIZE);

        
        kalCopyFrame(prAdapter->prGlueInfo,
                prNativePacket,
                pucOutputBuf + TX_HDR_SIZE);
    }
    else if(prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME) {
        prMsduInfo = (P_MSDU_INFO_T)prCmdInfo->prPacket;

        ASSERT(prMsduInfo->fgIs802_11 == TRUE);
        ASSERT(prMsduInfo->eSrc == TX_PACKET_MGMT);

        
        kalMemZero(&rHwTxHeader, sizeof(rHwTxHeader));

        u2OverallBufferLength = ((prMsduInfo->u2FrameLength + TX_HDR_SIZE) &
                                  (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);

        rHwTxHeader.u2TxByteCount_UserPriority = u2OverallBufferLength;
        rHwTxHeader.u2TxByteCount_UserPriority |=
                ((UINT_16)prMsduInfo->ucUserPriority << HIF_TX_HDR_USER_PRIORITY_OFFSET);

        ucEtherTypeOffsetInWord =
                (TX_HDR_SIZE + prMsduInfo->ucMacHeaderLength + prMsduInfo->ucLlcLength) >> 1;

        rHwTxHeader.ucEtherTypeOffset =
                ucEtherTypeOffsetInWord & HIF_TX_HDR_ETHER_TYPE_OFFSET_MASK;

        rHwTxHeader.ucResource_PktType_CSflags = (prMsduInfo->ucTC) << HIF_TX_HDR_RESOURCE_OFFSET;
        rHwTxHeader.ucResource_PktType_CSflags |=
                (UINT_8)(((prMsduInfo->ucPacketType) << HIF_TX_HDR_PACKET_TYPE_OFFSET) &
                                    (HIF_TX_HDR_PACKET_TYPE_MASK));

        rHwTxHeader.u2LLH = prMsduInfo->u2PalLLH;
        rHwTxHeader.ucStaRecIdx = prMsduInfo->ucStaRecIndex;
        rHwTxHeader.ucForwardingType_SessionID_Reserved =
            (prMsduInfo->ucPsForwardingType) | ((prMsduInfo->ucPsSessionID) << HIF_TX_HDR_PS_SESSION_ID_OFFSET)
            | ((prMsduInfo->fgIsBurstEnd)? HIF_TX_HDR_BURST_END_MASK : 0);

        rHwTxHeader.ucWlanHeaderLength = (prMsduInfo->ucMacHeaderLength & HIF_TX_HDR_WLAN_HEADER_LEN_MASK);
        rHwTxHeader.ucPktFormtId_Flags =
            (prMsduInfo->ucFormatID & HIF_TX_HDR_FORMAT_ID_MASK)
            | ((prMsduInfo->ucNetworkType << HIF_TX_HDR_NETWORK_TYPE_OFFSET) & HIF_TX_HDR_NETWORK_TYPE_MASK)
            | ((prMsduInfo->fgIs802_1x << HIF_TX_HDR_FLAG_1X_FRAME_OFFSET) & HIF_TX_HDR_FLAG_1X_FRAME_MASK)
            | ((prMsduInfo->fgIs802_11 << HIF_TX_HDR_FLAG_802_11_FORMAT_OFFSET) & HIF_TX_HDR_FLAG_802_11_FORMAT_MASK);

        rHwTxHeader.u2SeqNo = prMsduInfo->u2AclSN;

        if(prMsduInfo->pfTxDoneHandler) {
            rHwTxHeader.ucPacketSeqNo = prMsduInfo->ucTxSeqNum;
            rHwTxHeader.ucAck_BIP_BasicRate = HIF_TX_HDR_NEED_ACK;
        }
        else {
            rHwTxHeader.ucPacketSeqNo = 0;
            rHwTxHeader.ucAck_BIP_BasicRate = 0;
        }

        if(prMsduInfo->fgIsBIP) {
            rHwTxHeader.ucAck_BIP_BasicRate |= HIF_TX_HDR_BIP;
        }

        if(prMsduInfo->fgIsBasicRate) {
            rHwTxHeader.ucAck_BIP_BasicRate |= HIF_TX_HDR_BASIC_RATE;
        }

        
        kalMemCopy((PVOID)&pucOutputBuf[0], (PVOID)&rHwTxHeader, TX_HDR_SIZE);

        
        kalMemCopy(pucOutputBuf + TX_HDR_SIZE,
                prMsduInfo->prPacket,
                prMsduInfo->u2FrameLength);

        
        GLUE_DEC_REF_CNT(prTxCtrl->i4TxMgmtPendingNum);

        if (prMsduInfo->pfTxDoneHandler == NULL) {
            cnmMgtPktFree(prAdapter, prMsduInfo);
        }
        else {

            DBGLOG(INIT, TRACE,("Wait Cmd TxSeqNum:%d\n", prMsduInfo->ucTxSeqNum));

            KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);
            QUEUE_INSERT_TAIL(&(prTxCtrl->rTxMgmtTxingQueue), (P_QUE_ENTRY_T)prMsduInfo);
            KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);
        }
    }
    else {
        prWifiCmd = (P_WIFI_CMD_T)prCmdInfo->pucInfoBuffer;

        
        u2OverallBufferLength = TFCB_FRAME_PAD_TO_DW(
               (prCmdInfo->u2InfoBufLen) & (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);

        prWifiCmd->u2TxByteCount_UserPriority = u2OverallBufferLength;
        prWifiCmd->ucEtherTypeOffset = 0;
        prWifiCmd->ucResource_PktType_CSflags = (ucTC << HIF_TX_HDR_RESOURCE_OFFSET)
            | (UINT_8)((HIF_TX_PKT_TYPE_CMD << HIF_TX_HDR_PACKET_TYPE_OFFSET) & (HIF_TX_HDR_PACKET_TYPE_MASK));


        
        kalMemCopy((PVOID)&pucOutputBuf[0],
                   (PVOID)prCmdInfo->pucInfoBuffer,
                   prCmdInfo->u2InfoBufLen);

        ASSERT(u2OverallBufferLength <= prAdapter->u4CoalescingBufCachedSize);
    }

    
    HAL_WRITE_TX_PORT(prAdapter,
            ucPortIdx,
            (UINT_32)u2OverallBufferLength,
            (PUINT_8)pucOutputBuf,
            (UINT_32)prAdapter->u4CoalescingBufCachedSize);

    return WLAN_STATUS_SUCCESS;
} 


VOID
nicTxRelease (
    IN P_ADAPTER_T  prAdapter
    )
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prMsduInfo;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTxCtrl = &prAdapter->rTxCtrl;

    nicTxFlush(prAdapter);

    
    do {
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);
        QUEUE_REMOVE_HEAD(&prTxCtrl->rTxMgmtTxingQueue, prMsduInfo, P_MSDU_INFO_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TXING_MGMT_LIST);

        if(prMsduInfo) {
            
            ASSERT(prMsduInfo->eSrc == TX_PACKET_MGMT);
            ASSERT(prMsduInfo->pfTxDoneHandler != NULL);

            
            prMsduInfo->pfTxDoneHandler(prAdapter, prMsduInfo, TX_RESULT_LIFE_TIMEOUT);

            cnmMgtPktFree(prAdapter, prMsduInfo);
        }
        else {
            break;
        }
    } while(TRUE);

    return;
} 


VOID
nicProcessTxInterrupt(
    IN P_ADAPTER_T prAdapter
    )
{
    P_TX_CTRL_T prTxCtrl;
#if CFG_SDIO_INTR_ENHANCE
    P_SDIO_CTRL_T prSDIOCtrl;
#else
    UINT_32 au4TxCount[2];
#endif 

    ASSERT(prAdapter);

    prTxCtrl = &prAdapter->rTxCtrl;
    ASSERT(prTxCtrl);

     
#if CFG_SDIO_INTR_ENHANCE

    prSDIOCtrl = prAdapter->prSDIOCtrl;
    #if DBG
    
    #endif

    nicTxReleaseResource(prAdapter, (PUINT_8)&prSDIOCtrl->rTxInfo);
    kalMemZero(&prSDIOCtrl->rTxInfo, sizeof(prSDIOCtrl->rTxInfo));

#else

    HAL_MCR_RD(prAdapter, MCR_WTSR0, &au4TxCount[0]);
    HAL_MCR_RD(prAdapter, MCR_WTSR1, &au4TxCount[1]);
    DBGLOG(EMU, TRACE, ("MCR_WTSR0: 0x%x, MCR_WTSR1: 0x%x\n", au4TxCount[0], au4TxCount[1]));

    nicTxReleaseResource(prAdapter, (PUINT_8)au4TxCount);

#endif 

    nicTxAdjustTcq(prAdapter);

    
    if(kalGetTxPendingCmdCount(prAdapter->prGlueInfo) > 0
            || wlanGetTxPendingFrameCount(prAdapter) > 0) {
        kalSetEvent(prAdapter->prGlueInfo);
    }

    return;
} 


VOID
nicTxFreeMsduInfoPacket (
    IN P_ADAPTER_T    prAdapter,
    IN P_MSDU_INFO_T  prMsduInfoListHead
    )
{
    P_NATIVE_PACKET prNativePacket;
    P_MSDU_INFO_T prMsduInfo = prMsduInfoListHead;
    P_TX_CTRL_T prTxCtrl;


    ASSERT(prAdapter);
    ASSERT(prMsduInfoListHead);

    prTxCtrl = &prAdapter->rTxCtrl;

    while(prMsduInfo) {
        prNativePacket = prMsduInfo->prPacket;

        if(prMsduInfo->eSrc == TX_PACKET_OS) {
            kalSendComplete(prAdapter->prGlueInfo,
                    prNativePacket,
                    WLAN_STATUS_FAILURE);
        }
        else if(prMsduInfo->eSrc == TX_PACKET_MGMT) {
            P_MSDU_INFO_T prTempMsduInfo = prMsduInfo;
            if (prMsduInfo->pfTxDoneHandler) {
                prMsduInfo->pfTxDoneHandler(prAdapter, prMsduInfo, TX_RESULT_DROPPED_IN_DRIVER);
            }
            prMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);
            cnmMgtPktFree(prAdapter, prTempMsduInfo);
            continue;
        }
        else if(prMsduInfo->eSrc == TX_PACKET_FORWARDING) {
            GLUE_DEC_REF_CNT(prTxCtrl->i4PendingFwdFrameCount);
        }

        prMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);
    }

    return;
}


VOID
nicTxReturnMsduInfo (
    IN P_ADAPTER_T    prAdapter,
    IN P_MSDU_INFO_T  prMsduInfoListHead
    )
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prMsduInfo = prMsduInfoListHead, prNextMsduInfo;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTxCtrl = &prAdapter->rTxCtrl;
    ASSERT(prTxCtrl);

    while(prMsduInfo) {
        prNextMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);

        switch(prMsduInfo->eSrc) {
        case TX_PACKET_FORWARDING:
            wlanReturnPacket(prAdapter, prMsduInfo->prPacket);
            break;
        case TX_PACKET_OS:
        case TX_PACKET_OS_OID:
        case TX_PACKET_MGMT:
        default:
            break;
        }

        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
        QUEUE_INSERT_TAIL(&prTxCtrl->rFreeMsduInfoList, (P_QUE_ENTRY_T)prMsduInfo);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
        prMsduInfo = prNextMsduInfo;
    };

    return;
}



BOOLEAN
nicTxFillMsduInfo (
    IN P_ADAPTER_T     prAdapter,
    IN P_MSDU_INFO_T   prMsduInfo,
    IN P_NATIVE_PACKET prPacket
    )
{
    P_GLUE_INFO_T   prGlueInfo;
    UINT_8          ucPriorityParam;
    UINT_8          ucMacHeaderLen;
    UINT_8          aucEthDestAddr[PARAM_MAC_ADDR_LEN];
    BOOLEAN         fgIs1x = FALSE;
    BOOLEAN         fgIsPAL = FALSE;
    UINT_32         u4PacketLen;
    ULONG           u4SysTime;
    UINT_8          ucNetworkType;


    ASSERT(prAdapter);

    prGlueInfo = prAdapter->prGlueInfo;
    ASSERT(prGlueInfo);

    if (kalQoSFrameClassifierAndPacketInfo(prGlueInfo,
                prPacket,
                &ucPriorityParam,
                &u4PacketLen,
                aucEthDestAddr,
                &fgIs1x,
                &fgIsPAL,
                &ucNetworkType) == FALSE) {
        return FALSE;
    }

    #if CFG_ENABLE_PKT_LIFETIME_PROFILE
    nicTxLifetimeCheck (
        prAdapter,
        prMsduInfo,
        prPacket,
        ucPriorityParam,
        u4PacketLen,
        ucNetworkType);    
    #endif

    
    GLUE_SET_PKT_TID(prPacket, ucPriorityParam);

    if (fgIs1x) {
        GLUE_SET_PKT_FLAG_1X(prPacket);
    }

    if (fgIsPAL) {
        GLUE_SET_PKT_FLAG_PAL(prPacket);
    }

    ucMacHeaderLen = ETH_HLEN;

    
    GLUE_SET_PKT_HEADER_LEN(prPacket, ucMacHeaderLen);

    
    GLUE_SET_PKT_FRAME_LEN(prPacket, (UINT_16)u4PacketLen);

    
    u4SysTime = (OS_SYSTIME)kalGetTimeTick();
    GLUE_SET_PKT_ARRIVAL_TIME(prPacket, u4SysTime);

    prMsduInfo->prPacket = prPacket;
    prMsduInfo->fgIs802_1x = fgIs1x;
    prMsduInfo->fgIs802_11 = FALSE;
    prMsduInfo->ucNetworkType = ucNetworkType;
    prMsduInfo->ucUserPriority = ucPriorityParam;
    prMsduInfo->ucMacHeaderLength = ucMacHeaderLen;
    prMsduInfo->u2FrameLength = (UINT_16)u4PacketLen;
    COPY_MAC_ADDR(prMsduInfo->aucEthDestAddr, aucEthDestAddr);

    return TRUE;
}


WLAN_STATUS
nicTxAdjustTcq (
    IN P_ADAPTER_T  prAdapter
    )
{
    UINT_32 u4Num;
    TX_TCQ_ADJUST_T rTcqAdjust;
    P_TX_CTRL_T prTxCtrl;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTxCtrl = &prAdapter->rTxCtrl;
    ASSERT(prTxCtrl);

    qmAdjustTcQuotas(prAdapter, &rTcqAdjust, &prTxCtrl->rTc);
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);

    for (u4Num = 0 ; u4Num < TC_NUM ; u4Num++) {
        prTxCtrl->rTc.aucFreeBufferCount[u4Num] += rTcqAdjust.acVariation[u4Num];
        prTxCtrl->rTc.aucMaxNumOfBuffer[u4Num] += rTcqAdjust.acVariation[u4Num];

        ASSERT(prTxCtrl->rTc.aucFreeBufferCount[u4Num] >= 0);
        ASSERT(prTxCtrl->rTc.aucMaxNumOfBuffer[u4Num] >= 0);
    }

    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_RESOURCE);

    return WLAN_STATUS_SUCCESS;
}



WLAN_STATUS
nicTxFlush (
    IN P_ADAPTER_T  prAdapter
    )
{
    P_MSDU_INFO_T prMsduInfo;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
    prMsduInfo = qmFlushTxQueues(prAdapter);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);

    if(prMsduInfo != NULL) {
        nicTxFreeMsduInfoPacket(prAdapter, prMsduInfo);
        nicTxReturnMsduInfo(prAdapter, prMsduInfo);
    }

    return WLAN_STATUS_SUCCESS;
}


#if CFG_ENABLE_FW_DOWNLOAD
WLAN_STATUS
nicTxInitCmd (
    IN P_ADAPTER_T      prAdapter,
    IN P_CMD_INFO_T     prCmdInfo,
    IN UINT_8           ucTC
    )
{
    P_INIT_HIF_TX_HEADER_T prInitTxHeader;
    UINT_16 u2OverallBufferLength;
    PUINT_8 pucOutputBuf = (PUINT_8)NULL; 
    UINT_32 ucPortIdx;
    P_TX_CTRL_T prTxCtrl;

    ASSERT(prAdapter);
    ASSERT(prCmdInfo);
    ASSERT(ucTC == TC0_INDEX);

    prTxCtrl = &prAdapter->rTxCtrl;
    pucOutputBuf = prTxCtrl->pucTxCoalescingBufPtr;
    prInitTxHeader = (P_INIT_HIF_TX_HEADER_T)prCmdInfo->pucInfoBuffer;

    
    u2OverallBufferLength = TFCB_FRAME_PAD_TO_DW(
            (prCmdInfo->u2InfoBufLen) & (UINT_16)HIF_TX_HDR_TX_BYTE_COUNT_MASK);

    prInitTxHeader->u2TxByteCount = u2OverallBufferLength;
    prInitTxHeader->ucEtherTypeOffset = 0;
    prInitTxHeader->ucCSflags = 0;

    
    if (ucTC != TC4_INDEX) {
        ucPortIdx = 0;
    }
    else { 
        ucPortIdx = 1;
    }

    
    kalMemCopy((PVOID)&pucOutputBuf[0],
               (PVOID)prCmdInfo->pucInfoBuffer,
               prCmdInfo->u2InfoBufLen);

    ASSERT(u2OverallBufferLength <= prAdapter->u4CoalescingBufCachedSize);

    
    HAL_WRITE_TX_PORT(prAdapter,
            ucPortIdx,
            (UINT_32)u2OverallBufferLength,
            (PUINT_8)pucOutputBuf,
            (UINT_32)prAdapter->u4CoalescingBufCachedSize);

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS
nicTxInitResetResource (
    IN P_ADAPTER_T  prAdapter
    )
{
    P_TX_CTRL_T prTxCtrl;

    DEBUGFUNC("nicTxInitResetResource");

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC0_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC0;
    prTxCtrl->rTc.aucFreeBufferCount[TC0_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC0;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC1_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC1;
    prTxCtrl->rTc.aucFreeBufferCount[TC1_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC1;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC2_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC2;
    prTxCtrl->rTc.aucFreeBufferCount[TC2_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC2;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC3_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC3;
    prTxCtrl->rTc.aucFreeBufferCount[TC3_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC3;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC4_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC4;
    prTxCtrl->rTc.aucFreeBufferCount[TC4_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC4;

    prTxCtrl->rTc.aucMaxNumOfBuffer[TC5_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC5;
    prTxCtrl->rTc.aucFreeBufferCount[TC5_INDEX] = NIC_TX_INIT_BUFF_COUNT_TC5;

    return WLAN_STATUS_SUCCESS;

}

#endif


WLAN_STATUS
nicTxEnqueueMsdu (
    IN P_ADAPTER_T      prAdapter,
    IN P_MSDU_INFO_T    prMsduInfo
    )
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prNextMsduInfo, prRetMsduInfo, prMsduInfoHead;
    QUE_T qDataPort0, qDataPort1;
    P_CMD_INFO_T prCmdInfo;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    ASSERT(prMsduInfo);

    prTxCtrl = &prAdapter->rTxCtrl;
    ASSERT(prTxCtrl);

    QUEUE_INITIALIZE(&qDataPort0);
    QUEUE_INITIALIZE(&qDataPort1);

    
    while(prMsduInfo) {
        prNextMsduInfo = (P_MSDU_INFO_T)QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo);

        QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T)prMsduInfo) = NULL;

        if(prMsduInfo->eSrc == TX_PACKET_MGMT) {
            
            prMsduInfo->ucTC = TC4_INDEX;

            QUEUE_INSERT_TAIL(&qDataPort1, (P_QUE_ENTRY_T)prMsduInfo);
        }
        else {
            QUEUE_INSERT_TAIL(&qDataPort0, (P_QUE_ENTRY_T)prMsduInfo);
        }

        prMsduInfo = prNextMsduInfo;
    }

    if(qDataPort0.u4NumElem) {
        
        KAL_SPIN_LOCK_DECLARATION();
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
        prRetMsduInfo = qmEnqueueTxPackets(prAdapter, (P_MSDU_INFO_T)QUEUE_GET_HEAD(&qDataPort0));
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);

        
        if(prRetMsduInfo != NULL) { 
            nicTxFreeMsduInfoPacket(prAdapter, prRetMsduInfo);
            nicTxReturnMsduInfo(prAdapter, prRetMsduInfo);
        }
    }

    if(qDataPort1.u4NumElem) {
        prMsduInfoHead = (P_MSDU_INFO_T)QUEUE_GET_HEAD(&qDataPort1);

        if(qDataPort1.u4NumElem > nicTxGetFreeCmdCount(prAdapter)) {
            
            u4Status = WLAN_STATUS_FAILURE;

            
            while(prMsduInfoHead) {
                prNextMsduInfo = (P_MSDU_INFO_T) QUEUE_GET_NEXT_ENTRY(&prMsduInfoHead->rQueEntry);

                if (prMsduInfoHead->pfTxDoneHandler != NULL) {
                    prMsduInfoHead->pfTxDoneHandler(prAdapter, prMsduInfoHead, TX_RESULT_DROPPED_IN_DRIVER);
                }


                cnmMgtPktFree(prAdapter, prMsduInfoHead);

                prMsduInfoHead = prNextMsduInfo;
            }
        }
        else {
            
            while(prMsduInfoHead) {
                prNextMsduInfo = (P_MSDU_INFO_T) QUEUE_GET_NEXT_ENTRY(&prMsduInfoHead->rQueEntry);

                KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_RESOURCE);
                QUEUE_REMOVE_HEAD(&prAdapter->rFreeCmdList, prCmdInfo, P_CMD_INFO_T);
                KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_RESOURCE);

                if (prCmdInfo) {
                    GLUE_INC_REF_CNT(prTxCtrl->i4TxMgmtPendingNum);

                    kalMemZero(prCmdInfo, sizeof(CMD_INFO_T));

                    prCmdInfo->eCmdType             = COMMAND_TYPE_MANAGEMENT_FRAME;
                    prCmdInfo->u2InfoBufLen         = prMsduInfoHead->u2FrameLength;
                    prCmdInfo->pucInfoBuffer        = NULL;
                    prCmdInfo->prPacket             = (P_NATIVE_PACKET)prMsduInfoHead;
                    prCmdInfo->ucStaRecIndex        = prMsduInfoHead->ucStaRecIndex;
                    prCmdInfo->eNetworkType         = prMsduInfoHead->ucNetworkType;
                    prCmdInfo->pfCmdDoneHandler     = NULL;
                    prCmdInfo->pfCmdTimeoutHandler  = NULL;
                    prCmdInfo->fgIsOid              = FALSE;
                    prCmdInfo->fgSetQuery           = TRUE;
                    prCmdInfo->fgNeedResp           = FALSE;

                    kalEnqueueCommand(prAdapter->prGlueInfo, (P_QUE_ENTRY_T)prCmdInfo);
                }
                else {
                    
                    ASSERT(0);

                    u4Status = WLAN_STATUS_FAILURE;
                    cnmMgtPktFree(prAdapter, prMsduInfoHead);
                }

                prMsduInfoHead = prNextMsduInfo;
            }
        }
    }

    
    if(prTxCtrl->i4TxMgmtPendingNum > 0
            || kalGetTxPendingFrameCount(prAdapter->prGlueInfo) > 0) {
        kalSetEvent(prAdapter->prGlueInfo);
    }

    return u4Status;
}


UINT_32
nicTxGetFreeCmdCount (
    IN P_ADAPTER_T  prAdapter
    )
{
    ASSERT(prAdapter);

    return prAdapter->rFreeCmdList.u4NumElem;
}

