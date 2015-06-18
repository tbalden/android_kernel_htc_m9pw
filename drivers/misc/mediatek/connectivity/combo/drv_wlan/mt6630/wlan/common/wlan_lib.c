


#include "precomp.h"
#include "mgmt/ais_fsm.h"


const UINT_8 aucPriorityParam2TC[] = {
    TC1_INDEX,
    TC0_INDEX,
    TC0_INDEX,
    TC1_INDEX,
    TC2_INDEX,
    TC2_INDEX,
    TC3_INDEX,
    TC3_INDEX
};

typedef struct _CODE_MAPPING_T {
    UINT_32         u4RegisterValue;
    INT_32         u4TxpowerOffset;
} CODE_MAPPING_T, *P_CODE_MAPPING_T;

BOOLEAN fgIsBusAccessFailed = FALSE;



#define SIGNED_EXTEND(n, _sValue) \
	(((_sValue) & BIT((n)-1)) ? ((_sValue) | BITS(n, 31)) : \
	 ((_sValue) & ~BITS(n, 31)))

PFN_OID_HANDLER_FUNC apfnOidSetHandlerWOHwAccess[] = {
    wlanoidSetChannel,
    wlanoidSetBeaconInterval,
    wlanoidSetAtimWindow,
    wlanoidSetFrequency,
};

PFN_OID_HANDLER_FUNC apfnOidQueryHandlerWOHwAccess[] = {
    wlanoidQueryBssid,
    wlanoidQuerySsid,
    wlanoidQueryInfrastructureMode,
    wlanoidQueryAuthMode,
    wlanoidQueryEncryptionStatus,
    wlanoidQueryPmkid,
    wlanoidQueryNetworkTypeInUse,
    wlanoidQueryBssidList,
    wlanoidQueryAcpiDevicePowerState,
    wlanoidQuerySupportedRates,
    wlanoidQueryDesiredRates,
    wlanoidQuery802dot11PowerSaveProfile,
    wlanoidQueryBeaconInterval,
    wlanoidQueryAtimWindow,
    wlanoidQueryFrequency,
};

PFN_OID_HANDLER_FUNC apfnOidSetHandlerAllowedInRFTest[] = {
    wlanoidRftestSetTestMode,
    wlanoidRftestSetAbortTestMode,
    wlanoidRftestSetAutoTest,
    wlanoidSetMcrWrite,
    wlanoidSetEepromWrite
};

PFN_OID_HANDLER_FUNC apfnOidQueryHandlerAllowedInRFTest[] = {
    wlanoidRftestQueryAutoTest,
    wlanoidQueryMcrRead,
    wlanoidQueryEepromRead
}

;

PFN_OID_HANDLER_FUNC apfnOidWOTimeoutCheck[] = {
    wlanoidRftestSetTestMode,
    wlanoidRftestSetAbortTestMode,
    wlanoidSetAcpiDevicePowerState,
};




static WLAN_STATUS
wlanImageSectionDownloadStage(IN P_ADAPTER_T prAdapter,
	IN PVOID pvFwImageMapFile, IN UINT_32 index, IN UINT_32 u4FwImageFileLength, IN BOOLEAN fgValidHead, IN UINT_32 u4FwLoadAddr);


BOOLEAN wlanIsHandlerNeedHwAccess(IN PFN_OID_HANDLER_FUNC pfnOidHandler, IN BOOLEAN fgSetInfo)
{
	PFN_OID_HANDLER_FUNC *apfnOidHandlerWOHwAccess;
    UINT_32 i;
    UINT_32 u4NumOfElem;

    if (fgSetInfo) {
        apfnOidHandlerWOHwAccess = apfnOidSetHandlerWOHwAccess;
        u4NumOfElem = sizeof(apfnOidSetHandlerWOHwAccess) / sizeof(PFN_OID_HANDLER_FUNC);
	} else {
        apfnOidHandlerWOHwAccess = apfnOidQueryHandlerWOHwAccess;
        u4NumOfElem = sizeof(apfnOidQueryHandlerWOHwAccess) / sizeof(PFN_OID_HANDLER_FUNC);
    }

    for (i = 0; i < u4NumOfElem; i++) {
        if (apfnOidHandlerWOHwAccess[i] == pfnOidHandler) {
            return FALSE;
        }
    }

    return TRUE;
}   


VOID wlanCardEjected(IN P_ADAPTER_T prAdapter)
{
    DEBUGFUNC("wlanCardEjected");
	

    ASSERT(prAdapter);

     
    nicTxRelease(prAdapter, FALSE);

} 


P_ADAPTER_T wlanAdapterCreate(IN P_GLUE_INFO_T prGlueInfo)
{
	P_ADAPTER_T prAdpater = (P_ADAPTER_T) NULL;

    DEBUGFUNC("wlanAdapterCreate");

    do {
        prAdpater = (P_ADAPTER_T) kalMemAlloc(sizeof(ADAPTER_T), VIR_MEM_TYPE);

        if (!prAdpater) {
            DBGLOG(INIT, ERROR, ("Allocate ADAPTER memory ==> FAILED\n"));
            break;
        }
#if QM_TEST_MODE
        g_rQM.prAdapter = prAdpater;
#endif
        kalMemZero(prAdpater, sizeof(ADAPTER_T));
        prAdpater->prGlueInfo = prGlueInfo;

	} while (FALSE);

    return prAdpater;
} 


VOID wlanAdapterDestroy(IN P_ADAPTER_T prAdapter)
{

    if (!prAdapter) {
        return;
    }

    kalMemFree(prAdapter, VIR_MEM_TYPE, sizeof(ADAPTER_T));

    return;
}

WLAN_STATUS
wlanAdapterStart(IN P_ADAPTER_T prAdapter,
    IN P_REG_INFO_T prRegInfo,
		 IN PVOID pvFwImageMapFile, IN UINT_32 u4FwImageFileLength)
{
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
    UINT_32     i, u4Value = 0;
    UINT_32     u4WHISR = 0;
    UINT_16     au2TxCount[16];
#if CFG_ENABLE_FW_DOWNLOAD
	UINT_32 u4FwLoadAddr;
#if CFG_ENABLE_FW_DIVIDED_DOWNLOAD
    P_FIRMWARE_DIVIDED_DOWNLOAD_T prFwHead;
    BOOLEAN fgValidHead;
    const UINT_32 u4CRCOffset = offsetof(FIRMWARE_DIVIDED_DOWNLOAD_T, u4NumOfEntries);
#endif
#endif

    ASSERT(prAdapter);

    DEBUGFUNC("wlanAdapterStart");

	
	
    prAdapter->fgIsEnterD3ReqIssued = FALSE;

    prAdapter->u4OwnFailedCount = 0;
    prAdapter->u4OwnFailedLogCount = 0;

    QUEUE_INITIALIZE(&(prAdapter->rPendingCmdQueue));
#if CFG_SUPPORT_MULTITHREAD
    QUEUE_INITIALIZE(&prAdapter->rTxCmdQueue);
    QUEUE_INITIALIZE(&prAdapter->rTxCmdDoneQueue);
    QUEUE_INITIALIZE(&prAdapter->rTxP0Queue);
    QUEUE_INITIALIZE(&prAdapter->rTxP1Queue);
    QUEUE_INITIALIZE(&prAdapter->rRxQueue);
#endif

    
    kalMemSet(&(prAdapter->rWlanInfo), 0, sizeof(WLAN_INFO_T));

    for (i = 0; i < BSS_INFO_NUM; i++) {
        prAdapter->aprBssInfo[i] = &prAdapter->rWifiVar.arBssInfoPool[i];
    }
	prAdapter->aprBssInfo[P2P_DEV_BSS_INDEX] = &prAdapter->rWifiVar.rP2pDevInfo;


	
    fgIsBusAccessFailed = FALSE;

    do {
	    u4Status = nicAllocateAdapterMemory(prAdapter);
		if (u4Status != WLAN_STATUS_SUCCESS) {
            DBGLOG(INIT, ERROR, ("nicAllocateAdapterMemory Error!\n"));
            u4Status = WLAN_STATUS_FAILURE;
            break;
        }

        prAdapter->u4OsPacketFilter = PARAM_PACKET_FILTER_SUPPORTED;

#if defined(MT6630)
		DBGLOG(INIT, INFO, ("wlanAdapterStart(): Acquiring LP-OWN\n"));
        ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);
		DBGLOG(INIT, INFO, ("wlanAdapterStart(): Acquiring LP-OWN-end\n"));
		
#if !CFG_ENABLE_FULL_PM
        nicpmSetDriverOwn(prAdapter);
#endif

		if (prAdapter->fgIsFwOwn == TRUE) {
            DBGLOG(INIT, ERROR, ("nicpmSetDriverOwn() failed!\n"));
            u4Status = WLAN_STATUS_FAILURE;
            break;
        }
		
		u4Status = nicInitializeAdapter(prAdapter);
		if (u4Status != WLAN_STATUS_SUCCESS) {
            DBGLOG(INIT, ERROR, ("nicInitializeAdapter failed!\n"));
            u4Status = WLAN_STATUS_FAILURE;
            break;
        }
#endif

		
        nicInitSystemService(prAdapter);

		
        wlanInitFeatureOption(prAdapter);
#if CFG_SUPPORT_MTK_SYNERGY
	if (kalIsConfigurationExist(prAdapter->prGlueInfo) == TRUE) {
	        if (prRegInfo->prNvramSettings->u2FeatureReserved & BIT(MTK_FEATURE_2G_256QAM_DISABLED)) {
	            prAdapter->rWifiVar.aucMtkFeature[0] &= ~(MTK_SYNERGY_CAP_SUPPORT_24G_MCS89);
	        }
	}
#endif

        
        wlanCfgSetDebugLevel(prAdapter);

		
        nicTxInitialize(prAdapter);
        wlanDefTxPowerCfg(prAdapter);

		
        nicRxInitialize(prAdapter);
    
#if CFG_ENABLE_FW_DOWNLOAD
#if defined(MT6630)
        if (pvFwImageMapFile) {
            
            nicDisableInterrupt(prAdapter);

            
            nicTxInitResetResource(prAdapter);

            
            u4FwLoadAddr = prRegInfo->u4LoadAddress;

			DBGLOG(INIT, INFO, ("FW download Start\n"));
			
#if CFG_ENABLE_FW_DIVIDED_DOWNLOAD
			
			prFwHead = (P_FIRMWARE_DIVIDED_DOWNLOAD_T) pvFwImageMapFile;

			if (prFwHead->u4Signature == MTK_WIFI_SIGNATURE &&
			    prFwHead->u4CRC == wlanCRC32((PUINT_8) pvFwImageMapFile + u4CRCOffset,
							 u4FwImageFileLength - u4CRCOffset)) {
                fgValidHead = TRUE;
			} else {
                fgValidHead = FALSE;
            }

            
			if (fgValidHead == TRUE) {
				for (i = 0; i < prFwHead->u4NumOfEntries; i++) {
					u4Status = wlanImageSectionDownloadStage(prAdapter,
						pvFwImageMapFile, i, u4FwImageFileLength, TRUE, u4FwLoadAddr);
					if (u4Status == WLAN_STATUS_FAILURE) {
                        break;
                    }
                            }
			} else
#endif
            {
				u4Status = wlanImageSectionDownloadStage(prAdapter,
					pvFwImageMapFile, 0, u4FwImageFileLength, FALSE, u4FwLoadAddr);
				if (u4Status == WLAN_STATUS_FAILURE) {
                    break;
                }
                }

            
			if (u4Status != WLAN_STATUS_SUCCESS) {
                break;
            }
#if !CFG_ENABLE_FW_DOWNLOAD_ACK
			
			if (wlanImageQueryStatus(prAdapter) != WLAN_STATUS_SUCCESS) {
                DBGLOG(INIT, ERROR, ("Firmware download failed!\n"));
                u4Status = WLAN_STATUS_FAILURE;
                break;
            }
#endif
		} else {
            DBGLOG(INIT, ERROR, ("No Firmware found!\n"));
            u4Status = WLAN_STATUS_FAILURE;
            break;
        }
		DBGLOG(INIT, INFO, ("FW download End\n"));
        
#if CFG_OVERRIDE_FW_START_ADDRESS
		wlanConfigWifiFunc(prAdapter, TRUE, prRegInfo->u4StartAddress);
#else
		wlanConfigWifiFunc(prAdapter, FALSE, 0);
#endif
#endif
#endif

        DBGLOG(INIT, TRACE, ("wlanAdapterStart(): Waiting for Ready bit..\n"));
		
        i = 0;
		while (1) {
            HAL_MCR_RD(prAdapter, MCR_WCIR, &u4Value);

            if (u4Value & WCIR_WLAN_READY) {
                DBGLOG(INIT, TRACE, ("Ready bit asserted\n"));
                break;
			} else if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE
                    || fgIsBusAccessFailed == TRUE) {
                u4Status = WLAN_STATUS_FAILURE;
                break;
			} else if (i >= CFG_RESPONSE_POLLING_TIMEOUT) {
                UINT_32     u4MailBox0;

                nicGetMailbox(prAdapter, 0, &u4MailBox0);
				DBGLOG(INIT, ERROR, ("Waiting for Ready bit: Timeout, ID=%d\n",
                        (u4MailBox0 & 0x0000FFFF)));
                u4Status = WLAN_STATUS_FAILURE;
                break;
			} else {
                i++;
                kalMsleep(10);
            }
        }

		if (u4Status == WLAN_STATUS_SUCCESS) {
			
			HAL_READ_INTR_STATUS(prAdapter, 4, (PUINT_8) & u4WHISR);
			if (HAL_IS_TX_DONE_INTR(u4WHISR)) {
                HAL_READ_TX_RELEASED_COUNT(prAdapter, au2TxCount);
            }

            
            wlanQueryNicResourceInformation(prAdapter);

#if (CFG_SUPPORT_NIC_CAPABILITY == 1)
            
            wlanQueryNicCapability(prAdapter);
#endif

            
            wlanUpdateBasicConfig(prAdapter);

            
            wlanUpdateNetworkAddress(prAdapter);

            
            nicApplyNetworkAddress(prAdapter);

            
            kalIndicateStatusAndComplete(prAdapter->prGlueInfo,
						     WLAN_STATUS_MEDIA_DISCONNECT, NULL, 0);
        }

        RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);

		if (u4Status != WLAN_STATUS_SUCCESS) {
            break;
        }

        
        cnmTimerInitTimer(prAdapter,
                &prAdapter->rOidTimeoutTimer,
				  (PFN_MGMT_TIMEOUT_FUNC) wlanReleasePendingOid, (ULONG) NULL);

        prAdapter->ucOidTimeoutCount = 0;

        prAdapter->fgIsChipNoAck = FALSE;

        
        cnmTimerInitTimer(prAdapter,
                &prAdapter->rPacketDelaySetupTimer,
				  (PFN_MGMT_TIMEOUT_FUNC) wlanReturnPacketDelaySetupTimeout,
				  (ULONG) NULL);

        
        prAdapter->fgWiFiInSleepyState = FALSE;
        prAdapter->rAcpiState = ACPI_STATE_D0;

        
		if (prRegInfo->fgDisOnlineScan == 0) {
            prAdapter->fgEnOnlineScan = TRUE;
		} else {
            prAdapter->fgEnOnlineScan = FALSE;
        }

        
		if (prRegInfo->fgDisBcnLostDetection != 0) {
            prAdapter->fgDisBcnLostDetection = TRUE;
        }

        
        prAdapter->rWlanInfo.u2BeaconPeriod = CFG_INIT_ADHOC_BEACON_INTERVAL;
        prAdapter->rWlanInfo.u2AtimWindow = CFG_INIT_ADHOC_ATIM_WINDOW;

#if 1				
        prAdapter->fgEnArpFilter = prRegInfo->fgEnArpFilter;
        prAdapter->u4PsCurrentMeasureEn = prRegInfo->u4PsCurrentMeasureEn;

        prAdapter->u4UapsdAcBmp = prRegInfo->u4UapsdAcBmp;

        prAdapter->u4MaxSpLen = prRegInfo->u4MaxSpLen;

        DBGLOG(INIT, TRACE, ("[1] fgEnArpFilter:0x%x, u4UapsdAcBmp:0x%x, u4MaxSpLen:0x%x",
                prAdapter->fgEnArpFilter,
				     prAdapter->u4UapsdAcBmp, prAdapter->u4MaxSpLen));

        prAdapter->fgEnCtiaPowerMode = FALSE;

#endif

        
        nicInitMGMT(prAdapter, prRegInfo);

        
        prAdapter->rWifiVar.fgSupportWZCDisassociation = TRUE;

        
		if ((ENUM_REGISTRY_FIXED_RATE_T) (prRegInfo->u4FixedRate) < FIXED_RATE_NUM) {
			prAdapter->rWifiVar.eRateSetting =
			    (ENUM_REGISTRY_FIXED_RATE_T) (prRegInfo->u4FixedRate);
		} else {
            prAdapter->rWifiVar.eRateSetting = FIXED_RATE_NONE;
        }

		if (prAdapter->rWifiVar.eRateSetting == FIXED_RATE_NONE) {
            
            prAdapter->rWifiVar.ePreambleType = PREAMBLE_TYPE_AUTO;
		} else if ((prAdapter->rWifiVar.eRateSetting >= FIXED_RATE_MCS0_20M_400NS &&
                    prAdapter->rWifiVar.eRateSetting <= FIXED_RATE_MCS7_20M_400NS)
                || (prAdapter->rWifiVar.eRateSetting >= FIXED_RATE_MCS0_40M_400NS &&
                        prAdapter->rWifiVar.eRateSetting <= FIXED_RATE_MCS32_400NS)) {
            
            prAdapter->rWifiVar.ePreambleType = PREAMBLE_TYPE_SHORT;
		} else {
            
            prAdapter->rWifiVar.ePreambleType = PREAMBLE_TYPE_LONG;
        }

        
        prAdapter->rWifiVar.fgEnableJoinToHiddenSSID = FALSE;

        
        prAdapter->rWifiVar.fgIsShortSlotTimeOptionEnable = TRUE;

        
        nicSetAvailablePhyTypeSet(prAdapter);

#if 0 
#if 1				
        {
#if CFG_SUPPORT_PWR_MGT
        prAdapter->u4PowerMode = prRegInfo->u4PowerMode;
#if CFG_ENABLE_WIFI_DIRECT
			prAdapter->rWlanInfo.arPowerSaveMode[NETWORK_TYPE_P2P_INDEX].
			    ucNetTypeIndex = NETWORK_TYPE_P2P_INDEX;
			prAdapter->rWlanInfo.arPowerSaveMode[NETWORK_TYPE_P2P_INDEX].ucPsProfile =
			    ENUM_PSP_FAST_SWITCH;
#endif
#else
        prAdapter->u4PowerMode = ENUM_PSP_CONTINUOUS_ACTIVE;
#endif

			nicConfigPowerSaveProfile(prAdapter,
            prAdapter->prAisBssInfo->ucBssIndex,
						  prAdapter->u4PowerMode, FALSE);
        }

#endif
#endif

#if CFG_SUPPORT_NVRAM
        
		if (kalIsConfigurationExist(prAdapter->prGlueInfo) == TRUE) {
			wlanLoadManufactureData(prAdapter, prRegInfo);
		} else {
			DBGLOG(INIT, WARN, ("%s: load manufacture data fail\n", __func__));
		}	
#endif

#if 0
    
    nicRlmArUpdateParms(prAdapter,
        prRegInfo->u4ArSysParam0,
        prRegInfo->u4ArSysParam1,
				    prRegInfo->u4ArSysParam2, prRegInfo->u4ArSysParam3);
#endif
	} while (FALSE);

	if (u4Status == WLAN_STATUS_SUCCESS) {
		
        HAL_SET_INTR_STATUS_READ_CLEAR(prAdapter);
        HAL_SET_MAILBOX_READ_CLEAR(prAdapter, FALSE);

        
        nicEnableInterrupt(prAdapter);

	} else {
		
        nicReleaseAdapterMemory(prAdapter);
    }

    return u4Status;
} 

static WLAN_STATUS
wlanImageSectionDownloadStage(IN P_ADAPTER_T prAdapter,
	IN PVOID pvFwImageMapFile, IN UINT_32 index, IN UINT_32 u4FwImageFileLength, IN BOOLEAN fgValidHead, IN UINT_32 u4FwLoadAddr)
{
#if CFG_ENABLE_FW_DOWNLOAD
	UINT_32 u4ImgSecSize;
#if CFG_ENABLE_FW_DIVIDED_DOWNLOAD
	UINT_32 j;
	P_FIRMWARE_DIVIDED_DOWNLOAD_T prFwHead;
	WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
#endif
#endif

#if CFG_ENABLE_FW_DOWNLOAD
#if defined(MT6630)
#if CFG_ENABLE_FW_DIVIDED_DOWNLOAD
	
	prFwHead = (P_FIRMWARE_DIVIDED_DOWNLOAD_T) pvFwImageMapFile;
	do {
		if (fgValidHead == TRUE) {
			if (wlanImageSectionConfig(prAdapter,
						   prFwHead->arSection[index].u4DestAddr,
						   prFwHead->arSection[index].u4Length,
						   index == 0 ? TRUE : FALSE) !=
			    WLAN_STATUS_SUCCESS) {
				DBGLOG(INIT, ERROR,
				       ("Firmware download configuration failed!\n"));

				u4Status = WLAN_STATUS_FAILURE;
				break;
			} else {
				for (j = 0; j < prFwHead->arSection[index].u4Length;
				     j += CMD_PKT_SIZE_FOR_IMAGE) {
					if (j + CMD_PKT_SIZE_FOR_IMAGE <
					    prFwHead->arSection[index].u4Length)
						u4ImgSecSize =
						    CMD_PKT_SIZE_FOR_IMAGE;
					else
						u4ImgSecSize =
						    prFwHead->arSection[index].u4Length - j;

					if (wlanImageSectionDownload(prAdapter,
								     u4ImgSecSize,
								     (PUINT_8)
								     pvFwImageMapFile
								     +
								     prFwHead->arSection[index].u4Offset + j) !=
					    WLAN_STATUS_SUCCESS) {
						DBGLOG(INIT, ERROR,
						       ("Firmware scatter download failed!\n"));

						u4Status = WLAN_STATUS_FAILURE;
						break;
					}
				}

				
				if (u4Status == WLAN_STATUS_FAILURE) {
					break;
				}
			}
		} else {
			if (wlanImageSectionConfig(prAdapter,
						   u4FwLoadAddr,
						   u4FwImageFileLength,
						   TRUE) != WLAN_STATUS_SUCCESS) {
				DBGLOG(INIT, ERROR,
				       ("Firmware download configuration failed!\n"));

				u4Status = WLAN_STATUS_FAILURE;
				break;
			} else {
				for (j = 0; j < u4FwImageFileLength;
				     j += CMD_PKT_SIZE_FOR_IMAGE) {
					if (j + CMD_PKT_SIZE_FOR_IMAGE < u4FwImageFileLength)
						u4ImgSecSize = CMD_PKT_SIZE_FOR_IMAGE;
					else
						u4ImgSecSize = u4FwImageFileLength - j;

					if (wlanImageSectionDownload(prAdapter,
								     u4ImgSecSize,
								     (PUINT_8)pvFwImageMapFile + j) !=
					    WLAN_STATUS_SUCCESS) {
						DBGLOG(INIT, ERROR,
						       ("Firmware scatter download failed!\n"));

						u4Status = WLAN_STATUS_FAILURE;
						break;
					}
				}
			}
		}
	} while (0);
	return u4Status;
#endif
#endif
#endif
}



WLAN_STATUS wlanAdapterStop(IN P_ADAPTER_T prAdapter)
{
    UINT_32 i, u4Value = 0;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;

    ASSERT(prAdapter);

    
    nicUninitMGMT(prAdapter);

    
    kalClearCommandQueue(prAdapter->prGlueInfo);

#if CFG_SUPPORT_MULTITHREAD

    
    wlanClearTxCommandQueue(prAdapter);

    wlanClearTxCommandDoneQueue(prAdapter);

    wlanClearDataQueue(prAdapter);

    wlanClearRxToOsQueue(prAdapter);

#endif

	if (prAdapter->rAcpiState == ACPI_STATE_D0 &&
	    !wlanIsChipNoAck(prAdapter) && !kalIsCardRemoved(prAdapter->prGlueInfo)) {

        
        nicDisableInterrupt(prAdapter);

        ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

        
		if (prAdapter->fgIsFwOwn == FALSE
		    && wlanSendNicPowerCtrlCmd(prAdapter, 1) == WLAN_STATUS_SUCCESS) {
            
            i = 0;
			while (i < CFG_IST_LOOP_COUNT
			       && nicProcessIST(prAdapter) != WLAN_STATUS_NOT_INDICATING) {
                i++;
            };

            
            i = 0;
			while (1) {
                HAL_MCR_RD(prAdapter, MCR_WCIR, &u4Value);

                if ((u4Value & WCIR_WLAN_READY) == 0)
                    break;
				else if (kalIsCardRemoved(prAdapter->prGlueInfo)
                        || fgIsBusAccessFailed
                        || (i >= CFG_RESPONSE_POLLING_TIMEOUT)) {
                        
					DBGLOG(INIT, WARN,
					       ("%s: Failure to get RDY bit cleared! CardRemoved[%u] BusFailed[%u] Timeout[%u]",
						__func__,
                        kalIsCardRemoved(prAdapter->prGlueInfo),
						fgIsBusAccessFailed, i));
                    
                    break;
				} else {
                    i++;
                    kalMsleep(1);
                }
            }
        }

        
        nicpmSetFWOwn(prAdapter, FALSE);

#if CFG_FORCE_RESET_UNDER_BUS_ERROR
		if (HAL_TEST_FLAG(prAdapter, ADAPTER_FLAG_HW_ERR) == TRUE) {
            
            kalDevRegWrite(prAdapter->prGlueInfo, MCR_WHLPCR, WHLPCR_FW_OWN_REQ_CLR);

            
            kalMdelay(10);

            
            kalDevRegWrite(prAdapter->prGlueInfo, MCR_WSICR, WSICR_H2D_SW_INT_SET);

            
            kalDevRegWrite(prAdapter->prGlueInfo, MCR_WHLPCR, WHLPCR_FW_OWN_REQ_SET);
        }
#endif

        RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);
    }

    nicRxUninitialize(prAdapter);

    nicTxRelease(prAdapter, FALSE);

    
    nicUninitSystemService(prAdapter);

    nicReleaseAdapterMemory(prAdapter);

#if defined(_HIF_SPI)
    
    nicRestoreSpiDefMode(prAdapter);
#endif

    return u4Status;
} 


BOOL wlanISR(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgGlobalIntrCtrl)
{
    ASSERT(prAdapter);

    if (fgGlobalIntrCtrl) {
        nicDisableInterrupt(prAdapter);

		
    }

    return TRUE;
}

VOID wlanIST(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

    nicProcessIST(prAdapter);

	if (KAL_WAKE_LOCK_ACTIVE(prAdapter, &prAdapter->prGlueInfo->rIntrWakeLock)) {
        KAL_WAKE_UNLOCK(prAdapter, &prAdapter->prGlueInfo->rIntrWakeLock);
    }

    nicEnableInterrupt(prAdapter);

    RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);

    return;
}


WLAN_STATUS wlanProcessCommandQueue(IN P_ADAPTER_T prAdapter, IN P_QUE_T prCmdQue)
{
    WLAN_STATUS rStatus;
    QUE_T rTempCmdQue, rMergeCmdQue, rStandInCmdQue;
    P_QUE_T prTempCmdQue, prMergeCmdQue, prStandInCmdQue;
    P_QUE_ENTRY_T prQueueEntry;
    P_CMD_INFO_T prCmdInfo;
    P_MSDU_INFO_T prMsduInfo;
    ENUM_FRAME_ACTION_T eFrameAction = FRAME_ACTION_DROP_PKT;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    ASSERT(prCmdQue);

    prTempCmdQue = &rTempCmdQue;
    prMergeCmdQue = &rMergeCmdQue;
    prStandInCmdQue = &rStandInCmdQue;

    QUEUE_INITIALIZE(prTempCmdQue);
    QUEUE_INITIALIZE(prMergeCmdQue);
    QUEUE_INITIALIZE(prStandInCmdQue);

	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_QUE);

	
    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		switch (prCmdInfo->eCmdType) {
        case COMMAND_TYPE_GENERAL_IOCTL:
        case COMMAND_TYPE_NETWORK_IOCTL:
            
            eFrameAction = FRAME_ACTION_TX_PKT;
            break;

        case COMMAND_TYPE_SECURITY_FRAME:
            
            eFrameAction = qmGetFrameAction(prAdapter, prCmdInfo->ucBssIndex,
                    prCmdInfo->ucStaRecIndex, NULL, FRAME_TYPE_802_1X,
                    prCmdInfo->u2InfoBufLen);
            break;

        case COMMAND_TYPE_MANAGEMENT_FRAME:
            
			prMsduInfo = (P_MSDU_INFO_T) (prCmdInfo->prPacket);

            eFrameAction = qmGetFrameAction(prAdapter, prMsduInfo->ucBssIndex,
                    prMsduInfo->ucStaRecIndex, prMsduInfo, FRAME_TYPE_MMPDU, 
                    prMsduInfo->u2FrameLength);
            break;

        default:
            ASSERT(0);
            break;
        }

		
		if (eFrameAction == FRAME_ACTION_DROP_PKT) {
            DBGLOG(INIT, INFO, ("DROP CMD TYPE[%u] ID[0x%02X] SEQ[%u]\n",
                prCmdInfo->eCmdType, prCmdInfo->ucCID, prCmdInfo->ucCmdSeqNum));
            wlanReleaseCommand(prAdapter, prCmdInfo, TX_RESULT_DROPPED_IN_DRIVER);
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }
        else if(eFrameAction == FRAME_ACTION_QUEUE_PKT) {
            DBGLOG(INIT, TRACE, ("QUE back CMD TYPE[%u] ID[0x%02X] SEQ[%u]\n",
                prCmdInfo->eCmdType, prCmdInfo->ucCID, prCmdInfo->ucCmdSeqNum));
                
            QUEUE_INSERT_TAIL(prMergeCmdQue, prQueueEntry);
		} else if (eFrameAction == FRAME_ACTION_TX_PKT) {
			
#if CFG_SUPPORT_MULTITHREAD
            rStatus = wlanSendCommandMthread(prAdapter, prCmdInfo);

            if(rStatus == WLAN_STATUS_RESOURCES) {
                
                DBGLOG(INIT, WARN, ("NO Res CMD TYPE[%u] ID[0x%02X] SEQ[%u]\n", 
                    prCmdInfo->eCmdType, prCmdInfo->ucCID, prCmdInfo->ucCmdSeqNum));
                
                QUEUE_INSERT_TAIL(prMergeCmdQue, prQueueEntry);
                break;
			} else if (rStatus == WLAN_STATUS_PENDING) {
			} else if (rStatus == WLAN_STATUS_SUCCESS) {
			} else {
				P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) prQueueEntry;
                if (prCmdInfo->fgIsOid) {
					kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery,
						       prCmdInfo->u4SetInfoLen, rStatus);
                }
                cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
            }
            
#else            
            rStatus = wlanSendCommand(prAdapter, prCmdInfo);

            if(rStatus == WLAN_STATUS_RESOURCES) {
                

                DBGLOG(INIT, WARN, ("NO Resource for CMD TYPE[%u] ID[0x%02X] SEQ[%u]\n", 
                    prCmdInfo->eCmdType, prCmdInfo->ucCID, prCmdInfo->ucCmdSeqNum));
                
                QUEUE_INSERT_TAIL(prMergeCmdQue, prQueueEntry);
                break;
			} else if (rStatus == WLAN_STATUS_PENDING) {
				
                KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);
                QUEUE_INSERT_TAIL(&(prAdapter->rPendingCmdQueue), prQueueEntry);
                KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);
			} else {
				P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

                if (rStatus == WLAN_STATUS_SUCCESS) {
                    if (prCmdInfo->pfCmdDoneHandler) {
						prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
						    prCmdInfo->pucInfoBuffer);
                    }
				} else {
                    if (prCmdInfo->fgIsOid) {
						kalOidComplete(prAdapter->prGlueInfo, 
                            prCmdInfo->fgSetQuery, prCmdInfo->u4SetInfoLen, 
                            rStatus);
                    }
                }

                cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
            }
#endif            
		} else {
            ASSERT(0);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

	
	
    QUEUE_CONCATENATE_QUEUES(prMergeCmdQue, prTempCmdQue);

	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prStandInCmdQue, prCmdQue);

	
    QUEUE_CONCATENATE_QUEUES(prMergeCmdQue, prStandInCmdQue);

	
    QUEUE_MOVE_ALL(prCmdQue, prMergeCmdQue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_QUE);
    
#if CFG_SUPPORT_MULTITHREAD
    kalSetTxCmdEvent2Hif(prAdapter->prGlueInfo);
#endif


    return WLAN_STATUS_SUCCESS;
} 

/*!
* \brief This function will take CMD_INFO_T which carry some informations of
*        incoming OID and notify the NIC_TX to send CMD.
*
* \param prAdapter      Pointer of Adapter Data Structure
* \param prCmdInfo      Pointer of P_CMD_INFO_T
*
* \retval WLAN_STATUS_SUCCESS   : CMD was written to HIF and be freed(CMD Done) immediately.
* \retval WLAN_STATUS_RESOURCE  : No resource for current command, need to wait for previous
*                                 frame finishing their transmission.
* \retval WLAN_STATUS_FAILURE   : Get failure while access HIF or been rejected.
*/
WLAN_STATUS wlanSendCommand(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo)
{
    P_TX_CTRL_T prTxCtrl;
    UINT_8 ucTC; 
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

    ASSERT(prAdapter);
    ASSERT(prCmdInfo);
    prTxCtrl = &prAdapter->rTxCtrl;

    do {
		
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE || fgIsBusAccessFailed == TRUE) {
            rStatus = WLAN_STATUS_FAILURE;
            break;
        }
		
        if (!prCmdInfo->fgDriverDomainMCR) {
			
            ucTC = nicTxGetCmdResourceType(prCmdInfo);

			
			rStatus = nicTxAcquireResource(prAdapter, ucTC, nicTxGetCmdPageCount(prCmdInfo));
			if (rStatus == WLAN_STATUS_RESOURCES) {
            	DBGLOG(INIT, INFO, ("NO Resource:%d\n", ucTC));
                break;
            }
			
            rStatus = nicTxCmd(prAdapter, prCmdInfo, ucTC);

			
            if (rStatus == WLAN_STATUS_SUCCESS) {
                if ((!prCmdInfo->fgSetQuery) || (prCmdInfo->fgNeedResp)) {
                    rStatus = WLAN_STATUS_PENDING;
                }
            }
        }
		
        else {
            P_CMD_ACCESS_REG prCmdAccessReg;
			prCmdAccessReg =
			    (P_CMD_ACCESS_REG) (prCmdInfo->pucInfoBuffer + CMD_HDR_SIZE);

            if (prCmdInfo->fgSetQuery) {
				HAL_MCR_WR(prAdapter, (prCmdAccessReg->u4Address & BITS(2, 31)),	
                        prCmdAccessReg->u4Data);
			} else {
                P_CMD_ACCESS_REG prEventAccessReg;
                UINT_32 u4Address;

                u4Address = prCmdAccessReg->u4Address;
				prEventAccessReg = (P_CMD_ACCESS_REG) prCmdInfo->pucInfoBuffer;
                prEventAccessReg->u4Address = u4Address;

				HAL_MCR_RD(prAdapter, prEventAccessReg->u4Address & BITS(2, 31),	
                       &prEventAccessReg->u4Data);
            }
        }

	} while (FALSE);

    return rStatus;
} 

#if CFG_SUPPORT_MULTITHREAD

/*!
* \brief This function will take CMD_INFO_T which carry some informations of
*        incoming OID and notify the NIC_TX to send CMD.
*
* \param prAdapter      Pointer of Adapter Data Structure
* \param prCmdInfo      Pointer of P_CMD_INFO_T
*
* \retval WLAN_STATUS_SUCCESS   : CMD was written to HIF and be freed(CMD Done) immediately.
* \retval WLAN_STATUS_RESOURCE  : No resource for current command, need to wait for previous
*                                 frame finishing their transmission.
* \retval WLAN_STATUS_FAILURE   : Get failure while access HIF or been rejected.
*/
WLAN_STATUS wlanSendCommandMthread(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo)
{
    P_TX_CTRL_T prTxCtrl;
    UINT_8 ucTC; 
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;

    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue;

    KAL_SPIN_LOCK_DECLARATION();
 
    ASSERT(prAdapter);
    ASSERT(prCmdInfo);
    prTxCtrl = &prAdapter->rTxCtrl;

    prTempCmdQue = &rTempCmdQue;
    QUEUE_INITIALIZE(prTempCmdQue);
    
    do {
		
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE || fgIsBusAccessFailed == TRUE) {
            rStatus = WLAN_STATUS_FAILURE;
            break;
        }
		
        if (!prCmdInfo->fgDriverDomainMCR) {
			
            ucTC = nicTxGetCmdResourceType(prCmdInfo);

            
            rStatus = nicTxAcquireResource(prAdapter, ucTC, nicTxGetCmdPageCount(prCmdInfo));
            if (rStatus == WLAN_STATUS_RESOURCES) {
#if 0                
                DBGLOG(INIT, WARN, ("%s: NO Resource for CMD TYPE[%u] ID[0x%02X] SEQ[%u] TC[%u]\n", 
                    __FUNCTION__,
                    prCmdInfo->eCmdType, 
                    prCmdInfo->ucCID,
                    prCmdInfo->ucCmdSeqNum,
                    ucTC));                
#endif
                break;
            }
            
			
            if ((!prCmdInfo->fgSetQuery) || (prCmdInfo->fgNeedResp)) {
				
            } 
			QUEUE_INSERT_TAIL(prTempCmdQue, (P_QUE_ENTRY_T) prCmdInfo);
           
			
            if (rStatus == WLAN_STATUS_SUCCESS) {
                if ((!prCmdInfo->fgSetQuery) || 
                    (prCmdInfo->fgNeedResp)  || 
                    (prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME)) {
                    rStatus = WLAN_STATUS_PENDING;      
                }
            }
        }
		
        else {
			QUEUE_INSERT_TAIL(prTempCmdQue, (P_QUE_ENTRY_T) prCmdInfo);
            rStatus = WLAN_STATUS_PENDING;
        }
	} while (FALSE);

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);
    QUEUE_CONCATENATE_QUEUES(&(prAdapter->rTxCmdQueue), prTempCmdQue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);

    return rStatus;
} 

WLAN_STATUS wlanTxCmdMthread(IN P_ADAPTER_T prAdapter)
{
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue;
    QUE_T rTempCmdDoneQue;
    P_QUE_T prTempCmdDoneQue;
    P_QUE_ENTRY_T prQueueEntry;
    P_CMD_INFO_T prCmdInfo;
    P_CMD_ACCESS_REG prCmdAccessReg;
    P_CMD_ACCESS_REG prEventAccessReg;
    UINT_32 u4Address;
    UINT_32 u4TxDoneQueueSize;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTempCmdQue = &rTempCmdQue;
    QUEUE_INITIALIZE(prTempCmdQue);

    prTempCmdDoneQue = &rTempCmdDoneQue;
    QUEUE_INITIALIZE(prTempCmdDoneQue);

    KAL_ACQUIRE_MUTEX(prAdapter, MUTEX_TX_CMD_CLEAR);
    
    
    
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, &prAdapter->rTxCmdQueue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);

	
    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

        

        if (!prCmdInfo->fgDriverDomainMCR) {
            nicTxCmd(prAdapter, prCmdInfo, TC4_INDEX);

            if ((!prCmdInfo->fgSetQuery) || (prCmdInfo->fgNeedResp)) {
                KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);
				QUEUE_INSERT_TAIL(&(prAdapter->rPendingCmdQueue),
						  (P_QUE_ENTRY_T) prCmdInfo);
                KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);                
			} else {
                QUEUE_INSERT_TAIL(prTempCmdDoneQue, prQueueEntry);
            } 
			
		} else {
			prCmdAccessReg =
			    (P_CMD_ACCESS_REG) (prCmdInfo->pucInfoBuffer + CMD_HDR_SIZE);

            if (prCmdInfo->fgSetQuery) {
				HAL_MCR_WR(prAdapter, (prCmdAccessReg->u4Address & BITS(2, 31)),	
                        prCmdAccessReg->u4Data);
			} else {
                u4Address = prCmdAccessReg->u4Address;
				prEventAccessReg = (P_CMD_ACCESS_REG) prCmdInfo->pucInfoBuffer;
                prEventAccessReg->u4Address = u4Address;

				HAL_MCR_RD(prAdapter, prEventAccessReg->u4Address & BITS(2, 31),	
                       &prEventAccessReg->u4Data);
            }
            QUEUE_INSERT_TAIL(prTempCmdDoneQue, prQueueEntry);
        }
        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);
    QUEUE_CONCATENATE_QUEUES(&prAdapter->rTxCmdDoneQueue, prTempCmdDoneQue);
    u4TxDoneQueueSize = prAdapter->rTxCmdDoneQueue.u4NumElem;
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);

    KAL_RELEASE_MUTEX(prAdapter, MUTEX_TX_CMD_CLEAR);
    
    if (u4TxDoneQueueSize > 0) {
		
		set_bit(GLUE_FLAG_TX_CMD_DONE_BIT, &prAdapter->prGlueInfo->ulFlag);
        wake_up_interruptible(&prAdapter->prGlueInfo->waitq);
    }

    return WLAN_STATUS_SUCCESS;
}

WLAN_STATUS wlanTxCmdDoneMthread(IN P_ADAPTER_T prAdapter)
{
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry;
    P_CMD_INFO_T prCmdInfo;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTempCmdQue = &rTempCmdQue;
    QUEUE_INITIALIZE(prTempCmdQue);
    
	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, &prAdapter->rTxCmdDoneQueue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);

	
    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

        if (prCmdInfo->pfCmdDoneHandler) {
            prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo, prCmdInfo->pucInfoBuffer);
        }
		
        cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    return WLAN_STATUS_SUCCESS;
}

VOID wlanClearTxCommandQueue(IN P_ADAPTER_T prAdapter)
{
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;      
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

    KAL_SPIN_LOCK_DECLARATION();
    QUEUE_INITIALIZE(prTempCmdQue);

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, &prAdapter->rTxCmdQueue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->pfCmdTimeoutHandler) {
            prCmdInfo->pfCmdTimeoutHandler(prAdapter, prCmdInfo);
		} else {
            wlanReleaseCommand(prAdapter, prCmdInfo, TX_RESULT_QUEUE_CLEARANCE);
        }

        cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }
}

VOID
wlanClearTxOidCommand(
    IN P_ADAPTER_T              prAdapter
    )
{
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;      
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;
    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;

    KAL_SPIN_LOCK_DECLARATION();
    QUEUE_INITIALIZE(prTempCmdQue);

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);
    
    QUEUE_MOVE_ALL(prTempCmdQue, &prAdapter->rTxCmdQueue);
    
    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    
    while (prQueueEntry) {
        prCmdInfo = (P_CMD_INFO_T)prQueueEntry;

        if (prCmdInfo->fgIsOid) {

            if(prCmdInfo->pfCmdTimeoutHandler) {
                prCmdInfo->pfCmdTimeoutHandler(prAdapter, prCmdInfo);
            }
            else {
                wlanReleaseCommand(prAdapter, prCmdInfo, TX_RESULT_QUEUE_CLEARANCE);
            }
            
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

        }
        else {
            QUEUE_INSERT_TAIL(&prAdapter->rTxCmdQueue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_QUE);
}


VOID wlanClearTxCommandDoneQueue(IN P_ADAPTER_T prAdapter)
{
    QUE_T rTempCmdDoneQue;
    P_QUE_T prTempCmdDoneQue = &rTempCmdDoneQue;      
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

    KAL_SPIN_LOCK_DECLARATION();
    QUEUE_INITIALIZE(prTempCmdDoneQue);
    
	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);
    QUEUE_MOVE_ALL(prTempCmdDoneQue, &prAdapter->rTxCmdDoneQueue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_CMD_DONE_QUE);

	
    QUEUE_REMOVE_HEAD(prTempCmdDoneQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

        if (prCmdInfo->pfCmdDoneHandler) {
            prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo, prCmdInfo->pucInfoBuffer);
        }
		
        cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

        QUEUE_REMOVE_HEAD(prTempCmdDoneQue, prQueueEntry, P_QUE_ENTRY_T);
    }
}

VOID wlanClearDataQueue(IN P_ADAPTER_T prAdapter)
{
    QUE_T qDataPort0, qDataPort1;
    P_QUE_T prDataPort0, prDataPort1;
    
    KAL_SPIN_LOCK_DECLARATION();

    prDataPort0 = &qDataPort0;
    prDataPort1 = &qDataPort1;

    QUEUE_INITIALIZE(prDataPort0);
    QUEUE_INITIALIZE(prDataPort1);
        
	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_PORT_QUE);
    QUEUE_MOVE_ALL(prDataPort0, &prAdapter->rTxP0Queue);
    QUEUE_MOVE_ALL(prDataPort1, &prAdapter->rTxP1Queue);    
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_PORT_QUE);

	
	nicTxReturnMsduInfo(prAdapter, (P_MSDU_INFO_T) QUEUE_GET_HEAD(prDataPort0));
	nicTxReturnMsduInfo(prAdapter, (P_MSDU_INFO_T) QUEUE_GET_HEAD(prDataPort1));

}

VOID wlanClearRxToOsQueue(IN P_ADAPTER_T prAdapter)
{
    QUE_T rTempRxQue;
    P_QUE_T prTempRxQue = &rTempRxQue;      
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;

    KAL_SPIN_LOCK_DECLARATION();
    QUEUE_INITIALIZE(prTempRxQue);
    
	
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE);
    QUEUE_MOVE_ALL(prTempRxQue, &prAdapter->rRxQueue);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE); 

	
    QUEUE_REMOVE_HEAD(prTempRxQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		kalRxIndicateOnePkt(prAdapter->prGlueInfo,
				    (PVOID) GLUE_GET_PKT_DESCRIPTOR(prQueueEntry));
        QUEUE_REMOVE_HEAD(prTempRxQue, prQueueEntry, P_QUE_ENTRY_T);
    }    
     
}
#endif


VOID
wlanReleaseCommand(IN P_ADAPTER_T prAdapter,
		   IN P_CMD_INFO_T prCmdInfo, IN ENUM_TX_RESULT_CODE_T rTxDoneStatus)
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prMsduInfo;

    ASSERT(prAdapter);
    ASSERT(prCmdInfo);

    prTxCtrl = &prAdapter->rTxCtrl;

	switch (prCmdInfo->eCmdType) {
    case COMMAND_TYPE_GENERAL_IOCTL:
    case COMMAND_TYPE_NETWORK_IOCTL:
        DBGLOG(INIT, INFO, ("Free CMD: BSS[%u] ID[0x%x] SeqNum[%u] OID[%u]\n",
            prCmdInfo->ucBssIndex,
            prCmdInfo->ucCID,
            prCmdInfo->ucCmdSeqNum,
            prCmdInfo->fgIsOid));

        if (prCmdInfo->fgIsOid) {
            kalOidComplete(prAdapter->prGlueInfo,
                    prCmdInfo->fgSetQuery,
				       prCmdInfo->u4SetInfoLen, WLAN_STATUS_FAILURE);
        }
        break;

    case COMMAND_TYPE_SECURITY_FRAME:
        DBGLOG(INIT, INFO, ("Free SEC Frame: BSS[%u] StaRec[%u] SeqNum[%u]\n",
            prCmdInfo->ucBssIndex,
            prCmdInfo->ucStaRecIndex,
            prCmdInfo->ucCmdSeqNum));

        kalSecurityFrameSendComplete(prAdapter->prGlueInfo,
					     prCmdInfo->prPacket, WLAN_STATUS_FAILURE);
        break;

    case COMMAND_TYPE_MANAGEMENT_FRAME:
		prMsduInfo = (P_MSDU_INFO_T) prCmdInfo->prPacket;

        DBGLOG(INIT, INFO, ("Free MGMT Frame: BSS[%u] WIDX:PID[%u:%u] SEQ[%u] STA[%u] RSP[%u]\n",
            prMsduInfo->ucBssIndex,
            prMsduInfo->ucWlanIndex,
            prMsduInfo->ucPID,
            prMsduInfo->ucTxSeqNum,
            prMsduInfo->ucStaRecIndex,
            prMsduInfo->pfTxDoneHandler ? TRUE:FALSE));  

        
		if (prMsduInfo->pfTxDoneHandler != NULL) {
            prMsduInfo->pfTxDoneHandler(prAdapter, prMsduInfo, rTxDoneStatus);
        }

        GLUE_DEC_REF_CNT(prTxCtrl->i4TxMgmtPendingNum);
        cnmMgtPktFree(prAdapter, prMsduInfo);
        break;

    default:
        ASSERT(0);
        break;
    }

} 


VOID wlanReleasePendingOid(IN P_ADAPTER_T prAdapter, IN ULONG ulParamPtr)
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("wlanReleasePendingOid");

    ASSERT(prAdapter);

	if (prAdapter->prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
        DBGLOG(INIT, INFO, ("tx_thread stopped! Releasing pending OIDs ..\n"));
	} else {
        DBGLOG(INIT, ERROR, ("OID Timeout! Releasing pending OIDs ..\n"));
        prAdapter->ucOidTimeoutCount++;
        
		if (prAdapter->ucOidTimeoutCount >= WLAN_OID_NO_ACK_THRESHOLD) {
			if (!prAdapter->fgIsChipNoAck) {
				DBGLOG(INIT, WARN,
				       ("No response from chip for %u times, set NoAck flag!\n",
					prAdapter->ucOidTimeoutCount));

#if CFG_CHIP_RESET_SUPPORT
                glResetTrigger(prAdapter);
#endif
            }
        
            prAdapter->fgIsChipNoAck = TRUE;           
        }
    }

    do {
#if CFG_SUPPORT_MULTITHREAD        
        KAL_ACQUIRE_MUTEX(prAdapter, MUTEX_TX_CMD_CLEAR);
#endif

        
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);

        prCmdQue = &prAdapter->rPendingCmdQueue;
        QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
        while (prQueueEntry) {
			prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

            if (prCmdInfo->fgIsOid) {
                if (prCmdInfo->pfCmdTimeoutHandler) {
                    prCmdInfo->pfCmdTimeoutHandler(prAdapter, prCmdInfo);
				} else {
                    kalOidComplete(prAdapter->prGlueInfo,
                            prCmdInfo->fgSetQuery,
						       0, WLAN_STATUS_FAILURE);
                }    

                cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
			} else {
                QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
            }

            QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
        }

        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);

#if CFG_SUPPORT_MULTITHREAD
        
        wlanClearTxOidCommand(prAdapter);
#endif

		
        kalOidCmdClearance(prAdapter->prGlueInfo);

		
        kalOidClearance(prAdapter->prGlueInfo);

#if CFG_SUPPORT_MULTITHREAD   
        KAL_RELEASE_MUTEX(prAdapter, MUTEX_TX_CMD_CLEAR);
#endif
    } while(FALSE);

    return;
}


VOID wlanReleasePendingCMDbyBssIdx(IN P_ADAPTER_T prAdapter, IN UINT_8 ucBssIndex)
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    do {
		
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);

        prCmdQue = &prAdapter->rPendingCmdQueue;
        QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
        while (prQueueEntry) {
			prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

			DBGLOG(P2P, TRACE, ("Pending CMD for BSS:%d\n", prCmdInfo->ucBssIndex));

            if (prCmdInfo->ucBssIndex == ucBssIndex) {
                if (prCmdInfo->pfCmdTimeoutHandler) {
                    prCmdInfo->pfCmdTimeoutHandler(prAdapter, prCmdInfo);
				} else if (prCmdInfo->fgIsOid) {
                    kalOidComplete(prAdapter->prGlueInfo,
                            prCmdInfo->fgSetQuery,
						       0, WLAN_STATUS_FAILURE);
                }

                cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
			} else {
                QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
            }

            QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
        }

        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CMD_PENDING);


	} while (FALSE);

    return;
} 

VOID wlanReturnPacketDelaySetupTimeout(IN P_ADAPTER_T prAdapter, IN ULONG ulParamPtr)
{
    P_RX_CTRL_T prRxCtrl;
    P_SW_RFB_T prSwRfb = NULL;
    KAL_SPIN_LOCK_DECLARATION();
    WLAN_STATUS status = WLAN_STATUS_SUCCESS;
    P_QUE_T prQueList;

    ASSERT(prAdapter);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);
    
    prQueList = &prRxCtrl->rIndicatedRfbList;
	DBGLOG(RX, WARN, ("%s: IndicatedRfbList num = %u\n", __func__, prQueList->u4NumElem));

	while (QUEUE_IS_NOT_EMPTY(&prRxCtrl->rIndicatedRfbList)) {
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
        QUEUE_REMOVE_HEAD(&prRxCtrl->rIndicatedRfbList, prSwRfb, P_SW_RFB_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

        status = nicRxSetupRFB(prAdapter, prSwRfb);
        nicRxReturnRFB(prAdapter, prSwRfb);

		if (status != WLAN_STATUS_SUCCESS) {
            break;
        }
    }
    
	if (status != WLAN_STATUS_SUCCESS) {
		DBGLOG(RX, WARN,
		       ("Restart ReturnIndicatedRfb Timer (%u)\n",
			RX_RETURN_INDICATED_RFB_TIMEOUT_SEC));
		
        cnmTimerStartTimer(prAdapter,
                &prAdapter->rPacketDelaySetupTimer,
                SEC_TO_MSEC(RX_RETURN_INDICATED_RFB_TIMEOUT_SEC));
    }
}

VOID wlanReturnPacket(IN P_ADAPTER_T prAdapter, IN PVOID pvPacket)
{
    P_RX_CTRL_T prRxCtrl;
    P_SW_RFB_T prSwRfb = NULL;
    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("wlanReturnPacket");

    ASSERT(prAdapter);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    if (pvPacket) {
        kalPacketFree(prAdapter->prGlueInfo, pvPacket);
        RX_ADD_CNT(prRxCtrl, RX_DATA_RETURNED_COUNT, 1);
#if CFG_NATIVE_802_11
        if (GLUE_TEST_FLAG(prAdapter->prGlueInfo, GLUE_FLAG_HALT)) {
        }
#endif
    }

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
    QUEUE_REMOVE_HEAD(&prRxCtrl->rIndicatedRfbList, prSwRfb, P_SW_RFB_T);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
	if (!prSwRfb) {
        DBGLOG(RX, WARN, ("No free SwRfb!\n"));
        return;
    }

	if (nicRxSetupRFB(prAdapter, prSwRfb)) {
        DBGLOG(RX, WARN, ("Cannot allocate packet buffer for SwRfb!\n"));
		if (!timerPendingTimer(&prAdapter->rPacketDelaySetupTimer)) {
			DBGLOG(RX, WARN,
			       ("Start ReturnIndicatedRfb Timer (%u)\n",
				RX_RETURN_INDICATED_RFB_TIMEOUT_SEC));
			cnmTimerStartTimer(prAdapter, &prAdapter->rPacketDelaySetupTimer,
                    SEC_TO_MSEC(RX_RETURN_INDICATED_RFB_TIMEOUT_SEC));
        }
    }
    nicRxReturnRFB(prAdapter, prSwRfb);
}

/*!
* \brief This function is a required function that returns information about
*        the capabilities and status of the driver and/or its network adapter.
*
* \param[IN] prAdapter        Pointer to the Adapter structure.
* \param[IN] pfnOidQryHandler Function pointer for the OID query handler.
* \param[IN] pvInfoBuf        Points to a buffer for return the query information.
* \param[IN] u4QueryBufferLen Specifies the number of bytes at pvInfoBuf.
* \param[OUT] pu4QueryInfoLen  Points to the number of bytes it written or is needed.
*
* \retval WLAN_STATUS_xxx Different WLAN_STATUS code returned by different handlers.
*
*/
WLAN_STATUS
wlanQueryInformation(IN P_ADAPTER_T prAdapter,
    IN PFN_OID_HANDLER_FUNC pfnOidQryHandler,
		     IN PVOID pvInfoBuf, IN UINT_32 u4InfoBufLen, OUT PUINT_32 pu4QryInfoLen)
{
    WLAN_STATUS status = WLAN_STATUS_FAILURE;

    ASSERT(prAdapter);
    ASSERT(pu4QryInfoLen);

	
    if (prAdapter->u4PsCurrentMeasureEn &&
        (prAdapter->prGlueInfo->eParamMediaStateIndicated == PARAM_MEDIA_STATE_CONNECTED)) {
		return WLAN_STATUS_SUCCESS;	
    }
#if 1
    
	status = pfnOidQryHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4QryInfoLen);
#else
    if (wlanIsHandlerNeedHwAccess(pfnOidQryHandler, FALSE)) {
        ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

        
		if (prAdapter->fgWiFiInSleepyState == TRUE) {
            prAdapter->fgWiFiInSleepyState = FALSE;
        }

		status = pfnOidQryHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4QryInfoLen);

        RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);
	} else {
		status = pfnOidQryHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4QryInfoLen);
    }
#endif

    return status;

}

WLAN_STATUS
wlanSetInformation(IN P_ADAPTER_T prAdapter,
    IN PFN_OID_HANDLER_FUNC pfnOidSetHandler,
		   IN PVOID pvInfoBuf, IN UINT_32 u4InfoBufLen, OUT PUINT_32 pu4SetInfoLen)
{
    WLAN_STATUS status = WLAN_STATUS_FAILURE;

    ASSERT(prAdapter);
    ASSERT(pu4SetInfoLen);

	
    if (prAdapter->u4PsCurrentMeasureEn &&
        (prAdapter->prGlueInfo->eParamMediaStateIndicated == PARAM_MEDIA_STATE_CONNECTED)) {
		return WLAN_STATUS_SUCCESS;	
    }
#if 1
	status = pfnOidSetHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4SetInfoLen);
#else
    if (wlanIsHandlerNeedHwAccess(pfnOidSetHandler, TRUE)) {
        ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

        
		if (prAdapter->fgWiFiInSleepyState == TRUE) {
            prAdapter->fgWiFiInSleepyState = FALSE;
        }

		status = pfnOidSetHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4SetInfoLen);

        RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);
	} else {
		status = pfnOidSetHandler(prAdapter, pvInfoBuf, u4InfoBufLen, pu4SetInfoLen);
    }
#endif

    return status;
}


#if CFG_SUPPORT_WAPI
BOOLEAN wlanQueryWapiMode(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return prAdapter->rWifiVar.rConnSettings.fgWapiMode;
}
#endif


VOID wlanSetPromiscuousMode(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnablePromiscuousMode)
{
    ASSERT(prAdapter);

}

VOID wlanRxSetBroadcast(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnableBroadcast)
{
    ASSERT(prAdapter);
}

WLAN_STATUS wlanSendNicPowerCtrlCmd(IN P_ADAPTER_T prAdapter, IN UINT_8 ucPowerMode)
{
    WLAN_STATUS status = WLAN_STATUS_SUCCESS;
    P_GLUE_INFO_T prGlueInfo;
    P_CMD_INFO_T prCmdInfo;
    P_WIFI_CMD_T prWifiCmd;
    UINT_8 ucTC, ucCmdSeqNum;

    ASSERT(prAdapter);

    prGlueInfo = prAdapter->prGlueInfo;

    
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, (CMD_HDR_SIZE + sizeof(CMD_NIC_POWER_CTRL)));
    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }

    
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);
    DBGLOG(REQ, TRACE, ("ucCmdSeqNum =%d\n", ucCmdSeqNum));

    
    prCmdInfo->eCmdType = COMMAND_TYPE_GENERAL_IOCTL;
	prCmdInfo->u2InfoBufLen = (UINT_16) (CMD_HDR_SIZE + sizeof(CMD_NIC_POWER_CTRL));
    prCmdInfo->pfCmdDoneHandler = NULL;
    prCmdInfo->pfCmdTimeoutHandler = NULL;
    prCmdInfo->fgIsOid = TRUE;
    prCmdInfo->ucCID = CMD_ID_NIC_POWER_CTRL;
    prCmdInfo->fgSetQuery = TRUE;
    prCmdInfo->fgNeedResp = FALSE;
    prCmdInfo->fgDriverDomainMCR = FALSE;
    prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
    prCmdInfo->u4SetInfoLen = sizeof(CMD_NIC_POWER_CTRL);

    
	prWifiCmd = (P_WIFI_CMD_T) (prCmdInfo->pucInfoBuffer);
    prWifiCmd->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prWifiCmd->u2PQ_ID = CMD_PQ_ID;
    prWifiCmd->ucPktTypeID = CMD_PACKET_TYPE_ID;
    prWifiCmd->ucCID = prCmdInfo->ucCID;
    prWifiCmd->ucSetQuery = prCmdInfo->fgSetQuery;
    prWifiCmd->ucSeqNum = prCmdInfo->ucCmdSeqNum;

    kalMemZero(prWifiCmd->aucBuffer, sizeof(CMD_NIC_POWER_CTRL));
	((P_CMD_NIC_POWER_CTRL) (prWifiCmd->aucBuffer))->ucPowerMode = ucPowerMode;

    
    ucTC = TC4_INDEX;

	while (1) {
		
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE || fgIsBusAccessFailed == TRUE) {
            status = WLAN_STATUS_FAILURE;
            break;
        }
		
		if (nicTxAcquireResource(prAdapter, ucTC, nicTxGetCmdPageCount(prCmdInfo)) ==
		    WLAN_STATUS_RESOURCES) {
            if (nicTxPollingResource(prAdapter, ucTC) != WLAN_STATUS_SUCCESS) {
				DBGLOG(INIT, ERROR,
				       ("Fail to get TX resource return within timeout\n"));
                status = WLAN_STATUS_FAILURE;
                prAdapter->fgIsChipNoAck = TRUE;
                break;
			} else {
                continue;
            }
        }
		
        if (nicTxCmd(prAdapter, prCmdInfo, ucTC) != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR, ("Fail to transmit CMD_NIC_POWER_CTRL command\n"));
            status = WLAN_STATUS_FAILURE;
        }

        break;
    };

	
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

	
	if (ucPowerMode == 1) {
        prAdapter->fgIsEnterD3ReqIssued = TRUE;
    }

    return status;
}

BOOLEAN wlanIsHandlerAllowedInRFTest(IN PFN_OID_HANDLER_FUNC pfnOidHandler, IN BOOLEAN fgSetInfo)
{
	PFN_OID_HANDLER_FUNC *apfnOidHandlerAllowedInRFTest;
    UINT_32 i;
    UINT_32 u4NumOfElem;

    if (fgSetInfo) {
        apfnOidHandlerAllowedInRFTest = apfnOidSetHandlerAllowedInRFTest;
		u4NumOfElem =
		    sizeof(apfnOidSetHandlerAllowedInRFTest) / sizeof(PFN_OID_HANDLER_FUNC);
	} else {
        apfnOidHandlerAllowedInRFTest = apfnOidQueryHandlerAllowedInRFTest;
		u4NumOfElem =
		    sizeof(apfnOidQueryHandlerAllowedInRFTest) / sizeof(PFN_OID_HANDLER_FUNC);
    }

    for (i = 0; i < u4NumOfElem; i++) {
        if (apfnOidHandlerAllowedInRFTest[i] == pfnOidHandler) {
            return TRUE;
        }
    }

    return FALSE;
}

#if CFG_ENABLE_FW_DOWNLOAD
WLAN_STATUS
wlanImageSectionConfig(IN P_ADAPTER_T prAdapter,
		       IN UINT_32 u4DestAddr, IN UINT_32 u4ImgSecSize, IN BOOLEAN fgReset)
{
    P_CMD_INFO_T prCmdInfo;
    P_INIT_HIF_TX_HEADER_T prInitHifTxHeader;
    P_INIT_CMD_DOWNLOAD_CONFIG prInitCmdDownloadConfig;
    UINT_8 ucTC, ucCmdSeqNum;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;

    ASSERT(prAdapter);

    DEBUGFUNC("wlanImageSectionConfig");

    if (u4ImgSecSize == 0) {
        return WLAN_STATUS_SUCCESS;
    }
	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter,
					  sizeof(INIT_HIF_TX_HEADER_T) +
					  sizeof(INIT_CMD_DOWNLOAD_CONFIG));

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }

    prCmdInfo->u2InfoBufLen = sizeof(INIT_HIF_TX_HEADER_T) + sizeof(INIT_CMD_DOWNLOAD_CONFIG);

	
    ucTC = TC4_INDEX;

	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
	prInitHifTxHeader = (P_INIT_HIF_TX_HEADER_T) (prCmdInfo->pucInfoBuffer);
    prInitHifTxHeader->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prInitHifTxHeader->u2PQ_ID = INIT_CMD_PQ_ID;

    prInitHifTxHeader->rInitWifiCmd.ucCID = INIT_CMD_ID_DOWNLOAD_CONFIG;
    prInitHifTxHeader->rInitWifiCmd.ucPktTypeID = INIT_CMD_PACKET_TYPE_ID;
    prInitHifTxHeader->rInitWifiCmd.ucSeqNum = ucCmdSeqNum;

	
	prInitCmdDownloadConfig =
	    (P_INIT_CMD_DOWNLOAD_CONFIG) (prInitHifTxHeader->rInitWifiCmd.aucBuffer);
    prInitCmdDownloadConfig->u4Address = u4DestAddr;
    prInitCmdDownloadConfig->u4Length = u4ImgSecSize;
    prInitCmdDownloadConfig->u4DataMode = 0
#if CFG_ENABLE_FW_DOWNLOAD_ACK
	    | DOWNLOAD_CONFIG_ACK_OPTION	
#endif
#if CFG_ENABLE_FW_ENCRYPTION
        | DOWNLOAD_CONFIG_ENCRYPTION_MODE
#endif
        ;

	if (fgReset == TRUE) {
        prInitCmdDownloadConfig->u4DataMode |= DOWNLOAD_CONFIG_RESET_OPTION;
    }
	
	while (1) {
		
		if (nicTxAcquireResource
		    (prAdapter, ucTC,
		     nicTxGetPageCount(prCmdInfo->u2InfoBufLen, TRUE)) == WLAN_STATUS_RESOURCES) {
            if (nicTxPollingResource(prAdapter, ucTC) != WLAN_STATUS_SUCCESS) {
                u4Status = WLAN_STATUS_FAILURE;
				DBGLOG(INIT, ERROR,
				       ("Fail to get TX resource return within timeout\n"));
                break;
			} else {
                continue;
            }
        }
		
        if (nicTxInitCmd(prAdapter, prCmdInfo) != WLAN_STATUS_SUCCESS) {
            u4Status = WLAN_STATUS_FAILURE;
			DBGLOG(INIT, ERROR, ("Fail to transmit image download command\n"));
        }

        break;
    };

#if CFG_ENABLE_FW_DOWNLOAD_ACK
	
    u4Status = wlanImageSectionDownloadStatus(prAdapter, ucCmdSeqNum);
#endif

	
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

    return u4Status;
}


WLAN_STATUS
wlanImageSectionDownload(IN P_ADAPTER_T prAdapter, IN UINT_32 u4ImgSecSize, IN PUINT_8 pucImgSecBuf)
{
    P_CMD_INFO_T prCmdInfo;
    P_INIT_HIF_TX_HEADER_T prInitHifTxHeader;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;

    ASSERT(prAdapter);
    ASSERT(pucImgSecBuf);
    ASSERT(u4ImgSecSize <= CMD_PKT_SIZE_FOR_IMAGE);

    DEBUGFUNC("wlanImageSectionDownload");

    if (u4ImgSecSize == 0) {
        return WLAN_STATUS_SUCCESS;
    }
	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, sizeof(INIT_HIF_TX_HEADER_T) + u4ImgSecSize);

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }

	prCmdInfo->u2InfoBufLen = sizeof(INIT_HIF_TX_HEADER_T) + (UINT_16) u4ImgSecSize;

	
	prInitHifTxHeader = (P_INIT_HIF_TX_HEADER_T) (prCmdInfo->pucInfoBuffer);
    prInitHifTxHeader->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prInitHifTxHeader->u2PQ_ID = INIT_CMD_PDA_PQ_ID;

    prInitHifTxHeader->rInitWifiCmd.ucCID = 0;
    prInitHifTxHeader->rInitWifiCmd.ucPktTypeID = INIT_CMD_PDA_PACKET_TYPE_ID;
    prInitHifTxHeader->rInitWifiCmd.ucSeqNum = 0;

	
    kalMemCopy(prInitHifTxHeader->rInitWifiCmd.aucBuffer, pucImgSecBuf, u4ImgSecSize);

	
    if (nicTxInitCmd(prAdapter, prCmdInfo) != WLAN_STATUS_SUCCESS) {
        u4Status = WLAN_STATUS_FAILURE;
		DBGLOG(INIT, ERROR, ("Fail to transmit image download command\n"));
    }
	
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

    return u4Status;
}


VOID
wlanFwDvdDwnloadHandler(IN P_ADAPTER_T prAdapter,
		IN P_FIRMWARE_DIVIDED_DOWNLOAD_T prFwHead, IN PVOID pvFwImageMapFile, OUT WLAN_STATUS *u4Status)
{
	UINT_32 u4ImgSecSize, i, j;
	
	for (i = 0; i < prFwHead->u4NumOfEntries; i++) {
		if (wlanImageSectionConfig(prAdapter,
					prFwHead->arSection[i].u4DestAddr,
					prFwHead->arSection[i].u4Length,
					i == 0 ? TRUE : FALSE) != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR, ("Firmware download configuration failed!\n"));
			*u4Status = WLAN_STATUS_FAILURE;
			break;
		} else {
			for (j = 0; j < prFwHead->arSection[i].u4Length; j += CMD_PKT_SIZE_FOR_IMAGE) {
				if (j + CMD_PKT_SIZE_FOR_IMAGE < prFwHead->arSection[i].u4Length)
					u4ImgSecSize = CMD_PKT_SIZE_FOR_IMAGE;
				else
					u4ImgSecSize = prFwHead->arSection[i].u4Length - j;

				if (wlanImageSectionDownload(prAdapter,
							     u4ImgSecSize,
							     (PUINT_8)pvFwImageMapFile + prFwHead->arSection[i].u4Offset + j) != WLAN_STATUS_SUCCESS) {
					DBGLOG(INIT, ERROR, ("Firmware scatter download failed!\n"));
					*u4Status = WLAN_STATUS_FAILURE;
					break;
				}
			}

			
			if (*u4Status == WLAN_STATUS_FAILURE) {
				break;
			}
		}
	}
}


VOID
wlanFwDwnloadHandler(IN P_ADAPTER_T prAdapter,
	IN UINT_32 u4FwImgLength, IN PVOID pvFwImageMapFile, OUT WLAN_STATUS *u4Status)
{
	UINT_32 u4ImgSecSize, i;
	
	for (i = 0; i < u4FwImgLength; i += CMD_PKT_SIZE_FOR_IMAGE) {
		if (i + CMD_PKT_SIZE_FOR_IMAGE < u4FwImgLength)
			u4ImgSecSize = CMD_PKT_SIZE_FOR_IMAGE;
		else
			u4ImgSecSize = u4FwImgLength - i;

		if (wlanImageSectionDownload(prAdapter,
					     u4ImgSecSize,
					     (PUINT_8)pvFwImageMapFile + i) != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, ERROR, ("wlanImageSectionDownload failed!\n"));
			*u4Status = WLAN_STATUS_FAILURE;
			break;
		}
	}
}

#if !CFG_ENABLE_FW_DOWNLOAD_ACK
WLAN_STATUS wlanImageQueryStatus(IN P_ADAPTER_T prAdapter)
{
    P_CMD_INFO_T prCmdInfo;
    P_INIT_HIF_TX_HEADER_T prInitHifTxHeader;
    UINT_8 aucBuffer[sizeof(INIT_HIF_RX_HEADER_T) + sizeof(INIT_EVENT_PENDING_ERROR)];
    UINT_32 u4RxPktLength;
    P_INIT_HIF_RX_HEADER_T prInitHifRxHeader;
    P_INIT_EVENT_PENDING_ERROR prEventPendingError;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
    UINT_8 ucTC, ucCmdSeqNum;

    ASSERT(prAdapter);

    DEBUGFUNC("wlanImageQueryStatus");

	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, sizeof(INIT_HIF_TX_HEADER_T));

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }

    kalMemZero(prCmdInfo->pucInfoBuffer, sizeof(INIT_HIF_TX_HEADER_T));
    prCmdInfo->u2InfoBufLen = sizeof(INIT_HIF_TX_HEADER_T);

	
    ucTC = TC0_INDEX;

	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
	prInitHifTxHeader = (P_INIT_HIF_TX_HEADER_T) (prCmdInfo->pucInfoBuffer);

    prInitHifTxHeader->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prInitHifTxHeader->u2PQ_ID = INIT_CMD_PQ_ID;

    prInitHifTxHeader->rInitWifiCmd.ucCID = INIT_CMD_ID_QUERY_PENDING_ERROR;
    prInitHifTxHeader->rInitWifiCmd.ucPktTypeID = INIT_CMD_PACKET_TYPE_ID;
    prInitHifTxHeader->rInitWifiCmd.ucSeqNum = ucCmdSeqNum;

	
	while (1) {
		
		if (nicTxAcquireResource
		    (prAdapter, ucTC,
		     nicTxGetPageCount(prCmdInfo->u2InfoBufLen, TRUE)) == WLAN_STATUS_RESOURCES) {
            if (nicTxPollingResource(prAdapter, ucTC) != WLAN_STATUS_SUCCESS) {
                u4Status = WLAN_STATUS_FAILURE;
				DBGLOG(INIT, ERROR,
				       ("Fail to get TX resource return within timeout\n"));
                break;
			} else {
                continue;
            }
        }
		
        if (nicTxInitCmd(prAdapter, prCmdInfo) != WLAN_STATUS_SUCCESS) {
            u4Status = WLAN_STATUS_FAILURE;
			DBGLOG(INIT, ERROR, ("Fail to transmit image download command\n"));
        }

        break;
    };

	
    do {
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE || fgIsBusAccessFailed == TRUE) {
            u4Status = WLAN_STATUS_FAILURE;
		} else if (nicRxWaitResponse(prAdapter,
                    0,
                    aucBuffer,
					     sizeof(INIT_HIF_RX_HEADER_T) +
					     sizeof(INIT_EVENT_PENDING_ERROR),
                    &u4RxPktLength) != WLAN_STATUS_SUCCESS) {
            u4Status = WLAN_STATUS_FAILURE;
		} else {
            prInitHifRxHeader = (P_INIT_HIF_RX_HEADER_T) aucBuffer;

			
			if (prInitHifRxHeader->rInitWifiEvent.ucEID != INIT_EVENT_ID_PENDING_ERROR) {
                u4Status = WLAN_STATUS_FAILURE;
			} else if (prInitHifRxHeader->rInitWifiEvent.ucSeqNum != ucCmdSeqNum) {
                u4Status = WLAN_STATUS_FAILURE;
			} else {
				prEventPendingError =
				    (P_INIT_EVENT_PENDING_ERROR) (prInitHifRxHeader->rInitWifiEvent.
								  aucBuffer);
				if (prEventPendingError->ucStatus != 0) {	
                    u4Status = WLAN_STATUS_FAILURE;
				} else {
                    u4Status = WLAN_STATUS_SUCCESS;
                }
            }
        }
    } while (FALSE);

	
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

    return u4Status;
}


#else
WLAN_STATUS wlanImageSectionDownloadStatus(IN P_ADAPTER_T prAdapter, IN UINT_8 ucCmdSeqNum)
{
    UINT_8 aucBuffer[sizeof(INIT_HIF_RX_HEADER_T) + sizeof(INIT_EVENT_CMD_RESULT)];
    P_INIT_HIF_RX_HEADER_T prInitHifRxHeader;
    P_INIT_EVENT_CMD_RESULT prEventCmdResult;
    UINT_32 u4RxPktLength;
    WLAN_STATUS u4Status;

    ASSERT(prAdapter);

    do {
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == TRUE || fgIsBusAccessFailed == TRUE) {
            u4Status = WLAN_STATUS_FAILURE;
		} else if (nicRxWaitResponse(prAdapter,
                    0,
                    aucBuffer,
					     sizeof(INIT_HIF_RX_HEADER_T) +
					     sizeof(INIT_EVENT_CMD_RESULT),
                    &u4RxPktLength) != WLAN_STATUS_SUCCESS) {
            u4Status = WLAN_STATUS_FAILURE;
		} else {
            prInitHifRxHeader = (P_INIT_HIF_RX_HEADER_T) aucBuffer;

			
			if (prInitHifRxHeader->rInitWifiEvent.ucEID != INIT_EVENT_ID_CMD_RESULT) {
                u4Status = WLAN_STATUS_FAILURE;
			} else if (prInitHifRxHeader->rInitWifiEvent.ucSeqNum != ucCmdSeqNum) {
                u4Status = WLAN_STATUS_FAILURE;
			} else {
				prEventCmdResult =
				    (P_INIT_EVENT_CMD_RESULT) (prInitHifRxHeader->rInitWifiEvent.
							       aucBuffer);
				if (prEventCmdResult->ucStatus != 0) {	
                    u4Status = WLAN_STATUS_FAILURE;
				} else {
                    u4Status = WLAN_STATUS_SUCCESS;
                }
            }
        }
    } while (FALSE);

    return u4Status;
}


#endif
WLAN_STATUS
wlanConfigWifiFunc(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnable, IN UINT_32 u4StartAddress)
{
    P_CMD_INFO_T prCmdInfo;
    P_INIT_HIF_TX_HEADER_T prInitHifTxHeader;
    P_INIT_CMD_WIFI_START prInitCmdWifiStart;
    UINT_8 ucTC, ucCmdSeqNum;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;

    ASSERT(prAdapter);

    DEBUGFUNC("wlanConfigWifiFunc");

	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter,
					  sizeof(INIT_HIF_TX_HEADER_T) +
					  sizeof(INIT_CMD_WIFI_START));

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }

	kalMemZero(prCmdInfo->pucInfoBuffer, 
        sizeof(INIT_HIF_TX_HEADER_T) + sizeof(INIT_CMD_WIFI_START));
	prCmdInfo->u2InfoBufLen = 
        sizeof(INIT_HIF_TX_HEADER_T) + sizeof(INIT_CMD_WIFI_START);

	
    ucTC = TC0_INDEX;

	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
	prInitHifTxHeader = (P_INIT_HIF_TX_HEADER_T) (prCmdInfo->pucInfoBuffer);
    prInitHifTxHeader->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prInitHifTxHeader->u2PQ_ID = INIT_CMD_PQ_ID;

    prInitHifTxHeader->rInitWifiCmd.ucCID = INIT_CMD_ID_WIFI_START;
    prInitHifTxHeader->rInitWifiCmd.ucPktTypeID = INIT_CMD_PACKET_TYPE_ID;
    prInitHifTxHeader->rInitWifiCmd.ucSeqNum = ucCmdSeqNum;

	prInitCmdWifiStart = (P_INIT_CMD_WIFI_START) (prInitHifTxHeader->rInitWifiCmd.aucBuffer);
    prInitCmdWifiStart->u4Override = (fgEnable == TRUE ? 1 : 0);
    prInitCmdWifiStart->u4Address = u4StartAddress;

	
	while (1) {
		
		if (nicTxAcquireResource
		    (prAdapter, ucTC,
		     nicTxGetPageCount(prCmdInfo->u2InfoBufLen, TRUE)) == WLAN_STATUS_RESOURCES) {
            if (nicTxPollingResource(prAdapter, ucTC) != WLAN_STATUS_SUCCESS) {
                u4Status = WLAN_STATUS_FAILURE;
				DBGLOG(INIT, ERROR,
				       ("Fail to get TX resource return within timeout\n"));
                break;
			} else {
                continue;
            }
        }
		
        if (nicTxInitCmd(prAdapter, prCmdInfo) != WLAN_STATUS_SUCCESS) {
            u4Status = WLAN_STATUS_FAILURE;
			DBGLOG(INIT, ERROR, ("Fail to transmit WIFI start command\n"));
        }

        break;
    };

	
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

    return u4Status;
}


UINT_32 wlanCRC32(PUINT_8 buf, UINT_32 len)
{
    UINT_32 i, crc32 = 0xFFFFFFFF;
    const UINT_32 crc32_ccitt_table[256] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419,
        0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4,
        0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07,
        0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
        0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856,
        0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
        0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4,
        0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
        0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3,
        0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac, 0x51de003a,
        0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599,
        0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
        0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190,
        0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f,
        0x9fbfe4a5, 0xe8b8d433, 0x7807c9a2, 0x0f00f934, 0x9609a88e,
        0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
        0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed,
        0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3,
        0xfbd44c65, 0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
        0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a,
        0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5,
        0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010,
        0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17,
        0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6,
        0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615,
        0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
        0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 0xf00f9344,
        0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
        0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a,
        0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
        0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1,
        0xa6bc5767, 0x3fb506dd, 0x48b2364b, 0xd80d2bda, 0xaf0a1b4c,
        0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef,
        0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe,
        0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31,
        0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c,
        0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
        0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b,
        0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1,
        0x18b74777, 0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
        0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278,
        0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7,
        0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66,
        0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605,
        0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8,
        0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b,
		0x2d02ef8d
	};

    for (i = 0; i < len; i++)
        crc32 = crc32_ccitt_table[(crc32 ^ buf[i]) & 0xff] ^ (crc32 >> 8);

	return ~crc32;
}
#endif


WLAN_STATUS wlanProcessQueuedSwRfb(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfbListHead)
{
    P_SW_RFB_T prSwRfb, prNextSwRfb;
    P_TX_CTRL_T prTxCtrl;
    P_RX_CTRL_T prRxCtrl;

    ASSERT(prAdapter);
    ASSERT(prSwRfbListHead);

    prTxCtrl = &prAdapter->rTxCtrl;
    prRxCtrl = &prAdapter->rRxCtrl;

    prSwRfb = prSwRfbListHead;

    do {
		
		prNextSwRfb = (P_SW_RFB_T) QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T) prSwRfb);

		switch (prSwRfb->eDst) {
        case RX_PKT_DESTINATION_HOST:
            nicRxProcessPktWithoutReorder(prAdapter, prSwRfb);
            break;

        case RX_PKT_DESTINATION_FORWARD:
            nicRxProcessForwardPkt(prAdapter, prSwRfb);
            break;

        case RX_PKT_DESTINATION_HOST_WITH_FORWARD:
            nicRxProcessGOBroadcastPkt(prAdapter, prSwRfb);
            break;

        case RX_PKT_DESTINATION_NULL:
            nicRxReturnRFB(prAdapter, prSwRfb);
            break;

        default:
            break;
        }

#if CFG_HIF_RX_STARVATION_WARNING
        prRxCtrl->u4DequeuedCnt++;
#endif
        prSwRfb = prNextSwRfb;
	} while (prSwRfb);

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanProcessQueuedMsduInfo(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfoListHead)
{
    ASSERT(prAdapter);
    ASSERT(prMsduInfoListHead);

    nicTxFreeMsduInfoPacket(prAdapter, prMsduInfoListHead);
    nicTxReturnMsduInfo(prAdapter, prMsduInfoListHead);

    return WLAN_STATUS_SUCCESS;
}


BOOLEAN wlanoidTimeoutCheck(IN P_ADAPTER_T prAdapter, IN PFN_OID_HANDLER_FUNC pfnOidHandler)
{
	PFN_OID_HANDLER_FUNC *apfnOidHandlerWOTimeoutCheck;
    UINT_32 i;
    UINT_32 u4NumOfElem;
    UINT_32 u4OidTimeout;

    apfnOidHandlerWOTimeoutCheck = apfnOidWOTimeoutCheck;
    u4NumOfElem = sizeof(apfnOidWOTimeoutCheck) / sizeof(PFN_OID_HANDLER_FUNC);

    for (i = 0; i < u4NumOfElem; i++) {
        if (apfnOidHandlerWOTimeoutCheck[i] == pfnOidHandler) {
            return FALSE;
        }
    }

    
	if (wlanIsChipNoAck(prAdapter)) {
        u4OidTimeout = WLAN_OID_TIMEOUT_THRESHOLD_IN_RESETING;
		DBGLOG(INIT, INFO,
		       ("Decrease OID timeout to %ums due to NoACK/CHIP-RESET\n", u4OidTimeout));
	} else {
        u4OidTimeout = WLAN_OID_TIMEOUT_THRESHOLD;
    }
    
    
	cnmTimerStartTimer(prAdapter, &(prAdapter->rOidTimeoutTimer), u4OidTimeout);

    return TRUE;
}


VOID wlanoidClearTimeoutCheck(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    cnmTimerStopTimer(prAdapter, &(prAdapter->rOidTimeoutTimer));
}


WLAN_STATUS wlanUpdateNetworkAddress(IN P_ADAPTER_T prAdapter)
{
    const UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;
    PARAM_MAC_ADDRESS rMacAddr;
    UINT_32 u4SysTime;

    DEBUGFUNC("wlanUpdateNetworkAddress");

    ASSERT(prAdapter);

	if (kalRetrieveNetworkAddress(prAdapter->prGlueInfo, &rMacAddr) == FALSE
            || IS_BMCAST_MAC_ADDR(rMacAddr)
            || EQUAL_MAC_ADDR(aucZeroMacAddr, rMacAddr)) {
		
		if (prAdapter->fgIsEmbbededMacAddrValid == TRUE) {
#if CFG_SHOW_MACADDR_SOURCE
            DBGLOG(INIT, INFO, ("Using embedded MAC address"));
#endif
            return WLAN_STATUS_SUCCESS;
		} else {
#if CFG_SHOW_MACADDR_SOURCE
            DBGLOG(INIT, INFO, ("Using dynamically generated MAC address"));
#endif
			
            u4SysTime = (UINT_32) kalGetTimeTick();

            rMacAddr[0] = 0x00;
            rMacAddr[1] = 0x08;
            rMacAddr[2] = 0x22;

            kalMemCopy(&rMacAddr[3], &u4SysTime, 3);
        }
	} else {
#if CFG_SHOW_MACADDR_SOURCE
        DBGLOG(INIT, INFO, ("Using host-supplied MAC address"));
#endif
    }

    COPY_MAC_ADDR(prAdapter->rWifiVar.aucMacAddress, rMacAddr);

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanUpdateBasicConfig(IN P_ADAPTER_T prAdapter)
{
    UINT_8 ucCmdSeqNum;
    P_CMD_INFO_T prCmdInfo;
    P_WIFI_CMD_T prWifiCmd;
    P_CMD_BASIC_CONFIG_T prCmdBasicConfig;

    DEBUGFUNC("wlanUpdateBasicConfig");

    ASSERT(prAdapter);

    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, CMD_HDR_SIZE + sizeof(CMD_BASIC_CONFIG_T));

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }
	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
    prCmdInfo->eCmdType = COMMAND_TYPE_GENERAL_IOCTL;
    prCmdInfo->u2InfoBufLen = CMD_HDR_SIZE + sizeof(CMD_BASIC_CONFIG_T);
    prCmdInfo->pfCmdDoneHandler = NULL;
    prCmdInfo->pfCmdTimeoutHandler = NULL;
    prCmdInfo->fgIsOid = FALSE;
    prCmdInfo->ucCID = CMD_ID_BASIC_CONFIG;
    prCmdInfo->fgSetQuery = TRUE;
    prCmdInfo->fgNeedResp = FALSE;
    prCmdInfo->fgDriverDomainMCR = FALSE;
    prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
    prCmdInfo->u4SetInfoLen = sizeof(CMD_BASIC_CONFIG_T);

	
	prWifiCmd = (P_WIFI_CMD_T) (prCmdInfo->pucInfoBuffer);
    prWifiCmd->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prWifiCmd->u2PQ_ID = CMD_PQ_ID;
    prWifiCmd->ucPktTypeID = CMD_PACKET_TYPE_ID;
    prWifiCmd->ucCID = prCmdInfo->ucCID;
    prWifiCmd->ucSetQuery = prCmdInfo->fgSetQuery;
    prWifiCmd->ucSeqNum = prCmdInfo->ucCmdSeqNum;

	
	prCmdBasicConfig = (P_CMD_BASIC_CONFIG_T) (prWifiCmd->aucBuffer);
    prCmdBasicConfig->ucNative80211 = 0;
    prCmdBasicConfig->rCsumOffload.u2RxChecksum = 0;
    prCmdBasicConfig->rCsumOffload.u2TxChecksum = 0;

#if CFG_TCP_IP_CHKSUM_OFFLOAD
	if (prAdapter->u4CSUMFlags & CSUM_OFFLOAD_EN_TX_TCP)
        prCmdBasicConfig->rCsumOffload.u2TxChecksum |= BIT(2);

	if (prAdapter->u4CSUMFlags & CSUM_OFFLOAD_EN_TX_UDP)
        prCmdBasicConfig->rCsumOffload.u2TxChecksum |= BIT(1);

	if (prAdapter->u4CSUMFlags & CSUM_OFFLOAD_EN_TX_IP)
        prCmdBasicConfig->rCsumOffload.u2TxChecksum |= BIT(0);

	if (prAdapter->u4CSUMFlags & CSUM_OFFLOAD_EN_RX_TCP)
        prCmdBasicConfig->rCsumOffload.u2RxChecksum |= BIT(2);

	if (prAdapter->u4CSUMFlags & CSUM_OFFLOAD_EN_RX_UDP)
        prCmdBasicConfig->rCsumOffload.u2RxChecksum |= BIT(1);

	if (prAdapter->u4CSUMFlags & (CSUM_OFFLOAD_EN_RX_IPv4 | CSUM_OFFLOAD_EN_RX_IPv6))
        prCmdBasicConfig->rCsumOffload.u2RxChecksum |= BIT(0);
#endif

	if (wlanSendCommand(prAdapter, prCmdInfo) == WLAN_STATUS_RESOURCES) {
		kalEnqueueCommand(prAdapter->prGlueInfo, (P_QUE_ENTRY_T) prCmdInfo);

        return WLAN_STATUS_PENDING;
	} else {
        cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        
        return WLAN_STATUS_SUCCESS;
    }
}


BOOLEAN wlanQueryTestMode(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return prAdapter->fgTestMode;
}

BOOLEAN wlanProcessTxFrame(IN P_ADAPTER_T prAdapter, IN P_NATIVE_PACKET prPacket)
{
    UINT_32         u4SysTime;
    UINT_8          ucMacHeaderLen;
    TX_PACKET_INFO rTxPacketInfo;

    ASSERT(prAdapter);
    ASSERT(prPacket);

    if (kalQoSFrameClassifierAndPacketInfo(prAdapter->prGlueInfo,
					       prPacket, &rTxPacketInfo)) {
                
        
		GLUE_SET_PKT_TID(prPacket, rTxPacketInfo.ucPriorityParam);

		if (rTxPacketInfo.fgIs1X) {
            P_STA_RECORD_T          prStaRec;

			DBGLOG(RSN, INFO, ("T1X len=%d\n", rTxPacketInfo.u4PacketLen));

			prStaRec = cnmGetStaRecByAddress(prAdapter, 
                GLUE_GET_PKT_BSS_IDX(prPacket), rTxPacketInfo.aucEthDestAddr);
            
            GLUE_SET_PKT_FLAG_1X(prPacket);
            
			if (secIsProtected1xFrame(prAdapter, prStaRec)) {
                GLUE_SET_PKT_FLAG_PROTECTED_1X(prPacket);
            }
        }

		if (rTxPacketInfo.fgIs802_3) {
            GLUE_SET_PKT_FLAG_802_3(prPacket);
        }

		if (rTxPacketInfo.fgIsVlanExists) {
            GLUE_SET_PKT_FLAG_VLAN_EXIST(prPacket);
        }

        ucMacHeaderLen = ETHER_HEADER_LEN;

        
        GLUE_SET_PKT_HEADER_LEN(prPacket, ucMacHeaderLen);

        
		GLUE_SET_PKT_FRAME_LEN(prPacket, (UINT_16)rTxPacketInfo.u4PacketLen);

		
		u4SysTime = (OS_SYSTIME) kalGetTimeTick();
        GLUE_SET_PKT_ARRIVAL_TIME(prPacket, u4SysTime);
        
        return TRUE;
    }

    return FALSE;
}

BOOLEAN wlanProcessSecurityFrame(IN P_ADAPTER_T prAdapter, IN P_NATIVE_PACKET prPacket)
{
    P_CMD_INFO_T            prCmdInfo;
    P_STA_RECORD_T          prStaRec;
    UINT_8                  ucBssIndex;  
    UINT_32                 u4PacketLen;
    P_SEC_FRAME_INFO_T      prSecFrameInfo;
    UINT_8                  aucEthDestAddr[PARAM_MAC_ADDR_LEN];

    ASSERT(prAdapter);
    ASSERT(prPacket);

    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, sizeof(SEC_FRAME_INFO_T));

	u4PacketLen = (UINT_32) GLUE_GET_PKT_FRAME_LEN(prPacket);

    if (prCmdInfo) {
        ucBssIndex = GLUE_GET_PKT_BSS_IDX(prPacket);

        kalGetEthDestAddr(prAdapter->prGlueInfo, prPacket, aucEthDestAddr);

        prStaRec = cnmGetStaRecByAddress(prAdapter, ucBssIndex, aucEthDestAddr);

		prSecFrameInfo = (P_SEC_FRAME_INFO_T) prCmdInfo->pucInfoBuffer;
        prSecFrameInfo->fgIsProtected = GLUE_GET_PKT_IS_PROTECTED_1X(prPacket);

        prCmdInfo->eCmdType             = COMMAND_TYPE_SECURITY_FRAME;
		prCmdInfo->u2InfoBufLen = (UINT_16) u4PacketLen;
        prCmdInfo->prPacket             = prPacket;
		if (prStaRec) {
            prCmdInfo->ucStaRecIndex =  prStaRec->ucIndex;
		} else {
            prCmdInfo->ucStaRecIndex =  STA_REC_INDEX_NOT_FOUND;
        }
        prCmdInfo->ucBssIndex           = ucBssIndex;
        prCmdInfo->pfCmdDoneHandler     = wlanSecurityFrameTxDone;
        prCmdInfo->pfCmdTimeoutHandler  = wlanSecurityFrameTxTimeout;
        prCmdInfo->fgIsOid              = FALSE;
        prCmdInfo->fgSetQuery           = TRUE;
        prCmdInfo->fgNeedResp           = FALSE;

#if CFG_SUPPORT_MULTITHREAD
		nicTxComposeSecurityFrameDesc(prAdapter, prCmdInfo, prSecFrameInfo->ucTxDescBuffer,
					      NULL);
#endif

		kalEnqueueCommand(prAdapter->prGlueInfo, (P_QUE_ENTRY_T) prCmdInfo);

        GLUE_SET_EVENT(prAdapter->prGlueInfo);

        return TRUE;
	} else {
        DBGLOG(RSN, INFO, ("Failed to allocate CMD_INFO for 1X frame!!\n"));
    }

    return FALSE;
}

VOID
wlanSecurityFrameTxDone(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf)
{
    ASSERT(prAdapter);
    ASSERT(prCmdInfo);

	if (GET_BSS_INFO_BY_INDEX(prAdapter, prCmdInfo->ucBssIndex)->eNetworkType ==
	    NETWORK_TYPE_AIS && prAdapter->rWifiVar.rAisSpecificBssInfo.fgCounterMeasure) {
        P_STA_RECORD_T prSta = cnmGetStaRecByIndex(prAdapter, prCmdInfo->ucBssIndex);
        if (prSta) {
            kalMsleep(10);
            if (authSendDeauthFrame(prAdapter,
						GET_BSS_INFO_BY_INDEX(prAdapter,
								      prCmdInfo->ucBssIndex), prSta,
						(P_SW_RFB_T) NULL, REASON_CODE_MIC_FAILURE,
						(PFN_TX_DONE_HANDLER) NULL
						
						) != WLAN_STATUS_SUCCESS) {
                ASSERT(FALSE);
            }
			
        }
    }

    kalSecurityFrameSendComplete(prAdapter->prGlueInfo,
				     prCmdInfo->prPacket, WLAN_STATUS_SUCCESS);
}

VOID wlanSecurityFrameTxTimeout(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo)
{
    ASSERT(prAdapter);
    ASSERT(prCmdInfo);

    kalSecurityFrameSendComplete(prAdapter->prGlueInfo,
				     prCmdInfo->prPacket, WLAN_STATUS_FAILURE);
}


VOID wlanClearScanningResult(IN P_ADAPTER_T prAdapter)
{
    BOOLEAN fgKeepCurrOne = FALSE;
    UINT_32 i;

    ASSERT(prAdapter);

	
	if (kalGetMediaStateIndicated(prAdapter->prGlueInfo) == PARAM_MEDIA_STATE_CONNECTED) {
		for (i = 0; i < prAdapter->rWlanInfo.u4ScanResultNum; i++) {
			if (EQUAL_MAC_ADDR(prAdapter->rWlanInfo.rCurrBssId.arMacAddress,
                        prAdapter->rWlanInfo.arScanResult[i].arMacAddress)) {
                fgKeepCurrOne = TRUE;

				if (i != 0) {
					
                    kalMemCopy(&(prAdapter->rWlanInfo.arScanResult[0]),
                            &(prAdapter->rWlanInfo.arScanResult[i]),
                            OFFSET_OF(PARAM_BSSID_EX_T, aucIEs));
                }

				if (prAdapter->rWlanInfo.arScanResult[i].u4IELength > 0) {
					if (prAdapter->rWlanInfo.apucScanResultIEs[i] !=
					    &(prAdapter->rWlanInfo.aucScanIEBuf[0])) {
						
                        kalMemCopy(prAdapter->rWlanInfo.aucScanIEBuf,
							   prAdapter->rWlanInfo.
							   apucScanResultIEs[i],
							   prAdapter->rWlanInfo.arScanResult[i].
							   u4IELength);
                    }
					
					prAdapter->rWlanInfo.apucScanResultIEs[0] =
					    &(prAdapter->rWlanInfo.aucScanIEBuf[0]);
				} else {
                    prAdapter->rWlanInfo.apucScanResultIEs[0] = NULL;
                }

                break;
            }
        }
    }

	if (fgKeepCurrOne == TRUE) {
        prAdapter->rWlanInfo.u4ScanResultNum = 1;
        prAdapter->rWlanInfo.u4ScanIEBufferUsage =
            ALIGN_4(prAdapter->rWlanInfo.arScanResult[0].u4IELength);
	} else {
        prAdapter->rWlanInfo.u4ScanResultNum = 0;
        prAdapter->rWlanInfo.u4ScanIEBufferUsage = 0;
    }

    return;
}


VOID wlanClearBssInScanningResult(IN P_ADAPTER_T prAdapter, IN PUINT_8 arBSSID)
{
    UINT_32 i, j, u4IELength = 0, u4IEMoveLength;
    PUINT_8 pucIEPtr;

    ASSERT(prAdapter);

	
    i = 0;
	while (1) {
		if (i >= prAdapter->rWlanInfo.u4ScanResultNum) {
            break;
        }

		if (EQUAL_MAC_ADDR(arBSSID, prAdapter->rWlanInfo.arScanResult[i].arMacAddress)) {
			
            u4IELength = ALIGN_4(prAdapter->rWlanInfo.arScanResult[i].u4IELength);
            pucIEPtr = prAdapter->rWlanInfo.apucScanResultIEs[i];

			
			for (j = i + 1; j < prAdapter->rWlanInfo.u4ScanResultNum; j++) {
				kalMemCopy(&(prAdapter->rWlanInfo.arScanResult[j - 1]),
                        &(prAdapter->rWlanInfo.arScanResult[j]),
                        OFFSET_OF(PARAM_BSSID_EX_T, aucIEs));

				prAdapter->rWlanInfo.apucScanResultIEs[j - 1] =
                    prAdapter->rWlanInfo.apucScanResultIEs[j];
            }

            prAdapter->rWlanInfo.u4ScanResultNum--;

			
			if (u4IELength > 0) {
                u4IEMoveLength = prAdapter->rWlanInfo.u4ScanIEBufferUsage -
				    (((ULONG) pucIEPtr) + u4IELength -
				     ((ULONG) (&(prAdapter->rWlanInfo.aucScanIEBuf[0]))));

                kalMemCopy(pucIEPtr,
					   (PUINT_8) (((ULONG) pucIEPtr) + u4IELength),
                        u4IEMoveLength);

                prAdapter->rWlanInfo.u4ScanIEBufferUsage -= u4IELength;

				
				for (j = 0; j < prAdapter->rWlanInfo.u4ScanResultNum; j++) {
					if (prAdapter->rWlanInfo.apucScanResultIEs[j] > pucIEPtr) {
                        prAdapter->rWlanInfo.apucScanResultIEs[j] =
						    (PUINT_8) ((ULONG)
							       (prAdapter->rWlanInfo.
								apucScanResultIEs[j]) - u4IELength);
                    }
                }
            }
        }

        i++;
    }

    return;
}


#if CFG_TEST_WIFI_DIRECT_GO
VOID wlanEnableP2pFunction(IN P_ADAPTER_T prAdapter)
{
#if 0
	P_MSG_P2P_FUNCTION_SWITCH_T prMsgFuncSwitch = (P_MSG_P2P_FUNCTION_SWITCH_T) NULL;

	prMsgFuncSwitch =
	    (P_MSG_P2P_FUNCTION_SWITCH_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
						      sizeof(MSG_P2P_FUNCTION_SWITCH_T));
    if (!prMsgFuncSwitch) {
        ASSERT(FALSE);
        return;
    }


    prMsgFuncSwitch->rMsgHdr.eMsgId = MID_MNY_P2P_FUN_SWITCH;
    prMsgFuncSwitch->fgIsFuncOn = TRUE;


	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prMsgFuncSwitch, MSG_SEND_METHOD_BUF);
#endif
    return;
}

VOID wlanEnableATGO(IN P_ADAPTER_T prAdapter)
{

	P_MSG_P2P_CONNECTION_REQUEST_T prMsgConnReq = (P_MSG_P2P_CONNECTION_REQUEST_T) NULL;
	UINT_8 aucTargetDeviceID[MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	prMsgConnReq =
	    (P_MSG_P2P_CONNECTION_REQUEST_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
							 sizeof(MSG_P2P_CONNECTION_REQUEST_T));
    if (!prMsgConnReq) {
        ASSERT(FALSE);
        return;
    }

    prMsgConnReq->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_REQ;

    
    COPY_MAC_ADDR(prMsgConnReq->aucDeviceID, aucTargetDeviceID);
    prMsgConnReq->fgIsTobeGO = TRUE;
    prMsgConnReq->fgIsPersistentGroup = FALSE;

    

	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prMsgConnReq, MSG_SEND_METHOD_BUF);

    return;
}
#endif


WLAN_STATUS wlanQueryNicCapability(IN P_ADAPTER_T prAdapter)
{
    UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;
    UINT_8 ucCmdSeqNum;
    P_CMD_INFO_T prCmdInfo;
    P_WIFI_CMD_T prWifiCmd;
    UINT_32 u4RxPktLength;
    UINT_8 aucBuffer[sizeof(WIFI_EVENT_T) + sizeof(EVENT_NIC_CAPABILITY_T)];
    P_HW_MAC_RX_DESC_T prRxStatus;
    P_WIFI_EVENT_T prEvent;
    P_EVENT_NIC_CAPABILITY_T prEventNicCapability;

    ASSERT(prAdapter);

    DEBUGFUNC("wlanQueryNicCapability");

	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, CMD_HDR_SIZE + sizeof(EVENT_NIC_CAPABILITY_T));
    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }
	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
    prCmdInfo->eCmdType = COMMAND_TYPE_GENERAL_IOCTL;
    prCmdInfo->u2InfoBufLen = CMD_HDR_SIZE + sizeof(EVENT_NIC_CAPABILITY_T);
    prCmdInfo->pfCmdDoneHandler = NULL;
    prCmdInfo->fgIsOid = FALSE;
    prCmdInfo->ucCID = CMD_ID_GET_NIC_CAPABILITY;
    prCmdInfo->fgSetQuery = FALSE;
    prCmdInfo->fgNeedResp = TRUE;
    prCmdInfo->fgDriverDomainMCR = FALSE;
    prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
    prCmdInfo->u4SetInfoLen = 0;

	
	prWifiCmd = (P_WIFI_CMD_T) (prCmdInfo->pucInfoBuffer);
    prWifiCmd->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prWifiCmd->u2PQ_ID = CMD_PQ_ID;
    prWifiCmd->ucPktTypeID = CMD_PACKET_TYPE_ID;
    prWifiCmd->ucCID = prCmdInfo->ucCID;
    prWifiCmd->ucSetQuery = prCmdInfo->fgSetQuery;
    prWifiCmd->ucSeqNum = prCmdInfo->ucCmdSeqNum;

    wlanSendCommand(prAdapter, prCmdInfo);
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

	if (nicRxWaitResponse(prAdapter,
                1,
                aucBuffer,
                sizeof(WIFI_EVENT_T) + sizeof(EVENT_NIC_CAPABILITY_T),
                &u4RxPktLength) != WLAN_STATUS_SUCCESS) {
        return WLAN_STATUS_FAILURE;
    }
	
	prRxStatus = (P_HW_MAC_RX_DESC_T) aucBuffer;
	if (prRxStatus->u2PktTYpe != RXM_RXD_PKT_TYPE_SW_EVENT) {
        return WLAN_STATUS_FAILURE;
    }

	prEvent = (P_WIFI_EVENT_T) aucBuffer;
	if (prEvent->ucEID != EVENT_ID_NIC_CAPABILITY) {
        return WLAN_STATUS_FAILURE;
    }

	prEventNicCapability = (P_EVENT_NIC_CAPABILITY_T) (prEvent->aucBuffer);

    prAdapter->rVerInfo.u2FwProductID     = prEventNicCapability->u2ProductID;
    prAdapter->rVerInfo.u2FwOwnVersion    = prEventNicCapability->u2FwVersion;
    prAdapter->rVerInfo.u2FwPeerVersion   = prEventNicCapability->u2DriverVersion;
	prAdapter->fgIsHw5GBandDisabled = (BOOLEAN) prEventNicCapability->ucHw5GBandDisabled;
	prAdapter->fgIsEepromUsed = (BOOLEAN) prEventNicCapability->ucEepromUsed;
    prAdapter->fgIsEmbbededMacAddrValid   = (BOOLEAN)
        (!IS_BMCAST_MAC_ADDR(prEventNicCapability->aucMacAddr) &&
         !EQUAL_MAC_ADDR(aucZeroMacAddr, prEventNicCapability->aucMacAddr));

    COPY_MAC_ADDR(prAdapter->rWifiVar.aucPermanentAddress, prEventNicCapability->aucMacAddr);
    COPY_MAC_ADDR(prAdapter->rWifiVar.aucMacAddress, prEventNicCapability->aucMacAddr);

    prAdapter->u4FwCompileFlag0 = prEventNicCapability->u4CompileFlag0;
    prAdapter->u4FwCompileFlag1 = prEventNicCapability->u4CompileFlag1;
    prAdapter->u4FwFeatureFlag0 = prEventNicCapability->u4FeatureFlag0;
    prAdapter->u4FwFeatureFlag1 = prEventNicCapability->u4FeatureFlag1;
 
#if CFG_ENABLE_CAL_LOG
    DBGLOG(INIT, INFO, (" RF CAL FAIL  = (%d),BB CAL FAIL  = (%d)\n",
			    prEventNicCapability->ucRfCalFail, prEventNicCapability->ucBbCalFail));
#endif

    DBGLOG(INIT, INFO, ("FW Ver DEC[%u.%u] HEX[%x.%x], Driver Ver[%u.%u]\n", 
        (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
        (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
        (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
        (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
        (prAdapter->rVerInfo.u2FwPeerVersion >> 8), 
        (prAdapter->rVerInfo.u2FwPeerVersion & BITS(0, 7))));

    return WLAN_STATUS_SUCCESS;
}

#ifdef MT6628
static INT_32 wlanChangeCodeWord(INT_32 au4Input)
{

    UINT_16     i;
#if TXPWR_USE_PDSLOPE
    CODE_MAPPING_T arCodeTable[] = {
        {0X100,    -40},
        {0X104,    -35},
        {0X128,    -30},
        {0X14C,    -25},
        {0X170,    -20},
        {0X194,    -15},
        {0X1B8,    -10},
		{0X1DC, -5},
		{0, 0},
		{0X24, 5},
		{0X48, 10},
		{0X6C, 15},
		{0X90, 20},
		{0XB4, 25},
		{0XD8, 30},
		{0XFC, 35},
		{0XFF, 40},

    };
#else
    CODE_MAPPING_T arCodeTable[] = {
        {0X100,    0x80},
        {0X104,    0x80},
        {0X128,    0x80},
        {0X14C,    0x80},
        {0X170,    0x80},
        {0X194,    0x94},
        {0X1B8,    0XB8},
        {0X1DC,    0xDC},
		{0, 0},
		{0X24, 0x24},
		{0X48, 0x48},
		{0X6C, 0x6c},
		{0X90, 0x7F},
		{0XB4, 0x7F},
		{0XD8, 0x7F},
		{0XFC, 0x7F},
		{0XFF, 0x7F},

    };
#endif

    for (i = 0; i < sizeof(arCodeTable) / sizeof(CODE_MAPPING_T); i++) {

		if (arCodeTable[i].u4RegisterValue == au4Input) {
			return arCodeTable[i].u4TxpowerOffset;
        }
    }


    return 0;
}
#endif
#if TXPWR_USE_PDSLOPE

WLAN_STATUS wlanQueryPdMcr(IN P_ADAPTER_T prAdapter, P_PARAM_MCR_RW_STRUC_T prMcrRdInfo)
{
    UINT_8 ucCmdSeqNum;
    P_CMD_INFO_T prCmdInfo;
    P_WIFI_CMD_T prWifiCmd;
    UINT_32 u4RxPktLength;
    UINT_8 aucBuffer[sizeof(WIFI_EVENT_T) + sizeof(CMD_ACCESS_REG)];
    P_HW_MAC_RX_DESC_T prRxStatus;
    P_WIFI_EVENT_T prEvent;
    P_CMD_ACCESS_REG prCmdMcrQuery;
    ASSERT(prAdapter);


	
    prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, CMD_HDR_SIZE + sizeof(CMD_ACCESS_REG));

    if (!prCmdInfo) {
        DBGLOG(INIT, ERROR, ("Allocate CMD_INFO_T ==> FAILED.\n"));
        return WLAN_STATUS_FAILURE;
    }
	
    ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	
    prCmdInfo->eCmdType = COMMAND_TYPE_GENERAL_IOCTL;
	prCmdInfo->u2InfoBufLen = (UINT_16) (CMD_HDR_SIZE + sizeof(CMD_ACCESS_REG));
    prCmdInfo->pfCmdDoneHandler = NULL;
    prCmdInfo->pfCmdTimeoutHandler = nicOidCmdTimeoutCommon;
    prCmdInfo->fgIsOid = FALSE;
    prCmdInfo->ucCID = CMD_ID_ACCESS_REG;
    prCmdInfo->fgSetQuery = FALSE;
    prCmdInfo->fgNeedResp = TRUE;
    prCmdInfo->fgDriverDomainMCR = FALSE;
    prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
    prCmdInfo->u4SetInfoLen = sizeof(CMD_ACCESS_REG);

	
	prWifiCmd = (P_WIFI_CMD_T) (prCmdInfo->pucInfoBuffer);
    prWifiCmd->u2TxByteCount = prCmdInfo->u2InfoBufLen;
    prWifiCmd->u2PQ_ID = CMD_PQ_ID;
    prWifiCmd->ucPktTypeID = CMD_PACKET_TYPE_ID;
    prWifiCmd->ucCID = prCmdInfo->ucCID;
    prWifiCmd->ucSetQuery = prCmdInfo->fgSetQuery;
    prWifiCmd->ucSeqNum = prCmdInfo->ucCmdSeqNum;
    kalMemCopy(prWifiCmd->aucBuffer, prMcrRdInfo, sizeof(CMD_ACCESS_REG));

    wlanSendCommand(prAdapter, prCmdInfo);
    cmdBufFreeCmdInfo(prAdapter, prCmdInfo);

	if (nicRxWaitResponse(prAdapter,
                1,
                aucBuffer,
                sizeof(WIFI_EVENT_T) + sizeof(CMD_ACCESS_REG),
                &u4RxPktLength) != WLAN_STATUS_SUCCESS) {
        return WLAN_STATUS_FAILURE;
    }
	
	prRxStatus = (P_HW_MAC_RX_DESC_T) aucBuffer;
	if (prRxStatus->u2PktTYpe != RXM_RXD_PKT_TYPE_SW_EVENT) {
        return WLAN_STATUS_FAILURE;
    }


	prEvent = (P_WIFI_EVENT_T) aucBuffer;

	if (prEvent->ucEID != EVENT_ID_ACCESS_REG) {
        return WLAN_STATUS_FAILURE;
    }

	prCmdMcrQuery = (P_CMD_ACCESS_REG) (prEvent->aucBuffer);
    prMcrRdInfo->u4McrOffset = prCmdMcrQuery->u4Address;
    prMcrRdInfo->u4McrData = prCmdMcrQuery->u4Data;

    return WLAN_STATUS_SUCCESS;
}

static INT_32 wlanIntRound(INT_32 au4Input)
{


	if (au4Input >= 0) {
		if ((au4Input % 10) == 5) {
            au4Input = au4Input + 5;
            return au4Input;
        }
    }

	if (au4Input < 0) {
		if ((au4Input % 10) == -5) {
            au4Input = au4Input - 5;
            return au4Input;
        }
    }

    return au4Input;
}

static INT_32 wlanCal6628EfuseForm(IN P_ADAPTER_T prAdapter, INT_32 au4Input)
{

    PARAM_MCR_RW_STRUC_T rMcrRdInfo;
	INT_32 au4PdSlope, au4TxPwrOffset, au4TxPwrOffset_Round;
    INT_8          auTxPwrOffset_Round;

    rMcrRdInfo.u4McrOffset = 0x60205c68;
    rMcrRdInfo.u4McrData = 0;
    au4TxPwrOffset = au4Input;
	wlanQueryPdMcr(prAdapter, &rMcrRdInfo);

	au4PdSlope = (rMcrRdInfo.u4McrData) & BITS(0, 6);
	au4TxPwrOffset_Round = wlanIntRound((au4TxPwrOffset * au4PdSlope)) / 10;

    au4TxPwrOffset_Round = -au4TxPwrOffset_Round;

	if (au4TxPwrOffset_Round < -128) {
        au4TxPwrOffset_Round = 128;
	} else if (au4TxPwrOffset_Round < 0) {
        au4TxPwrOffset_Round += 256;
	} else if (au4TxPwrOffset_Round > 127) {
        au4TxPwrOffset_Round = 127;
    }

	auTxPwrOffset_Round = (UINT_8) au4TxPwrOffset_Round;

    return au4TxPwrOffset_Round;
}

#endif

#ifdef MT6628
static VOID wlanChangeNvram6620to6628(PUINT_8 pucEFUSE)
{


#define EFUSE_CH_OFFSET1_L_MASK_6620         BITS(0, 8)
#define EFUSE_CH_OFFSET1_L_SHIFT_6620        0
#define EFUSE_CH_OFFSET1_M_MASK_6620         BITS(9, 17)
#define EFUSE_CH_OFFSET1_M_SHIFT_6620        9
#define EFUSE_CH_OFFSET1_H_MASK_6620         BITS(18, 26)
#define EFUSE_CH_OFFSET1_H_SHIFT_6620        18
#define EFUSE_CH_OFFSET1_VLD_MASK_6620       BIT(27)
#define EFUSE_CH_OFFSET1_VLD_SHIFT_6620      27

#define EFUSE_CH_OFFSET1_L_MASK_5931         BITS(0, 7)
#define EFUSE_CH_OFFSET1_L_SHIFT_5931        0
#define EFUSE_CH_OFFSET1_M_MASK_5931         BITS(8, 15)
#define EFUSE_CH_OFFSET1_M_SHIFT_5931        8
#define EFUSE_CH_OFFSET1_H_MASK_5931         BITS(16, 23)
#define EFUSE_CH_OFFSET1_H_SHIFT_5931        16
#define EFUSE_CH_OFFSET1_VLD_MASK_5931       BIT(24)
#define EFUSE_CH_OFFSET1_VLD_SHIFT_5931      24
#define EFUSE_ALL_CH_OFFSET1_MASK_5931       BITS(25, 27)
#define EFUSE_ALL_CH_OFFSET1_SHIFT_5931      25




    INT_32         au4ChOffset;
	INT_16 au2ChOffsetL, au2ChOffsetM, au2ChOffsetH;


	au4ChOffset = *(UINT_32 *) (pucEFUSE + 72);

	if ((au4ChOffset & EFUSE_CH_OFFSET1_VLD_MASK_6620) && ((*(UINT_32 *) (pucEFUSE + 28)) == 0)) {


        au2ChOffsetL = ((au4ChOffset & EFUSE_CH_OFFSET1_L_MASK_6620) >>
            EFUSE_CH_OFFSET1_L_SHIFT_6620);

        au2ChOffsetM = ((au4ChOffset & EFUSE_CH_OFFSET1_M_MASK_6620) >>
            EFUSE_CH_OFFSET1_M_SHIFT_6620);

        au2ChOffsetH = ((au4ChOffset & EFUSE_CH_OFFSET1_H_MASK_6620) >>
            EFUSE_CH_OFFSET1_H_SHIFT_6620);

	    au2ChOffsetL = wlanChangeCodeWord(au2ChOffsetL);
	    au2ChOffsetM = wlanChangeCodeWord(au2ChOffsetM);
	    au2ChOffsetH = wlanChangeCodeWord(au2ChOffsetH);

	    au4ChOffset =  0;
		au4ChOffset |= *(UINT_32 *) (pucEFUSE + 72)
		    >> (EFUSE_CH_OFFSET1_VLD_SHIFT_6620 -
			EFUSE_CH_OFFSET1_VLD_SHIFT_5931) & EFUSE_CH_OFFSET1_VLD_MASK_5931;



		au4ChOffset |=
		    ((((UINT_32) au2ChOffsetL) << EFUSE_CH_OFFSET1_L_SHIFT_5931) &
		     EFUSE_CH_OFFSET1_L_MASK_5931);
		au4ChOffset |=
		    ((((UINT_32) au2ChOffsetM) << EFUSE_CH_OFFSET1_M_SHIFT_5931) &
		     EFUSE_CH_OFFSET1_M_MASK_5931);
		au4ChOffset |=
		    ((((UINT_32) au2ChOffsetH) << EFUSE_CH_OFFSET1_H_SHIFT_5931) &
		     EFUSE_CH_OFFSET1_H_MASK_5931);

		*((INT_32 *) ((pucEFUSE + 28))) = au4ChOffset;



    }

}
#endif

#if CFG_SUPPORT_NVRAM_5G
WLAN_STATUS wlanLoadManufactureData_5G(IN P_ADAPTER_T prAdapter, IN P_REG_INFO_T prRegInfo)
{

    
    P_BANDEDGE_5G_T pr5GBandEdge;

    ASSERT(prAdapter);

    pr5GBandEdge = &prRegInfo->prOldEfuseMapping->r5GBandEdgePwr;

     
	if (pr5GBandEdge->uc5GBandEdgePwrUsed != 0) {
        CMD_EDGE_TXPWR_LIMIT_T rCmdEdgeTxPwrLimit;

		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrCCK = 0;
		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrOFDM20 = pr5GBandEdge->c5GBandEdgeMaxPwrOFDM20;
		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrOFDM40 = pr5GBandEdge->c5GBandEdgeMaxPwrOFDM40;
		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrOFDM80 = pr5GBandEdge->c5GBandEdgeMaxPwrOFDM80;

        wlanSendSetQueryCmd(prAdapter,
                CMD_ID_SET_EDGE_TXPWR_LIMIT_5G,
                TRUE,
                FALSE,
                FALSE,
                NULL,
                NULL,
                sizeof(CMD_EDGE_TXPWR_LIMIT_T),
				    (PUINT_8) &rCmdEdgeTxPwrLimit, NULL, 0);

		
    }

    kalPrint("wlanLoadManufactureData_5G");
    

	
	if (prRegInfo->prOldEfuseMapping->uc5GChannelOffsetVaild) {
        CMD_POWER_OFFSET_T rCmdPowerOffset;
        UINT_8      i;

        rCmdPowerOffset.ucBand = BAND_5G;
		for (i = 0; i < MAX_SUBBAND_NUM_5G; i++) {
			rCmdPowerOffset.ucSubBandOffset[i] =
			    prRegInfo->prOldEfuseMapping->auc5GChOffset[i];
        }

        wlanSendSetQueryCmd(prAdapter,
                CMD_ID_SET_CHANNEL_PWR_OFFSET,
                TRUE,
                FALSE,
                FALSE,
                NULL,
                NULL,
				    sizeof(rCmdPowerOffset), (PUINT_8) &rCmdPowerOffset, NULL, 0);
		
    }

	
    if (prRegInfo->prOldEfuseMapping->uc11AcTxPwrValid) {
        
        CMD_TX_AC_PWR_T     rCmdAcPwr;
		kalMemCopy(&rCmdAcPwr.rAcPwr, &prRegInfo->prOldEfuseMapping->r11AcTxPwr,
			   sizeof(AC_PWR_SETTING_STRUCT));
        rCmdAcPwr.ucBand = BAND_5G;

         wlanSendSetQueryCmd(prAdapter,
            CMD_ID_SET_80211AC_TX_PWR,
            TRUE,
            FALSE,
            FALSE,
            NULL,
				    NULL, sizeof(CMD_TX_AC_PWR_T), (PUINT_8) &rCmdAcPwr, NULL, 0);
		
    }
    

    return WLAN_STATUS_SUCCESS;
}
#endif
WLAN_STATUS wlanLoadManufactureData(IN P_ADAPTER_T prAdapter, IN P_REG_INFO_T prRegInfo)
{
#if CFG_SUPPORT_RDD_TEST_MODE
    CMD_RDD_CH_T rRddParam;
#endif
    CMD_NVRAM_SETTING_T rCmdNvramSettings;

    ASSERT(prAdapter);

    
    kalGetConfigurationVersion(prAdapter->prGlueInfo,
            &(prAdapter->rVerInfo.u2Part1CfgOwnVersion),
            &(prAdapter->rVerInfo.u2Part1CfgPeerVersion),
            &(prAdapter->rVerInfo.u2Part2CfgOwnVersion),
            &(prAdapter->rVerInfo.u2Part2CfgPeerVersion));

#if (CFG_SW_NVRAM_VERSION_CHECK == 1)
	if (CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part1CfgPeerVersion
            || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part2CfgPeerVersion
            || prAdapter->rVerInfo.u2Part1CfgOwnVersion < CFG_DRV_PEER_VERSION
            || prAdapter->rVerInfo.u2Part2CfgOwnVersion < CFG_DRV_PEER_VERSION) {
        return WLAN_STATUS_FAILURE;
    }
#endif

	
	if (prAdapter->rVerInfo.u2Part1CfgOwnVersion == 0x0001) {
        prRegInfo->ucTxPwrValid = 1;
	} else {
        
		if (prRegInfo->ucTxPwrValid != 0) {
			
            
			nicUpdateTxPower(prAdapter, (P_CMD_TX_PWR_T) (&(prRegInfo->rTxPwr)));
        }
    }

    
	if (prRegInfo->ucEnable5GBand) {
#if CFG_SUPPORT_NVRAM_5G
		wlanLoadManufactureData_5G(prAdapter, prRegInfo);
#endif
		
		if (prAdapter->fgIsHw5GBandDisabled || prRegInfo->ucSupport5GBand == 0) {
            prAdapter->fgEnable5GBand = FALSE;
		} else {
            prAdapter->fgEnable5GBand = TRUE;
        }
	} else {
        prAdapter->fgEnable5GBand = FALSE;
    }

    
#if  defined(MT6628)
    wlanChangeNvram6620to6628(prRegInfo->aucEFUSE);
#endif

#if CFG_SUPPORT_NVRAM_5G
    
	if (prRegInfo->prOldEfuseMapping) {
		
		if (prRegInfo->prOldEfuseMapping->ucChannelOffsetVaild) {
            CMD_POWER_OFFSET_T rCmdPowerOffset;
            UINT_8      i;
    
            rCmdPowerOffset.ucBand = BAND_2G4;
			for (i = 0; i < 3; i++) {
				rCmdPowerOffset.ucSubBandOffset[i] =
				    prRegInfo->prOldEfuseMapping->aucChOffset[i];
            }
			rCmdPowerOffset.ucSubBandOffset[i] =
			    prRegInfo->prOldEfuseMapping->acAllChannelOffset;
    
            wlanSendSetQueryCmd(prAdapter,
                    CMD_ID_SET_CHANNEL_PWR_OFFSET,
                    TRUE,
                    FALSE,
                    FALSE,
                    NULL,
                    NULL,
                    sizeof(rCmdPowerOffset),
					    (PUINT_8) &rCmdPowerOffset, NULL, 0);
			
        }
    }
#else

    wlanSendSetQueryCmd(prAdapter,
            CMD_ID_SET_PHY_PARAM,
            TRUE,
            FALSE,
            FALSE,
            NULL,
            NULL,
			    sizeof(CMD_PHY_PARAM_T), (PUINT_8) (prRegInfo->aucEFUSE), NULL, 0);


#endif
	
    if (prRegInfo->ucRssiPathCompasationUsed) {
       CMD_RSSI_PATH_COMPASATION_T rCmdRssiPathCompasation; 

		rCmdRssiPathCompasation.c2GRssiCompensation =
		    prRegInfo->rRssiPathCompasation.c2GRssiCompensation;
		rCmdRssiPathCompasation.c5GRssiCompensation =
		    prRegInfo->rRssiPathCompasation.c5GRssiCompensation;

       wlanSendSetQueryCmd(prAdapter,
                CMD_ID_SET_PATH_COMPASATION,
                TRUE,
                FALSE,
                FALSE,
                NULL,
                NULL,
                sizeof(rCmdRssiPathCompasation),
				    (PUINT_8) &rCmdRssiPathCompasation, NULL, 0);
    }
#if CFG_SUPPORT_RDD_TEST_MODE
    rRddParam.ucRddTestMode = (UINT_8) prRegInfo->u4RddTestMode;
    rRddParam.ucRddShutCh = (UINT_8) prRegInfo->u4RddShutFreq;
    rRddParam.ucRddStartCh = (UINT_8) nicFreq2ChannelNum(prRegInfo->u4RddStartFreq);
    rRddParam.ucRddStopCh = (UINT_8) nicFreq2ChannelNum(prRegInfo->u4RddStopFreq);
    rRddParam.ucRddDfs = (UINT_8) prRegInfo->u4RddDfs;
    prAdapter->ucRddStatus = 0;
	nicUpdateRddTestMode(prAdapter, (P_CMD_RDD_CH_T) (&rRddParam));
#endif

    
    prAdapter->rWifiVar.rConnSettings.u2CountryCode =
        (((UINT_16) prRegInfo->au2CountryCode[0]) << 8) |
	    (((UINT_16) prRegInfo->au2CountryCode[1]) & BITS(0, 7));

#if 0 
    prAdapter->rWifiVar.rConnSettings.uc2G4BandwidthMode =
            prRegInfo->uc2G4BwFixed20M ? CONFIG_BW_20M : CONFIG_BW_20_40M;
    prAdapter->rWifiVar.rConnSettings.uc5GBandwidthMode =
            prRegInfo->uc5GBwFixed20M ? CONFIG_BW_20M : CONFIG_BW_20_40M;
#endif

    
    rlmDomainSendCmd(prAdapter, FALSE);

    
	if (prRegInfo->fg2G4BandEdgePwrUsed) {
        CMD_EDGE_TXPWR_LIMIT_T rCmdEdgeTxPwrLimit;

		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrCCK = prRegInfo->cBandEdgeMaxPwrCCK;
		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrOFDM20 = prRegInfo->cBandEdgeMaxPwrOFDM20;
		rCmdEdgeTxPwrLimit.cBandEdgeMaxPwrOFDM40 = prRegInfo->cBandEdgeMaxPwrOFDM40;

        wlanSendSetQueryCmd(prAdapter,
                CMD_ID_SET_EDGE_TXPWR_LIMIT,
                TRUE,
                FALSE,
                FALSE,
                NULL,
                NULL,
                sizeof(CMD_EDGE_TXPWR_LIMIT_T),
				    (PUINT_8) &rCmdEdgeTxPwrLimit, NULL, 0);
    }
	
    if (prRegInfo->prOldEfuseMapping->uc11AcTxPwrValid2G) {
        
        CMD_TX_AC_PWR_T     rCmdAcPwr;
		kalMemCopy(&rCmdAcPwr.rAcPwr, &prRegInfo->prOldEfuseMapping->r11AcTxPwr2G,
			   sizeof(AC_PWR_SETTING_STRUCT));
        rCmdAcPwr.ucBand = BAND_2G4;

         wlanSendSetQueryCmd(prAdapter,
            CMD_ID_SET_80211AC_TX_PWR,
            TRUE,
            FALSE,
            FALSE,
            NULL,
				    NULL, sizeof(CMD_TX_AC_PWR_T), (PUINT_8) &rCmdAcPwr, NULL, 0);
		
    }
    

    
	kalMemCopy(&rCmdNvramSettings.rNvramSettings,
		   &prRegInfo->prNvramSettings->u2Part1OwnVersion, sizeof(WIFI_CFG_PARAM_STRUCT));
    ASSERT(sizeof(WIFI_CFG_PARAM_STRUCT) == 512);
            wlanSendSetQueryCmd(prAdapter,
                    CMD_ID_SET_NVRAM_SETTINGS,
                    TRUE,
                    FALSE,
                    FALSE,
                    NULL,
                    NULL,
			    sizeof(rCmdNvramSettings), (PUINT_8) &rCmdNvramSettings, NULL, 0);

    return WLAN_STATUS_SUCCESS;
}


BOOLEAN wlanResetMediaStreamMode(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

	if (prAdapter->rWlanInfo.eLinkAttr.ucMediaStreamMode != 0) {
        prAdapter->rWlanInfo.eLinkAttr.ucMediaStreamMode = 0;

        return TRUE;
	} else {
        return FALSE;
    }
}


WLAN_STATUS wlanTimerTimeoutCheck(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    cnmTimerDoTimeOutCheck(prAdapter);

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanProcessMboxMessage(IN P_ADAPTER_T prAdapter)
{
    UINT_32 i;

    ASSERT(prAdapter);

	for (i = 0; i < MBOX_ID_TOTAL_NUM; i++) {
		mboxRcvAllMsg(prAdapter, (ENUM_MBOX_ID_T) i);
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanEnqueueTxPacket(IN P_ADAPTER_T prAdapter, IN P_NATIVE_PACKET prNativePacket)
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prMsduInfo;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);

    prTxCtrl = &prAdapter->rTxCtrl;

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
    QUEUE_REMOVE_HEAD(&prTxCtrl->rFreeMsduInfoList, prMsduInfo, P_MSDU_INFO_T);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);

	if (prMsduInfo == NULL) {
        return WLAN_STATUS_RESOURCES;
	} else {
        prMsduInfo->eSrc = TX_PACKET_OS;

		if (nicTxFillMsduInfo(prAdapter, prMsduInfo, prNativePacket) == FALSE) {

            kalSendComplete(prAdapter->prGlueInfo,
					prNativePacket, WLAN_STATUS_INVALID_PACKET);

            nicTxReturnMsduInfo(prAdapter, prMsduInfo);

            return WLAN_STATUS_INVALID_PACKET;
		} else {
		    
            wlanTxProfilingTagMsdu(prAdapter, prMsduInfo, TX_PROF_TAG_DRV_ENQUE);
        
			
            nicTxEnqueueMsdu(prAdapter, prMsduInfo);

            return WLAN_STATUS_SUCCESS;
        }
    }
}


WLAN_STATUS wlanFlushTxPendingPackets(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return nicTxFlush(prAdapter);
}


WLAN_STATUS wlanTxPendingPackets(IN P_ADAPTER_T prAdapter, IN OUT PBOOLEAN pfgHwAccess)
{
    P_TX_CTRL_T prTxCtrl;
    P_MSDU_INFO_T prMsduInfo;

    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

#if !CFG_SUPPORT_MULTITHREAD 
    ASSERT(pfgHwAccess);
#endif

	
#if CFG_SUPPORT_MULTITHREAD 
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
    prMsduInfo = qmDequeueTxPacketsMthread(prAdapter, &prTxCtrl->rTc);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
#else
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
    prMsduInfo = qmDequeueTxPackets(prAdapter, &prTxCtrl->rTc);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
#endif
	if (prMsduInfo != NULL) {
		if (kalIsCardRemoved(prAdapter->prGlueInfo) == FALSE) {
#if !CFG_SUPPORT_MULTITHREAD            
            
			if (*pfgHwAccess == FALSE) {
                *pfgHwAccess = TRUE;

                wlanAcquirePowerControl(prAdapter);
            }
#endif
			
#if CFG_SUPPORT_MULTITHREAD
            nicTxMsduInfoListMthread(prAdapter, prMsduInfo);
#else
            nicTxMsduInfoList(prAdapter, prMsduInfo);
#endif
			
            nicTxAdjustTcq(prAdapter);
		} else {
            wlanProcessQueuedMsduInfo(prAdapter, prMsduInfo);
        }
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanAcquirePowerControl(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

	

    ACQUIRE_POWER_CONTROL_FROM_PM(prAdapter);

    
	if (prAdapter->fgWiFiInSleepyState == TRUE) {
        prAdapter->fgWiFiInSleepyState = FALSE;
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS wlanReleasePowerControl(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

	

    RECLAIM_POWER_CONTROL_TO_PM(prAdapter, FALSE);

    return WLAN_STATUS_SUCCESS;
}


UINT_32 wlanGetTxPendingFrameCount(IN P_ADAPTER_T prAdapter)
{
    P_TX_CTRL_T prTxCtrl;
    UINT_32 u4Num;

    ASSERT(prAdapter);
    prTxCtrl = &prAdapter->rTxCtrl;

	u4Num =
	    kalGetTxPendingFrameCount(prAdapter->prGlueInfo) +
	    (UINT_32) (prTxCtrl->i4PendingFwdFrameCount);

    return u4Num;
}


ENUM_ACPI_STATE_T wlanGetAcpiState(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return prAdapter->rAcpiState;
}


VOID wlanSetAcpiState(IN P_ADAPTER_T prAdapter, IN ENUM_ACPI_STATE_T ePowerState)
{
    ASSERT(prAdapter);
    ASSERT(ePowerState <= ACPI_STATE_D3);

    prAdapter->rAcpiState = ePowerState;

    return;
}


UINT_8 wlanGetEcoVersion(IN P_ADAPTER_T prAdapter)
{
    UINT_8 ucEcoVersion;
    
    ASSERT(prAdapter);
    
#if CFG_MULTI_ECOVER_SUPPORT
    ucEcoVersion = mtk_wcn_wmt_hwver_get() + 1;
	
    return ucEcoVersion;
#else
	if (nicVerifyChipID(prAdapter) == TRUE) {
		return prAdapter->ucRevID + 1;
	} else {
        return 0;
    }
#endif    
}


VOID wlanDefTxPowerCfg(IN P_ADAPTER_T prAdapter)
{
    UINT_8 i;
    P_GLUE_INFO_T       prGlueInfo = prAdapter->prGlueInfo;
    P_SET_TXPWR_CTRL_T  prTxpwr;

    ASSERT(prGlueInfo);

    prTxpwr = &prGlueInfo->rTxPwr;

    prTxpwr->c2GLegacyStaPwrOffset = 0;
    prTxpwr->c2GHotspotPwrOffset = 0;
    prTxpwr->c2GP2pPwrOffset = 0;
    prTxpwr->c2GBowPwrOffset = 0;
    prTxpwr->c5GLegacyStaPwrOffset = 0;
    prTxpwr->c5GHotspotPwrOffset = 0;
    prTxpwr->c5GP2pPwrOffset = 0;
    prTxpwr->c5GBowPwrOffset = 0;
    prTxpwr->ucConcurrencePolicy = 0;
	for (i = 0; i < 3; i++)
        prTxpwr->acReserved1[i] = 0;

	for (i = 0; i < 14; i++)
        prTxpwr->acTxPwrLimit2G[i] = 63;

	for (i = 0; i < 4; i++)
        prTxpwr->acTxPwrLimit5G[i] = 63;

	for (i = 0; i < 2; i++)
        prTxpwr->acReserved2[i] = 0;

}

VOID
wlanSetPreferBandByNetwork(IN P_ADAPTER_T prAdapter, IN ENUM_BAND_T eBand, IN UINT_8 ucBssIndex)
{
    ASSERT(prAdapter);
    ASSERT(eBand <= BAND_NUM);
    ASSERT(ucBssIndex <= MAX_BSS_INDEX);
    ASSERT(ucBssIndex <= BSS_INFO_NUM);

    
    prAdapter->aePreferBand[ucBssIndex] = eBand;

    
	if (eBand == BAND_2G4) {
        scanRemoveBssDescByBandAndNetwork(prAdapter, BAND_5G, ucBssIndex);
	} else if (eBand == BAND_5G) {
        scanRemoveBssDescByBandAndNetwork(prAdapter, BAND_2G4, ucBssIndex);
    }

    return;
}

UINT_8 wlanGetChannelNumberByNetwork(IN P_ADAPTER_T prAdapter, IN UINT_8 ucBssIndex)
{
    P_BSS_INFO_T prBssInfo;

    ASSERT(prAdapter);
    ASSERT(ucBssIndex <= MAX_BSS_INDEX);

    prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

    return prBssInfo->ucPrimaryChannel;
}


WLAN_STATUS wlanCheckSystemConfiguration(IN P_ADAPTER_T prAdapter)
{
#if (CFG_NVRAM_EXISTENCE_CHECK == 1) || (CFG_SW_NVRAM_VERSION_CHECK == 1)
    const UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;
    const UINT_8 aucBCAddr[] = BC_MAC_ADDR;
    BOOLEAN fgIsConfExist = TRUE;
    BOOLEAN fgGenErrMsg = FALSE;
    P_REG_INFO_T prRegInfo = NULL;
    P_WLAN_BEACON_FRAME_T prBeacon = NULL;
    P_IE_SSID_T prSsid = NULL;
    UINT_32 u4ErrCode = 0;
    UINT_8 aucErrMsg[32];
    PARAM_SSID_T rSsid;
    PARAM_802_11_CONFIG_T rConfiguration;
    PARAM_RATES_EX rSupportedRates;
#endif

    DEBUGFUNC("wlanCheckSystemConfiguration");

    ASSERT(prAdapter);

#if (CFG_NVRAM_EXISTENCE_CHECK == 1)
	if (kalIsConfigurationExist(prAdapter->prGlueInfo) == FALSE) {
        fgIsConfExist = FALSE;
        fgGenErrMsg = TRUE;
    }
#endif

#if (CFG_SW_NVRAM_VERSION_CHECK == 1)
    prRegInfo = kalGetConfiguration(prAdapter->prGlueInfo);

#if (CFG_SUPPORT_PWR_LIMIT_COUNTRY == 1)
	if (fgIsConfExist == TRUE && (CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part1CfgPeerVersion || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part2CfgPeerVersion || prAdapter->rVerInfo.u2Part1CfgOwnVersion < CFG_DRV_PEER_VERSION || prAdapter->rVerInfo.u2Part2CfgOwnVersion < CFG_DRV_PEER_VERSION	
            || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2FwPeerVersion
            || prAdapter->rVerInfo.u2FwOwnVersion < CFG_DRV_PEER_VERSION
            || (prAdapter->fgIsEmbbededMacAddrValid == FALSE &&
                (IS_BMCAST_MAC_ADDR(prRegInfo->aucMacAddr)
					   || EQUAL_MAC_ADDR(aucZeroMacAddr,
							     prRegInfo->aucMacAddr)))
            || prRegInfo->ucTxPwrValid == 0
            || prAdapter->fgIsPowerLimitTableValid == FALSE )) {
        fgGenErrMsg = TRUE;
    }
#else
	if (fgIsConfExist == TRUE && (CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part1CfgPeerVersion || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part2CfgPeerVersion || prAdapter->rVerInfo.u2Part1CfgOwnVersion < CFG_DRV_PEER_VERSION || prAdapter->rVerInfo.u2Part2CfgOwnVersion < CFG_DRV_PEER_VERSION	
            || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2FwPeerVersion
            || prAdapter->rVerInfo.u2FwOwnVersion < CFG_DRV_PEER_VERSION
            || (prAdapter->fgIsEmbbededMacAddrValid == FALSE &&
                (IS_BMCAST_MAC_ADDR(prRegInfo->aucMacAddr)
					   || EQUAL_MAC_ADDR(aucZeroMacAddr,
							     prRegInfo->aucMacAddr)))
            || prRegInfo->ucTxPwrValid == 0)) {
        fgGenErrMsg = TRUE;
    }
#endif

#endif

	if (fgGenErrMsg == TRUE) {
		prBeacon =
		    cnmMemAlloc(prAdapter, RAM_TYPE_BUF,
				sizeof(WLAN_BEACON_FRAME_T) + sizeof(IE_SSID_T));

		
        kalMemZero(prBeacon, sizeof(WLAN_BEACON_FRAME_T) + sizeof(IE_SSID_T));

		
        prBeacon->u2FrameCtrl = MAC_FRAME_BEACON;
        COPY_MAC_ADDR(prBeacon->aucDestAddr, aucBCAddr);
        COPY_MAC_ADDR(prBeacon->aucSrcAddr, aucZeroMacAddr);
        COPY_MAC_ADDR(prBeacon->aucBSSID, aucZeroMacAddr);
        prBeacon->u2BeaconInterval = 100;
        prBeacon->u2CapInfo = CAP_INFO_ESS;

		
		prSsid = (P_IE_SSID_T) (&prBeacon->aucInfoElem[0]);
        prSsid->ucId = ELEM_ID_SSID;

		
        rConfiguration.u4Length             = sizeof(PARAM_802_11_CONFIG_T);
        rConfiguration.u4BeaconPeriod       = 100;
        rConfiguration.u4ATIMWindow         = 1;
        rConfiguration.u4DSConfig           = 2412;
        rConfiguration.rFHConfig.u4Length   = sizeof(PARAM_802_11_CONFIG_FH_T);

		
        kalMemZero(rSupportedRates, sizeof(PARAM_RATES_EX));
    }
#if (CFG_NVRAM_EXISTENCE_CHECK == 1)
#define NVRAM_ERR_MSG "NVRAM WARNING: Err = 0x01"
	if (kalIsConfigurationExist(prAdapter->prGlueInfo) == FALSE) {
        COPY_SSID(prSsid->aucSSID,
			  prSsid->ucLength, NVRAM_ERR_MSG, (UINT_8) (strlen(NVRAM_ERR_MSG)));

        kalIndicateBssInfo(prAdapter->prGlueInfo,
				   (PUINT_8) prBeacon,
				   OFFSET_OF(WLAN_BEACON_FRAME_T,
					     aucInfoElem) + OFFSET_OF(IE_SSID_T,
								      aucSSID) + prSsid->ucLength,
				   1, 0);

        COPY_SSID(rSsid.aucSsid, rSsid.u4SsidLen, NVRAM_ERR_MSG, strlen(NVRAM_ERR_MSG));
        nicAddScanResult(prAdapter,
                prBeacon->aucBSSID,
                &rSsid,
                0,
                0,
                PARAM_NETWORK_TYPE_FH,
                &rConfiguration,
                NET_TYPE_INFRA,
                rSupportedRates,
				 OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem) + OFFSET_OF(IE_SSID_T,
											 aucSSID) +
				 prSsid->ucLength - WLAN_MAC_MGMT_HEADER_LEN,
				 (PUINT_8) ((ULONG) (prBeacon) + WLAN_MAC_MGMT_HEADER_LEN));
    }
#endif

#if (CFG_SW_NVRAM_VERSION_CHECK == 1)
#define VER_ERR_MSG     "NVRAM WARNING: Err = 0x%02X"
	if (fgIsConfExist == TRUE) {
		if ((CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part1CfgPeerVersion || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2Part2CfgPeerVersion || prAdapter->rVerInfo.u2Part1CfgOwnVersion < CFG_DRV_PEER_VERSION || prAdapter->rVerInfo.u2Part2CfgOwnVersion < CFG_DRV_PEER_VERSION	
                    || CFG_DRV_OWN_VERSION < prAdapter->rVerInfo.u2FwPeerVersion
                    || prAdapter->rVerInfo.u2FwOwnVersion < CFG_DRV_PEER_VERSION)) {
            u4ErrCode |= NVRAM_ERROR_VERSION_MISMATCH;
        }


		if (prRegInfo->ucTxPwrValid == 0) {
            u4ErrCode |= NVRAM_ERROR_INVALID_TXPWR;
        }

		if (prAdapter->fgIsEmbbededMacAddrValid == FALSE &&
		    (IS_BMCAST_MAC_ADDR(prRegInfo->aucMacAddr)
		     || EQUAL_MAC_ADDR(aucZeroMacAddr, prRegInfo->aucMacAddr))) {
            u4ErrCode |= NVRAM_ERROR_INVALID_MAC_ADDR;
        }
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
        if(prAdapter->fgIsPowerLimitTableValid == FALSE) {
			u4ErrCode |= NVRAM_POWER_LIMIT_TABLE_INVALID;
        }
#endif
        if(u4ErrCode != 0) {
            sprintf(aucErrMsg, VER_ERR_MSG, (unsigned int)u4ErrCode);
            COPY_SSID(prSsid->aucSSID,
				  prSsid->ucLength, aucErrMsg, (UINT_8) (strlen(aucErrMsg)));

            kalIndicateBssInfo(prAdapter->prGlueInfo,
					   (PUINT_8) prBeacon,
					   OFFSET_OF(WLAN_BEACON_FRAME_T,
						     aucInfoElem) + OFFSET_OF(IE_SSID_T,
									      aucSSID) +
					   prSsid->ucLength, 1, 0);

			COPY_SSID(rSsid.aucSsid, rSsid.u4SsidLen, NVRAM_ERR_MSG,
				  strlen(NVRAM_ERR_MSG));
			nicAddScanResult(prAdapter, prBeacon->aucBSSID, &rSsid, 0, 0,
					 PARAM_NETWORK_TYPE_FH, &rConfiguration, NET_TYPE_INFRA,
					 rSupportedRates, OFFSET_OF(WLAN_BEACON_FRAME_T,
								    aucInfoElem) +
					 OFFSET_OF(IE_SSID_T,
						   aucSSID) + prSsid->ucLength -
					 WLAN_MAC_MGMT_HEADER_LEN,
					 (PUINT_8) ((ULONG) (prBeacon) +
						    WLAN_MAC_MGMT_HEADER_LEN));
        }
    }
#endif

	if (fgGenErrMsg == TRUE) {
        cnmMemFree(prAdapter, prBeacon);
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS 
wlanoidQueryStaStatistics(IN P_ADAPTER_T prAdapter,
    IN PVOID        pvQueryBuffer,
			  IN UINT_32 u4QueryBufferLen, OUT PUINT_32 pu4QueryInfoLen)
{
	WLAN_STATUS rResult = WLAN_STATUS_FAILURE;
	P_STA_RECORD_T prStaRec, prTempStaRec;
	P_PARAM_GET_STA_STATISTICS prQueryStaStatistics;
    UINT_8 ucStaRecIdx;
    P_QUE_MGT_T prQM = &prAdapter->rQM;
    CMD_GET_STA_STATISTICS_T rQueryCmdStaStatistics;
    UINT_8 ucIdx;
	
	DEBUGFUNC("wlanoidQueryStaStatistics");
	do {
        ASSERT(pvQueryBuffer);

		
        if ((prAdapter == NULL) || (pu4QueryInfoLen == NULL)) {
            break;
        }

		if ((u4QueryBufferLen) && (pvQueryBuffer == NULL)) {
			break;
		}

		if (u4QueryBufferLen < sizeof(PARAM_GET_STA_STA_STATISTICS)) {
			*pu4QueryInfoLen = sizeof(PARAM_GET_STA_STA_STATISTICS);
			rResult = WLAN_STATUS_BUFFER_TOO_SHORT;
			break;
		}
			 		
		prQueryStaStatistics = (P_PARAM_GET_STA_STATISTICS) pvQueryBuffer;
        *pu4QueryInfoLen = sizeof(PARAM_GET_STA_STA_STATISTICS);

		
		for (ucIdx = TC0_INDEX; ucIdx <= TC3_INDEX; ucIdx++) {
			prQueryStaStatistics->au4TcAverageQueLen[ucIdx] =
			    prQM->au4AverageQueLen[ucIdx];
			prQueryStaStatistics->au4TcCurrentQueLen[ucIdx] =
			    prQM->au4CurrentTcResource[ucIdx];
        }

		
        prStaRec = NULL;

		
		for (ucStaRecIdx = 0; ucStaRecIdx < CFG_NUM_OF_STA_RECORD; ucStaRecIdx++) {
            prTempStaRec = &(prAdapter->arStaRec[ucStaRecIdx]);
			if (prTempStaRec->fgIsValid && prTempStaRec->fgIsInUse) {
				if (EQUAL_MAC_ADDR
				    (prTempStaRec->aucMacAddr, prQueryStaStatistics->aucMacAddr)) {
                    prStaRec = prTempStaRec;
                    break;
                }
            }
        }

		if (!prStaRec) {
            rResult = WLAN_STATUS_INVALID_DATA;
            break;
        }

        prQueryStaStatistics->u4Flag |= BIT(0);

#if CFG_ENABLE_PER_STA_STATISTICS
		
        prQueryStaStatistics->u4TxTotalCount = prStaRec->u4TotalTxPktsNumber;
        prQueryStaStatistics->u4RxTotalCount = prStaRec->u4TotalRxPktsNumber;
        prQueryStaStatistics->u4TxExceedThresholdCount = prStaRec->u4ThresholdCounter;
        prQueryStaStatistics->u4TxMaxTime = prStaRec->u4MaxTxPktsTime;
		if (prStaRec->u4TotalTxPktsNumber) {
			prQueryStaStatistics->u4TxAverageProcessTime =
			    (prStaRec->u4TotalTxPktsTime / prStaRec->u4TotalTxPktsNumber);
		} else {
            prQueryStaStatistics->u4TxAverageProcessTime = 0;
        }

		for (ucIdx = TC0_INDEX; ucIdx <= TC3_INDEX; ucIdx++) {
			prQueryStaStatistics->au4TcResourceEmptyCount[ucIdx] =
                    prQM->au4QmTcResourceEmptyCounter[prStaRec->ucBssIndex][ucIdx];
            
            prQM->au4QmTcResourceEmptyCounter[prStaRec->ucBssIndex][ucIdx] = 0;
        }

		
		if (prQueryStaStatistics->ucReadClear) {
            prStaRec->u4ThresholdCounter = 0;
            prStaRec->u4TotalTxPktsNumber = 0;
            prStaRec->u4TotalTxPktsTime = 0;
            prStaRec->u4TotalRxPktsNumber = 0;
            prStaRec->u4MaxTxPktsTime = 0;
        }
#endif

		for (ucIdx = TC0_INDEX; ucIdx <= TC3_INDEX; ucIdx++) {
			prQueryStaStatistics->au4TcQueLen[ucIdx] =
			    prStaRec->arTxQueue[ucIdx].u4NumElem;
        }
        
        rResult = WLAN_STATUS_SUCCESS;

		
		if (prAdapter->u4FwCompileFlag0 & COMPILE_FLAG0_GET_STA_LINK_STATUS) {

            rQueryCmdStaStatistics.ucIndex = prStaRec->ucIndex;
			COPY_MAC_ADDR(rQueryCmdStaStatistics.aucMacAddr,
				      prQueryStaStatistics->aucMacAddr);
            rQueryCmdStaStatistics.ucReadClear = prQueryStaStatistics->ucReadClear;
            
            rResult = wlanSendSetQueryCmd(prAdapter,
                            CMD_ID_GET_STA_STATISTICS,
                            FALSE,
                            TRUE,
                            TRUE,
                            nicCmdEventQueryStaStatistics,
                            nicOidCmdTimeoutCommon,
                            sizeof(CMD_GET_STA_STATISTICS_T),
						      (PUINT_8) &rQueryCmdStaStatistics,
						      pvQueryBuffer, u4QueryBufferLen);
            
            prQueryStaStatistics->u4Flag |= BIT(1);
		} else {
            rResult = WLAN_STATUS_NOT_SUPPORTED;
        }
        
	} while (FALSE);
    
	return rResult;
} 



VOID wlanQueryNicResourceInformation(IN P_ADAPTER_T prAdapter)
{
	

	

	
    nicTxResetResource(prAdapter);
}

#if 0
VOID
wlanBindNetInterface(IN P_GLUE_INFO_T prGlueInfo,
		     IN UINT_8 ucNetInterfaceIndex, IN PVOID pvNetInterface)
{
    P_NET_INTERFACE_INFO_T prNetIfInfo;

    prNetIfInfo = &prGlueInfo->arNetInterfaceInfo[ucNetInterfaceIndex];

    prNetIfInfo->pvNetInterface = pvNetInterface;
}
#endif
VOID
wlanBindBssIdxToNetInterface(IN P_GLUE_INFO_T prGlueInfo,
			     IN UINT_8 ucBssIndex, IN PVOID pvNetInterface)
{
    P_NET_INTERFACE_INFO_T prNetIfInfo;

    if (ucBssIndex > MAX_BSS_INDEX) {
        return;
    }

    prNetIfInfo = &prGlueInfo->arNetInterfaceInfo[ucBssIndex];

    prNetIfInfo->ucBssIndex = ucBssIndex;
    prNetIfInfo->pvNetInterface = pvNetInterface;
	
}

#if 0
UINT_8 wlanGetBssIdxByNetInterface(IN P_GLUE_INFO_T prGlueInfo, IN PVOID pvNetInterface)
{
    UINT_8 ucIdx = 0;

    for (ucIdx = 0; ucIdx < HW_BSSID_NUM; ucIdx++) {
        if (prGlueInfo->arNetInterfaceInfo[ucIdx].pvNetInterface == pvNetInterface)
            break;
    }
    
    return ucIdx;
}
#endif
PVOID wlanGetNetInterfaceByBssIdx(IN P_GLUE_INFO_T prGlueInfo, IN UINT_8 ucBssIndex)
{
    return prGlueInfo->arNetInterfaceInfo[ucBssIndex].pvNetInterface;
}


UINT_8 wlanGetAisBssIndex(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);
    ASSERT(prAdapter->prAisBssInfo);

    return prAdapter->prAisBssInfo->ucBssIndex;
}

VOID wlanInitFeatureOption(IN P_ADAPTER_T prAdapter)
{
    P_WIFI_VAR_T prWifiVar = &prAdapter->rWifiVar;
    P_QUE_MGT_T prQM = &prAdapter->rQM;

    

	prWifiVar->ucQoS = (UINT_8) wlanCfgGetUint32(prAdapter, "Qos", FEATURE_ENABLED);
    
	prWifiVar->ucStaHt = (UINT_8) wlanCfgGetUint32(prAdapter, "StaHT", FEATURE_ENABLED);
	prWifiVar->ucStaVht = (UINT_8) wlanCfgGetUint32(prAdapter, "StaVHT", FEATURE_ENABLED);

	prWifiVar->ucApHt = (UINT_8) wlanCfgGetUint32(prAdapter, "ApHT", FEATURE_ENABLED);
	prWifiVar->ucApVht = (UINT_8) wlanCfgGetUint32(prAdapter, "ApVHT", FEATURE_ENABLED);
    
	prWifiVar->ucP2pGoHt = (UINT_8) wlanCfgGetUint32(prAdapter, "P2pGoHT", FEATURE_ENABLED);
	prWifiVar->ucP2pGoVht = (UINT_8) wlanCfgGetUint32(prAdapter, "P2pGoVHT", FEATURE_ENABLED);
    
	prWifiVar->ucP2pGcHt = (UINT_8) wlanCfgGetUint32(prAdapter, "P2pGcHT", FEATURE_ENABLED);
	prWifiVar->ucP2pGcVht = (UINT_8) wlanCfgGetUint32(prAdapter, "P2pGcVHT", FEATURE_ENABLED);

	prWifiVar->ucAmpduRx = (UINT_8) wlanCfgGetUint32(prAdapter, "AmpduRx", FEATURE_ENABLED);
	prWifiVar->ucAmpduTx = (UINT_8) wlanCfgGetUint32(prAdapter, "AmpduTx", FEATURE_ENABLED);

	prWifiVar->ucTspec = (UINT_8) wlanCfgGetUint32(prAdapter, "Tspec", FEATURE_DISABLED);
    
	prWifiVar->ucUapsd = (UINT_8) wlanCfgGetUint32(prAdapter, "Uapsd", FEATURE_ENABLED);
	prWifiVar->ucStaUapsd = (UINT_8) wlanCfgGetUint32(prAdapter, "StaUapsd", FEATURE_DISABLED);
	prWifiVar->ucApUapsd = (UINT_8) wlanCfgGetUint32(prAdapter, "ApUapsd", FEATURE_DISABLED);
	prWifiVar->ucP2pUapsd = (UINT_8) wlanCfgGetUint32(prAdapter, "P2pUapsd", FEATURE_ENABLED);

	prWifiVar->ucTxShortGI = (UINT_8) wlanCfgGetUint32(prAdapter, "SgiTx", FEATURE_ENABLED);
	prWifiVar->ucRxShortGI = (UINT_8) wlanCfgGetUint32(prAdapter, "SgiRx", FEATURE_ENABLED);

	prWifiVar->ucTxLdpc = (UINT_8) wlanCfgGetUint32(prAdapter, "LdpcTx", FEATURE_ENABLED);
	prWifiVar->ucRxLdpc = (UINT_8) wlanCfgGetUint32(prAdapter, "LdpcRx", FEATURE_ENABLED);

	prWifiVar->ucTxStbc = (UINT_8) wlanCfgGetUint32(prAdapter, "StbcTx", FEATURE_DISABLED);
	prWifiVar->ucRxStbc = (UINT_8) wlanCfgGetUint32(prAdapter, "StbcRx", FEATURE_ENABLED);

	prWifiVar->ucTxGf = (UINT_8) wlanCfgGetUint32(prAdapter, "GfTx", FEATURE_ENABLED);
	prWifiVar->ucRxGf = (UINT_8) wlanCfgGetUint32(prAdapter, "GfRx", FEATURE_ENABLED);

	prWifiVar->ucSigTaRts = (UINT_8) wlanCfgGetUint32(prAdapter, "SigTaRts", FEATURE_DISABLED);
	prWifiVar->ucDynBwRts = (UINT_8) wlanCfgGetUint32(prAdapter, "DynBwRts", FEATURE_DISABLED);
	prWifiVar->ucTxopPsTx = (UINT_8) wlanCfgGetUint32(prAdapter, "TxopPsTx", FEATURE_DISABLED);

    prWifiVar->ucStaHtBfee = (UINT_8)wlanCfgGetUint32(prAdapter, "StaHTBfee", FEATURE_DISABLED);
    prWifiVar->ucStaVhtBfee = (UINT_8)wlanCfgGetUint32(prAdapter, "StaVHTBfee", FEATURE_ENABLED);
    prWifiVar->ucStaBfer = (UINT_8)wlanCfgGetUint32(prAdapter, "StaBfer", FEATURE_DISABLED);

	prWifiVar->ucApWpsMode = (UINT_8) wlanCfgGetUint32(prAdapter, "ApWpsMode", 0);
    DBGLOG(INIT, INFO, ("ucApWpsMode = %u\n", prWifiVar->ucApWpsMode));

	prWifiVar->ucThreadScheduling = (UINT_8) wlanCfgGetUint32(prAdapter, "ThreadSched", 0);
	prWifiVar->ucThreadPriority =
	    (UINT_8) wlanCfgGetUint32(prAdapter, "ThreadPriority", WLAN_TX_THREAD_TASK_PRIORITY);
	prWifiVar->cThreadNice =
	    (INT_8) wlanCfgGetInt32(prAdapter, "ThreadNice", WLAN_TX_THREAD_TASK_NICE);

    prAdapter->rQM.u4MaxForwardBufferCount = 
	    (UINT_32) wlanCfgGetUint32(prAdapter, "ApForwardBufferCnt", QM_FWD_PKT_QUE_THRESHOLD);

	prWifiVar->ucApChannel = (UINT_8) wlanCfgGetUint32(prAdapter, "ApChannel", 0);

    prWifiVar->ucApSco = (UINT_8)wlanCfgGetUint32(prAdapter, "ApSco", 0);
    prWifiVar->ucP2pGoSco = (UINT_8)wlanCfgGetUint32(prAdapter, "P2pGoSco", 0);

	prWifiVar->ucStaBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "StaBw", MAX_BW_80MHZ);
	prWifiVar->ucSta2gBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "Sta2gBw", MAX_BW_40MHZ);
	prWifiVar->ucSta5gBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "Sta5gBw", MAX_BW_80MHZ);
	prWifiVar->ucP2p2gBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "P2p2gBw", MAX_BW_40MHZ);
	prWifiVar->ucP2p5gBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "P2p5gBw", MAX_BW_40MHZ);
	prWifiVar->ucApBandwidth = (UINT_8) wlanCfgGetUint32(prAdapter, "ApBw", MAX_BW_40MHZ);
    
	prWifiVar->ucStaDisconnectDetectTh =
	    (UINT_8) wlanCfgGetUint32(prAdapter, "StaDisconnectDetectTh", 0);
	prWifiVar->ucApDisconnectDetectTh =
	    (UINT_8) wlanCfgGetUint32(prAdapter, "ApDisconnectDetectTh", 0);
	prWifiVar->ucP2pDisconnectDetectTh =
	    (UINT_8) wlanCfgGetUint32(prAdapter, "P2pDisconnectDetectTh", 0);
    
	prWifiVar->ucTcRestrict = (UINT_8) wlanCfgGetUint32(prAdapter, "TcRestrict", 0xFF);
    
	prWifiVar->u4MaxTxDeQLimit =
	    (UINT_32) wlanCfgGetUint32(prAdapter, "MaxTxDeQLimit", 0x0);
	prWifiVar->ucAlwaysResetUsedRes =
	    (UINT_32) wlanCfgGetUint32(prAdapter, "AlwaysResetUsedRes", 0x0);

#if CFG_SUPPORT_MTK_SYNERGY
	prWifiVar->ucMtkOui = (UINT_8) wlanCfgGetUint32(prAdapter, "MtkOui", FEATURE_ENABLED);
	prWifiVar->u4MtkOuiCap = (UINT_32) wlanCfgGetUint32(prAdapter, "MtkOuiCap", 0);
    prWifiVar->aucMtkFeature[0] = 0xff;
    prWifiVar->aucMtkFeature[1] = 0xff;
    prWifiVar->aucMtkFeature[2] = 0xff;
    prWifiVar->aucMtkFeature[3] = 0xff;
#endif 

    prWifiVar->ucCmdRsvResource = (UINT_8)wlanCfgGetUint32(prAdapter, "TxCmdRsv", QM_CMD_RESERVED_THRESHOLD);
    prWifiVar->u4MgmtQueueDelayTimeout = (UINT_32)wlanCfgGetUint32(prAdapter, "TxMgmtQueTO", QM_MGMT_QUEUED_TIMEOUT); 

    
    prWifiVar->u4HifIstLoopCount = (UINT_32)wlanCfgGetUint32(prAdapter, "IstLoop", CFG_IST_LOOP_COUNT);
    prWifiVar->u4Rx2OsLoopCount = (UINT_32)wlanCfgGetUint32(prAdapter, "Rx2OsLoop", 4);
    prWifiVar->u4HifTxloopCount = (UINT_32)wlanCfgGetUint32(prAdapter, "HifTxLoop", 1);
    prWifiVar->u4TxFromOsLoopCount = (UINT_32)wlanCfgGetUint32(prAdapter, "OsTxLoop", 1);
    prWifiVar->u4TxRxLoopCount = (UINT_32)wlanCfgGetUint32(prAdapter, "Rx2ReorderLoop", 1);

    prWifiVar->u4NetifStopTh = (UINT_32)wlanCfgGetUint32(prAdapter, "NetifStopTh", CFG_TX_STOP_NETIF_PER_QUEUE_THRESHOLD);
    prWifiVar->u4NetifStartTh = (UINT_32)wlanCfgGetUint32(prAdapter, "NetifStartTh", CFG_TX_START_NETIF_PER_QUEUE_THRESHOLD);
    prWifiVar->ucTxBaSize = (UINT_8)wlanCfgGetUint32(prAdapter, "TxBaSize", 64);
    prWifiVar->ucRxHtBaSize = (UINT_8)wlanCfgGetUint32(prAdapter, "RxHtBaSize", 64);
    prWifiVar->ucRxVhtBaSize = (UINT_8)wlanCfgGetUint32(prAdapter, "RxVhtBaSize", 32);

    
    prWifiVar->ucExtraTxDone = (UINT_32)wlanCfgGetUint32(prAdapter, "ExtraTxDone", 1);
    prWifiVar->ucTxDbg = (UINT_32)wlanCfgGetUint32(prAdapter, "TxDbg", 0);
    
    kalMemZero(prWifiVar->au4TcPageCount, sizeof(prWifiVar->au4TcPageCount));
    
    prWifiVar->au4TcPageCount[TC0_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc0Page", NIC_TX_PAGE_COUNT_TC0);
    prWifiVar->au4TcPageCount[TC1_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc1Page", NIC_TX_PAGE_COUNT_TC1);
    prWifiVar->au4TcPageCount[TC2_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc2Page", NIC_TX_PAGE_COUNT_TC2);
    prWifiVar->au4TcPageCount[TC3_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc3Page", NIC_TX_PAGE_COUNT_TC3);
    prWifiVar->au4TcPageCount[TC4_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc4Page", NIC_TX_PAGE_COUNT_TC4);   
    prWifiVar->au4TcPageCount[TC5_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc5Page", NIC_TX_PAGE_COUNT_TC5);       
    
    prQM->au4MinReservedTcResource[TC0_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc0MinRsv", QM_MIN_RESERVED_TC0_RESOURCE);
    prQM->au4MinReservedTcResource[TC1_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc1MinRsv", QM_MIN_RESERVED_TC1_RESOURCE);
    prQM->au4MinReservedTcResource[TC2_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc2MinRsv", QM_MIN_RESERVED_TC2_RESOURCE);
    prQM->au4MinReservedTcResource[TC3_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc3MinRsv", QM_MIN_RESERVED_TC3_RESOURCE);
    prQM->au4MinReservedTcResource[TC4_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc4MinRsv", QM_MIN_RESERVED_TC4_RESOURCE);
    prQM->au4MinReservedTcResource[TC5_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc5MinRsv", QM_MIN_RESERVED_TC5_RESOURCE);

    prQM->au4GuaranteedTcResource[TC0_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc0Grt", QM_GUARANTEED_TC0_RESOURCE);
    prQM->au4GuaranteedTcResource[TC1_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc1Grt", QM_GUARANTEED_TC1_RESOURCE);
    prQM->au4GuaranteedTcResource[TC2_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc2Grt", QM_GUARANTEED_TC2_RESOURCE);
    prQM->au4GuaranteedTcResource[TC3_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc3Grt", QM_GUARANTEED_TC3_RESOURCE);
    prQM->au4GuaranteedTcResource[TC4_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc4Grt", QM_GUARANTEED_TC4_RESOURCE);
    prQM->au4GuaranteedTcResource[TC5_INDEX] = (UINT_32)wlanCfgGetUint32(prAdapter, "Tc5Grt", QM_GUARANTEED_TC5_RESOURCE);

    prQM->u4TimeToAdjustTcResource = (UINT_32)wlanCfgGetUint32(prAdapter, "TcAdjustTime", QM_INIT_TIME_TO_ADJUST_TC_RSC);
    prQM->u4TimeToUpdateQueLen = (UINT_32)wlanCfgGetUint32(prAdapter, "QueLenUpdateTime", QM_INIT_TIME_TO_UPDATE_QUE_LEN);
    prQM->u4QueLenMovingAverage = (UINT_32)wlanCfgGetUint32(prAdapter, "QueLenMovingAvg", QM_QUE_LEN_MOVING_AVE_FACTOR);
    prQM->u4ExtraReservedTcResource = (UINT_32)wlanCfgGetUint32(prAdapter, "TcExtraRsv", QM_EXTRA_RESERVED_RESOURCE_WHEN_BUSY);

}

VOID
wlanCfgSetSwCtrl(
    IN P_ADAPTER_T prAdapter
    )
{
    UINT_32 i = 0;
    CHAR   aucKey[WLAN_CFG_VALUE_LEN_MAX];
    CHAR   aucValue[WLAN_CFG_VALUE_LEN_MAX];
    const CHAR   acDelim[] = " ";    
    CHAR   *pcPtr = NULL;
    CHAR   *pcDupValue = NULL;
    
    UINT_32 au4Values[2];
    UINT_32 u4TokenCount = 0;
    UINT_32 u4BufLen = 0;
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    P_GLUE_INFO_T prGlueInfo = prAdapter->prGlueInfo;
    PARAM_CUSTOM_SW_CTRL_STRUC_T rSwCtrlInfo;

    for(i = 0;i < WLAN_CFG_SET_SW_CTRL_LEN_MAX; i++) {
        kalMemZero(aucValue, WLAN_CFG_VALUE_LEN_MAX);
        kalMemZero(aucKey, WLAN_CFG_VALUE_LEN_MAX);        
        kalSprintf(aucKey, "SwCtrl%d", i);

        
        if(wlanCfgGet(prAdapter, aucKey, aucValue, "", 0) != WLAN_STATUS_SUCCESS) continue;
        if(!kalStrCmp(aucValue, "")) continue;

        pcDupValue = aucValue;
        u4TokenCount = 0;

        while((pcPtr = kalStrSep((char **)(&pcDupValue), acDelim)) != NULL) {

            if(!kalStrCmp(pcPtr, "")) continue;

            au4Values[u4TokenCount] = kalStrtoul(pcPtr, NULL, 0);
            u4TokenCount++;

            
            if(u4TokenCount >= 2) break;
        }

        if(u4TokenCount != 2) continue;

        rSwCtrlInfo.u4Id = au4Values[0];
        rSwCtrlInfo.u4Data = au4Values[1]; 

        rStatus = kalIoctl(prGlueInfo,
            wlanoidSetSwCtrlWrite,
            &rSwCtrlInfo,
            sizeof(rSwCtrlInfo),
            FALSE,
            FALSE,
            TRUE,
            &u4BufLen);

    } 
}

VOID
wlanCfgSetChip(
    IN P_ADAPTER_T prAdapter
    )
{
    UINT_32 i = 0;
    CHAR   aucKey[WLAN_CFG_VALUE_LEN_MAX];
    CHAR   aucValue[WLAN_CFG_VALUE_LEN_MAX];   

    UINT_32 u4BufLen = 0;
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    P_GLUE_INFO_T prGlueInfo = prAdapter->prGlueInfo;
    PARAM_CUSTOM_CHIP_CONFIG_STRUC_T rChipConfigInfo;

    for(i = 0;i < WLAN_CFG_SET_CHIP_LEN_MAX; i++) {
        kalMemZero(aucValue, WLAN_CFG_VALUE_LEN_MAX);
        kalMemZero(aucKey, WLAN_CFG_VALUE_LEN_MAX);        
        kalSprintf(aucKey, "SetChip%d", i);

        
        if(wlanCfgGet(prAdapter, aucKey, aucValue, "", 0) != WLAN_STATUS_SUCCESS) continue;
        if(!kalStrCmp(aucValue, "")) continue;

        kalMemZero(&rChipConfigInfo, sizeof(rChipConfigInfo)); 

        rChipConfigInfo.ucType = CHIP_CONFIG_TYPE_WO_RESPONSE; 
        rChipConfigInfo.u2MsgSize = kalStrnLen(aucValue, WLAN_CFG_VALUE_LEN_MAX);
        kalStrnCpy(rChipConfigInfo.aucCmd, aucValue, CHIP_CONFIG_RESP_SIZE);

        rStatus = kalIoctl(prGlueInfo,
            wlanoidSetChipConfig,
            &rChipConfigInfo,
            sizeof(rChipConfigInfo),
            FALSE,
            FALSE,
            TRUE,
            &u4BufLen);  
    }
        
}

VOID
wlanCfgSetDebugLevel(
    IN P_ADAPTER_T prAdapter
    )
{
    UINT_32 i = 0;
    CHAR   aucKey[WLAN_CFG_VALUE_LEN_MAX];
    CHAR   aucValue[WLAN_CFG_VALUE_LEN_MAX];
    const CHAR   acDelim[] = " ";
    CHAR   *pcDupValue;    
    CHAR   *pcPtr = NULL;

    UINT_32 au4Values[2];
    UINT_32 u4TokenCount = 0;
    UINT_32 u4DbgIdx = 0;
    UINT_32 u4DbgMask = 0;    

    for(i = 0;i < WLAN_CFG_SET_DEBUG_LEVEL_LEN_MAX; i++) {
        kalMemZero(aucValue, WLAN_CFG_VALUE_LEN_MAX);
        kalMemZero(aucKey, WLAN_CFG_VALUE_LEN_MAX);
        kalSprintf(aucKey, "DbgLevel%d", i);
        
        
        if(wlanCfgGet(prAdapter, aucKey, aucValue, "", 0) != WLAN_STATUS_SUCCESS) continue;
        if(!kalStrCmp(aucValue, "")) continue;

        pcDupValue = aucValue;
        u4TokenCount = 0;

        while((pcPtr = kalStrSep((char **)(&pcDupValue), acDelim)) != NULL) {

            if(!kalStrCmp(pcPtr, "")) continue;

            au4Values[u4TokenCount] = kalStrtoul(pcPtr, NULL, 0);
            u4TokenCount++;

            
            if(u4TokenCount >= 2) break;
        }

        if(u4TokenCount != 2) continue;
            
        u4DbgIdx = au4Values[0];
        u4DbgMask = au4Values[1];

        
        if(u4DbgIdx == 0xFFFFFFFF) {
            wlanSetDebugLevel(DBG_ALL_MODULE_IDX, u4DbgMask);
            DBGLOG(INIT, INFO, ("Set ALL DBG module log level to [0x%02x]!", (UINT_8)u4DbgMask));
        }
        else if(u4DbgIdx == 0xFFFFFFFE) {
            wlanDebugInit();
            DBGLOG(INIT, INFO, ("Reset ALL DBG module log level to DEFAULT!"));
        }
        else if(u4DbgIdx < DBG_MODULE_NUM) {
            wlanSetDebugLevel(u4DbgIdx, u4DbgMask);
            DBGLOG(INIT, INFO, ("Set DBG module[%lu] log level to [0x%02x]!", u4DbgIdx, (UINT_8)u4DbgMask));
        }        
    }
    
}

VOID
wlanCfgSetCountryCode(
    IN P_ADAPTER_T prAdapter
    )
{
    CHAR   aucValue[WLAN_CFG_VALUE_LEN_MAX];
    	
    
    if(wlanCfgGet(prAdapter, "Country", aucValue, "", 0) == WLAN_STATUS_SUCCESS) {	
        prAdapter->rWifiVar.rConnSettings.u2CountryCode =
        (((UINT_16) aucValue[0]) << 8) | ((UINT_16) aucValue[1]) ;

        prAdapter->prDomainInfo = NULL; 
        rlmDomainSendCmd(prAdapter, TRUE);    	
    }
}


#if CFG_SUPPORT_CFG_FILE

P_WLAN_CFG_ENTRY_T wlanCfgGetEntry(IN P_ADAPTER_T prAdapter, const PCHAR pucKey)
{

    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;
    UINT_32 i;
    prWlanCfg = prAdapter->prWlanCfg;

    ASSERT(prWlanCfg);
    ASSERT(pucKey);

    prWlanCfgEntry = NULL;

	for (i = 0; i < WLAN_CFG_ENTRY_NUM_MAX; i++) {
        prWlanCfgEntry = &prWlanCfg->arWlanCfgBuf[i];
		if (prWlanCfgEntry->aucKey[0] != '\0') {
			DBGLOG(INIT, LOUD,
			       ("compare key %s saved key %s\n", pucKey, prWlanCfgEntry->aucKey));
			if (kalStrnCmp(pucKey, prWlanCfgEntry->aucKey, WLAN_CFG_KEY_LEN_MAX - 1) ==
			    0) {
                return prWlanCfgEntry;
            }
        }
    }

    DBGLOG(INIT, TRACE, ("wifi config there is no entry \'%s\'\n", pucKey));
    return NULL;

}

WLAN_STATUS
wlanCfgGet(IN P_ADAPTER_T prAdapter,
	   const PCHAR pucKey, PCHAR pucValue, PCHAR pucValueDef, UINT_32 u4Flags)
{

    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;
    prWlanCfg = prAdapter->prWlanCfg;

    ASSERT(prWlanCfg);
    ASSERT(pucValue);

    
    prWlanCfgEntry = wlanCfgGetEntry(prAdapter, pucKey);

	if (prWlanCfgEntry) {
		kalStrnCpy(pucValue, prWlanCfgEntry->aucValue, WLAN_CFG_VALUE_LEN_MAX - 1);
        return WLAN_STATUS_SUCCESS;
	} else {
		if (pucValueDef)
			kalStrnCpy(pucValue, pucValueDef, WLAN_CFG_VALUE_LEN_MAX - 1);
        return WLAN_STATUS_FAILURE;
    }

}

UINT_32 wlanCfgGetUint32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, UINT_32 u4ValueDef)
{
    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;
    UINT_32 u4Ret;
    prWlanCfg = prAdapter->prWlanCfg;

    ASSERT(prWlanCfg);

    u4Ret = u4ValueDef;
    
    prWlanCfgEntry = wlanCfgGetEntry(prAdapter, pucKey);

	if (prWlanCfgEntry) {
		u4Ret = kalStrtoul(prWlanCfgEntry->aucValue, NULL, 0);
    }
    return u4Ret;
}

INT_32 wlanCfgGetInt32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, INT_32 i4ValueDef)
{
    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;
    INT_32 i4Ret;
    prWlanCfg = prAdapter->prWlanCfg;

    ASSERT(prWlanCfg);

    i4Ret = i4ValueDef;
    
    prWlanCfgEntry = wlanCfgGetEntry(prAdapter, pucKey);

	if (prWlanCfgEntry) {
		i4Ret = kalStrtol(prWlanCfgEntry->aucValue, NULL, 0);
    }
    return i4Ret;
}



WLAN_STATUS
wlanCfgSet(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, PCHAR pucValue, UINT_32 u4Flags)
{

    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;
    UINT_32 u4EntryIndex;
    UINT_32 i;
    UINT_8 ucExist;

    prWlanCfg = prAdapter->prWlanCfg;
    ASSERT(prWlanCfg);
    ASSERT(pucKey);

    
    ucExist = 0;
    prWlanCfgEntry = wlanCfgGetEntry(prAdapter, pucKey);

	if (!prWlanCfgEntry) {
        
		for (i = 0; i < WLAN_CFG_ENTRY_NUM_MAX; i++) {
            prWlanCfgEntry = &prWlanCfg->arWlanCfgBuf[i];
			if (prWlanCfgEntry->aucKey[0] == '\0') {
                break;
            }
        }

        u4EntryIndex = i;
		if (u4EntryIndex < WLAN_CFG_ENTRY_NUM_MAX) {
            prWlanCfgEntry = &prWlanCfg->arWlanCfgBuf[u4EntryIndex];
			kalMemZero(prWlanCfgEntry, sizeof(WLAN_CFG_ENTRY_T));
		} else {
            prWlanCfgEntry = NULL;
            DBGLOG(INIT, ERROR, ("wifi config there is no empty entry\n"));
        }
    } 
    else {
        ucExist = 1;
    }

	if (prWlanCfgEntry) {
		if (ucExist == 0) {
			kalStrnCpy(prWlanCfgEntry->aucKey, pucKey, WLAN_CFG_KEY_LEN_MAX - 1);
			prWlanCfgEntry->aucKey[WLAN_CFG_KEY_LEN_MAX - 1] = '\0';
        }

		if (pucValue && pucValue[0] != '\0') {
			kalStrnCpy(prWlanCfgEntry->aucValue, pucValue, WLAN_CFG_VALUE_LEN_MAX - 1);
			prWlanCfgEntry->aucValue[WLAN_CFG_VALUE_LEN_MAX - 1] = '\0';

			if (ucExist) {
				if (prWlanCfgEntry->pfSetCb)
                    prWlanCfgEntry->pfSetCb(prAdapter, 
                            prWlanCfgEntry->aucKey, 
                            prWlanCfgEntry->aucValue, 
                            prWlanCfgEntry->pPrivate, 0);
            }
		} else {
            
            
			kalMemZero(prWlanCfgEntry, sizeof(WLAN_CFG_ENTRY_T));
        }

	}
	
	if (prWlanCfgEntry) {
        DBGLOG(INIT, INFO, ("Set wifi config exist %u \'%s\' \'%s\'\n",
                    ucExist, prWlanCfgEntry->aucKey, prWlanCfgEntry->aucValue));
        return WLAN_STATUS_SUCCESS;
	} else {
		if (pucKey) {
            DBGLOG(INIT, ERROR, ("Set wifi config error key \'%s\'\n", pucKey));
        }
		if (pucValue) {
            DBGLOG(INIT, ERROR, ("Set wifi config error value \'%s\'\n", pucValue));
        }
        return WLAN_STATUS_FAILURE;
    }

}

WLAN_STATUS
wlanCfgSetCb(IN P_ADAPTER_T prAdapter,
	     const PCHAR pucKey, WLAN_CFG_SET_CB pfSetCb, void *pPrivate, UINT_32 u4Flags)
{

    P_WLAN_CFG_ENTRY_T prWlanCfgEntry;
    P_WLAN_CFG_T prWlanCfg;

    prWlanCfg = prAdapter->prWlanCfg;
    ASSERT(prWlanCfg);

    
    prWlanCfgEntry = wlanCfgGetEntry(prAdapter, pucKey);

	if (prWlanCfgEntry) {
        prWlanCfgEntry->pfSetCb = pfSetCb;
        prWlanCfgEntry->pPrivate = pPrivate;
    }

	if (prWlanCfgEntry) {
        return WLAN_STATUS_SUCCESS;
	} else {
        return WLAN_STATUS_FAILURE;
    }

}

WLAN_STATUS wlanCfgSetUint32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, UINT_32 u4Value)
{

    P_WLAN_CFG_T prWlanCfg;
    UINT_8 aucBuf[WLAN_CFG_VALUE_LEN_MAX];

    prWlanCfg = prAdapter->prWlanCfg;

    ASSERT(prWlanCfg);

	kalMemZero(aucBuf, sizeof(aucBuf));

	kalSnprintf(aucBuf, WLAN_CFG_VALUE_LEN_MAX, "0x%x", (unsigned int)u4Value);

	return wlanCfgSet(prAdapter, pucKey, aucBuf, 0);
}

enum {
    STATE_EOF = 0,
    STATE_TEXT = 1,
    STATE_NEWLINE = 2
};

struct WLAN_CFG_PARSE_STATE_S {
    CHAR *ptr;
    CHAR *text;
    INT_32 nexttoken;
    UINT_32 maxSize;
};

INT_32 wlanCfgFindNextToken(struct WLAN_CFG_PARSE_STATE_S *state)
{
    CHAR *x = state->ptr;
    CHAR *s;

    if (state->nexttoken) {
        INT_32 t = state->nexttoken;
        state->nexttoken = 0;
        return t;
    }

    for (;;) {
        switch (*x) {
        case 0:
            state->ptr = x;
            return STATE_EOF;
        case '\n':
            x++;
            state->ptr = x;
            return STATE_NEWLINE;
        case ' ':
        case '\t':
        case '\r':
            x++;
            continue;
        case '#':
			while (*x && (*x != '\n'))
				x++;
            if (*x == '\n') {
				state->ptr = x + 1;
                return STATE_NEWLINE;
            } else {
                state->ptr = x;
                return STATE_EOF;
            }
        default:
            goto text;
        }
    }

textdone:
    state->ptr = x;
    *s = 0;
    return STATE_TEXT;
text:
    state->text = s = x;
textresume:
    for (;;) {
        switch (*x) {
        case 0:
            goto textdone;
        case ' ':
        case '\t':
        case '\r':
            x++;
            goto textdone;
        case '\n':
            state->nexttoken = STATE_NEWLINE;
            x++;
            goto textdone;
        case '"':
            x++;
            for (;;) {
                switch (*x) {
                case 0:
                        
                    state->ptr = x;
                    return STATE_EOF;
                case '"':
                    x++;
                    goto textresume;
                default:
                    *s++ = *x++;
                }
            }
            break;
        case '\\':
            x++;
            switch (*x) {
            case 0:
                goto textdone;
            case 'n':
                *s++ = '\n';
                break;
            case 'r':
                *s++ = '\r';
                break;
            case 't':
                *s++ = '\t';
                break;
            case '\\':
                *s++ = '\\';
                break;
            case '\r':
                    
                if (x[1] != '\n') {
                    x++;
                    continue;
                }
            case '\n':
                    
                x++;
                    
				while ((*x == ' ') || (*x == '\t'))
					x++;
                continue;
            default:
                    
                *s++ = *x++;
            }
            continue;
        default:
            *s++ = *x++;
        }
    }
    return STATE_EOF;
}

WLAN_STATUS wlanCfgParseArgument(CHAR *cmdLine, INT_32 *argc, CHAR *argv[]
        )
{  
    struct WLAN_CFG_PARSE_STATE_S state;  
    CHAR **args;  
    INT_32 nargs;  

	if (cmdLine == NULL || argc == NULL || argv == NULL) {
        ASSERT(0);
        return WLAN_STATUS_FAILURE;
    }
    args = argv;
    nargs = 0;  
    state.ptr = cmdLine;  
    state.nexttoken = 0;
    state.maxSize = 0;

	if (kalStrnLen(cmdLine, 512) >= 512) {
        ASSERT(0);
        return WLAN_STATUS_FAILURE;
    }
  
    for (;;) {  
        switch (wlanCfgFindNextToken(&state)) {  
        case STATE_EOF:  
            goto exit;
        case STATE_NEWLINE:  
            goto exit;
        case STATE_TEXT:  
            if (nargs < WLAN_CFG_ARGV_MAX) {  
                args[nargs++] = state.text;  
            }  
            break;  
        }  
    }  
  
exit:
    *argc = nargs;
    return WLAN_STATUS_SUCCESS;
}  



WLAN_STATUS
wlanCfgParseAddEntry(IN P_ADAPTER_T prAdapter,
    PUINT_8    pucKeyHead, 
		     PUINT_8 pucKeyTail, PUINT_8 pucValueHead, PUINT_8 pucValueTail)
{

    UINT_8 aucKey[WLAN_CFG_KEY_LEN_MAX];
    UINT_8 aucValue[WLAN_CFG_VALUE_LEN_MAX];
    UINT_32 u4Len;

    kalMemZero(aucKey, sizeof(aucKey));
    kalMemZero(aucValue, sizeof(aucValue));

	if ((pucKeyHead == NULL)
            || (pucValueHead == NULL) 
	    ) {
        return WLAN_STATUS_FAILURE;
    }

	if (pucKeyTail) {
		if (pucKeyHead > pucKeyTail)
            return WLAN_STATUS_FAILURE;
		u4Len = pucKeyTail - pucKeyHead + 1;
	} else {
		u4Len = kalStrnLen(pucKeyHead, WLAN_CFG_KEY_LEN_MAX - 1);
    }

	if (u4Len >= WLAN_CFG_KEY_LEN_MAX) {
		u4Len = WLAN_CFG_KEY_LEN_MAX - 1;
    }
    kalStrnCpy(aucKey, pucKeyHead, u4Len);

	if (pucValueTail) {
		if (pucValueHead > pucValueTail)
            return WLAN_STATUS_FAILURE;
		u4Len = pucValueTail - pucValueHead + 1;
	} else {
		u4Len = kalStrnLen(pucValueHead, WLAN_CFG_VALUE_LEN_MAX - 1);
    }

	if (u4Len >= WLAN_CFG_VALUE_LEN_MAX) {
		u4Len = WLAN_CFG_VALUE_LEN_MAX - 1;
    }
    kalStrnCpy(aucValue, pucValueHead, u4Len);

    return wlanCfgSet(prAdapter, aucKey, aucValue, 0);
}

enum {
    WAIT_KEY_HEAD = 0,
    WAIT_KEY_TAIL,
    WAIT_VALUE_HEAD,
    WAIT_VALUE_TAIL,
    WAIT_COMMENT_TAIL
};


WLAN_STATUS wlanCfgParse(IN P_ADAPTER_T prAdapter, PUINT_8 pucConfigBuf, UINT_32 u4ConfigBufLen)
{

    struct WLAN_CFG_PARSE_STATE_S state;  
    PCHAR  apcArgv[WLAN_CFG_ARGV_MAX];
    CHAR **args;  
    INT_32 nargs;  


	if (pucConfigBuf == NULL) {
        ASSERT(0);
        return WLAN_STATUS_FAILURE;
    }
	if (kalStrnLen(pucConfigBuf, 4000) >= 4000) {
        ASSERT(0);
        return WLAN_STATUS_FAILURE;
    }
	if (u4ConfigBufLen == 0) {
        return WLAN_STATUS_FAILURE;
    }
    args = apcArgv;
    nargs = 0;  
    state.ptr = pucConfigBuf;
    state.nexttoken = 0;
    state.maxSize = u4ConfigBufLen;
 
    for (;;) {  
        switch (wlanCfgFindNextToken(&state)) {  
        case STATE_EOF:  
			if (nargs > 1) {
				wlanCfgParseAddEntry(prAdapter, args[0], NULL, args[1], NULL);
            }
            goto exit;
        case STATE_NEWLINE:
			if (nargs > 1) {
				wlanCfgParseAddEntry(prAdapter, args[0], NULL, args[1], NULL);
            }
            nargs = 0;
            break;
        case STATE_TEXT:  
            if (nargs < WLAN_CFG_ARGV_MAX) {  
                args[nargs++] = state.text;  
            }  
            break;  
        }  
    }  
  
exit:
    return WLAN_STATUS_SUCCESS;

#if 0
     
    UINT_32 i;
    UINT_8 c;
    PUINT_8 pbuf;
    UINT_8 ucState;
    PUINT_8    pucKeyTail = NULL;
    PUINT_8    pucKeyHead = NULL;
    PUINT_8    pucValueHead = NULL;
    PUINT_8    pucValueTail = NULL;

    ucState = WAIT_KEY_HEAD;
    pbuf = pucConfigBuf;

	for (i = 0; i < u4ConfigBufLen; i++) {
        c = pbuf[i];
		if (c == '\r' || c == '\n') {

			if (ucState == WAIT_VALUE_TAIL) {
                
				if (pucValueHead) {
					wlanCfgParseAddEntry(prAdapter, pucKeyHead, pucKeyTail,
							     pucValueHead, pucValueTail);
                }
            }
            ucState = WAIT_KEY_HEAD;
            pucKeyTail = NULL;
            pucKeyHead = NULL;
            pucValueHead = NULL;
            pucValueTail = NULL;

		} else if (c == '=') {
			if (ucState == WAIT_KEY_TAIL) {
				pucKeyTail = &pbuf[i - 1];
                ucState = WAIT_VALUE_HEAD;
            }
		} else if (c == ' ' || c == '\t') {
			if (ucState == WAIT_KEY_HEAD) {


			} else if (ucState == WAIT_KEY_TAIL) {
				pucKeyTail = &pbuf[i - 1];
                ucState = WAIT_VALUE_HEAD;
            }
		} else {

			if (c == '#') {
                
				if (ucState == WAIT_KEY_HEAD) {
                    ucState = WAIT_COMMENT_TAIL;
				} else if (ucState == WAIT_VALUE_TAIL) {
                    pucValueTail = &pbuf[i];
                }

			} else {
				if (ucState == WAIT_KEY_HEAD) {
                    pucKeyHead = &pbuf[i];
					pucKeyTail = &pbuf[i];
                    ucState = WAIT_KEY_TAIL;
				} else if (ucState == WAIT_VALUE_HEAD) {
                    pucValueHead = &pbuf[i];
                    pucValueTail = &pbuf[i];
                    ucState = WAIT_VALUE_TAIL;
				} else if (ucState == WAIT_VALUE_TAIL) {
                    pucValueTail = &pbuf[i];
                }
            }
        }

    } 

	if (ucState == WAIT_VALUE_TAIL) {
        
		if (pucValueTail) {
			wlanCfgParseAddEntry(prAdapter, pucKeyHead, pucKeyTail, pucValueHead,
					     pucValueTail);
        }
    }
#endif

    return WLAN_STATUS_SUCCESS;
}

WLAN_STATUS
wlanCfgInit(IN P_ADAPTER_T prAdapter, PUINT_8 pucConfigBuf, UINT_32 u4ConfigBufLen, UINT_32 u4Flags)
{
    P_WLAN_CFG_T prWlanCfg;
	
    prAdapter->prWlanCfg = &prAdapter->rWlanCfg;
    prWlanCfg = prAdapter->prWlanCfg;

    kalMemZero(prWlanCfg, sizeof(WLAN_CFG_T));
    ASSERT(prWlanCfg);
    prWlanCfg->u4WlanCfgEntryNumMax = WLAN_CFG_ENTRY_NUM_MAX;
    prWlanCfg->u4WlanCfgKeyLenMax = WLAN_CFG_KEY_LEN_MAX;
    prWlanCfg->u4WlanCfgValueLenMax = WLAN_CFG_VALUE_LEN_MAX;

    DBGLOG(INIT, INFO, ("Init wifi config len %u max entry %u\n", 
                u4ConfigBufLen, prWlanCfg->u4WlanCfgEntryNumMax));
#if DBG
    
	wlanCfgSet(prAdapter, "ConfigValid", "0x123", 0);
	if (wlanCfgGetUint32(prAdapter, "ConfigValid", 0) != 0x123) {
		DBGLOG(INIT, INFO, ("wifi config error %u\n", __LINE__));
    }
	wlanCfgSet(prAdapter, "ConfigValid", "1", 0);
	if (wlanCfgGetUint32(prAdapter, "ConfigValid", 0) != 1) {
		DBGLOG(INIT, INFO, ("wifi config error %u\n", __LINE__));
    }
#endif

    

	if (pucConfigBuf && (u4ConfigBufLen > 0)) {
		wlanCfgParse(prAdapter, pucConfigBuf, u4ConfigBufLen);
    }


    return WLAN_STATUS_SUCCESS;
}

#endif 



INT_32 wlanHexToNum(CHAR c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}


INT_32 wlanHexToByte(PCHAR hex)
{
	INT_32 a, b;
	a = wlanHexToNum(*hex++);
	if (a < 0)
		return -1;
	b = wlanHexToNum(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}



INT_32 wlanHwAddrToBin(PCHAR txt, UINT_8 *addr)
{
	INT_32 i;
	PCHAR pos = txt;

	for (i = 0; i < 6; i++) {
		INT_32 a, b;

		while (*pos == ':' || *pos == '.' || *pos == '-')
			pos++;

		a = wlanHexToNum(*pos++);
		if (a < 0)
			return -1;
		b = wlanHexToNum(*pos++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
	}

	return pos - txt;
}

BOOLEAN wlanIsChipNoAck(IN P_ADAPTER_T prAdapter)
{
    BOOLEAN fgIsNoAck;

	fgIsNoAck = prAdapter->fgIsChipNoAck || kalIsResetting()
                || fgIsBusAccessFailed;

    return fgIsNoAck;
}

#if CFG_AUTO_CHANNEL_SEL_SUPPORT

WLAN_STATUS 
wlanoidQueryACSChannelList(IN P_ADAPTER_T prAdapter,
    IN PVOID        pvQueryBuffer,
			   IN UINT_32 u4QueryBufferLen, OUT PUINT_32 pu4QueryInfoLen)
{
        WLAN_STATUS rResult = WLAN_STATUS_FAILURE;
        P_PARAM_GET_LTE_MODE prLteMode;
        UINT_8 ucIdx;	
        P_PARAM_CHN_LOAD_INFO prChnLoad;
	
	DBGLOG(P2P, INFO, ("[Auto Channel]wlanoidQueryACSChannelList\n"));
	do {
        ASSERT(pvQueryBuffer);

        
        if ((prAdapter == NULL) || (pu4QueryInfoLen == NULL))
			break;
		if ((u4QueryBufferLen) && (pvQueryBuffer == NULL))
			break;
		
		prLteMode = (P_PARAM_GET_LTE_MODE) pvQueryBuffer;
		
		for (ucIdx = 0; ucIdx < MAX_AUTO_CHAL_NUM; ucIdx++) {
			prChnLoad =
			    (P_PARAM_CHN_LOAD_INFO) &(prAdapter->rWifiVar.rChnLoadInfo.
						       rEachChnLoad[ucIdx]);
			
			DBGLOG(P2P, INFO,
			       ("[Auto Channel] AP Num: Chn[%d]=%d\n", ucIdx + 1,
				prChnLoad->u2APNum));
		}
				
#if 0
		if (prAdapter->u4FwCompileFlag0 & COMPILE_FLAG0_GET_STA_LINK_STATUS) {
			printk("wlanoidQueryACSChannelList\n");
			CMD_ACCESS_REG rCmdAccessReg;
         	rCmdAccessReg.u4Address = 0xFFFFFFFF;
            rCmdAccessReg.u4Data = ELEM_RM_TYPE_ACS_CHN;
           
			rResult = wlanSendSetQueryCmd(prAdapter, CMD_ID_ACCESS_REG, TRUE, TRUE, TRUE, nicCmdEventQueryChannelLoad,	
			   nicOidCmdTimeoutCommon,
			   sizeof(CMD_ACCESS_REG),
						      (PUINT_8) &rCmdAccessReg,
						      pvQueryBuffer, u4QueryBufferLen);
            
            prQueryChnLoad->u4Flag |= BIT(1);
		} else {
            rResult = WLAN_STATUS_NOT_SUPPORTED;
        }
#endif
				
		prLteMode->u4Flags &= BIT(0);
		
		{
			CMD_GET_LTE_SAFE_CHN_T rQuery_LTE_SAFE_CHN;
			rResult = wlanSendSetQueryCmd(prAdapter, CMD_ID_GET_LTE_CHN, FALSE, TRUE, TRUE,	
						      nicCmdEventQueryLTESafeChn,	
										nicOidCmdTimeoutCommon,
										sizeof(CMD_GET_LTE_SAFE_CHN_T),
						      (PUINT_8) &rQuery_LTE_SAFE_CHN,
						      pvQueryBuffer, u4QueryBufferLen);
			DBGLOG(P2P, INFO, ("[Auto Channel] Get LTE Channels\n"));
			prLteMode->u4Flags |= BIT(1);
		}

				
		DBGLOG(P2P, INFO, ("[Auto Channel] Candidated Channels\n"));
	} while (FALSE);
    
	return rResult;
} 
#endif

#if CFG_ENABLE_PER_STA_STATISTICS
VOID
wlanTxLifetimeUpdateStaStats(IN P_ADAPTER_T prAdapter, 
    IN P_MSDU_INFO_T prMsduInfo)
{
    P_STA_RECORD_T prStaRec;
    UINT_32 u4DeltaTime;
    P_QUE_MGT_T prQM = &prAdapter->rQM;
    P_PKT_PROFILE_T prPktProfile = &prMsduInfo->rPktProfile;
    UINT_32 u4PktPrintPeriod = 0;

    prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

    if (prStaRec) {
    	u4DeltaTime = (UINT_32)(prPktProfile->rHifTxDoneTimestamp -
    		       prPktProfile->rHardXmitArrivalTimestamp);

        
    	prStaRec->u4TotalTxPktsNumber++;
    	prStaRec->u4TotalTxPktsTime += u4DeltaTime;
        
    	if (u4DeltaTime > prStaRec->u4MaxTxPktsTime) {
    		prStaRec->u4MaxTxPktsTime = u4DeltaTime;
    	}
    	if (u4DeltaTime >= NIC_TX_TIME_THRESHOLD) {
    		prStaRec->u4ThresholdCounter++;
    	}

    	if (u4PktPrintPeriod && (prStaRec->u4TotalTxPktsNumber >= u4PktPrintPeriod)) {
    		DBGLOG(TX, TRACE, ("[%u]N[%4lu]A[%5lu]M[%4lu]T[%4lu]E[%4lu]\n",
    			prStaRec->ucIndex,
    			prStaRec->u4TotalTxPktsNumber,
    			(prStaRec->u4TotalTxPktsTime/prStaRec->u4TotalTxPktsNumber),
    			prStaRec->u4MaxTxPktsTime,
    			prStaRec->u4ThresholdCounter,
    			prQM->au4QmTcResourceEmptyCounter[prStaRec->ucBssIndex][TC2_INDEX]));

    		prStaRec->u4TotalTxPktsNumber = 0;
    		prStaRec->u4TotalTxPktsTime = 0;
    		prStaRec->u4MaxTxPktsTime = 0;
    		prStaRec->u4ThresholdCounter = 0;
    		prQM->au4QmTcResourceEmptyCounter[prStaRec->ucBssIndex][TC2_INDEX] = 0;
    	}
    }
}
#endif

BOOLEAN
wlanTxLifetimeIsProfilingEnabled(IN P_ADAPTER_T prAdapter)
{
    BOOLEAN fgEnabled = FALSE;
#if CFG_SUPPORT_WFD
	P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;

	prWfdCfgSettings = &prAdapter->rWifiVar.rWfdConfigureSettings;

    if(prWfdCfgSettings->ucWfdEnable > 0)
        fgEnabled = TRUE;
#endif

    return fgEnabled;
}

BOOLEAN
wlanTxLifetimeIsTargetMsdu(IN P_ADAPTER_T prAdapter, 
    IN P_MSDU_INFO_T prMsduInfo)
{
    BOOLEAN fgResult = TRUE;

#if 0
	switch (prMsduInfo->ucTID) {
		
    case 1: 
    case 2: 

		
    case 0:
    case 3:
        fgResult = FALSE;
        break;
		
	case 4:
	case 5:

		
	case 6:
	case 7:
		fgResult = TRUE;
		break;
	default:
		break;
	}
#endif    
    return fgResult;
}

VOID
wlanTxLifetimeTagPacket(IN P_ADAPTER_T prAdapter,
    IN P_MSDU_INFO_T prMsduInfo, IN ENUM_TX_PROFILING_TAG_T eTag)
{
    P_PKT_PROFILE_T prPktProfile = &prMsduInfo->rPktProfile;

    if(!wlanTxLifetimeIsProfilingEnabled(prAdapter))
        return;
    
    switch(eTag) {
    case TX_PROF_TAG_OS_TO_DRV:
        
        break;

    case TX_PROF_TAG_DRV_ENQUE:
    	
    	prPktProfile->fgIsValid = FALSE;
        if(wlanTxLifetimeIsTargetMsdu(prAdapter, prMsduInfo)) {
        	
        	prPktProfile->fgIsValid = TRUE;

        	
        	prPktProfile->rHardXmitArrivalTimestamp = GLUE_GET_PKT_ARRIVAL_TIME(prMsduInfo->prPacket);

        	
        	prPktProfile->rEnqueueTimestamp = (OS_SYSTIME)kalGetTimeTick();
        }
        break;

    case TX_PROF_TAG_DRV_DEQUE:
        if(prPktProfile->fgIsValid)
            prPktProfile->rDequeueTimestamp = (OS_SYSTIME)kalGetTimeTick();
        break;
        
    case TX_PROF_TAG_DRV_TX_DONE:
        if(prPktProfile->fgIsValid) {
            BOOLEAN fgPrintCurPkt = FALSE;
            
            prPktProfile->rHifTxDoneTimestamp = (OS_SYSTIME)kalGetTimeTick();
            
			if (fgPrintCurPkt)
				PRINT_PKT_PROFILE(prPktProfile, "C");
            
#if CFG_ENABLE_PER_STA_STATISTICS
            wlanTxLifetimeUpdateStaStats(prAdapter, prMsduInfo);
#endif
        }
        break;
        
    case TX_PROF_TAG_MAC_TX_DONE:
        break;
        
    default:
        break;        
    }
}

VOID
wlanTxProfilingTagPacket(IN P_ADAPTER_T prAdapter,
    IN P_NATIVE_PACKET prPacket, IN ENUM_TX_PROFILING_TAG_T eTag)
{
#if CFG_MET_PACKET_TRACE_SUPPORT
    kalMetTagPacket(prAdapter->prGlueInfo, prPacket, eTag);
#endif
}

VOID
wlanTxProfilingTagMsdu(IN P_ADAPTER_T prAdapter,
    IN P_MSDU_INFO_T prMsduInfo, IN ENUM_TX_PROFILING_TAG_T eTag)
{
    wlanTxLifetimeTagPacket(prAdapter, prMsduInfo, eTag);
    
    wlanTxProfilingTagPacket(prAdapter, prMsduInfo->prPacket, eTag);
}


