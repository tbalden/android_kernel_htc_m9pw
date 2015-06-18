





#include "precomp.h"

#define AIS_ROAMING_CONNECTION_TRIAL_LIMIT  2
#define AIS_JOIN_TIMEOUT                    15

#define CTIA_MAGIC_SSID                     "ctia_test_only_*#*#3646633#*#*"
#define CTIA_MAGIC_SSID_LEN                 30

#define AIS_FSM_STATE_SEARCH_ACTION_PHASE_0	0
#define AIS_FSM_STATE_SEARCH_ACTION_PHASE_1	1
#define AIS_FSM_STATE_SEARCH_ACTION_PHASE_2	2



#if DBG
static PUINT_8 apucDebugAisState[AIS_STATE_NUM] = {
	(PUINT_8) DISP_STRING("AIS_STATE_IDLE"),
	(PUINT_8) DISP_STRING("AIS_STATE_SEARCH"),
	(PUINT_8) DISP_STRING("AIS_STATE_SCAN"),
	(PUINT_8) DISP_STRING("AIS_STATE_ONLINE_SCAN"),
	(PUINT_8) DISP_STRING("AIS_STATE_LOOKING_FOR"),
	(PUINT_8) DISP_STRING("AIS_STATE_WAIT_FOR_NEXT_SCAN"),
	(PUINT_8) DISP_STRING("AIS_STATE_REQ_CHANNEL_JOIN"),
	(PUINT_8) DISP_STRING("AIS_STATE_JOIN"),
	(PUINT_8) DISP_STRING("AIS_STATE_JOIN_FAILURE"),
	(PUINT_8) DISP_STRING("AIS_STATE_IBSS_ALONE"),
	(PUINT_8) DISP_STRING("AIS_STATE_IBSS_MERGE"),
	(PUINT_8) DISP_STRING("AIS_STATE_NORMAL_TR"),
	(PUINT_8) DISP_STRING("AIS_STATE_DISCONNECTING"),
	(PUINT_8) DISP_STRING("AIS_STATE_REQ_REMAIN_ON_CHANNEL"),
	(PUINT_8) DISP_STRING("AIS_STATE_REMAIN_ON_CHANNEL")
};

#endif				


static VOID
aisFsmRunEventScanDoneTimeOut (
	IN P_ADAPTER_T prAdapter,
	ULONG ulParam
	);

VOID aisInitializeConnectionSettings(IN P_ADAPTER_T prAdapter, IN P_REG_INFO_T prRegInfo)
{
	P_CONNECTION_SETTINGS_T prConnSettings;
	UINT_8 aucAnyBSSID[] = BC_BSSID;
	UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;
	int i = 0;

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	COPY_MAC_ADDR(prConnSettings->aucMacAddress, aucZeroMacAddr);

	prConnSettings->ucDelayTimeOfDisconnectEvent = AIS_DELAY_TIME_OF_DISCONNECT_SEC;

	COPY_MAC_ADDR(prConnSettings->aucBSSID, aucAnyBSSID);
	prConnSettings->fgIsConnByBssidIssued = FALSE;

	prConnSettings->fgIsConnReqIssued = FALSE;
	prConnSettings->fgIsDisconnectedByNonRequest = FALSE;

	prConnSettings->ucSSIDLen = 0;

	prConnSettings->eOPMode = NET_TYPE_INFRA;

	prConnSettings->eConnectionPolicy = CONNECT_BY_SSID_BEST_RSSI;

	if (prRegInfo) {
		prConnSettings->ucAdHocChannelNum =
		    (UINT_8) nicFreq2ChannelNum(prRegInfo->u4StartFreq);
		prConnSettings->eAdHocBand = prRegInfo->u4StartFreq < 5000000 ? BAND_2G4 : BAND_5G;
		prConnSettings->eAdHocMode = (ENUM_PARAM_AD_HOC_MODE_T) (prRegInfo->u4AdhocMode);
	}

	prConnSettings->eAuthMode = AUTH_MODE_OPEN;

	prConnSettings->eEncStatus = ENUM_ENCRYPTION_DISABLED;

	prConnSettings->fgIsScanReqIssued = FALSE;

	
	prConnSettings->u2BeaconPeriod = DOT11_BEACON_PERIOD_DEFAULT;

	prConnSettings->u2RTSThreshold = DOT11_RTS_THRESHOLD_DEFAULT;

	prConnSettings->u2DesiredNonHTRateSet = RATE_SET_ALL_ABG;

	 


	
	prConnSettings->bmfgApsdEnAc = PM_UAPSD_NONE;

	secInit(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	prConnSettings->fgIsEnableRoaming = FALSE;
#if CFG_SUPPORT_ROAMING
	if (prRegInfo) {
		prConnSettings->fgIsEnableRoaming =
		    ((prRegInfo->fgDisRoaming > 0) ? (FALSE) : (TRUE));
	}
#endif				

	prConnSettings->fgIsAdHocQoSEnable = FALSE;

#if CFG_SUPPORT_802_11AC
	prConnSettings->eDesiredPhyConfig = PHY_CONFIG_802_11ABGNAC;
#else
	prConnSettings->eDesiredPhyConfig = PHY_CONFIG_802_11ABGN;
#endif

	
	prConnSettings->uc2G4BandwidthMode = CONFIG_BW_20M;
	prConnSettings->uc5GBandwidthMode = CONFIG_BW_20_40M;

	prConnSettings->rRsnInfo.ucElemId = 0x30;
	prConnSettings->rRsnInfo.u2Version = 0x0001;
	prConnSettings->rRsnInfo.u4GroupKeyCipherSuite = 0;
	prConnSettings->rRsnInfo.u4PairwiseKeyCipherSuiteCount = 0;
	for (i = 0; i < MAX_NUM_SUPPORTED_CIPHER_SUITES; i++)
		prConnSettings->rRsnInfo.au4PairwiseKeyCipherSuite[i] = 0;
	prConnSettings->rRsnInfo.u4AuthKeyMgtSuiteCount = 0;
	for (i = 0; i < MAX_NUM_SUPPORTED_AKM_SUITES; i++)
		prConnSettings->rRsnInfo.au4AuthKeyMgtSuite[i] = 0;
	prConnSettings->rRsnInfo.u2RsnCap = 0;
	prConnSettings->rRsnInfo.fgRsnCapPresent = FALSE;

	return;
}				


VOID aisFsmInit(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_SPECIFIC_BSS_INFO_T prAisSpecificBssInfo;

	DEBUGFUNC("aisFsmInit()");
	DBGLOG(SW1, INFO, ("->aisFsmInit()\n"));

	prAdapter->prAisBssInfo = prAisBssInfo =
	    cnmGetBssInfoAndInit(prAdapter, NETWORK_TYPE_AIS, FALSE);
	ASSERT(prAisBssInfo);

	
	COPY_MAC_ADDR(prAdapter->prAisBssInfo->aucOwnMacAddr, prAdapter->rMyMacAddr);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisSpecificBssInfo = &(prAdapter->rWifiVar.rAisSpecificBssInfo);

	
	prAisFsmInfo->ePreviousState = AIS_STATE_IDLE;
	prAisFsmInfo->eCurrentState = AIS_STATE_IDLE;

	prAisFsmInfo->ucAvailableAuthTypes = 0;

	prAisFsmInfo->prTargetBssDesc = (P_BSS_DESC_T) NULL;

	prAisFsmInfo->ucSeqNumOfReqMsg = 0;
	prAisFsmInfo->ucSeqNumOfChReq = 0;
	prAisFsmInfo->ucSeqNumOfScanReq = 0;

	prAisFsmInfo->fgIsInfraChannelFinished = TRUE;
#if CFG_SUPPORT_ROAMING
	prAisFsmInfo->fgIsRoamingScanPending = FALSE;
#endif				
	prAisFsmInfo->fgIsChannelRequested = FALSE;
	prAisFsmInfo->fgIsChannelGranted = FALSE;

	
	cnmTimerInitTimer(prAdapter,
			  &prAisFsmInfo->rBGScanTimer,
			  (PFN_MGMT_TIMEOUT_FUNC) aisFsmRunEventBGSleepTimeOut, (ULONG) NULL);

	cnmTimerInitTimer(prAdapter,
			  &prAisFsmInfo->rIbssAloneTimer,
			  (PFN_MGMT_TIMEOUT_FUNC) aisFsmRunEventIbssAloneTimeOut, (ULONG) NULL);

	cnmTimerInitTimer(prAdapter,
			  &prAisFsmInfo->rIndicationOfDisconnectTimer,
			  (PFN_MGMT_TIMEOUT_FUNC) aisPostponedEventOfDisconnTimeout,
			  (ULONG) NULL);
	
	cnmTimerInitTimer(prAdapter,
					&prAisFsmInfo->rScanDoneTimer,
					(PFN_MGMT_TIMEOUT_FUNC)aisFsmRunEventScanDoneTimeOut,
					(ULONG)NULL);

	cnmTimerInitTimer(prAdapter,
			  &prAisFsmInfo->rJoinTimeoutTimer,
			  (PFN_MGMT_TIMEOUT_FUNC) aisFsmRunEventJoinTimeout, (ULONG) NULL);

	cnmTimerInitTimer(prAdapter,
			  &prAisFsmInfo->rDeauthDoneTimer,
			  (PFN_MGMT_TIMEOUT_FUNC) aisFsmRunEventDeauthTimeout, (ULONG) NULL);

	
	SET_NET_PWR_STATE_IDLE(prAdapter, prAisBssInfo->ucBssIndex);


	
	BSS_INFO_INIT(prAdapter, prAisBssInfo);
	COPY_MAC_ADDR(prAisBssInfo->aucOwnMacAddr, prAdapter->rWifiVar.aucMacAddress);

	
	
	prAisBssInfo->eBand = BAND_2G4;
	prAisBssInfo->ucPrimaryChannel = 1;
	prAisBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;

	
	prAisBssInfo->prBeacon = cnmMgtPktAlloc(prAdapter,
						OFFSET_OF(WLAN_BEACON_FRAME_T,
							  aucInfoElem[0]) + MAX_IE_LENGTH);

	if (prAisBssInfo->prBeacon) {
		prAisBssInfo->prBeacon->eSrc = TX_PACKET_MGMT;
		prAisBssInfo->prBeacon->ucStaRecIndex = 0xFF;	
	} else {
		ASSERT(0);
	}
	
	prAisBssInfo->ucBMCWlanIndex = WTBL_RESERVED_ENTRY;


#if 0
	prAisBssInfo->rPmProfSetupInfo.ucBmpDeliveryAC = PM_UAPSD_ALL;
	prAisBssInfo->rPmProfSetupInfo.ucBmpTriggerAC = PM_UAPSD_ALL;
	prAisBssInfo->rPmProfSetupInfo.ucUapsdSp = WMM_MAX_SP_LENGTH_2;
#else
	if (prAdapter->u4UapsdAcBmp == 0) {
		prAdapter->u4UapsdAcBmp = CFG_INIT_UAPSD_AC_BMP;
		
	}
	prAisBssInfo->rPmProfSetupInfo.ucBmpDeliveryAC = (UINT_8) prAdapter->u4UapsdAcBmp;
	prAisBssInfo->rPmProfSetupInfo.ucBmpTriggerAC = (UINT_8) prAdapter->u4UapsdAcBmp;
	prAisBssInfo->rPmProfSetupInfo.ucUapsdSp = (UINT_8) prAdapter->u4MaxSpLen;
#endif

	
	LINK_INITIALIZE(&prAisFsmInfo->rPendingReqList);

	
	
	
	

	
	

	return;
}				

VOID aisFsmUninit(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_SPECIFIC_BSS_INFO_T prAisSpecificBssInfo;

	DEBUGFUNC("aisFsmUninit()");
	DBGLOG(SW1, INFO, ("->aisFsmUninit()\n"));

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisSpecificBssInfo = &(prAdapter->rWifiVar.rAisSpecificBssInfo);

	
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rBGScanTimer);
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rIbssAloneTimer);
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rIndicationOfDisconnectTimer);
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rJoinTimeoutTimer);
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rScanDoneTimer);

	
	aisFsmFlushRequest(prAdapter);

	
	if (prAisBssInfo->prBeacon) {
		cnmMgtPktFree(prAdapter, prAisBssInfo->prBeacon);
		prAisBssInfo->prBeacon = NULL;
	}
#if CFG_SUPPORT_802_11W
	rsnStopSaQuery(prAdapter);
#endif

	if (prAisBssInfo) {
		cnmFreeBssInfo(prAdapter, prAisBssInfo);
		prAdapter->prAisBssInfo = NULL;
	}

	return;
}				


VOID aisFsmStateInit_JOIN(IN P_ADAPTER_T prAdapter, P_BSS_DESC_T prBssDesc)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_SPECIFIC_BSS_INFO_T prAisSpecificBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_STA_RECORD_T prStaRec;
	P_MSG_JOIN_REQ_T prJoinReqMsg;

	DEBUGFUNC("aisFsmStateInit_JOIN()");

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisSpecificBssInfo = &(prAdapter->rWifiVar.rAisSpecificBssInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	ASSERT(prBssDesc);

	
	prBssDesc->fgIsConnecting = TRUE;


	
	prStaRec = bssCreateStaRecFromBssDesc(prAdapter,
					      STA_TYPE_LEGACY_AP,
					      prAdapter->prAisBssInfo->ucBssIndex, prBssDesc);

	prAisFsmInfo->prTargetStaRec = prStaRec;

	
	if (prStaRec->ucStaState == STA_STATE_1)
		cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);


	
	if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_DISCONNECTED) {

		prStaRec->fgIsReAssoc = FALSE;

		switch (prConnSettings->eAuthMode) {
		case AUTH_MODE_OPEN:	
		case AUTH_MODE_WPA:
		case AUTH_MODE_WPA_PSK:
		case AUTH_MODE_WPA2:
		case AUTH_MODE_WPA2_PSK:
			prAisFsmInfo->ucAvailableAuthTypes = (UINT_8) AUTH_TYPE_OPEN_SYSTEM;
			break;


		case AUTH_MODE_SHARED:
			prAisFsmInfo->ucAvailableAuthTypes = (UINT_8) AUTH_TYPE_SHARED_KEY;
			break;


		case AUTH_MODE_AUTO_SWITCH:
			DBGLOG(AIS, LOUD, ("JOIN INIT: eAuthMode == AUTH_MODE_AUTO_SWITCH\n"));
			prAisFsmInfo->ucAvailableAuthTypes = (UINT_8) (AUTH_TYPE_OPEN_SYSTEM |
								       AUTH_TYPE_SHARED_KEY);
			break;

		default:
			ASSERT(!(prConnSettings->eAuthMode == AUTH_MODE_WPA_NONE));
			DBGLOG(AIS, ERROR,
			       ("JOIN INIT: Auth Algorithm : %d was not supported by JOIN\n",
				prConnSettings->eAuthMode));
			
			return;
		}

		
		prAisSpecificBssInfo->ucRoamingAuthTypes = prAisFsmInfo->ucAvailableAuthTypes;

		prStaRec->ucTxAuthAssocRetryLimit = TX_AUTH_ASSOCI_RETRY_LIMIT;

	} else {
		ASSERT(prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE);
		ASSERT(!prBssDesc->fgIsConnected);

		DBGLOG(AIS, LOUD, ("JOIN INIT: AUTH TYPE = %d for Roaming\n",
				   prAisSpecificBssInfo->ucRoamingAuthTypes));


		prStaRec->fgIsReAssoc = TRUE;	

		
		prAisFsmInfo->ucAvailableAuthTypes = prAisSpecificBssInfo->ucRoamingAuthTypes;

		prStaRec->ucTxAuthAssocRetryLimit = TX_AUTH_ASSOCI_RETRY_LIMIT_FOR_ROAMING;
	}


	
	if (prAisFsmInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_OPEN_SYSTEM) {

		DBGLOG(AIS, LOUD,
		       ("JOIN INIT: Try to do Authentication with AuthType == OPEN_SYSTEM.\n"));
		prAisFsmInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_OPEN_SYSTEM;

		prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_OPEN_SYSTEM;
	} else if (prAisFsmInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_SHARED_KEY) {

		DBGLOG(AIS, LOUD,
		       ("JOIN INIT: Try to do Authentication with AuthType == SHARED_KEY.\n"));

		prAisFsmInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_SHARED_KEY;

		prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_SHARED_KEY;
	} else if (prAisFsmInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_FAST_BSS_TRANSITION) {

		DBGLOG(AIS, LOUD,
		       ("JOIN INIT: Try to do Authentication with AuthType == FAST_BSS_TRANSITION.\n"));

		prAisFsmInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_FAST_BSS_TRANSITION;

		prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_FAST_BSS_TRANSITION;
	} else {
		ASSERT(0);
	}

	
	if (prBssDesc->ucSSIDLen) {
		COPY_SSID(prConnSettings->aucSSID,
			  prConnSettings->ucSSIDLen, prBssDesc->aucSSID, prBssDesc->ucSSIDLen);
	}

	
	prJoinReqMsg =
	    (P_MSG_JOIN_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_JOIN_REQ_T));
	if (!prJoinReqMsg) {

		ASSERT(0);	
		return;
	}

	prJoinReqMsg->rMsgHdr.eMsgId = MID_AIS_SAA_FSM_START;
	prJoinReqMsg->ucSeqNum = ++prAisFsmInfo->ucSeqNumOfReqMsg;
	prJoinReqMsg->prStaRec = prStaRec;

	if (1) {
		int j;
		P_FRAG_INFO_T prFragInfo;
		for (j = 0; j < MAX_NUM_CONCURRENT_FRAGMENTED_MSDUS; j++) {
			prFragInfo = &prStaRec->rFragInfo[j];

			if (prFragInfo->pr1stFrag) {
				
				prFragInfo->pr1stFrag = (P_SW_RFB_T) NULL;
			}
		}
	}

	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prJoinReqMsg, MSG_SEND_METHOD_BUF);

	return;
}				


BOOLEAN aisFsmStateInit_RetryJOIN(IN P_ADAPTER_T prAdapter, P_STA_RECORD_T prStaRec)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_JOIN_REQ_T prJoinReqMsg;

	DEBUGFUNC("aisFsmStateInit_RetryJOIN()");

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	
	if (!prAisFsmInfo->ucAvailableAuthTypes)
		return FALSE;


	if (prAisFsmInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_SHARED_KEY) {

		DBGLOG(AIS, INFO,
		       ("RETRY JOIN INIT: Retry Authentication with AuthType == SHARED_KEY.\n"));

		prAisFsmInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_SHARED_KEY;

		prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_SHARED_KEY;
	} else {
		DBGLOG(AIS, ERROR,
		       ("RETRY JOIN INIT: Retry Authentication with Unexpected AuthType.\n"));
		ASSERT(0);
	}

	prAisFsmInfo->ucAvailableAuthTypes = 0;	

	
	prJoinReqMsg =
	    (P_MSG_JOIN_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_JOIN_REQ_T));
	if (!prJoinReqMsg) {

		ASSERT(0);	
		return FALSE;
	}

	prJoinReqMsg->rMsgHdr.eMsgId = MID_AIS_SAA_FSM_START;
	prJoinReqMsg->ucSeqNum = ++prAisFsmInfo->ucSeqNumOfReqMsg;
	prJoinReqMsg->prStaRec = prStaRec;

	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prJoinReqMsg, MSG_SEND_METHOD_BUF);

	return TRUE;

}				


#if CFG_SUPPORT_ADHOC
VOID aisFsmStateInit_IBSS_ALONE(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prAisBssInfo;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prAisBssInfo = prAdapter->prAisBssInfo;

	
	if (prAisBssInfo->fgIsBeaconActivated) {

		
#if !CFG_SLT_SUPPORT
		cnmTimerStartTimer(prAdapter,
				   &prAisFsmInfo->rIbssAloneTimer,
				   SEC_TO_MSEC(AIS_IBSS_ALONE_TIMEOUT_SEC));
#endif
	}

	aisFsmCreateIBSS(prAdapter);

	return;
}				


VOID aisFsmStateInit_IBSS_MERGE(IN P_ADAPTER_T prAdapter, P_BSS_DESC_T prBssDesc)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prAisBssInfo;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;

	ASSERT(prBssDesc);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prAisBssInfo = prAdapter->prAisBssInfo;

	
	prBssDesc->fgIsConnecting = FALSE;
	prBssDesc->fgIsConnected = TRUE;

	
	prStaRec = bssCreateStaRecFromBssDesc(prAdapter,
					      STA_TYPE_ADHOC_PEER,
					      prAdapter->prAisBssInfo->ucBssIndex, prBssDesc);

	prStaRec->fgIsMerging = TRUE;

	prAisFsmInfo->prTargetStaRec = prStaRec;

	
	cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);

	
	aisFsmMergeIBSS(prAdapter, prStaRec);

	return;
}				

#endif				


VOID aisFsmStateAbort_JOIN(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_JOIN_ABORT_T prJoinAbortMsg;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	
	prJoinAbortMsg =
	    (P_MSG_JOIN_ABORT_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_JOIN_ABORT_T));
	if (!prJoinAbortMsg) {

		ASSERT(0);	
		return;
	}

	prJoinAbortMsg->rMsgHdr.eMsgId = MID_AIS_SAA_FSM_ABORT;
	prJoinAbortMsg->ucSeqNum = prAisFsmInfo->ucSeqNumOfReqMsg;
	prJoinAbortMsg->prStaRec = prAisFsmInfo->prTargetStaRec;

	scanRemoveConnFlagOfBssDescByBssid(prAdapter, prAisFsmInfo->prTargetStaRec->aucMacAddr);

	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prJoinAbortMsg, MSG_SEND_METHOD_BUF);

	
	aisFsmReleaseCh(prAdapter);

	
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rJoinTimeoutTimer);

	
	prAisFsmInfo->fgIsInfraChannelFinished = TRUE;

	return;
}				


VOID aisFsmStateAbort_SCAN(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_SCN_SCAN_CANCEL prScanCancelMsg;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

        DBGLOG(AIS, STATE, ("aisFsmStateAbort_SCAN \n"));

	
	prScanCancelMsg =
	    (P_MSG_SCN_SCAN_CANCEL) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
						sizeof(MSG_SCN_SCAN_CANCEL));
	if (!prScanCancelMsg) {

		ASSERT(0);	
		return;
	}

	prScanCancelMsg->rMsgHdr.eMsgId = MID_AIS_SCN_SCAN_CANCEL;
	prScanCancelMsg->ucSeqNum = prAisFsmInfo->ucSeqNumOfScanReq;
	prScanCancelMsg->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;
	prScanCancelMsg->fgIsChannelExt = FALSE;

	
	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prScanCancelMsg, MSG_SEND_METHOD_UNBUF);

	return;
}				


VOID aisFsmStateAbort_NORMAL_TR(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	

	
	aisFsmReleaseCh(prAdapter);

	
	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rJoinTimeoutTimer);

	
	prAisFsmInfo->fgIsInfraChannelFinished = TRUE;

	return;
}				


#if CFG_SUPPORT_ADHOC
VOID aisFsmStateAbort_IBSS(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_DESC_T prBssDesc;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	
	if (prAisFsmInfo->prTargetStaRec) {
		prBssDesc = scanSearchBssDescByTA(prAdapter,
						  prAisFsmInfo->prTargetStaRec->aucMacAddr);

		if (prBssDesc) {
			prBssDesc->fgIsConnected = FALSE;
			prBssDesc->fgIsConnecting = FALSE;
		}
	}
	
	aisFsmReleaseCh(prAdapter);

	return;
}
#endif				


VOID aisFsmSteps(IN P_ADAPTER_T prAdapter, ENUM_AIS_STATE_T eNextState)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_DESC_T prBssDesc;
	P_MSG_CH_REQ_T prMsgChReq;
	P_MSG_SCN_SCAN_REQ_V2 prScanReqMsg;
	P_AIS_REQ_HDR_T prAisReq;
	ENUM_BAND_T eBand;
	UINT_8 ucChannel;
	UINT_16 u2ScanIELen;

	BOOLEAN fgIsTransition = (BOOLEAN) FALSE;

	DEBUGFUNC("aisFsmSteps()");

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	do {

		
		prAisFsmInfo->ePreviousState = prAisFsmInfo->eCurrentState;

#if DBG
		DBGLOG(AIS, STATE, ("TRANSITION: [%s] -> [%s]\n",
				    apucDebugAisState[prAisFsmInfo->eCurrentState],
				    apucDebugAisState[eNextState]));
#else
		DBGLOG(AIS, STATE, ("[%d] TRANSITION: [%d] -> [%d]\n",
				    DBG_AIS_IDX, prAisFsmInfo->eCurrentState, eNextState));
#endif
		
		prAisFsmInfo->eCurrentState = eNextState;

		fgIsTransition = (BOOLEAN) FALSE;

		
		switch (prAisFsmInfo->eCurrentState) {
		case AIS_STATE_IDLE:

			prAisReq = aisFsmGetNextRequest(prAdapter);
			cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rScanDoneTimer);

			if (prAisReq == NULL || prAisReq->eReqType == AIS_REQUEST_RECONNECT) {
				if (prConnSettings->fgIsConnReqIssued == TRUE &&
				    prConnSettings->fgIsDisconnectedByNonRequest == FALSE) {

					prAisFsmInfo->fgTryScan = TRUE;

					SET_NET_ACTIVE(prAdapter,
						       prAdapter->prAisBssInfo->ucBssIndex);
					SET_NET_PWR_STATE_ACTIVE(prAdapter,
								 prAdapter->prAisBssInfo->
								 ucBssIndex);

					
					nicActivateNetwork(prAdapter,
							   prAdapter->prAisBssInfo->ucBssIndex);
                                        prAisBssInfo->fgIsNetRequestInActive = FALSE;

					
					prAisFsmInfo->ucConnTrialCount = 0;

					eNextState = AIS_STATE_SEARCH;
					fgIsTransition = TRUE;
				} else {
					SET_NET_PWR_STATE_IDLE(prAdapter,
							       prAdapter->prAisBssInfo->ucBssIndex);

					
#if CFG_SUPPORT_PNO
                                prAisBssInfo->fgIsNetRequestInActive = TRUE;
                                if(prAisBssInfo->fgIsPNOEnable){
                                    DBGLOG(BSS, INFO, ("[wlan index][Network]=%d fgIsPNOEnable && OP_MODE_INFRASTRUCTURE, KEEP ACTIVE \n", prAisBssInfo->ucBssIndex, 1));
                                }
                                else
#endif                                           
                                { 
					    UNSET_NET_ACTIVE(prAdapter,
							     prAdapter->prAisBssInfo->ucBssIndex);
                                            nicDeactivateNetwork(prAdapter, 
                                                             prAdapter->prAisBssInfo->ucBssIndex);
                                }

					
					if (prAisReq && (aisFsmIsRequestPending
						    (prAdapter, AIS_REQUEST_SCAN, TRUE) == TRUE)) {
						wlanClearScanningResult(prAdapter);
						eNextState = AIS_STATE_SCAN;

						fgIsTransition = TRUE;
					}
				}

				if (prAisReq) {
					
					cnmMemFree(prAdapter, prAisReq);
				}
			} else if (prAisReq->eReqType == AIS_REQUEST_SCAN) {
#if CFG_SUPPORT_ROAMING
				prAisFsmInfo->fgIsRoamingScanPending = FALSE;
#endif				
				wlanClearScanningResult(prAdapter);

				eNextState = AIS_STATE_SCAN;
				fgIsTransition = TRUE;

				
				cnmMemFree(prAdapter, prAisReq);
			} else if (prAisReq->eReqType == AIS_REQUEST_ROAMING_CONNECT
				   || prAisReq->eReqType == AIS_REQUEST_ROAMING_SEARCH) {
				
				
				cnmMemFree(prAdapter, prAisReq);
			} else if (prAisReq->eReqType == AIS_REQUEST_REMAIN_ON_CHANNEL) {
				eNextState = AIS_STATE_REQ_REMAIN_ON_CHANNEL;
				fgIsTransition = TRUE;

				
				cnmMemFree(prAdapter, prAisReq);
			}

			prAisFsmInfo->u4SleepInterval = AIS_BG_SCAN_INTERVAL_MIN_SEC;

			break;

		case AIS_STATE_SEARCH:
			
#if CFG_SLT_SUPPORT
			prBssDesc = prAdapter->rWifiVar.rSltInfo.prPseudoBssDesc;
#else
			prBssDesc =
			    scanSearchBssDescByPolicy(prAdapter,
						      prAdapter->prAisBssInfo->ucBssIndex);
#endif

			
			if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED) {
				if (prAisFsmInfo->ucConnTrialCount >
				    AIS_ROAMING_CONNECTION_TRIAL_LIMIT) {
#if CFG_SUPPORT_ROAMING
					roamingFsmRunEventFail(prAdapter,
							       ROAMING_FAIL_REASON_CONNLIMIT);
#endif				
					
					prAisFsmInfo->ucConnTrialCount = 0;

					
					prConnSettings->fgIsConnReqIssued = FALSE;

					eNextState = AIS_STATE_NORMAL_TR;
					fgIsTransition = TRUE;

					break;
				}
			}
			
			if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_DISCONNECTED) {

				
				if (prBssDesc) {

					
					
					prAisBssInfo->u4RsnSelectedGroupCipher =
					    prBssDesc->u4RsnSelectedGroupCipher;
					prAisBssInfo->u4RsnSelectedPairwiseCipher =
					    prBssDesc->u4RsnSelectedPairwiseCipher;
					prAisBssInfo->u4RsnSelectedAKMSuite =
					    prBssDesc->u4RsnSelectedAKMSuite;

					
					if (prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE) {

						prAisBssInfo->eCurrentOPMode =
						    OP_MODE_INFRASTRUCTURE;

						
						prAisFsmInfo->prTargetBssDesc = prBssDesc;

						
						eNextState = AIS_STATE_REQ_CHANNEL_JOIN;
						fgIsTransition = TRUE;

						
						prAisFsmInfo->ucConnTrialCount++;
					}
#if CFG_SUPPORT_ADHOC
					else if (prBssDesc->eBSSType == BSS_TYPE_IBSS) {

						prAisBssInfo->eCurrentOPMode = OP_MODE_IBSS;

						
						prAisFsmInfo->prTargetBssDesc = prBssDesc;

						eNextState = AIS_STATE_IBSS_MERGE;
						fgIsTransition = TRUE;
					}
#endif				
					else {
						ASSERT(0);
						eNextState = AIS_STATE_WAIT_FOR_NEXT_SCAN;
						fgIsTransition = TRUE;
					}
				}
				
				else {

					
					if (prConnSettings->eOPMode == NET_TYPE_INFRA)
						prAisFsmInfo->ucConnTrialCount++;

					
					if (prAisFsmInfo->fgTryScan) {
						eNextState = AIS_STATE_LOOKING_FOR;

						fgIsTransition = TRUE;
					}
					
					else {
						eNextState = aisFsmStateSearchAction(prAdapter,
										AIS_FSM_STATE_SEARCH_ACTION_PHASE_0);
						fgIsTransition = TRUE;
					}
				}
			}
			
			else {	

				
				if (prAisBssInfo->ucReasonOfDisconnect != DISCONNECT_REASON_CODE_REASSOCIATION &&
					((!prBssDesc) || 
					(prBssDesc->eBSSType != BSS_TYPE_INFRASTRUCTURE) ||	
					(prBssDesc->fgIsConnected) ||	
					(EQUAL_MAC_ADDR(prBssDesc->aucBSSID, prAisBssInfo->aucBSSID))) ) {
#if DBG
					if ((prBssDesc) && (prBssDesc->fgIsConnected)) {
						ASSERT(EQUAL_MAC_ADDR
						       (prBssDesc->aucBSSID,
							prAisBssInfo->aucBSSID));
					}
#endif				
					
					
#if CFG_SUPPORT_ROAMING
					roamingFsmRunEventFail(prAdapter,
							       ROAMING_FAIL_REASON_NOCANDIDATE);
#endif				

					
					eNextState = AIS_STATE_NORMAL_TR;
					fgIsTransition = TRUE;
				}
				
				else {
					if (prBssDesc == NULL) {
						fgIsTransition = TRUE;
						eNextState = aisFsmStateSearchAction(prAdapter,
										AIS_FSM_STATE_SEARCH_ACTION_PHASE_1);
					} else {
						aisFsmStateSearchAction(prAdapter, AIS_FSM_STATE_SEARCH_ACTION_PHASE_2);
						
						prAisFsmInfo->prTargetBssDesc = prBssDesc;

						
						prAisFsmInfo->ucConnTrialCount++;

						
						eNextState = AIS_STATE_REQ_CHANNEL_JOIN;
						fgIsTransition = TRUE;
					}
				}
			}

			break;

		case AIS_STATE_WAIT_FOR_NEXT_SCAN:

			DBGLOG(AIS, LOUD,
			       ("SCAN: Idle Begin - Current Time = %u\n", kalGetTimeTick()));

			cnmTimerStartTimer(prAdapter,
					   &prAisFsmInfo->rBGScanTimer,
					   SEC_TO_MSEC(prAisFsmInfo->u4SleepInterval));

			SET_NET_PWR_STATE_IDLE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

			if (prAisFsmInfo->u4SleepInterval < AIS_BG_SCAN_INTERVAL_MAX_SEC)
				prAisFsmInfo->u4SleepInterval <<= 1;

			break;

		case AIS_STATE_SCAN:
		case AIS_STATE_ONLINE_SCAN:
		case AIS_STATE_LOOKING_FOR:

			if (!IS_NET_ACTIVE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex)) {
				SET_NET_ACTIVE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

				
				nicActivateNetwork(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);
                                prAisBssInfo->fgIsNetRequestInActive = FALSE;
			}

			
			if (prAisFsmInfo->u4ScanIELength > 0) {
				u2ScanIELen = (UINT_16) prAisFsmInfo->u4ScanIELength;
			} else {
#if CFG_SUPPORT_WPS2
				u2ScanIELen = prAdapter->prGlueInfo->u2WSCIELen;
#else
				u2ScanIELen = 0;
#endif
			}

			prScanReqMsg = (P_MSG_SCN_SCAN_REQ_V2) cnmMemAlloc(prAdapter,
									   RAM_TYPE_MSG,
									   OFFSET_OF
									   (MSG_SCN_SCAN_REQ_V2,
									    aucIE) + u2ScanIELen);
			if (!prScanReqMsg) {
				ASSERT(0);	
				return;
			}

			prScanReqMsg->rMsgHdr.eMsgId = MID_AIS_SCN_SCAN_REQ_V2;
			prScanReqMsg->ucSeqNum = ++prAisFsmInfo->ucSeqNumOfScanReq;
			prScanReqMsg->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;

#if CFG_SUPPORT_RDD_TEST_MODE
			prScanReqMsg->eScanType = SCAN_TYPE_PASSIVE_SCAN;
#else
			if (prAisFsmInfo->eCurrentState == AIS_STATE_SCAN
			    || prAisFsmInfo->eCurrentState == AIS_STATE_ONLINE_SCAN) {
				if (prAisFsmInfo->ucScanSSIDNum == 0) {
#if CFG_SUPPORT_AIS_PASSIVE_SCAN
					prScanReqMsg->eScanType = SCAN_TYPE_PASSIVE_SCAN;

					prScanReqMsg->ucSSIDType = 0;
					prScanReqMsg->ucSSIDNum = 0;
#else
					prScanReqMsg->eScanType = SCAN_TYPE_ACTIVE_SCAN;

					prScanReqMsg->ucSSIDType = SCAN_REQ_SSID_WILDCARD;
					prScanReqMsg->ucSSIDNum = 0;
#endif
				} else if (prAisFsmInfo->ucScanSSIDNum == 1
					   && prAisFsmInfo->arScanSSID[0].u4SsidLen == 0) {
					prScanReqMsg->eScanType = SCAN_TYPE_ACTIVE_SCAN;

					prScanReqMsg->ucSSIDType = SCAN_REQ_SSID_WILDCARD;
					prScanReqMsg->ucSSIDNum = 0;
				} else {
					prScanReqMsg->eScanType = SCAN_TYPE_ACTIVE_SCAN;

					prScanReqMsg->ucSSIDType = SCAN_REQ_SSID_SPECIFIED;
					prScanReqMsg->ucSSIDNum = prAisFsmInfo->ucScanSSIDNum;
					prScanReqMsg->prSsid = prAisFsmInfo->arScanSSID;
				}
			} else {
				prScanReqMsg->eScanType = SCAN_TYPE_ACTIVE_SCAN;

				COPY_SSID(prAisFsmInfo->rRoamingSSID.aucSsid,
					  prAisFsmInfo->rRoamingSSID.u4SsidLen,
					  prConnSettings->aucSSID, prConnSettings->ucSSIDLen);

				
				prScanReqMsg->ucSSIDType = SCAN_REQ_SSID_SPECIFIED;
				prScanReqMsg->ucSSIDNum = 1;
				prScanReqMsg->prSsid = &(prAisFsmInfo->rRoamingSSID);
			}
#endif

			
			prScanReqMsg->u2ProbeDelay = 0;
			prScanReqMsg->u2ChannelDwellTime = 0;
			prScanReqMsg->u2TimeoutValue = 0;

			
			if (cnmAisInfraChannelFixed(prAdapter, &eBand, &ucChannel) == TRUE) {
				prScanReqMsg->eScanChannel = SCAN_CHANNEL_SPECIFIED;
				prScanReqMsg->ucChannelListNum = 1;
				prScanReqMsg->arChnlInfoList[0].eBand = eBand;
				prScanReqMsg->arChnlInfoList[0].ucChannelNum = ucChannel;
			} else if (prAdapter->aePreferBand[prAdapter->prAisBssInfo->ucBssIndex] ==
				   BAND_NULL) {
				if (prAdapter->fgEnable5GBand == TRUE)
					prScanReqMsg->eScanChannel = SCAN_CHANNEL_FULL;
				else
					prScanReqMsg->eScanChannel = SCAN_CHANNEL_2G4;

			} else if (prAdapter->aePreferBand[prAdapter->prAisBssInfo->ucBssIndex] ==
				   BAND_2G4) {
				prScanReqMsg->eScanChannel = SCAN_CHANNEL_2G4;
			} else if (prAdapter->aePreferBand[prAdapter->prAisBssInfo->ucBssIndex] ==
				   BAND_5G) {
				prScanReqMsg->eScanChannel = SCAN_CHANNEL_5G;
			} else {
				prScanReqMsg->eScanChannel = SCAN_CHANNEL_FULL;
				ASSERT(0);
			}

			if (prAisFsmInfo->u4ScanIELength > 0) {
				kalMemCopy(prScanReqMsg->aucIE, prAisFsmInfo->aucScanIEBuf,
					   prAisFsmInfo->u4ScanIELength);
			} else {
#if CFG_SUPPORT_WPS2
				if (prAdapter->prGlueInfo->u2WSCIELen > 0) {
					kalMemCopy(prScanReqMsg->aucIE,
						   &prAdapter->prGlueInfo->aucWSCIE,
						   prAdapter->prGlueInfo->u2WSCIELen);
				}
			}
#endif

			prScanReqMsg->u2IELen = u2ScanIELen;

			mboxSendMsg(prAdapter,
				    MBOX_ID_0, (P_MSG_HDR_T) prScanReqMsg, MSG_SEND_METHOD_BUF);

			prAisFsmInfo->fgTryScan = FALSE;	

			break;

		case AIS_STATE_REQ_CHANNEL_JOIN:
			
			prMsgChReq =
			    (P_MSG_CH_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
							 sizeof(MSG_CH_REQ_T));
			if (!prMsgChReq) {
				ASSERT(0);	
				return;
			}

			prMsgChReq->rMsgHdr.eMsgId = MID_MNY_CNM_CH_REQ;
			prMsgChReq->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;
			prMsgChReq->ucTokenID = ++prAisFsmInfo->ucSeqNumOfChReq;
			prMsgChReq->eReqType = CH_REQ_TYPE_JOIN;
			prMsgChReq->u4MaxInterval = AIS_JOIN_CH_REQUEST_INTERVAL;
			prMsgChReq->ucPrimaryChannel = prAisFsmInfo->prTargetBssDesc->ucChannelNum;
			prMsgChReq->eRfSco = prAisFsmInfo->prTargetBssDesc->eSco;
			prMsgChReq->eRfBand = prAisFsmInfo->prTargetBssDesc->eBand;

			
			prMsgChReq->eRfChannelWidth = prAisFsmInfo->prTargetBssDesc->eChannelWidth;
			prMsgChReq->ucRfCenterFreqSeg1 =
			    prAisFsmInfo->prTargetBssDesc->ucCenterFreqS1;
			prMsgChReq->ucRfCenterFreqSeg2 =
			    prAisFsmInfo->prTargetBssDesc->ucCenterFreqS2;

			mboxSendMsg(prAdapter,
				    MBOX_ID_0, (P_MSG_HDR_T) prMsgChReq, MSG_SEND_METHOD_BUF);

			prAisFsmInfo->fgIsChannelRequested = TRUE;
			break;

		case AIS_STATE_JOIN:
			aisFsmStateInit_JOIN(prAdapter, prAisFsmInfo->prTargetBssDesc);
			break;

		case AIS_STATE_JOIN_FAILURE:
			prConnSettings->fgIsDisconnectedByNonRequest = TRUE;

			nicMediaJoinFailure(prAdapter, prAdapter->prAisBssInfo->ucBssIndex,
					    WLAN_STATUS_JOIN_TIMEOUT);

			eNextState = AIS_STATE_IDLE;
			fgIsTransition = TRUE;

			break;

#if CFG_SUPPORT_ADHOC
		case AIS_STATE_IBSS_ALONE:
			aisFsmStateInit_IBSS_ALONE(prAdapter);
			break;

		case AIS_STATE_IBSS_MERGE:
			aisFsmStateInit_IBSS_MERGE(prAdapter, prAisFsmInfo->prTargetBssDesc);
			break;
#endif				

		case AIS_STATE_NORMAL_TR:
			if (prAisFsmInfo->fgIsInfraChannelFinished == FALSE) {
				
			} else {
				
				if (aisFsmIsRequestPending(prAdapter, AIS_REQUEST_SCAN, TRUE) ==
				    TRUE) {
					wlanClearScanningResult(prAdapter);
					eNextState = AIS_STATE_ONLINE_SCAN;
					fgIsTransition = TRUE;
				}
				
				else if (aisFsmIsRequestPending
					 (prAdapter, AIS_REQUEST_ROAMING_SEARCH, TRUE) == TRUE) {
					eNextState = AIS_STATE_LOOKING_FOR;
					fgIsTransition = TRUE;
				}
				
				else if (aisFsmIsRequestPending
					 (prAdapter, AIS_REQUEST_ROAMING_CONNECT, TRUE) == TRUE) {
					eNextState = AIS_STATE_SEARCH;
					fgIsTransition = TRUE;
				} else
				    if (aisFsmIsRequestPending
					(prAdapter, AIS_REQUEST_REMAIN_ON_CHANNEL, TRUE) == TRUE) {
					eNextState = AIS_STATE_REQ_REMAIN_ON_CHANNEL;
					fgIsTransition = TRUE;
				}
			}

			break;

		case AIS_STATE_DISCONNECTING:
			
			authSendDeauthFrame(prAdapter,
					    prAisBssInfo,
					    prAisBssInfo->prStaRecOfAP,
					    (P_SW_RFB_T) NULL,
					    REASON_CODE_DEAUTH_LEAVING_BSS, aisDeauthXmitComplete);
			cnmTimerStartTimer(prAdapter, &prAisFsmInfo->rDeauthDoneTimer, 100);
			break;

		case AIS_STATE_REQ_REMAIN_ON_CHANNEL:
			
			prMsgChReq =
			    (P_MSG_CH_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
							 sizeof(MSG_CH_REQ_T));
			if (!prMsgChReq) {
				ASSERT(0);	
				return;
			}

			
			kalMemZero(prMsgChReq, sizeof(MSG_CH_REQ_T));

			
			prMsgChReq->rMsgHdr.eMsgId = MID_MNY_CNM_CH_REQ;
			prMsgChReq->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;
			prMsgChReq->ucTokenID = ++prAisFsmInfo->ucSeqNumOfChReq;
			prMsgChReq->eReqType = CH_REQ_TYPE_JOIN;
			prMsgChReq->u4MaxInterval = prAisFsmInfo->rChReqInfo.u4DurationMs;
			prMsgChReq->ucPrimaryChannel = prAisFsmInfo->rChReqInfo.ucChannelNum;
			prMsgChReq->eRfSco = prAisFsmInfo->rChReqInfo.eSco;
			prMsgChReq->eRfBand = prAisFsmInfo->rChReqInfo.eBand;

			mboxSendMsg(prAdapter,
				    MBOX_ID_0, (P_MSG_HDR_T) prMsgChReq, MSG_SEND_METHOD_BUF);

			prAisFsmInfo->fgIsChannelRequested = TRUE;

			break;

		case AIS_STATE_REMAIN_ON_CHANNEL:
			SET_NET_ACTIVE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

			
			nicActivateNetwork(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);
                        prAisBssInfo->fgIsNetRequestInActive = FALSE;
			break;

		default:
			ASSERT(0);	
			break;

		}
	} while (fgIsTransition);

	return;

}				

enum _ENUM_AIS_STATE_T
aisFsmStateSearchAction(IN struct _ADAPTER_T *prAdapter, UINT_8 ucPhase)
{
	struct _CONNECTION_SETTINGS_T *prConnSettings;
	struct _BSS_INFO_T *prAisBssInfo;
	struct _AIS_FSM_INFO_T *prAisFsmInfo;
	struct _BSS_DESC_T *prBssDesc;
	enum _ENUM_AIS_STATE_T eState = AIS_STATE_IDLE;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

#if CFG_SLT_SUPPORT
	prBssDesc = prAdapter->rWifiVar.rSltInfo.prPseudoBssDesc;
#else
	prBssDesc =
		scanSearchBssDescByPolicy(prAdapter,
					  prAdapter->prAisBssInfo->ucBssIndex);
#endif


	if (ucPhase == AIS_FSM_STATE_SEARCH_ACTION_PHASE_0) {
		if (prConnSettings->eOPMode == NET_TYPE_INFRA) {

			
			
			aisFsmInsertRequest(prAdapter,
					    AIS_REQUEST_RECONNECT);
			eState = AIS_STATE_IDLE;
		}
#if CFG_SUPPORT_ADHOC
		else if ((prConnSettings->eOPMode == NET_TYPE_IBSS)
			 || (prConnSettings->eOPMode ==
			     NET_TYPE_AUTO_SWITCH)
			 || (prConnSettings->eOPMode ==
			     NET_TYPE_DEDICATED_IBSS)) {
			prAisBssInfo->eCurrentOPMode = OP_MODE_IBSS;
			prAisFsmInfo->prTargetBssDesc = NULL;
			eState = AIS_STATE_IBSS_ALONE;
		}
#endif				
		else {
			ASSERT(0);
			eState = AIS_STATE_WAIT_FOR_NEXT_SCAN;
		}
	} else if (ucPhase == AIS_FSM_STATE_SEARCH_ACTION_PHASE_1) {
		
		if (prConnSettings->eOPMode == NET_TYPE_INFRA)
			prAisFsmInfo->ucConnTrialCount++;
		
		if (prAisFsmInfo->fgTryScan)
			eState = AIS_STATE_LOOKING_FOR;

		
		else {
			if (prConnSettings->eOPMode ==
			    NET_TYPE_INFRA) {

				
				
				aisFsmInsertRequest(prAdapter,
						    AIS_REQUEST_RECONNECT);

				eState = AIS_STATE_IDLE;
			}
#if CFG_SUPPORT_ADHOC
			else if ((prConnSettings->eOPMode ==
				  NET_TYPE_IBSS)
				 || (prConnSettings->eOPMode ==
				     NET_TYPE_AUTO_SWITCH)
				 || (prConnSettings->eOPMode ==
				     NET_TYPE_DEDICATED_IBSS)) {

				prAisBssInfo->eCurrentOPMode =
				    OP_MODE_IBSS;
				prAisFsmInfo->prTargetBssDesc =
				    NULL;

				eState = AIS_STATE_IBSS_ALONE;
			}
#endif				
			else {
				ASSERT(0);
				eState =
				    AIS_STATE_WAIT_FOR_NEXT_SCAN;
			}
		}
	} else {
#if DBG
		if (prAisBssInfo->ucReasonOfDisconnect !=
		    DISCONNECT_REASON_CODE_REASSOCIATION) {
			ASSERT(UNEQUAL_MAC_ADDR
			       (prBssDesc->aucBSSID,
				prAisBssInfo->aucBSSID));
		}
#endif				
	}
	return eState;
}


VOID aisFsmRunEventScanDone(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_MSG_SCN_SCAN_DONE prScanDoneMsg;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;
	UINT_8 ucSeqNumOfCompMsg;
	P_CONNECTION_SETTINGS_T prConnSettings;

	DEBUGFUNC("aisFsmRunEventScanDone()");

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);
	DBGLOG(AIS, INFO, ("ScanDone\n"));

	DBGLOG(AIS, LOUD, ("EVENT-SCAN DONE: Current Time = %u\n", kalGetTimeTick()));

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	prScanDoneMsg = (P_MSG_SCN_SCAN_DONE) prMsgHdr;
	ASSERT(prScanDoneMsg->ucBssIndex == prAdapter->prAisBssInfo->ucBssIndex);

	ucSeqNumOfCompMsg = prScanDoneMsg->ucSeqNum;
	cnmMemFree(prAdapter, prMsgHdr);

	eNextState = prAisFsmInfo->eCurrentState;

	if (ucSeqNumOfCompMsg != prAisFsmInfo->ucSeqNumOfScanReq) {
		DBGLOG(AIS, WARN, ("SEQ NO of AIS SCN DONE MSG is not matched.\n"));
	} else {
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rScanDoneTimer);
		switch (prAisFsmInfo->eCurrentState) {
		case AIS_STATE_SCAN:
			prConnSettings->fgIsScanReqIssued = FALSE;

			
			prAisFsmInfo->u4ScanIELength = 0;

			kalScanDone(prAdapter->prGlueInfo, KAL_NETWORK_TYPE_AIS_INDEX,
				    WLAN_STATUS_SUCCESS);
			eNextState = AIS_STATE_IDLE;
#if CFG_SUPPORT_AGPS_ASSIST
			scanReportScanResultToAgps(prAdapter);
#endif

			break;

		case AIS_STATE_ONLINE_SCAN:
			prConnSettings->fgIsScanReqIssued = FALSE;

			
			prAisFsmInfo->u4ScanIELength = 0;

			kalScanDone(prAdapter->prGlueInfo, KAL_NETWORK_TYPE_AIS_INDEX,
				    WLAN_STATUS_SUCCESS);
#if CFG_SUPPORT_ROAMING
			eNextState = aisFsmRoamingScanResultsUpdate(prAdapter);
#else
			eNextState = AIS_STATE_NORMAL_TR;
#endif				
#if CFG_SUPPORT_AGPS_ASSIST
			scanReportScanResultToAgps(prAdapter);
#endif

			break;

		case AIS_STATE_LOOKING_FOR:
#if CFG_SUPPORT_ROAMING
			eNextState = aisFsmRoamingScanResultsUpdate(prAdapter);
#else
			eNextState = AIS_STATE_SEARCH;
#endif				
			break;

		default:
			break;

		}
	}

	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);


	return;
}				


VOID aisFsmRunEventAbort(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_MSG_AIS_ABORT_T prAisAbortMsg;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	UINT_8 ucReasonOfDisconnect;
	BOOLEAN fgDelayIndication;
	P_CONNECTION_SETTINGS_T prConnSettings;

	DEBUGFUNC("aisFsmRunEventAbort()");

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	prAisAbortMsg = (P_MSG_AIS_ABORT_T) prMsgHdr;
	ucReasonOfDisconnect = prAisAbortMsg->ucReasonOfDisconnect;
	fgDelayIndication = prAisAbortMsg->fgDelayIndication;

	cnmMemFree(prAdapter, prMsgHdr);

#if DBG
	DBGLOG(AIS, LOUD, ("EVENT-ABORT: Current State %s\n",
			   apucDebugAisState[prAisFsmInfo->eCurrentState]));
#else
	DBGLOG(AIS, LOUD, ("[%d] EVENT-ABORT: Current State [%d]\n",
			   DBG_AIS_IDX, prAisFsmInfo->eCurrentState));
#endif

	
	GET_CURRENT_SYSTIME(&(prAisFsmInfo->rJoinReqTime));

	
	if (ucReasonOfDisconnect == DISCONNECT_REASON_CODE_DEAUTHENTICATED
	    || ucReasonOfDisconnect == DISCONNECT_REASON_CODE_DISASSOCIATED) {
		prConnSettings->fgIsDisconnectedByNonRequest = TRUE;
	} else {
		prConnSettings->fgIsDisconnectedByNonRequest = FALSE;
	}
	
	
	if (ucReasonOfDisconnect == DISCONNECT_REASON_CODE_REASSOCIATION &&
			prAisFsmInfo->eCurrentState != AIS_STATE_DISCONNECTING) {

	    if(prAisFsmInfo->eCurrentState == AIS_STATE_NORMAL_TR &&
				prAisFsmInfo->fgIsInfraChannelFinished == TRUE) {
			aisFsmSteps(prAdapter, AIS_STATE_SEARCH);
	    }
	    else {
	        aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_SEARCH, TRUE);
	        aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_CONNECT, TRUE);
			aisFsmInsertRequest(prAdapter, AIS_REQUEST_ROAMING_CONNECT);
	    }
		return;
	}

	aisFsmIsRequestPending(prAdapter, AIS_REQUEST_RECONNECT, TRUE);
	aisFsmInsertRequest(prAdapter, AIS_REQUEST_RECONNECT);

	if (prAisFsmInfo->eCurrentState != AIS_STATE_DISCONNECTING) {
		
		aisFsmStateAbort(prAdapter, ucReasonOfDisconnect, fgDelayIndication);
	}

	return;
}				


VOID
aisFsmStateAbort(IN P_ADAPTER_T prAdapter, UINT_8 ucReasonOfDisconnect, BOOLEAN fgDelayIndication)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	BOOLEAN fgIsCheckConnected;

	ASSERT(prAdapter);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	fgIsCheckConnected = FALSE;

	
	prAisBssInfo->ucReasonOfDisconnect = ucReasonOfDisconnect;

	
	switch (prAisFsmInfo->eCurrentState) {
	case AIS_STATE_IDLE:
	case AIS_STATE_SEARCH:
	case AIS_STATE_JOIN_FAILURE:
		break;

	case AIS_STATE_WAIT_FOR_NEXT_SCAN:
		
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rBGScanTimer);

		
		fgIsCheckConnected = TRUE;
		break;

	case AIS_STATE_SCAN:
		
		aisFsmStateAbort_SCAN(prAdapter);

		
		if (aisFsmIsRequestPending(prAdapter, AIS_REQUEST_SCAN, FALSE) == FALSE)
			aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);


		break;

	case AIS_STATE_LOOKING_FOR:
		
		aisFsmStateAbort_SCAN(prAdapter);

		
		fgIsCheckConnected = TRUE;
		break;

	case AIS_STATE_REQ_CHANNEL_JOIN:
		
		aisFsmReleaseCh(prAdapter);

		
		fgIsCheckConnected = TRUE;
		break;

	case AIS_STATE_JOIN:
		
		aisFsmStateAbort_JOIN(prAdapter);

		
		fgIsCheckConnected = TRUE;
		break;

#if CFG_SUPPORT_ADHOC
	case AIS_STATE_IBSS_ALONE:
	case AIS_STATE_IBSS_MERGE:
		aisFsmStateAbort_IBSS(prAdapter);
		break;
#endif				

	case AIS_STATE_ONLINE_SCAN:
		
		aisFsmStateAbort_SCAN(prAdapter);

		
		if (aisFsmIsRequestPending(prAdapter, AIS_REQUEST_SCAN, FALSE) == FALSE)
			aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);


		fgIsCheckConnected = TRUE;
		break;

	case AIS_STATE_NORMAL_TR:
		fgIsCheckConnected = TRUE;
		break;

	case AIS_STATE_DISCONNECTING:
		
		aisFsmStateAbort_NORMAL_TR(prAdapter);

		break;

	case AIS_STATE_REQ_REMAIN_ON_CHANNEL:
		
		aisFsmReleaseCh(prAdapter);
		break;

	case AIS_STATE_REMAIN_ON_CHANNEL:
		
		aisFsmReleaseCh(prAdapter);

		
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rChannelTimeoutTimer);

		break;

	default:
		break;
	}

	if (fgIsCheckConnected && (PARAM_MEDIA_STATE_CONNECTED == prAisBssInfo->eConnectionState)) {

		
		if (prAisBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE &&
		    prAisBssInfo->ucReasonOfDisconnect == DISCONNECT_REASON_CODE_NEW_CONNECTION &&
		    prAisBssInfo->prStaRecOfAP && prAisBssInfo->prStaRecOfAP->fgIsInUse) {
			aisFsmSteps(prAdapter, AIS_STATE_DISCONNECTING);

			return;
		} else {
			
			aisFsmStateAbort_NORMAL_TR(prAdapter);
		}
	}

	aisFsmDisconnect(prAdapter, fgDelayIndication);

	return;

}				


VOID aisFsmRunEventJoinComplete(IN struct _ADAPTER_T *prAdapter, IN struct _MSG_HDR_T *prMsgHdr)
{
	struct _MSG_SAA_FSM_COMP_T *prJoinCompMsg;
	struct _AIS_FSM_INFO_T *prAisFsmInfo;
	enum _ENUM_AIS_STATE_T eNextState;
	struct _SW_RFB_T *prAssocRspSwRfb;

	DEBUGFUNC("aisFsmRunEventJoinComplete()");
	ASSERT(prMsgHdr);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prJoinCompMsg = (struct _MSG_SAA_FSM_COMP_T *) prMsgHdr;
	prAssocRspSwRfb = prJoinCompMsg->prSwRfb;

	eNextState = prAisFsmInfo->eCurrentState;

	
	if (prAisFsmInfo->eCurrentState == AIS_STATE_JOIN) {
		
		if (prJoinCompMsg->ucSeqNum == prAisFsmInfo->ucSeqNumOfReqMsg)
			eNextState = aisFsmJoinCompleteAction(prAdapter, prMsgHdr);
#if DBG
		else {
			DBGLOG(AIS, WARN,
			       ("SEQ NO of AIS JOIN COMP MSG is not matched.\n"));
		}
#endif				
	}
	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);

	if (prAssocRspSwRfb)
		nicRxReturnRFB(prAdapter, prAssocRspSwRfb);

	cnmMemFree(prAdapter, prMsgHdr);

	return;
}				

enum _ENUM_AIS_STATE_T
aisFsmJoinCompleteAction(IN struct _ADAPTER_T *prAdapter, IN struct _MSG_HDR_T *prMsgHdr)
{
	struct _MSG_SAA_FSM_COMP_T *prJoinCompMsg;
	struct _AIS_FSM_INFO_T *prAisFsmInfo;
	enum _ENUM_AIS_STATE_T eNextState;
	struct _STA_RECORD_T *prStaRec;
	struct _SW_RFB_T *prAssocRspSwRfb;
	struct _BSS_INFO_T *prAisBssInfo;
	OS_SYSTIME rCurrentTime;
	UINT_8 aucP2pSsid[] = CTIA_MAGIC_SSID;

	DEBUGFUNC("aisFsmJoinCompleteAction()");

	ASSERT(prMsgHdr);

	GET_CURRENT_SYSTIME(&rCurrentTime);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prJoinCompMsg = (struct _MSG_SAA_FSM_COMP_T *) prMsgHdr;
	prStaRec = prJoinCompMsg->prStaRec;
	prAssocRspSwRfb = prJoinCompMsg->prSwRfb;
	prAisBssInfo = prAdapter->prAisBssInfo;
	eNextState = prAisFsmInfo->eCurrentState;

	do {
		
		if (prJoinCompMsg->rJoinStatus == WLAN_STATUS_SUCCESS) {

			
			prAisFsmInfo->ucConnTrialCount = 0;

			
			if (prAisBssInfo->eConnectionState ==
			    PARAM_MEDIA_STATE_CONNECTED) {

#if CFG_SUPPORT_ROAMING
				
				aisFsmRoamingDisconnectPrevAP(prAdapter, prStaRec);

				
				aisUpdateBssInfoForRoamingAP(prAdapter, prStaRec,
							     prAssocRspSwRfb);
#endif				
			} else {
				
				aisChangeMediaState(prAdapter,
						    PARAM_MEDIA_STATE_CONNECTED);

				
				if ((prAisBssInfo->prStaRecOfAP) &&
				    (prAisBssInfo->prStaRecOfAP != prStaRec) &&
				    (prAisBssInfo->prStaRecOfAP->fgIsInUse)) {

					cnmStaRecChangeState(prAdapter,
							     prAisBssInfo->
							     prStaRecOfAP,
							     STA_STATE_1);
					cnmStaRecFree(prAdapter,
						      prAisBssInfo->prStaRecOfAP);
				}

				
				
				aisUpdateBssInfoForJOIN(prAdapter, prStaRec,
							prAssocRspSwRfb);

				
				cnmStaRecChangeState(prAdapter, prStaRec,
						     STA_STATE_3);

				
				nicUpdateRSSI(prAdapter,
					      prAdapter->prAisBssInfo->ucBssIndex,
					      (INT_8) (RCPI_TO_dBm
						       (prStaRec->ucRCPI)), 0);

				
				
				
				aisIndicationOfMediaStateToHost(prAdapter,
								PARAM_MEDIA_STATE_CONNECTED,
								FALSE);

				if (EQUAL_SSID
				    (aucP2pSsid, CTIA_MAGIC_SSID_LEN,
				     prAisBssInfo->aucSSID,
				     prAisBssInfo->ucSSIDLen)) {
					nicEnterCtiaMode(prAdapter, TRUE,
							 FALSE);
				}
			}

#if CFG_SUPPORT_ROAMING
			
			if (prAdapter->rWifiVar.rConnSettings.eConnectionPolicy != CONNECT_BY_BSSID)
				roamingFsmRunEventStart(prAdapter);
#endif				

			
			eNextState = AIS_STATE_NORMAL_TR;
		}
		
		else {
			
			if (aisFsmStateInit_RetryJOIN(prAdapter, prStaRec) == FALSE) {
				struct _BSS_DESC_T *prBssDesc;

				
				prStaRec->ucJoinFailureCount++;

				
				aisFsmReleaseCh(prAdapter);

				
				cnmTimerStopTimer(prAdapter,
						  &prAisFsmInfo->rJoinTimeoutTimer);

				
				prAisFsmInfo->fgIsInfraChannelFinished = TRUE;

				prBssDesc =
				    scanSearchBssDescByBssid(prAdapter,
							     prStaRec->aucMacAddr);

				if (prBssDesc == NULL)
					break;

				
				

				if (prBssDesc)
					prBssDesc->fgIsConnecting = FALSE;


				
				if (prStaRec != prAisBssInfo->prStaRecOfAP)
					cnmStaRecFree(prAdapter, prStaRec);

				if (prAisBssInfo->eConnectionState ==
				    PARAM_MEDIA_STATE_CONNECTED) {
#if CFG_SUPPORT_ROAMING
					eNextState = AIS_STATE_WAIT_FOR_NEXT_SCAN;
#endif				
				} else
				    if (CHECK_FOR_TIMEOUT
					(rCurrentTime, prAisFsmInfo->rJoinReqTime,
					 SEC_TO_SYSTIME(AIS_JOIN_TIMEOUT))) {
					
					eNextState = AIS_STATE_JOIN_FAILURE;
				} else {
					
					aisFsmInsertRequest(prAdapter,
							    AIS_REQUEST_RECONNECT);

					eNextState = AIS_STATE_IDLE;
				}
			}
		}
	} while (0);
	return eNextState;
}


#if CFG_SUPPORT_ADHOC
VOID aisFsmCreateIBSS(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;

	ASSERT(prAdapter);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	do {
		
		if (prAisFsmInfo->eCurrentState == AIS_STATE_IBSS_ALONE)
			aisUpdateBssInfoForCreateIBSS(prAdapter);

	} while (FALSE);

	return;
}				


VOID aisFsmMergeIBSS(IN P_ADAPTER_T prAdapter, IN P_STA_RECORD_T prStaRec)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;
	P_BSS_INFO_T prAisBssInfo;

	ASSERT(prAdapter);
	ASSERT(prStaRec);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;

	do {

		eNextState = prAisFsmInfo->eCurrentState;

		switch (prAisFsmInfo->eCurrentState) {
		case AIS_STATE_IBSS_MERGE:
			{
				P_BSS_DESC_T prBssDesc;

				
				aisChangeMediaState(prAdapter, PARAM_MEDIA_STATE_CONNECTED);

				
				bssInitializeClientList(prAdapter, prAisBssInfo);

				
				prBssDesc = scanSearchBssDescByBssid(prAdapter, prAisBssInfo->aucBSSID);
				if (prBssDesc != NULL) {
					prBssDesc->fgIsConnecting = FALSE;
					prBssDesc->fgIsConnected = FALSE;
				}
				
				bssAddClient(prAdapter, prAisBssInfo, prStaRec);

				
				cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);
				prStaRec->fgIsMerging = FALSE;

				
				aisUpdateBssInfoForMergeIBSS(prAdapter, prStaRec);

				

				
				aisIndicationOfMediaStateToHost(prAdapter,
								PARAM_MEDIA_STATE_CONNECTED, FALSE);

				
				eNextState = AIS_STATE_NORMAL_TR;

				
				aisFsmReleaseCh(prAdapter);

#if CFG_SLT_SUPPORT
				prAdapter->rWifiVar.rSltInfo.prPseudoStaRec = prStaRec;
#endif
			}
			break;

		default:
			break;
		}

		if (eNextState != prAisFsmInfo->eCurrentState)
			aisFsmSteps(prAdapter, eNextState);


	} while (FALSE);

	return;
}				


VOID aisFsmRunEventFoundIBSSPeer(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_MSG_AIS_IBSS_PEER_FOUND_T prAisIbssPeerFoundMsg;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;
	P_STA_RECORD_T prStaRec;
	P_BSS_INFO_T prAisBssInfo;
	P_BSS_DESC_T prBssDesc;
	BOOLEAN fgIsMergeIn;


	ASSERT(prMsgHdr);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;

	prAisIbssPeerFoundMsg = (P_MSG_AIS_IBSS_PEER_FOUND_T) prMsgHdr;

	ASSERT(prAisIbssPeerFoundMsg->ucBssIndex == prAdapter->prAisBssInfo->ucBssIndex);

	prStaRec = prAisIbssPeerFoundMsg->prStaRec;
	ASSERT(prStaRec);

	fgIsMergeIn = prAisIbssPeerFoundMsg->fgIsMergeIn;

	cnmMemFree(prAdapter, prMsgHdr);


	eNextState = prAisFsmInfo->eCurrentState;
	switch (prAisFsmInfo->eCurrentState) {
	case AIS_STATE_IBSS_ALONE:
		{
			
			if (fgIsMergeIn) {

				
				aisChangeMediaState(prAdapter, PARAM_MEDIA_STATE_CONNECTED);

				
				bssAddClient(prAdapter, prAisBssInfo, prStaRec);

#if CFG_SLT_SUPPORT
				
				prBssDesc = scanSearchBssDescByTA(prAdapter, prStaRec->aucMacAddr);

				if (prBssDesc != NULL) {
					prBssDesc->fgIsConnecting = FALSE;
					prBssDesc->fgIsConnected = TRUE;
				} else {
					ASSERT(0);	
				}

				
				prStaRec->fgIsQoS = TRUE;	
#else
				
				prBssDesc = scanSearchBssDescByBssid(prAdapter, prAisBssInfo->aucBSSID);

				if (prBssDesc != NULL) {
					prBssDesc->fgIsConnecting = FALSE;
					prBssDesc->fgIsConnected = TRUE;
				} else {
					ASSERT(0);	
				}


				
				prStaRec->fgIsQoS = FALSE;	

#endif

				cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);
				prStaRec->fgIsMerging = FALSE;

				
				nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

				
				aisIndicationOfMediaStateToHost(prAdapter,
								PARAM_MEDIA_STATE_CONNECTED, FALSE);

				
				nicPmIndicateBssConnected(prAdapter,
							  prAdapter->prAisBssInfo->ucBssIndex);

				
				eNextState = AIS_STATE_NORMAL_TR;

				
				aisFsmReleaseCh(prAdapter);
			}
			
			else {

				
				prBssDesc = scanSearchBssDescByTA(prAdapter, prStaRec->aucMacAddr);

				prAisFsmInfo->prTargetBssDesc = prBssDesc;

				
				eNextState = AIS_STATE_IBSS_MERGE;
			}
		}
		break;

	case AIS_STATE_NORMAL_TR:
		{

			
			if (fgIsMergeIn) {

				
				bssAddClient(prAdapter, prAisBssInfo, prStaRec);

#if CFG_SLT_SUPPORT
				
				prStaRec->fgIsQoS = TRUE;	
#else
				
				prStaRec->fgIsQoS = FALSE;	
#endif

				cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);
				prStaRec->fgIsMerging = FALSE;

			}
			
			else {

				
				prBssDesc = scanSearchBssDescByTA(prAdapter, prStaRec->aucMacAddr);

				prAisFsmInfo->prTargetBssDesc = prBssDesc;

				
				eNextState = AIS_STATE_IBSS_MERGE;

			}
		}
		break;

	default:
		break;
	}

	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);


	return;
}				
#endif				


VOID
aisIndicationOfMediaStateToHost(IN P_ADAPTER_T prAdapter,
				ENUM_PARAM_MEDIA_STATE_T eConnectionState,
				BOOLEAN fgDelayIndication)
{
	EVENT_CONNECTION_STATUS rEventConnStatus;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_FSM_INFO_T prAisFsmInfo;

	DEBUGFUNC("aisIndicationOfMediaStateToHost()");

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	
	

	if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_DISCONNECTED) {
		if (prAisBssInfo->eConnectionStateIndicated == eConnectionState)
			return;

	}

	if (!fgDelayIndication) {
		
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rIndicationOfDisconnectTimer);

		
		rEventConnStatus.ucMediaStatus = (UINT_8) eConnectionState;

		if (eConnectionState == PARAM_MEDIA_STATE_CONNECTED) {
			rEventConnStatus.ucReasonOfDisconnect = DISCONNECT_REASON_CODE_RESERVED;

			if (prAisBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE) {
				rEventConnStatus.ucInfraMode = (UINT_8) NET_TYPE_INFRA;
				rEventConnStatus.u2AID = prAisBssInfo->u2AssocId;
				rEventConnStatus.u2ATIMWindow = 0;
			} else if (prAisBssInfo->eCurrentOPMode == OP_MODE_IBSS) {
				rEventConnStatus.ucInfraMode = (UINT_8) NET_TYPE_IBSS;
				rEventConnStatus.u2AID = 0;
				rEventConnStatus.u2ATIMWindow = prAisBssInfo->u2ATIMWindow;
			} else {
				ASSERT(0);
			}

			COPY_SSID(rEventConnStatus.aucSsid,
				  rEventConnStatus.ucSsidLen,
				  prConnSettings->aucSSID, prConnSettings->ucSSIDLen);

			COPY_MAC_ADDR(rEventConnStatus.aucBssid, prAisBssInfo->aucBSSID);

			rEventConnStatus.u2BeaconPeriod = prAisBssInfo->u2BeaconInterval;
			rEventConnStatus.u4FreqInKHz =
			    nicChannelNum2Freq(prAisBssInfo->ucPrimaryChannel);

			switch (prAisBssInfo->ucNonHTBasicPhyType) {
			case PHY_TYPE_HR_DSSS_INDEX:
				rEventConnStatus.ucNetworkType = (UINT_8) PARAM_NETWORK_TYPE_DS;
				break;

			case PHY_TYPE_ERP_INDEX:
				rEventConnStatus.ucNetworkType = (UINT_8) PARAM_NETWORK_TYPE_OFDM24;
				break;

			case PHY_TYPE_OFDM_INDEX:
				rEventConnStatus.ucNetworkType = (UINT_8) PARAM_NETWORK_TYPE_OFDM5;
				break;

			default:
				ASSERT(0);
				rEventConnStatus.ucNetworkType = (UINT_8) PARAM_NETWORK_TYPE_DS;
				break;
			}
		} else {
			
			secClearPmkid(prAdapter);
			rEventConnStatus.ucReasonOfDisconnect = prAisBssInfo->ucReasonOfDisconnect;
		}

		
		nicMediaStateChange(prAdapter, prAdapter->prAisBssInfo->ucBssIndex,
				    &rEventConnStatus);
		prAisBssInfo->eConnectionStateIndicated = eConnectionState;
	} else {
		
		ASSERT(eConnectionState == PARAM_MEDIA_STATE_DISCONNECTED);

		DBGLOG(AIS, INFO, ("Postpone the indication of Disconnect for %d seconds\n",
				   prConnSettings->ucDelayTimeOfDisconnectEvent));

		cnmTimerStartTimer(prAdapter,
				   &prAisFsmInfo->rIndicationOfDisconnectTimer,
				   SEC_TO_MSEC(prConnSettings->ucDelayTimeOfDisconnectEvent));
	}

	return;
}				


VOID aisPostponedEventOfDisconnTimeout(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;

	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	if (prAisBssInfo->prStaRecOfAP) {
		

		prAisBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;
	}
	
	aisFsmIsRequestPending(prAdapter, AIS_REQUEST_RECONNECT, TRUE);
	prConnSettings->fgIsDisconnectedByNonRequest = TRUE;
	prAisBssInfo->u2DeauthReason = REASON_CODE_BEACON_TIMEOUT;
	
	aisIndicationOfMediaStateToHost(prAdapter, PARAM_MEDIA_STATE_DISCONNECTED, FALSE);

	return;
}				


VOID
aisUpdateBssInfoForJOIN(IN P_ADAPTER_T prAdapter,
			P_STA_RECORD_T prStaRec, P_SW_RFB_T prAssocRspSwRfb)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_WLAN_ASSOC_RSP_FRAME_T prAssocRspFrame;
	P_BSS_DESC_T prBssDesc;
	UINT_16 u2IELength;
	PUINT_8 pucIE;

	DEBUGFUNC("aisUpdateBssInfoForJOIN()");

	ASSERT(prStaRec);
	ASSERT(prAssocRspSwRfb);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prAssocRspFrame = (P_WLAN_ASSOC_RSP_FRAME_T) prAssocRspSwRfb->pvHeader;


	DBGLOG(AIS, INFO, ("Update AIS_BSS_INFO_T and apply settings to MAC\n"));


	
	
	prAisBssInfo->eCurrentOPMode = OP_MODE_INFRASTRUCTURE;

	
	COPY_SSID(prAisBssInfo->aucSSID,
		  prAisBssInfo->ucSSIDLen, prConnSettings->aucSSID, prConnSettings->ucSSIDLen);

	
	prAisBssInfo->ucPrimaryChannel = prAisFsmInfo->prTargetBssDesc->ucChannelNum;
	prAisBssInfo->eBand = prAisFsmInfo->prTargetBssDesc->eBand;


	
	
	prAisBssInfo->prStaRecOfAP = prStaRec;
	prAisBssInfo->u2AssocId = prStaRec->u2AssocId;

	
	prAisBssInfo->u2CapInfo = prStaRec->u2CapInfo;	

	if (prAisBssInfo->u2CapInfo & CAP_INFO_SHORT_PREAMBLE)
		prAisBssInfo->fgIsShortPreambleAllowed = TRUE;
	else
		prAisBssInfo->fgIsShortPreambleAllowed = FALSE;


#if CFG_SUPPORT_TDLS
	prAisBssInfo->fgTdlsIsProhibited = prStaRec->fgTdlsIsProhibited;
	prAisBssInfo->fgTdlsIsChSwProhibited = prStaRec->fgTdlsIsChSwProhibited;
#endif				

	
	prAisBssInfo->ucPhyTypeSet = prStaRec->ucDesiredPhyTypeSet;

	prAisBssInfo->ucNonHTBasicPhyType = prStaRec->ucNonHTBasicPhyType;

	prAisBssInfo->u2OperationalRateSet = prStaRec->u2OperationalRateSet;
	prAisBssInfo->u2BSSBasicRateSet = prStaRec->u2BSSBasicRateSet;

	nicTxUpdateBssDefaultRate(prAisBssInfo);

	
	
	COPY_MAC_ADDR(prAisBssInfo->aucBSSID, prAssocRspFrame->aucBSSID);


	u2IELength = (UINT_16) ((prAssocRspSwRfb->u2PacketLen - prAssocRspSwRfb->u2HeaderLen) -
				(OFFSET_OF(WLAN_ASSOC_RSP_FRAME_T, aucInfoElem[0]) -
				 WLAN_MAC_MGMT_HEADER_LEN));
	pucIE = prAssocRspFrame->aucInfoElem;


	
	
	mqmProcessAssocRsp(prAdapter, prAssocRspSwRfb, pucIE, u2IELength);

	prAisBssInfo->fgIsQBSS = prStaRec->fgIsQoS;

	
	prBssDesc = scanSearchBssDescByBssid(prAdapter, prAssocRspFrame->aucBSSID);
	if (prBssDesc) {
		prBssDesc->fgIsConnecting = FALSE;
		prBssDesc->fgIsConnected = TRUE;

		
		prAisBssInfo->u2BeaconInterval = prBssDesc->u2BeaconInterval;
	} else {
		
		ASSERT(0);
	}

	
	prAisBssInfo->ucDTIMPeriod = 0;
	prAisBssInfo->u2ATIMWindow = 0;

	prAisBssInfo->ucBeaconTimeoutCount = AIS_BEACON_TIMEOUT_COUNT_INFRA;

	
	rlmProcessAssocRsp(prAdapter, prAssocRspSwRfb, pucIE, u2IELength);

	
	nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	

	return;
}				


#if CFG_SUPPORT_ADHOC
VOID aisUpdateBssInfoForCreateIBSS(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	if (prAisBssInfo->fgIsBeaconActivated)
		return;

	
	
	prAisBssInfo->eCurrentOPMode = OP_MODE_IBSS;

	
	COPY_SSID(prAisBssInfo->aucSSID,
		  prAisBssInfo->ucSSIDLen, prConnSettings->aucSSID, prConnSettings->ucSSIDLen);

	
	prAisBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;
	prAisBssInfo->u2AssocId = 0;

	
	prAisBssInfo->ucPrimaryChannel = prConnSettings->ucAdHocChannelNum;
	prAisBssInfo->eBand = prConnSettings->eAdHocBand;

	if (prAisBssInfo->eBand == BAND_2G4) {
		
		prAisBssInfo->ucPhyTypeSet = prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11BGN;
		
		prAisBssInfo->ucConfigAdHocAPMode = AD_HOC_MODE_MIXED_11BG;
	} else {
		
		prAisBssInfo->ucPhyTypeSet = prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11ANAC;
		
		prAisBssInfo->ucConfigAdHocAPMode = AD_HOC_MODE_11A;
	}

	
	prAisBssInfo->u2BeaconInterval = prConnSettings->u2BeaconPeriod;
	prAisBssInfo->ucDTIMPeriod = 0;
	prAisBssInfo->u2ATIMWindow = prConnSettings->u2AtimWindow;

	prAisBssInfo->ucBeaconTimeoutCount = AIS_BEACON_TIMEOUT_COUNT_ADHOC;

	if (prConnSettings->eEncStatus == ENUM_ENCRYPTION1_ENABLED ||
	    prConnSettings->eEncStatus == ENUM_ENCRYPTION2_ENABLED ||
	    prConnSettings->eEncStatus == ENUM_ENCRYPTION3_ENABLED) {
		prAisBssInfo->fgIsProtection = TRUE;
	} else {
		prAisBssInfo->fgIsProtection = FALSE;
	}

	
	ibssInitForAdHoc(prAdapter, prAisBssInfo);
	
	bssInitializeClientList(prAdapter, prAisBssInfo);

	
	
	rlmBssInitForAPandIbss(prAdapter, prAisBssInfo);

	
	nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	bssUpdateBeaconContent(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	nicPmIndicateBssCreated(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	prAisBssInfo->fgIsBeaconActivated = TRUE;
	prAisBssInfo->fgHoldSameBssidForIBSS = TRUE;

	
	cnmTimerStartTimer(prAdapter,
			   &prAisFsmInfo->rIbssAloneTimer, SEC_TO_MSEC(AIS_IBSS_ALONE_TIMEOUT_SEC));

	return;

}				


VOID aisUpdateBssInfoForMergeIBSS(IN P_ADAPTER_T prAdapter, IN P_STA_RECORD_T prStaRec)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_DESC_T prBssDesc;
	
	


	ASSERT(prStaRec);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rIbssAloneTimer);

	if (!prAisBssInfo->fgIsBeaconActivated) {

		
		
		prAisBssInfo->eCurrentOPMode = OP_MODE_IBSS;

		
		COPY_SSID(prAisBssInfo->aucSSID,
			  prAisBssInfo->ucSSIDLen,
			  prConnSettings->aucSSID, prConnSettings->ucSSIDLen);

		
		prAisBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;
		prAisBssInfo->u2AssocId = 0;
	}
	
	
	prAisBssInfo->u2CapInfo = prStaRec->u2CapInfo;	

	if (prAisBssInfo->u2CapInfo & CAP_INFO_SHORT_PREAMBLE) {
		prAisBssInfo->fgIsShortPreambleAllowed = TRUE;
		prAisBssInfo->fgUseShortPreamble = TRUE;
	} else {
		prAisBssInfo->fgIsShortPreambleAllowed = FALSE;
		prAisBssInfo->fgUseShortPreamble = FALSE;
	}

	
	prAisBssInfo->fgUseShortSlotTime = FALSE;	
	prAisBssInfo->u2CapInfo &= ~CAP_INFO_SHORT_SLOT_TIME;

	if (prAisBssInfo->u2CapInfo & CAP_INFO_PRIVACY)
		prAisBssInfo->fgIsProtection = TRUE;
	else
		prAisBssInfo->fgIsProtection = FALSE;


	
	prAisBssInfo->ucPhyTypeSet = prStaRec->ucDesiredPhyTypeSet;

	prAisBssInfo->ucNonHTBasicPhyType = prStaRec->ucNonHTBasicPhyType;

	prAisBssInfo->u2OperationalRateSet = prStaRec->u2OperationalRateSet;
	prAisBssInfo->u2BSSBasicRateSet = prStaRec->u2BSSBasicRateSet;

	rateGetDataRatesFromRateSet(prAisBssInfo->u2OperationalRateSet,
				    prAisBssInfo->u2BSSBasicRateSet,
				    prAisBssInfo->aucAllSupportedRates,
				    &prAisBssInfo->ucAllSupportedRatesLen);

	


	
	prBssDesc = scanSearchBssDescByTA(prAdapter, prStaRec->aucMacAddr);
	if (prBssDesc) {
		prBssDesc->fgIsConnecting = FALSE;
		prBssDesc->fgIsConnected = TRUE;

		
		COPY_MAC_ADDR(prAisBssInfo->aucBSSID, prBssDesc->aucBSSID);

		
		prAisBssInfo->ucPrimaryChannel = prBssDesc->ucChannelNum;
		prAisBssInfo->eBand = prBssDesc->eBand;

		
		prAisBssInfo->u2BeaconInterval = prBssDesc->u2BeaconInterval;
		prAisBssInfo->ucDTIMPeriod = 0;
		prAisBssInfo->u2ATIMWindow = 0;	

		prAisBssInfo->ucBeaconTimeoutCount = AIS_BEACON_TIMEOUT_COUNT_ADHOC;
	} else {
		
		ASSERT(0);
	}


	
	
	nicTxUpdateBssDefaultRate(prAisBssInfo);

	
	rlmBssInitForAPandIbss(prAdapter, prAisBssInfo);

	
	nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	bssUpdateBeaconContent(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	nicPmIndicateBssConnected(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	
	prAisBssInfo->fgIsBeaconActivated = TRUE;
	prAisBssInfo->fgHoldSameBssidForIBSS = TRUE;

	return;
}				


BOOLEAN
aisValidateProbeReq(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb, OUT PUINT_32 pu4ControlFlags)
{
	P_WLAN_MAC_MGMT_HEADER_T prMgtHdr;
	P_BSS_INFO_T prBssInfo;
	P_IE_SSID_T prIeSsid = (P_IE_SSID_T) NULL;
	PUINT_8 pucIE;
	UINT_16 u2IELength;
	UINT_16 u2Offset = 0;
	BOOLEAN fgReplyProbeResp = FALSE;


	ASSERT(prSwRfb);
	ASSERT(pu4ControlFlags);

	prBssInfo = prAdapter->prAisBssInfo;

	
	prMgtHdr = (P_WLAN_MAC_MGMT_HEADER_T) prSwRfb->pvHeader;

	u2IELength = prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen;
	pucIE = (PUINT_8) ((ULONG) prSwRfb->pvHeader + prSwRfb->u2HeaderLen);

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		if (ELEM_ID_SSID == IE_ID(pucIE)) {
			if ((!prIeSsid) && (IE_LEN(pucIE) <= ELEM_MAX_LEN_SSID))
				prIeSsid = (P_IE_SSID_T) pucIE;

			break;
		}
	}			

	

	if (prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {

		if ((prIeSsid) && ((prIeSsid->ucLength == BC_SSID_LEN) ||	
				   EQUAL_SSID(prBssInfo->aucSSID, prBssInfo->ucSSIDLen,	
					      prIeSsid->aucSSID, prIeSsid->ucLength))) {
			fgReplyProbeResp = TRUE;
		}
	}

	return fgReplyProbeResp;

}				

#endif				

VOID aisFsmDisconnect(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgDelayIndication)
{
	P_BSS_INFO_T prAisBssInfo;
	UINT_8 aucP2pSsid[] = CTIA_MAGIC_SSID;

	ASSERT(prAdapter);

	prAisBssInfo = prAdapter->prAisBssInfo;

	nicPmIndicateBssAbort(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

#if CFG_SUPPORT_ADHOC
	if (prAisBssInfo->fgIsBeaconActivated) {
		nicUpdateBeaconIETemplate(prAdapter,
					  IE_UPD_METHOD_DELETE_ALL,
					  prAdapter->prAisBssInfo->ucBssIndex, 0, NULL, 0);

		prAisBssInfo->fgIsBeaconActivated = FALSE;
	}
#endif

	rlmBssAborted(prAdapter, prAisBssInfo);

	
	if (PARAM_MEDIA_STATE_CONNECTED == prAisBssInfo->eConnectionState) {

		
		{


			if (EQUAL_SSID
			    (aucP2pSsid, CTIA_MAGIC_SSID_LEN, prAisBssInfo->aucSSID,
			     prAisBssInfo->ucSSIDLen)) {
				nicEnterCtiaMode(prAdapter, FALSE, FALSE);
			}
		}

		if (prAisBssInfo->ucReasonOfDisconnect == DISCONNECT_REASON_CODE_RADIO_LOST) {
			scanRemoveBssDescByBssid(prAdapter, prAisBssInfo->aucBSSID);

			
			wlanClearBssInScanningResult(prAdapter, prAisBssInfo->aucBSSID);

			
			if (fgDelayIndication) {
				aisFsmIsRequestPending(prAdapter, AIS_REQUEST_RECONNECT, TRUE);
				aisFsmInsertRequest(prAdapter, AIS_REQUEST_RECONNECT);
			}
		} else {
			scanRemoveConnFlagOfBssDescByBssid(prAdapter, prAisBssInfo->aucBSSID);
		}

		if (fgDelayIndication) {
			if (OP_MODE_IBSS != prAisBssInfo->eCurrentOPMode)
				prAisBssInfo->fgHoldSameBssidForIBSS = FALSE;
		} else {
			prAisBssInfo->fgHoldSameBssidForIBSS = FALSE;
		}
	} else {
		prAisBssInfo->fgHoldSameBssidForIBSS = FALSE;
	}

	
	if (prAisBssInfo->ucReasonOfDisconnect != DISCONNECT_REASON_CODE_REASSOCIATION) {
		aisChangeMediaState(prAdapter, PARAM_MEDIA_STATE_DISCONNECTED);

		
		nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);
	}

	if (!fgDelayIndication) {
		
		if (prAisBssInfo->prStaRecOfAP) {
			
			prAisBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;
		}
	}
#if CFG_SUPPORT_ROAMING
	roamingFsmRunEventAbort(prAdapter);

	
	aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_SEARCH, TRUE);
	aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_CONNECT, TRUE);
#endif				

	
	aisIndicationOfMediaStateToHost(prAdapter,
					PARAM_MEDIA_STATE_DISCONNECTED, fgDelayIndication);


	
	aisFsmSteps(prAdapter, AIS_STATE_IDLE);

	return;
}				

static VOID
aisFsmRunEventScanDoneTimeOut (
    IN P_ADAPTER_T prAdapter,
    ULONG ulParam
    )
{
    P_AIS_FSM_INFO_T prAisFsmInfo;
    ENUM_AIS_STATE_T eNextState;
    P_CONNECTION_SETTINGS_T prConnSettings;

    DEBUGFUNC("aisFsmRunEventScanDoneTimeOut()");

    ASSERT(prAdapter);

    prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
    prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

    DBGLOG(AIS, STATE, ("aisFsmRunEventScanDoneTimeOut Current[%d]\n",prAisFsmInfo->eCurrentState));

	
	

    prConnSettings->fgIsScanReqIssued = FALSE;
    kalScanDone(prAdapter->prGlueInfo, KAL_NETWORK_TYPE_AIS_INDEX, WLAN_STATUS_SUCCESS);
    eNextState = prAisFsmInfo->eCurrentState;

    switch (prAisFsmInfo->eCurrentState) {
            case AIS_STATE_SCAN:
                prAisFsmInfo->u4ScanIELength = 0;
                eNextState = AIS_STATE_IDLE;
                break;
            case AIS_STATE_ONLINE_SCAN:
                
                prAisFsmInfo->u4ScanIELength = 0;
#if CFG_SUPPORT_ROAMING
                eNextState = aisFsmRoamingScanResultsUpdate(prAdapter);
#else
                eNextState = AIS_STATE_NORMAL_TR;
#endif 
                break;
            default:
                break;
        }

		
		aisFsmStateAbort_SCAN(prAdapter);


        if (eNextState != prAisFsmInfo->eCurrentState) {
            aisFsmSteps(prAdapter, eNextState);
        }

    return;
} 

VOID aisFsmRunEventBGSleepTimeOut(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;

	DEBUGFUNC("aisFsmRunEventBGSleepTimeOut()");

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	eNextState = prAisFsmInfo->eCurrentState;

	switch (prAisFsmInfo->eCurrentState) {
	case AIS_STATE_WAIT_FOR_NEXT_SCAN:
		DBGLOG(AIS, LOUD,
		       ("EVENT - SCAN TIMER: Idle End - Current Time = %u\n", kalGetTimeTick()));

		eNextState = AIS_STATE_LOOKING_FOR;

		SET_NET_PWR_STATE_ACTIVE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

		break;

	default:
		break;
	}

	
	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);


	return;
}				


VOID aisFsmRunEventIbssAloneTimeOut(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;

	DEBUGFUNC("aisFsmRunEventIbssAloneTimeOut()");

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	eNextState = prAisFsmInfo->eCurrentState;

	switch (prAisFsmInfo->eCurrentState) {
	case AIS_STATE_IBSS_ALONE:


		DBGLOG(AIS, LOUD, ("EVENT-IBSS ALONE TIMER: Start pairing\n"));

		prAisFsmInfo->fgTryScan = TRUE;

		
		aisFsmReleaseCh(prAdapter);

		
		eNextState = AIS_STATE_SEARCH;

		break;

	default:
		break;
	}


	
	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);


	return;
}				


VOID aisFsmRunEventJoinTimeout(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	ENUM_AIS_STATE_T eNextState;
	OS_SYSTIME rCurrentTime;

	DEBUGFUNC("aisFsmRunEventJoinTimeout()");

	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	eNextState = prAisFsmInfo->eCurrentState;

	GET_CURRENT_SYSTIME(&rCurrentTime);

	switch (prAisFsmInfo->eCurrentState) {
	case AIS_STATE_JOIN:
		DBGLOG(AIS, LOUD, ("EVENT- JOIN TIMEOUT\n"));

		
		aisFsmStateAbort_JOIN(prAdapter);

		
		prAisFsmInfo->prTargetStaRec->ucJoinFailureCount++;

		if (prAisFsmInfo->prTargetStaRec->ucJoinFailureCount < JOIN_MAX_RETRY_FAILURE_COUNT) {
			
			eNextState = AIS_STATE_SEARCH;
		} else if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED) {
			
			
			eNextState = AIS_STATE_WAIT_FOR_NEXT_SCAN;
		} else
		    if (!CHECK_FOR_TIMEOUT
			(rCurrentTime, prAisFsmInfo->rJoinReqTime,
			 SEC_TO_SYSTIME(AIS_JOIN_TIMEOUT))) {
			
			eNextState = AIS_STATE_WAIT_FOR_NEXT_SCAN;
		} else {
			
			eNextState = AIS_STATE_JOIN_FAILURE;
		}

		break;

	case AIS_STATE_NORMAL_TR:
		
		aisFsmReleaseCh(prAdapter);
		prAisFsmInfo->fgIsInfraChannelFinished = TRUE;

		
		if (aisFsmIsRequestPending(prAdapter, AIS_REQUEST_SCAN, TRUE) == TRUE) {
			wlanClearScanningResult(prAdapter);
			eNextState = AIS_STATE_ONLINE_SCAN;
		}

		break;

	default:
		
		aisFsmReleaseCh(prAdapter);
		break;

	}


	
	if (eNextState != prAisFsmInfo->eCurrentState)
		aisFsmSteps(prAdapter, eNextState);


	return;
}				

VOID aisFsmRunEventDeauthTimeout(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	aisDeauthXmitComplete(prAdapter, NULL, TX_RESULT_LIFE_TIMEOUT);
}

#if defined(CFG_TEST_MGMT_FSM) && (CFG_TEST_MGMT_FSM != 0)
VOID aisTest(VOID)
{
	P_MSG_AIS_ABORT_T prAisAbortMsg;
	P_CONNECTION_SETTINGS_T prConnSettings;
	UINT_8 aucSSID[] = "pci-11n";
	UINT_8 ucSSIDLen = 7;

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	prConnSettings->fgIsConnReqIssued = TRUE;
	prConnSettings->ucSSIDLen = ucSSIDLen;
	kalMemCopy(prConnSettings->aucSSID, aucSSID, ucSSIDLen);

	prAisAbortMsg =
	    (P_MSG_AIS_ABORT_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_AIS_ABORT_T));
	if (!prAisAbortMsg) {

		ASSERT(0);	
		return;
	}

	prAisAbortMsg->rMsgHdr.eMsgId = MID_HEM_AIS_FSM_ABORT;

	mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prAisAbortMsg, MSG_SEND_METHOD_BUF);

	wifi_send_msg(INDX_WIFI, MSG_ID_WIFI_IST, 0);

	return;
}
#endif				


VOID
aisFsmScanRequest(IN P_ADAPTER_T prAdapter,
		  IN P_PARAM_SSID_T prSsid, IN PUINT_8 pucIe, IN UINT_32 u4IeLength)
{
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_FSM_INFO_T prAisFsmInfo;

	DEBUGFUNC("aisFsmScanRequest()");

	ASSERT(prAdapter);
	ASSERT(u4IeLength <= MAX_IE_LENGTH);

	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	if (!prConnSettings->fgIsScanReqIssued) {
		prConnSettings->fgIsScanReqIssued = TRUE;

		if (prSsid == NULL) {
			prAisFsmInfo->ucScanSSIDNum = 0;
		} else {
			prAisFsmInfo->ucScanSSIDNum = 1;

			COPY_SSID(prAisFsmInfo->arScanSSID[0].aucSsid,
				  prAisFsmInfo->arScanSSID[0].u4SsidLen,
				  prSsid->aucSsid, prSsid->u4SsidLen);
		}

		if (u4IeLength > 0) {
			prAisFsmInfo->u4ScanIELength = u4IeLength;
			kalMemCopy(prAisFsmInfo->aucScanIEBuf, pucIe, u4IeLength);
		} else {
			prAisFsmInfo->u4ScanIELength = 0;
		}

		if (prAisFsmInfo->eCurrentState == AIS_STATE_NORMAL_TR) {
			if (prAisBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE
			    && prAisFsmInfo->fgIsInfraChannelFinished == FALSE) {
				
				aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);
			} else {
				if (prAisFsmInfo->fgIsChannelGranted == TRUE) {
					DBGLOG(AIS, WARN,
					       ("Scan Request with channel granted for join operation: %d, %d",
						prAisFsmInfo->fgIsChannelGranted,
						prAisFsmInfo->fgIsChannelRequested));
				}

				
				wlanClearScanningResult(prAdapter);
				aisFsmSteps(prAdapter, AIS_STATE_ONLINE_SCAN);
			}
		} else if (prAisFsmInfo->eCurrentState == AIS_STATE_IDLE) {
			wlanClearScanningResult(prAdapter);
			aisFsmSteps(prAdapter, AIS_STATE_SCAN);
		} else {
			aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);
		}
	} else {
		DBGLOG(AIS, WARN,
		       ("Scan Request dropped. (state: %d)\n", prAisFsmInfo->eCurrentState));
	}

	return;
}				


VOID
aisFsmScanRequestAdv(IN P_ADAPTER_T prAdapter,
		     IN UINT_8 ucSsidNum,
		     IN P_PARAM_SSID_T prSsid, IN PUINT_8 pucIe, IN UINT_32 u4IeLength)
{
	UINT_32 i;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_FSM_INFO_T prAisFsmInfo;

	DEBUGFUNC("aisFsmScanRequestAdv()");

	ASSERT(prAdapter);
	ASSERT(ucSsidNum <= SCN_SSID_MAX_NUM);
	ASSERT(u4IeLength <= MAX_IE_LENGTH);

	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	if (!prConnSettings->fgIsScanReqIssued) {
		prConnSettings->fgIsScanReqIssued = TRUE;

		if (ucSsidNum == 0) {
			prAisFsmInfo->ucScanSSIDNum = 0;
		} else {
			prAisFsmInfo->ucScanSSIDNum = ucSsidNum;

			for (i = 0; i < ucSsidNum; i++) {
				COPY_SSID(prAisFsmInfo->arScanSSID[i].aucSsid,
					  prAisFsmInfo->arScanSSID[i].u4SsidLen,
					  prSsid[i].aucSsid, prSsid[i].u4SsidLen);
			}
		}

		if (u4IeLength > 0) {
			prAisFsmInfo->u4ScanIELength = u4IeLength;
			kalMemCopy(prAisFsmInfo->aucScanIEBuf, pucIe, u4IeLength);
		} else {
			prAisFsmInfo->u4ScanIELength = 0;
		}

		if (prAisFsmInfo->eCurrentState == AIS_STATE_NORMAL_TR) {
			if (prAisBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE
			    && prAisFsmInfo->fgIsInfraChannelFinished == FALSE) {
				
				aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);
			} else {
				if (prAisFsmInfo->fgIsChannelGranted == TRUE) {
					DBGLOG(AIS, WARN,
					       ("Scan Request with channel granted for join operation: %d, %d",
						prAisFsmInfo->fgIsChannelGranted,
						prAisFsmInfo->fgIsChannelRequested));
				}

				
				wlanClearScanningResult(prAdapter);
				aisFsmSteps(prAdapter, AIS_STATE_ONLINE_SCAN);
			}
		} else if (prAisFsmInfo->eCurrentState == AIS_STATE_IDLE) {
			wlanClearScanningResult(prAdapter);
			aisFsmSteps(prAdapter, AIS_STATE_SCAN);
		} else {
			aisFsmInsertRequest(prAdapter, AIS_REQUEST_SCAN);
		}
	} else {
		DBGLOG(AIS, WARN,
		       ("Scan Request dropped. (state: %d)\n", prAisFsmInfo->eCurrentState));
	}

	return;
}				


VOID aisFsmRunEventChGrant(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_BSS_INFO_T prAisBssInfo;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_CH_GRANT_T prMsgChGrant;
	UINT_8 ucTokenID;
	UINT_32 u4GrantInterval;

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);

	prAisBssInfo = prAdapter->prAisBssInfo;
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prMsgChGrant = (P_MSG_CH_GRANT_T) prMsgHdr;

	ucTokenID = prMsgChGrant->ucTokenID;
	u4GrantInterval = prMsgChGrant->u4GrantInterval;

	
	cnmMemFree(prAdapter, prMsgHdr);

	if (prAisFsmInfo->eCurrentState == AIS_STATE_REQ_CHANNEL_JOIN &&
	    prAisFsmInfo->ucSeqNumOfChReq == ucTokenID) {
		
		prAisFsmInfo->u4ChGrantedInterval = u4GrantInterval;

		
		
		cnmTimerStartTimer(prAdapter,
				   &prAisFsmInfo->rJoinTimeoutTimer,
				   prAisFsmInfo->u4ChGrantedInterval - AIS_JOIN_CH_GRANT_THRESHOLD);
		
		prAisFsmInfo->fgIsInfraChannelFinished = FALSE;

		
		aisFsmSteps(prAdapter, AIS_STATE_JOIN);

		prAisFsmInfo->fgIsChannelGranted = TRUE;
	} else if (prAisFsmInfo->eCurrentState == AIS_STATE_REQ_REMAIN_ON_CHANNEL &&
		   prAisFsmInfo->ucSeqNumOfChReq == ucTokenID) {
		
		prAisFsmInfo->u4ChGrantedInterval = u4GrantInterval;

		
		cnmTimerStartTimer(prAdapter,
				   &prAisFsmInfo->rChannelTimeoutTimer,
				   prAisFsmInfo->u4ChGrantedInterval);

		
		aisFsmSteps(prAdapter, AIS_STATE_REMAIN_ON_CHANNEL);

		
		kalReadyOnChannel(prAdapter->prGlueInfo,
				  prAisFsmInfo->rChReqInfo.u8Cookie,
				  prAisFsmInfo->rChReqInfo.eBand,
				  prAisFsmInfo->rChReqInfo.eSco,
				  prAisFsmInfo->rChReqInfo.ucChannelNum,
				  prAisFsmInfo->rChReqInfo.u4DurationMs);

		prAisFsmInfo->fgIsChannelGranted = TRUE;
	} else {		
		
		aisFsmReleaseCh(prAdapter);
	}

	return;
}				


VOID aisFsmReleaseCh(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_CH_ABORT_T prMsgChAbort;

	ASSERT(prAdapter);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	if (prAisFsmInfo->fgIsChannelGranted == TRUE || prAisFsmInfo->fgIsChannelRequested == TRUE) {

		prAisFsmInfo->fgIsChannelRequested = FALSE;
		prAisFsmInfo->fgIsChannelGranted = FALSE;

		
		prMsgChAbort =
		    (P_MSG_CH_ABORT_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_CH_ABORT_T));
		if (!prMsgChAbort) {
			ASSERT(0);	
			return;
		}

		prMsgChAbort->rMsgHdr.eMsgId = MID_MNY_CNM_CH_ABORT;
		prMsgChAbort->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;
		prMsgChAbort->ucTokenID = prAisFsmInfo->ucSeqNumOfChReq;

		mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prMsgChAbort, MSG_SEND_METHOD_BUF);
	}

	return;
}				


VOID aisBssBeaconTimeout(IN P_ADAPTER_T prAdapter)
{
	P_BSS_INFO_T prAisBssInfo;
	BOOLEAN fgDoAbortIndication = FALSE;
	P_CONNECTION_SETTINGS_T prConnSettings;

	ASSERT(prAdapter);

	prAisBssInfo = prAdapter->prAisBssInfo;
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	if (PARAM_MEDIA_STATE_CONNECTED == prAisBssInfo->eConnectionState) {
		if (OP_MODE_INFRASTRUCTURE == prAisBssInfo->eCurrentOPMode) {
			P_STA_RECORD_T prStaRec = prAisBssInfo->prStaRecOfAP;

			if (prStaRec)
				fgDoAbortIndication = TRUE;

		} else if (OP_MODE_IBSS == prAisBssInfo->eCurrentOPMode) {
			fgDoAbortIndication = TRUE;
		}
	}
	
	if (fgDoAbortIndication) {
		prConnSettings->fgIsDisconnectedByNonRequest = FALSE;
		aisFsmStateAbort(prAdapter, DISCONNECT_REASON_CODE_RADIO_LOST, TRUE);
	}

	return;
}				


WLAN_STATUS
aisDeauthXmitComplete(IN P_ADAPTER_T prAdapter,
		      IN P_MSDU_INFO_T prMsduInfo, IN ENUM_TX_RESULT_CODE_T rTxDoneStatus)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;

	ASSERT(prAdapter);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	if (rTxDoneStatus == TX_RESULT_SUCCESS)
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rDeauthDoneTimer);


	if (prAisFsmInfo->eCurrentState == AIS_STATE_DISCONNECTING) {
		if (rTxDoneStatus != TX_RESULT_DROPPED_IN_DRIVER
		    && rTxDoneStatus != TX_RESULT_QUEUE_CLEARANCE) {
			aisFsmStateAbort(prAdapter, DISCONNECT_REASON_CODE_NEW_CONNECTION, FALSE);
		}
	} else {
		DBGLOG(AIS, WARN, ("DEAUTH frame transmitted without further handling"));
	}

	return WLAN_STATUS_SUCCESS;

}				

#if CFG_SUPPORT_ROAMING
VOID aisFsmRunEventRoamingDiscovery(IN P_ADAPTER_T prAdapter, UINT_32 u4ReqScan)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	ENUM_AIS_REQUEST_TYPE_T eAisRequest;

	DBGLOG(AIS, LOUD, ("aisFsmRunEventRoamingDiscovery()\n"));

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	prConnSettings->eConnectionPolicy = CONNECT_BY_SSID_BEST_RSSI;

	
#if CFG_SUPPORT_WFD
#if CFG_ENABLE_WIFI_DIRECT
	{
		
		P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;
		prWfdCfgSettings = &(prAdapter->rWifiVar.rWfdConfigureSettings);
		if ((prWfdCfgSettings->ucWfdEnable != 0)) {
			DBGLOG(ROAMING, INFO, ("WFD is running. Stop roaming. [WfdEnable:%u]\n", prWfdCfgSettings->ucWfdEnable));
			roamingFsmRunEventRoam(prAdapter);
			roamingFsmRunEventFail(prAdapter, ROAMING_FAIL_REASON_NOCANDIDATE);
			return;
		}
	}
#endif
#endif

	
	if (!u4ReqScan) {
		roamingFsmRunEventRoam(prAdapter);
		eAisRequest = AIS_REQUEST_ROAMING_CONNECT;
	} else {
		if (prAisFsmInfo->eCurrentState == AIS_STATE_ONLINE_SCAN
		    || prAisFsmInfo->eCurrentState == AIS_STATE_LOOKING_FOR) {
			eAisRequest = AIS_REQUEST_ROAMING_CONNECT;
		} else {
			eAisRequest = AIS_REQUEST_ROAMING_SEARCH;
		}
	}

	if (prAisFsmInfo->eCurrentState == AIS_STATE_NORMAL_TR
	    && prAisFsmInfo->fgIsInfraChannelFinished == TRUE) {
		if (eAisRequest == AIS_REQUEST_ROAMING_SEARCH)
			aisFsmSteps(prAdapter, AIS_STATE_LOOKING_FOR);
		else
			aisFsmSteps(prAdapter, AIS_STATE_SEARCH);
	} else {
		aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_SEARCH, TRUE);
		aisFsmIsRequestPending(prAdapter, AIS_REQUEST_ROAMING_CONNECT, TRUE);

		aisFsmInsertRequest(prAdapter, eAisRequest);
	}

	return;
}				

ENUM_AIS_STATE_T aisFsmRoamingScanResultsUpdate(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_ROAMING_INFO_T prRoamingFsmInfo;
	ENUM_AIS_STATE_T eNextState;

	DBGLOG(AIS, LOUD, ("->aisFsmRoamingScanResultsUpdate()\n"));

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prRoamingFsmInfo = (P_ROAMING_INFO_T) &(prAdapter->rWifiVar.rRoamingInfo);

	roamingFsmScanResultsUpdate(prAdapter);

	eNextState = prAisFsmInfo->eCurrentState;
	if (prRoamingFsmInfo->eCurrentState == ROAMING_STATE_DISCOVERY) {
		roamingFsmRunEventRoam(prAdapter);
		eNextState = AIS_STATE_SEARCH;
	} else if (prAisFsmInfo->eCurrentState == AIS_STATE_LOOKING_FOR) {
		eNextState = AIS_STATE_SEARCH;
	} else if (prAisFsmInfo->eCurrentState == AIS_STATE_ONLINE_SCAN) {
		eNextState = AIS_STATE_NORMAL_TR;
	}

	return eNextState;
}				

VOID aisFsmRoamingDisconnectPrevAP(IN P_ADAPTER_T prAdapter, IN P_STA_RECORD_T prTargetStaRec)
{
	P_BSS_INFO_T prAisBssInfo;

	DBGLOG(AIS, LOUD, ("aisFsmRoamingDisconnectPrevAP()"));

	ASSERT(prAdapter);

	prAisBssInfo = prAdapter->prAisBssInfo;

	nicPmIndicateBssAbort(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);

	

	
	if (PARAM_MEDIA_STATE_CONNECTED == prAisBssInfo->eConnectionState)
		scanRemoveConnFlagOfBssDescByBssid(prAdapter, prAisBssInfo->aucBSSID);

	
	aisChangeMediaState(prAdapter, PARAM_MEDIA_STATE_DISCONNECTED);

	
	prTargetStaRec->ucBssIndex = (MAX_BSS_INDEX + 1);	
	nicUpdateBss(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);
	prTargetStaRec->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;

	return;
}				

VOID
aisUpdateBssInfoForRoamingAP(IN P_ADAPTER_T prAdapter,
			     IN P_STA_RECORD_T prStaRec, IN P_SW_RFB_T prAssocRspSwRfb)
{
	P_BSS_INFO_T prAisBssInfo;

	DBGLOG(AIS, LOUD, ("aisUpdateBssInfoForRoamingAP()"));

	ASSERT(prAdapter);

	prAisBssInfo = prAdapter->prAisBssInfo;

	
	aisChangeMediaState(prAdapter, PARAM_MEDIA_STATE_CONNECTED);

	
	if ((prAisBssInfo->prStaRecOfAP) &&
	    (prAisBssInfo->prStaRecOfAP != prStaRec) && (prAisBssInfo->prStaRecOfAP->fgIsInUse)) {
		
		cnmStaRecFree(prAdapter, prAisBssInfo->prStaRecOfAP);
	}
	
	cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);

	
	aisUpdateBssInfoForJOIN(prAdapter, prStaRec, prAssocRspSwRfb);

	
	
	aisIndicationOfMediaStateToHost(prAdapter, PARAM_MEDIA_STATE_CONNECTED, FALSE);

	return;
}				

#endif				


BOOLEAN
aisFsmIsRequestPending(IN P_ADAPTER_T prAdapter,
		       IN ENUM_AIS_REQUEST_TYPE_T eReqType, IN BOOLEAN bRemove)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_AIS_REQ_HDR_T prPendingReqHdr, prPendingReqHdrNext;

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	
	LINK_FOR_EACH_ENTRY_SAFE(prPendingReqHdr,
				 prPendingReqHdrNext,
				 &(prAisFsmInfo->rPendingReqList), rLinkEntry, AIS_REQ_HDR_T) {
		
		if (prPendingReqHdr->eReqType == eReqType) {
			
			if (bRemove == TRUE) {
				LINK_REMOVE_KNOWN_ENTRY(&(prAisFsmInfo->rPendingReqList),
							&(prPendingReqHdr->rLinkEntry));

				cnmMemFree(prAdapter, prPendingReqHdr);
			}

			return TRUE;
		}
	}

	return FALSE;
}


P_AIS_REQ_HDR_T aisFsmGetNextRequest(IN P_ADAPTER_T prAdapter)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_AIS_REQ_HDR_T prPendingReqHdr;

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	LINK_REMOVE_HEAD(&(prAisFsmInfo->rPendingReqList), prPendingReqHdr, P_AIS_REQ_HDR_T);

	return prPendingReqHdr;
}


BOOLEAN aisFsmInsertRequest(IN P_ADAPTER_T prAdapter, IN ENUM_AIS_REQUEST_TYPE_T eReqType)
{
	P_AIS_REQ_HDR_T prAisReq;
	P_AIS_FSM_INFO_T prAisFsmInfo;

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

	prAisReq = (P_AIS_REQ_HDR_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(AIS_REQ_HDR_T));

	if (!prAisReq) {
		ASSERT(0);	
		return FALSE;
	}

	prAisReq->eReqType = eReqType;

	
	LINK_INSERT_TAIL(&prAisFsmInfo->rPendingReqList, &prAisReq->rLinkEntry);

	return TRUE;
}


VOID aisFsmFlushRequest(IN P_ADAPTER_T prAdapter)
{
	P_AIS_REQ_HDR_T prAisReq;

	ASSERT(prAdapter);

	while ((prAisReq = aisFsmGetNextRequest(prAdapter)) != NULL)
		cnmMemFree(prAdapter, prAisReq);


	return;
}


VOID aisFsmRunEventRemainOnChannel(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_MSG_REMAIN_ON_CHANNEL_T prRemainOnChannel;
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;

	DEBUGFUNC("aisFsmRunEventRemainOnChannel()");

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);

	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	prRemainOnChannel = (P_MSG_REMAIN_ON_CHANNEL_T) prMsgHdr;

	
	prAisFsmInfo->rChReqInfo.eBand = prRemainOnChannel->eBand;
	prAisFsmInfo->rChReqInfo.eSco = prRemainOnChannel->eSco;
	prAisFsmInfo->rChReqInfo.ucChannelNum = prRemainOnChannel->ucChannelNum;
	prAisFsmInfo->rChReqInfo.u4DurationMs = prRemainOnChannel->u4DurationMs;
	prAisFsmInfo->rChReqInfo.u8Cookie = prRemainOnChannel->u8Cookie;

	if (prAisFsmInfo->eCurrentState == AIS_STATE_IDLE
	    || prAisFsmInfo->eCurrentState == AIS_STATE_NORMAL_TR) {
		
		aisFsmSteps(prAdapter, AIS_STATE_REQ_REMAIN_ON_CHANNEL);
	} else {
		aisFsmInsertRequest(prAdapter, AIS_REQUEST_REMAIN_ON_CHANNEL);
	}

	
	cnmMemFree(prAdapter, prMsgHdr);

	return;
}


VOID aisFsmRunEventCancelRemainOnChannel(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;
	P_MSG_CANCEL_REMAIN_ON_CHANNEL_T prCancelRemainOnChannel;

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;

	prCancelRemainOnChannel = (P_MSG_CANCEL_REMAIN_ON_CHANNEL_T) prMsgHdr;

	
	if (prCancelRemainOnChannel->u8Cookie == prAisFsmInfo->rChReqInfo.u8Cookie) {

		
		if (prAisFsmInfo->eCurrentState == AIS_STATE_REQ_REMAIN_ON_CHANNEL) {
			
			aisFsmReleaseCh(prAdapter);
		} else if (prAisFsmInfo->eCurrentState == AIS_STATE_REMAIN_ON_CHANNEL) {
			
			aisFsmReleaseCh(prAdapter);

			
			cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rChannelTimeoutTimer);
		}

		
		aisFsmIsRequestPending(prAdapter, AIS_REQUEST_REMAIN_ON_CHANNEL, TRUE);

		
		if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED)
			aisFsmSteps(prAdapter, AIS_STATE_NORMAL_TR);
		else
			aisFsmSteps(prAdapter, AIS_STATE_IDLE);

	}

	
	cnmMemFree(prAdapter, prMsgHdr);

	return;
}


VOID aisFsmRunEventMgmtFrameTx(IN P_ADAPTER_T prAdapter, IN P_MSG_HDR_T prMsgHdr)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_MSG_MGMT_TX_REQUEST_T prMgmtTxMsg = (P_MSG_MGMT_TX_REQUEST_T) NULL;

	do {
		ASSERT((prAdapter != NULL) && (prMsgHdr != NULL));

		prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

		if (prAisFsmInfo == NULL)
			break;

		prMgmtTxMsg = (P_MSG_MGMT_TX_REQUEST_T) prMsgHdr;

		aisFuncTxMgmtFrame(prAdapter,
				   &prAisFsmInfo->rMgmtTxInfo,
				   prMgmtTxMsg->prMgmtMsduInfo, prMgmtTxMsg->u8Cookie);


	} while (FALSE);

	if (prMsgHdr)
		cnmMemFree(prAdapter, prMsgHdr);

	return;
}				


VOID aisFsmRunEventChannelTimeout(IN P_ADAPTER_T prAdapter, ULONG ulParamPtr)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_BSS_INFO_T prAisBssInfo;

	DEBUGFUNC("aisFsmRunEventRemainOnChannel()");

	ASSERT(prAdapter);
	prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
	prAisBssInfo = prAdapter->prAisBssInfo;

	if (prAisFsmInfo->eCurrentState == AIS_STATE_REMAIN_ON_CHANNEL) {
		
		aisFsmReleaseCh(prAdapter);

		
		cnmTimerStopTimer(prAdapter, &prAisFsmInfo->rChannelTimeoutTimer);

		
		kalRemainOnChannelExpired(prAdapter->prGlueInfo,
					  prAisFsmInfo->rChReqInfo.u8Cookie,
					  prAisFsmInfo->rChReqInfo.eBand,
					  prAisFsmInfo->rChReqInfo.eSco,
					  prAisFsmInfo->rChReqInfo.ucChannelNum);

		
		if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED)
			aisFsmSteps(prAdapter, AIS_STATE_NORMAL_TR);
		else
			aisFsmSteps(prAdapter, AIS_STATE_IDLE);

	} else {
		DBGLOG(AIS, WARN, ("Unexpected remain_on_channel timeout event\n"));
#if DBG
		DBGLOG(AIS, STATE,
		       ("CURRENT State: [%s]\n", apucDebugAisState[prAisFsmInfo->eCurrentState]));
#else
		DBGLOG(AIS, STATE,
		       ("[%d] CURRENT State: [%d]\n", DBG_AIS_IDX, prAisFsmInfo->eCurrentState));
#endif
	}

	return;
}


WLAN_STATUS
aisFsmRunEventMgmtFrameTxDone(IN P_ADAPTER_T prAdapter,
			      IN P_MSDU_INFO_T prMsduInfo, IN ENUM_TX_RESULT_CODE_T rTxDoneStatus)
{
	P_AIS_FSM_INFO_T prAisFsmInfo;
	P_AIS_MGMT_TX_REQ_INFO_T prMgmtTxReqInfo = (P_AIS_MGMT_TX_REQ_INFO_T) NULL;
	BOOLEAN fgIsSuccess = FALSE;

	do {
		ASSERT((prAdapter != NULL) && (prMsduInfo != NULL));

		prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);
		prMgmtTxReqInfo = &(prAisFsmInfo->rMgmtTxInfo);

		if (rTxDoneStatus == TX_RESULT_SUCCESS)
			fgIsSuccess = TRUE;


		if (prMgmtTxReqInfo->prMgmtTxMsdu == prMsduInfo) {
			kalIndicateMgmtTxStatus(prAdapter->prGlueInfo,
						prMgmtTxReqInfo->u8Cookie,
						fgIsSuccess,
						prMsduInfo->prPacket,
						(UINT_32) prMsduInfo->u2FrameLength);

			prMgmtTxReqInfo->prMgmtTxMsdu = NULL;
		}

	} while (FALSE);

	return WLAN_STATUS_SUCCESS;

}				


WLAN_STATUS
aisFuncTxMgmtFrame(IN P_ADAPTER_T prAdapter,
		   IN P_AIS_MGMT_TX_REQ_INFO_T prMgmtTxReqInfo,
		   IN P_MSDU_INFO_T prMgmtTxMsdu, IN UINT_64 u8Cookie)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;
	P_MSDU_INFO_T prTxMsduInfo = (P_MSDU_INFO_T) NULL;
	P_WLAN_MAC_HEADER_T prWlanHdr = (P_WLAN_MAC_HEADER_T) NULL;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;

	do {
		ASSERT((prAdapter != NULL) && (prMgmtTxReqInfo != NULL));

		if (prMgmtTxReqInfo->fgIsMgmtTxRequested) {

			
			
			prTxMsduInfo = prMgmtTxReqInfo->prMgmtTxMsdu;
			if (prTxMsduInfo != NULL) {

				kalIndicateMgmtTxStatus(prAdapter->prGlueInfo,
							prMgmtTxReqInfo->u8Cookie,
							FALSE,
							prTxMsduInfo->prPacket,
							(UINT_32) prTxMsduInfo->u2FrameLength);

				
				
				prMgmtTxReqInfo->prMgmtTxMsdu = NULL;
			}
			
			
		}

		ASSERT(prMgmtTxReqInfo->prMgmtTxMsdu == NULL);

		prWlanHdr =
		    (P_WLAN_MAC_HEADER_T) ((ULONG) prMgmtTxMsdu->prPacket +
					   MAC_TX_RESERVED_FIELD);
		prStaRec =
		    cnmGetStaRecByAddress(prAdapter, prAdapter->prAisBssInfo->ucBssIndex,
					  prWlanHdr->aucAddr1);

		TX_SET_MMPDU(prAdapter,
			     prMgmtTxMsdu,
			     (prStaRec !=
			      NULL) ? (prStaRec->ucBssIndex) : (prAdapter->prAisBssInfo->
								ucBssIndex),
			     (prStaRec != NULL) ? (prStaRec->ucIndex) : (STA_REC_INDEX_NOT_FOUND),
			     WLAN_MAC_MGMT_HEADER_LEN, prMgmtTxMsdu->u2FrameLength,
			     aisFsmRunEventMgmtFrameTxDone, MSDU_RATE_MODE_AUTO);
		prMgmtTxReqInfo->u8Cookie = u8Cookie;
		prMgmtTxReqInfo->prMgmtTxMsdu = prMgmtTxMsdu;
		prMgmtTxReqInfo->fgIsMgmtTxRequested = TRUE;

		nicTxConfigPktControlFlag(prMgmtTxMsdu, MSDU_CONTROL_FLAG_FORCE_TX, TRUE);

		
		nicTxEnqueueMsdu(prAdapter, prMgmtTxMsdu);

	} while (FALSE);

	return rWlanStatus;
}				


VOID aisFuncValidateRxActionFrame(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	P_AIS_FSM_INFO_T prAisFsmInfo = (P_AIS_FSM_INFO_T) NULL;

	DEBUGFUNC("aisFuncValidateRxActionFrame");

	do {

		ASSERT((prAdapter != NULL) && (prSwRfb != NULL));

		prAisFsmInfo = &(prAdapter->rWifiVar.rAisFsmInfo);

		if (1 ) {
			
			kalIndicateRxMgmtFrame(prAdapter->prGlueInfo, prSwRfb);
		}

	} while (FALSE);

	return;

}				
