#include "precomp.h"

APPEND_VAR_ATTRI_ENTRY_T txAssocRspAttributesTable[] = {
	{(P2P_ATTRI_HDR_LEN + P2P_ATTRI_MAX_LEN_STATUS), NULL, p2pFuncAppendAttriStatusForAssocRsp}	 
	, {(P2P_ATTRI_HDR_LEN + P2P_ATTRI_MAX_LEN_EXT_LISTEN_TIMING), NULL, p2pFuncAppendAttriExtListenTiming}	
};


APPEND_VAR_IE_ENTRY_T txProbeRspIETable[] = {
	{(ELEM_HDR_LEN + (RATE_NUM_SW - ELEM_MAX_LEN_SUP_RATES)), NULL, bssGenerateExtSuppRate_IE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_ERP), NULL, rlmRspGenerateErpIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP), NULL, rlmRspGenerateHtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_OP), NULL, rlmRspGenerateHtOpIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_RSN), NULL, rsnGenerateRSNIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_OBSS_SCAN), NULL, rlmRspGenerateObssScanIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_EXT_CAP), NULL, rlmRspGenerateExtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WPA), NULL, rsnGenerateWpaNoneIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WMM_PARAM), NULL, mqmGenerateWmmParamIE}	
#if CFG_SUPPORT_802_11AC
    ,{ (ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP),NULL,rlmRspGenerateVhtCapIE}   
    ,{ (ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_OP),NULL,rlmRspGenerateVhtOpIE}       
#endif
#if CFG_SUPPORT_MTK_SYNERGY
    ,{ (ELEM_HDR_LEN + ELEM_MIN_LEN_MTK_OUI),NULL,rlmGenerateMTKOuiIE}   
#endif

};

static VOID
p2pFuncParseBeaconVenderId(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucIE, IN P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo, OUT BOOL * ucNewSecMode);

static VOID
p2pFuncGetAttriListAction(IN P_ADAPTER_T prAdapter,
	IN P_IE_P2P_T prIe, IN UINT_8 ucOuiType,
	OUT PUINT_8 * pucAttriListStart, OUT UINT_16 * u2AttriListLen,
	OUT BOOLEAN * fgIsAllocMem, OUT BOOLEAN * fgBackupAttributes, OUT UINT_16 * u2BufferSize);

static VOID
p2pFuncProcessP2pProbeRspAction(IN P_ADAPTER_T prAdapter,
		IN PUINT_8 pucIEBuf, IN UINT_8 ucElemIdType,
		OUT UINT_8 * ucBssIdx, OUT P_BSS_INFO_T * prP2pBssInfo, OUT BOOLEAN * fgIsWSCIE,
		OUT BOOLEAN * fgIsP2PIE, OUT BOOLEAN * fgIsWFDIE);
static VOID
p2pFuncGetSpecAttriAction(IN P_IE_P2P_T prP2pIE,
		IN UINT_8 ucOuiType, IN UINT_8 ucAttriID,
		OUT P_ATTRIBUTE_HDR_T * prTargetAttri);
VOID
p2pFuncRequestScan(IN P_ADAPTER_T prAdapter,
		   IN UINT_8 ucBssIndex, IN P_P2P_SCAN_REQ_INFO_T prScanReqInfo)
{
	P_MSG_SCN_SCAN_REQ_V2 prScanReqV2 = (P_MSG_SCN_SCAN_REQ_V2) NULL;

#ifdef CFG_SUPPORT_BEAM_PLUS
	
	P_P2P_FSM_INFO_T prP2pFsmInfo = (P_P2P_FSM_INFO_T) NULL;
#endif


	DEBUGFUNC("p2pFuncRequestScan()");

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prScanReqInfo != NULL));

		if (prScanReqInfo->eChannelSet == SCAN_CHANNEL_SPECIFIED) {
			ASSERT_BREAK(prScanReqInfo->ucNumChannelList > 0);
			DBGLOG(P2P, LOUD,
			       ("P2P Scan Request Channel:%d\n",
				prScanReqInfo->arScanChannelList[0].ucChannelNum));
		}

		prScanReqV2 =
		    (P_MSG_SCN_SCAN_REQ_V2) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
							(sizeof(MSG_SCN_SCAN_REQ_V2) +
							 (sizeof(PARAM_SSID_T) *
							  prScanReqInfo->ucSsidNum)));
		if (!prScanReqV2) {
			ASSERT(0);	
			DBGLOG(P2P, ERROR,
			       ("p2pFuncRequestScan: Memory allocation fail, can not send SCAN MSG to scan module\n"));
			break;
		}

		prScanReqV2->rMsgHdr.eMsgId = MID_P2P_SCN_SCAN_REQ_V2;
		prScanReqV2->ucSeqNum = ++prScanReqInfo->ucSeqNumOfScnMsg;
		prScanReqV2->ucBssIndex = ucBssIndex;
		prScanReqV2->eScanType = prScanReqInfo->eScanType;
		prScanReqV2->eScanChannel = prScanReqInfo->eChannelSet;
		prScanReqV2->u2IELen = 0;
		prScanReqV2->prSsid =
		    (P_PARAM_SSID_T) ((ULONG) prScanReqV2 + sizeof(MSG_SCN_SCAN_REQ_V2));

		
		kalMemCopy(prScanReqV2->aucIE, prScanReqInfo->aucIEBuf, prScanReqInfo->u4BufLength);
		prScanReqV2->u2IELen = (UINT_16) prScanReqInfo->u4BufLength;

		prScanReqV2->u2ChannelDwellTime = prScanReqInfo->u2PassiveDewellTime;
		prScanReqV2->u2TimeoutValue = 0;
		prScanReqV2->u2ProbeDelay = 0;

		switch (prScanReqInfo->eChannelSet) {
		case SCAN_CHANNEL_SPECIFIED:
			{
				UINT_32 u4Idx = 0;
				P_RF_CHANNEL_INFO_T prDomainInfo =
				    (P_RF_CHANNEL_INFO_T) prScanReqInfo->arScanChannelList;

				if (prScanReqInfo->ucNumChannelList >
				    MAXIMUM_OPERATION_CHANNEL_LIST) {
					prScanReqInfo->ucNumChannelList =
					    MAXIMUM_OPERATION_CHANNEL_LIST;
				}


				for (u4Idx = 0; u4Idx < prScanReqInfo->ucNumChannelList; u4Idx++) {
					prScanReqV2->arChnlInfoList[u4Idx].ucChannelNum =
					    prDomainInfo->ucChannelNum;
					prScanReqV2->arChnlInfoList[u4Idx].eBand =
					    prDomainInfo->eBand;
					prDomainInfo++;
				}

				prScanReqV2->ucChannelListNum = prScanReqInfo->ucNumChannelList;
			}
		case SCAN_CHANNEL_FULL:
		case SCAN_CHANNEL_2G4:
		case SCAN_CHANNEL_P2P_SOCIAL:
			{
				
				P_PARAM_SSID_T prParamSsid = (P_PARAM_SSID_T) NULL;

				prParamSsid = prScanReqV2->prSsid;

				for (prScanReqV2->ucSSIDNum = 0;
				     prScanReqV2->ucSSIDNum < prScanReqInfo->ucSsidNum;
				     prScanReqV2->ucSSIDNum++) {
					COPY_SSID(prParamSsid->aucSsid,
						  prParamSsid->u4SsidLen,
						  prScanReqInfo->arSsidStruct[prScanReqV2->
									      ucSSIDNum].aucSsid,
						  prScanReqInfo->arSsidStruct[prScanReqV2->
									      ucSSIDNum].ucSsidLen);

					prParamSsid++;
				}


				
				if (prScanReqV2->ucSSIDNum == 0) {
					prScanReqV2->ucSSIDType = SCAN_REQ_SSID_P2P_WILDCARD;
				} else {
					prScanReqV2->ucSSIDType = SCAN_REQ_SSID_SPECIFIED;
				}
			}
			break;
		default:
			
			ASSERT(FALSE);
			break;
		}

		prScanReqInfo->fgIsScanRequest = TRUE;

		mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prScanReqV2, MSG_SEND_METHOD_BUF);

	} while (FALSE);

	return;
}				





VOID
p2pFuncCancelScan(IN P_ADAPTER_T prAdapter,
		  IN UINT_8 ucBssIndex, IN P_P2P_SCAN_REQ_INFO_T prScanInfo)
{
	P_MSG_SCN_SCAN_CANCEL prScanCancelMsg = (P_MSG_SCN_SCAN_CANCEL) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prScanInfo != NULL));

		if (!prScanInfo->fgIsScanRequest) {
			break;
		}


		if (prScanInfo->ucSeqNumOfScnMsg) {
			
			DBGLOG(P2P, TRACE, ("P2P Cancel Scan\n"));

			prScanCancelMsg =
			    (P_MSG_SCN_SCAN_CANCEL) cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
								sizeof(MSG_SCN_SCAN_CANCEL));
			if (!prScanCancelMsg) {
				
				DBGLOG(P2P, TRACE, ("Buffer not enough, can not cancel scan.\n"));
				ASSERT(FALSE);
				break;
			}

			prScanCancelMsg->rMsgHdr.eMsgId = MID_P2P_SCN_SCAN_CANCEL;
			prScanCancelMsg->ucBssIndex = ucBssIndex;
			prScanCancelMsg->ucSeqNum = prScanInfo->ucSeqNumOfScnMsg++;
			prScanCancelMsg->fgIsChannelExt = FALSE;
			prScanInfo->fgIsScanRequest = FALSE;

			mboxSendMsg(prAdapter,
				    MBOX_ID_0, (P_MSG_HDR_T) prScanCancelMsg, MSG_SEND_METHOD_BUF);


		}


	} while (FALSE);

	return;
}				


VOID
p2pFuncGCJoin(IN P_ADAPTER_T prAdapter,
	      IN P_BSS_INFO_T prP2pBssInfo, IN P_P2P_JOIN_INFO_T prP2pJoinInfo)
{
	P_MSG_JOIN_REQ_T prJoinReqMsg = (P_MSG_JOIN_REQ_T) NULL;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;
	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (prP2pBssInfo != NULL) && (prP2pJoinInfo != NULL));

		prBssDesc = prP2pJoinInfo->prTargetBssDesc;
		if ((prBssDesc) == NULL) {
			DBGLOG(P2P, ERROR, ("p2pFuncGCJoin: NO Target BSS Descriptor\n"));
			ASSERT(FALSE);
			break;
		}


		if (prBssDesc->ucSSIDLen) {
			COPY_SSID(prP2pBssInfo->aucSSID,
				  prP2pBssInfo->ucSSIDLen,
				  prBssDesc->aucSSID, prBssDesc->ucSSIDLen);
		}

		
		prBssDesc->fgIsConnecting = TRUE;

		
		prStaRec = bssCreateStaRecFromBssDesc(prAdapter,
						      (prBssDesc->
						       fgIsP2PPresent ? (STA_TYPE_P2P_GO)
						       : (STA_TYPE_LEGACY_AP)),
						      prP2pBssInfo->ucBssIndex, prBssDesc);

		if (prStaRec == NULL) {
			DBGLOG(P2P, TRACE, ("Create station record fail\n"));
			ASSERT(FALSE);
			break;
		}


		prP2pJoinInfo->prTargetStaRec = prStaRec;
		prP2pJoinInfo->fgIsJoinComplete = FALSE;
		prP2pJoinInfo->u4BufLength = 0;

		
		cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);


		if (prP2pBssInfo->eConnectionState == PARAM_MEDIA_STATE_DISCONNECTED) {
			prStaRec->fgIsReAssoc = FALSE;
			prP2pJoinInfo->ucAvailableAuthTypes = (UINT_8) AUTH_TYPE_OPEN_SYSTEM;
			prStaRec->ucTxAuthAssocRetryLimit = TX_AUTH_ASSOCI_RETRY_LIMIT;
		} else {
			DBGLOG(P2P, ERROR, ("JOIN INIT: Join Request when connected.\n"));
			ASSERT(FALSE);
			
			break;
		}


		
		if (prP2pJoinInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_OPEN_SYSTEM) {

			DBGLOG(P2P, TRACE,
			       ("JOIN INIT: Try to do Authentication with AuthType == OPEN_SYSTEM.\n"));

			prP2pJoinInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_OPEN_SYSTEM;

			prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_OPEN_SYSTEM;
		} else {
			DBGLOG(P2P, ERROR, ("JOIN INIT: ucAvailableAuthTypes Error.\n"));
			ASSERT(FALSE);
			break;
		}


		


		

		
		prJoinReqMsg =
		    (P_MSG_JOIN_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_JOIN_REQ_T));

		if (!prJoinReqMsg) {
			DBGLOG(P2P, TRACE, ("Allocation Join Message Fail\n"));
			ASSERT(FALSE);
			return;
		}

		prJoinReqMsg->rMsgHdr.eMsgId = MID_P2P_SAA_FSM_START;
		prJoinReqMsg->ucSeqNum = ++prP2pJoinInfo->ucSeqNumOfReqMsg;
		prJoinReqMsg->prStaRec = prStaRec;

		

		mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prJoinReqMsg, MSG_SEND_METHOD_BUF);

	} while (FALSE);

}				



VOID
p2pFuncUpdateBssInfoForJOIN(IN P_ADAPTER_T prAdapter,
			    IN P_BSS_DESC_T prBssDesc,
			    IN P_STA_RECORD_T prStaRec,
			    IN P_BSS_INFO_T prP2pBssInfo, IN P_SW_RFB_T prAssocRspSwRfb)
{
	P_WLAN_ASSOC_RSP_FRAME_T prAssocRspFrame = (P_WLAN_ASSOC_RSP_FRAME_T) NULL;
	UINT_16 u2IELength;
	PUINT_8 pucIE;

	DEBUGFUNC("p2pUpdateBssInfoForJOIN()");

	do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (prStaRec != NULL) &&
			     (prP2pBssInfo != NULL) && (prAssocRspSwRfb != NULL));

		prAssocRspFrame = (P_WLAN_ASSOC_RSP_FRAME_T) prAssocRspSwRfb->pvHeader;

		if (prBssDesc == NULL) {
			
			DBGLOG(P2P, TRACE, ("Target BSS NULL\n"));
			break;
		}

		DBGLOG(P2P, INFO, ("Update P2P_BSS_INFO_T and apply settings to MAC\n"));

		
		
		ASSERT_BREAK(prP2pBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE);

		if (UNEQUAL_MAC_ADDR(prBssDesc->aucBSSID, prAssocRspFrame->aucBSSID)) {
			ASSERT(FALSE);
		}
		
		COPY_SSID(prP2pBssInfo->aucSSID,
			  prP2pBssInfo->ucSSIDLen, prBssDesc->aucSSID, prBssDesc->ucSSIDLen);

		
		prP2pBssInfo->ucPrimaryChannel = prBssDesc->ucChannelNum;
		prP2pBssInfo->eBand = prBssDesc->eBand;

		
		
		prP2pBssInfo->prStaRecOfAP = prStaRec;
		prP2pBssInfo->u2AssocId = prStaRec->u2AssocId;

		
		prP2pBssInfo->u2CapInfo = prStaRec->u2CapInfo;	

		if (prP2pBssInfo->u2CapInfo & CAP_INFO_SHORT_PREAMBLE) {
			prP2pBssInfo->fgIsShortPreambleAllowed = TRUE;
		} else {
			prP2pBssInfo->fgIsShortPreambleAllowed = FALSE;
		}


		
		prP2pBssInfo->ucPhyTypeSet = prStaRec->ucDesiredPhyTypeSet;

		prP2pBssInfo->ucNonHTBasicPhyType = prStaRec->ucNonHTBasicPhyType;

		prP2pBssInfo->u2OperationalRateSet = prStaRec->u2OperationalRateSet;
		prP2pBssInfo->u2BSSBasicRateSet = prStaRec->u2BSSBasicRateSet;

		nicTxUpdateBssDefaultRate(prP2pBssInfo);

		
		
		COPY_MAC_ADDR(prP2pBssInfo->aucBSSID, prAssocRspFrame->aucBSSID);


		u2IELength =
		    (UINT_16) ((prAssocRspSwRfb->u2PacketLen - prAssocRspSwRfb->u2HeaderLen) -
			       (OFFSET_OF(WLAN_ASSOC_RSP_FRAME_T, aucInfoElem[0]) -
				WLAN_MAC_MGMT_HEADER_LEN));
		pucIE = prAssocRspFrame->aucInfoElem;


		
		
		mqmProcessAssocRsp(prAdapter, prAssocRspSwRfb, pucIE, u2IELength);

		prP2pBssInfo->fgIsQBSS = prStaRec->fgIsQoS;

		

		prBssDesc->fgIsConnecting = FALSE;
		prBssDesc->fgIsConnected = TRUE;

		
		prP2pBssInfo->u2BeaconInterval = prBssDesc->u2BeaconInterval;
		
		prP2pBssInfo->ucDTIMPeriod = 0;
		prP2pBssInfo->u2ATIMWindow = 0;

		prP2pBssInfo->ucBeaconTimeoutCount = AIS_BEACON_TIMEOUT_COUNT_INFRA;

		
		rlmProcessAssocRsp(prAdapter, prAssocRspSwRfb, pucIE, u2IELength);

		
		nicUpdateBss(prAdapter, prP2pBssInfo->ucBssIndex);

		
		

	} while (FALSE);

	return;
}				



WLAN_STATUS
p2pFunMgmtFrameTxDone(IN P_ADAPTER_T prAdapter,
		      IN P_MSDU_INFO_T prMsduInfo, IN ENUM_TX_RESULT_CODE_T rTxDoneStatus)
{
	BOOLEAN fgIsSuccess = FALSE;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prMsduInfo != NULL));


		if (rTxDoneStatus != TX_RESULT_SUCCESS) {
			DBGLOG(P2P, TRACE, ("Mgmt Frame TX Fail, Status:%d.\n", rTxDoneStatus));
		} else {
			fgIsSuccess = TRUE;
			DBGLOG(P2P, TRACE, ("Mgmt Frame TX Done.\n"));
		}

		kalP2PIndicateMgmtTxStatus(prAdapter->prGlueInfo, prMsduInfo, fgIsSuccess);


	} while (FALSE);

	return WLAN_STATUS_SUCCESS;

}				




WLAN_STATUS
p2pFuncTxMgmtFrame(IN P_ADAPTER_T prAdapter,
		   IN UINT_8 ucBssIndex, IN P_MSDU_INFO_T prMgmtTxMsdu, IN BOOLEAN fgNonCckRate)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;
	
	P_WLAN_MAC_HEADER_T prWlanHdr = (P_WLAN_MAC_HEADER_T) NULL;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;
	UINT_8 ucRetryLimit = 30;	
	BOOLEAN fgDrop = FALSE;

	do {
		ASSERT_BREAK(prAdapter != NULL);

		
		if (!IS_NET_ACTIVE(prAdapter, ucBssIndex)) {
			p2pDevFsmRunEventMgmtFrameTxDone(prAdapter, prMgmtTxMsdu,
							 TX_RESULT_DROPPED_IN_DRIVER);
			cnmMgtPktFree(prAdapter, prMgmtTxMsdu);
			fgDrop = TRUE;

			break;
		}

		prWlanHdr =
		    (P_WLAN_MAC_HEADER_T) ((ULONG) prMgmtTxMsdu->prPacket +
					   MAC_TX_RESERVED_FIELD);
		prStaRec = cnmGetStaRecByAddress(prAdapter, ucBssIndex, prWlanHdr->aucAddr1);
		

		switch (prWlanHdr->u2FrameCtrl & MASK_FRAME_TYPE) {
		case MAC_FRAME_PROBE_RSP:
			DBGLOG(P2P, TRACE, ("TX Probe Resposne Frame\n"));
            if(!nicTxIsMgmtResourceEnough(prAdapter)) {
                DBGLOG(P2P, TRACE, ("Drop Tx probe response due to resource issue\n"));
				fgDrop = TRUE;

    			break;
            }
			prMgmtTxMsdu =
			    p2pFuncProcessP2pProbeRsp(prAdapter, ucBssIndex, prMgmtTxMsdu);
			ucRetryLimit = 2;
			break;
		default:
			prMgmtTxMsdu->ucBssIndex = ucBssIndex;
			break;
		}

		if(fgDrop) {
			
			p2pDevFsmRunEventMgmtFrameTxDone(prAdapter, prMgmtTxMsdu,
							 TX_RESULT_DROPPED_IN_DRIVER);
			cnmMgtPktFree(prAdapter, prMgmtTxMsdu);

			break;
		}

		TX_SET_MMPDU(prAdapter,
			     prMgmtTxMsdu,
			     prMgmtTxMsdu->ucBssIndex,
			     (prStaRec != NULL) ? (prStaRec->ucIndex) : (STA_REC_INDEX_NOT_FOUND),
			     WLAN_MAC_MGMT_HEADER_LEN,
			     prMgmtTxMsdu->u2FrameLength,
			     p2pDevFsmRunEventMgmtFrameTxDone, MSDU_RATE_MODE_AUTO);

		nicTxSetPktRetryLimit(prMgmtTxMsdu, ucRetryLimit);

		nicTxConfigPktControlFlag(prMgmtTxMsdu, MSDU_CONTROL_FLAG_FORCE_TX, TRUE);

		nicTxEnqueueMsdu(prAdapter, prMgmtTxMsdu);

	} while (FALSE);

	return rWlanStatus;
}				





VOID
p2pFuncStartGO(IN P_ADAPTER_T prAdapter,
	       IN P_BSS_INFO_T prBssInfo,
	       IN P_P2P_CONNECTION_REQ_INFO_T prP2pConnReqInfo,
	       IN P_P2P_CHNL_REQ_INFO_T prP2pChnlReqInfo)
{
	do {
		ASSERT_BREAK((prAdapter != NULL) && (prBssInfo != NULL));

		if (prBssInfo->ucBssIndex >= P2P_DEV_BSS_INDEX) {
			DBGLOG(P2P, ERROR, ("P2P BSS exceed the number of P2P interface number."));
			ASSERT(FALSE);
			break;
		}

		DBGLOG(P2P, TRACE, ("p2pFuncStartGO:\n"));

		
		p2pFuncSwitchOPMode(prAdapter, prBssInfo, prBssInfo->eIntendOPMode, FALSE);

		prBssInfo->eIntendOPMode = OP_MODE_NUM;

		
		COPY_SSID(prBssInfo->aucSSID,
			  prBssInfo->ucSSIDLen,
			  prP2pConnReqInfo->rSsidStruct.aucSsid,
			  prP2pConnReqInfo->rSsidStruct.ucSsidLen);

		DBGLOG(P2P, TRACE, ("GO SSID:%s\n", prBssInfo->aucSSID));

		
		prBssInfo->prStaRecOfAP = (P_STA_RECORD_T) NULL;
		prBssInfo->u2AssocId = 0;


		
		prBssInfo->ucPrimaryChannel = prP2pChnlReqInfo->ucReqChnlNum;
		prBssInfo->eBand = prP2pChnlReqInfo->eBand;
		prBssInfo->eBssSCO = prP2pChnlReqInfo->eChnlSco;

		DBGLOG(P2P, TRACE, ("GO Channel:%d\n", prBssInfo->ucPrimaryChannel));


		if (prBssInfo->eBand == BAND_5G) {
			prBssInfo->ucPhyTypeSet = (prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11AN);	
			prBssInfo->ucConfigAdHocAPMode = AP_MODE_11A;	
		} else if (prP2pConnReqInfo->eConnRequest == P2P_CONNECTION_TYPE_PURE_AP) {
			prBssInfo->ucPhyTypeSet = (prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11BGN);	
			prBssInfo->ucConfigAdHocAPMode = AP_MODE_MIXED_11BG;	
		} else {
			ASSERT(prP2pConnReqInfo->eConnRequest == P2P_CONNECTION_TYPE_GO);
			prBssInfo->ucPhyTypeSet = (prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11GN);	
			prBssInfo->ucConfigAdHocAPMode = AP_MODE_11G_P2P;	
		}

		
		bssDetermineApBssInfoPhyTypeSet(prAdapter,
						(prP2pConnReqInfo->eConnRequest ==
						 P2P_CONNECTION_TYPE_PURE_AP) ? TRUE : FALSE,
						prBssInfo);

		prBssInfo->ucNonHTBasicPhyType = (UINT_8)
		    rNonHTApModeAttributes[prBssInfo->ucConfigAdHocAPMode].ePhyTypeIndex;
		prBssInfo->u2BSSBasicRateSet =
		    rNonHTApModeAttributes[prBssInfo->ucConfigAdHocAPMode].u2BSSBasicRateSet;
		prBssInfo->u2OperationalRateSet =
		    rNonHTPhyAttributes[prBssInfo->ucNonHTBasicPhyType].u2SupportedRateSet;

		if (prBssInfo->ucAllSupportedRatesLen == 0) {
			rateGetDataRatesFromRateSet(prBssInfo->u2OperationalRateSet,
						    prBssInfo->u2BSSBasicRateSet,
						    prBssInfo->aucAllSupportedRates,
						    &prBssInfo->ucAllSupportedRatesLen);
		}
		
		prBssInfo->u2ATIMWindow = 0;
		prBssInfo->ucBeaconTimeoutCount = 0;

		
#if CFG_SUPPORT_AAA
		if (prP2pConnReqInfo->eConnRequest == P2P_CONNECTION_TYPE_GO) {
			prBssInfo->fgIsProtection = TRUE;	
			
		} else {
			ASSERT(prP2pConnReqInfo->eConnRequest == P2P_CONNECTION_TYPE_PURE_AP);
			if (kalP2PGetCipher(prAdapter->prGlueInfo))
				prBssInfo->fgIsProtection = TRUE;
		}

		bssInitForAP(prAdapter, prBssInfo, TRUE);

		if ((prBssInfo->ucBMCWlanIndex >= WTBL_SIZE)) {
			prBssInfo->ucBMCWlanIndex =
			    secPrivacySeekForBcEntry(prAdapter, prBssInfo->ucBssIndex,
						     prBssInfo->aucBSSID, 0xff, CIPHER_SUITE_NONE,
						     0xff, 0x0, BIT(0));
		}


		nicQmUpdateWmmParms(prAdapter, prBssInfo->ucBssIndex);
#endif				


		
		
		rlmBssInitForAPandIbss(prAdapter, prBssInfo);

		
		nicUpdateBss(prAdapter, prBssInfo->ucBssIndex);

		
		bssUpdateBeaconContent(prAdapter, prBssInfo->ucBssIndex);

		
		nicPmIndicateBssCreated(prAdapter, prBssInfo->ucBssIndex);

	} while (FALSE);

	return;
}				

VOID p2pFuncStopGO(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prP2pBssInfo)
{
	do {
		ASSERT_BREAK((prAdapter != NULL) && (prP2pBssInfo != NULL));

		DBGLOG(P2P, TRACE, ("p2pFuncStopGO\n"));

		if ((prP2pBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT)
		    && (prP2pBssInfo->eIntendOPMode == OP_MODE_NUM)) {
			

			p2pFuncDissolve(prAdapter, prP2pBssInfo, TRUE,
					REASON_CODE_DEAUTH_LEAVING_BSS);

			DBGLOG(P2P, TRACE, ("Stop Beaconing\n"));
			nicPmIndicateBssAbort(prAdapter, prP2pBssInfo->ucBssIndex);

			
			rlmBssAborted(prAdapter, prP2pBssInfo);

			prP2pBssInfo->eIntendOPMode = OP_MODE_P2P_DEVICE;
		}

		DBGLOG(P2P, INFO, ("Re activate P2P Network.\n"));
		nicDeactivateNetwork(prAdapter, prP2pBssInfo->ucBssIndex);

		nicActivateNetwork(prAdapter, prP2pBssInfo->ucBssIndex);

	} while (FALSE);

}				


WLAN_STATUS
p2pFuncRoleToBssIdx(IN P_ADAPTER_T prAdapter, IN UINT_8 ucRoleIdx, OUT PUINT_8 pucBssIdx)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBssIdx != NULL));

		if (ucRoleIdx > BSS_P2P_NUM) {
			rWlanStatus = WLAN_STATUS_FAILURE;
			break;
		}

		*pucBssIdx = prAdapter->rWifiVar.aprP2pRoleFsmInfo[ucRoleIdx]->ucBssIndex;

	} while (FALSE);

	return rWlanStatus;
}				



















VOID
p2pFuncSwitchOPMode(IN P_ADAPTER_T prAdapter,
		    IN P_BSS_INFO_T prP2pBssInfo, IN ENUM_OP_MODE_T eOpMode, IN BOOLEAN fgSyncToFW)
{
	do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (prP2pBssInfo != NULL) && (eOpMode < OP_MODE_NUM));

		if (prP2pBssInfo->eCurrentOPMode != eOpMode) {
			DBGLOG(P2P, TRACE,
			       ("p2pFuncSwitchOPMode: Switch to from %d, to %d.\n",
				prP2pBssInfo->eCurrentOPMode, eOpMode));

			switch (prP2pBssInfo->eCurrentOPMode) {
			case OP_MODE_ACCESS_POINT:
				
				
				if (prP2pBssInfo->eIntendOPMode != OP_MODE_P2P_DEVICE) {
					p2pFuncStopGO(prAdapter, prP2pBssInfo);

					SET_NET_PWR_STATE_IDLE(prAdapter, prP2pBssInfo->ucBssIndex);
				}
				break;
			default:
				break;
			}


			prP2pBssInfo->eIntendOPMode = eOpMode;
			prP2pBssInfo->eCurrentOPMode = eOpMode;
			switch (eOpMode) {
			case OP_MODE_INFRASTRUCTURE:
				DBGLOG(P2P, TRACE, ("p2pFuncSwitchOPMode: Switch to Client.\n"));
			case OP_MODE_ACCESS_POINT:
				
				if (eOpMode == OP_MODE_ACCESS_POINT) {
					DBGLOG(P2P, TRACE,
					       ("p2pFuncSwitchOPMode: Switch to AP.\n"));
					prP2pBssInfo->ucSSIDLen = 0;
				}

				COPY_MAC_ADDR(prP2pBssInfo->aucOwnMacAddr,
					      prAdapter->rWifiVar.aucInterfaceAddress);
				COPY_MAC_ADDR(prP2pBssInfo->aucBSSID,
					      prAdapter->rWifiVar.aucInterfaceAddress);


				break;
			case OP_MODE_P2P_DEVICE:
				{
					
					DBGLOG(P2P, TRACE,
					       ("p2pFuncSwitchOPMode: Switch back to P2P Device.\n"));

					p2pChangeMediaState(prAdapter, prP2pBssInfo,
							    PARAM_MEDIA_STATE_DISCONNECTED);

					COPY_MAC_ADDR(prP2pBssInfo->aucOwnMacAddr,
						      prAdapter->rWifiVar.aucDeviceAddress);
					COPY_MAC_ADDR(prP2pBssInfo->aucBSSID,
						      prAdapter->rWifiVar.aucDeviceAddress);


				}
				break;
			default:
				ASSERT(FALSE);
				break;
			}

			if (1) {
				P2P_DISCONNECT_INFO rP2PDisInfo;

				rP2PDisInfo.ucRole = 2;
				wlanSendSetQueryCmd(prAdapter,
						    CMD_ID_P2P_ABORT,
						    TRUE,
						    FALSE,
						    FALSE,
						    NULL,
						    NULL,
						    sizeof(P2P_DISCONNECT_INFO),
						    (PUINT_8) &rP2PDisInfo, NULL, 0);
			}


			DBGLOG(P2P, TRACE,
			       ("The device address is changed to " MACSTR "\n",
				MAC2STR(prP2pBssInfo->aucOwnMacAddr)));
			DBGLOG(P2P, TRACE,
			       ("The BSSID is changed to " MACSTR "\n",
				MAC2STR(prP2pBssInfo->aucBSSID)));

			
			if ((fgSyncToFW) && (eOpMode != OP_MODE_ACCESS_POINT)) {
				nicUpdateBss(prAdapter, prP2pBssInfo->ucBssIndex);
			}
		}

	} while (FALSE);

	return;
}				








VOID
p2pFuncReleaseCh(IN P_ADAPTER_T prAdapter,
		 IN UINT_8 ucBssIdx, IN P_P2P_CHNL_REQ_INFO_T prChnlReqInfo)
{
	P_MSG_CH_ABORT_T prMsgChRelease = (P_MSG_CH_ABORT_T) NULL;

	DEBUGFUNC("p2pFuncReleaseCh()");

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prChnlReqInfo != NULL));

		if (!prChnlReqInfo->fgIsChannelRequested) {
			break;
		} else {
			DBGLOG(P2P, TRACE, ("P2P Release Channel\n"));
			prChnlReqInfo->fgIsChannelRequested = FALSE;
		}

		
		prMsgChRelease =
		    (P_MSG_CH_ABORT_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_CH_ABORT_T));
		if (!prMsgChRelease) {
			ASSERT(0);	
			break;
		}

		prMsgChRelease->rMsgHdr.eMsgId = MID_MNY_CNM_CH_ABORT;
		prMsgChRelease->ucBssIndex = ucBssIdx;
		prMsgChRelease->ucTokenID = prChnlReqInfo->ucSeqNumOfChReq++;

		mboxSendMsg(prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMsgChRelease, MSG_SEND_METHOD_BUF);

	} while (FALSE);

	return;
}				


VOID
p2pFuncAcquireCh(IN P_ADAPTER_T prAdapter,
		 IN UINT_8 ucBssIdx, IN P_P2P_CHNL_REQ_INFO_T prChnlReqInfo)
{
	P_MSG_CH_REQ_T prMsgChReq = (P_MSG_CH_REQ_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prChnlReqInfo != NULL));

		p2pFuncReleaseCh(prAdapter, ucBssIdx, prChnlReqInfo);

		
		prMsgChReq =
		    (P_MSG_CH_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_CH_REQ_T));

		if (!prMsgChReq) {
			ASSERT(0);	
			break;
		}

		prMsgChReq->rMsgHdr.eMsgId = MID_MNY_CNM_CH_REQ;
		prMsgChReq->ucBssIndex = ucBssIdx;
		prMsgChReq->ucTokenID = ++prChnlReqInfo->ucSeqNumOfChReq;
		prMsgChReq->eReqType = prChnlReqInfo->eChnlReqType;
		prMsgChReq->u4MaxInterval = prChnlReqInfo->u4MaxInterval;
		prMsgChReq->ucPrimaryChannel = prChnlReqInfo->ucReqChnlNum;
		prMsgChReq->eRfSco = prChnlReqInfo->eChnlSco;
		prMsgChReq->eRfBand = prChnlReqInfo->eBand;
		prMsgChReq->eRfChannelWidth = prChnlReqInfo->eChannelWidth;
		prMsgChReq->ucRfCenterFreqSeg1 = prChnlReqInfo->ucCenterFreqS1;
		prMsgChReq->ucRfCenterFreqSeg2 = prChnlReqInfo->ucCenterFreqS2;

		

		mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prMsgChReq, MSG_SEND_METHOD_BUF);

		prChnlReqInfo->fgIsChannelRequested = TRUE;

	} while (FALSE);

	return;
}				

#if 0
WLAN_STATUS
p2pFuncBeaconUpdate(IN P_ADAPTER_T prAdapter,
		    IN PUINT_8 pucBcnHdr,
		    IN UINT_32 u4HdrLen,
		    IN PUINT_8 pucBcnBody,
		    IN UINT_32 u4BodyLen, IN UINT_32 u4DtimPeriod, IN UINT_32 u4BcnInterval)
{
	WLAN_STATUS rResultStatus = WLAN_STATUS_INVALID_DATA;
	P_WLAN_BEACON_FRAME_T prBcnFrame = (P_WLAN_BEACON_FRAME_T) NULL;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;
	P_MSDU_INFO_T prBcnMsduInfo = (P_MSDU_INFO_T) NULL;
	PUINT_8 pucTIMBody = (PUINT_8) NULL;
	UINT_16 u2FrameLength = 0, UINT_16 u2OldBodyLen = 0;
	UINT_8 aucIEBuf[MAX_IE_LENGTH];

	do {
		ASSERT_BREAK(prAdapter != NULL);

		prP2pBssInfo = &(prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX]);
		prBcnMsduInfo = prP2pBssInfo->prBeacon ASSERT_BREAK(prBcnMsduInfo != NULL);

		
		prBcnFrame = prBcnMsduInfo->prPacket;

		ASSERT_BREAK(prBcnFrame != NULL);

		do {
			
			UINT_16 u2IELength = 0, u2Offset = 0;
			PUINT_8 pucIEBuf = prBcnFrame->aucInfoElem;

			u2IELength =
			    prBcnMsduInfo->u2FrameLength - prBcnMsduInfo->ucMacHeaderLength;

			IE_FOR_EACH(pucIEBuf, u2IELength, u2Offset) {
				if ((IE_ID(pucIEBuf) == ELEM_ID_TIM) ||
				    ((IE_ID(pucIEBuf) > ELEM_ID_IBSS_PARAM_SET))) {
				    pucTIMBody = pucIEBuf; break}
				    u2FrameLength += IE_SIZE(pucIEBuf); }

				    if (pucTIMBody == NULL) {
				    pucTIMBody = pucIEBuf; }

				    
				    u2OldBodyLen =
				    (UINT_16) ((UINT_32) pucTIMBody -
					       (UINT_32) prBcnFrame->aucInfoElem);
				    
				    kalMemCmp(aucIEBuf, pucTIMBody, u2OldBodyLen); } while (FALSE);
				    if (pucBcnHdr) {
				    kalMemCopy(prBcnMsduInfo->prPacket, pucBcnHdr, u4HdrLen); pucTIMBody = (PUINT_8) ((UINT_32) prBcnMsduInfo->prPacket + u4HdrLen); prBcnMsduInfo->ucMacHeaderLength = (WLAN_MAC_MGMT_HEADER_LEN + (TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN + CAP_INFO_FIELD_LEN)); u2FrameLength = u4HdrLen;	
				    } else {
				    
				    u2FrameLength += prBcnMsduInfo->ucMacHeaderLength; }


				    if (pucBcnBody) {
				    kalMemCopy(pucTIMBody, pucBcnBody, u4BodyLen);
					u2FrameLength += (UINT_16) u4BodyLen;
					} else {
				    kalMemCopy(pucTIMBody, aucIEBuf, u2OldBodyLen);
				    u2FrameLength += u2OldBodyLen; }

				    
				    prBcnMsduInfo->u2FrameLength = u2FrameLength;
				    prBcnMsduInfo->fgIs802_11 = TRUE;
				    prBcnMsduInfo->ucNetworkType = NETWORK_TYPE_P2P_INDEX;
				    prP2pBssInfo->u2BeaconInterval = (UINT_16) u4BcnInterval;
				    prP2pBssInfo->ucDTIMPeriod = (UINT_8) u4DtimPeriod;
				    prP2pBssInfo->u2CapInfo = prBcnFrame->u2CapInfo;
				    prBcnMsduInfo->ucPacketType = 3;
				    rResultStatus = nicUpdateBeaconIETemplate(prAdapter,
									      IE_UPD_METHOD_UPDATE_ALL,
									      NETWORK_TYPE_P2P_INDEX,
									      prP2pBssInfo->
									      u2CapInfo,
									      (PUINT_8) prBcnFrame->
									      aucInfoElem,
									      prBcnMsduInfo->
									      u2FrameLength -
									      OFFSET_OF
									      (WLAN_BEACON_FRAME_T,
									       aucInfoElem));
				    if (prP2pBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {
				    
				    nicPmIndicateBssAbort(prAdapter, NETWORK_TYPE_P2P_INDEX);
				    nicPmIndicateBssCreated(prAdapter, NETWORK_TYPE_P2P_INDEX); }

				    } while (FALSE); return rResultStatus; }	

#else
WLAN_STATUS
	p2pFuncBeaconUpdate(IN P_ADAPTER_T prAdapter,
		    IN P_BSS_INFO_T prP2pBssInfo,
		    IN P_P2P_BEACON_UPDATE_INFO_T prBcnUpdateInfo,
		    IN PUINT_8 pucNewBcnHdr,
		    IN UINT_32 u4NewHdrLen, IN PUINT_8 pucNewBcnBody, IN UINT_32 u4NewBodyLen)
{
WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;
P_WLAN_BEACON_FRAME_T prBcnFrame = (P_WLAN_BEACON_FRAME_T) NULL;
P_MSDU_INFO_T prBcnMsduInfo = (P_MSDU_INFO_T) NULL;
PUINT_8 pucIEBuf = (PUINT_8) NULL;
UINT_8 aucIEBuf[MAX_IE_LENGTH];

do {
	ASSERT_BREAK((prAdapter != NULL) && (prP2pBssInfo != NULL) && (prBcnUpdateInfo != NULL));

	prBcnMsduInfo = prP2pBssInfo->prBeacon;

#if DBG
	if (prBcnUpdateInfo->pucBcnHdr != NULL) {
		ASSERT((UINT_32) prBcnUpdateInfo->pucBcnHdr ==
		       ((UINT_32) prBcnMsduInfo->prPacket + MAC_TX_RESERVED_FIELD));
	}

	if (prBcnUpdateInfo->pucBcnBody != NULL) {
		ASSERT((UINT_32) prBcnUpdateInfo->pucBcnBody ==
		       ((UINT_32) prBcnUpdateInfo->pucBcnHdr +
			(UINT_32) prBcnUpdateInfo->u4BcnHdrLen));
	}
#endif
	prBcnFrame =
	    (P_WLAN_BEACON_FRAME_T) ((ULONG) prBcnMsduInfo->prPacket + MAC_TX_RESERVED_FIELD);

	if (!pucNewBcnBody) {
		
		pucNewBcnBody = prBcnUpdateInfo->pucBcnBody;
		ASSERT(u4NewBodyLen == 0);
		u4NewBodyLen = prBcnUpdateInfo->u4BcnBodyLen;
	} else {
		prBcnUpdateInfo->u4BcnBodyLen = u4NewBodyLen;
	}

	
	kalMemCopy(aucIEBuf, pucNewBcnBody, u4NewBodyLen);

	if (pucNewBcnHdr) {
		kalMemCopy(prBcnFrame, pucNewBcnHdr, u4NewHdrLen);
		prBcnUpdateInfo->pucBcnHdr = (PUINT_8) prBcnFrame;
		prBcnUpdateInfo->u4BcnHdrLen = u4NewHdrLen;
	}

	pucIEBuf =
	    (PUINT_8) ((ULONG) prBcnUpdateInfo->pucBcnHdr +
		       (ULONG) prBcnUpdateInfo->u4BcnHdrLen);
	kalMemCopy(pucIEBuf, aucIEBuf, u4NewBodyLen);
	prBcnUpdateInfo->pucBcnBody = pucIEBuf;

	
	prBcnMsduInfo->u2FrameLength =
	    (UINT_16) (prBcnUpdateInfo->u4BcnHdrLen + prBcnUpdateInfo->u4BcnBodyLen);

	prBcnMsduInfo->ucPacketType = TX_PACKET_TYPE_MGMT;
	prBcnMsduInfo->fgIs802_11 = TRUE;
	prBcnMsduInfo->ucBssIndex = prP2pBssInfo->ucBssIndex;


	
	COPY_MAC_ADDR(prP2pBssInfo->aucOwnMacAddr, prBcnFrame->aucSrcAddr);
	COPY_MAC_ADDR(prP2pBssInfo->aucBSSID, prBcnFrame->aucBSSID);
	prP2pBssInfo->u2CapInfo = prBcnFrame->u2CapInfo;

	p2pFuncParseBeaconContent(prAdapter,
				  prP2pBssInfo,
				  (PUINT_8) prBcnFrame->aucInfoElem,
				  (prBcnMsduInfo->u2FrameLength -
				   OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem)));

#if 1
	
#else
	nicUpdateBeaconIETemplate(prAdapter,
				  IE_UPD_METHOD_UPDATE_ALL,
				  NETWORK_TYPE_P2P_INDEX,
				  prBcnFrame->u2CapInfo,
				  (PUINT_8) prBcnFrame->aucInfoElem,
				  (prBcnMsduInfo->u2FrameLength -
				   OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem)));
#endif
} while (FALSE);

return rWlanStatus;
}				

#endif


#if 0
WLAN_STATUS
p2pFuncDeauth(IN P_ADAPTER_T prAdapter,
	      IN PUINT_8 pucPeerMacAddr,
	      IN UINT_16 u2ReasonCode,
	      IN PUINT_8 pucIEBuf, IN UINT_16 u2IELen, IN BOOLEAN fgSendDeauth)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_FAILURE;
	P_STA_RECORD_T prCliStaRec = (P_STA_RECORD_T) NULL;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;
	BOOLEAN fgIsStaFound = FALSE;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucPeerMacAddr != NULL));

		prP2pBssInfo = &(prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX]);

		prCliStaRec = cnmGetStaRecByAddress(prAdapter,
						    NETWORK_TYPE_P2P_INDEX, pucPeerMacAddr);

		switch (prP2pBssInfo->eCurrentOPMode) {
		case OP_MODE_ACCESS_POINT:
			{
				P_LINK_T prStaRecOfClientList = (P_LINK_T) NULL;
				P_LINK_ENTRY_T prLinkEntry = (P_LINK_ENTRY_T) NULL;

				prStaRecOfClientList = &(prP2pBssInfo->rStaRecOfClientList);

				LINK_FOR_EACH(prLinkEntry, prStaRecOfClientList) {
					if ((UINT_32) prCliStaRec == (UINT_32) prLinkEntry) {
						LINK_REMOVE_KNOWN_ENTRY(prStaRecOfClientList,
									&prCliStaRec->rLinkEntry);
						fgIsStaFound = TRUE;
						break;
					}
				}

			}
			break;
		case OP_MODE_INFRASTRUCTURE:
			ASSERT(prCliStaRec == prP2pBssInfo->prStaRecOfAP);
			if (prCliStaRec != prP2pBssInfo->prStaRecOfAP) {
				break;
			}
			prP2pBssInfo->prStaRecOfAP = NULL;
			fgIsStaFound = TRUE;
			break;
		default:
			break;
		}

		if (fgIsStaFound) {
			p2pFuncDisconnect(prAdapter, prCliStaRec, fgSendDeauth, u2ReasonCode);
		}

		rWlanStatus = WLAN_STATUS_SUCCESS;
	} while (FALSE);

	return rWlanStatus;
}				

WLAN_STATUS
p2pFuncDisassoc(IN P_ADAPTER_T prAdapter,
		IN PUINT_8 pucPeerMacAddr,
		IN UINT_16 u2ReasonCode,
		IN PUINT_8 pucIEBuf, IN UINT_16 u2IELen, IN BOOLEAN fgSendDisassoc)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_FAILURE;
	P_STA_RECORD_T prCliStaRec = (P_STA_RECORD_T) NULL;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;
	BOOLEAN fgIsStaFound = FALSE;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucPeerMacAddr != NULL));

		prP2pBssInfo = &(prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX]);

		prCliStaRec = cnmGetStaRecByAddress(prAdapter,
						    NETWORK_TYPE_P2P_INDEX, pucPeerMacAddr);

		switch (prP2pBssInfo->eCurrentOPMode) {
		case OP_MODE_ACCESS_POINT:
			{
				P_LINK_T prStaRecOfClientList = (P_LINK_T) NULL;
				P_LINK_ENTRY_T prLinkEntry = (P_LINK_ENTRY_T) NULL;

				prStaRecOfClientList = &(prP2pBssInfo->rStaRecOfClientList);

				LINK_FOR_EACH(prLinkEntry, prStaRecOfClientList) {
					if ((UINT_32) prCliStaRec == (UINT_32) prLinkEntry) {
						LINK_REMOVE_KNOWN_ENTRY(prStaRecOfClientList,
									&prCliStaRec->rLinkEntry);
						fgIsStaFound = TRUE;
						
						break;
					}
				}

			}
			break;
		case OP_MODE_INFRASTRUCTURE:
			ASSERT(prCliStaRec == prP2pBssInfo->prStaRecOfAP);
			if (prCliStaRec != prP2pBssInfo->prStaRecOfAP) {
				break;
			}
			
			prP2pBssInfo->prStaRecOfAP = NULL;
			fgIsStaFound = TRUE;
			break;
		default:
			break;
		}

		if (fgIsStaFound) {

			p2pFuncDisconnect(prAdapter, prCliStaRec, fgSendDisassoc, u2ReasonCode);
			
			

		}

		rWlanStatus = WLAN_STATUS_SUCCESS;
	} while (FALSE);

	return rWlanStatus;
}				

#endif




VOID
p2pFuncDissolve(IN P_ADAPTER_T prAdapter,
		IN P_BSS_INFO_T prP2pBssInfo, IN BOOLEAN fgSendDeauth, IN UINT_16 u2ReasonCode)
{
	DEBUGFUNC("p2pFuncDissolve()");

	do {

		ASSERT_BREAK((prAdapter != NULL) && (prP2pBssInfo != NULL));

		switch (prP2pBssInfo->eCurrentOPMode) {
		case OP_MODE_INFRASTRUCTURE:
			
			if (prP2pBssInfo->prStaRecOfAP) {
				kalP2PGCIndicateConnectionStatus(prAdapter->prGlueInfo,
								 (UINT_8) prP2pBssInfo->
								 u4PrivateData, NULL, NULL, 0,
								 REASON_CODE_DEAUTH_LEAVING_BSS);

				
				p2pFuncDisconnect(prAdapter,
						  prP2pBssInfo,
						  prP2pBssInfo->prStaRecOfAP,
						  fgSendDeauth, u2ReasonCode);
			}

			
			p2pChangeMediaState(prAdapter, prP2pBssInfo,
					    PARAM_MEDIA_STATE_DISCONNECTED);

			prP2pBssInfo->prStaRecOfAP = NULL;

			break;
		case OP_MODE_ACCESS_POINT:
			{
			    P_STA_RECORD_T prCurrStaRec;

				
				authSendDeauthFrame(prAdapter,
						    prP2pBssInfo,
						    NULL,
						    (P_SW_RFB_T) NULL,
						    u2ReasonCode, (PFN_TX_DONE_HANDLER) NULL);

                prCurrStaRec = bssRemoveHeadClient(prAdapter,
                    prP2pBssInfo);
                while(prCurrStaRec) {
					
					

					p2pFuncDisconnect(prAdapter, prP2pBssInfo, prCurrStaRec,
					    TRUE, u2ReasonCode);

                    prCurrStaRec = bssRemoveHeadClient(prAdapter,
                        prP2pBssInfo);                    
                }
			}

			break;
		default:
			return;	
		}

		
		wlanAcquirePowerControl(prAdapter);
		wlanProcessCommandQueue(prAdapter, &prAdapter->prGlueInfo->rCmdQueue);
		wlanReleasePowerControl(prAdapter);

		kalMdelay(100);

		
		p2pChangeMediaState(prAdapter, prP2pBssInfo, PARAM_MEDIA_STATE_DISCONNECTED);

	} while (FALSE);

	return;
}				


VOID
p2pFuncDisconnect(IN P_ADAPTER_T prAdapter,
		  IN P_BSS_INFO_T prP2pBssInfo,
		  IN P_STA_RECORD_T prStaRec, IN BOOLEAN fgSendDeauth, IN UINT_16 u2ReasonCode)
{
	ENUM_PARAM_MEDIA_STATE_T eOriMediaStatus;

	DBGLOG(P2P, INFO, ("p2pFuncDisconnect()"));

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prStaRec != NULL) && (prP2pBssInfo != NULL));

		ASSERT_BREAK(prP2pBssInfo->eNetworkType == NETWORK_TYPE_P2P);

		ASSERT_BREAK(prP2pBssInfo->ucBssIndex < P2P_DEV_BSS_INDEX);

		eOriMediaStatus = prP2pBssInfo->eConnectionState;

		
		if (prP2pBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {
			P_P2P_ROLE_FSM_INFO_T prP2pRoleFsmInfo =
			    P2P_ROLE_INDEX_2_ROLE_FSM_INFO(prAdapter, prP2pBssInfo->u4PrivateData);

			kalP2PGOStationUpdate(prAdapter->prGlueInfo, prP2pRoleFsmInfo->ucRoleIndex,
					      prStaRec, FALSE);
		} else {
			scanRemoveConnFlagOfBssDescByBssid(prAdapter, prP2pBssInfo->aucBSSID);
		}

		if (fgSendDeauth) {
			
			authSendDeauthFrame(prAdapter,
					    prP2pBssInfo,
					    prStaRec,
					    (P_SW_RFB_T) NULL,
					    u2ReasonCode,
					    (PFN_TX_DONE_HANDLER) p2pRoleFsmRunEventDeauthTxDone);

			
			cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);

			
			p2pFuncResetStaRecStatus(prAdapter, prStaRec);
		} else {
			
			cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);

			
			p2pFuncResetStaRecStatus(prAdapter, prStaRec);

			cnmStaRecFree(prAdapter, prStaRec);

			if ((prP2pBssInfo->eCurrentOPMode != OP_MODE_ACCESS_POINT) ||
			    (bssGetClientCount(prAdapter, prP2pBssInfo) == 0)) {
				DBGLOG(P2P, TRACE, ("No More Client, Media Status DISCONNECTED\n"));
				p2pChangeMediaState(prAdapter, prP2pBssInfo,
						    PARAM_MEDIA_STATE_DISCONNECTED);
			}

			if (eOriMediaStatus != prP2pBssInfo->eConnectionState) {
				
				nicUpdateBss(prAdapter, prP2pBssInfo->ucBssIndex);
			}

		}

		if (prP2pBssInfo->eCurrentOPMode != OP_MODE_ACCESS_POINT) {
			
			nicPmIndicateBssAbort(prAdapter, prP2pBssInfo->ucBssIndex);

			
			rlmBssAborted(prAdapter, prP2pBssInfo);
		}

	} while (FALSE);

	return;

}				





VOID
p2pFuncSetChannel(IN P_ADAPTER_T prAdapter,
		  IN UINT_8 ucRoleIdx, IN P_RF_CHANNEL_INFO_T prRfChannelInfo)
{
	P_P2P_ROLE_FSM_INFO_T prP2pRoleFsmInfo = (P_P2P_ROLE_FSM_INFO_T) NULL;
	P_P2P_CONNECTION_REQ_INFO_T prP2pConnReqInfo = (P_P2P_CONNECTION_REQ_INFO_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prRfChannelInfo != NULL));

		prP2pRoleFsmInfo = P2P_ROLE_INDEX_2_ROLE_FSM_INFO(prAdapter, ucRoleIdx);
		if (!prP2pRoleFsmInfo) {
			break;
		}
		prP2pConnReqInfo = &(prP2pRoleFsmInfo->rConnReqInfo);

		prP2pConnReqInfo->rChannelInfo.ucChannelNum = prRfChannelInfo->ucChannelNum;
		prP2pConnReqInfo->rChannelInfo.eBand = prRfChannelInfo->eBand;

	} while (FALSE);

	return;
}				



BOOLEAN
p2pFuncRetryJOIN(IN P_ADAPTER_T prAdapter,
		 IN P_STA_RECORD_T prStaRec, IN P_P2P_JOIN_INFO_T prJoinInfo)
{
	P_MSG_JOIN_REQ_T prJoinReqMsg = (P_MSG_JOIN_REQ_T) NULL;
	BOOLEAN fgRetValue = FALSE;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prStaRec != NULL) && (prJoinInfo != NULL));

		
		if (!prJoinInfo->ucAvailableAuthTypes) {
			break;
		}

		if (prJoinInfo->ucAvailableAuthTypes & (UINT_8) AUTH_TYPE_SHARED_KEY) {

			DBGLOG(P2P, INFO,
			       ("RETRY JOIN INIT: Retry Authentication with AuthType == SHARED_KEY.\n"));

			prJoinInfo->ucAvailableAuthTypes &= ~(UINT_8) AUTH_TYPE_SHARED_KEY;

			prStaRec->ucAuthAlgNum = (UINT_8) AUTH_ALGORITHM_NUM_SHARED_KEY;
		} else {
			DBGLOG(P2P, ERROR,
			       ("RETRY JOIN INIT: Retry Authentication with Unexpected AuthType.\n"));
			ASSERT(0);
			break;
		}

		prJoinInfo->ucAvailableAuthTypes = 0;	

		
		prJoinReqMsg =
		    (P_MSG_JOIN_REQ_T) cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_JOIN_REQ_T));
		if (!prJoinReqMsg) {
			ASSERT(0);	
			break;
		}

		prJoinReqMsg->rMsgHdr.eMsgId = MID_P2P_SAA_FSM_START;
		prJoinReqMsg->ucSeqNum = ++prJoinInfo->ucSeqNumOfReqMsg;
		prJoinReqMsg->prStaRec = prStaRec;

		mboxSendMsg(prAdapter, MBOX_ID_0, (P_MSG_HDR_T) prJoinReqMsg, MSG_SEND_METHOD_BUF);


		fgRetValue = TRUE;
	} while (FALSE);

	return fgRetValue;



}				




P_BSS_INFO_T p2pFuncBSSIDFindBssInfo(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBSSID)
{
	P_BSS_INFO_T prBssInfo = (P_BSS_INFO_T) NULL;
	UINT_8 ucBssIdx = 0;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBSSID != NULL));

		for (ucBssIdx = 0; ucBssIdx < BSS_INFO_NUM; ucBssIdx++) {
			if (!IS_NET_ACTIVE(prAdapter, ucBssIdx)) {
				continue;
			}

			prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIdx);

			if (EQUAL_MAC_ADDR(prBssInfo->aucBSSID, pucBSSID) && IS_BSS_P2P(prBssInfo)) {
				break;
			}

			prBssInfo = NULL;
		}

	} while (FALSE);

	return prBssInfo;
}				




BOOLEAN
p2pFuncValidateAuth(IN P_ADAPTER_T prAdapter,
		    IN P_BSS_INFO_T prP2pBssInfo,
		    IN P_SW_RFB_T prSwRfb, IN PP_STA_RECORD_T pprStaRec, OUT PUINT_16 pu2StatusCode)
{
	BOOLEAN fgReplyAuth = TRUE;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;
	P_WLAN_AUTH_FRAME_T prAuthFrame = (P_WLAN_AUTH_FRAME_T) NULL;

	DBGLOG(P2P, TRACE, ("p2pValidate Authentication Frame\n"))

	    do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (prP2pBssInfo != NULL) &&
			     (prSwRfb != NULL) && (pprStaRec != NULL) && (pu2StatusCode != NULL));

		
		*pu2StatusCode = STATUS_CODE_REQ_DECLINED;
		prAuthFrame = (P_WLAN_AUTH_FRAME_T) prSwRfb->pvHeader;

		if ((prP2pBssInfo->eCurrentOPMode != OP_MODE_ACCESS_POINT) ||
		    (prP2pBssInfo->eIntendOPMode != OP_MODE_NUM)) {
			
			fgReplyAuth = FALSE;
			DBGLOG(P2P, WARN,
			       ("Current OP mode is not under AP mode. (%d)\n",
				prP2pBssInfo->eCurrentOPMode));
			break;
		}

		prStaRec = cnmGetStaRecByAddress(prAdapter,
						 prP2pBssInfo->ucBssIndex, prAuthFrame->aucSrcAddr);

		if (!prStaRec) {
			prStaRec = cnmStaRecAlloc(prAdapter, STA_TYPE_P2P_GC,
						  prP2pBssInfo->ucBssIndex,
						  prAuthFrame->aucSrcAddr);

			
			ASSERT(prStaRec);

			prSwRfb->ucStaRecIdx = prStaRec->ucIndex;

			prStaRec->u2BSSBasicRateSet = prP2pBssInfo->u2BSSBasicRateSet;

			prStaRec->u2DesiredNonHTRateSet = RATE_SET_ERP_P2P;

			prStaRec->u2OperationalRateSet = RATE_SET_ERP_P2P;
			prStaRec->ucPhyTypeSet = PHY_TYPE_SET_802_11GN;

			
			nicTxUpdateStaRecDefaultRate(prStaRec);

			
			cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);
		} else {
			prSwRfb->ucStaRecIdx = prStaRec->ucIndex;

			if ((prStaRec->ucStaState > STA_STATE_1) && (IS_STA_IN_P2P(prStaRec))) {

				cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);

				p2pFuncResetStaRecStatus(prAdapter, prStaRec);

				bssRemoveClient(prAdapter, prP2pBssInfo, prStaRec);
			}

		}
#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
		if (bssGetClientCount(prAdapter, prP2pBssInfo) >= P2P_MAXIMUM_CLIENT_COUNT ||
		    kalP2PMaxClients(prAdapter->prGlueInfo, 
		        bssGetClientCount(prAdapter, prP2pBssInfo))) {
#else
		if (bssGetClientCount(prAdapter, prP2pBssInfo) >= P2P_MAXIMUM_CLIENT_COUNT) {
#endif
			
			
			DBGLOG(P2P, WARN, ("Group Limit Full. (%d)\n", 
			    bssGetClientCount(prAdapter, prP2pBssInfo)));
			cnmStaRecFree(prAdapter, prStaRec);
			break;
		}
#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
		else {
			
			if (prAuthFrame->aucSrcAddr) {
				if (kalP2PCmpBlackList
				    (prAdapter->prGlueInfo, prAuthFrame->aucSrcAddr)) {
					fgReplyAuth = FALSE;
					return fgReplyAuth;
				}
			}
		}
#endif
		
		prStaRec->eStaType = STA_TYPE_P2P_GC;

		
		prStaRec->u2StatusCode = STATUS_CODE_SUCCESSFUL;

		prStaRec->ucJoinFailureCount = 0;

		*pprStaRec = prStaRec;

		*pu2StatusCode = STATUS_CODE_SUCCESSFUL;

	} while (FALSE);


	return fgReplyAuth;

}				




VOID p2pFuncResetStaRecStatus(IN P_ADAPTER_T prAdapter, IN P_STA_RECORD_T prStaRec)
{
	do {
		if ((prAdapter == NULL) || (prStaRec == NULL)) {
			ASSERT(FALSE);
			break;
		}


		prStaRec->u2StatusCode = STATUS_CODE_SUCCESSFUL;
		prStaRec->u2ReasonCode = REASON_CODE_RESERVED;
		prStaRec->ucJoinFailureCount = 0;
		prStaRec->fgTransmitKeyExist = FALSE;

		prStaRec->fgSetPwrMgtBit = FALSE;

	} while (FALSE);

	return;
}				



VOID
p2pFuncInitConnectionSettings(IN P_ADAPTER_T prAdapter,
			      IN P_P2P_CONNECTION_SETTINGS_T prP2PConnSettings,
			      IN BOOLEAN fgIsApMode)
{
	P_WIFI_VAR_T prWifiVar = NULL;
	ASSERT(prP2PConnSettings);

	prWifiVar = &(prAdapter->rWifiVar);
	ASSERT(prWifiVar);

	prP2PConnSettings->fgIsApMode = fgIsApMode;

#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
	prP2PConnSettings->fgIsWPSMode = prWifiVar->ucApWpsMode;
#endif

	return;
}				





BOOLEAN
p2pFuncValidateAssocReq(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb, OUT PUINT_16 pu2StatusCode)
{
	BOOLEAN fgReplyAssocResp = TRUE;
	P_WLAN_ASSOC_REQ_FRAME_T prAssocReqFrame = (P_WLAN_ASSOC_REQ_FRAME_T) NULL;
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;


	do {
		ASSERT_BREAK((prAdapter != NULL) && (prSwRfb != NULL) && (pu2StatusCode != NULL));

		*pu2StatusCode = STATUS_CODE_REQ_DECLINED;
		prAssocReqFrame = (P_WLAN_ASSOC_REQ_FRAME_T) prSwRfb->pvHeader;

		prP2pBssInfo = p2pFuncBSSIDFindBssInfo(prAdapter, prAssocReqFrame->aucBSSID);

		if (prP2pBssInfo == NULL) {
			DBGLOG(P2P, ERROR, ("RX ASSOC frame without BSS active / BSSID match\n"));
			ASSERT(FALSE);
			break;
		}

		prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);

		if (prStaRec == NULL) {
			
			fgReplyAssocResp = FALSE;
			ASSERT(FALSE);
			break;
		} else {
			ASSERT(prSwRfb->prRxStatusGroup3);
			prStaRec->ucRCPI = HAL_RX_STATUS_GET_RCPI(prSwRfb->prRxStatusGroup3);
		}

		prStaRec->u2DesiredNonHTRateSet &= prP2pBssInfo->u2OperationalRateSet;
		prStaRec->ucDesiredPhyTypeSet = prStaRec->ucPhyTypeSet & prP2pBssInfo->ucPhyTypeSet;

		if (prStaRec->ucDesiredPhyTypeSet == 0) {
			
			*pu2StatusCode = STATUS_CODE_ASSOC_DENIED_RATE_NOT_SUPPORTED;
			break;
		}

		*pu2StatusCode = STATUS_CODE_SUCCESSFUL;

	} while (FALSE);

	return fgReplyAssocResp;

}				




BOOLEAN
p2pFuncParseCheckForP2PInfoElem(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBuf, OUT PUINT_8 pucOuiType)
{
	UINT_8 aucWfaOui[] = VENDOR_OUI_WFA_SPECIFIC;
	P_IE_WFA_T prWfaIE = (P_IE_WFA_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBuf != NULL) && (pucOuiType != NULL));

		prWfaIE = (P_IE_WFA_T) pucBuf;

		if (IE_LEN(pucBuf) <= ELEM_MIN_LEN_WFA_OUI_TYPE_SUBTYPE) {
			break;
		} else if (prWfaIE->aucOui[0] != aucWfaOui[0] ||
			   prWfaIE->aucOui[1] != aucWfaOui[1] ||
			   prWfaIE->aucOui[2] != aucWfaOui[2]) {
			break;
		}

		*pucOuiType = prWfaIE->ucOuiType;

		return TRUE;
	} while (FALSE);

	return FALSE;
}				




BOOLEAN
p2pFuncValidateProbeReq(IN P_ADAPTER_T prAdapter,
			IN P_SW_RFB_T prSwRfb,
			OUT PUINT_32 pu4ControlFlags,
			IN BOOLEAN fgIsDevInterface, IN UINT_8 ucRoleIdx)
{
	BOOLEAN fgIsReplyProbeRsp = FALSE;

	DEBUGFUNC("p2pFuncValidateProbeReq");

	do {

		ASSERT_BREAK((prAdapter != NULL) && (prSwRfb != NULL));

		

		if (prAdapter->u4OsPacketFilter & PARAM_PACKET_FILTER_PROBE_REQ) {
			
			kalP2PIndicateRxMgmtFrame(prAdapter->prGlueInfo, prSwRfb, fgIsDevInterface,
						  ucRoleIdx);
		}

	} while (FALSE);

	return fgIsReplyProbeRsp;

}				



VOID p2pFuncValidateRxActionFrame(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	DEBUGFUNC("p2pFuncValidateRxActionFrame");

	do {

		ASSERT_BREAK((prAdapter != NULL) && (prSwRfb != NULL));

		

		if (PARAM_PACKET_FILTER_ACTION_FRAME) {
			
			kalP2PIndicateRxMgmtFrame(prAdapter->prGlueInfo, prSwRfb, TRUE, 0);
		}

	} while (FALSE);

	return;

}				




BOOLEAN p2pFuncIsAPMode(IN P_P2P_CONNECTION_SETTINGS_T prP2pConnSettings)
{
	if (prP2pConnSettings) {
		if (prP2pConnSettings->fgIsWPSMode == 1) {
			return FALSE;
		}
		return prP2pConnSettings->fgIsApMode;
	} else {
		return FALSE;
	}
}




VOID
p2pFuncParseBeaconContent(IN P_ADAPTER_T prAdapter,
			  IN P_BSS_INFO_T prP2pBssInfo, IN PUINT_8 pucIEInfo, IN UINT_32 u4IELen)
{
	PUINT_8 pucIE = (PUINT_8) NULL;
	UINT_16 u2Offset = 0;
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	BOOL ucNewSecMode = FALSE;
	BOOL ucOldSecMode = FALSE;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prP2pBssInfo != NULL));

		if (u4IELen == 0) {
			break;
		}

		prP2pSpecificBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;
		prP2pSpecificBssInfo->u2AttributeLen = 0;

		ASSERT_BREAK(pucIEInfo != NULL);

		pucIE = pucIEInfo;

		ucOldSecMode = kalP2PGetCipher(prAdapter->prGlueInfo);

		if (prP2pBssInfo->u2CapInfo & MASK_FC_PROTECTED_FRAME) {
			kalP2PSetCipher(prAdapter->prGlueInfo, IW_AUTH_CIPHER_WEP40);
			ucNewSecMode = TRUE;
		}

		IE_FOR_EACH(pucIE, u4IELen, u2Offset) {
			switch (IE_ID(pucIE)) {
			case ELEM_ID_SSID:	
				{

                    
                    
                    

				}
				break;
			case ELEM_ID_SUP_RATES:	
				{
					DBGLOG(P2P, TRACE, ("Support Rate IE\n"));
					kalMemCopy(prP2pBssInfo->aucAllSupportedRates,
						   SUP_RATES_IE(pucIE)->aucSupportedRates,
						   SUP_RATES_IE(pucIE)->ucLength);

					prP2pBssInfo->ucAllSupportedRatesLen =
					    SUP_RATES_IE(pucIE)->ucLength;

					DBGLOG_MEM8(P2P, TRACE,
						    SUP_RATES_IE(pucIE)->aucSupportedRates,
						    SUP_RATES_IE(pucIE)->ucLength);
				}
				break;
			case ELEM_ID_DS_PARAM_SET:	
				{
					DBGLOG(P2P, TRACE,
					       ("DS PARAM IE: %d.\n",
						DS_PARAM_IE(pucIE)->ucCurrChnl));

					

					
				}
				break;
			case ELEM_ID_TIM:	
				TIM_IE(pucIE)->ucDTIMPeriod = prP2pBssInfo->ucDTIMPeriod;
				DBGLOG(P2P, TRACE,
				       ("TIM IE, Len:%d, DTIM:%d\n", IE_LEN(pucIE),
					TIM_IE(pucIE)->ucDTIMPeriod));
				break;
			case ELEM_ID_ERP_INFO:	
				{
#if 1
					
					DBGLOG(P2P, TRACE,
					       ("ERP IE will be over write by driver\n"));
					DBGLOG(P2P, TRACE,
					       ("    ucERP: %x.\n", ERP_INFO_IE(pucIE)->ucERP));

#else
					
					DBGLOG(P2P, TRACE, ("ERP IE.\n"));

					prP2pBssInfo->ucPhyTypeSet |= PHY_TYPE_SET_802_11GN;

					ASSERT(prP2pBssInfo->eBand == BAND_2G4);

					prP2pBssInfo->fgObssErpProtectMode =
					    ((ERP_INFO_IE(pucIE)->
					      ucERP & ERP_INFO_USE_PROTECTION) ? TRUE : FALSE);

					prP2pBssInfo->fgErpProtectMode =
					    ((ERP_INFO_IE(pucIE)->
					      ucERP & (ERP_INFO_USE_PROTECTION |
						       ERP_INFO_NON_ERP_PRESENT)) ? TRUE : FALSE);
#endif

				}
				break;
			case ELEM_ID_HT_CAP:	
				{
#if 1
					DBGLOG(P2P, TRACE,
					       ("HT CAP IE would be overwritten by driver\n"));

					DBGLOG(P2P, TRACE,
					       ("HT Cap Info:%x, AMPDU Param:%x\n",
						HT_CAP_IE(pucIE)->u2HtCapInfo,
						HT_CAP_IE(pucIE)->ucAmpduParam));

					DBGLOG(P2P, TRACE,
					       ("HT Extended Cap Info:%x, TX Beamforming Cap Info:%lx, Ant Selection Cap Info%x\n",
						HT_CAP_IE(pucIE)->u2HtExtendedCap,
						HT_CAP_IE(pucIE)->u4TxBeamformingCap,
						HT_CAP_IE(pucIE)->ucAselCap));
#else
					prP2pBssInfo->ucPhyTypeSet |= PHY_TYPE_SET_802_11N;

					
					if ((HT_CAP_IE(pucIE)->u2HtCapInfo &
					     (HT_CAP_INFO_SUP_CHNL_WIDTH | HT_CAP_INFO_SHORT_GI_40M
					      | HT_CAP_INFO_DSSS_CCK_IN_40M)) == 0) {
						prP2pBssInfo->fgAssoc40mBwAllowed = FALSE;
					} else {
						prP2pBssInfo->fgAssoc40mBwAllowed = TRUE;
					}

					if ((HT_CAP_IE(pucIE)->u2HtCapInfo &
					     (HT_CAP_INFO_SHORT_GI_20M | HT_CAP_INFO_SHORT_GI_40M))
					    == 0) {
						prAdapter->rWifiVar.rConnSettings.
						    fgRxShortGIDisabled = TRUE;
					} else {
						prAdapter->rWifiVar.rConnSettings.
						    fgRxShortGIDisabled = FALSE;
					}

					
					DBGLOG(P2P, TRACE,
					       ("AMPDU setting from supplicant:0x%x, & default value:0x%x\n",
						(UINT_8) HT_CAP_IE(pucIE)->ucAmpduParam,
						(UINT_8) AMPDU_PARAM_DEFAULT_VAL));

					
					
					

					
					ASSERT(HT_CAP_IE(pucIE)->u2HtExtendedCap ==
					       (HT_EXT_CAP_DEFAULT_VAL &
						~(HT_EXT_CAP_PCO |
						  HT_EXT_CAP_PCO_TRANS_TIME_NONE)));

					
					ASSERT(HT_CAP_IE(pucIE)->u4TxBeamformingCap ==
					       TX_BEAMFORMING_CAP_DEFAULT_VAL);

					
					ASSERT(HT_CAP_IE(pucIE)->ucAselCap == ASEL_CAP_DEFAULT_VAL);
#endif
				}
				break;
			case ELEM_ID_RSN:	
				{
					RSN_INFO_T rRsnIe;

					DBGLOG(P2P, TRACE, ("RSN IE\n"));
					kalP2PSetCipher(prAdapter->prGlueInfo, IW_AUTH_CIPHER_CCMP);
					ucNewSecMode = TRUE;

					if (rsnParseRsnIE(prAdapter, RSN_IE(pucIE), &rRsnIe)) {
						prP2pBssInfo->u4RsnSelectedGroupCipher =
						    RSN_CIPHER_SUITE_CCMP;
						prP2pBssInfo->u4RsnSelectedPairwiseCipher =
						    RSN_CIPHER_SUITE_CCMP;
						prP2pBssInfo->u4RsnSelectedAKMSuite =
						    RSN_AKM_SUITE_PSK;
						prP2pBssInfo->u2RsnSelectedCapInfo =
						    rRsnIe.u2RsnCap;
					}
				}
				break;
			case ELEM_ID_EXTENDED_SUP_RATES:	
				
				DBGLOG(P2P, TRACE, ("Ex Support Rate IE\n"));
				kalMemCopy(&
					   (prP2pBssInfo->
					    aucAllSupportedRates[prP2pBssInfo->
								 ucAllSupportedRatesLen]),
					   EXT_SUP_RATES_IE(pucIE)->aucExtSupportedRates,
					   EXT_SUP_RATES_IE(pucIE)->ucLength);

				DBGLOG_MEM8(P2P, TRACE,
					    EXT_SUP_RATES_IE(pucIE)->aucExtSupportedRates,
					    EXT_SUP_RATES_IE(pucIE)->ucLength);

				prP2pBssInfo->ucAllSupportedRatesLen +=
				    EXT_SUP_RATES_IE(pucIE)->ucLength;
				break;
			case ELEM_ID_HT_OP:
			  
				{
#if 1
					DBGLOG(P2P, TRACE,
					       ("HT OP IE would be overwritten by driver\n"));

					DBGLOG(P2P, TRACE,
					       ("    Primary Channel: %x, Info1: %x, Info2: %x, Info3: %x\n",
						HT_OP_IE(pucIE)->ucPrimaryChannel,
						HT_OP_IE(pucIE)->ucInfo1, HT_OP_IE(pucIE)->u2Info2,
						HT_OP_IE(pucIE)->u2Info3));
#else
					UINT_16 u2Info2 = 0;
					prP2pBssInfo->ucPhyTypeSet |= PHY_TYPE_SET_802_11N;

					DBGLOG(P2P, TRACE, ("HT OP IE\n"));

					
					ASSERT(HT_OP_IE(pucIE)->ucPrimaryChannel ==
					       prP2pBssInfo->ucPrimaryChannel);

					
					prP2pBssInfo->ucHtOpInfo1 = HT_OP_IE(pucIE)->ucInfo1;

					
					u2Info2 = HT_OP_IE(pucIE)->u2Info2;

					if (u2Info2 & HT_OP_INFO2_NON_GF_HT_STA_PRESENT) {
						ASSERT(prP2pBssInfo->eGfOperationMode !=
						       GF_MODE_NORMAL);
						u2Info2 &= ~HT_OP_INFO2_NON_GF_HT_STA_PRESENT;
					}

					if (u2Info2 & HT_OP_INFO2_OBSS_NON_HT_STA_PRESENT) {
						prP2pBssInfo->eObssHtProtectMode =
						    HT_PROTECT_MODE_NON_MEMBER;
						u2Info2 &= ~HT_OP_INFO2_OBSS_NON_HT_STA_PRESENT;
					}

					switch (u2Info2 & HT_OP_INFO2_HT_PROTECTION) {
					case HT_PROTECT_MODE_NON_HT:
						prP2pBssInfo->eHtProtectMode =
						    HT_PROTECT_MODE_NON_HT;
						break;
					case HT_PROTECT_MODE_NON_MEMBER:
						prP2pBssInfo->eHtProtectMode = HT_PROTECT_MODE_NONE;
						prP2pBssInfo->eObssHtProtectMode =
						    HT_PROTECT_MODE_NON_MEMBER;
						break;
					default:
						prP2pBssInfo->eHtProtectMode =
						    HT_OP_IE(pucIE)->u2Info2;
						break;
					}

					
					prP2pBssInfo->u2HtOpInfo3 = HT_OP_IE(pucIE)->u2Info3;

					
					DBGLOG_MEM8(P2P, TRACE, HT_OP_IE(pucIE)->aucBasicMcsSet,
						    16);
#endif
				}
				break;
			case ELEM_ID_OBSS_SCAN_PARAMS:	
				{
					DBGLOG(P2P, TRACE,
					       ("ELEM_ID_OBSS_SCAN_PARAMS IE would be replaced by driver\n"));
				}
				break;
			case ELEM_ID_EXTENDED_CAP:	
				{
					DBGLOG(P2P, TRACE,
					       ("ELEM_ID_EXTENDED_CAP IE would be replaced by driver\n"));
				}
				break;
			case ELEM_ID_VENDOR:	
				DBGLOG(P2P, TRACE, ("Vender Specific IE\n"));
				{
					p2pFuncParseBeaconVenderId(prAdapter, pucIE, prP2pSpecificBssInfo, &ucNewSecMode);
					
				}
				break;
			default:
				DBGLOG(P2P, TRACE, ("Unprocessed element ID:%d\n", IE_ID(pucIE)));
				break;
			}
		}

		if (!ucNewSecMode && ucOldSecMode)
			kalP2PSetCipher(prAdapter->prGlueInfo, IW_AUTH_CIPHER_NONE);

	} while (FALSE);

	return;
}				


static VOID
p2pFuncParseBeaconVenderId(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucIE, IN P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo, OUT BOOL *ucNewSecMode)
{
	do {
		UINT_8 ucOuiType;
		UINT_16 u2SubTypeVersion;
		if (rsnParseCheckForWFAInfoElem (prAdapter, pucIE, &ucOuiType, &u2SubTypeVersion)) {
			if ((ucOuiType == VENDOR_OUI_TYPE_WPA) && (u2SubTypeVersion == VERSION_WPA)) {
				kalP2PSetCipher(prAdapter->prGlueInfo, IW_AUTH_CIPHER_TKIP);
				*ucNewSecMode = TRUE;
		                kalMemCopy(prP2pSpecificBssInfo->aucWpaIeBuffer, pucIE, IE_SIZE(pucIE));
                		prP2pSpecificBssInfo->u2WpaIeLen = IE_SIZE(pucIE);
				DBGLOG(P2P, TRACE, ("WPA IE in supplicant\n"));
			} else if ((ucOuiType == VENDOR_OUI_TYPE_WPS)) {
				kalP2PUpdateWSC_IE(prAdapter->prGlueInfo, 0, pucIE, IE_SIZE(pucIE));
				DBGLOG(P2P, TRACE, ("WPS IE in supplicant\n"));
			} else if ((ucOuiType == VENDOR_OUI_TYPE_WMM)) {
				DBGLOG(P2P, TRACE, ("WMM IE in supplicant\n"));
			}
			
		} else
		    if (p2pFuncParseCheckForP2PInfoElem
			(prAdapter, pucIE, &ucOuiType)) {
			
			
			if (ucOuiType == VENDOR_OUI_TYPE_P2P) {
				kalMemCopy(&prP2pSpecificBssInfo->aucAttributesCache[prP2pSpecificBssInfo->u2AttributeLen],
					pucIE,
					IE_SIZE(pucIE));
				prP2pSpecificBssInfo->u2AttributeLen += IE_SIZE(pucIE);
				DBGLOG(P2P, TRACE, ("P2P IE in supplicant\n"));
			} else if (ucOuiType == VENDOR_OUI_TYPE_WFD) {

				kalMemCopy(&prP2pSpecificBssInfo->aucAttributesCache[prP2pSpecificBssInfo->u2AttributeLen],
					pucIE,
					IE_SIZE(pucIE));

				prP2pSpecificBssInfo->u2AttributeLen += IE_SIZE(pucIE);
			} else {
				DBGLOG(P2P, TRACE, ("Unknown 50-6F-9A-%d IE.\n", ucOuiType));
			}
		} else {
			kalMemCopy(&prP2pSpecificBssInfo->aucAttributesCache[prP2pSpecificBssInfo->u2AttributeLen],
				pucIE,
				IE_SIZE(pucIE));

			prP2pSpecificBssInfo->u2AttributeLen += IE_SIZE(pucIE);
			DBGLOG(P2P, TRACE, ("Driver unprocessed Vender Specific IE\n"));
			ASSERT(FALSE);
		}
	} while (0);
}

P_BSS_DESC_T
p2pFuncKeepOnConnection(IN P_ADAPTER_T prAdapter,
			IN P_BSS_INFO_T prBssInfo,
			IN P_P2P_CONNECTION_REQ_INFO_T prConnReqInfo,
			IN P_P2P_CHNL_REQ_INFO_T prChnlReqInfo,
			IN P_P2P_SCAN_REQ_INFO_T prScanReqInfo)
{
	P_BSS_DESC_T prTargetBss = (P_BSS_DESC_T) NULL;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (prConnReqInfo != NULL) &&
			     (prChnlReqInfo != NULL) && (prScanReqInfo != NULL));

		if (prP2pBssInfo->eCurrentOPMode != OP_MODE_INFRASTRUCTURE) {
			break;
		}
		
		ASSERT(prConnReqInfo->eConnRequest == P2P_CONNECTION_TYPE_GC);

		
		prTargetBss = scanP2pSearchDesc(prAdapter, prConnReqInfo);

		if (prTargetBss == NULL) {
			
			
			prScanReqInfo->ucNumChannelList = 1;
			prScanReqInfo->eScanType = SCAN_TYPE_ACTIVE_SCAN;
			prScanReqInfo->eChannelSet = SCAN_CHANNEL_FULL;
			prScanReqInfo->u4BufLength = 0;	
			prScanReqInfo->fgIsAbort = TRUE;
		} else {
			prChnlReqInfo->u8Cookie = 0;
			prChnlReqInfo->ucReqChnlNum = prTargetBss->ucChannelNum;
			prChnlReqInfo->eBand = prTargetBss->eBand;
			prChnlReqInfo->eChnlSco = prTargetBss->eSco;
			prChnlReqInfo->u4MaxInterval = AIS_JOIN_CH_REQUEST_INTERVAL;
			prChnlReqInfo->eChnlReqType = CH_REQ_TYPE_JOIN;

			prChnlReqInfo->eChannelWidth = prTargetBss->eChannelWidth;
			prChnlReqInfo->ucCenterFreqS1 = prTargetBss->ucCenterFreqS1;
			prChnlReqInfo->ucCenterFreqS2 = prTargetBss->ucCenterFreqS2;
		}

	} while (FALSE);

	return prTargetBss;
}				

VOID
p2pFuncStoreAssocRspIEBuffer(IN P_ADAPTER_T prAdapter,
			     IN P_P2P_JOIN_INFO_T prP2pJoinInfo, IN P_SW_RFB_T prSwRfb)
{
	P_WLAN_ASSOC_RSP_FRAME_T prAssocRspFrame = (P_WLAN_ASSOC_RSP_FRAME_T) NULL;
	INT_16 i2IELen = 0;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prP2pJoinInfo != NULL) && (prSwRfb != NULL));

		prAssocRspFrame = (P_WLAN_ASSOC_RSP_FRAME_T) prSwRfb->pvHeader;

		if (prAssocRspFrame->u2FrameCtrl != MAC_FRAME_ASSOC_RSP) {
			break;
		}

		i2IELen = prSwRfb->u2PacketLen - (WLAN_MAC_HEADER_LEN +
						  CAP_INFO_FIELD_LEN +
						  STATUS_CODE_FIELD_LEN + AID_FIELD_LEN);


		if (i2IELen <= 0) {
			break;
		}

		prP2pJoinInfo->u4BufLength = (UINT_32) i2IELen;

		kalMemCopy(prP2pJoinInfo->aucIEBuf, prAssocRspFrame->aucInfoElem,
			   prP2pJoinInfo->u4BufLength);

	} while (FALSE);


	return;
}				




VOID
p2pFuncMgmtFrameRegister(IN P_ADAPTER_T prAdapter,
			 IN UINT_16 u2FrameType,
			 IN BOOLEAN fgIsRegistered, OUT PUINT_32 pu4P2pPacketFilter)
{
	UINT_32 u4NewPacketFilter = 0;

	DEBUGFUNC("p2pFuncMgmtFrameRegister");

	do {
		ASSERT_BREAK(prAdapter != NULL);

		if (pu4P2pPacketFilter) {
			u4NewPacketFilter = *pu4P2pPacketFilter;
		}

		switch (u2FrameType) {
		case MAC_FRAME_PROBE_REQ:
			if (fgIsRegistered) {
				u4NewPacketFilter |= PARAM_PACKET_FILTER_PROBE_REQ;
				DBGLOG(P2P, TRACE, ("Open packet filer probe request\n"));
			} else {
				u4NewPacketFilter &= ~PARAM_PACKET_FILTER_PROBE_REQ;
				DBGLOG(P2P, TRACE, ("Close packet filer probe request\n"));
			}
			break;
		case MAC_FRAME_ACTION:
			if (fgIsRegistered) {
				u4NewPacketFilter |= PARAM_PACKET_FILTER_ACTION_FRAME;
				DBGLOG(P2P, TRACE, ("Open packet filer action frame.\n"));
			} else {
				u4NewPacketFilter &= ~PARAM_PACKET_FILTER_ACTION_FRAME;
				DBGLOG(P2P, TRACE, ("Close packet filer action frame.\n"));
			}
			break;
		default:
			DBGLOG(P2P, TRACE, ("Ask frog to add code for mgmt:%x\n", u2FrameType));
			break;
		}

		if (pu4P2pPacketFilter) {
			*pu4P2pPacketFilter = u4NewPacketFilter;
		}

		prAdapter->u4OsPacketFilter &= ~PARAM_PACKET_FILTER_P2P_MASK;
		prAdapter->u4OsPacketFilter |= u4NewPacketFilter;

		DBGLOG(P2P, TRACE, ("P2P Set PACKET filter:0x%lx\n", prAdapter->u4OsPacketFilter));

		wlanSendSetQueryCmd(prAdapter,
				    CMD_ID_SET_RX_FILTER,
				    TRUE,
				    FALSE,
				    FALSE,
				    nicCmdEventSetCommon,
				    nicOidCmdTimeoutCommon,
				    sizeof(UINT_32),
				    (PUINT_8) &prAdapter->u4OsPacketFilter,
				    &u4NewPacketFilter, sizeof(u4NewPacketFilter)
		    );

	} while (FALSE);

	return;
}				


VOID p2pFuncUpdateMgmtFrameRegister(IN P_ADAPTER_T prAdapter, IN UINT_32 u4OsFilter)
{

	do {

		
		

		if ((prAdapter->u4OsPacketFilter & PARAM_PACKET_FILTER_P2P_MASK) ^ u4OsFilter) {

			prAdapter->u4OsPacketFilter &= ~PARAM_PACKET_FILTER_P2P_MASK;

			prAdapter->u4OsPacketFilter |= (u4OsFilter & PARAM_PACKET_FILTER_P2P_MASK);

			wlanSendSetQueryCmd(prAdapter,
					    CMD_ID_SET_RX_FILTER,
					    TRUE,
					    FALSE,
					    FALSE,
					    nicCmdEventSetCommon,
					    nicOidCmdTimeoutCommon,
					    sizeof(UINT_32),
					    (PUINT_8) &prAdapter->u4OsPacketFilter,
					    &u4OsFilter, sizeof(u4OsFilter)
			    );
			DBGLOG(P2P, TRACE,
			       ("P2P Set PACKET filter:0x%lx\n", prAdapter->u4OsPacketFilter));
		}

	} while (FALSE);




	return;
}				


VOID
p2pFuncGetStationInfo(IN P_ADAPTER_T prAdapter,
		      IN PUINT_8 pucMacAddr, OUT P_P2P_STATION_INFO_T prStaInfo)
{

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucMacAddr != NULL) && (prStaInfo != NULL));

		prStaInfo->u4InactiveTime = 0;
		prStaInfo->u4RxBytes = 0;
		prStaInfo->u4TxBytes = 0;
		prStaInfo->u4RxPackets = 0;
		prStaInfo->u4TxPackets = 0;
		

	} while (FALSE);

	return;
}				

BOOLEAN
p2pFuncGetAttriList(IN P_ADAPTER_T prAdapter,
		    IN UINT_8 ucOuiType,
		    IN PUINT_8 pucIE,
		    IN UINT_16 u2IELength, OUT PPUINT_8 ppucAttriList, OUT PUINT_16 pu2AttriListLen)
{
	BOOLEAN fgIsAllocMem = FALSE;
	UINT_8 aucWfaOui[] = VENDOR_OUI_WFA_SPECIFIC;
	UINT_16 u2Offset = 0;
	P_IE_P2P_T prIe = (P_IE_P2P_T) NULL;
	PUINT_8 pucAttriListStart = (PUINT_8) NULL;
	UINT_16 u2AttriListLen = 0, u2BufferSize;
	BOOLEAN fgBackupAttributes = FALSE;

	u2BufferSize = 0;

	do {
		ASSERT_BREAK((prAdapter != NULL) &&
			     (pucIE != NULL) &&
			     (u2IELength != 0) &&
			     (ppucAttriList != NULL) && (pu2AttriListLen != NULL));

		if (ppucAttriList) {
			*ppucAttriList = NULL;
		}
		if (pu2AttriListLen) {
			*pu2AttriListLen = 0;
		}

		if (ucOuiType == VENDOR_OUI_TYPE_WPS) {
			aucWfaOui[0] = 0x00;
			aucWfaOui[1] = 0x50;
			aucWfaOui[2] = 0xF2;
		} else if ((ucOuiType != VENDOR_OUI_TYPE_P2P)
#if CFG_SUPPORT_WFD
			   && (ucOuiType != VENDOR_OUI_TYPE_WFD)
#endif
		    ) {
			DBGLOG(P2P, INFO, ("Not supported OUI Type to parsing 0x%x\n", ucOuiType));
			break;
		}


		IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
			if (ELEM_ID_VENDOR == IE_ID(pucIE)) {
				prIe = (P_IE_P2P_T) pucIE;

				if (prIe->ucLength <= P2P_OUI_TYPE_LEN) {
					continue;

				}

				if ((prIe->aucOui[0] == aucWfaOui[0]) &&
				    (prIe->aucOui[1] == aucWfaOui[1]) &&
				    (prIe->aucOui[2] == aucWfaOui[2]) &&
				    (ucOuiType == prIe->ucOuiType)) {
					p2pFuncGetAttriListAction(prAdapter,
						prIe, ucOuiType,
						&pucAttriListStart, &u2AttriListLen,
						&fgIsAllocMem, &fgBackupAttributes, &u2BufferSize);
				}	
			}	
		}		


	} while (FALSE);

	if (pucAttriListStart) {
		PUINT_8 pucAttribute = pucAttriListStart;
		DBGLOG(P2P, LOUD, ("Checking Attribute Length.\n"));
		if (ucOuiType == VENDOR_OUI_TYPE_P2P) {
			P2P_ATTRI_FOR_EACH(pucAttribute, u2AttriListLen, u2Offset);
		} else if (ucOuiType == VENDOR_OUI_TYPE_WFD) {
		} else if (ucOuiType == VENDOR_OUI_TYPE_WPS) {
			
			WSC_ATTRI_FOR_EACH(pucAttribute, u2AttriListLen, u2Offset) {
				DBGLOG(P2P, LOUD, ("Attribute ID:%d, Length:%d.\n",
						   WSC_ATTRI_ID(pucAttribute),
						   WSC_ATTRI_LEN(pucAttribute)));
			}
		} else {
		}

		ASSERT(u2Offset == u2AttriListLen);

		*ppucAttriList = pucAttriListStart;
		*pu2AttriListLen = u2AttriListLen;

	} else {
		*ppucAttriList = (PUINT_8) NULL;
		*pu2AttriListLen = 0;
	}

	return fgIsAllocMem;
}				

static VOID
p2pFuncGetAttriListAction(IN P_ADAPTER_T prAdapter,
	IN P_IE_P2P_T prIe, IN UINT_8 ucOuiType,
	OUT PUINT_8 *pucAttriListStart, OUT UINT_16 *u2AttriListLen,
	OUT BOOLEAN *fgIsAllocMem, OUT BOOLEAN *fgBackupAttributes, OUT UINT_16 *u2BufferSize)
{
	do {
		if (!(*pucAttriListStart)) {
			*pucAttriListStart = &prIe->aucP2PAttributes[0];
			if (prIe->ucLength > P2P_OUI_TYPE_LEN) {
				*u2AttriListLen = (UINT_16) (prIe->ucLength - P2P_OUI_TYPE_LEN);
			} else {
				ASSERT(FALSE);
			}
		} else {
			
			UINT_16 u2CopyLen;
			if (FALSE == *fgBackupAttributes) {
				P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

				*fgBackupAttributes = TRUE;
				if (ucOuiType == VENDOR_OUI_TYPE_P2P) {
					kalMemCopy(&prP2pSpecificBssInfo->aucAttributesCache[0],
						   *pucAttriListStart,
						   *u2AttriListLen);

					*pucAttriListStart = &prP2pSpecificBssInfo->aucAttributesCache[0];
					*u2BufferSize = P2P_MAXIMUM_ATTRIBUTE_LEN;
				} else if (ucOuiType == VENDOR_OUI_TYPE_WPS) {
					kalMemCopy(&prP2pSpecificBssInfo->aucWscAttributesCache[0],
						   *pucAttriListStart,
						   *u2AttriListLen);
					*pucAttriListStart = &prP2pSpecificBssInfo->aucWscAttributesCache[0];
					*u2BufferSize = WPS_MAXIMUM_ATTRIBUTES_CACHE_SIZE;
				}
#if CFG_SUPPORT_WFD
				else if (ucOuiType == VENDOR_OUI_TYPE_WFD) {
					PUINT_8 pucTmpBuf = (PUINT_8) NULL;
					pucTmpBuf = (PUINT_8)kalMemAlloc
					    (WPS_MAXIMUM_ATTRIBUTES_CACHE_SIZE,
					     VIR_MEM_TYPE);

					if (pucTmpBuf != NULL) {
						*fgIsAllocMem = TRUE;
					} else {
						
						ASSERT(FALSE);
						break;
					}

					kalMemCopy(pucTmpBuf,
						   *pucAttriListStart,
						   *u2AttriListLen);
					*pucAttriListStart = pucTmpBuf;
					*u2BufferSize = WPS_MAXIMUM_ATTRIBUTES_CACHE_SIZE;
				}
#endif
				else {
					*fgBackupAttributes = FALSE;
				}
			}
			u2CopyLen = (UINT_16)(prIe->ucLength - P2P_OUI_TYPE_LEN);

			if (((*u2AttriListLen) + u2CopyLen) > (*u2BufferSize)) {
				u2CopyLen = (*u2BufferSize) - (*u2AttriListLen);
				DBGLOG(P2P, WARN,
				       ("Length of received P2P attributes > maximum cache size.\n"));
			}

			if (u2CopyLen) {
				kalMemCopy((PUINT_8)((ULONG)(*pucAttriListStart) + (ULONG)(*u2AttriListLen)),
					   &prIe->aucP2PAttributes[0],
					   u2CopyLen);
				*u2AttriListLen += u2CopyLen;
			}

		}
	} while (0);
}
P_MSDU_INFO_T
p2pFuncProcessP2pProbeRsp(IN P_ADAPTER_T prAdapter,
			  IN UINT_8 ucBssIdx, IN P_MSDU_INFO_T prMgmtTxMsdu)
{
	P_MSDU_INFO_T prRetMsduInfo = prMgmtTxMsdu;
	P_WLAN_PROBE_RSP_FRAME_T prProbeRspFrame = (P_WLAN_PROBE_RSP_FRAME_T) NULL;
	PUINT_8 pucIEBuf = (PUINT_8) NULL;
	UINT_16 u2Offset = 0, u2IELength = 0, u2ProbeRspHdrLen = 0;
	BOOLEAN fgIsP2PIE = FALSE, fgIsWSCIE = FALSE;
	BOOLEAN fgIsWFDIE = FALSE;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;
	UINT_16 u2EstimateSize = 0, u2EstimatedExtraIELen = 0;
	UINT_32 u4IeArraySize = 0, u4Idx = 0;


	do {
		ASSERT_BREAK((prAdapter != NULL) && (prMgmtTxMsdu != NULL));

		prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIdx);

		
		prProbeRspFrame =
		    (P_WLAN_PROBE_RSP_FRAME_T) ((ULONG) prMgmtTxMsdu->prPacket +
						MAC_TX_RESERVED_FIELD);
		ASSERT_BREAK((prProbeRspFrame->u2FrameCtrl & MASK_FRAME_TYPE) ==
			     MAC_FRAME_PROBE_RSP);

		
		u2ProbeRspHdrLen =
		    (WLAN_MAC_MGMT_HEADER_LEN + TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN +
		     CAP_INFO_FIELD_LEN);
		pucIEBuf = prProbeRspFrame->aucInfoElem;
		u2IELength = prMgmtTxMsdu->u2FrameLength - u2ProbeRspHdrLen;

#if CFG_SUPPORT_WFD
		
		
		prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen = 0;
#endif

		IE_FOR_EACH(pucIEBuf, u2IELength, u2Offset) {
			switch (IE_ID(pucIEBuf)) {
			case ELEM_ID_SSID:
				{
					p2pFuncProcessP2pProbeRspAction(prAdapter, pucIEBuf, ELEM_ID_SSID,
						&ucBssIdx, &prP2pBssInfo, &fgIsWSCIE,
						&fgIsP2PIE, &fgIsWFDIE);
				}
				break;
			case ELEM_ID_VENDOR:
				{
					p2pFuncProcessP2pProbeRspAction(prAdapter, pucIEBuf, ELEM_ID_VENDOR,
						&ucBssIdx, &prP2pBssInfo, &fgIsWSCIE,
						&fgIsP2PIE, &fgIsWFDIE);
				}
				break;
			default:
				break;
			}

		}


		
		u2EstimateSize = WLAN_MAC_MGMT_HEADER_LEN +
		    TIMESTAMP_FIELD_LEN +
		    BEACON_INTERVAL_FIELD_LEN +
		    CAP_INFO_FIELD_LEN +
		    (ELEM_HDR_LEN + ELEM_MAX_LEN_SSID) +
		    (ELEM_HDR_LEN + ELEM_MAX_LEN_SUP_RATES) +
		    (ELEM_HDR_LEN + ELEM_MAX_LEN_DS_PARAMETER_SET);

		u2EstimatedExtraIELen = 0;

		u4IeArraySize = sizeof(txProbeRspIETable) / sizeof(APPEND_VAR_IE_ENTRY_T);
		for (u4Idx = 0; u4Idx < u4IeArraySize; u4Idx++) {
			if (txProbeRspIETable[u4Idx].u2EstimatedFixedIELen) {
				u2EstimatedExtraIELen +=
				    txProbeRspIETable[u4Idx].u2EstimatedFixedIELen;
			}

			else {
				ASSERT(txProbeRspIETable[u4Idx].pfnCalculateVariableIELen);

				u2EstimatedExtraIELen +=
				    (UINT_16) (txProbeRspIETable[u4Idx].
					       pfnCalculateVariableIELen(prAdapter, ucBssIdx,
									 NULL));
			}

		}


		if (fgIsWSCIE) {
			u2EstimatedExtraIELen += kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 2);
		}

		if (fgIsP2PIE) {
			u2EstimatedExtraIELen += kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 1);
            u2EstimatedExtraIELen += p2pFuncCalculateP2P_IE_NoA(prAdapter, ucBssIdx, NULL);
		}
#if CFG_SUPPORT_WFD
		ASSERT(sizeof(prAdapter->prGlueInfo->prP2PInfo->aucWFDIE) >=
		       prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen);
		if (fgIsWFDIE) {
			u2EstimatedExtraIELen += prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen;
		}
#if 0
		u2EstimatedExtraIELen += prAdapter->prGlueInfo->prP2PInfo->u2VenderIELen;
#endif
#endif


		u2EstimateSize += u2EstimatedExtraIELen;
		if ((u2EstimateSize) > (prRetMsduInfo->u2FrameLength)) {
			prRetMsduInfo = cnmMgtPktAlloc(prAdapter, u2EstimateSize);

			if (prRetMsduInfo == NULL) {
				DBGLOG(P2P, WARN,
				       ("No packet for sending new probe response, use original one\n"));
				prRetMsduInfo = prMgmtTxMsdu;
				break;
			}

		}

		prRetMsduInfo->ucBssIndex = ucBssIdx;


		
		bssComposeBeaconProbeRespFrameHeaderAndFF((PUINT_8)
							  ((ULONG) (prRetMsduInfo->prPacket) +
							   MAC_TX_RESERVED_FIELD),
							  prProbeRspFrame->aucDestAddr,
							  prProbeRspFrame->aucSrcAddr,
							  prProbeRspFrame->aucBSSID,
							  prProbeRspFrame->u2BeaconInterval,
							  prProbeRspFrame->u2CapInfo);

		prRetMsduInfo->u2FrameLength =
		    (WLAN_MAC_MGMT_HEADER_LEN + TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN +
		     CAP_INFO_FIELD_LEN);

		bssBuildBeaconProbeRespFrameCommonIEs(prRetMsduInfo,
						      prP2pBssInfo, prProbeRspFrame->aucDestAddr);


		for (u4Idx = 0; u4Idx < u4IeArraySize; u4Idx++) {
			if (txProbeRspIETable[u4Idx].pfnAppendIE) {
				txProbeRspIETable[u4Idx].pfnAppendIE(prAdapter, prRetMsduInfo);
			}

		}

		if (fgIsWSCIE) {
			kalP2PGenWSC_IE(prAdapter->prGlueInfo,
					2,
					(PUINT_8) ((ULONG) prRetMsduInfo->prPacket +
						   (ULONG) prRetMsduInfo->u2FrameLength));

			prRetMsduInfo->u2FrameLength +=
			    (UINT_16) kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 2);
		}

		if (fgIsP2PIE) {
			kalP2PGenWSC_IE(prAdapter->prGlueInfo,
					1,
					(PUINT_8) ((ULONG) prRetMsduInfo->prPacket +
						   (ULONG) prRetMsduInfo->u2FrameLength));

			prRetMsduInfo->u2FrameLength +=
			    (UINT_16) kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 1);
			p2pFuncGenerateP2P_IE_NoA(prAdapter, prRetMsduInfo);

		}
#if CFG_SUPPORT_WFD

		if (fgIsWFDIE > 0) {
			ASSERT(prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen > 0);
			kalMemCopy((PUINT_8)
				   ((ULONG) prRetMsduInfo->prPacket +
				    (ULONG) prRetMsduInfo->u2FrameLength),
				   prAdapter->prGlueInfo->prP2PInfo->aucWFDIE,
				   prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen);
			prRetMsduInfo->u2FrameLength +=
			    (UINT_16) prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen;

		}
#if 0
		if (prAdapter->prGlueInfo->prP2PInfo->u2VenderIELen > 0) {
			kalMemCopy((PUINT_8)
				   ((UINT_32) prRetMsduInfo->prPacket +
				    (UINT_32) prRetMsduInfo->u2FrameLength),
				   prAdapter->prGlueInfo->prP2PInfo->aucVenderIE,
				   prAdapter->prGlueInfo->prP2PInfo->u2VenderIELen);
			prRetMsduInfo->u2FrameLength +=
			    (UINT_16) prAdapter->prGlueInfo->prP2PInfo->u2VenderIELen;
		}
#endif
#endif				


	} while (FALSE);

	if (prRetMsduInfo != prMgmtTxMsdu) {
		cnmMgtPktFree(prAdapter, prMgmtTxMsdu);
	}


	return prRetMsduInfo;
}				

static VOID
p2pFuncProcessP2pProbeRspAction(IN P_ADAPTER_T prAdapter,
		IN PUINT_8 pucIEBuf, IN UINT_8 ucElemIdType,
		OUT UINT_8 *ucBssIdx, OUT P_BSS_INFO_T *prP2pBssInfo, OUT BOOLEAN *fgIsWSCIE,
		OUT BOOLEAN *fgIsP2PIE, OUT BOOLEAN *fgIsWFDIE)
{
	switch (ucElemIdType) {
	case ELEM_ID_SSID:
		{
			if (SSID_IE(pucIEBuf)->ucLength > 7) {
				for ((*ucBssIdx) = 0; (*ucBssIdx) < MAX_BSS_INDEX; (*ucBssIdx)++) {
					*prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, *ucBssIdx);
					if (!(*prP2pBssInfo)) {
						continue;
					}
					if (EQUAL_SSID((*prP2pBssInfo)->aucSSID,
							(*prP2pBssInfo)->ucSSIDLen,
							SSID_IE(pucIEBuf)->aucSSID,
							SSID_IE(pucIEBuf)->ucLength)) {
						break;
					}
				}
				if ((*ucBssIdx) == P2P_DEV_BSS_INDEX) {
					*prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, *ucBssIdx);
				}
			} else {
				*prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter,
							  P2P_DEV_BSS_INDEX);
				COPY_SSID((*prP2pBssInfo)->aucSSID,
					  (*prP2pBssInfo)->ucSSIDLen,
					  SSID_IE(pucIEBuf)->aucSSID,
					  SSID_IE(pucIEBuf)->ucLength);

			}
		}
		break;
	case ELEM_ID_VENDOR:
		{
			UINT_8 ucOuiType = 0;
			UINT_16 u2SubTypeVersion = 0;

			if (rsnParseCheckForWFAInfoElem
			    (prAdapter, pucIEBuf, &ucOuiType, &u2SubTypeVersion)) {
				if (ucOuiType == VENDOR_OUI_TYPE_WPS) {
					kalP2PUpdateWSC_IE(prAdapter->prGlueInfo, 2,
							   pucIEBuf,
							   IE_SIZE(pucIEBuf));
					*fgIsWSCIE = TRUE;
				}

			} else
			    if (p2pFuncParseCheckForP2PInfoElem
				(prAdapter, pucIEBuf, &ucOuiType)) {
				if (ucOuiType == VENDOR_OUI_TYPE_P2P) {
					
					kalP2PUpdateWSC_IE(prAdapter->prGlueInfo, 1,
							   pucIEBuf,
							   IE_SIZE(pucIEBuf));
					*fgIsP2PIE = TRUE;
				}
#if CFG_SUPPORT_WFD
				else if (ucOuiType == VENDOR_OUI_TYPE_WFD) {
					DBGLOG(P2P, INFO, ("WFD IE is found in probe resp (supp). Len %u\n", IE_SIZE(pucIEBuf)));
					if ((sizeof(prAdapter->prGlueInfo->prP2PInfo->aucWFDIE) >=
						(prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen + IE_SIZE(pucIEBuf)))) {
						*fgIsWFDIE = TRUE;
						kalMemCopy(prAdapter->prGlueInfo->prP2PInfo->aucWFDIE,
							   pucIEBuf,
							   IE_SIZE(pucIEBuf));
						prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen += IE_SIZE(pucIEBuf);
					}
				}	
#endif
			} else {
				DBGLOG(P2P, INFO, ("Other vender IE is found in probe resp (supp). Len %u\n", IE_SIZE(pucIEBuf)));
			}
		}
		break;
	default:
		break;
	}
}


#if 0				
UINT_32
p2pFuncCalculateExtra_IELenForBeacon(IN P_ADAPTER_T prAdapter,
				     IN ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex,
				     IN P_STA_RECORD_T prStaRec)
{

	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpeBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	UINT_32 u4IELen = 0;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (eNetTypeIndex == NETWORK_TYPE_P2P_INDEX));

		if (p2pFuncIsAPMode(prAdapter->rWifiVar.prP2pFsmInfo)) {
			break;
		}

		prP2pSpeBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

		u4IELen = prP2pSpeBssInfo->u2IELenForBCN;

	} while (FALSE);

	return u4IELen;
}				

VOID p2pFuncGenerateExtra_IEForBeacon(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpeBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	PUINT_8 pucIEBuf = (PUINT_8) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prMsduInfo != NULL));

		prP2pSpeBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

		if (p2pFuncIsAPMode(prAdapter->rWifiVar.prP2pFsmInfo)) {

			break;
		}

		pucIEBuf =
		    (PUINT_8) ((UINT_32) prMsduInfo->prPacket +
			       (UINT_32) prMsduInfo->u2FrameLength);

		kalMemCopy(pucIEBuf, prP2pSpeBssInfo->aucBeaconIECache,
			   prP2pSpeBssInfo->u2IELenForBCN);

		prMsduInfo->u2FrameLength += prP2pSpeBssInfo->u2IELenForBCN;

	} while (FALSE);

	return;
}				


#else
UINT_32
p2pFuncCalculateP2p_IELenForBeacon(IN P_ADAPTER_T prAdapter,
				   IN UINT_8 ucBssIdx, IN P_STA_RECORD_T prStaRec)
{
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpeBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	UINT_32 u4IELen = 0;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (ucBssIdx < BSS_INFO_NUM));

		if (!prAdapter->fgIsP2PRegistered) {
			break;
		}


		if (p2pFuncIsAPMode(prAdapter->rWifiVar.prP2PConnSettings)) {
			break;
		}

		prP2pSpeBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

		u4IELen = prP2pSpeBssInfo->u2AttributeLen;

	} while (FALSE);

	return u4IELen;
}				


VOID p2pFuncGenerateP2p_IEForBeacon(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpeBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	PUINT_8 pucIEBuf = (PUINT_8) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prMsduInfo != NULL));

		if (!prAdapter->fgIsP2PRegistered) {
			break;
		}

		prP2pSpeBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

		if (p2pFuncIsAPMode(prAdapter->rWifiVar.prP2PConnSettings)) {

			break;
		}

		pucIEBuf =
		    (PUINT_8) ((ULONG) prMsduInfo->prPacket +
			       (ULONG) prMsduInfo->u2FrameLength);

		kalMemCopy(pucIEBuf, prP2pSpeBssInfo->aucAttributesCache,
			   prP2pSpeBssInfo->u2AttributeLen);

		prMsduInfo->u2FrameLength += prP2pSpeBssInfo->u2AttributeLen;

	} while (FALSE);

	return;
}				





UINT_32
p2pFuncCalculateWSC_IELenForBeacon(IN P_ADAPTER_T prAdapter,
				   IN UINT_8 ucBssIdx, IN P_STA_RECORD_T prStaRec)
{
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;

	prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIdx);

	if (prP2pBssInfo->eNetworkType != NETWORK_TYPE_P2P) {
		return 0;
	}

	return kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 0);
}				


VOID p2pFuncGenerateWSC_IEForBeacon(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{
	PUINT_8 pucBuffer;
	UINT_16 u2IELen = 0;
	P_BSS_INFO_T prP2pBssInfo = (P_BSS_INFO_T) NULL;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prP2pBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMsduInfo->ucBssIndex);

	if (prP2pBssInfo->eNetworkType != NETWORK_TYPE_P2P) {
		return;
	}

	u2IELen = (UINT_16) kalP2PCalWSC_IELen(prAdapter->prGlueInfo, 0);

	pucBuffer = (PUINT_8) ((ULONG) prMsduInfo->prPacket +
			       (ULONG) prMsduInfo->u2FrameLength);

	ASSERT(pucBuffer);

	
	kalP2PGenWSC_IE(prAdapter->prGlueInfo, 0, pucBuffer);

	prMsduInfo->u2FrameLength += u2IELen;

	return;
}				

#endif
UINT_32
p2pFuncCalculateP2p_IELenForAssocRsp(IN P_ADAPTER_T prAdapter,
				     IN UINT_8 ucBssIndex, IN P_STA_RECORD_T prStaRec)
{
	P_BSS_INFO_T prBssInfo = (P_BSS_INFO_T) NULL;

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	if (prBssInfo->eNetworkType != NETWORK_TYPE_P2P) {
		return 0;
	}

	return p2pFuncCalculateP2P_IELen(prAdapter,
					 ucBssIndex,
					 prStaRec,
					 txAssocRspAttributesTable,
					 sizeof(txAssocRspAttributesTable) /
					 sizeof(APPEND_VAR_ATTRI_ENTRY_T));

}				






VOID p2pFuncGenerateP2p_IEForAssocRsp(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{
	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (prMsduInfo != NULL));


		prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

		if (IS_STA_IN_P2P(prStaRec)) {
			DBGLOG(P2P, TRACE, ("Generate NULL P2P IE for Assoc Rsp.\n"));

			p2pFuncGenerateP2P_IE(prAdapter,
					      TRUE,
					      &prMsduInfo->u2FrameLength,
					      prMsduInfo->prPacket,
					      1500,
					      txAssocRspAttributesTable,
					      sizeof(txAssocRspAttributesTable) /
					      sizeof(APPEND_VAR_ATTRI_ENTRY_T));
		} else {

			DBGLOG(P2P, TRACE, ("Legacy device, no P2P IE.\n"));
		}

	} while (FALSE);

	return;

}				




UINT_32
p2pFuncCalculateP2P_IELen(IN P_ADAPTER_T prAdapter,
			  IN UINT_8 ucBssIndex,
			  IN P_STA_RECORD_T prStaRec,
			  IN APPEND_VAR_ATTRI_ENTRY_T arAppendAttriTable[],
			  IN UINT_32 u4AttriTableSize)
{

	UINT_32 u4OverallAttriLen, u4Dummy;
	UINT_16 u2EstimatedFixedAttriLen;
	UINT_32 i;


	
	u4OverallAttriLen = 0;

	for (i = 0; i < u4AttriTableSize; i++) {
		u2EstimatedFixedAttriLen = arAppendAttriTable[i].u2EstimatedFixedAttriLen;

		if (u2EstimatedFixedAttriLen) {
			u4OverallAttriLen += u2EstimatedFixedAttriLen;
		} else {
			ASSERT(arAppendAttriTable[i].pfnCalculateVariableAttriLen);

			u4OverallAttriLen +=
			    arAppendAttriTable[i].pfnCalculateVariableAttriLen(prAdapter, prStaRec);
		}
	}

	u4Dummy = u4OverallAttriLen;
	u4OverallAttriLen += P2P_IE_OUI_HDR;

	for (; (u4Dummy > P2P_MAXIMUM_ATTRIBUTE_LEN);) {
		u4OverallAttriLen += P2P_IE_OUI_HDR;
		u4Dummy -= P2P_MAXIMUM_ATTRIBUTE_LEN;
	}

	return u4OverallAttriLen;
}				


VOID
p2pFuncGenerateP2P_IE(IN P_ADAPTER_T prAdapter,
		      IN BOOLEAN fgIsAssocFrame,
		      IN PUINT_16 pu2Offset,
		      IN PUINT_8 pucBuf,
		      IN UINT_16 u2BufSize,
		      IN APPEND_VAR_ATTRI_ENTRY_T arAppendAttriTable[], IN UINT_32 u4AttriTableSize)
{
	PUINT_8 pucBuffer = (PUINT_8) NULL;
	P_IE_P2P_T prIeP2P = (P_IE_P2P_T) NULL;
	UINT_32 u4OverallAttriLen;
	UINT_32 u4AttriLen;
	UINT_8 aucWfaOui[] = VENDOR_OUI_WFA_SPECIFIC;
	UINT_8 aucTempBuffer[P2P_MAXIMUM_ATTRIBUTE_LEN];
	UINT_32 i;


	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBuf != NULL));

		pucBuffer = (PUINT_8) ((ULONG) pucBuf + (*pu2Offset));

		ASSERT_BREAK(pucBuffer != NULL);

		
		ASSERT_BREAK((u2BufSize - (*pu2Offset)) >= P2P_IE_OUI_HDR);

		prIeP2P = (P_IE_P2P_T) pucBuffer;

		prIeP2P->ucId = ELEM_ID_P2P;

		prIeP2P->aucOui[0] = aucWfaOui[0];
		prIeP2P->aucOui[1] = aucWfaOui[1];
		prIeP2P->aucOui[2] = aucWfaOui[2];
		prIeP2P->ucOuiType = VENDOR_OUI_TYPE_P2P;

		(*pu2Offset) += P2P_IE_OUI_HDR;

		
		u4OverallAttriLen = 0;


		for (i = 0; i < u4AttriTableSize; i++) {

			if (arAppendAttriTable[i].pfnAppendAttri) {
				u4AttriLen =
				    arAppendAttriTable[i].pfnAppendAttri(prAdapter, fgIsAssocFrame,
									 pu2Offset, pucBuf,
									 u2BufSize);

				u4OverallAttriLen += u4AttriLen;

				if (u4OverallAttriLen > P2P_MAXIMUM_ATTRIBUTE_LEN) {
					u4OverallAttriLen -= P2P_MAXIMUM_ATTRIBUTE_LEN;

					prIeP2P->ucLength =
					    (VENDOR_OUI_TYPE_LEN + P2P_MAXIMUM_ATTRIBUTE_LEN);

					pucBuffer =
					    (PUINT_8) ((ULONG) prIeP2P +
						       (VENDOR_OUI_TYPE_LEN +
							P2P_MAXIMUM_ATTRIBUTE_LEN));

					prIeP2P = (P_IE_P2P_T) ((ULONG) prIeP2P +
								(ELEM_HDR_LEN +
								 (VENDOR_OUI_TYPE_LEN +
								  P2P_MAXIMUM_ATTRIBUTE_LEN)));

					kalMemCopy(aucTempBuffer, pucBuffer, u4OverallAttriLen);

					prIeP2P->ucId = ELEM_ID_P2P;

					prIeP2P->aucOui[0] = aucWfaOui[0];
					prIeP2P->aucOui[1] = aucWfaOui[1];
					prIeP2P->aucOui[2] = aucWfaOui[2];
					prIeP2P->ucOuiType = VENDOR_OUI_TYPE_P2P;

					kalMemCopy(prIeP2P->aucP2PAttributes, aucTempBuffer,
						   u4OverallAttriLen);
					(*pu2Offset) += P2P_IE_OUI_HDR;
				}

			}

		}

		prIeP2P->ucLength = (UINT_8) (VENDOR_OUI_TYPE_LEN + u4OverallAttriLen);


	} while (FALSE);

	return;
}				

UINT_32
p2pFuncAppendAttriStatusForAssocRsp(IN P_ADAPTER_T prAdapter,
				    IN BOOLEAN fgIsAssocFrame,
				    IN PUINT_16 pu2Offset, IN PUINT_8 pucBuf, IN UINT_16 u2BufSize)
{
	PUINT_8 pucBuffer;
	P_P2P_ATTRI_STATUS_T prAttriStatus;
	UINT_32 u4AttriLen = 0;

	ASSERT(prAdapter);
	ASSERT(pucBuf);


	if (fgIsAssocFrame) {
		return u4AttriLen;
	}
	

	pucBuffer = (PUINT_8) ((ULONG) pucBuf + (ULONG) (*pu2Offset));

	ASSERT(pucBuffer);
	prAttriStatus = (P_P2P_ATTRI_STATUS_T) pucBuffer;

	ASSERT(u2BufSize >= ((*pu2Offset) + (UINT_16) u4AttriLen));




	prAttriStatus->ucId = P2P_ATTRI_ID_STATUS;
	WLAN_SET_FIELD_16(&prAttriStatus->u2Length, P2P_ATTRI_MAX_LEN_STATUS);

	prAttriStatus->ucStatusCode = P2P_STATUS_SUCCESS;

	u4AttriLen = (P2P_ATTRI_HDR_LEN + P2P_ATTRI_MAX_LEN_STATUS);

	(*pu2Offset) += (UINT_16) u4AttriLen;

	return u4AttriLen;
}				

UINT_32
p2pFuncAppendAttriExtListenTiming(IN P_ADAPTER_T prAdapter,
				  IN BOOLEAN fgIsAssocFrame,
				  IN PUINT_16 pu2Offset, IN PUINT_8 pucBuf, IN UINT_16 u2BufSize)
{
	UINT_32 u4AttriLen = 0;
	P_P2P_ATTRI_EXT_LISTEN_TIMING_T prP2pExtListenTiming =
	    (P_P2P_ATTRI_EXT_LISTEN_TIMING_T) NULL;
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	PUINT_8 pucBuffer = NULL;

	ASSERT(prAdapter);
	ASSERT(pucBuf);

	if (fgIsAssocFrame) {
		return u4AttriLen;
	}
	

	prP2pSpecificBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

	u4AttriLen = (P2P_ATTRI_HDR_LEN + P2P_ATTRI_MAX_LEN_EXT_LISTEN_TIMING);

	ASSERT(u2BufSize >= ((*pu2Offset) + (UINT_16) u4AttriLen));

	pucBuffer = (PUINT_8) ((ULONG) pucBuf + (ULONG) (*pu2Offset));

	ASSERT(pucBuffer);

	prP2pExtListenTiming = (P_P2P_ATTRI_EXT_LISTEN_TIMING_T) pucBuffer;

	prP2pExtListenTiming->ucId = P2P_ATTRI_ID_EXT_LISTEN_TIMING;
	WLAN_SET_FIELD_16(&prP2pExtListenTiming->u2Length, P2P_ATTRI_MAX_LEN_EXT_LISTEN_TIMING);
	WLAN_SET_FIELD_16(&prP2pExtListenTiming->u2AvailInterval,
			  prP2pSpecificBssInfo->u2AvailabilityInterval);
	WLAN_SET_FIELD_16(&prP2pExtListenTiming->u2AvailPeriod,
			  prP2pSpecificBssInfo->u2AvailabilityPeriod);

	(*pu2Offset) += (UINT_16) u4AttriLen;

	return u4AttriLen;
}				


P_IE_HDR_T
p2pFuncGetSpecIE(IN P_ADAPTER_T prAdapter,
		 IN PUINT_8 pucIEBuf,
		 IN UINT_16 u2BufferLen, IN UINT_8 ucElemID, IN PBOOLEAN pfgIsMore)
{
	P_IE_HDR_T prTargetIE = (P_IE_HDR_T) NULL;
	PUINT_8 pucIE = (PUINT_8) NULL;
	UINT_16 u2Offset = 0;


	do {
		ASSERT_BREAK((prAdapter != NULL)
			     && (pucIEBuf != NULL));

		pucIE = pucIEBuf;

		if (pfgIsMore) {
			*pfgIsMore = FALSE;
		}


		IE_FOR_EACH(pucIE, u2BufferLen, u2Offset) {
			if (IE_ID(pucIE) == ucElemID) {
				if ((prTargetIE) && (pfgIsMore)) {

					*pfgIsMore = TRUE;
					break;
				} else {
					prTargetIE = (P_IE_HDR_T) pucIE;

					if (pfgIsMore == NULL) {
						break;
					}

				}

			}
		}

	} while (FALSE);

	return prTargetIE;
}				



P_ATTRIBUTE_HDR_T
p2pFuncGetSpecAttri(IN P_ADAPTER_T prAdapter,
		    IN UINT_8 ucOuiType,
		    IN PUINT_8 pucIEBuf, IN UINT_16 u2BufferLen, IN UINT_8 ucAttriID)
{
	P_IE_P2P_T prP2pIE = (P_IE_P2P_T) NULL;
	P_ATTRIBUTE_HDR_T prTargetAttri = (P_ATTRIBUTE_HDR_T) NULL;
	BOOLEAN fgIsMore = FALSE;
	PUINT_8 pucIE = (PUINT_8) NULL;
	UINT_16 u2BufferLenLeft = 0;


	DBGLOG(P2P, INFO,
	       ("Check AssocReq Oui type %u attri %u for len %u\n", ucOuiType, ucAttriID,
		u2BufferLen));

	do {
		ASSERT_BREAK((prAdapter != NULL)
			     && (pucIEBuf != NULL));

		u2BufferLenLeft = u2BufferLen;
		pucIE = pucIEBuf;

		do {
			fgIsMore = FALSE;
			prP2pIE = (P_IE_P2P_T) p2pFuncGetSpecIE(prAdapter,
								pucIE,
								u2BufferLenLeft,
								ELEM_ID_VENDOR, &fgIsMore);
			if (prP2pIE) {
				ASSERT((ULONG) prP2pIE >= (ULONG) pucIE);
				u2BufferLenLeft = u2BufferLen - (UINT_16) (((ULONG) prP2pIE) - ((ULONG) pucIEBuf));

				DBGLOG(P2P, INFO, ("Find vendor id %u len %u oui %u more %u LeftLen %u\n",
					IE_ID(prP2pIE), IE_LEN(prP2pIE), prP2pIE->ucOuiType,
					fgIsMore, u2BufferLenLeft));

				if (IE_LEN(prP2pIE) > P2P_OUI_TYPE_LEN) {
					p2pFuncGetSpecAttriAction(prP2pIE, ucOuiType, ucAttriID, &prTargetAttri);
				}
				
				pucIE = (PUINT_8) (((ULONG) prP2pIE) + IE_SIZE(prP2pIE));
			}
			
		} while (prP2pIE && fgIsMore && u2BufferLenLeft);

	} while (FALSE);

	return prTargetAttri;
}

static VOID
p2pFuncGetSpecAttriAction(IN P_IE_P2P_T prP2pIE,
		IN UINT_8 ucOuiType, IN UINT_8 ucAttriID,
		OUT P_ATTRIBUTE_HDR_T *prTargetAttri)
{
	PUINT_8 pucAttri = (PUINT_8) NULL;
	UINT_16 u2OffsetAttri = 0;
	UINT_8 aucWfaOui[] = VENDOR_OUI_WFA_SPECIFIC;
	if (prP2pIE->ucOuiType == ucOuiType) {
		switch (ucOuiType) {
		case VENDOR_OUI_TYPE_WPS:
			aucWfaOui[0] = 0x00;
			aucWfaOui[1] = 0x50;
			aucWfaOui[2] = 0xF2;
			break;
		case VENDOR_OUI_TYPE_P2P:
			break;
		case VENDOR_OUI_TYPE_WPA:
		case VENDOR_OUI_TYPE_WMM:
		case VENDOR_OUI_TYPE_WFD:
		default:
			break;
		}


		if ((prP2pIE->aucOui[0] == aucWfaOui[0]) &&
		    (prP2pIE->aucOui[1] == aucWfaOui[1]) &&
		    (prP2pIE->aucOui[2] == aucWfaOui[2])) {

			u2OffsetAttri = 0;
			pucAttri = prP2pIE->aucP2PAttributes;

			if (ucOuiType == VENDOR_OUI_TYPE_WPS) {
				WSC_ATTRI_FOR_EACH(pucAttri, (IE_LEN(prP2pIE) - P2P_IE_OUI_HDR), u2OffsetAttri) {
					if (WSC_ATTRI_ID(pucAttri) == ucAttriID) {
						*prTargetAttri = (P_ATTRIBUTE_HDR_T)pucAttri;
						break;
					}

				}

			} else if (ucOuiType == VENDOR_OUI_TYPE_P2P) {
				P2P_ATTRI_FOR_EACH(pucAttri, (IE_LEN(prP2pIE) - P2P_IE_OUI_HDR), u2OffsetAttri) {
					if (ATTRI_ID(pucAttri) == ucAttriID) {
						*prTargetAttri = (P_ATTRIBUTE_HDR_T)pucAttri;
						break;
					}
				}

			}
#if CFG_SUPPORT_WFD
			else if (ucOuiType == VENDOR_OUI_TYPE_WFD) {
				WFD_ATTRI_FOR_EACH(pucAttri, (IE_LEN(prP2pIE) - P2P_IE_OUI_HDR), u2OffsetAttri) {
					if (ATTRI_ID(pucAttri) ==
					    (UINT_8) ucAttriID) {
						*prTargetAttri = (P_ATTRIBUTE_HDR_T)pucAttri;
						break;
					}
				}
			}
#endif
			else {
				
			}
		}
	}	
}


WLAN_STATUS
p2pFuncGenerateBeaconProbeRsp(IN P_ADAPTER_T prAdapter,
			      IN P_BSS_INFO_T prBssInfo,
			      IN P_MSDU_INFO_T prMsduInfo, IN BOOLEAN fgIsProbeRsp)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;
	P_WLAN_BEACON_FRAME_T prBcnFrame = (P_WLAN_BEACON_FRAME_T) NULL;


	do {

		ASSERT_BREAK((prAdapter != NULL) && (prBssInfo != NULL) && (prMsduInfo != NULL));






		prBcnFrame = (P_WLAN_BEACON_FRAME_T) prMsduInfo->prPacket;

		return nicUpdateBeaconIETemplate(prAdapter,
						 IE_UPD_METHOD_UPDATE_ALL,
						 prBssInfo->ucBssIndex,
						 prBssInfo->u2CapInfo,
						 (PUINT_8) prBcnFrame->aucInfoElem,
						 prMsduInfo->u2FrameLength -
						 OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem));

	} while (FALSE);

	return rWlanStatus;
}				


WLAN_STATUS
p2pFuncComposeBeaconProbeRspTemplate(IN P_ADAPTER_T prAdapter,
				     IN P_BSS_INFO_T prP2pBssInfo,
				     IN PUINT_8 pucBcnBuffer,
				     IN UINT_32 u4BcnBufLen,
				     IN BOOLEAN fgIsProbeRsp,
				     IN P_P2P_PROBE_RSP_UPDATE_INFO_T prP2pProbeRspInfo,
				     IN BOOLEAN fgSynToFW)
{
	WLAN_STATUS rWlanStatus = WLAN_STATUS_SUCCESS;
	P_MSDU_INFO_T prMsduInfo = (P_MSDU_INFO_T) NULL;
	P_WLAN_MAC_HEADER_T prWlanBcnFrame = (P_WLAN_MAC_HEADER_T) NULL;

	PUINT_8 pucBuffer = (PUINT_8) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBcnBuffer != NULL)
			     && (prP2pBssInfo != NULL));

		prWlanBcnFrame = (P_WLAN_MAC_HEADER_T) pucBcnBuffer;

		if ((prWlanBcnFrame->u2FrameCtrl != MAC_FRAME_BEACON) && (!fgIsProbeRsp)) {
			rWlanStatus = WLAN_STATUS_INVALID_DATA;
			break;
		}

		else if (prWlanBcnFrame->u2FrameCtrl != MAC_FRAME_PROBE_RSP) {
			rWlanStatus = WLAN_STATUS_INVALID_DATA;
			break;
		}



		if (fgIsProbeRsp) {
			ASSERT_BREAK(prP2pProbeRspInfo != NULL);

			if (!prP2pProbeRspInfo->prProbeRspMsduTemplate) {
				cnmMgtPktFree(prAdapter, prP2pProbeRspInfo->prProbeRspMsduTemplate);
			}

			prP2pProbeRspInfo->prProbeRspMsduTemplate =
			    cnmMgtPktAlloc(prAdapter, u4BcnBufLen);

			prMsduInfo = prP2pProbeRspInfo->prProbeRspMsduTemplate;

			prMsduInfo->eSrc = TX_PACKET_MGMT;
			prMsduInfo->ucStaRecIndex = 0xFF;
			prMsduInfo->ucBssIndex = prP2pBssInfo->ucBssIndex;

		} else {
			prMsduInfo = prP2pBssInfo->prBeacon;

			if (prMsduInfo == NULL) {
				rWlanStatus = WLAN_STATUS_FAILURE;
				break;
			}

			if (u4BcnBufLen >
			    (OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem[0]) + MAX_IE_LENGTH)) {
				
				ASSERT(FALSE);
				break;
			}

		}


		pucBuffer = (PUINT_8) ((ULONG) (prMsduInfo->prPacket) + MAC_TX_RESERVED_FIELD);

		kalMemCopy(pucBuffer, pucBcnBuffer, u4BcnBufLen);

		prMsduInfo->fgIs802_11 = TRUE;
		prMsduInfo->u2FrameLength = (UINT_16) u4BcnBufLen;

		if (fgSynToFW) {
			rWlanStatus =
			    p2pFuncGenerateBeaconProbeRsp(prAdapter, prP2pBssInfo, prMsduInfo,
							  fgIsProbeRsp);
		}

	} while (FALSE);

	return rWlanStatus;

}				



UINT_32
wfdFuncCalculateWfdIELenForAssocRsp(IN P_ADAPTER_T prAdapter,
				    IN UINT_8 ucBssIndex, IN P_STA_RECORD_T prStaRec)
{

#if CFG_SUPPORT_WFD_COMPOSE_IE
	UINT_16 u2EstimatedExtraIELen = 0;
	P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;
	P_BSS_INFO_T prBssInfo = (P_BSS_INFO_T) NULL;

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	if (prBssInfo->eNetworkType != NETWORK_TYPE_P2P) {
		return 0;
	}

	prWfdCfgSettings = &(prAdapter->rWifiVar.rWfdConfigureSettings);

	if (IS_STA_P2P_TYPE(prStaRec) && (prWfdCfgSettings->ucWfdEnable > 0)) {

		u2EstimatedExtraIELen = prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen;
		ASSERT(u2EstimatedExtraIELen < 128);
	}
	return u2EstimatedExtraIELen;

#else
	return 0;
#endif
}				



VOID wfdFuncGenerateWfdIEForAssocRsp(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{

#if CFG_SUPPORT_WFD_COMPOSE_IE
	P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;
	P_STA_RECORD_T prStaRec;
	UINT_16 u2EstimatedExtraIELen;

	prWfdCfgSettings = &(prAdapter->rWifiVar.rWfdConfigureSettings);

	do {
		ASSERT_BREAK((prMsduInfo != NULL) && (prAdapter != NULL));

		prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

		if (prStaRec) {
			if (IS_STA_P2P_TYPE(prStaRec)) {

				if (prWfdCfgSettings->ucWfdEnable > 0) {
					u2EstimatedExtraIELen =
					    prAdapter->prGlueInfo->prP2PInfo->u2WFDIELen;
					if (u2EstimatedExtraIELen > 0) {
						ASSERT(u2EstimatedExtraIELen < 128);
						ASSERT(sizeof
						       (prAdapter->prGlueInfo->prP2PInfo->
							aucWFDIE) >=
						       prAdapter->prGlueInfo->prP2PInfo->
						       u2WFDIELen);
						kalMemCopy((prMsduInfo->prPacket +
							    prMsduInfo->u2FrameLength),
							   prAdapter->prGlueInfo->prP2PInfo->
							   aucWFDIE, u2EstimatedExtraIELen);
						prMsduInfo->u2FrameLength += u2EstimatedExtraIELen;
					}
				}
			}	
		} else {
		}
	} while (FALSE);

	return;
#else

	return;
#endif
}				
VOID
p2pFuncComposeNoaAttribute (
    IN P_ADAPTER_T      prAdapter,
    IN UINT_8           ucBssIndex,
    OUT PUINT_8         aucNoaAttrArray,
    OUT PUINT_32        pu4Len
    )
{
    P_BSS_INFO_T prBssInfo = NULL;
    P_P2P_ATTRI_NOA_T prNoaAttr = NULL;
    P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = NULL;
    P_NOA_DESCRIPTOR_T prNoaDesc = NULL;
    UINT_32 u4NumOfNoaDesc = 0;
    UINT_32 i = 0;

    prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);
    prP2pSpecificBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;

    prNoaAttr = (P_P2P_ATTRI_NOA_T)aucNoaAttrArray;

    prNoaAttr->ucId = P2P_ATTRI_ID_NOTICE_OF_ABSENCE;
    prNoaAttr->ucIndex = prP2pSpecificBssInfo->ucNoAIndex;

    if (prP2pSpecificBssInfo->fgEnableOppPS) {
        prNoaAttr->ucCTWOppPSParam = P2P_CTW_OPPPS_PARAM_OPPPS_FIELD |
                        (prP2pSpecificBssInfo->u2CTWindow & P2P_CTW_OPPPS_PARAM_CTWINDOW_MASK);
    } else {
        prNoaAttr->ucCTWOppPSParam = 0;
    }

    for (i = 0; i < prP2pSpecificBssInfo->ucNoATimingCount; i++) {
        if(prP2pSpecificBssInfo->arNoATiming[i].fgIsInUse) {
            prNoaDesc = (P_NOA_DESCRIPTOR_T)&prNoaAttr->aucNoADesc[u4NumOfNoaDesc];
            
            prNoaDesc->ucCountType = prP2pSpecificBssInfo->arNoATiming[i].ucCount;
            prNoaDesc->u4Duration = prP2pSpecificBssInfo->arNoATiming[i].u4Duration;
            prNoaDesc->u4Interval = prP2pSpecificBssInfo->arNoATiming[i].u4Interval;
            prNoaDesc->u4StartTime = prP2pSpecificBssInfo->arNoATiming[i].u4StartTime;
            
            u4NumOfNoaDesc++;
        }
    }

    
    prNoaAttr->u2Length = 2 + u4NumOfNoaDesc * sizeof(NOA_DESCRIPTOR_T);

    
    *pu4Len = P2P_ATTRI_HDR_LEN + prNoaAttr->u2Length;
}

UINT_32
p2pFuncCalculateP2P_IE_NoA (
    IN P_ADAPTER_T prAdapter,
    IN UINT_8 ucBssIdx,
    IN P_STA_RECORD_T prStaRec
    )
{
    P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = NULL;
    UINT_8 ucIdx;
    UINT_32 u4NumOfNoaDesc = 0;

    if (p2pFuncIsAPMode(prAdapter->rWifiVar.prP2PConnSettings)) {
        return 0;
    }

    prP2pSpecificBssInfo = prAdapter->rWifiVar.prP2pSpecificBssInfo;    

    for (ucIdx = 0; ucIdx < prP2pSpecificBssInfo->ucNoATimingCount; ucIdx++) {
        if(prP2pSpecificBssInfo->arNoATiming[ucIdx].fgIsInUse) {           
            u4NumOfNoaDesc++;
        }
    }
    
    
    
    return (P2P_ATTRI_HDR_LEN + 2 + (u4NumOfNoaDesc * sizeof(NOA_DESCRIPTOR_T)));
}

VOID
p2pFuncGenerateP2P_IE_NoA(
    IN P_ADAPTER_T      prAdapter,
    IN P_MSDU_INFO_T    prMsduInfo
    )
{
    P_IE_P2P_T prIeP2P;
    UINT_8 aucWfaOui[] = VENDOR_OUI_WFA_SPECIFIC;
    UINT_32 u4AttributeLen;

    if(p2pFuncIsAPMode(prAdapter->rWifiVar.prP2PConnSettings)) {
        return;
    }

    prIeP2P = (P_IE_P2P_T)((ULONG)prMsduInfo->prPacket + (UINT_32)prMsduInfo->u2FrameLength);

    prIeP2P->ucId = ELEM_ID_P2P;
    prIeP2P->aucOui[0] = aucWfaOui[0];
    prIeP2P->aucOui[1] = aucWfaOui[1];
    prIeP2P->aucOui[2] = aucWfaOui[2];
    prIeP2P->ucOuiType = VENDOR_OUI_TYPE_P2P; 

    
    p2pFuncComposeNoaAttribute(prAdapter, prMsduInfo->ucBssIndex, prIeP2P->aucP2PAttributes, &u4AttributeLen);

    prIeP2P->ucLength = VENDOR_OUI_TYPE_LEN + u4AttributeLen;

    prMsduInfo->u2FrameLength += (ELEM_HDR_LEN + prIeP2P->ucLength);
    
}
