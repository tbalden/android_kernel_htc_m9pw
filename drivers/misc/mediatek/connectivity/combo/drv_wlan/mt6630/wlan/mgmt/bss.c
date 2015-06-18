





#include "precomp.h"



const PUINT_8 apucNetworkType[NETWORK_TYPE_NUM] = {
	(PUINT_8) "AIS",
	(PUINT_8) "P2P",
	(PUINT_8) "BOW",
	(PUINT_8) "MBSS"
};

const PUINT_8 apucNetworkOpMode[] = {
	(PUINT_8) "INFRASTRUCTURE",
	(PUINT_8) "IBSS",
	(PUINT_8) "ACCESS_POINT",
	(PUINT_8) "P2P_DEVICE",
	(PUINT_8) "BOW"
};

#if (CFG_SUPPORT_ADHOC) || (CFG_SUPPORT_AAA)
APPEND_VAR_IE_ENTRY_T txBcnIETable[] = {
	{(ELEM_HDR_LEN + (RATE_NUM_SW - ELEM_MAX_LEN_SUP_RATES)), NULL, bssGenerateExtSuppRate_IE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_ERP), NULL, rlmRspGenerateErpIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP), NULL, rlmRspGenerateHtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_OP), NULL, rlmRspGenerateHtOpIE}	
#if CFG_ENABLE_WIFI_DIRECT
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_OBSS_SCAN), NULL, rlmRspGenerateObssScanIE}	
#endif
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_EXT_CAP), NULL, rlmRspGenerateExtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WPA), NULL, rsnGenerateWpaNoneIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WMM_PARAM), NULL, mqmGenerateWmmParamIE}	
#if CFG_ENABLE_WIFI_DIRECT
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WPA), NULL, rsnGenerateWPAIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_RSN), NULL, rsnGenerateRSNIE}	
#if 0				
	, {0, p2pFuncCalculateExtra_IELenForBeacon, p2pFuncGenerateExtra_IEForBeacon}	
#else
	, {0, p2pFuncCalculateP2p_IELenForBeacon, p2pFuncGenerateP2p_IEForBeacon}	
	, {0, p2pFuncCalculateWSC_IELenForBeacon, p2pFuncGenerateWSC_IEForBeacon}	
#endif
   ,{ 0,                                                    p2pFuncCalculateP2P_IE_NoA,     p2pFuncGenerateP2P_IE_NoA   }   
#endif				
#if CFG_SUPPORT_802_11AC
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP), NULL, rlmRspGenerateVhtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_OP), NULL, rlmRspGenerateVhtOpIE}	
#endif
#if CFG_SUPPORT_MTK_SYNERGY
	, {(ELEM_HDR_LEN + ELEM_MIN_LEN_MTK_OUI), NULL, rlmGenerateMTKOuiIE}	
#endif

};


APPEND_VAR_IE_ENTRY_T txProbRspIETable[] = {
	{(ELEM_HDR_LEN + (RATE_NUM_SW - ELEM_MAX_LEN_SUP_RATES)), NULL, bssGenerateExtSuppRate_IE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_ERP), NULL, rlmRspGenerateErpIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP), NULL, rlmRspGenerateHtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_HT_OP), NULL, rlmRspGenerateHtOpIE}	
#if CFG_ENABLE_WIFI_DIRECT
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_RSN), NULL, rsnGenerateRSNIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_OBSS_SCAN), NULL, rlmRspGenerateObssScanIE}	
#endif
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_EXT_CAP), NULL, rlmRspGenerateExtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WPA), NULL, rsnGenerateWpaNoneIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_WMM_PARAM), NULL, mqmGenerateWmmParamIE}	
#if CFG_SUPPORT_802_11AC
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP), NULL, rlmRspGenerateVhtCapIE}	
	, {(ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_OP), NULL, rlmRspGenerateVhtOpIE}	
#endif
#if CFG_SUPPORT_MTK_SYNERGY
	, {(ELEM_HDR_LEN + ELEM_MIN_LEN_MTK_OUI), NULL, rlmGenerateMTKOuiIE}	
#endif

};

#endif				







VOID
bssDetermineStaRecPhyTypeSet(IN P_ADAPTER_T prAdapter,
			     IN P_BSS_DESC_T prBssDesc, OUT P_STA_RECORD_T prStaRec)
{
	P_WIFI_VAR_T prWifiVar = &prAdapter->rWifiVar;
	UINT_8 ucHtOption = FEATURE_ENABLED;
	UINT_8 ucVhtOption = FEATURE_ENABLED;

	prStaRec->ucPhyTypeSet = prBssDesc->ucPhyTypeSet;

	
	if (prStaRec->eStaType == STA_TYPE_LEGACY_AP) {
		if (!((prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION3_ENABLED) ||
		      (prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION3_KEY_ABSENT)
		      || (prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION_DISABLED)
		      || (prAdapter->prGlueInfo->u2WSCAssocInfoIELen)
#if CFG_SUPPORT_WAPI
		      || (prAdapter->prGlueInfo->u2WapiAssocInfoIESz)
#endif
		    )) {
			DBGLOG(BSS, INFO,
			       ("Ignore the HT Bit for TKIP as pairwise cipher configed!\n"));
			prStaRec->ucPhyTypeSet &= ~(PHY_TYPE_BIT_HT | PHY_TYPE_BIT_VHT);
		}

		ucHtOption = prWifiVar->ucStaHt;
		ucVhtOption = prWifiVar->ucStaVht;
	}
	
	else if (prStaRec->eStaType == STA_TYPE_P2P_GO) {
		ucHtOption = prWifiVar->ucP2pGcHt;
		ucVhtOption = prWifiVar->ucP2pGcVht;
	}

	
	if (IS_FEATURE_DISABLED(ucHtOption))
		prStaRec->ucPhyTypeSet &= ~PHY_TYPE_BIT_HT;
	else if (IS_FEATURE_FORCE_ENABLED(ucHtOption))
		prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_HT;


	if (IS_FEATURE_DISABLED(ucVhtOption))
		prStaRec->ucPhyTypeSet &= ~PHY_TYPE_BIT_VHT;
	else if (IS_FEATURE_FORCE_ENABLED(ucVhtOption))
		prStaRec->ucPhyTypeSet |= PHY_TYPE_BIT_VHT;


	prStaRec->ucDesiredPhyTypeSet =
	    prStaRec->ucPhyTypeSet & prAdapter->rWifiVar.ucAvailablePhyTypeSet;

}


VOID
bssDetermineApBssInfoPhyTypeSet(IN P_ADAPTER_T prAdapter,
				IN BOOLEAN fgIsPureAp, OUT P_BSS_INFO_T prBssInfo)
{
	P_WIFI_VAR_T prWifiVar = &prAdapter->rWifiVar;
	UINT_8 ucHtOption = FEATURE_ENABLED;
	UINT_8 ucVhtOption = FEATURE_ENABLED;

	
	if (fgIsPureAp) {
		ucHtOption = prWifiVar->ucApHt;
		ucVhtOption = prWifiVar->ucApVht;
	}
	
	else {
		ucHtOption = prWifiVar->ucP2pGoHt;
		ucVhtOption = prWifiVar->ucP2pGoVht;
	}

	
	if (IS_FEATURE_DISABLED(ucHtOption))
		prBssInfo->ucPhyTypeSet &= ~PHY_TYPE_BIT_HT;
	else if (IS_FEATURE_ENABLED(ucHtOption))
		prBssInfo->ucPhyTypeSet |= PHY_TYPE_BIT_HT;


	if (IS_FEATURE_DISABLED(ucVhtOption)) {
		prBssInfo->ucPhyTypeSet &= ~PHY_TYPE_BIT_VHT;
	} else if (IS_FEATURE_FORCE_ENABLED(ucVhtOption) ||
		   (IS_FEATURE_ENABLED(ucVhtOption) && (prBssInfo->eBand == BAND_5G))) {

		
		prBssInfo->ucPhyTypeSet |= PHY_TYPE_BIT_VHT;
	}

	prBssInfo->ucPhyTypeSet &= prAdapter->rWifiVar.ucAvailablePhyTypeSet;

}


P_STA_RECORD_T
bssCreateStaRecFromBssDesc(IN P_ADAPTER_T prAdapter,
			   IN ENUM_STA_TYPE_T eStaType,
			   IN UINT_8 ucBssIndex, IN P_BSS_DESC_T prBssDesc)
{
	P_STA_RECORD_T prStaRec;
	UINT_8 ucNonHTPhyTypeSet;
	P_CONNECTION_SETTINGS_T prConnSettings;

	ASSERT(prBssDesc);

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	
	prStaRec = cnmGetStaRecByAddress(prAdapter, ucBssIndex, prBssDesc->aucSrcAddr);
	if (!prStaRec) {
		prStaRec = cnmStaRecAlloc(prAdapter, eStaType, ucBssIndex, prBssDesc->aucSrcAddr);

		if (!prStaRec) {
			DBGLOG(BSS, WARN,
			       ("STA_REC entry is full, cannot acquire new entry for [" MACSTR
				"]!!\n", MAC2STR(prBssDesc->aucSrcAddr)));
			ASSERT(FALSE);
			return NULL;
		}

		prStaRec->ucStaState = STA_STATE_1;
		prStaRec->ucJoinFailureCount = 0;
		
	}
	
	prStaRec->u2CapInfo = prBssDesc->u2CapInfo;

	prStaRec->u2OperationalRateSet = prBssDesc->u2OperationalRateSet;
	prStaRec->u2BSSBasicRateSet = prBssDesc->u2BSSBasicRateSet;

#if 1
	bssDetermineStaRecPhyTypeSet(prAdapter, prBssDesc, prStaRec);
#else
	prStaRec->ucPhyTypeSet = prBssDesc->ucPhyTypeSet;

	if (IS_STA_IN_AIS(prStaRec)) {
		if (!((prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION3_ENABLED) ||
		      (prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION3_KEY_ABSENT)
		      || (prAdapter->rWifiVar.rConnSettings.eEncStatus == ENUM_ENCRYPTION_DISABLED)
		      || (prAdapter->prGlueInfo->u2WSCAssocInfoIELen)
#if CFG_SUPPORT_WAPI
		      || (prAdapter->prGlueInfo->u2WapiAssocInfoIESz)
#endif
		    )) {
			DBGLOG(BSS, INFO,
			       ("Ignore the HT Bit for TKIP as pairwise cipher configed!\n"));
			prStaRec->ucPhyTypeSet &= ~PHY_TYPE_BIT_HT;
		}
	}

	prStaRec->ucDesiredPhyTypeSet =
	    prStaRec->ucPhyTypeSet & prAdapter->rWifiVar.ucAvailablePhyTypeSet;
#endif

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

	
	prStaRec->u2DesiredNonHTRateSet =
	    (prStaRec->u2OperationalRateSet & prConnSettings->u2DesiredNonHTRateSet);

	
	if (IS_AP_STA(prStaRec)) {
		if (prBssDesc->ucDTIMPeriod)
			prStaRec->ucDTIMPeriod = prBssDesc->ucDTIMPeriod;
		else
			prStaRec->ucDTIMPeriod = 0;	

	}
	
	prStaRec->fgDiagnoseConnection = FALSE;

	
	
	mqmProcessScanResult(prAdapter, prBssDesc, prStaRec);

	
	
	nicTxUpdateStaRecDefaultRate(prStaRec);

	return prStaRec;

}				


VOID bssComposeNullFrame(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBuffer, IN P_STA_RECORD_T prStaRec)
{
	P_WLAN_MAC_HEADER_T prNullFrame;
	P_BSS_INFO_T prBssInfo;
	UINT_16 u2FrameCtrl;
	UINT_8 ucBssIndex;

	ASSERT(prStaRec);
	ucBssIndex = prStaRec->ucBssIndex;

	ASSERT(ucBssIndex <= MAX_BSS_INDEX);

	ASSERT(pucBuffer);
	
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);	
	ASSERT(prBssInfo);

	prNullFrame = (P_WLAN_MAC_HEADER_T) pucBuffer;

	
	u2FrameCtrl = MAC_FRAME_NULL;

	if (IS_AP_STA(prStaRec)) {
		u2FrameCtrl |= MASK_FC_TO_DS;

		if (prStaRec->fgSetPwrMgtBit)
			u2FrameCtrl |= MASK_FC_PWR_MGT;

	} else if (IS_CLIENT_STA(prStaRec)) {
		u2FrameCtrl |= MASK_FC_FROM_DS;
	} else if (IS_DLS_STA(prStaRec)) {
		
	} else {
		
		ASSERT(0);
		return;
	}

	
	
	
	prNullFrame->u2FrameCtrl = u2FrameCtrl;	

	
	COPY_MAC_ADDR(prNullFrame->aucAddr1, prStaRec->aucMacAddr);

	
	COPY_MAC_ADDR(prNullFrame->aucAddr2, prBssInfo->aucOwnMacAddr);

	
	COPY_MAC_ADDR(prNullFrame->aucAddr3, prBssInfo->aucBSSID);

	
	prNullFrame->u2SeqCtrl = 0;

	return;

}				


VOID
bssComposeQoSNullFrame(IN P_ADAPTER_T prAdapter,
		       IN PUINT_8 pucBuffer,
		       IN P_STA_RECORD_T prStaRec, IN UINT_8 ucUP, IN BOOLEAN fgSetEOSP)
{
	P_WLAN_MAC_HEADER_QOS_T prQoSNullFrame;
	P_BSS_INFO_T prBssInfo;
	UINT_16 u2FrameCtrl;
	UINT_16 u2QosControl;
	UINT_8 ucBssIndex;

	ASSERT(prStaRec);
	ucBssIndex = prStaRec->ucBssIndex;

	ASSERT(ucBssIndex <= MAX_BSS_INDEX);

	ASSERT(pucBuffer);
	
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);	
	ASSERT(prBssInfo);

	prQoSNullFrame = (P_WLAN_MAC_HEADER_QOS_T) pucBuffer;

	
	u2FrameCtrl = MAC_FRAME_QOS_NULL;

	if (IS_AP_STA(prStaRec)) {
		u2FrameCtrl |= MASK_FC_TO_DS;

		if (prStaRec->fgSetPwrMgtBit)
			u2FrameCtrl |= MASK_FC_PWR_MGT;

	} else if (IS_CLIENT_STA(prStaRec)) {
		u2FrameCtrl |= MASK_FC_FROM_DS;
	} else if (IS_DLS_STA(prStaRec)) {
		
	} else {
		
		ASSERT(0);
		return;
	}

	
	
	
	prQoSNullFrame->u2FrameCtrl = u2FrameCtrl;	

	
	COPY_MAC_ADDR(prQoSNullFrame->aucAddr1, prStaRec->aucMacAddr);

	
	COPY_MAC_ADDR(prQoSNullFrame->aucAddr2, prBssInfo->aucOwnMacAddr);

	
	COPY_MAC_ADDR(prQoSNullFrame->aucAddr3, prBssInfo->aucBSSID);

	
	prQoSNullFrame->u2SeqCtrl = 0;

	u2QosControl = (UINT_16) (ucUP & WMM_QC_UP_MASK);

	if (fgSetEOSP)
		u2QosControl |= WMM_QC_EOSP;

	
	prQoSNullFrame->u2QosCtrl = u2QosControl;	

	return;

}				


WLAN_STATUS
bssSendNullFrame(IN P_ADAPTER_T prAdapter,
		 IN P_STA_RECORD_T prStaRec, IN PFN_TX_DONE_HANDLER pfTxDoneHandler)
{
	P_MSDU_INFO_T prMsduInfo;
	UINT_16 u2EstimatedFrameLen;


	
	
	u2EstimatedFrameLen = MAC_TX_RESERVED_FIELD + WLAN_MAC_HEADER_LEN;

	
	prMsduInfo = cnmMgtPktAlloc(prAdapter, u2EstimatedFrameLen);
	if (prMsduInfo == NULL) {
		DBGLOG(BSS, WARN, ("No PKT_INFO_T for sending Null Frame.\n"));
		return WLAN_STATUS_RESOURCES;
	}
	
	bssComposeNullFrame(prAdapter,
			    (PUINT_8) ((ULONG) prMsduInfo->prPacket + MAC_TX_RESERVED_FIELD),
			    prStaRec);
#if 0
	
	TXM_SET_DATA_PACKET(
				    prStaRec,
				    prMsduInfo,
				   
				   (prMsduInfo->pucBuffer + MAC_TX_RESERVED_FIELD),
				    WLAN_MAC_HEADER_LEN,
				   
				   (prMsduInfo->pucBuffer + MAC_TX_RESERVED_FIELD +
				    WLAN_MAC_HEADER_LEN),
				    0,
				    (UINT_8) prStaRec->ucNetTypeIndex,
				    0  ,
				    TRUE,
				    0  ,
				    0  ,
				    FALSE,
				    pfTxDoneHandler,
				    PS_FORWARDING_TYPE_NON_PS,
				    0  ,
				    TRUE,
				   
				   g_aprBssInfo[prStaRec->ucNetTypeIndex]->ucHwDefaultFixedRateCode,
				   
				   BSS_DEFAULT_CONN_TEST_NULL_FRAME_RETRY_LIMIT,
				    0  ,
				    0  ,
				    FALSE
	    );

	
	NIC_HIF_TX_SET_NEXT_MSDU_INFO(prMsduInfo, NULL);

	

	
	
	txmSendFwDataPackets(prMsduInfo);
#endif

	TX_SET_MMPDU(prAdapter,
		     prMsduInfo,
		     prStaRec->ucBssIndex,
		     prStaRec->ucIndex,
		     WLAN_MAC_HEADER_LEN,
		     WLAN_MAC_HEADER_LEN, pfTxDoneHandler, MSDU_RATE_MODE_AUTO);

	
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	return WLAN_STATUS_SUCCESS;

}				


WLAN_STATUS
bssSendQoSNullFrame(IN P_ADAPTER_T prAdapter,
		    IN P_STA_RECORD_T prStaRec,
		    IN UINT_8 ucUP, IN PFN_TX_DONE_HANDLER pfTxDoneHandler)
{
	P_MSDU_INFO_T prMsduInfo;
	UINT_16 u2EstimatedFrameLen;


	
	
	u2EstimatedFrameLen = MAC_TX_RESERVED_FIELD + WLAN_MAC_HEADER_QOS_LEN;

	
	prMsduInfo = cnmMgtPktAlloc(prAdapter, u2EstimatedFrameLen);
	if (prMsduInfo == NULL) {
		DBGLOG(BSS, WARN, ("No PKT_INFO_T for sending Null Frame.\n"));
		return WLAN_STATUS_RESOURCES;
	}
	
	bssComposeQoSNullFrame(prAdapter,
			       (PUINT_8) ((ULONG) (prMsduInfo->prPacket) + MAC_TX_RESERVED_FIELD),
			       prStaRec, ucUP, FALSE);
#if 0
	
	TXM_SET_DATA_PACKET(
				    prStaRec,
				    prMsduInfo,
				   
				   (prMsduInfo->pucBuffer + MAC_TX_RESERVED_FIELD),
				    WLAN_MAC_HEADER_QOS_LEN,
				   
				   (prMsduInfo->pucBuffer + MAC_TX_RESERVED_FIELD +
				    WLAN_MAC_HEADER_QOS_LEN),
				    0,
				    (UINT_8) prStaRec->ucNetTypeIndex,
				    0  ,
				    TRUE,
				    0  ,
				    0  ,
				    FALSE,
				    pfTxDoneHandler,
				    PS_FORWARDING_TYPE_NON_PS,
				    0  ,
				    TRUE,
				   
				   g_aprBssInfo[prStaRec->ucNetTypeIndex]->ucHwDefaultFixedRateCode,
				    TXM_DEFAULT_DATA_FRAME_RETRY_LIMIT,
				    0  ,
				    0  ,
				    FALSE
	    );

	
	NIC_HIF_TX_SET_NEXT_MSDU_INFO(prMsduInfo, NULL);

	

	
	
	txmSendFwDataPackets(prMsduInfo);
#endif

	TX_SET_MMPDU(prAdapter,
		     prMsduInfo,
		     prStaRec->ucBssIndex,
		     prStaRec->ucIndex,
		     WLAN_MAC_HEADER_QOS_LEN,
		     WLAN_MAC_HEADER_QOS_LEN, pfTxDoneHandler, MSDU_RATE_MODE_AUTO);

	
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	return WLAN_STATUS_SUCCESS;

}				


#if (CFG_SUPPORT_ADHOC) || (CFG_SUPPORT_AAA)
VOID bssGenerateExtSuppRate_IE(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	PUINT_8 pucBuffer;
	UINT_8 ucExtSupRatesLen;


	ASSERT(prMsduInfo);

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMsduInfo->ucBssIndex);
	ASSERT(prBssInfo);

	pucBuffer = (PUINT_8) ((ULONG) prMsduInfo->prPacket +
			       (ULONG) prMsduInfo->u2FrameLength);
	ASSERT(pucBuffer);

	if (prBssInfo->ucAllSupportedRatesLen > ELEM_MAX_LEN_SUP_RATES)

		ucExtSupRatesLen = prBssInfo->ucAllSupportedRatesLen - ELEM_MAX_LEN_SUP_RATES;
	else
		ucExtSupRatesLen = 0;


	
	if (ucExtSupRatesLen) {

		EXT_SUP_RATES_IE(pucBuffer)->ucId = ELEM_ID_EXTENDED_SUP_RATES;
		EXT_SUP_RATES_IE(pucBuffer)->ucLength = ucExtSupRatesLen;

		kalMemCopy(EXT_SUP_RATES_IE(pucBuffer)->aucExtSupportedRates,
			   &prBssInfo->aucAllSupportedRates[ELEM_MAX_LEN_SUP_RATES],
			   ucExtSupRatesLen);

		prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
	}

	return;
}				


VOID
bssBuildBeaconProbeRespFrameCommonIEs(IN P_MSDU_INFO_T prMsduInfo,
				      IN P_BSS_INFO_T prBssInfo, IN PUINT_8 pucDestAddr)
{
	PUINT_8 pucBuffer;
	UINT_8 ucSupRatesLen;


	ASSERT(prMsduInfo);
	ASSERT(prBssInfo);

	pucBuffer = (PUINT_8) ((ULONG) prMsduInfo->prPacket +
			       (ULONG) prMsduInfo->u2FrameLength);
	ASSERT(pucBuffer);

	
	
	SSID_IE(pucBuffer)->ucId = ELEM_ID_SSID;
#if 0
    SSID_IE(pucBuffer)->ucLength = prBssInfo->ucSSIDLen;
    if (prBssInfo->ucSSIDLen) {
        kalMemCopy(SSID_IE(pucBuffer)->aucSSID, prBssInfo->aucSSID, prBssInfo->ucSSIDLen);
    }
#else
    if (prBssInfo->eHiddenSsidType == ENUM_HIDDEN_SSID_LEN) {
        if ((!pucDestAddr) && 
            (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT)) {
            SSID_IE(pucBuffer)->ucLength = 0;
        } else { 
            SSID_IE(pucBuffer)->ucLength = prBssInfo->ucSSIDLen;
            if (prBssInfo->ucSSIDLen) {
                kalMemCopy(SSID_IE(pucBuffer)->aucSSID, prBssInfo->aucSSID, prBssInfo->ucSSIDLen);
            }
        }
    }
    else {
        SSID_IE(pucBuffer)->ucLength = prBssInfo->ucSSIDLen;
        if (prBssInfo->ucSSIDLen) {
            kalMemCopy(SSID_IE(pucBuffer)->aucSSID, prBssInfo->aucSSID, prBssInfo->ucSSIDLen);
        }
    }  
#endif


	prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
	pucBuffer += IE_SIZE(pucBuffer);


	
	if (prBssInfo->ucAllSupportedRatesLen > ELEM_MAX_LEN_SUP_RATES)

		ucSupRatesLen = ELEM_MAX_LEN_SUP_RATES;
	else
		ucSupRatesLen = prBssInfo->ucAllSupportedRatesLen;


	if (ucSupRatesLen) {
		SUP_RATES_IE(pucBuffer)->ucId = ELEM_ID_SUP_RATES;
		SUP_RATES_IE(pucBuffer)->ucLength = ucSupRatesLen;
		kalMemCopy(SUP_RATES_IE(pucBuffer)->aucSupportedRates,
			   prBssInfo->aucAllSupportedRates, ucSupRatesLen);

		prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
		pucBuffer += IE_SIZE(pucBuffer);
	}

	
	if (prBssInfo->eBand == BAND_2G4) {
		DS_PARAM_IE(pucBuffer)->ucId = ELEM_ID_DS_PARAM_SET;
		DS_PARAM_IE(pucBuffer)->ucLength = ELEM_MAX_LEN_DS_PARAMETER_SET;
		DS_PARAM_IE(pucBuffer)->ucCurrChnl = prBssInfo->ucPrimaryChannel;

		prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
		pucBuffer += IE_SIZE(pucBuffer);
	}

	
	if (prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {
		IBSS_PARAM_IE(pucBuffer)->ucId = ELEM_ID_IBSS_PARAM_SET;
		IBSS_PARAM_IE(pucBuffer)->ucLength = ELEM_MAX_LEN_IBSS_PARAMETER_SET;
		WLAN_SET_FIELD_16(&(IBSS_PARAM_IE(pucBuffer)->u2ATIMWindow),
				  prBssInfo->u2ATIMWindow);

		prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
		pucBuffer += IE_SIZE(pucBuffer);
	}

	
	if ((!pucDestAddr) &&	
	    (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT)) {

#if CFG_ENABLE_WIFI_DIRECT
		
		if (prBssInfo->eNetworkType == NETWORK_TYPE_P2P) {
#if 0
			P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo;
			UINT_8 ucBitmapControl = 0;
			UINT_32 u4N1, u4N2;


			prP2pSpecificBssInfo = &(prAdapter->rWifiVar.rP2pSpecificBssInfo);

			
			prP2pSpecificBssInfo->ucBitmapCtrl = 0;
			kalMemZero(prP2pSpecificBssInfo->aucPartialVirtualBitmap,
				   sizeof(prP2pSpecificBssInfo->aucPartialVirtualBitmap));


			
			TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
			TIM_IE(pucBuffer)->ucDTIMCount = prBssInfo->ucDTIMCount;
			TIM_IE(pucBuffer)->ucDTIMPeriod = prBssInfo->ucDTIMPeriod;

			
			if (prBssInfo->ucDTIMCount == 0)
				

			

			
			u4N1 = (prP2pSpecificBssInfo->u2SmallestAID >> 4) << 1;	
			u4N2 = prP2pSpecificBssInfo->u2LargestAID >> 3;	

			ASSERT(u4N2 >= u4N1);

			kalMemCopy(TIM_IE(pucBuffer)->aucPartialVirtualMap,
				   &prP2pSpecificBssInfo->aucPartialVirtualBitmap[u4N1],
				   ((u4N2 - u4N1) + 1));

			
			
			if (prBssInfo->ucDTIMCount == 0)
				ucBitmapControl = prP2pSpecificBssInfo->ucBitmapCtrl;

			TIM_IE(pucBuffer)->ucBitmapControl = ucBitmapControl | (UINT_8) u4N1;

			TIM_IE(pucBuffer)->ucLength = ((u4N2 - u4N1) + 4);

			prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
#else

			
			TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
			
			TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP);
			TIM_IE(pucBuffer)->ucDTIMCount = 0 ;	
			TIM_IE(pucBuffer)->ucDTIMPeriod = prBssInfo->ucDTIMPeriod;
			
			TIM_IE(pucBuffer)->ucBitmapControl = 0 ;

			prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);

#endif

		} else
#endif				
		{
		}



	}

	return;
}				


VOID
bssComposeBeaconProbeRespFrameHeaderAndFF(IN PUINT_8 pucBuffer,
					  IN PUINT_8 pucDestAddr,
					  IN PUINT_8 pucOwnMACAddress,
					  IN PUINT_8 pucBSSID,
					  IN UINT_16 u2BeaconInterval, IN UINT_16 u2CapInfo)
{
	P_WLAN_BEACON_FRAME_T prBcnProbRspFrame;
	UINT_8 aucBCAddr[] = BC_MAC_ADDR;
	UINT_16 u2FrameCtrl;

	DEBUGFUNC("bssComposeBeaconProbeRespFrameHeaderAndFF");
	


	ASSERT(pucBuffer);
	ASSERT(pucOwnMACAddress);
	ASSERT(pucBSSID);

	prBcnProbRspFrame = (P_WLAN_BEACON_FRAME_T) pucBuffer;

	
	
	if (pucDestAddr) {
		u2FrameCtrl = MAC_FRAME_PROBE_RSP;
	} else {
		u2FrameCtrl = MAC_FRAME_BEACON;
		pucDestAddr = aucBCAddr;
	}
	
	prBcnProbRspFrame->u2FrameCtrl = u2FrameCtrl;	

	
	COPY_MAC_ADDR(prBcnProbRspFrame->aucDestAddr, pucDestAddr);

	
	COPY_MAC_ADDR(prBcnProbRspFrame->aucSrcAddr, pucOwnMACAddress);

	
	COPY_MAC_ADDR(prBcnProbRspFrame->aucBSSID, pucBSSID);

	
	prBcnProbRspFrame->u2SeqCtrl = 0;


	
	

	
	
	prBcnProbRspFrame->u2BeaconInterval = u2BeaconInterval;	

	
	
	prBcnProbRspFrame->u2CapInfo = u2CapInfo;	

	return;
}				


WLAN_STATUS bssUpdateBeaconContent(IN P_ADAPTER_T prAdapter, IN UINT_8 ucBssIndex)
{
	P_BSS_INFO_T prBssInfo;
	P_MSDU_INFO_T prMsduInfo;
	P_WLAN_BEACON_FRAME_T prBcnFrame;
	UINT_32 i;

	DEBUGFUNC("bssUpdateBeaconContent");
	DBGLOG(INIT, LOUD, ("\n"));

	ASSERT(ucBssIndex <= MAX_BSS_INDEX);

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	
	
	
	prMsduInfo = prBssInfo->prBeacon;

	
	if (prMsduInfo == NULL)
		return WLAN_STATUS_SUCCESS;

	
	bssComposeBeaconProbeRespFrameHeaderAndFF((PUINT_8)
						  ((ULONG) (prMsduInfo->prPacket) +
						   MAC_TX_RESERVED_FIELD), NULL,
						  prBssInfo->aucOwnMacAddr, prBssInfo->aucBSSID,
						  prBssInfo->u2BeaconInterval,
						  prBssInfo->u2CapInfo);


	prMsduInfo->u2FrameLength = (WLAN_MAC_MGMT_HEADER_LEN +
				     (TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN +
				      CAP_INFO_FIELD_LEN));

	prMsduInfo->ucBssIndex = ucBssIndex;

	
	bssBuildBeaconProbeRespFrameCommonIEs(prMsduInfo, prBssInfo, NULL);


	

	
	for (i = 0; i < sizeof(txBcnIETable) / sizeof(APPEND_VAR_IE_ENTRY_T); i++) {
		if (txBcnIETable[i].pfnAppendIE)
			txBcnIETable[i].pfnAppendIE(prAdapter, prMsduInfo);

	}

	prBcnFrame = (P_WLAN_BEACON_FRAME_T) prMsduInfo->prPacket;

	return nicUpdateBeaconIETemplate(prAdapter,
					 IE_UPD_METHOD_UPDATE_ALL,
					 ucBssIndex,
					 prBssInfo->u2CapInfo,
					 (PUINT_8) prBcnFrame->aucInfoElem,
					 prMsduInfo->u2FrameLength - OFFSET_OF(WLAN_BEACON_FRAME_T,
									       aucInfoElem));


}				


WLAN_STATUS
bssSendBeaconProbeResponse(IN P_ADAPTER_T prAdapter,
			   IN UINT_8 ucBssIndex, IN PUINT_8 pucDestAddr, IN UINT_32 u4ControlFlags)
{
	P_BSS_INFO_T prBssInfo;
	P_MSDU_INFO_T prMsduInfo;
	UINT_16 u2EstimatedFrameLen;
	UINT_16 u2EstimatedFixedIELen;
	UINT_16 u2EstimatedExtraIELen;
	P_APPEND_VAR_IE_ENTRY_T prIeArray = NULL;
	UINT_32 u4IeArraySize = 0;
	UINT_32 i;

	ASSERT(ucBssIndex <= MAX_BSS_INDEX);

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	if (!pucDestAddr) {	
		prIeArray = &txBcnIETable[0];
		u4IeArraySize = sizeof(txBcnIETable) / sizeof(APPEND_VAR_IE_ENTRY_T);
	} else {
		prIeArray = &txProbRspIETable[0];
		u4IeArraySize = sizeof(txProbRspIETable) / sizeof(APPEND_VAR_IE_ENTRY_T);
	}


	
	

	
	u2EstimatedFrameLen = MAC_TX_RESERVED_FIELD +
	    WLAN_MAC_MGMT_HEADER_LEN +
	    TIMESTAMP_FIELD_LEN +
	    BEACON_INTERVAL_FIELD_LEN +
	    CAP_INFO_FIELD_LEN +
	    (ELEM_HDR_LEN + ELEM_MAX_LEN_SSID) +
	    (ELEM_HDR_LEN + ELEM_MAX_LEN_SUP_RATES) +
	    (ELEM_HDR_LEN + ELEM_MAX_LEN_DS_PARAMETER_SET) +
	    (ELEM_HDR_LEN + ELEM_MAX_LEN_IBSS_PARAMETER_SET) +
	    (ELEM_HDR_LEN + (3 + MAX_LEN_TIM_PARTIAL_BMP));

	
	u2EstimatedExtraIELen = 0;

	for (i = 0; i < u4IeArraySize; i++) {
		u2EstimatedFixedIELen = prIeArray[i].u2EstimatedFixedIELen;

		if (u2EstimatedFixedIELen) {
			u2EstimatedExtraIELen += u2EstimatedFixedIELen;
		} else {
			ASSERT(prIeArray[i].pfnCalculateVariableIELen);

			u2EstimatedExtraIELen += (UINT_16)
			    prIeArray[i].pfnCalculateVariableIELen(prAdapter, ucBssIndex, NULL);
		}
	}

	u2EstimatedFrameLen += u2EstimatedExtraIELen;
	prMsduInfo = cnmMgtPktAlloc(prAdapter, u2EstimatedFrameLen);
	if (prMsduInfo == NULL) {
		DBGLOG(BSS, WARN, ("No PKT_INFO_T for sending %s.\n",
				   ((!pucDestAddr) ? "Beacon" : "Probe Response")));
		return WLAN_STATUS_RESOURCES;
	}

	
	
#if CFG_ENABLE_WIFI_DIRECT
	if (u4ControlFlags & BSS_PROBE_RESP_USE_P2P_DEV_ADDR) {
		if (prAdapter->fgIsP2PRegistered) {
			bssComposeBeaconProbeRespFrameHeaderAndFF((PUINT_8)
								  ((ULONG) (prMsduInfo->prPacket)
								   + MAC_TX_RESERVED_FIELD),
								  pucDestAddr,
								  prAdapter->rWifiVar.
								  aucDeviceAddress,
								  prAdapter->rWifiVar.
								  aucDeviceAddress,
								  DOT11_BEACON_PERIOD_DEFAULT,
								  (prBssInfo->
								   u2CapInfo & ~(CAP_INFO_ESS |
										 CAP_INFO_IBSS)));
		}
	} else
#endif				
	{
		bssComposeBeaconProbeRespFrameHeaderAndFF((PUINT_8)
							  ((ULONG) (prMsduInfo->prPacket) +
							   MAC_TX_RESERVED_FIELD), pucDestAddr,
							  prBssInfo->aucOwnMacAddr,
							  prBssInfo->aucBSSID,
							  prBssInfo->u2BeaconInterval,
							  prBssInfo->u2CapInfo);
	}


	

	TX_SET_MMPDU(prAdapter,
		     prMsduInfo,
		     ucBssIndex,
		     STA_REC_INDEX_BMCAST,
		     WLAN_MAC_MGMT_HEADER_LEN,
		     (WLAN_MAC_MGMT_HEADER_LEN + TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN +
		      CAP_INFO_FIELD_LEN), NULL, MSDU_RATE_MODE_AUTO);

	
	bssBuildBeaconProbeRespFrameCommonIEs(prMsduInfo, prBssInfo, pucDestAddr);


	

	
	for (i = 0; i < u4IeArraySize; i++) {
		if (prIeArray[i].pfnAppendIE)
			prIeArray[i].pfnAppendIE(prAdapter, prMsduInfo);

	}

	

	
	nicTxEnqueueMsdu(prAdapter, prMsduInfo);

	return WLAN_STATUS_SUCCESS;

}				


WLAN_STATUS bssProcessProbeRequest(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	P_WLAN_MAC_MGMT_HEADER_T prMgtHdr;
	P_BSS_INFO_T prBssInfo;
	UINT_8 ucBssIndex;
	UINT_8 aucBCBSSID[] = BC_BSSID;
	BOOLEAN fgIsBcBssid;
	BOOLEAN fgReplyProbeResp;
	UINT_32 u4CtrlFlagsForProbeResp = 0;
	ENUM_BAND_T eBand;
	UINT_8 ucHwChannelNum;


	ASSERT(prSwRfb);

	
	prMgtHdr = (P_WLAN_MAC_MGMT_HEADER_T) prSwRfb->pvHeader;

	if (EQUAL_MAC_ADDR(aucBCBSSID, prMgtHdr->aucBSSID))
		fgIsBcBssid = TRUE;
	else
		fgIsBcBssid = FALSE;



	
	for (ucBssIndex = 0; ucBssIndex <= P2P_DEV_BSS_INDEX; ucBssIndex++) {

		if ((ucBssIndex >= BSS_INFO_NUM) && (ucBssIndex != P2P_DEV_BSS_INDEX))
			continue;


		if (!IS_NET_ACTIVE(prAdapter, ucBssIndex))
			continue;


		prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

		if ((!fgIsBcBssid) && UNEQUAL_MAC_ADDR(prBssInfo->aucBSSID, prMgtHdr->aucBSSID))
			continue;


		eBand = HAL_RX_STATUS_GET_RF_BAND(prSwRfb->prRxStatus);
		ucHwChannelNum = HAL_RX_STATUS_GET_CHNL_NUM(prSwRfb->prRxStatus);

		if (prBssInfo->eBand != eBand)
			continue;


		if (prBssInfo->ucPrimaryChannel != ucHwChannelNum)
			continue;


		fgReplyProbeResp = FALSE;

		if (NETWORK_TYPE_AIS == prBssInfo->eNetworkType) {

#if CFG_SUPPORT_ADHOC
			fgReplyProbeResp =
			    aisValidateProbeReq(prAdapter, prSwRfb, &u4CtrlFlagsForProbeResp);
#endif
		}
#if CFG_ENABLE_WIFI_DIRECT
		else if ((prAdapter->fgIsP2PRegistered) &&
			 (NETWORK_TYPE_P2P == prBssInfo->eNetworkType)) {

			fgReplyProbeResp =
			    p2pFuncValidateProbeReq(prAdapter, prSwRfb, &u4CtrlFlagsForProbeResp,
						    (prBssInfo->ucBssIndex == P2P_DEV_BSS_INDEX),
						    (UINT_8) prBssInfo->u4PrivateData);
		}
#endif
#if CFG_ENABLE_BT_OVER_WIFI
		else if (NETWORK_TYPE_BOW == prBssInfo->eNetworkType) {

			fgReplyProbeResp =
			    bowValidateProbeReq(prAdapter, prSwRfb, &u4CtrlFlagsForProbeResp);
		}
#endif

		if (fgReplyProbeResp) {
			if (nicTxGetFreeCmdCount(prAdapter) > (CFG_TX_MAX_CMD_PKT_NUM / 2)) {
				
				bssSendBeaconProbeResponse(prAdapter, ucBssIndex,
							   prMgtHdr->aucSrcAddr,
							   u4CtrlFlagsForProbeResp);
			}
		}
	}

	return WLAN_STATUS_SUCCESS;

}				


#if 0				
WLAN_STATUS bssProcessProbeRequest(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	P_WLAN_MAC_MGMT_HEADER_T prMgtHdr;
	P_BSS_INFO_T prBssInfo;
	P_IE_SSID_T prIeSsid = (P_IE_SSID_T) NULL;
	P_IE_SUPPORTED_RATE_T prIeSupportedRate = (P_IE_SUPPORTED_RATE_T) NULL;
	P_IE_EXT_SUPPORTED_RATE_T prIeExtSupportedRate = (P_IE_EXT_SUPPORTED_RATE_T) NULL;
	PUINT_8 pucIE;
	UINT_16 u2IELength;
	UINT_16 u2Offset = 0;
	UINT_8 aucBCBSSID[] = BC_BSSID;
	ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex;
	BOOLEAN fgReplyProbeResp;
#if CFG_ENABLE_WIFI_DIRECT
	BOOLEAN fgP2PTargetDeviceFound;
	UINT_8 aucP2PWildcardSSID[] = P2P_WILDCARD_SSID;
#endif

	ASSERT(prSwRfb);

	
	prMgtHdr = (P_WLAN_MAC_MGMT_HEADER_T) prSwRfb->pvHeader;

	u2IELength = prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen;
	pucIE = (PUINT_8) ((UINT_32) prSwRfb->pvHeader + prSwRfb->u2HeaderLen);

	prIeSsid = (P_IE_SSID_T) NULL;

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		switch (IE_ID(pucIE)) {
		case ELEM_ID_SSID:
			if ((!prIeSsid) && (IE_LEN(pucIE) <= ELEM_MAX_LEN_SSID))
				prIeSsid = (P_IE_SSID_T) pucIE;

			break;

		case ELEM_ID_SUP_RATES:
			
			if (IE_LEN(pucIE) <= RATE_NUM_SW)
				prIeSupportedRate = SUP_RATES_IE(pucIE);

			break;

		case ELEM_ID_EXTENDED_SUP_RATES:
			prIeExtSupportedRate = EXT_SUP_RATES_IE(pucIE);
			break;

#if CFG_ENABLE_WIFI_DIRECT
			
		case ELEM_ID_P2P:

			break;
#endif

			
		}
	}			

	
	for (eNetTypeIndex = NETWORK_TYPE_AIS_INDEX; eNetTypeIndex < NETWORK_TYPE_INDEX_NUM;
	     eNetTypeIndex++) {

		if (!IS_NET_ACTIVE(prAdapter, eNetTypeIndex))
			continue;


		prBssInfo = &(prAdapter->rWifiVar.arBssInfo[eNetTypeIndex]);

		if (UNEQUAL_MAC_ADDR(aucBCBSSID, prMgtHdr->aucBSSID) &&
		    UNEQUAL_MAC_ADDR(prBssInfo->aucBSSID, prMgtHdr->aucBSSID)) {
			
			continue;
		}

		fgReplyProbeResp = FALSE;

		if (NETWORK_TYPE_AIS_INDEX == eNetTypeIndex) {

			if (prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {

				


				if (prIeSsid) {
					if ((prIeSsid->ucLength == BC_SSID_LEN) ||	
					    EQUAL_SSID(prBssInfo->aucSSID,
						       prBssInfo->ucSSIDLen,
						       prIeSsid->aucSSID,
						       prIeSsid->ucLength))
						fgReplyProbeResp = TRUE;

				}

			}
		}
#if CFG_ENABLE_WIFI_DIRECT
		else if (NETWORK_TYPE_P2P_INDEX == eNetTypeIndex) {

			

			if ((prIeSsid) &&
			    ((prIeSsid->ucLength == BC_SSID_LEN) ||
			     (EQUAL_SSID(aucP2PWildcardSSID,
					 P2P_WILDCARD_SSID_LEN,
					 prIeSsid->aucSSID, prIeSsid->ucLength)))) {
				if (p2pFsmRunEventRxProbeRequestFrame(prAdapter, prSwRfb)) {
					
					P_P2P_FSM_INFO_T prP2pFsmInfo = (P_P2P_FSM_INFO_T) NULL;

					
					

					P_MSG_SCN_SCAN_CANCEL prScanCancelMsg;

					prP2pFsmInfo = prAdapter->rWifiVar.prP2pFsmInfo;

					
					prScanCancelMsg =
					    (P_MSG_SCN_SCAN_CANCEL) cnmMemAlloc(prAdapter,
										RAM_TYPE_MSG,
										sizeof
										(MSG_SCN_SCAN_CANCEL));
					if (!prScanCancelMsg) {
						ASSERT(0);	
						continue;
					}

					prScanCancelMsg->rMsgHdr.eMsgId = MID_P2P_SCN_SCAN_CANCEL;
					prScanCancelMsg->ucSeqNum = prP2pFsmInfo->ucSeqNumOfScnMsg;
					prScanCancelMsg->ucNetTypeIndex =
					    (UINT_8) NETWORK_TYPE_P2P_INDEX;
					prScanCancelMsg->fgIsChannelExt = TRUE;

					mboxSendMsg(prAdapter,
						    MBOX_ID_0,
						    (P_MSG_HDR_T) prScanCancelMsg,
						    MSG_SEND_METHOD_BUF);
				}
			} else {
				continue;
			}

#if 0				
			if (prAdapter->rWifiVar.prP2pFsmInfo->eCurrentState == P2P_STATE_LISTEN) {

				if (prIeSupportedRate || prIeExtSupportedRate) {
					UINT_16 u2OperationalRateSet, u2BSSBasicRateSet;
					BOOLEAN fgIsUnknownBssBasicRate;
					
					rateGetRateSetFromIEs(prIeSupportedRate, prIeExtSupportedRate,
									&u2OperationalRateSet,
									&u2BSSBasicRateSet, &fgIsUnknownBssBasicRate);

					if (u2OperationalRateSet & ~RATE_SET_HR_DSSS)
						continue;

				}
			}
			

			if (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {
				


				if (prIeSsid) {
					UINT_8 aucSSID[] = P2P_WILDCARD_SSID;

					if ((prIeSsid->ucLength == BC_SSID_LEN) ||	
					    EQUAL_SSID(prBssInfo->aucSSID,
						       prBssInfo->ucSSIDLen,
						       prIeSsid->aucSSID,
						       prIeSsid->ucLength)
					    || EQUAL_SSID(aucSSID, P2P_WILDCARD_SSID_LEN,
							  prIeSsid->aucSSID,
							  prIeSsid->ucLength)) {
						fgReplyProbeResp = TRUE;
					}
				}


				
			}
#endif
		}
#endif
		else
			ASSERT(eNetTypeIndex < NETWORK_TYPE_INDEX_NUM);


		if (fgReplyProbeResp)
			bssSendBeaconProbeResponse(prAdapter, eNetTypeIndex, prMgtHdr->aucSrcAddr);


	}

	return WLAN_STATUS_SUCCESS;

}				
#endif


VOID bssInitializeClientList(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo)
{
	P_LINK_T prStaRecOfClientList;

	ASSERT(prBssInfo);

	prStaRecOfClientList = &prBssInfo->rStaRecOfClientList;

	if (!LINK_IS_EMPTY(prStaRecOfClientList))
		LINK_INITIALIZE(prStaRecOfClientList);

    DBGLOG(BSS, INFO, ("Init BSS[%u] Client List\n", prBssInfo->ucBssIndex));

    bssCheckClientList(prAdapter, prBssInfo);

	return;
}				



VOID
bssAddClient(IN P_ADAPTER_T prAdapter,
    IN P_BSS_INFO_T prBssInfo, IN P_STA_RECORD_T prStaRec)
{
	P_LINK_T prClientList;
    P_STA_RECORD_T prCurrStaRec;

	ASSERT(prBssInfo);

	prClientList = &prBssInfo->rStaRecOfClientList;

	LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry, STA_RECORD_T) {

		if (prCurrStaRec == prStaRec) {
			DBGLOG(BSS, WARN, ("Current Client List already contains that "
                "STA_RECORD_T["MACSTR"]\n", MAC2STR(prStaRec->aucMacAddr)));
			return;
		}
	}

	LINK_INSERT_TAIL(prClientList, &prStaRec->rLinkEntry);

    bssCheckClientList(prAdapter, prBssInfo);

	return;
}				


BOOLEAN
bssRemoveClient(IN P_ADAPTER_T prAdapter,
    IN P_BSS_INFO_T prBssInfo, IN P_STA_RECORD_T prStaRec)
{
	P_LINK_T prClientList;
    P_STA_RECORD_T prCurrStaRec;
    
	ASSERT(prBssInfo);

	prClientList = &prBssInfo->rStaRecOfClientList;

	LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry, STA_RECORD_T) {

		if (prCurrStaRec == prStaRec) {

			LINK_REMOVE_KNOWN_ENTRY(prClientList, &prStaRec->rLinkEntry);

			return TRUE;
		}
	}

	DBGLOG(BSS, INFO, ("Current Client List didn't contain that STA_RECORD_T[" 
        MACSTR"] before removing.\n", MAC2STR(prStaRec->aucMacAddr)));

    bssCheckClientList(prAdapter, prBssInfo);

	return FALSE;
}				

P_STA_RECORD_T
bssRemoveClientByMac(IN P_ADAPTER_T prAdapter,
    IN P_BSS_INFO_T prBssInfo, IN PUINT_8 pucMac)
{
	P_LINK_T prClientList;
    P_STA_RECORD_T prCurrStaRec;

	ASSERT(prBssInfo);

	prClientList = &prBssInfo->rStaRecOfClientList;

	LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry, STA_RECORD_T) {

		if (EQUAL_MAC_ADDR(prCurrStaRec->aucMacAddr, pucMac)) {

			LINK_REMOVE_KNOWN_ENTRY(prClientList,
						&prCurrStaRec->rLinkEntry);

			return prCurrStaRec;
		}
	}

	DBGLOG(BSS, INFO, ("Current Client List didn't contain that STA_RECORD_T[" 
        MACSTR"] before removing.\n", MAC2STR(pucMac)));

    bssCheckClientList(prAdapter, prBssInfo);

	return NULL;
}

P_STA_RECORD_T
bssRemoveHeadClient(IN P_ADAPTER_T prAdapter,
    IN P_BSS_INFO_T prBssInfo)
{
	P_LINK_T prStaRecOfClientList;
    P_STA_RECORD_T prStaRec = NULL;
    
	ASSERT(prBssInfo);

	prStaRecOfClientList = &prBssInfo->rStaRecOfClientList;

    if(!LINK_IS_EMPTY(prStaRecOfClientList)) {
		LINK_REMOVE_HEAD(prStaRecOfClientList, prStaRec, P_STA_RECORD_T);        
    }

    bssCheckClientList(prAdapter, prBssInfo);

    return prStaRec;
}

UINT_32
bssGetClientCount(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo)
{
    return prBssInfo->rStaRecOfClientList.u4NumElem;
}

VOID
bssDumpClientList(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo)
{
	P_LINK_T prClientList;
    P_STA_RECORD_T prCurrStaRec;
    UINT_8 ucCount = 0;
    
	ASSERT(prBssInfo);

	prClientList = &prBssInfo->rStaRecOfClientList;

	DBGLOG(SW4, INFO, ("Dump BSS[%u] Client List NUM[%u]\n", 
        prBssInfo->ucBssIndex, prClientList->u4NumElem));

    LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry, STA_RECORD_T) {

        if(!prCurrStaRec) {
		    DBGLOG(SW4, INFO, ("[%2u] is NULL STA_REC\n", ucCount));
            break;
        }
        else {
		    DBGLOG(SW4, INFO, ("[%2u] STA[%u] ["MACSTR"]\n", ucCount,
		        prCurrStaRec->ucIndex, MAC2STR(prCurrStaRec->aucMacAddr)));
        }
        
        ucCount++;        
    }
}

VOID
bssCheckClientList(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo)
{
	P_LINK_T prClientList;
    P_STA_RECORD_T prCurrStaRec;
    UINT_8 ucCount = 0;
    BOOLEAN fgError = FALSE;
    
	ASSERT(prBssInfo);

	prClientList = &prBssInfo->rStaRecOfClientList;

    
    if(prClientList->u4NumElem > P2P_MAXIMUM_CLIENT_COUNT) {
    	DBGLOG(SW4, INFO, ("BSS[%u] Client List NUM[%u] ERR\n", 
            prBssInfo->ucBssIndex, prClientList->u4NumElem));
        
        fgError = TRUE;
    }

    
    if(prClientList->u4NumElem == 0) {
        if((PVOID)prClientList->prNext != (PVOID)prClientList) {
            fgError = TRUE;
        }
        if((PVOID)prClientList->prPrev != (PVOID)prClientList) {
            fgError = TRUE;
        }
        
        if(fgError) {
        	DBGLOG(SW4, INFO, ("BSS[%u] Client List PTR next/prev[%p/%p] ERR\n", 
                prBssInfo->ucBssIndex, prClientList->prNext, 
                prClientList->prPrev));        
        }
    }

    
    LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry, STA_RECORD_T) {
        if(!prCurrStaRec) {
		    fgError = TRUE;
        	DBGLOG(SW4, INFO, ("BSS[%u] Client List NULL PTR ERR\n", 
                prBssInfo->ucBssIndex)); 
            
            break;
        }
        
        ucCount++;
    }

    
    if(ucCount != prClientList->u4NumElem) {
    	DBGLOG(SW4, INFO, ("BSS[%u] Client List NUM[%u] REAL CNT[%u] ERR\n", 
            prBssInfo->ucBssIndex, prClientList->u4NumElem, ucCount));
        
        fgError = TRUE;
    }

    if(fgError) {
        bssDumpClientList(prAdapter, prBssInfo);
    }
        
}

#endif				


#if CFG_SUPPORT_ADHOC
VOID
ibssProcessMatchedBeacon(IN P_ADAPTER_T prAdapter,
			 IN P_BSS_INFO_T prBssInfo, IN P_BSS_DESC_T prBssDesc, IN UINT_8 ucRCPI)
{
	P_STA_RECORD_T prStaRec = NULL;

	BOOLEAN fgIsCheckCapability = FALSE;
	BOOLEAN fgIsCheckTSF = FALSE;
	BOOLEAN fgIsGoingMerging = FALSE;
	BOOLEAN fgIsSameBSSID;


	ASSERT(prBssInfo);
	ASSERT(prBssDesc);

	
	if (!prBssInfo->fgIsBeaconActivated)
		return;

	
	prStaRec = cnmGetStaRecByAddress(prAdapter,
					 prAdapter->prAisBssInfo->ucBssIndex,
					 prBssDesc->aucSrcAddr);

	fgIsSameBSSID = UNEQUAL_MAC_ADDR(prBssInfo->aucBSSID, prBssDesc->aucBSSID) ? FALSE : TRUE;


	
	if (fgIsSameBSSID) {

		if (!prStaRec) {

			
			fgIsCheckCapability = TRUE;

			
			fgIsGoingMerging = TRUE;

		} else {

			ASSERT((prStaRec->ucBssIndex == prAdapter->prAisBssInfo->ucBssIndex) &&
			       IS_ADHOC_STA(prStaRec));

			if (prStaRec->ucStaState != STA_STATE_3) {

				if (!prStaRec->fgIsMerging) {

					
					
					
					fgIsCheckCapability = TRUE;

					
					fgIsGoingMerging = TRUE;
				} else {
					
					GET_CURRENT_SYSTIME(&prStaRec->rUpdateTime);
				}
			} else {
				
				GET_CURRENT_SYSTIME(&prStaRec->rUpdateTime);
			}

		}
	} else {


		fgIsCheckCapability = TRUE;

		
		fgIsCheckTSF = TRUE;

		
	}


	
	if (fgIsCheckCapability) {
		BOOLEAN fgIsCapabilityMatched = FALSE;

		do {
			if (!
			    (prBssDesc->
			     ucPhyTypeSet & (prAdapter->rWifiVar.ucAvailablePhyTypeSet))) {
				DBGLOG(BSS, LOUD,
				       ("IBSS MERGE: Ignore Peer MAC: " MACSTR
					" - Unsupported Phy.\n", MAC2STR(prBssDesc->aucSrcAddr)));

				break;
			}

			if (prBssDesc->fgIsUnknownBssBasicRate) {
				DBGLOG(BSS, LOUD,
				       ("IBSS MERGE: Ignore Peer MAC: " MACSTR
					" - Unknown Basic Rate.\n",
					MAC2STR(prBssDesc->aucSrcAddr)));

				break;
			}

			if (ibssCheckCapabilityForAdHocMode(prAdapter, prBssDesc) ==
			    WLAN_STATUS_FAILURE) {
				DBGLOG(BSS, LOUD,
				       ("IBSS MERGE: Ignore Peer MAC: " MACSTR
					" - Capability is not matched.\n",
					MAC2STR(prBssDesc->aucSrcAddr)));

				break;
			}

			fgIsCapabilityMatched = TRUE;
		} while (FALSE);

		if (!fgIsCapabilityMatched) {

			if (prStaRec) {

				DBGLOG(BSS, LOUD,
				       ("IBSS MERGE: Ignore Peer MAC: " MACSTR
					" - Capability is not matched.\n",
					MAC2STR(prBssDesc->aucSrcAddr)));
			}

			return;
		}

		DBGLOG(BSS, LOUD,
		       ("IBSS MERGE: Peer MAC: " MACSTR " - Check capability was passed.\n",
			MAC2STR(prBssDesc->aucSrcAddr)));
	}


	if (fgIsCheckTSF) {
#if CFG_SLT_SUPPORT
		fgIsGoingMerging = TRUE;
#else
		if (prBssDesc->fgIsLargerTSF)
			fgIsGoingMerging = TRUE;
		else
			return;

#endif
	}


	if (fgIsGoingMerging) {
		P_MSG_AIS_IBSS_PEER_FOUND_T prAisIbssPeerFoundMsg;


		
		prBssDesc->fgIsConnecting = TRUE;
		prBssDesc->fgIsConnected = FALSE;

		
		prStaRec = bssCreateStaRecFromBssDesc(prAdapter,
						      STA_TYPE_ADHOC_PEER,
						      prAdapter->prAisBssInfo->ucBssIndex,
						      prBssDesc);

		if (!prStaRec) {
			
			return;
		}

		prStaRec->fgIsMerging = TRUE;

		
		prStaRec->ucRCPI = ucRCPI;

		
		prAisIbssPeerFoundMsg = (P_MSG_AIS_IBSS_PEER_FOUND_T)
		    cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_AIS_IBSS_PEER_FOUND_T));

		if (!prAisIbssPeerFoundMsg) {

			ASSERT(0);	
			return;
		}

		prAisIbssPeerFoundMsg->rMsgHdr.eMsgId = MID_SCN_AIS_FOUND_IBSS;
		prAisIbssPeerFoundMsg->ucBssIndex = prAdapter->prAisBssInfo->ucBssIndex;
		prAisIbssPeerFoundMsg->prStaRec = prStaRec;

		if (fgIsSameBSSID) {
			prAisIbssPeerFoundMsg->fgIsMergeIn = TRUE;
		} else {
#if CFG_SLT_SUPPORT
			prAisIbssPeerFoundMsg->fgIsMergeIn = TRUE;
#else
			prAisIbssPeerFoundMsg->fgIsMergeIn =
			    (prBssDesc->fgIsLargerTSF) ? FALSE : TRUE;
#endif
		}

		mboxSendMsg(prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prAisIbssPeerFoundMsg, MSG_SEND_METHOD_BUF);

	}

	return;
}				


WLAN_STATUS ibssCheckCapabilityForAdHocMode(IN P_ADAPTER_T prAdapter, IN P_BSS_DESC_T prBssDesc)
{
	P_CONNECTION_SETTINGS_T prConnSettings;
	WLAN_STATUS rStatus = WLAN_STATUS_FAILURE;


	ASSERT(prBssDesc);
	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);

	do {
		
		if ((prConnSettings->eAdHocMode == AD_HOC_MODE_11B) &&
		    (prBssDesc->u2BSSBasicRateSet & ~RATE_SET_HR_DSSS)) {
			break;
		} else if ((prConnSettings->eAdHocMode == AD_HOC_MODE_11A) &&
			   (prBssDesc->u2BSSBasicRateSet & ~RATE_SET_OFDM)) {
			break;
		}
		
#if 0				
		if (prConnSettings->eAdHocMode == AD_HOC_MODE_11G) {
			if (((prConnSettings->fgIsShortSlotTimeOptionEnable) &&
			     !(prBssDesc->u2CapInfo & CAP_INFO_SHORT_SLOT_TIME)) ||
			    (!(prConnSettings->fgIsShortSlotTimeOptionEnable) &&
			     (prBssDesc->u2CapInfo & CAP_INFO_SHORT_SLOT_TIME))) {
				break;
			}
		}
#endif

		
		if (prBssDesc->u2ATIMWindow) {
			DBGLOG(BSS, INFO, ("AdHoc PS was not supported(ATIM Window: %d)\n",
					   prBssDesc->u2ATIMWindow));
			break;
		}
		
		if (!rsnPerformPolicySelection(prAdapter, prBssDesc))
			break;


		rStatus = WLAN_STATUS_SUCCESS;
	} while (FALSE);

	return rStatus;

}				


VOID ibssInitForAdHoc(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo)
{
	UINT_8 aucBSSID[MAC_ADDR_LEN];
	PUINT_16 pu2BSSID = (PUINT_16) &aucBSSID[0];
	UINT_32 i;


	ASSERT(prBssInfo);
	ASSERT(prBssInfo->eCurrentOPMode == OP_MODE_IBSS);


	
	prBssInfo->ucNonHTBasicPhyType = (UINT_8)
	    rNonHTAdHocModeAttributes[prBssInfo->ucConfigAdHocAPMode].ePhyTypeIndex;
	prBssInfo->u2BSSBasicRateSet =
	    rNonHTAdHocModeAttributes[prBssInfo->ucConfigAdHocAPMode].u2BSSBasicRateSet;


	prBssInfo->u2OperationalRateSet =
	    rNonHTPhyAttributes[prBssInfo->ucNonHTBasicPhyType].u2SupportedRateSet;

	rateGetDataRatesFromRateSet(prBssInfo->u2OperationalRateSet,
				    prBssInfo->u2BSSBasicRateSet,
				    prBssInfo->aucAllSupportedRates,
				    &prBssInfo->ucAllSupportedRatesLen);

	
	if (!prBssInfo->fgHoldSameBssidForIBSS) {

		for (i = 0; i < sizeof(aucBSSID) / sizeof(UINT_16); i++)
			pu2BSSID[i] = (UINT_16) (kalRandomNumber() & 0xFFFF);


		aucBSSID[0] &= ~0x01;	
		aucBSSID[0] |= 0x02;	

		COPY_MAC_ADDR(prBssInfo->aucBSSID, aucBSSID);
	}

	
	if (rNonHTPhyAttributes[prBssInfo->ucNonHTBasicPhyType].fgIsShortPreambleOptionImplemented &&
		
		((prAdapter->rWifiVar.ePreambleType == PREAMBLE_TYPE_SHORT) ||
		(prAdapter->rWifiVar.ePreambleType == PREAMBLE_TYPE_AUTO))) {

		prBssInfo->fgIsShortPreambleAllowed = TRUE;
		prBssInfo->fgUseShortPreamble = TRUE;
	} else {
		prBssInfo->fgIsShortPreambleAllowed = FALSE;
		prBssInfo->fgUseShortPreamble = FALSE;
	}


	
	
	prBssInfo->fgUseShortSlotTime = FALSE;	


	
	prBssInfo->u2CapInfo = CAP_INFO_IBSS;

	if (prBssInfo->fgIsProtection)
		prBssInfo->u2CapInfo |= CAP_INFO_PRIVACY;


	if (prBssInfo->fgIsShortPreambleAllowed)
		prBssInfo->u2CapInfo |= CAP_INFO_SHORT_PREAMBLE;


	if (prBssInfo->fgUseShortSlotTime)
		prBssInfo->u2CapInfo |= CAP_INFO_SHORT_SLOT_TIME;


	
	nicTxUpdateBssDefaultRate(prBssInfo);

	return;
}				

#endif				


#if CFG_SUPPORT_AAA

VOID bssInitForAP(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo, IN BOOLEAN fgIsRateUpdate)
{
	P_AC_QUE_PARMS_T prACQueParms;

	ENUM_WMM_ACI_T eAci;

	UINT_8 auCWminLog2ForBcast[WMM_AC_INDEX_NUM] = { 4 , 4 , 3 , 2  };
	UINT_8 auCWmaxLog2ForBcast[WMM_AC_INDEX_NUM] = { 10, 10, 4, 3 };
	UINT_8 auAifsForBcast[WMM_AC_INDEX_NUM] = { 3, 7, 2, 2 };
	UINT_8 auTxopForBcast[WMM_AC_INDEX_NUM] = { 0, 0, 94, 47 };	

	UINT_8 auCWminLog2[WMM_AC_INDEX_NUM] = { 4 , 4 , 3 , 2  };
	UINT_8 auCWmaxLog2[WMM_AC_INDEX_NUM] = { 6, 10, 4, 3 };
	UINT_8 auAifs[WMM_AC_INDEX_NUM] = { 3, 7, 1, 1 };
	UINT_8 auTxop[WMM_AC_INDEX_NUM] = { 0, 0, 94, 47 };	

	DEBUGFUNC("bssInitForAP");
	DBGLOG(BSS, LOUD, ("\n"));

	ASSERT(prBssInfo);
	ASSERT((prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT)
	       || (prBssInfo->eCurrentOPMode == OP_MODE_BOW));

#if 0
	prAdapter->rWifiVar.rConnSettings.fgRxShortGIDisabled = TRUE;
	prAdapter->rWifiVar.rConnSettings.uc2G4BandwidthMode = CONFIG_BW_20M;
	prAdapter->rWifiVar.rConnSettings.uc5GBandwidthMode = CONFIG_BW_20M;
#endif


	
	prBssInfo->ucNonHTBasicPhyType = (UINT_8)
	    rNonHTApModeAttributes[prBssInfo->ucConfigAdHocAPMode].ePhyTypeIndex;
	prBssInfo->u2BSSBasicRateSet =
	    rNonHTApModeAttributes[prBssInfo->ucConfigAdHocAPMode].u2BSSBasicRateSet;


	prBssInfo->u2OperationalRateSet =
	    rNonHTPhyAttributes[prBssInfo->ucNonHTBasicPhyType].u2SupportedRateSet;

	
	prBssInfo->u2BSSBasicRateSet &= ~RATE_SET_BIT_1M;
	

	if (fgIsRateUpdate) {
		rateGetDataRatesFromRateSet(prBssInfo->u2OperationalRateSet,
					    prBssInfo->u2BSSBasicRateSet,
					    prBssInfo->aucAllSupportedRates,
					    &prBssInfo->ucAllSupportedRatesLen);
	}
	
	COPY_MAC_ADDR(prBssInfo->aucBSSID, prBssInfo->aucOwnMacAddr);


	
	if (rNonHTPhyAttributes[prBssInfo->ucNonHTBasicPhyType].fgIsShortPreambleOptionImplemented &&
		
		((prAdapter->rWifiVar.ePreambleType == PREAMBLE_TYPE_SHORT) ||
		(prAdapter->rWifiVar.ePreambleType == PREAMBLE_TYPE_AUTO))) {

		prBssInfo->fgIsShortPreambleAllowed = TRUE;
		prBssInfo->fgUseShortPreamble = TRUE;
	} else {
		prBssInfo->fgIsShortPreambleAllowed = FALSE;
		prBssInfo->fgUseShortPreamble = FALSE;
	}


	
	prBssInfo->fgUseShortSlotTime = TRUE;

	
	prBssInfo->u2CapInfo = CAP_INFO_ESS;

	if (prBssInfo->fgIsProtection)
		prBssInfo->u2CapInfo |= CAP_INFO_PRIVACY;


	if (prBssInfo->fgIsShortPreambleAllowed)
		prBssInfo->u2CapInfo |= CAP_INFO_SHORT_PREAMBLE;


	if (prBssInfo->fgUseShortSlotTime)
		prBssInfo->u2CapInfo |= CAP_INFO_SHORT_SLOT_TIME;


	
	nicTxUpdateBssDefaultRate(prBssInfo);


	

	prACQueParms = prBssInfo->arACQueParmsForBcast;

	for (eAci = 0; eAci < WMM_AC_INDEX_NUM; eAci++) {

		prACQueParms[eAci].ucIsACMSet = FALSE;
		prACQueParms[eAci].u2Aifsn = auAifsForBcast[eAci];
		prACQueParms[eAci].u2CWmin = BIT(auCWminLog2ForBcast[eAci]) - 1;
		prACQueParms[eAci].u2CWmax = BIT(auCWmaxLog2ForBcast[eAci]) - 1;
		prACQueParms[eAci].u2TxopLimit = auTxopForBcast[eAci];

		prBssInfo->aucCWminLog2ForBcast[eAci] = auCWminLog2ForBcast[eAci];	
		prBssInfo->aucCWmaxLog2ForBcast[eAci] = auCWmaxLog2ForBcast[eAci];

		DBGLOG(BSS, INFO,
		       ("Bcast: eAci = %d, ACM = %d, Aifsn = %d, CWmin = %d, CWmax = %d, TxopLimit = %d\n",
			eAci, prACQueParms[eAci].ucIsACMSet, prACQueParms[eAci].u2Aifsn,
			prACQueParms[eAci].u2CWmin, prACQueParms[eAci].u2CWmax,
			prACQueParms[eAci].u2TxopLimit));

	}

	prACQueParms = prBssInfo->arACQueParms;

	for (eAci = 0; eAci < WMM_AC_INDEX_NUM; eAci++) {

		prACQueParms[eAci].ucIsACMSet = FALSE;
		prACQueParms[eAci].u2Aifsn = auAifs[eAci];
		prACQueParms[eAci].u2CWmin = BIT(auCWminLog2[eAci]) - 1;
		prACQueParms[eAci].u2CWmax = BIT(auCWmaxLog2[eAci]) - 1;
		prACQueParms[eAci].u2TxopLimit = auTxop[eAci];

		DBGLOG(BSS, INFO,
		       ("eAci = %d, ACM = %d, Aifsn = %d, CWmin = %d, CWmax = %d, TxopLimit = %d\n",
			eAci, prACQueParms[eAci].ucIsACMSet, prACQueParms[eAci].u2Aifsn,
			prACQueParms[eAci].u2CWmin, prACQueParms[eAci].u2CWmax,
			prACQueParms[eAci].u2TxopLimit));
	}

	
	
	


	return;
}				

#if 0
VOID bssUpdateDTIMCount(IN P_ADAPTER_T prAdapter, IN ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex)
{
	P_BSS_INFO_T prBssInfo;


	ASSERT(eNetTypeIndex < NETWORK_TYPE_INDEX_NUM);

	prBssInfo = &(prAdapter->rWifiVar.arBssInfo[eNetTypeIndex]);

	if (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {

		
		if (prBssInfo->ucDTIMCount > 0) {
			prBssInfo->ucDTIMCount--;
		} else {

			ASSERT(prBssInfo->ucDTIMPeriod > 0);

			prBssInfo->ucDTIMCount = prBssInfo->ucDTIMPeriod - 1;
		}
	}

	return;
}				


VOID bssSetTIMBitmap(IN P_ADAPTER_T prAdapter, IN P_BSS_INFO_T prBssInfo, IN UINT_16 u2AssocId)
{

	ASSERT(prBssInfo);

	if (prBssInfo->ucNetTypeIndex == NETWORK_TYPE_P2P_INDEX) {
		P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo;


		prP2pSpecificBssInfo = &(prAdapter->rWifiVar.rP2pSpecificBssInfo);

		
		if (u2AssocId == 0) {

			prP2pSpecificBssInfo->ucBitmapCtrl |= (UINT_8) BIT(0);
		} else {
			PUINT_8 pucPartialVirtualBitmap;
			UINT_8 ucBitmapToSet;


			pucPartialVirtualBitmap = &prP2pSpecificBssInfo->aucPartialVirtualBitmap[(u2AssocId >> 3)];
			ucBitmapToSet = (UINT_8) BIT((u2AssocId % 8));

			if (*pucPartialVirtualBitmap & ucBitmapToSet) {
				
				return;
			}

			*pucPartialVirtualBitmap |= ucBitmapToSet;

			
			if ((u2AssocId < prP2pSpecificBssInfo->u2SmallestAID) ||
			    (prP2pSpecificBssInfo->u2SmallestAID == 0)) {
				prP2pSpecificBssInfo->u2SmallestAID = u2AssocId;
			}

			if ((u2AssocId > prP2pSpecificBssInfo->u2LargestAID) ||
			    (prP2pSpecificBssInfo->u2LargestAID == 0)) {
				prP2pSpecificBssInfo->u2LargestAID = u2AssocId;
			}
		}
	}

	return;
}				
#endif

#endif				


VOID bssCreateStaRecFromAuth(IN P_ADAPTER_T prAdapter)
{

}


VOID bssUpdateStaRecFromAssocReq(IN P_ADAPTER_T prAdapter)
{

}


VOID bssDumpBssInfo(IN P_ADAPTER_T prAdapter, IN UINT_8 ucBssIndex)
{
	P_BSS_INFO_T prBssInfo;
	
	

	if (ucBssIndex > MAX_BSS_INDEX) {
		DBGLOG(SW4, INFO, ("Invalid BssInfo index[%u], skip dump!\n", ucBssIndex));
		return;
	}

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	if (!prBssInfo) {
		DBGLOG(SW4, INFO, ("Invalid BssInfo index[%u], skip dump!\n", ucBssIndex));
		return;
	}

	DBGLOG(SW4, INFO, ("OWNMAC[" MACSTR "] BSSID[" MACSTR "] SSID[%s]\n",
			   MAC2STR(prBssInfo->aucOwnMacAddr),
			   MAC2STR(prBssInfo->aucBSSID), prBssInfo->aucSSID));

	DBGLOG(SW4, INFO, ("BSS IDX[%u] Type[%s] OPMode[%s] ConnState[%u] Absent[%u]\n",
			   prBssInfo->ucBssIndex,
			   apucNetworkType[prBssInfo->eNetworkType],
			   apucNetworkOpMode[prBssInfo->eCurrentOPMode],
			   prBssInfo->eConnectionState, prBssInfo->fgIsNetAbsent));

	DBGLOG(SW4, INFO,
	       ("Channel[%u] Band[%u] SCO[%u] Assoc40mBwAllowed[%u] 40mBwAllowed[%u] MaxBw[%u]\n",
		prBssInfo->ucPrimaryChannel, prBssInfo->eBand, prBssInfo->eBssSCO,
		prBssInfo->fgAssoc40mBwAllowed, prBssInfo->fg40mBwAllowed, cnmGetBssMaxBw(prAdapter,
											  prBssInfo->
											  ucBssIndex)));

	DBGLOG(SW4, INFO, ("QBSS[%u] CapInfo[0x%04x] AID[%u]\n",
			   prBssInfo->fgIsQBSS, prBssInfo->u2CapInfo, prBssInfo->u2AssocId));

	DBGLOG(SW4, INFO, ("ShortPreamble Allowed[%u] EN[%u], ShortSlotTime[%u]\n",
			   prBssInfo->fgIsShortPreambleAllowed,
			   prBssInfo->fgUseShortPreamble, prBssInfo->fgUseShortSlotTime));

	DBGLOG(SW4, INFO, ("PhyTypeSet: Basic[0x%02x] NonHtBasic[0x%02x]\n",
			   prBssInfo->ucPhyTypeSet, prBssInfo->ucNonHTBasicPhyType));

	DBGLOG(SW4, INFO, ("RateSet: BssBasic[0x%04x] Operational[0x%04x]\n",
			   prBssInfo->u2BSSBasicRateSet, prBssInfo->u2OperationalRateSet));

	DBGLOG(SW4, INFO, ("ATIMWindow[%u] DTIM Period[%u] Count[%u]\n",
			   prBssInfo->u2ATIMWindow,
			   prBssInfo->ucDTIMPeriod, prBssInfo->ucDTIMCount));

	DBGLOG(SW4, INFO, ("HT Operation Info1[0x%02x] Info2[0x%04x] Info3[0x%04x]\n",
			   prBssInfo->ucHtOpInfo1, prBssInfo->u2HtOpInfo2, prBssInfo->u2HtOpInfo3));

	DBGLOG(SW4, INFO, ("ProtectMode HT[%u] ERP[%u], OperationMode GF[%u] RIFS[%u]\n",
			   prBssInfo->eHtProtectMode,
			   prBssInfo->fgErpProtectMode,
			   prBssInfo->eGfOperationMode, prBssInfo->eRifsOperationMode));

	DBGLOG(SW4, INFO, ("(OBSS) ProtectMode HT[%u] ERP[%u], OperationMode GF[%u] RIFS[%u]\n",
			   prBssInfo->eObssHtProtectMode,
			   prBssInfo->fgObssErpProtectMode,
			   prBssInfo->eObssGfOperationMode, prBssInfo->fgObssRifsOperationMode));

	DBGLOG(SW4, INFO, ("======== Dump Connected Client ========\n"));

#if 0
	DBGLOG(SW4, INFO, ("NumOfClient[%u]\n", bssGetClientCount(prAdapter, 
        prBssInfo)));

	prStaRecOfClientList = &prBssInfo->rStaRecOfClientList;

	LINK_FOR_EACH_ENTRY(prCurrStaRec, prStaRecOfClientList, rLinkEntry, STA_RECORD_T) {
		DBGLOG(SW4, INFO, ("STA[%u] [" MACSTR "]\n",
				   prCurrStaRec->ucIndex, MAC2STR(prCurrStaRec->aucMacAddr)));
	}
#else
    bssDumpClientList(prAdapter, prBssInfo);
#endif

	DBGLOG(SW4, INFO, ("============== Dump Done ==============\n"));
}
