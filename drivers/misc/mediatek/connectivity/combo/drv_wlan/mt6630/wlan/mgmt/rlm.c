





#include "precomp.h"




BOOLEAN g_bCaptureDone = FALSE;
BOOLEAN g_bIcapEnable = FALSE;
UINT_16 g_u2DumpIndex = 0;
BOOLEAN g_fgHasChannelSwitchIE = FALSE;

static VOID rlmFillHtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo);

static VOID
rlmFillExtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo);

static VOID rlmFillHtOpIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo);

static UINT_8
rlmRecIeInfoForClient(P_ADAPTER_T prAdapter,
		      P_BSS_INFO_T prBssInfo, PUINT_8 pucIE, UINT_16 u2IELength);

static BOOLEAN
rlmRecBcnFromNeighborForClient(P_ADAPTER_T prAdapter,
			       P_BSS_INFO_T prBssInfo,
			       P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength);

static BOOLEAN
rlmRecBcnInfoForClient(P_ADAPTER_T prAdapter,
		       P_BSS_INFO_T prBssInfo,
		       P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength);

static VOID rlmBssReset(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo);

#if CFG_SUPPORT_802_11AC
static VOID
rlmFillVhtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo);
#endif

VOID rlmFsmEventInit(P_ADAPTER_T prAdapter)
{
	ASSERT(prAdapter);


    
    rlmObssInit(prAdapter);
	
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
    rlmDomainCheckCountryPowerLimitTable (prAdapter);
#endif
	
    g_fgHasChannelSwitchIE = FALSE;
	g_bCaptureDone = FALSE ; 
	g_bIcapEnable = FALSE ; 
	g_u2DumpIndex = 0;
}

VOID rlmFsmEventUninit(P_ADAPTER_T prAdapter)
{
	P_BSS_INFO_T prBssInfo;
	UINT_8 i;

	ASSERT(prAdapter);

	for (i = 0; i < BSS_INFO_NUM; i++) {
		prBssInfo = prAdapter->aprBssInfo[i];

		rlmBssReset(prAdapter, prBssInfo);
	}
}

VOID rlmReqGenerateHtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	if ((prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11N) &&
	    (!prStaRec || (prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11N))) {

		rlmFillHtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

VOID rlmReqGenerateExtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	if ((prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11N) &&
	    (!prStaRec || (prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11N))) {

		rlmFillExtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
#if CFG_SUPPORT_PASSPOINT
	else if (prAdapter->prGlueInfo->fgConnectHS20AP == TRUE) {
		hs20FillExtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
#endif				
}

VOID rlmRspGenerateHtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11N(prBssInfo) && (ucPhyTypeSet & PHY_TYPE_SET_802_11N)) {

		rlmFillHtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

VOID rlmRspGenerateExtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11N(prBssInfo) && (ucPhyTypeSet & PHY_TYPE_SET_802_11N)) {

		rlmFillExtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

VOID rlmRspGenerateHtOpIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11N(prBssInfo) && (ucPhyTypeSet & PHY_TYPE_SET_802_11N)) {

		rlmFillHtOpIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

VOID rlmRspGenerateErpIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	P_IE_ERP_T prErpIe;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11GN(prBssInfo) && prBssInfo->eBand == BAND_2G4 &&
	    (ucPhyTypeSet & PHY_TYPE_SET_802_11GN)) {
		prErpIe = (P_IE_ERP_T)
		    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

		
		prErpIe->ucId = ELEM_ID_ERP_INFO;
		prErpIe->ucLength = 1;

		prErpIe->ucERP = prBssInfo->fgObssErpProtectMode ? ERP_INFO_USE_PROTECTION : 0;

		if (prBssInfo->fgErpProtectMode) {
			prErpIe->ucERP |= (ERP_INFO_NON_ERP_PRESENT | ERP_INFO_USE_PROTECTION);
		}

		
		if (!prBssInfo->fgUseShortPreamble) {
			prErpIe->ucERP |= ERP_INFO_BARKER_PREAMBLE_MODE;
		}

		ASSERT(IE_SIZE(prErpIe) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_ERP));

		prMsduInfo->u2FrameLength += IE_SIZE(prErpIe);
	}
}

#if CFG_SUPPORT_MTK_SYNERGY
VOID rlmGenerateMTKOuiIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	PUINT_8 pucBuffer;
	UINT_8 aucMtkOui[] = VENDOR_OUI_MTK;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	if (prAdapter->rWifiVar.ucMtkOui == FEATURE_DISABLED) {
		return;
	}

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	pucBuffer = (PUINT_8) ((ULONG) prMsduInfo->prPacket +
			       (ULONG) prMsduInfo->u2FrameLength);

	MTK_OUI_IE(pucBuffer)->ucId = ELEM_ID_VENDOR;
	MTK_OUI_IE(pucBuffer)->ucLength = ELEM_MIN_LEN_MTK_OUI;
	MTK_OUI_IE(pucBuffer)->aucOui[0] = aucMtkOui[0];
	MTK_OUI_IE(pucBuffer)->aucOui[1] = aucMtkOui[1];
	MTK_OUI_IE(pucBuffer)->aucOui[2] = aucMtkOui[2];
  MTK_OUI_IE(pucBuffer)->aucCapability[0] = MTK_SYNERGY_CAP0 & (prAdapter->rWifiVar.aucMtkFeature[0]);
  MTK_OUI_IE(pucBuffer)->aucCapability[1] = MTK_SYNERGY_CAP1 & (prAdapter->rWifiVar.aucMtkFeature[1]);
  MTK_OUI_IE(pucBuffer)->aucCapability[2] = MTK_SYNERGY_CAP2 & (prAdapter->rWifiVar.aucMtkFeature[2]);
  MTK_OUI_IE(pucBuffer)->aucCapability[3] = MTK_SYNERGY_CAP3 & (prAdapter->rWifiVar.aucMtkFeature[3]);

	prMsduInfo->u2FrameLength += IE_SIZE(pucBuffer);
	pucBuffer += IE_SIZE(pucBuffer);
}				

BOOLEAN rlmParseCheckMTKOuiIE(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBuf, IN PUINT_32 pu4Cap)
{
	UINT_8 aucMtkOui[] = VENDOR_OUI_MTK;
	P_IE_MTK_OUI_T prMtkOuiIE = (P_IE_MTK_OUI_T) NULL;

	do {
		ASSERT_BREAK((prAdapter != NULL) && (pucBuf != NULL));

		prMtkOuiIE = (P_IE_MTK_OUI_T) pucBuf;

		if (prAdapter->rWifiVar.ucMtkOui == FEATURE_DISABLED) {
			break;
		} else if (IE_LEN(pucBuf) < ELEM_MIN_LEN_MTK_OUI) {
			break;
		} else if (prMtkOuiIE->aucOui[0] != aucMtkOui[0] ||
			   prMtkOuiIE->aucOui[1] != aucMtkOui[1] ||
			   prMtkOuiIE->aucOui[2] != aucMtkOui[2]) {
			break;
		}

    
    prMtkOuiIE->aucCapability[0] = prMtkOuiIE->aucCapability[0] & (prAdapter->rWifiVar.aucMtkFeature[0]);
    prMtkOuiIE->aucCapability[1] = prMtkOuiIE->aucCapability[1] & (prAdapter->rWifiVar.aucMtkFeature[1]);
    prMtkOuiIE->aucCapability[2] = prMtkOuiIE->aucCapability[2] & (prAdapter->rWifiVar.aucMtkFeature[2]);
    prMtkOuiIE->aucCapability[3] = prMtkOuiIE->aucCapability[3] & (prAdapter->rWifiVar.aucMtkFeature[3]);

		kalMemCopy(pu4Cap, prMtkOuiIE->aucCapability, sizeof(UINT_32));

		return TRUE;
	} while (FALSE);

	return FALSE;
}				


#endif

static VOID rlmFillHtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo)
{
	P_IE_HT_CAP_T prHtCap;
	P_SUP_MCS_SET_FIELD prSupMcsSet;
	BOOLEAN fg40mAllowed;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	fg40mAllowed = prBssInfo->fgAssoc40mBwAllowed;

	prHtCap = (P_IE_HT_CAP_T)
	    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

	
	prHtCap->ucId = ELEM_ID_HT_CAP;
	prHtCap->ucLength = sizeof(IE_HT_CAP_T) - ELEM_HDR_LEN;

	prHtCap->u2HtCapInfo = HT_CAP_INFO_DEFAULT_VAL;

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxShortGI)) {
		prHtCap->u2HtCapInfo |= (HT_CAP_INFO_SHORT_GI_20M | HT_CAP_INFO_SHORT_GI_40M);
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxLdpc) && (wlanGetEcoVersion(prAdapter) > 2)) {
		prHtCap->u2HtCapInfo |= HT_CAP_INFO_LDPC_CAP;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxStbc)) {
		prHtCap->u2HtCapInfo |= HT_CAP_INFO_RX_STBC_1_SS;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxGf)) {
		prHtCap->u2HtCapInfo |= HT_CAP_INFO_HT_GF;
	}

	if (!fg40mAllowed) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SUP_CHNL_WIDTH |
					  HT_CAP_INFO_SHORT_GI_40M | HT_CAP_INFO_DSSS_CCK_IN_40M);
	}

	prHtCap->ucAmpduParam = AMPDU_PARAM_DEFAULT_VAL;

	prSupMcsSet = &prHtCap->rSupMcsSet;
	kalMemZero((PVOID) & prSupMcsSet->aucRxMcsBitmask[0], SUP_MCS_RX_BITMASK_OCTET_NUM);

	prSupMcsSet->aucRxMcsBitmask[0] = BITS(0, 7);

	if (fg40mAllowed) {
		prSupMcsSet->aucRxMcsBitmask[32 / 8] = BIT(0);	
	}
	prSupMcsSet->u2RxHighestSupportedRate = SUP_MCS_RX_DEFAULT_HIGHEST_RATE;
	prSupMcsSet->u4TxRateInfo = SUP_MCS_TX_DEFAULT_VAL;

	prHtCap->u2HtExtendedCap = HT_EXT_CAP_DEFAULT_VAL;
	if (!fg40mAllowed || prBssInfo->eCurrentOPMode != OP_MODE_INFRASTRUCTURE) {
		prHtCap->u2HtExtendedCap &= ~(HT_EXT_CAP_PCO | HT_EXT_CAP_PCO_TRANS_TIME_NONE);
	}

	prHtCap->u4TxBeamformingCap = TX_BEAMFORMING_CAP_DEFAULT_VAL;
	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucStaHtBfee)) {
		prHtCap->u4TxBeamformingCap = TX_BEAMFORMING_CAP_BFEE;
	}

	prHtCap->ucAselCap = ASEL_CAP_DEFAULT_VAL;


	ASSERT(IE_SIZE(prHtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP));

	prMsduInfo->u2FrameLength += IE_SIZE(prHtCap);
}

static VOID rlmFillExtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo)
{
	P_EXT_CAP_T prExtCap;
	BOOLEAN fg40mAllowed, fgAppendVhtCap;
    P_STA_RECORD_T prStaRec;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	fg40mAllowed = prBssInfo->fgAssoc40mBwAllowed;

	
	prExtCap = (P_EXT_CAP_T)
	    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

	prExtCap->ucId = ELEM_ID_EXTENDED_CAP;
#if 0				
	if (prAdapter->prGlueInfo->fgConnectHS20AP == TRUE)
		prExtCap->ucLength = ELEM_MAX_LEN_EXT_CAP;
	else
#endif
		prExtCap->ucLength = 1;

    
	kalMemZero(prExtCap->aucCapabilities, ELEM_MAX_LEN_EXT_CAP);

	prExtCap->aucCapabilities[0] = ELEM_EXT_CAP_DEFAULT_VAL;

	if (!fg40mAllowed) {
		prExtCap->aucCapabilities[0] &= ~ELEM_EXT_CAP_20_40_COEXIST_SUPPORT;
	}

	if (prBssInfo->eCurrentOPMode != OP_MODE_INFRASTRUCTURE) {
		prExtCap->aucCapabilities[0] &= ~ELEM_EXT_CAP_PSMP_CAP;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);
    
#if CFG_SUPPORT_802_11AC
    fgAppendVhtCap = FALSE;

    
    if(prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11AC) {
        
        
        if(prStaRec) {
            if(prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11AC) {
                fgAppendVhtCap = TRUE;
            }
        }
        else if((RLM_NET_IS_11AC(prBssInfo)) && 
            (prBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE)){
            fgAppendVhtCap = TRUE;
        }
    }

	if(fgAppendVhtCap) {
		if (prExtCap->ucLength < ELEM_MAX_LEN_EXT_CAP) {
			prExtCap->ucLength = ELEM_MAX_LEN_EXT_CAP;
		}

		SET_EXT_CAP(prExtCap->aucCapabilities, ELEM_MAX_LEN_EXT_CAP,
			    ELEM_EXT_CAP_OP_MODE_NOTIFICATION_BIT);

	}
#endif

#if CFG_SUPPORT_PASSPOINT
	if (prAdapter->prGlueInfo->fgConnectHS20AP == TRUE) {

		if (prExtCap->ucLength < ELEM_MAX_LEN_EXT_CAP) {
			prExtCap->ucLength = ELEM_MAX_LEN_EXT_CAP;
		}

		SET_EXT_CAP(prExtCap->aucCapabilities, ELEM_MAX_LEN_EXT_CAP,
			    ELEM_EXT_CAP_INTERWORKING_BIT);

		
		SET_EXT_CAP(prExtCap->aucCapabilities, ELEM_MAX_LEN_EXT_CAP,
			    ELEM_EXT_CAP_WNM_NOTIFICATION_BIT);
	}
#endif				

	ASSERT(IE_SIZE(prExtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_EXT_CAP));

	prMsduInfo->u2FrameLength += IE_SIZE(prExtCap);
}

static VOID rlmFillHtOpIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo)
{
	P_IE_HT_OP_T prHtOp;
	UINT_16 i;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	prHtOp = (P_IE_HT_OP_T)
	    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

	
	prHtOp->ucId = ELEM_ID_HT_OP;
	prHtOp->ucLength = sizeof(IE_HT_OP_T) - ELEM_HDR_LEN;

	
	prHtOp->ucPrimaryChannel = prBssInfo->ucPrimaryChannel;
	prHtOp->ucInfo1 = prBssInfo->ucHtOpInfo1;

	
	if (prBssInfo->eHtProtectMode == HT_PROTECT_MODE_NON_HT) {
		prHtOp->u2Info2 = (UINT_8) HT_PROTECT_MODE_NON_HT;
	} else if (prBssInfo->eObssHtProtectMode == HT_PROTECT_MODE_NON_MEMBER) {
		prHtOp->u2Info2 = (UINT_8) HT_PROTECT_MODE_NON_MEMBER;
	} else {
		
		prHtOp->u2Info2 = (UINT_8) prBssInfo->eHtProtectMode;
	}

	if (prBssInfo->eGfOperationMode != GF_MODE_NORMAL) {
		prHtOp->u2Info2 |= HT_OP_INFO2_NON_GF_HT_STA_PRESENT;
	}

	if (0   &&
	    prBssInfo->eObssHtProtectMode == HT_PROTECT_MODE_NON_MEMBER) {
		prHtOp->u2Info2 |= HT_OP_INFO2_OBSS_NON_HT_STA_PRESENT;
	}

	prHtOp->u2Info3 = prBssInfo->u2HtOpInfo3;	

	
	for (i = 0; i < 16; i++) {
		prHtOp->aucBasicMcsSet[i] = 0;
	}

	ASSERT(IE_SIZE(prHtOp) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_HT_OP));

	prMsduInfo->u2FrameLength += IE_SIZE(prHtOp);
}

#if CFG_SUPPORT_802_11AC

VOID rlmReqGenerateVhtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	if ((prAdapter->rWifiVar.ucAvailablePhyTypeSet & PHY_TYPE_SET_802_11AC) &&
	    (!prStaRec || (prStaRec->ucPhyTypeSet & PHY_TYPE_SET_802_11AC))) {

		rlmFillVhtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

VOID rlmRspGenerateVhtCapIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11AC(prBssInfo) && (ucPhyTypeSet & PHY_TYPE_SET_802_11AC)) {

		rlmFillVhtCapIE(prAdapter, prBssInfo, prMsduInfo);
	}

}

VOID rlmRspGenerateVhtOpIE(P_ADAPTER_T prAdapter, P_MSDU_INFO_T prMsduInfo)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPhyTypeSet;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	prStaRec = cnmGetStaRecByIndex(prAdapter, prMsduInfo->ucStaRecIndex);

	prBssInfo = prAdapter->aprBssInfo[prMsduInfo->ucBssIndex];
	if (!prBssInfo) {
		return;
	}

	if (!IS_BSS_ACTIVE(prBssInfo)) {
		return;
	}

	
	if (prStaRec) {
		
		ucPhyTypeSet = prStaRec->ucPhyTypeSet;
	} else {
		
		ucPhyTypeSet = prBssInfo->ucPhyTypeSet;
	}

	if (RLM_NET_IS_11AC(prBssInfo) && (ucPhyTypeSet & PHY_TYPE_SET_802_11AC)) {

		rlmFillVhtOpIE(prAdapter, prBssInfo, prMsduInfo);
	}
}

static VOID rlmFillVhtCapIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo)
{
	P_IE_VHT_CAP_T prVhtCap;
	P_VHT_SUPPORTED_MCS_FIELD prVhtSupportedMcsSet;
	UINT_8 i;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	prVhtCap = (P_IE_VHT_CAP_T)
	    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

	prVhtCap->ucId = ELEM_ID_VHT_CAP;
	prVhtCap->ucLength = sizeof(IE_VHT_CAP_T) - ELEM_HDR_LEN;
	prVhtCap->u4VhtCapInfo = VHT_CAP_INFO_DEFAULT_VAL;

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucStaVhtBfee) && (wlanGetEcoVersion(prAdapter) > 2)) {
		prVhtCap->u4VhtCapInfo |= FIELD_VHT_CAP_INFO_BF;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxShortGI)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_80;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxLdpc) && (wlanGetEcoVersion(prAdapter) > 2)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_RX_LDPC;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxStbc)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_RX_STBC_ONE_STREAM;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucTxStbc)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_TX_STBC;
	}

	
	prVhtSupportedMcsSet = &prVhtCap->rVhtSupportedMcsSet;
	kalMemZero((PVOID) prVhtSupportedMcsSet, sizeof(VHT_SUPPORTED_MCS_FIELD));

	for (i = 0; i < 8; i++) {
		prVhtSupportedMcsSet->u2RxMcsMap |= BITS(2 * i, (2 * i + 1));
		prVhtSupportedMcsSet->u2TxMcsMap |= BITS(2 * i, (2 * i + 1));
	}

	prVhtSupportedMcsSet->u2RxMcsMap &=
	    (VHT_CAP_INFO_MCS_MAP_MCS9 << VHT_CAP_INFO_MCS_1SS_OFFSET);
	prVhtSupportedMcsSet->u2TxMcsMap &=
	    (VHT_CAP_INFO_MCS_MAP_MCS9 << VHT_CAP_INFO_MCS_1SS_OFFSET);
	prVhtSupportedMcsSet->u2RxHighestSupportedDataRate = VHT_CAP_INFO_DEFAULT_HIGHEST_DATA_RATE;
	prVhtSupportedMcsSet->u2TxHighestSupportedDataRate = VHT_CAP_INFO_DEFAULT_HIGHEST_DATA_RATE;

	ASSERT(IE_SIZE(prVhtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP));

	prMsduInfo->u2FrameLength += IE_SIZE(prVhtCap);

}

VOID rlmFillVhtOpIE(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, P_MSDU_INFO_T prMsduInfo)
{
	P_IE_VHT_OP_T prVhtOp;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMsduInfo);

	prVhtOp = (P_IE_VHT_OP_T)
	    (((PUINT_8) prMsduInfo->prPacket) + prMsduInfo->u2FrameLength);

	
	prVhtOp->ucId = ELEM_ID_VHT_OP;
	prVhtOp->ucLength = sizeof(IE_VHT_OP_T) - ELEM_HDR_LEN;

	ASSERT(IE_SIZE(prVhtOp) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_OP));

	prVhtOp->ucVhtOperation[0] = prBssInfo->ucVhtChannelWidth;	
	prVhtOp->ucVhtOperation[1] = prBssInfo->ucVhtChannelFrequencyS1;
	prVhtOp->ucVhtOperation[2] = prBssInfo->ucVhtChannelFrequencyS2;


	prVhtOp->u2VhtBasicMcsSet = prBssInfo->u2VhtBasicMcsSet;

	prMsduInfo->u2FrameLength += IE_SIZE(prVhtOp);
}


#endif

static UINT_8
rlmRecIeInfoForClient(P_ADAPTER_T prAdapter,
		      P_BSS_INFO_T prBssInfo, PUINT_8 pucIE, UINT_16 u2IELength)
{
	UINT_16 u2Offset;
	P_STA_RECORD_T prStaRec;
	P_IE_HT_CAP_T prHtCap;
	P_IE_HT_OP_T prHtOp;
	P_IE_OBSS_SCAN_PARAM_T prObssScnParam;
	UINT_8 ucERP, ucPrimaryChannel;
	P_WIFI_VAR_T prWifiVar = &prAdapter->rWifiVar;
#if CFG_SUPPORT_QUIET && 0
	BOOLEAN fgHasQuietIE = FALSE;
#endif

#if CFG_SUPPORT_802_11AC
	P_IE_VHT_OP_T prVhtOp;
	P_IE_VHT_CAP_T prVhtCap;
	P_IE_OP_MODE_NOTIFICATION_T prOPModeNotification;	
	BOOLEAN fgHasOPModeIE = FALSE;
	UINT_8 ucVhtOpModeChannelWidth = 0;
	UINT_8 ucMaxBwAllowed;
#endif

#if CFG_SUPPORT_DFS
    BOOLEAN      					fgHasWideBandIE = FALSE;
    BOOLEAN      					fgHasSCOIE = FALSE;
    BOOLEAN                         fgHasChannelSwitchIE = FALSE;
    UINT_8       					ucChannelAnnouncePri;
    ENUM_CHNL_EXT_T       			eChannelAnnounceSco;
    UINT_8       					ucChannelAnnounceChannelS1 = 0;
    UINT_8       					ucChannelAnnounceChannelS2 = 0;
    UINT_8       					ucChannelAnnounceVhtBw;
    P_IE_CHANNEL_SWITCH_T 			prChannelSwitchAnnounceIE;
    P_IE_SECONDARY_OFFSET_T 		prSecondaryOffsetIE;
    P_IE_WIDE_BAND_CHANNEL_T 		prWideBandChannelIE;
#endif
	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(pucIE);

	prStaRec = prBssInfo->prStaRecOfAP;
	ASSERT(prStaRec);
	if (!prStaRec) {
		return 0;
	}

	prBssInfo->fgUseShortPreamble = prBssInfo->fgIsShortPreambleAllowed;
	ucPrimaryChannel = 0;
	prObssScnParam = NULL;
	ucMaxBwAllowed = cnmGetBssMaxBw(prAdapter, prBssInfo->ucBssIndex);

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		switch (IE_ID(pucIE)) {
		case ELEM_ID_HT_CAP:
			if (!RLM_NET_IS_11N(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_HT_CAP_T) - 2)) {
				break;
			}
			prHtCap = (P_IE_HT_CAP_T) pucIE;
			prStaRec->ucMcsSet = prHtCap->rSupMcsSet.aucRxMcsBitmask[0];
			prStaRec->fgSupMcs32 =
			    (prHtCap->rSupMcsSet.aucRxMcsBitmask[32 / 8] & BIT(0)) ? TRUE : FALSE;

			kalMemCopy(prStaRec->aucRxMcsBitmask, prHtCap->rSupMcsSet.aucRxMcsBitmask,
				   sizeof(prStaRec->
					  aucRxMcsBitmask) );
			prStaRec->u2RxHighestSupportedRate =
			    prHtCap->rSupMcsSet.u2RxHighestSupportedRate;
			prStaRec->u4TxRateInfo = prHtCap->rSupMcsSet.u4TxRateInfo;

			prStaRec->u2HtCapInfo = prHtCap->u2HtCapInfo;
			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxLdpc)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_LDPC_CAP;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxLdpc)
				   || (wlanGetEcoVersion(prAdapter) <= 2)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_LDPC_CAP;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxStbc)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_TX_STBC;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxStbc)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_TX_STBC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxShortGI)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_SHORT_GI_20M;
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_SHORT_GI_40M;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxShortGI)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_SHORT_GI_20M;
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_SHORT_GI_40M;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxGf)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_HT_GF;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxGf)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_HT_GF;
			}

			prStaRec->ucAmpduParam = prHtCap->ucAmpduParam;
			prStaRec->u2HtExtendedCap = prHtCap->u2HtExtendedCap;
			prStaRec->u4TxBeamformingCap = prHtCap->u4TxBeamformingCap;
			prStaRec->ucAselCap = prHtCap->ucAselCap;
			break;

		case ELEM_ID_HT_OP:
			if (!RLM_NET_IS_11N(prBssInfo) || IE_LEN(pucIE) != (sizeof(IE_HT_OP_T) - 2)) {
				break;
			}
			prHtOp = (P_IE_HT_OP_T) pucIE;
			if (ucPrimaryChannel == 0) {
				ucPrimaryChannel = prHtOp->ucPrimaryChannel;
			}
			prBssInfo->ucHtOpInfo1 = prHtOp->ucInfo1;
			prBssInfo->u2HtOpInfo2 = prHtOp->u2Info2;
			prBssInfo->u2HtOpInfo3 = prHtOp->u2Info3;

			if (!prBssInfo->fg40mBwAllowed) {
				prBssInfo->ucHtOpInfo1 &=
				    ~(HT_OP_INFO1_SCO | HT_OP_INFO1_STA_CHNL_WIDTH);
			}

			if ((prBssInfo->ucHtOpInfo1 & HT_OP_INFO1_SCO) != CHNL_EXT_RES) {
				prBssInfo->eBssSCO = (ENUM_CHNL_EXT_T)
				    (prBssInfo->ucHtOpInfo1 & HT_OP_INFO1_SCO);
			}

			prBssInfo->eHtProtectMode = (ENUM_HT_PROTECT_MODE_T)
			    (prBssInfo->u2HtOpInfo2 & HT_OP_INFO2_HT_PROTECTION);

			
			if ((prBssInfo->u2HtOpInfo2 & HT_OP_INFO2_OBSS_NON_HT_STA_PRESENT)
			    && 0 ) {
				prBssInfo->eGfOperationMode = GF_MODE_DISALLOWED;
			} else if (prBssInfo->u2HtOpInfo2 & HT_OP_INFO2_NON_GF_HT_STA_PRESENT) {
				prBssInfo->eGfOperationMode = GF_MODE_PROTECT;
			} else {
				prBssInfo->eGfOperationMode = GF_MODE_NORMAL;
			}

			prBssInfo->eRifsOperationMode =
			    (prBssInfo->ucHtOpInfo1 & HT_OP_INFO1_RIFS_MODE) ?
			    RIFS_MODE_NORMAL : RIFS_MODE_DISALLOWED;

			break;

#if CFG_SUPPORT_802_11AC
		case ELEM_ID_VHT_CAP:
			if (!RLM_NET_IS_11AC(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_VHT_CAP_T) - 2)) {
				break;
			}

			prVhtCap = (P_IE_VHT_CAP_T) pucIE;

			prStaRec->u4VhtCapInfo = prVhtCap->u4VhtCapInfo;
			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxLdpc)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_RX_LDPC;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxLdpc)
				   || (wlanGetEcoVersion(prAdapter) <= 2)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_RX_LDPC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxStbc)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_TX_STBC;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxStbc)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_TX_STBC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxopPsTx)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_VHT_TXOP_PS;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxopPsTx)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_VHT_TXOP_PS;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prWifiVar->ucTxShortGI)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_80;
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_160_80P80;
			} else if (IS_FEATURE_DISABLED(prWifiVar->ucTxShortGI)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_SHORT_GI_80;
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_SHORT_GI_160_80P80;
			}

			prStaRec->u2VhtRxMcsMap = prVhtCap->rVhtSupportedMcsSet.u2RxMcsMap;
			prStaRec->u2VhtRxHighestSupportedDataRate =
			    prVhtCap->rVhtSupportedMcsSet.u2RxHighestSupportedDataRate;
			prStaRec->u2VhtTxMcsMap = prVhtCap->rVhtSupportedMcsSet.u2TxMcsMap;
			prStaRec->u2VhtTxHighestSupportedDataRate =
			    prVhtCap->rVhtSupportedMcsSet.u2TxHighestSupportedDataRate;


			break;

		case ELEM_ID_VHT_OP:
			if (!RLM_NET_IS_11AC(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_VHT_OP_T) - 2)) {
				break;
			}

			prVhtOp = (P_IE_VHT_OP_T) pucIE;

			prBssInfo->ucVhtChannelWidth = prVhtOp->ucVhtOperation[0];
			prBssInfo->ucVhtChannelFrequencyS1 = prVhtOp->ucVhtOperation[1];
			prBssInfo->ucVhtChannelFrequencyS2 = prVhtOp->ucVhtOperation[2];
			prBssInfo->u2VhtBasicMcsSet = prVhtOp->u2VhtBasicMcsSet;

			break;


		case ELEM_ID_OP_MODE:
			if (!RLM_NET_IS_11AC(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_OP_MODE_NOTIFICATION_T) - 2)) {
				break;
			}
			prOPModeNotification = (P_IE_OP_MODE_NOTIFICATION_T) pucIE;

			if ((prOPModeNotification->ucOpMode & VHT_OP_MODE_RX_NSS_TYPE)
			    != VHT_OP_MODE_RX_NSS_TYPE) {
				fgHasOPModeIE = TRUE;
				ucVhtOpModeChannelWidth =
				    ((prOPModeNotification->ucOpMode) & VHT_OP_MODE_CHANNEL_WIDTH);
			}

            break;
#if CFG_SUPPORT_DFS
        case ELEM_ID_WIDE_BAND_CHANNEL_SWITCH:
            if (!RLM_NET_IS_11AC(prBssInfo) ||
                IE_LEN(pucIE) != (sizeof(IE_WIDE_BAND_CHANNEL_T) - 2)) {
                break;
            }
            DBGLOG(RLM, INFO, ("[Channel Switch] ELEM_ID_WIDE_BAND_CHANNEL_SWITCH, 11AC\n"));
            prWideBandChannelIE = (P_IE_WIDE_BAND_CHANNEL_T) pucIE;
            ucChannelAnnounceVhtBw = prWideBandChannelIE->ucNewChannelWidth;
            ucChannelAnnounceChannelS1 = prWideBandChannelIE->ucChannelS1;
            ucChannelAnnounceChannelS2 = prWideBandChannelIE->ucChannelS2;
            fgHasWideBandIE = TRUE;
            DBGLOG(RLM, INFO, ("[Ch] BW=%d, s1=%d, s2=%d\n",ucChannelAnnounceVhtBw,ucChannelAnnounceChannelS1,ucChannelAnnounceChannelS2));
            break;
#endif

#endif
		case ELEM_ID_20_40_BSS_COEXISTENCE:
			if (!RLM_NET_IS_11N(prBssInfo)) {
				break;
			}
			
			break;

		case ELEM_ID_OBSS_SCAN_PARAMS:
			if (!RLM_NET_IS_11N(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_OBSS_SCAN_PARAM_T) - 2)) {
				break;
			}
			
			prObssScnParam = (P_IE_OBSS_SCAN_PARAM_T) pucIE;
			break;

		case ELEM_ID_EXTENDED_CAP:
			if (!RLM_NET_IS_11N(prBssInfo)) {
				break;
			}
			
			break;

		case ELEM_ID_ERP_INFO:
			if (IE_LEN(pucIE) != (sizeof(IE_ERP_T) - 2) || prBssInfo->eBand != BAND_2G4) {
				break;
			}
			ucERP = ERP_INFO_IE(pucIE)->ucERP;
			prBssInfo->fgErpProtectMode =
			    (ucERP & ERP_INFO_USE_PROTECTION) ? TRUE : FALSE;

			if (ucERP & ERP_INFO_BARKER_PREAMBLE_MODE) {
				prBssInfo->fgUseShortPreamble = FALSE;
			}
			break;

        case ELEM_ID_DS_PARAM_SET:
            if (IE_LEN(pucIE) == ELEM_MAX_LEN_DS_PARAMETER_SET) {
                ucPrimaryChannel = DS_PARAM_IE(pucIE)->ucCurrChnl;
            }
            break;
#if CFG_SUPPORT_DFS
        case ELEM_ID_CH_SW_ANNOUNCEMENT:
		    if (IE_LEN(pucIE) != (sizeof(IE_CHANNEL_SWITCH_T) - 2)) {
                break;
            }

            prChannelSwitchAnnounceIE = (P_IE_CHANNEL_SWITCH_T) pucIE;
		
            DBGLOG(RLM, INFO, ("[Ch] Count=%d\n",prChannelSwitchAnnounceIE->ucChannelSwitchCount));

            if(prChannelSwitchAnnounceIE->ucChannelSwitchMode == 1){
                if (prChannelSwitchAnnounceIE->ucChannelSwitchCount <= 3 ) {
                    DBGLOG(RLM, INFO, ("[Ch] switch channel [%d]->[%d] \n", prBssInfo->ucPrimaryChannel, prChannelSwitchAnnounceIE->ucNewChannelNum));
                    ucChannelAnnouncePri = prChannelSwitchAnnounceIE->ucNewChannelNum; 
                    fgHasChannelSwitchIE = TRUE;
                    g_fgHasChannelSwitchIE = TRUE;
                }
                if (RLM_NET_IS_11AC(prBssInfo)) {
                    DBGLOG(RLM,INFO,("Send Operation Action Frame"));
                    rlmSendOpModeNotificationFrame(prAdapter, prStaRec, VHT_OP_MODE_CHANNEL_WIDTH_20, 1);
                }
                else {
                    DBGLOG(RLM,INFO,("Skip Send Operation Action Frame"));
                }
            }

            break;
	    case ELEM_ID_SCO :
			if (IE_LEN(pucIE) != (sizeof(IE_SECONDARY_OFFSET_T) - 2)) {
                break;
            }
			
            prSecondaryOffsetIE = (P_IE_SECONDARY_OFFSET_T) pucIE;
            DBGLOG(RLM, INFO, ("[Channel Switch] SCO [%d]->[%d] \n", prBssInfo->eBssSCO, prSecondaryOffsetIE->ucSecondaryOffset));
            eChannelAnnounceSco = (ENUM_CHNL_EXT_T)prSecondaryOffsetIE->ucSecondaryOffset;
            fgHasSCOIE = TRUE;
            break;
#endif

#if CFG_SUPPORT_QUIET && 0
		case ELEM_ID_QUIET:
			rrmQuietHandleQuietIE(prBssInfo, (P_IE_QUIET_T) pucIE);
			fgHasQuietIE = TRUE;
			break;
#endif
		default:
			break;
		}		
	}			

	if ((prBssInfo->eBand == BAND_2G4 && ucPrimaryChannel > 14) ||
	    (prBssInfo->eBand != BAND_2G4 && (ucPrimaryChannel >= 200 || ucPrimaryChannel <= 14))) {
		ucPrimaryChannel = 0;
	}
#if CFG_SUPPORT_802_11AC
	if (fgHasOPModeIE == TRUE) {

		
		if ((ucVhtOpModeChannelWidth == 0) || (ucVhtOpModeChannelWidth == 1)) {
			prBssInfo->ucVhtChannelWidth = 0;
		}
		
		else if (ucVhtOpModeChannelWidth == 2) {
			prBssInfo->ucVhtChannelWidth = 1;
		}
	}
#endif

#if CFG_SUPPORT_DFS 

    if (fgHasChannelSwitchIE != FALSE) {
		P_BSS_DESC_T prBssDesc;
		prBssInfo->ucPrimaryChannel = ucChannelAnnouncePri;
		prBssInfo->ucVhtChannelWidth = 0;
		prBssInfo->ucVhtChannelFrequencyS1 = 0;
		prBssInfo->ucVhtChannelFrequencyS2 = 0;
		prBssInfo->eBssSCO= 0;

		prBssDesc = scanSearchBssDescByBssid(prAdapter, prBssInfo->aucBSSID);

		if (prBssDesc) { 
			DBGLOG(RLM, INFO, ("DFS: BSS: "MACSTR" Desc found, channel from %u to %u\n ",
		    MAC2STR(prBssInfo->aucBSSID),prBssDesc->ucChannelNum, ucChannelAnnouncePri));
  		    prBssDesc->ucChannelNum = ucChannelAnnouncePri;
		}
		else {
			DBGLOG(RLM,INFO, ("DFS: BSS: "MACSTR" Desc is not found\n ",
		    MAC2STR(prBssInfo->aucBSSID)));
		}
		
		if (fgHasWideBandIE != FALSE) {
			prBssInfo->ucVhtChannelWidth = ucChannelAnnounceVhtBw;
        	prBssInfo->ucVhtChannelFrequencyS1 = ucChannelAnnounceChannelS1;
			prBssInfo->ucVhtChannelFrequencyS2 = ucChannelAnnounceChannelS2;
		}
		if (fgHasSCOIE != FALSE) {
        	prBssInfo->eBssSCO = eChannelAnnounceSco;
		}
    }
#endif

#if CFG_SUPPORT_DFS
    
	DBGLOG(RLM, INFO, ("Ch : SwitchIE = %d\n",g_fgHasChannelSwitchIE));
	if (g_fgHasChannelSwitchIE == TRUE) {
		prBssInfo->eBssSCO = CHNL_EXT_SCN;
		prBssInfo->ucVhtChannelWidth = CW_20_40MHZ;
		prBssInfo->ucVhtChannelFrequencyS1 = 0;
		prBssInfo->ucVhtChannelFrequencyS2 = 255;
	    prBssInfo->ucHtOpInfo1 &=
                 ~(HT_OP_INFO1_SCO | HT_OP_INFO1_STA_CHNL_WIDTH);
		DBGLOG(RLM, INFO, ("Ch : DFS has Appeared\n"));
	}
#endif

    
    if (prBssInfo->eBand == BAND_2G4) {
        if (prBssInfo->ucVhtChannelWidth == CW_80MHZ) {

            
            DBGLOG(RLM, WARN, ("rlmRecIeInfoForClient: B=%d, W=%d\n",prBssInfo->eBand, prBssInfo->ucVhtChannelWidth));
            DBGLOG(RLM, WARN, ("IE Length= %u\n",u2IELength));
            DBGLOG_MEM8(RLM, WARN, pucIE, u2IELength);

            
            prBssInfo->ucVhtChannelWidth = CW_20_40MHZ; 
            prBssInfo->ucVhtChannelFrequencyS1 = 0;
            prBssInfo->ucVhtChannelFrequencyS2 = 0;
			
            
            prBssInfo->eBssSCO = CHNL_EXT_SCN;
            prBssInfo->ucHtOpInfo1 &=
            ~(HT_OP_INFO1_SCO | HT_OP_INFO1_STA_CHNL_WIDTH);
        }
    }

#if CFG_SUPPORT_QUIET && 0
	if (!fgHasQuietIE) {
		rrmQuietIeNotExist(prAdapter, prBssInfo);
	}
#endif

	
	if (!prAdapter->fgEnOnlineScan || !prObssScnParam ||
	    !(prStaRec->u2HtCapInfo & HT_CAP_INFO_SUP_CHNL_WIDTH) ||
	    prBssInfo->eBand != BAND_2G4 || !prBssInfo->fg40mBwAllowed) {

		
		prBssInfo->u2ObssScanInterval = 0;
	} else {
		if (prObssScnParam->u2TriggerScanInterval < OBSS_SCAN_MIN_INTERVAL) {
			prObssScnParam->u2TriggerScanInterval = OBSS_SCAN_MIN_INTERVAL;
		}
		if (prBssInfo->u2ObssScanInterval != prObssScnParam->u2TriggerScanInterval) {

			prBssInfo->u2ObssScanInterval = prObssScnParam->u2TriggerScanInterval;

			
			cnmTimerStartTimer(prAdapter, &prBssInfo->rObssScanTimer,
					   prBssInfo->u2ObssScanInterval * MSEC_PER_SEC);
		}
	}

	return ucPrimaryChannel;
}

static BOOLEAN
rlmRecBcnFromNeighborForClient(P_ADAPTER_T prAdapter,
			       P_BSS_INFO_T prBssInfo,
			       P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength)
{
	UINT_16 u2Offset, i;
	UINT_8 ucPriChannel, ucSecChannel;
	ENUM_CHNL_EXT_T eSCO;
	BOOLEAN fgHtBss, fg20mReq;

	ASSERT(prAdapter);
	ASSERT(prBssInfo && prSwRfb);
	ASSERT(pucIE);

	
	ucPriChannel = 0;
	eSCO = CHNL_EXT_SCN;

	fgHtBss = FALSE;
	fg20mReq = FALSE;

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		switch (IE_ID(pucIE)) {
		case ELEM_ID_HT_CAP:
			{
				P_IE_HT_CAP_T prHtCap;

				if (IE_LEN(pucIE) != (sizeof(IE_HT_CAP_T) - 2)) {
					break;
				}

				prHtCap = (P_IE_HT_CAP_T) pucIE;
				if (prHtCap->u2HtCapInfo & HT_CAP_INFO_40M_INTOLERANT) {
					fg20mReq = TRUE;
				}
				fgHtBss = TRUE;
				break;
			}
		case ELEM_ID_HT_OP:
			{
				P_IE_HT_OP_T prHtOp;

				if (IE_LEN(pucIE) != (sizeof(IE_HT_OP_T) - 2)) {
					break;
				}

				prHtOp = (P_IE_HT_OP_T) pucIE;
				if (ucPriChannel == 0) {
					ucPriChannel = prHtOp->ucPrimaryChannel;
				}

				if ((prHtOp->ucInfo1 & HT_OP_INFO1_SCO) != CHNL_EXT_RES) {
					eSCO =
					    (ENUM_CHNL_EXT_T) (prHtOp->ucInfo1 & HT_OP_INFO1_SCO);
				}
				break;
			}
		case ELEM_ID_20_40_BSS_COEXISTENCE:
			{
				P_IE_20_40_COEXIST_T prCoexist;

				if (IE_LEN(pucIE) != (sizeof(IE_20_40_COEXIST_T) - 2)) {
					break;
				}

				prCoexist = (P_IE_20_40_COEXIST_T) pucIE;
				if (prCoexist->ucData & BSS_COEXIST_40M_INTOLERANT) {
					fg20mReq = TRUE;
				}
				break;
			}
		case ELEM_ID_DS_PARAM_SET:
			if (IE_LEN(pucIE) != (sizeof(IE_DS_PARAM_SET_T) - 2)) {
				break;
			}
			ucPriChannel = DS_PARAM_IE(pucIE)->ucCurrChnl;
			break;

		default:
			break;
		}
	}


	if (HAL_RX_STATUS_GET_RF_BAND(prSwRfb->prRxStatus) != BAND_2G4) {
		return FALSE;
	}

	if (ucPriChannel == 0 || ucPriChannel > 14) {
		ucPriChannel = HAL_RX_STATUS_GET_CHNL_NUM(prSwRfb->prRxStatus);
	}

	if (fgHtBss) {
		ASSERT(prBssInfo->auc2G_PriChnlList[0] <= CHNL_LIST_SZ_2G);
		for (i = 1; i <= prBssInfo->auc2G_PriChnlList[0] && i <= CHNL_LIST_SZ_2G; i++) {
			if (prBssInfo->auc2G_PriChnlList[i] == ucPriChannel) {
				break;
			}
		}
		if ((i > prBssInfo->auc2G_PriChnlList[0]) && (i <= CHNL_LIST_SZ_2G)) {
			prBssInfo->auc2G_PriChnlList[i] = ucPriChannel;
			prBssInfo->auc2G_PriChnlList[0]++;
		}

		
		if (eSCO != CHNL_EXT_SCN) {
			ucSecChannel = (eSCO == CHNL_EXT_SCA) ?
			    (ucPriChannel + 4) : (ucPriChannel - 4);

			ASSERT(prBssInfo->auc2G_SecChnlList[0] <= CHNL_LIST_SZ_2G);
			for (i = 1; i <= prBssInfo->auc2G_SecChnlList[0] &&
			     i <= CHNL_LIST_SZ_2G; i++) {
				if (prBssInfo->auc2G_SecChnlList[i] == ucSecChannel) {
					break;
				}
			}
			if ((i > prBssInfo->auc2G_SecChnlList[0]) && (i <= CHNL_LIST_SZ_2G)) {
				prBssInfo->auc2G_SecChnlList[i] = ucSecChannel;
				prBssInfo->auc2G_SecChnlList[0]++;
			}
		}

		
		if (fg20mReq) {
			ASSERT(prBssInfo->auc2G_20mReqChnlList[0] <= CHNL_LIST_SZ_2G);
			for (i = 1; i <= prBssInfo->auc2G_20mReqChnlList[0] &&
			     i <= CHNL_LIST_SZ_2G; i++) {
				if (prBssInfo->auc2G_20mReqChnlList[i] == ucPriChannel) {
					break;
				}
			}
			if ((i > prBssInfo->auc2G_20mReqChnlList[0]) && (i <= CHNL_LIST_SZ_2G)) {
				prBssInfo->auc2G_20mReqChnlList[i] = ucPriChannel;
				prBssInfo->auc2G_20mReqChnlList[0]++;
			}
		}
	} else {
		
		ASSERT(prBssInfo->auc2G_NonHtChnlList[0] <= CHNL_LIST_SZ_2G);
		for (i = 1; i <= prBssInfo->auc2G_NonHtChnlList[0] && i <= CHNL_LIST_SZ_2G; i++) {
			if (prBssInfo->auc2G_NonHtChnlList[i] == ucPriChannel) {
				break;
			}
		}
		if ((i > prBssInfo->auc2G_NonHtChnlList[0]) && (i <= CHNL_LIST_SZ_2G)) {
			prBssInfo->auc2G_NonHtChnlList[i] = ucPriChannel;
			prBssInfo->auc2G_NonHtChnlList[0]++;
		}

	}

	return FALSE;
}


static BOOLEAN
rlmRecBcnInfoForClient(P_ADAPTER_T prAdapter,
		       P_BSS_INFO_T prBssInfo,
		       P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength)
{
	ASSERT(prAdapter);
	ASSERT(prBssInfo && prSwRfb);
	ASSERT(pucIE);

#if 0				
	if (IS_SCAN_ACTIVE()
	    
	    ) {
		return FALSE;
	}
#endif

	
	prBssInfo->u2CapInfo = ((P_WLAN_BEACON_FRAME_T) (prSwRfb->pvHeader))->u2CapInfo;
	prBssInfo->fgUseShortSlotTime =
	    ((prBssInfo->u2CapInfo & CAP_INFO_SHORT_SLOT_TIME)
	     || (prBssInfo->eBand != BAND_2G4)) ? TRUE : FALSE;

	rlmRecIeInfoForClient(prAdapter, prBssInfo, pucIE, u2IELength);

	return TRUE;
}

VOID rlmProcessBcn(P_ADAPTER_T prAdapter, P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength)
{
	P_BSS_INFO_T prBssInfo;
	BOOLEAN fgNewParameter;
	UINT_8 i;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);
	ASSERT(pucIE);

	fgNewParameter = FALSE;

	for (i = 0; i < BSS_INFO_NUM; i++) {
		prBssInfo = prAdapter->aprBssInfo[i];

		if (IS_BSS_BOW(prBssInfo)) {
			continue;
		}

		if (IS_BSS_ACTIVE(prBssInfo)) {
			if (prBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE &&
			    prBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED) {
				
				if (EQUAL_MAC_ADDR(prBssInfo->aucBSSID, ((P_WLAN_MAC_MGMT_HEADER_T)
									 (prSwRfb->pvHeader))->
						   aucBSSID)) {

					fgNewParameter = rlmRecBcnInfoForClient(prAdapter,
										prBssInfo, prSwRfb,
										pucIE, u2IELength);
				} else {
					fgNewParameter = rlmRecBcnFromNeighborForClient(prAdapter,
											prBssInfo,
											prSwRfb,
											pucIE,
											u2IELength);
				}
			}
#if CFG_ENABLE_WIFI_DIRECT
			else if (prAdapter->fgIsP2PRegistered &&
				 (prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT ||
				  prBssInfo->eCurrentOPMode == OP_MODE_P2P_DEVICE)) {
				
				rlmRecBcnFromNeighborForClient(prAdapter,
							       prBssInfo, prSwRfb, pucIE,
							       u2IELength);
			}
#endif
			else if (prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {
				
			}

			
			if (fgNewParameter) {
				rlmSyncOperationParams(prAdapter, prBssInfo);
				fgNewParameter = FALSE;
			}
		}		
	}
}

VOID
rlmProcessAssocRsp(P_ADAPTER_T prAdapter, P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_8 ucPriChannel;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);
	ASSERT(pucIE);

	prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);
	if (!prStaRec) {
		return;
	}

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);
	if (!prBssInfo) {
		return;
	}

	if (prStaRec != prBssInfo->prStaRecOfAP) {
		return;
	}

	rlmBssReset(prAdapter, prBssInfo);

	prBssInfo->fgUseShortSlotTime =
	    ((prBssInfo->u2CapInfo & CAP_INFO_SHORT_SLOT_TIME)
	     || (prBssInfo->eBand != BAND_2G4)) ? TRUE : FALSE;
	ucPriChannel = rlmRecIeInfoForClient(prAdapter, prBssInfo, pucIE, u2IELength);
	
	if (ucPriChannel > 0) {

		if (ucPriChannel != prBssInfo->ucPrimaryChannel) {

		DBGLOG(RLM, INFO, ("[WARNNING] Bss channel (%d) is not the same as Peer IE channel (%d) \n"
			,prBssInfo->ucPrimaryChannel,ucPriChannel));

		}
		
		
	}

	if (!RLM_NET_IS_11N(prBssInfo) || !(prStaRec->u2HtCapInfo & HT_CAP_INFO_SUP_CHNL_WIDTH)) {
		prBssInfo->fg40mBwAllowed = FALSE;
	}

}

VOID rlmFillSyncCmdParam(P_CMD_SET_BSS_RLM_PARAM_T prCmdBody, P_BSS_INFO_T prBssInfo)
{
	ASSERT(prCmdBody && prBssInfo);
	if (!prCmdBody || !prBssInfo) {
		return;
	}

	prCmdBody->ucBssIndex = prBssInfo->ucBssIndex;
	prCmdBody->ucRfBand = (UINT_8) prBssInfo->eBand;
	prCmdBody->ucPrimaryChannel = prBssInfo->ucPrimaryChannel;
	prCmdBody->ucRfSco = (UINT_8) prBssInfo->eBssSCO;
	prCmdBody->ucErpProtectMode = (UINT_8) prBssInfo->fgErpProtectMode;
	prCmdBody->ucHtProtectMode = (UINT_8) prBssInfo->eHtProtectMode;
	prCmdBody->ucGfOperationMode = (UINT_8) prBssInfo->eGfOperationMode;
	prCmdBody->ucTxRifsMode = (UINT_8) prBssInfo->eRifsOperationMode;
	prCmdBody->u2HtOpInfo3 = prBssInfo->u2HtOpInfo3;
	prCmdBody->u2HtOpInfo2 = prBssInfo->u2HtOpInfo2;
	prCmdBody->ucHtOpInfo1 = prBssInfo->ucHtOpInfo1;
	prCmdBody->ucUseShortPreamble = prBssInfo->fgUseShortPreamble;
	prCmdBody->ucUseShortSlotTime = prBssInfo->fgUseShortSlotTime;
	prCmdBody->ucVhtChannelWidth = prBssInfo->ucVhtChannelWidth;
	prCmdBody->ucVhtChannelFrequencyS1 = prBssInfo->ucVhtChannelFrequencyS1;
	prCmdBody->ucVhtChannelFrequencyS2 = prBssInfo->ucVhtChannelFrequencyS2;
	prCmdBody->u2VhtBasicMcsSet = prBssInfo->u2BSSBasicRateSet;
    
    if (RLM_NET_PARAM_VALID(prBssInfo)) {
         DBGLOG(RLM, INFO, ("N=%d b=%d c=%d s=%d e=%d h=%d I=0x%02x l=%d p=%d w=%d s1=%d s2=%d\n",
            prCmdBody->ucBssIndex, prCmdBody->ucRfBand,
            prCmdBody->ucPrimaryChannel, prCmdBody->ucRfSco,
            prCmdBody->ucErpProtectMode, prCmdBody->ucHtProtectMode,
            prCmdBody->ucHtOpInfo1, prCmdBody->ucUseShortSlotTime,
            prCmdBody->ucUseShortPreamble,
            prCmdBody->ucVhtChannelWidth,
            prCmdBody->ucVhtChannelFrequencyS1,
            prCmdBody->ucVhtChannelFrequencyS2            
            ));
    }
    else {
        DBGLOG(RLM, INFO, ("N=%d closed\n", prCmdBody->ucBssIndex));
    }
}

VOID rlmSyncOperationParams(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo)
{
	P_CMD_SET_BSS_RLM_PARAM_T prCmdBody;
	WLAN_STATUS rStatus;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);

	prCmdBody = (P_CMD_SET_BSS_RLM_PARAM_T)
	    cnmMemAlloc(prAdapter, RAM_TYPE_BUF, sizeof(CMD_SET_BSS_RLM_PARAM_T));

	
	
	if (!prCmdBody) {
		DBGLOG(RLM, WARN, ("No buf for sync RLM params (Net=%d)\n", prBssInfo->ucBssIndex));
		return;
	}

	rlmFillSyncCmdParam(prCmdBody, prBssInfo);

	rStatus = wlanSendSetQueryCmd(prAdapter,	
				      CMD_ID_SET_BSS_RLM_PARAM,	
				      TRUE,	
				      FALSE,	
				      FALSE,	
				      NULL,	
				      NULL,	
				      sizeof(CMD_SET_BSS_RLM_PARAM_T),	
				      (PUINT_8) prCmdBody,	
				      NULL,	
				      0	
	    );

	
	if (rStatus != WLAN_STATUS_PENDING) {
		DBGLOG(RLM, WARN, ("rlmSyncOperationParams set cmd fail\n"));
	}

	cnmMemFree(prAdapter, prCmdBody);
}

#if CFG_SUPPORT_AAA
VOID
rlmProcessAssocReq(P_ADAPTER_T prAdapter, P_SW_RFB_T prSwRfb, PUINT_8 pucIE, UINT_16 u2IELength)
{
	P_BSS_INFO_T prBssInfo;
	P_STA_RECORD_T prStaRec;
	UINT_16 u2Offset;
	P_IE_HT_CAP_T prHtCap;
#if CFG_SUPPORT_802_11AC
	P_IE_VHT_CAP_T prVhtCap;
#endif

	ASSERT(prAdapter);
	ASSERT(prSwRfb);
	ASSERT(pucIE);

	prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);
	if (!prStaRec) {
		return;
	}
	ASSERT(prStaRec->ucBssIndex <= MAX_BSS_INDEX);

	prBssInfo = prAdapter->aprBssInfo[prStaRec->ucBssIndex];

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		switch (IE_ID(pucIE)) {
		case ELEM_ID_HT_CAP:
			if (!RLM_NET_IS_11N(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_HT_CAP_T) - 2)) {
				break;
			}
			prHtCap = (P_IE_HT_CAP_T) pucIE;
			prStaRec->ucMcsSet = prHtCap->rSupMcsSet.aucRxMcsBitmask[0];
			prStaRec->fgSupMcs32 =
			    (prHtCap->rSupMcsSet.aucRxMcsBitmask[32 / 8] & BIT(0)) ? TRUE : FALSE;

			prStaRec->u2HtCapInfo = prHtCap->u2HtCapInfo;

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxLdpc)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_LDPC_CAP;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxLdpc)
				   || (wlanGetEcoVersion(prAdapter) <= 2)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_LDPC_CAP;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxStbc)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_TX_STBC;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxStbc)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_TX_STBC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxShortGI)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_SHORT_GI_20M;
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_SHORT_GI_40M;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxShortGI)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_SHORT_GI_20M;
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_SHORT_GI_40M;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxGf)) {
				prStaRec->u2HtCapInfo |= HT_CAP_INFO_HT_GF;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxGf)) {
				prStaRec->u2HtCapInfo &= ~HT_CAP_INFO_HT_GF;
			}

			prStaRec->ucAmpduParam = prHtCap->ucAmpduParam;
			prStaRec->u2HtExtendedCap = prHtCap->u2HtExtendedCap;
			prStaRec->u4TxBeamformingCap = prHtCap->u4TxBeamformingCap;
			prStaRec->ucAselCap = prHtCap->ucAselCap;
			break;

#if CFG_SUPPORT_802_11AC
		case ELEM_ID_VHT_CAP:
			if (!RLM_NET_IS_11AC(prBssInfo) ||
			    IE_LEN(pucIE) != (sizeof(IE_VHT_CAP_T) - 2)) {
				break;
			}

			prVhtCap = (P_IE_VHT_CAP_T) pucIE;

			prStaRec->u4VhtCapInfo = prVhtCap->u4VhtCapInfo;

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxLdpc)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_RX_LDPC;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxLdpc)
				   || (wlanGetEcoVersion(prAdapter) <= 2)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_RX_LDPC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxStbc)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_TX_STBC;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxStbc)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_TX_STBC;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxopPsTx)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_VHT_TXOP_PS;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxopPsTx)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_VHT_TXOP_PS;
			}

			
			if (IS_FEATURE_FORCE_ENABLED(prAdapter->rWifiVar.ucTxShortGI)) {
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_80;
				prStaRec->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_160_80P80;
			} else if (IS_FEATURE_DISABLED(prAdapter->rWifiVar.ucTxShortGI)) {
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_SHORT_GI_80;
				prStaRec->u4VhtCapInfo &= ~VHT_CAP_INFO_SHORT_GI_160_80P80;
			}

			prStaRec->u2VhtRxMcsMap = prVhtCap->rVhtSupportedMcsSet.u2RxMcsMap;
			prStaRec->u2VhtRxHighestSupportedDataRate =
			    prVhtCap->rVhtSupportedMcsSet.u2RxHighestSupportedDataRate;
			prStaRec->u2VhtTxMcsMap = prVhtCap->rVhtSupportedMcsSet.u2TxMcsMap;
			prStaRec->u2VhtTxHighestSupportedDataRate =
			    prVhtCap->rVhtSupportedMcsSet.u2TxHighestSupportedDataRate;

			break;
#endif

		default:
			break;
		}		
	}			
}
#endif				

VOID rlmBssInitForAPandIbss(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo)
{
	ASSERT(prAdapter);
	ASSERT(prBssInfo);

#if CFG_ENABLE_WIFI_DIRECT
	if (prAdapter->fgIsP2PRegistered && prBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT) {

		rlmBssInitForAP(prAdapter, prBssInfo);
	}
#endif
}

VOID rlmBssAborted(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo)
{
	ASSERT(prAdapter);
	ASSERT(prBssInfo);

	rlmBssReset(prAdapter, prBssInfo);

	prBssInfo->fg40mBwAllowed = FALSE;
	prBssInfo->fgAssoc40mBwAllowed = FALSE;

}

static VOID rlmBssReset(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo)
{
	ASSERT(prAdapter);
	ASSERT(prBssInfo);

	
	prBssInfo->ucHtOpInfo1 = 0;	
	prBssInfo->u2HtOpInfo2 = 0;
	prBssInfo->u2HtOpInfo3 = 0;

#if CFG_SUPPORT_802_11AC
	prBssInfo->ucVhtChannelWidth = 0;	
	prBssInfo->ucVhtChannelFrequencyS1 = 0;	
	prBssInfo->ucVhtChannelFrequencyS2 = 0;
	prBssInfo->u2VhtBasicMcsSet = 0;	
#endif

	prBssInfo->eBssSCO = 0;
	prBssInfo->fgErpProtectMode = 0;
	prBssInfo->eHtProtectMode = 0;
	prBssInfo->eGfOperationMode = 0;
	prBssInfo->eRifsOperationMode = 0;

	
	prBssInfo->auc2G_20mReqChnlList[0] = 0;
	prBssInfo->auc2G_NonHtChnlList[0] = 0;
	prBssInfo->auc2G_PriChnlList[0] = 0;
	prBssInfo->auc2G_SecChnlList[0] = 0;
	prBssInfo->auc5G_20mReqChnlList[0] = 0;
	prBssInfo->auc5G_NonHtChnlList[0] = 0;
	prBssInfo->auc5G_PriChnlList[0] = 0;
	prBssInfo->auc5G_SecChnlList[0] = 0;

	
	cnmTimerStopTimer(prAdapter, &prBssInfo->rObssScanTimer);
	prBssInfo->u2ObssScanInterval = 0;

	prBssInfo->fgObssErpProtectMode = 0;	
	prBssInfo->eObssHtProtectMode = 0;	
	prBssInfo->eObssGfOperationMode = 0;	
	prBssInfo->fgObssRifsOperationMode = 0;	
	prBssInfo->fgObssActionForcedTo20M = 0;	
	prBssInfo->fgObssBeaconForcedTo20M = 0;	
}


#if CFG_SUPPORT_TDLS
UINT_32 rlmFillVhtCapIEByAdapter(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, UINT_8 *pOutBuf)
{
	P_IE_VHT_CAP_T prVhtCap;
	P_VHT_SUPPORTED_MCS_FIELD prVhtSupportedMcsSet;
	UINT_8 i;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	

	prVhtCap = (P_IE_VHT_CAP_T) pOutBuf;


	prVhtCap->ucId = ELEM_ID_VHT_CAP;
	prVhtCap->ucLength = sizeof(IE_VHT_CAP_T) - ELEM_HDR_LEN;
	prVhtCap->u4VhtCapInfo = VHT_CAP_INFO_DEFAULT_VAL;

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxShortGI)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_SHORT_GI_80;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxLdpc)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_RX_LDPC;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxStbc)) {
		prVhtCap->u4VhtCapInfo |= VHT_CAP_INFO_RX_STBC_ONE_STREAM;
	}

	
	prVhtSupportedMcsSet = &prVhtCap->rVhtSupportedMcsSet;
	kalMemZero((PVOID) prVhtSupportedMcsSet, sizeof(VHT_SUPPORTED_MCS_FIELD));

	for (i = 0; i < 8; i++) {
		prVhtSupportedMcsSet->u2RxMcsMap |= BITS(2 * i, (2 * i + 1));
		prVhtSupportedMcsSet->u2TxMcsMap |= BITS(2 * i, (2 * i + 1));
	}

	prVhtSupportedMcsSet->u2RxMcsMap &=
	    (VHT_CAP_INFO_MCS_MAP_MCS9 << VHT_CAP_INFO_MCS_1SS_OFFSET);
	prVhtSupportedMcsSet->u2TxMcsMap &=
	    (VHT_CAP_INFO_MCS_MAP_MCS9 << VHT_CAP_INFO_MCS_1SS_OFFSET);
	prVhtSupportedMcsSet->u2RxHighestSupportedDataRate = VHT_CAP_INFO_DEFAULT_HIGHEST_DATA_RATE;
	prVhtSupportedMcsSet->u2TxHighestSupportedDataRate = VHT_CAP_INFO_DEFAULT_HIGHEST_DATA_RATE;

	ASSERT(IE_SIZE(prVhtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_VHT_CAP));

	return IE_SIZE(prVhtCap);
}
#endif

#if CFG_SUPPORT_TDLS
UINT_32
rlmFillHtCapIEByParams(BOOLEAN fg40mAllowed,
		       BOOLEAN fgShortGIDisabled,
		       UINT_8 u8SupportRxSgi20,
		       UINT_8 u8SupportRxSgi40,
		       UINT_8 u8SupportRxGf, ENUM_OP_MODE_T eCurrentOPMode, UINT_8 *pOutBuf)
{
	P_IE_HT_CAP_T prHtCap;
	P_SUP_MCS_SET_FIELD prSupMcsSet;

	ASSERT(pOutBuf);

	prHtCap = (P_IE_HT_CAP_T) pOutBuf;

	
	prHtCap->ucId = ELEM_ID_HT_CAP;
	prHtCap->ucLength = sizeof(IE_HT_CAP_T) - ELEM_HDR_LEN;

	prHtCap->u2HtCapInfo = HT_CAP_INFO_DEFAULT_VAL;
	if (!fg40mAllowed) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SUP_CHNL_WIDTH |
					  HT_CAP_INFO_SHORT_GI_40M | HT_CAP_INFO_DSSS_CCK_IN_40M);
	}
	if (fgShortGIDisabled) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SHORT_GI_20M | HT_CAP_INFO_SHORT_GI_40M);
	}

	if (u8SupportRxSgi20 == 2) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SHORT_GI_20M);
	}
	if (u8SupportRxSgi40 == 2) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SHORT_GI_40M);
	}
	if (u8SupportRxGf == 2) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_HT_GF);
	}

	prHtCap->ucAmpduParam = AMPDU_PARAM_DEFAULT_VAL;

	prSupMcsSet = &prHtCap->rSupMcsSet;
	kalMemZero((PVOID) & prSupMcsSet->aucRxMcsBitmask[0], SUP_MCS_RX_BITMASK_OCTET_NUM);

	prSupMcsSet->aucRxMcsBitmask[0] = BITS(0, 7);

	if (fg40mAllowed) {
		prSupMcsSet->aucRxMcsBitmask[32 / 8] = BIT(0);	
	}
	prSupMcsSet->u2RxHighestSupportedRate = SUP_MCS_RX_DEFAULT_HIGHEST_RATE;
	prSupMcsSet->u4TxRateInfo = SUP_MCS_TX_DEFAULT_VAL;

	prHtCap->u2HtExtendedCap = HT_EXT_CAP_DEFAULT_VAL;
	if (!fg40mAllowed || eCurrentOPMode != OP_MODE_INFRASTRUCTURE) {
		prHtCap->u2HtExtendedCap &= ~(HT_EXT_CAP_PCO | HT_EXT_CAP_PCO_TRANS_TIME_NONE);
	}

	prHtCap->u4TxBeamformingCap = TX_BEAMFORMING_CAP_DEFAULT_VAL;

	prHtCap->ucAselCap = ASEL_CAP_DEFAULT_VAL;


	ASSERT(IE_SIZE(prHtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP));

	return IE_SIZE(prHtCap);
}

UINT_32 rlmFillHtCapIEByAdapter(P_ADAPTER_T prAdapter, P_BSS_INFO_T prBssInfo, UINT_8 *pOutBuf)
{
	P_IE_HT_CAP_T prHtCap;
	P_SUP_MCS_SET_FIELD prSupMcsSet;
	BOOLEAN fg40mAllowed;

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(pOutBuf);

	fg40mAllowed = prBssInfo->fgAssoc40mBwAllowed;

	prHtCap = (P_IE_HT_CAP_T) pOutBuf;


	
	prHtCap->ucId = ELEM_ID_HT_CAP;
	prHtCap->ucLength = sizeof(IE_HT_CAP_T) - ELEM_HDR_LEN;

	prHtCap->u2HtCapInfo = HT_CAP_INFO_DEFAULT_VAL;
	if (!fg40mAllowed) {
		prHtCap->u2HtCapInfo &= ~(HT_CAP_INFO_SUP_CHNL_WIDTH |
					  HT_CAP_INFO_SHORT_GI_40M | HT_CAP_INFO_DSSS_CCK_IN_40M);
	}
	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxShortGI)) {
		prHtCap->u2HtCapInfo |= (HT_CAP_INFO_SHORT_GI_20M | HT_CAP_INFO_SHORT_GI_40M);
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxLdpc)) {
		prHtCap->u2HtCapInfo |= HT_CAP_INFO_LDPC_CAP;
	}

	if (IS_FEATURE_ENABLED(prAdapter->rWifiVar.ucRxStbc)) {
		prHtCap->u2HtCapInfo |= HT_CAP_INFO_RX_STBC_1_SS;
	}

	prHtCap->ucAmpduParam = AMPDU_PARAM_DEFAULT_VAL;

	prSupMcsSet = &prHtCap->rSupMcsSet;
	kalMemZero((PVOID) & prSupMcsSet->aucRxMcsBitmask[0], SUP_MCS_RX_BITMASK_OCTET_NUM);

	prSupMcsSet->aucRxMcsBitmask[0] = BITS(0, 7);

	if (fg40mAllowed) {
		prSupMcsSet->aucRxMcsBitmask[32 / 8] = BIT(0);	
	}
	prSupMcsSet->u2RxHighestSupportedRate = SUP_MCS_RX_DEFAULT_HIGHEST_RATE;
	prSupMcsSet->u4TxRateInfo = SUP_MCS_TX_DEFAULT_VAL;

	prHtCap->u2HtExtendedCap = HT_EXT_CAP_DEFAULT_VAL;
	if (!fg40mAllowed || prBssInfo->eCurrentOPMode != OP_MODE_INFRASTRUCTURE) {
		prHtCap->u2HtExtendedCap &= ~(HT_EXT_CAP_PCO | HT_EXT_CAP_PCO_TRANS_TIME_NONE);
	}

	prHtCap->u4TxBeamformingCap = TX_BEAMFORMING_CAP_DEFAULT_VAL;

	prHtCap->ucAselCap = ASEL_CAP_DEFAULT_VAL;


	ASSERT(IE_SIZE(prHtCap) <= (ELEM_HDR_LEN + ELEM_MAX_LEN_HT_CAP));

	return IE_SIZE(prHtCap);

}


#endif

#if CFG_SUPPORT_DFS
VOID
rlmProcessSpecMgtAction(
        P_ADAPTER_T prAdapter,
        P_SW_RFB_T  prSwRfb
        )
{
	PUINT_8 pucIE;
	P_STA_RECORD_T prStaRec;
    P_BSS_INFO_T prBssInfo;
    UINT_16 u2IELength;
    UINT_16 u2Offset = 0;
	P_IE_CHANNEL_SWITCH_T 			prChannelSwitchAnnounceIE;
	P_IE_SECONDARY_OFFSET_T 		prSecondaryOffsetIE;
	P_IE_WIDE_BAND_CHANNEL_T 		prWideBandChannelIE;
	P_ACTION_CHANNEL_SWITCH_FRAME    prRxFrame;
	BOOLEAN      					fgHasWideBandIE = FALSE;
	BOOLEAN 						fgHasSCOIE = FALSE;
	BOOLEAN                         fgHasChannelSwitchIE = FALSE;
	
	DBGLOG(RLM, INFO, ("[Mgt Action]rlmProcessSpecMgtAction\n"));
    ASSERT(prAdapter);
    ASSERT(prSwRfb);
	
    u2IELength = (prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) -
            (UINT_16)(OFFSET_OF(ACTION_CHANNEL_SWITCH_FRAME, aucInfoElem[0]) - WLAN_MAC_MGMT_HEADER_LEN);
	
    prRxFrame = (P_ACTION_CHANNEL_SWITCH_FRAME) prSwRfb->pvHeader;
    pucIE = prRxFrame->aucInfoElem;

    prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);


	DBGLOG_MEM8(RLM, INFO, pucIE, u2IELength);
    if (prRxFrame->ucAction == ACTION_CHNL_SWITCH) {
        IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
            switch (IE_ID(pucIE)) {
       
            case ELEM_ID_WIDE_BAND_CHANNEL_SWITCH:
                if (!RLM_NET_IS_11AC(prBssInfo) ||
                    IE_LEN(pucIE) != (sizeof(IE_WIDE_BAND_CHANNEL_T) - 2)) {
                    DBGLOG(RLM, INFO, ("[Mgt Action] ELEM_ID_WIDE_BAND_CHANNEL_SWITCH, Length\n"));
					break;
                }
                DBGLOG(RLM, INFO, ("[Mgt Action] ELEM_ID_WIDE_BAND_CHANNEL_SWITCH, 11AC\n"));
                prWideBandChannelIE = (P_IE_WIDE_BAND_CHANNEL_T) pucIE;
                prBssInfo->ucVhtChannelWidth = prWideBandChannelIE->ucNewChannelWidth;
                prBssInfo->ucVhtChannelFrequencyS1 = prWideBandChannelIE->ucChannelS1;
                prBssInfo->ucVhtChannelFrequencyS2 = prWideBandChannelIE->ucChannelS2;
                fgHasWideBandIE = TRUE;
                break;
				
            case ELEM_ID_CH_SW_ANNOUNCEMENT:
                if (IE_LEN(pucIE) != (sizeof(IE_CHANNEL_SWITCH_T) - 2)) {
                    DBGLOG(RLM, INFO, ("[Mgt Action] ELEM_ID_CH_SW_ANNOUNCEMENT, Length\n"));
                    break;
                }

                prChannelSwitchAnnounceIE = (P_IE_CHANNEL_SWITCH_T) pucIE;
			
                if(prChannelSwitchAnnounceIE->ucChannelSwitchMode == 1){
                    DBGLOG(RLM, INFO, ("[Mgt Action] switch channel [%d]->[%d] \n", prBssInfo->ucPrimaryChannel, prChannelSwitchAnnounceIE->ucNewChannelNum));
                    prBssInfo->ucPrimaryChannel = prChannelSwitchAnnounceIE->ucNewChannelNum;
                }
                else {
                    DBGLOG(RLM, INFO, ("[Mgt Action] ucChannelSwitchMode = 0 \n"));
                }
				
                fgHasChannelSwitchIE = TRUE;
                break;
            case ELEM_ID_SCO :
                if (IE_LEN(pucIE) != (sizeof(IE_SECONDARY_OFFSET_T) - 2)) {
                    DBGLOG(RLM, INFO, ("[Mgt Action] ELEM_ID_SCO, Length\n"));
                    break;
                }
                prSecondaryOffsetIE = (P_IE_SECONDARY_OFFSET_T) pucIE;
                DBGLOG(RLM, INFO, ("[Mgt Action] SCO [%d]->[%d] \n", prBssInfo->eBssSCO, prSecondaryOffsetIE->ucSecondaryOffset));
                prBssInfo->eBssSCO = prSecondaryOffsetIE->ucSecondaryOffset;
                fgHasSCOIE = TRUE;
                break;
			default:
            	break;
			}
		}    
		if (fgHasChannelSwitchIE != FALSE) {
			if (fgHasWideBandIE == FALSE) {
                prBssInfo->ucVhtChannelWidth = 0;
                prBssInfo->ucVhtChannelFrequencyS1 =  prBssInfo->ucPrimaryChannel;
                prBssInfo->ucVhtChannelFrequencyS2 = 0 ;
            }
            if (fgHasSCOIE == FALSE) {
                prBssInfo->eBssSCO = CHNL_EXT_SCN;
            }
        }
        nicUpdateBss(prAdapter, prBssInfo->ucBssIndex);
     }

    return;
}

VOID
rlmSendOpModeNotificationFrame(
    P_ADAPTER_T     prAdapter,
    P_STA_RECORD_T  prStaRec,
    UINT_8          ucChannelWidth,
    UINT_8          ucNss
    )
{

    P_MSDU_INFO_T                       prMsduInfo;
    P_ACTION_OP_MODE_NOTIFICATION_FRAME prTxFrame;
    P_BSS_INFO_T                        prBssInfo;
    UINT_16                             u2EstimatedFrameLen;

    
    if (!prStaRec) {
        return;
    }

    prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);
    if(!prBssInfo) {
        return;
    }

    
    u2EstimatedFrameLen = MAC_TX_RESERVED_FIELD + sizeof(ACTION_OP_MODE_NOTIFICATION_FRAME);

    
    prMsduInfo = (P_MSDU_INFO_T) cnmMgtPktAlloc(prAdapter, u2EstimatedFrameLen);

    if (!prMsduInfo)
        return;

    kalMemZero(prMsduInfo->prPacket, u2EstimatedFrameLen);

    prTxFrame = prMsduInfo->prPacket;

    
    prTxFrame->u2FrameCtrl = MAC_FRAME_ACTION;
    
    COPY_MAC_ADDR(prTxFrame->aucDestAddr, prStaRec->aucMacAddr);
    COPY_MAC_ADDR(prTxFrame->aucSrcAddr, prBssInfo->aucOwnMacAddr);
    COPY_MAC_ADDR(prTxFrame->aucBSSID, prBssInfo->aucBSSID);

    
    prTxFrame->ucCategory = CATEGORY_VHT_ACTION;
    prTxFrame->ucAction = ACTION_OPERATING_MODE_NOTIFICATION;
    
    prTxFrame->ucOperatingMode |= (ucChannelWidth & VHT_OP_MODE_CHANNEL_WIDTH);

    if(ucNss == 0) {
        ucNss = 1;
    }
    prTxFrame->ucOperatingMode |= (((ucNss - 1) << 4) & VHT_OP_MODE_RX_NSS);
    prTxFrame->ucOperatingMode &= ~VHT_OP_MODE_RX_NSS_TYPE;

    
    TX_SET_MMPDU(
        prAdapter,
        prMsduInfo,
        prBssInfo->ucBssIndex,
        prStaRec->ucIndex,
        WLAN_MAC_MGMT_HEADER_LEN,
        sizeof(ACTION_OP_MODE_NOTIFICATION_FRAME),
        NULL,
        MSDU_RATE_MODE_AUTO
        ); 

    
    nicTxEnqueueMsdu(prAdapter, prMsduInfo);
	
}


#endif

