





#include "precomp.h"








VOID
cnmInit (
    P_ADAPTER_T     prAdapter
    )
{
    return;
} 

VOID
cnmUninit (
    P_ADAPTER_T     prAdapter
    )
{
    return;
} 

VOID
cnmChMngrRequestPrivilege (
    P_ADAPTER_T     prAdapter,
    P_MSG_HDR_T     prMsgHdr
    )
{
    P_MSG_CH_REQ_T          prMsgChReq;
    P_CMD_CH_PRIVILEGE_T    prCmdBody;
    WLAN_STATUS             rStatus;

    ASSERT(prAdapter);
    ASSERT(prMsgHdr);

    prMsgChReq = (P_MSG_CH_REQ_T) prMsgHdr;

    prCmdBody = (P_CMD_CH_PRIVILEGE_T)
            cnmMemAlloc(prAdapter, RAM_TYPE_BUF, sizeof(CMD_CH_PRIVILEGE_T));
    ASSERT(prCmdBody);

    
    if (!prCmdBody) {
        DBGLOG(CNM, ERROR, ("ChReq: fail to get buf (net=%d, token=%d)\n",
            prMsgChReq->ucNetTypeIndex, prMsgChReq->ucTokenID));

        cnmMemFree(prAdapter, prMsgHdr);
        return;
    }

    DBGLOG(CNM, INFO, ("ChReq net=%d token=%d b=%d c=%d s=%d\n",
        prMsgChReq->ucNetTypeIndex, prMsgChReq->ucTokenID,
        prMsgChReq->eRfBand, prMsgChReq->ucPrimaryChannel,
        prMsgChReq->eRfSco));

    prCmdBody->ucNetTypeIndex = prMsgChReq->ucNetTypeIndex;
    prCmdBody->ucTokenID = prMsgChReq->ucTokenID;
    prCmdBody->ucAction = CMD_CH_ACTION_REQ;    
    prCmdBody->ucPrimaryChannel = prMsgChReq->ucPrimaryChannel;
    prCmdBody->ucRfSco = (UINT_8) prMsgChReq->eRfSco;
    prCmdBody->ucRfBand = (UINT_8) prMsgChReq->eRfBand;
    prCmdBody->ucReqType = (UINT_8) prMsgChReq->eReqType;
    prCmdBody->ucReserved = 0;
    prCmdBody->u4MaxInterval= prMsgChReq->u4MaxInterval;
    COPY_MAC_ADDR(prCmdBody->aucBSSID, prMsgChReq->aucBSSID);

    ASSERT(prCmdBody->ucNetTypeIndex < NETWORK_TYPE_INDEX_NUM);

    
    if (prCmdBody->ucNetTypeIndex >= NETWORK_TYPE_INDEX_NUM) {
        DBGLOG(CNM, ERROR, ("CNM: ChReq with wrong netIdx=%d\n\n",
            prCmdBody->ucNetTypeIndex));
    }

    rStatus = wlanSendSetQueryCmd (
                prAdapter,                  
                CMD_ID_CH_PRIVILEGE,        
                TRUE,                       
                FALSE,                      
                FALSE,                      
                NULL,                       
                NULL,                       
                sizeof(CMD_CH_PRIVILEGE_T), 
                (PUINT_8) prCmdBody,        
                NULL,                       
                0                           
                );

    ASSERT(rStatus == WLAN_STATUS_PENDING);

    cnmMemFree(prAdapter, prCmdBody);
    cnmMemFree(prAdapter, prMsgHdr);

    return;
} 

VOID
cnmChMngrAbortPrivilege (
    P_ADAPTER_T prAdapter,
    P_MSG_HDR_T prMsgHdr
    )
{
    P_MSG_CH_ABORT_T        prMsgChAbort;
    P_CMD_CH_PRIVILEGE_T    prCmdBody;
    WLAN_STATUS             rStatus;

    ASSERT(prAdapter);
    ASSERT(prMsgHdr);

    prMsgChAbort = (P_MSG_CH_ABORT_T) prMsgHdr;

    prCmdBody = (P_CMD_CH_PRIVILEGE_T)
            cnmMemAlloc(prAdapter, RAM_TYPE_BUF, sizeof(CMD_CH_PRIVILEGE_T));
    ASSERT(prCmdBody);

    
    if (!prCmdBody) {
        DBGLOG(CNM, ERROR, ("ChAbort: fail to get buf (net=%d, token=%d)\n",
            prMsgChAbort->ucNetTypeIndex, prMsgChAbort->ucTokenID));

        cnmMemFree(prAdapter, prMsgHdr);
        return;
    }

    DBGLOG(CNM, INFO, ("ChAbort net=%d token=%d\n",
        prMsgChAbort->ucNetTypeIndex, prMsgChAbort->ucTokenID));

    prCmdBody->ucNetTypeIndex = prMsgChAbort->ucNetTypeIndex;
    prCmdBody->ucTokenID = prMsgChAbort->ucTokenID;
    prCmdBody->ucAction = CMD_CH_ACTION_ABORT;  

    ASSERT(prCmdBody->ucNetTypeIndex < NETWORK_TYPE_INDEX_NUM);

    
    if (prCmdBody->ucNetTypeIndex >= NETWORK_TYPE_INDEX_NUM) {
        DBGLOG(CNM, ERROR, ("CNM: ChAbort with wrong netIdx=%d\n\n",
            prCmdBody->ucNetTypeIndex));
    }

    rStatus = wlanSendSetQueryCmd (
                prAdapter,                  
                CMD_ID_CH_PRIVILEGE,        
                TRUE,                       
                FALSE,                      
                FALSE,                      
                NULL,                       
                NULL,                       
                sizeof(CMD_CH_PRIVILEGE_T), 
                (PUINT_8) prCmdBody,        
                NULL,                       
                0                           
                );

    ASSERT(rStatus == WLAN_STATUS_PENDING);

    cnmMemFree(prAdapter, prCmdBody);
    cnmMemFree(prAdapter, prMsgHdr);

    return;
} 

VOID
cnmChMngrHandleChEvent (
    P_ADAPTER_T     prAdapter,
    P_WIFI_EVENT_T  prEvent
    )
{
    P_EVENT_CH_PRIVILEGE_T  prEventBody;
    P_MSG_CH_GRANT_T        prChResp;

    ASSERT(prAdapter);
    ASSERT(prEvent);

    prEventBody = (P_EVENT_CH_PRIVILEGE_T) (prEvent->aucBuffer);
    prChResp = (P_MSG_CH_GRANT_T)
                cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(MSG_CH_GRANT_T));
    ASSERT(prChResp);

    
    if (!prChResp) {
        DBGLOG(CNM, ERROR, ("ChGrant: fail to get buf (net=%d, token=%d)\n",
            prEventBody->ucNetTypeIndex, prEventBody->ucTokenID));

        return;
    }

    DBGLOG(CNM, INFO, ("ChGrant net=%d token=%d ch=%d sco=%d\n",
        prEventBody->ucNetTypeIndex, prEventBody->ucTokenID,
        prEventBody->ucPrimaryChannel, prEventBody->ucRfSco));

    ASSERT(prEventBody->ucNetTypeIndex < NETWORK_TYPE_INDEX_NUM);
    ASSERT(prEventBody->ucStatus == EVENT_CH_STATUS_GRANT);

    
    if (prEventBody->ucNetTypeIndex == NETWORK_TYPE_AIS_INDEX) {
        prChResp->rMsgHdr.eMsgId = MID_CNM_AIS_CH_GRANT;
    }
#if CFG_ENABLE_WIFI_DIRECT
    else if ((prAdapter->fgIsP2PRegistered) &&
             (prEventBody->ucNetTypeIndex == NETWORK_TYPE_P2P_INDEX)) {
        prChResp->rMsgHdr.eMsgId = MID_CNM_P2P_CH_GRANT;
    }
#endif
#if CFG_ENABLE_BT_OVER_WIFI
    else if (prEventBody->ucNetTypeIndex == NETWORK_TYPE_BOW_INDEX) {
        prChResp->rMsgHdr.eMsgId = MID_CNM_BOW_CH_GRANT;
    }
#endif
    else {
        cnmMemFree(prAdapter, prChResp);
        return;
    }

    prChResp->ucNetTypeIndex = prEventBody->ucNetTypeIndex;
    prChResp->ucTokenID = prEventBody->ucTokenID;
    prChResp->ucPrimaryChannel = prEventBody->ucPrimaryChannel;
    prChResp->eRfSco = (ENUM_CHNL_EXT_T) prEventBody->ucRfSco;
    prChResp->eRfBand = (ENUM_BAND_T) prEventBody->ucRfBand;
    prChResp->eReqType = (ENUM_CH_REQ_TYPE_T) prEventBody->ucReqType;
    prChResp->u4GrantInterval = prEventBody->u4GrantInterval;

    mboxSendMsg(prAdapter,
                MBOX_ID_0,
                (P_MSG_HDR_T) prChResp,
                MSG_SEND_METHOD_BUF);
}

BOOLEAN
cnmPreferredChannel (
    P_ADAPTER_T         prAdapter,
    P_ENUM_BAND_T       prBand,
    PUINT_8             pucPrimaryChannel,
    P_ENUM_CHNL_EXT_T   prBssSCO
    )
{
    P_BSS_INFO_T    prBssInfo;

    ASSERT(prAdapter);
    ASSERT(prBand);
    ASSERT(pucPrimaryChannel);
    ASSERT(prBssSCO);

    prBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX];

    if (RLM_NET_PARAM_VALID(prBssInfo)) {
        *prBand = prBssInfo->eBand;
        *pucPrimaryChannel = prBssInfo->ucPrimaryChannel;
        *prBssSCO = prBssInfo->eBssSCO;

        return TRUE;
    }

    return FALSE;
}

BOOLEAN
cnmAisInfraChannelFixed (
    P_ADAPTER_T         prAdapter,
    P_ENUM_BAND_T       prBand,
    PUINT_8             pucPrimaryChannel
    )
{
#if CFG_ENABLE_WIFI_DIRECT ||(CFG_ENABLE_BT_OVER_WIFI && CFG_BOW_LIMIT_AIS_CHNL)
    P_BSS_INFO_T    prBssInfo;
#endif

#if CFG_ENABLE_WIFI_DIRECT
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_P2P_INDEX) &&
		p2pFuncIsAPMode(prAdapter->rWifiVar.prP2pFsmInfo)) {

        ASSERT(prAdapter->fgIsP2PRegistered);

        prBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_P2P_INDEX];

        *prBand = prBssInfo->eBand;
        *pucPrimaryChannel = prBssInfo->ucPrimaryChannel;

        return TRUE;
    }
#endif

#if CFG_ENABLE_BT_OVER_WIFI && CFG_BOW_LIMIT_AIS_CHNL
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_BOW_INDEX)) {

        prBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_BOW_INDEX];

        *prBand = prBssInfo->eBand;
        *pucPrimaryChannel = prBssInfo->ucPrimaryChannel;

        return TRUE;
    }
#endif

    return FALSE;
}
#if CFG_SUPPORT_CHNL_CONFLICT_REVISE
BOOLEAN
cnmAisDetectP2PChannel (
    P_ADAPTER_T         prAdapter,
    P_ENUM_BAND_T       prBand,
    PUINT_8             pucPrimaryChannel
    )
{
	P_WIFI_VAR_T prWifiVar = &prAdapter->rWifiVar;
	P_BSS_INFO_T prP2PBssInfo = &prWifiVar->arBssInfo[NETWORK_TYPE_P2P_INDEX];
#if CFG_ENABLE_WIFI_DIRECT
	if (prP2PBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED ||
		(prP2PBssInfo->eCurrentOPMode == OP_MODE_ACCESS_POINT && 
		 prP2PBssInfo->eIntendOPMode == OP_MODE_NUM)) {
		*prBand = prP2PBssInfo->eBand;
        *pucPrimaryChannel = prP2PBssInfo->ucPrimaryChannel;
        return TRUE;
	}
#endif
	return FALSE;
}
#endif
VOID
cnmAisInfraConnectNotify (
    P_ADAPTER_T         prAdapter
    )
{
#if CFG_ENABLE_BT_OVER_WIFI
    P_BSS_INFO_T    prAisBssInfo, prBowBssInfo;

    prAisBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX];
    prBowBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_BOW_INDEX];

    if (RLM_NET_PARAM_VALID(prAisBssInfo) && RLM_NET_PARAM_VALID(prBowBssInfo)){
        if (prAisBssInfo->eBand != prBowBssInfo->eBand ||
            prAisBssInfo->ucPrimaryChannel != prBowBssInfo->ucPrimaryChannel) {

            
            bowNotifyAllLinkDisconnected(prAdapter);
        }
    }
#endif
}

BOOLEAN
cnmAisIbssIsPermitted (
    P_ADAPTER_T     prAdapter
    )
{
#if CFG_ENABLE_WIFI_DIRECT
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_P2P_INDEX)) {
        return FALSE;
    }
#endif

#if CFG_ENABLE_BT_OVER_WIFI
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_BOW_INDEX)) {
        return FALSE;
    }
#endif

    return TRUE;
}

BOOLEAN
cnmP2PIsPermitted (
    P_ADAPTER_T     prAdapter
    )
{
    P_BSS_INFO_T    prBssInfo;

    prBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX];

    if (IS_BSS_ACTIVE(prBssInfo) &&
        prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {
        return FALSE;
    }

#if CFG_ENABLE_BT_OVER_WIFI
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_BOW_INDEX)) {
        
        bowNotifyAllLinkDisconnected(prAdapter);
    }
#endif

    return TRUE;
}

BOOLEAN
cnmBowIsPermitted (
    P_ADAPTER_T     prAdapter
    )
{
    P_BSS_INFO_T    prBssInfo;

    prBssInfo = &prAdapter->rWifiVar.arBssInfo[NETWORK_TYPE_AIS_INDEX];

    if (IS_BSS_ACTIVE(prBssInfo) &&
        prBssInfo->eCurrentOPMode == OP_MODE_IBSS) {
        return FALSE;
    }

#if CFG_ENABLE_WIFI_DIRECT
    if (IS_NET_ACTIVE(prAdapter, NETWORK_TYPE_P2P_INDEX)) {
        return FALSE;
    }
#endif

    return TRUE;
}

BOOLEAN
cnmBss40mBwPermitted (
    P_ADAPTER_T                 prAdapter,
    ENUM_NETWORK_TYPE_INDEX_T   eNetTypeIdx
    )
{
    P_BSS_INFO_T    prBssInfo;
    UINT_8          i;

    for (i = 0; i < NETWORK_TYPE_INDEX_NUM; i++) {
        if (i != (UINT_8) eNetTypeIdx) {
            prBssInfo = &prAdapter->rWifiVar.arBssInfo[i];

            if (IS_BSS_ACTIVE(prBssInfo) && (prBssInfo->fg40mBwAllowed ||
                prBssInfo->fgAssoc40mBwAllowed)) {
                return FALSE;
            }
        }
    }

    return TRUE;
}


