





#include "precomp.h"

#define REPLICATED_BEACON_TIME_THRESHOLD        (3000)
#define REPLICATED_BEACON_FRESH_PERIOD          (10000)
#define REPLICATED_BEACON_STRENGTH_THRESHOLD    (32)

#define ROAMING_NO_SWING_RCPI_STEP              (20)






VOID scnInit(IN P_ADAPTER_T prAdapter)
{
	P_SCAN_INFO_T prScanInfo;
	P_BSS_DESC_T prBSSDesc;
	PUINT_8 pucBSSBuff;
	UINT_32 i;


	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	pucBSSBuff = &prScanInfo->aucScanBuffer[0];


	DBGLOG(SCN, INFO, ("->scnInit()\n"));

	
	prScanInfo->eCurrentState = SCAN_STATE_IDLE;

	prScanInfo->rLastScanCompletedTime = (OS_SYSTIME) 0;

	LINK_INITIALIZE(&prScanInfo->rPendingMsgList);


	
	kalMemZero((PVOID) pucBSSBuff, SCN_MAX_BUFFER_SIZE);

	LINK_INITIALIZE(&prScanInfo->rFreeBSSDescList);
	LINK_INITIALIZE(&prScanInfo->rBSSDescList);

	for (i = 0; i < CFG_MAX_NUM_BSS_LIST; i++) {

		prBSSDesc = (P_BSS_DESC_T) pucBSSBuff;

		LINK_INSERT_TAIL(&prScanInfo->rFreeBSSDescList, &prBSSDesc->rLinkEntry);

		pucBSSBuff += ALIGN_4(sizeof(BSS_DESC_T));
	}
	
	ASSERT(((ULONG) pucBSSBuff - (ULONG) &prScanInfo->aucScanBuffer[0]) ==
	       SCN_MAX_BUFFER_SIZE);

	
	prScanInfo->fgIsSparseChannelValid = FALSE;

	
	prScanInfo->fgNloScanning = FALSE;

	return;
}				


VOID scnFreeAllPendingScanRquests (IN P_ADAPTER_T prAdapter)
{
    P_SCAN_INFO_T prScanInfo;
    P_MSG_HDR_T prMsgHdr;
    P_MSG_SCN_SCAN_REQ prScanReqMsg;
    P_MSG_SCN_SCAN_REQ_V2 prScanReqV2Msg;


    prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

    
    while (!LINK_IS_EMPTY(&(prScanInfo->rPendingMsgList))) {
        
    
    LINK_REMOVE_HEAD(&(prScanInfo->rPendingMsgList), prMsgHdr,
                 P_MSG_HDR_T);
    if(prMsgHdr) {
        prScanReqMsg =  (P_MSG_SCN_SCAN_REQ)prMsgHdr;   
            DBGLOG(SCN, INFO, ("free scan request eMsgId[%d] ucSeqNum [%d] BSSID[%d]!! \n",prMsgHdr->eMsgId, prScanReqMsg->ucSeqNum, prScanReqMsg->ucBssIndex));
            cnmMemFree(prAdapter, prMsgHdr);
    }
    else {
        
        ASSERT(0);
		}
                
    }
}


VOID scnUninit(IN P_ADAPTER_T prAdapter)
{
	P_SCAN_INFO_T prScanInfo;


	ASSERT(prAdapter);
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	DBGLOG(SCN, INFO, ("->scnUninit()\n"));

        scnFreeAllPendingScanRquests(prAdapter);

	DBGLOG(SCN, INFO, ("scnFreeAllPendingScanrRquests !! \n"));

	
	prScanInfo->eCurrentState = SCAN_STATE_IDLE;

	prScanInfo->rLastScanCompletedTime = (OS_SYSTIME) 0;

	

	
	LINK_INITIALIZE(&prScanInfo->rFreeBSSDescList);
	LINK_INITIALIZE(&prScanInfo->rBSSDescList);

	return;
}				



P_BSS_DESC_T scanSearchBssDescByBssid(IN P_ADAPTER_T prAdapter, IN UINT_8 aucBSSID[]
    )
{
	return scanSearchBssDescByBssidAndSsid(prAdapter, aucBSSID, FALSE, NULL);
}


P_BSS_DESC_T
scanSearchBssDescByBssidAndSsid(IN P_ADAPTER_T prAdapter,
				IN UINT_8 aucBSSID[],
				IN BOOLEAN fgCheckSsid, IN P_PARAM_SSID_T prSsid)
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_BSS_DESC_T prBssDesc;
	P_BSS_DESC_T prDstBssDesc = (P_BSS_DESC_T) NULL;


	ASSERT(prAdapter);
	ASSERT(aucBSSID);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prBSSDescList = &prScanInfo->rBSSDescList;

	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID, aucBSSID)) {
			if (fgCheckSsid == FALSE || prSsid == NULL) {
				return prBssDesc;
			} else {
				if (EQUAL_SSID(prBssDesc->aucSSID,
					       prBssDesc->ucSSIDLen,
					       prSsid->aucSsid, prSsid->u4SsidLen)) {
					return prBssDesc;
				} else if (prDstBssDesc == NULL
					   && prBssDesc->fgIsHiddenSSID == TRUE) {
					prDstBssDesc = prBssDesc;
				} else if (prBssDesc->eBSSType == BSS_TYPE_P2P_DEVICE) {
					
					COPY_SSID(prBssDesc->aucSSID,
						  prBssDesc->ucSSIDLen,
						  prSsid->aucSsid, (UINT_8) (prSsid->u4SsidLen));
					return prBssDesc;
				}
			}
		}
	}

	return prDstBssDesc;

}				


P_BSS_DESC_T scanSearchBssDescByTA(IN P_ADAPTER_T prAdapter, IN UINT_8 aucSrcAddr[]
    )
{
	return scanSearchBssDescByTAAndSsid(prAdapter, aucSrcAddr, FALSE, NULL);
}

P_BSS_DESC_T
scanSearchBssDescByTAAndSsid(IN P_ADAPTER_T prAdapter,
			     IN UINT_8 aucSrcAddr[],
			     IN BOOLEAN fgCheckSsid, IN P_PARAM_SSID_T prSsid)
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_BSS_DESC_T prBssDesc;
	P_BSS_DESC_T prDstBssDesc = (P_BSS_DESC_T) NULL;


	ASSERT(prAdapter);
	ASSERT(aucSrcAddr);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prBSSDescList = &prScanInfo->rBSSDescList;

	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		if (EQUAL_MAC_ADDR(prBssDesc->aucSrcAddr, aucSrcAddr)) {
			if (fgCheckSsid == FALSE || prSsid == NULL) {
				return prBssDesc;
			} else {
				if (EQUAL_SSID(prBssDesc->aucSSID,
					       prBssDesc->ucSSIDLen,
					       prSsid->aucSsid, prSsid->u4SsidLen)) {
					return prBssDesc;
				} else if (prDstBssDesc == NULL
					   && prBssDesc->fgIsHiddenSSID == TRUE) {
					prDstBssDesc = prBssDesc;
				}
			}
		}
	}

	return prDstBssDesc;

}				


P_BSS_DESC_T
scanSearchExistingBssDesc(IN P_ADAPTER_T prAdapter,
			  IN ENUM_BSS_TYPE_T eBSSType, IN UINT_8 aucBSSID[], IN UINT_8 aucSrcAddr[]
    )
{
	return scanSearchExistingBssDescWithSsid(prAdapter,
						 eBSSType, aucBSSID, aucSrcAddr, FALSE, NULL);
}


P_BSS_DESC_T
scanSearchExistingBssDescWithSsid(IN P_ADAPTER_T prAdapter,
				  IN ENUM_BSS_TYPE_T eBSSType,
				  IN UINT_8 aucBSSID[],
				  IN UINT_8 aucSrcAddr[],
				  IN BOOLEAN fgCheckSsid, IN P_PARAM_SSID_T prSsid)
{
	P_SCAN_INFO_T prScanInfo;
	P_BSS_DESC_T prBssDesc, prIBSSBssDesc;

	ASSERT(prAdapter);
	ASSERT(aucSrcAddr);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);


	switch (eBSSType) {
	case BSS_TYPE_P2P_DEVICE:
		fgCheckSsid = FALSE;
	case BSS_TYPE_INFRASTRUCTURE:
	case BSS_TYPE_BOW_DEVICE:
		{
			prBssDesc =
			    scanSearchBssDescByBssidAndSsid(prAdapter, aucBSSID, fgCheckSsid,
							    prSsid);

			

			return prBssDesc;
		}

	case BSS_TYPE_IBSS:
		{
			prIBSSBssDesc =
			    scanSearchBssDescByBssidAndSsid(prAdapter, aucBSSID, fgCheckSsid,
							    prSsid);
			prBssDesc =
			    scanSearchBssDescByTAAndSsid(prAdapter, aucSrcAddr, fgCheckSsid,
							 prSsid);

			if (prBssDesc) {
				if ((!prIBSSBssDesc) ||	
				    (prBssDesc == prIBSSBssDesc)) {	

					return prBssDesc;
				} else {	
					P_LINK_T prBSSDescList;
					P_LINK_T prFreeBSSDescList;


					prBSSDescList = &prScanInfo->rBSSDescList;
					prFreeBSSDescList = &prScanInfo->rFreeBSSDescList;

					
					LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDesc);

					
					LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDesc->rLinkEntry);

					return prIBSSBssDesc;
				}
			}

			if (prIBSSBssDesc) {	

				return prIBSSBssDesc;
			}
			
			break;	
		}

	default:
		break;
	}


	return (P_BSS_DESC_T) NULL;

}				


VOID scanRemoveBssDescsByPolicy(IN P_ADAPTER_T prAdapter, IN UINT_32 u4RemovePolicy)
{
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_LINK_T prFreeBSSDescList;
	P_BSS_DESC_T prBssDesc;


	ASSERT(prAdapter);

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prBSSDescList = &prScanInfo->rBSSDescList;
	prFreeBSSDescList = &prScanInfo->rFreeBSSDescList;

	
	

	if (u4RemovePolicy & SCN_RM_POLICY_TIMEOUT) {
		P_BSS_DESC_T prBSSDescNext;
		OS_SYSTIME rCurrentTime;


		GET_CURRENT_SYSTIME(&rCurrentTime);

		
		LINK_FOR_EACH_ENTRY_SAFE(prBssDesc, prBSSDescNext, prBSSDescList, rLinkEntry,
					 BSS_DESC_T) {

			if ((u4RemovePolicy & SCN_RM_POLICY_EXCLUDE_CONNECTED) &&
			    (prBssDesc->fgIsConnected || prBssDesc->fgIsConnecting)) {
				
				continue;
			}

			if (CHECK_FOR_TIMEOUT(rCurrentTime, prBssDesc->rUpdateTime,
					      SEC_TO_SYSTIME(SCN_BSS_DESC_REMOVE_TIMEOUT_SEC))) {

				
				

				
				LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDesc);

				
				LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDesc->rLinkEntry);
			}
		}
	} else if (u4RemovePolicy & SCN_RM_POLICY_OLDEST_HIDDEN) {
		P_BSS_DESC_T prBssDescOldest = (P_BSS_DESC_T) NULL;


		
		LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

			if ((u4RemovePolicy & SCN_RM_POLICY_EXCLUDE_CONNECTED) &&
			    (prBssDesc->fgIsConnected || prBssDesc->fgIsConnecting)) {
				
				continue;
			}

			if (!prBssDesc->fgIsHiddenSSID) {
				continue;
			}

			if (!prBssDescOldest) {	
				prBssDescOldest = prBssDesc;
				continue;
			}

			if (TIME_BEFORE(prBssDesc->rUpdateTime, prBssDescOldest->rUpdateTime)) {
				prBssDescOldest = prBssDesc;
			}
		}

		if (prBssDescOldest) {

			
			

			
			LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDescOldest);

			
			LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDescOldest->rLinkEntry);
		}
	} else if (u4RemovePolicy & SCN_RM_POLICY_SMART_WEAKEST) {
		P_BSS_DESC_T prBssDescWeakest = (P_BSS_DESC_T) NULL;
		P_BSS_DESC_T prBssDescWeakestSameSSID = (P_BSS_DESC_T) NULL;
		UINT_32 u4SameSSIDCount = 0;


		
		LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

			if ((u4RemovePolicy & SCN_RM_POLICY_EXCLUDE_CONNECTED) &&
			    (prBssDesc->fgIsConnected || prBssDesc->fgIsConnecting)) {
				
				continue;
			}

			if ((!prBssDesc->fgIsHiddenSSID) &&
			    (EQUAL_SSID(prBssDesc->aucSSID,
					prBssDesc->ucSSIDLen,
					prConnSettings->aucSSID, prConnSettings->ucSSIDLen))) {

				u4SameSSIDCount++;

				if (!prBssDescWeakestSameSSID) {
					prBssDescWeakestSameSSID = prBssDesc;
				} else if (prBssDesc->ucRCPI < prBssDescWeakestSameSSID->ucRCPI) {
					prBssDescWeakestSameSSID = prBssDesc;
				}
			}

			if (!prBssDescWeakest) {	
				prBssDescWeakest = prBssDesc;
				continue;
			}

			if (prBssDesc->ucRCPI < prBssDescWeakest->ucRCPI) {
				prBssDescWeakest = prBssDesc;
			}

		}

		if ((u4SameSSIDCount >= SCN_BSS_DESC_SAME_SSID_THRESHOLD) &&
		    (prBssDescWeakestSameSSID)) {
			prBssDescWeakest = prBssDescWeakestSameSSID;
		}

		if (prBssDescWeakest) {

			
			

			
			LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDescWeakest);

			
			LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDescWeakest->rLinkEntry);
		}
	} else if (u4RemovePolicy & SCN_RM_POLICY_ENTIRE) {
		P_BSS_DESC_T prBSSDescNext;

		LINK_FOR_EACH_ENTRY_SAFE(prBssDesc, prBSSDescNext, prBSSDescList, rLinkEntry,
					 BSS_DESC_T) {

			if ((u4RemovePolicy & SCN_RM_POLICY_EXCLUDE_CONNECTED) &&
			    (prBssDesc->fgIsConnected || prBssDesc->fgIsConnecting)) {
				
				continue;
			}

			
			LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDesc);

			
			LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDesc->rLinkEntry);
		}

	}

	return;

}				


VOID scanRemoveBssDescByBssid(IN P_ADAPTER_T prAdapter, IN UINT_8 aucBSSID[]
    )
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_LINK_T prFreeBSSDescList;
	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;
	P_BSS_DESC_T prBSSDescNext;


	ASSERT(prAdapter);
	ASSERT(aucBSSID);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prBSSDescList = &prScanInfo->rBSSDescList;
	prFreeBSSDescList = &prScanInfo->rFreeBSSDescList;

	
	LINK_FOR_EACH_ENTRY_SAFE(prBssDesc, prBSSDescNext, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID, aucBSSID)) {

			
			LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDesc);

			
			LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDesc->rLinkEntry);

			
		}
	}

	return;
}				


VOID
scanRemoveBssDescByBandAndNetwork(IN P_ADAPTER_T prAdapter,
				  IN ENUM_BAND_T eBand, IN UINT_8 ucBssIndex)
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_LINK_T prFreeBSSDescList;
	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;
	P_BSS_DESC_T prBSSDescNext;
	BOOLEAN fgToRemove;

	ASSERT(prAdapter);
	ASSERT(eBand <= BAND_NUM);
	ASSERT(ucBssIndex <= MAX_BSS_INDEX);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prBSSDescList = &prScanInfo->rBSSDescList;
	prFreeBSSDescList = &prScanInfo->rFreeBSSDescList;


	if (eBand == BAND_NULL) {
		return;		
	}

	
	LINK_FOR_EACH_ENTRY_SAFE(prBssDesc, prBSSDescNext, prBSSDescList, rLinkEntry, BSS_DESC_T) {
		fgToRemove = FALSE;

		if (prBssDesc->eBand == eBand) {
			switch (GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex)->eNetworkType) {
			case NETWORK_TYPE_AIS:
				if ((prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE)
				    || (prBssDesc->eBSSType == BSS_TYPE_IBSS)) {
					fgToRemove = TRUE;
				}
				break;

			case NETWORK_TYPE_P2P:
				if (prBssDesc->eBSSType == BSS_TYPE_P2P_DEVICE) {
					fgToRemove = TRUE;
				}
				break;

			case NETWORK_TYPE_BOW:
				if (prBssDesc->eBSSType == BSS_TYPE_BOW_DEVICE) {
					fgToRemove = TRUE;
				}
				break;

			default:
				ASSERT(0);
				break;
			}
		}

		if (fgToRemove == TRUE) {
			
			LINK_REMOVE_KNOWN_ENTRY(prBSSDescList, prBssDesc);

			
			LINK_INSERT_TAIL(prFreeBSSDescList, &prBssDesc->rLinkEntry);
		}
	}

	return;
}				


VOID scanRemoveConnFlagOfBssDescByBssid(IN P_ADAPTER_T prAdapter, IN UINT_8 aucBSSID[]
    )
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;


	ASSERT(prAdapter);
	ASSERT(aucBSSID);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prBSSDescList = &prScanInfo->rBSSDescList;

	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID, aucBSSID)) {
			prBssDesc->fgIsConnected = FALSE;
			prBssDesc->fgIsConnecting = FALSE;

			
		}
	}

	return;

}				


P_BSS_DESC_T scanAllocateBssDesc(IN P_ADAPTER_T prAdapter)
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prFreeBSSDescList;
	P_BSS_DESC_T prBssDesc;


	ASSERT(prAdapter);
	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prFreeBSSDescList = &prScanInfo->rFreeBSSDescList;

	LINK_REMOVE_HEAD(prFreeBSSDescList, prBssDesc, P_BSS_DESC_T);

	if (prBssDesc) {
		P_LINK_T prBSSDescList;

		kalMemZero(prBssDesc, sizeof(BSS_DESC_T));

#if CFG_ENABLE_WIFI_DIRECT
		LINK_INITIALIZE(&(prBssDesc->rP2pDeviceList));
		prBssDesc->fgIsP2PPresent = FALSE;
#endif				

		prBSSDescList = &prScanInfo->rBSSDescList;

		LINK_INSERT_TAIL(prBSSDescList, &prBssDesc->rLinkEntry);
	}

	return prBssDesc;

}				


P_BSS_DESC_T scanAddToBssDesc(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	P_BSS_DESC_T prBssDesc = NULL;
	UINT_16 u2CapInfo;
	ENUM_BSS_TYPE_T eBSSType = BSS_TYPE_INFRASTRUCTURE;

	PUINT_8 pucIE;
	UINT_16 u2IELength;
	UINT_16 u2Offset = 0;

	P_WLAN_BEACON_FRAME_T prWlanBeaconFrame = (P_WLAN_BEACON_FRAME_T) NULL;
	P_IE_SSID_T prIeSsid = (P_IE_SSID_T) NULL;
	P_IE_SUPPORTED_RATE_T prIeSupportedRate = (P_IE_SUPPORTED_RATE_T) NULL;
	P_IE_EXT_SUPPORTED_RATE_T prIeExtSupportedRate = (P_IE_EXT_SUPPORTED_RATE_T) NULL;
	UINT_8 ucHwChannelNum = 0;
	UINT_8 ucIeDsChannelNum = 0;
	UINT_8 ucIeHtChannelNum = 0;
	BOOLEAN fgIsValidSsid = FALSE, fgEscape = FALSE;
	PARAM_SSID_T rSsid;
	UINT_64 u8Timestamp;
	BOOLEAN fgIsNewBssDesc = FALSE;

	UINT_32 i;
	UINT_8 ucSSIDChar;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);
    if(!prSwRfb->prRxStatusGroup3){
        DBGLOG(SCN, INFO, ("prRxStatusGroup3 NULL \n"));
        DBGLOG(SCN, INFO, ("Dump Rx packet, u2PacketLen = %d\n", prSwRfb->u2PacketLen)); 
        DBGLOG_MEM8(SCN, INFO, prSwRfb->pvHeader, prSwRfb->u2PacketLen); 
        ASSERT(prSwRfb->prRxStatusGroup3);
        return NULL;
    }

	prWlanBeaconFrame = (P_WLAN_BEACON_FRAME_T) prSwRfb->pvHeader;

	WLAN_GET_FIELD_16(&prWlanBeaconFrame->u2CapInfo, &u2CapInfo);
	WLAN_GET_FIELD_64(&prWlanBeaconFrame->au4Timestamp[0], &u8Timestamp);

	
	switch (u2CapInfo & CAP_INFO_BSS_TYPE) {
	case CAP_INFO_ESS:
		
		eBSSType = BSS_TYPE_INFRASTRUCTURE;
		break;

	case CAP_INFO_IBSS:
		eBSSType = BSS_TYPE_IBSS;
		break;
	case 0:
		
		eBSSType = BSS_TYPE_P2P_DEVICE;
		break;

#if CFG_ENABLE_BT_OVER_WIFI
		
#endif

	default:
		return NULL;
	}

	
	pucIE = prWlanBeaconFrame->aucInfoElem;
	u2IELength = (prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) -
	    (UINT_16) OFFSET_OF(WLAN_BEACON_FRAME_BODY_T, aucInfoElem[0]);

	if (u2IELength > CFG_IE_BUFFER_SIZE) {
		u2IELength = CFG_IE_BUFFER_SIZE;
	}

	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {
		switch (IE_ID(pucIE)) {
		case ELEM_ID_SSID:
			if (IE_LEN(pucIE) <= ELEM_MAX_LEN_SSID) {
				ucSSIDChar = '\0';

				
				if (IE_LEN(pucIE) == 0) {
					fgIsValidSsid = FALSE;
				}
				
				
				else {
					for (i = 0; i < IE_LEN(pucIE); i++) {
						ucSSIDChar |= SSID_IE(pucIE)->aucSSID[i];
					}

					if (ucSSIDChar) {
						fgIsValidSsid = TRUE;
					}
				}

				
				if (fgIsValidSsid == TRUE) {
					COPY_SSID(rSsid.aucSsid,
						  rSsid.u4SsidLen,
						  SSID_IE(pucIE)->aucSSID,
						  SSID_IE(pucIE)->ucLength);
				}
			}
			fgEscape = TRUE;
			break;
		default:
			break;
		}

		if (fgEscape == TRUE) {
			break;
		}
	}


	
	prBssDesc = scanSearchExistingBssDescWithSsid(prAdapter,
						      eBSSType,
						      (PUINT_8) prWlanBeaconFrame->aucBSSID,
						      (PUINT_8) prWlanBeaconFrame->aucSrcAddr,
						      fgIsValidSsid,
						      fgIsValidSsid == TRUE ? &rSsid : NULL);

	if (prBssDesc == (P_BSS_DESC_T) NULL) {
		fgIsNewBssDesc = TRUE;

		do {
			
			prBssDesc = scanAllocateBssDesc(prAdapter);
			if (prBssDesc) {
				break;
			}
			
			scanRemoveBssDescsByPolicy(prAdapter,
						   (SCN_RM_POLICY_EXCLUDE_CONNECTED |
						    SCN_RM_POLICY_OLDEST_HIDDEN |
                    		SCN_RM_POLICY_TIMEOUT));

			
			prBssDesc = scanAllocateBssDesc(prAdapter);
			if (prBssDesc) {
				break;
			}
			
			scanRemoveBssDescsByPolicy(prAdapter,
						   (SCN_RM_POLICY_EXCLUDE_CONNECTED |
						    SCN_RM_POLICY_SMART_WEAKEST));

			
			prBssDesc = scanAllocateBssDesc(prAdapter);
			if (prBssDesc) {
				break;
			}
			
			
			return NULL;

		} while (FALSE);

	} else {
		OS_SYSTIME rCurrentTime;

		
		
		

		GET_CURRENT_SYSTIME(&rCurrentTime);

		ASSERT(prSwRfb->prRxStatusGroup3);

		if (prBssDesc->eBSSType != eBSSType) {
			prBssDesc->eBSSType = eBSSType;
		} else if (HAL_RX_STATUS_GET_CHNL_NUM(prSwRfb->prRxStatus) !=
			   prBssDesc->ucChannelNum
			   && prBssDesc->ucRCPI >
			   HAL_RX_STATUS_GET_RCPI(prSwRfb->prRxStatusGroup3)) {

			
			ASSERT(prSwRfb->prRxStatusGroup3);
			if ((prBssDesc->ucRCPI -
			     HAL_RX_STATUS_GET_RCPI(prSwRfb->prRxStatusGroup3)) >=
			    REPLICATED_BEACON_STRENGTH_THRESHOLD
			    && rCurrentTime - prBssDesc->rUpdateTime <=
			    REPLICATED_BEACON_FRESH_PERIOD) {
				return prBssDesc;
			}
			
			else if (rCurrentTime - prBssDesc->rUpdateTime <=
				 REPLICATED_BEACON_TIME_THRESHOLD) {
				return prBssDesc;
			}
		}

		
		if (prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE
		    && u8Timestamp < prBssDesc->u8TimeStamp.QuadPart) {
			BOOLEAN fgIsConnected, fgIsConnecting;

			
			fgIsNewBssDesc = TRUE;

			
			fgIsConnected = prBssDesc->fgIsConnected;
			fgIsConnecting = prBssDesc->fgIsConnecting;
			scanRemoveBssDescByBssid(prAdapter, prBssDesc->aucBSSID);

			prBssDesc = scanAllocateBssDesc(prAdapter);
			if (!prBssDesc) {
				return NULL;
			}

			
			prBssDesc->fgIsConnected = fgIsConnected;
			prBssDesc->fgIsConnecting = fgIsConnecting;
		}
	}
#if 1

	prBssDesc->u2RawLength = prSwRfb->u2PacketLen;
	kalMemCopy(prBssDesc->aucRawBuf, prWlanBeaconFrame, prBssDesc->u2RawLength);
#endif

	
	if (fgIsNewBssDesc == FALSE && prBssDesc->fgIsConnecting) {
		return prBssDesc;
	}
	
	prBssDesc->eBSSType = eBSSType;	

	COPY_MAC_ADDR(prBssDesc->aucSrcAddr, prWlanBeaconFrame->aucSrcAddr);

	COPY_MAC_ADDR(prBssDesc->aucBSSID, prWlanBeaconFrame->aucBSSID);

	prBssDesc->u8TimeStamp.QuadPart = u8Timestamp;

	WLAN_GET_FIELD_16(&prWlanBeaconFrame->u2BeaconInterval, &prBssDesc->u2BeaconInterval);

	prBssDesc->u2CapInfo = u2CapInfo;


	
	u2IELength = (prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) -
	    (UINT_16) OFFSET_OF(WLAN_BEACON_FRAME_BODY_T, aucInfoElem[0]);

	if (u2IELength > CFG_IE_BUFFER_SIZE) {
		u2IELength = CFG_IE_BUFFER_SIZE;
		prBssDesc->fgIsIEOverflow = TRUE;
	} else {
		prBssDesc->fgIsIEOverflow = FALSE;
	}
	prBssDesc->u2IELength = u2IELength;

	kalMemCopy(prBssDesc->aucIEBuf, prWlanBeaconFrame->aucInfoElem, u2IELength);

	
	prBssDesc->fgIsERPPresent = FALSE;
	prBssDesc->fgIsHTPresent = FALSE;
	prBssDesc->eSco = CHNL_EXT_SCN;
	prBssDesc->fgIEWAPI = FALSE;
	prBssDesc->fgIERSN = FALSE;
	prBssDesc->fgIEWPA = FALSE;
	prBssDesc->eChannelWidth = CW_20_40MHZ; 
	prBssDesc->ucCenterFreqS1 = 0;
	prBssDesc->ucCenterFreqS2 = 0;

	
	pucIE = prWlanBeaconFrame->aucInfoElem;


	IE_FOR_EACH(pucIE, u2IELength, u2Offset) {

		switch (IE_ID(pucIE)) {
		case ELEM_ID_SSID:
			if ((!prIeSsid) &&	
			    (IE_LEN(pucIE) <= ELEM_MAX_LEN_SSID)) {
				BOOLEAN fgIsHiddenSSID = FALSE;
				ucSSIDChar = '\0';


				prIeSsid = (P_IE_SSID_T) pucIE;

				
				if (IE_LEN(pucIE) == 0) {
					fgIsHiddenSSID = TRUE;
				}
				
				
				else {
					for (i = 0; i < IE_LEN(pucIE); i++) {
						ucSSIDChar |= SSID_IE(pucIE)->aucSSID[i];
					}

					if (!ucSSIDChar) {
						fgIsHiddenSSID = TRUE;
					}
				}

				
				if (!fgIsHiddenSSID) {
					COPY_SSID(prBssDesc->aucSSID,
						  prBssDesc->ucSSIDLen,
						  SSID_IE(pucIE)->aucSSID,
						  SSID_IE(pucIE)->ucLength);
				}

			}
			break;

		case ELEM_ID_SUP_RATES:
			
			if ((!prIeSupportedRate) && (IE_LEN(pucIE) <= RATE_NUM_SW)) {
				prIeSupportedRate = SUP_RATES_IE(pucIE);
			}
			break;

		case ELEM_ID_DS_PARAM_SET:
			if (IE_LEN(pucIE) == ELEM_MAX_LEN_DS_PARAMETER_SET) {
				ucIeDsChannelNum = DS_PARAM_IE(pucIE)->ucCurrChnl;
			}
			break;

		case ELEM_ID_TIM:
			if (IE_LEN(pucIE) <= ELEM_MAX_LEN_TIM) {
				prBssDesc->ucDTIMPeriod = TIM_IE(pucIE)->ucDTIMPeriod;
			}
			break;

		case ELEM_ID_IBSS_PARAM_SET:
			if (IE_LEN(pucIE) == ELEM_MAX_LEN_IBSS_PARAMETER_SET) {
				prBssDesc->u2ATIMWindow = IBSS_PARAM_IE(pucIE)->u2ATIMWindow;
			}
			break;

#if 0				
		case ELEM_ID_COUNTRY_INFO:
			prBssDesc->prIECountry = (P_IE_COUNTRY_T) pucIE;
			break;
#endif

		case ELEM_ID_ERP_INFO:
			if (IE_LEN(pucIE) == ELEM_MAX_LEN_ERP) {
				prBssDesc->fgIsERPPresent = TRUE;
			}
			break;

		case ELEM_ID_EXTENDED_SUP_RATES:
			if (!prIeExtSupportedRate) {
				prIeExtSupportedRate = EXT_SUP_RATES_IE(pucIE);
			}
			break;

		case ELEM_ID_RSN:
			if (rsnParseRsnIE(prAdapter, RSN_IE(pucIE), &prBssDesc->rRSNInfo)) {
				prBssDesc->fgIERSN = TRUE;
				prBssDesc->u2RsnCap = prBssDesc->rRSNInfo.u2RsnCap;
				if (prAdapter->rWifiVar.rConnSettings.eAuthMode == AUTH_MODE_WPA2) {
					rsnCheckPmkidCache(prAdapter, prBssDesc);
				}
			}
			break;

		case ELEM_ID_HT_CAP:
			prBssDesc->fgIsHTPresent = TRUE;
			break;

		case ELEM_ID_HT_OP:
			if (IE_LEN(pucIE) != (sizeof(IE_HT_OP_T) - 2)) {
				break;
			}

			if ((((P_IE_HT_OP_T) pucIE)->ucInfo1 & HT_OP_INFO1_SCO) != CHNL_EXT_RES) {
				prBssDesc->eSco = (ENUM_CHNL_EXT_T)
				    (((P_IE_HT_OP_T) pucIE)->ucInfo1 & HT_OP_INFO1_SCO);
			}
			ucIeHtChannelNum = ((P_IE_HT_OP_T) pucIE)->ucPrimaryChannel;

			break;
		case ELEM_ID_VHT_CAP:
			prBssDesc->fgIsVHTPresent = TRUE;
			break;

		case ELEM_ID_VHT_OP:
			if (IE_LEN(pucIE) != (sizeof(IE_VHT_OP_T) - 2)) {
				break;
			}

			prBssDesc->eChannelWidth =
			    (ENUM_CHANNEL_WIDTH_T) (((P_IE_VHT_OP_T) pucIE)->ucVhtOperation[0]);
			prBssDesc->ucCenterFreqS1 =
			    (ENUM_CHANNEL_WIDTH_T) (((P_IE_VHT_OP_T) pucIE)->ucVhtOperation[1]);
			prBssDesc->ucCenterFreqS2 =
			    (ENUM_CHANNEL_WIDTH_T) (((P_IE_VHT_OP_T) pucIE)->ucVhtOperation[2]);

			break;
#if CFG_SUPPORT_WAPI
		case ELEM_ID_WAPI:
			if (wapiParseWapiIE(WAPI_IE(pucIE), &prBssDesc->rIEWAPI)) {
				prBssDesc->fgIEWAPI = TRUE;
			}
			break;
#endif

		case ELEM_ID_VENDOR:	
			{
				UINT_8 ucOuiType;
				UINT_16 u2SubTypeVersion;
				if (rsnParseCheckForWFAInfoElem
				    (prAdapter, pucIE, &ucOuiType, &u2SubTypeVersion)) {
					if ((ucOuiType == VENDOR_OUI_TYPE_WPA)
						&& (u2SubTypeVersion == VERSION_WPA)
						&& (rsnParseWpaIE(prAdapter, WPA_IE(pucIE), &prBssDesc->rWPAInfo))) {
							prBssDesc->fgIEWPA = TRUE;
					}
				}
#if CFG_ENABLE_WIFI_DIRECT
				if (prAdapter->fgIsP2PRegistered) {
					if ((p2pFuncParseCheckForP2PInfoElem(prAdapter, pucIE, &ucOuiType))
						&& (ucOuiType == VENDOR_OUI_TYPE_P2P)) {
							prBssDesc->fgIsP2PPresent = TRUE;
					}
				}
#endif				
			}
			break;

			
		}
	}


	
	

	if (prBssDesc->ucSSIDLen == 0) {
		prBssDesc->fgIsHiddenSSID = TRUE;
	} else {
		prBssDesc->fgIsHiddenSSID = FALSE;
	}


	
	if (prIeSupportedRate || prIeExtSupportedRate) {
		rateGetRateSetFromIEs(prIeSupportedRate,
				      prIeExtSupportedRate,
				      &prBssDesc->u2OperationalRateSet,
				      &prBssDesc->u2BSSBasicRateSet,
				      &prBssDesc->fgIsUnknownBssBasicRate);
	}

	
	{
		P_HW_MAC_RX_DESC_T prRxStatus;
		UINT_8 ucRxRCPI;

		prRxStatus = prSwRfb->prRxStatus;
		ASSERT(prRxStatus);

		
		prBssDesc->fgIsLargerTSF = HAL_RX_STATUS_GET_TCL(prRxStatus);

		
		prBssDesc->eBand = HAL_RX_STATUS_GET_RF_BAND(prRxStatus);

		
		ucHwChannelNum = HAL_RX_STATUS_GET_CHNL_NUM(prRxStatus);

		ASSERT(prSwRfb->prRxStatusGroup3);
		ucRxRCPI = (UINT_8) HAL_RX_STATUS_GET_RCPI(prSwRfb->prRxStatusGroup3);
		if (BAND_2G4 == prBssDesc->eBand) {

			

			if (ucIeDsChannelNum >= 1 && ucIeDsChannelNum <= 14) {

				
				if ((ucIeDsChannelNum == ucHwChannelNum) ||
				    (ucRxRCPI > prBssDesc->ucRCPI)) {
					prBssDesc->ucRCPI = ucRxRCPI;
				}
				
				prBssDesc->ucChannelNum = ucIeDsChannelNum;
			} else if (ucIeHtChannelNum >= 1 && ucIeHtChannelNum <= 14) {
				
				if ((ucIeHtChannelNum == ucHwChannelNum) ||
				    (ucRxRCPI > prBssDesc->ucRCPI)) {
					prBssDesc->ucRCPI = ucRxRCPI;
				}
				
				prBssDesc->ucChannelNum = ucIeHtChannelNum;
			} else {
				prBssDesc->ucRCPI = ucRxRCPI;

				prBssDesc->ucChannelNum = ucHwChannelNum;
			}
			
			
			if (prBssDesc->eChannelWidth == CW_80MHZ) {

				
				DBGLOG(RLM, WARN, ("scanAddToBssDesc: B=%d, W=%d\n",prBssDesc->eBand, prBssDesc->eChannelWidth));
				DBGLOG(RLM, WARN, ("IE Length= %u\n",u2IELength));
				DBGLOG_MEM8(RLM, WARN, pucIE, u2IELength);

				
				prBssDesc->eChannelWidth = CW_20_40MHZ; 
				prBssDesc->ucCenterFreqS1 = 0;
				prBssDesc->ucCenterFreqS2 = 0;
				
				
				prBssDesc->eSco = CHNL_EXT_SCN;
			}
			
		}
		
		else {
			if (ucIeHtChannelNum >= 1 && ucIeHtChannelNum < 200) {
				
				if ((ucIeHtChannelNum == ucHwChannelNum) ||
				    (ucRxRCPI > prBssDesc->ucRCPI)) {
					prBssDesc->ucRCPI = ucRxRCPI;
				}
				
				prBssDesc->ucChannelNum = ucIeHtChannelNum;
			} else {
				
				prBssDesc->ucRCPI = ucRxRCPI;

				prBssDesc->ucChannelNum = ucHwChannelNum;
			}
		}
	}


	
	prBssDesc->ucPhyTypeSet = 0;

	if (BAND_2G4 == prBssDesc->eBand) {
		
		if (prBssDesc->fgIsHTPresent) {
			prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_HT;
		}

		
		if (!(prBssDesc->u2BSSBasicRateSet & RATE_SET_BIT_HT_PHY)) {
			
			if ((prBssDesc->u2OperationalRateSet & RATE_SET_OFDM) ||
			    prBssDesc->fgIsERPPresent) {
				prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_ERP;
			}

			
			if (!(prBssDesc->u2BSSBasicRateSet & RATE_SET_OFDM)) {
				
				if ((prBssDesc->u2OperationalRateSet & RATE_SET_HR_DSSS)) {
					prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_HR_DSSS;
				}
			}
		}
	} else {		
		
		if (prBssDesc->fgIsVHTPresent) {
			prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_VHT;
		}

		if (prBssDesc->fgIsHTPresent) {
			prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_HT;
		}

		
		if (!(prBssDesc->u2BSSBasicRateSet & RATE_SET_BIT_HT_PHY)) {
			
			prBssDesc->ucPhyTypeSet |= PHY_TYPE_BIT_OFDM;

			
		}
	}


	
	GET_CURRENT_SYSTIME(&prBssDesc->rUpdateTime);

	return prBssDesc;
}


WLAN_STATUS
scanAddScanResult(IN P_ADAPTER_T prAdapter, IN P_BSS_DESC_T prBssDesc, IN P_SW_RFB_T prSwRfb)
{
	P_SCAN_INFO_T prScanInfo;
	UINT_8 aucRatesEx[PARAM_MAX_LEN_RATES_EX];
	P_WLAN_BEACON_FRAME_T prWlanBeaconFrame;
	PARAM_MAC_ADDRESS rMacAddr;
	PARAM_SSID_T rSsid;
	ENUM_PARAM_NETWORK_TYPE_T eNetworkType;
	PARAM_802_11_CONFIG_T rConfiguration;
	ENUM_PARAM_OP_MODE_T eOpMode;
	UINT_8 ucRateLen = 0;
	UINT_32 i;

	ASSERT(prAdapter);
	ASSERT(prSwRfb);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	if (prBssDesc->eBand == BAND_2G4) {
		if ((prBssDesc->u2OperationalRateSet & RATE_SET_OFDM)
		    || prBssDesc->fgIsERPPresent) {
			eNetworkType = PARAM_NETWORK_TYPE_OFDM24;
		} else {
			eNetworkType = PARAM_NETWORK_TYPE_DS;
		}
	} else {
		ASSERT(prBssDesc->eBand == BAND_5G);
		eNetworkType = PARAM_NETWORK_TYPE_OFDM5;
	}

	if (prBssDesc->eBSSType == BSS_TYPE_P2P_DEVICE) {
		
		return WLAN_STATUS_FAILURE;
	}

	prWlanBeaconFrame = (P_WLAN_BEACON_FRAME_T) prSwRfb->pvHeader;
	COPY_MAC_ADDR(rMacAddr, prWlanBeaconFrame->aucBSSID);
	COPY_SSID(rSsid.aucSsid, rSsid.u4SsidLen, prBssDesc->aucSSID, prBssDesc->ucSSIDLen);

	rConfiguration.u4Length = sizeof(PARAM_802_11_CONFIG_T);
	rConfiguration.u4BeaconPeriod = (UINT_32) prWlanBeaconFrame->u2BeaconInterval;
	rConfiguration.u4ATIMWindow = prBssDesc->u2ATIMWindow;
	rConfiguration.u4DSConfig = nicChannelNum2Freq(prBssDesc->ucChannelNum);
	rConfiguration.rFHConfig.u4Length = sizeof(PARAM_802_11_CONFIG_FH_T);

	rateGetDataRatesFromRateSet(prBssDesc->u2OperationalRateSet, 0, aucRatesEx, &ucRateLen);

	for (i = ucRateLen; i < sizeof(aucRatesEx) / sizeof(aucRatesEx[0]); i++) {
		aucRatesEx[i] = 0;
	}

	switch (prBssDesc->eBSSType) {
	case BSS_TYPE_IBSS:
		eOpMode = NET_TYPE_IBSS;
		break;

	case BSS_TYPE_INFRASTRUCTURE:
	case BSS_TYPE_P2P_DEVICE:
	case BSS_TYPE_BOW_DEVICE:
	default:
		eOpMode = NET_TYPE_INFRA;
		break;
	}

	DBGLOG(SCN, TRACE,
	       ("ind %s %d %d\n", prBssDesc->aucSSID, prBssDesc->ucChannelNum, prBssDesc->ucRCPI));

	kalIndicateBssInfo(prAdapter->prGlueInfo,
			   (PUINT_8) prSwRfb->pvHeader,
			   prSwRfb->u2PacketLen,
			   prBssDesc->ucChannelNum, RCPI_TO_dBm(prBssDesc->ucRCPI));

	nicAddScanResult(prAdapter,
			 rMacAddr,
			 &rSsid,
			 prWlanBeaconFrame->u2CapInfo & CAP_INFO_PRIVACY ? 1 : 0,
			 RCPI_TO_dBm(prBssDesc->ucRCPI),
			 eNetworkType,
			 &rConfiguration,
			 eOpMode,
			 aucRatesEx,
			 prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen,
			 (PUINT_8) ((ULONG) (prSwRfb->pvHeader) + WLAN_MAC_MGMT_HEADER_LEN));

	return WLAN_STATUS_SUCCESS;

}				

BOOLEAN scanCheckBssIsLegal(IN P_ADAPTER_T prAdapter, P_BSS_DESC_T prBssDesc)
{
	BOOLEAN fgAddToScanResult = FALSE;
	ENUM_BAND_T eBand;
	UINT_8 ucChannel;

	ASSERT(prAdapter);
	
	if (rlmDomainIsLegalChannel(prAdapter, prBssDesc->eBand, prBssDesc->ucChannelNum) == TRUE) {
		
		if (cnmAisInfraChannelFixed(prAdapter, &eBand, &ucChannel) == TRUE &&
		    (eBand != prBssDesc->eBand || ucChannel != prBssDesc->ucChannelNum)) {
			fgAddToScanResult = FALSE;
		} else {
			fgAddToScanResult = TRUE;
		}
	}

	return fgAddToScanResult;

}

WLAN_STATUS scanProcessBeaconAndProbeResp(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
	P_SCAN_INFO_T prScanInfo;
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	P_BSS_INFO_T prAisBssInfo;
	P_WLAN_BEACON_FRAME_T prWlanBeaconFrame = (P_WLAN_BEACON_FRAME_T) NULL;
#if CFG_SLT_SUPPORT
	P_SLT_INFO_T prSltInfo = (P_SLT_INFO_T) NULL;
#endif

	ASSERT(prAdapter);
	ASSERT(prSwRfb);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	
	if ((prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) <
	    (TIMESTAMP_FIELD_LEN + BEACON_INTERVAL_FIELD_LEN + CAP_INFO_FIELD_LEN)) {
#ifndef _lint
		ASSERT(0);
#endif				
		return rStatus;
	}
#if CFG_SLT_SUPPORT
	prSltInfo = &prAdapter->rWifiVar.rSltInfo;

	if (prSltInfo->fgIsDUT) {
		DBGLOG(P2P, INFO, ("\n\rBCN: RX\n"));
		prSltInfo->u4BeaconReceiveCnt++;
		return WLAN_STATUS_SUCCESS;
	} else {
		return WLAN_STATUS_SUCCESS;
	}
#endif


	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prAisBssInfo = prAdapter->prAisBssInfo;
	prWlanBeaconFrame = (P_WLAN_BEACON_FRAME_T) prSwRfb->pvHeader;

	
	prBssDesc = scanAddToBssDesc(prAdapter, prSwRfb);

	if (prBssDesc) {

		
		if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED &&
		    ((prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE
		      && prConnSettings->eOPMode != NET_TYPE_IBSS)
		     || (prBssDesc->eBSSType == BSS_TYPE_IBSS
			 && prConnSettings->eOPMode != NET_TYPE_INFRA))
		    && EQUAL_MAC_ADDR(prBssDesc->aucBSSID, prAisBssInfo->aucBSSID)
		    && EQUAL_SSID(prBssDesc->aucSSID, prBssDesc->ucSSIDLen, prAisBssInfo->aucSSID,
				  prAisBssInfo->ucSSIDLen)) {
			BOOLEAN fgNeedDisconnect = FALSE;

#if CFG_SUPPORT_BEACON_CHANGE_DETECTION
			
			if (prAisBssInfo->u2OperationalRateSet != prBssDesc->u2OperationalRateSet) {
				fgNeedDisconnect = TRUE;
			}
#endif

			
			if (fgNeedDisconnect == TRUE) {
				aisBssBeaconTimeout(prAdapter);
			}
		}
		
		if (((prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE
		      && prConnSettings->eOPMode != NET_TYPE_IBSS)
		     || (prBssDesc->eBSSType == BSS_TYPE_IBSS
			 && prConnSettings->eOPMode != NET_TYPE_INFRA))) {
			if (prAisBssInfo->eConnectionState == PARAM_MEDIA_STATE_CONNECTED) {

				if ((!prAisBssInfo->ucDTIMPeriod) &&
				    EQUAL_MAC_ADDR(prBssDesc->aucBSSID, prAisBssInfo->aucBSSID) &&
				    (prAisBssInfo->eCurrentOPMode == OP_MODE_INFRASTRUCTURE) &&
				    ((prWlanBeaconFrame->u2FrameCtrl & MASK_FRAME_TYPE) ==
				     MAC_FRAME_BEACON)) {

					prAisBssInfo->ucDTIMPeriod = prBssDesc->ucDTIMPeriod;

					
					nicPmIndicateBssConnected(prAdapter,
								  prAisBssInfo->ucBssIndex);
				}
			}
#if CFG_SUPPORT_ADHOC
			if (EQUAL_SSID(prBssDesc->aucSSID,
				       prBssDesc->ucSSIDLen,
				       prConnSettings->aucSSID,
				       prConnSettings->ucSSIDLen) &&
			    (prBssDesc->eBSSType == BSS_TYPE_IBSS) &&
			    (prAisBssInfo->eCurrentOPMode == OP_MODE_IBSS)) {

				ASSERT(prSwRfb->prRxStatusGroup3);

				ibssProcessMatchedBeacon(prAdapter, prAisBssInfo, prBssDesc,
							 (UINT_8) HAL_RX_STATUS_GET_RCPI(prSwRfb->
											 prRxStatusGroup3));
			}
#endif				
		}

		rlmProcessBcn(prAdapter,
			      prSwRfb,
			      ((P_WLAN_BEACON_FRAME_T) (prSwRfb->pvHeader))->aucInfoElem,
			      (prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) -
			      (UINT_16) (OFFSET_OF(WLAN_BEACON_FRAME_BODY_T, aucInfoElem[0])));

		mqmProcessBcn(prAdapter,
			      prSwRfb,
			      ((P_WLAN_BEACON_FRAME_T) (prSwRfb->pvHeader))->aucInfoElem,
			      (prSwRfb->u2PacketLen - prSwRfb->u2HeaderLen) -
			      (UINT_16) (OFFSET_OF(WLAN_BEACON_FRAME_BODY_T, aucInfoElem[0])));

		
		if (prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE
		    || prBssDesc->eBSSType == BSS_TYPE_IBSS) {
			
			if (prConnSettings->fgIsScanReqIssued) {
				BOOLEAN fgAddToScanResult;

				fgAddToScanResult = scanCheckBssIsLegal(prAdapter, prBssDesc);

				if (fgAddToScanResult == TRUE) {
					rStatus = scanAddScanResult(prAdapter, prBssDesc, prSwRfb);
				}
			}
		}
#if CFG_ENABLE_WIFI_DIRECT
		if (prAdapter->fgIsP2PRegistered) {
			scanP2pProcessBeaconAndProbeResp(prAdapter,
							 prSwRfb,
							 &rStatus, prBssDesc, prWlanBeaconFrame);
		}
#endif
	}

	return rStatus;

}				


P_BSS_DESC_T scanSearchBssDescByPolicy(IN P_ADAPTER_T prAdapter, IN UINT_8 ucBssIndex)
{
	P_CONNECTION_SETTINGS_T prConnSettings;
	P_BSS_INFO_T prBssInfo;
	P_AIS_SPECIFIC_BSS_INFO_T prAisSpecBssInfo;
	P_SCAN_INFO_T prScanInfo;

	P_LINK_T prBSSDescList;

	P_BSS_DESC_T prBssDesc = (P_BSS_DESC_T) NULL;
	P_BSS_DESC_T prPrimaryBssDesc = (P_BSS_DESC_T) NULL;
	P_BSS_DESC_T prCandidateBssDesc = (P_BSS_DESC_T) NULL;

	P_STA_RECORD_T prStaRec = (P_STA_RECORD_T) NULL;
	P_STA_RECORD_T prPrimaryStaRec;
	P_STA_RECORD_T prCandidateStaRec = (P_STA_RECORD_T) NULL;

	OS_SYSTIME rCurrentTime;

	
	BOOLEAN fgIsFindFirst = (BOOLEAN) FALSE;

	BOOLEAN fgIsFindBestRSSI = (BOOLEAN) FALSE;
	BOOLEAN fgIsFindBestEncryptionLevel = (BOOLEAN) FALSE;
	

	
	

	BOOLEAN fgIsFixedChannel;
	ENUM_BAND_T eBand;
	UINT_8 ucChannel;

	ASSERT(prAdapter);

	prConnSettings = &(prAdapter->rWifiVar.rConnSettings);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIndex);

	prAisSpecBssInfo = &(prAdapter->rWifiVar.rAisSpecificBssInfo);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);
	prBSSDescList = &prScanInfo->rBSSDescList;

	GET_CURRENT_SYSTIME(&rCurrentTime);

	
	if (prBssInfo->eNetworkType == NETWORK_TYPE_AIS) {
#if CFG_SUPPORT_CHNL_CONFLICT_REVISE
		fgIsFixedChannel = cnmAisDetectP2PChannel(prAdapter, &eBand, &ucChannel);
#else
		fgIsFixedChannel = cnmAisInfraChannelFixed(prAdapter, &eBand, &ucChannel);
#endif
	} else {
		fgIsFixedChannel = FALSE;
	}

#if DBG
	if (prConnSettings->ucSSIDLen < ELEM_MAX_LEN_SSID) {
		prConnSettings->aucSSID[prConnSettings->ucSSIDLen] = '\0';
	}
#endif

#if 0
	DBGLOG(SCN, INFO, ("SEARCH: Num Of BSS_DESC_T = %d, Look for SSID: %s\n",
			   prBSSDescList->u4NumElem, prConnSettings->aucSSID));
#endif

	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		

#if 0
		DBGLOG(SCN, INFO, ("SEARCH: [" MACSTR "], SSID:%s\n",
				   MAC2STR(prBssDesc->aucBSSID), prBssDesc->aucSSID));
#endif

		
		
		if (!(prBssDesc->ucPhyTypeSet & (prAdapter->rWifiVar.ucAvailablePhyTypeSet))) {

			DBGLOG(SCN, INFO, ("SEARCH: Ignore unsupported ucPhyTypeSet = %x\n",
					   prBssDesc->ucPhyTypeSet));
			continue;
		}
		
		if (prBssDesc->fgIsUnknownBssBasicRate) {

			continue;
		}
		
		if (fgIsFixedChannel == TRUE &&
		    (prBssDesc->eBand != eBand || prBssDesc->ucChannelNum != ucChannel)) {
			continue;
		}
		
		if (rlmDomainIsLegalChannel(prAdapter, prBssDesc->eBand, prBssDesc->ucChannelNum) ==
		    FALSE) {
			continue;
		}
		
		if (CHECK_FOR_TIMEOUT(rCurrentTime, prBssDesc->rUpdateTime,
				      SEC_TO_SYSTIME(SCN_BSS_DESC_STALE_SEC))) {

			continue;
		}
		
		
		prStaRec = cnmGetStaRecByAddress(prAdapter, ucBssIndex, prBssDesc->aucSrcAddr);

		if (prStaRec) {
#if 0				
			if (prStaRec->u2ReasonCode != REASON_CODE_RESERVED) {
				DBGLOG(SCN, INFO,
				       ("SEARCH: Ignore BSS with previous Reason Code = %d\n",
					prStaRec->u2ReasonCode));
				continue;
			} else
#endif
			if (prStaRec->u2StatusCode != STATUS_CODE_SUCCESSFUL) {
				if ((prStaRec->ucJoinFailureCount < JOIN_MAX_RETRY_FAILURE_COUNT) ||
				    (CHECK_FOR_TIMEOUT(rCurrentTime,
						       prStaRec->rLastJoinTime,
						       SEC_TO_SYSTIME(JOIN_RETRY_INTERVAL_SEC)))) {

					if (prStaRec->ucJoinFailureCount >=
					    JOIN_MAX_RETRY_FAILURE_COUNT) {
						prStaRec->ucJoinFailureCount = 0;
					}
					DBGLOG(SCN, INFO,
					       ("SEARCH: Try to join BSS again which has Status Code = %d (Curr = %ld/Last Join = %ld)\n",
						prStaRec->u2StatusCode, rCurrentTime,
						prStaRec->rLastJoinTime));
				} else {
					DBGLOG(SCN, INFO,
					       ("SEARCH: Ignore BSS which reach maximum Join Retry Count = %d\n",
						JOIN_MAX_RETRY_FAILURE_COUNT));
					continue;
				}

			}
		}

		
		if (prBssInfo->eNetworkType == NETWORK_TYPE_AIS) {

			
			
			if (((prConnSettings->eOPMode == NET_TYPE_INFRA) &&
			     (prBssDesc->eBSSType != BSS_TYPE_INFRASTRUCTURE)) ||
			    ((prConnSettings->eOPMode == NET_TYPE_IBSS
			      || prConnSettings->eOPMode == NET_TYPE_DEDICATED_IBSS)
			     && (prBssDesc->eBSSType != BSS_TYPE_IBSS))) {

				DBGLOG(SCN, INFO, ("SEARCH: Ignore eBSSType = %s\n",
						   ((prBssDesc->eBSSType ==
						     BSS_TYPE_INFRASTRUCTURE) ? "INFRASTRUCTURE" :
						    "IBSS")));
				continue;
			}
			
			if ((prConnSettings->fgIsConnByBssidIssued) &&
			    (prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE)) {

				if (UNEQUAL_MAC_ADDR(prConnSettings->aucBSSID, prBssDesc->aucBSSID)) {

					DBGLOG(SCN, INFO,
					       ("SEARCH: Ignore due to BSSID was not matched!\n"));
					continue;
				}
			}
#if CFG_SUPPORT_ADHOC
			
			if (prBssDesc->eBSSType == BSS_TYPE_IBSS) {
				OS_SYSTIME rCurrentTime;

				
				GET_CURRENT_SYSTIME(&rCurrentTime);
				if (CHECK_FOR_TIMEOUT(rCurrentTime, prBssDesc->rUpdateTime,
						      SEC_TO_SYSTIME
						      (SCN_ADHOC_BSS_DESC_TIMEOUT_SEC))) {
					DBGLOG(SCN, LOUD,
					       ("SEARCH: Skip old record of BSS Descriptor - BSSID:["
						MACSTR "]\n\n", MAC2STR(prBssDesc->aucBSSID)));
					continue;
				}
				
				if (ibssCheckCapabilityForAdHocMode(prAdapter, prBssDesc) ==
				    WLAN_STATUS_FAILURE) {

					DBGLOG(SCN, INFO,
					       ("SEARCH: Ignore BSS DESC MAC: " MACSTR
						", Capability is not supported for current AdHoc Mode.\n",
						MAC2STR(prPrimaryBssDesc->aucBSSID)));

					continue;
				}

				
				if (prBssInfo->fgIsBeaconActivated &&
				    UNEQUAL_MAC_ADDR(prBssInfo->aucBSSID, prBssDesc->aucBSSID)) {

					DBGLOG(SCN, LOUD,
					       ("SEARCH: prBssDesc->fgIsLargerTSF = %d\n",
						prBssDesc->fgIsLargerTSF));

					if (!prBssDesc->fgIsLargerTSF) {
						DBGLOG(SCN, INFO,
						       ("SEARCH: Ignore BSS DESC MAC: [" MACSTR
							"], Smaller TSF\n",
							MAC2STR(prBssDesc->aucBSSID)));
						continue;
					}
				}
			}
#endif				

		}



#if 0				
		
		if (prBssDesc->eBSSType == BSS_TYPE_IBSS) {
			OS_SYSTIME rCurrentTime;

			GET_CURRENT_SYSTIME(&rCurrentTime);
			if (CHECK_FOR_TIMEOUT(rCurrentTime, prBssDesc->rUpdateTime,
					      SEC_TO_SYSTIME(BSS_DESC_TIMEOUT_SEC))) {
				DBGLOG(SCAN, TRACE,
				       ("Skip old record of BSS Descriptor - BSSID:[" MACSTR
					"]\n\n", MAC2STR(prBssDesc->aucBSSID)));
				continue;
			}
		}

		if ((prBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE) &&
		    (prAdapter->eConnectionState == MEDIA_STATE_CONNECTED)) {
			OS_SYSTIME rCurrentTime;

			GET_CURRENT_SYSTIME(&rCurrentTime);
			if (CHECK_FOR_TIMEOUT(rCurrentTime, prBssDesc->rUpdateTime,
					      SEC_TO_SYSTIME(BSS_DESC_TIMEOUT_SEC))) {
				DBGLOG(SCAN, TRACE,
				       ("Skip old record of BSS Descriptor - BSSID:[" MACSTR
					"]\n\n", MAC2STR(prBssDesc->aucBSSID)));
				continue;
			}
		}

		
		
		if (prPrimaryBssDesc->eBSSType == BSS_TYPE_IBSS) {
			
			if (ibssCheckCapabilityForAdHocMode(prAdapter, prPrimaryBssDesc) ==
			    WLAN_STATUS_FAILURE) {

				DBGLOG(SCAN, TRACE,
				       ("Ignore BSS DESC MAC: " MACSTR
					", Capability is not supported for current AdHoc Mode.\n",
					MAC2STR(prPrimaryBssDesc->aucBSSID)));

				continue;
			}

			
			if (prAdapter->fgIsIBSSActive &&
			    UNEQUAL_MAC_ADDR(prBssInfo->aucBSSID, prPrimaryBssDesc->aucBSSID)) {

				if (!fgIsLocalTSFRead) {
					NIC_GET_CURRENT_TSF(prAdapter, &rCurrentTsf);

					DBGLOG(SCAN, TRACE,
					       ("\n\nCurrent TSF : %08lx-%08lx\n\n",
						rCurrentTsf.u.HighPart, rCurrentTsf.u.LowPart));
				}

				if (rCurrentTsf.QuadPart > prPrimaryBssDesc->u8TimeStamp.QuadPart) {
					DBGLOG(SCAN, TRACE,
					       ("Ignore BSS DESC MAC: [" MACSTR
						"], Current BSSID: [" MACSTR "].\n",
						MAC2STR(prPrimaryBssDesc->aucBSSID),
						MAC2STR(prBssInfo->aucBSSID)));

					DBGLOG(SCAN, TRACE,
					       ("\n\nBSS's TSF : %08lx-%08lx\n\n",
						prPrimaryBssDesc->u8TimeStamp.u.HighPart,
						prPrimaryBssDesc->u8TimeStamp.u.LowPart));

					prPrimaryBssDesc->fgIsLargerTSF = FALSE;
					continue;
				} else {
					prPrimaryBssDesc->fgIsLargerTSF = TRUE;
				}

			}
		}
		
		if (rsnPerformPolicySelection(prPrimaryBssDesc)) {

			if (prPrimaryBssDesc->ucEncLevel > 0) {
				fgIsFindBestEncryptionLevel = TRUE;

				fgIsFindFirst = FALSE;
			}
		} else {
			
			continue;
		}

		
		if (prAdapter->rWifiVar.rConnSettings.eAuthMode == AUTH_MODE_WPA2) {
			rsnUpdatePmkidCandidateList(prPrimaryBssDesc);
			if (prAdapter->rWifiVar.rAisBssInfo.u4PmkidCandicateCount) {
				prAdapter->rWifiVar.rAisBssInfo.fgIndicatePMKID =
				    rsnCheckPmkidCandicate();
			}
		}
#endif


		prPrimaryBssDesc = (P_BSS_DESC_T) NULL;

		
		switch (prConnSettings->eConnectionPolicy) {
		case CONNECT_BY_SSID_BEST_RSSI:
			
			if (prAdapter->rWifiVar.fgEnableJoinToHiddenSSID
			    && prBssDesc->fgIsHiddenSSID) {
				if (prConnSettings->ucSSIDLen) {
					prPrimaryBssDesc = prBssDesc;

					fgIsFindBestRSSI = TRUE;
				}

			} else if (EQUAL_SSID(prBssDesc->aucSSID,
					      prBssDesc->ucSSIDLen,
					      prConnSettings->aucSSID, prConnSettings->ucSSIDLen)) {
				prPrimaryBssDesc = prBssDesc;

				fgIsFindBestRSSI = TRUE;
			}
			break;

		case CONNECT_BY_SSID_ANY:
			if (!prBssDesc->fgIsHiddenSSID) {
				prPrimaryBssDesc = prBssDesc;

				fgIsFindFirst = TRUE;
			}
			break;

		case CONNECT_BY_BSSID:
			if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID, prConnSettings->aucBSSID)) {
				prPrimaryBssDesc = prBssDesc;
			}
			break;

		default:
			break;
		}


		
		if (prPrimaryBssDesc == NULL) {
			continue;
		}
		
		if (prPrimaryBssDesc->eBSSType == BSS_TYPE_INFRASTRUCTURE) {
#if CFG_SUPPORT_WAPI
			if (prAdapter->rWifiVar.rConnSettings.fgWapiMode) {
				if (wapiPerformPolicySelection(prAdapter, prPrimaryBssDesc)) {
					fgIsFindFirst = TRUE;
				} else {
					
					continue;
				}
			} else
#endif
			if (rsnPerformPolicySelection(prAdapter, prPrimaryBssDesc)) {
				if (prAisSpecBssInfo->fgCounterMeasure) {
					DBGLOG(RSN, INFO,
					       ("Skip while at counter measure period!!!\n"));
					continue;
				}

				if (prPrimaryBssDesc->ucEncLevel > 0) {
					fgIsFindBestEncryptionLevel = TRUE;

					fgIsFindFirst = FALSE;
				}
			} else {
				
				continue;
			}
		} else {
			
		}

		prPrimaryStaRec = prStaRec;

		
		if (!prCandidateBssDesc) {
			prCandidateBssDesc = prPrimaryBssDesc;
			prCandidateStaRec = prPrimaryStaRec;

			
			if (fgIsFindFirst) {
				break;
			}
		} else {
			
			if (prCandidateBssDesc->fgIsHiddenSSID) {
				if (!prPrimaryBssDesc->fgIsHiddenSSID) {
					prCandidateBssDesc = prPrimaryBssDesc;	
					prCandidateStaRec = prPrimaryStaRec;
					continue;
				}
			} else {
				if (prPrimaryBssDesc->fgIsHiddenSSID) {
					continue;
				}
			}


			
			if (fgIsFindBestRSSI) {
				DBGLOG(SCN, TRACE,
				       ("Candidate [" MACSTR "]: RCPI = %d, Primary [" MACSTR
					"]: RCPI = %d\n", MAC2STR(prCandidateBssDesc->aucBSSID),
					prCandidateBssDesc->ucRCPI,
					MAC2STR(prPrimaryBssDesc->aucBSSID),
					prPrimaryBssDesc->ucRCPI));

				ASSERT(!(prCandidateBssDesc->fgIsConnected &&
					 prPrimaryBssDesc->fgIsConnected));

				
				if (prCandidateBssDesc->fgIsConnected) {
					if (prCandidateBssDesc->ucRCPI +
					    ROAMING_NO_SWING_RCPI_STEP <=
					    prPrimaryBssDesc->ucRCPI) {

						prCandidateBssDesc = prPrimaryBssDesc;
						prCandidateStaRec = prPrimaryStaRec;
						continue;
					}
				} else if (prPrimaryBssDesc->fgIsConnected) {
					if (prCandidateBssDesc->ucRCPI <
					    prPrimaryBssDesc->ucRCPI + ROAMING_NO_SWING_RCPI_STEP) {

						prCandidateBssDesc = prPrimaryBssDesc;
						prCandidateStaRec = prPrimaryStaRec;
						continue;
					}
				} else if (prCandidateBssDesc->ucRCPI < prPrimaryBssDesc->ucRCPI) {
					prCandidateBssDesc = prPrimaryBssDesc;
					prCandidateStaRec = prPrimaryStaRec;
					continue;
				}
			}
#if 0
			
			if (fgIsFindMinChannelLoad) {

				
			}
#endif
		}
	}

	return prCandidateBssDesc;

}				

VOID
scanReportBss2Cfg80211(IN P_ADAPTER_T prAdapter,
		       IN ENUM_BSS_TYPE_T eBSSType, IN P_BSS_DESC_T SpecificprBssDesc)
{
	P_SCAN_INFO_T prScanInfo = NULL;
	P_LINK_T prBSSDescList = NULL;
	P_BSS_DESC_T prBssDesc = NULL;
	RF_CHANNEL_INFO_T rChannelInfo;



	ASSERT(prAdapter);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prBSSDescList = &prScanInfo->rBSSDescList;

	DBGLOG(SCN, TRACE, ("scanReportBss2Cfg80211\n"));

	if (SpecificprBssDesc) {
		{
			
			if (!scanCheckBssIsLegal(prAdapter, SpecificprBssDesc)) {
				DBGLOG(SCN, TRACE, ("Remove specific SSID[%s %d]\n",
						    prBssDesc->aucSSID, prBssDesc->ucChannelNum));
				return;
			}

			DBGLOG(SCN, TRACE,
			       ("Report Specific SSID[%s]\n", SpecificprBssDesc->aucSSID));
			if (eBSSType == BSS_TYPE_INFRASTRUCTURE) {

				kalIndicateBssInfo(prAdapter->prGlueInfo,
						   (PUINT_8) SpecificprBssDesc->aucRawBuf,
						   SpecificprBssDesc->u2RawLength,
						   SpecificprBssDesc->ucChannelNum,
						   RCPI_TO_dBm(SpecificprBssDesc->ucRCPI));
			} else {


				rChannelInfo.ucChannelNum = SpecificprBssDesc->ucChannelNum;
				rChannelInfo.eBand = SpecificprBssDesc->eBand;
				kalP2PIndicateBssInfo(prAdapter->prGlueInfo,
						      (PUINT_8) SpecificprBssDesc->aucRawBuf,
						      SpecificprBssDesc->u2RawLength,
						      &rChannelInfo,
						      RCPI_TO_dBm(SpecificprBssDesc->ucRCPI));

			}

#if CFG_ENABLE_WIFI_DIRECT
			SpecificprBssDesc->fgIsP2PReport = FALSE;
#endif
		}
	} else {
		
		LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {
#if CFG_AUTO_CHANNEL_SEL_SUPPORT
			
			P_PARAM_CHN_LOAD_INFO prChnLoad = NULL;

			if ((prBssDesc->ucChannelNum <= 48) && (prBssDesc->ucChannelNum >= 1)) {
				if (prBssDesc->ucChannelNum <= 14)
					prChnLoad =
					    (P_PARAM_CHN_LOAD_INFO) &(prAdapter->rWifiVar.rChnLoadInfo.rEachChnLoad[prBssDesc->ucChannelNum - 1]);
				else
					prChnLoad =
					    (P_PARAM_CHN_LOAD_INFO) &(prAdapter->rWifiVar.rChnLoadInfo.rEachChnLoad[(prBssDesc->ucChannelNum / 4) + 5]);
				prChnLoad->u2APNum++;
				prChnLoad->ucChannel=prBssDesc->ucChannelNum;
			}
			DBGLOG(SCN, TRACE, ("chNum=%d,apNum=%d\n", prBssDesc->ucChannelNum, prChnLoad->u2APNum));
#endif

			
			if (!scanCheckBssIsLegal(prAdapter, prBssDesc)) {
				DBGLOG(SCN, TRACE, ("Remove SSID[%s %d]\n",
						    prBssDesc->aucSSID, prBssDesc->ucChannelNum));
				continue;
			}

			if ((prBssDesc->eBSSType == eBSSType)
#if CFG_ENABLE_WIFI_DIRECT
			    || ((eBSSType == BSS_TYPE_P2P_DEVICE) &&
				(prBssDesc->fgIsP2PReport == TRUE))
#endif
			    ) {

				DBGLOG(SCN, TRACE, ("Report ALL SSID[%s %d]\n",
						    prBssDesc->aucSSID, prBssDesc->ucChannelNum));

				if (eBSSType == BSS_TYPE_INFRASTRUCTURE) {
					if (prBssDesc->u2RawLength != 0) {
						kalIndicateBssInfo(prAdapter->prGlueInfo,
								   (PUINT_8) prBssDesc->aucRawBuf,
								   prBssDesc->u2RawLength,
								   prBssDesc->ucChannelNum,
								   RCPI_TO_dBm(prBssDesc->ucRCPI));
						kalMemZero(prBssDesc->aucRawBuf,
							   CFG_RAW_BUFFER_SIZE);
						prBssDesc->u2RawLength = 0;

#if CFG_ENABLE_WIFI_DIRECT
						prBssDesc->fgIsP2PReport = FALSE;
#endif
					}
				} else {
#if CFG_ENABLE_WIFI_DIRECT
					if (prBssDesc->fgIsP2PReport == TRUE)
#endif
					{
						rChannelInfo.ucChannelNum = prBssDesc->ucChannelNum;
						rChannelInfo.eBand = prBssDesc->eBand;

						kalP2PIndicateBssInfo(prAdapter->prGlueInfo,
								      (PUINT_8) prBssDesc->
								      aucRawBuf,
								      prBssDesc->u2RawLength,
								      &rChannelInfo,
								      RCPI_TO_dBm(prBssDesc->
										  ucRCPI));

						

#if CFG_ENABLE_WIFI_DIRECT
						prBssDesc->fgIsP2PReport = FALSE;
#endif
					}
				}
			}

		}
#if CFG_AUTO_CHANNEL_SEL_SUPPORT
		prAdapter->rWifiVar.rChnLoadInfo.fgDataReadyBit = TRUE;
#endif

	}

}


#if CFG_SUPPORT_PASSPOINT
P_BSS_DESC_T
scanSearchBssDescByBssidAndLatestUpdateTime(IN P_ADAPTER_T prAdapter, IN UINT_8 aucBSSID[]
    )
{
	P_SCAN_INFO_T prScanInfo;
	P_LINK_T prBSSDescList;
	P_BSS_DESC_T prBssDesc;
	P_BSS_DESC_T prDstBssDesc = (P_BSS_DESC_T) NULL;
	OS_SYSTIME rLatestUpdateTime = 0;

	ASSERT(prAdapter);
	ASSERT(aucBSSID);

	prScanInfo = &(prAdapter->rWifiVar.rScanInfo);

	prBSSDescList = &prScanInfo->rBSSDescList;

	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {

		if (EQUAL_MAC_ADDR(prBssDesc->aucBSSID, aucBSSID)) {
			if (!rLatestUpdateTime
			    || CHECK_FOR_EXPIRATION(prBssDesc->rUpdateTime, rLatestUpdateTime)) {
				prDstBssDesc = prBssDesc;
				COPY_SYSTIME(rLatestUpdateTime, prBssDesc->rUpdateTime);
			}
		}
	}

	return prDstBssDesc;

}				

#endif				

#if CFG_SUPPORT_AGPS_ASSIST
VOID scanReportScanResultToAgps(P_ADAPTER_T prAdapter) {
	P_LINK_T prBSSDescList = &prAdapter->rWifiVar.rScanInfo.rBSSDescList;
	P_BSS_DESC_T prBssDesc = NULL;
	P_AGPS_AP_LIST_T prAgpsApList = kalMemAlloc(sizeof(AGPS_AP_LIST_T), VIR_MEM_TYPE);
	P_AGPS_AP_INFO_T prAgpsInfo = &prAgpsApList->arApInfo[0];
	P_SCAN_INFO_T prScanInfo = &prAdapter->rWifiVar.rScanInfo;
	UINT_8 ucIndex = 0;
	
	LINK_FOR_EACH_ENTRY(prBssDesc, prBSSDescList, rLinkEntry, BSS_DESC_T) {
		if (prBssDesc->rUpdateTime < prScanInfo->rLastScanCompletedTime)
			continue;
		COPY_MAC_ADDR(prAgpsInfo->aucBSSID, prBssDesc->aucBSSID);
		prAgpsInfo->ePhyType = AGPS_PHY_G;
		prAgpsInfo->u2Channel = prBssDesc->ucChannelNum;
		prAgpsInfo->i2ApRssi = RCPI_TO_dBm(prBssDesc->ucRCPI);
		prAgpsInfo++;
		ucIndex++;
		if (ucIndex == SCN_AGPS_AP_LIST_MAX_NUM)
			break;
	}
	prAgpsApList->ucNum = ucIndex;
	GET_CURRENT_SYSTIME(&prScanInfo->rLastScanCompletedTime);
	
	kalIndicateAgpsNotify(prAdapter, AGPS_EVENT_WLAN_AP_LIST, (PUINT_8)prAgpsApList, sizeof(AGPS_AP_LIST_T));
	kalMemFree(prAgpsApList, VIR_MEM_TYPE, sizeof(AGPS_AP_LIST_T));
}
#endif 

