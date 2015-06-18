






#include "config.h"

#if CFG_ENABLE_WIFI_DIRECT && CFG_ENABLE_WIFI_DIRECT_CFG_80211
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>

#include "precomp.h"
#include "gl_cfg80211.h"
#include "gl_p2p_os.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wformat"
#endif









BOOLEAN
mtk_p2p_cfg80211func_channel_format_switch(IN struct ieee80211_channel *channel,
					   IN enum nl80211_channel_type channel_type,
					   IN P_RF_CHANNEL_INFO_T prRfChnlInfo,
					   IN P_ENUM_CHNL_EXT_T prChnlSco);


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)


INT_32
mtk_Netdev_To_RoleIdx(P_GL_P2P_INFO_T prGlP2pInfo, struct net_device *ndev, PUINT_8 pucRoleIdx)
{
	INT_32 i4Ret = -1;
	UINT_32 u4Idx = 0;

	if (pucRoleIdx == NULL) {
		return i4Ret;
	}

	for (u4Idx = 0; u4Idx < KAL_P2P_NUM; u4Idx++) {
		if (prGlP2pInfo->aprRoleHandler[u4Idx] == ndev) {
			*pucRoleIdx = (UINT_8) u4Idx;
			i4Ret = 0;
		}
	}
#if  1
	i4Ret = 0;
	*pucRoleIdx = 0;
#endif
	return i4Ret;

}				

#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)

#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
struct wireless_dev *
#else
struct net_device *
#endif
mtk_p2p_cfg80211_add_iface(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
			   const char *name,
#else
			   char *name,
#endif
			   enum nl80211_iftype type, u32 *flags, struct vif_params *params)
{
	
#if 0
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	struct net_device *prNewNetDevice = NULL;
	UINT_32 u4Idx = 0;
	P_GL_P2P_INFO_T prP2pInfo = (P_GL_P2P_INFO_T) NULL;


	do {
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
		if (prGlueInfo == NULL) {
			break;
		}

		prP2pInfo = prGlueInfo->prP2PInfo;

		for (u4Idx = 0; u4Idx < KAL_P2P_NUM; u4Idx++) {
			if (prP2pInfo->aprRoleHandler[u4Idx] == NULL) {
				p2pRoleFsmInit(prGlueInfo->prAdapter, u4Idx);
			} else if (prP2pInfo->aprRoleHandler[u4Idx] == prP2pInfo->prDevHandler) {
				break;
			}
		}

		if (u4Idx == 2) {
			
			break;
		}

		prNewNetDevice =
		    alloc_netdev_mq(sizeof(P_GLUE_INFO_T), name, ether_setup, CFG_MAX_TXQ_NUM);

		if (prNewNetDevice == NULL) {
			break;
		}

		prP2pInfo->aprRoleHandler[u4Idx] = prNewNetDevice;

		*((P_GLUE_INFO_T *) netdev_priv(prNewNetDevice)) = prGlueInfo;

		memcpy(prNewNetDevice->perm_addr,
		       prGlueInfo->prAdapter->rWifiVar.aucInterfaceAddress, ETH_ALEN);
		prNewNetDevice->dev_addr = prNewNetDevice->perm_addr;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
		prNewNetDevice->netdev_ops = &p2p_netdev_ops;
#else
		ASSERT(FALSE);
#endif

#if (MTK_WCN_HIF_SDIO == 0)
		SET_NETDEV_DEV(prNewNetDevice, &(prGlueInfo->rHifInfo.func->dev));
#endif


#if CFG_ENABLE_WIFI_DIRECT_CFG_80211
		prNewNetDevice->ieee80211_ptr = &(prP2pInfo->wdev);
#endif

#if CFG_TCP_IP_CHKSUM_OFFLOAD
		prNewNetDevice->features = NETIF_F_IP_CSUM;
#endif				


	} while (FALSE);


	return prNewNetDevice;

#else
	return NULL;
#endif
}				


int mtk_p2p_cfg80211_del_iface(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
			       struct wireless_dev *wdev
#else
			       struct net_device *dev
#endif
    )
{
	
	return 0;
}				


int mtk_p2p_cfg80211_add_key(struct wiphy *wiphy,
			     struct net_device *ndev,
			     u8 key_index,
			     bool pairwise, const u8 *mac_addr, struct key_params *params)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	INT_32 i4Rslt = -EINVAL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	P2P_PARAM_KEY_T rKey;
	UINT_8 ucRoleIdx = 0;
	const UINT_8 aucBCAddr[] = BC_MAC_ADDR;
	const UINT_8 aucZeroMacAddr[] = NULL_MAC_ADDR;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) != 0) {
		return -EINVAL;
	}
#if DBG
	DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_add_key\n"));
	if (mac_addr) {
		DBGLOG(RSN, TRACE,
		       ("keyIdx = %d pairwise = %d mac = " MACSTR "\n", key_index, pairwise,
			MAC2STR(mac_addr)));
	} else {
		DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d null mac\n", key_index, pairwise));
	}
	DBGLOG(RSN, TRACE, ("Cipher = %x\n", params->cipher));
	DBGLOG_MEM8(RSN, TRACE, params->key, params->key_len);
#endif

	

	kalMemZero(&rKey, sizeof(P2P_PARAM_KEY_T));

	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
				ucRoleIdx, &rKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	rKey.u4KeyIndex = key_index;

	if (mac_addr) {
		if (EQUAL_MAC_ADDR(mac_addr, aucZeroMacAddr))
			COPY_MAC_ADDR(rKey.arBSSID, aucBCAddr);
		else
			COPY_MAC_ADDR(rKey.arBSSID, mac_addr);

		if (pairwise) {
			
			
			
			rKey.u4KeyIndex |= BIT(31);
			rKey.u4KeyIndex |= BIT(30);
		}
	} else {
		COPY_MAC_ADDR(rKey.arBSSID, aucBCAddr);
	}

	if (params->key) {
		kalMemCopy(rKey.aucKeyMaterial, params->key, params->key_len);
	}
	rKey.u4KeyLength = params->key_len;
	rKey.u4Length = ((ULONG) &(((P_P2P_PARAM_KEY_T) 0)->aucKeyMaterial)) + rKey.u4KeyLength;

	rStatus = kalIoctl(prGlueInfo,
			   wlanoidSetAddP2PKey,
			   &rKey, rKey.u4Length, FALSE, FALSE, TRUE, &u4BufLen);
	if (rStatus == WLAN_STATUS_SUCCESS)
		i4Rslt = 0;

	return i4Rslt;
}


int mtk_p2p_cfg80211_get_key(struct wiphy *wiphy,
			     struct net_device *ndev,
			     u8 key_index,
			     bool pairwise,
			     const u8 *mac_addr,
			     void *cookie, void (*callback) (void *cookie, struct key_params *)
    )
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	

	return -EINVAL;
}


int mtk_p2p_cfg80211_del_key(struct wiphy *wiphy,
			     struct net_device *ndev,
			     u8 key_index, bool pairwise, const u8 *mac_addr)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	PARAM_REMOVE_KEY_T rRemoveKey;
	INT_32 i4Rslt = -EINVAL;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	UINT_32 u4BufLen = 0;
	UINT_8 ucRoleIdx = 0;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


	if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) < 0) {
		return -EINVAL;
	}
#if DBG
	DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_del_key\n"));
	if (mac_addr) {
		DBGLOG(RSN, TRACE,
		       ("keyIdx = %d pairwise = %d mac = " MACSTR "\n", key_index, pairwise,
			MAC2STR(mac_addr)));
	} else {
		DBGLOG(RSN, TRACE, ("keyIdx = %d pairwise = %d null mac\n", key_index, pairwise));
	}
#endif

	kalMemZero(&rRemoveKey, sizeof(PARAM_REMOVE_KEY_T));

	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
				ucRoleIdx, &rRemoveKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	if (mac_addr)
		COPY_MAC_ADDR(rRemoveKey.arBSSID, mac_addr);
	rRemoveKey.u4KeyIndex = key_index;
	rRemoveKey.u4Length = sizeof(PARAM_REMOVE_KEY_T);

	rStatus = kalIoctl(prGlueInfo,
			   wlanoidSetRemoveP2PKey,
			   &rRemoveKey, rRemoveKey.u4Length, FALSE, FALSE, TRUE, &u4BufLen);

	if (rStatus == WLAN_STATUS_SUCCESS)
		i4Rslt = 0;

	return i4Rslt;
}


int
mtk_p2p_cfg80211_set_default_key(struct wiphy *wiphy,
				 struct net_device *netdev,
				 u8 key_index, bool unicast, bool multicast)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	
	UINT_8 ucRoleIdx = 0;
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	INT_32 i4Rst = -EINVAL;
	
	

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, netdev, &ucRoleIdx) != 0) {
		return -EINVAL;
	}
#if DBG
	DBGLOG(RSN, TRACE, ("mtk_p2p_cfg80211_set_default_key\n"));
	DBGLOG(RSN, TRACE,
	       ("keyIdx = %d unicast = %d multicast = %d\n", key_index, unicast, multicast));
#endif
#if 0
	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
				ucRoleIdx, &rDefaultKey.ucBssIdx) != WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	rDefaultKey.ucKeyID = key_index;
	rDefaultKey.ucUnicast = unicast;
	rDefaultKey.ucMulticast = multicast;
	if (rDefaultKey.ucUnicast && !rDefaultKey.ucMulticast)
		return -EINVAL;

	if (rDefaultKey.ucUnicast && rDefaultKey.ucMulticast)
		fgDef = TRUE;

	if (!rDefaultKey.ucUnicast && rDefaultKey.ucMulticast)
		fgMgtDef = TRUE;
#endif
#if 0
	rStatus = kalIoctl(prGlueInfo,
			   wlanoidSetDefaultP2PKey,
			   &rDefaultKey,
			   sizeof(PARAM_DEFAULT_KEY_T), FALSE, FALSE, TRUE, FALSE, &u4BufLen);
#endif

	if (rStatus == WLAN_STATUS_SUCCESS)
		i4Rst = 0;

	return i4Rst;
}

int mtk_p2p_cfg80211_get_station(struct wiphy *wiphy,
				 struct net_device *ndev, u8 *mac, struct station_info *sinfo)
{
	INT_32 i4RetRslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	P_GL_P2P_INFO_T prP2pGlueInfo = (P_GL_P2P_INFO_T) NULL;
	P2P_STATION_INFO_T rP2pStaInfo;

	ASSERT(wiphy);

	do {
		if ((wiphy == NULL) || (ndev == NULL) || (sinfo == NULL) || (mac == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_get_station\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
		prP2pGlueInfo = prGlueInfo->prP2PInfo;

		sinfo->filled = 0;

		
		
		p2pFuncGetStationInfo(prGlueInfo->prAdapter, mac, &rP2pStaInfo);

		
		sinfo->filled |= STATION_INFO_INACTIVE_TIME;
		sinfo->inactive_time = rP2pStaInfo.u4InactiveTime;
		sinfo->generation = prP2pGlueInfo->i4Generation;

		i4RetRslt = 0;
	} while (FALSE);

	return i4RetRslt;
}

int mtk_p2p_cfg80211_scan(struct wiphy *wiphy,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
			  struct net_device *ndev,
#endif
			  struct cfg80211_scan_request *request)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	P_GL_P2P_INFO_T prP2pGlueInfo = (P_GL_P2P_INFO_T) NULL;
	P_MSG_P2P_SCAN_REQUEST_T prMsgScanRequest = (P_MSG_P2P_SCAN_REQUEST_T) NULL;
	UINT_32 u4MsgSize = 0, u4Idx = 0;
	INT_32 i4RetRslt = -EINVAL;
	P_RF_CHANNEL_INFO_T prRfChannelInfo = (P_RF_CHANNEL_INFO_T) NULL;
	P_P2P_SSID_STRUCT_T prSsidStruct = (P_P2P_SSID_STRUCT_T) NULL;
	struct ieee80211_channel *prChannel = NULL;
	struct cfg80211_ssid *prSsid = NULL;
	UINT_8 ucBssIdx = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
	UINT_8 ucRoleIdx = 0;
#endif
        BOOLEAN fgIsFullChanScan = FALSE;

	


	do {
		if ((wiphy == NULL) || (request == NULL)) {
			break;
		}

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		prP2pGlueInfo = prGlueInfo->prP2PInfo;

		if (prP2pGlueInfo == NULL) {
			ASSERT(FALSE);
			break;
		}

		DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_scan.\n"));


		if (prP2pGlueInfo->prScanRequest != NULL) {
			
			DBGLOG(P2P, TRACE,
			       ("There have been a scan request on-going processing.\n"));
			break;
		}

		prP2pGlueInfo->prScanRequest = request;

		
		if (request->n_channels > MAXIMUM_OPERATION_CHANNEL_LIST) {
			request->n_channels = MAXIMUM_OPERATION_CHANNEL_LIST;
                        fgIsFullChanScan = TRUE;
			DBGLOG(P2P, INFO, ("Channel list exceed the maximun support.\n"));
		}
		
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		if (ndev == prGlueInfo->prP2PInfo->prDevHandler) {
			ucBssIdx = P2P_DEV_BSS_INDEX;
			DBGLOG(P2P, TRACE, ("Device Port Scan.\n"));
		} else if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) < 0) {
			DBGLOG(P2P, TRACE, ("Can not find role port.\n"));
			break;
		} else {
			if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter,
						ucRoleIdx, &ucBssIdx) != WLAN_STATUS_SUCCESS) {
				DBGLOG(P2P, ERROR, ("Role Port to BSS index fail.\n"));
				break;
			}
			DBGLOG(P2P, TRACE, ("Role Port Scan.\n"));
		}
#else
		ucBssIdx = P2P_DEV_BSS_INDEX;
		DBGLOG(P2P, TRACE, ("Device Port Scan.\n"));
#endif

		u4MsgSize = sizeof(MSG_P2P_SCAN_REQUEST_T) +
		    (request->n_channels * sizeof(RF_CHANNEL_INFO_T)) +
		    (request->n_ssids * sizeof(PARAM_SSID_T)) + request->ie_len;

		prMsgScanRequest = cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, u4MsgSize);

		if (prMsgScanRequest == NULL) {
			ASSERT(FALSE);
			i4RetRslt = -ENOMEM;
			break;
		}

		DBGLOG(P2P, TRACE, ("Generating scan request message.\n"));

		prMsgScanRequest->rMsgHdr.eMsgId = MID_MNY_P2P_DEVICE_DISCOVERY;
		prMsgScanRequest->eScanType = SCAN_TYPE_ACTIVE_SCAN;
		prMsgScanRequest->ucBssIdx = ucBssIdx;



		DBGLOG(P2P, INFO, ("Requesting channel number:%d.\n", request->n_channels));

		for (u4Idx = 0; u4Idx < request->n_channels; u4Idx++) {
			
			prRfChannelInfo = &(prMsgScanRequest->arChannelListInfo[u4Idx]);
			prChannel = request->channels[u4Idx];

			prRfChannelInfo->ucChannelNum =
			    nicFreq2ChannelNum(prChannel->center_freq * 1000);
			DBGLOG(P2P, TRACE,
			       ("Scanning Channel:%d,  freq: %d\n", prRfChannelInfo->ucChannelNum,
				prChannel->center_freq));
			switch (prChannel->band) {
			case IEEE80211_BAND_2GHZ:
				prRfChannelInfo->eBand = BAND_2G4;
				break;
			case IEEE80211_BAND_5GHZ:
				prRfChannelInfo->eBand = BAND_5G;
				break;
			default:
				DBGLOG(P2P, INFO, ("UNKNOWN Band info from supplicant\n"));
				prRfChannelInfo->eBand = BAND_NULL;
				break;
			}

			
			prRfChannelInfo++;
		}
		prMsgScanRequest->u4NumChannel = request->n_channels;
                if(fgIsFullChanScan){
                    prMsgScanRequest->u4NumChannel = SCN_P2P_FULL_SCAN_PARAM;
                    DBGLOG(P2P, INFO, ("request->n_channels = SCN_P2P_FULL_SCAN_PARAM ;\n"));
                }
		DBGLOG(P2P, INFO, ("Finish channel list u4NumChannel[%d].\n",prMsgScanRequest->u4NumChannel));

		
		prSsid = request->ssids;
		prSsidStruct = (P_P2P_SSID_STRUCT_T) prRfChannelInfo;
		if (request->n_ssids) {
			ASSERT((ULONG) prSsidStruct ==
			       (ULONG) &(prMsgScanRequest->arChannelListInfo[u4Idx]));
			prMsgScanRequest->prSSID = prSsidStruct;
		}

		for (u4Idx = 0; u4Idx < request->n_ssids; u4Idx++) {
			COPY_SSID(prSsidStruct->aucSsid,
				  prSsidStruct->ucSsidLen,
				  request->ssids->ssid, request->ssids->ssid_len);

			prSsidStruct++;
			prSsid++;
		}

		prMsgScanRequest->i4SsidNum = request->n_ssids;

		DBGLOG(P2P, TRACE, ("Finish SSID list:%d.\n", request->n_ssids));

        
        prMsgScanRequest->pucIEBuf = (PUINT_8)prSsidStruct;
        if (request->ie_len) {
            kalMemCopy(prMsgScanRequest->pucIEBuf, request->ie, request->ie_len);
            prMsgScanRequest->u4IELen = request->ie_len;
        }
        else {
			prMsgScanRequest->u4IELen = 0;
        }

		DBGLOG(P2P, TRACE, ("Finish IE Buffer.\n"));

#if CFG_AUTO_CHANNEL_SEL_SUPPORT
		prGlueInfo->prAdapter->rWifiVar.rChnLoadInfo.fgDataReadyBit = FALSE;
#endif

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMsgScanRequest, MSG_SEND_METHOD_BUF);

		i4RetRslt = 0;
	} while (FALSE);

	return i4RetRslt;
}				

int mtk_p2p_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = NULL;


	do {
		if (wiphy == NULL) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_wiphy_params\n"));
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (changed & WIPHY_PARAM_RETRY_SHORT) {
			
			DBGLOG(P2P, TRACE, ("The RETRY short param is changed.\n"));
		}

		if (changed & WIPHY_PARAM_RETRY_LONG) {
			
			DBGLOG(P2P, TRACE, ("The RETRY long param is changed.\n"));
		}


		if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
			
			DBGLOG(P2P, TRACE, ("The RETRY fragmentation threshold is changed.\n"));
		}

		if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
			
			DBGLOG(P2P, TRACE, ("The RETRY RTS threshold is changed.\n"));
		}

		if (changed & WIPHY_PARAM_COVERAGE_CLASS) {
			
			DBGLOG(P2P, TRACE, ("The coverage class is changed???\n"));
		}

		i4Rslt = 0;
	} while (FALSE);




	return i4Rslt;
}				



int
mtk_p2p_cfg80211_join_ibss(struct wiphy *wiphy,
			   struct net_device *dev, struct cfg80211_ibss_params *params)
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	

	return -EINVAL;
}

int mtk_p2p_cfg80211_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	

	return -EINVAL;
}

int mtk_p2p_cfg80211_set_txpower(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				 struct wireless_dev *wdev,
#endif
				 enum nl80211_tx_power_setting type, int mbm)
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	

	return -EINVAL;
}

int mtk_p2p_cfg80211_get_txpower(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				 struct wireless_dev *wdev,
#endif
				 int *dbm)
{
	P_GLUE_INFO_T prGlueInfo = NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	

	return -EINVAL;
}

int
mtk_p2p_cfg80211_set_power_mgmt(struct wiphy *wiphy,
				struct net_device *ndev, bool enabled, int timeout)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	PARAM_POWER_MODE ePowerMode;
	PARAM_POWER_MODE_T rPowerMode;
	UINT_32 u4Leng;
	UINT_8 ucRoleIdx;
	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	if (enabled) {
		ePowerMode = Param_PowerModeFast_PSP;
	} else {
		ePowerMode = Param_PowerModeCAM;
	}

	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_power_mgmt ps=%d.\n", enabled));

	if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) != 0) {
		return -EINVAL;
	}

	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter, ucRoleIdx, &rPowerMode.ucBssIdx) !=
	    WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	rPowerMode.ePowerMode = ePowerMode;

	
	kalIoctl(prGlueInfo,
		 wlanoidSet802dot11PowerSaveProfile,
		 &rPowerMode, sizeof(rPowerMode), FALSE, FALSE, TRUE, &u4Leng);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
int
mtk_p2p_cfg80211_start_ap(struct wiphy *wiphy,
			  struct net_device *dev, struct cfg80211_ap_settings *settings)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T) NULL;
	P_MSG_P2P_START_AP_T prP2pStartAPMsg = (P_MSG_P2P_START_AP_T) NULL;
	PUINT_8 pucBuffer = (PUINT_8) NULL;
	UINT_8 ucRoleIdx = 0;
	struct cfg80211_chan_def *chandef;
	


	do {
		if ((wiphy == NULL) || (settings == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_start_ap.\n"));
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		chandef = &settings->chandef;

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}

		if (chandef) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
			mtk_p2p_cfg80211_set_channel(wiphy, chandef);
#endif


		}


		prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T) cnmMemAlloc(prGlueInfo->prAdapter,
									    RAM_TYPE_MSG,
									    (sizeof
									     (MSG_P2P_BEACON_UPDATE_T)
									     +
									     settings->beacon.
									     head_len +
									     settings->beacon.
									     tail_len));

		if (prP2pBcnUpdateMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}


		prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
		prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
		pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

		if (settings->beacon.head_len != 0) {
			kalMemCopy(pucBuffer, settings->beacon.head, settings->beacon.head_len);

			prP2pBcnUpdateMsg->u4BcnHdrLen = settings->beacon.head_len;

			prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

			pucBuffer =
			    (PUINT_8) ((ULONG) pucBuffer + (ULONG) settings->beacon.head_len);
		} else {
			prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

			prP2pBcnUpdateMsg->pucBcnHdr = NULL;
		}

		if (settings->beacon.tail_len != 0) {
			UINT_8 ucLen = settings->beacon.tail_len;

			prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

			
			
			TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
			TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP) ;	
			TIM_IE(pucBuffer)->ucDTIMCount = 0 ;	
			TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
			TIM_IE(pucBuffer)->ucBitmapControl = 0 ;	
			ucLen += IE_SIZE(pucBuffer);
			pucBuffer += IE_SIZE(pucBuffer);

			kalMemCopy(pucBuffer, settings->beacon.tail, settings->beacon.tail_len);

			prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
		} else {
			prP2pBcnUpdateMsg->u4BcnBodyLen = 0;

			prP2pBcnUpdateMsg->pucBcnBody = NULL;
		}


		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prP2pBcnUpdateMsg, MSG_SEND_METHOD_BUF);


		prP2pStartAPMsg = (P_MSG_P2P_START_AP_T) cnmMemAlloc(prGlueInfo->prAdapter,
								     RAM_TYPE_MSG,
								     sizeof(MSG_P2P_START_AP_T));

		if (prP2pStartAPMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prP2pStartAPMsg->rMsgHdr.eMsgId = MID_MNY_P2P_START_AP;

		prP2pStartAPMsg->fgIsPrivacy = settings->privacy;

		prP2pStartAPMsg->u4BcnInterval = settings->beacon_interval;

		prP2pStartAPMsg->u4DtimPeriod = settings->dtim_period;

		
		prP2pStartAPMsg->ucHiddenSsidType = settings->hidden_ssid;

		prP2pStartAPMsg->ucRoleIdx = ucRoleIdx;

		kalP2PSetRole(prGlueInfo, 2);

		COPY_SSID(prP2pStartAPMsg->aucSsid,
			  prP2pStartAPMsg->u2SsidLen, settings->ssid, settings->ssid_len);

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prP2pStartAPMsg, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;



	} while (FALSE);

	return i4Rslt;



	return i4Rslt;
}				



int
mtk_p2p_cfg80211_change_beacon(struct wiphy *wiphy,
			       struct net_device *dev, struct cfg80211_beacon_data *info)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T) NULL;
	PUINT_8 pucBuffer = (PUINT_8) NULL;
	UINT_8 ucRoleIdx = 0;

	do {
		if ((wiphy == NULL) || (info == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_change_beacon.\n"));
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}


		if ((info->head_len != 0) || (info->tail_len != 0)) {
			prP2pBcnUpdateMsg =
			    (P_MSG_P2P_BEACON_UPDATE_T) cnmMemAlloc(prGlueInfo->prAdapter,
								    RAM_TYPE_MSG,
								    (sizeof(MSG_P2P_BEACON_UPDATE_T)
								     + info->head_len +
								     info->tail_len));


			if (prP2pBcnUpdateMsg == NULL) {
				ASSERT(FALSE);
				i4Rslt = -ENOMEM;
				break;
			}


			prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
			prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
			pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

			if (info->head_len != 0) {
				kalMemCopy(pucBuffer, info->head, info->head_len);

				prP2pBcnUpdateMsg->u4BcnHdrLen = info->head_len;

				prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

				pucBuffer =
				    (PUINT_8) ((ULONG) pucBuffer + (ULONG) info->head_len);
			} else {
				prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

				prP2pBcnUpdateMsg->pucBcnHdr = NULL;
			}

			if (info->tail_len != 0) {
				UINT_8 ucLen = info->tail_len;

				prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

				
				
				TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
				TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP) ;	
				TIM_IE(pucBuffer)->ucDTIMCount = 0 ;	
				TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
				TIM_IE(pucBuffer)->ucBitmapControl = 0 ;	
				ucLen += IE_SIZE(pucBuffer);
				pucBuffer += IE_SIZE(pucBuffer);

				kalMemCopy(pucBuffer, info->tail, info->tail_len);

				prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
			} else {
				prP2pBcnUpdateMsg->u4BcnBodyLen = 0;
				prP2pBcnUpdateMsg->pucBcnBody = NULL;
			}

			kalP2PSetRole(prGlueInfo, 2);

			mboxSendMsg(prGlueInfo->prAdapter,
				    MBOX_ID_0,
				    (P_MSG_HDR_T) prP2pBcnUpdateMsg, MSG_SEND_METHOD_BUF);

		}


		




	} while (FALSE);

	return i4Rslt;
}				

#else
int
mtk_p2p_cfg80211_add_set_beacon(struct wiphy *wiphy,
				struct net_device *dev, struct beacon_parameters *info)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_BEACON_UPDATE_T prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T) NULL;
	P_MSG_P2P_START_AP_T prP2pStartAPMsg = (P_MSG_P2P_START_AP_T) NULL;
	PUINT_8 pucBuffer = (PUINT_8) NULL;
	P_IE_SSID_T prSsidIE = (P_IE_SSID_T) NULL;
	UINT_8 ucRoleIdx = 0;

	do {
		if ((wiphy == NULL) || (info == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_add_set_beacon.\n"));
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2pInfo, dev, &ucRoleIdx) < 0) {
			break;
		}

		prP2pBcnUpdateMsg = (P_MSG_P2P_BEACON_UPDATE_T) cnmMemAlloc(prGlueInfo->prAdapter,
									    RAM_TYPE_MSG,
									    (sizeof
									     (MSG_P2P_BEACON_UPDATE_T)
									     + info->head_len +
									     info->tail_len));

		if (prP2pBcnUpdateMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prP2pBcnUpdateMsg->ucRoleIndex = ucRoleIdx;
		prP2pBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_P2P_BEACON_UPDATE;
		pucBuffer = prP2pBcnUpdateMsg->aucBuffer;

		if (info->head_len != 0) {
			kalMemCopy(pucBuffer, info->head, info->head_len);

			prP2pBcnUpdateMsg->u4BcnHdrLen = info->head_len;

			prP2pBcnUpdateMsg->pucBcnHdr = pucBuffer;

			pucBuffer = (PUINT_8) ((UINT_32) pucBuffer + (UINT_32) info->head_len);
		} else {
			prP2pBcnUpdateMsg->u4BcnHdrLen = 0;

			prP2pBcnUpdateMsg->pucBcnHdr = NULL;
		}

		if (info->tail_len != 0) {
			UINT_8 ucLen = info->tail_len;

			prP2pBcnUpdateMsg->pucBcnBody = pucBuffer;

			
			
			TIM_IE(pucBuffer)->ucId = ELEM_ID_TIM;
			TIM_IE(pucBuffer)->ucLength = (3 + MAX_LEN_TIM_PARTIAL_BMP) ;	
			TIM_IE(pucBuffer)->ucDTIMCount = 0 ;	
			TIM_IE(pucBuffer)->ucDTIMPeriod = 1;
			TIM_IE(pucBuffer)->ucBitmapControl = 0 ;	
			ucLen += IE_SIZE(pucBuffer);
			pucBuffer += IE_SIZE(pucBuffer);

			kalMemCopy(pucBuffer, info->tail, info->tail_len);

			prP2pBcnUpdateMsg->u4BcnBodyLen = ucLen;
		} else {
			prP2pBcnUpdateMsg->u4BcnBodyLen = 0;

			prP2pBcnUpdateMsg->pucBcnBody = NULL;
		}


		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prP2pBcnUpdateMsg, MSG_SEND_METHOD_BUF);


		prP2pStartAPMsg = (P_MSG_P2P_START_AP_T) cnmMemAlloc(prGlueInfo->prAdapter,
								     RAM_TYPE_MSG,
								     sizeof(MSG_P2P_START_AP_T));

		if (prP2pStartAPMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prP2pStartAPMsg->rMsgHdr.eMsgId = MID_MNY_P2P_START_AP;

		prP2pStartAPMsg->fgIsPrivacy = FALSE;

		prP2pStartAPMsg->u4BcnInterval = info->interval;

		prP2pStartAPMsg->u4DtimPeriod = info->dtim_period;

		
		prP2pStartAPMsg->ucHiddenSsidType = ENUM_HIDDEN_SSID_NONE;

#if 0
		if (info->head_len > OFFSET_OF(WLAN_BEACON_FRAME_T, aucInfoElem)) {
			P_WLAN_BEACON_FRAME_T prWlanBcnFrame = info->head;

			prSsidIE = (P_IE_HDR_T) p2pFuncGetSpecIE(prGlueInfo->prAdapter,
								 (PUINT_8) prWlanBcnFrame->
								 aucInfoElem,
								 (info->head_len -
								  OFFSET_OF(WLAN_BEACON_FRAME_T,
									    aucInfoElem)),
								 ELEM_ID_SSID, NULL);

			kalMemCopy(prP2pStartAPMsg->aucSsid, SSID_IE(prSsidIE)->aucSSID,
				   IE_LEN(prSsidIE));

		}
#endif

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prP2pStartAPMsg, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;

	} while (FALSE);

	return i4Rslt;
}

#endif

int mtk_p2p_cfg80211_stop_ap(struct wiphy *wiphy, struct net_device *dev)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_SWITCH_OP_MODE_T prP2pSwitchMode = (P_MSG_P2P_SWITCH_OP_MODE_T) NULL;
	UINT_8 ucRoleIdx = 0;

	do {
		if (wiphy == NULL) {
			break;
		}


		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_stop_ap.\n"));
		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}
		
		prP2pSwitchMode =
		    cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
				sizeof(MSG_P2P_SWITCH_OP_MODE_T));

		if (prP2pSwitchMode == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prP2pSwitchMode->rMsgHdr.eMsgId = MID_MNY_P2P_STOP_AP;
		prP2pSwitchMode->ucRoleIdx = ucRoleIdx;

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prP2pSwitchMode, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);


	return i4Rslt;
}				

int
mtk_p2p_cfg80211_deauth(struct wiphy *wiphy,
			struct net_device *dev, struct cfg80211_deauth_request *req
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
			, void *cookie
#endif
    )
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	
	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_deauth.\n"));

	return -EINVAL;
}				


int
mtk_p2p_cfg80211_disassoc(struct wiphy *wiphy,
			  struct net_device *dev, struct cfg80211_disassoc_request *req
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
			  , void *cookie
#endif
    )
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_disassoc.\n"));

	

	return -EINVAL;
}				


int mtk_p2p_cfg80211_remain_on_channel(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				       struct wireless_dev *wdev,
#else
				       struct net_device *dev,
#endif
				       struct ieee80211_channel *chan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
				       enum nl80211_channel_type channel_type,
#endif
				       unsigned int duration, u64 *cookie)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T) NULL;
	P_MSG_P2P_CHNL_REQUEST_T prMsgChnlReq = (P_MSG_P2P_CHNL_REQUEST_T) NULL;

	DBGLOG(P2P, STATE, ("mtk_p2p_cfg80211_remain_on_channel\n"));

	do {
		if ((wiphy == NULL) ||
		    
		    (chan == NULL) || (cookie == NULL)) {
			break;
		}

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
		prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		if (prGlueP2pInfo->prDevHandler != dev) {
			ASSERT(FALSE);
			
			DBGLOG(P2P, WARN,
			       ("Group Interface also has remain on channel requirement\n"));
			break;
		}
#endif

		*cookie = prGlueP2pInfo->u8Cookie++;

		prMsgChnlReq =
		    cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
				sizeof(MSG_P2P_CHNL_REQUEST_T));

		if (prMsgChnlReq == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_remain_on_channel:%d\n", (INT_32) *cookie));

		prMsgChnlReq->rMsgHdr.eMsgId = MID_MNY_P2P_CHNL_REQ;
		prMsgChnlReq->u8Cookie = *cookie;
		prMsgChnlReq->u4Duration = duration;
		prMsgChnlReq->eChnlReqType = CH_REQ_TYPE_P2P_LISTEN;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		mtk_p2p_cfg80211func_channel_format_switch(chan,
							   channel_type,
							   &prMsgChnlReq->rChannelInfo,
							   &prMsgChnlReq->eChnlSco);
#else
		mtk_p2p_cfg80211func_channel_format_switch(chan,
							   NL80211_CHAN_NO_HT,
							   &prMsgChnlReq->rChannelInfo,
							   &prMsgChnlReq->eChnlSco);
#endif

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMsgChnlReq, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);


	return i4Rslt;
}



int mtk_p2p_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
					      struct wireless_dev *wdev,
#else
					      struct net_device *dev,
#endif
					      u64 cookie)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T) NULL;
	P_MSG_P2P_CHNL_ABORT_T prMsgChnlAbort = (P_MSG_P2P_CHNL_ABORT_T) NULL;

	do {
		if ((wiphy == NULL)
		    
		    ) {
			break;
		}


		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
		prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		if (prGlueP2pInfo->prDevHandler != dev) {
			ASSERT(FALSE);
			
			DBGLOG(P2P, WARN,
			       ("Group Interface also has remain on channel requirement\n"));
			break;
		}
#endif

		prMsgChnlAbort =
		    cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG, sizeof(MSG_P2P_CHNL_ABORT_T));

		if (prMsgChnlAbort == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		DBGLOG(P2P, TRACE,
		       ("mtk_p2p_cfg80211_cancel_remain_on_channel%d\n", (INT_32) cookie));

		prMsgChnlAbort->rMsgHdr.eMsgId = MID_MNY_P2P_CHNL_ABORT;
		prMsgChnlAbort->u8Cookie = cookie;


		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMsgChnlAbort, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;
}				

int mtk_p2p_cfg80211_mgmt_tx(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
			     struct wireless_dev *wdev,
#else
			     struct net_device *dev,
#endif
			     struct ieee80211_channel *chan, bool offchan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
			     enum nl80211_channel_type channel_type, bool channel_type_valid,
#endif
			     unsigned int wait, const u8 *buf, size_t len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
			     bool no_cck, bool dont_wait_for_ack,
#endif
			     u64 *cookie)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	P_GL_P2P_INFO_T prGlueP2pInfo = (P_GL_P2P_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_MGMT_TX_REQUEST_T prMsgTxReq = (P_MSG_P2P_MGMT_TX_REQUEST_T) NULL;
	P_MSDU_INFO_T prMgmtFrame = (P_MSDU_INFO_T) NULL;
	PUINT_8 pucFrameBuf = (PUINT_8) NULL;
	PUINT_64 pu8GlCookie = (PUINT_64) NULL;
	UINT_8 ucRoleIdx = 0, ucBssIdx = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	struct net_device *dev = NULL;
#endif

	do {
		if ((wiphy == NULL) || (buf == NULL) || (len == 0) ||
		    
		    (cookie == NULL)) {
			break;
		}

		DBGLOG(P2P, STATE, ("mtk_p2p_cfg80211_mgmt_tx\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));
		prGlueP2pInfo = prGlueInfo->prP2PInfo;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		dev = wdev->netdev;
#endif
		if (mtk_Netdev_To_RoleIdx(prGlueP2pInfo, dev, &ucRoleIdx) < 0) {
			
			ucBssIdx = P2P_DEV_BSS_INDEX;
		} else {
			ASSERT(ucRoleIdx < KAL_P2P_NUM);
			
			if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter, ucRoleIdx, &ucBssIdx) < 0) {
				
				break;
			}
		}

		*cookie = prGlueP2pInfo->u8Cookie++;

		
		prMsgTxReq =
		    cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
				sizeof(MSG_P2P_MGMT_TX_REQUEST_T));

		if (prMsgTxReq == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		if (offchan) {
			DBGLOG(P2P, TRACE, ("   Off channel TRUE\n"));
			prMsgTxReq->fgIsOffChannel = TRUE;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
			mtk_p2p_cfg80211func_channel_format_switch(chan,
								   channel_type,
								   &prMsgTxReq->rChannelInfo,
								   &prMsgTxReq->eChnlExt);
#else
			mtk_p2p_cfg80211func_channel_format_switch(chan,
								   NL80211_CHAN_NO_HT,
								   &prMsgTxReq->rChannelInfo,
								   &prMsgTxReq->eChnlExt);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
			if (!channel_type_valid) {
				prMsgTxReq->eChnlExt = CHNL_EXT_RES;
			}
#endif
		} else {
			prMsgTxReq->fgIsOffChannel = FALSE;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
		if (no_cck) {
			prMsgTxReq->fgNoneCckRate = TRUE;
		} else {
			prMsgTxReq->fgNoneCckRate = FALSE;
		}

		if (dont_wait_for_ack) {
			prMsgTxReq->fgIsWaitRsp = FALSE;
		} else {
			prMsgTxReq->fgIsWaitRsp = TRUE;
		}
#endif
		prMgmtFrame =
		    cnmMgtPktAlloc(prGlueInfo->prAdapter,
				   (UINT_32) (len + sizeof(UINT_64) + MAC_TX_RESERVED_FIELD));
		prMsgTxReq->prMgmtMsduInfo = prMgmtFrame;
		if (prMsgTxReq->prMgmtMsduInfo == NULL) {
			
			i4Rslt = -ENOMEM;
			break;
		}

		prMsgTxReq->u8Cookie = *cookie;
		prMsgTxReq->rMsgHdr.eMsgId = MID_MNY_P2P_MGMT_TX;
		prMsgTxReq->ucBssIdx = ucBssIdx;

		pucFrameBuf = (PUINT_8) ((ULONG) prMgmtFrame->prPacket + MAC_TX_RESERVED_FIELD);

		pu8GlCookie =
		    (PUINT_64) ((ULONG) prMgmtFrame->prPacket + (ULONG) len +
				MAC_TX_RESERVED_FIELD);

		kalMemCopy(pucFrameBuf, buf, len);

		*pu8GlCookie = *cookie;

		prMgmtFrame->u2FrameLength = len;

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMsgTxReq, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);

	if ((i4Rslt != 0) && (prMsgTxReq != NULL)) {
		if (prMsgTxReq->prMgmtMsduInfo != NULL) {
			cnmMgtPktFree(prGlueInfo->prAdapter, prMsgTxReq->prMgmtMsduInfo);
		}

		cnmMemFree(prGlueInfo->prAdapter, prMsgTxReq);
	}

	return i4Rslt;
}				


int mtk_p2p_cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
					 struct wireless_dev *wdev,
#else
					 struct net_device *dev,
#endif
					 u64 cookie)
{
	INT_32 i4Rslt = -EINVAL;

	return i4Rslt;
}				


int
mtk_p2p_cfg80211_change_bss(struct wiphy *wiphy,
			    struct net_device *dev, struct bss_parameters *params)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


	switch (params->use_cts_prot) {
	case -1:
		DBGLOG(P2P, TRACE, ("CTS protection no change\n"));
		break;
	case 0:
		DBGLOG(P2P, TRACE, ("CTS protection disable.\n"));
		break;
	case 1:
		DBGLOG(P2P, TRACE, ("CTS protection enable\n"));
		break;
	default:
		DBGLOG(P2P, TRACE, ("CTS protection unknown\n"));
		break;
	}



	switch (params->use_short_preamble) {
	case -1:
		DBGLOG(P2P, TRACE, ("Short prreamble no change\n"));
		break;
	case 0:
		DBGLOG(P2P, TRACE, ("Short prreamble disable.\n"));
		break;
	case 1:
		DBGLOG(P2P, TRACE, ("Short prreamble enable\n"));
		break;
	default:
		DBGLOG(P2P, TRACE, ("Short prreamble unknown\n"));
		break;
	}



#if 0
	
	p2pFuncChangeBssParam(prGlueInfo->prAdapter,
			      prBssInfo->fgIsProtection,
			      prBssInfo->fgIsShortPreambleAllowed, prBssInfo->fgUseShortSlotTime,
			      
			      
			      
			      
	    );
#else
	i4Rslt = 0;
#endif

	return i4Rslt;
}				



int mtk_p2p_cfg80211_del_station(struct wiphy *wiphy, struct net_device *dev, u8 *mac)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_CONNECTION_ABORT_T prDisconnectMsg = (P_MSG_P2P_CONNECTION_ABORT_T) NULL;
	UINT_8 aucBcMac[] = BC_MAC_ADDR;
	UINT_8 ucRoleIdx = 0;


	do {
		if ((wiphy == NULL) || (dev == NULL)) {
			break;
		}

		if (mac == NULL) {
			mac = aucBcMac;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_del_station.\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}
		
		prDisconnectMsg =
		    (P_MSG_P2P_CONNECTION_ABORT_T) cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
							       sizeof(MSG_P2P_CONNECTION_ABORT_T));

		if (prDisconnectMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prDisconnectMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_ABORT;
		prDisconnectMsg->ucRoleIdx = ucRoleIdx;
		COPY_MAC_ADDR(prDisconnectMsg->aucTargetID, mac);
		prDisconnectMsg->u2ReasonCode = REASON_CODE_UNSPECIFIED;
		prDisconnectMsg->fgSendDeauth = TRUE;

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prDisconnectMsg, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;

}				


int
mtk_p2p_cfg80211_connect(struct wiphy *wiphy,
			 struct net_device *dev, struct cfg80211_connect_params *sme)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_MSG_P2P_CONNECTION_REQUEST_T prConnReqMsg = (P_MSG_P2P_CONNECTION_REQUEST_T) NULL;
	UINT_8 ucRoleIdx = 0;


	do {
		if ((wiphy == NULL) || (dev == NULL) || (sme == NULL)) {
			break;
		}

		DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_connect.\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}

		prConnReqMsg =
		    (P_MSG_P2P_CONNECTION_REQUEST_T) cnmMemAlloc(prGlueInfo->prAdapter,
								 RAM_TYPE_MSG,
								 (sizeof
								  (MSG_P2P_CONNECTION_REQUEST_T) +
								  sme->ie_len));

		if (prConnReqMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prConnReqMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_REQ;
		prConnReqMsg->ucRoleIdx = ucRoleIdx;

		COPY_SSID(prConnReqMsg->rSsid.aucSsid,
			  prConnReqMsg->rSsid.ucSsidLen, sme->ssid, sme->ssid_len);

		COPY_MAC_ADDR(prConnReqMsg->aucBssid, sme->bssid);

		DBGLOG(P2P, TRACE, ("Assoc Req IE Buffer Length:%d\n", sme->ie_len));
		kalMemCopy(prConnReqMsg->aucIEBuf, sme->ie, sme->ie_len);
		prConnReqMsg->u4IELen = sme->ie_len;

		kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_NONE);

		if (sme->crypto.n_ciphers_pairwise) {
			switch (sme->crypto.ciphers_pairwise[0]) {
			case WLAN_CIPHER_SUITE_WEP40:
			case WLAN_CIPHER_SUITE_WEP104:
				kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_WEP40);
				break;
			case WLAN_CIPHER_SUITE_TKIP:
				kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_TKIP);
				break;
			case WLAN_CIPHER_SUITE_CCMP:
			case WLAN_CIPHER_SUITE_AES_CMAC:
				kalP2PSetCipher(prGlueInfo, IW_AUTH_CIPHER_CCMP);
				break;
			default:
				DBGLOG(REQ, WARN, ("invalid cipher pairwise (%d)\n",
						   sme->crypto.ciphers_pairwise[0]));
				return -EINVAL;
			}
		}

		mtk_p2p_cfg80211func_channel_format_switch(sme->channel,
							   NL80211_CHAN_NO_HT,
							   &prConnReqMsg->rChannelInfo,
							   &prConnReqMsg->eChnlSco);

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prConnReqMsg, MSG_SEND_METHOD_BUF);


		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;
}				

int mtk_p2p_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev, u16 reason_code)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_MSG_P2P_CONNECTION_ABORT_T prDisconnMsg = (P_MSG_P2P_CONNECTION_ABORT_T) NULL;
	UINT_8 aucBCAddr[] = BC_MAC_ADDR;
	UINT_8 ucRoleIdx = 0;

	do {
		if ((wiphy == NULL) || (dev == NULL)) {
			break;
		}

		DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_disconnect.\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}
		prDisconnMsg =
		    (P_MSG_P2P_CONNECTION_ABORT_T) cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
							       sizeof(MSG_P2P_CONNECTION_ABORT_T));

		if (prDisconnMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prDisconnMsg->rMsgHdr.eMsgId = MID_MNY_P2P_CONNECTION_ABORT;
		prDisconnMsg->ucRoleIdx = ucRoleIdx;
		prDisconnMsg->u2ReasonCode = reason_code;
		prDisconnMsg->fgSendDeauth = TRUE;
		COPY_MAC_ADDR(prDisconnMsg->aucTargetID, aucBCAddr);

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prDisconnMsg, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;
}				


int
mtk_p2p_cfg80211_change_iface(IN struct wiphy *wiphy,
			      IN struct net_device *ndev,
			      IN enum nl80211_iftype type,
			      IN u32 *flags, IN struct vif_params *params)
{
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	INT_32 i4Rslt = -EINVAL;
	P_MSG_P2P_SWITCH_OP_MODE_T prSwitchModeMsg = (P_MSG_P2P_SWITCH_OP_MODE_T) NULL;
	UINT_8 ucRoleIdx = 0;

	do {
		if ((wiphy == NULL) || (ndev == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_change_iface.\n"));

		if (ndev->ieee80211_ptr) {
			ndev->ieee80211_ptr->iftype = type;
		}

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, ndev, &ucRoleIdx) != 0) {
			DBGLOG(P2P, TRACE,
			       ("Device Interface no need to change interface type.\n"));
			return 0;
		}
		
		prSwitchModeMsg =
		    (P_MSG_P2P_SWITCH_OP_MODE_T) cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
							     sizeof(MSG_P2P_SWITCH_OP_MODE_T));

		if (prSwitchModeMsg == NULL) {
			ASSERT(FALSE);
			i4Rslt = -ENOMEM;
			break;
		}

		prSwitchModeMsg->rMsgHdr.eMsgId = MID_MNY_P2P_FUN_SWITCH;
		prSwitchModeMsg->ucRoleIdx = ucRoleIdx;

		switch (type) {
		case NL80211_IFTYPE_P2P_CLIENT:
			DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_P2P_CLIENT.\n"));
		case NL80211_IFTYPE_STATION:
			if (type == NL80211_IFTYPE_STATION) {
				DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_STATION.\n"));
			}
			prSwitchModeMsg->eOpMode = OP_MODE_INFRASTRUCTURE;
			kalP2PSetRole(prGlueInfo, 1);
			break;
		case NL80211_IFTYPE_AP:
			DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_AP.\n"));
			kalP2PSetRole(prGlueInfo, 2);
		case NL80211_IFTYPE_P2P_GO:
			if (type == NL80211_IFTYPE_P2P_GO) {
				DBGLOG(P2P, TRACE, ("NL80211_IFTYPE_P2P_GO not AP.\n"));
			}
			prSwitchModeMsg->eOpMode = OP_MODE_ACCESS_POINT;
			kalP2PSetRole(prGlueInfo, 2);
			break;
		default:
			DBGLOG(P2P, TRACE, ("Other type :%d .\n", type));
			prSwitchModeMsg->eOpMode = OP_MODE_P2P_DEVICE;
			kalP2PSetRole(prGlueInfo, 0);
			break;
		}


		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prSwitchModeMsg, MSG_SEND_METHOD_BUF);

		i4Rslt = 0;

	} while (FALSE);

	return i4Rslt;

}				


int mtk_p2p_cfg80211_set_channel(IN struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
				 struct cfg80211_chan_def *chandef
#else
				 IN struct net_device *dev,
				 IN struct ieee80211_channel *chan,
				 IN enum nl80211_channel_type channel_type
#endif
    )
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;
	RF_CHANNEL_INFO_T rRfChnlInfo;
	UINT_8 ucRoleIdx = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	struct net_device *dev = (struct net_device *)wiphy_dev(wiphy);
#endif

	do {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		if ((wiphy == NULL) || (chandef == NULL)) {
			break;
		}
#else
		if ((wiphy == NULL) || (dev == NULL) || (chan == NULL)) {
			break;
		}
#endif
		DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_set_channel.\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		mtk_p2p_cfg80211func_channel_format_switch(chan, channel_type, &rRfChnlInfo, NULL);

#else
		mtk_p2p_cfg80211func_channel_format_switch(chandef->chan,
							   chandef->width, &rRfChnlInfo, NULL);
#endif
		if (mtk_Netdev_To_RoleIdx(prGlueInfo->prP2PInfo, dev, &ucRoleIdx) < 0) {
			break;
		}

		p2pFuncSetChannel(prGlueInfo->prAdapter, ucRoleIdx, &rRfChnlInfo);

		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;

}


int
mtk_p2p_cfg80211_set_bitrate_mask(IN struct wiphy *wiphy,
				  IN struct net_device *dev,
				  IN const u8 *peer, IN const struct cfg80211_bitrate_mask *mask)
{
	INT_32 i4Rslt = -EINVAL;
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;

	do {
		if ((wiphy == NULL) || (dev == NULL) || (mask == NULL)) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_set_bitrate_mask\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

		

		i4Rslt = 0;
	} while (FALSE);

	return i4Rslt;
}				


void mtk_p2p_cfg80211_mgmt_frame_register(IN struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
					  struct wireless_dev *wdev,
#else
					  IN struct net_device *dev,
#endif
					  IN u16 frame_type, IN bool reg)
{
#if 0
	P_MSG_P2P_MGMT_FRAME_REGISTER_T prMgmtFrameRegister =
	    (P_MSG_P2P_MGMT_FRAME_REGISTER_T) NULL;
#endif
	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) NULL;

	do {
		if ((wiphy == NULL)
		    
		    ) {
			break;
		}

		DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_mgmt_frame_register\n"));

		prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));


		switch (frame_type) {
		case MAC_FRAME_PROBE_REQ:
			if (reg) {
				prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter |=
				    PARAM_PACKET_FILTER_PROBE_REQ;
				DBGLOG(P2P, TRACE, ("Open packet filer probe request\n"));
			} else {
				prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter &=
				    ~PARAM_PACKET_FILTER_PROBE_REQ;
				DBGLOG(P2P, TRACE, ("Close packet filer probe request\n"));
			}
			break;
		case MAC_FRAME_ACTION:
			if (reg) {
				prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter |=
				    PARAM_PACKET_FILTER_ACTION_FRAME;
				DBGLOG(P2P, TRACE, ("Open packet filer action frame.\n"));
			} else {
				prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter &=
				    ~PARAM_PACKET_FILTER_ACTION_FRAME;
				DBGLOG(P2P, TRACE, ("Close packet filer action frame.\n"));
			}
			break;
		default:
			DBGLOG(P2P, ERROR, ("Ask frog to add code for mgmt:%x\n", frame_type));
			break;
		}



		set_bit(GLUE_FLAG_FRAME_FILTER_BIT, &prGlueInfo->ulFlag);

		
		wake_up_interruptible(&prGlueInfo->waitq);

		if (in_interrupt()) {
			DBGLOG(P2P, TRACE, ("It is in interrupt level\n"));
		}


#if 0


		prMgmtFrameRegister =
		    (P_MSG_P2P_MGMT_FRAME_REGISTER_T) cnmMemAlloc(prGlueInfo->prAdapter,
								  RAM_TYPE_MSG,
								  sizeof
								  (MSG_P2P_MGMT_FRAME_REGISTER_T));

		if (prMgmtFrameRegister == NULL) {
			ASSERT(FALSE);
			break;
		}

		prMgmtFrameRegister->rMsgHdr.eMsgId = MID_MNY_P2P_MGMT_FRAME_REGISTER;

		prMgmtFrameRegister->u2FrameType = frame_type;
		prMgmtFrameRegister->fgIsRegister = reg;

		mboxSendMsg(prGlueInfo->prAdapter,
			    MBOX_ID_0, (P_MSG_HDR_T) prMgmtFrameRegister, MSG_SEND_METHOD_BUF);

#endif

	} while (FALSE);


	return;
}				


BOOLEAN
mtk_p2p_cfg80211func_channel_format_switch(IN struct ieee80211_channel *channel,
					   IN enum nl80211_channel_type channel_type,
					   IN P_RF_CHANNEL_INFO_T prRfChnlInfo,
					   IN P_ENUM_CHNL_EXT_T prChnlSco)
{
	BOOLEAN fgIsValid = FALSE;

	do {
		if (channel == NULL) {
			break;
		}

		if (prRfChnlInfo) {
			prRfChnlInfo->ucChannelNum =
			    nicFreq2ChannelNum(channel->center_freq * 1000);

			switch (channel->band) {
			case IEEE80211_BAND_2GHZ:
				prRfChnlInfo->eBand = BAND_2G4;
				break;
			case IEEE80211_BAND_5GHZ:
				prRfChnlInfo->eBand = BAND_5G;
				break;
			default:
				prRfChnlInfo->eBand = BAND_2G4;
				break;
			}

		}


		if (prChnlSco) {

			switch (channel_type) {
			case NL80211_CHAN_NO_HT:
				*prChnlSco = CHNL_EXT_SCN;
				break;
			case NL80211_CHAN_HT20:
				*prChnlSco = CHNL_EXT_SCN;
				break;
			case NL80211_CHAN_HT40MINUS:
				*prChnlSco = CHNL_EXT_SCA;
				break;
			case NL80211_CHAN_HT40PLUS:
				*prChnlSco = CHNL_EXT_SCB;
				break;
			default:
				ASSERT(FALSE);
				*prChnlSco = CHNL_EXT_SCN;
				break;
			}
		}

		fgIsValid = TRUE;

	} while (FALSE);

	return fgIsValid;
}



#if CONFIG_NL80211_TESTMODE
int mtk_p2p_cfg80211_testmode_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_NL80211_DRIVER_TEST_PARAMS prParams = (P_NL80211_DRIVER_TEST_PARAMS) NULL;
	INT_32 i4Status = -EINVAL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_testmode_cmd\n"));

	if (data && len) {
		prParams = (P_NL80211_DRIVER_TEST_PARAMS) data;
	} else {
		DBGLOG(P2P, ERROR, ("mtk_p2p_cfg80211_testmode_cmd, data is NULL\n"));
		return i4Status;
	}
	if (prParams->index >> 24 == 0x01) {
		
		prParams->index = prParams->index & ~BITS(24, 31);
	} else {
		
		mtk_p2p_cfg80211_testmode_p2p_sigma_pre_cmd(wiphy, data, len);
		i4Status = 0;
		return i4Status;
	}

	
	prParams->index = prParams->index & ~BITS(24, 31);

	if (prParams) {
		switch (prParams->index) {
		case 1:	
#if CFG_SUPPORT_HOTSPOT_OPTIMIZATION
			{
				P_NL80211_DRIVER_SW_CMD_PARAMS prParamsCmd;
				prParamsCmd = (P_NL80211_DRIVER_SW_CMD_PARAMS) data;

				if ((prParamsCmd->adr & 0xffff0000) == 0xffff0000) {
					i4Status =
					    mtk_p2p_cfg80211_testmode_sw_cmd(wiphy, data, len);
					break;
				}
			}
#endif
			i4Status = mtk_p2p_cfg80211_testmode_p2p_sigma_cmd(wiphy, data, len);
			break;
		case 2:	
#if CFG_SUPPORT_WFD
			
			
#endif
			break;
		case 3:	
#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
			i4Status =
			    mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd(wiphy, data, len);
#endif
			break;
		case 0x10:
			i4Status =
			    mtk_cfg80211_testmode_get_sta_statistics(wiphy, data, len, prGlueInfo);
			break;
#if CFG_SUPPORT_NFC_BEAM_PLUS
		case 0x11:	
			if (data && len) {
				P_NL80211_DRIVER_SET_NFC_PARAMS prParams =
				    (P_NL80211_DRIVER_SET_NFC_PARAMS) data;
				DBGLOG(P2P, INFO, ("NFC: BEAM[%d]\n", prParams->NFC_Enable));
			}
			break;
		case 0x12:	
			DBGLOG(P2P, INFO, ("NFC: Polling\n"));
			i4Status =
			    mtk_cfg80211_testmode_get_scan_done(wiphy, data, len, prGlueInfo);
			break;
#endif
#if CFG_AUTO_CHANNEL_SEL_SUPPORT
		case 0x30:
			i4Status =
			    mtk_cfg80211_testmode_get_lte_channel(wiphy, data, len, prGlueInfo);
			break;
#endif

		default:
			i4Status = -EINVAL;
			break;
		}
	}

	return i4Status;

}


int mtk_p2p_cfg80211_testmode_p2p_sigma_pre_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	NL80211_DRIVER_TEST_PRE_PARAMS rParams;
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	
	UINT_32 index_mode;
	UINT_32 index;
	INT_32 value;
	int status = 0;
	UINT_32 u4Leng;
	UINT_8 ucBssIdx;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	kalMemZero(&rParams, sizeof(NL80211_DRIVER_TEST_PRE_PARAMS));

	prP2pSpecificBssInfo = prGlueInfo->prAdapter->rWifiVar.prP2pSpecificBssInfo;
	

	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_cmd\n"));

	if (data && len)
		memcpy(&rParams, data, len);

	DBGLOG(P2P, TRACE, ("NL80211_ATTR_TESTDATA,idx_mode=%d idx=%d value=%lu\n",
			    (INT_16) rParams.idx_mode, (INT_16) rParams.idx, rParams.value));

	index_mode = rParams.idx_mode;
	index = rParams.idx;
	value = rParams.value;

	
	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter, 0, &ucBssIdx) != WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	switch (index) {
	case 0:		
		break;
	case 1:		
		break;
	case 4:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaDurationMs = value;
		
		
		break;
	case 5:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaIntervalMs = value;
		
		
		break;
	case 6:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaCount = value;
		
		break;
	case 100:		
		
		
		break;
	case 101:		
		
		break;
	case 102:		
		
		
		
		
		break;
	case 103:		
		kalP2PSetWscMode(prGlueInfo, value);
		break;
	case 104:		
		break;
	case 105:		
		break;
	case 106:		
		{
			PARAM_POWER_MODE_T rPowerMode;

			rPowerMode.ePowerMode = Param_PowerModeMAX_PSP;
			rPowerMode.ucBssIdx = ucBssIdx;

			kalIoctl(prGlueInfo,
				 wlanoidSet802dot11PowerSaveProfile,
				 &rPowerMode, sizeof(rPowerMode), FALSE, FALSE, TRUE, &u4Leng);
		}
		break;
	case 107:		
		prP2pSpecificBssInfo->rOppPsParam.u4CTwindowMs = value;
		
		break;
	case 108:		
		{
			PARAM_POWER_MODE_T rPowerMode;

			rPowerMode.ePowerMode = value;
			rPowerMode.ucBssIdx = ucBssIdx;

			kalIoctl(prGlueInfo,
				 wlanoidSet802dot11PowerSaveProfile,
				 &rPowerMode, sizeof(rPowerMode), FALSE, FALSE, TRUE, &u4Leng);
		}
		break;
	default:
		break;
	}

	return status;

}


int mtk_p2p_cfg80211_testmode_p2p_sigma_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_NL80211_DRIVER_P2P_SIGMA_PARAMS prParams = (P_NL80211_DRIVER_P2P_SIGMA_PARAMS) NULL;
	P_P2P_SPECIFIC_BSS_INFO_T prP2pSpecificBssInfo = (P_P2P_SPECIFIC_BSS_INFO_T) NULL;
	
	UINT_32 index;
	INT_32 value;
	int status = 0;
	UINT_32 u4Leng;
	UINT_8 ucBssIdx;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	prP2pSpecificBssInfo = prGlueInfo->prAdapter->rWifiVar.prP2pSpecificBssInfo;
	

	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_p2p_sigma_cmd\n"));

	if (data && len)
		prParams = (P_NL80211_DRIVER_P2P_SIGMA_PARAMS) data;

	index = (INT_32) prParams->idx;
	value = (INT_32) prParams->value;

	DBGLOG(P2P, TRACE, ("NL80211_ATTR_TESTDATA, idx=%lu value=%lu\n",
			    (INT_32) prParams->idx, (INT_32) prParams->value));

	
	if (p2pFuncRoleToBssIdx(prGlueInfo->prAdapter, 0, &ucBssIdx) != WLAN_STATUS_SUCCESS) {
		return -EINVAL;
	}

	switch (index) {
	case 0:		
		break;
	case 1:		
		break;
	case 4:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaDurationMs = value;
		
		
		break;
	case 5:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaIntervalMs = value;
		
		
		break;
	case 6:		
		prP2pSpecificBssInfo->rNoaParam.u4NoaCount = value;
		
		break;
	case 100:		
		
		
		break;
	case 101:		
		
		break;
	case 102:		
		
		
		break;
	case 103:		
		kalP2PSetWscMode(prGlueInfo, value);
		break;
	case 104:		
		break;
	case 105:		
		break;
	case 106:		
		{
			PARAM_POWER_MODE_T rPowerMode;

			rPowerMode.ePowerMode = Param_PowerModeMAX_PSP;
			rPowerMode.ucBssIdx = ucBssIdx;

			kalIoctl(prGlueInfo,
				 wlanoidSet802dot11PowerSaveProfile,
				 &rPowerMode, sizeof(rPowerMode), FALSE, FALSE, TRUE, &u4Leng);
		}
		break;
	case 107:		
		prP2pSpecificBssInfo->rOppPsParam.u4CTwindowMs = value;
		
		break;
	case 108:		
		{
			PARAM_POWER_MODE_T rPowerMode;

			rPowerMode.ePowerMode = value;
			rPowerMode.ucBssIdx = ucBssIdx;

			kalIoctl(prGlueInfo,
				 wlanoidSet802dot11PowerSaveProfile,
				 &rPowerMode, sizeof(rPowerMode), FALSE, FALSE, TRUE, &u4Leng);
		}
		break;
	case 109:		
#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
		kalP2PSetMaxClients(prGlueInfo, value);
#endif
		break;
	case 110:		
#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER
		kalIoctl(prGlueInfo,
			 wlanoidSetP2pWPSmode, &value, sizeof(value), FALSE, FALSE, TRUE, &u4Leng);
#endif
		break;
	default:
		break;
	}

	return status;

}

#if CFG_SUPPORT_WFD && 0
int mtk_p2p_cfg80211_testmode_wfd_update_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_NL80211_DRIVER_WFD_PARAMS prParams = (P_NL80211_DRIVER_WFD_PARAMS) NULL;
	int status = 0;
	P_WFD_CFG_SETTINGS_T prWfdCfgSettings = (P_WFD_CFG_SETTINGS_T) NULL;
	P_MSG_WFD_CONFIG_SETTINGS_CHANGED_T prMsgWfdCfgUpdate =
	    (P_MSG_WFD_CONFIG_SETTINGS_CHANGED_T) NULL;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	prParams = (P_NL80211_DRIVER_WFD_PARAMS) data;


	DBGLOG(P2P, INFO, ("mtk_p2p_cfg80211_testmode_wfd_update_cmd\n"));

#if 1

	DBGLOG(P2P, INFO, ("WFD Enable:%x\n", prParams->WfdEnable));
	DBGLOG(P2P, INFO, ("WFD Session Available:%x\n", prParams->WfdSessionAvailable));
	DBGLOG(P2P, INFO, ("WFD Couple Sink Status:%x\n", prParams->WfdCoupleSinkStatus));
	
	DBGLOG(P2P, INFO, ("WFD Device Info:%x\n", prParams->WfdDevInfo));
	DBGLOG(P2P, INFO, ("WFD Control Port:%x\n", prParams->WfdControlPort));
	DBGLOG(P2P, INFO, ("WFD Maximum Throughput:%x\n", prParams->WfdMaximumTp));
	DBGLOG(P2P, INFO, ("WFD Extend Capability:%x\n", prParams->WfdExtendCap));
	DBGLOG(P2P, INFO,
	       ("WFD Couple Sink Addr " MACSTR "\n", MAC2STR(prParams->WfdCoupleSinkAddress)));
	DBGLOG(P2P, INFO,
	       ("WFD Associated BSSID " MACSTR "\n", MAC2STR(prParams->WfdAssociatedBssid)));
	
	
	DBGLOG(P2P, INFO, ("WFD Video Port:%x\n", prParams->WfdVideoPort));
	DBGLOG(P2P, INFO, ("WFD Audio Port:%x\n", prParams->WfdAudioPort));
	DBGLOG(P2P, INFO, ("WFD Flag:%x\n", prParams->WfdFlag));
	DBGLOG(P2P, INFO, ("WFD Policy:%x\n", prParams->WfdPolicy));
	DBGLOG(P2P, INFO, ("WFD State:%x\n", prParams->WfdState));
	
	DBGLOG(P2P, INFO, ("WFD Session Info Length:%x\n", prParams->WfdSessionInformationIELen));
	
	DBGLOG(P2P, INFO,
	       ("WFD Primary Sink Addr " MACSTR "\n", MAC2STR(prParams->aucWfdPrimarySinkMac)));
	DBGLOG(P2P, INFO,
	       ("WFD Secondary Sink Addr " MACSTR "\n",
		MAC2STR(prParams->aucWfdSecondarySinkMac)));
	DBGLOG(P2P, INFO, ("WFD Advanced Flag:%x\n", prParams->WfdAdvanceFlag));
	DBGLOG(P2P, INFO, ("WFD Sigma mode:%x\n", prParams->WfdSigmaMode));
	
	
	

#endif

	prWfdCfgSettings = &(prGlueInfo->prAdapter->rWifiVar.rWfdConfigureSettings);

	kalMemCopy(&prWfdCfgSettings->u4WfdCmdType, &prParams->WfdCmdType,
		   sizeof(WFD_CFG_SETTINGS_T));

	prMsgWfdCfgUpdate =
	    cnmMemAlloc(prGlueInfo->prAdapter, RAM_TYPE_MSG,
			sizeof(MSG_WFD_CONFIG_SETTINGS_CHANGED_T));

	if (prMsgWfdCfgUpdate == NULL) {
		ASSERT(FALSE);
		return status;
	}

	prMsgWfdCfgUpdate->rMsgHdr.eMsgId = MID_MNY_P2P_WFD_CFG_UPDATE;
	prMsgWfdCfgUpdate->prWfdCfgSettings = prWfdCfgSettings;


	mboxSendMsg(prGlueInfo->prAdapter,
		    MBOX_ID_0, (P_MSG_HDR_T) prMsgWfdCfgUpdate, MSG_SEND_METHOD_BUF);
#if 0				
	prWfdCfgSettings->u4WfdFlag |= WFD_FLAGS_DEV_INFO_VALID;
	prWfdCfgSettings->u2WfdDevInfo = 123;
	prWfdCfgSettings->u2WfdControlPort = 456;
	prWfdCfgSettings->u2WfdMaximumTp = 789;


	prWfdCfgSettings->u4WfdFlag |= WFD_FLAGS_SINK_INFO_VALID;
	prWfdCfgSettings->ucWfdCoupleSinkStatus = 0xAB;
	{
		UINT_8 aucTestAddr[MAC_ADDR_LEN] = { 0x77, 0x66, 0x55, 0x44, 0x33, 0x22 };
		COPY_MAC_ADDR(prWfdCfgSettings->aucWfdCoupleSinkAddress, aucTestAddr);
	}

	prWfdCfgSettings->u4WfdFlag |= WFD_FLAGS_EXT_CAPABILITY_VALID;
	prWfdCfgSettings->u2WfdExtendCap = 0xCDE;

#endif

	return status;

}
#endif				

#if CFG_SUPPORT_HOTSPOT_WPS_MANAGER

int
mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_NL80211_DRIVER_hotspot_block_PARAMS prParams =
	    (P_NL80211_DRIVER_hotspot_block_PARAMS) NULL;
	int fgIsValid = 0;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

	if (data && len)
		prParams = (P_NL80211_DRIVER_hotspot_block_PARAMS) data;

	DBGLOG(P2P, TRACE, ("mtk_p2p_cfg80211_testmode_hotspot_block_list_cmd\n"));

	fgIsValid = kalP2PSetBlackList(prGlueInfo, prParams->aucBssid, prParams->ucblocked);

	return fgIsValid;

}

#endif

int mtk_p2p_cfg80211_testmode_sw_cmd(IN struct wiphy *wiphy, IN void *data, IN int len)
{
	P_GLUE_INFO_T prGlueInfo = NULL;
	P_NL80211_DRIVER_SW_CMD_PARAMS prParams = (P_NL80211_DRIVER_SW_CMD_PARAMS) NULL;
	WLAN_STATUS rstatus = WLAN_STATUS_SUCCESS;
	int fgIsValid = 0;
	UINT_32 u4SetInfoLen = 0;

	ASSERT(wiphy);

	prGlueInfo = *((P_GLUE_INFO_T *) wiphy_priv(wiphy));

#if 1
	printk("--> %s()\n", __func__);
#endif

	if (data && len)
		prParams = (P_NL80211_DRIVER_SW_CMD_PARAMS) data;

	if (prParams) {
		if (prParams->set == 1) {
			rstatus = kalIoctl(prGlueInfo,
					   (PFN_OID_HANDLER_FUNC) wlanoidSetSwCtrlWrite,
					   &prParams->adr,
					   (UINT_32) 8, FALSE, FALSE, TRUE, &u4SetInfoLen);
		}
	}

	if (WLAN_STATUS_SUCCESS != rstatus) {
		fgIsValid = -EFAULT;
	}

	return fgIsValid;
}


#endif


#endif				

#endif				
