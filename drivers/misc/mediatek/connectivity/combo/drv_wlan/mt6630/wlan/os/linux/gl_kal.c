





#include "gl_os.h"
#include "gl_kal.h"
#include "gl_wext.h"
#include "precomp.h"
#if CFG_SUPPORT_AGPS_ASSIST
#include <net/netlink.h>
#endif



#if DBG
int allocatedMemSize = 0;
#endif


extern struct delayed_work sched_workq;

static PVOID pvIoBuffer;
static UINT_32 pvIoBufferSize;
static UINT_32 pvIoBufferUsage;





#if CFG_ENABLE_FW_DOWNLOAD

static struct file *filp;
static uid_t orgfsuid;
static gid_t orgfsgid;
static mm_segment_t orgfs;

static PUINT_8 apucFwPath[] = { 
	(PUINT_8) "/storage/sdcard0/",
	(PUINT_8) "/etc/firmware/",
	NULL
};

static PUINT_8 apucFwNameE2[] = {
	(PUINT_8) CFG_FW_FILENAME "_MT6630_E2",
	(PUINT_8) CFG_FW_FILENAME "_MT6630",
	NULL
};

static PUINT_8 apucFwNameE3[] = {
	(PUINT_8) CFG_FW_FILENAME "_MT6630_E3",
	(PUINT_8) CFG_FW_FILENAME "_MT6630",
	(PUINT_8) CFG_FW_FILENAME "_MT6630_E2",
	NULL
};

static PUINT_8 apucFwName[] = {
	(PUINT_8) CFG_FW_FILENAME "_MT6630",
	(PUINT_8) CFG_FW_FILENAME "_MT6630_E2",
	(PUINT_8) CFG_FW_FILENAME "_MT6630_E3",
	NULL
};

static PPUINT_8 appucFwNameTable[] = {
	apucFwName,
	apucFwNameE2,
	apucFwNameE3,
	apucFwNameE3,
};

WLAN_STATUS kalFirmwareOpen(IN P_GLUE_INFO_T prGlueInfo)
{
	UINT_8 ucPathIdx, ucNameIdx;
	PPUINT_8 apucNameTable;
	UINT_8 ucMaxEcoVer = (sizeof(appucFwNameTable) / sizeof(PPUINT_8));
	UINT_8 ucCurEcoVer = wlanGetEcoVersion(prGlueInfo->prAdapter);
	UINT_8 aucFwName[128];
	BOOLEAN fgResult = FALSE;
	

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
	orgfsuid = current->fsuid;
	orgfsgid = current->fsgid;
	current->fsuid = current->fsgid = 0;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	struct cred *cred = (struct cred *)get_current_cred();
	orgfsuid = cred->fsuid;
	orgfsgid = cred->fsgid;
	cred->fsuid = cred->fsgid = 0;
#else
	struct cred *cred = get_task_cred(current);
	orgfsuid = cred->fsuid;
	orgfsgid = cred->fsgid;
	cred->fsuid = cred->fsgid = 0;
#endif

	ASSERT(prGlueInfo);


	orgfs = get_fs();
	set_fs(get_ds());

	
	if(ucMaxEcoVer < ucCurEcoVer) {
		apucNameTable = apucFwName;
	}
	else {
		apucNameTable = appucFwNameTable[ucCurEcoVer - 1];
	}

	
	for(ucPathIdx = 0; apucFwPath[ucPathIdx]; ucPathIdx++) {
		for(ucNameIdx = 0; apucNameTable[ucNameIdx]; ucNameIdx++) {
			
			kalSprintf(aucFwName, "%s%s", apucFwPath[ucPathIdx], 
				apucNameTable[ucNameIdx]);
			
			filp = filp_open(aucFwName, O_RDONLY, 0);
			if (IS_ERR(filp)) {
				DBGLOG(INIT, TRACE, ("Open FW image: %s failed, errno[%d]\n", 
					aucFwName, ERR_PTR((LONG)filp)));
				continue;
			}
			else {
				DBGLOG(INIT, TRACE, ("Open FW image: %s done\n", aucFwName));
				fgResult = TRUE;
				break;
			}
		}
		
		if(fgResult) {
			break;
		}
	}

	
	if(fgResult) {
		DBGLOG(INIT, INFO, ("Open FW image: %s done\n", aucFwName));
	}
	else {
		DBGLOG(INIT, ERROR, ("Open FW image failed! Cur/Max ECO Ver[E%u/E%u]\n",
			ucCurEcoVer, ucMaxEcoVer));
        
        
    	for(ucPathIdx = 0; apucFwPath[ucPathIdx]; ucPathIdx++) {
    		for(ucNameIdx = 0; apucNameTable[ucNameIdx]; ucNameIdx++) {
    			
    			kalSprintf(aucFwName, "%s%s", apucFwPath[ucPathIdx], 
    				apucNameTable[ucNameIdx]);
    			
    			filp = filp_open(aucFwName, O_RDONLY, 0);
    			if (IS_ERR(filp)) {
    				DBGLOG(INIT, INFO, ("Open FW image: %s failed, errno[%d]\n", 
    					aucFwName, ERR_PTR((LONG)filp)));
    			}
    			else {
    				DBGLOG(INIT, INFO, ("Open FW image: %s done\n", aucFwName));
    			}
    		}
    	}
		goto error_open;
	}

	return WLAN_STATUS_SUCCESS;

error_open:
	
	set_fs(orgfs);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
	current->fsuid = orgfsuid;
	current->fsgid = orgfsgid;
#else
	cred->fsuid = orgfsuid;
	cred->fsgid = orgfsgid;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	put_cred(cred);
#endif
#endif
	return WLAN_STATUS_FAILURE;
}


WLAN_STATUS kalFirmwareClose(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	if ((filp != NULL) && !IS_ERR(filp)) {
		
		filp_close(filp, NULL);

		
		set_fs(orgfs);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
		current->fsuid = orgfsuid;
		current->fsgid = orgfsgid;
#else
		{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
			struct cred *cred = (struct cred *)get_current_cred();
#else
			struct cred *cred = get_task_cred(current);
#endif
			cred->fsuid = orgfsuid;
			cred->fsgid = orgfsgid;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
			put_cred(cred);
#endif
		}
#endif
		filp = NULL;
	}

	return WLAN_STATUS_SUCCESS;
}

WLAN_STATUS
kalFirmwareLoad(IN P_GLUE_INFO_T prGlueInfo,
		OUT PVOID prBuf, IN UINT_32 u4Offset, OUT PUINT_32 pu4Size)
{
	ASSERT(prGlueInfo);
	ASSERT(pu4Size);
	ASSERT(prBuf);

	

	
	if ((filp == NULL) || IS_ERR(filp) || (filp->f_op == NULL) || (filp->f_op->read == NULL)) {
		goto error_read;
	} else {
		filp->f_pos = u4Offset;
		*pu4Size = filp->f_op->read(filp, prBuf, *pu4Size, &filp->f_pos);
	}

	return WLAN_STATUS_SUCCESS;

error_read:
	return WLAN_STATUS_FAILURE;
}


WLAN_STATUS kalFirmwareSize(IN P_GLUE_INFO_T prGlueInfo, OUT PUINT_32 pu4Size)
{
	ASSERT(prGlueInfo);
	ASSERT(pu4Size);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 12)
	*pu4Size = filp->f_path.dentry->d_inode->i_size;
#else
	*pu4Size = filp->f_dentry->d_inode->i_size;
#endif

	return WLAN_STATUS_SUCCESS;
}



PVOID
kalFirmwareImageMapping(IN P_GLUE_INFO_T prGlueInfo,
			OUT PPVOID ppvMapFileBuf, OUT PUINT_32 pu4FileLength)
{
	DEBUGFUNC("kalFirmwareImageMapping");

	ASSERT(prGlueInfo);
	ASSERT(ppvMapFileBuf);
	ASSERT(pu4FileLength);

	do {
		
		if (kalFirmwareOpen(prGlueInfo) != WLAN_STATUS_SUCCESS) {
			break;
		} else {
			UINT_32 u4FwSize = 0;
			PVOID prFwBuffer = NULL;
			
			kalFirmwareSize(prGlueInfo, &u4FwSize);
			
			prFwBuffer = vmalloc(ALIGN_4(u4FwSize));
			
			if (kalFirmwareLoad(prGlueInfo, prFwBuffer, 0, &u4FwSize) !=
			    WLAN_STATUS_SUCCESS) {
				vfree(prFwBuffer);
				kalFirmwareClose(prGlueInfo);
				break;
			}
			
			*pu4FileLength = u4FwSize;
			*ppvMapFileBuf = prFwBuffer;

			return prFwBuffer;
		}

	} while (FALSE);

	return NULL;
}


VOID
kalFirmwareImageUnmapping(IN P_GLUE_INFO_T prGlueInfo, IN PVOID prFwHandle, IN PVOID pvMapFileBuf)
{
	DEBUGFUNC("kalFirmwareImageUnmapping");

	ASSERT(prGlueInfo);

	
	if (pvMapFileBuf) {
		vfree(pvMapFileBuf);
	}

	kalFirmwareClose(prGlueInfo);
}

#endif

#if 0


PVOID
kalFirmwareImageMapping(IN P_GLUE_INFO_T prGlueInfo,
			OUT PPVOID ppvMapFileBuf, OUT PUINT_32 pu4FileLength)
{
	INT_32 i4Ret = 0;

	DEBUGFUNC("kalFirmwareImageMapping");

	ASSERT(prGlueInfo);
	ASSERT(ppvMapFileBuf);
	ASSERT(pu4FileLength);

	do {
		GL_HIF_INFO_T *prHifInfo = &prGlueInfo->rHifInfo;
		prGlueInfo->prFw = NULL;

		
		i4Ret = request_firmware(&prGlueInfo->prFw, CFG_FW_FILENAME, &prHifInfo->func->dev);

		if (i4Ret) {
			printk(KERN_INFO DRV_NAME "fw %s:request failed %d\n", CFG_FW_FILENAME,
			       i4Ret);
			break;
		} else {
			*pu4FileLength = prGlueInfo->prFw->size;
			*ppvMapFileBuf = prGlueInfo->prFw->data;
			return prGlueInfo->prFw->data;
		}

	} while (FALSE);

	return NULL;
}


VOID
kalFirmwareImageUnmapping(IN P_GLUE_INFO_T prGlueInfo, IN PVOID prFwHandle, IN PVOID pvMapFileBuf)
{
	DEBUGFUNC("kalFirmwareImageUnmapping");

	ASSERT(prGlueInfo);
	ASSERT(pvMapFileBuf);

	release_firmware(prGlueInfo->prFw);

}
#endif

VOID
kalAcquireSpinLock(IN P_GLUE_INFO_T prGlueInfo,
		   IN ENUM_SPIN_LOCK_CATEGORY_E rLockCategory, OUT PULONG plFlags)
{
	ULONG ulFlags = 0;

	ASSERT(prGlueInfo);
	ASSERT(plFlags);

	if (rLockCategory < SPIN_LOCK_NUM) {

#if CFG_USE_SPIN_LOCK_BOTTOM_HALF
		spin_lock_bh(&prGlueInfo->rSpinLock[rLockCategory]);
#else				
		spin_lock_irqsave(&prGlueInfo->rSpinLock[rLockCategory], ulFlags);
#endif				

		*plFlags = ulFlags;
	}

	return;
}				


VOID
kalReleaseSpinLock(IN P_GLUE_INFO_T prGlueInfo,
		   IN ENUM_SPIN_LOCK_CATEGORY_E rLockCategory, IN ULONG ulFlags)
{
	ASSERT(prGlueInfo);

	if (rLockCategory < SPIN_LOCK_NUM) {

#if CFG_USE_SPIN_LOCK_BOTTOM_HALF
		spin_unlock_bh(&prGlueInfo->rSpinLock[rLockCategory]);
#else				
		spin_unlock_irqrestore(&prGlueInfo->rSpinLock[rLockCategory], ulFlags);
#endif				

	}

	return;
}				

VOID
kalAcquireMutex (
    IN P_GLUE_INFO_T                prGlueInfo,
    IN ENUM_MUTEX_CATEGORY_E        rMutexCategory
    )
{
    ASSERT(prGlueInfo);

    if (rMutexCategory < MUTEX_NUM) {
        DBGLOG(INIT, TRACE, ("MUTEX_LOCK[%u] Try to acquire\n", rMutexCategory));
        mutex_lock(&prGlueInfo->arMutex[rMutexCategory]);
        DBGLOG(INIT, TRACE, ("MUTEX_LOCK[%u] Acquired\n", rMutexCategory));
    }

    return;
} 


VOID
kalReleaseMutex (
    IN P_GLUE_INFO_T                prGlueInfo,
    IN ENUM_MUTEX_CATEGORY_E        rMutexCategory
    )
{
    ASSERT(prGlueInfo);

    if (rMutexCategory < MUTEX_NUM) {
        mutex_unlock(&prGlueInfo->arMutex[rMutexCategory]);
        DBGLOG(INIT, TRACE, ("MUTEX_UNLOCK[%u]\n", rMutexCategory));
    }

    return;
} 


VOID kalUpdateMACAddress(IN P_GLUE_INFO_T prGlueInfo, IN PUINT_8 pucMacAddr)
{
	ASSERT(prGlueInfo);
	ASSERT(pucMacAddr);

	if (UNEQUAL_MAC_ADDR(prGlueInfo->prDevHandler->dev_addr, pucMacAddr)) {
		memcpy(prGlueInfo->prDevHandler->dev_addr, pucMacAddr, PARAM_MAC_ADDR_LEN);
	}

	return;
}


#if CFG_TCP_IP_CHKSUM_OFFLOAD
VOID kalQueryTxChksumOffloadParam(IN PVOID pvPacket, OUT PUINT_8 pucFlag)
{
	struct sk_buff *skb = (struct sk_buff *)pvPacket;
	UINT_8 ucFlag = 0;

	ASSERT(pvPacket);
	ASSERT(pucFlag);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	if (skb->ip_summed == CHECKSUM_HW)
#else
	if (skb->ip_summed == CHECKSUM_PARTIAL)
#endif
	{

#if DBG
		if (skb->protocol != __constant_htons(ETH_P_IP)) {
			
		} else
#endif
			ucFlag |= (TX_CS_IP_GEN | TX_CS_TCP_UDP_GEN);
	}

	*pucFlag = ucFlag;

	return;
}				


/* 4 2007/10/8, mikewu, this is rewritten by Mike */
VOID kalUpdateRxCSUMOffloadParam(IN PVOID pvPacket, IN ENUM_CSUM_RESULT_T aeCSUM[]
    )
{
	struct sk_buff *skb = (struct sk_buff *)pvPacket;

	ASSERT(pvPacket);

	if ((aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_SUCCESS
	     || aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_SUCCESS)
	    && ((aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_SUCCESS)
		|| (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_SUCCESS))) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	} else {
		skb->ip_summed = CHECKSUM_NONE;
#if DBG
		if (aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_NONE
		    && aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_NONE) {
			DBGLOG(RX, TRACE, ("RX: \"non-IPv4/IPv6\" Packet\n"));
		} else if (aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_FAILED) {
			DBGLOG(RX, TRACE, ("RX: \"bad IP Checksum\" Packet\n"));
		} else if (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_FAILED) {
			DBGLOG(RX, TRACE, ("RX: \"bad TCP Checksum\" Packet\n"));
		} else if (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_FAILED) {
			DBGLOG(RX, TRACE, ("RX: \"bad UDP Checksum\" Packet\n"));
		} else {

		}
#endif
	}

}				
#endif				


VOID kalPacketFree(IN P_GLUE_INFO_T prGlueInfo, IN PVOID pvPacket)
{
	dev_kfree_skb((struct sk_buff *)pvPacket);
}


PVOID kalPacketAlloc(IN P_GLUE_INFO_T prGlueInfo, IN UINT_32 u4Size, OUT PUINT_8 * ppucData)
{
	struct sk_buff *prSkb = dev_alloc_skb(u4Size);

	if (prSkb) {
		*ppucData = (PUINT_8) (prSkb->data);

        kalResetPacket(prGlueInfo, (P_NATIVE_PACKET)prSkb);
	}
#if DBG
	{
		PUINT_32 pu4Head = (PUINT_32) &prSkb->cb[0];
		*pu4Head = (UINT_32) prSkb->head;
		DBGLOG(RX, TRACE,
		       ("prSkb->head = %#lx, prSkb->cb = %#lx\n", (UINT_32) prSkb->head, *pu4Head));
	}
#endif
	return (PVOID) prSkb;
}

WLAN_STATUS
kalProcessRxPacket(IN P_GLUE_INFO_T prGlueInfo,
		   IN PVOID pvPacket, IN PUINT_8 pucPacketStart, IN UINT_32 u4PacketLen,
		   
		   IN BOOLEAN fgIsRetain, IN ENUM_CSUM_RESULT_T aerCSUM[]
    )
{
	WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
	struct sk_buff *skb = (struct sk_buff *)pvPacket;
	
	skb->data = (unsigned char *)pucPacketStart;

    
    skb_reset_tail_pointer(skb);
    skb_trim(skb, 0);

    
	skb_put(skb, u4PacketLen);

#if CFG_TCP_IP_CHKSUM_OFFLOAD
	kalUpdateRxCSUMOffloadParam(skb, aerCSUM);
#endif

	return rStatus;
}

WLAN_STATUS kalRxIndicatePkts(IN P_GLUE_INFO_T prGlueInfo, IN PVOID apvPkts[], IN UINT_8 ucPktNum)
{
	UINT_8 ucIdx = 0;

	ASSERT(prGlueInfo);
	ASSERT(apvPkts);

	for (ucIdx = 0; ucIdx < ucPktNum; ucIdx++) {

		kalRxIndicateOnePkt(prGlueInfo, apvPkts[ucIdx]);

	}

	KAL_WAKE_LOCK_TIMEOUT(prGlueInfo->prAdapter, &prGlueInfo->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_RX_TIMEOUT));

	return WLAN_STATUS_SUCCESS;
}

WLAN_STATUS kalRxIndicateOnePkt(IN P_GLUE_INFO_T prGlueInfo, IN PVOID pvPkt)
{
	struct net_device *prNetDev = prGlueInfo->prDevHandler;
	struct sk_buff *prSkb = NULL;

	ASSERT(prGlueInfo);
	ASSERT(pvPkt);

	prSkb = pvPkt;
#if DBG && 0
	do {
		PUINT_8 pu4Head = (PUINT_8) &prSkb->cb[0];
		UINT_32 u4HeadValue = 0;
		kalMemCopy(&u4HeadValue, pu4Head, sizeof(u4HeadValue));
		DBGLOG(RX, TRACE,
		       ("prSkb->head = 0x%p, prSkb->cb = 0x%lx\n", pu4Head, u4HeadValue));
	} while (0);
#endif

#if 1
	prNetDev =
	    (struct net_device *)wlanGetNetInterfaceByBssIdx(prGlueInfo,
							     GLUE_GET_PKT_BSS_IDX(prSkb));
	if (!prNetDev) {
		prNetDev = prGlueInfo->prDevHandler;
	}
#if CFG_SUPPORT_SNIFFER
    if (prGlueInfo->fgIsEnableMon) {
        prNetDev = prGlueInfo->prMonDevHandler;
    }
#endif    
	prNetDev->stats.rx_bytes += prSkb->len;
	prNetDev->stats.rx_packets++;
#else
	if (GLUE_GET_PKT_IS_P2P(prSkb)) {
		
#if CFG_ENABLE_WIFI_DIRECT
		if (prGlueInfo->prAdapter->fgIsP2PRegistered) {
			prNetDev = kalP2PGetDevHdlr(prGlueInfo);
		}
		
		
		prGlueInfo->prP2PInfo->rNetDevStats.rx_bytes += prSkb->len;
		prGlueInfo->prP2PInfo->rNetDevStats.rx_packets++;

#else
		prNetDev = prGlueInfo->prDevHandler;
#endif
	} else if (GLUE_GET_PKT_IS_PAL(prSkb)) {
		
#if CFG_ENABLE_BT_OVER_WIFI && CFG_BOW_SEPARATE_DATA_PATH
		if (prGlueInfo->rBowInfo.fgIsNetRegistered) {
			prNetDev = prGlueInfo->rBowInfo.prDevHandler;
		}
#else
		prNetDev = prGlueInfo->prDevHandler;
#endif
	} else {
		
		prNetDev = prGlueInfo->prDevHandler;
		prGlueInfo->rNetDevStats.rx_bytes += prSkb->len;
		prGlueInfo->rNetDevStats.rx_packets++;

	}
#endif
	prNetDev->last_rx = jiffies;
#if CFG_SUPPORT_SNIFFER
    if (prGlueInfo->fgIsEnableMon) {
        skb_reset_mac_header(prSkb);
        prSkb->ip_summed = CHECKSUM_UNNECESSARY;
        prSkb->pkt_type = PACKET_OTHERHOST;
        prSkb->protocol = __constant_htons(ETH_P_802_2);
    }
    else {
        prSkb->protocol = eth_type_trans(prSkb, prNetDev);
    }
#else
	prSkb->protocol = eth_type_trans(prSkb, prNetDev);
#endif
	prSkb->dev = prNetDev;
    
    
    if(prSkb->tail > prSkb->end){
		DBGLOG(RX, ERROR, ("kalRxIndicateOnePkt [prSkb = 0x%p][prSkb->len = %d][prSkb->protocol = 0x%02X] %p,%p\n", (PUINT_8)prSkb, prSkb->len, prSkb->protocol, prSkb->tail, prSkb->end));
		DBGLOG_MEM32(RX, ERROR, (PUINT_32)prSkb->data, prSkb->len);
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
	if (!in_interrupt()) {
		netif_rx_ni(prSkb);	
	} else {
		netif_rx(prSkb);
	}
#else
	netif_rx(prSkb);
#endif

	wlanReturnPacket(prGlueInfo->prAdapter, NULL);

	return WLAN_STATUS_SUCCESS;
}


VOID
kalIndicateStatusAndComplete(IN P_GLUE_INFO_T prGlueInfo,
			     IN WLAN_STATUS eStatus, IN PVOID pvBuf, IN UINT_32 u4BufLen)
{

	UINT_32 bufLen;
	P_PARAM_STATUS_INDICATION_T pStatus = (P_PARAM_STATUS_INDICATION_T) pvBuf;
	P_PARAM_AUTH_EVENT_T pAuth = (P_PARAM_AUTH_EVENT_T) pStatus;
	P_PARAM_PMKID_CANDIDATE_LIST_T pPmkid = (P_PARAM_PMKID_CANDIDATE_LIST_T) (pStatus + 1);
	PARAM_MAC_ADDRESS arBssid;
	struct cfg80211_scan_request *prScanRequest = NULL;
	PARAM_SSID_T ssid;
	struct ieee80211_channel *prChannel = NULL;
	struct cfg80211_bss *bss;
	UINT_8 ucChannelNum;
	P_BSS_DESC_T prBssDesc = NULL;

	GLUE_SPIN_LOCK_DECLARATION();

	kalMemZero(arBssid, MAC_ADDR_LEN);

	ASSERT(prGlueInfo);

	switch (eStatus) {
	case WLAN_STATUS_ROAM_OUT_FIND_BEST:
	case WLAN_STATUS_MEDIA_CONNECT:

		prGlueInfo->eParamMediaStateIndicated = PARAM_MEDIA_STATE_CONNECTED;

		
		wlanQueryInformation(prGlueInfo->prAdapter,
				     wlanoidQueryBssid, &arBssid[0], sizeof(arBssid), &bufLen);
		wext_indicate_wext_event(prGlueInfo, SIOCGIWAP, arBssid, bufLen);

		
		netif_carrier_on(prGlueInfo->prDevHandler);

		do {
			
			wlanQueryInformation(prGlueInfo->prAdapter,
					     wlanoidQuerySsid, &ssid, sizeof(ssid), &bufLen);

			ssid.aucSsid[(ssid.u4SsidLen >= PARAM_MAX_LEN_SSID) ?
				     (PARAM_MAX_LEN_SSID - 1) : ssid.u4SsidLen] = '\0';
			DBGLOG(INIT, INFO, ("[wifi] %s netif_carrier_on [ssid:%s " MACSTR "]\n",
					    prGlueInfo->prDevHandler->name,
					    ssid.aucSsid, MAC2STR(arBssid)));
		} while (0);

		if (prGlueInfo->fgIsRegistered == TRUE) {
			
			ucChannelNum =
			    wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter,
							  prGlueInfo->prAdapter->prAisBssInfo->
							  ucBssIndex);
			if (ucChannelNum <= 14) {
				prChannel =
				    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
							  ieee80211_channel_to_frequency
							  (ucChannelNum, IEEE80211_BAND_2GHZ));
			} else {
				prChannel =
				    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
							  ieee80211_channel_to_frequency
							  (ucChannelNum, IEEE80211_BAND_5GHZ));
			}

			
			bss = cfg80211_get_bss(priv_to_wiphy(prGlueInfo), prChannel, arBssid,
					       ssid.aucSsid, ssid.u4SsidLen,
					       WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);

			if (bss == NULL) {
				
				prBssDesc =
				    ((P_AIS_FSM_INFO_T)
				     (&(prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo)))->
				    prTargetBssDesc;

				if (prBssDesc != NULL) {
					bss = cfg80211_inform_bss(priv_to_wiphy(prGlueInfo), prChannel, arBssid, 0,	
								  WLAN_CAPABILITY_ESS, prBssDesc->u2BeaconInterval,	
								  prBssDesc->aucIEBuf,	
								  prBssDesc->u2IELength,	
								  RCPI_TO_dBm(prBssDesc->ucRCPI) * 100,	
								  GFP_KERNEL);
				}
			}


			
			if (eStatus == WLAN_STATUS_ROAM_OUT_FIND_BEST
				   && prGlueInfo->prDevHandler->ieee80211_ptr->sme_state ==
				   CFG80211_SME_CONNECTED) {
				struct ieee80211_channel *prChannel = NULL;
				UINT_8 ucChannelNum =
				    wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter,
								  prGlueInfo->prAdapter->
								  prAisBssInfo->ucBssIndex);

				if (ucChannelNum <= 14) {
					prChannel =
					    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
								  ieee80211_channel_to_frequency
								  (ucChannelNum,
								   IEEE80211_BAND_2GHZ));
				} else {
					prChannel =
					    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
								  ieee80211_channel_to_frequency
								  (ucChannelNum,
								   IEEE80211_BAND_5GHZ));
				}

				cfg80211_roamed(prGlueInfo->prDevHandler,
						prChannel,
						arBssid,
						prGlueInfo->aucReqIe,
						prGlueInfo->u4ReqIeLength,
						prGlueInfo->aucRspIe,
						prGlueInfo->u4RspIeLength, GFP_KERNEL);
			}else if  ( prGlueInfo->prDevHandler->ieee80211_ptr->sme_state ==
			    CFG80211_SME_CONNECTING) {
				cfg80211_connect_result(prGlueInfo->prDevHandler, arBssid,
							prGlueInfo->aucReqIe,
							prGlueInfo->u4ReqIeLength,
							prGlueInfo->aucRspIe,
							prGlueInfo->u4RspIeLength,
							WLAN_STATUS_SUCCESS, GFP_KERNEL);
			} 
		}

		break;

	case WLAN_STATUS_MEDIA_DISCONNECT:
	case WLAN_STATUS_MEDIA_DISCONNECT_LOCALLY:
		
		wext_indicate_wext_event(prGlueInfo, SIOCGIWAP, NULL, 0);
		

#if 1				
		DBGLOG(INIT, INFO,
		       ("[wifi] %s netif_carrier_off\n", prGlueInfo->prDevHandler->name));
#endif

		netif_carrier_off(prGlueInfo->prDevHandler);

		if (prGlueInfo->fgIsRegistered == TRUE
		    && prGlueInfo->prDevHandler->ieee80211_ptr->sme_state ==
		    CFG80211_SME_CONNECTED && eStatus == WLAN_STATUS_MEDIA_DISCONNECT) {
			P_BSS_INFO_T prBssInfo = prGlueInfo->prAdapter->prAisBssInfo;
			UINT_16 u2DeauthReason = 0;

			if (prBssInfo) {
				u2DeauthReason = prBssInfo->u2DeauthReason;
			}
			
			cfg80211_disconnected(prGlueInfo->prDevHandler, u2DeauthReason,
					      NULL, 0, GFP_KERNEL);
		}

		prGlueInfo->eParamMediaStateIndicated = PARAM_MEDIA_STATE_DISCONNECTED;

		break;

	case WLAN_STATUS_SCAN_COMPLETE:
		
		wext_indicate_wext_event(prGlueInfo, SIOCGIWSCAN, NULL, 0);

		
		GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
		if (prGlueInfo->prScanRequest != NULL) {
			prScanRequest = prGlueInfo->prScanRequest;
			prGlueInfo->prScanRequest = NULL;
		}
		GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

		
		if (prScanRequest != NULL) {
            DBGLOG(SCN, INFO, ("cfg80211_scan_done send event \n" ));
			cfg80211_scan_done(prScanRequest, FALSE);
            DBGLOG(SCN, INFO, ("cfg80211_scan_done event done \n" ));
		}
		break;

#if 0
	case WLAN_STATUS_MSDU_OK:
		if (netif_running(prGlueInfo->prDevHandler)) {
			netif_wake_queue(prGlueInfo->prDevHandler);
		}
		break;
#endif

	case WLAN_STATUS_MEDIA_SPECIFIC_INDICATION:
		if (pStatus) {
			switch (pStatus->eStatusType) {
			case ENUM_STATUS_TYPE_AUTHENTICATION:
				
				if ((pAuth->arRequest[0].u4Flags ==
				     PARAM_AUTH_REQUEST_PAIRWISE_ERROR) ||
				    (pAuth->arRequest[0].u4Flags ==
				     PARAM_AUTH_REQUEST_GROUP_ERROR)) {
					cfg80211_michael_mic_failure(prGlueInfo->prDevHandler, NULL,
								     (pAuth->arRequest[0].u4Flags ==
								      PARAM_AUTH_REQUEST_PAIRWISE_ERROR)
								     ? NL80211_KEYTYPE_PAIRWISE :
								     NL80211_KEYTYPE_GROUP, 0, NULL,
								     GFP_KERNEL);
					wext_indicate_wext_event(prGlueInfo, IWEVMICHAELMICFAILURE,
								 (unsigned char *)&pAuth->
								 arRequest[0],
								 pAuth->arRequest[0].u4Length);
				}
				break;

			case ENUM_STATUS_TYPE_CANDIDATE_LIST:
				{
					UINT_32 i = 0;

					P_PARAM_PMKID_CANDIDATE_T prPmkidCand =
					    (P_PARAM_PMKID_CANDIDATE_T) &pPmkid->
					    arCandidateList[0];

					for (i = 0; i < pPmkid->u4NumCandidates; i++) {
						wext_indicate_wext_event(prGlueInfo,
									 IWEVPMKIDCAND,
									 (unsigned char *)&pPmkid->
									 arCandidateList[i],
									 pPmkid->u4NumCandidates);
						prPmkidCand += sizeof(PARAM_PMKID_CANDIDATE_T);
					}
				}
				break;

			default:
				
				break;
			}
		} else {
		}
		break;

#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS
	case WLAN_STATUS_BWCS_UPDATE:
		{
			wext_indicate_wext_event(prGlueInfo, IWEVCUSTOM, pvBuf, sizeof(PTA_IPC_T));
		}

		break;

#endif
	case WLAN_STATUS_JOIN_TIMEOUT:
		cfg80211_connect_result(prGlueInfo->prDevHandler,
					arBssid,
					prGlueInfo->aucReqIe,
					prGlueInfo->u4ReqIeLength,
					prGlueInfo->aucRspIe,
					prGlueInfo->u4RspIeLength,
					WLAN_STATUS_AUTH_TIMEOUT, GFP_KERNEL);
		break;

	default:
		break;
	}
}				


VOID
kalUpdateReAssocReqInfo(IN P_GLUE_INFO_T prGlueInfo,
			IN PUINT_8 pucFrameBody,
			IN UINT_32 u4FrameBodyLen, IN BOOLEAN fgReassocRequest)
{
	PUINT_8 cp;

	ASSERT(prGlueInfo);

	
	prGlueInfo->u4ReqIeLength = 0;

	if (fgReassocRequest) {
		if (u4FrameBodyLen < 15) {
			return;
		}
	} else {
		if (u4FrameBodyLen < 9) {
			return;
		}
	}

	cp = pucFrameBody;

	if (fgReassocRequest) {
		
		
		
		cp += 10;
		u4FrameBodyLen -= 10;
	} else {
		
		
		cp += 4;
		u4FrameBodyLen -= 4;
	}

	wext_indicate_wext_event(prGlueInfo, IWEVASSOCREQIE, cp, u4FrameBodyLen);

	if (u4FrameBodyLen <= CFG_CFG80211_IE_BUF_LEN) {
		prGlueInfo->u4ReqIeLength = u4FrameBodyLen;
		kalMemCopy(prGlueInfo->aucReqIe, cp, u4FrameBodyLen);
	}

	return;
}


VOID
kalUpdateReAssocRspInfo(IN P_GLUE_INFO_T prGlueInfo,
			IN PUINT_8 pucFrameBody, IN UINT_32 u4FrameBodyLen)
{
	UINT_32 u4IEOffset = 6;	
	UINT_32 u4IELength = u4FrameBodyLen - u4IEOffset;

	ASSERT(prGlueInfo);

	
	prGlueInfo->u4RspIeLength = 0;

	if (u4IELength <= CFG_CFG80211_IE_BUF_LEN) {
		prGlueInfo->u4RspIeLength = u4IELength;
		kalMemCopy(prGlueInfo->aucRspIe, pucFrameBody + u4IEOffset, u4IELength);
	}

}				

VOID
kalResetPacket(IN P_GLUE_INFO_T prGlueInfo,
    IN P_NATIVE_PACKET prPacket)
{
    struct sk_buff *prSkb = (struct sk_buff *)prPacket;

    
    kalMemZero(prSkb->cb, sizeof(prSkb->cb));
}

WLAN_STATUS
kalHardStartXmit(struct sk_buff *prSkb,
		 IN struct net_device *prDev, P_GLUE_INFO_T prGlueInfo, UINT_8 ucBssIndex)
{
	P_QUE_ENTRY_T prQueueEntry = NULL;
	P_QUE_T prTxQueue = NULL;
	UINT_16 u2QueueIdx = 0;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prSkb);
	ASSERT(prGlueInfo);

	if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
		DBGLOG(INIT, INFO, ("GLUE_FLAG_HALT skip tx\n"));
		dev_kfree_skb(prSkb);
		return WLAN_STATUS_ADAPTER_NOT_READY;
	}

	prQueueEntry = (P_QUE_ENTRY_T) GLUE_GET_PKT_QUEUE_ENTRY(prSkb);
	prTxQueue = &prGlueInfo->rTxQueue;

	GLUE_SET_PKT_BSS_IDX(prSkb, ucBssIndex);

#if CFG_DBG_GPIO_PINS
	
	mtk_wcn_stp_debug_gpio_assert(IDX_TX_REQ, DBG_TIE_LOW);
	kalUdelay(1);
	mtk_wcn_stp_debug_gpio_assert(IDX_TX_REQ, DBG_TIE_HIGH);
#endif

    
	if (!wlanProcessTxFrame(prGlueInfo->prAdapter, (P_NATIVE_PACKET) prSkb)) {
		
		printk(KERN_INFO DRV_NAME "Cannot extract content, skip this frame\n");
		dev_kfree_skb(prSkb);
		return WLAN_STATUS_INVALID_PACKET;
	}

    
    wlanTxProfilingTagPacket(prGlueInfo->prAdapter, (P_NATIVE_PACKET)prSkb, TX_PROF_TAG_OS_TO_DRV);

	
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	u2QueueIdx = skb_get_queue_mapping(prSkb);

	if (u2QueueIdx >= CFG_MAX_TXQ_NUM) {
		printk(KERN_INFO DRV_NAME "Incorrect queue index, skip this frame\n");
		dev_kfree_skb(prSkb);
		return WLAN_STATUS_INVALID_PACKET;
	}
#endif

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
	QUEUE_INSERT_TAIL(prTxQueue, prQueueEntry);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

	GLUE_INC_REF_CNT(prGlueInfo->i4TxPendingFrameNum);
	GLUE_INC_REF_CNT(prGlueInfo->ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx]);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	if (GLUE_GET_REF_CNT
	    (prGlueInfo->ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx])
	    >= prGlueInfo->prAdapter->rWifiVar.u4NetifStopTh) {
		netif_stop_subqueue(prDev, u2QueueIdx);

		DBGLOG(TX, INFO,
		       ("Stop subqueue for BSS[%u] QIDX[%u] PKT_LEN[%u] TOT_CNT[%ld] PER-Q_CNT[%ld]\n",
			ucBssIndex, u2QueueIdx, prSkb->len,
			GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum),
			GLUE_GET_REF_CNT(prGlueInfo->
					 ai4TxPendingFrameNumPerQueue[ucBssIndex]
					 [u2QueueIdx])));
	}
#else
	if (GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum) >=
	    CFG_TX_STOP_NETIF_QUEUE_THRESHOLD) {
		netif_stop_queue(prDev);

		DBGLOG(TX, INFO,
		       ("Stop queue for BSS[%u] QIDX[%u] PKT_LEN[%u] TOT_CNT[%ld] PER-Q_CNT[%ld]\n",
			ucBssIndex, u2QueueIdx, prSkb->len,
			GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum),
			GLUE_GET_REF_CNT(prGlueInfo->
					 ai4TxPendingFrameNumPerQueue[ucBssIndex]
					 [u2QueueIdx])));
	}
#endif

	
	prDev->stats.tx_bytes += prSkb->len;
	prDev->stats.tx_packets++;

	DBGLOG(TX, LOUD,
	       ("Enqueue frame for BSS[%u] QIDX[%u] PKT_LEN[%u] TOT_CNT[%ld] PER-Q_CNT[%ld]\n",
		ucBssIndex, u2QueueIdx, prSkb->len,
		GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum),
		GLUE_GET_REF_CNT(prGlueInfo->
				 ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx])));

	kalSetEvent(prGlueInfo);

	return WLAN_STATUS_SUCCESS;
}				

WLAN_STATUS kalResetStats(IN struct net_device *prDev)
{
	DBGLOG(QM, INFO, ("Reset NetDev[0x%p] statistics\n", prDev));

	kalMemZero(kalGetStats(prDev), sizeof(struct net_device_stats));

	return WLAN_STATUS_SUCCESS;
}

PVOID kalGetStats(IN struct net_device *prDev)
{
	return (PVOID) &prDev->stats;
}				


VOID kalSendCompleteAndAwakeQueue(IN P_GLUE_INFO_T prGlueInfo, IN PVOID pvPacket)
{
	struct net_device *prDev = NULL;
	struct sk_buff *prSkb = NULL;
	UINT_16 u2QueueIdx = 0;
	UINT_8 ucBssIndex = 0;
	BOOLEAN fgIsValidDevice = TRUE;

	ASSERT(pvPacket);
	

	prSkb = (struct sk_buff *)pvPacket;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	u2QueueIdx = skb_get_queue_mapping(prSkb);
#endif
	ASSERT(u2QueueIdx < CFG_MAX_TXQ_NUM);

	ucBssIndex = GLUE_GET_PKT_BSS_IDX(pvPacket);

#if CFG_ENABLE_WIFI_DIRECT
	{
		P_BSS_INFO_T prBssInfo = GET_BSS_INFO_BY_INDEX(prGlueInfo->prAdapter, ucBssIndex);

		
		if ((prBssInfo->eNetworkType == NETWORK_TYPE_P2P) &&
		    (prGlueInfo->prAdapter->fgIsP2PRegistered == FALSE)) {
			fgIsValidDevice = FALSE;
		}
	}
#endif

#if 0
	if ((GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum) <= 0)) {
		UINT_8 ucBssIdx;
		UINT_16 u2QIdx;


		printk("TxPendingFrameNum[%u] CurFrameId[%u]\n", prGlueInfo->i4TxPendingFrameNum,
		       GLUE_GET_PKT_ARRIVAL_TIME(pvPacket));

		for (ucBssIdx = 0; ucBssIdx < HW_BSSID_NUM; ucBssIdx++) {
			for (u2QIdx = 0; u2QIdx < CFG_MAX_TXQ_NUM; u2QIdx++) {
				printk("BSS[%u] Q[%u] TxPendingFrameNum[%u]\n",
				       ucBssIdx,
				       u2QIdx,
				       prGlueInfo->ai4TxPendingFrameNumPerQueue[ucBssIdx][u2QIdx]);
			}
		}
	}

	ASSERT((GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum) > 0));
#endif

	GLUE_DEC_REF_CNT(prGlueInfo->i4TxPendingFrameNum);
	GLUE_DEC_REF_CNT(prGlueInfo->ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx]);

	DBGLOG(TX, LOUD,
	       ("Release frame for BSS[%u] QIDX[%u] PKT_LEN[%u] TOT_CNT[%ld] PER-Q_CNT[%ld]\n",
		ucBssIndex, u2QueueIdx, prSkb->len,
		GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum),
		GLUE_GET_REF_CNT(prGlueInfo->
				 ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx])));

	prDev = prSkb->dev;

	ASSERT(prDev);

	if (fgIsValidDevice == TRUE) {
		UINT_32 u4StartTh = prGlueInfo->prAdapter->rWifiVar.u4NetifStartTh;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
		if (netif_subqueue_stopped(prDev, prSkb) &&
		    prGlueInfo->ai4TxPendingFrameNumPerQueue[ucBssIndex][u2QueueIdx] <= u4StartTh) {
			netif_wake_subqueue(prDev, u2QueueIdx);
			DBGLOG(TX, INFO,
			       ("WakeUp Queue BSS[%u] QIDX[%u] PKT_LEN[%u] TOT_CNT[%ld] PER-Q_CNT[%ld]\n",
				ucBssIndex, u2QueueIdx, prSkb->len,
				GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum),
				GLUE_GET_REF_CNT(prGlueInfo->
						 ai4TxPendingFrameNumPerQueue[ucBssIndex]
						 [u2QueueIdx])));
		}
#else
		if (prGlueInfo->i4TxPendingFrameNum < CFG_TX_STOP_NETIF_QUEUE_THRESHOLD) {
			netif_wake_queue(prGlueInfo->prDevHandler);
		}
#endif
	}

	dev_kfree_skb((struct sk_buff *)pvPacket);

	DBGLOG(TX, LOUD, ("----- pending frame %d -----\n", prGlueInfo->i4TxPendingFrameNum));

	return;
}


VOID kalQueryRegistryMacAddr(IN P_GLUE_INFO_T prGlueInfo, OUT PUINT_8 paucMacAddr)
{
	UINT_8 aucZeroMac[MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 }
	DEBUGFUNC("kalQueryRegistryMacAddr");

	ASSERT(prGlueInfo);
	ASSERT(paucMacAddr);

	kalMemCopy((PVOID) paucMacAddr, (PVOID) aucZeroMac, MAC_ADDR_LEN);

	return;
}				

#if CFG_SUPPORT_EXT_CONFIG
UINT_32 kalReadExtCfg(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	if (0 != prGlueInfo->u4ExtCfgLength) {
		DBGLOG(INIT, TRACE, ("Read external configuration data -- OK\n"));
	} else {
		DBGLOG(INIT, TRACE, ("Read external configuration data -- fail\n"));
	}

	return prGlueInfo->u4ExtCfgLength;
}
#endif

BOOLEAN
kalQoSFrameClassifierAndPacketInfo(IN P_GLUE_INFO_T prGlueInfo,
    IN P_NATIVE_PACKET prPacket, OUT P_TX_PACKET_INFO prTxPktInfo)
{
	UINT_32 u4PacketLen;
	UINT_16 u2EtherTypeLen;
	struct sk_buff *prSkb = (struct sk_buff *)prPacket;
	PUINT_8 aucLookAheadBuf = NULL;
	UINT_8 ucEthTypeLenOffset = ETHER_HEADER_LEN - ETHER_TYPE_LEN;
	PUINT_8 pucNextProtocol = NULL;

	u4PacketLen = prSkb->len;

	if (u4PacketLen < ETHER_HEADER_LEN) {
		DBGLOG(INIT, WARN, ("Invalid Ether packet length: %lu\n", u4PacketLen));
		return FALSE;
	}

	aucLookAheadBuf = prSkb->data;

    
    kalMemZero(prTxPktInfo, sizeof(TX_PACKET_INFO));

	
	WLAN_GET_FIELD_BE16(&aucLookAheadBuf[ucEthTypeLenOffset], &u2EtherTypeLen);

	
	if (u2EtherTypeLen == ETH_P_VLAN) {
		prTxPktInfo->fgIsVlanExists = TRUE;
		ucEthTypeLenOffset += ETH_802_1Q_HEADER_LEN;
		WLAN_GET_FIELD_BE16(&aucLookAheadBuf[ucEthTypeLenOffset], &u2EtherTypeLen);
	}
	
	pucNextProtocol = &aucLookAheadBuf[ucEthTypeLenOffset + ETHER_TYPE_LEN];

	
	switch (u2EtherTypeLen) {

#if 0
		
	case ETH_P_IPV4:

        
        if(!kalTxProfilingIsEnabled(prGlueInfo, prPacket)) {
            break;
        }
        
		{
			PUINT_8 pucIpHdr = pucNextProtocol;
			UINT_8 ucIpVersion;
			UINT_8 ucIpTos;

			
			if (u4PacketLen < (ucEthTypeLenOffset + ETHER_TYPE_LEN + IPV4_HDR_LEN)) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv4 packet length: %lu\n", u4PacketLen));
				return FALSE;
			}

			
			ucIpVersion = (pucIpHdr[0] & IP_VERSION_MASK) >> IP_VERSION_OFFSET;
			if (ucIpVersion != IP_VERSION_4) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv4 packet version: %u\n", ucIpVersion));
				return FALSE;
			}
			
			ucIpTos = *(pucIpHdr + IPV4_HDR_TOS_OFFSET);
			ucUserPriority =
			    ((ucIpTos & IPV4_HDR_TOS_PREC_MASK) >> IPV4_HDR_TOS_PREC_OFFSET);
			
                     
		}
		break;

		
	case ETH_P_IPV6:
		{
			PUINT_8 pucIpHdr = pucNextProtocol;
			UINT_8 ucIpVersion;

			
			if (u4PacketLen < (ucEthTypeLenOffset + ETHER_TYPE_LEN + IPV6_HDR_LEN)) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv6 packet length: %lu\n", u4PacketLen));
				return FALSE;
			}

			
			ucIpVersion = (pucIpHdr[0] & IP_VERSION_MASK) >> IP_VERSION_OFFSET;
			if (ucIpVersion != IP_VERSION_6) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv6 packet version: %u\n", ucIpVersion));
				return FALSE;
			}

			
			ucUserPriority =
			    ((pucIpHdr[0] & IPV6_HDR_TC_PREC_MASK) >> IPV6_HDR_TC_PREC_OFFSET);
		}
		break;
#endif

	case ETH_P_1X:
	case ETH_P_PRE_1X:
#if CFG_SUPPORT_WAPI
	case ETH_WPI_1X:
#endif
		prTxPktInfo->fgIs1X = TRUE;
		break;

	default:
		
		if (u2EtherTypeLen <= ETH_802_3_MAX_LEN) {
			prTxPktInfo->fgIs802_3 = TRUE;
		}
		break;
	}

	
	

	
	
	
	prTxPktInfo->ucPriorityParam = prSkb->priority;

	
	
	prTxPktInfo->u4PacketLen = u4PacketLen;

	kalMemCopy(prTxPktInfo->aucEthDestAddr, aucLookAheadBuf, PARAM_MAC_ADDR_LEN);

	return TRUE;
}				

BOOLEAN
kalGetEthDestAddr(IN P_GLUE_INFO_T prGlueInfo,
		  IN P_NATIVE_PACKET prPacket, OUT PUINT_8 pucEthDestAddr)
{
	struct sk_buff *prSkb = (struct sk_buff *)prPacket;
	PUINT_8 aucLookAheadBuf = NULL;

	
	if (!prPacket || !prGlueInfo) {
		return FALSE;
	}

	aucLookAheadBuf = prSkb->data;

	kalMemCopy(pucEthDestAddr, aucLookAheadBuf, PARAM_MAC_ADDR_LEN);

	return TRUE;
}


VOID
kalOidComplete(IN P_GLUE_INFO_T prGlueInfo,
	       IN BOOLEAN fgSetQuery, IN UINT_32 u4SetQueryInfoLen, IN WLAN_STATUS rOidStatus)
{

	ASSERT(prGlueInfo);
	
	wlanoidClearTimeoutCheck(prGlueInfo->prAdapter);

	prGlueInfo->rPendStatus = rOidStatus;

	prGlueInfo->u4OidCompleteFlag = 1;
	
	if (!completion_done(&prGlueInfo->rPendComp)) {
		complete(&prGlueInfo->rPendComp);
	} else {
		DBGLOG(INIT, WARN, ("SKIP multiple OID complete!\n"));
		WARN_ON(TRUE);
	}

	
}

VOID kalOidClearance(IN P_GLUE_INFO_T prGlueInfo)
{

}





WLAN_STATUS
kalIoctl(IN P_GLUE_INFO_T prGlueInfo,
	 IN PFN_OID_HANDLER_FUNC pfnOidHandler,
	 IN PVOID pvInfoBuf,
	 IN UINT_32 u4InfoBufLen,
	 IN BOOL fgRead, IN BOOL fgWaitResp, IN BOOL fgCmd, OUT PUINT_32 pu4QryInfoLen)
{
	extern BOOLEAN fgIsResetting;
	P_GL_IO_REQ_T prIoReq = NULL;
	WLAN_STATUS ret = WLAN_STATUS_SUCCESS;

	if (fgIsResetting == TRUE)
		return WLAN_STATUS_SUCCESS;

	
	ASSERT(prGlueInfo);

	
	
	
	

	if (down_interruptible(&g_halt_sem)) {
		return WLAN_STATUS_FAILURE;
	}

	if (g_u4HaltFlag) {
		up(&g_halt_sem);
		return WLAN_STATUS_ADAPTER_NOT_READY;
	}

	if (down_interruptible(&prGlueInfo->ioctl_sem)) {
		up(&g_halt_sem);
		return WLAN_STATUS_FAILURE;
	}

	

	

	prIoReq = &(prGlueInfo->OidEntry);

	ASSERT(prIoReq);

	
	prIoReq->prAdapter = prGlueInfo->prAdapter;
	prIoReq->pfnOidHandler = pfnOidHandler;
	prIoReq->pvInfoBuf = pvInfoBuf;
	prIoReq->u4InfoBufLen = u4InfoBufLen;
	prIoReq->pu4QryInfoLen = pu4QryInfoLen;
	prIoReq->fgRead = fgRead;
	prIoReq->fgWaitResp = fgWaitResp;
	prIoReq->rStatus = WLAN_STATUS_FAILURE;

	
	prGlueInfo->rPendStatus = WLAN_STATUS_FAILURE;
	
	prGlueInfo->u4OidCompleteFlag = 0;

	
	prIoReq->u4Flag = fgCmd;

	
	set_bit(GLUE_FLAG_OID_BIT, &prGlueInfo->ulFlag);

	
	KAL_WAKE_LOCK_TIMEOUT(prGlueInfo->prAdapter, &prGlueInfo->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_THREAD_WAKEUP_TIMEOUT));

	
	wake_up_interruptible(&prGlueInfo->waitq);

	
	wait_for_completion(&prGlueInfo->rPendComp);
	{
		
		
		if (prIoReq->rStatus == WLAN_STATUS_PENDING) {
			ret = prGlueInfo->rPendStatus;
		} else {
			ret = prIoReq->rStatus;
		}
	}
#if 0
	else {
		
		
		if (fgCmd) {
			prGlueInfo->u4TimeoutFlag = 1;
			wlanReleasePendingOid(prGlueInfo->prAdapter, 0);
		}
		ret = WLAN_STATUS_FAILURE;
	}
#endif

	
	clear_bit(GLUE_FLAG_OID_BIT, &prGlueInfo->ulFlag);

	up(&prGlueInfo->ioctl_sem);
	up(&g_halt_sem);

	return ret;
}

VOID kalClearSecurityFrames(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;

	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;
	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);
	
	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME) {
			if (prCmdInfo->pfCmdTimeoutHandler) {
				prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
			} else {
				wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo,
						   TX_RESULT_QUEUE_CLEARANCE);
			}
			cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			QUEUE_INSERT_TAIL(prReturnCmdQue, prQueueEntry);
		}

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_CONCATENATE_QUEUES_HEAD(prCmdQue, prReturnCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID kalClearSecurityFramesByBssIdx(IN P_GLUE_INFO_T prGlueInfo, IN UINT_8 ucBssIndex)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;

	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;
	GLUE_SPIN_LOCK_DECLARATION();


	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);
	
	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME &&
		    prCmdInfo->ucBssIndex == ucBssIndex) {
			if (prCmdInfo->pfCmdTimeoutHandler) {
				prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
			} else {
				wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo,
						   TX_RESULT_QUEUE_CLEARANCE);
			}
			cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			QUEUE_INSERT_TAIL(prReturnCmdQue, prQueueEntry);
		}

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_CONCATENATE_QUEUES_HEAD(prCmdQue, prReturnCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID kalClearMgmtFrames(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);
	
	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME) {
			wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo,
					   TX_RESULT_QUEUE_CLEARANCE);
			cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			QUEUE_INSERT_TAIL(prReturnCmdQue, prQueueEntry);
		}

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_CONCATENATE_QUEUES_HEAD(prCmdQue, prReturnCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID kalClearMgmtFramesByBssIdx(IN P_GLUE_INFO_T prGlueInfo, IN UINT_8 ucBssIndex)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);
	
	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME &&
		    prCmdInfo->ucBssIndex == ucBssIndex) {
			wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo,
					   TX_RESULT_QUEUE_CLEARANCE);
			cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			QUEUE_INSERT_TAIL(prReturnCmdQue, prQueueEntry);
		}

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_CONCATENATE_QUEUES_HEAD(prCmdQue, prReturnCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}				


VOID kalClearCommandQueue(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);

	
	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {
		prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

		if (prCmdInfo->pfCmdTimeoutHandler) {
			prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo,
					   TX_RESULT_QUEUE_CLEARANCE);
		}

		cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}
}

UINT_32
kalProcessTxPacket(
    P_GLUE_INFO_T       prGlueInfo,
    struct sk_buff      *prSkb
    )
{
    UINT_32 u4Status = WLAN_STATUS_SUCCESS;

    if (NULL == prSkb) {
        DBGLOG(INIT, WARN, ("prSkb == NULL in tx\n"));
        return u4Status;
    }

    
    if(GLUE_GET_PKT_IS_1X(prSkb)) {
        if(wlanProcessSecurityFrame(prGlueInfo->prAdapter, (P_NATIVE_PACKET)prSkb)) {
            u4Status = WLAN_STATUS_SUCCESS;
            GLUE_INC_REF_CNT(prGlueInfo->i4TxPendingSecurityFrameNum);
        }
        else {
            u4Status = WLAN_STATUS_RESOURCES;
        }
    }
    
    else {
        u4Status = wlanEnqueueTxPacket(prGlueInfo->prAdapter, (P_NATIVE_PACKET)prSkb);
    }

    return u4Status;
}


VOID kalProcessTxReq(P_GLUE_INFO_T prGlueInfo, PBOOLEAN pfgNeedHwAccess)
{
	P_QUE_T prCmdQue = NULL;
	P_QUE_T prTxQueue = NULL;
    QUE_T               rTempQue;
    P_QUE_T             prTempQue = &rTempQue;
    QUE_T               rTempReturnQue;
    P_QUE_T             prTempReturnQue = &rTempReturnQue;
	P_QUE_ENTRY_T prQueueEntry = NULL;
    
	UINT_32 u4Status;
#if CFG_SUPPORT_MULTITHREAD
	UINT_32 u4CmdCount = 0;
#endif
    UINT_32             u4TxLoopCount;

	
	GLUE_SPIN_LOCK_DECLARATION();

	prTxQueue = &prGlueInfo->rTxQueue;
	prCmdQue = &prGlueInfo->rCmdQueue;

    QUEUE_INITIALIZE(prTempQue);
    QUEUE_INITIALIZE(prTempReturnQue);

    u4TxLoopCount = prGlueInfo->prAdapter->rWifiVar.u4TxFromOsLoopCount;

	
	wlanProcessMboxMessage(prGlueInfo->prAdapter);

	
#if CFG_SUPPORT_MULTITHREAD
	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	u4CmdCount = prCmdQue->u4NumElem;
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	if (u4CmdCount > 0) {
#else
	if (prCmdQue->u4NumElem > 0) {
		if (*pfgNeedHwAccess == FALSE) {
			*pfgNeedHwAccess = TRUE;

			wlanAcquirePowerControl(prGlueInfo->prAdapter);
		}
#endif
		wlanProcessCommandQueue(prGlueInfo->prAdapter, prCmdQue);
	}

    while(u4TxLoopCount--) {
        while(QUEUE_IS_NOT_EMPTY(prTxQueue)) {
		GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
            QUEUE_MOVE_ALL(prTempQue, prTxQueue);
		GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

            
            while (QUEUE_IS_NOT_EMPTY(prTempQue)) {
                QUEUE_REMOVE_HEAD(prTempQue, prQueueEntry, P_QUE_ENTRY_T);

		if (NULL == prQueueEntry) {
			break;
		}

                u4Status = kalProcessTxPacket(prGlueInfo, (struct sk_buff *)GLUE_GET_PKT_DESCRIPTOR(prQueueEntry));
#if 0
        prSkb = (struct sk_buff *) GLUE_GET_PKT_DESCRIPTOR(prQueueEntry);
		ASSERT(prSkb);
		if (NULL == prSkb) {
			DBGLOG(INIT, WARN, ("prSkb == NULL in tx\n"));
			continue;
		}

		
		if (GLUE_GET_PKT_IS_1X(prSkb)) {
			if (wlanProcessSecurityFrame
			    (prGlueInfo->prAdapter, (P_NATIVE_PACKET) prSkb)) {
				u4Status = WLAN_STATUS_SUCCESS;
				GLUE_INC_REF_CNT(prGlueInfo->i4TxPendingSecurityFrameNum);
			} else {
				u4Status = WLAN_STATUS_RESOURCES;
			}
		}
		
		else {
			u4Status =
			    wlanEnqueueTxPacket(prGlueInfo->prAdapter, (P_NATIVE_PACKET) prSkb);
		}
#endif
                
                if(u4Status == WLAN_STATUS_RESOURCES) {
                    QUEUE_INSERT_TAIL(prTempReturnQue, prQueueEntry);
                    break;
                }        
            }

            if (wlanGetTxPendingFrameCount(prGlueInfo->prAdapter) > 0) {
                wlanTxPendingPackets(prGlueInfo->prAdapter, pfgNeedHwAccess);
            }

		
            if(QUEUE_IS_NOT_EMPTY(prTempReturnQue)) {
                QUEUE_CONCATENATE_QUEUES(prTempReturnQue, prTempQue);
                
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
                QUEUE_CONCATENATE_QUEUES_HEAD(prTxQueue, prTempReturnQue);
			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

			break;
		}
	}

	if (wlanGetTxPendingFrameCount(prGlueInfo->prAdapter) > 0) {
		wlanTxPendingPackets(prGlueInfo->prAdapter, pfgNeedHwAccess);
	}
    }

	return;
}

#if CFG_SUPPORT_MULTITHREAD


int hif_thread(void *data)
{
	struct net_device *dev = data;
	P_GLUE_INFO_T prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(dev));
	int ret = 0;
	KAL_WAKE_LOCK_T rHifThreadWakeLock;

	KAL_WAKE_LOCK_INIT(prGlueInfo->prAdapter, &rHifThreadWakeLock, "WLAN hif_thread");
	KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rHifThreadWakeLock);

	DBGLOG(INIT, INFO, ("hif_thread starts running ID=%d\n", KAL_GET_CURRENT_THREAD_ID()));
	prGlueInfo->u4HifThreadPid = KAL_GET_CURRENT_THREAD_ID();

	set_user_nice(current, prGlueInfo->prAdapter->rWifiVar.cThreadNice);

	while (TRUE) {

		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			DBGLOG(INIT, INFO, ("hif_thread should stop now...\n"));
			break;
		}

		
		if (!(prGlueInfo->ulFlag & GLUE_FLAG_HIF_PROCESS)) {
			KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rHifThreadWakeLock);
		}

		do {
			ret = wait_event_interruptible(prGlueInfo->waitq_hif,
						       ((prGlueInfo->ulFlag & GLUE_FLAG_HIF_PROCESS) != 0));
		} while (ret != 0);

		if (!KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rHifThreadWakeLock)) {
			KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rHifThreadWakeLock);
		}

		wlanAcquirePowerControl(prGlueInfo->prAdapter);

		
		if (test_and_clear_bit(GLUE_FLAG_INT_BIT, &prGlueInfo->ulFlag)) {
			prGlueInfo->prAdapter->fgIsIntEnable = FALSE;
			if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
				
				DBGLOG(INIT, INFO, ("ignore pending interrupt\n"));
			} else {
				
				wlanIST(prGlueInfo->prAdapter);
			}
		}

		
		if (test_and_clear_bit(GLUE_FLAG_HIF_TX_CMD_BIT, &prGlueInfo->ulFlag)) {
			wlanTxCmdMthread(prGlueInfo->prAdapter);
		}

		
		if (test_and_clear_bit(GLUE_FLAG_HIF_TX_BIT, &prGlueInfo->ulFlag)) {
			nicTxMsduQueueMthread(prGlueInfo->prAdapter);
		}

		
		if (test_and_clear_bit(GLUE_FLAG_HIF_FW_OWN_BIT, &prGlueInfo->ulFlag)) {
			prGlueInfo->prAdapter->fgWiFiInSleepyState = TRUE;
		}

		
		wlanReleasePowerControl(prGlueInfo->prAdapter);
	}

	complete(&prGlueInfo->rHifHaltComp);

	if (KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rHifThreadWakeLock)) {
		KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rHifThreadWakeLock);
	}
	KAL_WAKE_LOCK_DESTROY(prGlueInfo->prAdapter, &rHifThreadWakeLock);

	return 0;
}

int rx_thread(void *data)
{
	struct net_device *dev = data;
	P_GLUE_INFO_T prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(dev));

	QUE_T rTempRxQue;
	P_QUE_T prTempRxQue = NULL;
	P_QUE_ENTRY_T prQueueEntry = NULL;

	int ret = 0;
	KAL_WAKE_LOCK_T rRxThreadWakeLock;
    UINT_32             u4LoopCount;

	
	KAL_SPIN_LOCK_DECLARATION();

	KAL_WAKE_LOCK_INIT(prGlueInfo->prAdapter, &rRxThreadWakeLock, "WLAN rx_thread");
	KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rRxThreadWakeLock);

	DBGLOG(INIT, INFO, ("rx_thread starts running ID=%d\n", KAL_GET_CURRENT_THREAD_ID()));
	prGlueInfo->u4RxThreadPid = KAL_GET_CURRENT_THREAD_ID();

	set_user_nice(current, prGlueInfo->prAdapter->rWifiVar.cThreadNice);

    prTempRxQue = &rTempRxQue;
    
	while (TRUE) {

		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			DBGLOG(INIT, INFO, ("rx_thread should stop now...\n"));
			break;
		}

		
		if (!(prGlueInfo->ulFlag & GLUE_FLAG_RX_PROCESS)) {
			KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rRxThreadWakeLock);
		}

		do {
			ret = wait_event_interruptible(prGlueInfo->waitq_rx,
						       ((prGlueInfo->ulFlag & GLUE_FLAG_RX_PROCESS) != 0));
		} while (ret != 0);

		if (!KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rRxThreadWakeLock)) {
			KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rRxThreadWakeLock);
		}

		if (test_and_clear_bit(GLUE_FLAG_RX_TO_OS_BIT, &prGlueInfo->ulFlag)) {
            u4LoopCount = prGlueInfo->prAdapter->rWifiVar.u4Rx2OsLoopCount;

            while(u4LoopCount--) {
                while(QUEUE_IS_NOT_EMPTY(&prGlueInfo->prAdapter->rRxQueue)) {
			QUEUE_INITIALIZE(prTempRxQue);

			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_RX_TO_OS_QUE);
			QUEUE_MOVE_ALL(prTempRxQue, &prGlueInfo->prAdapter->rRxQueue);
			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_RX_TO_OS_QUE);

                    while(QUEUE_IS_NOT_EMPTY(prTempRxQue)) {
			QUEUE_REMOVE_HEAD(prTempRxQue, prQueueEntry, P_QUE_ENTRY_T);
                kalRxIndicateOnePkt(prGlueInfo, (PVOID)GLUE_GET_PKT_DESCRIPTOR(prQueueEntry));
			}

			KAL_WAKE_LOCK_TIMEOUT(prGlueInfo->prAdapter, &prGlueInfo->rTimeoutWakeLock,
					      MSEC_TO_JIFFIES(WAKE_LOCK_RX_TIMEOUT));
		}
	}
        }       
    }

	complete(&prGlueInfo->rRxHaltComp);

	if (KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rRxThreadWakeLock)) {
		KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rRxThreadWakeLock);
	}
	KAL_WAKE_LOCK_DESTROY(prGlueInfo->prAdapter, &rRxThreadWakeLock);

	return 0;
}
#endif



int tx_thread(void *data)
{
	struct net_device *dev = data;
	P_GLUE_INFO_T prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(dev));
	P_GL_IO_REQ_T prIoReq = NULL;
	int ret = 0;
	BOOLEAN fgNeedHwAccess = FALSE;
	KAL_WAKE_LOCK_T rTxThreadWakeLock;

#if CFG_SUPPORT_MULTITHREAD
	prGlueInfo->u4TxThreadPid = KAL_GET_CURRENT_THREAD_ID();
#endif

	current->flags |= PF_NOFREEZE;
	ASSERT(prGlueInfo);
	ASSERT(prGlueInfo->prAdapter);
	set_user_nice(current, prGlueInfo->prAdapter->rWifiVar.cThreadNice);

	KAL_WAKE_LOCK_INIT(prGlueInfo->prAdapter, &rTxThreadWakeLock, "WLAN tx_thread");
	KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rTxThreadWakeLock);

	DBGLOG(INIT, INFO, ("tx_thread starts running...\n"));

	while (TRUE) {

#if CFG_ENABLE_WIFI_DIRECT
		
		if (test_and_clear_bit(GLUE_FLAG_SUB_MOD_MULTICAST_BIT, &prGlueInfo->ulFlag)) {
			p2pSetMulticastListWorkQueueWrapper(prGlueInfo);
		}
#endif

		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			DBGLOG(INIT, INFO, ("tx_thread should stop now...\n"));
			break;
		}

		
		if (!(prGlueInfo->ulFlag & GLUE_FLAG_TX_PROCESS)) {
			KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rTxThreadWakeLock);
		}

		do {
			ret = wait_event_interruptible(prGlueInfo->waitq,
						       ((prGlueInfo->ulFlag & GLUE_FLAG_TX_PROCESS) != 0));
		} while (ret != 0);

		if (!KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rTxThreadWakeLock)) {
			KAL_WAKE_LOCK(prGlueInfo->prAdapter, &rTxThreadWakeLock);
		}
#if CFG_DBG_GPIO_PINS
		
		mtk_wcn_stp_debug_gpio_assert(IDX_TX_THREAD, DBG_TIE_LOW);
#endif

#if CFG_ENABLE_WIFI_DIRECT
		
		if (test_and_clear_bit(GLUE_FLAG_SUB_MOD_MULTICAST_BIT, &prGlueInfo->ulFlag)) {
			p2pSetMulticastListWorkQueueWrapper(prGlueInfo);
		}

		if (test_and_clear_bit(GLUE_FLAG_FRAME_FILTER_BIT, &prGlueInfo->ulFlag)) {
			p2pFuncUpdateMgmtFrameRegister(prGlueInfo->prAdapter,
						       prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter);
		}
#endif
		if (test_and_clear_bit(GLUE_FLAG_FRAME_FILTER_AIS_BIT, &prGlueInfo->ulFlag)) {
			P_AIS_FSM_INFO_T prAisFsmInfo = (P_AIS_FSM_INFO_T) NULL;
			
			prAisFsmInfo = &(prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo);
			prAisFsmInfo->u4AisPacketFilter = prGlueInfo->u4OsMgmtFrameFilter;
		}

		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			DBGLOG(INIT, INFO, ("<1>tx_thread should stop now...\n"));
			break;
		}

		fgNeedHwAccess = FALSE;

#if CFG_SUPPORT_SDIO_READ_WRITE_PATTERN
		if (prGlueInfo->fgEnSdioTestPattern == TRUE) {
			if (fgNeedHwAccess == FALSE) {
				fgNeedHwAccess = TRUE;

				wlanAcquirePowerControl(prGlueInfo->prAdapter);
			}

			if (prGlueInfo->fgIsSdioTestInitialized == FALSE) {
				
				kalDevRegWrite(prGlueInfo, MCR_WTMCR, 0x00080002);
				prGlueInfo->fgIsSdioTestInitialized = TRUE;
			}

			if (prGlueInfo->fgSdioReadWriteMode == TRUE) {
				
				kalDevPortRead(prGlueInfo,
					       MCR_WTMDR,
					       256,
					       prGlueInfo->aucSdioTestBuffer,
					       sizeof(prGlueInfo->aucSdioTestBuffer));
			} else {
				
				kalDevPortWrite(prGlueInfo,
						MCR_WTMDR,
						172,
						prGlueInfo->aucSdioTestBuffer,
						sizeof(prGlueInfo->aucSdioTestBuffer));
			}
		}
#endif
#if CFG_SUPPORT_MULTITHREAD
#else
		
		if (test_and_clear_bit(GLUE_FLAG_INT_BIT, &prGlueInfo->ulFlag)) {

			if (fgNeedHwAccess == FALSE) {
				fgNeedHwAccess = TRUE;

				wlanAcquirePowerControl(prGlueInfo->prAdapter);
			}

			prGlueInfo->prAdapter->fgIsIntEnable = FALSE;
			

			if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
				
				DBGLOG(INIT, INFO, ("ignore pending interrupt\n"));
			} else {
				wlanIST(prGlueInfo->prAdapter);
			}
		}
#endif
		
#if 0
		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			printk(KERN_INFO DRV_NAME "<2>tx_thread should stop now...\n");
			break;
		}
#endif

		do {
			if (test_and_clear_bit(GLUE_FLAG_OID_BIT, &prGlueInfo->ulFlag)) {
				
				prIoReq = &(prGlueInfo->OidEntry);
				if (FALSE == prIoReq->fgRead) {
					prIoReq->rStatus = wlanSetInformation(prIoReq->prAdapter,
									      prIoReq->
									      pfnOidHandler,
									      prIoReq->pvInfoBuf,
									      prIoReq->u4InfoBufLen,
									      prIoReq->
									      pu4QryInfoLen);
				} else {
					prIoReq->rStatus = wlanQueryInformation(prIoReq->prAdapter,
										prIoReq->
										pfnOidHandler,
										prIoReq->pvInfoBuf,
										prIoReq->
										u4InfoBufLen,
										prIoReq->
										pu4QryInfoLen);
				}

				if (prIoReq->rStatus != WLAN_STATUS_PENDING) {
					
					if (!completion_done(&prGlueInfo->rPendComp)) {
						complete(&prGlueInfo->rPendComp);
					} else {
						DBGLOG(INIT, WARN,
						       ("SKIP multiple OID complete!\n"));
					}
				} else {
					wlanoidTimeoutCheck(prGlueInfo->prAdapter,
							    prIoReq->pfnOidHandler);
				}
			}

		} while (FALSE);


#if 0
		if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
			printk(KERN_INFO DRV_NAME "<3>tx_thread should stop now...\n");
			break;
		}
#endif

		if (test_and_clear_bit(GLUE_FLAG_TXREQ_BIT, &prGlueInfo->ulFlag)) {
			kalProcessTxReq(prGlueInfo, &fgNeedHwAccess);
		}
#if CFG_SUPPORT_MULTITHREAD
		
		if (test_and_clear_bit(GLUE_FLAG_RX_BIT, &prGlueInfo->ulFlag)) {
			nicRxProcessRFBs(prGlueInfo->prAdapter);
		}
		if (test_and_clear_bit(GLUE_FLAG_TX_CMD_DONE_BIT, &prGlueInfo->ulFlag)) {
			wlanTxCmdDoneMthread(prGlueInfo->prAdapter);
		}
#endif

		

		

		
#if CFG_SUPPORT_MULTITHREAD
#else
		if (fgNeedHwAccess == TRUE) {
			wlanReleasePowerControl(prGlueInfo->prAdapter);
		}
#endif
		
		if (test_and_clear_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag)) {
			wlanTimerTimeoutCheck(prGlueInfo->prAdapter);
		}
#if CFG_SUPPORT_SDIO_READ_WRITE_PATTERN
		if (prGlueInfo->fgEnSdioTestPattern == TRUE) {
			kalSetEvent(prGlueInfo);
		}
#endif

#if CFG_DBG_GPIO_PINS
		
		if (!prGlueInfo->ulFlag) {
			mtk_wcn_stp_debug_gpio_assert(IDX_TX_THREAD, DBG_TIE_HIGH);
		}
#endif
	}

#if 0
	if (fgNeedHwAccess == TRUE) {
		wlanReleasePowerControl(prGlueInfo->prAdapter);
	}
#endif

	
	if (GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum) > 0) {
		kalFlushPendingTxPackets(prGlueInfo);
	}

	
	if (GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingSecurityFrameNum) > 0) {
		kalClearSecurityFrames(prGlueInfo);
	}

	
	wlanReleasePendingOid(prGlueInfo->prAdapter, 0);


	

	DBGLOG(INIT, INFO, ("mtk_sdiod stops\n"));
	complete(&prGlueInfo->rHaltComp);

	if (KAL_WAKE_LOCK_ACTIVE(prGlueInfo->prAdapter, &rTxThreadWakeLock)) {
		KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &rTxThreadWakeLock);
	}
	KAL_WAKE_LOCK_DESTROY(prGlueInfo->prAdapter, &rTxThreadWakeLock);

	return 0;

}

BOOLEAN kalIsCardRemoved(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return FALSE;
	
}

BOOLEAN kalRetrieveNetworkAddress(IN P_GLUE_INFO_T prGlueInfo, IN OUT PARAM_MAC_ADDRESS *prMacAddr)
{
	ASSERT(prGlueInfo);

	if (prGlueInfo->fgIsMacAddrOverride == FALSE) {
#if !defined(CONFIG_X86)
		UINT_32 i;
		BOOLEAN fgIsReadError = FALSE;

		for (i = 0; i < MAC_ADDR_LEN; i += 2) {
			if (kalCfgDataRead16(prGlueInfo,
					     OFFSET_OF(WIFI_CFG_PARAM_STRUCT, aucMacAddress) + i,
					     (PUINT_16) (((PUINT_8) prMacAddr) + i)) == FALSE) {
				fgIsReadError = TRUE;
				break;
			}
		}

		if (fgIsReadError == TRUE) {
			return FALSE;
		} else {
			return TRUE;
		}
#else
		
		return FALSE;
#endif
	} else {
		COPY_MAC_ADDR(prMacAddr, prGlueInfo->rMacAddrOverride);

		return TRUE;
	}
}


VOID kalFlushPendingTxPackets(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prTxQue;
	P_QUE_ENTRY_T prQueueEntry;
	PVOID prPacket;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	prTxQue = &(prGlueInfo->rTxQueue);

	if (GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum)) {
		while (TRUE) {
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
			QUEUE_REMOVE_HEAD(prTxQue, prQueueEntry, P_QUE_ENTRY_T);
			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

			if (prQueueEntry == NULL) {
				break;
			}

			prPacket = GLUE_GET_PKT_DESCRIPTOR(prQueueEntry);

			kalSendComplete(prGlueInfo, prPacket, WLAN_STATUS_NOT_ACCEPTED);
		}
	}
}

ENUM_PARAM_MEDIA_STATE_T kalGetMediaStateIndicated(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return prGlueInfo->eParamMediaStateIndicated;
}


VOID
kalSetMediaStateIndicated(IN P_GLUE_INFO_T prGlueInfo,
			  IN ENUM_PARAM_MEDIA_STATE_T eParamMediaStateIndicate)
{
	ASSERT(prGlueInfo);

	prGlueInfo->eParamMediaStateIndicated = eParamMediaStateIndicate;
}


VOID kalOidCmdClearance(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prCmdQue;
	QUE_T rTempCmdQue;
	P_QUE_T prTempCmdQue = &rTempCmdQue;
	QUE_T rReturnCmdQue;
	P_QUE_T prReturnCmdQue = &rReturnCmdQue;
	P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T) NULL;
	P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T) NULL;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	QUEUE_INITIALIZE(prReturnCmdQue);

	prCmdQue = &prGlueInfo->rCmdQueue;

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

	QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	while (prQueueEntry) {

		if (((P_CMD_INFO_T) prQueueEntry)->fgIsOid) {
			prCmdInfo = (P_CMD_INFO_T) prQueueEntry;
			break;
		} else {
			QUEUE_INSERT_TAIL(prReturnCmdQue, prQueueEntry);
		}

		QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_CONCATENATE_QUEUES_HEAD(prCmdQue, prReturnCmdQue);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);


	if (prCmdInfo) {
		if (prCmdInfo->pfCmdTimeoutHandler) {
			prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
		} else {
			kalOidComplete(prGlueInfo,
				       prCmdInfo->fgSetQuery, 0, WLAN_STATUS_NOT_ACCEPTED);
		}

		prGlueInfo->u4OidCompleteFlag = 1;
		cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
	}
}


VOID kalEnqueueCommand(IN P_GLUE_INFO_T prGlueInfo, IN P_QUE_ENTRY_T prQueueEntry)
{
	P_QUE_T prCmdQue;
	P_CMD_INFO_T prCmdInfo;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);
	ASSERT(prQueueEntry);

	prCmdQue = &prGlueInfo->rCmdQueue;

	prCmdInfo = (P_CMD_INFO_T) prQueueEntry;

	DBGLOG(INIT, INFO, ("EN-Q CMD TYPE[%u] ID[0x%02X] SEQ[%u] to CMD Q\n",
			    prCmdInfo->eCmdType, prCmdInfo->ucCID, prCmdInfo->ucCmdSeqNum));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
	QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}

VOID kalHandleAssocInfo(IN P_GLUE_INFO_T prGlueInfo, IN P_EVENT_ASSOC_INFO prAssocInfo)
{
	
}

UINT_32 kalGetFwLoadAddress(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return prGlueInfo->rRegInfo.u4LoadAddress;
}


UINT_32 kalGetFwStartAddress(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return prGlueInfo->rRegInfo.u4StartAddress;
}


VOID
kalSecurityFrameSendComplete(IN P_GLUE_INFO_T prGlueInfo, IN PVOID pvPacket, IN WLAN_STATUS rStatus)
{
	ASSERT(pvPacket);

	
	kalSendCompleteAndAwakeQueue(prGlueInfo, pvPacket);
	GLUE_DEC_REF_CNT(prGlueInfo->i4TxPendingSecurityFrameNum);
}

UINT_32 kalGetTxPendingFrameCount(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return (UINT_32) (GLUE_GET_REF_CNT(prGlueInfo->i4TxPendingFrameNum));
}


UINT_32 kalGetTxPendingCmdCount(IN P_GLUE_INFO_T prGlueInfo)
{
	P_QUE_T prCmdQue;

	ASSERT(prGlueInfo);
	prCmdQue = &prGlueInfo->rCmdQueue;

	return prCmdQue->u4NumElem;
}




VOID kalOsTimerInitialize(IN P_GLUE_INFO_T prGlueInfo, IN PVOID prTimerHandler)
{

	ASSERT(prGlueInfo);

	init_timer(&(prGlueInfo->tickfn));
	prGlueInfo->tickfn.function = prTimerHandler;
	prGlueInfo->tickfn.data = (unsigned long)prGlueInfo;
}

BOOLEAN kalSetTimer(IN P_GLUE_INFO_T prGlueInfo, IN UINT_32 u4Interval)
{
	ASSERT(prGlueInfo);
	del_timer_sync(&(prGlueInfo->tickfn));

	prGlueInfo->tickfn.expires = jiffies + u4Interval * HZ / MSEC_PER_SEC;
	add_timer(&(prGlueInfo->tickfn));

	return TRUE;		
}

BOOLEAN kalCancelTimer(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	clear_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag);

	if (del_timer_sync(&(prGlueInfo->tickfn)) >= 0) {
		return TRUE;
	} else {
		return FALSE;
	}
}

VOID
kalScanDone(IN P_GLUE_INFO_T prGlueInfo,
	    IN ENUM_KAL_NETWORK_TYPE_INDEX_T eNetTypeIdx, IN WLAN_STATUS status)
{
	ASSERT(prGlueInfo);

	scanReportBss2Cfg80211(prGlueInfo->prAdapter, BSS_TYPE_INFRASTRUCTURE, NULL);

	
	wlanCheckSystemConfiguration(prGlueInfo->prAdapter);

	kalIndicateStatusAndComplete(prGlueInfo, WLAN_STATUS_SCAN_COMPLETE, NULL, 0);
}

UINT_32 kalRandomNumber(VOID)
{
	UINT_32 number = 0;

	get_random_bytes(&number, 4);

	return number;
}

VOID kalTimeoutHandler(unsigned long arg)
{

	P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) arg;

	ASSERT(prGlueInfo);

	
	set_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag);
	wake_up_interruptible(&prGlueInfo->waitq);

	return;
}


VOID kalSetEvent(P_GLUE_INFO_T pr)
{
	set_bit(GLUE_FLAG_TXREQ_BIT, &pr->ulFlag);
	wake_up_interruptible(&pr->waitq);
}

#if CFG_SUPPORT_MULTITHREAD
VOID kalSetTxEvent2Hif(P_GLUE_INFO_T pr)
{
	if (!pr->hif_thread) {
		return;
	}

	KAL_WAKE_LOCK_TIMEOUT(pr->prAdapter, &pr->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_THREAD_WAKEUP_TIMEOUT));

	set_bit(GLUE_FLAG_HIF_TX_BIT, &pr->ulFlag);
	wake_up_interruptible(&pr->waitq_hif);
}

VOID kalSetFwOwnEvent2Hif(P_GLUE_INFO_T pr)
{
	if (!pr->hif_thread) {
		return;
	}

	KAL_WAKE_LOCK_TIMEOUT(pr->prAdapter, &pr->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_THREAD_WAKEUP_TIMEOUT));

	set_bit(GLUE_FLAG_HIF_FW_OWN_BIT, &pr->ulFlag);
	wake_up_interruptible(&pr->waitq_hif);
}

VOID kalSetTxEvent2Rx(P_GLUE_INFO_T pr)
{
	if (!pr->rx_thread) {
		return;
	}

	KAL_WAKE_LOCK_TIMEOUT(pr->prAdapter, &pr->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_THREAD_WAKEUP_TIMEOUT));

	set_bit(GLUE_FLAG_RX_TO_OS_BIT, &pr->ulFlag);
	wake_up_interruptible(&pr->waitq_rx);
}

VOID kalSetTxCmdEvent2Hif(P_GLUE_INFO_T pr)
{
	if (!pr->hif_thread) {
		return;
	}

	KAL_WAKE_LOCK_TIMEOUT(pr->prAdapter, &pr->rTimeoutWakeLock,
			      MSEC_TO_JIFFIES(WAKE_LOCK_THREAD_WAKEUP_TIMEOUT));

	set_bit(GLUE_FLAG_HIF_TX_CMD_BIT, &pr->ulFlag);
	wake_up_interruptible(&pr->waitq_hif);
}
#endif
BOOLEAN kalIsConfigurationExist(IN P_GLUE_INFO_T prGlueInfo)
{
#if !defined(CONFIG_X86)
	ASSERT(prGlueInfo);

	return prGlueInfo->fgNvramAvailable;
#else
	
	return FALSE;
#endif
}


P_REG_INFO_T kalGetConfiguration(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return &(prGlueInfo->rRegInfo);
}


VOID
kalGetConfigurationVersion(IN P_GLUE_INFO_T prGlueInfo,
			   OUT PUINT_16 pu2Part1CfgOwnVersion,
			   OUT PUINT_16 pu2Part1CfgPeerVersion,
			   OUT PUINT_16 pu2Part2CfgOwnVersion, OUT PUINT_16 pu2Part2CfgPeerVersion)
{
	ASSERT(prGlueInfo);

	ASSERT(pu2Part1CfgOwnVersion);
	ASSERT(pu2Part1CfgPeerVersion);
	ASSERT(pu2Part2CfgOwnVersion);
	ASSERT(pu2Part2CfgPeerVersion);

	kalCfgDataRead16(prGlueInfo,
			 OFFSET_OF(WIFI_CFG_PARAM_STRUCT, u2Part1OwnVersion),
			 pu2Part1CfgOwnVersion);

	kalCfgDataRead16(prGlueInfo,
			 OFFSET_OF(WIFI_CFG_PARAM_STRUCT, u2Part1PeerVersion),
			 pu2Part1CfgPeerVersion);

	kalCfgDataRead16(prGlueInfo,
			 OFFSET_OF(WIFI_CFG_PARAM_STRUCT, u2Part2OwnVersion),
			 pu2Part2CfgOwnVersion);

	kalCfgDataRead16(prGlueInfo,
			 OFFSET_OF(WIFI_CFG_PARAM_STRUCT, u2Part2PeerVersion),
			 pu2Part2CfgPeerVersion);

	return;
}

BOOLEAN kalWSCGetActiveState(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return prGlueInfo->fgWpsActive;
}


VOID
kalUpdateRSSI(IN P_GLUE_INFO_T prGlueInfo,
	      IN ENUM_KAL_NETWORK_TYPE_INDEX_T eNetTypeIdx, IN INT_8 cRssi, IN INT_8 cLinkQuality)
{
	struct iw_statistics *pStats = (struct iw_statistics *)NULL;

	ASSERT(prGlueInfo);

	switch (eNetTypeIdx) {
	case KAL_NETWORK_TYPE_AIS_INDEX:
		pStats = (struct iw_statistics *)(&(prGlueInfo->rIwStats));
		break;
#if CFG_ENABLE_WIFI_DIRECT
#if CFG_SUPPORT_P2P_RSSI_QUERY
	case KAL_NETWORK_TYPE_P2P_INDEX:
		pStats = (struct iw_statistics *)(&(prGlueInfo->rP2pIwStats));
		break;
#endif
#endif
	default:
		break;

	}

	if (pStats) {
		pStats->qual.qual = cLinkQuality;
		pStats->qual.noise = 0;
		pStats->qual.updated = IW_QUAL_QUAL_UPDATED | IW_QUAL_NOISE_UPDATED;
		pStats->qual.level = 0x100 + cRssi;
		pStats->qual.updated |= IW_QUAL_LEVEL_UPDATED;
	}


	return;
}


BOOLEAN kalInitIOBuffer(VOID)
{
	UINT_32 u4Size;

	if (CFG_COALESCING_BUFFER_SIZE >= CFG_RX_COALESCING_BUFFER_SIZE) {
		u4Size = CFG_COALESCING_BUFFER_SIZE + sizeof(ENHANCE_MODE_DATA_STRUCT_T);
	} else {
		u4Size = CFG_RX_COALESCING_BUFFER_SIZE + sizeof(ENHANCE_MODE_DATA_STRUCT_T);
	}

	pvIoBuffer = kmalloc(u4Size, GFP_KERNEL);
	if (pvIoBuffer) {
		pvIoBufferSize = u4Size;
		pvIoBufferUsage = 0;

		return TRUE;
	}

	return FALSE;
}


VOID kalUninitIOBuffer(VOID)
{
	if (pvIoBuffer) {
		kfree(pvIoBuffer);

		pvIoBuffer = (PVOID) NULL;
		pvIoBufferSize = 0;
		pvIoBufferUsage = 0;
	}

	return;
}


PVOID kalAllocateIOBuffer(IN UINT_32 u4AllocSize)
{
	PVOID ret = (PVOID) NULL;

	if (pvIoBuffer) {
		if (u4AllocSize <= (pvIoBufferSize - pvIoBufferUsage)) {
			ret = (PVOID) &(((PUINT_8) (pvIoBuffer))[pvIoBufferUsage]);
			pvIoBufferUsage += u4AllocSize;
		}
	} else {
		
		ret = (PVOID) kalMemAlloc(u4AllocSize, PHY_MEM_TYPE);
	}

	return ret;
}


VOID kalReleaseIOBuffer(IN PVOID pvAddr, IN UINT_32 u4Size)
{
	if (pvIoBuffer) {
		pvIoBufferUsage -= u4Size;
	} else {
		
		kalMemFree(pvAddr, PHY_MEM_TYPE, u4Size);
	}
}

VOID
kalGetChannelList(IN P_GLUE_INFO_T prGlueInfo,
		  IN ENUM_BAND_T eSpecificBand,
		  IN UINT_8 ucMaxChannelNum,
		  IN PUINT_8 pucNumOfChannel, IN P_RF_CHANNEL_INFO_T paucChannelList)
{
	rlmDomainGetChnlList(prGlueInfo->prAdapter,
			     eSpecificBand, ucMaxChannelNum, pucNumOfChannel, paucChannelList);
}


BOOL kalIsAPmode(IN P_GLUE_INFO_T prGlueInfo)
{
#if 0				
#if CFG_ENABLE_WIFI_DIRECT
	if (IS_NET_ACTIVE(prGlueInfo->prAdapter, NETWORK_TYPE_P2P_INDEX) &&
	    p2pFuncIsAPMode(prGlueInfo->prAdapter->rWifiVar.prP2pFsmInfo))
		return TRUE;
#endif
#endif

	return FALSE;
}


#if CFG_SUPPORT_802_11W
UINT_32 kalGetMfpSetting(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

	return prGlueInfo->rWpaInfo.u4Mfp;
}
#endif

struct file *kalFileOpen(const char *path, int flags, int rights)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}

VOID kalFileClose(struct file *file)
{
	filp_close(file, NULL);
}

UINT_32
kalFileRead(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}

UINT_32
kalFileWrite(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, &offset);

	set_fs(oldfs);
	return ret;
}

UINT_32 kalWriteToFile(const PUINT_8 pucPath, BOOLEAN fgDoAppend, PUINT_8 pucData, UINT_32 u4Size)
{
	struct file *file = NULL;
	UINT_32 ret;
	UINT_32 u4Flags = 0;

	if (fgDoAppend) {
		u4Flags = O_APPEND;
	}

	file = kalFileOpen(pucPath, O_WRONLY | O_CREAT | u4Flags, S_IRWXU);
	ret = kalFileWrite(file, 0, pucData, u4Size);
	kalFileClose(file);

	return ret;
}


INT_32 kalReadToFile(const PUINT_8 pucPath, PUINT_8 pucData, UINT_32 u4Size, PUINT_32 pu4ReadSize)
{
	struct file *file = NULL;
	INT_32 ret = -1;
	UINT_32 u4ReadSize = 0;

	DBGLOG(INIT, INFO, ("kalReadToFile() path %s\n", pucPath));

	file = kalFileOpen(pucPath, O_RDONLY, 0);

	if ((file != NULL) && !IS_ERR(file)) {
		u4ReadSize = kalFileRead(file, 0, pucData, u4Size);
		kalFileClose(file);
		if (pu4ReadSize)
			*pu4ReadSize = u4ReadSize;
		ret = 0;
	}
	return ret;
}

UINT_32 kalCheckPath(const PUINT_8 pucPath)
{
	struct file *file = NULL;
	UINT_32 u4Flags = 0;

	file = kalFileOpen(pucPath, O_WRONLY | O_CREAT | u4Flags, S_IRWXU);
	if (!file) {
		return -1;
	}


	kalFileClose(file);
	return 1;
}

VOID
kalIndicateBssInfo(IN P_GLUE_INFO_T prGlueInfo,
		   IN PUINT_8 pucBeaconProbeResp,
		   IN UINT_32 u4FrameLen, IN UINT_8 ucChannelNum, IN INT_32 i4SignalStrength)
{
	struct wiphy *wiphy;
	struct ieee80211_channel *prChannel = NULL;

	ASSERT(prGlueInfo);
	wiphy = priv_to_wiphy(prGlueInfo);

	
	if (ucChannelNum <= 14) {
		prChannel =
		    ieee80211_get_channel(wiphy,
					  ieee80211_channel_to_frequency(ucChannelNum,
									 IEEE80211_BAND_2GHZ));
	} else {
		prChannel =
		    ieee80211_get_channel(wiphy,
					  ieee80211_channel_to_frequency(ucChannelNum,
									 IEEE80211_BAND_5GHZ));
	}

	if (prChannel != NULL && prGlueInfo->fgIsRegistered == TRUE) {
		struct cfg80211_bss *bss;
#if CFG_SUPPORT_TSF_USING_BOOTTIME
		struct ieee80211_mgmt *prMgmtFrame = (struct ieee80211_mgmt *)pucBeaconProbeResp;
		prMgmtFrame->u.beacon.timestamp = kalGetBootTime();
#endif

		
		bss = cfg80211_inform_bss_frame(wiphy,
						prChannel,
						(struct ieee80211_mgmt *)pucBeaconProbeResp,
						u4FrameLen, i4SignalStrength * 100, GFP_KERNEL);

		if (!bss) {
			DBGLOG(REQ, WARN, ("cfg80211_inform_bss_frame() returned with NULL\n"));
		} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
			cfg80211_put_bss(wiphy, bss);
#else
			cfg80211_put_bss(bss);
#endif
		}
	}

	return;
}

VOID
kalReadyOnChannel(IN P_GLUE_INFO_T prGlueInfo,
		  IN UINT_64 u8Cookie,
		  IN ENUM_BAND_T eBand,
		  IN ENUM_CHNL_EXT_T eSco, IN UINT_8 ucChannelNum, IN UINT_32 u4DurationMs)
{
	struct ieee80211_channel *prChannel = NULL;
	enum nl80211_channel_type rChannelType;

	

	if (prGlueInfo->fgIsRegistered == TRUE) {
		if (ucChannelNum <= 14) {
			prChannel =
			    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
						  ieee80211_channel_to_frequency(ucChannelNum,
										 IEEE80211_BAND_2GHZ));
		} else {
			prChannel =
			    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
						  ieee80211_channel_to_frequency(ucChannelNum,
										 IEEE80211_BAND_5GHZ));
		}

		switch (eSco) {
		case CHNL_EXT_SCN:
			rChannelType = NL80211_CHAN_NO_HT;
			break;

		case CHNL_EXT_SCA:
			rChannelType = NL80211_CHAN_HT40MINUS;
			break;

		case CHNL_EXT_SCB:
			rChannelType = NL80211_CHAN_HT40PLUS;
			break;

		case CHNL_EXT_RES:
		default:
			rChannelType = NL80211_CHAN_HT20;
			break;
		}

		cfg80211_ready_on_channel(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
						 prGlueInfo->prDevHandler->ieee80211_ptr,
#else
						 prGlueInfo->prDevHandler,
#endif
						 u8Cookie, prChannel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
						 rChannelType,
#endif
						 u4DurationMs, GFP_KERNEL);
	}

	return;
}

VOID
kalRemainOnChannelExpired(IN P_GLUE_INFO_T prGlueInfo,
			  IN UINT_64 u8Cookie,
			  IN ENUM_BAND_T eBand, IN ENUM_CHNL_EXT_T eSco, IN UINT_8 ucChannelNum)
{
	struct ieee80211_channel *prChannel = NULL;
	enum nl80211_channel_type rChannelType;

	ucChannelNum =
	    wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter,
					  prGlueInfo->prAdapter->prAisBssInfo->ucBssIndex);

	if (prGlueInfo->fgIsRegistered == TRUE) {
		if (ucChannelNum <= 14) {
			prChannel =
			    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
						  ieee80211_channel_to_frequency(ucChannelNum,
										 IEEE80211_BAND_2GHZ));
		} else {
			prChannel =
			    ieee80211_get_channel(priv_to_wiphy(prGlueInfo),
						  ieee80211_channel_to_frequency(ucChannelNum,
										 IEEE80211_BAND_5GHZ));
		}

		switch (eSco) {
		case CHNL_EXT_SCN:
			rChannelType = NL80211_CHAN_NO_HT;
			break;

		case CHNL_EXT_SCA:
			rChannelType = NL80211_CHAN_HT40MINUS;
			break;

		case CHNL_EXT_SCB:
			rChannelType = NL80211_CHAN_HT40PLUS;
			break;

		case CHNL_EXT_RES:
		default:
			rChannelType = NL80211_CHAN_HT20;
			break;
		}

		cfg80211_remain_on_channel_expired(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
							  prGlueInfo->prDevHandler->ieee80211_ptr,
#else
							  prGlueInfo->prDevHandler,
#endif
							  u8Cookie, prChannel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
							  rChannelType,
#endif
							  GFP_KERNEL);
	}

	return;
}


VOID
kalIndicateMgmtTxStatus(IN P_GLUE_INFO_T prGlueInfo,
			IN UINT_64 u8Cookie,
			IN BOOLEAN fgIsAck, IN PUINT_8 pucFrameBuf, IN UINT_32 u4FrameLen)
{

	do {
		if ((prGlueInfo == NULL)
		    || (pucFrameBuf == NULL)
		    || (u4FrameLen == 0)) {
			DBGLOG(AIS, TRACE,
			       ("Unexpected pointer PARAM. 0x%lx, 0x%lx, %ld.", prGlueInfo,
				pucFrameBuf, u4FrameLen));
			ASSERT(FALSE);
			break;
		}

		cfg80211_mgmt_tx_status(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
					       prGlueInfo->prDevHandler->ieee80211_ptr,
#else
					       prGlueInfo->prDevHandler,
#endif
					       u8Cookie,
					       pucFrameBuf, u4FrameLen, fgIsAck, GFP_KERNEL);

	} while (FALSE);

}				


VOID kalIndicateRxMgmtFrame(IN P_GLUE_INFO_T prGlueInfo, IN P_SW_RFB_T prSwRfb)
{
	INT_32 i4Freq = 0;
	UINT_8 ucChnlNum = 0;

	do {
		if ((prGlueInfo == NULL) || (prSwRfb == NULL)) {
			ASSERT(FALSE);
			break;
		}

		ucChnlNum = (UINT_8) HAL_RX_STATUS_GET_CHNL_NUM(prSwRfb->prRxStatus);

		i4Freq = nicChannelNum2Freq(ucChnlNum) / 1000;

		cfg80211_rx_mgmt(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
					prGlueInfo->prDevHandler->ieee80211_ptr,
#else
					prGlueInfo->prDevHandler,
#endif
					i4Freq,	
					RCPI_TO_dBm((UINT_8)
						    HAL_RX_STATUS_GET_RCPI(prSwRfb->
									   prRxStatusGroup3)),
					prSwRfb->pvHeader, prSwRfb->u2PacketLen, GFP_ATOMIC);
	} while (FALSE);

}				


#if CFG_SUPPORT_SDIO_READ_WRITE_PATTERN
BOOLEAN kalSetSdioTestPattern(IN P_GLUE_INFO_T prGlueInfo, IN BOOLEAN fgEn, IN BOOLEAN fgRead)
{
	const UINT_8 aucPattern[] = {
		0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55,
		0xaa, 0x55, 0x80, 0x80, 0x80, 0x7f, 0x80, 0x80,
		0x80, 0x7f, 0x7f, 0x7f, 0x80, 0x7f, 0x7f, 0x7f,
		0x40, 0x40, 0x40, 0xbf, 0x40, 0x40, 0x40, 0xbf,
		0xbf, 0xbf, 0x40, 0xbf, 0xbf, 0xbf, 0x20, 0x20,
		0x20, 0xdf, 0x20, 0x20, 0x20, 0xdf, 0xdf, 0xdf,
		0x20, 0xdf, 0xdf, 0xdf, 0x10, 0x10, 0x10, 0xef,
		0x10, 0x10, 0x10, 0xef, 0xef, 0xef, 0x10, 0xef,
		0xef, 0xef, 0x08, 0x08, 0x08, 0xf7, 0x08, 0x08,
		0x08, 0xf7, 0xf7, 0xf7, 0x08, 0xf7, 0xf7, 0xf7,
		0x04, 0x04, 0x04, 0xfb, 0x04, 0x04, 0x04, 0xfb,
		0xfb, 0xfb, 0x04, 0xfb, 0xfb, 0xfb, 0x02, 0x02,
		0x02, 0xfd, 0x02, 0x02, 0x02, 0xfd, 0xfd, 0xfd,
		0x02, 0xfd, 0xfd, 0xfd, 0x01, 0x01, 0x01, 0xfe,
		0x01, 0x01, 0x01, 0xfe, 0xfe, 0xfe, 0x01, 0xfe,
		0xfe, 0xfe, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00,
		0x00, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff,
		0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff,
		0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
		0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
		0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff,
		0x00, 0x00, 0x00, 0xff
	};
	UINT_32 i;

	ASSERT(prGlueInfo);

	
	prGlueInfo->fgEnSdioTestPattern = fgEn;
	prGlueInfo->fgSdioReadWriteMode = fgRead;

	if (fgRead == FALSE) {
		/* fill buffer for data to be written */
		for (i = 0; i < sizeof(aucPattern); i++) {
			prGlueInfo->aucSdioTestBuffer[i] = aucPattern[i];
		}
	}

	return TRUE;
}
#endif

#if (CFG_MET_PACKET_TRACE_SUPPORT == 1)
#define PROC_MET_PROF_CTRL                 "met_ctrl"
#define PROC_MET_PROF_PORT                 "met_port"

struct proc_dir_entry *pMetProcDir; 
void *pMetGlobalData = NULL;

#endif
VOID kalSchedScanResults(IN P_GLUE_INFO_T prGlueInfo)
{
	ASSERT(prGlueInfo);

    scanReportBss2Cfg80211(prGlueInfo->prAdapter, BSS_TYPE_INFRASTRUCTURE, NULL);

    
	cfg80211_sched_scan_results(priv_to_wiphy(prGlueInfo));
    

	return;
}


VOID kalSchedScanStopped(IN P_GLUE_INFO_T prGlueInfo)
{
    

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

    #if 1
	
	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
	if (prGlueInfo->prSchedScanRequest != NULL) {
		prGlueInfo->prSchedScanRequest = NULL;
	}
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
    #endif
    DBGLOG(SCN, INFO, ("cfg80211_sched_scan_stopped send event \n" ));

	
    
    DBGLOG(SCN, INFO, ("start work queue to send event  \n" ));
    schedule_delayed_work(&sched_workq, 0); 
    DBGLOG(SCN, INFO, ("Tx_thread return from kalSchedScanStoppped \n" ));
	return;
}

BOOLEAN
kalGetIPv4Address(IN struct net_device *prDev,
		  IN UINT_32 u4MaxNumOfAddr,
		  OUT PUINT_8 pucIpv4Addrs, OUT PUINT_32 pu4NumOfIpv4Addr)
{
	UINT_32 u4NumIPv4 = 0;
	UINT_32 u4AddrLen = IPV4_ADDR_LEN;
	struct in_ifaddr *prIfa;

	
	if (!prDev || !(prDev->ip_ptr) || !((struct in_device *)(prDev->ip_ptr))->ifa_list) {
		DBGLOG(INIT, INFO, ("IPv4 address is not avaliable for dev(0x%p)\n", prDev));

		*pu4NumOfIpv4Addr = 0;
		return FALSE;
	}

	prIfa = ((struct in_device *)(prDev->ip_ptr))->ifa_list;

	
	while ((u4NumIPv4 < u4MaxNumOfAddr) && prIfa) {
		kalMemCopy(&pucIpv4Addrs[u4NumIPv4 * u4AddrLen], &prIfa->ifa_local, u4AddrLen);
		prIfa = prIfa->ifa_next;

		DBGLOG(INIT, INFO,
		       ("IPv4 addr [%u][" IPV4STR "]\n", u4NumIPv4,
			IPV4TOSTR(&pucIpv4Addrs[u4NumIPv4 * u4AddrLen])));

		u4NumIPv4++;
	}

	*pu4NumOfIpv4Addr = u4NumIPv4;

	return TRUE;
}

BOOLEAN
kalGetIPv6Address(IN struct net_device *prDev,
		  IN UINT_32 u4MaxNumOfAddr,
		  OUT PUINT_8 pucIpv6Addrs, OUT PUINT_32 pu4NumOfIpv6Addr)
{
	UINT_32 u4NumIPv6 = 0;
	UINT_32 u4AddrLen = IPV6_ADDR_LEN;
	struct inet6_ifaddr *prIfa;
	struct list_head *prAddrList;

	
	if (!prDev || !(prDev->ip6_ptr)) {
		DBGLOG(INIT, INFO, ("IPv6 address is not avaliable for dev(0x%p)\n", prDev));

		*pu4NumOfIpv6Addr = 0;
		return FALSE;
	}

	prAddrList = &((struct inet6_dev *)(prDev->ip6_ptr))->addr_list;

	
	list_for_each_entry(prIfa, prAddrList, if_list) {
		kalMemCopy(&pucIpv6Addrs[u4NumIPv6 * u4AddrLen], &prIfa->addr, u4AddrLen);

		DBGLOG(INIT, INFO,
		       ("IPv6 addr [%u][" IPV6STR "]\n", u4NumIPv6,
			IPV6TOSTR(&pucIpv6Addrs[u4NumIPv6 * u4AddrLen])));

		if ((u4NumIPv6 + 1) >= u4MaxNumOfAddr) {
			break;
		}
		u4NumIPv6++;
	}

	*pu4NumOfIpv6Addr = u4NumIPv6;

	return TRUE;
}

VOID
kalSetNetAddress(IN P_GLUE_INFO_T prGlueInfo,
		 IN UINT_8 ucBssIdx,
		 IN PUINT_8 pucIPv4Addr,
		 IN UINT_32 u4NumIPv4Addr, IN PUINT_8 pucIPv6Addr, IN UINT_32 u4NumIPv6Addr)
{
	WLAN_STATUS rStatus = WLAN_STATUS_FAILURE;
	UINT_32 u4SetInfoLen = 0;
	UINT_32 u4Len = OFFSET_OF(PARAM_NETWORK_ADDRESS_LIST, arAddress);
	P_PARAM_NETWORK_ADDRESS_LIST prParamNetAddrList;
	P_PARAM_NETWORK_ADDRESS prParamNetAddr;
	UINT_32 i, u4AddrLen;

	
	
	u4Len += (((sizeof(PARAM_NETWORK_ADDRESS) - 1) + IPV4_ADDR_LEN) * u4NumIPv4Addr);
	
	u4Len += (((sizeof(PARAM_NETWORK_ADDRESS) - 1) + IPV6_ADDR_LEN) * u4NumIPv6Addr);

	
	prParamNetAddrList = (P_PARAM_NETWORK_ADDRESS_LIST) kalMemAlloc(u4Len, VIR_MEM_TYPE);

	if (!prParamNetAddrList) {
		DBGLOG(INIT, WARN,
		       ("Fail to alloc buffer for setting BSS[%u] network address!\n", ucBssIdx));
		return;
	}
	
	prParamNetAddrList->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;
	prParamNetAddrList->u4AddressCount = 0;
	prParamNetAddrList->ucBssIdx = ucBssIdx;

	
	u4AddrLen = IPV4_ADDR_LEN;
	prParamNetAddr = prParamNetAddrList->arAddress;
	for (i = 0; i < u4NumIPv4Addr; i++) {
		prParamNetAddr->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;
		prParamNetAddr->u2AddressLength = u4AddrLen;
		kalMemCopy(prParamNetAddr->aucAddress, &pucIPv4Addr[i * u4AddrLen], u4AddrLen);

		prParamNetAddr = (P_PARAM_NETWORK_ADDRESS) ((ULONG) prParamNetAddr +
							    (ULONG) (u4AddrLen +
								       OFFSET_OF
								       (PARAM_NETWORK_ADDRESS,
									aucAddress)));
	}
	prParamNetAddrList->u4AddressCount += u4NumIPv4Addr;

	
	u4AddrLen = IPV6_ADDR_LEN;
	for (i = 0; i < u4NumIPv6Addr; i++) {
		prParamNetAddr->u2AddressType = PARAM_PROTOCOL_ID_TCP_IP;
		prParamNetAddr->u2AddressLength = u4AddrLen;
		kalMemCopy(prParamNetAddr->aucAddress, &pucIPv6Addr[i * u4AddrLen], u4AddrLen);

		prParamNetAddr = (P_PARAM_NETWORK_ADDRESS) ((ULONG) prParamNetAddr +
							    (ULONG) (u4AddrLen +
								       OFFSET_OF
								       (PARAM_NETWORK_ADDRESS,
									aucAddress)));
	}
	prParamNetAddrList->u4AddressCount += u4NumIPv6Addr;

	
	rStatus = kalIoctl(prGlueInfo,
			   wlanoidSetNetworkAddress,
			   (PVOID) prParamNetAddrList, u4Len, FALSE, FALSE, TRUE, &u4SetInfoLen);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(REQ, WARN, ("%s: Fail to set network address\n", __func__));
	}

	kalMemFree(prParamNetAddrList, VIR_MEM_TYPE, u4Len);

}

VOID
kalSetNetAddressFromInterface(IN P_GLUE_INFO_T prGlueInfo,
			      IN struct net_device *prDev, IN BOOLEAN fgSet)
{
	UINT_32 u4NumIPv4, u4NumIPv6;
	UINT_8 pucIPv4Addr[IPV4_ADDR_LEN * CFG_PF_ARP_NS_MAX_NUM],
	    pucIPv6Addr[IPV6_ADDR_LEN * CFG_PF_ARP_NS_MAX_NUM];
	P_NETDEV_PRIVATE_GLUE_INFO prNetDevPrivate = (P_NETDEV_PRIVATE_GLUE_INFO) NULL;

	prNetDevPrivate = (P_NETDEV_PRIVATE_GLUE_INFO) netdev_priv(prDev);

	if (prNetDevPrivate->prGlueInfo != prGlueInfo) {
		DBGLOG(REQ, WARN,
		       ("%s: unexpected prGlueInfo(0x%p)!\n", __func__,
			prNetDevPrivate->prGlueInfo));
	}

	u4NumIPv4 = 0;
	u4NumIPv6 = 0;

	if (fgSet) {
		kalGetIPv4Address(prDev, CFG_PF_ARP_NS_MAX_NUM, pucIPv4Addr, &u4NumIPv4);
		kalGetIPv6Address(prDev, CFG_PF_ARP_NS_MAX_NUM, pucIPv6Addr, &u4NumIPv6);
	}

	if (u4NumIPv4 + u4NumIPv6 > CFG_PF_ARP_NS_MAX_NUM) {
		if (u4NumIPv4 >= CFG_PF_ARP_NS_MAX_NUM) {
			u4NumIPv4 = CFG_PF_ARP_NS_MAX_NUM;
			u4NumIPv6 = 0;
		} else {
			u4NumIPv6 = CFG_PF_ARP_NS_MAX_NUM - u4NumIPv4;
		}
	}

	kalSetNetAddress(prGlueInfo, prNetDevPrivate->ucBssIdx, pucIPv4Addr, u4NumIPv4, pucIPv6Addr,
			 u4NumIPv6);
}

#if CFG_MET_PACKET_TRACE_SUPPORT

BOOLEAN
kalMetCheckProfilingPacket(IN P_GLUE_INFO_T prGlueInfo,
    IN P_NATIVE_PACKET prPacket)
{
	UINT_32 u4PacketLen;
	UINT_16 u2EtherTypeLen;
	struct sk_buff *prSkb = (struct sk_buff *)prPacket;
	PUINT_8 aucLookAheadBuf = NULL;
	UINT_8 ucEthTypeLenOffset = ETHER_HEADER_LEN - ETHER_TYPE_LEN;
	PUINT_8 pucNextProtocol = NULL;

	u4PacketLen = prSkb->len;

	if (u4PacketLen < ETHER_HEADER_LEN) {
		DBGLOG(INIT, WARN, ("Invalid Ether packet length: %lu\n", u4PacketLen));
		return FALSE;
	}

	aucLookAheadBuf = prSkb->data;

	
	WLAN_GET_FIELD_BE16(&aucLookAheadBuf[ucEthTypeLenOffset], &u2EtherTypeLen);

	
	if (u2EtherTypeLen == ETH_P_VLAN) {
		ucEthTypeLenOffset += ETH_802_1Q_HEADER_LEN;
		WLAN_GET_FIELD_BE16(&aucLookAheadBuf[ucEthTypeLenOffset], &u2EtherTypeLen);
	}
	
	pucNextProtocol = &aucLookAheadBuf[ucEthTypeLenOffset + ETHER_TYPE_LEN];

	
	switch (u2EtherTypeLen) {
        
		
	case ETH_P_IPV4:        
		{
			PUINT_8 pucIpHdr = pucNextProtocol;
			UINT_8 ucIpVersion;

			
			if (u4PacketLen < (ucEthTypeLenOffset + ETHER_TYPE_LEN + IPV4_HDR_LEN)) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv4 packet length: %lu\n", u4PacketLen));
				return FALSE;
			}

			
			ucIpVersion = (pucIpHdr[0] & IP_VERSION_MASK) >> IP_VERSION_OFFSET;
			if (ucIpVersion != IP_VERSION_4) {
				DBGLOG(INIT, WARN,
				       ("Invalid IPv4 packet version: %u\n", ucIpVersion));
				return FALSE;
			}

            if(pucIpHdr[IPV4_HDR_IP_PROTOCOL_OFFSET] == IP_PROTOCOL_UDP) {
                PUINT_8 pucUdpHdr = &pucIpHdr[IPV4_HDR_LEN];
                UINT_16 u2UdpDstPort;
                UINT_16 u2UdpSrcPort;

                
                WLAN_GET_FIELD_BE16(&pucUdpHdr[UDP_HDR_DST_PORT_OFFSET], &u2UdpDstPort);

                
                WLAN_GET_FIELD_BE16(&pucUdpHdr[UDP_HDR_SRC_PORT_OFFSET], &u2UdpSrcPort);
                
                if(u2UdpSrcPort == prGlueInfo->u2MetUdpPort) {
                    UINT_16 u2IpId;

                    
                    WLAN_GET_FIELD_BE16(&pucIpHdr[IPV4_HDR_IP_IDENTIFICATION_OFFSET], &u2IpId);
#if 0
                    DBGLOG(INIT, INFO, ("TX PKT PROTOCOL[0x%x] UDP DST port[%u] IP_ID[%u]\n", 
                        pucIpHdr[IPV4_HDR_IP_PROTOCOL_OFFSET], u2UdpDstPort, u2IpId));
#endif
                    GLUE_SET_PKT_IP_ID(prPacket, u2IpId);

                    return TRUE;
                }
            }       
		}
		break;

	default:
		break;
	}

	return FALSE;
}				

static unsigned long __read_mostly tracing_mark_write_addr = 0;
static void inline __mt_update_tracing_mark_write_addr(void)
{
      if(unlikely(0 == tracing_mark_write_addr))
            tracing_mark_write_addr = kallsyms_lookup_name("tracing_mark_write");
}


VOID
kalMetTagPacket(IN P_GLUE_INFO_T prGlueInfo,
    IN P_NATIVE_PACKET prPacket, IN ENUM_TX_PROFILING_TAG_T eTag)
{
    if(!prGlueInfo->fgMetProfilingEn)
        return;

    switch(eTag) {
    case TX_PROF_TAG_OS_TO_DRV:
        if(kalMetCheckProfilingPacket(prGlueInfo, prPacket)) {     
            
            __mt_update_tracing_mark_write_addr();
#ifdef CONFIG_TRACING
            event_trace_printk(tracing_mark_write_addr, "S|%d|%s|%d\n", current->tgid, "WIFI-CHIP", GLUE_GET_PKT_IP_ID(prPacket));
#endif
            GLUE_SET_PKT_FLAG_PROF_MET(prPacket);
        }
        break;
        
    case TX_PROF_TAG_DRV_TX_DONE:
        if(GLUE_GET_PKT_IS_PROF_MET(prPacket)) {
            
            __mt_update_tracing_mark_write_addr();
#ifdef CONFIG_TRACING
            event_trace_printk(tracing_mark_write_addr, "F|%d|%s|%d\n", current->tgid, "WIFI-CHIP", GLUE_GET_PKT_IP_ID(prPacket));
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
kalMetInit(IN P_GLUE_INFO_T prGlueInfo)
{
    prGlueInfo->fgMetProfilingEn = FALSE;
    prGlueInfo->u2MetUdpPort = 0;
}
#if 0
static int
kalMetWriteProcfs (
    struct file *file,
    const char __user *buffer,
    size_t count,
    loff_t *off
    )  
{
    char acBuf[128 + 1]; 
    UINT_32 u4CopySize;
    int u16MetUdpPort;
    int u8MetProfEnable;
    IN P_GLUE_INFO_T prGlueInfo;
    ssize_t result;
    

    u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
    result = copy_from_user(acBuf, buffer, u4CopySize);
    acBuf[u4CopySize] = '\0';

    if (sscanf(acBuf, " %d %d", &u8MetProfEnable, &u16MetUdpPort) == 2) {
        printk("MET_PROF: Write MET PROC Enable=%d UDP_PORT=%d\n", u8MetProfEnable, u16MetUdpPort);
    }
    if (pMetGlobalData != NULL) {
        prGlueInfo = (P_GLUE_INFO_T) pMetGlobalData;
            prGlueInfo->fgMetProfilingEn = (BOOLEAN)u8MetProfEnable;
		    prGlueInfo->u2MetUdpPort = (UINT_16)u16MetUdpPort;
		}	
    return count;
}
#endif
static int
kalMetCtrlWriteProcfs (
    struct file *file,
    const char __user *buffer,
    size_t count,
    loff_t *off
    )  
{
    char acBuf[128 + 1]; 
    UINT_32 u4CopySize;
    int u8MetProfEnable;
    IN P_GLUE_INFO_T prGlueInfo;
    ssize_t result;
    

    u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
    result = copy_from_user(acBuf, buffer, u4CopySize);
    acBuf[u4CopySize] = '\0';

    if (sscanf(acBuf, " %d", &u8MetProfEnable) == 1) {
        printk("MET_PROF: Write MET PROC Enable=%d \n", u8MetProfEnable);
    }
    if (pMetGlobalData != NULL) {
        prGlueInfo = (P_GLUE_INFO_T) pMetGlobalData;
        prGlueInfo->fgMetProfilingEn = (UINT_8)u8MetProfEnable;
		}	
    return count;
}

static int
kalMetPortWriteProcfs (
    struct file *file,
    const char __user *buffer,
    size_t count,
    loff_t *off
    )  
{
    char acBuf[128 + 1]; 
    UINT_32 u4CopySize;
    int u16MetUdpPort;
    IN P_GLUE_INFO_T prGlueInfo;
    ssize_t result;
    

    u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
    result= copy_from_user(acBuf, buffer, u4CopySize);
    acBuf[u4CopySize] = '\0';

    if (sscanf(acBuf, " %d", &u16MetUdpPort) == 1) {
        printk("MET_PROF: Write MET PROC UDP_PORT=%d\n", u16MetUdpPort);
    }
    if (pMetGlobalData != NULL) {
        prGlueInfo = (P_GLUE_INFO_T) pMetGlobalData;
		    prGlueInfo->u2MetUdpPort = (UINT_16)u16MetUdpPort;
		}	
    return count;
}
#if 0
struct file_operations rMetProcFops = {
    write:   kalMetWriteProcfs
    	
};
#endif  
struct file_operations rMetProcCtrlFops = {
    write:   kalMetCtrlWriteProcfs
    	
};
struct file_operations rMetProcPortFops = {
    write:   kalMetPortWriteProcfs
    	
};
 
int kalMetInitProcfs(
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    
    if (init_net.proc_net == (struct proc_dir_entry *)NULL) {
        DBGLOG(INIT, INFO, ("init proc fs fail: proc_net == NULL\n"));
        return -ENOENT;
    }
    
    pMetProcDir = proc_mkdir("wlan0", init_net.proc_net);
    if (pMetProcDir == NULL) {
        return -ENOENT;
    }      
         
    
    proc_create(PROC_MET_PROF_CTRL, 0, pMetProcDir, &rMetProcCtrlFops);
    proc_create(PROC_MET_PROF_PORT, 0, pMetProcDir, &rMetProcPortFops);
    
    pMetGlobalData = (void *) prGlueInfo;
    
    return 0;  
}
int kalMetRemoveProcfs(void)
{ 

    if (init_net.proc_net == (struct proc_dir_entry *)NULL) {
        DBGLOG(INIT, WARN, ("remove proc fs fail: proc_net == NULL\n"));
        return -ENOENT;
    }	   	
    remove_proc_entry(PROC_MET_PROF_CTRL,      pMetProcDir);
    
    remove_proc_entry("wlan0", init_net.proc_net);
    
    pMetGlobalData = NULL;
    
    return 0;
}


#endif
#if CFG_SUPPORT_AGPS_ASSIST
BOOLEAN kalIndicateAgpsNotify(P_ADAPTER_T prAdapter, UINT_8 cmd, PUINT_8 data, UINT_16 dataLen){
	P_GLUE_INFO_T prGlueInfo = prAdapter->prGlueInfo;
	
	struct sk_buff *skb = cfg80211_testmode_alloc_event_skb(priv_to_wiphy(prGlueInfo),
																dataLen, GFP_KERNEL);
	
	if (unlikely(nla_put(skb, MTK_ATTR_AGPS_CMD, sizeof(cmd), &cmd) < 0))
		goto nla_put_failure;
	if (dataLen > 0 && data && unlikely(nla_put(skb, MTK_ATTR_AGPS_DATA, dataLen, data) < 0))
    	goto nla_put_failure;
	if (unlikely(nla_put(skb, MTK_ATTR_AGPS_IFINDEX, sizeof(UINT_32), &prGlueInfo->prDevHandler->ifindex) < 0))
    	goto nla_put_failure;
	
	if (unlikely(nla_put(skb, MTK_ATTR_AGPS_IFNAME, 5, prGlueInfo->prDevHandler->name) < 0))
    	goto nla_put_failure;
	cfg80211_testmode_event(skb, GFP_KERNEL);
	return TRUE;
	
nla_put_failure:
	kfree_skb(skb);
	return FALSE;
}
#endif

UINT_64 kalGetBootTime(void) 
{
	struct timespec ts;
	UINT_64 bootTime = 0;
	get_monotonic_boottime(&ts);
	bootTime = ts.tv_sec;
	bootTime *= USEC_PER_SEC;
	bootTime += ts.tv_nsec/NSEC_PER_USEC;
	return bootTime;
}
