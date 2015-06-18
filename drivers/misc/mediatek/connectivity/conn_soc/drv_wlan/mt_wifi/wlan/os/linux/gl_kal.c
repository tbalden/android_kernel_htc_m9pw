





#include "gl_os.h"
#include "gl_wext.h"
#include "precomp.h"
#if defined(CONFIG_MTK_TC1_FEATURE)
#include <tc1_partition.h>
#endif
#if CFG_SUPPORT_AGPS_ASSIST
#include <net/netlink.h>
#endif



#if DBG
int allocatedMemSize = 0;
#endif

extern struct semaphore g_halt_sem;
extern int g_u4HaltFlag;

static PVOID pvIoBuffer = NULL;

#ifdef MTK_DMA_BUF_MEMCPY_SUP
static PVOID pvIoPhyBuf = NULL;
static PVOID pvDmaBuffer = NULL;
static PVOID pvDmaPhyBuf = NULL;
#endif 

static UINT_32 pvIoBufferSize = 0;
static UINT_32 pvIoBufferUsage = 0;



#if defined(MT6620) && CFG_MULTI_ECOVER_SUPPORT
typedef enum _ENUM_WMTHWVER_TYPE_T{
    WMTHWVER_MT6620_E1 = 0x0,
    WMTHWVER_MT6620_E2 = 0x1,
    WMTHWVER_MT6620_E3 = 0x2,
    WMTHWVER_MT6620_E4 = 0x3,
    WMTHWVER_MT6620_E5 = 0x4,
    WMTHWVER_MT6620_E6 = 0x5,
    WMTHWVER_MT6620_MAX,
    WMTHWVER_INVALID = 0xff
} ENUM_WMTHWVER_TYPE_T, *P_ENUM_WMTHWVER_TYPE_T;

extern ENUM_WMTHWVER_TYPE_T
mtk_wcn_wmt_hwver_get(
    VOID
    );
#elif defined(MT5931) && CFG_MULTI_ECOVER_SUPPORT
typedef enum _ENUM_HWVER_TYPE_T{
    HWVER_MT5931_E1 = 0x1,
    HWVER_MT5931_E2 = 0x2,
    HWVER_MT5931_E3 = 0x3,
    HWVER_MT5931_MAX,
    HWVER_INVALID = 0xff
} ENUM_HWVER_TYPE_T, *P_ENUM_HWVER_TYPE_T;
#endif


VOID kalHifAhbKalWakeLockTimeout(
    IN P_GLUE_INFO_T                prGlueInfo
    )
{
    KAL_WAKE_LOCK_TIMEOUT(prGlueInfo->prAdapter,
                        &(prGlueInfo->rAhbIsrWakeLock),
                        (HZ/10)); 
}

#if CFG_ENABLE_FW_DOWNLOAD

static struct file *filp = NULL;
static uid_t orgfsuid;
static gid_t orgfsgid;
static mm_segment_t orgfs;

WLAN_STATUS
kalFirmwareOpen (
    IN P_GLUE_INFO_T                prGlueInfo
    )
{
	UINT_8 aucFilePath[50];


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
    orgfsuid = current->fsuid;
    orgfsgid = current->fsgid;
    current->fsuid = current->fsgid = 0;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
    struct cred *cred = (struct cred *) get_current_cred();
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


    
#if defined(MT6620) & CFG_MULTI_ECOVER_SUPPORT
    switch(mtk_wcn_wmt_hwver_get()) {
    case WMTHWVER_MT6620_E1:
    case WMTHWVER_MT6620_E2:
    case WMTHWVER_MT6620_E3:
    case WMTHWVER_MT6620_E4:
    case WMTHWVER_MT6620_E5:
        filp = filp_open("/etc/firmware/"CFG_FW_FILENAME, O_RDONLY, 0);
        break;

    case WMTHWVER_MT6620_E6:
    default:
        filp = filp_open("/etc/firmware/"CFG_FW_FILENAME"_E6", O_RDONLY, 0);
        break;
    }
#elif defined(MT5931) && CFG_MULTI_ECOVER_SUPPORT
    switch (wlanGetEcoVersion(prGlueInfo->prAdapter)) {
    case HWVER_MT5931_E1:
    case HWVER_MT5931_E2:
        filp = filp_open("/etc/firmware/"CFG_FW_FILENAME"_E2", O_RDONLY, 0);
        break;
    case HWVER_MT5931_E3:
    default:
        filp = filp_open("/etc/firmware/"CFG_FW_FILENAME, O_RDONLY, 0);
        break;
    }
#elif defined(MT6628)
	kalMemZero(aucFilePath, sizeof(aucFilePath));
	kalMemCopy(aucFilePath, "/etc/firmware/"CFG_FW_FILENAME"_",
		strlen("/etc/firmware/"CFG_FW_FILENAME"_"));
	glGetChipInfo(prGlueInfo, &aucFilePath[strlen("/etc/firmware/"CFG_FW_FILENAME"_")]);

	DBGLOG(INIT, TRACE, ("open file: %s\n", aucFilePath));

    filp = filp_open(aucFilePath, O_RDONLY, 0);
#else
    filp = filp_open("/etc/firmware/"CFG_FW_FILENAME, O_RDONLY, 0);
#endif
    if (IS_ERR(filp)) {
        DBGLOG(INIT, INFO, ("Open FW image: %s failed\n", CFG_FW_FILENAME));
        goto error_open;
    }
    DBGLOG(INIT, INFO, ("Open FW image: %s done\n", CFG_FW_FILENAME));
    return WLAN_STATUS_SUCCESS;

error_open:
    
    set_fs(orgfs);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
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


WLAN_STATUS
kalFirmwareClose (
    IN P_GLUE_INFO_T                prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    if ((filp != NULL) && !IS_ERR(filp)) {
        
        filp_close(filp, NULL);

        
        set_fs(orgfs);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29))
        current->fsuid = orgfsuid;
        current->fsgid = orgfsgid;
#else
        {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
            struct cred *cred = (struct cred *) get_current_cred();
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
kalFirmwareLoad (
    IN P_GLUE_INFO_T                prGlueInfo,
    OUT PVOID                       prBuf,
    IN UINT_32                      u4Offset,
    OUT PUINT_32                    pu4Size
    )
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


WLAN_STATUS
kalFirmwareSize (
    IN P_GLUE_INFO_T                prGlueInfo,
    OUT PUINT_32                     pu4Size
    )
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
kalFirmwareImageMapping (
    IN P_GLUE_INFO_T    prGlueInfo,
    OUT PPVOID          ppvMapFileBuf,
    OUT PUINT_32        pu4FileLength
    )
{
    DEBUGFUNC("kalFirmwareImageMapping");

    ASSERT(prGlueInfo);
    ASSERT(ppvMapFileBuf);
    ASSERT(pu4FileLength);

    do {
        
        if (kalFirmwareOpen(prGlueInfo) != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, TRACE, ("kalFirmwareOpen fail!\n"));
            break;
        } else {
            UINT_32 u4FwSize = 0;
            PVOID prFwBuffer = NULL;
            
            kalFirmwareSize(prGlueInfo, &u4FwSize);
            
            prFwBuffer = vmalloc(ALIGN_4(u4FwSize));
            
            if (kalFirmwareLoad(prGlueInfo, prFwBuffer, 0, &u4FwSize) != WLAN_STATUS_SUCCESS) {
                vfree(prFwBuffer);
                kalFirmwareClose(prGlueInfo);
				DBGLOG(INIT, TRACE, ("kalFirmwareLoad fail!\n"));
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
kalFirmwareImageUnmapping (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PVOID            prFwHandle,
    IN PVOID            pvMapFileBuf
    )
{
    DEBUGFUNC("kalFirmwareImageUnmapping");

    ASSERT(prGlueInfo);

    
    if(pvMapFileBuf) {
        vfree(pvMapFileBuf);
    }

    kalFirmwareClose(prGlueInfo);
}

#endif

#if 0


PVOID
kalFirmwareImageMapping (
    IN P_GLUE_INFO_T    prGlueInfo,
    OUT PPVOID          ppvMapFileBuf,
    OUT PUINT_32        pu4FileLength
    )
{
    INT_32 i4Ret = 0;

    DEBUGFUNC("kalFirmwareImageMapping");

    ASSERT(prGlueInfo);
    ASSERT(ppvMapFileBuf);
    ASSERT(pu4FileLength);

    do {
        GL_HIF_INFO_T *prHifInfo = &prGlueInfo->rHifInfo;
        prGlueInfo->prFw = NULL;

        
        i4Ret = request_firmware(&prGlueInfo->prFw, CFG_FW_FILENAME, prHifInfo->Dev);

        if (i4Ret) {
            printk (KERN_INFO DRV_NAME"fw %s:request failed %d\n", CFG_FW_FILENAME, i4Ret);
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
kalFirmwareImageUnmapping (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PVOID            prFwHandle,
    IN PVOID            pvMapFileBuf
    )
{
    DEBUGFUNC("kalFirmwareImageUnmapping");

    ASSERT(prGlueInfo);
    ASSERT(pvMapFileBuf);

    release_firmware(prGlueInfo->prFw);

}
#endif

extern UINT_32 u4MemAllocCnt, u4MemFreeCnt;

VOID
kalAcquireSpinLock (
    IN P_GLUE_INFO_T                prGlueInfo,
    IN ENUM_SPIN_LOCK_CATEGORY_E    rLockCategory,
    OUT PUINT_32                    pu4Flags
    )
{
    UINT_32 u4Flags = 0;

    ASSERT(prGlueInfo);
    ASSERT(pu4Flags);

    if (rLockCategory < SPIN_LOCK_NUM) {

#if CFG_USE_SPIN_LOCK_BOTTOM_HALF
        spin_lock_bh(&prGlueInfo->rSpinLock[rLockCategory]);
#else 
        spin_lock_irqsave(&prGlueInfo->rSpinLock[rLockCategory], u4Flags);
#endif 

        *pu4Flags = u4Flags;
    }

    return;
} 


VOID
kalReleaseSpinLock (
    IN P_GLUE_INFO_T                prGlueInfo,
    IN ENUM_SPIN_LOCK_CATEGORY_E    rLockCategory,
    IN UINT_32                      u4Flags
    )
{
    ASSERT(prGlueInfo);

    if (rLockCategory < SPIN_LOCK_NUM) {


#if CFG_USE_SPIN_LOCK_BOTTOM_HALF
        spin_unlock_bh(&prGlueInfo->rSpinLock[rLockCategory]);
#else 
        spin_unlock_irqrestore(&prGlueInfo->rSpinLock[rLockCategory], u4Flags);
#endif 

    }

    return;
} 


VOID
kalUpdateMACAddress (
    IN P_GLUE_INFO_T prGlueInfo,
    IN PUINT_8 pucMacAddr
    )
{
    ASSERT(prGlueInfo);
    ASSERT(pucMacAddr);

    if(UNEQUAL_MAC_ADDR(prGlueInfo->prDevHandler->dev_addr, pucMacAddr)) {
        memcpy(prGlueInfo->prDevHandler->dev_addr, pucMacAddr, PARAM_MAC_ADDR_LEN);
    }

    return;
}


#if CFG_TCP_IP_CHKSUM_OFFLOAD
VOID
kalQueryTxChksumOffloadParam (
    IN PVOID pvPacket,
    OUT PUINT_8 pucFlag
    )
{
    struct sk_buff *skb = (struct sk_buff *) pvPacket;
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
            
        }
        else
#endif
        ucFlag |= (TX_CS_IP_GEN | TX_CS_TCP_UDP_GEN);
    }

    *pucFlag = ucFlag;

    return;
} 


//4 2007/10/8, mikewu, this is rewritten by Mike
VOID
kalUpdateRxCSUMOffloadParam (
    IN PVOID pvPacket,
    IN ENUM_CSUM_RESULT_T aeCSUM[]
    )
{
    struct sk_buff *skb = (struct sk_buff *)pvPacket;

    ASSERT(pvPacket);

    if ( (aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_SUCCESS || aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_SUCCESS)&&
        ( (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_SUCCESS) || (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_SUCCESS)) ) {
        skb->ip_summed = CHECKSUM_UNNECESSARY;
    }
    else {
        skb->ip_summed = CHECKSUM_NONE;
#if DBG
        if (aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_NONE && aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_NONE) {
            DBGLOG(RX, TRACE, ("RX: \"non-IPv4/IPv6\" Packet\n"));
        }
        else if (aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_FAILED) {
            DBGLOG(RX, TRACE, ("RX: \"bad IP Checksum\" Packet\n"));
        }
        else if (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_FAILED) {
            DBGLOG(RX, TRACE, ("RX: \"bad TCP Checksum\" Packet\n"));
        }
        else if (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_FAILED) {
            DBGLOG(RX, TRACE, ("RX: \"bad UDP Checksum\" Packet\n"));
        }
        else {

        }
#endif
    }

} 
#endif 


VOID
kalPacketFree(
    IN P_GLUE_INFO_T   prGlueInfo,
    IN PVOID           pvPacket
    )
{
    dev_kfree_skb((struct sk_buff *) pvPacket);
}


PVOID
kalPacketAlloc (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN UINT_32          u4Size,
    OUT PUINT_8         *ppucData
    )
{
    struct sk_buff  *prSkb = dev_alloc_skb(u4Size);

    if (prSkb) {
        *ppucData = (PUINT_8) (prSkb->data);
    }
#if DBG
{
    PUINT_32 pu4Head = (PUINT_32)&prSkb->cb[0];
    *pu4Head = (UINT_32)prSkb->head;
    DBGLOG(RX, TRACE, ("prSkb->head = %#x, prSkb->cb = %#x\n", (UINT_32)prSkb->head, *pu4Head));
}
#endif
    return (PVOID) prSkb;
}

WLAN_STATUS
kalProcessRxPacket (
    IN P_GLUE_INFO_T      prGlueInfo,
    IN PVOID              pvPacket,
    IN PUINT_8            pucPacketStart,
    IN UINT_32            u4PacketLen,
    
    IN BOOLEAN            fgIsRetain,
    IN ENUM_CSUM_RESULT_T aerCSUM[]
    )
{
    WLAN_STATUS rStatus = WLAN_STATUS_SUCCESS;
    struct sk_buff *skb = (struct sk_buff *)pvPacket;


    skb->data = pucPacketStart;
    skb_reset_tail_pointer(skb); 
	skb_trim(skb, 0); 
	skb_put(skb, u4PacketLen); 

#if CFG_TCP_IP_CHKSUM_OFFLOAD
    kalUpdateRxCSUMOffloadParam(skb, aerCSUM);
#endif

    return rStatus;
}


#if (CONF_HIF_LOOPBACK_AUTO == 1)
extern int
wlanHardStartXmit(
    struct sk_buff *prSkb,
    struct net_device *prDev
    );

unsigned int testmode = 0;
unsigned int testlen = 64;

void
kalDevLoopbkAuto(
    IN GLUE_INFO_T      *GlueInfo
    )
{
#define HIF_LOOPBK_AUTO_TEST_LEN    1600
    static unsigned int txcnt;
    struct sk_buff *MsduInfo;
    UINT_8 *Pkt;
    UINT_32 RegVal;
    UINT_32 PktLen = 16;


    
    if (testmode != 0)
    {
        PktLen = kalRandomNumber() % 1520;
        if (PktLen < 64)
            PktLen = 64;
    }
    else
    {
        PktLen = testlen ++;
        if (PktLen > 1520)
        {
            testmode = 1;
            PktLen = 64;
        }
    }

    printk("kalDevLoopbkAuto> Send a packet to HIF (len = %d) (total = %d)...\n",
            PktLen, ++txcnt);

    
    MsduInfo = kalPacketAlloc(GlueInfo, HIF_LOOPBK_AUTO_TEST_LEN, &Pkt);
    if (MsduInfo == NULL) {
        printk("No PKT_INFO_T for sending loopback packet!\n");
        return;
    }

    
    MsduInfo->dev = GlueInfo->prDevHandler;
    if (MsduInfo->dev == NULL)
    {
        printk("MsduInfo->dev == NULL!!\n");
        kalPacketFree(GlueInfo, MsduInfo);
        return;
    }

    MsduInfo->len = PktLen;
    kalMemSet(MsduInfo->data, 0xff, 6);
    kalMemSet(MsduInfo->data+6, 0x5a, PktLen-6);

    
    wlanHardStartXmit(MsduInfo, MsduInfo->dev);

#if 0
    PktLen += 4;
    if (PktLen >= 1600)
        PktLen = 16;
#endif

    
}


int
kalDevLoopbkThread(
    IN void *data
    )
{
    struct net_device *dev = data;
    P_GLUE_INFO_T GlueInfo = *((P_GLUE_INFO_T *) netdev_priv(dev));
    GL_HIF_INFO_T *HifInfo = &GlueInfo->rHifInfo;
    int ret;
    static int test = 0;



    while (TRUE) {
        ret = wait_event_interruptible(HifInfo->HifWaitq,
                (HifInfo->HifLoopbkFlg != 0));

        if (HifInfo->HifLoopbkFlg == 0xFFFFFFFF)
            break;

        while (TRUE)
        {

#if 0 
{
    static int test = 0;
    if (++test == 5)
    {
        *(volatile unsigned int *)0xF0000054 |= 4;
        printk("disable GDMA clock...\n");
        testgdmaclock = 1;
    }
}
#endif

            if (GlueInfo->i4TxPendingFrameNum < 64)
            {
                printk("GlueInfo->i4TxPendingFrameNum = %d\n", GlueInfo->i4TxPendingFrameNum);
                kalDevLoopbkAuto(GlueInfo);

                if (testmode == 0)
                    kalMsleep(3000);
            }
            else
                kalMsleep(1);
        }
    }
}


void
kalDevLoopbkRxHandle(
    IN P_ADAPTER_T    prAdapter,
    IN OUT P_SW_RFB_T prSwRfb
    )
{
    static unsigned int rxcnt;
    UINT_32 i;
    UINT_8 *Buf = prSwRfb->pucRecvBuff + sizeof(HIF_TX_HEADER_T);
    P_HIF_RX_HEADER_T prHifRxHdr = prSwRfb->prHifRxHdr;
    UINT_32 len = prHifRxHdr->u2PacketLen - sizeof(HIF_TX_HEADER_T);


    if (len > 1600)
    {
        while(1)
        printk("HIF> Loopback len > 1600!!! error!!!\n");
    }

    for(i=0; i<6; i++)
    {
        if (Buf[i] != 0xff)
        {
			while(1)
			{
	            printk("HIF> Loopbk dst addr error (len = %d)!\n", len);
	            dumpMemory8(prSwRfb->pucRecvBuff, prHifRxHdr->u2PacketLen);
			}
        }
    }

    for(i=6; i<len; i++)
    {
        if (Buf[i] != 0x5a)
        {
			while(1)
			{
	            printk("HIF> Loopbk error (len = %d)!\n", len);
	            dumpMemory8(prSwRfb->pucRecvBuff, prHifRxHdr->u2PacketLen);
			}
        }
    }

    printk("HIF> Loopbk OK (len = %d) (total = %d)!\n",
            len, ++rxcnt);
}
#endif 


WLAN_STATUS
kalRxIndicatePkts (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PVOID            apvPkts[],
    IN UINT_8           ucPktNum
    )
{
    UINT_8 ucIdx = 0;
    struct net_device *prNetDev = prGlueInfo->prDevHandler;
    struct sk_buff *prSkb = NULL;

    ASSERT(prGlueInfo);
    ASSERT(apvPkts);

#if CFG_BOW_TEST
    UINT_32 i;
#endif

    for(ucIdx = 0; ucIdx < ucPktNum; ucIdx++) {
        prSkb = apvPkts[ucIdx];
#if DBG
        do {
            PUINT_8 pu4Head = (PUINT_8)&prSkb->cb[0];
            UINT_32 u4HeadValue = 0;
            kalMemCopy(&u4HeadValue, pu4Head, sizeof(u4HeadValue));
            DBGLOG(RX, TRACE, ("prSkb->head = %p, prSkb->cb = 0x%x\n", pu4Head, u4HeadValue));
        } while (0);
#endif

        if (GLUE_GET_PKT_IS_P2P(prSkb)) {
            
#if CFG_ENABLE_WIFI_DIRECT
            if(prGlueInfo->prAdapter->fgIsP2PRegistered) {
                prNetDev = kalP2PGetDevHdlr(prGlueInfo);
            }

            
            
            prGlueInfo->prP2PInfo->rNetDevStats.rx_bytes += prSkb->len;
	        prGlueInfo->prP2PInfo->rNetDevStats.rx_packets++;
            
#else
            prNetDev = prGlueInfo->prDevHandler;
#endif
        }
        else if (GLUE_GET_PKT_IS_PAL(prSkb)) {
            
#if CFG_ENABLE_BT_OVER_WIFI && CFG_BOW_SEPARATE_DATA_PATH
            if (prGlueInfo->rBowInfo.fgIsNetRegistered) {
            prNetDev = prGlueInfo->rBowInfo.prDevHandler;
            }
#else
            prNetDev = prGlueInfo->prDevHandler;
#endif
        }
        else {
            
            prNetDev = prGlueInfo->prDevHandler;
            prGlueInfo->rNetDevStats.rx_bytes += prSkb->len;
            prGlueInfo->rNetDevStats.rx_packets++;

        }

		
		if (kalMemCmp(prSkb->data, prSkb->data+6, 6) == 0)
		{
			
			
			DBGLOG(RX, EVENT,
				("kalRxIndicatePkts got from us!!! Drop it! (["MACSTR"] len %d)\n",
				MAC2STR(prSkb->data), prSkb->len));
			wlanReturnPacket(prGlueInfo->prAdapter, prSkb);
			continue;
		}

#if (CFG_SUPPORT_TDLS == 1)
		if (TdlsexRxFrameDrop(prGlueInfo, prSkb->data) == TRUE)
		{
			
			DBGLOG(TDLS, WARN,
				("<tdls_fme> %s: drop a received packet from "MACSTR" %u\n",
				__FUNCTION__, MAC2STR(prSkb->data),
				(UINT32)((P_ADAPTER_T)(prGlueInfo->prAdapter))->rRxCtrl.rFreeSwRfbList.u4NumElem));
			wlanReturnPacket(prGlueInfo->prAdapter, prSkb);
			continue;
		}

		TdlsexRxFrameHandle(prGlueInfo, prSkb->data, prSkb->len);
#endif 

        prNetDev->last_rx = jiffies;
        prSkb->protocol = eth_type_trans(prSkb, prNetDev);
        prSkb->dev = prNetDev;
        
        DBGLOG(RX, EVENT, ("kalRxIndicatePkts len = %d\n", prSkb->len));

#if CFG_BOW_TEST
    DBGLOG(BOW, TRACE, ("Rx sk_buff->len: %d\n", prSkb->len));
    DBGLOG(BOW, TRACE, ("Rx sk_buff->data_len: %d\n", prSkb->data_len));
    DBGLOG(BOW, TRACE, ("Rx sk_buff->data:\n"));

        for(i = 0; i < prSkb->len; i++)
        {
        DBGLOG(BOW, TRACE, ("%4x", prSkb->data[i]));

            if((i+1)%16 ==0)
            {
            DBGLOG(BOW, TRACE, ("\n"));
            }
        }

    DBGLOG(BOW, TRACE, ("\n"));
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
        if(!in_interrupt()){
            netif_rx_ni(prSkb); 
        }
        else {
            netif_rx(prSkb);
        }
#else
        netif_rx(prSkb);
#endif

        wlanReturnPacket(prGlueInfo->prAdapter, NULL);
    }

    return WLAN_STATUS_SUCCESS;
}

UINT_32 ScanCnt = 0, ScanDoneFailCnt = 0;
VOID
kalIndicateStatusAndComplete (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN WLAN_STATUS      eStatus,
    IN PVOID            pvBuf,
    IN UINT_32          u4BufLen
    )
{
    UINT_32 bufLen;
    P_PARAM_STATUS_INDICATION_T pStatus = (P_PARAM_STATUS_INDICATION_T) pvBuf;
    P_PARAM_AUTH_EVENT_T pAuth = (P_PARAM_AUTH_EVENT_T) pStatus;
    P_PARAM_PMKID_CANDIDATE_LIST_T pPmkid =
        (P_PARAM_PMKID_CANDIDATE_LIST_T) (pStatus + 1);
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
            wlanoidQueryBssid,
            &arBssid[0],
            sizeof(arBssid),
            &bufLen);
        wext_indicate_wext_event(prGlueInfo, SIOCGIWAP, arBssid, bufLen);

        
        netif_carrier_on(prGlueInfo->prDevHandler);

        do {
            
            wlanQueryInformation(prGlueInfo->prAdapter,
                wlanoidQuerySsid,
                &ssid,
                sizeof(ssid),
                &bufLen);

            ssid.aucSsid[(ssid.u4SsidLen >= PARAM_MAX_LEN_SSID) ?
                (PARAM_MAX_LEN_SSID - 1) : ssid.u4SsidLen ] = '\0';
            DBGLOG(INIT, INFO, ("[wifi] %s netif_carrier_on [ssid:%s " MACSTR "]\n",
                prGlueInfo->prDevHandler->name,
                ssid.aucSsid,
                MAC2STR(arBssid)));
        } while(0);

        if(prGlueInfo->fgIsRegistered == TRUE) {
            
            ucChannelNum = wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter, NETWORK_TYPE_AIS_INDEX);
            if(ucChannelNum <= 14) {
                prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_2GHZ));
            }
            else {
                prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_5GHZ));
            }

            
            bss = cfg80211_get_bss(priv_to_wiphy(prGlueInfo), prChannel, arBssid,
                    ssid.aucSsid, ssid.u4SsidLen, 
                    WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);

            if(bss == NULL) {
                
                prBssDesc = wlanGetTargetBssDescByNetwork(prGlueInfo->prAdapter, NETWORK_TYPE_AIS_INDEX);

                if(prBssDesc != NULL) {
                    bss = cfg80211_inform_bss(priv_to_wiphy(prGlueInfo),
                        prChannel,
                        arBssid,
                        0,                                      
                        WLAN_CAPABILITY_ESS,
                        prBssDesc->u2BeaconInterval,            
                        prBssDesc->aucIEBuf,                    
                        prBssDesc->u2IELength,                  
                        RCPI_TO_dBm(prBssDesc->ucRCPI) * 100,   
                        GFP_KERNEL);
                }
            }

            
            if(eStatus == WLAN_STATUS_MEDIA_CONNECT 
                    && prGlueInfo->prDevHandler->ieee80211_ptr->sme_state == CFG80211_SME_CONNECTING) {
                cfg80211_connect_result(prGlueInfo->prDevHandler,
                        arBssid,
                        prGlueInfo->aucReqIe,
                        prGlueInfo->u4ReqIeLength,
                        prGlueInfo->aucRspIe,
                        prGlueInfo->u4RspIeLength,
                        WLAN_STATUS_SUCCESS,
                        GFP_KERNEL);
            }
            else if(eStatus == WLAN_STATUS_ROAM_OUT_FIND_BEST
                    && prGlueInfo->prDevHandler->ieee80211_ptr->sme_state == CFG80211_SME_CONNECTED) {
                cfg80211_roamed_bss(prGlueInfo->prDevHandler, 
                        bss,
                        prGlueInfo->aucReqIe,
                        prGlueInfo->u4ReqIeLength,
                        prGlueInfo->aucRspIe,
                        prGlueInfo->u4RspIeLength,
                        GFP_KERNEL);
            }
        }

        break;

    case WLAN_STATUS_MEDIA_DISCONNECT:
        
        wext_indicate_wext_event(prGlueInfo, SIOCGIWAP, NULL, 0);
        

#if 1   
         DBGLOG(INIT, INFO, ("[wifi] %s netif_carrier_off %d\n",
				prGlueInfo->prDevHandler->name,
				prGlueInfo->prDevHandler->ieee80211_ptr->sme_state));
#endif

        netif_carrier_off(prGlueInfo->prDevHandler);

        if(prGlueInfo->fgIsRegistered == TRUE
                && prGlueInfo->prDevHandler->ieee80211_ptr->sme_state == CFG80211_SME_CONNECTED) {
            P_WIFI_VAR_T prWifiVar = &prGlueInfo->prAdapter->rWifiVar;
			UINT_16 u2DeauthReason = prWifiVar->arBssInfo[NETWORK_TYPE_AIS_INDEX].u2DeauthReason;
            
			DBGLOG(INIT, INFO, ("[wifi] %s cfg80211_disconnected\n", prGlueInfo->prDevHandler->name));
            cfg80211_disconnected(prGlueInfo->prDevHandler, u2DeauthReason, NULL, 0, GFP_KERNEL);
        }

        prGlueInfo->eParamMediaStateIndicated = PARAM_MEDIA_STATE_DISCONNECTED;

        break;

    case WLAN_STATUS_SCAN_COMPLETE:
        
        wext_indicate_wext_event(prGlueInfo, SIOCGIWSCAN, NULL, 0);

        
        GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
        if(prGlueInfo->prScanRequest != NULL) {
            prScanRequest = prGlueInfo->prScanRequest;
            prGlueInfo->prScanRequest = NULL;
        }
        GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

        
		DBGLOG(INIT, TRACE, ("[ais] scan complete %p %d %d\n",
				prScanRequest, ScanCnt, ScanDoneFailCnt));

        if(prScanRequest != NULL) {
            cfg80211_scan_done(prScanRequest, FALSE);
        }
        break;
        case WLAN_STATUS_CONNECT_INDICATION:
            
            if(prGlueInfo->prDevHandler->ieee80211_ptr->sme_state == CFG80211_SME_CONNECTING) {
            cfg80211_connect_result(prGlueInfo->prDevHandler,
                    prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo.prTargetBssDesc->aucBSSID,
                    prGlueInfo->aucReqIe,
                    prGlueInfo->u4ReqIeLength,
                    prGlueInfo->aucRspIe,
                    prGlueInfo->u4RspIeLength,
                    REASON_CODE_UNSPECIFIED,
                    GFP_KERNEL);
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
                                       (pAuth->arRequest[0].u4Flags == PARAM_AUTH_REQUEST_PAIRWISE_ERROR) ? NL80211_KEYTYPE_PAIRWISE : NL80211_KEYTYPE_GROUP,
                                       0, NULL, GFP_KERNEL);
                    wext_indicate_wext_event(prGlueInfo,
                        IWEVMICHAELMICFAILURE,
                        (unsigned char *)&pAuth->arRequest[0],
                        pAuth->arRequest[0].u4Length);
                }
                break;

            case ENUM_STATUS_TYPE_CANDIDATE_LIST:
                {
                 UINT_32  i = 0;

                 P_PARAM_PMKID_CANDIDATE_T prPmkidCand = (P_PARAM_PMKID_CANDIDATE_T)&pPmkid->arCandidateList[0];

                 for (i=0; i<pPmkid->u4NumCandidates; i++) {
                    wext_indicate_wext_event(prGlueInfo,
                        IWEVPMKIDCAND,
                        (unsigned char *)&pPmkid->arCandidateList[i],
                        pPmkid->u4NumCandidates);
                    prPmkidCand += sizeof(PARAM_PMKID_CANDIDATE_T);
                }
                }
                break;

            default:
                
                break;
            }
        }
        else {
        }
        break;

#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS
    case WLAN_STATUS_BWCS_UPDATE:
    {
        wext_indicate_wext_event(prGlueInfo, IWEVCUSTOM, pvBuf, sizeof(PTA_IPC_T));
    }

        break;

#endif

    default:
        break;
    }
} 


VOID
kalUpdateReAssocReqInfo (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PUINT_8          pucFrameBody,
    IN UINT_32          u4FrameBodyLen,
    IN BOOLEAN          fgReassocRequest
    )
{
    PUINT_8 cp;

    ASSERT(prGlueInfo);

    
    prGlueInfo->u4ReqIeLength = 0;

    if (fgReassocRequest) {
        if (u4FrameBodyLen < 15) {
            return;
        }
    }
    else {
        if (u4FrameBodyLen < 9) {
            return;
        }
    }

    cp = pucFrameBody;

    if (fgReassocRequest) {
        
        
        
        cp += 10;
        u4FrameBodyLen -= 10;
    }
    else {
        
        
        cp += 4;
        u4FrameBodyLen -= 4;
    }

    wext_indicate_wext_event(prGlueInfo, IWEVASSOCREQIE, cp, u4FrameBodyLen);

    if(u4FrameBodyLen <= CFG_CFG80211_IE_BUF_LEN) {
        prGlueInfo->u4ReqIeLength = u4FrameBodyLen;
        kalMemCopy(prGlueInfo->aucReqIe, cp, u4FrameBodyLen);
    }

    return;
}


VOID
kalUpdateReAssocRspInfo (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PUINT_8          pucFrameBody,
    IN UINT_32          u4FrameBodyLen
    )
{
    UINT_32 u4IEOffset = 6; 
    UINT_32 u4IELength = u4FrameBodyLen - u4IEOffset;

    ASSERT(prGlueInfo);

    
    prGlueInfo->u4RspIeLength = 0;

    if(u4IELength <= CFG_CFG80211_IE_BUF_LEN) {
        prGlueInfo->u4RspIeLength = u4IELength;
        kalMemCopy(prGlueInfo->aucRspIe, pucFrameBody + u4IEOffset, u4IELength);
    }

}   


VOID
kalSendCompleteAndAwakeQueue (
    IN P_GLUE_INFO_T prGlueInfo,
    IN PVOID pvPacket
    )
{

    struct net_device   *prDev = NULL;
    struct sk_buff      *prSkb = NULL;
    UINT_16             u2QueueIdx = 0;
    UINT_8              ucNetworkType = 0;
    BOOLEAN             fgIsValidDevice = TRUE;


    ASSERT(pvPacket);
    ASSERT(prGlueInfo->i4TxPendingFrameNum);

    prSkb = (struct sk_buff *) pvPacket;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
    u2QueueIdx = skb_get_queue_mapping(prSkb);
#endif
    ASSERT(u2QueueIdx < CFG_MAX_TXQ_NUM);

    if (GLUE_GET_PKT_IS_PAL(prSkb)) {
        ucNetworkType = NETWORK_TYPE_BOW_INDEX;
    } else if (GLUE_GET_PKT_IS_P2P(prSkb)) {
        ucNetworkType = NETWORK_TYPE_P2P_INDEX;

#if CFG_ENABLE_WIFI_DIRECT
        
        if(prGlueInfo->prAdapter->fgIsP2PRegistered == FALSE) {
            fgIsValidDevice = FALSE;
        }
#endif
    } else {
        ucNetworkType = NETWORK_TYPE_AIS_INDEX;
    }

    GLUE_DEC_REF_CNT(prGlueInfo->i4TxPendingFrameNum);
    GLUE_DEC_REF_CNT(prGlueInfo->ai4TxPendingFrameNumPerQueue[ucNetworkType][u2QueueIdx]);
    prDev = prSkb->dev;

    ASSERT(prDev);

    if(fgIsValidDevice == TRUE) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
        if (netif_subqueue_stopped(prDev, prSkb) &&
                prGlueInfo->ai4TxPendingFrameNumPerQueue[ucNetworkType][u2QueueIdx] <= CFG_TX_START_NETIF_PER_QUEUE_THRESHOLD) {
            netif_wake_subqueue(prDev, u2QueueIdx);

#if (CONF_HIF_LOOPBACK_AUTO == 1)
            prGlueInfo->rHifInfo.HifLoopbkFlg &= ~0x01;
#endif 
        }
#else
        if (prGlueInfo->i4TxPendingFrameNum < CFG_TX_STOP_NETIF_QUEUE_THRESHOLD) {
            netif_wake_queue(prGlueInfo->prDevHandler);

#if (CONF_HIF_LOOPBACK_AUTO == 1)
            prGlueInfo->rHifInfo.HifLoopbkFlg &= ~0x01;
#endif 
        }
#endif
    }


    dev_kfree_skb((struct sk_buff *) pvPacket);

    DBGLOG(TX, EVENT, ("----- pending frame %d -----\n",
		prGlueInfo->i4TxPendingFrameNum));

    return;
}


VOID
kalQueryRegistryMacAddr (
    IN  P_GLUE_INFO_T   prGlueInfo,
    OUT PUINT_8        paucMacAddr
    )
{
    UINT_8 aucZeroMac[MAC_ADDR_LEN] = {0,0,0,0,0,0}
    DEBUGFUNC("kalQueryRegistryMacAddr");

    ASSERT(prGlueInfo);
    ASSERT(paucMacAddr);

    kalMemCopy((PVOID) paucMacAddr, (PVOID)aucZeroMac, MAC_ADDR_LEN);

    return;
} 

#if CFG_SUPPORT_EXT_CONFIG
UINT_32
kalReadExtCfg (
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    if (0 != prGlueInfo->u4ExtCfgLength) {
        DBGLOG(INIT, TRACE, ("Read external configuration data -- OK\n"));
    }
    else {
        DBGLOG(INIT, TRACE, ("Read external configuration data -- fail\n"));
    }

    return prGlueInfo->u4ExtCfgLength;
}
#endif


BOOLEAN
kalQoSFrameClassifierAndPacketInfo (
    IN P_GLUE_INFO_T prGlueInfo,
    IN P_NATIVE_PACKET prPacket,
    OUT PUINT_8 pucPriorityParam,
    OUT PUINT_32 pu4PacketLen,
    OUT PUINT_8 pucEthDestAddr,
    OUT PBOOLEAN pfgIs1X,
    OUT PBOOLEAN pfgIsPAL,
    OUT PUINT_8 pucNetworkType
    )
{

    UINT_32 u4PacketLen;


    UINT_8 ucUserPriority = USER_PRIORITY_DEFAULT; 
    UINT_16 u2EtherTypeLen;
    struct sk_buff *prSkb = (struct sk_buff *) prPacket;
    PUINT_8 aucLookAheadBuf = NULL;

    DEBUGFUNC("kalQoSFrameClassifierAndPacketInfo");

    u4PacketLen = prSkb->len;

    if (u4PacketLen < ETH_HLEN) {
        DBGLOG(INIT, WARN, ("Invalid Ether packet length: %u\n", (UINT_32)u4PacketLen));
        return FALSE;
    }

    aucLookAheadBuf = prSkb->data;

    *pfgIs1X = FALSE;
    *pfgIsPAL = FALSE;

    
    u2EtherTypeLen = (aucLookAheadBuf[ETH_TYPE_LEN_OFFSET] << 8) | (aucLookAheadBuf[ETH_TYPE_LEN_OFFSET + 1]);

    if ((u2EtherTypeLen == ETH_P_IP) &&
        (u4PacketLen >= LOOK_AHEAD_LEN)) {
        PUINT_8 pucIpHdr = &aucLookAheadBuf[ETH_HLEN];
        UINT_8 ucIpVersion;

        ucIpVersion = (pucIpHdr[0] & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
        
        if (ucIpVersion == IPVERSION) {
            UINT_8 ucIpTos;
            
            ucIpTos = pucIpHdr[1];
            ucUserPriority = ((ucIpTos & IPTOS_PREC_MASK) >> IPTOS_PREC_OFFSET);
        }

        
    }
    else if (u2EtherTypeLen == ETH_P_1X) { 
        
        *pfgIs1X = TRUE;
    }
    else if (u2EtherTypeLen == ETH_P_PRE_1X) { 
        
        *pfgIs1X = TRUE;
    }
#if CFG_SUPPORT_WAPI
    else if (u2EtherTypeLen == ETH_WPI_1X) {
        *pfgIs1X = TRUE;
    }
#endif
#if (CFG_SUPPORT_TDLS == 1)
	else if (u2EtherTypeLen == TDLS_FRM_PROT_TYPE) {
		TDLSEX_UP_ASSIGN(ucUserPriority);
	}
#endif 
    else if (u2EtherTypeLen <= 1500) { 
        UINT_8 ucDSAP, ucSSAP, ucControl;
        UINT_8 aucOUI[3];

        ucDSAP = *(PUINT_8) &aucLookAheadBuf[ETH_LLC_OFFSET];
        ucSSAP = *(PUINT_8) &aucLookAheadBuf[ETH_LLC_OFFSET + 1];
        ucControl = *(PUINT_8) &aucLookAheadBuf[ETH_LLC_OFFSET + 2];

        aucOUI[0] = *(PUINT_8) &aucLookAheadBuf[ETH_SNAP_OFFSET];
        aucOUI[1] = *(PUINT_8) &aucLookAheadBuf[ETH_SNAP_OFFSET + 1];
        aucOUI[2] = *(PUINT_8) &aucLookAheadBuf[ETH_SNAP_OFFSET + 2];

        if(ucDSAP == ETH_LLC_DSAP_SNAP &&
                ucSSAP == ETH_LLC_SSAP_SNAP &&
                ucControl == ETH_LLC_CONTROL_UNNUMBERED_INFORMATION &&
                aucOUI[0] == ETH_SNAP_BT_SIG_OUI_0 &&
                aucOUI[1] == ETH_SNAP_BT_SIG_OUI_1 &&
                aucOUI[2] == ETH_SNAP_BT_SIG_OUI_2) {

            UINT_16 tmp = ((aucLookAheadBuf[ETH_SNAP_OFFSET + 3] << 8) | aucLookAheadBuf[ETH_SNAP_OFFSET + 4]);

            *pfgIsPAL = TRUE;
            ucUserPriority = (UINT_8)prSkb->priority;

            if (tmp == BOW_PROTOCOL_ID_SECURITY_FRAME) {
                *pfgIs1X = TRUE;
            }
        }
    }

    
    *pucPriorityParam = ucUserPriority;

    
    
    *pu4PacketLen = u4PacketLen;

    kalMemCopy(pucEthDestAddr, aucLookAheadBuf, PARAM_MAC_ADDR_LEN);


    
#if CFG_ENABLE_BT_OVER_WIFI
    if(*pfgIsPAL == TRUE) {
        *pucNetworkType = NETWORK_TYPE_BOW_INDEX;
    }
    else
#endif
    {
#if CFG_ENABLE_WIFI_DIRECT
        if(prGlueInfo->prAdapter->fgIsP2PRegistered && GLUE_GET_PKT_IS_P2P(prPacket)) {
            *pucNetworkType = NETWORK_TYPE_P2P_INDEX;
        }
        else
#endif
        {
            *pucNetworkType = NETWORK_TYPE_AIS_INDEX;
        }
    }
    return TRUE;
} 

VOID
kalOidComplete (
    IN P_GLUE_INFO_T prGlueInfo,
    IN BOOLEAN fgSetQuery,
    IN UINT_32 u4SetQueryInfoLen,
    IN WLAN_STATUS rOidStatus
    )
{

    ASSERT(prGlueInfo);
    
    wlanoidClearTimeoutCheck(prGlueInfo->prAdapter);

    
    prGlueInfo->rPendStatus = rOidStatus;
    complete(&prGlueInfo->rPendComp);
    prGlueInfo->u4OidCompleteFlag = 1;
    
    
}

VOID
kalOidClearance(
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    
     
    if (prGlueInfo->u4OidCompleteFlag != 1) {
        complete(&prGlueInfo->rPendComp);
    }
    
}





WLAN_STATUS
kalIoctl (
    IN P_GLUE_INFO_T prGlueInfo,
    IN PFN_OID_HANDLER_FUNC pfnOidHandler,
    IN PVOID pvInfoBuf,
    IN UINT_32 u4InfoBufLen,
    IN BOOLEAN fgRead,
    IN BOOLEAN fgWaitResp,
    IN BOOLEAN fgCmd,
    IN BOOLEAN fgIsP2pOid,
    OUT PUINT_32 pu4QryInfoLen
    )
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

    if(down_interruptible(&prGlueInfo->ioctl_sem)) {
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
    prIoReq->fgWaitResp= fgWaitResp;
    prIoReq->rStatus = WLAN_STATUS_FAILURE;
#if CFG_ENABLE_WIFI_DIRECT
    prIoReq->fgIsP2pOid = fgIsP2pOid;
#endif

    
    prGlueInfo->rPendStatus = WLAN_STATUS_FAILURE;
    
    

    
    prIoReq->u4Flag = fgCmd;

    
    set_bit(GLUE_FLAG_OID_BIT, &prGlueInfo->ulFlag);

    
    wake_up_interruptible(&prGlueInfo->waitq);

    
    
    
    wait_for_completion(&prGlueInfo->rPendComp); {
        
        
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

    up(&prGlueInfo->ioctl_sem);
    up(&g_halt_sem);

    return ret;
}

VOID
kalClearSecurityFrames(
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;

    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;
    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);

    
    prCmdQue = &prGlueInfo->rCmdQueue;



    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {
        prCmdInfo = (P_CMD_INFO_T)prQueueEntry;

        if (prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME) {
            prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
            cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
        }
        else {
            QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    QUEUE_CONCATENATE_QUEUES(prCmdQue, prTempCmdQue);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID
kalClearSecurityFramesByNetType(
    IN P_GLUE_INFO_T prGlueInfo,
    IN ENUM_NETWORK_TYPE_INDEX_T eNetworkTypeIdx
    )
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;

    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;
    GLUE_SPIN_LOCK_DECLARATION();


    ASSERT(prGlueInfo);

    
    prCmdQue = &prGlueInfo->rCmdQueue;

    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {
        prCmdInfo = (P_CMD_INFO_T)prQueueEntry;

        if (prCmdInfo->eCmdType == COMMAND_TYPE_SECURITY_FRAME &&
                prCmdInfo->eNetworkType == eNetworkTypeIdx) {
            prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
            cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
        }
        else {
            QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    QUEUE_CONCATENATE_QUEUES(prCmdQue, prTempCmdQue);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID
kalClearMgmtFrames(
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;
    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;

    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);

    
    prCmdQue = &prGlueInfo->rCmdQueue;

    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {
        prCmdInfo = (P_CMD_INFO_T)prQueueEntry;

        if (prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME) {
            wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo);
            cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
        }
        else {
            QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    QUEUE_CONCATENATE_QUEUES(prCmdQue, prTempCmdQue);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}


VOID
kalClearMgmtFramesByNetType (
    IN P_GLUE_INFO_T prGlueInfo,
    IN ENUM_NETWORK_TYPE_INDEX_T eNetworkTypeIdx
    )
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;
    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;

    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);

    
    prCmdQue = &prGlueInfo->rCmdQueue;

    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {
        prCmdInfo = (P_CMD_INFO_T)prQueueEntry;

        if (prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME &&
                prCmdInfo->eNetworkType == eNetworkTypeIdx) {
            wlanReleaseCommand(prGlueInfo->prAdapter, prCmdInfo);
            cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
        }
        else {
            QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    QUEUE_CONCATENATE_QUEUES(prCmdQue, prTempCmdQue);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
} 



int tx_thread(void *data)
{
	extern UINT_32 TaskIsrCnt;
    struct net_device   *dev = data;
    P_GLUE_INFO_T       prGlueInfo = *((P_GLUE_INFO_T *) netdev_priv(dev));

    P_QUE_ENTRY_T       prQueueEntry = NULL;
    P_GL_IO_REQ_T       prIoReq = NULL;
    P_QUE_T             prTxQueue = NULL;
    P_QUE_T             prCmdQue = NULL;

    int                 ret = 0;

    BOOLEAN             fgNeedHwAccess = FALSE;

    struct sk_buff      *prSkb = NULL;

    
    GLUE_SPIN_LOCK_DECLARATION();

    prTxQueue = &prGlueInfo->rTxQueue;
    prCmdQue = &prGlueInfo->rCmdQueue;

    current->flags |= PF_NOFREEZE;

    DBGLOG(INIT, INFO, ("tx_thread starts running... \n"));

    while (TRUE) {

#if CFG_ENABLE_WIFI_DIRECT
        
        if (test_and_clear_bit(GLUE_FLAG_SUB_MOD_MULTICAST_BIT, &prGlueInfo->ulFlag)) {
            p2pSetMulticastListWorkQueueWrapper(prGlueInfo);
        }
#endif

		if (test_and_clear_bit(GLUE_FLAG_FRAME_FILTER_AIS_BIT, &prGlueInfo->ulFlag)) {
			P_AIS_FSM_INFO_T prAisFsmInfo = (P_AIS_FSM_INFO_T)NULL;
			
			prAisFsmInfo = &(prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo);
			prAisFsmInfo->u4AisPacketFilter = prGlueInfo->u4OsMgmtFrameFilter;
		}

        if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
            KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &(prGlueInfo->prAdapter)->rTxThreadWakeLock);
            DBGLOG(INIT, INFO, ("tx_thread should stop now...\n"));
            break;
        }

        KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &(prGlueInfo->prAdapter)->rTxThreadWakeLock);

        ret = wait_event_interruptible(prGlueInfo->waitq,
                (prGlueInfo->ulFlag != 0));

        KAL_WAKE_LOCK(prGlueInfo->prAdapter, &(prGlueInfo->prAdapter)->rTxThreadWakeLock);


#if CFG_DBG_GPIO_PINS
        
        mtk_wcn_stp_debug_gpio_assert(IDX_TX_THREAD, DBG_TIE_LOW);
#endif
#if CFG_ENABLE_WIFI_DIRECT
        
        if (test_and_clear_bit(GLUE_FLAG_SUB_MOD_MULTICAST_BIT, &prGlueInfo->ulFlag)) {
            p2pSetMulticastListWorkQueueWrapper(prGlueInfo);
        }

        if (test_and_clear_bit(GLUE_FLAG_FRAME_FILTER_BIT, &prGlueInfo->ulFlag)) {
            p2pFuncUpdateMgmtFrameRegister(prGlueInfo->prAdapter, prGlueInfo->prP2PInfo->u4OsMgmtFrameFilter);
        }

        
#endif
        if (test_and_clear_bit(GLUE_FLAG_FRAME_FILTER_AIS_BIT, &prGlueInfo->ulFlag)) {
            P_AIS_FSM_INFO_T prAisFsmInfo = (P_AIS_FSM_INFO_T)NULL;
            
            prAisFsmInfo = &(prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo);
            prAisFsmInfo->u4AisPacketFilter = prGlueInfo->u4OsMgmtFrameFilter;
        }

        if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
            KAL_WAKE_UNLOCK(prGlueInfo->prAdapter, &(prGlueInfo->prAdapter)->rTxThreadWakeLock);
            DBGLOG(INIT, INFO, ("<1>tx_thread should stop now...\n"));
            break;
        }

        fgNeedHwAccess = FALSE;

        
        if (test_and_clear_bit(GLUE_FLAG_INT_BIT, &prGlueInfo->ulFlag)) {
            if (fgNeedHwAccess == FALSE) {
                fgNeedHwAccess = TRUE;

                wlanAcquirePowerControl(prGlueInfo->prAdapter);
            }

            prGlueInfo->prAdapter->fgIsIntEnable = FALSE;
            

            if (prGlueInfo->ulFlag & GLUE_FLAG_HALT) {
                
                DBGLOG(INIT, INFO, ("ignore pending interrupt\n"));
            }
            else {
				TaskIsrCnt ++;
                wlanIST(prGlueInfo->prAdapter);
            }
        }

        
        #if 0
        if (prGlueInfo->u4Flag & GLUE_FLAG_HALT) {
            printk(KERN_INFO DRV_NAME"<2>tx_thread should stop now...\n");
            break;
        }
        #endif

        do {
            if (test_and_clear_bit(GLUE_FLAG_OID_BIT, &prGlueInfo->ulFlag)) {
                
                prGlueInfo->u4OidCompleteFlag = 0;

                prIoReq = &(prGlueInfo->OidEntry);
#if CFG_ENABLE_WIFI_DIRECT
                if(prGlueInfo->prAdapter->fgIsP2PRegistered == FALSE
                    && prIoReq->fgIsP2pOid == TRUE) {
                }
                else
#endif
                {
                    if (FALSE == prIoReq->fgRead) {
                        prIoReq->rStatus = wlanSetInformation(
                                prIoReq->prAdapter,
                                prIoReq->pfnOidHandler,
                                prIoReq->pvInfoBuf,
                                prIoReq->u4InfoBufLen,
                                prIoReq->pu4QryInfoLen);
                    } else {
                        prIoReq->rStatus = wlanQueryInformation(
                                prIoReq->prAdapter,
                                prIoReq->pfnOidHandler,
                                prIoReq->pvInfoBuf,
                                prIoReq->u4InfoBufLen,
                                prIoReq->pu4QryInfoLen);
                    }

                    if (prIoReq->rStatus != WLAN_STATUS_PENDING) {
                        complete(&prGlueInfo->rPendComp);
                    }
                    else {
                        wlanoidTimeoutCheck(prGlueInfo->prAdapter, prIoReq->pfnOidHandler);
                    }
                }
             }

        } while (FALSE);


        #if 0
        if (prGlueInfo->u4Flag & GLUE_FLAG_HALT) {
            printk(KERN_INFO DRV_NAME"<3>tx_thread should stop now...\n");
            break;
        }
        #endif

        if (test_and_clear_bit(GLUE_FLAG_TXREQ_BIT, &prGlueInfo->ulFlag))
        {
            
            wlanProcessMboxMessage(prGlueInfo->prAdapter);

            
            do {
                if (prCmdQue->u4NumElem > 0) {
                    if (fgNeedHwAccess == FALSE) {
                        fgNeedHwAccess = TRUE;

                        wlanAcquirePowerControl(prGlueInfo->prAdapter);
                    }
                    wlanProcessCommandQueue(prGlueInfo->prAdapter, prCmdQue);
                }
            } while (FALSE);

            
            {
                while (QUEUE_IS_NOT_EMPTY(prTxQueue)) {
                    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
                    QUEUE_REMOVE_HEAD(prTxQueue, prQueueEntry, P_QUE_ENTRY_T);
                    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

                    ASSERT(prQueueEntry);
                    if (NULL == prQueueEntry) {
                        break;
                    }

                    prSkb = (struct sk_buff *) GLUE_GET_PKT_DESCRIPTOR(prQueueEntry);
                    ASSERT(prSkb);
                    if (NULL == prSkb) {
                        DBGLOG(INIT, WARN, ("prSkb == NULL in tx\n"));
                        continue;
                    }

#if (CFG_SUPPORT_TDLS_DBG == 1)
					if (prSkb != NULL)
					{
						UINT8 *pkt = prSkb->data;
						UINT16 u2Identifier;
						if ((*(pkt+12) == 0x08) && (*(pkt+13) == 0x00))
						{
							
							u2Identifier = ((*(pkt+18)) << 8) | (*(pkt+19));
							printk("<de> %d\n", u2Identifier);
						}
					}
#endif
                    if(wlanEnqueueTxPacket(prGlueInfo->prAdapter,
                                (P_NATIVE_PACKET)prSkb) == WLAN_STATUS_RESOURCES) {
                        
                        GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
                        QUEUE_INSERT_HEAD(prTxQueue, prQueueEntry);
                        GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

                        break;
                    }
                }

                if (wlanGetTxPendingFrameCount(prGlueInfo->prAdapter) > 0) {
                    
                    wlanTxPendingPackets(prGlueInfo->prAdapter, &fgNeedHwAccess);
                }
            }

        }

        

        

        
        if (fgNeedHwAccess == TRUE) {
            wlanReleasePowerControl(prGlueInfo->prAdapter);
        }

        
        if (test_and_clear_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag)) {
            wlanTimerTimeoutCheck(prGlueInfo->prAdapter);
        }

    #if CFG_DBG_GPIO_PINS
        
        if (!prGlueInfo->ulFlag){
            mtk_wcn_stp_debug_gpio_assert(IDX_TX_THREAD, DBG_TIE_HIGH);
        }
    #endif
    }


    #if 0
    if (fgNeedHwAccess == TRUE) {
        wlanReleasePowerControl(prGlueInfo->prAdapter);
    }
    #endif

    
    
    if (prGlueInfo->i4TxPendingFrameNum > 0) {
        kalFlushPendingTxPackets(prGlueInfo);
    }

    
    if (prGlueInfo->i4TxPendingSecurityFrameNum > 0) {
        kalClearSecurityFrames(prGlueInfo);
    }

    
    wlanReleasePendingOid(prGlueInfo->prAdapter, 0);


    

    DBGLOG(INIT, INFO, ("mtk_sdiod stops\n"));
    complete(&prGlueInfo->rHaltComp);

    return 0;

}

BOOLEAN
kalIsCardRemoved(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return FALSE;
    
}

BOOLEAN
kalRetrieveNetworkAddress(
    IN P_GLUE_INFO_T prGlueInfo,
    IN OUT PARAM_MAC_ADDRESS * prMacAddr
    )
{
    ASSERT(prGlueInfo);

    if(prGlueInfo->fgIsMacAddrOverride == FALSE) {
    #if !defined(CONFIG_X86)
	#if !defined(CONFIG_MTK_TC1_FEATURE)
			UINT_32 i;
	#endif
        BOOLEAN fgIsReadError = FALSE;

	#if !defined(CONFIG_MTK_TC1_FEATURE)
        for(i = 0 ; i < MAC_ADDR_LEN ; i+=2) {
            if(kalCfgDataRead16(prGlueInfo,
                        OFFSET_OF(WIFI_CFG_PARAM_STRUCT, aucMacAddress) + i,
                        (PUINT_16) (((PUINT_8)prMacAddr) + i)) == FALSE) {
                fgIsReadError = TRUE;
                break;
            }
        }
	#else
		TC1_FAC_NAME(FacReadWifiMacAddr)((unsigned char *)prMacAddr);
	#endif

        if(fgIsReadError == TRUE) {
            return FALSE;
        }
        else {
            return TRUE;
        }
    #else
        
        return FALSE;
    #endif
    }
    else {
        COPY_MAC_ADDR(prMacAddr, prGlueInfo->rMacAddrOverride);

        return TRUE;
    }
}


VOID
kalFlushPendingTxPackets(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    P_QUE_T prTxQue;
    P_QUE_ENTRY_T prQueueEntry;
    PVOID prPacket;

    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);

    prTxQue = &(prGlueInfo->rTxQueue);

    if (prGlueInfo->i4TxPendingFrameNum) {
        while (TRUE) {
            GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);
            QUEUE_REMOVE_HEAD(prTxQue, prQueueEntry, P_QUE_ENTRY_T);
            GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_TX_QUE);

            if (prQueueEntry == NULL) {
                break;
            }

            prPacket = GLUE_GET_PKT_DESCRIPTOR(prQueueEntry);

            kalSendComplete(prGlueInfo,
                    prPacket,
                    WLAN_STATUS_NOT_ACCEPTED);
        }
    }
}

ENUM_PARAM_MEDIA_STATE_T
kalGetMediaStateIndicated(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return prGlueInfo->eParamMediaStateIndicated;
}


VOID
kalSetMediaStateIndicated(
    IN P_GLUE_INFO_T            prGlueInfo,
    IN ENUM_PARAM_MEDIA_STATE_T eParamMediaStateIndicate
    )
{
    ASSERT(prGlueInfo);

    prGlueInfo->eParamMediaStateIndicated = eParamMediaStateIndicate;
}


VOID
kalOidCmdClearance(
    IN P_GLUE_INFO_T prGlueInfo
    )
{
    P_QUE_T prCmdQue;
    QUE_T rTempCmdQue;
    P_QUE_T prTempCmdQue = &rTempCmdQue;
    P_QUE_ENTRY_T prQueueEntry = (P_QUE_ENTRY_T)NULL;
    P_CMD_INFO_T prCmdInfo = (P_CMD_INFO_T)NULL;

    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);

    prCmdQue = &prGlueInfo->rCmdQueue;

    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_MOVE_ALL(prTempCmdQue, prCmdQue);

    QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    while (prQueueEntry) {

        if (((P_CMD_INFO_T)prQueueEntry)->fgIsOid) {
            prCmdInfo = (P_CMD_INFO_T)prQueueEntry;
            break;
        }
        else {
            QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
        }

        QUEUE_REMOVE_HEAD(prTempCmdQue, prQueueEntry, P_QUE_ENTRY_T);
    }

    QUEUE_CONCATENATE_QUEUES(prCmdQue, prTempCmdQue);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);

    if (prCmdInfo) {
        if (prCmdInfo->pfCmdTimeoutHandler) {
            prCmdInfo->pfCmdTimeoutHandler(prGlueInfo->prAdapter, prCmdInfo);
        }
        else {
            kalOidComplete(prGlueInfo,
                    prCmdInfo->fgSetQuery,
                    0,
                    WLAN_STATUS_NOT_ACCEPTED);
        }

        prGlueInfo->u4OidCompleteFlag = 1;
        cmdBufFreeCmdInfo(prGlueInfo->prAdapter, prCmdInfo);
    }
}


VOID
kalEnqueueCommand(
    IN P_GLUE_INFO_T prGlueInfo,
    IN P_QUE_ENTRY_T prQueueEntry
    )
{
    P_QUE_T prCmdQue;
    P_CMD_INFO_T prCmdInfo;
    P_MSDU_INFO_T prMsduInfo;

    GLUE_SPIN_LOCK_DECLARATION();

    ASSERT(prGlueInfo);
    ASSERT(prQueueEntry);

    prCmdQue = &prGlueInfo->rCmdQueue;

    prCmdInfo = (P_CMD_INFO_T)prQueueEntry;
    if (prCmdInfo->prPacket &&
        prCmdInfo->eCmdType == COMMAND_TYPE_MANAGEMENT_FRAME) {
        prMsduInfo = (P_MSDU_INFO_T)(prCmdInfo->prPacket);
        prMsduInfo->eCmdType = prCmdInfo->eCmdType;
        prMsduInfo->ucCID = prCmdInfo->ucCID;
        prMsduInfo->u4InqueTime = kalGetTimeTick();
    }

    GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
    QUEUE_INSERT_TAIL(prCmdQue, prQueueEntry);
    GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_CMD_QUE);
}

VOID
kalHandleAssocInfo(
    IN P_GLUE_INFO_T prGlueInfo,
    IN P_EVENT_ASSOC_INFO prAssocInfo
    )
{
    
}

UINT_32
kalGetFwLoadAddress(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return prGlueInfo->rRegInfo.u4LoadAddress;
}


UINT_32
kalGetFwStartAddress(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return prGlueInfo->rRegInfo.u4StartAddress;
}


VOID
kalSecurityFrameSendComplete (
    IN P_GLUE_INFO_T prGlueInfo,
    IN PVOID pvPacket,
    IN WLAN_STATUS rStatus
    )
{
    ASSERT(pvPacket);

    dev_kfree_skb((struct sk_buff *) pvPacket);
    GLUE_DEC_REF_CNT(prGlueInfo->i4TxPendingSecurityFrameNum);
}

UINT_32
kalGetTxPendingFrameCount(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return (UINT_32)(prGlueInfo->i4TxPendingFrameNum);
}


UINT_32
kalGetTxPendingCmdCount(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    P_QUE_T prCmdQue;

    ASSERT(prGlueInfo);
    prCmdQue = &prGlueInfo->rCmdQueue;

    return prCmdQue->u4NumElem;
}




VOID
kalOsTimerInitialize (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN PVOID            prTimerHandler
    )
{

    ASSERT(prGlueInfo);

    init_timer(&(prGlueInfo->tickfn));
    prGlueInfo->tickfn.function = prTimerHandler;
    prGlueInfo->tickfn.data = (ULONG) prGlueInfo;
}

BOOLEAN
kalSetTimer(
    IN P_GLUE_INFO_T    prGlueInfo,
    IN UINT_32          u4Interval
    )
{
    ASSERT(prGlueInfo);
    del_timer_sync(&(prGlueInfo->tickfn));

    prGlueInfo->tickfn.expires = jiffies + u4Interval * HZ / MSEC_PER_SEC;
    add_timer(&(prGlueInfo->tickfn));

    return TRUE; 
}
BOOLEAN
kalCancelTimer(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    clear_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag);

    if (del_timer_sync(&(prGlueInfo->tickfn)) >=0) {
        return TRUE;
    } else {
        return FALSE;
    }
}
VOID
kalScanDone(
    IN P_GLUE_INFO_T                 prGlueInfo,
    IN ENUM_KAL_NETWORK_TYPE_INDEX_T eNetTypeIdx,
    IN WLAN_STATUS                   status
    )
{
    P_AIS_FSM_INFO_T prAisFsmInfo;

    ASSERT(prGlueInfo);

    prAisFsmInfo = &(prGlueInfo->prAdapter->rWifiVar.rAisFsmInfo);
	
	scanReportBss2Cfg80211(prGlueInfo->prAdapter,BSS_TYPE_INFRASTRUCTURE,NULL);	
    cnmTimerStopTimer(prGlueInfo->prAdapter, &prAisFsmInfo->rScanDoneTimer);

    
    wlanCheckSystemConfiguration(prGlueInfo->prAdapter);

    kalIndicateStatusAndComplete(prGlueInfo, WLAN_STATUS_SCAN_COMPLETE, NULL, 0);
}

UINT_32
kalRandomNumber(
    VOID
    )
{
    UINT_32 number = 0;

    get_random_bytes(&number, 4);

    return number;
}

VOID
kalTimeoutHandler (ULONG arg)
{

    P_GLUE_INFO_T prGlueInfo = (P_GLUE_INFO_T) arg;

    ASSERT(prGlueInfo);

    
    set_bit(GLUE_FLAG_TIMEOUT_BIT, &prGlueInfo->ulFlag);
    wake_up_interruptible(&prGlueInfo->waitq);

    return;
}


VOID
kalSetEvent (P_GLUE_INFO_T pr) {
    set_bit(GLUE_FLAG_TXREQ_BIT, &pr->ulFlag);
    wake_up_interruptible(&pr->waitq);
}


BOOLEAN
kalIsConfigurationExist(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
#if !defined(CONFIG_X86)
    ASSERT(prGlueInfo);

    return prGlueInfo->fgNvramAvailable;
#else
    
    return FALSE;
#endif
}


P_REG_INFO_T
kalGetConfiguration(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return &(prGlueInfo->rRegInfo);
}


VOID
kalGetConfigurationVersion(
    IN P_GLUE_INFO_T    prGlueInfo,
    OUT PUINT_16        pu2Part1CfgOwnVersion,
    OUT PUINT_16        pu2Part1CfgPeerVersion,
    OUT PUINT_16        pu2Part2CfgOwnVersion,
    OUT PUINT_16        pu2Part2CfgPeerVersion
    )
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

BOOLEAN
kalWSCGetActiveState(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return (prGlueInfo->fgWpsActive);
}


VOID
kalUpdateRSSI(
    IN P_GLUE_INFO_T                    prGlueInfo,
    IN ENUM_KAL_NETWORK_TYPE_INDEX_T    eNetTypeIdx,
    IN INT_8                            cRssi,
    IN INT_8                            cLinkQuality
    )
{
    struct iw_statistics *pStats = (struct iw_statistics *)NULL;

    ASSERT(prGlueInfo);

    switch(eNetTypeIdx) {
    case KAL_NETWORK_TYPE_AIS_INDEX:
        pStats =  (struct iw_statistics *) (&(prGlueInfo->rIwStats));
        break;
#if CFG_ENABLE_WIFI_DIRECT
#if CFG_SUPPORT_P2P_RSSI_QUERY
    case KAL_NETWORK_TYPE_P2P_INDEX:
        pStats =  (struct iw_statistics *) (&(prGlueInfo->rP2pIwStats));
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


BOOLEAN
kalInitIOBuffer(
    VOID
    )
{
    UINT_32 u4Size;

    if(CFG_COALESCING_BUFFER_SIZE >= CFG_RX_COALESCING_BUFFER_SIZE) {
        u4Size = CFG_COALESCING_BUFFER_SIZE + sizeof(ENHANCE_MODE_DATA_STRUCT_T);
    }
    else {
        u4Size = CFG_RX_COALESCING_BUFFER_SIZE + sizeof(ENHANCE_MODE_DATA_STRUCT_T);
    }

#ifdef MTK_DMA_BUF_MEMCPY_SUP
    pvDmaBuffer = dma_alloc_coherent(NULL, CFG_RX_MAX_PKT_SIZE, &pvDmaPhyBuf, GFP_KERNEL);
    if (pvDmaBuffer == NULL)
        return FALSE;
#endif 

    pvIoBuffer = kmalloc(u4Size, GFP_KERNEL);
    if(pvIoBuffer) {
        pvIoBufferSize = u4Size;
        pvIoBufferUsage = 0;

        return TRUE;
    }

    return FALSE;
}


VOID
kalUninitIOBuffer(
    VOID
    )
{
    if(pvIoBuffer) {
        kfree(pvIoBuffer);
#ifdef MTK_DMA_BUF_MEMCPY_SUP
        dma_free_coherent(NULL, CFG_RX_MAX_PKT_SIZE, pvDmaBuffer, pvDmaPhyBuf);
#endif 

        pvIoBuffer = (PVOID) NULL;
        pvIoBufferSize = 0;
        pvIoBufferUsage = 0;
    }

    return;
}


PVOID
kalAllocateIOBuffer(
    IN UINT_32 u4AllocSize
    )
{
    PVOID ret = (PVOID)NULL;

    if(pvIoBuffer) {
        if(u4AllocSize <= (pvIoBufferSize - pvIoBufferUsage)) {
            ret = (PVOID)&(((PUINT_8)(pvIoBuffer))[pvIoBufferUsage]);
            pvIoBufferUsage += u4AllocSize;
        }
    }
    else {
        
        ret = (PVOID) kalMemAlloc(u4AllocSize, PHY_MEM_TYPE);
    }

    return ret;
}


VOID
kalReleaseIOBuffer(
    IN PVOID pvAddr,
    IN UINT_32 u4Size
    )
{
    if(pvIoBuffer) {
        pvIoBufferUsage -= u4Size;
    }
    else {
        
        kalMemFree(pvAddr, PHY_MEM_TYPE, u4Size);
    }
}

VOID
kalGetChannelList(
    IN P_GLUE_INFO_T           prGlueInfo,
    IN ENUM_BAND_T             eSpecificBand,
    IN UINT_8                  ucMaxChannelNum,
    IN PUINT_8                 pucNumOfChannel,
    IN P_RF_CHANNEL_INFO_T     paucChannelList
    )
{
    rlmDomainGetChnlList(prGlueInfo->prAdapter,
                              eSpecificBand,
                              ucMaxChannelNum,
                              pucNumOfChannel,
                              paucChannelList);
}


BOOLEAN
kalIsAPmode(
    IN P_GLUE_INFO_T           prGlueInfo
    )
{
#if CFG_ENABLE_WIFI_DIRECT
    if (IS_NET_ACTIVE(prGlueInfo->prAdapter, NETWORK_TYPE_P2P_INDEX) &&
        p2pFuncIsAPMode(prGlueInfo->prAdapter->rWifiVar.prP2pFsmInfo))
        return TRUE;
#endif

    return FALSE;
}


#ifdef MTK_DMA_BUF_MEMCPY_SUP
ULONG

kalIOPhyAddrGet (
    IN ULONG                      VirtAddr
    )
{
    ULONG PhyAddr;


    if ((VirtAddr >= (ULONG)pvIoBuffer) &&
        (VirtAddr <= ((ULONG)(pvIoBuffer) + pvIoBufferSize)))
    {
        PhyAddr = (ULONG)pvIoPhyBuf;
        PhyAddr += (VirtAddr - (ULONG)(pvIoBuffer));
        return PhyAddr;
    }

    return 0;
}


VOID
kalDmaBufGet (
    OUT VOID                        **VirtAddr,
    OUT VOID                        **PhyAddr
    )
{
    *VirtAddr = pvDmaBuffer;
    *PhyAddr = pvDmaPhyBuf;
}
#endif 


#if CFG_SUPPORT_802_11W
UINT_32
kalGetMfpSetting(
    IN P_GLUE_INFO_T    prGlueInfo
    )
{
    ASSERT(prGlueInfo);

    return (prGlueInfo->rWpaInfo.u4Mfp);
}
#endif

struct file*
kalFileOpen(
    const char* path,
    int flags,
    int rights)
{
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

VOID
kalFileClose(
    struct file* file)
{
    filp_close(file, NULL);
}

UINT_32
kalFileRead(
    struct file* file,
    UINT_64 offset,
    UINT_8* data,
    UINT_32 size)
{
    mm_segment_t oldfs;
    INT_32 ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

UINT_32
kalFileWrite(
    struct file* file,
    UINT_64 offset,
    UINT_8* data,
    UINT_32 size)
{
    mm_segment_t oldfs;
    INT_32 ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

UINT_32
kalWriteToFile(
    const PUINT_8 pucPath,
    BOOLEAN fgDoAppend,
    PUINT_8 pucData,
    UINT_32 u4Size)
{
    struct file* file = NULL;
    UINT_32 ret;
    UINT_32 u4Flags = 0;

    if(fgDoAppend) {
        u4Flags = O_APPEND;
    }

    file = kalFileOpen(pucPath, O_WRONLY | O_CREAT | u4Flags, S_IRWXU);
    ret = kalFileWrite(file, 0, pucData, u4Size);
    kalFileClose(file);

    return ret;
}

INT_32
kalReadToFile(
    const PUINT_8 pucPath,
    PUINT_8 pucData,
    UINT_32 u4Size,
    PUINT_32 pu4ReadSize
    )
{
    struct file* file = NULL;
    INT_32 ret = -1;
    UINT_32 u4ReadSize = 0;

    DBGLOG(INIT, INFO, ("kalReadToFile() path %s\n", pucPath));

    file = kalFileOpen(pucPath, O_RDONLY , 0);

    if ((file != NULL) && !IS_ERR(file)) {
        u4ReadSize = kalFileRead(file, 0, pucData, u4Size);
        kalFileClose(file);
        if(pu4ReadSize) *pu4ReadSize = u4ReadSize;
        ret = 0;
    }
    return ret;
}


VOID
kalIndicateBssInfo (
    IN P_GLUE_INFO_T        prGlueInfo,
    IN PUINT_8              pucBeaconProbeResp,
    IN UINT_32              u4FrameLen,
    IN UINT_8               ucChannelNum,
    IN INT_32               i4SignalStrength
    )
{
    struct wiphy *wiphy;
    struct ieee80211_channel *prChannel = NULL;

    ASSERT(prGlueInfo);
    wiphy = priv_to_wiphy(prGlueInfo);

    
    if(ucChannelNum <= 14) {
        prChannel = ieee80211_get_channel(wiphy, ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_2GHZ));
    }
    else {
        prChannel = ieee80211_get_channel(wiphy, ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_5GHZ));
    }

    if(prChannel != NULL && prGlueInfo->prScanRequest != NULL) {
        struct cfg80211_bss *bss;
#if CFG_SUPPORT_TSF_USING_BOOTTIME
		struct ieee80211_mgmt *prMgmtFrame = (struct ieee80211_mgmt *)pucBeaconProbeResp;
		prMgmtFrame->u.beacon.timestamp = kalGetBootTime();
#endif
		ScanCnt ++;
		
        
        bss = cfg80211_inform_bss_frame(wiphy,
                prChannel,
                (struct ieee80211_mgmt *)pucBeaconProbeResp,
                u4FrameLen,
                i4SignalStrength * 100,
                GFP_KERNEL);

        if(!bss) {
			ScanDoneFailCnt ++;
            DBGLOG(SCN, TRACE, ("cfg80211_inform_bss_frame() returned with NULL\n"));
        }
        else {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
            cfg80211_put_bss(wiphy, bss);
#else
            cfg80211_put_bss(bss);
#endif
			DBGLOG(SCN, TRACE, ("iok\n"));
        }
    }

    return;
}


VOID
kalReadyOnChannel (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN UINT_64          u8Cookie,
    IN ENUM_BAND_T      eBand,
    IN ENUM_CHNL_EXT_T  eSco,
    IN UINT_8           ucChannelNum,
    IN UINT_32          u4DurationMs
    )
{
    struct ieee80211_channel *prChannel = NULL;
    enum nl80211_channel_type rChannelType;
    
    

    if(prGlueInfo->fgIsRegistered == TRUE) {
        if(ucChannelNum <= 14) {
            prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_2GHZ));
        }
        else {
            prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_5GHZ));
        }

        switch(eSco) {
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
        cfg80211_ready_on_channel(prGlueInfo->prDevHandler->ieee80211_ptr, u8Cookie, prChannel, u4DurationMs, GFP_KERNEL);
#else
        cfg80211_ready_on_channel(prGlueInfo->prDevHandler, u8Cookie, prChannel, rChannelType, u4DurationMs, GFP_KERNEL);
#endif
    }

    return;
}


VOID
kalRemainOnChannelExpired (
    IN P_GLUE_INFO_T    prGlueInfo,
    IN UINT_64          u8Cookie,
    IN ENUM_BAND_T      eBand,
    IN ENUM_CHNL_EXT_T  eSco,
    IN UINT_8           ucChannelNum
    )
{
    struct ieee80211_channel *prChannel = NULL;
    enum nl80211_channel_type rChannelType;
    
    ucChannelNum = wlanGetChannelNumberByNetwork(prGlueInfo->prAdapter, NETWORK_TYPE_AIS_INDEX);

    if(prGlueInfo->fgIsRegistered == TRUE) {
        if(ucChannelNum <= 14) {
            prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_2GHZ));
        }
        else {
            prChannel = ieee80211_get_channel(priv_to_wiphy(prGlueInfo), ieee80211_channel_to_frequency(ucChannelNum, IEEE80211_BAND_5GHZ));
        }

        switch(eSco) {
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
        cfg80211_remain_on_channel_expired(prGlueInfo->prDevHandler->ieee80211_ptr, u8Cookie, prChannel, GFP_KERNEL);
#else
        cfg80211_remain_on_channel_expired(prGlueInfo->prDevHandler, u8Cookie, prChannel, rChannelType, GFP_KERNEL);
#endif
    }

    return;
}


VOID
kalIndicateMgmtTxStatus (
    IN P_GLUE_INFO_T prGlueInfo,
    IN UINT_64 u8Cookie,
    IN BOOLEAN fgIsAck,
    IN PUINT_8 pucFrameBuf,
    IN UINT_32 u4FrameLen
    )
{

    do {
        if ((prGlueInfo == NULL) ||
                (pucFrameBuf == NULL) ||
                (u4FrameLen == 0)) {
            DBGLOG(AIS, TRACE, ("Unexpected pointer PARAM. %p, %p, %u.",
				prGlueInfo, pucFrameBuf, u4FrameLen));
            ASSERT(FALSE);
            break;
        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
        cfg80211_mgmt_tx_status(prGlueInfo->prDevHandler->ieee80211_ptr,
                        u8Cookie,
                        pucFrameBuf,
                        u4FrameLen,
                        fgIsAck,
                        GFP_KERNEL);
#else
        cfg80211_mgmt_tx_status(prGlueInfo->prDevHandler,
                        u8Cookie,
                        pucFrameBuf,
                        u4FrameLen,
                        fgIsAck,
                        GFP_KERNEL);
#endif

    } while (FALSE);

} 


VOID
kalIndicateRxMgmtFrame (
    IN P_GLUE_INFO_T prGlueInfo,
    IN P_SW_RFB_T prSwRfb
    )
{
#define DBG_MGMT_FRAME_INDICATION 1
    INT_32 i4Freq = 0;
    UINT_8 ucChnlNum = 0;
#if DBG_MGMT_FRAME_INDICATION
    P_WLAN_MAC_HEADER_T prWlanHeader = (P_WLAN_MAC_HEADER_T)NULL;
#endif

    do {
        if ((prGlueInfo == NULL) || (prSwRfb == NULL)) {
            ASSERT(FALSE);
            break;
        }

        ucChnlNum = prSwRfb->prHifRxHdr->ucHwChannelNum;

#if DBG_MGMT_FRAME_INDICATION
        prWlanHeader = (P_WLAN_MAC_HEADER_T)prSwRfb->pvHeader;

        switch (prWlanHeader->u2FrameCtrl) {
        case MAC_FRAME_PROBE_REQ:
            DBGLOG(AIS, TRACE, ("RX Probe Req at channel %d ", ucChnlNum));
            break;
        case MAC_FRAME_PROBE_RSP:
            DBGLOG(AIS, TRACE, ("RX Probe Rsp at channel %d ", ucChnlNum));
            break;
        case MAC_FRAME_ACTION:
            printk("RX Action frame at channel %d ", ucChnlNum);
            break;
        default:
            DBGLOG(AIS, TRACE, ("RX Packet:%d at channel %d ", prWlanHeader->u2FrameCtrl, ucChnlNum));
            break;
        }

#endif
        i4Freq = nicChannelNum2Freq(ucChnlNum) / 1000;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
        cfg80211_rx_mgmt(prGlueInfo->prDevHandler->ieee80211_ptr, 
                            i4Freq,
                            RCPI_TO_dBm(prSwRfb->prHifRxHdr->ucRcpi),
                            prSwRfb->pvHeader,
                            prSwRfb->u2PacketLen,
                            GFP_KERNEL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
        cfg80211_rx_mgmt(prGlueInfo->prDevHandler, 
                            i4Freq,
                            RCPI_TO_dBm(prSwRfb->prHifRxHdr->ucRcpi),
                            prSwRfb->pvHeader,
                            prSwRfb->u2PacketLen,
                            GFP_KERNEL);
#else
        cfg80211_rx_mgmt(prGlueInfo->prDevHandler, 
                            i4Freq,
                            prSwRfb->pvHeader,
                            prSwRfb->u2PacketLen,
                            GFP_KERNEL);
#endif

    } while (FALSE);

} 

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

#if (CFG_SUPPORT_MET_PROFILING == 1)
#define PROC_MET_PROF_CTRL                 "met_ctrl"
#define PROC_MET_PROF_PORT                 "met_port"

struct proc_dir_entry *pMetProcDir; 
void *pMetGlobalData = NULL;
static unsigned long __read_mostly tracing_mark_write_addr = 0;

static void inline __mt_update_tracing_mark_write_addr(void)
{
      if(unlikely(0 == tracing_mark_write_addr))
            tracing_mark_write_addr = kallsyms_lookup_name("tracing_mark_write");
}

VOID
kalMetProfilingStart (
    IN P_GLUE_INFO_T prGlueInfo,
    IN struct sk_buff *prSkb
    )
{
    UINT_8 ucIpVersion;	
    UINT_16 u2UdpSrcPort;
    UINT_16 u2RtpSn;
    PUINT_8 pucEthHdr = prSkb->data;
    PUINT_8 pucIpHdr, pucUdpHdr, pucRtpHdr;
	
    
    
    
    
    if (prGlueInfo->u8MetProfEnable == 1) {
        u2UdpSrcPort = prGlueInfo->u16MetUdpPort;
        if ((*(pucEthHdr + 12) == 0x08) && (*(pucEthHdr + 13) == 0x00)) {
            
            pucIpHdr = pucEthHdr + ETH_HLEN;
            ucIpVersion = (*pucIpHdr & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
            if (ucIpVersion == IPVERSION) {
                if(pucIpHdr[IPV4_HDR_IP_PROTOCOL_OFFSET] == IP_PROTOCOL_UDP) {
                    
                    pucUdpHdr = pucIpHdr + IP_HEADER_LEN;
                    
                    if (((UINT_16)pucUdpHdr[0] << 8 |(UINT_16)pucUdpHdr[1]) == u2UdpSrcPort) {
                        
                        pucRtpHdr = pucUdpHdr + 8;
                        u2RtpSn = (UINT_16)pucRtpHdr[2] << 8 | pucRtpHdr[3];
                        
                        #ifdef CONFIG_TRACING
                        __mt_update_tracing_mark_write_addr();
                        if (tracing_mark_write_addr != 0) {
                            event_trace_printk(tracing_mark_write_addr, "S|%d|%s|%d\n", current->tgid, "WIFI-CHIP", u2RtpSn);
                        }
                        #endif
                    }
                }
            }
        }
    }
}

VOID
kalMetProfilingFinish (
    IN P_ADAPTER_T  prAdapter,
    IN P_MSDU_INFO_T prMsduInfo
    )
{
    UINT_8 ucIpVersion;	
    UINT_16 u2UdpSrcPort;
    UINT_16 u2RtpSn;
    struct sk_buff *prSkb = (struct sk_buff *)prMsduInfo->prPacket;	
    PUINT_8 pucEthHdr = prSkb->data;
    PUINT_8 pucIpHdr, pucUdpHdr, pucRtpHdr;
    P_GLUE_INFO_T       prGlueInfo = prAdapter->prGlueInfo;
	
    
    
    
    
    if (prGlueInfo->u8MetProfEnable == 1) {
        u2UdpSrcPort = prGlueInfo->u16MetUdpPort;
        if ((*(pucEthHdr + 12) == 0x08) && (*(pucEthHdr + 13) == 0x00)) {
            
            pucIpHdr = pucEthHdr + ETH_HLEN;
            ucIpVersion = (*pucIpHdr & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
            if (ucIpVersion == IPVERSION) {
                if(pucIpHdr[IPV4_HDR_IP_PROTOCOL_OFFSET] == IP_PROTOCOL_UDP) {
                    
                    pucUdpHdr = pucIpHdr + IP_HEADER_LEN;
                    
                    if (((UINT_16)pucUdpHdr[0] << 8 |(UINT_16)pucUdpHdr[1]) == u2UdpSrcPort) {
                        
                        pucRtpHdr = pucUdpHdr + 8;
                        u2RtpSn = (UINT_16)pucRtpHdr[2] << 8 | pucRtpHdr[3];
                        
                        #ifdef CONFIG_TRACING
                        __mt_update_tracing_mark_write_addr();
                        if (tracing_mark_write_addr != 0) {
                            event_trace_printk(tracing_mark_write_addr, "F|%d|%s|%d\n", current->tgid, "WIFI-CHIP", u2RtpSn);
                        }
                        #endif                        
                    }
                }
            }
        }
    }
}

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
    

    u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
    copy_from_user(acBuf, buffer, u4CopySize);
    acBuf[u4CopySize] = '\0';

    if (sscanf(acBuf, " %d", &u8MetProfEnable) == 1) {
        printk("MET_PROF: Write MET PROC Enable=%d \n", u8MetProfEnable);
    }
    if (pMetGlobalData != NULL) {
        prGlueInfo = (P_GLUE_INFO_T) pMetGlobalData;
        prGlueInfo->u8MetProfEnable = (UINT_8)u8MetProfEnable;
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
    

    u4CopySize = (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
    copy_from_user(acBuf, buffer, u4CopySize);
    acBuf[u4CopySize] = '\0';

    if (sscanf(acBuf, " %d", &u16MetUdpPort) == 1) {
        printk("MET_PROF: Write MET PROC UDP_PORT=%d\n", u16MetUdpPort);
    }
    if (pMetGlobalData != NULL) {
        prGlueInfo = (P_GLUE_INFO_T) pMetGlobalData;
		    prGlueInfo->u16MetUdpPort = (UINT_16)u16MetUdpPort;
		}	
    return count;
}

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
    remove_proc_entry(PROC_MET_PROF_PORT,      pMetProcDir);
    
    remove_proc_entry("wlan0", init_net.proc_net);
    
    pMetGlobalData = NULL;
    
    return 0;
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
