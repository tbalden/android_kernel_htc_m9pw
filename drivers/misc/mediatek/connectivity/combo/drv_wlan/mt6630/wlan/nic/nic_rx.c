





#include "precomp.h"
#include "que_mgt.h"

#ifndef LINUX
#include <limits.h>
#else
#include <linux/limits.h>
#endif



#define RX_RESPONSE_TIMEOUT (1000)

#if CFG_SUPPORT_SNIFFER
const EMU_MAC_RATE_INFO_T arMcsRate2PhyRate[] = 
{
    
    RATE_INFO(PHY_RATE_MCS0 ,   65,     72,     135,    150,    293,    325,    585,    650),
    RATE_INFO(PHY_RATE_MCS1 ,   130,    144,    270,    300,    585,    650,    1170,   1300),
    RATE_INFO(PHY_RATE_MCS2 ,   195,    217,    405,    450,    878,    975,    1755,   1950),
    RATE_INFO(PHY_RATE_MCS3 ,   260,    289,    540,    600,    1170,   1300,   2340,   2600),
    RATE_INFO(PHY_RATE_MCS4 ,   390,    433,    810,    900,    1755,   1950,   3510,   3900),
    RATE_INFO(PHY_RATE_MCS5 ,   520,    578,    1080,   1200,   2340,   2600,   4680,   5200),
    RATE_INFO(PHY_RATE_MCS6 ,   585,    650,    1215,   1350,   2633,   2925,   5265,   5850),
    RATE_INFO(PHY_RATE_MCS7 ,   650,    722,    1350,   1500,   2925,   3250,   5850,   6500),
    RATE_INFO(PHY_RATE_MCS8 ,   780,    867,    1620,   1800,   3510,   3900,   7020,   7800),
    RATE_INFO(PHY_RATE_MCS9 ,   0,      0,      1800,   2000,   3900,   4333,   7800,   8667),
    RATE_INFO(PHY_RATE_MCS32 ,  0,      0,      60,     67,      0,      0,      0,      0)
};

const UINT_8 aucHwRate2PhyRate[] = {
    RATE_1M,    
    RATE_2M,    
    RATE_5_5M,  
    RATE_11M,   
    RATE_1M,    
    RATE_2M,    
    RATE_5_5M,  
    RATE_11M,   
    RATE_48M,   
    RATE_24M,   
    RATE_12M,   
    RATE_6M,    
    RATE_54M,   
    RATE_36M,   
    RATE_18M,   
    RATE_9M     
};
#endif




#if CFG_MGMT_FRAME_HANDLING
static PROCESS_RX_MGT_FUNCTION apfnProcessRxMgtFrame[MAX_NUM_OF_FC_SUBTYPES] = {
#if CFG_SUPPORT_AAA
    aaaFsmRunEventRxAssoc,              
#else
    NULL,                               
#endif				
    saaFsmRunEventRxAssoc,              
#if CFG_SUPPORT_AAA
    aaaFsmRunEventRxAssoc,              
#else
    NULL,                               
#endif				
    saaFsmRunEventRxAssoc,              
#if CFG_SUPPORT_ADHOC || CFG_ENABLE_WIFI_DIRECT
    bssProcessProbeRequest,             
#else
    NULL,                               
#endif				
    scanProcessBeaconAndProbeResp,      
    NULL,                               
    NULL,                               
    scanProcessBeaconAndProbeResp,      
    NULL,                               
    saaFsmRunEventRxDisassoc,           
    authCheckRxAuthFrameTransSeq,       
    saaFsmRunEventRxDeauth,             
    nicRxProcessActionFrame,            
    NULL,                               
    NULL                                
};
#endif




VOID nicRxInitialize(IN P_ADAPTER_T prAdapter)
{
    P_RX_CTRL_T prRxCtrl;
    PUINT_8 pucMemHandle;
	P_SW_RFB_T prSwRfb = (P_SW_RFB_T) NULL;
    UINT_32 i;

    DEBUGFUNC("nicRxInitialize");

    ASSERT(prAdapter);
    prRxCtrl = &prAdapter->rRxCtrl;

	
    kalMemZero((PVOID) prRxCtrl->pucRxCached, prRxCtrl->u4RxCachedSize);

	
    QUEUE_INITIALIZE(&prRxCtrl->rFreeSwRfbList);
    QUEUE_INITIALIZE(&prRxCtrl->rReceivedRfbList);
    QUEUE_INITIALIZE(&prRxCtrl->rIndicatedRfbList);

    pucMemHandle = prRxCtrl->pucRxCached;
    for (i = CFG_RX_MAX_PKT_NUM; i != 0; i--) {
		prSwRfb = (P_SW_RFB_T) pucMemHandle;

        nicRxSetupRFB(prAdapter, prSwRfb);
        nicRxReturnRFB(prAdapter, prSwRfb);

        pucMemHandle += ALIGN_4(sizeof(SW_RFB_T));
    }

    ASSERT(prRxCtrl->rFreeSwRfbList.u4NumElem == CFG_RX_MAX_PKT_NUM);
    
	ASSERT((UINT_32) (pucMemHandle - prRxCtrl->pucRxCached) == prRxCtrl->u4RxCachedSize);

	
    RX_RESET_ALL_CNTS(prRxCtrl);

#if CFG_SDIO_RX_AGG
    prRxCtrl->pucRxCoalescingBufPtr = prAdapter->pucCoalescingBufCached;
    HAL_CFG_MAX_HIF_RX_LEN_NUM(prAdapter, CFG_SDIO_MAX_RX_AGG_NUM);
#else
    HAL_CFG_MAX_HIF_RX_LEN_NUM(prAdapter, 1);
#endif

#if CFG_HIF_STATISTICS
    prRxCtrl->u4TotalRxAccessNum = 0;
    prRxCtrl->u4TotalRxPacketNum = 0;
#endif

#if CFG_HIF_RX_STARVATION_WARNING
    prRxCtrl->u4QueuedCnt = 0;
    prRxCtrl->u4DequeuedCnt = 0;
#endif

    return;
} 


VOID nicRxUninitialize(IN P_ADAPTER_T prAdapter)
{
    P_RX_CTRL_T prRxCtrl;
	P_SW_RFB_T prSwRfb = (P_SW_RFB_T) NULL;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    nicRxFlush(prAdapter);

    do {
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
        QUEUE_REMOVE_HEAD(&prRxCtrl->rReceivedRfbList, prSwRfb, P_SW_RFB_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
		if (prSwRfb) {
            if (prSwRfb->pvPacket) {
                kalPacketFree(prAdapter->prGlueInfo, prSwRfb->pvPacket);
            }
            prSwRfb->pvPacket = NULL;
		} else {
            break;
        }
	} while (TRUE);

    do {
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
        QUEUE_REMOVE_HEAD(&prRxCtrl->rFreeSwRfbList, prSwRfb, P_SW_RFB_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
		if (prSwRfb) {
            if (prSwRfb->pvPacket) {
                kalPacketFree(prAdapter->prGlueInfo, prSwRfb->pvPacket);
            }
            prSwRfb->pvPacket = NULL;
		} else {
            break;
        }
	} while (TRUE);

    return;
} 


VOID nicRxFillRFB(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
    P_HW_MAC_RX_DESC_T prRxStatus;

    UINT_32 u4PktLen = 0;
	
    UINT_32 u4HeaderOffset;
    UINT_16 u2RxStatusOffset;

    DEBUGFUNC("nicRxFillRFB");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prRxStatus = prSwRfb->prRxStatus;
    ASSERT(prRxStatus);

	u4PktLen = (UINT_32) HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus);
	u4HeaderOffset = (UINT_32) (HAL_RX_STATUS_GET_HEADER_OFFSET(prRxStatus));
	

	
	
    u2RxStatusOffset = sizeof(HW_MAC_RX_DESC_T);
	prSwRfb->ucGroupVLD = (UINT_8) HAL_RX_STATUS_GET_GROUP_VLD(prRxStatus);
    if (prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_4)) {
		prSwRfb->prRxStatusGroup4 =
		    (P_HW_MAC_RX_STS_GROUP_4_T) ((P_UINT_8) prRxStatus + u2RxStatusOffset);
        u2RxStatusOffset += sizeof(HW_MAC_RX_STS_GROUP_4_T);

    }
    if (prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_1)) {
		prSwRfb->prRxStatusGroup1 =
		    (P_HW_MAC_RX_STS_GROUP_1_T) ((P_UINT_8) prRxStatus + u2RxStatusOffset);
        u2RxStatusOffset += sizeof(HW_MAC_RX_STS_GROUP_1_T);

    }
    if (prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_2)) {
		prSwRfb->prRxStatusGroup2 =
		    (P_HW_MAC_RX_STS_GROUP_2_T) ((P_UINT_8) prRxStatus + u2RxStatusOffset);
        u2RxStatusOffset += sizeof(HW_MAC_RX_STS_GROUP_2_T);

    }
    if (prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_3)) {
		prSwRfb->prRxStatusGroup3 =
		    (P_HW_MAC_RX_STS_GROUP_3_T) ((P_UINT_8) prRxStatus + u2RxStatusOffset);
        u2RxStatusOffset += sizeof(HW_MAC_RX_STS_GROUP_3_T);
    }

    prSwRfb->u2RxStatusOffst = u2RxStatusOffset;
	prSwRfb->pvHeader = (PUINT_8) prRxStatus + u2RxStatusOffset + u4HeaderOffset;
	prSwRfb->u2PacketLen = (UINT_16) (u4PktLen - (u2RxStatusOffset + u4HeaderOffset));
	prSwRfb->ucWlanIdx = (UINT_8) HAL_RX_STATUS_GET_WLAN_IDX(prRxStatus);
	prSwRfb->ucStaRecIdx =
	    secGetStaIdxByWlanIdx(prAdapter, (UINT_8) HAL_RX_STATUS_GET_WLAN_IDX(prRxStatus));
    prSwRfb->prStaRec = cnmGetStaRecByIndex(prAdapter, prSwRfb->ucStaRecIdx);

	
	

#if 0
	if (prHifRxHdr->ucReorder & HIF_RX_HDR_80211_HEADER_FORMAT) {
        prSwRfb->u4HifRxHdrFlag |= HIF_RX_HDR_FLAG_802_11_FORMAT;
        DBGLOG(RX, TRACE, ("HIF_RX_HDR_FLAG_802_11_FORMAT\n"));
    }

	if (prHifRxHdr->ucReorder & HIF_RX_HDR_DO_REORDER) {
        prSwRfb->u4HifRxHdrFlag |= HIF_RX_HDR_FLAG_DO_REORDERING;
        DBGLOG(RX, TRACE, ("HIF_RX_HDR_FLAG_DO_REORDERING\n"));

        
		if (prHifRxHdr->u2SeqNoTid & HIF_RX_HDR_BAR_FRAME) {
            prSwRfb->u4HifRxHdrFlag |= HIF_RX_HDR_FLAG_BAR_FRAME;
            DBGLOG(RX, TRACE, ("HIF_RX_HDR_FLAG_BAR_FRAME\n"));
        }

        prSwRfb->u2SSN = prHifRxHdr->u2SeqNoTid & HIF_RX_HDR_SEQ_NO_MASK;
		prSwRfb->ucTid = (UINT_8) ((prHifRxHdr->u2SeqNoTid & HIF_RX_HDR_TID_MASK)
                        >> HIF_RX_HDR_TID_OFFSET);
		DBGLOG(RX, TRACE, ("u2SSN = %d, ucTid = %d\n", prSwRfb->u2SSN, prSwRfb->ucTid));
    }

	if (prHifRxHdr->ucReorder & HIF_RX_HDR_WDS) {
        prSwRfb->u4HifRxHdrFlag |= HIF_RX_HDR_FLAG_AMP_WDS;
        DBGLOG(RX, TRACE, ("HIF_RX_HDR_FLAG_AMP_WDS\n"));
    }
#endif
}


#if CFG_TCP_IP_CHKSUM_OFFLOAD || CFG_TCP_IP_CHKSUM_OFFLOAD_NDIS_60
VOID
nicRxFillChksumStatus(IN P_ADAPTER_T prAdapter,
		      IN OUT P_SW_RFB_T prSwRfb, IN UINT_32 u4TcpUdpIpCksStatus)
{

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

	if (prAdapter->u4CSUMFlags != CSUM_NOT_SUPPORTED) {
		if (u4TcpUdpIpCksStatus & RX_CS_TYPE_IPv4) {	
            prSwRfb->aeCSUM[CSUM_TYPE_IPV6] = CSUM_RES_NONE;
			if (u4TcpUdpIpCksStatus & RX_CS_STATUS_IP) {	
                prSwRfb->aeCSUM[CSUM_TYPE_IPV4] = CSUM_RES_FAILED;
            } else {
                prSwRfb->aeCSUM[CSUM_TYPE_IPV4] = CSUM_RES_SUCCESS;
            }

			if (u4TcpUdpIpCksStatus & RX_CS_TYPE_TCP) {	
                prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_NONE;
				if (u4TcpUdpIpCksStatus & RX_CS_STATUS_TCP) {	
                    prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_FAILED;
                } else {
                    prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_SUCCESS;
                }
			} else if (u4TcpUdpIpCksStatus & RX_CS_TYPE_UDP) {	
                prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_NONE;
				if (u4TcpUdpIpCksStatus & RX_CS_STATUS_UDP) {	
                    prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_FAILED;
                } else {
                    prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_SUCCESS;
                }
			} else {
                prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_NONE;
                prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_NONE;
            }
		} else if (u4TcpUdpIpCksStatus & RX_CS_TYPE_IPv6) {	
            prSwRfb->aeCSUM[CSUM_TYPE_IPV4] = CSUM_RES_NONE;
            prSwRfb->aeCSUM[CSUM_TYPE_IPV6] = CSUM_RES_SUCCESS;

			if (u4TcpUdpIpCksStatus & RX_CS_TYPE_TCP) {	
                prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_NONE;
				if (u4TcpUdpIpCksStatus & RX_CS_STATUS_TCP) {	
                    prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_FAILED;
                } else {
                    prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_SUCCESS;
                }
			} else if (u4TcpUdpIpCksStatus & RX_CS_TYPE_UDP) {	
                prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_NONE;
				if (u4TcpUdpIpCksStatus & RX_CS_STATUS_UDP) {	
                    prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_FAILED;
                } else {
                    prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_SUCCESS;
                }
			} else {
                prSwRfb->aeCSUM[CSUM_TYPE_UDP] = CSUM_RES_NONE;
                prSwRfb->aeCSUM[CSUM_TYPE_TCP] = CSUM_RES_NONE;
            }
		} else {
            prSwRfb->aeCSUM[CSUM_TYPE_IPV4] = CSUM_RES_NONE;
            prSwRfb->aeCSUM[CSUM_TYPE_IPV6] = CSUM_RES_NONE;
        }
    }

}
#endif 


P_SW_RFB_T
incRxDefragMPDU(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSWRfb, OUT P_QUE_T prReturnedQue)
{

	P_SW_RFB_T prOutputSwRfb = (P_SW_RFB_T) NULL;
#if 1
    P_RX_CTRL_T prRxCtrl;
    P_FRAG_INFO_T prFragInfo;
    UINT_32 i = 0, j;
    UINT_16 u2SeqCtrl, u2FrameCtrl;
    UINT_8 ucFragNum;
    BOOLEAN fgFirst = FALSE;
    BOOLEAN fgLast = FALSE;
    OS_SYSTIME rCurrentTime;
    P_WLAN_MAC_HEADER_T prWlanHeader = NULL;
    P_HW_MAC_RX_DESC_T prRxStatus = NULL;
    P_HW_MAC_RX_STS_GROUP_4_T prRxStatusGroup4 = NULL;

    DEBUGFUNC("nicRx: rxmDefragMPDU\n");

    ASSERT(prSWRfb);

    prRxCtrl = &prAdapter->rRxCtrl;

    prRxStatus = prSWRfb->prRxStatus;
    ASSERT(prRxStatus);

    if (HAL_RX_STATUS_IS_HEADER_TRAN(prRxStatus) == FALSE) {
		prWlanHeader = (P_WLAN_MAC_HEADER_T) prSWRfb->pvHeader;
        prSWRfb->u2SequenceControl = prWlanHeader->u2SeqCtrl;
        u2FrameCtrl = prWlanHeader->u2FrameCtrl;
	} else {
        prRxStatusGroup4 = prSWRfb->prRxStatusGroup4;
        prSWRfb->u2SequenceControl = HAL_RX_STATUS_GET_SEQFrag_NUM(prRxStatusGroup4);
        u2FrameCtrl = HAL_RX_STATUS_GET_FRAME_CTL_FIELD(prRxStatusGroup4);
    }
    u2SeqCtrl = prSWRfb->u2SequenceControl;
	ucFragNum = (UINT_8) (u2SeqCtrl & MASK_SC_FRAG_NUM);
    prSWRfb->u2FrameCtrl = u2FrameCtrl; 

    if (!(u2FrameCtrl & MASK_FC_MORE_FRAG)) {
        
        if (ucFragNum) {
			DBGLOG(RX, LOUD,
			       ("FC %04x M %04x SQ %04x\n", u2FrameCtrl,
				(u2FrameCtrl & MASK_FC_MORE_FRAG), u2SeqCtrl));
            fgLast = TRUE;
        }
        
        else {
            return prSWRfb;
        }
    }
    
    else {
        if (ucFragNum == 0) {
			DBGLOG(RX, LOUD,
			       ("FC %04x M %04x SQ %04x\n", u2FrameCtrl,
				(u2FrameCtrl & MASK_FC_MORE_FRAG), u2SeqCtrl));
            fgFirst = TRUE;
		} else {
			DBGLOG(RX, LOUD,
			       ("FC %04x M %04x SQ %04x\n", u2FrameCtrl,
				(u2FrameCtrl & MASK_FC_MORE_FRAG), u2SeqCtrl));
        }
    }

    GET_CURRENT_SYSTIME(&rCurrentTime);

    for (j = 0; j < MAX_NUM_CONCURRENT_FRAGMENTED_MSDUS; j++) {
         prFragInfo = &prSWRfb->prStaRec->rFragInfo[j];
         if (prFragInfo->pr1stFrag) {
            if (CHECK_FOR_EXPIRATION(rCurrentTime, prFragInfo->rReceiveLifetimeLimit)) {

				
                prFragInfo->pr1stFrag->eDst = RX_PKT_DESTINATION_NULL;
				QUEUE_INSERT_TAIL(prReturnedQue,
						  (P_QUE_ENTRY_T) prFragInfo->pr1stFrag);

				prFragInfo->pr1stFrag = (P_SW_RFB_T) NULL;
            }
        }
    }


    for (i = 0; i < MAX_NUM_CONCURRENT_FRAGMENTED_MSDUS; i++) {

        prFragInfo = &prSWRfb->prStaRec->rFragInfo[i];

		if (fgFirst) {	

			if (prFragInfo->pr1stFrag == (P_SW_RFB_T) NULL) {	
                break;
            }
		} else {	

			if (prFragInfo->pr1stFrag == (P_SW_RFB_T) NULL) {
                continue;
            }

            if (RXM_IS_QOS_DATA_FRAME(u2FrameCtrl)) {
                if (RXM_IS_QOS_DATA_FRAME(prFragInfo->pr1stFrag->u2FrameCtrl)) {
                    if (u2SeqCtrl == prFragInfo->u2NextFragSeqCtrl) {
                        break;
                    }
                }
			} else {
                if (!RXM_IS_QOS_DATA_FRAME(prFragInfo->pr1stFrag->u2FrameCtrl)) {
                    if (u2SeqCtrl == prFragInfo->u2NextFragSeqCtrl) {
                        break;
                    }
                }
            }
        }
    }

    if (i >= MAX_NUM_CONCURRENT_FRAGMENTED_MSDUS) {

        nicRxReturnRFB(prAdapter, prSWRfb);

		return (P_SW_RFB_T) NULL;
    }

    ASSERT(prFragInfo);

	
    prSWRfb->u2HeaderLen = HAL_RX_STATUS_GET_HEADER_LEN(prRxStatus);
	prSWRfb->pucPayload =
	    (PUINT_8) (((ULONG)prSWRfb->pvHeader) + prSWRfb->u2HeaderLen);
	prSWRfb->u2PayloadLength =
	    (UINT_16) (HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus) -
		       ((ULONG) prSWRfb->pucPayload - (ULONG) prRxStatus));

    if (fgFirst) {
        DBGLOG(RX, LOUD, ("rxDefragMPDU first\n"));

        SET_EXPIRATION_TIME(prFragInfo->rReceiveLifetimeLimit,
                            TU_TO_SYSTIME(DOT11_RECEIVE_LIFETIME_TU_DEFAULT));

        prFragInfo->pr1stFrag = prSWRfb;
        
        prFragInfo->pucNextFragStart =
            (PUINT_8) prSWRfb->pucRecvBuff + HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus);

        prFragInfo->u2NextFragSeqCtrl = u2SeqCtrl + 1;
        DBGLOG(RX, LOUD, ("First: nextFragmentSeqCtrl = %04x, u2SeqCtrl = %04x\n",
            prFragInfo->u2NextFragSeqCtrl, u2SeqCtrl));

		
		
	} else {
        prFragInfo->pr1stFrag->prRxStatus->u2RxByteCount += prSWRfb->u2PayloadLength;        

        if (prFragInfo->pr1stFrag->prRxStatus->u2RxByteCount > CFG_RX_MAX_PKT_SIZE) {

            prFragInfo->pr1stFrag->eDst = RX_PKT_DESTINATION_NULL;
			QUEUE_INSERT_TAIL(prReturnedQue, (P_QUE_ENTRY_T) prFragInfo->pr1stFrag);

			prFragInfo->pr1stFrag = (P_SW_RFB_T) NULL;

            nicRxReturnRFB(prAdapter, prSWRfb);
		} else {
            kalMemCopy(prFragInfo->pucNextFragStart,
                prSWRfb->pucPayload, prSWRfb->u2PayloadLength);            
			
            prFragInfo->pr1stFrag->u2PacketLen += prSWRfb->u2PayloadLength;  
            prFragInfo->pr1stFrag->u2PayloadLength += prSWRfb->u2PayloadLength;
                
			if (fgLast) {	
                DBGLOG(RX, LOUD, ("Defrag: finished\n"));

                prOutputSwRfb = prFragInfo->pr1stFrag;

				prFragInfo->pr1stFrag = (P_SW_RFB_T) NULL;
			} else {
                DBGLOG(RX, LOUD, ("Defrag: mid fraged\n"));

                prFragInfo->pucNextFragStart += prSWRfb->u2PayloadLength;                

                prFragInfo->u2NextFragSeqCtrl++;
            }

            nicRxReturnRFB(prAdapter, prSWRfb);
        }
    }

	
	
	
#endif
    return prOutputSwRfb;
} 


BOOLEAN nicRxIsDuplicateFrame(IN OUT P_SW_RFB_T prSwRfb)
{

    UINT_32 u4SeqCtrlCacheIdx;
    UINT_16 u2SequenceControl, u2FrameCtrl;
    BOOLEAN fgIsDuplicate = FALSE, fgIsAmsduSubframe = FALSE;
    P_WLAN_MAC_HEADER_T prWlanHeader = NULL;
    P_HW_MAC_RX_DESC_T prRxStatus = NULL;
    P_HW_MAC_RX_STS_GROUP_4_T prRxStatusGroup4 = NULL;

    DEBUGFUNC("nicRx: Enter rxmIsDuplicateFrame()\n");

    ASSERT(prSwRfb);

	if (!prSwRfb->prStaRec) {
        return FALSE;
    }

    prRxStatus = prSwRfb->prRxStatus;
    ASSERT(prRxStatus);

    fgIsAmsduSubframe = HAL_RX_STATUS_GET_PAYLOAD_FORMAT(prRxStatus);
    if (HAL_RX_STATUS_IS_HEADER_TRAN(prRxStatus) == FALSE) {
		prWlanHeader = (P_WLAN_MAC_HEADER_T) prSwRfb->pvHeader;
        u2SequenceControl = prWlanHeader->u2SeqCtrl;
        u2FrameCtrl = prWlanHeader->u2FrameCtrl;
	} else {
        prRxStatusGroup4 = prSwRfb->prRxStatusGroup4;
        u2SequenceControl = HAL_RX_STATUS_GET_SEQFrag_NUM(prRxStatusGroup4);
        u2FrameCtrl = HAL_RX_STATUS_GET_FRAME_CTL_FIELD(prRxStatusGroup4);
    }
    prSwRfb->u2SequenceControl = u2SequenceControl;


    
	if (RXM_IS_QOS_DATA_FRAME(u2FrameCtrl)) {	
		if (prSwRfb->prStaRec->aprRxReorderParamRefTbl[prSwRfb->ucTid]) {


			DBGLOG(RX, LOUD, ("RX: SC=0x%X (BA Entry present)\n", u2SequenceControl));

            
            prSwRfb->prStaRec->au2CachedSeqCtrl[prSwRfb->ucTid] = u2SequenceControl;


			
#if 0
			DBGLOG(RXM, LOUD, ("RXM: SC= 0x%X (Cache[%d] updated) with BA\n",
                u2SequenceControl, prSwRfb->ucTID));

			if (g_prMqm->arRxBaTable[prSwRfb->prStaRec->aucRxBaTable[prSwRfb->ucTID]].
			    ucStatus == BA_ENTRY_STATUS_DELETING) {
				DBGLOG(RXM, LOUD,
				       ("RXM: SC= 0x%X (Cache[%d] updated) with DELETING BA ****************\n",
                    u2SequenceControl, prSwRfb->ucTID));
            }
#endif

            
            return FALSE; 
		} else {

			if (prSwRfb->prStaRec->
			    ucDesiredPhyTypeSet & (PHY_TYPE_BIT_HT | PHY_TYPE_BIT_VHT)) {
                u4SeqCtrlCacheIdx = prSwRfb->ucTid;
			} else {
				if (prSwRfb->ucTid < 8) {	
                    u4SeqCtrlCacheIdx = aucTid2ACI[prSwRfb->ucTid];
				} else {
					DBGLOG(RX, WARN,
					       ("RXM: (Warning) Unkown QoS Data with TID=%d\n",
                        prSwRfb->ucTid));

                    return TRUE; 
                }
            }

        }
    }
    
	else {
        u4SeqCtrlCacheIdx = TID_NUM;
    }


    
	if (u2FrameCtrl & MASK_FC_RETRY) {
		if (u2SequenceControl != prSwRfb->prStaRec->au2CachedSeqCtrl[u4SeqCtrlCacheIdx]) {
            prSwRfb->prStaRec->au2CachedSeqCtrl[u4SeqCtrlCacheIdx] = u2SequenceControl;
			if (fgIsAmsduSubframe == RX_PAYLOAD_FORMAT_FIRST_SUB_AMSDU) {
				prSwRfb->prStaRec->afgIsIgnoreAmsduDuplicate[u4SeqCtrlCacheIdx] =
				    TRUE;
            }
			DBGLOG(RX, LOUD, ("RXM: SC= 0x%X (Cache[%lu] updated)\n",
                u2SequenceControl, u4SeqCtrlCacheIdx));
		} else {
            
            if (prSwRfb->prStaRec->afgIsIgnoreAmsduDuplicate[u4SeqCtrlCacheIdx]) {
                if (fgIsAmsduSubframe == RX_PAYLOAD_FORMAT_LAST_SUB_AMSDU) {
					prSwRfb->prStaRec->
					    afgIsIgnoreAmsduDuplicate[u4SeqCtrlCacheIdx] = FALSE;
                }
			} else {
                fgIsDuplicate = TRUE;
				DBGLOG(RX, LOUD, ("RXM: SC= 0x%X (Cache[%lu] duplicate)\n",
                    u2SequenceControl, u4SeqCtrlCacheIdx));
            }
        }
    }

    
	else {

        prSwRfb->prStaRec->au2CachedSeqCtrl[u4SeqCtrlCacheIdx] = u2SequenceControl;
        prSwRfb->prStaRec->afgIsIgnoreAmsduDuplicate[u4SeqCtrlCacheIdx] = FALSE;

		DBGLOG(RX, LOUD, ("RXM: SC= 0x%X (Cache[%lu] updated)\n",
            u2SequenceControl, u4SeqCtrlCacheIdx));
    }

    return fgIsDuplicate;
}

VOID nicRxProcessPktWithoutReorder(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    P_RX_CTRL_T prRxCtrl;
    P_TX_CTRL_T prTxCtrl;
    BOOL fgIsRetained = FALSE;
    UINT_32 u4CurrentRxBufferCount;
	
#if CFG_SUPPORT_MULTITHREAD
    KAL_SPIN_LOCK_DECLARATION();
#endif    
    DEBUGFUNC("nicRxProcessPktWithoutReorder");
	

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    prTxCtrl = &prAdapter->rTxCtrl;
    ASSERT(prTxCtrl);

    u4CurrentRxBufferCount = prRxCtrl->rFreeSwRfbList.u4NumElem;

#if defined(LINUX)
    fgIsRetained = FALSE;
#else
    fgIsRetained = (((u4CurrentRxBufferCount +
                    qmGetRxReorderQueuedBufferCount(prAdapter) +
                    prTxCtrl->i4PendingFwdFrameCount) < CFG_RX_RETAINED_PKT_THRESHOLD) ?
                           TRUE : FALSE);
#endif

	
#if CFG_ENABLE_PER_STA_STATISTICS
	if (prSwRfb->prStaRec && (prAdapter->rWifiVar.rWfdConfigureSettings.ucWfdEnable > 0)) {
        prSwRfb->prStaRec->u4TotalRxPktsNumber++;
    }
#endif
    if (kalProcessRxPacket(prAdapter->prGlueInfo,
                         prSwRfb->pvPacket,
                         prSwRfb->pvHeader,
			       (UINT_32) prSwRfb->u2PacketLen,
			       fgIsRetained, prSwRfb->aeCSUM) != WLAN_STATUS_SUCCESS) {
        DBGLOG(RX, ERROR, ("kalProcessRxPacket return value != WLAN_STATUS_SUCCESS\n"));
        ASSERT(0);

        nicRxReturnRFB(prAdapter, prSwRfb);
        return;
	} else {
#if !CFG_SUPPORT_MULTITHREAD 
        prRxCtrl->apvIndPacket[prRxCtrl->ucNumIndPacket] = prSwRfb->pvPacket;
        prRxCtrl->ucNumIndPacket++;
#endif        
    }

#if CFG_SUPPORT_MULTITHREAD
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE);
	QUEUE_INSERT_TAIL(&(prAdapter->rRxQueue),
			  (P_QUE_ENTRY_T) GLUE_GET_PKT_QUEUE_ENTRY(prSwRfb->pvPacket));
        prRxCtrl->ucNumIndPacket++;
#endif    

    if (fgIsRetained) {
        prRxCtrl->apvRetainedPacket[prRxCtrl->ucNumRetainedPacket] = prSwRfb->pvPacket;
        prRxCtrl->ucNumRetainedPacket++;
            
        nicRxSetupRFB(prAdapter, prSwRfb);
        nicRxReturnRFB(prAdapter, prSwRfb);
	} else {
        prSwRfb->pvPacket = NULL;
        nicRxReturnRFB(prAdapter, prSwRfb);
    }

#if CFG_SUPPORT_MULTITHREAD
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE);
#endif      
}


VOID nicRxProcessForwardPkt(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    P_MSDU_INFO_T prMsduInfo, prRetMsduInfoList;
    P_TX_CTRL_T prTxCtrl;
    P_RX_CTRL_T prRxCtrl;
    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxProcessForwardPkt");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prTxCtrl = &prAdapter->rTxCtrl;
    prRxCtrl = &prAdapter->rRxCtrl;

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);
    QUEUE_REMOVE_HEAD(&prTxCtrl->rFreeMsduInfoList, prMsduInfo, P_MSDU_INFO_T);
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_TX_MSDU_INFO_LIST);

	if (prMsduInfo &&
       kalProcessRxPacket(prAdapter->prGlueInfo,
                prSwRfb->pvPacket,
                prSwRfb->pvHeader,
			       (UINT_32) prSwRfb->u2PacketLen,
			       prRxCtrl->rFreeSwRfbList.u4NumElem <
			       CFG_RX_RETAINED_PKT_THRESHOLD ? TRUE : FALSE,
                prSwRfb->aeCSUM) == WLAN_STATUS_SUCCESS) {

        prMsduInfo->eSrc = TX_PACKET_FORWARDING;

		
		wlanProcessTxFrame(prAdapter, (P_NATIVE_PACKET) (prSwRfb->pvPacket));
		
		nicTxFillMsduInfo(prAdapter, prMsduInfo, (P_NATIVE_PACKET) (prSwRfb->pvPacket));

        prMsduInfo->ucBssIndex = secGetBssIdxByWlanIdx(prAdapter, prSwRfb->ucWlanIdx);

		
        prSwRfb->pvPacket = NULL;
        nicRxReturnRFB(prAdapter, prSwRfb);

		
        GLUE_INC_REF_CNT(prTxCtrl->i4PendingFwdFrameCount);

		
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);
        prRetMsduInfoList = qmEnqueueTxPackets(prAdapter, prMsduInfo);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_QM_TX_QUEUE);

		if (prRetMsduInfoList != NULL) {	
            nicTxFreeMsduInfoPacket(prAdapter, prRetMsduInfoList);
            nicTxReturnMsduInfo(prAdapter, prRetMsduInfoList);
        }
        
		if (prTxCtrl->i4PendingFwdFrameCount > 0) {
            kalSetEvent(prAdapter->prGlueInfo);
        }
	} else {		
        DBGLOG(QM, INFO, ("No Tx MSDU_INFO for forwarding frames\n"));
        nicRxReturnRFB(prAdapter, prSwRfb);
    }

    return;
}


VOID nicRxProcessGOBroadcastPkt(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    P_SW_RFB_T prSwRfbDuplicated;
    P_TX_CTRL_T prTxCtrl;
    P_RX_CTRL_T prRxCtrl;
    P_HW_MAC_RX_DESC_T prRxStatus;

    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxProcessGOBroadcastPkt");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prTxCtrl = &prAdapter->rTxCtrl;
    prRxCtrl = &prAdapter->rRxCtrl;

    prRxStatus = prSwRfb->prRxStatus;
    ASSERT(prRxStatus);

    ASSERT(CFG_NUM_OF_QM_RX_PKT_NUM >= 16);

	if (prRxCtrl->rFreeSwRfbList.u4NumElem
	    >= (CFG_RX_MAX_PKT_NUM - (CFG_NUM_OF_QM_RX_PKT_NUM - 16 ))) {

        
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
        QUEUE_REMOVE_HEAD(&prRxCtrl->rFreeSwRfbList, prSwRfbDuplicated, P_SW_RFB_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

		if (prSwRfbDuplicated) {
            kalMemCopy(prSwRfbDuplicated->pucRecvBuff,
                    prSwRfb->pucRecvBuff,
				   ALIGN_4(prRxStatus->u2RxByteCount + HIF_RX_HW_APPENDED_LEN));

            prSwRfbDuplicated->ucPacketType = RX_PKT_TYPE_RX_DATA;
            prSwRfbDuplicated->ucStaRecIdx = prSwRfb->ucStaRecIdx;
            nicRxFillRFB(prAdapter, prSwRfbDuplicated);

            
            prSwRfbDuplicated->eDst = RX_PKT_DESTINATION_FORWARD;

            
            nicRxProcessForwardPkt(prAdapter, prSwRfbDuplicated);
        }
	} else {
		DBGLOG(RX, WARN,
		       ("Stop to forward BMC packet due to less free Sw Rfb %lu\n",
			prRxCtrl->rFreeSwRfbList.u4NumElem));
    }

    
    prSwRfb->eDst = RX_PKT_DESTINATION_HOST;
    nicRxProcessPktWithoutReorder(prAdapter, prSwRfb);

    return;
}

#if CFG_SUPPORT_SNIFFER
VOID nicRxFillRadiotapMCS(IN OUT P_MONITOR_RADIOTAP_T prMonitorRadiotap, IN P_HW_MAC_RX_STS_GROUP_3_T prRxStatusGroup3)
{
	UINT_8 ucFrMode;
	UINT_8 ucShortGI;
    UINT_8 ucRxMode;
	UINT_8 ucLDPC;
    UINT_8 ucSTBC;
	UINT_8 ucNess;

    ucFrMode = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_FR_MODE_MASK) >> RX_VT_FR_MODE_OFFSET); 
    ucShortGI = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_SHORT_GI) ? 1 : 0; 
    ucRxMode = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_RX_MODE_MASK) >> RX_VT_RX_MODE_OFFSET);
    ucLDPC = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_LDPC) ? 1 : 0; 
    ucSTBC = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_STBC_MASK) >> RX_VT_STBC_OFFSET); 
    ucNess = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_NESS_MASK) >> RX_VT_NESS_OFFSET); 
   
    prMonitorRadiotap->ucMcsKnown = ( BITS(0,6) |
                                    (((ucNess & BIT(1)) >> 1) << 7)
                                    );
    
    prMonitorRadiotap->ucMcsFlags = ( (ucFrMode) |
                                    (ucShortGI << 2) |
                                    ((ucRxMode & BIT(0)) << 3) |
                                    (ucLDPC << 4) |
                                    (ucSTBC << 5) |
                                    ((ucNess & BIT(0)) << 7)
                                    );  
    
    prMonitorRadiotap->ucMcsMcs = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_RX_RATE_MASK);  
}

VOID nicRxFillRadiotapVHT(IN OUT P_MONITOR_RADIOTAP_T prMonitorRadiotap, IN P_HW_MAC_RX_STS_GROUP_3_T prRxStatusGroup3)
{
	UINT_8 ucSTBC;
	UINT_8 ucTxopPsNotAllow;
	UINT_8 ucShortGI;
	UINT_8 ucNsym;
	UINT_8 ucLDPC;
	UINT_8 ucBeamFormed;
	UINT_8 ucFrMode;
	UINT_8 ucNsts;
	UINT_8 ucMcs;

    prMonitorRadiotap->u2VhtKnown = BITS(0,8);

    ucSTBC = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_STBC_MASK) >> RX_VT_STBC_OFFSET); 
    ucTxopPsNotAllow = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_TXOP_PS_NOT_ALLOWED) ? 1 : 0; 
    ucShortGI = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_SHORT_GI) ? 1 : 0; 
    ucNsym = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_SHORT_GI_NSYM) ? 1 : 0; 
    ucLDPC = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_LDPC) ? 1 : 0; 
    ucBeamFormed = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_BEAMFORMED) ? 1 : 0; 
    prMonitorRadiotap->ucVhtFlags = ( (ucSTBC) | 
                                    (ucTxopPsNotAllow << 1) | 
                                    (ucShortGI << 2) |
                                    (ucNsym << 3) |
                                    (ucLDPC << 4) | 
                                    (ucBeamFormed << 5)
                                    );
                                
    ucFrMode = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_FR_MODE_MASK) >> RX_VT_FR_MODE_OFFSET); 
    switch (ucFrMode) {
    case RX_VT_FR_MODE_20:
        prMonitorRadiotap->ucVhtBandwidth = 0;
        break;
    case RX_VT_FR_MODE_40:
        prMonitorRadiotap->ucVhtBandwidth = 1;
        break;
    case RX_VT_FR_MODE_80:
        prMonitorRadiotap->ucVhtBandwidth = 4;
        break;
    case RX_VT_FR_MODE_160:
        prMonitorRadiotap->ucVhtBandwidth = 11;
        break;
    default:
        prMonitorRadiotap->ucVhtBandwidth = 0;
    }
    
    
    ucNsts = (((prRxStatusGroup3)->u4RxVector[1] & RX_VT_NSTS_MASK) >> RX_VT_NSTS_OFFSET) + 1; 
    
    
    ucMcs = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_RX_RATE_AC_MASK);  
    
    prMonitorRadiotap->aucVhtMcsNss[0] = ((ucMcs << 4) | (ucNsts - ucSTBC)); 
    
    prMonitorRadiotap->ucVhtCoding = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_CODING_MASK) >> RX_VT_CODING_OFFSET); 
    
    prMonitorRadiotap->ucVhtGroupId = (((((prRxStatusGroup3)->u4RxVector[1] & RX_VT_GROUPID_1_MASK) >> RX_VT_GROUPID_1_OFFSET) << 2) |
                                    (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_GROUPID_0_MASK) >> RX_VT_GROUPID_0_OFFSET)); 
    
    prMonitorRadiotap->u2VhtPartialAid = ((((prRxStatusGroup3)->u4RxVector[2] & RX_VT_AID_1_MASK) << 4) |
                                    (((prRxStatusGroup3)->u4RxVector[1] & RX_VT_AID_0_MASK) >> RX_VT_AID_0_OFFSET)); 

}

VOID nicRxProcessMonitorPacket(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
	struct sk_buff *prSkb = NULL;
	P_RX_CTRL_T prRxCtrl;
	P_HW_MAC_RX_DESC_T prRxStatus;
	P_HW_MAC_RX_STS_GROUP_2_T prRxStatusGroup2;
	P_HW_MAC_RX_STS_GROUP_3_T prRxStatusGroup3;
	MONITOR_RADIOTAP_T rMonitorRadiotap;
	RADIOTAP_FIELD_VENDOR_T rRadiotapFieldVendor;
	PUINT_8 prVendorNsOffset;
	UINT_32 u4VendorNsLen;
	UINT_32 u4RadiotapLen;
	UINT_32 u4ItPresent;
	UINT_8 aucMtkOui[] = VENDOR_OUI_MTK;
	UINT_8 ucRxRate;
	UINT_8 ucRxMode;
	UINT_8 ucChanNum;
	UINT_8 ucMcs;
	UINT_8 ucFrMode;
	UINT_8 ucShortGI;

#if CFG_SUPPORT_MULTITHREAD
    KAL_SPIN_LOCK_DECLARATION();
#endif 

	DEBUGFUNC("nicRxProcessMonitorPacket");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prRxCtrl = &prAdapter->rRxCtrl;	
														
	nicRxFillRFB(prAdapter, prSwRfb);
	
	
	if (((prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_2)) == 0) ||
		((prSwRfb->ucGroupVLD & BIT(RX_GROUP_VLD_3)) == 0)) {
		nicRxReturnRFB(prAdapter, prSwRfb);
		return;
	}
	
	prRxStatus = prSwRfb->prRxStatus;				
	prRxStatusGroup2 = prSwRfb->prRxStatusGroup2;					
	prRxStatusGroup3 = prSwRfb->prRxStatusGroup3;

    
    u4VendorNsLen = sizeof(RADIOTAP_FIELD_VENDOR_T);
    rRadiotapFieldVendor.aucOUI[0] = aucMtkOui[0];
    rRadiotapFieldVendor.aucOUI[1] = aucMtkOui[1];
    rRadiotapFieldVendor.aucOUI[2] = aucMtkOui[2];
    rRadiotapFieldVendor.ucSubNamespace = 0;
    rRadiotapFieldVendor.u2DataLen = u4VendorNsLen - 6;
    rRadiotapFieldVendor.ucData = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_FR_MODE_MASK) >> RX_VT_FR_MODE_OFFSET); 
        
	ucRxMode = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_RX_MODE_MASK) >> RX_VT_RX_MODE_OFFSET);
    
	if (ucRxMode == RX_VT_VHT_MODE) {
		u4RadiotapLen = RADIOTAP_LEN_VHT;
		u4ItPresent = RADIOTAP_FIELDS_VHT;
	}
	else if ((ucRxMode == RX_VT_MIXED_MODE) || (ucRxMode == RX_VT_GREEN_MODE)) {
		u4RadiotapLen = RADIOTAP_LEN_HT;
		u4ItPresent = RADIOTAP_FIELDS_HT;
	}
	else {
		u4RadiotapLen = RADIOTAP_LEN_LEGACY;
		u4ItPresent = RADIOTAP_FIELDS_LEGACY;
	}
	
	
    prVendorNsOffset = (PUINT_8)&rMonitorRadiotap + u4RadiotapLen;
    u4RadiotapLen += u4VendorNsLen;
	kalMemSet(&rMonitorRadiotap, 0, sizeof(MONITOR_RADIOTAP_T));
    kalMemCopy(prVendorNsOffset, (PUINT_8)&rRadiotapFieldVendor, u4VendorNsLen);
	rMonitorRadiotap.u2ItLen = cpu_to_le16(u4RadiotapLen);
	rMonitorRadiotap.u4ItPresent = u4ItPresent;
									
	
	rMonitorRadiotap.u8MacTime = (prRxStatusGroup2->u4Timestamp);

	
	if (HAL_RX_STATUS_IS_FRAG(prRxStatus) == TRUE) {
		rMonitorRadiotap.ucFlags |= BIT(3);
	}
					
	if (HAL_RX_STATUS_IS_FCS_ERROR(prRxStatus) == TRUE) {
		rMonitorRadiotap.ucFlags |= BIT(6);
	}

		
	if ((ucRxMode == RX_VT_LEGACY_CCK) || (ucRxMode == RX_VT_LEGACY_OFDM)) {
		
		ucRxRate = ((prRxStatusGroup3)->u4RxVector[0] & BITS(0,3));
		rMonitorRadiotap.ucRate = aucHwRate2PhyRate[ucRxRate];
	}
	else {
		ucMcs = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_RX_RATE_AC_MASK);
		ucFrMode = (((prRxStatusGroup3)->u4RxVector[0] & RX_VT_FR_MODE_MASK) >> RX_VT_FR_MODE_OFFSET); 
		ucShortGI = ((prRxStatusGroup3)->u4RxVector[0] & RX_VT_SHORT_GI) ? 1 : 0; 

		
		if (arMcsRate2PhyRate[ucMcs].u4PhyRate[ucFrMode][ucShortGI] > 1275)
			rMonitorRadiotap.ucRate = 0xFF;
		else
			rMonitorRadiotap.ucRate = arMcsRate2PhyRate[ucMcs].u4PhyRate[ucFrMode][ucShortGI] / 5;
	}
					
	
	if (ucRxMode == RX_VT_LEGACY_CCK) {
		rMonitorRadiotap.u2ChFlags |= BIT(5);
	}
	else { 
		rMonitorRadiotap.u2ChFlags |= BIT(6);
	}
	
	ucChanNum = HAL_RX_STATUS_GET_CHNL_NUM(prRxStatus);
	if (HAL_RX_STATUS_GET_RF_BAND(prRxStatus) == BAND_2G4) {
		rMonitorRadiotap.u2ChFlags |= BIT(7);
		rMonitorRadiotap.u2ChFrequency = (ucChanNum * 5 + 2407);
	}
	else { 
		rMonitorRadiotap.u2ChFlags |= BIT(8);
		rMonitorRadiotap.u2ChFrequency = (ucChanNum * 5 + 5000);
	}
					
	
	rMonitorRadiotap.ucAntennaSignal = (((prRxStatusGroup3)->u4RxVector[3] & RX_VT_IB_RSSI_MASK));
					
	
	rMonitorRadiotap.ucAntennaNoise = ((((prRxStatusGroup3)->u4RxVector[5] & RX_VT_NF0_MASK) >> 1) + 128);
					
	
	rMonitorRadiotap.ucAntenna = ((prRxStatusGroup3)->u4RxVector[2] & RX_VT_SEL_ANT) ? 1 : 0;

	
	if ((u4ItPresent & RADIOTAP_FIELD_MCS)) {
        nicRxFillRadiotapMCS(&rMonitorRadiotap, prRxStatusGroup3);
	}
	
	
	if (HAL_RX_STATUS_IS_AMPDU_SUB_FRAME(prRxStatus)) {
		if (HAL_RX_STATUS_GET_RXV_SEQ_NO(prRxStatus)) {
			++prRxCtrl->u4AmpduRefNum;
		}
		rMonitorRadiotap.u4AmpduRefNum = prRxCtrl->u4AmpduRefNum;
	} 

	
	if ((u4ItPresent & RADIOTAP_FIELD_VHT)) {
        nicRxFillRadiotapVHT(&rMonitorRadiotap, prRxStatusGroup3);
	}
						
	prSwRfb->pvHeader -= u4RadiotapLen;
	kalMemCopy(prSwRfb->pvHeader, &rMonitorRadiotap, u4RadiotapLen);
   
    prSkb = (struct sk_buff *)(prSwRfb->pvPacket);  
    prSkb->data = (unsigned char *)(prSwRfb->pvHeader);
    
    skb_reset_tail_pointer(prSkb);
    skb_trim(prSkb, 0);
    skb_put(prSkb, (u4RadiotapLen + prSwRfb->u2PacketLen));

#if CFG_SUPPORT_MULTITHREAD
    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE);
    QUEUE_INSERT_TAIL(&(prAdapter->rRxQueue), (P_QUE_ENTRY_T) GLUE_GET_PKT_QUEUE_ENTRY(prSwRfb->pvPacket));
    prRxCtrl->ucNumIndPacket++;
#else
    prRxCtrl->apvIndPacket[prRxCtrl->ucNumIndPacket] = prSwRfb->pvPacket;   
    prRxCtrl->ucNumIndPacket++;
#endif

    prSwRfb->pvPacket = NULL;
    nicRxReturnRFB(prAdapter, prSwRfb);
    
#if CFG_SUPPORT_MULTITHREAD
    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_TO_OS_QUE);
#endif 
}
#endif

VOID nicRxProcessDataPacket(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
    P_RX_CTRL_T prRxCtrl;
    P_SW_RFB_T prRetSwRfb, prNextSwRfb;
    P_HW_MAC_RX_DESC_T prRxStatus;
    BOOLEAN fgDrop;

    DEBUGFUNC("nicRxProcessDataPacket");
	

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    fgDrop = FALSE;

    prRxStatus = prSwRfb->prRxStatus;
    prRxCtrl = &prAdapter->rRxCtrl;

	
    prSwRfb->fgDataFrame = TRUE;
    prSwRfb->fgFragFrame = FALSE;
    prSwRfb->fgReorderBuffer = FALSE;

	
    if (HAL_RX_STATUS_IS_CIPHER_MISMATCH(prRxStatus)) {
		
		prRxStatus->u2StatusFlag =
		    prRxStatus->u2StatusFlag & !RX_STATUS_FLAG_CIPHER_MISMATCH;
    }
	
    if (prRxStatus->u2StatusFlag == RXS_DW2_AMPDU_nERR_VALUE) {
        prSwRfb->fgReorderBuffer = TRUE;
    }
	
    else if ((prRxStatus->u2StatusFlag & RXS_DW2_RX_nERR_BITMAP) == RXS_DW2_RX_nERR_VALUE) {
        if ((prRxStatus->u2StatusFlag & RXS_DW2_RX_nDATA_BITMAP) == RXS_DW2_RX_nDATA_VALUE) {
            prSwRfb->fgDataFrame = FALSE;
        }
        if ((prRxStatus->u2StatusFlag & RXS_DW2_RX_FRAG_BITMAP) == RXS_DW2_RX_FRAG_VALUE) {
            prSwRfb->fgFragFrame = TRUE;
        }
	} else {
        fgDrop = TRUE;
		if (!HAL_RX_STATUS_IS_ICV_ERROR(prRxStatus)
		    && HAL_RX_STATUS_IS_TKIP_MIC_ERROR(prRxStatus)) {
            P_STA_RECORD_T prStaRec;
            
            prStaRec = cnmGetStaRecByAddress(prAdapter,
                    prAdapter->prAisBssInfo->ucBssIndex,
							 prAdapter->rWlanInfo.rCurrBssId.
							 arMacAddress);
            if (prStaRec) {
                DBGLOG(RSN, EVENT, ("MIC_ERR_PKT\n"));
                rsnTkipHandleMICFailure(prAdapter, prStaRec, 0);
            }
		} else if (HAL_RX_STATUS_IS_LLC_MIS(prRxStatus)) {
            DBGLOG(RSN, EVENT, ("LLC_MIS_ERR\n"));
            fgDrop = FALSE; 
        }
    }

#if 0 
    if (prSwRfb->u2PacketLen > 14) {
		PUINT_8 pc = (PUINT_8) prSwRfb->pvHeader;
        UINT_16 u2Etype = 0;

        u2Etype = (pc[ETHER_TYPE_LEN_OFFSET] << 8) | (pc[ETHER_TYPE_LEN_OFFSET + 1]);

#if CFG_SUPPORT_WAPI
        if (u2Etype == ETH_P_1X || u2Etype == ETH_WPI_1X) {
            DBGLOG(RSN, INFO, ("R1X len=%d\n", prSwRfb->u2PacketLen));
        }
#else
        if (u2Etype == ETH_P_1X) {
            DBGLOG(RSN, INFO, ("R1X len=%d\n", prSwRfb->u2PacketLen));
        }
#endif
        else if (u2Etype == ETH_P_PRE_1X) {
            DBGLOG(RSN, INFO, ("Pre R1X len=%d\n", prSwRfb->u2PacketLen));
        }
    }
#endif

#if CFG_TCP_IP_CHKSUM_OFFLOAD || CFG_TCP_IP_CHKSUM_OFFLOAD_NDIS_60
	if (fgDrop == FALSE) {
        UINT_32 u4TcpUdpIpCksStatus;
		PUINT_32 pu4Temp;
		pu4Temp = (PUINT_32)prRxStatus;
		u4TcpUdpIpCksStatus = *(pu4Temp+(ALIGN_4(prRxStatus->u2RxByteCount)>>2));
        nicRxFillChksumStatus(prAdapter, prSwRfb, u4TcpUdpIpCksStatus);
    }
#endif 

	
    if (prAdapter->fgTestMode == FALSE && fgDrop == FALSE) {
#if CFG_HIF_RX_STARVATION_WARNING
        prRxCtrl->u4QueuedCnt++;
#endif
        nicRxFillRFB(prAdapter, prSwRfb);
		GLUE_SET_PKT_BSS_IDX(prSwRfb->pvPacket,
				     secGetBssIdxByWlanIdx(prAdapter, prSwRfb->ucWlanIdx));

		prRetSwRfb = qmHandleRxPackets(prAdapter, prSwRfb);
		if (prRetSwRfb != NULL) {
            do {
				
				prNextSwRfb =
				    (P_SW_RFB_T) QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T) prRetSwRfb);

				switch (prRetSwRfb->eDst) {
                case RX_PKT_DESTINATION_HOST:
                    nicRxProcessPktWithoutReorder(prAdapter, prRetSwRfb);
                    break;

                case RX_PKT_DESTINATION_FORWARD:
                    nicRxProcessForwardPkt(prAdapter, prRetSwRfb);
                    break;

                case RX_PKT_DESTINATION_HOST_WITH_FORWARD:
                    nicRxProcessGOBroadcastPkt(prAdapter, prRetSwRfb);
                    break;

                case RX_PKT_DESTINATION_NULL:
                    nicRxReturnRFB(prAdapter, prRetSwRfb);
                    RX_INC_CNT(prRxCtrl, RX_DST_NULL_DROP_COUNT);
                    RX_INC_CNT(prRxCtrl, RX_DROP_TOTAL_COUNT);
                    break;

                default:
                    break;
                }
#if CFG_HIF_RX_STARVATION_WARNING
                prRxCtrl->u4DequeuedCnt++;
#endif
                prRetSwRfb = prNextSwRfb;
			} while (prRetSwRfb);
        }
	} else {
        nicRxReturnRFB(prAdapter, prSwRfb);
        RX_INC_CNT(prRxCtrl, RX_CLASS_ERR_DROP_COUNT);
        RX_INC_CNT(prRxCtrl, RX_DROP_TOTAL_COUNT);
    }
}


VOID nicRxProcessEventPacket(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
    P_CMD_INFO_T prCmdInfo;
    P_MSDU_INFO_T prMsduInfo;
    P_WIFI_EVENT_T prEvent;
    P_GLUE_INFO_T prGlueInfo;
    BOOLEAN fgIsNewVersion;

    DEBUGFUNC("nicRxProcessEventPacket");
	

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prEvent = (P_WIFI_EVENT_T) prSwRfb->pucRecvBuff;
    prGlueInfo = prAdapter->prGlueInfo;

    DBGLOG(INIT, INFO, ("RX EVENT: ID[0x%02X] SEQ[%u] LEN[%u]\n", 
			    prEvent->ucEID, prEvent->ucSeqNum, prEvent->u2PacketLength));
    
	
	switch (prEvent->ucEID) {
#if 0 
    case EVENT_ID_CMD_RESULT:
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
            P_EVENT_CMD_RESULT prCmdResult;
			prCmdResult = (P_EVENT_CMD_RESULT) ((PUINT_8) prEvent + EVENT_HDR_SIZE);

            
            ASSERT(prCmdInfo->fgSetQuery == FALSE || prCmdInfo->fgNeedResp == TRUE);

			if (prCmdResult->ucStatus == 0) {	
				if (prCmdInfo->pfCmdDoneHandler) {
					prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
								    prEvent->aucBuffer);
				} else if (prCmdInfo->fgIsOid == TRUE) {
					kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery,
						       0, WLAN_STATUS_SUCCESS);
                }
			} else if (prCmdResult->ucStatus == 1) {	
				if (prCmdInfo->fgIsOid == TRUE)
					kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery,
						       0, WLAN_STATUS_FAILURE);
			} else if (prCmdResult->ucStatus == 2) {	
				if (prCmdInfo->fgIsOid == TRUE)
					kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery,
						       0, WLAN_STATUS_NOT_SUPPORTED);
                }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }

        break;
#endif

#if 0
    case EVENT_ID_CONNECTION_STATUS:
        
        {
            P_EVENT_CONNECTION_STATUS prConnectionStatus;
            prConnectionStatus = (P_EVENT_CONNECTION_STATUS) (prEvent->aucBuffer);

			DbgPrint("RX EVENT: EVENT_ID_CONNECTION_STATUS = %d\n",
				 prConnectionStatus->ucMediaStatus);
			if (prConnectionStatus->ucMediaStatus == PARAM_MEDIA_STATE_DISCONNECTED) {	
				if (kalGetMediaStateIndicated(prGlueInfo) !=
				    PARAM_MEDIA_STATE_DISCONNECTED) {

                    kalIndicateStatusAndComplete(prGlueInfo,
                            WLAN_STATUS_MEDIA_DISCONNECT,
								     NULL, 0);

                    prAdapter->rWlanInfo.u4SysTime = kalGetTimeTick();
                }
			} else if (prConnectionStatus->ucMediaStatus == PARAM_MEDIA_STATE_CONNECTED) {	
                prAdapter->rWlanInfo.u4SysTime = kalGetTimeTick();

				
                prAdapter->rWlanInfo.rCurrBssId.rSsid.u4SsidLen
                    = prConnectionStatus->ucSsidLen;
                kalMemCopy(prAdapter->rWlanInfo.rCurrBssId.rSsid.aucSsid,
                        prConnectionStatus->aucSsid,
                        prConnectionStatus->ucSsidLen);

                kalMemCopy(prAdapter->rWlanInfo.rCurrBssId.arMacAddress,
					   prConnectionStatus->aucBssid, MAC_ADDR_LEN);

				prAdapter->rWlanInfo.rCurrBssId.u4Privacy = prConnectionStatus->ucEncryptStatus;	
				prAdapter->rWlanInfo.rCurrBssId.rRssi = 0;	
				prAdapter->rWlanInfo.rCurrBssId.eNetworkTypeInUse = PARAM_NETWORK_TYPE_AUTOMODE;	
                prAdapter->rWlanInfo.rCurrBssId.rConfiguration.u4BeaconPeriod
                    = prConnectionStatus->u2BeaconPeriod;
                prAdapter->rWlanInfo.rCurrBssId.rConfiguration.u4ATIMWindow
                    = prConnectionStatus->u2ATIMWindow;
                prAdapter->rWlanInfo.rCurrBssId.rConfiguration.u4DSConfig
                    = prConnectionStatus->u4FreqInKHz;
                prAdapter->rWlanInfo.ucNetworkType
                    = prConnectionStatus->ucNetworkType;

				switch (prConnectionStatus->ucInfraMode) {
                case 0:
                    prAdapter->rWlanInfo.rCurrBssId.eOpMode = NET_TYPE_IBSS;
                    break;
                case 1:
                    prAdapter->rWlanInfo.rCurrBssId.eOpMode = NET_TYPE_INFRA;
                    break;
                case 2:
                default:
					prAdapter->rWlanInfo.rCurrBssId.eOpMode =
					    NET_TYPE_AUTO_SWITCH;
                    break;
                }
				
                kalIndicateStatusAndComplete(prGlueInfo,
							     WLAN_STATUS_MEDIA_CONNECT, NULL, 0);
            }
        }
        break;

    case EVENT_ID_SCAN_RESULT:
        
        break;
#endif

    case EVENT_ID_RX_ADDBA:
        
        qmHandleEventRxAddBa(prAdapter, prEvent);
        break;

    case EVENT_ID_RX_DELBA:
        
        qmHandleEventRxDelBa(prAdapter, prEvent);
        break;

    case EVENT_ID_CHECK_REORDER_BUBBLE:
        qmHandleEventCheckReorderBubble(prAdapter, prEvent);
        break;

    case EVENT_ID_LINK_QUALITY:
#if CFG_ENABLE_WIFI_DIRECT && CFG_SUPPORT_P2P_RSSI_QUERY
        if (prEvent->u2PacketLen == EVENT_HDR_SIZE + sizeof(EVENT_LINK_QUALITY_EX)) {
			P_EVENT_LINK_QUALITY_EX prLqEx =
			    (P_EVENT_LINK_QUALITY_EX) (prEvent->aucBuffer);

            if (prLqEx->ucIsLQ0Rdy) {
				nicUpdateLinkQuality(prAdapter, 0, (P_EVENT_LINK_QUALITY) prLqEx);
            }
            if (prLqEx->ucIsLQ1Rdy) {
				nicUpdateLinkQuality(prAdapter, 1, (P_EVENT_LINK_QUALITY) prLqEx);
            }
		} else {
            
            DBGLOG(P2P, WARN, ("Old FW version, not support P2P RSSI query.\n"));

            
			nicUpdateLinkQuality(prAdapter, 0,
					     (P_EVENT_LINK_QUALITY) (prEvent->aucBuffer));
        }
#else
		
        {
            UINT_8          ucBssIndex;
            P_BSS_INFO_T    prBssInfo;
            for (ucBssIndex = 0; ucBssIndex < BSS_INFO_NUM; ucBssIndex++) {
                prBssInfo = prAdapter->aprBssInfo[ucBssIndex];

				if ((prBssInfo->eNetworkType == NETWORK_TYPE_AIS)
				    && (prBssInfo->fgIsInUse))
                    break;
            }

            if (ucBssIndex >= BSS_INFO_NUM) {
                ucBssIndex = 1; 
            }
			
			nicUpdateLinkQuality(prAdapter, ucBssIndex,
					     (P_EVENT_LINK_QUALITY_V2) (prEvent->aucBuffer));
        }
        
#endif

        
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
            if (prCmdInfo->pfCmdDoneHandler) {
				prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
							    prEvent->aucBuffer);
			} else if (prCmdInfo->fgIsOid) {
				kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery, 0,
					       WLAN_STATUS_SUCCESS);
            }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }
#ifndef LINUX
		if (prAdapter->rWlanInfo.eRssiTriggerType == ENUM_RSSI_TRIGGER_GREATER &&
		    prAdapter->rWlanInfo.rRssiTriggerValue >=
		    (PARAM_RSSI) (prAdapter->rLinkQuality.cRssi)) {
            prAdapter->rWlanInfo.eRssiTriggerType = ENUM_RSSI_TRIGGER_TRIGGERED;

            kalIndicateStatusAndComplete(prGlueInfo,
                    WLAN_STATUS_MEDIA_SPECIFIC_INDICATION,
						     (PVOID) & (prAdapter->rWlanInfo.
								rRssiTriggerValue),
						     sizeof(PARAM_RSSI));
		} else if (prAdapter->rWlanInfo.eRssiTriggerType == ENUM_RSSI_TRIGGER_LESS
			   && prAdapter->rWlanInfo.rRssiTriggerValue <=
			   (PARAM_RSSI) (prAdapter->rLinkQuality.cRssi)) {
            prAdapter->rWlanInfo.eRssiTriggerType = ENUM_RSSI_TRIGGER_TRIGGERED;

            kalIndicateStatusAndComplete(prGlueInfo,
                    WLAN_STATUS_MEDIA_SPECIFIC_INDICATION,
						     (PVOID) & (prAdapter->rWlanInfo.
								rRssiTriggerValue),
						     sizeof(PARAM_RSSI));
        }
#endif

        break;

    case EVENT_ID_MIC_ERR_INFO:
        {
            P_EVENT_MIC_ERR_INFO prMicError;
			
            P_STA_RECORD_T prStaRec;

            DBGLOG(RSN, EVENT, ("EVENT_ID_MIC_ERR_INFO\n"));

			prMicError = (P_EVENT_MIC_ERR_INFO) (prEvent->aucBuffer);
            prStaRec = cnmGetStaRecByAddress(prAdapter,
                    prAdapter->prAisBssInfo->ucBssIndex,
							 prAdapter->rWlanInfo.rCurrBssId.
							 arMacAddress);
            ASSERT(prStaRec);

            if (prStaRec) {
				rsnTkipHandleMICFailure(prAdapter, prStaRec,
							(BOOLEAN) prMicError->u4Flags);
			} else {
                DBGLOG(RSN, INFO, ("No STA rec!!\n"));
            }
#if 0
			prAuthEvent = (P_PARAM_AUTH_EVENT_T) prAdapter->aucIndicationEventBuffer;

            
            prAuthEvent->rStatus.eStatusType = ENUM_STATUS_TYPE_AUTHENTICATION;

            
            prAuthEvent->arRequest[0].u4Length = sizeof(PARAM_AUTH_REQUEST_T);
			kalMemCopy((PVOID) prAuthEvent->arRequest[0].arBssid, (PVOID) prAdapter->rWlanInfo.rCurrBssId.arMacAddress,	
                PARAM_MAC_ADDR_LEN);

            if (prMicError->u4Flags != 0) {
                prAuthEvent->arRequest[0].u4Flags = PARAM_AUTH_REQUEST_GROUP_ERROR;
			} else {
				prAuthEvent->arRequest[0].u4Flags =
				    PARAM_AUTH_REQUEST_PAIRWISE_ERROR;
            }

            kalIndicateStatusAndComplete(prAdapter->prGlueInfo,
                WLAN_STATUS_MEDIA_SPECIFIC_INDICATION,
						     (PVOID) prAuthEvent,
						     sizeof(PARAM_STATUS_INDICATION_T) +
						     sizeof(PARAM_AUTH_REQUEST_T));
#endif
        }
        break;

#if 0 
    case EVENT_ID_ASSOC_INFO:
        {
            P_EVENT_ASSOC_INFO prAssocInfo;
			prAssocInfo = (P_EVENT_ASSOC_INFO) (prEvent->aucBuffer);

            kalHandleAssocInfo(prAdapter->prGlueInfo, prAssocInfo);
        }
        break;

    case EVENT_ID_802_11_PMKID:
        {
            P_PARAM_AUTH_EVENT_T           prAuthEvent;
            PUINT_8                        cp;
            UINT_32                        u4LenOfUsedBuffer;

			prAuthEvent = (P_PARAM_AUTH_EVENT_T) prAdapter->aucIndicationEventBuffer;

            prAuthEvent->rStatus.eStatusType = ENUM_STATUS_TYPE_CANDIDATE_LIST;

			u4LenOfUsedBuffer = (UINT_32) (prEvent->u2PacketLength - 8);

            prAuthEvent->arRequest[0].u4Length = u4LenOfUsedBuffer;

			cp = (PUINT_8) &prAuthEvent->arRequest[0];

            
			kalMemCopy(cp, (P_EVENT_PMKID_CANDIDATE_LIST_T) (prEvent->aucBuffer),
				   prEvent->u2PacketLength - 8);

            kalIndicateStatusAndComplete(prAdapter->prGlueInfo,
                WLAN_STATUS_MEDIA_SPECIFIC_INDICATION,
						     (PVOID) prAuthEvent,
						     sizeof(PARAM_STATUS_INDICATION_T) +
						     u4LenOfUsedBuffer);
        }
        break;
#endif

#if 0
    case EVENT_ID_ACTIVATE_STA_REC_T:
        {
            P_EVENT_ACTIVATE_STA_REC_T prActivateStaRec;
			prActivateStaRec = (P_EVENT_ACTIVATE_STA_REC_T) (prEvent->aucBuffer);

			DbgPrint("RX EVENT: EVENT_ID_ACTIVATE_STA_REC_T Index:%d, MAC:[" MACSTR
				 "]\n", prActivateStaRec->ucStaRecIdx,
                MAC2STR(prActivateStaRec->aucMacAddr));

            qmActivateStaRec(prAdapter,
					 (UINT_32) prActivateStaRec->ucStaRecIdx,
					 ((prActivateStaRec->fgIsQoS) ? TRUE : FALSE),
                             prActivateStaRec->ucNetworkTypeIndex,
					 ((prActivateStaRec->fgIsAP) ? TRUE : FALSE),
                             prActivateStaRec->aucMacAddr);

        }
        break;

    case EVENT_ID_DEACTIVATE_STA_REC_T:
        {
            P_EVENT_DEACTIVATE_STA_REC_T prDeactivateStaRec;
			prDeactivateStaRec = (P_EVENT_DEACTIVATE_STA_REC_T) (prEvent->aucBuffer);

			DbgPrint("RX EVENT: EVENT_ID_DEACTIVATE_STA_REC_T Index:%d, MAC:[" MACSTR
				 "]\n", prDeactivateStaRec->ucStaRecIdx);

			qmDeactivateStaRec(prAdapter, prDeactivateStaRec->ucStaRecIdx);
        }
        break;
#endif

    case EVENT_ID_SCAN_DONE:
        fgIsNewVersion = FALSE;
        if (prEvent->u2PacketLength > (EVENT_HDR_SIZE + sizeof(EVENT_SCAN_DONE)- SCAN_DONE_DIFFERENCE)){
            fgIsNewVersion = TRUE;
        }
	scnEventScanDone(prAdapter, (P_EVENT_SCAN_DONE)(prEvent->aucBuffer), fgIsNewVersion  );
        break;

    case EVENT_ID_NLO_DONE:
        DBGLOG(INIT, INFO, ("EVENT_ID_NLO_DONE \n"));
		scnEventNloDone(prAdapter, (P_EVENT_NLO_DONE_T) (prEvent->aucBuffer));
#if CFG_SUPPORT_PNO        
        prAdapter->prAisBssInfo->fgIsPNOEnable = FALSE;
        if(prAdapter->prAisBssInfo->fgIsNetRequestInActive && prAdapter->prAisBssInfo->fgIsPNOEnable){
            UNSET_NET_ACTIVE(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);        
            DBGLOG(INIT, INFO, ("INACTIVE  AIS from  ACTIVEto disable PNO \n"));
            
            nicDeactivateNetwork(prAdapter, prAdapter->prAisBssInfo->ucBssIndex);
        }
#endif  
        break;

    case EVENT_ID_TX_DONE:
        {
            P_EVENT_TX_DONE_T prTxDone;
			prTxDone = (P_EVENT_TX_DONE_T) (prEvent->aucBuffer);

            DBGLOG(INIT, INFO,("EVENT_ID_TX_DONE WIDX:PID[%u:%u] Status[%u] SN[%u]\n",
                prTxDone->ucWlanIndex, prTxDone->ucPacketSeq, prTxDone->ucStatus, prTxDone->u2SequenceNumber));

            
			prMsduInfo =
			    nicGetPendingTxMsduInfo(prAdapter, prTxDone->ucWlanIndex,
						    prTxDone->ucPacketSeq);

#if CFG_SUPPORT_802_11V_TIMING_MEASUREMENT
            DBGLOG(INIT, TRACE, ("EVENT_ID_TX_DONE u4TimeStamp = %x u2AirDelay = %x\n",
                prTxDone->au4Reserved1, prTxDone->au4Reserved2));

            wnmReportTimingMeas(prAdapter, prMsduInfo->ucStaRecIndex,
					    prTxDone->au4Reserved1,
					    prTxDone->au4Reserved1 + prTxDone->au4Reserved2);
#endif

			if (prMsduInfo) {
				prMsduInfo->pfTxDoneHandler(prAdapter, prMsduInfo,
							    (ENUM_TX_RESULT_CODE_T) (prTxDone->
										     ucStatus));

                cnmMgtPktFree(prAdapter, prMsduInfo);
            }
        }
        break;

    case EVENT_ID_SLEEPY_INFO:
        {
            P_EVENT_SLEEPY_INFO_T prEventSleepyNotify;
			prEventSleepyNotify = (P_EVENT_SLEEPY_INFO_T) (prEvent->aucBuffer);

			

			prAdapter->fgWiFiInSleepyState =
			    (BOOLEAN) (prEventSleepyNotify->ucSleepyState);

#if CFG_SUPPORT_MULTITHREAD
			if (prEventSleepyNotify->ucSleepyState) {
                kalSetFwOwnEvent2Hif(prGlueInfo);
            }
#endif            
        }
        break;
    case EVENT_ID_BT_OVER_WIFI:
#if CFG_ENABLE_BT_OVER_WIFI
        {
            UINT_8 aucTmp[sizeof(AMPC_EVENT) + sizeof(BOW_LINK_DISCONNECTED)];
            P_EVENT_BT_OVER_WIFI prEventBtOverWifi;
            P_AMPC_EVENT prBowEvent;
            P_BOW_LINK_CONNECTED prBowLinkConnected;
            P_BOW_LINK_DISCONNECTED prBowLinkDisconnected;

			prEventBtOverWifi = (P_EVENT_BT_OVER_WIFI) (prEvent->aucBuffer);

			
			prBowEvent = (P_AMPC_EVENT) aucTmp;

			if (prEventBtOverWifi->ucLinkStatus == 0) {
				
                prBowEvent->rHeader.ucEventId = BOW_EVENT_ID_LINK_CONNECTED;
                prBowEvent->rHeader.ucSeqNumber = 0;
                prBowEvent->rHeader.u2PayloadLength = sizeof(BOW_LINK_CONNECTED);

				
				prBowLinkConnected =
				    (P_BOW_LINK_CONNECTED) (prBowEvent->aucPayload);
				prBowLinkConnected->rChannel.ucChannelNum =
				    prEventBtOverWifi->ucSelectedChannel;
				kalMemZero(prBowLinkConnected->aucPeerAddress, MAC_ADDR_LEN);	

                kalIndicateBOWEvent(prAdapter->prGlueInfo, prBowEvent);
			} else {
				
                prBowEvent->rHeader.ucEventId = BOW_EVENT_ID_LINK_DISCONNECTED;
                prBowEvent->rHeader.ucSeqNumber = 0;
                prBowEvent->rHeader.u2PayloadLength = sizeof(BOW_LINK_DISCONNECTED);

				
				prBowLinkDisconnected =
				    (P_BOW_LINK_DISCONNECTED) (prBowEvent->aucPayload);
				prBowLinkDisconnected->ucReason = 0;	
				kalMemZero(prBowLinkDisconnected->aucPeerAddress, MAC_ADDR_LEN);	

                kalIndicateBOWEvent(prAdapter->prGlueInfo, prBowEvent);
            }
        }
        break;
#endif
    case EVENT_ID_STATISTICS:
        
        prAdapter->fgIsStatValid = TRUE;
        prAdapter->rStatUpdateTime = kalGetTimeTick();
        kalMemCopy(&prAdapter->rStatStruct, prEvent->aucBuffer, sizeof(EVENT_STATISTICS));

        
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
            if (prCmdInfo->pfCmdDoneHandler) {
				prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
							    prEvent->aucBuffer);
			} else if (prCmdInfo->fgIsOid) {
				kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery, 0,
					       WLAN_STATUS_SUCCESS);
            }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }

        break;

    case EVENT_ID_CH_PRIVILEGE:
        cnmChMngrHandleChEvent(prAdapter, prEvent);
        break;

    case EVENT_ID_BSS_ABSENCE_PRESENCE:
        qmHandleEventBssAbsencePresence(prAdapter, prEvent);
        break;

    case EVENT_ID_STA_CHANGE_PS_MODE:
        qmHandleEventStaChangePsMode(prAdapter, prEvent);
        break;
#if CFG_ENABLE_WIFI_DIRECT
    case EVENT_ID_STA_UPDATE_FREE_QUOTA:
        qmHandleEventStaUpdateFreeQuota(prAdapter, prEvent);
        break;
#endif
    case EVENT_ID_BSS_BEACON_TIMEOUT:
		DBGLOG(INIT, INFO, ("EVENT_ID_BSS_BEACON_TIMEOUT\n"));

        if (prAdapter->fgDisBcnLostDetection == FALSE) {
			P_BSS_INFO_T prBssInfo = (P_BSS_INFO_T) NULL;
            P_EVENT_BSS_BEACON_TIMEOUT_T prEventBssBeaconTimeout;
			prEventBssBeaconTimeout =
			    (P_EVENT_BSS_BEACON_TIMEOUT_T) (prEvent->aucBuffer);

            if (prEventBssBeaconTimeout->ucBssIndex >= BSS_INFO_NUM) {
                break;
            }

            DBGLOG(INIT, INFO, ("Reason code: %d \n", prEventBssBeaconTimeout->ucReasonCode));

			prBssInfo =
			    GET_BSS_INFO_BY_INDEX(prAdapter, prEventBssBeaconTimeout->ucBssIndex);

			if (prEventBssBeaconTimeout->ucBssIndex ==
			    prAdapter->prAisBssInfo->ucBssIndex) {
                aisBssBeaconTimeout(prAdapter);
            }
#if CFG_ENABLE_WIFI_DIRECT
			else if ((prBssInfo->eNetworkType == NETWORK_TYPE_P2P)) {
                p2pRoleFsmRunEventBeaconTimeout(prAdapter, prBssInfo);
            }
#endif
#if CFG_ENABLE_BT_OVER_WIFI
			else if (GET_BSS_INFO_BY_INDEX
				 (prAdapter,
				  prEventBssBeaconTimeout->ucBssIndex)->eNetworkType ==
				 NETWORK_TYPE_BOW) {
            }
#endif
            else {
				DBGLOG(RX, ERROR,
				       ("EVENT_ID_BSS_BEACON_TIMEOUT: (ucBssIndex = %d)\n",
                            prEventBssBeaconTimeout->ucBssIndex));
            }
        }

        break;
    case EVENT_ID_UPDATE_NOA_PARAMS:
#if CFG_ENABLE_WIFI_DIRECT
		if (prAdapter->fgIsP2PRegistered) {
            P_EVENT_UPDATE_NOA_PARAMS_T prEventUpdateNoaParam;
			prEventUpdateNoaParam = (P_EVENT_UPDATE_NOA_PARAMS_T) (prEvent->aucBuffer);

			if (GET_BSS_INFO_BY_INDEX(prAdapter, prEventUpdateNoaParam->ucBssIndex)->
			    eNetworkType == NETWORK_TYPE_P2P) {
                p2pProcessEvent_UpdateNOAParam(prAdapter,
                                                prEventUpdateNoaParam->ucBssIndex,
                                                prEventUpdateNoaParam);
            } else {
                ASSERT(0);
            }
        }
#else
        ASSERT(0);
#endif
        break;

    case EVENT_ID_STA_AGING_TIMEOUT:
#if CFG_ENABLE_WIFI_DIRECT
        {
            if (prAdapter->fgDisStaAgingTimeoutDetection == FALSE) {
                P_EVENT_STA_AGING_TIMEOUT_T prEventStaAgingTimeout;
                P_STA_RECORD_T prStaRec;
				P_BSS_INFO_T prBssInfo = (P_BSS_INFO_T) NULL;

				prEventStaAgingTimeout =
				    (P_EVENT_STA_AGING_TIMEOUT_T) (prEvent->aucBuffer);
				prStaRec =
				    cnmGetStaRecByIndex(prAdapter,
							prEventStaAgingTimeout->ucStaRecIdx);
                if (prStaRec == NULL) {
                    break;
                }

				DBGLOG(INIT, INFO, ("EVENT_ID_STA_AGING_TIMEOUT %u " MACSTR "\n",
						    prEventStaAgingTimeout->ucStaRecIdx,
						    MAC2STR(prStaRec->aucMacAddr)));

                prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prStaRec->ucBssIndex);

                bssRemoveClient(prAdapter, prBssInfo, prStaRec);

                
                if (prAdapter->fgIsP2PRegistered) {
					p2pFuncDisconnect(prAdapter, prBssInfo, prStaRec, TRUE,
							  REASON_CODE_DISASSOC_INACTIVITY);
                }


			}
			
        }
#endif
        break;

    case EVENT_ID_AP_OBSS_STATUS:
#if CFG_ENABLE_WIFI_DIRECT
		if (prAdapter->fgIsP2PRegistered) {
			rlmHandleObssStatusEventPkt(prAdapter,
						    (P_EVENT_AP_OBSS_STATUS_T) prEvent->aucBuffer);
        }
#endif
        break;

    case EVENT_ID_ROAMING_STATUS:
#if CFG_SUPPORT_ROAMING
        {
            P_CMD_ROAMING_TRANSIT_T prTransit;

			prTransit = (P_CMD_ROAMING_TRANSIT_T) (prEvent->aucBuffer);
            roamingFsmProcessEvent(prAdapter, prTransit);
        }
#endif 
        break;
    case EVENT_ID_SEND_DEAUTH:
#if DBG
        {
            P_WLAN_MAC_HEADER_T prWlanMacHeader;

			prWlanMacHeader = (P_WLAN_MAC_HEADER_T) &prEvent->aucBuffer[0];
			DBGLOG(RX, INFO,
			       ("nicRx: aucAddr1: " MACSTR "\n",
				MAC2STR(prWlanMacHeader->aucAddr1)));
			DBGLOG(RX, INFO,
			       ("nicRx: aucAddr2: " MACSTR "\n",
				MAC2STR(prWlanMacHeader->aucAddr2)));
        }
#endif
          
		prSwRfb->pvHeader = (P_WLAN_MAC_HEADER_T) &prEvent->aucBuffer[0];
          if (WLAN_STATUS_SUCCESS == authSendDeauthFrame(prAdapter,
                                                       NULL,
                                                       NULL,
                                                       prSwRfb,
                                                       REASON_CODE_CLASS_3_ERR,
							       (PFN_TX_DONE_HANDLER) NULL)) {
            DBGLOG(RX, INFO, ("Send Deauth Error\n"));
        }
          break;

#if CFG_SUPPORT_RDD_TEST_MODE
    case EVENT_ID_UPDATE_RDD_STATUS:
        {
            P_EVENT_RDD_STATUS_T prEventRddStatus;

            prEventRddStatus = (P_EVENT_RDD_STATUS_T) (prEvent->aucBuffer);

            prAdapter->ucRddStatus = prEventRddStatus->ucRddStatus;
        }

        break;
#endif

#if CFG_SUPPORT_BCM && CFG_SUPPORT_BCM_BWCS
    case EVENT_ID_UPDATE_BWCS_STATUS:
        {
            P_PTA_IPC_T prEventBwcsStatus;

            prEventBwcsStatus = (P_PTA_IPC_T) (prEvent->aucBuffer);

#if CFG_SUPPORT_BCM_BWCS_DEBUG
			printk(KERN_INFO DRV_NAME "BCM BWCS Event: %02x%02x%02x%02x\n",
			       prEventBwcsStatus->u.aucBTPParams[0],
                prEventBwcsStatus->u.aucBTPParams[1],
                prEventBwcsStatus->u.aucBTPParams[2],
                prEventBwcsStatus->u.aucBTPParams[3]);

			printk(KERN_INFO DRV_NAME
			       "BCM BWCS Event: aucBTPParams[0] = %02x, aucBTPParams[1] = %02x, aucBTPParams[2] = %02x, aucBTPParams[3] = %02x\n",
                prEventBwcsStatus->u.aucBTPParams[0],
                prEventBwcsStatus->u.aucBTPParams[1],
                prEventBwcsStatus->u.aucBTPParams[2],
                prEventBwcsStatus->u.aucBTPParams[3]);
#endif

            kalIndicateStatusAndComplete(prAdapter->prGlueInfo,
                WLAN_STATUS_BWCS_UPDATE,
						     (PVOID) prEventBwcsStatus, sizeof(PTA_IPC_T));
        }

        break;

    case EVENT_ID_UPDATE_BCM_DEBUG:
        {
            P_PTA_IPC_T prEventBwcsStatus;

            prEventBwcsStatus = (P_PTA_IPC_T) (prEvent->aucBuffer);

#if CFG_SUPPORT_BCM_BWCS_DEBUG
			printk(KERN_INFO DRV_NAME "BCM FW status: %02x%02x%02x%02x\n",
			       prEventBwcsStatus->u.aucBTPParams[0],
                prEventBwcsStatus->u.aucBTPParams[1],
                prEventBwcsStatus->u.aucBTPParams[2],
                prEventBwcsStatus->u.aucBTPParams[3]);

			printk(KERN_INFO DRV_NAME
			       "BCM FW status: aucBTPParams[0] = %02x, aucBTPParams[1] = %02x, aucBTPParams[2] = %02x, aucBTPParams[3] = %02x\n",
                prEventBwcsStatus->u.aucBTPParams[0],
                prEventBwcsStatus->u.aucBTPParams[1],
                prEventBwcsStatus->u.aucBTPParams[2],
                prEventBwcsStatus->u.aucBTPParams[3]);
#endif
        }

        break;
#endif
    case EVENT_ID_ADD_PKEY_DONE:
        {
            P_EVENT_ADD_KEY_DONE_INFO prAddKeyDone;
            P_STA_RECORD_T prStaRec;

			prAddKeyDone = (P_EVENT_ADD_KEY_DONE_INFO) (prEvent->aucBuffer);

			DBGLOG(RSN, EVENT,
			       ("EVENT_ID_ADD_PKEY_DONE BSSIDX=%d " MACSTR "\n",
				prAddKeyDone->ucBSSIndex, MAC2STR(prAddKeyDone->aucStaAddr)));

            prStaRec = cnmGetStaRecByAddress(prAdapter,
                    prAddKeyDone->ucBSSIndex,
                    prAddKeyDone->aucStaAddr);

            if (prStaRec) {
				DBGLOG(RSN, EVENT,
				       ("STA " MACSTR " Add Key Done!!\n",
					MAC2STR(prStaRec->aucMacAddr)));
                prStaRec->fgIsTxKeyReady = TRUE;
                qmUpdateStaRec(prAdapter, prStaRec);
            }
        }
        break;
    case EVENT_ID_ICAP_DONE:
        {
            P_EVENT_ICAP_STATUS_T prEventIcapStatus;
            PARAM_CUSTOM_MEM_DUMP_STRUC_T rMemDumpInfo;
            UINT_32     u4QueryInfo;

            prEventIcapStatus = (P_EVENT_ICAP_STATUS_T) (prEvent->aucBuffer);

            rMemDumpInfo.u4Address = prEventIcapStatus->u4StartAddress;
            rMemDumpInfo.u4Length = prEventIcapStatus->u4IcapSieze;

			wlanoidQueryMemDump(prAdapter, &rMemDumpInfo, sizeof(rMemDumpInfo),
					    &u4QueryInfo);
            
        }

        break;
    case EVENT_ID_DEBUG_MSG:
        {
            P_EVENT_DEBUG_MSG_T prEventDebugMsg;
            UINT_16  u2DebugMsgId;
            UINT_8   ucMsgType;
            UINT_8   ucFlags;
            UINT_32  u4Value;
            UINT_16  u2MsgSize;
            P_UINT_8 pucMsg;

            prEventDebugMsg = (P_EVENT_DEBUG_MSG_T) (prEvent->aucBuffer);

            u2DebugMsgId = prEventDebugMsg->u2DebugMsgId;
            ucMsgType = prEventDebugMsg->ucMsgType;
            ucFlags = prEventDebugMsg->ucFlags;
            u4Value = prEventDebugMsg->u4Value;
            u2MsgSize = prEventDebugMsg->u2MsgSize;
            pucMsg = prEventDebugMsg->aucMsg;

			DBGLOG(SW4, INFO, ("DEBUG_MSG Id %u Type %u Fg 0x%x Val 0x%x Size %u\n",
					   u2DebugMsgId, ucMsgType, ucFlags, u4Value, u2MsgSize));

			if (u2MsgSize <= DEBUG_MSG_SIZE_MAX) {
				if (ucMsgType >= DEBUG_MSG_TYPE_END) {
                    ucMsgType = DEBUG_MSG_TYPE_MEM32;
                }

				if (ucMsgType == DEBUG_MSG_TYPE_ASCII) {
                    pucMsg[u2MsgSize] = '\0';
                    DBGLOG(SW4, INFO, ("%s\n", pucMsg));
				} else if (ucMsgType == DEBUG_MSG_TYPE_MEM32) {

#if CFG_SUPPORT_XLOG
					
#else
					
#endif
					DBGLOG_MEM32(SW4, INFO, pucMsg, u2MsgSize);
				} else if (prEventDebugMsg->ucMsgType == DEBUG_MSG_TYPE_MEM8) {
#if CFG_SUPPORT_XLOG
#else
					
#endif
					DBGLOG_MEM8(SW4, INFO, pucMsg, u2MsgSize);
				} else {
#if CFG_SUPPORT_XLOG
					
#else
					
#endif
					DBGLOG_MEM32(SW4, INFO, pucMsg, u2MsgSize);
                }
            } 
            else {
				DBGLOG(SW4, INFO, ("Debug msg size %u is too large.\n", u2MsgSize));
            }
        }
        break;
        
#if CFG_SUPPORT_BATCH_SCAN
    case EVENT_ID_BATCH_RESULT:
        DBGLOG(SCN, TRACE, ("Got EVENT_ID_BATCH_RESULT"));

        
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
            if (prCmdInfo->pfCmdDoneHandler) {
				prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
							    prEvent->aucBuffer);
			} else if (prCmdInfo->fgIsOid) {
				kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery, 0,
					       WLAN_STATUS_SUCCESS);
            }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }

        break;
#endif 


#if CFG_SUPPORT_TDLS
        case EVENT_ID_TDLS:
            
		TdlsexEventHandle(prAdapter->prGlueInfo,
				  (UINT_8 *) prEvent->aucBuffer,
				  (UINT_32) (prEvent->u2PacketLength - 8));
            break;
#endif 

    case EVENT_ID_DUMP_MEM:
		DBGLOG(INIT, INFO, ("%s: EVENT_ID_DUMP_MEM\n", __func__));
        
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
			DBGLOG(INIT, INFO, (": ==> 1\n"));
            if (prCmdInfo->pfCmdDoneHandler) {
				prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
							    prEvent->aucBuffer);
			} else if (prCmdInfo->fgIsOid) {
				kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery, 0,
					       WLAN_STATUS_SUCCESS);
            }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
		} else {
			
			DBGLOG(INIT, INFO, (": ==> 2\n"));
			nicEventQueryMemDump(prAdapter, prEvent->aucBuffer);
        }
        break;

    case EVENT_ID_ACCESS_REG:
    case EVENT_ID_NIC_CAPABILITY:
		
    case EVENT_ID_ACCESS_EEPROM:
    case EVENT_ID_TEST_STATUS:
    default:
        prCmdInfo = nicGetPendingCmdInfo(prAdapter, prEvent->ucSeqNum);

		if (prCmdInfo != NULL) {
            if (prCmdInfo->pfCmdDoneHandler) {
				prCmdInfo->pfCmdDoneHandler(prAdapter, prCmdInfo,
							    prEvent->aucBuffer);
			} else if (prCmdInfo->fgIsOid) {
				kalOidComplete(prAdapter->prGlueInfo, prCmdInfo->fgSetQuery, 0,
					       WLAN_STATUS_SUCCESS);
            }
			
            cmdBufFreeCmdInfo(prAdapter, prCmdInfo);
        }

        break;
    }

    
	if (prGlueInfo->prAdapter->fgIsChipNoAck) {
        DBGLOG(INIT, WARN, ("Got response from chip, clear NoAck flag!\n"));
        WARN_ON(TRUE);
    }
    prGlueInfo->prAdapter->ucOidTimeoutCount = 0;
    prGlueInfo->prAdapter->fgIsChipNoAck = FALSE;

    nicRxReturnRFB(prAdapter, prSwRfb);
}


VOID nicRxProcessMgmtPacket(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
    UINT_8 ucSubtype;
#if CFG_SUPPORT_802_11W
	
#endif
    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    nicRxFillRFB(prAdapter, prSwRfb);

	ucSubtype = (*(PUINT_8) (prSwRfb->pvHeader) & MASK_FC_SUBTYPE) >> OFFSET_OF_FC_SUBTYPE;

#if CFG_RX_PKTS_DUMP
    {
        P_WLAN_MAC_MGMT_HEADER_T prWlanMgmtHeader;
        UINT_16 u2TxFrameCtrl;

		u2TxFrameCtrl = (*(PUINT_8) (prSwRfb->pvHeader) & MASK_FRAME_TYPE);
        if (prAdapter->rRxCtrl.u4RxPktsDumpTypeMask & BIT(HIF_RX_PKT_TYPE_MANAGEMENT)) {
            if (u2TxFrameCtrl == MAC_FRAME_BEACON ||
                  u2TxFrameCtrl == MAC_FRAME_PROBE_RSP) {

				prWlanMgmtHeader = (P_WLAN_MAC_MGMT_HEADER_T) (prSwRfb->pvHeader);

				DBGLOG(SW4, INFO, ("QM RX MGT: net %u sta idx %u wlan idx %u ssn %u ptype %u subtype %u 11 %u\n", prSwRfb->prStaRec->ucBssIndex, prSwRfb->ucStaRecIdx, prSwRfb->ucWlanIdx, prWlanMgmtHeader->u2SeqCtrl,	
						   prSwRfb->ucPacketType, ucSubtype));
				

				DBGLOG_MEM8(SW4, TRACE, (PUINT_8) prSwRfb->pvHeader,
					    prSwRfb->u2PacketLen);
            }
        }
    }
#endif
#if CFG_SUPPORT_802_11W
    if (HAL_RX_STATUS_IS_ICV_ERROR(prSwRfb->prRxStatus)) {
        if (HAL_RX_STATUS_GET_SEC_MODE(prSwRfb->prRxStatus) == CIPHER_SUITE_BIP) {
			DBGLOG(RSN, INFO, ("[MFP] RX with BIP ICV ERROR\n"));
		} else {
			DBGLOG(RSN, INFO, ("[MFP] RX with ICV ERROR\n"));
        }
        nicRxReturnRFB(prAdapter, prSwRfb);
        RX_INC_CNT(&prAdapter->rRxCtrl, RX_DROP_TOTAL_COUNT);
        return;     
    }
#endif

	if (prAdapter->fgTestMode == FALSE) {
#if CFG_MGMT_FRAME_HANDLING
		if (apfnProcessRxMgtFrame[ucSubtype]) {
			switch (apfnProcessRxMgtFrame[ucSubtype] (prAdapter, prSwRfb)) {
            case WLAN_STATUS_PENDING:
                return;
            case WLAN_STATUS_SUCCESS:
            case WLAN_STATUS_FAILURE:
                break;

            default:
				DBGLOG(RX, WARN,
				       ("Unexpected MMPDU(0x%02X) returned with abnormal status\n",
					ucSubtype));
                break;
            }
        }
#endif
    }

    nicRxReturnRFB(prAdapter, prSwRfb);
}

VOID nicRxProcessRFBs(IN P_ADAPTER_T prAdapter)
{
    P_RX_CTRL_T prRxCtrl;
    P_SW_RFB_T prSwRfb = (P_SW_RFB_T)NULL;
    QUE_T rTempRfbList;
    P_QUE_T prTempRfbList = &rTempRfbList;
    UINT_32 u4RxLoopCount;
    
    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxProcessRFBs");

    ASSERT(prAdapter);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    prRxCtrl->ucNumIndPacket = 0;
    prRxCtrl->ucNumRetainedPacket = 0;
    u4RxLoopCount = prAdapter->rWifiVar.u4TxRxLoopCount;

    QUEUE_INITIALIZE(prTempRfbList);

    while(u4RxLoopCount--) {
        while(QUEUE_IS_NOT_EMPTY(&prRxCtrl->rReceivedRfbList)) {
        
            KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
            QUEUE_MOVE_ALL(prTempRfbList, &prRxCtrl->rReceivedRfbList);
            KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);

            while(QUEUE_IS_NOT_EMPTY(prTempRfbList)) {
                QUEUE_REMOVE_HEAD(prTempRfbList, prSwRfb, P_SW_RFB_T);
                
                switch(prSwRfb->ucPacketType){
                case RX_PKT_TYPE_RX_DATA:
#if CFG_SUPPORT_SNIFFER
                    if (prAdapter->prGlueInfo->fgIsEnableMon) {
                        nicRxProcessMonitorPacket(prAdapter, prSwRfb);
                        break;
                    }
#endif	
                    nicRxProcessDataPacket(prAdapter, prSwRfb);
                    break;

                case RX_PKT_TYPE_SW_DEFINED:
                    
                    if ((prSwRfb->prRxStatus->u2PktTYpe & RXM_RXD_PKT_TYPE_SW_BITMAP) == RXM_RXD_PKT_TYPE_SW_EVENT) {
                        nicRxProcessEventPacket(prAdapter, prSwRfb);
                    }
                    
                    else if ((prSwRfb->prRxStatus->u2PktTYpe & RXM_RXD_PKT_TYPE_SW_BITMAP) == RXM_RXD_PKT_TYPE_SW_FRAME) {
                        nicRxProcessMgmtPacket(prAdapter, prSwRfb);
                    }
                    else {
                        DBGLOG(RX, ERROR, ("[%s]ERROR: u2PktTYpe(0x%04X) is OUT OF DEF.!!!\n", __func__, prSwRfb->prRxStatus->u2PktTYpe));
                        ASSERT(0);
                    }
                    break;

                
                
                case RX_PKT_TYPE_TX_STATUS:
                case RX_PKT_TYPE_RX_VECTOR:
                case RX_PKT_TYPE_TM_REPORT:
                default:
                    RX_INC_CNT(prRxCtrl, RX_TYPE_ERR_DROP_COUNT);
                    RX_INC_CNT(prRxCtrl, RX_DROP_TOTAL_COUNT);
                    DBGLOG(RX, ERROR, ("ucPacketType = %d\n", prSwRfb->ucPacketType));
                    break;
                }

            }

            if (prRxCtrl->ucNumIndPacket > 0) {
                RX_ADD_CNT(prRxCtrl, RX_DATA_INDICATION_COUNT, prRxCtrl->ucNumIndPacket);
                RX_ADD_CNT(prRxCtrl, RX_DATA_RETAINED_COUNT, prRxCtrl->ucNumRetainedPacket);
#if CFG_SUPPORT_MULTITHREAD 
                kalSetTxEvent2Rx(prAdapter->prGlueInfo);
#else
                
                
#if CFG_NATIVE_802_11
                kalRxIndicatePkts(prAdapter->prGlueInfo, (UINT_32)prRxCtrl->ucNumIndPacket, (UINT_32)prRxCtrl->ucNumRetainedPacket);
#else
                kalRxIndicatePkts(prAdapter->prGlueInfo, prRxCtrl->apvIndPacket, (UINT_32)prRxCtrl->ucNumIndPacket);
#endif
#endif     
            }      
        }
    }
} 


#if !CFG_SDIO_INTR_ENHANCE
WLAN_STATUS nicRxReadBuffer(IN P_ADAPTER_T prAdapter, IN OUT P_SW_RFB_T prSwRfb)
{
    P_RX_CTRL_T prRxCtrl;
    PUINT_8 pucBuf;
    P_HW_MAC_RX_DESC_T prRxStatus;
    UINT_32 u4PktLen = 0, u4ReadBytes;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
    BOOL fgResult = TRUE;
    UINT_32 u4RegValue;
    UINT_32 rxNum;

    DEBUGFUNC("nicRxReadBuffer");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    pucBuf = prSwRfb->pucRecvBuff;
    prRxStatus = prSwRfb->prRxStatus;

    ASSERT(prRxStatus);
    ASSERT(pucBuf);
    DBGLOG(RX, TRACE, ("pucBuf= 0x%x, prRxStatus= 0x%x\n", pucBuf, prRxStatus));

    do {
        
        HAL_MCR_RD(prAdapter, MCR_WRPLR, &u4RegValue);
        if (!fgResult) {
            DBGLOG(RX, ERROR, ("Read RX Packet Lentgh Error\n"));
            return WLAN_STATUS_FAILURE;
        }
		
		if (u4RegValue == 0) {
            DBGLOG(RX, ERROR, ("No RX packet\n"));
            return WLAN_STATUS_FAILURE;
        }

        u4PktLen = u4RegValue & BITS(0, 15);
		if (u4PktLen != 0) {
            rxNum = 0;
		} else {
            rxNum = 1;
            u4PktLen = (u4RegValue & BITS(16, 31)) >> 16;
        }

        DBGLOG(RX, TRACE, ("RX%d: u4PktLen = %d\n", rxNum, u4PktLen));

		
        u4ReadBytes = ALIGN_4(u4PktLen) + 4;
        HAL_READ_RX_PORT(prAdapter, rxNum, u4ReadBytes, pucBuf, CFG_RX_MAX_PKT_SIZE);

		
		
		if (u4PktLen != (UINT_32) HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus)) {
           DBGLOG(RX, ERROR, ("Read u4PktLen = %d, prHifRxHdr->u2PacketLen: %d\n",
                                u4PktLen, HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus)));
#if DBG
			dumpMemory8((PUINT_8) prRxStatus,
				    (HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus) >
				     4096) ? 4096 : prRxStatus->u2RxByteCount);
#endif
            ASSERT(0);
        }
        

		prSwRfb->ucPacketType = (UINT_8) HAL_RX_STATUS_GET_PKT_TYPE(prRxStatus);
        DBGLOG(RX, TRACE, ("ucPacketType = %d\n", prSwRfb->ucPacketType));

		prSwRfb->ucStaRecIdx =
		    secGetStaIdxByWlanIdx(prAdapter,
					  (UINT_8) HAL_RX_STATUS_GET_WLAN_IDX(prRxStatus));

        
        if (!fgResult) {
            return WLAN_STATUS_FAILURE;
        }

        DBGLOG(RX, TRACE, ("Dump RX buffer, length = 0x%x\n", u4ReadBytes));
        DBGLOG_MEM8(RX, TRACE, pucBuf, u4ReadBytes);
	} while (FALSE);

    return u4Status;
}


VOID nicRxReceiveRFBs(IN P_ADAPTER_T prAdapter)
{
    P_RX_CTRL_T prRxCtrl;
	P_SW_RFB_T prSwRfb = (P_SW_RFB_T) NULL;
    P_HW_MAC_RX_DESC_T prRxStatus;
    UINT_32 u4HwAppendDW;
	PUINT_32 pu4Temp;

    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxReceiveRFBs");

    ASSERT(prAdapter);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    do {
        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
        QUEUE_REMOVE_HEAD(&prRxCtrl->rFreeSwRfbList, prSwRfb, P_SW_RFB_T);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

        if (!prSwRfb) {
            DBGLOG(RX, TRACE, ("No More RFB\n"));
            break;
        }
		
        if (nicRxReadBuffer(prAdapter, prSwRfb) == WLAN_STATUS_FAILURE) {
            DBGLOG(RX, TRACE, ("halRxFillRFB failed\n"));
            nicRxReturnRFB(prAdapter, prSwRfb);
            break;
        }

        KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
        QUEUE_INSERT_TAIL(&prRxCtrl->rReceivedRfbList, &prSwRfb->rQueEntry);
        RX_INC_CNT(prRxCtrl, RX_MPDU_TOTAL_COUNT);
        KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);

        prRxStatus = prSwRfb->prRxStatus;
        ASSERT(prRxStatus);

		pu4Temp = (PUINT_32)prRxStatus;
		u4HwAppendDW = *(pu4Temp+(ALIGN_4(prRxStatus->u2RxByteCount)>>2));
        DBGLOG(RX, TRACE, ("u4HwAppendDW = 0x%x\n", u4HwAppendDW));
        DBGLOG(RX, TRACE, ("u2PacketLen = 0x%x\n", HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus)));
	} while (FALSE);

    return;

} 

#else

WLAN_STATUS
nicRxEnhanceReadBuffer(IN P_ADAPTER_T prAdapter,
		       IN UINT_32 u4DataPort, IN UINT_16 u2RxLength, IN OUT P_SW_RFB_T prSwRfb)
{
    P_RX_CTRL_T prRxCtrl;
    PUINT_8 pucBuf;
    P_HW_MAC_RX_DESC_T prRxStatus;
    UINT_32 u4PktLen = 0;
    WLAN_STATUS u4Status = WLAN_STATUS_FAILURE;
    BOOL fgResult = TRUE;

    DEBUGFUNC("nicRxEnhanceReadBuffer");

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    pucBuf = prSwRfb->pucRecvBuff;
    ASSERT(pucBuf);

    prRxStatus = prSwRfb->prRxStatus;
    ASSERT(prRxStatus);

	

    do {
		
        HAL_READ_RX_PORT(prAdapter,
                         u4DataPort,
                         ALIGN_4(u2RxLength + HIF_RX_HW_APPENDED_LEN),
				 pucBuf, CFG_RX_MAX_PKT_SIZE);

        if (!fgResult) {
            DBGLOG(RX, ERROR, ("Read RX Packet Lentgh Error\n"));
            break;
        }

		u4PktLen = (UINT_32) (HAL_RX_STATUS_GET_RX_BYTE_CNT(prRxStatus));
		

		prSwRfb->ucPacketType = (UINT_8) HAL_RX_STATUS_GET_PKT_TYPE(prRxStatus);
		

		prSwRfb->ucStaRecIdx =
		    secGetStaIdxByWlanIdx(prAdapter,
					  (UINT_8) HAL_RX_STATUS_GET_WLAN_IDX(prRxStatus));

		
        if (u4PktLen == 0) {
            DBGLOG(RX, ERROR, ("Packet Length = %lu\n", u4PktLen));
            ASSERT(0);
            break;
        }
		
		
        if (u4PktLen > CFG_RX_MAX_PKT_SIZE) {
            DBGLOG(RX, TRACE, ("Read RX Packet Lentgh Error (%lu)\n", u4PktLen));
            ASSERT(0);
            break;
        }

        u4Status = WLAN_STATUS_SUCCESS;
	} while (FALSE);

    DBGLOG_MEM8(RX, TRACE, pucBuf, ALIGN_4(u2RxLength + HIF_RX_HW_APPENDED_LEN));
    return u4Status;
}


VOID nicRxSDIOReceiveRFBs(IN P_ADAPTER_T prAdapter)
{
    P_SDIO_CTRL_T prSDIOCtrl;
    P_RX_CTRL_T prRxCtrl;
	P_SW_RFB_T prSwRfb = (P_SW_RFB_T) NULL;
    UINT_32 i, rxNum;
    UINT_16 u2RxPktNum, u2RxLength = 0, u2Tmp = 0;
    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxSDIOReceiveRFBs");

    ASSERT(prAdapter);

    prSDIOCtrl = prAdapter->prSDIOCtrl;
    ASSERT(prSDIOCtrl);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

	for (rxNum = 0; rxNum < 2; rxNum++) {
		u2RxPktNum =
		    (rxNum ==
		     0 ? prSDIOCtrl->rRxInfo.u.u2NumValidRx0Len : prSDIOCtrl->rRxInfo.u.
		     u2NumValidRx1Len);

		if (u2RxPktNum == 0) {
            continue;
        }

        for (i = 0; i < u2RxPktNum; i++) {
			if (rxNum == 0) {
                HAL_READ_RX_LENGTH(prAdapter, &u2RxLength, &u2Tmp);
			} else if (rxNum == 1) {
                HAL_READ_RX_LENGTH(prAdapter, &u2Tmp, &u2RxLength);
            }

            if (!u2RxLength) {
                break;
            }


            KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
            QUEUE_REMOVE_HEAD(&prRxCtrl->rFreeSwRfbList, prSwRfb, P_SW_RFB_T);
            KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

            if (!prSwRfb) {
                DBGLOG(RX, TRACE, ("No More RFB\n"));
                break;
            }
            ASSERT(prSwRfb);

			if (nicRxEnhanceReadBuffer(prAdapter, rxNum, u2RxLength, prSwRfb) ==
			    WLAN_STATUS_FAILURE) {
                DBGLOG(RX, TRACE, ("nicRxEnhanceRxReadBuffer failed\n"));
                nicRxReturnRFB(prAdapter, prSwRfb);
                break;
            }
			

            KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
            QUEUE_INSERT_TAIL(&prRxCtrl->rReceivedRfbList, &prSwRfb->rQueEntry);
            RX_INC_CNT(prRxCtrl, RX_MPDU_TOTAL_COUNT);
            KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
        }
    }

    prSDIOCtrl->rRxInfo.u.u2NumValidRx0Len = 0;
    prSDIOCtrl->rRxInfo.u.u2NumValidRx1Len = 0;

    return;
}				

#endif 



#if CFG_SDIO_RX_AGG
VOID nicRxSDIOAggReceiveRFBs(IN P_ADAPTER_T prAdapter)
{
    P_ENHANCE_MODE_DATA_STRUCT_T prEnhDataStr;
    P_RX_CTRL_T prRxCtrl;
    P_SDIO_CTRL_T prSDIOCtrl;
	P_SW_RFB_T prSwRfb = (P_SW_RFB_T) NULL;
    UINT_32 u4RxLength;
    UINT_32 i, rxNum;
    UINT_32 u4RxAggCount = 0, u4RxAggLength = 0;
    UINT_32 u4RxAvailAggLen, u4CurrAvailFreeRfbCnt;
    PUINT_8 pucSrcAddr;
    P_HW_MAC_RX_DESC_T prRxStatus;
    BOOL fgResult = TRUE;
    BOOLEAN fgIsRxEnhanceMode;
    UINT_16 u2RxPktNum;
#if CFG_SDIO_RX_ENHANCE
    UINT_32 u4MaxLoopCount = CFG_MAX_RX_ENHANCE_LOOP_COUNT;
#endif

    KAL_SPIN_LOCK_DECLARATION();

    DEBUGFUNC("nicRxSDIOAggReceiveRFBs");

    ASSERT(prAdapter);
    prEnhDataStr = prAdapter->prSDIOCtrl;
    prRxCtrl = &prAdapter->rRxCtrl;
    prSDIOCtrl = prAdapter->prSDIOCtrl;

#if CFG_SDIO_RX_ENHANCE
    fgIsRxEnhanceMode = TRUE;
#else
    fgIsRxEnhanceMode = FALSE;
#endif

    do {
#if CFG_SDIO_RX_ENHANCE
        
        u4MaxLoopCount--;
        if (u4MaxLoopCount == 0) {
            break;
        }
#endif

		if (prEnhDataStr->rRxInfo.u.u2NumValidRx0Len == 0 &&
                prEnhDataStr->rRxInfo.u.u2NumValidRx1Len == 0) {
            break;
        }

		for (rxNum = 0; rxNum < 2; rxNum++) {
			u2RxPktNum =
			    (rxNum ==
			     0 ? prEnhDataStr->rRxInfo.u.u2NumValidRx0Len : prEnhDataStr->rRxInfo.u.
			     u2NumValidRx1Len);

			
            ASSERT(u2RxPktNum <= 16);

            if (u2RxPktNum > 16)
                  continue;

			if (u2RxPktNum == 0)
                continue;

#if CFG_HIF_STATISTICS
            prRxCtrl->u4TotalRxAccessNum++;
            prRxCtrl->u4TotalRxPacketNum += u2RxPktNum;
#endif

            u4CurrAvailFreeRfbCnt = prRxCtrl->rFreeSwRfbList.u4NumElem;

			
			if (u4CurrAvailFreeRfbCnt < u2RxPktNum) {
#if CFG_HIF_RX_STARVATION_WARNING
				DbgPrint("FreeRfb is not enough: %d available, need %d\n",
					 u4CurrAvailFreeRfbCnt, u2RxPktNum);
				DbgPrint("Queued Count: %d / Dequeud Count: %d\n",
					 prRxCtrl->u4QueuedCnt, prRxCtrl->u4DequeuedCnt);
#endif
                continue;
            }
#if CFG_SDIO_RX_ENHANCE
			u4RxAvailAggLen =
			    CFG_RX_COALESCING_BUFFER_SIZE - (sizeof(ENHANCE_MODE_DATA_STRUCT_T) +
							     4 );
#else
            u4RxAvailAggLen = CFG_RX_COALESCING_BUFFER_SIZE;
#endif
            u4RxAggCount = 0;

			for (i = 0; i < u2RxPktNum; i++) {
                u4RxLength = (rxNum == 0 ?
					      (UINT_32) prEnhDataStr->rRxInfo.u.au2Rx0Len[i] :
					      (UINT_32) prEnhDataStr->rRxInfo.u.au2Rx1Len[i]);

                if (!u4RxLength) {
                    ASSERT(0);
                    break;
                }

                if (ALIGN_4(u4RxLength + HIF_RX_HW_APPENDED_LEN) < u4RxAvailAggLen) {
                    if (u4RxAggCount < u4CurrAvailFreeRfbCnt) {
						u4RxAvailAggLen -=
						    ALIGN_4(u4RxLength + HIF_RX_HW_APPENDED_LEN);
                        u4RxAggCount++;
					} else {
						
						DBGLOG(RX, ERROR,
						       ("[%s] RxAggCount(%d) is greater than AvailableFreeCount(%d)\n",
							__func__, u4RxAggCount,
                            u4CurrAvailFreeRfbCnt));                        
                        
                        ASSERT(0);
                        break;
                    }
				} else {
					
					DBGLOG(RX, ERROR,
					       ("[puff][%s] i(%d); RxPktNum(%d); Request_len(%d) > Available_len(%d)\n",
                        __func__,
                        i,
                        u2RxPktNum,
                        (ALIGN_4(u4RxLength + HIF_RX_HW_APPENDED_LEN)),
                        u4RxAvailAggLen));
		
                    
                    break;
                }
            }

            u4RxAggLength = (CFG_RX_COALESCING_BUFFER_SIZE - u4RxAvailAggLen);
			
			

            HAL_READ_RX_PORT(prAdapter,
                         rxNum,
                         u4RxAggLength,
                         prRxCtrl->pucRxCoalescingBufPtr,
                         CFG_RX_COALESCING_BUFFER_SIZE);
            if (!fgResult) {
                DBGLOG(RX, ERROR, ("Read RX Agg Packet Error\n"));
                continue;
            }

            pucSrcAddr = prRxCtrl->pucRxCoalescingBufPtr;
            for (i = 0; i < u4RxAggCount; i++) {
                UINT_16 u2PktLength;

                u2PktLength = (rxNum == 0 ?
                        prEnhDataStr->rRxInfo.u.au2Rx0Len[i] :
                        prEnhDataStr->rRxInfo.u.au2Rx1Len[i]);

				if (ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN) > CFG_RX_MAX_PKT_SIZE) {
				   DBGLOG(RX, ERROR, ("[%s] Request_len(%d) is greater than CFG_RX_MAX_PKT_SIZE(%d)...Drop the unexpected packet...\n",
					   __func__,
					   (ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN)),
					   CFG_RX_MAX_PKT_SIZE));				   
				   DBGLOG_MEM32(RX, ERROR, pucSrcAddr, ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN));
				   
				   pucSrcAddr += ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN);
				   RX_INC_CNT(prRxCtrl, RX_DROP_TOTAL_COUNT); 
				   continue;
				}

                KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
                QUEUE_REMOVE_HEAD(&prRxCtrl->rFreeSwRfbList, prSwRfb, P_SW_RFB_T);
                KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

                ASSERT(prSwRfb);
                kalMemCopy(prSwRfb->pucRecvBuff, pucSrcAddr,
                        ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN));

				
				

                prRxStatus = prSwRfb->prRxStatus;
                ASSERT(prRxStatus);

				prSwRfb->ucPacketType =
				    (UINT_8) HAL_RX_STATUS_GET_PKT_TYPE(prRxStatus);
				
#if DBG
				DBGLOG(RX, TRACE,
				       ("Rx status flag = %x wlan index = %d SecMode = %d\n",
					prRxStatus->u2StatusFlag, prRxStatus->ucWlanIdx,
                             HAL_RX_STATUS_GET_SEC_MODE(prRxStatus)));
#endif

                KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);
                QUEUE_INSERT_TAIL(&prRxCtrl->rReceivedRfbList, &prSwRfb->rQueEntry);
                RX_INC_CNT(prRxCtrl, RX_MPDU_TOTAL_COUNT);
                KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_QUE);

                pucSrcAddr += ALIGN_4(u2PktLength + HIF_RX_HW_APPENDED_LEN);
				
            }

#if CFG_SDIO_RX_ENHANCE
			kalMemCopy(prAdapter->prSDIOCtrl, (pucSrcAddr + 4),
				   sizeof(ENHANCE_MODE_DATA_STRUCT_T));

            
			if ((prSDIOCtrl->u4WHISR & WHISR_TX_DONE_INT) == 0 &&
                    (prSDIOCtrl->rTxInfo.au4WTSR[0] | prSDIOCtrl->rTxInfo.au4WTSR[1])) {
                prSDIOCtrl->u4WHISR |= WHISR_TX_DONE_INT;
            }

			if ((prSDIOCtrl->u4WHISR & BIT(31)) == 0 &&
                    HAL_GET_MAILBOX_READ_CLEAR(prAdapter) == TRUE &&
                    (prSDIOCtrl->u4RcvMailbox0 != 0 || prSDIOCtrl->u4RcvMailbox1 != 0)) {
                prSDIOCtrl->u4WHISR |= BIT(31);
            }

            
			nicProcessIST_impl(prAdapter,
					   prSDIOCtrl->
					   u4WHISR & (~(WHISR_RX0_DONE_INT | WHISR_RX1_DONE_INT)));
#endif
        }

#if !CFG_SDIO_RX_ENHANCE
        prEnhDataStr->rRxInfo.u.u2NumValidRx0Len = 0;
        prEnhDataStr->rRxInfo.u.u2NumValidRx1Len = 0;
#endif
	} while ((prEnhDataStr->rRxInfo.u.u2NumValidRx0Len
                || prEnhDataStr->rRxInfo.u.u2NumValidRx1Len)
            && fgIsRxEnhanceMode);

    return;
}
#endif 


WLAN_STATUS nicRxSetupRFB(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    PVOID   pvPacket;
    PUINT_8 pucRecvBuff;

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    if (!prSwRfb->pvPacket) {
        kalMemZero(prSwRfb, sizeof(SW_RFB_T));
		pvPacket = kalPacketAlloc(prAdapter->prGlueInfo, CFG_RX_MAX_PKT_SIZE, &pucRecvBuff);
        if (pvPacket == NULL) {
            return WLAN_STATUS_RESOURCES;
        }

        prSwRfb->pvPacket = pvPacket;
		prSwRfb->pucRecvBuff = (PVOID) pucRecvBuff;
	} else {
		kalMemZero(((PUINT_8) prSwRfb + OFFSET_OF(SW_RFB_T, prRxStatus)),
			   (sizeof(SW_RFB_T) - OFFSET_OF(SW_RFB_T, prRxStatus)));
    }

	
	
	prSwRfb->prRxStatus = (P_HW_MAC_RX_DESC_T) (prSwRfb->pucRecvBuff);

    return WLAN_STATUS_SUCCESS;

} 


VOID nicRxReturnRFB(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    P_RX_CTRL_T prRxCtrl;
    P_QUE_ENTRY_T prQueEntry;
    KAL_SPIN_LOCK_DECLARATION();

    ASSERT(prAdapter);
    ASSERT(prSwRfb);
    prRxCtrl = &prAdapter->rRxCtrl;
    prQueEntry = &prSwRfb->rQueEntry;

    ASSERT(prQueEntry);

    KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);

    if (prSwRfb->pvPacket) {
        QUEUE_INSERT_TAIL(&prRxCtrl->rFreeSwRfbList, prQueEntry);
	} else {
        QUEUE_INSERT_TAIL(&prRxCtrl->rIndicatedRfbList, prQueEntry);
    }

    KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_RX_FREE_QUE);
    return;
} 

VOID nicProcessRxInterrupt(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

#if CFG_SDIO_INTR_ENHANCE
#if CFG_SDIO_RX_AGG
        nicRxSDIOAggReceiveRFBs(prAdapter);
#else
        nicRxSDIOReceiveRFBs(prAdapter);
#endif
#else
    nicRxReceiveRFBs(prAdapter);
#endif 

#if CFG_SUPPORT_MULTITHREAD
	set_bit(GLUE_FLAG_RX_BIT, &(prAdapter->prGlueInfo->ulFlag));
    wake_up_interruptible(&(prAdapter->prGlueInfo->waitq));
#else
    nicRxProcessRFBs(prAdapter);
#endif

    return;

} 


#if CFG_TCP_IP_CHKSUM_OFFLOAD
VOID nicRxUpdateCSUMStatistics(IN P_ADAPTER_T prAdapter, IN const ENUM_CSUM_RESULT_T aeCSUM[]
    )
{
    P_RX_CTRL_T prRxCtrl;

    ASSERT(prAdapter);
    ASSERT(aeCSUM);

    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    if ((aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_SUCCESS) ||
        (aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_SUCCESS)) {

        RX_INC_CNT(prRxCtrl, RX_CSUM_IP_SUCCESS_COUNT);
	} else if ((aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_FAILED) ||
             (aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_FAILED)) {

        RX_INC_CNT(prRxCtrl, RX_CSUM_IP_FAILED_COUNT);
	} else if ((aeCSUM[CSUM_TYPE_IPV4] == CSUM_RES_NONE) &&
             (aeCSUM[CSUM_TYPE_IPV6] == CSUM_RES_NONE)) {

        RX_INC_CNT(prRxCtrl, RX_CSUM_UNKNOWN_L3_PKT_COUNT);
	} else {
        ASSERT(0);
    }

    if (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_SUCCESS) {
        RX_INC_CNT(prRxCtrl, RX_CSUM_TCP_SUCCESS_COUNT);
	} else if (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_FAILED) {
        RX_INC_CNT(prRxCtrl, RX_CSUM_TCP_FAILED_COUNT);
	} else if (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_SUCCESS) {
        RX_INC_CNT(prRxCtrl, RX_CSUM_UDP_SUCCESS_COUNT);
	} else if (aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_FAILED) {
        RX_INC_CNT(prRxCtrl, RX_CSUM_UDP_FAILED_COUNT);
	} else if ((aeCSUM[CSUM_TYPE_UDP] == CSUM_RES_NONE) &&
             (aeCSUM[CSUM_TYPE_TCP] == CSUM_RES_NONE)) {

        RX_INC_CNT(prRxCtrl, RX_CSUM_UNKNOWN_L4_PKT_COUNT);
	} else {
        ASSERT(0);
    }

    return;
} 
#endif 


VOID nicRxQueryStatus(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBuffer, OUT PUINT_32 pu4Count)
{
    P_RX_CTRL_T prRxCtrl;
    PUINT_8 pucCurrBuf = pucBuffer;


    ASSERT(prAdapter);
    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

	
    ASSERT(pu4Count);

    SPRINTF(pucCurrBuf, ("\n\nRX CTRL STATUS:"));
    SPRINTF(pucCurrBuf, ("\n==============="));
    SPRINTF(pucCurrBuf, ("\nFREE RFB w/i BUF LIST :%9ld", prRxCtrl->rFreeSwRfbList.u4NumElem));
	SPRINTF(pucCurrBuf,
		("\nFREE RFB w/o BUF LIST :%9ld", prRxCtrl->rIndicatedRfbList.u4NumElem));
	SPRINTF(pucCurrBuf,
		("\nRECEIVED RFB LIST     :%9ld", prRxCtrl->rReceivedRfbList.u4NumElem));

    SPRINTF(pucCurrBuf, ("\n\n"));

	

    return;
} 


VOID nicRxClearStatistics(IN P_ADAPTER_T prAdapter)
{
    P_RX_CTRL_T prRxCtrl;

    ASSERT(prAdapter);
    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

    RX_RESET_ALL_CNTS(prRxCtrl);
    return;
}


VOID nicRxQueryStatistics(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucBuffer, OUT PUINT_32 pu4Count)
{
    P_RX_CTRL_T prRxCtrl;
    PUINT_8 pucCurrBuf = pucBuffer;

    ASSERT(prAdapter);
    prRxCtrl = &prAdapter->rRxCtrl;
    ASSERT(prRxCtrl);

	
    ASSERT(pu4Count);

#define SPRINTF_RX_COUNTER(eCounter) \
    SPRINTF(pucCurrBuf, ("%-30s : %ld\n", #eCounter, (UINT_32)prRxCtrl->au8Statistics[eCounter]))

    SPRINTF_RX_COUNTER(RX_MPDU_TOTAL_COUNT);
    SPRINTF_RX_COUNTER(RX_SIZE_ERR_DROP_COUNT);
    SPRINTF_RX_COUNTER(RX_DATA_INDICATION_COUNT);
    SPRINTF_RX_COUNTER(RX_DATA_RETURNED_COUNT);
    SPRINTF_RX_COUNTER(RX_DATA_RETAINED_COUNT);

#if CFG_TCP_IP_CHKSUM_OFFLOAD || CFG_TCP_IP_CHKSUM_OFFLOAD_NDIS_60
    SPRINTF_RX_COUNTER(RX_CSUM_TCP_FAILED_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_UDP_FAILED_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_IP_FAILED_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_TCP_SUCCESS_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_UDP_SUCCESS_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_IP_SUCCESS_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_UNKNOWN_L4_PKT_COUNT);
    SPRINTF_RX_COUNTER(RX_CSUM_UNKNOWN_L3_PKT_COUNT);
    SPRINTF_RX_COUNTER(RX_IP_V6_PKT_CCOUNT);
#endif

	

    nicRxClearStatistics(prAdapter);

    return;
}

WLAN_STATUS
nicRxWaitResponse(IN P_ADAPTER_T prAdapter,
    IN UINT_8 ucPortIdx,
		  OUT PUINT_8 pucRspBuffer, IN UINT_32 u4MaxRespBufferLen, OUT PUINT_32 pu4Length)
{
    UINT_32 u4Value = 0, u4PktLen = 0, i = 0;
    WLAN_STATUS u4Status = WLAN_STATUS_SUCCESS;
    BOOL fgResult = TRUE;
    UINT_32 u4Time, u4Current;
    P_RX_CTRL_T prRxCtrl;
    P_WIFI_EVENT_T prEvent;

    DEBUGFUNC("nicRxWaitResponse");

    ASSERT(prAdapter);
    ASSERT(pucRspBuffer);
    ASSERT(ucPortIdx < 2);

    prRxCtrl = &prAdapter->rRxCtrl;

	u4Time = (UINT_32) kalGetTimeTick();

    do {
        
        HAL_MCR_RD(prAdapter, MCR_WRPLR, &u4Value);

        if (!fgResult) {
            DBGLOG(RX, ERROR, ("Read Response Packet Error\n"));
            return WLAN_STATUS_FAILURE;
        }

		if (ucPortIdx == 0) {
            u4PktLen = u4Value & 0xFFFF;
		} else {
            u4PktLen = (u4Value >> 16) & 0xFFFF;
        }

        DBGLOG(RX, TRACE, ("i = %lu, u4PktLen = %lu\n", i, u4PktLen));

        if (u4PktLen == 0) {
            
			u4Current = (UINT_32) kalGetTimeTick();

			if ((u4Current > u4Time) && ((u4Current - u4Time) > RX_RESPONSE_TIMEOUT)) {
                return WLAN_STATUS_FAILURE;
			} else if (u4Current < u4Time
				   && ((u4Current + (0xFFFFFFFF - u4Time)) > RX_RESPONSE_TIMEOUT)) {
                return WLAN_STATUS_FAILURE;
            }

            
            kalUdelay(50);

            i++;
		} else if (u4PktLen > u4MaxRespBufferLen) {
			DBGLOG(RX, WARN,
			       ("Not enough Event Buffer: required length = 0x%lx, available buffer length = %lu\n",
                u4PktLen, u4MaxRespBufferLen));

            return WLAN_STATUS_FAILURE;
		} else {
#if (CFG_ENABLE_READ_EXTRA_4_BYTES == 1)
#if CFG_SDIO_RX_AGG
            HAL_PORT_RD(prAdapter,
                        ucPortIdx == 0 ? MCR_WRDR0 : MCR_WRDR1,
                        ALIGN_4(u4PktLen + 4),
				    prRxCtrl->pucRxCoalescingBufPtr, CFG_RX_COALESCING_BUFFER_SIZE);
            kalMemCopy(pucRspBuffer, prRxCtrl->pucRxCoalescingBufPtr, u4PktLen);
#else
#error "Please turn on RX coalescing"
#endif
#else
            HAL_PORT_RD(prAdapter,
                        ucPortIdx == 0 ? MCR_WRDR0 : MCR_WRDR1,
				    u4PktLen, pucRspBuffer, u4MaxRespBufferLen);
#endif

            
            if (!fgResult) {
                DBGLOG(RX, ERROR, ("Read Response Packet Error\n"));
                return WLAN_STATUS_FAILURE;
            }

			DBGLOG(RX, TRACE, ("Dump Response buffer, length = 0x%lx\n", u4PktLen));
            DBGLOG_MEM8(RX, TRACE, pucRspBuffer, u4PktLen);

			prEvent = (P_WIFI_EVENT_T) pucRspBuffer;
            DBGLOG(INIT, TRACE, ("RX EVENT: ID[0x%02X] SEQ[%u] LEN[%u]\n", 
                prEvent->ucEID,
					     prEvent->ucSeqNum, prEvent->u2PacketLength));

            *pu4Length = u4PktLen;
            break;
        }
	} while (TRUE);

    return u4Status;
}

VOID nicRxEnablePromiscuousMode(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return;
} 


VOID nicRxDisablePromiscuousMode(IN P_ADAPTER_T prAdapter)
{
    ASSERT(prAdapter);

    return;
} 


WLAN_STATUS nicRxFlush(IN P_ADAPTER_T prAdapter)
{
    P_SW_RFB_T prSwRfb;

    ASSERT(prAdapter);
	prSwRfb = qmFlushRxQueues(prAdapter);
	if (prSwRfb != NULL) {
        do {
            P_SW_RFB_T prNextSwRfb;

			
			prNextSwRfb = (P_SW_RFB_T) QUEUE_GET_NEXT_ENTRY((P_QUE_ENTRY_T) prSwRfb);

			
            nicRxReturnRFB(prAdapter, prSwRfb);

            prSwRfb = prNextSwRfb;
		} while (prSwRfb);
    }

    return WLAN_STATUS_SUCCESS;
}


WLAN_STATUS nicRxProcessActionFrame(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfb)
{
    P_WLAN_ACTION_FRAME prActFrame;
#if CFG_SUPPORT_802_11W
    BOOL               fgRobustAction = FALSE;
    P_AIS_SPECIFIC_BSS_INFO_T prAisSpecBssInfo;
#endif

    ASSERT(prAdapter);
    ASSERT(prSwRfb);

    DBGLOG(RSN, TRACE, ("[Rx] nicRxProcessActionFrame\n"));

    if (prSwRfb->u2PacketLen < sizeof(WLAN_ACTION_FRAME) - 1) {
        return WLAN_STATUS_INVALID_PACKET;
    }
    prActFrame = (P_WLAN_ACTION_FRAME) prSwRfb->pvHeader;

	
    
#if CFG_SUPPORT_802_11W
    if ((prActFrame->ucCategory <= CATEGORY_PROTECTED_DUAL_OF_PUBLIC_ACTION && 
         prActFrame->ucCategory != CATEGORY_PUBLIC_ACTION && 
         prActFrame->ucCategory != CATEGORY_HT_ACTION)  || 
        (prActFrame->ucCategory == CATEGORY_VENDOR_SPECIFIC_ACTION_PROTECTED)) {
        fgRobustAction = TRUE;
    }
	

    if (fgRobustAction && prSwRfb->prStaRec && 
	    GET_BSS_INFO_BY_INDEX(prAdapter,
				  prSwRfb->prStaRec->ucBssIndex)->eNetworkType ==
	    NETWORK_TYPE_AIS) {
        prAisSpecBssInfo = &(prAdapter->rWifiVar.rAisSpecificBssInfo);

		DBGLOG(RSN, INFO,
		       ("[Rx]RobustAction %x %x %x\n", prSwRfb->prRxStatus->u2StatusFlag,
			prSwRfb->prRxStatus->ucWlanIdx, prSwRfb->prRxStatus->ucTidSecMode));

		if (prAisSpecBssInfo->fgMgmtProtection
		    && (!(prActFrame->u2FrameCtrl & MASK_FC_PROTECTED_FRAME)
			&& (HAL_RX_STATUS_GET_SEC_MODE(prSwRfb->prRxStatus) ==
			    CIPHER_SUITE_CCMP))) {
			DBGLOG(RSN, INFO,
			       ("[MFP] Not handle and drop un-protected robust action frame!!\n"));
            return WLAN_STATUS_INVALID_PACKET;
        }
    }
	
#endif

    switch (prActFrame->ucCategory) {
#if CFG_M0VE_BA_TO_DRIVER
    case CATEGORY_BLOCK_ACK_ACTION:
        DBGLOG(RX, WARN, ("[Puff][%s] Rx CATEGORY_BLOCK_ACK_ACTION\n", __func__));

        if (prSwRfb->prStaRec) {
            mqmHandleBaActionFrame(prAdapter, prSwRfb);
        }

        break;
#endif
    case CATEGORY_PUBLIC_ACTION:
#if 0				
#else
		if (prAdapter->prAisBssInfo &&
                prSwRfb->prStaRec &&
                prSwRfb->prStaRec->ucBssIndex == prAdapter->prAisBssInfo->ucBssIndex) {
            aisFuncValidateRxActionFrame(prAdapter, prSwRfb);
        }
#endif

		if (prAdapter->prAisBssInfo &&
        prAdapter->prAisBssInfo->ucBssIndex == KAL_NETWORK_TYPE_AIS_INDEX) {
            aisFuncValidateRxActionFrame(prAdapter, prSwRfb);
    }
#if CFG_ENABLE_WIFI_DIRECT
        if (prAdapter->fgIsP2PRegistered) {
            rlmProcessPublicAction(prAdapter, prSwRfb);

			p2pFuncValidateRxActionFrame(prAdapter, prSwRfb);

        }
#endif
        break;

    case CATEGORY_HT_ACTION:
#if CFG_ENABLE_WIFI_DIRECT
        if (prAdapter->fgIsP2PRegistered) {
            rlmProcessHtAction(prAdapter, prSwRfb);
        }
#endif
        break;
    case CATEGORY_VENDOR_SPECIFIC_ACTION:
#if CFG_ENABLE_WIFI_DIRECT
        if (prAdapter->fgIsP2PRegistered) {
            p2pFuncValidateRxActionFrame(prAdapter, prSwRfb);
        }
#endif
        break;
#if CFG_SUPPORT_802_11W
    case CATEGORY_SA_QUERY_ACTION:
        {
            P_BSS_INFO_T prBssInfo;

            if (prSwRfb->prStaRec) {
				prBssInfo =
				    GET_BSS_INFO_BY_INDEX(prAdapter, prSwRfb->prStaRec->ucBssIndex);
                ASSERT(prBssInfo);
                if ((prBssInfo->eNetworkType == NETWORK_TYPE_AIS) &&
				    prAdapter->rWifiVar.rAisSpecificBssInfo.
				    fgMgmtProtection ) {
                    
                    rsnSaQueryAction(prAdapter, prSwRfb);
                }
            }
        }
        break;
#endif
#if CFG_SUPPORT_802_11V
    case CATEGORY_WNM_ACTION:
        {
            wnmWNMAction(prAdapter, prSwRfb);
        }
        break;
#endif

#if CFG_SUPPORT_DFS 
	case CATEGORY_SPEC_MGT:
		{
			
			   DBGLOG(RLM, INFO, ("[Channel Switch]nicRxProcessActionFrame\n"));
			   
			   
			
		}
		break;
#endif
    default:
        break;
    } 


    return WLAN_STATUS_SUCCESS;
}
