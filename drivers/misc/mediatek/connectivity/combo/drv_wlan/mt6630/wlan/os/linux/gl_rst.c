





#include <linux/kernel.h>
#include <linux/workqueue.h>

#include "precomp.h"
#include "gl_rst.h"

#if CFG_CHIP_RESET_SUPPORT

#define CFG_WMT_RESET_API_SUPPORT   1

BOOLEAN fgIsResetting = FALSE;

static RESET_STRUCT_T wifi_rst;

static void mtk_wifi_reset(struct work_struct *work);

static void *glResetCallback(ENUM_WMTDRV_TYPE_T eSrcType,
			     ENUM_WMTDRV_TYPE_T eDstType,
			     ENUM_WMTMSG_TYPE_T eMsgType,
			     void *prMsgBody, unsigned int u4MsgLength);

#if CFG_WMT_RESET_API_SUPPORT
extern MTK_WCN_BOOL mtk_wcn_wmt_do_reset(ENUM_WMTDRV_TYPE_T type);
#endif

VOID glResetInit(VOID)
{
	
	mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_WIFI, (PF_WMT_CB) glResetCallback);

	
	INIT_WORK(&(wifi_rst.rst_work), mtk_wifi_reset);

	return;
}


VOID glResetUninit(VOID)
{
	
	mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_WIFI);

	return;
}


static void *glResetCallback(ENUM_WMTDRV_TYPE_T eSrcType,
			     ENUM_WMTDRV_TYPE_T eDstType,
			     ENUM_WMTMSG_TYPE_T eMsgType, void *prMsgBody, unsigned int u4MsgLength)
{
	switch (eMsgType) {
	case WMTMSG_TYPE_RESET:
		if (u4MsgLength == sizeof(ENUM_WMTRSTMSG_TYPE_T)) {
			P_ENUM_WMTRSTMSG_TYPE_T prRstMsg = (P_ENUM_WMTRSTMSG_TYPE_T) prMsgBody;

			switch (*prRstMsg) {
			case WMTRSTMSG_RESET_START:
				DBGLOG(INIT, WARN, ("Whole chip reset start!\n"));
				fgIsResetting = TRUE;
				wifi_reset_start();
				break;

			case WMTRSTMSG_RESET_END:
				DBGLOG(INIT, WARN, ("Whole chip reset end!\n"));
				fgIsResetting = FALSE;
				wifi_rst.rst_data = RESET_SUCCESS;
				schedule_work(&(wifi_rst.rst_work));
				break;

			case WMTRSTMSG_RESET_END_FAIL:
				DBGLOG(INIT, WARN, ("Whole chip reset fail!\n"));
				fgIsResetting = FALSE;
				wifi_rst.rst_data = RESET_FAIL;
				schedule_work(&(wifi_rst.rst_work));
				break;

			default:
				break;
			}
		}
		break;

	default:
		break;
	}

	return NULL;
}


static void mtk_wifi_reset(struct work_struct *work)
{
	RESET_STRUCT_T *rst = container_of(work, RESET_STRUCT_T, rst_work);
	wifi_reset_end(rst->rst_data);
	return;
}


VOID glSendResetRequest(VOID)
{
	
	return;
}


BOOLEAN kalIsResetting(VOID)
{
	return fgIsResetting;
}

BOOLEAN glResetTrigger(P_ADAPTER_T prAdapter)
{
    BOOLEAN fgResult = TRUE;;

#if CFG_WMT_RESET_API_SUPPORT
    if(kalIsResetting()) {
        DBGLOG(INIT, ERROR, ("Skip triggering whole-chip reset during reseting!" 
            "Chip[%04X E%u] FW Ver DEC[%u.%u] HEX[%x.%x], Driver Ver[%u.%u]\n", 
            MTK_CHIP_REV, 
            wlanGetEcoVersion(prAdapter),
            (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
            (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
            (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
            (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
            (prAdapter->rVerInfo.u2FwPeerVersion >> 8), 
            (prAdapter->rVerInfo.u2FwPeerVersion & BITS(0, 7))));
        
        fgResult = TRUE;
    }
    else {
        DBGLOG(INIT, ERROR, ("Trigger whole-chip reset! Chip[%04X E%u]" 
            "FW Ver DEC[%u.%u] HEX[%x.%x], Driver Ver[%u.%u]\n", 
            MTK_CHIP_REV, 
            wlanGetEcoVersion(prAdapter),
            (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
            (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
            (prAdapter->rVerInfo.u2FwOwnVersion >> 8),
            (prAdapter->rVerInfo.u2FwOwnVersion & BITS(0, 7)),
            (prAdapter->rVerInfo.u2FwPeerVersion >> 8), 
            (prAdapter->rVerInfo.u2FwPeerVersion & BITS(0, 7))));
        
        fgResult = mtk_wcn_wmt_do_reset(WMTDRV_TYPE_WIFI);
    }
#endif

    return fgResult;
}

#endif 
