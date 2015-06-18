


#ifndef _WMT_LIB_H_
#define _WMT_LIB_H_


#include "wmt_core.h"
#include "wmt_exp.h"
#include <mach/mtk_wcn_cmb_stub.h>
#include "stp_wmt.h"
#include "wmt_plat.h"
#include "wmt_idc.h"
#include "osal.h"



#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
#define USE_NEW_PROC_FS_FLAG 1
#else
#define USE_NEW_PROC_FS_FLAG 0
#endif



#define WMT_OP_BUF_SIZE (16)
#if 0				
#define WMT_LOG_LOUD    4
#define WMT_LOG_DBG     3
#define WMT_LOG_INFO    2
#define WMT_LOG_WARN    1
#define WMT_LOG_ERR     0
#endif
typedef enum _ENUM_WMTRSTRET_TYPE_T {
	WMTRSTRET_SUCCESS = 0x0,
	WMTRSTRET_FAIL = 0x1,
	WMTRSTRET_ONGOING = 0x2,
	WMTRSTRET_MAX
} ENUM_WMTRSTRET_TYPE_T, *P_ENUM_WMTRSTRET_TYPE_T;

#define MAX_ANT_RAM_CODE_DOWN_TIME 3000
#define WMT_LIB_RX_TIMEOUT 2000	
#define MAX_WIFI_ON_TIME 5500

#define WMT_PWRON_RTY_DFT 2
#define MAX_RETRY_TIME_DUE_TO_RX_TIMEOUT WMT_PWRON_RTY_DFT * WMT_LIB_RX_TIMEOUT
#define MAX_EACH_FUNC_ON_WHEN_CHIP_POWER_ON_ALREADY WMT_LIB_RX_TIMEOUT	
#define MAX_FUNC_ON_TIME (MAX_WIFI_ON_TIME + MAX_RETRY_TIME_DUE_TO_RX_TIMEOUT + MAX_EACH_FUNC_ON_WHEN_CHIP_POWER_ON_ALREADY * 3 + MAX_ANT_RAM_CODE_DOWN_TIME)

#define MAX_EACH_FUNC_OFF WMT_LIB_RX_TIMEOUT + 1000	
#define MAX_FUNC_OFF_TIME MAX_EACH_FUNC_OFF * 4

#define MAX_EACH_WMT_CMD WMT_LIB_RX_TIMEOUT + 1000	

#define MAX_GPIO_CTRL_TIME (2000)	

#if 0				
extern UINT32 gWmtDbgLvl;
#ifndef DFT_TAG
#define DFT_TAG         "[WMT-DFT]"
#endif

#define WMT_LOUD_FUNC(fmt, arg...)   if (gWmtDbgLvl >= WMT_LOG_LOUD) { osal_dbg_print(DFT_TAG "[L]%s:"  fmt, __FUNCTION__ ,##arg);}
#define WMT_INFO_FUNC(fmt, arg...)   if (gWmtDbgLvl >= WMT_LOG_INFO) { osal_info_print(DFT_TAG "[I]%s:"  fmt, __FUNCTION__ ,##arg);}
#define WMT_WARN_FUNC(fmt, arg...)   if (gWmtDbgLvl >= WMT_LOG_WARN) { osal_warn_print(DFT_TAG "[W]%s:"  fmt, __FUNCTION__ ,##arg);}
#define WMT_ERR_FUNC(fmt, arg...)    if (gWmtDbgLvl >= WMT_LOG_ERR) { osal_err_print(DFT_TAG "[E]%s(%d):"  fmt, __FUNCTION__ , __LINE__, ##arg);}
#define WMT_DBG_FUNC(fmt, arg...)    if (gWmtDbgLvl >= WMT_LOG_DBG) { osal_dbg_print(DFT_TAG "[D]%s:"  fmt, __FUNCTION__ ,##arg);}
#define WMT_TRC_FUNC(f)              if (gWmtDbgLvl >= WMT_LOG_DBG) { osal_dbg_print(DFT_TAG "<%s> <%d>\n", __FUNCTION__, __LINE__);}
#endif


#define WMT_LIB_AIF_FLAG_MASK (0x1UL)
#define WMT_LIB_AIF_FLAG_SHARE (0x1UL << 0)
#define WMT_LIB_AIF_FLAG_SEPARATE (0x0UL << 0)


typedef enum {
	WMT_STAT_PWR = 0,	
	WMT_STAT_STP_REG = 1,	
	WMT_STAT_STP_OPEN = 2,	
	WMT_STAT_STP_EN = 3,	
	WMT_STAT_STP_RDY = 4,	
	WMT_STAT_RX = 5,	
	WMT_STAT_CMD = 6,	
	WMT_STAT_SDIO1_ON = 7,	
	WMT_STAT_SDIO2_ON = 8,	
	WMT_STAT_SDIO_WIFI_ON = 9,	
	WMT_STAT_SDIO_STP_ON = 10,	
	WMT_STAT_RST_ON = 11,
	WMT_STAT_MAX
} WMT_STAT;

typedef enum _ENUM_WMTRSTSRC_TYPE_T {
	WMTRSTSRC_RESET_BT = 0x0,
	WMTRSTSRC_RESET_FM = 0x1,
	WMTRSTSRC_RESET_GPS = 0x2,
	WMTRSTSRC_RESET_WIFI = 0x3,
	WMTRSTSRC_RESET_STP = 0x4,
	WMTRSTSRC_RESET_TEST = 0x5,
	WMTRSTSRC_RESET_MAX
} ENUM_WMTRSTSRC_TYPE_T, *P_ENUM_WMTRSTSRC_TYPE_T;


typedef struct {
	PF_WMT_CB fDrvRst[WMTDRV_TYPE_MAX];
} WMT_FDRV_CB, *P_WMT_FDRV_CB;


typedef struct {
	UINT32 dowloadSeq;
	UINT8 addRess[4];
	UINT8 patchName[256];
} WMT_PATCH_INFO, *P_WMT_PATCH_INFO;


typedef struct _DEV_WMT_ {

	OSAL_SLEEPABLE_LOCK psm_lock;
	OSAL_SLEEPABLE_LOCK idc_lock;
	
 
	OSAL_THREAD thread;
 
	OSAL_EVENT rWmtdWq;	
	 
	OSAL_BIT_OP_VAR state;

	
 
	OSAL_EVENT rWmtRxWq;	
 
	WMT_FDRV_CB rFdrvCb;

	
	WMT_HIF_CONF rWmtHifConf;
	WMT_GEN_CONF rWmtGenConf;

	
	UINT8 cPatchName[NAME_MAX + 1];
	UINT8 cFullPatchName[NAME_MAX + 1];
	UINT32 patchNum;
	const osal_firmware *pPatch;

	UINT8 cWmtcfgName[NAME_MAX + 1];
	const osal_firmware *pWmtCfg;

	const osal_firmware *pNvram;

	
	INT8 cUartName[NAME_MAX + 1];

	OSAL_OP_Q rFreeOpQ;	
	OSAL_OP_Q rActiveOpQ;	
	OSAL_OP arQue[WMT_OP_BUF_SIZE];	
	P_OSAL_OP pCurOP;	

	
	UINT8 cCmd[NAME_MAX + 1];
	INT32 cmdResult;
 
	OSAL_SIGNAL cmdResp;
	OSAL_EVENT cmdReq;

	
	
	UINT32 chip_id;
	UINT32 hw_ver;
	UINT32 fw_ver;
	
	
	ENUM_WMTHWVER_TYPE_T eWmtHwVer;

	P_WMT_PATCH_INFO pWmtPatchInfo;
} DEV_WMT, *P_DEV_WMT;



extern INT32 wmt_lib_init(VOID);
extern INT32 wmt_lib_deinit(VOID);
extern INT32 wmt_lib_tx(PUINT8 data, UINT32 size, PUINT32 writtenSize);
extern INT32 wmt_lib_tx_raw(PUINT8 data, UINT32 size, PUINT32 writtenSize);
extern INT32 wmt_lib_rx(PUINT8 buff, UINT32 buffLen, PUINT32 readSize);
extern VOID wmt_lib_flush_rx(VOID);

#if CFG_WMT_PS_SUPPORT
extern INT32 wmt_lib_ps_set_idle_time(UINT32 psIdleTime);
extern INT32 wmt_lib_ps_init(VOID);
extern INT32 wmt_lib_ps_deinit(VOID);
extern INT32 wmt_lib_ps_enable(VOID);
extern INT32 wmt_lib_ps_ctrl(UINT32 state);

extern INT32 wmt_lib_ps_disable(VOID);
extern VOID wmt_lib_ps_irq_cb(VOID);
#endif
extern VOID wmt_lib_ps_set_sdio_psop(PF_WMT_SDIO_PSOP own_cb);

extern P_OSAL_OP wmt_lib_get_free_op(VOID);
extern INT32 wmt_lib_put_op_to_free_queue(P_OSAL_OP pOp);
extern MTK_WCN_BOOL wmt_lib_put_act_op(P_OSAL_OP pOp);

extern UINT32 wmt_lib_get_icinfo(ENUM_WMT_CHIPINFO_TYPE_T type);

extern MTK_WCN_BOOL wmt_lib_is_therm_ctrl_support(VOID);
extern MTK_WCN_BOOL wmt_lib_is_dsns_ctrl_support(VOID);
extern INT32 wmt_lib_trigger_cmd_signal(INT32 result);
extern PUINT8 wmt_lib_get_cmd(VOID);
extern P_OSAL_EVENT wmt_lib_get_cmd_event(VOID);
extern INT32 wmt_lib_set_patch_name(PUINT8 cPatchName);
extern INT32 wmt_lib_set_uart_name(PINT8 cUartName);
extern INT32 wmt_lib_set_hif(unsigned long hifconf);
extern P_WMT_HIF_CONF wmt_lib_get_hif(VOID);
extern MTK_WCN_BOOL wmt_lib_get_cmd_status(VOID);

#if 0				
extern INT32 wmt_lib_set_aif(CMB_STUB_AIF_X aif, MTK_WCN_BOOL share);	
#endif
extern INT32 wmt_lib_host_awake_get(VOID);
extern INT32 wmt_lib_host_awake_put(VOID);
extern UINT32 wmt_lib_dbg_level_set(UINT32 level);

extern INT32 wmt_lib_msgcb_reg(ENUM_WMTDRV_TYPE_T eType, PF_WMT_CB pCb);

extern INT32 wmt_lib_msgcb_unreg(ENUM_WMTDRV_TYPE_T eType);
ENUM_WMTRSTRET_TYPE_T wmt_lib_cmb_rst(ENUM_WMTRSTSRC_TYPE_T src);
MTK_WCN_BOOL wmt_lib_sw_rst(INT32 baudRst);
MTK_WCN_BOOL wmt_lib_hw_rst(VOID);
INT32 wmt_lib_reg_rw(UINT32 isWrite, UINT32 offset, PUINT32 pvalue, UINT32 mask);
INT32 wmt_lib_efuse_rw(UINT32 isWrite, UINT32 offset, PUINT32 pvalue, UINT32 mask);
INT32 wmt_lib_sdio_ctrl(UINT32 on);

extern INT32 DISABLE_PSM_MONITOR(VOID);
extern VOID ENABLE_PSM_MONITOR(VOID);
extern INT32 wmt_lib_notify_stp_sleep(VOID);
extern VOID wmt_lib_psm_lock_release(VOID);
extern INT32 wmt_lib_psm_lock_aquire(VOID);
extern VOID wmt_lib_idc_lock_release(VOID);
extern INT32 wmt_lib_idc_lock_aquire(VOID);
extern INT32 wmt_lib_set_stp_wmt_last_close(UINT32 value);

extern VOID wmt_lib_set_patch_num(unsigned long num);
extern VOID wmt_lib_set_patch_info(P_WMT_PATCH_INFO pPatchinfo);

extern INT32 wmt_lib_set_current_op(P_DEV_WMT pWmtDev, P_OSAL_OP pOp);
extern P_OSAL_OP wmt_lib_get_current_op(P_DEV_WMT pWmtDev);

extern INT32 wmt_lib_merge_if_flag_ctrl(UINT32 enable);
extern INT32 wmt_lib_merge_if_flag_get(UINT32 enable);

extern PUINT8 wmt_lib_get_cpupcr_xml_format(PUINT32 len);
extern UINT32 wmt_lib_set_host_assert_info(UINT32 type, UINT32 reason, UINT32 en);
#if CFG_WMT_LTE_COEX_HANDLING
extern MTK_WCN_BOOL wmt_lib_handle_idc_msg(ipc_ilm_t *idc_infor);
#endif
#if WMT_FOR_SDIO_1V_AUTOK
extern UINT32 wmt_lib_get_drv_status(UINT32 type);
#endif
extern INT32 wmt_lib_tm_temp_query(VOID);
extern INT32 wmt_lib_trigger_reset(VOID);

#endif				
