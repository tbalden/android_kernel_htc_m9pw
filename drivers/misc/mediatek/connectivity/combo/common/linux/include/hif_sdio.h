


#ifndef _HIF_SDIO_H
#define _HIF_SDIO_H
#define HIF_SDIO_DEBUG  (0)	
#define HIF_SDIO_API_EXTENSION      (0)

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <linux/mm.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <asm/atomic.h>

#include "osal_typedef.h"
#include "osal.h"
#include "wmt_exp.h"


#define CFG_CLIENT_COUNT  (11)

#define HIF_DEFAULT_BLK_SIZE  (256)
#define HIF_DEFAULT_VENDOR    (0x037A)

#define HIF_SDIO_LOG_LOUD    4
#define HIF_SDIO_LOG_DBG     3
#define HIF_SDIO_LOG_INFO    2
#define HIF_SDIO_LOG_WARN    1
#define HIF_SDIO_LOG_ERR     0



typedef struct _MTK_WCN_HIF_SDIO_FUNCINFO MTK_WCN_HIF_SDIO_FUNCINFO;

typedef UINT32 MTK_WCN_HIF_SDIO_CLTCTX;

typedef INT32(*MTK_WCN_HIF_SDIO_PROBE) (MTK_WCN_HIF_SDIO_CLTCTX, const MTK_WCN_HIF_SDIO_FUNCINFO * );
typedef INT32(*MTK_WCN_HIF_SDIO_REMOVE) (MTK_WCN_HIF_SDIO_CLTCTX);
typedef INT32(*MTK_WCN_HIF_SDIO_IRQ) (MTK_WCN_HIF_SDIO_CLTCTX);

struct _MTK_WCN_HIF_SDIO_FUNCINFO {
	UINT16 manf_id;		
	UINT16 card_id;		
	UINT16 func_num;	
	UINT16 blk_sz;		
};

typedef struct _MTK_WCN_HIF_SDIO_CLTINFO {
	const MTK_WCN_HIF_SDIO_FUNCINFO *func_tbl;	
	UINT32 func_tbl_size;	
	MTK_WCN_HIF_SDIO_PROBE hif_clt_probe;	
	MTK_WCN_HIF_SDIO_REMOVE hif_clt_remove;	
	MTK_WCN_HIF_SDIO_IRQ hif_clt_irq;	
} MTK_WCN_HIF_SDIO_CLTINFO;

typedef struct _MTK_WCN_HIF_SDIO_REGISTINFO {
	const MTK_WCN_HIF_SDIO_CLTINFO *sdio_cltinfo;	
	const MTK_WCN_HIF_SDIO_FUNCINFO *func_info;	
} MTK_WCN_HIF_SDIO_REGISTINFO;

typedef struct _MTK_WCN_HIF_SDIO_PROBEINFO {
	struct sdio_func *func;	
	PVOID private_data_p;	
	MTK_WCN_BOOL on_by_wmt;	
	
	MTK_WCN_BOOL sdio_irq_enabled;	
	INT32 clt_idx;		
} MTK_WCN_HIF_SDIO_PROBEINFO;

typedef struct _MTK_WCN_HIF_SDIO_CLT_PROBE_WORKERINFO {
	struct work_struct probe_work;	
	MTK_WCN_HIF_SDIO_REGISTINFO *registinfo_p;	
	INT8 probe_idx;		
} MTK_WCN_HIF_SDIO_CLT_PROBE_WORKERINFO;

typedef struct _MTK_WCN_HIF_SDIO_LOCKINFO {
	spinlock_t probed_list_lock;	
	spinlock_t clt_list_lock;	
} MTK_WCN_HIF_SDIO_LOCKINFO;

typedef struct _MTK_WCN_HIF_SDIO_DS_CLT_INFO {
	MTK_WCN_HIF_SDIO_CLTCTX ctx;
	UINT16 func_num;
	UINT8 act_flag;
	UINT8 ds_en_flag;
} MTK_WCN_HIF_SDIO_DS_CLT_INFO;

typedef struct _MTK_WCN_HIF_SDIO_DS_INFO {
	UINT32 chip_id;		
	UINT32 reg_offset;	
	UINT8 value;		
	MTK_WCN_HIF_SDIO_DS_CLT_INFO clt_info[2];	
	struct mutex lock;
} MTK_WCN_HIF_SDIO_DS_INFO;


typedef enum {
	HIF_SDIO_ERR_SUCCESS = 0,
	HIF_SDIO_ERR_FAIL = HIF_SDIO_ERR_SUCCESS - 1,	
	HIF_SDIO_ERR_INVALID_PARAM = HIF_SDIO_ERR_FAIL - 1,
	HIF_SDIO_ERR_DUPLICATED = HIF_SDIO_ERR_INVALID_PARAM - 1,
	HIF_SDIO_ERR_UNSUP_MANF_ID = HIF_SDIO_ERR_DUPLICATED - 1,
	HIF_SDIO_ERR_UNSUP_CARD_ID = HIF_SDIO_ERR_UNSUP_MANF_ID - 1,
	HIF_SDIO_ERR_INVALID_FUNC_NUM = HIF_SDIO_ERR_UNSUP_CARD_ID - 1,
	HIF_SDIO_ERR_INVALID_BLK_SZ = HIF_SDIO_ERR_INVALID_FUNC_NUM - 1,
	HIF_SDIO_ERR_NOT_PROBED = HIF_SDIO_ERR_INVALID_BLK_SZ - 1,
	HIF_SDIO_ERR_ALRDY_ON = HIF_SDIO_ERR_NOT_PROBED - 1,
	HIF_SDIO_ERR_ALRDY_OFF = HIF_SDIO_ERR_ALRDY_ON - 1,
	HIF_SDIO_ERR_CLT_NOT_REG = HIF_SDIO_ERR_ALRDY_OFF - 1,
} MTK_WCN_HIF_SDIO_ERR;





#if WMT_PLAT_ALPS
#ifdef CONFIG_SDIOAUTOK_SUPPORT
#define MTK_HIF_SDIO_AUTOK_ENABLED 1
#else
#define MTK_HIF_SDIO_AUTOK_ENABLED 0
#endif
#else
#define MTK_HIF_SDIO_AUTOK_ENABLED 0
#endif

#define CLTCTX(cid, func, blk_sz, idx) \
    (MTK_WCN_HIF_SDIO_CLTCTX)( (((UINT32)(cid) & 0xFFFFUL) << 16) | \
        (((UINT32)(func) & 0xFUL) << 4) | \
        (((UINT32)(blk_sz) & 0xFF00UL) << 0) | \
        (((UINT32)idx & 0xFUL) << 0) )

#define CLTCTX_CID(ctx) (((ctx) >> 16) & 0xFFFF)
#define CLTCTX_FUNC(ctx) (((ctx) >> 4) & 0xF)
#define CLTCTX_BLK_SZ(ctx) (((ctx) >> 0) & 0xFF00)
#define CLTCTX_IDX(ctx) ((ctx) & 0xF)
#define CLTCTX_IDX_VALID(idx) ((idx >= 0) && (idx < CFG_CLIENT_COUNT))


#define MTK_WCN_HIF_SDIO_FUNC(manf, card, func, b_sz) \
    .manf_id = (manf), .card_id = (card), .func_num = (func), .blk_sz = (b_sz)

#ifdef DFT_TAG
#undef DFT_TAG
#endif

#ifndef DFT_TAG
#define DFT_TAG         "[HIF-SDIO]"
#endif

extern UINT32 gHifSdioDbgLvl;

#define HIF_SDIO_LOUD_FUNC(fmt, arg...)   if (gHifSdioDbgLvl >= HIF_SDIO_LOG_LOUD) { osal_dbg_print(DFT_TAG"[L]%s:"  fmt, __FUNCTION__ ,##arg);}
#define HIF_SDIO_DBG_FUNC(fmt, arg...)    if (gHifSdioDbgLvl >= HIF_SDIO_LOG_DBG) { osal_dbg_print(DFT_TAG"[D]%s:"  fmt, __FUNCTION__ ,##arg);}
#define HIF_SDIO_INFO_FUNC(fmt, arg...)   if (gHifSdioDbgLvl >= HIF_SDIO_LOG_INFO) { osal_info_print(DFT_TAG"[I]%s:"  fmt, __FUNCTION__ ,##arg);}
#define HIF_SDIO_WARN_FUNC(fmt, arg...)   if (gHifSdioDbgLvl >= HIF_SDIO_LOG_WARN) { osal_warn_print(DFT_TAG"[W]%s(%d):"  fmt, __FUNCTION__ , __LINE__, ##arg);}
#define HIF_SDIO_ERR_FUNC(fmt, arg...)    if (gHifSdioDbgLvl >= HIF_SDIO_LOG_ERR) { osal_err_print(DFT_TAG"[E]%s(%d):"  fmt, __FUNCTION__ , __LINE__, ##arg);}

#if HIF_SDIO_DEBUG
#define HIF_SDIO_ASSERT(expr)    if (!(expr)) { \
			    osal_dbg_print("assertion failed! %s[%d]: %s\n",\
                                __FUNCTION__, __LINE__, #expr); \
			    osal_bug_on(!(expr));\
			}
#else
#define HIF_SDIO_ASSERT(expr)    do {} while (0)
#endif


extern INT32 mtk_wcn_hif_sdio_client_reg(const MTK_WCN_HIF_SDIO_CLTINFO *pinfo);

extern INT32 mtk_wcn_hif_sdio_client_unreg(const MTK_WCN_HIF_SDIO_CLTINFO *pinfo);

extern INT32 mtk_wcn_hif_sdio_readb(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, PUINT8 pvb);

extern INT32 mtk_wcn_hif_sdio_writeb(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, UINT8 vb);

extern INT32 mtk_wcn_hif_sdio_readl(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, PUINT32 pvl);

extern INT32 mtk_wcn_hif_sdio_writel(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, UINT32 vl);

extern INT32 mtk_wcn_hif_sdio_read_buf(MTK_WCN_HIF_SDIO_CLTCTX ctx,
				       UINT32 offset, PUINT32 pbuf, UINT32 len);

extern INT32 mtk_wcn_hif_sdio_write_buf(MTK_WCN_HIF_SDIO_CLTCTX ctx,
					UINT32 offset, PUINT32 pbuf, UINT32 len);

extern VOID mtk_wcn_hif_sdio_set_drvdata(MTK_WCN_HIF_SDIO_CLTCTX ctx, PVOID private_data_p);

extern PVOID mtk_wcn_hif_sdio_get_drvdata(MTK_WCN_HIF_SDIO_CLTCTX ctx);

extern INT32 mtk_wcn_hif_sdio_wmt_control(WMT_SDIO_FUNC_TYPE func_type, MTK_WCN_BOOL is_on);

extern INT32 mtk_wcn_hif_sdio_bus_set_power(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 pwrState);

extern VOID mtk_wcn_hif_sdio_get_dev(MTK_WCN_HIF_SDIO_CLTCTX ctx, struct device **dev);

extern INT32 mtk_wcn_hif_sdio_update_cb_reg(INT32 (*ts_update) (VOID)
    );

extern VOID mtk_wcn_hif_sdio_enable_irq(MTK_WCN_HIF_SDIO_CLTCTX ctx, MTK_WCN_BOOL enable);

extern INT32 mtk_wcn_hif_sdio_do_autok(MTK_WCN_HIF_SDIO_CLTCTX ctx);

extern INT32 mtk_wcn_hif_sdio_f0_writeb(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, UINT8 vb);

extern INT32 mtk_wcn_hif_sdio_f0_readb(MTK_WCN_HIF_SDIO_CLTCTX ctx, UINT32 offset, PUINT8 pvb);


#define DELETE_HIF_SDIO_CHRDEV 1
#if !(DELETE_HIF_SDIO_CHRDEV)
INT32 mtk_wcn_hif_sdio_tell_chipid(INT32 chipId);
INT32 mtk_wcn_hif_sdio_query_chipid(INT32 waitFlag);
#endif

extern INT32 mtk_wcn_hif_sdio_en_deep_sleep(MTK_WCN_HIF_SDIO_CLTCTX ctx);

extern INT32 mtk_wcn_hif_sdio_dis_deep_sleep(MTK_WCN_HIF_SDIO_CLTCTX ctx);



#if MTK_HIF_SDIO_AUTOK_ENABLED
extern int wait_sdio_autok_ready(PVOID);
#endif


#endif				
