





#define STP_SDIO_DBG_SUPPORT 1
#define STP_SDIO_RXDBG 1	
#define STP_SDIO_TXDBG 1	
#define STP_SDIO_TXPERFDBG 1	
#define STP_SDIO_OWNBACKDBG 1	
#define STP_SDIO_NEW_IRQ_HANDLER 1

#include <linux/mm.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include "osal.h"
#include "stp_exp.h"
#include "hif_sdio.h"
#include "stp_sdio.h"
#include "wmt_lib.h"


#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG "[STP SDIO]"

#define STPSDIO_LOG_LOUD    5
#define STPSDIO_LOG_DBG     4
#define STPSDIO_LOG_HINT    3
#define STPSDIO_LOG_INFO    2
#define STPSDIO_LOG_WARN    1
#define STPSDIO_LOG_ERR     0

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG
#define STP_SDIO_RXDBG_COUNT (0x10UL)
#define STP_SDIO_RXDBG_COUNT_MASK (STP_SDIO_RXDBG_COUNT - 1)
struct stp_sdio_rxdbg {
	UINT32 ts;
	UINT32 bus_rxlen;
	UINT32 chisr_rxlen;
	UINT8 rx_pkt_buf[STP_SDIO_RX_BUF_SIZE];
};
#endif

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
#define STP_SDIO_TXDBG_COUNT (0x10UL)
#define STP_SDIO_TXDBG_COUNT_MASK (STP_SDIO_TXDBG_COUNT - 1)
struct stp_sdio_txdbg {
	UINT32 ts;
	UINT32 bus_txlen;
	UINT32 four_byte_align_len;
	UINT8 tx_pkt_buf[STP_SDIO_TX_ENTRY_SIZE];
};
#define STP_SDIO_TXDBG_MAX_SIZE (0x20)
#endif


static INT32 stp_sdio_irq(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx);
static INT32
stp_sdio_probe(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx,
	       const MTK_WCN_HIF_SDIO_FUNCINFO *sdio_func_infop);
static INT32 stp_sdio_remove(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx);

#if STP_SDIO_DBG_SUPPORT && (STP_SDIO_TXDBG || STP_SDIO_TXPERFDBG)
static VOID stp_sdio_txdbg_dump(VOID);
#endif

static VOID stp_sdio_rx_wkr(struct work_struct *work);
static VOID stp_sdio_tx_wkr(struct work_struct *work);
#if STP_SDIO_OWN_THREAD
static INT32 _stp_sdio_wait_for_msg(PVOID pvData);
static VOID stp_sdio_tx_rx_handling(PVOID pData);
#endif
static INT32 stp_sdio_host_info_deinit(PPUINT8 ppTxBuf, PPUINT8 ppRxBuf);
static INT32 stp_sdio_host_info_init(PPUINT8 ppTxBuf, PPUINT8 ppRxBuf);
static INT32 stp_sdio_host_info_op(INT32 opId);
static INT32 stp_sdio_rc_reg_readl_retry (MTK_WCN_HIF_SDIO_CLTCTX clt_ctx, UINT32 offset, UINT32 retry_limit, INT32 *p_ret);

static const MTK_WCN_HIF_SDIO_FUNCINFO mtk_stp_sdio_id_tbl[] = {
	
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x018B, 1, STP_SDIO_BLK_SIZE)},
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x018C, 1, STP_SDIO_BLK_SIZE)},	

	
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x6619, 1, STP_SDIO_BLK_SIZE)},

	
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x020B, 1, STP_SDIO_BLK_SIZE)},
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x020C, 1, STP_SDIO_BLK_SIZE)},	

	
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x6628, 2, STP_SDIO_BLK_SIZE)},	

	
	{MTK_WCN_HIF_SDIO_FUNC(0x037A, 0x6630, 2, STP_SDIO_BLK_SIZE)},	
	{  },
};

wait_queue_head_t g_ownback_done;
INT32 is_wait_ownback = 0;

const MTK_WCN_HIF_SDIO_CLTINFO g_stp_sdio_cltinfo = {
	.func_tbl = mtk_stp_sdio_id_tbl,
	.func_tbl_size = (sizeof(mtk_stp_sdio_id_tbl) / sizeof(MTK_WCN_HIF_SDIO_FUNCINFO) - 1),
	.hif_clt_irq = stp_sdio_irq,
	.hif_clt_probe = stp_sdio_probe,
	.hif_clt_remove = stp_sdio_remove,
};

MTK_WCN_STP_SDIO_HIF_INFO g_stp_sdio_host_info;	
MTK_WCN_STP_SDIO_HIF_INFO *const gp_info = &g_stp_sdio_host_info;

UINT32 g_stp_sdio_host_count;

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG
#define STP_SDIO_RXDBG_PROCNAME "driver/stp_sdio_rxdbg"
static struct proc_dir_entry *gStpSdioRxDbgEntry = NULL;
static INT32 stp_sdio_rxdbg_cnt;
static struct stp_sdio_rxdbg stp_sdio_rxdbg_buffer[STP_SDIO_RXDBG_COUNT];

#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_rxdbg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t stp_sdio_rxdbg_write(struct file *filp, const char __user *buf, size_t count,
			     loff_t *f_pos);
static struct file_operations stp_sdio_rxdbg_fops = {
	.read = stp_sdio_rxdbg_read,
	.write = stp_sdio_rxdbg_write,
};
#endif

#endif
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_OWNBACKDBG
#define STP_SDIO_OWNDBG_PROCNAME "driver/stp_sdio_own"
static struct proc_dir_entry *gStpSdioOwnEntry = NULL;
#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_own_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t stp_sdio_own_write(struct file *filp, const char __user *buf, size_t count,
			   loff_t *f_pos);
static struct file_operations stp_sdio_own_fops = {
	.read = stp_sdio_own_read,
	.write = stp_sdio_own_write,
};
#endif

#endif
#if STP_SDIO_DBG_SUPPORT && (STP_SDIO_TXDBG || STP_SDIO_TXPERFDBG)
#define STP_SDIO_TXDBG_PROCNAME "driver/stp_sdio_txdbg"
static struct proc_dir_entry *gStpSdioTxDbgEntry = NULL;

#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_txdbg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t stp_sdio_txdbg_write(struct file *filp, const char __user *buf, size_t count,
			     loff_t *f_pos);
static struct file_operations stp_sdio_txdbg_fops = {
	.read = stp_sdio_txdbg_read,
	.write = stp_sdio_txdbg_write,
};
#endif


#if STP_SDIO_TXDBG
static INT32 stp_sdio_txdbg_cnt;
static struct stp_sdio_txdbg stp_sdio_txdbg_buffer[STP_SDIO_TXDBG_COUNT];
#endif

#if STP_SDIO_TXPERFDBG
static UINT32 stp_sdio_txperf_worker_cnt;

static UINT32 stp_sdio_txperf_fifo_left;
static UINT32 stp_sdio_txperf_to_send;
static UINT32 stp_sdio_txperf_fifo_lmt_cnt;

static UINT32 stp_sdio_txperf_txed_pkt_num;
static UINT32 stp_sdio_txperf_pkt_num_lmt_cnt;
#endif
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MediaTek Inc WCN_SE_CS3");
MODULE_DESCRIPTION("Read-Copy Update tracing for hierarchical implementation");

UINT32 gStpSdioDbgLvl = STPSDIO_LOG_INFO;

#define STPSDIO_LOUD_FUNC(fmt, arg...) \
	if (gStpSdioDbgLvl >= STPSDIO_LOG_LOUD) \
		do { \
			pr_err(DFT_TAG "[L]%s:"  fmt, __func__ , ##arg); \
		} while (0)

#define STPSDIO_DBG_FUNC(fmt, arg...) \
	if (gStpSdioDbgLvl >= STPSDIO_LOG_DBG) \
		do { \
			pr_err(DFT_TAG "[D]%s:"  fmt, __func__ , ##arg); \
		} while (0)

#define STPSDIO_HINT_FUNC(fmt, arg...) \
	if (gStpSdioDbgLvl >= STPSDIO_LOG_HINT) \
		do { \
			pr_err(DFT_TAG "[I]%s:"	fmt, __func__ , ##arg); \
		} while (0)

#define STPSDIO_INFO_FUNC(fmt, arg...) \
	if (gStpSdioDbgLvl >= STPSDIO_LOG_INFO) \
		do { \
			pr_err(DFT_TAG "[I]%s:"  fmt, __func__ , ##arg); \
		} while (0)

#define STPSDIO_WARN_FUNC(fmt, arg...)	\
	if (gStpSdioDbgLvl >= STPSDIO_LOG_WARN) \
		do { \
			pr_err(DFT_TAG "[W]%s:"  fmt, __func__ , ##arg); \
		} while (0)

#define STPSDIO_ERR_FUNC(fmt, arg...)	\
	if (gStpSdioDbgLvl >= STPSDIO_LOG_ERR) \
		do { \
			pr_err(DFT_TAG "[E]%s(%d):"  fmt, __func__ , __LINE__, ##arg); \
		} while (0)

#define STPSDIO_TRC_FUNC(f) \
	if (gStpSdioDbgLvl >= STPSDIO_LOG_DBG) \
		do { \
			pr_err(DFT_TAG "<%s> <%d>\n", __func__, __LINE__); \
		} while (0)



INT32 stp_sdio_host_info_op(INT32 opId)
{
	INT32 iRet = 0;
#if KMALLOC_UPDATE
    static PUINT8 p_tx_buffer = NULL;
    static PUINT8 p_rx_buffer = NULL;
	if (NULL == p_tx_buffer) {
		p_tx_buffer = kmalloc(STP_SDIO_TX_BUF_CNT * STP_SDIO_TX_ENTRY_SIZE, GFP_ATOMIC);
		if (NULL == p_tx_buffer) {
			STPSDIO_ERR_FUNC
			    ("memory allocate for g_stp_sdio_host_info.pkt_buf.tx_buf fail!\n");
			iRet = -1;
		} else {
			STPSDIO_INFO_FUNC
			    ("memory allocate for g_stp_sdio_host_info.pkt_buf.tx_buf succeed!\n");
			iRet = 0;
		}
	} else {
		STPSDIO_INFO_FUNC
		    ("memory already allocated for g_stp_sdio_host_info.pkt_buf.tx_buf!\n");
		iRet = 0;
	}
	if (NULL == p_rx_buffer) {
		p_rx_buffer = kmalloc(STP_SDIO_RX_BUF_SIZE, GFP_ATOMIC);
		if (NULL == p_rx_buffer) {
			STPSDIO_ERR_FUNC
			    ("memory allocate for g_stp_sdio_host_info.pkt_buf.rx_buf fail!\n");
			iRet = -1;
		} else {
			STPSDIO_INFO_FUNC
			    ("memory allocate for g_stp_sdio_host_info.pkt_buf.rx_buf succeed!\n");
			iRet = 0;
		}
	} else {
		STPSDIO_INFO_FUNC
		    ("memory already allocated for g_stp_sdio_host_info.pkt_buf.rx_buf!\n");
		iRet = 0;
	}

#endif
	if (0 == iRet && 0 == opId) {
		iRet = stp_sdio_host_info_deinit(&p_tx_buffer, &p_rx_buffer);
	} else if (0 == iRet) {
		iRet = stp_sdio_host_info_init(&p_tx_buffer, &p_rx_buffer);
	} else {
		STPSDIO_ERR_FUNC("iRet (%d)!\n", iRet);
	}

	return iRet;
}

INT32 stp_sdio_host_info_init(PPUINT8 ppTxBuf, PPUINT8 ppRxBuf)
{
	
	memset(&g_stp_sdio_host_info, 0, sizeof(g_stp_sdio_host_info));

#if KMALLOC_UPDATE
	g_stp_sdio_host_info.pkt_buf.tx_buf = *ppTxBuf;
	g_stp_sdio_host_info.pkt_buf.rx_buf = *ppRxBuf;
#endif
	return 0;
}

INT32 stp_sdio_host_info_deinit(PPUINT8 ppTxBuf, PPUINT8 ppRxBuf)
{
	STPSDIO_DBG_FUNC("clear g_stp_sdio_host_count end\n");
#if KMALLOC_UPDATE
	if (NULL != *ppTxBuf) {
		kfree(*ppTxBuf);
		*ppTxBuf = NULL;
	}
	if (NULL != *ppRxBuf) {
		kfree(*ppRxBuf);
		*ppRxBuf = NULL;
	}
#endif
	return 0;
}
INT32 _stp_sdio_issue_fake_coredump (UINT8 *str)
{
	#define MAX_STRING_LENGTH 64
	UINT8 AssertStr[4 + MAX_STRING_LENGTH + 1 + 2] = {0};
	UINT32 length = strlen (str) >= MAX_STRING_LENGTH ? MAX_STRING_LENGTH : strlen (str);
	
	
	AssertStr[0] = 0x80;
	AssertStr[1] = 0x50;
	AssertStr[2] = (length + 1) & 0xff;
	AssertStr[3] = 0x0;
	
	memcpy (&AssertStr[4], str, length);
	
	AssertStr[4 + length] = '\n';
	
	AssertStr[4 + length + 1] = 0x0;
	AssertStr[4 + length + 2] = 0x0;
	
	mtk_wcn_stp_parser_data(&AssertStr[0], 4 + length + 1 + 2);
	
	
	length = strlen ("coredump end");
	AssertStr[0] = 0x80;
	AssertStr[1] = 0x50;
	AssertStr[2] = (length + 2) & 0xff;
	AssertStr[3] = 0x0;
	memcpy (&AssertStr[4], "coredump end", length);
	
	AssertStr[4 + length] = '\r';
	AssertStr[4 + length + 1] = '\n';
	
	AssertStr[4 + length + 2] = 0x0;
	AssertStr[4 + length + 3] = 0x0;
	mtk_wcn_stp_parser_data(&AssertStr[0], 4 + length + 2 + 2);
	
	STPSDIO_ERR_FUNC("trigger fake coredump with str:[%s] finished\n", str);
	return 0;
}


static SDIO_PS_OP stp_sdio_get_own_state(VOID)
{
	SDIO_PS_OP ret = OWN_SET;
	INT32 i_ret = 0;
	UINT32 chlcpr_value = 0x0;
	MTK_WCN_HIF_SDIO_CLTCTX clt_ctx;

	clt_ctx = gp_info->sdio_cltctx;
    i_ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
    if (i_ret) {
		ret = OWN_SET;
		STPSDIO_ERR_FUNC("read CHLPCR fail(%d), return\n", ret);
		return ret;
	}

	if ((chlcpr_value & C_FW_COM_DRV_OWN) == C_FW_COM_DRV_OWN) {
		ret = OWN_CLR;
	} else {
		ret = OWN_SET;
	}

	return ret;
}


static INT32 stp_sdio_do_own_clr(INT32 wait)
{
#define CLR_OWN_RETRY 50
	INT32 ret = -1;
	UINT32 time_out;
	INT32 retry;
	UINT32 chlcpr_value = 0x0;
	UINT32 delay_us = 500;
	MTK_WCN_HIF_SDIO_CLTCTX clt_ctx;

	clt_ctx = gp_info->sdio_cltctx;
	retry = 40;

	
	STPSDIO_DBG_FUNC("Do FW-Own back!(Wakeup)\n");
	if (wait) {
		
		init_waitqueue_head(&g_ownback_done);

		is_wait_ownback = 1;
	}
   while (retry-- > 0) {
#if COHEC_00006052
	    ret = mtk_wcn_hif_sdio_writeb(clt_ctx, (UINT32)(CHLPCR+0x1), (UINT8)(C_FW_OWN_REQ_CLR>>8));
#else
	    ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_OWN_REQ_CLR);
#endif 
	    if (ret) {
	        STPSDIO_ERR_FUNC("request firmware own back fail(%d)\n", ret);
	        osal_dbg_assert_aee("<HIF_SDIO> sdio_write ERROR","write CHLPCR by SDIO report error");
			if (0 == retry)
	        goto out;
	    } else
                break;
   	};
	retry = 1200;
	do {
		ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
		if (ret) {
			
            STPSDIO_ERR_FUNC("get CHLPCR information rx error!(%d)\n", ret);
			goto out;
		}

		if ((chlcpr_value & C_FW_COM_DRV_OWN) == C_FW_COM_DRV_OWN) {
            
            STPSDIO_DBG_FUNC("firmware ownback is polled!(%d)\n", CLR_OWN_RETRY - retry);
            udelay(2000);
			break;
		} else {
			STPSDIO_DBG_FUNC
			    ("firmware ownback is no polled, wait for (%d us) and retry\n",
			     delay_us);
			udelay(delay_us);
		}
		if (0 == (retry - 1)%40)
		    STPSDIO_ERR_FUNC ("own back failed in %d us, something might goes wrong\n", 40*delay_us);

		if (0 == (retry - 1)%200) {
#if COHEC_00006052
			ret = mtk_wcn_hif_sdio_writeb(clt_ctx, (UINT32)(CHLPCR+0x1), (UINT8)(C_FW_OWN_REQ_CLR>>8));
#else
			ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_OWN_REQ_CLR);
#endif 
			if (ret) {
				STPSDIO_ERR_FUNC("request firmware own back fail(%d)\n", ret);
			}
			STPSDIO_ERR_FUNC ("own back failed in %d us, write again\n", 200*delay_us);
		}
	}
	while (retry-- > 0);
	if (wait) {
		
		time_out = wait_event_timeout(g_ownback_done,
					      is_wait_ownback == 0, (1000 / (1000 / HZ)));

		if (time_out != 0) {
			STPSDIO_INFO_FUNC("Do FW-Own back!(Wakeup) succeed\n");
			ret = 0;
		} else {
			STPSDIO_ERR_FUNC("Do FW-Own back!(Wakeup) fail\n");
			ret = -1;
		}
	} else {
		ret = retry > 0 ? 0 : -1;
	}

 out:
	return ret;
}

#if 0
static INT32 stp_sdio_do_own_set(VOID)
{
#define SET_OWN_RETRY 10
	INT32 ret = -1;
	UINT32 chlcpr_value = 0x0;
	UINT32 retry = SET_OWN_RETRY;
	UINT32 delay_us = 500;
	UINT32 tx_fifo_clear_cnt = 0;
	MTK_WCN_HIF_SDIO_CLTCTX clt_ctx;
	MTK_WCN_STP_SDIO_HIF_INFO *p_info;
	UINT32 comp_count = 0;

#define SAFE_CNT_TO_SET_OWN 5

	clt_ctx = gp_info->sdio_cltctx;
	p_info = gp_info;

	while (retry) {
#if STP_SDIO_NEW_TXRING
#else
		spin_lock_irqsave(&gp_info->pkt_buf.rd_idx_lock, gp_info->pkt_buf.rd_irq_flag);
		comp_count = atomic_read(&p_info->firmware_info.tx_comp_num);

		if ((comp_count == p_info->firmware_info.tx_packet_num)
		    && (0 == p_info->rx_pkt_len)) {
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);
			msleep(1);
			tx_fifo_clear_cnt++;

			
			
			STPSDIO_DBG_FUNC("complete count(%d), tx_packet_num(%d), rx_pkt_len(%d)!\n",
					 comp_count, p_info->firmware_info.tx_packet_num,
					 p_info->rx_pkt_len);
		} else {
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);
			STPSDIO_WARN_FUNC
			    ("set own (sleep) is temporarily not permitted! complete count(%d), tx_packet_num(%d), rx_pkt_len(%d) wait\n",
			     comp_count, p_info->firmware_info.tx_packet_num, p_info->rx_pkt_len);
			retry--;
			tx_fifo_clear_cnt = 0;
			msleep(2);
		}
#endif

		if (tx_fifo_clear_cnt == SAFE_CNT_TO_SET_OWN) {
			STPSDIO_DBG_FUNC("set own (sleep) is permitted!\n");
			break;
		}
	}

	if (!retry) {
		STPSDIO_WARN_FUNC("set own (sleep) is not permitted, bus is busy.\n");
		return -2;
	}
	
	STPSDIO_INFO_FUNC("set firmware own! (sleeping)\n");
	ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_OWN_REQ_SET);
	if (ret) {
		STPSDIO_ERR_FUNC("set firmware own! (sleeping) fail\n");
		goto out;
	}
	
	retry = 10;
	do {
		ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
		if (ret) {
			
			STPSDIO_ERR_FUNC("get CHISR information rx error!(%d)\n", ret);
			goto out;
		}

		if ((chlcpr_value & C_FW_COM_DRV_OWN) == 0) {
			
			STPSDIO_INFO_FUNC("set firmware own succeful, retry:%d\n",
					  SET_OWN_RETRY - retry);
			break;
		} else {
			STPSDIO_INFO_FUNC("set firmware own!  wait for (%d us) and retry\n",
					  delay_us);
			udelay(delay_us);
		}
	} while (retry-- > 0);

	ret = (retry > 0) ? 0 : -1;

 out:
	return ret;
}
#endif
INT32 stp_sdio_own_ctrl(SDIO_PS_OP op)
{
	INT32 ret = -1;
    P_OSAL_SIGNAL pOsalSignal = NULL;

	if (0 == g_stp_sdio_host_count) {
		STPSDIO_WARN_FUNC("g_stp_sdio_host_count is 0, do nothing!\n");
		return 0;
	}
	pOsalSignal = &gp_info->isr_check_complete;
	pOsalSignal->timeoutValue = 100;
	osal_signal_init(pOsalSignal);
	switch (op) {
	case OWN_SET:
		
		gp_info->wakeup_flag = 0;
		gp_info->sleep_flag = 1;
		osal_trigger_event(&gp_info->tx_rx_event);
		STPSDIO_LOUD_FUNC("before op(%d)\n", op);
		osal_wait_for_signal(pOsalSignal);
		STPSDIO_LOUD_FUNC("after op(%d)\n", op);
		ret = 0;
		break;

	case OWN_CLR:
		
		gp_info->sleep_flag = 0;
		gp_info->wakeup_flag = 1;
		osal_trigger_event(&gp_info->tx_rx_event);
		STPSDIO_LOUD_FUNC("before op(%d)\n", op);
		osal_wait_for_signal(pOsalSignal);
		STPSDIO_LOUD_FUNC("after op(%d)\n", op);
		ret = 0;
		break;
	case OWN_STATE:
		
		STPSDIO_INFO_FUNC("omit op(%d)\n", op);
		ret = 0;
		break;
	default:
		STPSDIO_WARN_FUNC("omit op(%d)\n", op);
		ret = -1;
		break;
	}
	osal_signal_deinit(pOsalSignal);
	return ret;
}

#if STP_SDIO_OWN_THREAD

static INT32 _stp_sdio_wait_for_msg(PVOID pvData)
{
	MTK_WCN_STP_SDIO_HIF_INFO *pInfo = (MTK_WCN_STP_SDIO_HIF_INFO *) pvData;
#if STP_SDIO_NEW_TXRING
	STPSDIO_LOUD_FUNC
	    ("len(%u), wr_cnt(%u), rd_cnt(%u), irq_pending(%u), sleep(%u), wakeup(%u)\n",
	     pInfo->rx_pkt_len, pInfo->pkt_buf.wr_cnt, pInfo->pkt_buf.rd_cnt, pInfo->irq_pending,
	     pInfo->sleep_flag, pInfo->wakeup_flag);
	return (0 != pInfo->rx_pkt_len)
	    || (pInfo->pkt_buf.wr_cnt != pInfo->pkt_buf.rd_cnt)
	    || (0 != pInfo->irq_pending)
	    || (0 != pInfo->sleep_flag)
	    || (0 != pInfo->wakeup_flag)
	    || (osal_thread_should_stop(&pInfo->tx_rx_thread));
#else
	STPSDIO_LOUD_FUNC
	    ("len(%u), rd_idx(%u), wr_idx(%u), irq_pending(%u), sleep(%u), wakeup(%u)\n",
	     pInfo->rx_pkt_len, pInfo->pkt_buf.rd_idx, pInfo->pkt_buf.wr_idx, pInfo->irq_pending,
	     pInfo->sleep_flag, pInfo->wakeup_flag);
	return (pInfo->rx_pkt_len != 0)
	    || (pInfo->pkt_buf.rd_idx != pInfo->pkt_buf.wr_idx)
	    || (0 != pInfo->irq_pending)
	    || (0 != pInfo->sleep_flag)
	    || (0 != pInfo->wakeup_flag)
	    || (osal_thread_should_stop(&pInfo->tx_rx_thread));
#endif
}



static VOID stp_sdio_tx_rx_handling(PVOID pData)
{
	MTK_WCN_STP_SDIO_HIF_INFO *pInfo = (MTK_WCN_STP_SDIO_HIF_INFO *) pData;
	UINT32 iRet = 0;
	UINT32 chisr = 0;
	UINT32 chlcpr_value = 0;
	UINT32 tx_comp = 0;
	MTK_WCN_HIF_SDIO_CLTCTX clt_ctx;
	UINT32 while_loop_counter = 0;
	UINT32 own_fail_counter = 0;
    	UINT32 val = 0;

	STPSDIO_INFO_FUNC("stp_tx_rx_thread start running...\n");
	if (NULL == pInfo) {
		STPSDIO_WARN_FUNC("sanity check fail, (pInfo == NULL)\n");
		return;
	}
	clt_ctx = pInfo->sdio_cltctx;
	STPSDIO_INFO_FUNC("stp_tx_rx_thread runns\n");
	while (!osal_thread_should_stop(&pInfo->tx_rx_thread)) {
		while_loop_counter++;

		
		if (OWN_SET == stp_sdio_get_own_state()) {
			STPSDIO_DBG_FUNC("OWN on fw side!\n");

			mtk_wcn_hif_sdio_dis_deep_sleep(clt_ctx);

			if (0 == stp_sdio_do_own_clr(0)) {
				STPSDIO_DBG_FUNC("set OWN to driver side ok!\n");
				pInfo->awake_flag = 1;
				own_fail_counter = 0;
			} else {
				mtk_wcn_hif_sdio_readl(clt_ctx, CCIR, &val);
				STPSDIO_ERR_FUNC("******CCIR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &val);
				STPSDIO_ERR_FUNC("******CHLPCR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CSDIOCSR, &val);
				STPSDIO_ERR_FUNC("******CSDIOCSR == 0x%x*****\n", val);
				if ((0 != pInfo->sleep_flag) || (0 != pInfo->wakeup_flag)) {
					pInfo->wakeup_flag = 0;
					pInfo->sleep_flag = 0;
					STPSDIO_WARN_FUNC("set OWN to driver side error, but someone is wait for sleep/wakeup event, signal it to return\n");
					osal_raise_signal(&pInfo->isr_check_complete);
				}
				if (0 == (own_fail_counter % 50))
				{
					STPSDIO_ERR_FUNC("set OWN to driver side error!\n");
					
					if (0 == own_fail_counter)
						_stp_sdio_issue_fake_coredump ("ABT: STP-SDIO clear f/w own failed");
				}
				own_fail_counter++;
			}
		} else {
			STPSDIO_DBG_FUNC("OWN on driver side!\n");
			pInfo->awake_flag = 1;
		}
		
		if ((0 != pInfo->wakeup_flag) && (0 != pInfo->awake_flag))
        {
        	while_loop_counter = 0;
            STPSDIO_DBG_FUNC("clr firmware own! (wakeup) ok\n");
            pInfo->wakeup_flag = 0;
            osal_raise_signal(&pInfo->isr_check_complete);
        }
		
		if ((0 != pInfo->irq_pending) && (1 == pInfo->awake_flag)) {
			while_loop_counter = 0;

			
			
			iRet = mtk_wcn_hif_sdio_readl(clt_ctx, CHISR, &chisr);
#if 0
			{
				
				STPSDIO_INFO_FUNC("CHISR(0x%08x)\n", chisr);
				iRet = -5;
			}
#endif
			if (iRet) {
				
				
                STPSDIO_ERR_FUNC("get CHISR information rx error, ret:%d\n", iRet);
				if (-5 == iRet)
				{
					chisr = stp_sdio_rc_reg_readl_retry(clt_ctx, CSR, 100, &iRet);
					pInfo->irq_pending = 0;
				}
				
            }
			else
				pInfo->irq_pending = 0;
			
			if (0 != pInfo->dump_flag) {
				STPSDIO_ERR_FUNC("CHISR(0x%08x)\n", chisr);
			} else {
				STPSDIO_DBG_FUNC("CHISR(0x%08x)\n", chisr);
			}

			if (0x0 == chisr) {
            	gStpSdioDbgLvl = STPSDIO_LOG_DBG;
				STPSDIO_ERR_FUNC("******CHISR == 0*****\n");

				mtk_wcn_hif_sdio_readl(clt_ctx, CCIR, &val);
				STPSDIO_ERR_FUNC("******CCIR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &val);
				STPSDIO_ERR_FUNC("******CHLPCR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CSDIOCSR, &val);
				STPSDIO_ERR_FUNC("******CSDIOCSR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CHCR, &val);
				STPSDIO_ERR_FUNC("******CHCR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CHISR, &val);
				STPSDIO_ERR_FUNC("******CHISR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CHIER, &val);
				STPSDIO_ERR_FUNC("******CHIER == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CTFSR, &val);
				STPSDIO_ERR_FUNC("******CTFSR == 0x%x*****\n", val);
				mtk_wcn_hif_sdio_readl(clt_ctx, CRPLR, &val);
				STPSDIO_ERR_FUNC("******CRPLR == 0x%x*****\n", val);
            }
			else
			{
				gStpSdioDbgLvl = STPSDIO_LOG_INFO;
			}

			if (chisr & FW_OWN_BACK_INT) {
				STPSDIO_HINT_FUNC("FW_OWN_BACK_INT\n");
			}

			
            if ( (chisr & TX_EMPTY) || (chisr & TX_UNDER_THOLD) ) {
				while_loop_counter = 0;
                STPSDIO_DBG_FUNC("Tx interrupt\n");
                
                tx_comp = (chisr & TX_COMPLETE_COUNT) >> 4;

                tx_comp = atomic_add_return(tx_comp, &pInfo->firmware_info.tx_comp_num);
                if (tx_comp > STP_SDIO_TX_PKT_MAX_CNT) {
                    STPSDIO_ERR_FUNC("Abnormal accumulated comp count(%d) chisr(0x%x)\n",
                    tx_comp, chisr);
                }
            }
            if (1 == pInfo->awake_flag)
            {
                stp_sdio_tx_wkr(&pInfo->tx_work);
            }
				
			if (chisr & RX_DONE) {
				
				if (pInfo->rx_pkt_len) {
					STPSDIO_ERR_FUNC("rx worker is not finished yet!\n");
				}

				
				pInfo->rx_pkt_len = (chisr & RX_PKT_LEN) >> 16;
				STPSDIO_HINT_FUNC("rx_pkt_len(%d)\n", pInfo->rx_pkt_len);
				
				if ((0 == pInfo->rx_pkt_len)
				    || (STP_SDIO_RX_FIFO_SIZE < pInfo->rx_pkt_len)) {
					STPSDIO_ERR_FUNC
					    ("abnormal rx_pkt_len(%d) in CHISR(0x%08x) skip rx_worker\n",
					     pInfo->rx_pkt_len, chisr);
					pInfo->rx_pkt_len = 0;
				} else {
					if (pInfo->rx_pkt_len & 0x3) {
						STPSDIO_WARN_FUNC
						    ("rx data len is not 4 bytes allignment, CHISR(0x%08x), rx len (%d).\n",
						     chisr, pInfo->rx_pkt_len);
					}
				}
				
				stp_sdio_rx_wkr(&pInfo->rx_work);
                mtk_wcn_stp_wmt_sdio_host_awake();
			}

				}

		
		
		if (1 == pInfo->awake_flag) {
			stp_sdio_tx_wkr(&pInfo->tx_work);
		}


		
		
		STPSDIO_DBG_FUNC("enable COM IRQ\n");
#if COHEC_00006052
		iRet = mtk_wcn_hif_sdio_writeb(clt_ctx, CHLPCR, C_FW_INT_EN_SET);
#else
		iRet = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_INT_EN_SET);
#endif				
		if (iRet) {
			STPSDIO_ERR_FUNC("enable IRQ fail. iRet:%d\n", iRet);
		} else {
			STPSDIO_HINT_FUNC("enable COM IRQ\n");
			iRet = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
			if (iRet) {
				STPSDIO_ERR_FUNC("read CHLPCR fail. iRet:%d\n", iRet);
			} else {
				if (chlcpr_value & C_FW_INT_EN_SET) {
					STPSDIO_HINT_FUNC("enable COM IRQ okay (0x%x)\n",
							  chlcpr_value);

					
				}
#if 0
				else {
					STPSDIO_ERR_FUNC
					    ("**************Fatal Error, enable COM IRQ fail*********\n");
				}
#endif
			}

		}
        
		if ((0 != pInfo->sleep_flag) && (0 == pInfo->wakeup_flag) && mtk_wcn_stp_is_ready()
		    && (0 == pInfo->irq_pending)
		    && (0 == pInfo->firmware_info.tx_packet_num)
		    && (0 == pInfo->rx_pkt_len)
		    && !mtk_wcn_stp_coredump_start_get()	
		    ) {
			
			
			while_loop_counter = 0;
#if COHEC_00006052
			iRet =
			    mtk_wcn_hif_sdio_writeb(clt_ctx, (UINT32) (CHLPCR + 0x01),
						    (UINT8) (C_FW_OWN_REQ_SET >> 8));
#else
			iRet = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_OWN_REQ_SET);
#endif				
			if (iRet) {
				STPSDIO_ERR_FUNC("set firmware own! (sleeping) fail\n");
			} else {
				iRet = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
				if (iRet) {
					STPSDIO_ERR_FUNC
					    ("get firmware own! (sleeping) fail iRet:%d\n", iRet);
				} else {
					if (!(chlcpr_value & C_FW_COM_DRV_OWN)) {
						while_loop_counter = 0;
						STPSDIO_DBG_FUNC
						    ("set firmware own! (sleeping) ok\n");
						pInfo->awake_flag = 0;
						pInfo->sleep_flag = 0;
						mtk_wcn_hif_sdio_en_deep_sleep(clt_ctx);

						osal_raise_signal(&pInfo->isr_check_complete);
					} else {
						STPSDIO_ERR_FUNC
						    ("set firmware own! (sleeping) fail, set CLR BACK\n");
						
						
#if COHEC_00006052
						iRet =
						    mtk_wcn_hif_sdio_writeb(clt_ctx,
									    (UINT32) (CHLPCR +
										      0x01),
									    (UINT8)
									    (C_FW_OWN_REQ_CLR >>
									     8));
#else
						iRet =
						    mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR,
									    C_FW_OWN_REQ_CLR);
#endif				
					}
				}
			}
		} else {
#if 0
			iRet = mtk_wcn_hif_sdio_readl(clt_ctx, CHISR, &chisr);
			if (iRet) {
				
				
				STPSDIO_ERR_FUNC("get CHISR information rx error!\n");
			}
			STPSDIO_HINT_FUNC("CHISR(0x%08x)\n", chisr);
#endif
		}
		if (1000 < while_loop_counter) {
			while_loop_counter = 0;
			pInfo->dump_flag = 1;

			STPSDIO_ERR_FUNC
			    ("sleeping check result. stp_is_ready(%d) irq_pending(%d) tx_packet_num(%d) rx_pkt_len(%d)\n",
			     mtk_wcn_stp_is_ready(), pInfo->irq_pending,
			     pInfo->firmware_info.tx_packet_num, pInfo->rx_pkt_len);
			
			pInfo->irq_pending = 1;

			if ((0 != pInfo->sleep_flag) || (0 != pInfo->wakeup_flag)) {
				
				pInfo->wakeup_flag = 0;
				pInfo->sleep_flag = 0;
				osal_raise_signal(&pInfo->isr_check_complete);
				if (STP_SDIO_TX_PKT_MAX_CNT == pInfo->firmware_info.tx_packet_num)
				{
					STPSDIO_ERR_FUNC("STP tx window full, proberbly STP-SDIO cannot handle tx request. tx_packet_num:%d\n", pInfo->firmware_info.tx_packet_num);
					osal_sleep_ms(200);
				}
			}
		} else {
			pInfo->dump_flag = 0;
		}
		osal_wait_for_event(&pInfo->tx_rx_event, _stp_sdio_wait_for_msg, (PVOID)pInfo);
		
		STPSDIO_DBG_FUNC("stp_tx_rx_thread receive signal\n");
    }
	
	if ((0 != pInfo->sleep_flag) || (0 != pInfo->wakeup_flag))
	{
		pInfo->wakeup_flag = 0;
		pInfo->sleep_flag = 0;
		STPSDIO_WARN_FUNC("stp_tx_rx_thread exiting, but someone is wait for sleep/wakeup event, signal it to return\n");
		osal_raise_signal(&pInfo->isr_check_complete);
	}
	while (0 < osal_signal_active_state(&pInfo->isr_check_complete))
	{
		STPSDIO_WARN_FUNC("is_check_complete signal in active state, wait for a moment.\n");
		osal_sleep_ms(10);
	}
	STPSDIO_INFO_FUNC("stp_tx_rx_thread exit\n");
	return;
}

#endif

/*!
 * \brief Tx callback function for STP-CORE module
 *
 * \details A function registered to STP-CORE to provide STP-SDIO tx method.
 *      Multiple STP packet may be aggregated when available.
 *
 * \param[IN] data STP packet buffer to be sent through STP-SDIO
 * \param[IN] size STP packet length to be sent
 * \param[OUT] written_size Accepted buffer length by STP-SDIO. Shall be $size
 *      if success.
 *
 *\note Tx may do aggregation to previous entry with lock protection. If no
 *      aggregation is done, protection is NOT required.
 *
 * \retval 0 success
 * \retval -1 invalid input parameters
 * \todo return !0 when fail case (TBD)
 */
#if STP_SDIO_NEW_TXRING
INT32 stp_sdio_tx(const PUINT8 data, const UINT32 size, PUINT32 written_size)
{
	PUINT8 pkt_bufp;
	UINT32 prev_wr_idx;
	UINT32 prev_size;
	MTK_WCN_STP_SDIO_PKT_BUF *pb;
	UINT32 idx;

	STPSDIO_LOUD_FUNC("enter\n");
	if (written_size) {
		*written_size = 0;
	}

	
	if ((STP_SDIO_TX_FIFO_SIZE < size)
	    || (!data)) {
		STPSDIO_LOUD_FUNC("invalid size(%ld) > fifo(%d) or data(0x%p)\n",
				  size, STP_SDIO_TX_FIFO_SIZE, data);
		return -1;
	}

	pb = &gp_info->pkt_buf;
	

	spin_lock_irqsave(&pb->rd_cnt_lock, pb->rd_irq_flag);
	if (pb->rd_cnt == pb->wr_cnt) {
		spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);
		
		idx = pb->wr_cnt & STP_SDIO_TX_BUF_CNT_MASK;
		pb->tx_buf_ts[idx] = jiffies;
		pb->tx_buf_sz[idx] = size + STP_SDIO_HDR_SIZE;
#if KMALLOC_UPDATE
		pkt_bufp = pb->tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + STP_SDIO_HDR_SIZE;
#else
		pkt_bufp = &pb->tx_buf[idx][STP_SDIO_HDR_SIZE];
#endif
		memcpy(pkt_bufp, data, size);
		++(pb->wr_cnt);

		if (written_size) {
			*written_size = size;
		}
		STPSDIO_DBG_FUNC("(Empty) Enqueue done\n");
	}
	
	else if ((pb->wr_cnt - pb->rd_cnt) < STP_SDIO_TX_BUF_CNT) {
		prev_wr_idx = (pb->wr_cnt - 1) & STP_SDIO_TX_BUF_CNT_MASK;
		prev_size = pb->tx_buf_sz[prev_wr_idx];

		
		if (((pb->rd_cnt + 1) != pb->wr_cnt)
		    && ((size + prev_size) <= STP_SDIO_TX_FIFO_SIZE)) {
#if KMALLOC_UPDATE
			pkt_bufp = pb->tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + prev_size;
#else
			pkt_bufp = &pb->tx_buf[prev_wr_idx][prev_size];
#endif
			pb->tx_buf_sz[prev_wr_idx] += size;

			memcpy(pkt_bufp, data, size);
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);

			STPSDIO_DBG_FUNC("(Not empty-aggre) Enqueue done\n");

			if (written_size) {
				*written_size = size;
			}
		}
		
		else {
			
			if ((pb->wr_cnt - pb->rd_cnt) == 1) {
				pb->full_flag = MTK_WCN_BOOL_TRUE;
			}
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);

			idx = pb->wr_cnt & STP_SDIO_TX_BUF_CNT_MASK;
			pb->tx_buf_ts[idx] = jiffies;
			pb->tx_buf_sz[idx] = size + STP_SDIO_HDR_SIZE;
#if KMALLOC_UPDATE
			pkt_bufp = pb->tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + STP_SDIO_HDR_SIZE;
#else
			pkt_bufp = &pb->tx_buf[idx][STP_SDIO_HDR_SIZE];
#endif
			memcpy(pkt_bufp, data, size);
			++(pb->wr_cnt);

			STPSDIO_DBG_FUNC("(Not empty-no aggre) Enqueue done\n");
			if (written_size) {
				*written_size = size;
			}
		}
	}
	
	else
		 {
		if (!((pb->wr_cnt - pb->rd_cnt) >= STP_SDIO_TX_BUF_CNT)) {
			STPSDIO_ERR_FUNC
			    ("abnormal condition and flow, wr_cnt(0x%x), rd_cnt(0x%x)\n",
			     pb->wr_cnt, pb->rd_cnt);
		}
		prev_wr_idx = (pb->wr_cnt - 1) & STP_SDIO_TX_BUF_CNT_MASK;
		prev_size = pb->tx_buf_sz[prev_wr_idx];

		
		if ((size + prev_size) <= STP_SDIO_TX_FIFO_SIZE) {
			if (0 != prev_size) {
				STPSDIO_ERR_FUNC
				    ("abnormal condition and flow, wr_cnt(0x%x), rd_cnt(0x%x), prev(%d), prev_size(%d)\n",
				     pb->wr_cnt, pb->rd_cnt, prev_wr_idx, prev_size);
			}
#if KMALLOC_UPDATE
			pkt_bufp = pb->tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + prev_size;
#else
			pkt_bufp = &pb->tx_buf[prev_wr_idx][prev_size];
#endif
			pb->tx_buf_sz[prev_wr_idx] += size;

			memcpy(pkt_bufp, data, size);
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);

			STPSDIO_DBG_FUNC("(full-aggre) Enqueue done\n");

			if (written_size) {
				*written_size = size;
			}
		}
		
		else {
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);
			STPSDIO_WARN_FUNC("Local Tx buffer is full !\n");

			
			
			
			wait_event_interruptible(pb->fullwait_q,
						 (MTK_WCN_BOOL_FALSE == pb->full_flag));
			STPSDIO_INFO_FUNC("wait event return\n");

			spin_lock_irqsave(&pb->rd_cnt_lock, pb->rd_irq_flag);
			
			if ((pb->wr_cnt - pb->rd_cnt) == 1) {
				pb->full_flag = MTK_WCN_BOOL_TRUE;
			}
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);

			
			idx = pb->wr_cnt & STP_SDIO_TX_BUF_CNT_MASK;
			pb->tx_buf_ts[idx] = jiffies;
			pb->tx_buf_sz[idx] = size + STP_SDIO_HDR_SIZE;
#if KMALLOC_UPDATE
			pkt_bufp = pb->tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + STP_SDIO_HDR_SIZE;
#else
			pkt_bufp = &pb->tx_buf[idx][STP_SDIO_HDR_SIZE];
#endif
			
			memcpy(pkt_bufp, data, size);
			++(pb->wr_cnt);

			if (written_size) {
				*written_size = size;
			}
		}
		}

	
#if STP_SDIO_OWN_THREAD
	
	STPSDIO_DBG_FUNC("osal_trigger_event gp_info->tx_rx_event\n");
	osal_trigger_event(&gp_info->tx_rx_event);
#else
	schedule_work(&gp_info->tx_work);
#endif
	return 0;
}

#else
INT32 stp_sdio_tx(const PUINT8 data, const UINT32 size, PUINT32 written_size)
{
	PUINT8 pkt_bufp;
	UINT32 prev_wr_idx;
	UINT32 buf_allocation;
	UINT32 room;

	STPSDIO_LOUD_FUNC("enter\n");

	
	*written_size = 0;

	spin_lock_irqsave(&gp_info->pkt_buf.rd_idx_lock, gp_info->pkt_buf.rd_irq_flag);
	
	if ((gp_info->pkt_buf.rd_idx == gp_info->pkt_buf.wr_idx)
	    && (MTK_WCN_BOOL_FALSE == gp_info->pkt_buf.full_flag)) {
		spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock, gp_info->pkt_buf.rd_irq_flag);
		
#if KMALLOC_UPDATE
		*(gp_info->pkt_buf.tx_buf + gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0) =
		    (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
		*(gp_info->pkt_buf.tx_buf + gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1) =
		    (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
#else
		gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][0] =
		    (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
		gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][1] =
		    (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
#endif
		gp_info->pkt_buf.tx_buf_ts[gp_info->pkt_buf.wr_idx] = jiffies;

		STPSDIO_DBG_FUNC("(Empty) Enqueue done\n");
#if KMALLOC_UPDATE
		pkt_bufp =
		    gp_info->pkt_buf.tx_buf + gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE +
		    STP_SDIO_HDR_SIZE;
#else
		pkt_bufp = &gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][STP_SDIO_HDR_SIZE];
#endif
		memcpy(pkt_bufp, data, size);
		*written_size = size;

		gp_info->pkt_buf.wr_idx = (gp_info->pkt_buf.wr_idx + 1) % STP_SDIO_TX_BUF_CNT;
	}

	
	else if (gp_info->pkt_buf.rd_idx != gp_info->pkt_buf.wr_idx) {
		prev_wr_idx =
		    (gp_info->pkt_buf.wr_idx - 1 + STP_SDIO_TX_BUF_CNT) % STP_SDIO_TX_BUF_CNT;
		
#if KMALLOC_UPDATE
		buf_allocation =
		    *(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1);
		buf_allocation =
		    (buf_allocation << 8) | *(gp_info->pkt_buf.tx_buf +
					      prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0);
#else
		buf_allocation = gp_info->pkt_buf.tx_buf[prev_wr_idx][1];
		buf_allocation = (buf_allocation << 8) | gp_info->pkt_buf.tx_buf[prev_wr_idx][0];
#endif
		
		if ((prev_wr_idx != gp_info->pkt_buf.rd_idx)
		    && ((size + buf_allocation) <= STP_SDIO_TX_FIFO_SIZE)) {
#if KMALLOC_UPDATE
			pkt_bufp =
			    gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE +
			    buf_allocation;

			buf_allocation += size;
			*(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0) =
			    (UINT8) (buf_allocation & 0xff);
			*(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1) =
			    (UINT8) (buf_allocation >> 8);
#else
			pkt_bufp = &gp_info->pkt_buf.tx_buf[prev_wr_idx][buf_allocation];

			buf_allocation += size;
			gp_info->pkt_buf.tx_buf[prev_wr_idx][0] = (UINT8) (buf_allocation & 0xff);
			gp_info->pkt_buf.tx_buf[prev_wr_idx][1] = (UINT8) (buf_allocation >> 8);
#endif
			memcpy(pkt_bufp, data, size);
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);

			STPSDIO_DBG_FUNC("(Not empty-aggre) Enqueue done\n");

			*written_size = size;
		}
		
		else {
			
			room = (gp_info->pkt_buf.wr_idx >= gp_info->pkt_buf.rd_idx) ?
			    (STP_SDIO_TX_BUF_CNT -
			     (gp_info->pkt_buf.wr_idx -
			      gp_info->pkt_buf.rd_idx)) : (gp_info->pkt_buf.rd_idx -
							   gp_info->pkt_buf.wr_idx);
			if (room == 1) {
				gp_info->pkt_buf.full_flag = MTK_WCN_BOOL_TRUE;
			}
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);


			
#if KMALLOC_UPDATE
			*(gp_info->pkt_buf.tx_buf +
			  gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0) =
  (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
			*(gp_info->pkt_buf.tx_buf +
			  gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1) =
  (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
			gp_info->pkt_buf.tx_buf_ts[gp_info->pkt_buf.wr_idx] = jiffies;

			pkt_bufp =
			    gp_info->pkt_buf.tx_buf +
			    gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + STP_SDIO_HDR_SIZE;
#else
			gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][0] =
			    (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
			gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][1] =
			    (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
			gp_info->pkt_buf.tx_buf_ts[gp_info->pkt_buf.wr_idx] = jiffies;

			pkt_bufp =
			    &gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][STP_SDIO_HDR_SIZE];
#endif
			memcpy(pkt_bufp, data, size);

			STPSDIO_DBG_FUNC("(Not empty-no aggre) Enqueue done\n");

			*written_size = size;

			gp_info->pkt_buf.wr_idx =
			    (gp_info->pkt_buf.wr_idx + 1) % STP_SDIO_TX_BUF_CNT;
		}
	}

	
	else if (MTK_WCN_BOOL_FALSE != gp_info->pkt_buf.full_flag) {
		prev_wr_idx =
		    (gp_info->pkt_buf.wr_idx - 1 + STP_SDIO_TX_BUF_CNT) % STP_SDIO_TX_BUF_CNT;
#if KMALLOC_UPDATE
		buf_allocation =
		    *(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1);
		buf_allocation =
		    (buf_allocation << 8) | *(gp_info->pkt_buf.tx_buf +
					      prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0);
#else
		buf_allocation = gp_info->pkt_buf.tx_buf[prev_wr_idx][1];
		buf_allocation = (buf_allocation << 8) | gp_info->pkt_buf.tx_buf[prev_wr_idx][0];
#endif
		
		if ((buf_allocation != 0)
		    && ((size + buf_allocation) <= STP_SDIO_TX_FIFO_SIZE)) {
#if KMALLOC_UPDATE
			pkt_bufp =
			    gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE +
			    buf_allocation;
#else
			pkt_bufp = &gp_info->pkt_buf.tx_buf[prev_wr_idx][buf_allocation];
#endif
			buf_allocation += size;
#if KMALLOC_UPDATE
			*(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0) =
			    (UINT8) (buf_allocation & 0xff);
			*(gp_info->pkt_buf.tx_buf + prev_wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1) =
			    (UINT8) (buf_allocation >> 8);
#else
			gp_info->pkt_buf.tx_buf[prev_wr_idx][0] = (UINT8) (buf_allocation & 0xff);
			gp_info->pkt_buf.tx_buf[prev_wr_idx][1] = (UINT8) (buf_allocation >> 8);
#endif
			
			memcpy(pkt_bufp, data, size);
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);

			STPSDIO_DBG_FUNC("(full-aggre) Enqueue done\n");

			*written_size = size;
		}

		
		else {
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);
			STPSDIO_WARN_FUNC("Local Tx buffer is full !!!!!\n");

			
			
			
			wait_event_interruptible(gp_info->pkt_buf.fullwait_q,
						 (!gp_info->pkt_buf.full_flag));
			STPSDIO_INFO_FUNC("wait event return\n");

			spin_lock_irqsave(&gp_info->pkt_buf.rd_idx_lock,
					  gp_info->pkt_buf.rd_irq_flag);
			
			room = (gp_info->pkt_buf.wr_idx >= gp_info->pkt_buf.rd_idx) ?
			    (STP_SDIO_TX_BUF_CNT -
			     (gp_info->pkt_buf.wr_idx -
			      gp_info->pkt_buf.rd_idx)) : (gp_info->pkt_buf.rd_idx -
							   gp_info->pkt_buf.wr_idx);
			if (room == 1) {
				gp_info->pkt_buf.full_flag = MTK_WCN_BOOL_TRUE;
			}
			spin_unlock_irqrestore(&gp_info->pkt_buf.rd_idx_lock,
					       gp_info->pkt_buf.rd_irq_flag);

			
#if KMALLOC_UPDATE
			*(gp_info->pkt_buf.tx_buf +
			  gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 0) =
  (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
			*(gp_info->pkt_buf.tx_buf +
			  gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + 1) =
  (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
			gp_info->pkt_buf.tx_buf_ts[gp_info->pkt_buf.wr_idx] = jiffies;

			pkt_bufp =
			    gp_info->pkt_buf.tx_buf +
			    gp_info->pkt_buf.wr_idx * STP_SDIO_TX_ENTRY_SIZE + STP_SDIO_HDR_SIZE;
#else
			gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][0] =
			    (UINT8) ((size + STP_SDIO_HDR_SIZE) & 0xff);
			gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][1] =
			    (UINT8) ((size + STP_SDIO_HDR_SIZE) >> 8);
			gp_info->pkt_buf.tx_buf_ts[gp_info->pkt_buf.wr_idx] = jiffies;

			pkt_bufp =
			    &gp_info->pkt_buf.tx_buf[gp_info->pkt_buf.wr_idx][STP_SDIO_HDR_SIZE];
#endif
			
			memcpy(pkt_bufp, data, size);
			*written_size = size;
			gp_info->pkt_buf.wr_idx =
			    (gp_info->pkt_buf.wr_idx + 1) % STP_SDIO_TX_BUF_CNT;
		}
	}
	
#if STP_SDIO_OWN_THREAD
	
	STPSDIO_DBG_FUNC("osal_trigger_event gp_info->tx_rx_event\n");
	osal_trigger_event(&gp_info->tx_rx_event);
#else
	schedule_work(&gp_info->tx_work);
#endif

	return 0;
}
#endif				

static VOID stp_sdio_check_tx_sanity(const MTK_WCN_STP_SDIO_HIF_INFO *p_info, const UINT32 id)
{
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
	if ((p_info) && (0 == p_info->firmware_info.tx_packet_num)) {
		if (!(p_info->tx_pkt_list.pkt_rd_cnt == p_info->tx_pkt_list.pkt_wr_cnt)) {
			STPSDIO_ERR_FUNC
			    ("abnormal fifo_size(%d) pkt_num(%d) pkt_rd(0x%x, %ld) pkt_wr(0x%x, %ld)!(%d)\n",
			     p_info->firmware_info.tx_fifo_size,
			     p_info->firmware_info.tx_packet_num, p_info->tx_pkt_list.pkt_rd_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_rd_cnt),
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_wr_cnt), id);
			
		}
		if (p_info->firmware_info.tx_fifo_size != STP_SDIO_TX_FIFO_SIZE) {
			STPSDIO_ERR_FUNC
			    ("abnormal fifo_size(%d) pkt_num(%d) pkt_rd(0x%x, %ld) pkt_wr(0x%x, %ld)!(%d)\n",
			     p_info->firmware_info.tx_fifo_size,
			     p_info->firmware_info.tx_packet_num, p_info->tx_pkt_list.pkt_rd_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_rd_cnt),
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_wr_cnt), id);
			
		}
	} else {
		if ((p_info) && (p_info->tx_pkt_list.pkt_rd_cnt == p_info->tx_pkt_list.pkt_wr_cnt)) {
			STPSDIO_ERR_FUNC
			    ("abnormal fifo_size(%d) pkt_num(%d) pkt_rd(0x%x, %ld) pkt_wr(0x%x, %ld)!(%d)\n",
			     p_info->firmware_info.tx_fifo_size,
			     p_info->firmware_info.tx_packet_num, p_info->tx_pkt_list.pkt_rd_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_rd_cnt),
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_wr_cnt), id);
			
		}
	}
#endif
}

static VOID stp_sdio_tx_wkr_comp(MTK_WCN_STP_SDIO_HIF_INFO * const p_info)
{
	INT32 comp_count;
	UINT32 idx;

	comp_count = atomic_read(&p_info->firmware_info.tx_comp_num);
	atomic_sub(comp_count, &p_info->firmware_info.tx_comp_num);

	
	if (p_info->firmware_info.tx_packet_num >= comp_count) {
		STPSDIO_DBG_FUNC("tx_pack_num(%d), comp_count(%d),tx_comp_num(%d)\n",
				 p_info->firmware_info.tx_packet_num, comp_count,
				 atomic_read(&p_info->firmware_info.tx_comp_num));
		p_info->firmware_info.tx_packet_num -= comp_count;
	} else {
		STPSDIO_ERR_FUNC("abnormal complete count(%d), tx_packet_num(%d)!\n",
				 comp_count, p_info->firmware_info.tx_packet_num);
		
	}

	while (comp_count > 0) {
		if (p_info->tx_pkt_list.pkt_rd_cnt == p_info->tx_pkt_list.pkt_wr_cnt) {
			STPSDIO_ERR_FUNC
			    ("tx complete count(%d) but tx_pkt_list empty, rd_cnt(0x%ud, idx:%lx), wr_cnt(0x%ud, idx:%lx)!\n",
			     comp_count, p_info->tx_pkt_list.pkt_rd_cnt,
			     p_info->tx_pkt_list.pkt_rd_cnt & STP_SDIO_TX_PKT_LIST_SIZE_MASK,
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     p_info->tx_pkt_list.pkt_wr_cnt & STP_SDIO_TX_PKT_LIST_SIZE_MASK);
			break;
		}

		idx = p_info->tx_pkt_list.pkt_rd_cnt++ & STP_SDIO_TX_PKT_LIST_SIZE_MASK;
		p_info->firmware_info.tx_fifo_size += p_info->tx_pkt_list.pkt_size_list[idx];
		p_info->tx_pkt_list.out_ts[idx] = jiffies;
		--comp_count;
	}
}

/*!
 * \brief Handle STP-SDIO Tx buffer and send to bus
 *
 * \details Handle STP-SDIO Tx buffer and send SDIO packet to bus if everything
 *      is checked ok.
 *
 * \note Tx count to FIFO is counted on a 4-byte aligned base. 1~3 bytes padding
 *      are also sent into HW FIFO and SHALL be trimmed off by firmware.
 *      tx_fifo_size is deducted in this bus when writting data to bus and added
 *      back in handle_tx_comp() function.
 *
 * \note Data length written to bus shall be 4-byte aligned AND block_size
 *      aligned if length > block_size. Padding bytes added for block_size is
 *      removed by HW.
 *
 * \note Max accumulated Tx size to FIFO is limited by STP_SDIO_TX_FIFO_SIZE and
 *      it is (2080) for MT6620. It is NOT limited to 256*5=1280 bytes.
 *
 * \note Max outstanding Tx packet count is limited by STP_SDIO_TX_PKT_MAX_CNT
 *      and it is (7) for MT6620.
 *
 * \param[IN] work Tx work struct work_struct pointer used by STP-SDIO
 *
 * \retval none.
 */
#if STP_SDIO_NEW_TXRING
static VOID stp_sdio_tx_wkr(
				   
				   struct work_struct *work)
{
	MTK_WCN_STP_SDIO_HIF_INFO *p_info;
	UINT32 bus_txlen;
	UINT32 four_byte_align_len;
	PUINT8 buf_tx;
	INT32 ret;
	UINT32 idx;
	MTK_WCN_STP_SDIO_PKT_BUF *pb;

	p_info = container_of(work, MTK_WCN_STP_SDIO_HIF_INFO, tx_work);
	ret = HIF_SDIO_ERR_SUCCESS;
	pb = &p_info->pkt_buf;

	STPSDIO_LOUD_FUNC("start\n");

	
	do {
		
		stp_sdio_tx_wkr_comp(p_info);
		
		stp_sdio_check_tx_sanity(p_info, 1);

		
		if (p_info->pkt_buf.wr_cnt == p_info->pkt_buf.rd_cnt) {
			
			STPSDIO_DBG_FUNC("Tx entry ring buffer empty\n");
			break;
		}
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
		++stp_sdio_txperf_worker_cnt;
#endif

		if (STP_SDIO_TX_PKT_MAX_CNT <= p_info->firmware_info.tx_packet_num) {
			STPSDIO_DBG_FUNC
			    ("tx_packet_num(%ld) limit,  tx_fifo_size(%ld), four_byte_align_len(%ld)\n",
			     p_info->firmware_info.tx_packet_num,
			     p_info->firmware_info.tx_fifo_size, four_byte_align_len);
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
			++stp_sdio_txperf_pkt_num_lmt_cnt;
#endif
			break;
		} else {
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
			stp_sdio_txperf_txed_pkt_num += p_info->firmware_info.tx_packet_num;
#endif
		}

		idx = pb->rd_cnt & STP_SDIO_TX_BUF_CNT_MASK;

		
		bus_txlen = pb->tx_buf_sz[idx];
		
#if KMALLOC_UPDATE
		buf_tx = gp_info->pkt_buf.tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + 0;
#else
		buf_tx = &pb->tx_buf[idx][0];
#endif
		buf_tx[0] = (UINT8) (bus_txlen & 0xff);
		buf_tx[1] = (UINT8) ((bus_txlen >> 8) & 0xff);

		
		bus_txlen += 0x3;
		bus_txlen &= ~(0x3UL);
		four_byte_align_len = bus_txlen;

		
		if (four_byte_align_len > STP_SDIO_TX_FIFO_SIZE) {
			STPSDIO_ERR_FUNC("abnormal four_byte_align_len(%d) > TX_FIFO_SIZE(%ld)!!\n",
					 four_byte_align_len, STP_SDIO_TX_FIFO_SIZE);
		}

		if (p_info->firmware_info.tx_fifo_size >= four_byte_align_len) {
			
			if (STP_SDIO_BLK_SIZE < bus_txlen) {
				bus_txlen += (STP_SDIO_BLK_SIZE - 1);
				bus_txlen &= ~((UINT32) STP_SDIO_BLK_SIZE - 1);
			}

			
			if (STP_SDIO_TX_ENTRY_SIZE < bus_txlen) {
				STPSDIO_ERR_FUNC
				    ("abnormal bus_txlen(%d) > STP_SDIO_TX_ENTRY_SIZE(%ld)!!\n",
				     bus_txlen, STP_SDIO_TX_ENTRY_SIZE);
			}

			++(p_info->firmware_info.tx_packet_num);
			
			p_info->firmware_info.tx_fifo_size -= four_byte_align_len;
			
			idx = p_info->tx_pkt_list.pkt_wr_cnt++ & STP_SDIO_TX_PKT_LIST_SIZE_MASK;
			p_info->tx_pkt_list.pkt_size_list[idx] = four_byte_align_len;
			p_info->tx_pkt_list.in_ts[idx] = jiffies;
			p_info->tx_pkt_list.out_ts[idx] = 0;

			STPSDIO_DBG_FUNC
			    ("update tx packet size list wr(0x%x, %ld) rd(0x%x, %ld), tx fifo(size:%d), pkt_num(%d)done\n",
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_wr_cnt),
			     p_info->tx_pkt_list.pkt_rd_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_rd_cnt),
			     p_info->firmware_info.tx_fifo_size,
			     p_info->firmware_info.tx_packet_num);

			
			ret =
			    mtk_wcn_hif_sdio_write_buf(p_info->sdio_cltctx, CTDR, (PUINT32) buf_tx,
						       bus_txlen);
			STPSDIO_DBG_FUNC("write to CTDR done\n");

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
			do {
				idx = stp_sdio_txdbg_cnt++ & STP_SDIO_TXDBG_COUNT_MASK;
				
				stp_sdio_txdbg_buffer[idx].ts = jiffies;
				stp_sdio_txdbg_buffer[idx].bus_txlen = bus_txlen;
				stp_sdio_txdbg_buffer[idx].four_byte_align_len =
				    four_byte_align_len;
				
				if (bus_txlen <= STP_SDIO_TX_ENTRY_SIZE) {
					memcpy(&stp_sdio_txdbg_buffer[idx].tx_pkt_buf[0], buf_tx,
					       bus_txlen);
				} else {
					memcpy(&stp_sdio_txdbg_buffer[idx].tx_pkt_buf[0], buf_tx,
					       STP_SDIO_TX_ENTRY_SIZE);
					STPSDIO_ERR_FUNC("abnormal bus_txlen (%d)!\n", bus_txlen);
				}
			} while (0);
#endif
			if (ret) {
				STPSDIO_ERR_FUNC("get CTDR information Tx error(%d)!\n", ret);

				
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
				stp_sdio_txdbg_dump();
#endif
				osal_dbg_assert_aee("<HIF_SDIO> sdio_writesb ERROR","write data by SDIO report error");
			}

			
			
			
			
			

			spin_lock_irqsave(&pb->rd_cnt_lock, pb->rd_irq_flag);
			++(pb->rd_cnt);
			
			if (MTK_WCN_BOOL_FALSE != pb->full_flag) {
				pb->full_flag = MTK_WCN_BOOL_FALSE;
				wake_up_interruptible(&pb->fullwait_q);
			}
			spin_unlock_irqrestore(&pb->rd_cnt_lock, pb->rd_irq_flag);
		} else {
			
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
			stp_sdio_txperf_fifo_left += p_info->firmware_info.tx_fifo_size;
			stp_sdio_txperf_to_send += four_byte_align_len;
			++stp_sdio_txperf_fifo_lmt_cnt;
#endif

			STPSDIO_DBG_FUNC
			    ("tx_fifo_size(%d), four_byte_align_len(%d), tx_packet_num(%d)\n",
			     p_info->firmware_info.tx_fifo_size, four_byte_align_len,
			     p_info->firmware_info.tx_packet_num);
			break;
		}

		stp_sdio_check_tx_sanity(p_info, 2);
	} while (1);

	STPSDIO_LOUD_FUNC("end\n");
}

#else
static VOID stp_sdio_tx_wkr(
				   
				   struct work_struct *work)
{
	MTK_WCN_STP_SDIO_HIF_INFO *p_info;
	UINT32 bus_txlen;
	UINT32 four_byte_align_len;
	PUINT8 buf_tx;
	INT32 ret;
	UINT32 idx;
	MTK_WCN_STP_SDIO_PKT_BUF *pb;

	p_info = container_of(work, MTK_WCN_STP_SDIO_HIF_INFO, tx_work);
	ret = HIF_SDIO_ERR_SUCCESS;
	pb = &p_info->pkt_buf;

	STPSDIO_LOUD_FUNC("start\n");

	
	do {
		
		stp_sdio_tx_wkr_comp(p_info);
		stp_sdio_check_tx_sanity(p_info, 1);

		
		if ((p_info->pkt_buf.wr_idx == p_info->pkt_buf.rd_idx)
		    && (MTK_WCN_BOOL_FALSE == p_info->pkt_buf.full_flag)) {
			STPSDIO_DBG_FUNC("Tx ring buffer is empty\n");
			break;
		}

		
#if KMALLOC_UPDATE
		buf_tx =
		    gp_info->pkt_buf.tx_buf + p_info->pkt_buf.rd_idx * STP_SDIO_TX_ENTRY_SIZE + 0;
#else
		buf_tx = &p_info->pkt_buf.tx_buf[p_info->pkt_buf.rd_idx][0];
#endif
		bus_txlen = buf_tx[1];
		bus_txlen = (bus_txlen << 8) | buf_tx[0];

		
		bus_txlen += 0x3;
		bus_txlen &= ~(0x3UL);
		four_byte_align_len = bus_txlen;
		
		if (four_byte_align_len > STP_SDIO_TX_FIFO_SIZE) {
			STPSDIO_ERR_FUNC("abnormal four_byte_align_len(%d) > TX_FIFO_SIZE(%ld)!!\n",
					 four_byte_align_len, STP_SDIO_TX_FIFO_SIZE);
		}

		if ((p_info->firmware_info.tx_fifo_size >= four_byte_align_len)
		    && (p_info->firmware_info.tx_packet_num < STP_SDIO_TX_PKT_MAX_CNT)) {
			
			if (STP_SDIO_BLK_SIZE < bus_txlen) {
				bus_txlen += (STP_SDIO_BLK_SIZE - 1);
				bus_txlen &= ~((UINT32) STP_SDIO_BLK_SIZE - 1);
			}

			
			if (STP_SDIO_TX_ENTRY_SIZE < bus_txlen) {
				STPSDIO_ERR_FUNC
				    ("abnormal bus_txlen(%d) > STP_SDIO_TX_ENTRY_SIZE(%ld)!!\n",
				     bus_txlen, STP_SDIO_TX_ENTRY_SIZE);
			}

			++(p_info->firmware_info.tx_packet_num);
			
			p_info->firmware_info.tx_fifo_size -= four_byte_align_len;
			
			idx = p_info->tx_pkt_list.pkt_wr_cnt++ & STP_SDIO_TX_PKT_LIST_SIZE_MASK;
			p_info->tx_pkt_list.pkt_size_list[idx] = four_byte_align_len;
			p_info->tx_pkt_list.in_ts[idx] = jiffies;
			p_info->tx_pkt_list.out_ts[idx] = 0;

			STPSDIO_DBG_FUNC
			    ("update tx packet size list wr(0x%x, %ld) rd(0x%x, %ld), tx fifo(size:%d), pkt_num(%d)done\n",
			     p_info->tx_pkt_list.pkt_wr_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_wr_cnt),
			     p_info->tx_pkt_list.pkt_rd_cnt,
			     STP_SDIO_GET_PKT_AR_IDX(p_info->tx_pkt_list.pkt_rd_cnt),
			     p_info->firmware_info.tx_fifo_size,
			     p_info->firmware_info.tx_packet_num);

			
			ret =
			    mtk_wcn_hif_sdio_write_buf(p_info->sdio_cltctx, CTDR, (PUINT32) buf_tx,
						       bus_txlen);
			STPSDIO_DBG_FUNC("write to CTDR done\n");

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
			do {
				idx = stp_sdio_txdbg_cnt++ & STP_SDIO_TXDBG_COUNT_MASK;
				
				stp_sdio_txdbg_buffer[idx].ts = jiffies;
				stp_sdio_txdbg_buffer[idx].bus_txlen = bus_txlen;
				stp_sdio_txdbg_buffer[idx].four_byte_align_len =
				    four_byte_align_len;
				
				if (bus_txlen <= STP_SDIO_TX_ENTRY_SIZE) {
					memcpy(&stp_sdio_txdbg_buffer[idx].tx_pkt_buf[0], buf_tx,
					       bus_txlen);
				} else {
					memcpy(&stp_sdio_txdbg_buffer[idx].tx_pkt_buf[0], buf_tx,
					       STP_SDIO_TX_ENTRY_SIZE);
					STPSDIO_ERR_FUNC("abnormal bus_txlen(%d)!\n", bus_txlen);
				}
			} while (0);
#endif

			if (ret) {
				STPSDIO_ERR_FUNC("get CTDR information Tx error(%d)!\n", ret);
				
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
				stp_sdio_txdbg_dump();
#endif
				osal_dbg_assert_aee("<HIF_SDIO> sdio_writesb ERROR","write data by SDIO report error");
			}

			
			
			
			
			

			spin_lock_irqsave(&p_info->pkt_buf.rd_idx_lock,
					  p_info->pkt_buf.rd_irq_flag);
			
			p_info->pkt_buf.rd_idx = (p_info->pkt_buf.rd_idx + 1) % STP_SDIO_TX_BUF_CNT;
			
			if (MTK_WCN_BOOL_FALSE != p_info->pkt_buf.full_flag) {
				p_info->pkt_buf.full_flag = MTK_WCN_BOOL_FALSE;
				wake_up_interruptible(&p_info->pkt_buf.fullwait_q);
			}
			spin_unlock_irqrestore(&p_info->pkt_buf.rd_idx_lock,
					       p_info->pkt_buf.rd_irq_flag);
		} else {
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
			stp_sdio_txperf_fifo_left += p_info->firmware_info.tx_fifo_size;
			stp_sdio_txperf_to_send += four_byte_align_len;
			++stp_sdio_txperf_fifo_lmt_cnt;
#endif
			
			STPSDIO_DBG_FUNC
			    ("tx_fifo_size(%d), four_byte_align_len(%d), tx_packet_num(%d)\n",
			     p_info->firmware_info.tx_fifo_size, four_byte_align_len,
			     p_info->firmware_info.tx_packet_num);
			break;
		}

		stp_sdio_check_tx_sanity(p_info, 2);
	} while (1);

	STPSDIO_LOUD_FUNC("end\n");
}
#endif				

static VOID stp_sdio_rx_wkr(struct work_struct *work)
{
	PUINT8 bufp;
	UINT32 bus_rxlen;
	UINT32 chisr_rxlen;
	INT32 ret;
	MTK_WCN_STP_SDIO_HIF_INFO *p_info;

	p_info = container_of(work, MTK_WCN_STP_SDIO_HIF_INFO, rx_work);

	
	
	chisr_rxlen = p_info->rx_pkt_len;


	if (STP_SDIO_RX_FIFO_SIZE < chisr_rxlen) {
		
		STPSDIO_ERR_FUNC("abnormal chisr_rxlen(%d) rx_worker stop\n", chisr_rxlen);
		return;
	}

	bus_rxlen = chisr_rxlen;
	bus_rxlen += 0x3;
	bus_rxlen &= ~(0x3UL);

	
	if (STP_SDIO_BLK_SIZE < bus_rxlen) {
		bus_rxlen += (STP_SDIO_BLK_SIZE - 1);
		bus_rxlen &= ~((UINT32) STP_SDIO_BLK_SIZE - 1);
	}
	ret = mtk_wcn_hif_sdio_read_buf(p_info->sdio_cltctx,
					CRDR, (PUINT32) & p_info->pkt_buf.rx_buf[0], bus_rxlen);
	if (ret) {
		
		STPSDIO_ERR_FUNC("read CRDR len(%d) rx error!(%d)\n", bus_rxlen, ret);
		p_info->rx_pkt_len = 0;
		STPSDIO_HINT_FUNC("set to p_info->rx_pkt_len 0\n");
		osal_dbg_assert_aee("<HIF_SDIO> sdio_readsb ERROR","read data by SDIO report error");
		return;
	}
	p_info->rx_pkt_len = 0;
	STPSDIO_HINT_FUNC("set to p_info->rx_pkt_len 0\n");
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG
	do {
		UINT32 idx = stp_sdio_rxdbg_cnt++ & (STP_SDIO_RXDBG_COUNT - 1);
		
		stp_sdio_rxdbg_buffer[idx].ts = jiffies;
		stp_sdio_rxdbg_buffer[idx].chisr_rxlen = chisr_rxlen;
		stp_sdio_rxdbg_buffer[idx].bus_rxlen = bus_rxlen;
		
		memcpy(&stp_sdio_rxdbg_buffer[idx].rx_pkt_buf[0], &p_info->pkt_buf.rx_buf[0],
		       bus_rxlen);
	} while (0);
#endif

	bufp = &p_info->pkt_buf.rx_buf[4];

	
	
	bus_rxlen = p_info->pkt_buf.rx_buf[1];
	bus_rxlen = (bus_rxlen << 8) | p_info->pkt_buf.rx_buf[0];
	STPSDIO_DBG_FUNC("bus_rxlen(%d) rx_len in chisr(%d)\n", bus_rxlen, chisr_rxlen);
	if (bus_rxlen != chisr_rxlen) {
		STPSDIO_ERR_FUNC("abnormal bus_rxlen(%d) in SDIO packet header!in chisr(%d)\n",
				 bus_rxlen, chisr_rxlen);
		return;
	}
	if (p_info->pkt_buf.rx_buf[2] || p_info->pkt_buf.rx_buf[3]) {
		STPSDIO_ERR_FUNC("abnormal p_info->pkt_buf.rx_buf[2](0x%02x) [3](0x%02x)\n",
				 p_info->pkt_buf.rx_buf[2], p_info->pkt_buf.rx_buf[3]);
		return;
	}

	if (STP_SDIO_HDR_SIZE < bus_rxlen) {
		bus_rxlen -= STP_SDIO_HDR_SIZE;
		
        ret = mtk_wcn_stp_parser_data(bufp, bus_rxlen);
#if STP_SDIO_DBG_SUPPORT && (STP_SDIO_TXDBG || STP_SDIO_TXPERFDBG)
		if (ret && (0 == p_info->tx_dbg_dump_flag)) {
			p_info->tx_dbg_dump_flag = 1;
			stp_sdio_txdbg_dump();
		}
#endif
	} else {
		STPSDIO_ERR_FUNC("abnormal rx length(%d, %d)\n", bus_rxlen, chisr_rxlen);
	}

}

#if STP_SDIO_NEW_IRQ_HANDLER

static INT32 stp_sdio_irq(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx)
{

	INT32 iRet = 0;
	UINT32 chlcpr_value = 0;
	MTK_WCN_STP_SDIO_HIF_INFO *p_info = gp_info;

	STPSDIO_HINT_FUNC("disable IRQ\n");
	
#if COHEC_00006052
	iRet = mtk_wcn_hif_sdio_writeb(clt_ctx, CHLPCR, C_FW_INT_EN_CLR);
#else
	iRet = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_INT_EN_CLR);
#endif				
	if (iRet) {
		STPSDIO_ERR_FUNC("disalbe IRQ fail\n");
	} else {
		iRet = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
		if (iRet) {
			STPSDIO_ERR_FUNC("read CHLPCR fail. iRet(%d)\n", iRet);
		} else {
			if (!(chlcpr_value & C_FW_INT_EN_SET)) {
				STPSDIO_DBG_FUNC("disable COM IRQ okay (0x%x)\n", chlcpr_value);
				p_info->irq_pending = 1;

				
				STPSDIO_DBG_FUNC("signal stp_tx_rx\n");
				osal_trigger_event(&gp_info->tx_rx_event);
			} else {
				STPSDIO_ERR_FUNC
				    ("**********disable COM IRQ fail, don't signal stp-sdio thread******\n");
			}
		}

	}
	return 0;

}

#else
static INT32 stp_sdio_irq(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx)
{
	
	MTK_WCN_STP_SDIO_HIF_INFO *p_info = NULL;
	UINT32 chisr = 0;
	UINT32 tx_comp;
	INT32 ret;

	
	STPSDIO_LOUD_FUNC("enter\n");
	p_info = gp_info;
	ret = HIF_SDIO_ERR_SUCCESS;
	
	if (OWN_CLR == stp_sdio_get_own_state()) {
		STPSDIO_DBG_FUNC("OWN on driver side!\n");
	} else {
		STPSDIO_DBG_FUNC("OWN on fw side!\n");
		if (0 == stp_sdio_do_own_clr(0)) {
			STPSDIO_DBG_FUNC("set OWN to driver side ok!\n");
		} else {
			STPSDIO_ERR_FUNC("set OWN to driver side error!\n");
			return -1;
		}

	}
retry:
	
	ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHISR, &chisr);
	if (ret) {
		
		
        STPSDIO_ERR_FUNC("get CHISR information rx error,ret:%d\n", ret);
		if (-5 == ret)
		{
			STPSDIO_ERR_FUNC("get CHISR DAT CRC error, retry.\n");
			chisr = stp_sdio_rc_reg_readl_retry(clt_ctx, CSR, 100);
		}
		else
	        goto retry;
	}
	STPSDIO_HINT_FUNC("CHISR(0x%08x)\n", chisr);
	if (0x0 == chisr) {
		STPSDIO_ERR_FUNC("******CHISR == 0*****\n");
		return 0;
	}
	
	if (chisr & FW_OWN_BACK_INT) {
		STPSDIO_INFO_FUNC("FW_OWN_BACK_INT\n");

		if (is_wait_ownback) {
			is_wait_ownback = 0;
			wake_up(&g_ownback_done);
		} else {
			mtk_wcn_stp_wmt_sdio_host_awake();
			
			
			
		}
	}
	
	if (chisr & RX_DONE) {
		

		
		if (p_info->rx_pkt_len) {
			STPSDIO_ERR_FUNC("rx worker is not finished yet!!!(%d)\n",
					 p_info->rx_pkt_len);
		}

		
		p_info->rx_pkt_len = (chisr & RX_PKT_LEN) >> 16;
		STPSDIO_HINT_FUNC("rx_pkt_len(%d)\n", p_info->rx_pkt_len);
		
		if ((0 == p_info->rx_pkt_len)
		    || (STP_SDIO_RX_FIFO_SIZE < p_info->rx_pkt_len)) {
			STPSDIO_ERR_FUNC
			    ("abnormal rx_pkt_len(%d) in CHISR(0x%08x) skip rx_worker\n",
			     p_info->rx_pkt_len, chisr);
			p_info->rx_pkt_len = 0;
		} else {
			if (p_info->rx_pkt_len & 0x3) {
				STPSDIO_WARN_FUNC
				    ("rx data len is not 4 bytes allignment, CHISR(0x%08x), rx len (%d).\n",
				     chisr, p_info->rx_pkt_len);
			}
#if STP_SDIO_OWN_THREAD
			
			STPSDIO_DBG_FUNC("osal_trigger_event gp_info->tx_rx_event\n");
			osal_trigger_event(&gp_info->tx_rx_event);
#else
			schedule_work(&p_info->rx_work);
#endif
		}
	}
	
	if ((chisr & TX_EMPTY) || (chisr & TX_UNDER_THOLD)) {
		STPSDIO_DBG_FUNC("Tx interrupt\n");
		
		tx_comp = (chisr & TX_COMPLETE_COUNT) >> 4;
#if 0
		atomic_add(tx_comp, &p_info->firmware_info.tx_comp_num);
		
		tx_comp = atomic_read(&p_info->firmware_info.tx_comp_num);
#else
		tx_comp = atomic_add_return(tx_comp, &p_info->firmware_info.tx_comp_num);
#endif
		if (tx_comp > STP_SDIO_TX_PKT_MAX_CNT) {
			STPSDIO_ERR_FUNC("Abnormal accumulated comp count(%d) chisr(0x%x)\n",
					 tx_comp, chisr);
		}

		
		
#if STP_SDIO_OWN_THREAD
		
		STPSDIO_DBG_FUNC("osal_trigger_event gp_info->tx_rx_event\n");
		osal_trigger_event(&gp_info->tx_rx_event);
#else
		schedule_work(&p_info->tx_work);
#endif
	}

	STPSDIO_LOUD_FUNC("exit\n");
	return ret;
}
#endif

#if STP_SDIO_POLL_OWNBACK_INTR
static INT32
stp_sdio_ownback_poll(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx, UINT32 retry, UINT32 delay_us)
{
	INT32 ret;
	UINT32 chlpcr = 0;

	do {
		
		
		ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlpcr);
		if (ret) {
			
			STPSDIO_ERR_FUNC("get CHLPCR information rx error!(%d)\n", ret);
			return ret;
		}

		
		if (chlpcr & C_FW_COM_DRV_OWN) {
			
			STPSDIO_INFO_FUNC("Driver own is polled!(%d)\n", retry);
			break;
		}
		udelay(delay_us);
	} while (retry-- > 0);

	
	return (chlpcr & C_FW_COM_DRV_OWN) ? 0 : -HIF_SDIO_ERR_FAIL;
}
#endif

static INT32
stp_sdio_probe(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx,
	       const MTK_WCN_HIF_SDIO_FUNCINFO *sdio_func_infop)
{
	INT32 ret = HIF_SDIO_ERR_SUCCESS;
	UINT32 i = 0, chlcpr_value = 0;

	STPSDIO_DBG_FUNC("sdio_cltctx: 0x%08x\n", clt_ctx);

	if (g_stp_sdio_host_count) {
		STPSDIO_ERR_FUNC("g_stp_sdio_host_count(%d) probed already!\n",
				 g_stp_sdio_host_count);
		return -1;
	}
	
	if (g_stp_sdio_host_info.sdio_cltctx == clt_ctx) {
		STPSDIO_WARN_FUNC("sdio_cltctx(%d) already probed!\n", clt_ctx);
		return 0;
	}
	
	
	g_stp_sdio_host_info.sdio_cltctx = clt_ctx;

	
	for (i = 0; i < STP_SDIO_TX_BUF_CNT; ++i) {
#if KMALLOC_UPDATE
		UINT8 *pData = g_stp_sdio_host_info.pkt_buf.tx_buf + i * STP_SDIO_TX_ENTRY_SIZE + 0;
		memset(pData, 0, STP_SDIO_TX_ENTRY_SIZE);
#else
		memset(&g_stp_sdio_host_info.pkt_buf.tx_buf[i][0], 0,
		       sizeof(g_stp_sdio_host_info.pkt_buf.tx_buf[i]));
#endif
	}

#if STP_SDIO_NEW_TXRING
	spin_lock_init(&g_stp_sdio_host_info.pkt_buf.rd_cnt_lock);
	g_stp_sdio_host_info.pkt_buf.wr_cnt = 0;
	g_stp_sdio_host_info.pkt_buf.rd_cnt = 0;
#else
	
	spin_lock_init(&g_stp_sdio_host_info.pkt_buf.rd_idx_lock);
	g_stp_sdio_host_info.pkt_buf.wr_idx = 0;
	g_stp_sdio_host_info.pkt_buf.rd_idx = 0;
#endif
	g_stp_sdio_host_info.pkt_buf.full_flag = MTK_WCN_BOOL_FALSE;

	
	init_waitqueue_head(&g_stp_sdio_host_info.pkt_buf.fullwait_q);

	
	memset(&g_stp_sdio_host_info.tx_pkt_list.pkt_size_list[0], 0,
	       sizeof(g_stp_sdio_host_info.tx_pkt_list.pkt_size_list));
	g_stp_sdio_host_info.tx_pkt_list.pkt_rd_cnt = 0;
	g_stp_sdio_host_info.tx_pkt_list.pkt_wr_cnt = 0;

	
	

	
	g_stp_sdio_host_info.firmware_info.tx_fifo_size = STP_SDIO_TX_FIFO_SIZE;
	g_stp_sdio_host_info.firmware_info.tx_packet_num = 0;
	atomic_set(&g_stp_sdio_host_info.firmware_info.tx_comp_num, 0);

#if STP_SDIO_OWN_THREAD
	
	
	g_stp_sdio_host_info.irq_pending = 0;
	g_stp_sdio_host_info.sleep_flag = 0;
	g_stp_sdio_host_info.wakeup_flag = 0;
	osal_event_init(&g_stp_sdio_host_info.tx_rx_event);
	g_stp_sdio_host_info.tx_rx_thread.pThreadFunc = stp_sdio_tx_rx_handling;
	g_stp_sdio_host_info.tx_rx_thread.pThreadData = (PVOID) &g_stp_sdio_host_info;
	g_stp_sdio_host_info.tx_dbg_dump_flag = 0;
	osal_strncpy((PINT8)&g_stp_sdio_host_info.tx_rx_thread.threadName,
		     "stp_sdio_tx_rx", osal_sizeof(g_stp_sdio_host_info.tx_rx_thread.threadName));
	ret = osal_thread_create(&g_stp_sdio_host_info.tx_rx_thread);
	if (ret < 0) {
		STPSDIO_ERR_FUNC("osal_thread_create fail...\n");
		goto out;
	}
#else
	
	INIT_WORK(&g_stp_sdio_host_info.tx_work, stp_sdio_tx_wkr);
	INIT_WORK(&g_stp_sdio_host_info.rx_work, stp_sdio_rx_wkr);
#endif
	 
	g_stp_sdio_host_info.private_info.stp_sdio_host_idx = g_stp_sdio_host_count;
	mtk_wcn_hif_sdio_set_drvdata(clt_ctx, &g_stp_sdio_host_info.private_info);

	++g_stp_sdio_host_count;

	
#if COHEC_00006052
	ret =
	    mtk_wcn_hif_sdio_writeb(clt_ctx, (UINT32) (CHLPCR + 0x1),
				    (UINT8) (C_FW_OWN_REQ_CLR >> 8));
#else
	ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_OWN_REQ_CLR);
#endif				
	if (ret) {
		STPSDIO_ERR_FUNC("request FW-Own back fail!(%d)\n", ret);
		goto out;
	}
	STPSDIO_INFO_FUNC("request FW-Own back done\n");

#if STP_SDIO_POLL_OWNBACK_INTR
	
	ret = stp_sdio_ownback_poll(clt_ctx, 10, 100);
	if (ret) {
		STPSDIO_ERR_FUNC("poll FW-Own back fail!(%d)\n", ret);
		goto out;
	}
#endif
	
	mtk_wcn_hif_sdio_enable_irq(clt_ctx, MTK_WCN_BOOL_TRUE);
	
	 
	ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHIER, (FIRMWARE_INT | TX_FIFO_OVERFLOW |
						       FW_INT_IND_INDICATOR | TX_COMPLETE_COUNT |
						       TX_UNDER_THOLD | TX_EMPTY | RX_DONE));
	if (ret) {
		STPSDIO_ERR_FUNC("set interrupt output fail!(%d)\n", ret);
		goto out;
	}
	STPSDIO_DBG_FUNC("set interrupt output done\n");
#if COHEC_00006052
	ret = mtk_wcn_hif_sdio_writeb(clt_ctx, CHLPCR, C_FW_INT_EN_SET);	
#else
	ret = mtk_wcn_hif_sdio_writel(clt_ctx, CHLPCR, C_FW_INT_EN_SET);	
#endif				
	if (ret) {
		STPSDIO_ERR_FUNC("enable interrupt fail!(%d)\n", ret);
		goto out;
	}
	ret = mtk_wcn_hif_sdio_readl(clt_ctx, CHLPCR, &chlcpr_value);
	if (ret) {
		STPSDIO_ERR_FUNC("Read CHLPCR fail!(%d)\n", ret);
		goto out;
	} else {
		if (chlcpr_value & C_FW_INT_EN_SET) {
			STPSDIO_DBG_FUNC("enable interrupt okay (0x%x)\n", chlcpr_value);
		}
	}

	STPSDIO_DBG_FUNC("enable interrupt done\n");

#if STP_SDIO_OWN_THREAD
	ret = osal_thread_run(&g_stp_sdio_host_info.tx_rx_thread);
	if (ret < 0) {
		STPSDIO_ERR_FUNC("osal_thread_run fail...\n");
		goto out;
	}
#endif

	
	mtk_wcn_stp_register_if_tx(STP_SDIO_IF_TX, (MTK_WCN_STP_IF_TX)stp_sdio_tx);

#if 0				
	
	pr_warn(KERN_INFO DFT_TAG "%s: set stp sdio mode\n", __func__);
	mtk_wcn_stp_set_sdio_mode(1);

	
	pr_warn(KERN_INFO DFT_TAG "%s: stp enable\n", __func__);
	mtk_wcn_stp_enable(1);
#endif

 out:
	
	
	if (ret) {
#if STP_SDIO_OWN_THREAD
		osal_thread_destroy(&g_stp_sdio_host_info.tx_rx_thread);
#endif
		--g_stp_sdio_host_count;
	}
	return ret;
}

static INT32 stp_sdio_remove(const MTK_WCN_HIF_SDIO_CLTCTX clt_ctx)
{

	STPSDIO_LOUD_FUNC("start\n");
	if (g_stp_sdio_host_info.sdio_cltctx == clt_ctx) {
		STPSDIO_INFO_FUNC("sdio_cltctx(%d) found\n", clt_ctx);
	} else {
		STPSDIO_ERR_FUNC("sdio_cltctx(%d) not found\n", clt_ctx);
		return -1;
	}
    --g_stp_sdio_host_count;
	
	mtk_wcn_hif_sdio_enable_irq(clt_ctx, MTK_WCN_BOOL_FALSE);
	
	mtk_wcn_stp_register_if_tx(STP_SDIO_IF_TX, NULL);

	
#if STP_SDIO_OWN_THREAD
	
	
	osal_thread_destroy(&g_stp_sdio_host_info.tx_rx_thread);
	osal_event_deinit(&g_stp_sdio_host_info.tx_rx_event);
	STPSDIO_INFO_FUNC("destroy STP-SDIO tx_rx_thread\n");
#else
	flush_scheduled_work();
	STPSDIO_INFO_FUNC("flush scheduled work end\n");
#endif

	
	

	
	stp_sdio_host_info_op(1);


	STPSDIO_INFO_FUNC("clear g_stp_sdio_host_info[p_priv->stp_sdio_host_idx] done\n");

	STPSDIO_LOUD_FUNC("end\n");
	return 0;
}
static INT32 stp_sdio_rc_reg_readl_retry (MTK_WCN_HIF_SDIO_CLTCTX clt_ctx, UINT32 offset, UINT32 retry_limit, INT32 *p_ret)
{
	INT32 ret = -1;
	INT32 value = 0;
	#define MAX_RETRY_NUM 100
	
	UINT32 card_id = CLTCTX_CID(clt_ctx);
	if (card_id != 0x6630)
	{
		STPSDIO_INFO_FUNC("card_id is :0x%x, does not support CSR (Common Snapshot Register)\n", card_id);
		if (p_ret)
			*p_ret = 0;
		return value;
	}
	STPSDIO_INFO_FUNC("clt_ctx:0x%x, offset:0x%x, retry_limit:%d\n", clt_ctx, offset, retry_limit);

	retry_limit = retry_limit == 0 ? 1 : retry_limit;
	retry_limit = retry_limit > MAX_RETRY_NUM ? MAX_RETRY_NUM : retry_limit;
	
	while (retry_limit > 0)
	{
		ret = mtk_wcn_hif_sdio_readl(clt_ctx, offset, &value);
		if (ret != 0)
		{
			STPSDIO_ERR_FUNC("read Common Snapshot Register failed, ret:%d\n", ret);
		}else
		{
			STPSDIO_DBG_FUNC("CSR:0x%x\n", value);
			break;
		}
		retry_limit--;
	}
	if (p_ret)
		*p_ret = ret;
	return value;
}


#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG

#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_rxdbg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	UINT32 idx;
	UINT32 i;
	UINT32 j;
	PUINT8 pbuf;
	UINT32 len;

	if (*f_pos > 0) {
		return 0;
	}

	for (i = 0; i < STP_SDIO_RXDBG_COUNT; ++i) {
		idx = (stp_sdio_rxdbg_cnt - 1 - i) & STP_SDIO_TXDBG_COUNT_MASK;
		len = stp_sdio_rxdbg_buffer[idx].bus_rxlen;
		if (0 == len) {
			pr_warn(KERN_INFO DFT_TAG "idx(0x%x) 0 == len dump skip\n",
				stp_sdio_rxdbg_cnt);
		}
		pr_warn(KERN_INFO DFT_TAG "idx(0x%x) chisr_rxlen(%d) bus_rxlen(%d) ts(%d)\n",
			stp_sdio_rxdbg_cnt, stp_sdio_rxdbg_buffer[idx].chisr_rxlen, len,
			stp_sdio_rxdbg_buffer[idx].ts);
		for (j = 0; j < STP_SDIO_RX_BUF_SIZE && j < len; j += 16) {
			pbuf = &stp_sdio_rxdbg_buffer[idx].rx_pkt_buf[j];
			pr_warn(KERN_INFO DFT_TAG
				"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x   0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6],
				pbuf[7], pbuf[8], pbuf[9], pbuf[10], pbuf[11], pbuf[12], pbuf[13],
				pbuf[14], pbuf[15]);
			msleep(10);
		}
		pr_warn(KERN_INFO DFT_TAG "dump ok\n");
	}

	return 0;
}

ssize_t stp_sdio_rxdbg_write(struct file *filp, const char __user *buf, size_t count,
			     loff_t *f_pos)
{
	unsigned long len = count;
	pr_warn(KERN_INFO DFT_TAG "write parameter len = %lu\n\r", len);

	return len;
}

#else

static int stp_sdio_rxdbg_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	UINT32 idx;
	UINT32 i;
	UINT32 j;
	PUINT8 pbuf;
	UINT32 len;

	if (off > 0) {
		return 0;
	}

	for (i = 0; i < STP_SDIO_RXDBG_COUNT; ++i) {
		idx = (stp_sdio_rxdbg_cnt - 1 - i) & STP_SDIO_TXDBG_COUNT_MASK;
		len = stp_sdio_rxdbg_buffer[idx].bus_rxlen;
		if (0 == len) {
			pr_warn(KERN_INFO DFT_TAG "idx(0x%x) 0 == len dump skip\n",
				stp_sdio_rxdbg_cnt);
		}
		pr_warn(KERN_INFO DFT_TAG "idx(0x%x) chisr_rxlen(%d) bus_rxlen(%d) ts(%d)\n",
			stp_sdio_rxdbg_cnt, stp_sdio_rxdbg_buffer[idx].chisr_rxlen, len,
			stp_sdio_rxdbg_buffer[idx].ts);
		for (j = 0; j < STP_SDIO_RX_BUF_SIZE && j < len; j += 16) {
			pbuf = &stp_sdio_rxdbg_buffer[idx].rx_pkt_buf[j];
			pr_warn(KERN_INFO DFT_TAG
				"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x   0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6],
				pbuf[7], pbuf[8], pbuf[9], pbuf[10], pbuf[11], pbuf[12], pbuf[13],
				pbuf[14], pbuf[15]);
			msleep(10);
		}
		pr_warn(KERN_INFO DFT_TAG "dump ok\n");
	}

	return 0;
}

static int
stp_sdio_rxdbg_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long len = count;
	pr_warn(KERN_INFO DFT_TAG "write parameter len = %lu\n\r", len);
	return len;
}
#endif

INT32 stp_sdio_rxdbg_setup(VOID)
{
	stp_sdio_rxdbg_cnt = 0;

#if USE_NEW_PROC_FS_FLAG
	gStpSdioRxDbgEntry = proc_create(STP_SDIO_RXDBG_PROCNAME, 0644, NULL, &stp_sdio_rxdbg_fops);
	if (gStpSdioRxDbgEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
#else
	gStpSdioRxDbgEntry = create_proc_entry(STP_SDIO_RXDBG_PROCNAME, 0644, NULL);
	if (gStpSdioRxDbgEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
	gStpSdioRxDbgEntry->read_proc = stp_sdio_rxdbg_read;
	gStpSdioRxDbgEntry->write_proc = stp_sdio_rxdbg_write;

	pr_warn(KERN_INFO DFT_TAG "Create /proc entry succeed\n\r");
#endif

	return 0;
}

INT32 stp_sdio_rxdbg_remove(VOID)
{
#if USE_NEW_PROC_FS_FLAG
	if (NULL != gStpSdioRxDbgEntry) {
		proc_remove(gStpSdioRxDbgEntry);
	}
#else

	if (NULL != gStpSdioRxDbgEntry) {
		remove_proc_entry(STP_SDIO_RXDBG_PROCNAME, NULL);
	}
#endif
	return 0;
}
#endif

#if STP_SDIO_DBG_SUPPORT && (STP_SDIO_TXDBG || STP_SDIO_TXPERFDBG)

static VOID stp_sdio_txperf_dump(VOID)
{
#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXPERFDBG
	UINT32 cnt;
	UINT32 fifo;
	UINT32 data;
	UINT32 wkr;
	UINT32 pkt_num;
	UINT32 lmt_cnt;

	
	cnt = stp_sdio_txperf_fifo_lmt_cnt;
	fifo = stp_sdio_txperf_fifo_left;
	data = stp_sdio_txperf_to_send;

	wkr = stp_sdio_txperf_worker_cnt;
	pkt_num = stp_sdio_txperf_txed_pkt_num;
	lmt_cnt = stp_sdio_txperf_pkt_num_lmt_cnt;

	pr_warn(KERN_INFO DFT_TAG "txwait_fifo_left(%d), txwait_to_send(%d), txwait_count(%d)\n",
		fifo, data, cnt);
	if (cnt) {
		pr_warn(KERN_INFO DFT_TAG "avg left(%d), to_send(%d)\n",
			(fifo / cnt), (data / cnt));
	}
	pr_warn(KERN_INFO DFT_TAG "tx_worker_cnt(%d), pkt_num(%d), pkt_num_lmt_cnt(%d)\n",
		wkr, pkt_num, lmt_cnt);

#endif
}

static VOID stp_sdio_txdbg_dump(VOID)
{
#if STP_SDIO_TXDBG
	UINT32 idx;
	UINT32 i;
	UINT32 j;
	PUINT8 pbuf;
	UINT32 len;

	for (i = 0; i < STP_SDIO_TXDBG_COUNT; ++i) {
		idx = (stp_sdio_txdbg_cnt - 1 - i) & STP_SDIO_TXDBG_COUNT_MASK;
		len = stp_sdio_txdbg_buffer[idx].bus_txlen;
		if (0 == len) {
			STPSDIO_DBG_FUNC("idx(%x) 0 == len dump skip\n", idx);
			continue;
		}

		len = len > STP_SDIO_TXDBG_MAX_SIZE ? STP_SDIO_TXDBG_MAX_SIZE : len;
		STPSDIO_DBG_FUNC(
				"stp_sdio_txdbg_buffer idx(%x) bus_txlen(0x%x, %d) ts(%d)\n", idx, len, len,
			stp_sdio_txdbg_buffer[idx].ts);
		for (j = 0; j < STP_SDIO_TX_ENTRY_SIZE && j < len; j += 16) {
			pbuf = &stp_sdio_txdbg_buffer[idx].tx_pkt_buf[j];
			STPSDIO_DBG_FUNC(
				"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x   0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
				pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6],
				pbuf[7], pbuf[8], pbuf[9], pbuf[10], pbuf[11], pbuf[12], pbuf[13],
				pbuf[14], pbuf[15]);
			msleep(10);
		}
		STPSDIO_DBG_FUNC("stp_sdio_txdbg_buffer dump ok\n");
	}

	for (i = 0; i < STP_SDIO_TXDBG_COUNT; ++i) {
		idx = (stp_sdio_txdbg_cnt - 1 - i) & STP_SDIO_TXDBG_COUNT_MASK;
		len = stp_sdio_txdbg_buffer[idx].bus_txlen;
		STPSDIO_DBG_FUNC(
			"stp_sdio_txdbg_buffer idx(%x) bus_txlen(0x%x, %d) ts(%d)\n", idx, len, len,
			stp_sdio_txdbg_buffer[idx].ts);
	}
	STPSDIO_DBG_FUNC(
			"Dump tx info: pkt_num(%d) fifo(%d) pkt_list.rd(0x%x, %ld) pkt_list.wr(0x%x, %ld)\n",
			gp_info->firmware_info.tx_packet_num, gp_info->firmware_info.tx_fifo_size,
			gp_info->tx_pkt_list.pkt_rd_cnt,
			STP_SDIO_GET_PKT_AR_IDX(gp_info->tx_pkt_list.pkt_rd_cnt),
			gp_info->tx_pkt_list.pkt_wr_cnt,
			STP_SDIO_GET_PKT_AR_IDX(gp_info->tx_pkt_list.pkt_wr_cnt));

	for (i = 0; i < STP_SDIO_TX_PKT_LIST_SIZE; ++i) {
		idx = STP_SDIO_GET_PKT_AR_IDX(gp_info->tx_pkt_list.pkt_wr_cnt - 1 - i);
		STPSDIO_DBG_FUNC(
				"tx_pkt_list idx(0x%x, %d) size(0x%x, %d), in_ts(%d), out_ts(%d)\n",
				(gp_info->tx_pkt_list.pkt_wr_cnt - 1 - i), idx,
				gp_info->tx_pkt_list.pkt_size_list[idx],
				gp_info->tx_pkt_list.pkt_size_list[idx], gp_info->tx_pkt_list.in_ts[idx],
				gp_info->tx_pkt_list.out_ts[idx]);
	}

#if STP_SDIO_NEW_TXRING
	STPSDIO_DBG_FUNC("\n\ndump pkt_buf.tx_buf: rd(%d) wr(%d) full(%d)\n",
			gp_info->pkt_buf.rd_cnt, gp_info->pkt_buf.wr_cnt, gp_info->pkt_buf.full_flag);
#else
	STPSDIO_DBG_FUNC("\n\ndump pkt_buf.tx_buf: rdi(%d) wri(%d) full(%d)\n",
			gp_info->pkt_buf.rd_idx, gp_info->pkt_buf.wr_idx, gp_info->pkt_buf.full_flag);
#endif

	for (i = 0; i < STP_SDIO_TX_BUF_CNT; ++i) {
#if STP_SDIO_NEW_TXRING
		idx = (gp_info->pkt_buf.wr_cnt - 1 - i) & STP_SDIO_TX_BUF_CNT_MASK;
		len = gp_info->pkt_buf.tx_buf_sz[idx];
#else
		idx = (gp_info->pkt_buf.wr_idx - 1 - i + STP_SDIO_TX_BUF_CNT) % STP_SDIO_TX_BUF_CNT;
#if KMALLOC_UPDATE
		len = *(gp_info->pkt_buf.tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + 1);
		len = (len << 8) | *(gp_info->pkt_buf.tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + 0);
#else
		len = gp_info->pkt_buf.tx_buf[idx][1];
		len = (len << 8) | gp_info->pkt_buf.tx_buf[idx][0];
#endif

#endif
		STPSDIO_DBG_FUNC("pkt_buf.tx_buf idx(%x) ts(%d) len(%d)\n",
				idx, gp_info->pkt_buf.tx_buf_ts[idx], len);
        if (0 == len) {
			STPSDIO_DBG_FUNC("idx(%x) 0 == len dump skip\n", idx);
			continue;
        }
		len = len > STP_SDIO_TXDBG_MAX_SIZE ? STP_SDIO_TXDBG_MAX_SIZE : len;
		for (j = 0; j < STP_SDIO_TX_ENTRY_SIZE && j < len; j += 16) {
#if KMALLOC_UPDATE
			pbuf = gp_info->pkt_buf.tx_buf + idx * STP_SDIO_TX_ENTRY_SIZE + j;
#else
			pbuf = &gp_info->pkt_buf.tx_buf[idx][j];
#endif
			STPSDIO_DBG_FUNC(
					"[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x   0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n",
					pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6],
					pbuf[7], pbuf[8], pbuf[9], pbuf[10], pbuf[11], pbuf[12], pbuf[13],
					pbuf[14], pbuf[15]);
			msleep(10);
		}
		STPSDIO_DBG_FUNC("pkt_buf.tx_buf dump ok\n");
	}
#endif				
}

#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_txdbg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	if (*f_pos > 0) {
		return 0;
	}

	stp_sdio_txdbg_dump();
	stp_sdio_txperf_dump();
	return 0;
}

ssize_t stp_sdio_txdbg_write(struct file *filp, const char __user *buf, size_t count,
			     loff_t *f_pos)
{
	unsigned long len = count;
	pr_warn(KERN_INFO DFT_TAG "write parameter len = %lu\n\r", len);
	return len;
}

#else

static int stp_sdio_txdbg_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	if (off > 0) {
		return 0;
	}

	stp_sdio_txdbg_dump();
	stp_sdio_txperf_dump();
	return 0;
}

static int
stp_sdio_txdbg_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long len = count;
	pr_warn(KERN_INFO DFT_TAG "write parameter len = %lu\n\r", len);
	return len;
}
#endif



#if USE_NEW_PROC_FS_FLAG


ssize_t stp_sdio_own_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	if (*f_pos > 0) {
		return 0;
	}

	return 0;

}

#else

static int
stp_sdio_owndbg_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	if (off > 0) {
		return 0;
	}

	return 0;
}
#endif

#if USE_NEW_PROC_FS_FLAG
ssize_t stp_sdio_own_write(struct file *filp, const char __user *buffer, size_t count,
			   loff_t *f_pos)
#else
static int
stp_sdio_owndbg_write(struct file *file, const char *buffer, unsigned long count, void *data)
#endif
{
	unsigned long len = count;
	PINT8 pBuf = NULL;
	PINT8 pToken = NULL;
	PINT8 pDelimiter = " \t";
	INT32 x = 0;
	INT8 buf[128] = { 0 };
	if (len >= osal_sizeof(buf)) {
		STPSDIO_ERR_FUNC("input handling fail!\n");
		len = osal_sizeof(buf) - 1;
		return -1;
	}

	if (copy_from_user(buf, buffer, len)) {
		STPSDIO_ERR_FUNC("copy_from_user error.\n");
		return -EFAULT;
	}
	buf[len] = '\0';
	pBuf = buf;
	pToken = osal_strsep(&pBuf, pDelimiter);
	x = NULL != pToken ? osal_strtol(pToken, NULL, 16) : 0;
	if (0 == x) {
		STPSDIO_INFO_FUNC("stp_sdio_own_ctrl(OWN_CLR)\n\r");
		stp_sdio_own_ctrl(OWN_CLR);
	} else if (2 == x) {
		STPSDIO_INFO_FUNC("stp_sdio_own_ctrl(OWN_SET) -->Sleep\n\r");
		stp_sdio_own_ctrl(OWN_SET);
	} else if (3 == x) {
		gStpSdioDbgLvl = STPSDIO_LOG_WARN;
		STPSDIO_WARN_FUNC("set STP-SDIO LogLevel to STPSDIO_LOG_WARN\n\r");
	} else if (4 == x) {
		gStpSdioDbgLvl = STPSDIO_LOG_INFO;
		STPSDIO_INFO_FUNC("set STP-SDIO LogLevel to STPSDIO_LOG_INFO\n\r");
	} else if (5 == x) {
		gStpSdioDbgLvl = STPSDIO_LOG_HINT;
		STPSDIO_INFO_FUNC("set STP-SDIO LogLevel to STPSDIO_LOG_HINT\n\r");
	} else if (6 == x) {
		gStpSdioDbgLvl = STPSDIO_LOG_DBG;
		STPSDIO_INFO_FUNC("set STP-SDIO LogLevel to STPSDIO_LOG_DBG\n\r");
	} else if (7 == x) {
		gStpSdioDbgLvl = STPSDIO_LOG_LOUD;
		STPSDIO_INFO_FUNC("set STP-SDIO LogLevel to STPSDIO_LOG_LOUD\n\r");
	}

	return len;
}


INT32 stp_sdio_txdbg_setup(VOID)
{
#if USE_NEW_PROC_FS_FLAG
	gStpSdioTxDbgEntry = proc_create(STP_SDIO_TXDBG_PROCNAME, 0644, NULL, &stp_sdio_txdbg_fops);
	if (gStpSdioTxDbgEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
#else

	gStpSdioTxDbgEntry = create_proc_entry(STP_SDIO_TXDBG_PROCNAME, 0644, NULL);
	if (gStpSdioTxDbgEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
	gStpSdioTxDbgEntry->read_proc = stp_sdio_txdbg_read;
	gStpSdioTxDbgEntry->write_proc = stp_sdio_txdbg_write;

	pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
#endif

#if STP_SDIO_TXPERFDBG
	stp_sdio_txperf_worker_cnt = 0;

	stp_sdio_txperf_fifo_left = 0;
	stp_sdio_txperf_to_send = 0;
	stp_sdio_txperf_fifo_lmt_cnt = 0;

	stp_sdio_txperf_txed_pkt_num = 0;
	stp_sdio_txperf_pkt_num_lmt_cnt = 0;
#endif

#if STP_SDIO_TXDBG
	stp_sdio_txdbg_cnt = 0;
#endif

	return 0;
}

INT32 stp_sdio_txdbg_remove(VOID)
{

#if USE_NEW_PROC_FS_FLAG
	if (NULL != gStpSdioTxDbgEntry) {
		proc_remove(gStpSdioTxDbgEntry);
	}
#else

	if (NULL != gStpSdioTxDbgEntry) {
		remove_proc_entry(STP_SDIO_TXDBG_PROCNAME, NULL);
	}
#endif
	return 0;
}

#endif				

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_OWNBACKDBG

INT32 stp_sdio_owndbg_setup(VOID)
{
#if USE_NEW_PROC_FS_FLAG
	gStpSdioOwnEntry = proc_create(STP_SDIO_OWNDBG_PROCNAME, 0644, NULL, &stp_sdio_own_fops);
	if (gStpSdioOwnEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
#else

	gStpSdioOwnEntry = create_proc_entry(STP_SDIO_OWNDBG_PROCNAME, 0644, NULL);
	if (gStpSdioOwnEntry == NULL) {
		pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
		return -1;
	}
	gStpSdioOwnEntry->read_proc = stp_sdio_owndbg_read;
	gStpSdioOwnEntry->write_proc = stp_sdio_owndbg_write;

	pr_warn(KERN_WARNING DFT_TAG "Unable to create /proc entry\n\r");
#endif
	return 0;
}



INT32 stp_sdio_owndbg_remove(VOID)
{
#if USE_NEW_PROC_FS_FLAG
	if (NULL != gStpSdioOwnEntry) {
		proc_remove(gStpSdioOwnEntry);
	}
#else

	if (NULL != gStpSdioOwnEntry) {
		remove_proc_entry(STP_SDIO_OWNDBG_PROCNAME, NULL);
	}
#endif
	return 0;
}
#endif

static int stp_sdio_init(void)
{
	INT32 ret;
	INT32 i;

	STPSDIO_LOUD_FUNC("start\n");

	
	stp_sdio_host_info_op(1);
	g_stp_sdio_host_count = 0;
	
#if 0				
	g_stp_sdio_cltinfo.func_tbl = mtk_stp_sdio_id_tbl;
	g_stp_sdio_cltinfo.func_tbl_size =
	    sizeof(mtk_stp_sdio_id_tbl) / sizeof(MTK_WCN_HIF_SDIO_FUNCINFO) - 1;
	g_stp_sdio_cltinfo.hif_clt_irq = stp_sdio_irq;
	g_stp_sdio_cltinfo.hif_clt_probe = stp_sdio_probe;
	g_stp_sdio_cltinfo.hif_clt_remove = stp_sdio_remove;
#endif

	STPSDIO_INFO_FUNC("cltinfo func table size:%d\n", g_stp_sdio_cltinfo.func_tbl_size);
	for (i = 0; i < g_stp_sdio_cltinfo.func_tbl_size; i++) {
		STPSDIO_INFO_FUNC("manf_id:0x%x, card_id:0x%x, func_num:%d, blk_size:%d\n",
				  mtk_stp_sdio_id_tbl[i].manf_id, mtk_stp_sdio_id_tbl[i].card_id,
				  mtk_stp_sdio_id_tbl[i].func_num, mtk_stp_sdio_id_tbl[i].blk_sz);
	}

	
	ret = mtk_wcn_hif_sdio_client_reg(&g_stp_sdio_cltinfo);
	if (ret) {
		STPSDIO_ERR_FUNC("mtk_wcn_hif_sdio_client_reg fail(%d)!\n", ret);
	}

	ret = mtk_wcn_stp_wmt_sdio_op_reg(stp_sdio_own_ctrl);
	if (ret) {
		STPSDIO_ERR_FUNC
		    ("mtk_wcn_stp_wmt_sdio_op_reg(mtk_wcn_stp_sdio_own_ctrl) fail(%d)!\n", ret);
	}

	STPSDIO_LOUD_FUNC("end\n");

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG
	stp_sdio_rxdbg_setup();
#endif

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
	stp_sdio_txdbg_setup();
#endif

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_OWNBACKDBG
	stp_sdio_owndbg_setup();
#endif

	STPSDIO_INFO_FUNC
	    ("blk_size(%ld), tx_buf_cnt(%ld), fifo tx(%ld) rx(%ld), buf tx(%ld) rx(%ld)\n",
	     STP_SDIO_BLK_SIZE, STP_SDIO_TX_BUF_CNT, STP_SDIO_TX_FIFO_SIZE, STP_SDIO_RX_FIFO_SIZE,
	     STP_SDIO_TX_ENTRY_SIZE, STP_SDIO_TX_ENTRY_SIZE);

	return ret;
}

static void stp_sdio_exit(void)
{
	STPSDIO_LOUD_FUNC("start\n");

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_TXDBG
	stp_sdio_txdbg_remove();
#endif

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_RXDBG
	stp_sdio_rxdbg_remove();
#endif

#if STP_SDIO_DBG_SUPPORT && STP_SDIO_OWNBACKDBG
	stp_sdio_owndbg_remove();
#endif

	
	mtk_wcn_stp_register_if_tx(STP_SDIO_IF_TX, 0x0);

	
	
	mtk_wcn_hif_sdio_client_unreg(&g_stp_sdio_cltinfo);

	
	flush_scheduled_work();
	STPSDIO_DBG_FUNC("flush scheduled work end\n");

	
	

	
	g_stp_sdio_host_count = 0;
	stp_sdio_host_info_op(0);
	
	STPSDIO_DBG_FUNC("end\n");

	return;
}



#ifdef MTK_WCN_REMOVE_KERNEL_MODULE

INT32 mtk_wcn_stp_sdio_drv_init(VOID)
{
	return stp_sdio_init();

}

VOID mtk_wcn_stp_sdio_drv_exit(VOID)
{
	return stp_sdio_exit();
}


EXPORT_SYMBOL(mtk_wcn_stp_sdio_drv_init);
EXPORT_SYMBOL(mtk_wcn_stp_sdio_drv_exit);
#else

module_init(stp_sdio_init);
module_exit(stp_sdio_exit);

#endif
