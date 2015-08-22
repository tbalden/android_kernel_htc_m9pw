#include <linux/uaccess.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <mach/mt_smi.h>
#include <mach/mt_vcore_dvfs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include <linux/mtk_gpu_utility.h>

#include "mmdvfs_mgr.h"

#undef pr_fmt
#define pr_fmt(fmt) "[" MMDVFS_LOG_TAG "]" fmt

#if IS_ENABLED(CONFIG_ARM64)
	
	#define MMDVFS_ENABLE	1
#else
	
	#define MMDVFS_ENABLE	0
#endif

#if MMDVFS_ENABLE
#include <mach/fliper.h>
#endif

#define MMDVFS_ENABLE_WQHD	0

#define MMDVFS_GPU_LOADING_NUM	30
#define MMDVFS_GPU_LOADING_START_INDEX	10
#define MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS	100
#define MMDVFS_GPU_LOADING_THRESHOLD	18

#if defined(CONFIG_MTK_VCORE10_FEATURE)
#define MMDVFS_WQHD_1_0V
#endif

#ifdef MMDVFS_WQHD_1_0V
#include "disp_session.h"
extern int primary_display_switch_mode_for_mmdvfs(int sess_mode, unsigned int session, int blocking);
#endif

#if (MMDVFS_GPU_LOADING_START_INDEX >= MMDVFS_GPU_LOADING_NUM)
	#error "start index too large"
#endif

#define MMDVFS_PIXEL_NUM_720P	(1280 * 720)
#define MMDVFS_PIXEL_NUM_2160P	(3840 * 2160)
#define MMDVFS_PIXEL_NUM_1080P	(2100 * 1300)
#define MMDVFS_PIXEL_NUM_2M		(2100 * 1300)
#define MMDVFS_PIXEL_NUM_SENSOR_FULL (13000000)

#define MMDVFS_DISPLAY_SIZE_FHD	(1920 * 1216)

extern unsigned int DISP_GetScreenWidth(void);
extern unsigned int DISP_GetScreenHeight(void);

enum {
	MMDVFS_CAM_MON_SCEN = SMI_BWC_SCEN_CNT,
	MMDVFS_SCEN_MHL,
	MMDVFS_SCEN_COUNT
};

static mmdvfs_voltage_enum g_mmdvfs_scenario_voltage[MMDVFS_SCEN_COUNT] = {MMDVFS_VOLTAGE_DEFAULT};
static mmdvfs_voltage_enum g_mmdvfs_current_step;
static unsigned int g_mmdvfs_concurrency;
static MTK_SMI_BWC_MM_INFO *g_mmdvfs_info;
static MTK_MMDVFS_CMD g_mmdvfs_cmd;

typedef struct
{
	
	struct timer_list timer;

	
	struct workqueue_struct *work_queue;
	struct work_struct work;
	
	
	unsigned int gpu_loadings[MMDVFS_GPU_LOADING_NUM];
	int gpu_loading_index;
} mmdvfs_gpu_monitor_struct;

typedef struct
{
	spinlock_t scen_lock;
	int is_mhl_enable;
	mmdvfs_gpu_monitor_struct gpu_monitor;

} mmdvfs_context_struct;

typedef enum
{	
	MMDVFS_STEP_LOW = 0,
	MMDVFS_STEP_HIGH,

	MMDVFS_STEP_LOW2LOW,		
	MMDVFS_STEP_HIGH2LOW,	
	MMDVFS_STEP_LOW2HIGH,	
	MMDVFS_STEP_HIGH2HIGH,  
} mmdvfs_step_enum;

typedef enum
{
	MMDVFS_LCD_SIZE_FHD,
	MMDVFS_LCD_SIZE_WQHD,
	MMDVFS_LCD_SIZE_END_OF_ENUM
} mmdvfs_lcd_size_enum;

static mmdvfs_context_struct g_mmdvfs_mgr_cntx;
static mmdvfs_context_struct * const g_mmdvfs_mgr = &g_mmdvfs_mgr_cntx;

static mmdvfs_lcd_size_enum mmdvfs_get_lcd_resolution(void)
{
	if (DISP_GetScreenWidth() * DISP_GetScreenHeight() <= MMDVFS_DISPLAY_SIZE_FHD) {
		return MMDVFS_LCD_SIZE_FHD;
	}

	return MMDVFS_LCD_SIZE_WQHD;
}

static mmdvfs_voltage_enum mmdvfs_get_default_step(void)
{
#ifdef MMDVFS_WQHD_1_0V
	return MMDVFS_VOLTAGE_LOW;
#else
	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_FHD) {
		return MMDVFS_VOLTAGE_LOW;
	}

	return MMDVFS_VOLTAGE_HIGH;
#endif
}

static mmdvfs_voltage_enum mmdvfs_get_current_step(void)
{
	return g_mmdvfs_current_step;
}

static mmdvfs_voltage_enum mmdvfs_query(MTK_SMI_BWC_SCEN scenario, MTK_MMDVFS_CMD *cmd)
{
	mmdvfs_voltage_enum step = mmdvfs_get_default_step();
	unsigned int venc_size;
	MTK_MMDVFS_CMD cmd_default;

	venc_size = g_mmdvfs_info->video_record_size[0] * g_mmdvfs_info->video_record_size[1];

	
	if (cmd == NULL) {
		memset(&cmd_default, 0, sizeof(MTK_MMDVFS_CMD));
		cmd_default.camera_mode = MMDVFS_CAMERA_MODE_FLAG_DEFAULT;
		cmd = &cmd_default;
	}

	
	if (cmd->sensor_size == 0) {
		cmd->sensor_size = g_mmdvfs_cmd.sensor_size;
	}

	if (cmd->sensor_fps == 0) {
		cmd->sensor_fps = g_mmdvfs_cmd.sensor_fps;
	}

	if (cmd->camera_mode == MMDVFS_CAMERA_MODE_FLAG_DEFAULT) {
		cmd->camera_mode = g_mmdvfs_cmd.camera_mode;
	}	

	
	switch (scenario) {
		case SMI_BWC_SCEN_VR:
			if (cmd->sensor_size >= MMDVFS_PIXEL_NUM_2160P) {
				
				step = MMDVFS_VOLTAGE_HIGH;
			} else if (cmd->camera_mode &
					   (MMDVFS_CAMERA_MODE_FLAG_PIP | MMDVFS_CAMERA_MODE_FLAG_VFB | MMDVFS_CAMERA_MODE_FLAG_EIS_2_0)) {
				
				step = MMDVFS_VOLTAGE_HIGH;
			}		
			break;
			
		case SMI_BWC_SCEN_VR_SLOW:
            step = MMDVFS_VOLTAGE_HIGH; 

			break;

		case SMI_BWC_SCEN_ICFP:
			step = MMDVFS_VOLTAGE_HIGH;
			break;

		default:
			break;
	}

	return step;
}

static void mmdvfs_update_cmd(MTK_MMDVFS_CMD *cmd)
{
	if (cmd == NULL) {
		return;
	}

	if (cmd->sensor_size) {	
		g_mmdvfs_cmd.sensor_size = cmd->sensor_size;
	}

	if (cmd->sensor_fps) {
		g_mmdvfs_cmd.sensor_fps = cmd->sensor_fps;
	}

	
	MMDVFSMSG("update cm %d\n", cmd->camera_mode);

	
		g_mmdvfs_cmd.camera_mode = cmd->camera_mode;
	
}

static void mmdvfs_dump_info(void)
{
	MMDVFSMSG("CMD %d %d %d\n", g_mmdvfs_cmd.sensor_size, g_mmdvfs_cmd.sensor_fps, g_mmdvfs_cmd.camera_mode);
	MMDVFSMSG("INFO VR %d %d\n", g_mmdvfs_info->video_record_size[0], g_mmdvfs_info->video_record_size[1]);	
}

static void mmdvfs_timer_callback(unsigned long data)
{
	mmdvfs_gpu_monitor_struct *gpu_monitor = (mmdvfs_gpu_monitor_struct *)data;
	
	unsigned int gpu_loading = 0;

	if (mtk_get_gpu_loading(&gpu_loading)) {
		
	}

	
	gpu_monitor->gpu_loadings[gpu_monitor->gpu_loading_index++] = gpu_loading;

	
	if (gpu_monitor->gpu_loading_index < MMDVFS_GPU_LOADING_NUM - 1)
	{
		mod_timer(&gpu_monitor->timer, jiffies + msecs_to_jiffies(MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS));
	} else {
		
		int i;
		int avg_loading;
		unsigned int sum = 0;

		for (i = MMDVFS_GPU_LOADING_START_INDEX; i < MMDVFS_GPU_LOADING_NUM; i++)
		{
			sum += gpu_monitor->gpu_loadings[i];
		}

		avg_loading = sum / MMDVFS_GPU_LOADING_NUM;

		MMDVFSMSG("gpuload %d AVG %d\n", jiffies_to_msecs(jiffies), avg_loading);

		
		if (avg_loading <= MMDVFS_GPU_LOADING_THRESHOLD) {
			queue_work(gpu_monitor->work_queue, &gpu_monitor->work);
		}
	}
	
}

static void mmdvfs_gpu_monitor_work(struct work_struct *work)
{
	MMDVFSMSG("WQ %d\n", jiffies_to_msecs(jiffies));
}

static void mmdvfs_init_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;

	
	setup_timer(gpu_timer, mmdvfs_timer_callback, (unsigned long)gm);

	gm->work_queue = create_singlethread_workqueue("mmdvfs_gpumon");
	INIT_WORK(&gm->work, mmdvfs_gpu_monitor_work);	
}

static void mmdvfs_cam_work_handler(struct work_struct *work)
{
	MMDVFSMSG("CAM handler %d\n", jiffies_to_msecs(jiffies));
	mmdvfs_set_step(MMDVFS_CAM_MON_SCEN, mmdvfs_get_default_step());
}

static DECLARE_DELAYED_WORK(g_mmdvfs_cam_work, mmdvfs_cam_work_handler);

static void mmdvfs_stop_cam_monitor(void)
{
	cancel_delayed_work_sync(&g_mmdvfs_cam_work);
}

#define MMDVFS_CAM_MON_DELAY (4 * HZ)
static void mmdvfs_start_cam_monitor(void)
{
	mmdvfs_stop_cam_monitor();
	MMDVFSMSG("CAM start %d\n", jiffies_to_msecs(jiffies));
	mmdvfs_set_step(MMDVFS_CAM_MON_SCEN, MMDVFS_VOLTAGE_HIGH);
	
    schedule_delayed_work(&g_mmdvfs_cam_work, MMDVFS_CAM_MON_DELAY);
}

#if MMDVFS_ENABLE_WQHD

static void mmdvfs_start_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;
	
	gm->gpu_loading_index = 0;
	memset(gm->gpu_loadings, 0, sizeof(unsigned int) * MMDVFS_GPU_LOADING_NUM);

	mod_timer(gpu_timer, jiffies + msecs_to_jiffies(MMDVFS_GPU_LOADING_SAMPLE_DURATION_IN_MS));	
}

static void mmdvfs_stop_gpu_monitor(mmdvfs_gpu_monitor_struct *gm)
{
	struct timer_list *gpu_timer = &gm->timer;

	
	flush_workqueue(gm->work_queue);
	
	del_timer(gpu_timer);
}

#endif 

int mmdvfs_set_step(MTK_SMI_BWC_SCEN scenario, mmdvfs_voltage_enum step)
{
	int i, scen_index;
	unsigned int concurrency;
	mmdvfs_voltage_enum final_step = mmdvfs_get_default_step();

	if (step == MMDVFS_VOLTAGE_DEFAULT_STEP)
		step = final_step;

#if !MMDVFS_ENABLE
	return 0;
#endif


#if 0
#if !MMDVFS_ENABLE_WQHD
	
	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {
		return 0;
	}
#endif 
#endif

	MMDVFSMSG("MMDVFS set voltage scen %d step %d\n", scenario, step);

	if ((scenario >= (MTK_SMI_BWC_SCEN)MMDVFS_SCEN_COUNT) || (scenario < SMI_BWC_SCEN_NORMAL))
	{
		MMDVFSERR("invalid scenario\n");
		return -1;
	}

	
	mmdvfs_dump_info();

	
	scen_index = (int)scenario;

	spin_lock(&g_mmdvfs_mgr->scen_lock);
	
	g_mmdvfs_scenario_voltage[scen_index] = step;

	concurrency = 0;
	for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
		if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH)
			concurrency |= 1 << i;
	}

	
	for (i = 0; i < MMDVFS_SCEN_COUNT; i++) {
		if (g_mmdvfs_scenario_voltage[i] == MMDVFS_VOLTAGE_HIGH) {
			final_step = MMDVFS_VOLTAGE_HIGH;
			break;
		}
	}

	g_mmdvfs_current_step = final_step;
	
	spin_unlock(&g_mmdvfs_mgr->scen_lock);

	MMDVFSMSG("MMDVFS set voltage scen %d step %d final %d (%x)\n", scenario, step, final_step, concurrency);

#if	MMDVFS_ENABLE
	
    if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {	
#ifdef MMDVFS_WQHD_1_0V
		
		MMDVFSMSG("WQHD10 %d\n", final_step);
		if (final_step == MMDVFS_VOLTAGE_HIGH) {
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_PERF);
		} else {
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_UNREQ);
		}
#else 

		if (!g_mmdvfs_concurrency && (scenario == SMI_BWC_SCEN_UI_IDLE) && (step == MMDVFS_VOLTAGE_LOW)) {
			
			MMDVFSMSG("UI LP\n");
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_LOW_PWR);
		} else {
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_UNREQ);
		}
#endif 
	} else { 
		MMDVFSMSG("FHD %d\n", final_step);	
		if (final_step == MMDVFS_VOLTAGE_HIGH) {
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_PERF);
		} else {
			vcorefs_request_dvfs_opp(KR_MM_SCEN, OPPI_UNREQ);
		}
	}
#endif 

	return 0;
}

void mmdvfs_handle_cmd(MTK_MMDVFS_CMD *cmd)
{
#if !MMDVFS_ENABLE
	return;
#endif

	MMDVFSMSG("MMDVFS handle cmd %u s %d\n", cmd->type, cmd->scen);	
	
	switch (cmd->type) {
		case MTK_MMDVFS_CMD_TYPE_SET:
			
			mmdvfs_update_cmd(cmd);
			if (!(g_mmdvfs_concurrency & (1 << cmd->scen))) {
				MMDVFSMSG("invalid set scen %d\n", cmd->scen);
				cmd->ret = -1;
			} else {
				cmd->ret = mmdvfs_set_step(cmd->scen, mmdvfs_query(cmd->scen, cmd));
			}
			break;
			
		case MTK_MMDVFS_CMD_TYPE_QUERY:
		{	
		#ifndef MMDVFS_WQHD_1_0V
			if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {
				
				cmd->ret = (unsigned int)MMDVFS_STEP_HIGH2HIGH;
			} else
		#endif
			{

				mmdvfs_voltage_enum query_voltage = mmdvfs_query(cmd->scen, cmd);
				mmdvfs_voltage_enum current_voltage = mmdvfs_get_current_step();
				
				if (current_voltage < query_voltage) {
					cmd->ret = (unsigned int)MMDVFS_STEP_LOW2HIGH;
				} else if (current_voltage > query_voltage) {
					cmd->ret = (unsigned int)MMDVFS_STEP_HIGH2LOW;
				} else {
					cmd->ret = (unsigned int)(query_voltage == MMDVFS_VOLTAGE_HIGH ? MMDVFS_STEP_HIGH2HIGH : MMDVFS_STEP_LOW2LOW);
				}
			}

			MMDVFSMSG("query %d\n", cmd->ret);
			
			break;
		}
		
		default:
			MMDVFSMSG("invalid mmdvfs cmd\n");
			BUG();
			break;
	}
}

void mmdvfs_notify_scenario_exit(MTK_SMI_BWC_SCEN scen)
{
#if !MMDVFS_ENABLE
	return;
#endif

	MMDVFSMSG("leave %d\n", scen);

	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {
	#if MMDVFS_ENABLE_WQHD	
		if (scen == SMI_BWC_SCEN_VP) {
			mmdvfs_stop_gpu_monitor(&g_mmdvfs_mgr->gpu_monitor);
		}
	#endif 

	#ifdef MMDVFS_WQHD_1_0V
	if (scen == SMI_BWC_SCEN_VR)
	mmdvfs_start_cam_monitor();
	#endif

	} else if ((scen == SMI_BWC_SCEN_VR) &&  
				(g_mmdvfs_cmd.camera_mode & MMDVFS_CAMERA_MODE_FLAG_PIP)) {
		
		mmdvfs_start_cam_monitor();
	}

	
	mmdvfs_set_step(scen, mmdvfs_get_default_step());
}

void mmdvfs_notify_scenario_enter(MTK_SMI_BWC_SCEN scen)
{
#if !MMDVFS_ENABLE
	return;
#endif

	MMDVFSMSG("enter %d\n", scen);

	if (mmdvfs_get_lcd_resolution() == MMDVFS_LCD_SIZE_WQHD) {
	#ifndef MMDVFS_WQHD_1_0V
		
		if (scen != SMI_BWC_SCEN_NORMAL)
			mmdvfs_set_step(scen, MMDVFS_VOLTAGE_HIGH);
	#else
		switch (scen) {
			case SMI_BWC_SCEN_ICFP:
			case SMI_BWC_SCEN_WFD:
				mmdvfs_set_step(scen, MMDVFS_VOLTAGE_HIGH);
				break;

			case SMI_BWC_SCEN_VR:
			case SMI_BWC_SCEN_VR_SLOW:
				mmdvfs_set_step(scen, mmdvfs_query(scen, NULL));
				
				mmdvfs_set_step(SMI_BWC_SCEN_ICFP, mmdvfs_get_default_step());
				break;
			default:
				break;
		}
	#endif
	#if MMDVFS_ENABLE_WQHD
		if (scen == SMI_BWC_SCEN_VP) {
			mmdvfs_start_gpu_monitor(&g_mmdvfs_mgr->gpu_monitor);
		}
	#endif 
	} else {	
		switch (scen) {
			case SMI_BWC_SCEN_ICFP:
				mmdvfs_set_step(scen, MMDVFS_VOLTAGE_HIGH);
				break;

			case SMI_BWC_SCEN_VR:
			case SMI_BWC_SCEN_VR_SLOW:		
				mmdvfs_set_step(scen, mmdvfs_query(scen, NULL));
				
				mmdvfs_set_step(SMI_BWC_SCEN_ICFP, mmdvfs_get_default_step());
				break;
				
			default:
				break;
		}
	}	
}

void mmdvfs_init(MTK_SMI_BWC_MM_INFO *info)
{
#if !MMDVFS_ENABLE
	return;
#endif
	
	spin_lock_init(&g_mmdvfs_mgr->scen_lock);
	
	g_mmdvfs_current_step = mmdvfs_get_default_step();

	g_mmdvfs_info = info;

	mmdvfs_init_gpu_monitor(&g_mmdvfs_mgr->gpu_monitor);
}

void mmdvfs_mhl_enable(int enable)
{
	g_mmdvfs_mgr->is_mhl_enable = enable;
#ifdef MMDVFS_WQHD_1_0V
	if (enable) {
		mmdvfs_set_step(MMDVFS_SCEN_MHL, MMDVFS_VOLTAGE_HIGH);
	} else {
		mmdvfs_set_step(MMDVFS_SCEN_MHL, MMDVFS_VOLTAGE_DEFAULT_STEP);
	}
#endif 

}

void mmdvfs_notify_scenario_concurrency(unsigned int u4Concurrency)
{

	
	if (u4Concurrency & ((1 << SMI_BWC_SCEN_VP) | (1 << SMI_BWC_SCEN_VR) | (1 << SMI_BWC_SCEN_VR_SLOW))) {
	#if MMDVFS_ENABLE
		MMDVFSMSG("fliper high\n");
		fliper_set_bw(BW_THRESHOLD_HIGH);
	#endif		
	} else {
	#if MMDVFS_ENABLE	
		MMDVFSMSG("fliper normal\n");	
		fliper_restore_bw();
	#endif		
	}

	g_mmdvfs_concurrency = u4Concurrency;
}

int mmdvfs_is_default_step_need_perf(void)
{
	if (mmdvfs_get_default_step() == MMDVFS_VOLTAGE_LOW) {
		return 0;
	}

	return 1;
}

void mmdvfs_mm_clock_switch_notify(int is_before, int is_to_high)
{
	
#ifdef MMDVFS_WQHD_1_0V
	int session_id;

	if (mmdvfs_get_lcd_resolution() != MMDVFS_LCD_SIZE_WQHD)
		return;

	session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);

	if (!is_before && is_to_high) {
		MMDVFSMSG("DL\n");
		
		primary_display_switch_mode_for_mmdvfs(DISP_SESSION_DIRECT_LINK_MODE, session_id, 0);
	} else if (is_before && !is_to_high) {
		
		MMDVFSMSG("DC\n");
		primary_display_switch_mode_for_mmdvfs(DISP_SESSION_DECOUPLE_MODE, session_id, 1);
	}
#endif 
}
