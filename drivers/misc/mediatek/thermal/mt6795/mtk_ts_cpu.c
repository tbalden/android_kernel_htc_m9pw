#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include "mach/dma.h"
#include <mach/sync_write.h>
#include <mach/mt_irq.h>
#include "mach/mtk_thermal_monitor.h"
#include <linux/seq_file.h>
#include <linux/slab.h>
#include "mach/mt_typedefs.h"
#include "mach/mt_thermal.h"
#include <mach/mt_clkmgr.h>
#include <mach/mt_spm.h>
#include <mach/mt_ptp.h>
#include <mach/wd_api.h>
#include <linux/mtk_gpu_utility.h>
#include <linux/time.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#define __MT_MTK_TS_CPU_C__

#ifdef CONFIG_OF

u32 thermal_irq_number = 0;
void __iomem *thermal_base;
void __iomem *tscpu_auxadc_base;
void __iomem *tscpu_apmixed_base;
void __iomem *tscpu_pericfg_base;

int thermal_phy_base;
int tscpu_auxadc_phy_base;
int tscpu_apmixed_phy_base;
#endif
#define MTK_TS_CPU_RT                       (1)
#if MTK_TS_CPU_RT
#include <linux/sched.h>
#include <linux/kthread.h>
#endif


#define THERMAL_PERFORMANCE_PROFILE         (0)

#define THERMAL_GPIO_OUT_TOGGLE             (1)

#define CPT_ADAPTIVE_AP_COOLER              (1)

#define CONFIG_SUPPORT_MET_MTKTSCPU         (0)

#define THERMAL_CONTROLLER_HW_FILTER        (1) 

#define THERMAL_CONTROLLER_HW_TP            (1)

#define MTK_TS_CPU_SW_FILTER                (1)

#define MTKTSCPU_FAST_POLLING               (1)

#if CPT_ADAPTIVE_AP_COOLER
#define MAX_CPT_ADAPTIVE_COOLERS            (3)

#define THERMAL_HEADROOM                    (1)

#define CONTINUOUS_TM                       (1)
#endif

#define THERMAL_DRV_FAST_POLL_HRTIMER          (1)

#define THERMAL_DRV_UPDATE_TEMP_DIRECT_TO_MET  (1)

extern struct proc_dir_entry * mtk_thermal_get_proc_drv_therm_dir_entry(void);

#ifdef CONFIG_ARM64
extern int max_power_budget;
#else
static int max_power_budget = 8001;
#endif

static void tscpu_fast_initial_sw_workaround(void);

#define MIN(_a_, _b_) ((_a_) > (_b_) ? (_b_) : (_a_))
#define MAX(_a_, _b_) ((_a_) > (_b_) ? (_a_) : (_b_))

#if CPT_ADAPTIVE_AP_COOLER
static int g_curr_temp = 0;
static int g_prev_temp = 0;
#endif

#if MTKTSCPU_FAST_POLLING
static int fast_polling_trip_temp = 40000;
static int fast_polling_factor = 5;
static int cur_fp_factor = 1;
static int next_fp_factor = 1;
#endif

#define thermal_readl(addr)         DRV_Reg32(addr)
#define thermal_writel(addr, val)   mt_reg_sync_writel((val), ((void *)addr))
#define thermal_setl(addr, val)     mt_reg_sync_writel(thermal_readl(addr) | (val), ((void *)addr))
#define thermal_clrl(addr, val)     mt_reg_sync_writel(thermal_readl(addr) & ~(val), ((void *)addr))

static int g_max_temp = 50000;

static unsigned int interval = 1000; 
static int trip_temp[10] = {117000,100000,85000,75000,65000,55000,45000,35000,25000,15000};

static unsigned int *cl_dev_state;
static unsigned int cl_dev_sysrst_state=0;
#if CPT_ADAPTIVE_AP_COOLER
static unsigned int cl_dev_adp_cpu_state[MAX_CPT_ADAPTIVE_COOLERS] = {0};
static unsigned int cl_dev_adp_cpu_state_active = 0;
static int active_adp_cooler = 0;
#endif
static struct thermal_zone_device *thz_dev;

static struct thermal_cooling_device **cl_dev = NULL;
static struct thermal_cooling_device *cl_dev_sysrst = NULL;
#if CPT_ADAPTIVE_AP_COOLER
static struct thermal_cooling_device *cl_dev_adp_cpu[MAX_CPT_ADAPTIVE_COOLERS] = {NULL};
#endif

#if CPT_ADAPTIVE_AP_COOLER
static int TARGET_TJS[MAX_CPT_ADAPTIVE_COOLERS] = { 85000, 0 };
static int PACKAGE_THETA_JA_RISES[MAX_CPT_ADAPTIVE_COOLERS] = { 35, 0 };
static int PACKAGE_THETA_JA_FALLS[MAX_CPT_ADAPTIVE_COOLERS] = { 25, 0 };
static int MINIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 1200, 0 };
static int MAXIMUM_CPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 4400, 0 };
static int MINIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 350, 0 };
static int MAXIMUM_GPU_POWERS[MAX_CPT_ADAPTIVE_COOLERS] = { 960, 0 };
static int FIRST_STEP_TOTAL_POWER_BUDGETS[MAX_CPT_ADAPTIVE_COOLERS] = { 3300, 0 };
static int MINIMUM_BUDGET_CHANGES[MAX_CPT_ADAPTIVE_COOLERS] = { 50, 0 };

#if (THERMAL_HEADROOM == 1) || (CONTINUOUS_TM == 1)
extern int bts_cur_temp; 
#endif

#if THERMAL_HEADROOM
static int p_Tpcb_correlation = 0;
static int Tpcb_trip_point = 0;
static int thp_max_cpu_power = 0;
static int thp_p_tj_correlation = 0;
static int thp_threshold_tj = 0;
#endif

#if CONTINUOUS_TM
static int ctm_on = 0; 
static int MAX_TARGET_TJ = -1;
static int STEADY_TARGET_TJ = -1;
static int TRIP_TPCB = -1;
static int STEADY_TARGET_TPCB = -1;
static int MAX_EXIT_TJ = -1;
static int STEADY_EXIT_TJ = -1;
static int COEF_AE = -1;
static int COEF_BE = -1;
static int COEF_AX = -1;
static int COEF_BX = -1;
static int current_ETJ = -1;
#endif
#endif

static U32 calefuse1=0;
static U32 calefuse2=0;
static U32 calefuse3=0;

#if CPT_ADAPTIVE_AP_COOLER
static int mtktscpu_atm = 1;
static int tt_ratio_high_rise = 1;
static int tt_ratio_high_fall = 1;
static int tt_ratio_low_rise = 1;
static int tt_ratio_low_fall = 1;
static int tp_ratio_high_rise = 1;
static int tp_ratio_high_fall = 0;
static int tp_ratio_low_rise = 0;
static int tp_ratio_low_fall = 0;
#endif

static int mtktscpu_debug_log = 0;
static int kernelmode = 0;
static int g_THERMAL_TRIP[10] = {0,0,0,0,0,0,0,0,0,0};

static int num_trip=5;

int MA_len_temp=0;
static int proc_write_flag=0;
static char *cooler_name;
#define CPU_COOLER_NUM 34

static DEFINE_MUTEX(TS_lock);

#if CPT_ADAPTIVE_AP_COOLER
static char adaptive_cooler_name[] = "cpu_adaptive_";
#endif

static char g_bind0[20]="mtktscpu-sysrst";
static char g_bind1[20]="cpu02";
static char g_bind2[20]="cpu15";
static char g_bind3[20]="cpu22";
static char g_bind4[20]="cpu28";
static char g_bind5[20]="";
static char g_bind6[20]="";
static char g_bind7[20]="";
static char g_bind8[20]="";
static char g_bind9[20]="";

static int read_curr_temp;

static int polling_trip_temp1 = 30000;
static int polling_trip_temp2 = 15000;
static int polling_factor1 = 2;
static int polling_factor2 = 4;

#define MTKTSCPU_TEMP_CRIT 120000 

#if MTK_TS_CPU_RT
static struct task_struct *ktp_thread_handle = NULL;
#endif

static int tc_mid_trip = -275000;

#define tscpu_dprintk(fmt, args...)   \
do {                                    \
	if (mtktscpu_debug_log) {                \
		xlog_printk(ANDROID_LOG_INFO, "Power/CPU_Thermal", fmt, ##args); \
	}                                   \
} while(0)

#define tscpu_printk(fmt, args...)   \
do {                                    \
	xlog_printk(ANDROID_LOG_INFO, "Power/CPU_Thermal", fmt, ##args); \
} while(0)

static int mtktscpu_switch_bank(thermal_bank_name bank);
static void tscpu_reset_thermal(void);
extern int mt_cpufreq_thermal_protect(unsigned int limited_power);
extern u32 get_devinfo_with_index(u32 index);
extern void mt_gpufreq_thermal_protect(unsigned int limited_power);
extern unsigned int mt_gpufreq_get_cur_freq(void);
static S32 temperature_to_raw_room(U32 ret);
static void set_tc_trigger_hw_protect(int temperature, int temperature2);
static void tscpu_config_all_tc_hw_protect( int temperature, int temperature2);
static void thermal_initial_all_bank(void);
static void read_each_bank_TS(thermal_bank_name bank_num);

#ifdef CONFIG_ARM64
static int CPU_TS1_T=0,CPU_TS2_T=0;
static int LTE_TS4_T=0,TS_ABB_T=0;
static int GPU_TS3_T=0,SOC_TS4_T=0;
static int SOC_TS1_T=0,SOC_TS2_T=0,SOC_TS3_T=0;

static int CPU_TS1_R=0,CPU_TS2_R=0;
static int LTE_TS4_R=0,TS_ABB_R=0;
static int GPU_TS3_R=0,SOC_TS4_R=0;
static int SOC_TS1_R=0,SOC_TS2_R=0,SOC_TS3_R=0;
#else
static int CA7_TS1_T=0,CA7_TS2_T=0;
static int CA15_TS1_T=0,CA15_TS3_T=0;
static int GPU_TS3_T=0,GPU_TS4_T=0;
static int CORE_TS2_T=0,CORE_TS4_T=0,CORE_TSABB_T=0;

static int CA7_TS1_R=0,CA7_TS2_R=0;
static int CA15_TS1_R=0,CA15_TS3_R=0;
static int GPU_TS3_R=0,GPU_TS4_R=0;
static int CORE_TS2_R=0,CORE_TS4_R=0,CORE_TSABB_R=0;
#endif

int last_abb_t=0;
int last_CPU1_t=0;
int last_CPU2_t=0;

static int g_tc_resume=0;

static S32 g_adc_ge_t = 0;
static S32 g_adc_oe_t = 0;
static S32 g_o_vtsmcu1 = 0;
static S32 g_o_vtsmcu2 = 0;
static S32 g_o_vtsmcu3 = 0;
static S32 g_o_vtsmcu4 = 0;
static S32 g_o_vtsabb = 0;
static S32 g_degc_cali = 0;
static S32 g_adc_cali_en_t = 0;
static S32 g_o_slope = 0;
static S32 g_o_slope_sign = 0;
static S32 g_id = 0;

static S32 g_ge = 0;
static S32 g_oe = 0;
static S32 g_gain = 0;
static S32 g_x_roomt[THERMAL_SENSOR_NUM] = {0};

static int Num_of_OPP=0;

#if 0
static int Num_of_GPU_OPP=1; 
#else 
static int Num_of_GPU_OPP=0;
#endif

#define y_curr_repeat_times 1
#define THERMAL_NAME    "mtk-thermal"

struct mtk_cpu_power_info
{
	unsigned int cpufreq_khz;
	unsigned int cpufreq_ncpu;
	unsigned int cpufreq_power;
};
struct mtk_gpu_power_info
{
	unsigned int gpufreq_khz;
	unsigned int gpufreq_power;
};
static struct mtk_cpu_power_info *mtk_cpu_power = NULL;
static int tscpu_num_opp = 0;
static struct mtk_gpu_power_info *mtk_gpu_power = NULL;

static int tscpu_cpu_dmips[CPU_COOLER_NUM] = {0};
int mtktscpu_limited_dmips = 0;

static bool talking_flag=false;
static int temperature_switch=0;
int thermal_fast_init(void);
#if THERMAL_GPIO_OUT_TOGGLE
void tscpu_set_GPIO_toggle_for_monitor(void);
#endif

#define TS_MS_TO_NS(x) (x * 1000 * 1000)
static struct hrtimer ts_tempinfo_hrtimer;
static int g_hr_timer_polling_delay = TS_MS_TO_NS(50);

#if THERMAL_DRV_UPDATE_TEMP_DIRECT_TO_MET
static int a_tscpu_all_temp[THERMAL_SENSOR_NUM] = {0};
static DEFINE_MUTEX(MET_GET_TEMP_LOCK);
static met_thermalsampler_funcMET g_pThermalSampler = NULL;
void mt_thermalsampler_registerCB(met_thermalsampler_funcMET pCB)
{
    g_pThermalSampler = pCB;
}
EXPORT_SYMBOL(mt_thermalsampler_registerCB);

static DEFINE_SPINLOCK(tscpu_met_spinlock);
void tscpu_met_lock(unsigned long *flags)
{
	spin_lock_irqsave(&tscpu_met_spinlock, *flags);
}
EXPORT_SYMBOL(tscpu_met_lock);

void tscpu_met_unlock(unsigned long *flags)
{
	spin_unlock_irqrestore(&tscpu_met_spinlock, *flags);
}
EXPORT_SYMBOL(tscpu_met_unlock);
#endif

#ifdef __KERNEL__
#define CONFIG_THERMAL_AEE_RR_REC (0)
#endif

#if (CONFIG_THERMAL_AEE_RR_REC==1)
enum thermal_state {
	TSCPU_SUSPEND = 0,
	TSCPU_RESUME,
	TSCPU_NORMAL,
};

extern signed int 	  *aee_rr_rec_thermal_temp1(void);
extern signed int 	  *aee_rr_rec_thermal_temp2(void);
extern signed int 	  *aee_rr_rec_thermal_temp3(void);
extern signed int 	  *aee_rr_rec_thermal_temp4(void);
extern signed int 	  *aee_rr_rec_thermal_temp5(void);
extern unsigned char  *aee_rr_rec_thermal_state(void);

static signed int *p_thermal_temp1;
static signed int *p_thermal_temp2;
static signed int *p_thermal_temp3;
static signed int *p_thermal_temp4;
static signed int *p_thermal_temp5;
static unsigned char *p_thermal_state;

static void _mt_thermal_aee_init(void)
{
	p_thermal_temp1=aee_rr_rec_thermal_temp1();
    p_thermal_temp2=aee_rr_rec_thermal_temp2();
    p_thermal_temp3=aee_rr_rec_thermal_temp3();
    p_thermal_temp4=aee_rr_rec_thermal_temp4();
    p_thermal_temp5=aee_rr_rec_thermal_temp5();
    p_thermal_state=aee_rr_rec_thermal_state();

	*p_thermal_temp1 = 0;
	*p_thermal_temp2 = 0;
	*p_thermal_temp3 = 0;
	*p_thermal_temp4 = 0;
	*p_thermal_temp5 = 0;
	*p_thermal_state = 0;
}
#endif

void set_taklking_flag(bool flag)
{
	talking_flag = flag;
	tscpu_printk("talking_flag=%d\n", talking_flag);
	return;
}

static unsigned int adaptive_cpu_power_limit = 0x7FFFFFFF, static_cpu_power_limit = 0x7FFFFFFF;
static unsigned int prv_adp_cpu_pwr_lim, prv_stc_cpu_pwr_lim;

static void set_adaptive_cpu_power_limit(unsigned int limit)
{
    unsigned int final_limit;
    prv_adp_cpu_pwr_lim = adaptive_cpu_power_limit;
    limit = (limit < max_power_budget) ? limit : 0;
    adaptive_cpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;
    final_limit = MIN(adaptive_cpu_power_limit, static_cpu_power_limit);

    if (prv_adp_cpu_pwr_lim != adaptive_cpu_power_limit &&
        (adaptive_cpu_power_limit < max_power_budget || adaptive_cpu_power_limit == 0x7FFFFFFF))
    {
#ifdef CONFIG_ARM64
        tscpu_printk("set_adaptive_cpu_power_limit %d, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0,
                     CPU_TS1_T, CPU_TS2_T, LTE_TS4_T, TS_ABB_T, GPU_TS3_T, SOC_TS1_T, SOC_TS2_T, SOC_TS3_T, SOC_TS4_T);
#else
        tscpu_printk("set_adaptive_cpu_power_limit %d, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0,
                     CA7_TS1_T, CA15_TS1_T, CA7_TS2_T, CORE_TS2_T, CA15_TS3_T, GPU_TS3_T, GPU_TS4_T, CORE_TS4_T, CORE_TSABB_T);
#endif
        mt_cpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);
    }
}

static void set_static_cpu_power_limit(unsigned int limit)
{
    unsigned int final_limit;
    prv_stc_cpu_pwr_lim = static_cpu_power_limit;
    static_cpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;
    final_limit = MIN(adaptive_cpu_power_limit, static_cpu_power_limit);

    if (prv_stc_cpu_pwr_lim != static_cpu_power_limit)
    {
#ifdef CONFIG_ARM64
        tscpu_printk("set_static_cpu_power_limit %d, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0,
                     CPU_TS1_T, CPU_TS2_T, LTE_TS4_T, TS_ABB_T, GPU_TS3_T, SOC_TS1_T, SOC_TS2_T, SOC_TS3_T, SOC_TS4_T);
#else
        tscpu_printk("set_static_cpu_power_limit %d, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0,
                     CA7_TS1_T, CA15_TS1_T, CA7_TS2_T, CORE_TS2_T, CA15_TS3_T, GPU_TS3_T, GPU_TS4_T, CORE_TS4_T, CORE_TSABB_T);
#endif
        mt_cpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);
    }
}

static unsigned int adaptive_gpu_power_limit = 0x7FFFFFFF, static_gpu_power_limit = 0x7FFFFFFF;

static void set_adaptive_gpu_power_limit(unsigned int limit)
{
    unsigned int final_limit;

    adaptive_gpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;
    final_limit = MIN(adaptive_gpu_power_limit, static_gpu_power_limit);
    tscpu_printk("set_adaptive_gpu_power_limit %d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0);
    mt_gpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);
}

static void set_static_gpu_power_limit(unsigned int limit)
{
    unsigned int final_limit;

    static_gpu_power_limit = (limit != 0) ? limit : 0x7FFFFFFF;
    final_limit = MIN(adaptive_gpu_power_limit, static_gpu_power_limit);
    tscpu_printk("set_static_gpu_power_limit %d\n", (final_limit != 0x7FFFFFFF) ? final_limit : 0);
    mt_gpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);
}


int tscpu_thermal_clock_on(void)
{
	int ret=-1;
	tscpu_dprintk("tscpu_thermal_clock_on\n");
    ret = enable_clock(MT_CG_PERI_THERM, "THERMAL");
    return ret;
}

int tscpu_thermal_clock_off(void)
{
	int ret=-1;
	tscpu_dprintk("tscpu_thermal_clock_off\n");
    ret = disable_clock(MT_CG_PERI_THERM, "THERMAL");
    return ret;
}

void get_thermal_all_register(void)
{
	tscpu_dprintk("get_thermal_all_register\n");

	tscpu_dprintk("TEMPMSR1    		  = 0x%8x\n", DRV_Reg32(TEMPMSR1));
	tscpu_dprintk("TEMPMSR2            = 0x%8x\n", DRV_Reg32(TEMPMSR2));

	tscpu_dprintk("TEMPMONCTL0   	  = 0x%8x\n", DRV_Reg32(TEMPMONCTL0));
	tscpu_dprintk("TEMPMONCTL1   	  = 0x%8x\n", DRV_Reg32(TEMPMONCTL1));
	tscpu_dprintk("TEMPMONCTL2   	  = 0x%8x\n", DRV_Reg32(TEMPMONCTL2));
	tscpu_dprintk("TEMPMONINT    	  = 0x%8x\n", DRV_Reg32(TEMPMONINT));
	tscpu_dprintk("TEMPMONINTSTS 	  = 0x%8x\n", DRV_Reg32(TEMPMONINTSTS));
	tscpu_dprintk("TEMPMONIDET0  	  = 0x%8x\n", DRV_Reg32(TEMPMONIDET0));

	tscpu_dprintk("TEMPMONIDET1  	  = 0x%8x\n", DRV_Reg32(TEMPMONIDET1));
	tscpu_dprintk("TEMPMONIDET2  	  = 0x%8x\n", DRV_Reg32(TEMPMONIDET2));
	tscpu_dprintk("TEMPH2NTHRE   	  = 0x%8x\n", DRV_Reg32(TEMPH2NTHRE));
	tscpu_dprintk("TEMPHTHRE     	  = 0x%8x\n", DRV_Reg32(TEMPHTHRE));
	tscpu_dprintk("TEMPCTHRE     	  = 0x%8x\n", DRV_Reg32(TEMPCTHRE));
	tscpu_dprintk("TEMPOFFSETH   	  = 0x%8x\n", DRV_Reg32(TEMPOFFSETH));

	tscpu_dprintk("TEMPOFFSETL   	  = 0x%8x\n", DRV_Reg32(TEMPOFFSETL));
	tscpu_dprintk("TEMPMSRCTL0   	  = 0x%8x\n", DRV_Reg32(TEMPMSRCTL0));
	tscpu_dprintk("TEMPMSRCTL1   	  = 0x%8x\n", DRV_Reg32(TEMPMSRCTL1));
	tscpu_dprintk("TEMPAHBPOLL   	  = 0x%8x\n", DRV_Reg32(TEMPAHBPOLL));
	tscpu_dprintk("TEMPAHBTO     	  = 0x%8x\n", DRV_Reg32(TEMPAHBTO));
	tscpu_dprintk("TEMPADCPNP0   	  = 0x%8x\n", DRV_Reg32(TEMPADCPNP0));

	tscpu_dprintk("TEMPADCPNP1   	  = 0x%8x\n", DRV_Reg32(TEMPADCPNP1));
	tscpu_dprintk("TEMPADCPNP2   	  = 0x%8x\n", DRV_Reg32(TEMPADCPNP2));
	tscpu_dprintk("TEMPADCMUX    	  = 0x%8x\n", DRV_Reg32(TEMPADCMUX));
	tscpu_dprintk("TEMPADCEXT    	  = 0x%8x\n", DRV_Reg32(TEMPADCEXT));
	tscpu_dprintk("TEMPADCEXT1   	  = 0x%8x\n", DRV_Reg32(TEMPADCEXT1));
	tscpu_dprintk("TEMPADCEN     	  = 0x%8x\n", DRV_Reg32(TEMPADCEN));

	tscpu_dprintk("TEMPPNPMUXADDR      = 0x%8x\n", DRV_Reg32(TEMPPNPMUXADDR));
	tscpu_dprintk("TEMPADCMUXADDR      = 0x%8x\n", DRV_Reg32(TEMPADCMUXADDR));
	tscpu_dprintk("TEMPADCEXTADDR      = 0x%8x\n", DRV_Reg32(TEMPADCEXTADDR));
	tscpu_dprintk("TEMPADCEXT1ADDR     = 0x%8x\n", DRV_Reg32(TEMPADCEXT1ADDR));
	tscpu_dprintk("TEMPADCENADDR       = 0x%8x\n", DRV_Reg32(TEMPADCENADDR));
	tscpu_dprintk("TEMPADCVALIDADDR    = 0x%8x\n", DRV_Reg32(TEMPADCVALIDADDR));

	tscpu_dprintk("TEMPADCVOLTADDR     = 0x%8x\n", DRV_Reg32(TEMPADCVOLTADDR));
	tscpu_dprintk("TEMPRDCTRL          = 0x%8x\n", DRV_Reg32(TEMPRDCTRL));
	tscpu_dprintk("TEMPADCVALIDMASK    = 0x%8x\n", DRV_Reg32(TEMPADCVALIDMASK));
	tscpu_dprintk("TEMPADCVOLTAGESHIFT = 0x%8x\n", DRV_Reg32(TEMPADCVOLTAGESHIFT));
	tscpu_dprintk("TEMPADCWRITECTRL    = 0x%8x\n", DRV_Reg32(TEMPADCWRITECTRL));
	tscpu_dprintk("TEMPMSR0            = 0x%8x\n", DRV_Reg32(TEMPMSR0));

	tscpu_dprintk("TEMPIMMD0           = 0x%8x\n", DRV_Reg32(TEMPIMMD0));
	tscpu_dprintk("TEMPIMMD1           = 0x%8x\n", DRV_Reg32(TEMPIMMD1));
	tscpu_dprintk("TEMPIMMD2           = 0x%8x\n", DRV_Reg32(TEMPIMMD2));
	tscpu_dprintk("TEMPPROTCTL         = 0x%8x\n", DRV_Reg32(TEMPPROTCTL));

	tscpu_dprintk("TEMPPROTTA          = 0x%8x\n", DRV_Reg32(TEMPPROTTA));
	tscpu_dprintk("TEMPPROTTB 		  = 0x%8x\n", DRV_Reg32(TEMPPROTTB));
	tscpu_dprintk("TEMPPROTTC 		  = 0x%8x\n", DRV_Reg32(TEMPPROTTC));
	tscpu_dprintk("TEMPSPARE0 		  = 0x%8x\n", DRV_Reg32(TEMPSPARE0));
	tscpu_dprintk("TEMPSPARE1 		  = 0x%8x\n", DRV_Reg32(TEMPSPARE1));
	tscpu_dprintk("TEMPSPARE2 		  = 0x%8x\n", DRV_Reg32(TEMPSPARE2));
	tscpu_dprintk("TEMPSPARE3 		  = 0x%8x\n", DRV_Reg32(TEMPSPARE3));
	tscpu_dprintk("0x11001040 		  = 0x%8x\n", DRV_Reg32(0xF1001040));
}

void get_thermal_slope_intercept(struct TS_PTPOD *ts_info,thermal_bank_name ts_name)
{
	unsigned int temp0, temp1, temp2;
	struct TS_PTPOD ts_ptpod;
    S32 x_roomt;

	tscpu_dprintk("get_thermal_slope_intercept\n");
#ifdef CONFIG_ARM64
	switch(ts_name){
        case THERMAL_BANK0:
            x_roomt = g_x_roomt[0];
            break;
        case THERMAL_BANK1:
            x_roomt = g_x_roomt[1];
            break;
        case THERMAL_BANK2:
            x_roomt = g_x_roomt[2];
            break;
        case THERMAL_BANK3:
            x_roomt = g_x_roomt[0];
            break;
        default:
            x_roomt = g_x_roomt[0];
            break;
    }
#else
	switch(ts_name){
        case THERMAL_BANK0:
            x_roomt = g_x_roomt[0];
            break;
        case THERMAL_BANK1:
            x_roomt = g_x_roomt[2];
            break;
        case THERMAL_BANK2:
            x_roomt = g_x_roomt[3];
            break;
        case THERMAL_BANK3:
            x_roomt = g_x_roomt[1];
            break;
        default:
            x_roomt = g_x_roomt[2];
            break;
    }
#endif
	temp0 = (10000*100000/g_gain)*15/18;

	if(g_o_slope_sign==0)
	{
		temp1 = temp0/(165+g_o_slope);
	}
	else
	{
		temp1 = temp0/(165-g_o_slope);
	}
	ts_ptpod.ts_MTS = temp1;

	temp0 = (g_degc_cali *10 / 2);
	temp1 = ((10000*100000/4096/g_gain)*g_oe + x_roomt*10)*15/18;

	if(g_o_slope_sign==0)
	{
		temp2 = temp1*10/(165+g_o_slope);
	}
	else
	{
		temp2 = temp1*10/(165-g_o_slope);
	}

	ts_ptpod.ts_BTS = (temp0+temp2-250)*4/10;


	ts_info->ts_MTS = ts_ptpod.ts_MTS;
	ts_info->ts_BTS = ts_ptpod.ts_BTS;
	tscpu_printk("ts_MTS=%d, ts_BTS=%d\n",ts_ptpod.ts_MTS, ts_ptpod.ts_BTS);

	return;
}
EXPORT_SYMBOL(get_thermal_slope_intercept);

#if 0
static void dump_spm_reg(void)
{
	

	tscpu_printk("SPM_SLEEP_ISR_RAW_STA       =0x%08x\n",spm_read(SPM_SLEEP_ISR_RAW_STA));
	tscpu_printk("SPM_PCM_REG13_DATA          =0x%08x\n",spm_read(SPM_PCM_REG13_DATA));
	tscpu_printk("SPM_SLEEP_WAKEUP_EVENT_MASK =0x%08x\n",spm_read(SPM_SLEEP_WAKEUP_EVENT_MASK));
	tscpu_printk("SPM_POWERON_CONFIG_SET      =0x%08x\n",spm_read(SPM_POWERON_CONFIG_SET));
	tscpu_printk("SPM_POWER_ON_VAL1		     =0x%08x\n",spm_read(SPM_POWER_ON_VAL1));
	tscpu_printk("SPM_PCM_IM_LEN		     	 =0x%08x\n",spm_read(SPM_PCM_IM_LEN));
   	tscpu_printk("SPM_PCM_PWR_IO_EN	   	     =0x%08x\n",spm_read(SPM_PCM_PWR_IO_EN));

   	tscpu_printk("SPM_PCM_CON0			     =0x%08x\n",spm_read(SPM_PCM_CON0));
   	tscpu_printk("SPM_PCM_CON1			     =0x%08x\n",spm_read(SPM_PCM_CON1));
   	tscpu_printk("SPM_PCM_IM_PTR		     	 =0x%08x\n",spm_read(SPM_PCM_IM_PTR));
   	tscpu_printk("SPM_PCM_REG1_DATA			 =0x%08x\n",spm_read(SPM_PCM_REG1_DATA));
   	tscpu_printk("SPM_PCM_REG2_DATA			 =0x%08x\n",spm_read(SPM_PCM_REG2_DATA));
   	tscpu_printk("SPM_PCM_REG3_DATA			 =0x%08x\n",spm_read(SPM_PCM_REG3_DATA));
   	tscpu_printk("SPM_PCM_REG7_DATA			 =0x%08x\n",spm_read(SPM_PCM_REG7_DATA));
   	tscpu_printk("SPM_PCM_REG9_DATA			 =0x%08x\n",spm_read(SPM_PCM_REG9_DATA));
   	tscpu_printk("SPM_PCM_REG12_DATA		 	 =0x%08x\n",spm_read(SPM_PCM_REG12_DATA));
   	tscpu_printk("SPM_PCM_REG14_DATA		 	 =0x%08x\n",spm_read(SPM_PCM_REG14_DATA));
   	tscpu_printk("SPM_PCM_REG15_DATA		 	 =0x%08x\n",spm_read(SPM_PCM_REG15_DATA));
   	tscpu_printk("SPM_PCM_FSM_STA			 =0x%08x\n",spm_read(SPM_PCM_FSM_STA));
}
#endif

static void thermal_interrupt_handler(int bank)
{
	U32 ret = 0;
	unsigned long flags;

	mt_ptp_lock(&flags);

	mtktscpu_switch_bank(bank);

	ret = DRV_Reg32(TEMPMONINTSTS);
	
	tscpu_printk("thermal_interrupt_handler,bank=0x%08x,ret=0x%08x\n",bank,ret);
	

    
    

	
	if (ret & THERMAL_MON_CINTSTS0)
	{
		tscpu_printk("thermal_isr: thermal sensor point 0 - cold interrupt trigger\n");
	}
	if (ret & THERMAL_MON_HINTSTS0)
	{
		tscpu_printk("<<<thermal_isr>>>: thermal sensor point 0 - hot interrupt trigger\n");
	}

	if (ret & THERMAL_MON_HINTSTS1)
	{
		tscpu_printk("<<<thermal_isr>>>: thermal sensor point 1 - hot interrupt trigger\n");
	}

	if (ret & THERMAL_MON_HINTSTS2)
	{
		tscpu_printk("<<<thermal_isr>>>: thermal sensor point 2 - hot interrupt trigger\n");
	}

	if(ret & THERMAL_tri_SPM_State0)
		tscpu_printk("thermal_isr: Thermal state0 to trigger SPM state0\n");
	if(ret & THERMAL_tri_SPM_State1){
        
#if MTK_TS_CPU_RT
 
 
 

#ifdef CONFIG_ARM64
    tscpu_printk("THERMAL_tri_SPM_State1, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", CPU_TS1_T, CPU_TS2_T, LTE_TS4_T,
    		TS_ABB_T, GPU_TS3_T, SOC_TS1_T, SOC_TS2_T, SOC_TS3_T, SOC_TS4_T);
#else
    tscpu_printk("THERMAL_tri_SPM_State1, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", CA7_TS1_T, CA15_TS1_T, CA7_TS2_T,
    		CORE_TS2_T, CA15_TS3_T, GPU_TS3_T, GPU_TS4_T, CORE_TS4_T, CORE_TSABB_T);
#endif
        wake_up_process(ktp_thread_handle);
#endif
    }
	if(ret & THERMAL_tri_SPM_State2)
		tscpu_printk("thermal_isr: Thermal state2 to trigger SPM state2\n");

	mt_ptp_unlock(&flags);
}

static irqreturn_t thermal_all_bank_interrupt_handler(int irq, void *dev_id)
{
	U32 ret = 0;

	

	
	
	

    ret = DRV_Reg32(THERMINTST);
    ret = ret & 0xF;
    printk("thermal_interrupt_handler : THERMINTST = 0x%x\n", ret);

	if((ret & 0x1) == 0)
		thermal_interrupt_handler(THERMAL_BANK0);

    if((ret & 0x2) == 0)
		thermal_interrupt_handler(THERMAL_BANK1);

    if((ret & 0x4) == 0)
		thermal_interrupt_handler(THERMAL_BANK2);

    if((ret & 0x8) == 0)
    	thermal_interrupt_handler(THERMAL_BANK3);

	return IRQ_HANDLED;
}

static void thermal_reset_and_initial(void)
{
	

	

	
	
    

    THERMAL_WRAP_WR32(0x0000000C, TEMPMONCTL1);   

#if THERMAL_CONTROLLER_HW_FILTER == 2
    THERMAL_WRAP_WR32(0x07E007E0, TEMPMONCTL2); 
    THERMAL_WRAP_WR32(0x001F7972, TEMPAHBPOLL); 
    THERMAL_WRAP_WR32(0x00000049, TEMPMSRCTL0); 
#elif THERMAL_CONTROLLER_HW_FILTER == 4
    THERMAL_WRAP_WR32(0x050A050A, TEMPMONCTL2);  
    THERMAL_WRAP_WR32(0x001424C4, TEMPAHBPOLL);  
    THERMAL_WRAP_WR32(0x000000DB, TEMPMSRCTL0);  
#elif THERMAL_CONTROLLER_HW_FILTER == 8
    THERMAL_WRAP_WR32(0x03390339, TEMPMONCTL2);  
    THERMAL_WRAP_WR32(0x000C96FA, TEMPAHBPOLL);  
    THERMAL_WRAP_WR32(0x00000124, TEMPMSRCTL0);  
#elif THERMAL_CONTROLLER_HW_FILTER == 16
    THERMAL_WRAP_WR32(0x01C001C0, TEMPMONCTL2);  
	THERMAL_WRAP_WR32(0x0006FE8B, TEMPAHBPOLL);  
    THERMAL_WRAP_WR32(0x0000016D, TEMPMSRCTL0);  
#else 
    THERMAL_WRAP_WR32(0x000101AD, TEMPMONCTL2);	 
    
    THERMAL_WRAP_WR32(0x00000300, TEMPAHBPOLL);  
    THERMAL_WRAP_WR32(0x00000000, TEMPMSRCTL0);  
#endif

	THERMAL_WRAP_WR32(0xFFFFFFFF, TEMPAHBTO);      

	THERMAL_WRAP_WR32(0x00000000, TEMPMONIDET0);   
	THERMAL_WRAP_WR32(0x00000000, TEMPMONIDET1);   

	THERMAL_WRAP_WR32(0x800, TEMPADCMUX);                         
	THERMAL_WRAP_WR32((UINT32) AUXADC_CON1_CLR_P, TEMPADCMUXADDR);

	THERMAL_WRAP_WR32(0x800, TEMPADCEN);                          
	THERMAL_WRAP_WR32((UINT32) AUXADC_CON1_SET_P, TEMPADCENADDR); 
																  

	THERMAL_WRAP_WR32((UINT32) AUXADC_DAT11_P, TEMPADCVALIDADDR); 
	THERMAL_WRAP_WR32((UINT32) AUXADC_DAT11_P, TEMPADCVOLTADDR);  
	THERMAL_WRAP_WR32(0x0, TEMPRDCTRL);               			  
	THERMAL_WRAP_WR32(0x0000002C, TEMPADCVALIDMASK);              
	THERMAL_WRAP_WR32(0x0, TEMPADCVOLTAGESHIFT);                  
	THERMAL_WRAP_WR32(0x2, TEMPADCWRITECTRL);                     
}

static void thermal_enable_all_periodoc_sensing_point_Bank0(void)
{
	THERMAL_WRAP_WR32(0x00000003, TEMPMONCTL0);            
}

static void thermal_enable_all_periodoc_sensing_point_Bank1(void)
{
	THERMAL_WRAP_WR32(0x00000003, TEMPMONCTL0);            
}

static void thermal_enable_all_periodoc_sensing_point_Bank2(void)
{
	THERMAL_WRAP_WR32(0x00000003, TEMPMONCTL0);            
}

static void thermal_enable_all_periodoc_sensing_point_Bank3(void)
{
#ifdef CONFIG_ARM64
	THERMAL_WRAP_WR32(0x0000000F, TEMPMONCTL0);             
#else
	THERMAL_WRAP_WR32(0x00000007, TEMPMONCTL0);             
#endif
}

#ifdef CONFIG_ARM64
static void thermal_config_Bank0_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank0_TS:\n");

	

	
    
	
    THERMAL_WRAP_WR32(0x0,TEMPADCPNP0);                    

    
    
	
    THERMAL_WRAP_WR32(0x1,TEMPADCPNP1);                    


    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);
}

static void thermal_config_Bank1_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank1_TS\n");
	

	
    
	
    THERMAL_WRAP_WR32(0x3,TEMPADCPNP0);                    

	
    
	
    THERMAL_WRAP_WR32(0x10,TEMPADCPNP1);                   

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);
}

static void thermal_config_Bank2_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank2_TS\n");

	

    
	
    THERMAL_WRAP_WR32(0x2,TEMPADCPNP0);                    


	
	
    THERMAL_WRAP_WR32(0x3,TEMPADCPNP1);                    

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);
}

static void thermal_config_Bank3_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank3_TS\n");

	

    
	
    THERMAL_WRAP_WR32(0x0,TEMPADCPNP0);                    

    
	
    THERMAL_WRAP_WR32(0x1,TEMPADCPNP1);                    

    
	
    THERMAL_WRAP_WR32(0x2,TEMPADCPNP2);                    


    
	
    THERMAL_WRAP_WR32(0x3,TEMPADCPNP3);                    

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);
}

#else
static void thermal_config_Bank0_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank0_TS:\n");

	
    
	
    THERMAL_WRAP_WR32(0x0,TEMPADCPNP0);                    

    
	
    THERMAL_WRAP_WR32(0x1,TEMPADCPNP1);                    

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);

    
}

static void thermal_config_Bank1_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank1_TS\n");

	
    
	
    THERMAL_WRAP_WR32(0x0,TEMPADCPNP0);                    

    
	
    THERMAL_WRAP_WR32(0x2,TEMPADCPNP1);                    

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);

    
}

static void thermal_config_Bank2_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank2_TS\n");

	

    
	
    THERMAL_WRAP_WR32(0x2,TEMPADCPNP0);                    

    
	
    THERMAL_WRAP_WR32(0x3,TEMPADCPNP1);                    

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);

    
}

static void thermal_config_Bank3_TS(void)
{
	tscpu_dprintk( "thermal_config_Bank3_TS\n");

	
    
	
    THERMAL_WRAP_WR32(0x1,TEMPADCPNP0);                    

    
	
    THERMAL_WRAP_WR32(0x3,TEMPADCPNP1);                    

    
	
    THERMAL_WRAP_WR32(0x10,TEMPADCPNP2);                   

    THERMAL_WRAP_WR32((UINT32) TS_CON1_P,TEMPPNPMUXADDR);  
   	THERMAL_WRAP_WR32(0x3, TEMPADCWRITECTRL);

    
}
#endif

static void thermal_config_TS_in_banks(thermal_bank_name bank_num)
{
	tscpu_dprintk( "thermal_config_TS_in_banks bank_num=%d\n",bank_num);

	switch(bank_num){
        case THERMAL_BANK0:
            thermal_config_Bank0_TS();
            break;
        case THERMAL_BANK1:
            thermal_config_Bank1_TS();
            break;
        case THERMAL_BANK2:
            thermal_config_Bank2_TS();
            break;
        case THERMAL_BANK3:
            thermal_config_Bank3_TS();
            break;
        default:
            thermal_config_Bank0_TS();
            break;
    }
}

static void thermal_enable_all_periodoc_sensing_point(thermal_bank_name bank_num)
{
	tscpu_dprintk( "thermal_config_TS_in_banks bank_num=%d\n",bank_num);

	switch(bank_num){
        case THERMAL_BANK0:
            thermal_enable_all_periodoc_sensing_point_Bank0();
            break;
        case THERMAL_BANK1:
            thermal_enable_all_periodoc_sensing_point_Bank1();
            break;
        case THERMAL_BANK2:
            thermal_enable_all_periodoc_sensing_point_Bank2();
            break;
        case THERMAL_BANK3:
            thermal_enable_all_periodoc_sensing_point_Bank3();
            break;
        default:
            thermal_enable_all_periodoc_sensing_point_Bank0();
            break;
    }
}
#if 0 
static void set_thermal_ctrl_trigger_SPM2(int temperature)
{
#if THERMAL_CONTROLLER_HW_TP
	int temp = 0;
	int raw_high;
	tscpu_printk("set_thermal_ctrl_trigger_SPM2 temperature=%d\n", temperature);

	
	raw_high   = temperature_to_raw_abb(temperature);


	THERMAL_WRAP_WR32(temp | 0x80000000, TEMPMONINT);	

	temp = DRV_Reg32(TEMPMONINT);
	

	THERMAL_WRAP_WR32(0x20000, TEMPPROTCTL);
	THERMAL_WRAP_WR32(raw_high, TEMPPROTTC);


#endif
}
#endif
#if 0
static void set_tc_trigger_hw_protect_test(int temp_hot, int temp_normal, int temp_low, int loffset)
{
	int temp = 0;
    int offset = 0;
	int raw_high, raw_middle, raw_low;
    int raw_high_offset, raw_hot_normal, raw_low_offset;
	printk( "set_tc_trigger_hw_protect_test temp_hot=%d, temp_normal=%d, temp_low=%d, loffset=%d\n",
        temp_hot, temp_normal,temp_low,loffset);

	offset         = temperature_to_raw_room(loffset);
    raw_high       = temperature_to_raw_room(temp_hot);
	raw_hot_normal = temperature_to_raw_room(temp_normal);
	raw_low        = temperature_to_raw_room(temp_low);

	raw_high_offset = raw_high        + offset;
	raw_hot_normal  = raw_high_offset + offset;
	raw_low_offset  = raw_hot_normal  + offset;

	temp = DRV_Reg32(TEMPMONINT);
    printk("set_tc_trigger_hw_protect_test 1 TEMPMONINT:temp=0x%x\n",temp);


	THERMAL_WRAP_WR32(raw_high,        TEMPHTHRE);     
	
	THERMAL_WRAP_WR32(raw_hot_normal,  TEMPH2NTHRE);    
	
	THERMAL_WRAP_WR32(raw_low,         TEMPCTHRE);      

	THERMAL_WRAP_WR32(temp | 0x00004E73, TEMPMONINT);	
    printk("set_tc_trigger_hw_protect_test 2 TEMPMONINT:temp=0x%x\n",temp);
}
#endif
#if 0 
static void set_thermal_ctrl_trigger_SPM(int temperature, int temperature2)
{
#if THERMAL_CONTROLLER_HW_TP
	int temp = 0;
	int raw_high, raw_middle, raw_low;
	printk("set_thermal_ctrl_trigger_SPM t11=%d t22=%d\n", temperature, temperature2);

	raw_high   = temperature_to_raw_room(80000);
	raw_middle = temperature_to_raw_room(60000);
	raw_low    = temperature_to_raw_room(45000);

	temp = DRV_Reg32(TEMPMONINT);
    printk("TEMPMONINT:1 temp=0x%x\n",temp);
	
	
	printk("TEMPMONINT:2 temp=0x%x\n",temp);

	printk("raw_high=%d,raw_middle=%d,raw_low=%d\n",raw_high,raw_middle,raw_low);

	printk("high=%d,middle=%d,low=%d\n",80000,60000,45000);

	
	
    
	
	

	THERMAL_WRAP_WR32(raw_high, TEMPHTHRE);	
	THERMAL_WRAP_WR32(raw_middle, TEMPH2NTHRE);	
	THERMAL_WRAP_WR32(raw_low,  TEMPCTHRE);	

	
	
	
	
	
	
		THERMAL_WRAP_WR32(temp | 0x00000842, TEMPMONINT);	

	printk("TEMPMONINT:3 temp=0x%x\n",DRV_Reg32(TEMPMONINT));
#endif
}
#endif

static void set_tc_trigger_hw_protect(int temperature, int temperature2)
{
	int temp = 0;
	int raw_high, raw_middle, raw_low;

	
    tscpu_dprintk( "set_tc_trigger_hw_protect t1=%d t2=%d\n", temperature, temperature2);

	
	raw_high   = temperature_to_raw_room(temperature);
    if (temperature2 > -275000)
		raw_middle = temperature_to_raw_room(temperature2);
	raw_low    = temperature_to_raw_room(5000);

	temp = DRV_Reg32(TEMPMONINT);
	
	
	THERMAL_WRAP_WR32(temp & 0x00000000, TEMPMONINT);	

#ifdef CONFIG_ARM64
	THERMAL_WRAP_WR32(0x10000, TEMPPROTCTL);
#else
	THERMAL_WRAP_WR32(0x20000, TEMPPROTCTL);
#endif
	THERMAL_WRAP_WR32(raw_low, TEMPPROTTA);
    if (temperature2 > -275000)
		THERMAL_WRAP_WR32(raw_middle, TEMPPROTTB); 

	THERMAL_WRAP_WR32(raw_high, TEMPPROTTC);

	
	
	
	if (temperature2 > -275000)
		THERMAL_WRAP_WR32(temp | 0xC0000000, TEMPMONINT);	
	else
		THERMAL_WRAP_WR32(temp | 0x80000000, TEMPMONINT);	

	
}

int mtk_cpufreq_register(struct mtk_cpu_power_info *freqs, int num)
{
	int i = 0;
    int gpu_power = 0;
	tscpu_dprintk("mtk_cpufreq_register\n");

	tscpu_num_opp = num;

	mtk_cpu_power = kzalloc((num) * sizeof(struct mtk_cpu_power_info), GFP_KERNEL);
	if (mtk_cpu_power==NULL)
		return -ENOMEM;

	
	if (0 != Num_of_GPU_OPP && NULL != mtk_gpu_power)
        gpu_power = mtk_gpu_power[Num_of_GPU_OPP-1].gpufreq_power;
    else
        tscpu_printk("Num_of_GPU_OPP is 0!\n");

#if 0
	if(Num_of_GPU_OPP==3)
        gpu_power = mtk_gpu_power[2].gpufreq_power;
    else if(Num_of_GPU_OPP==2)
        gpu_power = mtk_gpu_power[1].gpufreq_power;
	else 
        gpu_power = mtk_gpu_power[0].gpufreq_power;
#endif

	for (i=0; i<num; i++)
	{
	    int dmips = freqs[i].cpufreq_khz * freqs[i].cpufreq_ncpu / 1000;

	    int cl_id = (((freqs[i].cpufreq_power + gpu_power) + 99) / 100) - 7; 

		mtk_cpu_power[i].cpufreq_khz = freqs[i].cpufreq_khz;
		mtk_cpu_power[i].cpufreq_ncpu = freqs[i].cpufreq_ncpu;
		mtk_cpu_power[i].cpufreq_power = freqs[i].cpufreq_power;

        if (cl_id < CPU_COOLER_NUM)
        {
            if (tscpu_cpu_dmips[cl_id] < dmips)
                tscpu_cpu_dmips[cl_id] = dmips;
        }
	}

    {
        
        int base = (mtk_cpu_power[num-1].cpufreq_khz * mtk_cpu_power[num-1].cpufreq_ncpu)/1000;
    	for (i=0; i<CPU_COOLER_NUM; i++)
    	{
    	    if (0 == tscpu_cpu_dmips[i] || tscpu_cpu_dmips[i] < base)
    	        tscpu_cpu_dmips[i] = base;
    	    else
    	        base = tscpu_cpu_dmips[i];
    	}
    	mtktscpu_limited_dmips = base;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_cpufreq_register);

static int init_cooler(void)
{
	int i;
	int num = CPU_COOLER_NUM; 

	cl_dev_state = kzalloc((num) * sizeof(unsigned int), GFP_KERNEL);
	if(cl_dev_state==NULL)
		return -ENOMEM;

	cl_dev = (struct thermal_cooling_device **)kzalloc((num) * sizeof(struct thermal_cooling_device *), GFP_KERNEL);
	if(cl_dev==NULL)
		return -ENOMEM;

	cooler_name = kzalloc((num) * sizeof(char) * 20, GFP_KERNEL);
	if(cooler_name==NULL)
		return -ENOMEM;

	for(i=0; i<num; i++)
		sprintf(cooler_name+(i*20), "cpu%02d", i); 

	Num_of_OPP = num; 
	return 0;
}

int mtk_gpufreq_register(struct mtk_gpu_power_info *freqs, int num)
{
	int i = 0;
	tscpu_dprintk("mtk_gpufreq_register\n");
	mtk_gpu_power = kzalloc((num) * sizeof(struct mtk_gpu_power_info), GFP_KERNEL);
	if (mtk_gpu_power == NULL)
		return -ENOMEM;

	for (i=0; i<num; i++)
	{
		mtk_gpu_power[i].gpufreq_khz = freqs[i].gpufreq_khz;
		mtk_gpu_power[i].gpufreq_power = freqs[i].gpufreq_power;

		tscpu_dprintk("[%d].gpufreq_khz=%u, .gpufreq_power=%u\n",
                        i, freqs[i].gpufreq_khz, freqs[i].gpufreq_power);
	}

	Num_of_GPU_OPP = num; 
	return 0;
}
EXPORT_SYMBOL(mtk_gpufreq_register);

void mtkts_dump_cali_info(void)
{
	tscpu_dprintk("[calibration] g_adc_ge_t      = 0x%x\n",g_adc_ge_t);
	tscpu_dprintk("[calibration] g_adc_oe_t      = 0x%x\n",g_adc_oe_t);
	tscpu_dprintk("[calibration] g_degc_cali     = 0x%x\n",g_degc_cali);
	tscpu_dprintk("[calibration] g_adc_cali_en_t = 0x%x\n",g_adc_cali_en_t);
	tscpu_dprintk("[calibration] g_o_slope       = 0x%x\n",g_o_slope);
	tscpu_dprintk("[calibration] g_o_slope_sign  = 0x%x\n",g_o_slope_sign);
	tscpu_dprintk("[calibration] g_id            = 0x%x\n",g_id);

	tscpu_dprintk("[calibration] g_o_vtsmcu2     = 0x%x\n",g_o_vtsmcu2);
	tscpu_dprintk("[calibration] g_o_vtsmcu3     = 0x%x\n",g_o_vtsmcu3);
	tscpu_dprintk("[calibration] g_o_vtsmcu4     = 0x%x\n",g_o_vtsmcu4);
}

static void thermal_cal_prepare(void)
{
	U32 temp0 = 0, temp1 = 0, temp2 = 0;
#if 1
	
	
	

	temp0 = get_devinfo_with_index(29);
	temp1 = get_devinfo_with_index(28);
	temp2 = get_devinfo_with_index(30);

    calefuse1 = temp0;
    calefuse2 = temp1;
    calefuse3 = temp2;
	
	
#else

	temp0 = DRV_Reg32(0xF020652C);
    temp1 = DRV_Reg32(0xF0206528);
    temp2 = DRV_Reg32(0xF0206530);
#endif

	tscpu_printk("[calibration] temp0=0x%x, temp1=0x%x\n", temp0, temp1);
	

    g_adc_ge_t     = ((temp0 & 0xFFC00000)>>22);
	g_adc_oe_t     = ((temp0 & 0x003FF000)>>12);

	g_o_vtsmcu1    = (temp1 & 0x03FE0000)>>17;  
	g_o_vtsmcu2    = (temp1 & 0x0001FF00)>>8;   
	g_o_vtsmcu3    = (temp0 & 0x000001FF);      
	g_o_vtsmcu4    = (temp2 & 0xFF800000)>>23;  
 	g_o_vtsabb     = (temp2 & 0x007FC000)>>14;	

	g_degc_cali    = (temp1 & 0x0000007E)>>1;   
	g_adc_cali_en_t= (temp1 & 0x00000001);		
	g_o_slope_sign = (temp1 & 0x00000080)>>7;   
	g_o_slope      = (temp1 & 0xFC000000)>>26;  

	g_id           = (temp0 & 0x00000200)>>9;   

	if(g_id==0)
	{
		g_o_slope=0;
	}

	
	if(g_adc_cali_en_t == 1)
	{
		
	}
	else
	{
		tscpu_printk("This sample is not Thermal calibrated\n");
        #if 1 
		g_adc_ge_t     = 512;
		g_adc_oe_t     = 512;
		g_degc_cali    = 40;
		g_o_slope      = 0;
		g_o_slope_sign = 0;
        g_o_vtsmcu1    = 260;
		g_o_vtsmcu2    = 260;
		g_o_vtsmcu3    = 260;
		g_o_vtsmcu4    = 260;
		g_o_vtsabb     = 260;
		#endif
	}

	
}

static void thermal_cal_prepare_2(U32 ret)
{
	S32 format_1= 0,format_2= 0, format_3= 0, format_4= 0, format_5= 0;

	tscpu_printk("thermal_cal_prepare_2\n");

	g_ge = ((g_adc_ge_t - 512) * 10000 ) / 4096; 
	g_oe =  (g_adc_oe_t - 512);

	g_gain = (10000 + g_ge);

	format_1   = (g_o_vtsmcu1 + 3350 - g_oe);
	format_2   = (g_o_vtsmcu2 + 3350 - g_oe);
	format_3   = (g_o_vtsmcu3 + 3350 - g_oe);
	format_4   = (g_o_vtsmcu4 + 3350 - g_oe);
	format_5   = (g_o_vtsabb  + 3350 - g_oe);

	g_x_roomt[0]   = (((format_1   * 10000) / 4096) * 10000) / g_gain; 
	g_x_roomt[1]   = (((format_2   * 10000) / 4096) * 10000) / g_gain; 
	g_x_roomt[2]   = (((format_3   * 10000) / 4096) * 10000) / g_gain; 
	g_x_roomt[3]   = (((format_4   * 10000) / 4096) * 10000) / g_gain; 
	g_x_roomt[4]   = (((format_5   * 10000) / 4096) * 10000) / g_gain; 

    tscpu_dprintk("[calibration] g_ge         = 0x%x\n",g_ge);
    tscpu_dprintk("[calibration] g_gain       = 0x%x\n",g_gain);

    tscpu_dprintk("[calibration] g_x_roomt1   = 0x%x\n",g_x_roomt[0]);
    tscpu_dprintk("[calibration] g_x_roomt2   = 0x%x\n",g_x_roomt[1]);
    tscpu_dprintk("[calibration] g_x_roomt3   = 0x%x\n",g_x_roomt[2]);
    tscpu_dprintk("[calibration] g_x_roomt4   = 0x%x\n",g_x_roomt[3]);
    tscpu_dprintk("[calibration] g_x_roomtabb = 0x%x\n",g_x_roomt[4]);
}

#if THERMAL_CONTROLLER_HW_TP
static S32 temperature_to_raw_room(U32 ret)
{
	

	S32 t_curr = ret;
	S32 format_1 = 0;
	S32 format_2 = 0;
	S32 format_3[THERMAL_SENSOR_NUM] = {0};
	S32 format_4[THERMAL_SENSOR_NUM] = {0};
    S32 i, index=0, temp = 0;

	tscpu_dprintk("temperature_to_raw_room\n");

	if(g_o_slope_sign==0)
	{
		format_1 = t_curr-(g_degc_cali*1000/2);
		format_2 = format_1 * (165 + g_o_slope) * 18/15;
		format_2 = format_2 - 2*format_2;

		for (i=0; i<THERMAL_SENSOR_NUM; i++)
		{
			format_3[i] = format_2/1000 + g_x_roomt[i]*10;
			format_4[i] = (format_3[i]*4096/10000*g_gain)/100000 + g_oe;
			tscpu_dprintk("[Temperature_to_raw_roomt][roomt%d] format_1=%d, format_2=%d, format_3=%d, format_4=%d\n", i, format_1, format_2, format_3[i], format_4[i]);
		}

		
	}
	else 
	{
		format_1 = t_curr-(g_degc_cali*1000/2);
		format_2 = format_1 * (165 - g_o_slope) * 18/15;
		format_2 = format_2 - 2*format_2;

		for (i=0; i<THERMAL_SENSOR_NUM; i++)
		{
			format_3[i] = format_2/1000 + g_x_roomt[i]*10;
			format_4[i] = (format_3[i]*4096/10000*g_gain)/100000 + g_oe;
			tscpu_dprintk("[Temperature_to_raw_roomt][roomt%d] format_1=%d, format_2=%d, format_3=%d, format_4=%d\n", i, format_1, format_2, format_3[i], format_4[i]);
		}

		
	}

	temp = 0;
	for (i=0; i<THERMAL_SENSOR_NUM; i++)
	{
		if (temp < format_4[i])
		{
			temp = format_4[i];
			index = i;
		}
	}

	tscpu_dprintk("[Temperature_to_raw_roomt] temperature=%d, raw[%d]=%d", ret, index, format_4[index]);
	return format_4[index];
}
#endif

static S32 raw_to_temperature_roomt(U32 ret, thermal_sensor_name ts_name)
{
	S32 t_current = 0;
	S32 y_curr = ret;
	S32 format_1 = 0;
	S32 format_2 = 0;
	S32 format_3 = 0;
	S32 format_4 = 0;
	S32 xtoomt=0;

    xtoomt = g_x_roomt[ts_name];

	tscpu_dprintk("raw_to_temperature_room,ts_name=%d,xtoomt=%d\n",ts_name,xtoomt);

	if(ret==0)
	{
		return 0;
	}

	
	format_1 = ((g_degc_cali*10) >> 1);
	format_2 = (y_curr - g_oe);
	format_3 = (((((format_2) * 10000) >> 12 ) * 10000) / g_gain) - xtoomt;
	format_3 = format_3 * 15/18;

	
	if(g_o_slope_sign==0)
	{
		
		format_4 = ((format_3 * 100) / (165+g_o_slope)); 
	}
	else
	{
		
		format_4 = ((format_3 * 100) / (165-g_o_slope)); 
	}
	format_4 = format_4 - (format_4 << 1);

	
	t_current = format_1 + format_4; 

	tscpu_dprintk("raw_to_temperature_room,t_current=%d\n",t_current);
	return t_current;
}

static void thermal_calibration(void)
{
	if (g_adc_cali_en_t == 0)
		printk("#####  Not Calibration  ######\n");

	
	thermal_cal_prepare_2(0);
}

int get_immediate_temp2_wrap(void)
{
	int curr_temp;
#ifdef CONFIG_ARM64
	curr_temp = TS_ABB_T;
#else
	curr_temp = CORE_TSABB_T;
#endif
    tscpu_dprintk("get_immediate_temp2_wrap curr_temp=%d\n", curr_temp);

    return curr_temp;
}



int get_immediate_ts1_wrap(void)
{
	int curr_temp;
#ifdef CONFIG_ARM64
	curr_temp = MAX(CPU_TS1_T, SOC_TS1_T);
#else
	curr_temp = MAX(CA7_TS1_T, CA15_TS1_T);
#endif
    tscpu_dprintk("get_immediate_ts1_wrap curr_temp=%d\n", curr_temp);

    return curr_temp;
}

int get_immediate_ts2_wrap(void)
{
	int curr_temp;
#ifdef CONFIG_ARM64
	curr_temp = MAX(CPU_TS2_T, SOC_TS2_T);
#else
	curr_temp = MAX(CA7_TS2_T, CORE_TS2_T);
#endif
    tscpu_dprintk("get_immediate_ts2_wrap curr_temp=%d\n", curr_temp);

    return curr_temp;
}

int get_immediate_ts3_wrap(void)
{
	int curr_temp;
#ifdef CONFIG_ARM64
	curr_temp = MAX(GPU_TS3_T, SOC_TS3_T);
#else
	curr_temp = MAX(GPU_TS3_T, CA15_TS3_T);
#endif

    tscpu_dprintk("get_immediate_ts3_wrap curr_temp=%d\n", curr_temp);

    return curr_temp;
}

int get_immediate_ts4_wrap(void)
{
	int curr_temp;

#ifdef CONFIG_ARM64
	curr_temp = MAX(LTE_TS4_T, SOC_TS4_T);
#else
	curr_temp = MAX(GPU_TS4_T, CORE_TS4_T);
#endif
    tscpu_dprintk("get_immediate_ts4_wrap curr_temp=%d\n", curr_temp);

    return curr_temp;
}

static int read_tc_raw_and_temp(volatile u32 * tempmsr_name,thermal_sensor_name ts_name,int *ts_raw)
{
	int temp=0,raw=0;


    tscpu_dprintk("read_tc_raw_temp,tempmsr_name=0x%x,ts_name=%d\n",tempmsr_name,ts_name);

    raw = (tempmsr_name != 0) ? (DRV_Reg32(tempmsr_name)& 0x0fff) : 0;
	temp = (tempmsr_name != 0) ? raw_to_temperature_roomt(raw,ts_name) : 0;

    *ts_raw = raw;
    tscpu_dprintk("read_tc_raw_temp,ts_raw=%d,temp=%d\n",*ts_raw,temp*100);
	return (temp*100);
}


static void read_each_bank_TS(thermal_bank_name bank_num)
{
	tscpu_dprintk("read_each_bank_TS,bank_num=%d\n",bank_num);

	switch(bank_num){
	    case THERMAL_BANK0:
            #ifdef CONFIG_ARM64
			
			CPU_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&CPU_TS1_R);
			CPU_TS2_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR2,&CPU_TS2_R);
            #else
			
			CA7_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&CA7_TS1_R);
			CA7_TS2_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR2,&CA7_TS2_R);
            #endif
	        break;
	    case THERMAL_BANK1:
            #ifdef CONFIG_ARM64
			
			LTE_TS4_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR4,&LTE_TS4_R);
			TS_ABB_T  = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSORABB,&TS_ABB_R);
            #else
            
			CA15_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&CA15_TS1_R);
			CA15_TS3_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR3,&CA15_TS3_R);
            #endif
	        break;
	    case THERMAL_BANK2:
            #ifdef CONFIG_ARM64
			
			GPU_TS3_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR3,&GPU_TS3_R);
			SOC_TS4_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR4,&SOC_TS4_R);
            #else
			
			GPU_TS3_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR3,&GPU_TS3_R);
			GPU_TS4_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR4,&GPU_TS4_R);
            #endif
	        break;
	    case THERMAL_BANK3:
            #ifdef CONFIG_ARM64
			
			SOC_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&SOC_TS1_R);
			SOC_TS2_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR2,&SOC_TS2_R);
			SOC_TS3_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR2,THERMAL_SENSOR3,&SOC_TS3_R);
			SOC_TS4_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR3,THERMAL_SENSOR4,&SOC_TS4_R);

            #else
            
			CORE_TS2_T 	 = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR2,&CORE_TS2_R);
			CORE_TS4_T 	 = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR4,&CORE_TS4_R);
			CORE_TSABB_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR2,THERMAL_SENSORABB,&CORE_TSABB_R);
            #endif
	        break;
		default:
            #ifdef CONFIG_ARM64
			
			CPU_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&CPU_TS1_R);
			CPU_TS2_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR2,&CPU_TS2_R);
			#else
			CA7_TS1_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR0,THERMAL_SENSOR1,&CA7_TS1_R);
			CA7_TS2_T = read_tc_raw_and_temp((volatile u32 *)TEMPMSR1,THERMAL_SENSOR2,&CA7_TS2_R);
            #endif
			break;
    }
}

static void read_all_bank_temperature(void)
{
	int i=0;
    unsigned long flags;
	tscpu_dprintk("read_all_bank_temperature\n");

	mt_ptp_lock(&flags);

    for(i=0;i<ROME_BANK_NUM;i++){
		mtktscpu_switch_bank(i);
		read_each_bank_TS(i);
    }

	mt_ptp_unlock(&flags);
}

static int tscpu_get_temp(struct thermal_zone_device *thermal,
                          unsigned long *t)
{
#if MTK_TS_CPU_SW_FILTER == 1
    int ret = 0;
	int curr_temp;
#ifndef CONFIG_ARM64
    int curr_temp1;
    int curr_temp2;
#endif
    int temp_temp;
	int bank0_T;
	int bank1_T;
	int bank2_T;
	int bank3_T;

    static int last_cpu_real_temp = 0;

#if (THERMAL_DRV_FAST_POLL_HRTIMER==0)
	read_all_bank_temperature();
#endif

#ifdef CONFIG_ARM64
	bank0_T    = MAX(CPU_TS1_T,CPU_TS2_T);
   	bank1_T    = LTE_TS4_T;
	bank2_T    = GPU_TS3_T;
    bank3_T    = MAX(MAX(SOC_TS1_T,SOC_TS2_T),MAX(SOC_TS4_T,SOC_TS3_T));
    curr_temp  = MAX(MAX(bank0_T,bank1_T),MAX(bank2_T,bank3_T));
#else
	bank0_T    = MAX(CA7_TS1_T,CA7_TS2_T);
   	bank1_T    = MAX(CA15_TS1_T,CA15_TS3_T);
	bank2_T    = MAX(GPU_TS3_T,GPU_TS4_T);
    bank3_T    = MAX(CORE_TS2_T,CORE_TS4_T);
    bank3_T    = MAX(bank3_T,CORE_TSABB_T);
    curr_temp1 = MAX(bank0_T,bank1_T);
	curr_temp2 = MAX(bank2_T,bank3_T);
    curr_temp  = MAX(curr_temp1,curr_temp2);
#endif
	
	tscpu_dprintk("bank0_T =%d,bank1_T =%d,bank2_T =%d,bank3_T =%d\n",bank0_T,bank1_T,bank2_T,bank3_T);
	
	tscpu_dprintk("tscpu_get_temp MAX CPU T=%d\n",curr_temp);

    



#if (CONFIG_THERMAL_AEE_RR_REC== 1)
	#ifdef CONFIG_ARM64
	p_thermal_temp1 = CPU_TS1_T;
    p_thermal_temp2 = CPU_TS2_T;
    p_thermal_temp3 = GPU_TS3_T;
    p_thermal_temp4 = LTE_TS4_T;
    p_thermal_temp5 = CORE_TSABB_T;
	p_thermal_state = TSCPU_NORMAL;
	#else
	p_thermal_temp1 = CA7_TS1_T;
    p_thermal_temp2 = CA7_TS2_T;
    p_thermal_temp3 = CA15_TS3_T;
    p_thermal_temp4 = GPU_TS4_T;
    p_thermal_temp5 = CORE_TSABB_T;
	p_thermal_state = TSCPU_NORMAL;
	#endif
#endif

	if((curr_temp > (trip_temp[0] - 15000)) || (curr_temp < -30000) || (curr_temp > 85000) )
	{
		tscpu_printk("CPU T=%d\n",curr_temp);
    }

    temp_temp = curr_temp;
    if (curr_temp != 0) 
    {
        if ((curr_temp > 150000) || (curr_temp < -20000)) 
        {
            tscpu_printk("CPU temp invalid=%d\n", curr_temp);
            temp_temp = 50000;
            ret = -1;
        }
        else if (last_cpu_real_temp != 0)
        {
            if ((curr_temp - last_cpu_real_temp > 40000) || (last_cpu_real_temp - curr_temp > 40000)) 
            {
                tscpu_printk("CPU temp float hugely temp=%d, lasttemp=%d\n", curr_temp, last_cpu_real_temp);
               
                temp_temp = 50000;
                ret = -1;
            }
        }
    }

    last_cpu_real_temp = curr_temp;
    curr_temp = temp_temp;
	
#else
    int ret = 0;
	int curr_temp;

	curr_temp = get_immediate_temp1();

	tscpu_dprintk("tscpu_get_temp CPU T1=%d\n",curr_temp);

	if((curr_temp > (trip_temp[0] - 15000)) || (curr_temp<-30000))
		printk("[Power/CPU_Thermal] CPU T=%d\n",curr_temp);
#endif

	read_curr_temp = curr_temp;
	*t = (unsigned long) curr_temp;

#if MTKTSCPU_FAST_POLLING
    cur_fp_factor = next_fp_factor;

    if (curr_temp >= fast_polling_trip_temp)
    {
        next_fp_factor = fast_polling_factor;
        
        thermal->polling_delay = interval/fast_polling_factor;
    }
    else
    {
        next_fp_factor = 1;
        thermal->polling_delay = interval;
    }
#endif

    
    if ((int) *t >= polling_trip_temp1)
#if MTKTSCPU_FAST_POLLING
        ;
#else
        thermal->polling_delay = interval;
#endif
    else if ((int) *t < polling_trip_temp2)
        thermal->polling_delay = interval * polling_factor2;
    else
        thermal->polling_delay = interval * polling_factor1;

	
    g_hr_timer_polling_delay = TS_MS_TO_NS(thermal->polling_delay);

    
#if CPT_ADAPTIVE_AP_COOLER
	g_prev_temp = g_curr_temp;
	g_curr_temp = curr_temp;
#endif

#if THERMAL_GPIO_OUT_TOGGLE
	
	tscpu_set_GPIO_toggle_for_monitor();
#endif

	g_max_temp = curr_temp;

	return ret;
}

int tscpu_get_temp_by_bank(thermal_bank_name ts_name)
{
	int bank_T = 0;

	tscpu_dprintk("tscpu_get_temp_by_bank,bank=%d\n",ts_name);

#ifdef CONFIG_ARM64
	if(ts_name == THERMAL_BANK0){
		bank_T = MAX(CPU_TS1_T,CPU_TS2_T);
	}
	else if(ts_name == THERMAL_BANK1){
		bank_T = LTE_TS4_T;
	}
	else if(ts_name == THERMAL_BANK2){
		bank_T = GPU_TS3_T;
	}
	else if(ts_name == THERMAL_BANK3){
		bank_T = MAX(MAX(SOC_TS1_T,SOC_TS2_T),MAX(SOC_TS3_T,SOC_TS4_T));
	}
#else
	if(ts_name == THERMAL_BANK0){
		bank_T = MAX(CA7_TS1_T,CA7_TS2_T);
	}
	else if(ts_name == THERMAL_BANK1){
		bank_T = MAX(CA15_TS1_T,CA15_TS3_T);
	}
	else if(ts_name == THERMAL_BANK2){
		bank_T = MAX(GPU_TS3_T,GPU_TS4_T);
	}
	else if(ts_name == THERMAL_BANK3){
		bank_T = MAX(CORE_TS2_T,CORE_TS4_T);
		bank_T = MAX(bank_T,CORE_TSABB_T);
	}
#endif
#if 0
	printk("\n\n");
	printk("Bank 0 : CA7  (TS1 = %d,TS2 = %d)\n",CA7_TS1_T,CA7_TS2_T);
	printk("Bank 1 : CA15 (TS1 = %d,TS3 = %d)\n",CA15_TS1_T,CA15_TS3_T);
	printk("Bank 2 : GPU  (TS3 = %d,TS4 = %d)\n",GPU_TS3_T,GPU_TS4_T);
	printk("Bank 3 : CORE (TS2 = %d,TS4 = %d,TSABB = %d)\n",CORE_TS2_T,CORE_TS4_T,CORE_TSABB_T);
#endif

	return bank_T;
}

#if 0
int tscpu_get_Tj_temp(void)
{
	
	return g_max_temp;
}
EXPORT_SYMBOL(tscpu_get_Tj_temp);
#endif

static int tscpu_bind(struct thermal_zone_device *thermal,
						struct thermal_cooling_device *cdev)
{
	int table_val = 0;

	if(!strcmp(cdev->type, g_bind0))
	{
		table_val = 0;

		tscpu_config_all_tc_hw_protect(trip_temp[0], tc_mid_trip);
		
	}
	else if(!strcmp(cdev->type, g_bind1))
	{
		table_val = 1;
		tc_mid_trip = trip_temp[1]; 

		tscpu_config_all_tc_hw_protect(trip_temp[0], tc_mid_trip);
		
	}
	else if(!strcmp(cdev->type, g_bind2))
	{
		table_val = 2;
		
	}
	else if(!strcmp(cdev->type, g_bind3))
	{
		table_val = 3;
		
	}
	else if(!strcmp(cdev->type, g_bind4))
	{
		table_val = 4;
		
	}
	else if(!strcmp(cdev->type, g_bind5))
	{
		table_val = 5;
		
	}
	else if(!strcmp(cdev->type, g_bind6))
	{
		table_val = 6;
		
	}
	else if(!strcmp(cdev->type, g_bind7))
	{
		table_val = 7;
		
	}
	else if(!strcmp(cdev->type, g_bind8))
	{
		table_val = 8;
		
	}
	else if(!strcmp(cdev->type, g_bind9))
	{
		table_val = 9;
		
	}
	else
	{
		return 0;
	}

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		tscpu_printk("tscpu_bind error binding cooling dev\n");
		return -EINVAL;
	} else {
		tscpu_printk("tscpu_bind binding OK, %d\n", table_val);
	}

	return 0;
}

static int tscpu_unbind(struct thermal_zone_device *thermal,
                           struct thermal_cooling_device *cdev)
{
	int table_val=0;

	if(!strcmp(cdev->type, g_bind0))
	{
		table_val = 0;
		
	}
	else if(!strcmp(cdev->type, g_bind1))
	{
		table_val = 1;
		tc_mid_trip = -275000; 
		
	}
	else if(!strcmp(cdev->type, g_bind2))
	{
		table_val = 2;
		
	}
	else if(!strcmp(cdev->type, g_bind3))
	{
		table_val = 3;
		
	}
	else if(!strcmp(cdev->type, g_bind4))
	{
		table_val = 4;
		
	}
	else if(!strcmp(cdev->type, g_bind5))
	{
		table_val = 5;
		
	}
	else if(!strcmp(cdev->type, g_bind6))
	{
		table_val = 6;
		
	}
	else if(!strcmp(cdev->type, g_bind7))
	{
		table_val = 7;
		
	}
	else if(!strcmp(cdev->type, g_bind8))
	{
		table_val = 8;
		
	}
	else if(!strcmp(cdev->type, g_bind9))
	{
		table_val = 9;
		
	}
	else
		return 0;


	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		tscpu_printk("tscpu_unbind error unbinding cooling dev\n");
	return -EINVAL;
	} else {
		tscpu_printk("tscpu_unbind unbinding OK\n");
	}

	return 0;
}

static int tscpu_get_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED
		: THERMAL_DEVICE_DISABLED;
	return 0;
}

static int tscpu_set_mode(struct thermal_zone_device *thermal,
							enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int tscpu_get_trip_type(struct thermal_zone_device *thermal, int trip,
								enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int tscpu_get_trip_temp(struct thermal_zone_device *thermal, int trip,
								unsigned long *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int tscpu_get_crit_temp(struct thermal_zone_device *thermal,
								unsigned long *temperature)
{
	*temperature = MTKTSCPU_TEMP_CRIT;
	return 0;
}

static struct thermal_zone_device_ops mtktscpu_dev_ops = {
	.bind = tscpu_bind,
	.unbind = tscpu_unbind,
	.get_temp = tscpu_get_temp,
	.get_mode = tscpu_get_mode,
	.set_mode = tscpu_set_mode,
	.get_trip_type = tscpu_get_trip_type,
	.get_trip_temp = tscpu_get_trip_temp,
	.get_crit_temp = tscpu_get_crit_temp,
};

static int previous_step=-1;

static int tscpu_set_power_consumption_state(void)
{
	int i=0;
	int power=0;

	tscpu_dprintk("tscpu_set_power_consumption_state Num_of_OPP=%d\n",Num_of_OPP);

	
	for(i=0; i<Num_of_OPP; i++)
	{
		if(1==cl_dev_state[i])
		{
			if(i!=previous_step)
			{
				tscpu_printk("previous_opp=%d, now_opp=%d\n", previous_step, i);
				previous_step=i;
				mtktscpu_limited_dmips = tscpu_cpu_dmips[previous_step];
				if(Num_of_GPU_OPP==3)
				{
					power = (i*100+700) - mtk_gpu_power[Num_of_GPU_OPP-1].gpufreq_power;
					set_static_cpu_power_limit(power);
					set_static_gpu_power_limit(mtk_gpu_power[Num_of_GPU_OPP-1].gpufreq_power);
                    tscpu_dprintk("Num_of_GPU_OPP=%d, gpufreq_power=%d, power=%d\n", Num_of_GPU_OPP, mtk_gpu_power[Num_of_GPU_OPP-1].gpufreq_power, power);
				}
				else if(Num_of_GPU_OPP==2)
				{
					power = (i*100+700) - mtk_gpu_power[1].gpufreq_power;
					set_static_cpu_power_limit(power);
					set_static_gpu_power_limit(mtk_gpu_power[1].gpufreq_power);
                    tscpu_dprintk("Num_of_GPU_OPP=%d, gpufreq_power=%d, power=%d\n", Num_of_GPU_OPP, mtk_gpu_power[1].gpufreq_power, power);
				}
				else if(Num_of_GPU_OPP==1)
				{
					#if 0
			        
			        
					power = (i*100+700) - 653;
                    #else
                    power = (i*100+700) - mtk_gpu_power[0].gpufreq_power;
                    #endif
					set_static_cpu_power_limit(power);
                    tscpu_dprintk("Num_of_GPU_OPP=%d, gpufreq_power=%d, power=%d\n", Num_of_GPU_OPP, mtk_gpu_power[0].gpufreq_power,power);
				}
				else 
				{
				    power = (i*100+700);
				    set_static_cpu_power_limit(power);
				    tscpu_dprintk("Num_of_GPU_OPP=%d, gpufreq_power=%d, power=%d\n", Num_of_GPU_OPP, mtk_gpu_power[0].gpufreq_power, power);
				}
			}
			break;
		}
	}

	
	if (i == Num_of_OPP)
	{
		if (previous_step != -1)
		{
			previous_step = -1;
			mtktscpu_limited_dmips = tscpu_cpu_dmips[CPU_COOLER_NUM-1]; 
			tscpu_printk("Free all static thermal limit, previous_opp=%d\n", previous_step);
			set_static_cpu_power_limit(0);
			set_static_gpu_power_limit(0);
		}
	}
	return 0;
}

#if CPT_ADAPTIVE_AP_COOLER

extern unsigned long (*mtk_thermal_get_gpu_loading_fp)(void);

static int GPU_L_H_TRIP = 80, GPU_L_L_TRIP = 40;

static int TARGET_TJ = 65000;
static int TARGET_TJ_HIGH = 66000;
static int TARGET_TJ_LOW = 64000;
static int PACKAGE_THETA_JA_RISE = 10;
static int PACKAGE_THETA_JA_FALL = 10;
static int MINIMUM_CPU_POWER = 500;
static int MAXIMUM_CPU_POWER = 1240;
static int MINIMUM_GPU_POWER = 676;
static int MAXIMUM_GPU_POWER = 676;
static int MINIMUM_TOTAL_POWER = 500+676;
static int MAXIMUM_TOTAL_POWER = 1240+676;
static int FIRST_STEP_TOTAL_POWER_BUDGET = 1750;

static int MINIMUM_BUDGET_CHANGE = 50;

static int P_adaptive(int total_power, unsigned int gpu_loading)
{
    
    
    static int cpu_power = 0, gpu_power = 0;
    static int last_cpu_power = 0, last_gpu_power = 0;

    last_cpu_power = cpu_power;
    last_gpu_power = gpu_power;

    if (total_power == 0)
    {
        cpu_power = gpu_power = 0;
#if THERMAL_HEADROOM
        if (thp_max_cpu_power != 0)
            set_adaptive_cpu_power_limit((unsigned int) MAX(thp_max_cpu_power, MINIMUM_CPU_POWER));
        else
            set_adaptive_cpu_power_limit(0);
#else
        set_adaptive_cpu_power_limit(0);
#endif
        set_adaptive_gpu_power_limit(0);
        return 0;
    }

    if (total_power <= MINIMUM_TOTAL_POWER)
    {
        cpu_power = MINIMUM_CPU_POWER;
        gpu_power = MINIMUM_GPU_POWER;
    }
    else if (total_power >= MAXIMUM_TOTAL_POWER)
    {
        cpu_power = MAXIMUM_CPU_POWER;
        gpu_power = MAXIMUM_GPU_POWER;
    }
    else
    {
        int max_allowed_gpu_power = MIN((total_power - MINIMUM_CPU_POWER), MAXIMUM_GPU_POWER);
        int highest_possible_gpu_power = -1;
        
        int i = 0;
        unsigned int cur_gpu_freq = mt_gpufreq_get_cur_freq();
        
        unsigned int cur_gpu_power = 0;
        unsigned int next_lower_gpu_power = 0;

        
        for (; i < Num_of_GPU_OPP; i++)
        {
            if ((mtk_gpu_power[i].gpufreq_power <= max_allowed_gpu_power) &&
                (-1 == highest_possible_gpu_power))
            {
                highest_possible_gpu_power = mtk_gpu_power[i].gpufreq_power;
                
            }

            if (mtk_gpu_power[i].gpufreq_khz == cur_gpu_freq)
            {
                next_lower_gpu_power = cur_gpu_power = mtk_gpu_power[i].gpufreq_power;
                

                if ((i != Num_of_GPU_OPP - 1) && (mtk_gpu_power[i+1].gpufreq_power >= MINIMUM_GPU_POWER))
                {
                    next_lower_gpu_power = mtk_gpu_power[i+1].gpufreq_power;
                }
            }
        }

        
        if (gpu_loading > GPU_L_H_TRIP)
        {
            gpu_power = highest_possible_gpu_power;
        }
        else if (gpu_loading <= GPU_L_L_TRIP)
        {
            gpu_power = MIN(next_lower_gpu_power, highest_possible_gpu_power);
            gpu_power = MAX(gpu_power, MINIMUM_GPU_POWER);
        }
        else
        {
            gpu_power = MIN(highest_possible_gpu_power, cur_gpu_power);
        }

        cpu_power = MIN((total_power - gpu_power), MAXIMUM_CPU_POWER);
    }

    if (cpu_power != last_cpu_power)
    {
        set_adaptive_cpu_power_limit(cpu_power);
	}
    if (gpu_power != last_gpu_power)
    {
        set_adaptive_gpu_power_limit(gpu_power);
    }

    tscpu_dprintk("%s cpu %d, gpu %d\n", __func__, cpu_power, gpu_power);

    return 0;
}

static int _adaptive_power(long prev_temp, long curr_temp, unsigned int gpu_loading)
{
	static int triggered = 0, total_power = 0;
	int delta_power = 0;

	if (cl_dev_adp_cpu_state_active == 1)
	{
        tscpu_dprintk("%s %d %d %d %d %d %d %d\n", __func__, FIRST_STEP_TOTAL_POWER_BUDGET, PACKAGE_THETA_JA_RISE, PACKAGE_THETA_JA_FALL, MINIMUM_BUDGET_CHANGE, MINIMUM_CPU_POWER, MAXIMUM_CPU_POWER, MINIMUM_GPU_POWER, MAXIMUM_GPU_POWER);

		
		if (!triggered)
		{
			if (curr_temp < TARGET_TJ)
				return 0;
			else
			{
				triggered = 1;
				switch(mtktscpu_atm){
                case 1:	
				case 2:	
#if MTKTSCPU_FAST_POLLING
                    total_power = FIRST_STEP_TOTAL_POWER_BUDGET - ((curr_temp - TARGET_TJ)*tt_ratio_high_rise+(curr_temp - prev_temp)*tp_ratio_high_rise)/(PACKAGE_THETA_JA_RISE*cur_fp_factor);
#else
                    total_power = FIRST_STEP_TOTAL_POWER_BUDGET - ((curr_temp - TARGET_TJ)*tt_ratio_high_rise+(curr_temp - prev_temp)*tp_ratio_high_rise)/PACKAGE_THETA_JA_RISE;
#endif
                    break;
                case 0:
                default: 
			        total_power = FIRST_STEP_TOTAL_POWER_BUDGET;
                }
				tscpu_dprintk("%s Tp %d, Tc %d, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
				return P_adaptive(total_power, gpu_loading);
			}
		}

		
		switch(mtktscpu_atm)
        {
        case 1:	
		case 2:	
            if((curr_temp >= TARGET_TJ_HIGH) && (curr_temp > prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                total_power -= MAX(((curr_temp - TARGET_TJ)*tt_ratio_high_rise+(curr_temp - prev_temp)*tp_ratio_high_rise)/(PACKAGE_THETA_JA_RISE*cur_fp_factor), MINIMUM_BUDGET_CHANGE);
#else
                total_power -= MAX(((curr_temp - TARGET_TJ)*tt_ratio_high_rise+(curr_temp - prev_temp)*tp_ratio_high_rise)/PACKAGE_THETA_JA_RISE, MINIMUM_BUDGET_CHANGE);
#endif
            }
            else if((curr_temp >= TARGET_TJ_HIGH) && (curr_temp <= prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                total_power -= MAX(((curr_temp - TARGET_TJ)*tt_ratio_high_fall-(prev_temp - curr_temp)*tp_ratio_high_fall)/(PACKAGE_THETA_JA_FALL*cur_fp_factor), MINIMUM_BUDGET_CHANGE);
#else
                total_power -= MAX(((curr_temp - TARGET_TJ)*tt_ratio_high_fall-(prev_temp - curr_temp)*tp_ratio_high_fall)/PACKAGE_THETA_JA_FALL, MINIMUM_BUDGET_CHANGE);
#endif
            }
            else if((curr_temp <= TARGET_TJ_LOW) && (curr_temp > prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                total_power += MAX(((TARGET_TJ - curr_temp)*tt_ratio_low_rise-(curr_temp - prev_temp)*tp_ratio_low_rise)/(PACKAGE_THETA_JA_RISE*cur_fp_factor), MINIMUM_BUDGET_CHANGE);
#else
                total_power += MAX(((TARGET_TJ - curr_temp)*tt_ratio_low_rise-(curr_temp - prev_temp)*tp_ratio_low_rise)/PACKAGE_THETA_JA_RISE, MINIMUM_BUDGET_CHANGE);
#endif
            }
            else if((curr_temp <= TARGET_TJ_LOW) && (curr_temp <= prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                total_power += MAX(((TARGET_TJ - curr_temp)*tt_ratio_low_fall+(prev_temp - curr_temp)*tp_ratio_low_fall)/(PACKAGE_THETA_JA_FALL*cur_fp_factor), MINIMUM_BUDGET_CHANGE);
#else
                total_power += MAX(((TARGET_TJ - curr_temp)*tt_ratio_low_fall+(prev_temp - curr_temp)*tp_ratio_low_fall)/PACKAGE_THETA_JA_FALL, MINIMUM_BUDGET_CHANGE);
#endif
            }

            total_power = (total_power > MINIMUM_TOTAL_POWER) ? total_power : MINIMUM_TOTAL_POWER;
            total_power = (total_power < MAXIMUM_TOTAL_POWER) ? total_power : MAXIMUM_TOTAL_POWER;
            break;

        case 0:
        default: 
            if ((curr_temp > TARGET_TJ_HIGH) && (curr_temp >= prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                delta_power = (curr_temp - prev_temp) / (PACKAGE_THETA_JA_RISE*cur_fp_factor);
#else
                delta_power = (curr_temp - prev_temp) / PACKAGE_THETA_JA_RISE;
#endif
                if (prev_temp > TARGET_TJ_HIGH)
                {
                    delta_power = (delta_power > MINIMUM_BUDGET_CHANGE) ? delta_power : MINIMUM_BUDGET_CHANGE;
                }
                total_power -= delta_power;
                total_power = (total_power > MINIMUM_TOTAL_POWER) ? total_power : MINIMUM_TOTAL_POWER;
            }

            if ((curr_temp < TARGET_TJ_LOW) && (curr_temp <= prev_temp))
            {
#if MTKTSCPU_FAST_POLLING
                delta_power = (prev_temp - curr_temp) / (PACKAGE_THETA_JA_FALL*cur_fp_factor);
#else
                delta_power = (prev_temp - curr_temp) / PACKAGE_THETA_JA_FALL;
#endif
                if (prev_temp < TARGET_TJ_LOW)
                {
                    delta_power = (delta_power > MINIMUM_BUDGET_CHANGE) ? delta_power : MINIMUM_BUDGET_CHANGE;
                }
                total_power += delta_power;
                total_power = (total_power < MAXIMUM_TOTAL_POWER) ? total_power : MAXIMUM_TOTAL_POWER;
            }
            break;
        }

        tscpu_dprintk("%s Tp %d, Tc %d, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
		return P_adaptive(total_power, gpu_loading);
	}
#if CONTINUOUS_TM
    else if ((cl_dev_adp_cpu_state_active == 1) && (ctm_on ==1) && (curr_temp < current_ETJ))
    {
        tscpu_printk("CTM exit curr_temp %d cetj %d\n", TARGET_TJ, current_ETJ);
        
        if (triggered)
		{
			triggered = 0;
			tscpu_dprintk("%s Tp %d, Tc %d, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
			return P_adaptive(0, 0);
		}
#if THERMAL_HEADROOM
		else
		{
            if (thp_max_cpu_power != 0)
		        set_adaptive_cpu_power_limit((unsigned int) MAX(thp_max_cpu_power, MINIMUM_CPU_POWER));
		    else
		        set_adaptive_cpu_power_limit(0);
		}
#endif
    }
#endif
	else
	{
		if (triggered)
		{
			triggered = 0;
			tscpu_dprintk("%s Tp %d, Tc %d, Pt %d\n", __func__, prev_temp, curr_temp, total_power);
			return P_adaptive(0, 0);
		}
#if THERMAL_HEADROOM
		else
		{
            if (thp_max_cpu_power != 0)
		        set_adaptive_cpu_power_limit((unsigned int) MAX(thp_max_cpu_power, MINIMUM_CPU_POWER));
		    else
		        set_adaptive_cpu_power_limit(0);
		}
#endif
	}

	return 0;
}

static int decide_ttj(void)
{
    int i = 0;
    int active_cooler_id = -1;
    int ret = 117000; 
    int temp_cl_dev_adp_cpu_state_active = 0;
    for (; i < MAX_CPT_ADAPTIVE_COOLERS; i++)
    {
        if (cl_dev_adp_cpu_state[i])
        {
            ret = MIN(ret, TARGET_TJS[i]);
            temp_cl_dev_adp_cpu_state_active = 1;

            if (ret == TARGET_TJS[i])
                active_cooler_id = i;
        }
    }
    cl_dev_adp_cpu_state_active = temp_cl_dev_adp_cpu_state_active;
    TARGET_TJ = ret;
#if CONTINUOUS_TM
    if (ctm_on == 1)
    {
        TARGET_TJ = MIN(MAX_TARGET_TJ, MAX(STEADY_TARGET_TJ, (COEF_AE - COEF_BE*bts_cur_temp/1000)));
        current_ETJ = MIN(MAX_EXIT_TJ, MAX(STEADY_EXIT_TJ, (COEF_AX - COEF_BX*bts_cur_temp/1000)));
        
    }
#endif
    
    
    
    TARGET_TJ_HIGH = TARGET_TJ_LOW = TARGET_TJ;

    if (0 <= active_cooler_id && MAX_CPT_ADAPTIVE_COOLERS > active_cooler_id)
    {
        PACKAGE_THETA_JA_RISE = PACKAGE_THETA_JA_RISES[active_cooler_id];
        PACKAGE_THETA_JA_FALL = PACKAGE_THETA_JA_FALLS[active_cooler_id];
        MINIMUM_CPU_POWER = MINIMUM_CPU_POWERS[active_cooler_id];
        MAXIMUM_CPU_POWER = MAXIMUM_CPU_POWERS[active_cooler_id];
        MINIMUM_GPU_POWER = MINIMUM_GPU_POWERS[active_cooler_id];
        MAXIMUM_GPU_POWER = MAXIMUM_GPU_POWERS[active_cooler_id];
        MINIMUM_TOTAL_POWER = MINIMUM_CPU_POWER+MINIMUM_GPU_POWER;
        MAXIMUM_TOTAL_POWER = MAXIMUM_CPU_POWER+MAXIMUM_GPU_POWER;
        FIRST_STEP_TOTAL_POWER_BUDGET = FIRST_STEP_TOTAL_POWER_BUDGETS[active_cooler_id];
        MINIMUM_BUDGET_CHANGE = MINIMUM_BUDGET_CHANGES[active_cooler_id];
    }
    else
    {
        MINIMUM_CPU_POWER = MINIMUM_CPU_POWERS[0];
        MAXIMUM_CPU_POWER = MAXIMUM_CPU_POWERS[0];
    }

#if THERMAL_HEADROOM
    MAXIMUM_CPU_POWER -= p_Tpcb_correlation * MAX((bts_cur_temp - Tpcb_trip_point), 0) / 1000;

    

    thp_max_cpu_power = (thp_threshold_tj - read_curr_temp) * thp_p_tj_correlation / 1000 + 0; 

    if (thp_max_cpu_power != 0)
        MAXIMUM_CPU_POWER = MIN(MAXIMUM_CPU_POWER, thp_max_cpu_power);

    MAXIMUM_CPU_POWER = MAX(MAXIMUM_CPU_POWER, MINIMUM_CPU_POWER);

    
#endif

    return ret;
}
#endif

static int dtm_cpu_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	
	*state = 1;
	return 0;
}

static int dtm_cpu_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	int i=0;
	

	for(i=0; i<Num_of_OPP; i++)
	{
		if(!strcmp(cdev->type, &cooler_name[i*20]))
		{
			*state = cl_dev_state[i];
		}
	}
	return 0;
}

static int dtm_cpu_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	int i=0;
	

	for(i=0; i<Num_of_OPP; i++)
	{
		if(!strcmp(cdev->type, &cooler_name[i*20]))
		{
			cl_dev_state[i]=state;
			tscpu_set_power_consumption_state();
			break;
		}
	}
	return 0;
}

static int sysrst_cpu_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	
	*state = 1;
	return 0;
}

static int sysrst_cpu_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	
	*state = cl_dev_sysrst_state;
	return 0;
}

static int sysrst_cpu_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	cl_dev_sysrst_state = state;

	if(cl_dev_sysrst_state == 1)
	{
	    
		tscpu_printk("sysrst_cpu_set_cur_state = 1\n");
		tscpu_printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		tscpu_printk("*****************************************\n");
		tscpu_printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");

#ifndef CONFIG_ARM64
        BUG();
#else
        *(unsigned int*) 0x0 = 0xdead; 
#endif
	}
	return 0;
}

#if CPT_ADAPTIVE_AP_COOLER
static int adp_cpu_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	
	*state = 1;
	return 0;
}

static int adp_cpu_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	
    *state = cl_dev_adp_cpu_state[(cdev->type[13] - '0')];
	
	return 0;
}

static int adp_cpu_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
    int ttj = 117000;

	cl_dev_adp_cpu_state[(cdev->type[13] - '0')] = state;

    ttj = decide_ttj(); 

	

    if (active_adp_cooler == (int) (cdev->type[13] - '0'))
	{
	    unsigned int gpu_loading = 0; 
	    if (!mtk_get_gpu_loading(&gpu_loading))
	        gpu_loading = 0;

		_adaptive_power(g_prev_temp, g_curr_temp, (unsigned int) gpu_loading);
		
	}
	return 0;
}
#endif


static struct thermal_cooling_device_ops mtktscpu_cooling_F0x2_ops = {
	.get_max_state = dtm_cpu_get_max_state,
	.get_cur_state = dtm_cpu_get_cur_state,
	.set_cur_state = dtm_cpu_set_cur_state,
};

#if CPT_ADAPTIVE_AP_COOLER
static struct thermal_cooling_device_ops mtktscpu_cooler_adp_cpu_ops = {
	.get_max_state = adp_cpu_get_max_state,
	.get_cur_state = adp_cpu_get_cur_state,
	.set_cur_state = adp_cpu_set_cur_state,
};
#endif

static struct thermal_cooling_device_ops mtktscpu_cooling_sysrst_ops = {
	.get_max_state = sysrst_cpu_get_max_state,
	.get_cur_state = sysrst_cpu_get_cur_state,
	.set_cur_state = sysrst_cpu_set_cur_state,
};

static int tscpu_read_Tj_out(struct seq_file *m, void *v)
{
	int ts_con0=0;

	
	ts_con0 = DRV_Reg32(TS_CON0);

	seq_printf(m, "TS_CON0:0x%x\n", ts_con0);


	return 0;
}

static ssize_t tscpu_write_Tj_out(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int lv_Tj_out_flag;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &lv_Tj_out_flag) == 1)
	{
		if(lv_Tj_out_flag==1){
			
			THERMAL_WRAP_WR32(DRV_Reg32(TS_CON0) | 0x00010000, TS_CON0);
		}
        else{
			
			THERMAL_WRAP_WR32(DRV_Reg32(TS_CON0) & 0xfffeffff, TS_CON0);
        }

		tscpu_dprintk("tscpu_write_Tj_out lv_Tj_out_flag=%d\n", lv_Tj_out_flag);
		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_write_Tj_out bad argument\n");
	}
	return -EINVAL;
}

#if THERMAL_GPIO_OUT_TOGGLE
static int g_trigger_temp = 95000;
static int g_GPIO_out_enable = 0;
static int g_GPIO_already_set = 0;

#define GPIO118_MODE           (GPIO_BASE + 0x0770)
#define GPIO118_DIR            (GPIO_BASE + 0x0070)
#define GPIO118_DOUT           (GPIO_BASE + 0x0470)

void tscpu_set_GPIO_toggle_for_monitor(void)
{
	int lv_GPIO118_MODE,lv_GPIO118_DIR, lv_GPIO118_DOUT;
	tscpu_dprintk("tscpu_set_GPIO_toggle_for_monitor,g_GPIO_out_enable=%d\n",g_GPIO_out_enable);

	if(g_GPIO_out_enable == 1)
	{
	    if(g_max_temp > g_trigger_temp){

			tscpu_printk("g_max_temp %d > g_trigger_temp %d \n",g_max_temp ,g_trigger_temp);

			g_GPIO_out_enable =0;
			g_GPIO_already_set=1;

			lv_GPIO118_MODE = thermal_readl(GPIO118_MODE);
		    lv_GPIO118_DIR  = thermal_readl(GPIO118_DIR);
		    lv_GPIO118_DOUT = thermal_readl(GPIO118_DOUT);

            tscpu_printk("tscpu_set_GPIO_toggle_for_monitor:lv_GPIO118_MODE=0x%x,lv_GPIO118_DIR=0x%x,lv_GPIO118_DOUT=0x%x,\n",
                lv_GPIO118_MODE,lv_GPIO118_DIR,lv_GPIO118_DOUT);


			
			
			thermal_clrl(GPIO118_DOUT,0x00000040);
			udelay(200);
			thermal_setl(GPIO118_DOUT,0x00000040);
	    }else{
	        if(g_GPIO_already_set==1){
		        
		        g_GPIO_already_set=0;
				
				
				
				thermal_clrl(GPIO118_DOUT,0x00000040);
	        }
	    }
	}
}

static int tscpu_read_GPIO_out(struct seq_file *m, void *v)
{

	seq_printf(m, "GPIO out enable:%d, trigger temperature=%d\n", g_GPIO_out_enable,g_trigger_temp);

	return 0;
}

static ssize_t tscpu_write_GPIO_out(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[512];
	char TEMP[10],ENABLE[10];
    unsigned int valTEMP,valENABLE;
	int len = 0;
	int lv_GPIO118_MODE,lv_GPIO118_DIR;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

    if (sscanf(desc, "%s %d %s %d ",TEMP, &valTEMP, ENABLE, &valENABLE) == 4)
	{
		

        if (!strcmp(TEMP, "TEMP")) {
            g_trigger_temp = valTEMP;
            tscpu_printk("g_trigger_temp=%d\n",valTEMP);
        }else{
			tscpu_printk("tscpu_write_GPIO_out TEMP bad argument\n");
            return -EINVAL;
        }

        if (!strcmp(ENABLE, "ENABLE")) {
            g_GPIO_out_enable = valENABLE;
            tscpu_printk("g_GPIO_out_enable=%d,g_GPIO_already_set=%d\n",valENABLE,g_GPIO_already_set);
        }else{
			tscpu_printk("tscpu_write_GPIO_out ENABLE bad argument\n");
            return -EINVAL;
        }

		lv_GPIO118_MODE = thermal_readl(GPIO118_MODE);
	    lv_GPIO118_DIR  = thermal_readl(GPIO118_DIR);

		thermal_clrl(GPIO118_MODE,0x00000E00);
		thermal_setl(GPIO118_DIR, 0x00000040);
		thermal_clrl(GPIO118_DOUT,0x00000040);
		return count;
	}
	else
	{
		tscpu_printk("tscpu_write_GPIO_out bad argument\n");
	}

	return -EINVAL;
}
#endif

static int tscpu_read_opp(struct seq_file *m, void *v)
{
    unsigned int cpu_power, gpu_power;
    unsigned int gpu_loading = 0;

    cpu_power = MIN(adaptive_cpu_power_limit, static_cpu_power_limit);
    gpu_power = MIN(adaptive_gpu_power_limit, static_gpu_power_limit);

#if CPT_ADAPTIVE_AP_COOLER
    if (!mtk_get_gpu_loading(&gpu_loading))
        gpu_loading = 0;

    seq_printf(m, "%d,%d,%d,%d\n",
                    (int)((cpu_power != 0x7FFFFFFF)?cpu_power:0),
                    (int)((gpu_power != 0x7FFFFFFF)?gpu_power:0),
                    (int)gpu_loading,
                    (int)mt_gpufreq_get_cur_freq());
#else
    seq_printf(m, "%d,%d,0,%d\n",
                    (int)((cpu_power != 0x7FFFFFFF)?cpu_power:0),
                    (int)((gpu_power != 0x7FFFFFFF)?gpu_power:0),
                    (int)mt_gpufreq_get_cur_freq());
#endif

	return 0;
}

static int tscpu_read_temperature_info(struct seq_file *m, void *v)
{
    seq_printf(m, "current temp:%d\n", read_curr_temp);
  	seq_printf(m, "calefuse1:0x%x\n", calefuse1);
  	seq_printf(m, "calefuse2:0x%x\n", calefuse2);
   	seq_printf(m, "calefuse3:0x%x\n", calefuse3);
  	seq_printf(m, "g_adc_ge_t:%d\n", g_adc_ge_t);
    seq_printf(m, "g_adc_oe_t:%d\n", g_adc_oe_t);
    seq_printf(m, "g_degc_cali:%d\n", g_degc_cali);
    seq_printf(m, "g_adc_cali_en_t:%d\n", g_adc_cali_en_t);
    seq_printf(m, "g_o_slope:%d\n", g_o_slope);
    seq_printf(m, "g_o_slope_sign:%d\n", g_o_slope_sign);
    seq_printf(m, "g_id:%d\n", g_id);
    seq_printf(m, "g_o_vtsmcu1:%d\n", g_o_vtsmcu1);
    seq_printf(m, "g_o_vtsmcu2:%d\n", g_o_vtsmcu2);
    seq_printf(m, "g_o_vtsmcu3:%d\n", g_o_vtsmcu3);
    seq_printf(m, "g_o_vtsmcu4:%d\n", g_o_vtsmcu4);
    seq_printf(m, "g_o_vtsabb :%d\n" , g_o_vtsabb);
	return 0;
}

static int tscpu_talking_flag_read(struct seq_file *m, void *v)
{
  	seq_printf(m, "%d\n", talking_flag);
	return 0;
}

static ssize_t tscpu_talking_flag_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int lv_talking_flag;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &lv_talking_flag) == 1)
	{
		talking_flag = lv_talking_flag;
		tscpu_dprintk("tscpu_talking_flag_write talking_flag=%d\n", talking_flag);
		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_talking_flag_write bad argument\n");
	}
	return -EINVAL;
}

static int tscpu_set_temperature_read(struct seq_file *m, void *v)
{
  	seq_printf(m, "%d\n", temperature_switch);
	return 0;
}

static ssize_t tscpu_set_temperature_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int lv_tempe_switch;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	tscpu_dprintk("tscpu_set_temperature_write\n");

	if (sscanf(desc, "%d", &lv_tempe_switch) == 1)
	{
		temperature_switch = lv_tempe_switch;

		tscpu_config_all_tc_hw_protect(temperature_switch, tc_mid_trip);

		tscpu_dprintk("tscpu_set_temperature_write temperature_switch=%d\n", temperature_switch);
		return count;
	}
	else
	{
		tscpu_printk("tscpu_set_temperature_write bad argument\n");
	}
	return -EINVAL;
}

static int tscpu_read_log(struct seq_file *m, void *v)
{
	seq_printf(m, "[ tscpu_read_log] log = %d\n", mtktscpu_debug_log);
	return 0;
}

static int tscpu_read_cal(struct seq_file *m, void *v)
{
	return 0;
}

static int tscpu_read(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m,
	            "[tscpu_read]%d\ntrip_0=%d %d %s\ntrip_1=%d %d %s\ntrip_2=%d %d %s\ntrip_3=%d %d %s\ntrip_4=%d %d %s\ntrip_5=%d %d %s\ntrip_6=%d %d %s\ntrip_7=%d %d %s\ntrip_8=%d %d %s\ntrip_9=%d %d %s\ninterval=%d\n",
                num_trip,
				trip_temp[0],g_THERMAL_TRIP[0],g_bind0,
				trip_temp[1],g_THERMAL_TRIP[1],g_bind1,
				trip_temp[2],g_THERMAL_TRIP[2],g_bind2,
				trip_temp[3],g_THERMAL_TRIP[3],g_bind3,
				trip_temp[4],g_THERMAL_TRIP[4],g_bind4,
				trip_temp[5],g_THERMAL_TRIP[5],g_bind5,
				trip_temp[6],g_THERMAL_TRIP[6],g_bind6,
				trip_temp[7],g_THERMAL_TRIP[7],g_bind7,
				trip_temp[8],g_THERMAL_TRIP[8],g_bind8,
				trip_temp[9],g_THERMAL_TRIP[9],g_bind9,
				interval);

    for (i=0; i < Num_of_GPU_OPP; i++)
        seq_printf(m, "g %d %d %d\n", i, mtk_gpu_power[i].gpufreq_khz, mtk_gpu_power[i].gpufreq_power);

    for (i=0; i < tscpu_num_opp; i++)
        seq_printf(m, "c %d %d %d %d\n", i, mtk_cpu_power[i].cpufreq_khz, mtk_cpu_power[i].cpufreq_ncpu, mtk_cpu_power[i].cpufreq_power);

    for (i=0; i < CPU_COOLER_NUM; i++)
        seq_printf(m, "d %d %d\n", i, tscpu_cpu_dmips[i]);

	return 0;
}

static ssize_t tscpu_write_log(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int log_switch;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &log_switch) == 1)
	
	{
		mtktscpu_debug_log = log_switch;

		return count;
	}
	else
	{
		tscpu_printk("mtktscpu_write_log bad argument\n");
	}
	return -EINVAL;
}

#if CPT_ADAPTIVE_AP_COOLER
static int tscpu_read_atm(struct seq_file *m, void *v)
{

	seq_printf(m, "[tscpu_read_atm] ver = %d\n", mtktscpu_atm);
	seq_printf(m, "tt_ratio_high_rise = %d\n", tt_ratio_high_rise);
	seq_printf(m, "tt_ratio_high_fall = %d\n", tt_ratio_high_fall);
	seq_printf(m, "tt_ratio_low_rise = %d\n", tt_ratio_low_rise);
	seq_printf(m, "tt_ratio_low_fall = %d\n", tt_ratio_low_fall);
	seq_printf(m, "tp_ratio_high_rise = %d\n", tp_ratio_high_rise);
	seq_printf(m, "tp_ratio_high_fall = %d\n", tp_ratio_high_fall);
	seq_printf(m, "tp_ratio_low_rise = %d\n", tp_ratio_low_rise);
	seq_printf(m, "tp_ratio_low_fall = %d\n", tp_ratio_low_fall);

	return 0;
}

static ssize_t tscpu_write_atm(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	int atm_ver;
	int tmp_tt_ratio_high_rise;
	int tmp_tt_ratio_high_fall;
	int tmp_tt_ratio_low_rise;
	int tmp_tt_ratio_low_fall;
	int tmp_tp_ratio_high_rise;
	int tmp_tp_ratio_high_fall;
	int tmp_tp_ratio_low_rise;
	int tmp_tp_ratio_low_fall;
	int len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d %d %d %d %d %d %d ",
		&atm_ver, &tmp_tt_ratio_high_rise, &tmp_tt_ratio_high_fall, &tmp_tt_ratio_low_rise, &tmp_tt_ratio_low_fall,
		&tmp_tp_ratio_high_rise, &tmp_tp_ratio_high_fall, &tmp_tp_ratio_low_rise, &tmp_tp_ratio_low_fall) == 9)
	{
		mtktscpu_atm = atm_ver;
		tt_ratio_high_rise = tmp_tt_ratio_high_rise;
		tt_ratio_high_fall = tmp_tt_ratio_high_fall;
		tt_ratio_low_rise = tmp_tt_ratio_low_rise;
		tt_ratio_low_fall = tmp_tt_ratio_low_fall;
		tp_ratio_high_rise = tmp_tp_ratio_high_rise;
		tp_ratio_high_fall = tmp_tp_ratio_high_fall;
		tp_ratio_low_rise = tmp_tp_ratio_low_rise;
		tp_ratio_low_fall = tmp_tp_ratio_low_fall;
		return count;
	}
	else
	{
		tscpu_printk("mtktscpu_write_atm bad argument\n");
	}
	return -EINVAL;
}

static int tscpu_read_dtm_setting(struct seq_file *m, void *v)
{
	int i;

    for (i = 0; i < MAX_CPT_ADAPTIVE_COOLERS; i++)
    {
        seq_printf(m, "%s%02d\n", adaptive_cooler_name, i);
    	seq_printf(m, " first_step = %d\n", FIRST_STEP_TOTAL_POWER_BUDGETS[i]);
    	seq_printf(m, " theta rise = %d\n", PACKAGE_THETA_JA_RISES[i]);
    	seq_printf(m, " theta fall = %d\n", PACKAGE_THETA_JA_FALLS[i]);
    	seq_printf(m, " min_budget_change = %d\n", MINIMUM_BUDGET_CHANGES[i]);
    	seq_printf(m, " m cpu = %d\n", MINIMUM_CPU_POWERS[i]);
    	seq_printf(m, " M cpu = %d\n", MAXIMUM_CPU_POWERS[i]);
    	seq_printf(m, " m gpu = %d\n", MINIMUM_GPU_POWERS[i]);
    	seq_printf(m, " M gpu = %d\n", MAXIMUM_GPU_POWERS[i]);
    }
	return 0;
}

static ssize_t tscpu_write_dtm_setting(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	
	
	int len = 0;
	int i_id = -1, i_first_step = -1, i_theta_r = -1, i_theta_f = -1, i_budget_change = -1, i_min_cpu_pwr = -1, i_max_cpu_pwr = -1, i_min_gpu_pwr = -1, i_max_gpu_pwr = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (9 <= sscanf(desc, "%d %d %d %d %d %d %d %d %d", &i_id, &i_first_step, &i_theta_r, &i_theta_f, &i_budget_change, &i_min_cpu_pwr, &i_max_cpu_pwr, &i_min_gpu_pwr, &i_max_gpu_pwr))
	{
        tscpu_printk("tscpu_write_dtm_setting input %d %d %d %d %d %d %d %d %d\n", i_id, i_first_step, i_theta_r, i_theta_f, i_budget_change, i_min_cpu_pwr, i_max_cpu_pwr, i_min_gpu_pwr, i_max_gpu_pwr);

        if (i_id >= 0 && i_id < MAX_CPT_ADAPTIVE_COOLERS)
        {
            if (i_first_step>0) FIRST_STEP_TOTAL_POWER_BUDGETS[i_id] = i_first_step;
            if (i_theta_r>0) PACKAGE_THETA_JA_RISES[i_id] = i_theta_r;
            if (i_theta_f>0) PACKAGE_THETA_JA_FALLS[i_id] = i_theta_f;
            if (i_budget_change>0) MINIMUM_BUDGET_CHANGES[i_id] = i_budget_change;
            if (i_min_cpu_pwr>0) MINIMUM_CPU_POWERS[i_id] = i_min_cpu_pwr;
            if (i_max_cpu_pwr>0) MAXIMUM_CPU_POWERS[i_id] = i_max_cpu_pwr;
            if (i_min_gpu_pwr>0) MINIMUM_GPU_POWERS[i_id] = i_min_gpu_pwr;
            if (i_max_gpu_pwr>0) MAXIMUM_GPU_POWERS[i_id] = i_max_gpu_pwr;

            active_adp_cooler = i_id;

            tscpu_printk("tscpu_write_dtm_setting applied %d %d %d %d %d %d %d %d %d\n", i_id, FIRST_STEP_TOTAL_POWER_BUDGETS[i_id], PACKAGE_THETA_JA_RISES[i_id], PACKAGE_THETA_JA_FALLS[i_id], MINIMUM_BUDGET_CHANGES[i_id], MINIMUM_CPU_POWERS[i_id], MAXIMUM_CPU_POWERS[i_id], MINIMUM_GPU_POWERS[i_id], MAXIMUM_GPU_POWERS[i_id]);
        }
        else
        {
            tscpu_dprintk("tscpu_write_dtm_setting out of range\n");
        }

		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_write_dtm_setting bad argument\n");
	}
	return -EINVAL;
}

static int tscpu_read_gpu_threshold(struct seq_file *m, void *v)
{
    seq_printf(m, "H %d L %d\n", GPU_L_H_TRIP, GPU_L_L_TRIP);

	return 0;
}

static ssize_t tscpu_write_gpu_threshold(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;

	int gpu_h = -1, gpu_l = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (2 <= sscanf(desc, "%d %d", &gpu_h, &gpu_l))
	{
        tscpu_printk("tscpu_write_gpu_threshold input %d %d\n", gpu_h, gpu_l);

        if ((gpu_h > 0) && (gpu_l > 0) && (gpu_h > gpu_l))
        {
            GPU_L_H_TRIP = gpu_h;
            GPU_L_L_TRIP = gpu_l;

            tscpu_printk("tscpu_write_gpu_threshold applied %d %d\n", GPU_L_H_TRIP, GPU_L_L_TRIP);
        }
        else
        {
            tscpu_dprintk("tscpu_write_gpu_threshold out of range\n");
        }

		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_write_gpu_threshold bad argument\n");
	}
	return -EINVAL;
}

#if THERMAL_HEADROOM
static int tscpu_read_thp(struct seq_file *m, void *v)
{
    seq_printf(m, "Tpcb pt coef %d\n", p_Tpcb_correlation);
    seq_printf(m, "Tpcb threshold %d\n", Tpcb_trip_point);
    seq_printf(m, "Tj pt coef %d\n", thp_p_tj_correlation);
    seq_printf(m, "thp tj threshold %d\n", thp_threshold_tj);
	return 0;
}

static ssize_t tscpu_write_thp(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int tpcb_coef = -1, tpcb_trip = -1, thp_coef = -1, thp_threshold = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (4 <= sscanf(desc, "%d %d %d %d", &tpcb_coef, &tpcb_trip, &thp_coef, &thp_threshold))
	{
        tscpu_printk("%s input %d %d\n", __func__, tpcb_coef, tpcb_trip, thp_coef, thp_threshold);

        p_Tpcb_correlation = tpcb_coef;
        Tpcb_trip_point = tpcb_trip;
        thp_p_tj_correlation = thp_coef;
        thp_threshold_tj = thp_threshold;

		return count;
	}
	else
	{
		tscpu_dprintk("%s bad argument\n", __func__);
	}
	return -EINVAL;
}
#endif

#if CONTINUOUS_TM
static int tscpu_read_ctm(struct seq_file *m, void *v)
{
    seq_printf(m, "ctm %d\n", ctm_on);
    seq_printf(m, "Target Tj 0 %d\n", MAX_TARGET_TJ);
    seq_printf(m, "Target Tj 2 %d\n", STEADY_TARGET_TJ);
    seq_printf(m, "Tpcb 1 %d\n", TRIP_TPCB);
    seq_printf(m, "Tpcb 2 %d\n", STEADY_TARGET_TPCB);
    seq_printf(m, "Exit Tj 0 %d\n", MAX_EXIT_TJ);
    seq_printf(m, "Exit Tj 2 %d\n", STEADY_EXIT_TJ);
    seq_printf(m, "Enter_a %d\n", COEF_AE);
    seq_printf(m, "Enter_b %d\n", COEF_BE);
    seq_printf(m, "Exit_a %d\n", COEF_AX);
    seq_printf(m, "Exit_b %d\n", COEF_BX);
	return 0;
}

static ssize_t tscpu_write_ctm(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[256];
	int len = 0;
	int t_ctm_on = -1, t_MAX_TARGET_TJ = -1, t_STEADY_TARGET_TJ = -1, t_TRIP_TPCB = -1, t_STEADY_TARGET_TPCB = -1, t_MAX_EXIT_TJ = -1, t_STEADY_EXIT_TJ = -1,
	    t_COEF_AE = -1, t_COEF_BE = -1, t_COEF_AX = -1, t_COEF_BX = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (11 <= sscanf(desc, "%d %d %d %d %d %d %d %d %d %d %d", &t_ctm_on
	                , &t_MAX_TARGET_TJ, &t_STEADY_TARGET_TJ, &t_TRIP_TPCB, &t_STEADY_TARGET_TPCB
	                , &t_MAX_EXIT_TJ, &t_STEADY_EXIT_TJ, &t_COEF_AE, &t_COEF_BE, &t_COEF_AX, &t_COEF_BX))
	{
        tscpu_printk("%s input %d %d %d %d %d %d %d %d %d %d %d\n", __func__, t_ctm_on,
                    t_MAX_TARGET_TJ, t_STEADY_TARGET_TJ, t_TRIP_TPCB, t_STEADY_TARGET_TPCB,
                    t_MAX_EXIT_TJ, t_STEADY_EXIT_TJ, t_COEF_AE, t_COEF_BE, t_COEF_AX, t_COEF_BX);

        
        ctm_on = t_ctm_on; 

        MAX_TARGET_TJ = t_MAX_TARGET_TJ;
        STEADY_TARGET_TJ = t_STEADY_TARGET_TJ;
        TRIP_TPCB = t_TRIP_TPCB;
        STEADY_TARGET_TPCB = t_STEADY_TARGET_TPCB;
        MAX_EXIT_TJ = t_MAX_EXIT_TJ;
        STEADY_EXIT_TJ = t_STEADY_EXIT_TJ;

        COEF_AE = t_COEF_AE;
        COEF_BE = t_COEF_BE;
        COEF_AX = t_COEF_AX;
        COEF_BX = t_COEF_BX;

		return count;
	}
	else
	{
		tscpu_dprintk("%s bad argument\n", __func__);
	}
	return -EINVAL;
}
#endif
#endif

#if MTKTSCPU_FAST_POLLING
static int tscpu_read_fastpoll(struct seq_file *m, void *v)
{
    seq_printf(m, "trip %d factor %d\n", fast_polling_trip_temp, fast_polling_factor);
	return 0;
}

static ssize_t tscpu_write_fastpoll(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int trip = -1, factor = -1;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (2 <= sscanf(desc, "%d %d", &trip, &factor))
	{
        tscpu_printk("tscpu_write_fastpoll input %d %d\n", trip, factor);

        if ((trip >= 0) && (factor > 0))
        {
            fast_polling_trip_temp = trip;
            fast_polling_factor = factor;

            tscpu_printk("tscpu_write_fastpoll applied %d %d\n", fast_polling_trip_temp, fast_polling_factor);
        }
        else
        {
            tscpu_dprintk("tscpu_write_fastpoll out of range\n");
        }

		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_write_fastpoll bad argument\n");
	}
	return -EINVAL;
}
#endif

int tscpu_register_thermal(void);
void tscpu_unregister_thermal(void);

static ssize_t tscpu_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len=0,time_msec=0;
	int trip[10]={0};
	int t_type[10]={0};
	int i;
	char bind0[20],bind1[20],bind2[20],bind3[20],bind4[20];
	char bind5[20],bind6[20],bind7[20],bind8[20],bind9[20];
	char desc[512];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
	{
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d %s %d %d",
				&num_trip, &trip[0],&t_type[0],bind0, &trip[1],&t_type[1],bind1,
				&trip[2],&t_type[2],bind2, &trip[3],&t_type[3],bind3,
				&trip[4],&t_type[4],bind4, &trip[5],&t_type[5],bind5,
				&trip[6],&t_type[6],bind6, &trip[7],&t_type[7],bind7,
				&trip[8],&t_type[8],bind8, &trip[9],&t_type[9],bind9,
				&time_msec, &MA_len_temp) == 33)
	{

		tscpu_dprintk("tscpu_write tscpu_unregister_thermal MA_len_temp=%d\n",MA_len_temp);

		tscpu_unregister_thermal();

		for(i=0; i<num_trip; i++)
			g_THERMAL_TRIP[i] = t_type[i];

		g_bind0[0]=g_bind1[0]=g_bind2[0]=g_bind3[0]=g_bind4[0]=g_bind5[0]=g_bind6[0]=g_bind7[0]=g_bind8[0]=g_bind9[0]='\0';

		for(i=0; i<20; i++)
		{
			g_bind0[i]=bind0[i];
			g_bind1[i]=bind1[i];
			g_bind2[i]=bind2[i];
			g_bind3[i]=bind3[i];
			g_bind4[i]=bind4[i];
			g_bind5[i]=bind5[i];
			g_bind6[i]=bind6[i];
			g_bind7[i]=bind7[i];
			g_bind8[i]=bind8[i];
			g_bind9[i]=bind9[i];
		}

#if CPT_ADAPTIVE_AP_COOLER
        
        for (i = 0; i < MAX_CPT_ADAPTIVE_COOLERS; i++)
        {
            TARGET_TJS[i] = 117000;
        }

		if(!strncmp(bind0, adaptive_cooler_name, 13))
		{
		    TARGET_TJS[(bind0[13] - '0')] = trip[0];
		}

		if(!strncmp(bind1, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind1[13] - '0')] = trip[1];
		}

		if(!strncmp(bind2, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind2[13] - '0')] = trip[2];
		}

		if(!strncmp(bind3, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind3[13] - '0')] = trip[3];
		}

		if(!strncmp(bind4, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind4[13] - '0')] = trip[4];
		}

		if(!strncmp(bind5, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind5[13] - '0')] = trip[5];
		}

		if(!strncmp(bind6, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind6[13] - '0')] = trip[6];
		}

		if(!strncmp(bind7, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind7[13] - '0')] = trip[7];
		}

		if(!strncmp(bind8, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind8[13] - '0')] = trip[8];
		}

		if(!strncmp(bind9, adaptive_cooler_name, 13))
		{
			TARGET_TJS[(bind9[13] - '0')] = trip[9];
		}

		tscpu_dprintk("tscpu_write TTJ0=%d, TTJ1=%d, TTJ2=%d\n", TARGET_TJS[0], TARGET_TJS[1], TARGET_TJS[2]);
#endif

		tscpu_dprintk("tscpu_write g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,g_THERMAL_TRIP_4=%d,\
				g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
				g_THERMAL_TRIP[0],g_THERMAL_TRIP[1],g_THERMAL_TRIP[2],g_THERMAL_TRIP[3],g_THERMAL_TRIP[4],
				g_THERMAL_TRIP[5],g_THERMAL_TRIP[6],g_THERMAL_TRIP[7],g_THERMAL_TRIP[8],g_THERMAL_TRIP[9]);
		tscpu_dprintk("tscpu_write cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\
				cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s\n",
				g_bind0,g_bind1,g_bind2,g_bind3,g_bind4,g_bind5,g_bind6,g_bind7,g_bind8,g_bind9);

		for(i=0; i<num_trip; i++)
		{
			trip_temp[i]=trip[i];
		}

		interval=time_msec;

		tscpu_dprintk("tscpu_write trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,trip_4_temp=%d,\
				trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,time_ms=%d, num_trip=%d\n",
				trip_temp[0],trip_temp[1],trip_temp[2],trip_temp[3],trip_temp[4],
				trip_temp[5],trip_temp[6],trip_temp[7],trip_temp[8],trip_temp[9],interval, num_trip);
		tscpu_dprintk("tscpu_write tscpu_register_thermal\n");
		tscpu_register_thermal();

		proc_write_flag=1;

		return count;
	}
	else
	{
		tscpu_dprintk("tscpu_write bad argument\n");
	}

	return -EINVAL;
}

int tscpu_register_DVFS_hotplug_cooler(void)
{
	int i;

	tscpu_dprintk("tscpu_register_DVFS_hotplug_cooler\n");
	for(i=0; i<Num_of_OPP; i++)
	{
		cl_dev[i] = mtk_thermal_cooling_device_register(&cooler_name[i*20], NULL,
					 &mtktscpu_cooling_F0x2_ops);
	}
	cl_dev_sysrst = mtk_thermal_cooling_device_register("mtktscpu-sysrst", NULL,
					&mtktscpu_cooling_sysrst_ops);
#if CPT_ADAPTIVE_AP_COOLER
    cl_dev_adp_cpu[0] = mtk_thermal_cooling_device_register("cpu_adaptive_0", NULL,
					&mtktscpu_cooler_adp_cpu_ops);

    cl_dev_adp_cpu[1] = mtk_thermal_cooling_device_register("cpu_adaptive_1", NULL,
					&mtktscpu_cooler_adp_cpu_ops);

    cl_dev_adp_cpu[2] = mtk_thermal_cooling_device_register("cpu_adaptive_2", NULL,
					&mtktscpu_cooler_adp_cpu_ops);
#endif
	return 0;
}

int tscpu_register_thermal(void)
{
	tscpu_dprintk("tscpu_register_thermal\n");

	
    if (NULL == thz_dev) {
	    thz_dev = mtk_thermal_zone_device_register("mtktscpu", num_trip, NULL,
                                                   &mtktscpu_dev_ops, 0, 0, 0, interval);
    }

	return 0;
}

void tscpu_unregister_DVFS_hotplug_cooler(void)
{
	int i;
	for(i=0; i<Num_of_OPP; i++)
	{
		if(cl_dev[i])
		{
			mtk_thermal_cooling_device_unregister(cl_dev[i]);
			cl_dev[i] = NULL;
		}
	}
	if(cl_dev_sysrst) {
		mtk_thermal_cooling_device_unregister(cl_dev_sysrst);
		cl_dev_sysrst = NULL;
	}

#if CPT_ADAPTIVE_AP_COOLER
    if(cl_dev_adp_cpu[0]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[0]);
		cl_dev_adp_cpu[0] = NULL;
	}

	if(cl_dev_adp_cpu[1]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[1]);
		cl_dev_adp_cpu[1] = NULL;
	}

	if(cl_dev_adp_cpu[2]) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_cpu[2]);
		cl_dev_adp_cpu[2] = NULL;
	}
#endif
}

void tscpu_unregister_thermal(void)
{
	tscpu_dprintk("tscpu_unregister_thermal\n");
	if(thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static void thermal_pause_all_periodoc_temp_sensing(void)
{
	int i=0;
	unsigned long flags;
    int temp;

    

	mt_ptp_lock(&flags);

    
    for(i=0;i<ROME_BANK_NUM;i++){

		mtktscpu_switch_bank(i);
		temp = DRV_Reg32(TEMPMSRCTL1);
		DRV_WriteReg32(TEMPMSRCTL1, (temp | 0x10E));
    }

	mt_ptp_unlock(&flags);
}

static void thermal_release_all_periodoc_temp_sensing(void)
{
	int i=0;
	unsigned long flags;
    int temp;

    

	mt_ptp_lock(&flags);

    
    for(i=0;i<ROME_BANK_NUM;i++){

		mtktscpu_switch_bank(i);
		temp = DRV_Reg32(TEMPMSRCTL1);
		DRV_WriteReg32(TEMPMSRCTL1, ( (temp & (~0x10E)) ));
    }

	mt_ptp_unlock(&flags);
}

static void thermal_disable_all_periodoc_temp_sensing(void)
{
	int i=0;
	unsigned long flags;

   

	mt_ptp_lock(&flags);

    
    for(i=0;i<ROME_BANK_NUM;i++){

		mtktscpu_switch_bank(i);
        
	    THERMAL_WRAP_WR32(0x00000000, TEMPMONCTL0);
    }

	mt_ptp_unlock(&flags);
}

static void tscpu_clear_all_temp(void)
{
#ifdef CONFIG_ARM64
	CPU_TS1_T=0;
	CPU_TS2_T=0;
	LTE_TS4_T=0;
    TS_ABB_T=0;
	GPU_TS3_T=0;
    SOC_TS4_T=0;
	SOC_TS1_T=0;
    SOC_TS2_T=0;
    SOC_TS3_T=0;
#else
	CA7_TS1_T=0;
	CA7_TS2_T=0;
	CA15_TS1_T=0;
	CA15_TS3_T=0;
	GPU_TS3_T=0;
	GPU_TS4_T=0;
	CORE_TS2_T=0;
	CORE_TS4_T=0;
	CORE_TSABB_T=0;
#endif
}

static int tscpu_thermal_suspend(struct platform_device *dev, pm_message_t state)
{
    int cnt=0;
    int temp=0;
	tscpu_printk("tscpu_thermal_suspend\n");

#if THERMAL_PERFORMANCE_PROFILE
	struct timeval begin, end;
	unsigned long val;

	do_gettimeofday(&begin);
#endif

	g_tc_resume = 1;

    if(talking_flag==false)
    {
    	tscpu_dprintk("tscpu_thermal_suspend no talking\n");

		#if (CONFIG_THERMAL_AEE_RR_REC== 1)
		p_thermal_state = TSCPU_SUSPEND;
		#endif

        while(cnt < 50)
		{
			temp = (DRV_Reg32(THAHBST0) >> 16);
			if(cnt>20)
	            printk(KERN_CRIT "THAHBST0 = 0x%x,cnt=%d, %d\n", temp,cnt,__LINE__);
            if(temp == 0x0){
                
				thermal_pause_all_periodoc_temp_sensing();
				break;
            }
            udelay(2);
            cnt++;
		}

		
		thermal_disable_all_periodoc_temp_sensing();

		

		
		THERMAL_WRAP_WR32(DRV_Reg32(TS_CON1) | 0x00000030, TS_CON1); 
    }

#if THERMAL_PERFORMANCE_PROFILE
    do_gettimeofday(&end);

    
    printk("suspend time spent, sec : %lu , usec : %lu \n", (end.tv_sec - begin.tv_sec), (end.tv_usec - begin.tv_usec));
#endif
	return 0;
}

static int tscpu_thermal_resume(struct platform_device *dev)
{
	int temp=0;
	int cnt=0;
	tscpu_printk("tscpu_thermal_resume\n");
	g_tc_resume = 1;

	if(talking_flag==false)
	{
		#if (CONFIG_THERMAL_AEE_RR_REC== 1)
		p_thermal_state = TSCPU_RESUME;
		#endif
        tscpu_reset_thermal();

		temp = DRV_Reg32(TS_CON1);
		temp &=~(0x00000030);										  
		THERMAL_WRAP_WR32(temp, TS_CON1);	                          
		udelay(200);

		tscpu_fast_initial_sw_workaround();

        while(cnt < 50)
		{
			temp = (DRV_Reg32(THAHBST0) >> 16);
			if(cnt>20)
	            printk(KERN_CRIT "THAHBST0 = 0x%x,cnt=%d, %d\n", temp,cnt,__LINE__);

            if(temp == 0x0){
                
				thermal_pause_all_periodoc_temp_sensing();
				break;
            }
            udelay(2);
            cnt++;
		}
        
		thermal_disable_all_periodoc_temp_sensing();

	    thermal_initial_all_bank();

		thermal_release_all_periodoc_temp_sensing();

		tscpu_clear_all_temp();

		tscpu_config_all_tc_hw_protect(trip_temp[0], tc_mid_trip);
	}

	g_tc_resume = 2;

	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id mt_thermal_of_match[] = {
	{ .compatible = "mediatek,THERM_CTRL", },
	{},
};
#endif


static struct platform_driver mtk_thermal_driver = {
	.remove     = NULL,
	.shutdown   = NULL,
	.probe      = NULL,
	.suspend	= tscpu_thermal_suspend,
	.resume		= tscpu_thermal_resume,
	.driver     = {
		.name = THERMAL_NAME,
		#ifdef CONFIG_OF
		.of_match_table = mt_thermal_of_match,
		#endif
    },
};


#if MTK_TS_CPU_RT
static int ktp_limited = -275000;

static int ktp_thread(void *arg)
{
    int max_temp = 0;

	int bank0_T;
	int bank1_T;
	int bank2_T;
	int bank3_T;
#ifndef CONFIG_ARM64
	int curr_temp1;
	int curr_temp2;
#endif

    struct sched_param param = { .sched_priority = 98 };

    sched_setscheduler(current, SCHED_FIFO, &param);
    set_current_state(TASK_INTERRUPTIBLE);

    tscpu_printk("ktp_thread 1st run\n");

    schedule();

    for(;;)
	{
        int temp_tc_mid_trip = tc_mid_trip;
        int temp_ktp_limited = ktp_limited;

	    tscpu_printk("ktp_thread awake,tc_mid_trip=%d\n",tc_mid_trip);
	    if (kthread_should_stop()) break;

#ifdef CONFIG_ARM64
		bank0_T    = MAX(CPU_TS1_T,CPU_TS2_T);
	   	bank1_T    = LTE_TS4_T;
		bank2_T    = GPU_TS3_T;
	    bank3_T    = MAX(MAX(SOC_TS1_T,SOC_TS2_T),MAX(SOC_TS4_T,SOC_TS3_T));
	    max_temp   = MAX(MAX(bank0_T,bank1_T),MAX(bank2_T,bank3_T));
#else
		bank0_T    = MAX(CA7_TS1_T,CA7_TS2_T);
	   	bank1_T    = MAX(CA15_TS1_T,CA15_TS3_T);
		bank2_T    = MAX(GPU_TS3_T,GPU_TS4_T);
	    bank3_T    = MAX(CORE_TS2_T,CORE_TS4_T);
	    bank3_T    = MAX(bank3_T,CORE_TSABB_T);
	    curr_temp1 = MAX(bank0_T,bank1_T);
		curr_temp2 = MAX(bank2_T,bank3_T);
	    max_temp   = MAX(curr_temp1,curr_temp2);
#endif
	    tscpu_printk("ktp_thread temp=%d\n", max_temp);

	    if ((temp_tc_mid_trip > -275000) && (max_temp >= (temp_tc_mid_trip-5000))) 
        {
            
            tscpu_printk("ktp_thread overheat %d\n", max_temp);

            
            mt_cpufreq_thermal_protect(713);
            mt_gpufreq_thermal_protect(341);

            ktp_limited = temp_tc_mid_trip;

            msleep(20 * 1000);
        }
        else if ((temp_ktp_limited > -275000) && (max_temp < temp_ktp_limited))
        {
            unsigned int final_limit;
            final_limit = MIN(static_cpu_power_limit, adaptive_cpu_power_limit);
            tscpu_printk("ktp_thread unlimit cpu=%d\n", final_limit);
            mt_cpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);


            final_limit = MIN(static_gpu_power_limit, adaptive_gpu_power_limit);
            tscpu_printk("ktp_thread unlimit gpu=%d\n", final_limit);
            mt_gpufreq_thermal_protect((final_limit != 0x7FFFFFFF) ? final_limit : 0);

            ktp_limited = -275000;

            set_current_state(TASK_INTERRUPTIBLE);
            schedule();
        }
        else
        {
            tscpu_printk("ktp_thread else temp=%d, trip=%d, ltd=%d\n", max_temp, temp_tc_mid_trip, temp_ktp_limited);
            set_current_state(TASK_INTERRUPTIBLE);
            schedule();
        }
	}

	tscpu_printk("ktp_thread stopped\n");

    return 0;
}
#endif

#if THERMAL_GPIO_OUT_TOGGLE
static int tscpu_GPIO_out(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_GPIO_out, NULL);
}
static const struct file_operations mtktscpu_GPIO_out_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_GPIO_out,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_GPIO_out,
    .release = single_release,
};
#endif

static int tscpu_Tj_out(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_Tj_out, NULL);
}
static const struct file_operations mtktscpu_Tj_out_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_Tj_out,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_Tj_out,
    .release = single_release,
};

static int tscpu_open_opp(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_opp, NULL);
}
static const struct file_operations mtktscpu_opp_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_open_opp,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int tscpu_open_log(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_log, NULL);
}
static const struct file_operations mtktscpu_log_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_open_log,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_log,
    .release = single_release,
};

static int tscpu_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read, NULL);
}
static const struct file_operations mtktscpu_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write,
    .release = single_release,
};

static int tscpu_cal_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_cal, NULL);
}
static const struct file_operations mtktscpu_cal_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_cal_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int tscpu_read_temperature_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_temperature_info, NULL);
}
static const struct file_operations mtktscpu_read_temperature_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_read_temperature_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write,
    .release = single_release,
};

static int tscpu_set_temperature_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_set_temperature_read, NULL);
}
static const struct file_operations mtktscpu_set_temperature_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_set_temperature_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_set_temperature_write,
    .release = single_release,
};

static int tscpu_talking_flag_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_talking_flag_read, NULL);
}
static const struct file_operations mtktscpu_talking_flag_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_talking_flag_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_talking_flag_write,
    .release = single_release,
};

#if CPT_ADAPTIVE_AP_COOLER
static int tscpu_open_atm(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_atm, NULL);
}
static const struct file_operations mtktscpu_atm_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_open_atm,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_atm,
    .release = single_release,
};

static int tscpu_dtm_setting_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_dtm_setting, NULL);
}
static const struct file_operations mtktscpu_dtm_setting_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_dtm_setting_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_dtm_setting,
    .release = single_release,
};

static int tscpu_gpu_threshold_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_gpu_threshold, NULL);
}
static const struct file_operations mtktscpu_gpu_threshold_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_gpu_threshold_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_gpu_threshold,
    .release = single_release,
};

#if THERMAL_HEADROOM
static int tscpu_thp_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_thp, NULL);
}

static const struct file_operations mtktscpu_thp_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_thp_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_thp,
    .release = single_release,
};
#endif

#if CONTINUOUS_TM
static int tscpu_ctm_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_ctm, NULL);
}

static const struct file_operations mtktscpu_ctm_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_ctm_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_ctm,
    .release = single_release,
};
#endif
#endif

#if MTKTSCPU_FAST_POLLING
static int tscpu_fastpoll_open(struct inode *inode, struct file *file)
{
    return single_open(file, tscpu_read_fastpoll, NULL);
}
static const struct file_operations mtktscpu_fastpoll_fops = {
    .owner = THIS_MODULE,
    .open = tscpu_fastpoll_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .write = tscpu_write_fastpoll,
    .release = single_release,
};
#endif

static int mtktscpu_switch_bank(thermal_bank_name bank)
{
	tscpu_dprintk( "mtktscpu_switch_bank =bank=%d\n",bank);

	switch(bank){
        case THERMAL_BANK0:
            thermal_clrl(PTPCORESEL, 0xF);
            break;
        case THERMAL_BANK1:
        	thermal_clrl(PTPCORESEL, 0xF);
            thermal_setl(PTPCORESEL, 0x1);
            break;
        case THERMAL_BANK2:
        	thermal_clrl(PTPCORESEL, 0xF);
            thermal_setl(PTPCORESEL, 0x2);
            break;
        case THERMAL_BANK3:
        	thermal_clrl(PTPCORESEL, 0xF);
            thermal_setl(PTPCORESEL, 0x3);
            break;
        default:
            thermal_clrl(PTPCORESEL, 0xF);
            break;
    }
    return 0;
}

static void thermal_initial_all_bank(void)
{
	int i=0;
    unsigned long flags;
    UINT32 temp = 0;
	

	

	mt_ptp_lock(&flags);

	
	temp = DRV_Reg32(AUXADC_CON0_V);
	temp &= 0xFFFFF7FF;
	THERMAL_WRAP_WR32(temp, AUXADC_CON0_V);        
	THERMAL_WRAP_WR32(0x800, AUXADC_CON1_CLR_V);    

    
    for(i=0;i<ROME_BANK_NUM;i++){
        mtktscpu_switch_bank(i);
        thermal_reset_and_initial();
        thermal_config_TS_in_banks(i);
    }

	
	THERMAL_WRAP_WR32(0x800, AUXADC_CON1_SET_V);

    
    for(i=0;i<ROME_BANK_NUM;i++){
        mtktscpu_switch_bank(i);
        thermal_enable_all_periodoc_sensing_point(i);
    }
	mt_ptp_unlock(&flags);
}

static void tscpu_config_all_tc_hw_protect( int temperature, int temperature2)
{
	int i=0,wd_api_ret;
	unsigned long flags;
	struct wd_api *wd_api;

    tscpu_dprintk( "tscpu_config_all_tc_hw_protect,temperature=%d,temperature2=%d,\n",temperature,temperature2);

#if THERMAL_PERFORMANCE_PROFILE
	struct timeval begin, end;
	unsigned long val;

	do_gettimeofday(&begin);
#endif
	
	wd_api_ret = get_wd_api(&wd_api);
    if(wd_api_ret >=0 ){
		wd_api->wd_thermal_direct_mode_config(WD_REQ_DIS, WD_REQ_RST_MODE);
	}else{
		tscpu_printk("%d FAILED TO GET WD API\n",__LINE__);
        BUG();
    }
#if THERMAL_PERFORMANCE_PROFILE
    do_gettimeofday(&end);

    
    printk("resume time spent, sec : %lu , usec : %lu \n", (end.tv_sec - begin.tv_sec), (end.tv_usec - begin.tv_usec));
#endif
	mt_ptp_lock(&flags);

    
    for(i=0;i<ROME_BANK_NUM;i++){

		mtktscpu_switch_bank(i);

	    set_tc_trigger_hw_protect(temperature, temperature2); 
    }

	mt_ptp_unlock(&flags);

    wd_api->wd_thermal_direct_mode_config(WD_REQ_EN, WD_REQ_RST_MODE);
}

static void tscpu_reset_thermal(void)
{
	int temp = 0;
	
	temp = DRV_Reg32(PERI_GLOBALCON_RST0); 
	temp |= 0x00010000;
	THERMAL_WRAP_WR32(temp, PERI_GLOBALCON_RST0);

	temp = DRV_Reg32(PERI_GLOBALCON_RST0);
	temp &= 0xFFFEFFFF;
	THERMAL_WRAP_WR32(temp, PERI_GLOBALCON_RST0);
}

static void tscpu_fast_initial_sw_workaround(void)
{
	int i=0;
    unsigned long flags;
	

	

	mt_ptp_lock(&flags);

    
    for(i=0;i<ROME_BANK_NUM;i++){
	    mtktscpu_switch_bank(i);
	    thermal_fast_init();
    }

	mt_ptp_unlock(&flags);
}

#if THERMAL_DRV_UPDATE_TEMP_DIRECT_TO_MET
int tscpu_get_cpu_temp_met(MTK_THERMAL_SENSOR_CPU_ID_MET id)
{
	unsigned long flags;

    if (id < 0 || id >= MTK_THERMAL_SENSOR_CPU_COUNT)
        return -127000;
    else if (ATM_CPU_LIMIT == id)
    {
        return (adaptive_cpu_power_limit != 0x7FFFFFFF) ? adaptive_cpu_power_limit : 0;
    }
    else if (ATM_GPU_LIMIT == id)
    {
        return (adaptive_gpu_power_limit != 0x7FFFFFFF) ? adaptive_gpu_power_limit : 0;
    }
    else
    {
        tscpu_met_lock(&flags);
        if (a_tscpu_all_temp[id] == 0)
        {
            tscpu_met_unlock(&flags);
            return -127000;
        }
        else
        {
            int ret = a_tscpu_all_temp[id];
            tscpu_met_unlock(&flags);
            return ret;
        }
    }
}
EXPORT_SYMBOL(tscpu_get_cpu_temp_met);
#endif

#if THERMAL_DRV_FAST_POLL_HRTIMER
static enum hrtimer_restart tscpu_update_tempinfo(struct hrtimer *timer)
{
	ktime_t ktime;
	unsigned long flags;
	

	if(g_tc_resume==0){
		read_all_bank_temperature();
    }
	else if(g_tc_resume==2){

		
		g_tc_resume=0;
    }

	#if THERMAL_DRV_UPDATE_TEMP_DIRECT_TO_MET

	tscpu_met_lock(&flags);
#ifdef CONFIG_ARM64
	a_tscpu_all_temp[0]=MAX(CPU_TS1_T, SOC_TS1_T);
	a_tscpu_all_temp[1]=MAX(CPU_TS2_T, SOC_TS2_T);
	a_tscpu_all_temp[2]=MAX(GPU_TS3_T, SOC_TS3_T);
	a_tscpu_all_temp[3]=MAX(LTE_TS4_T, SOC_TS4_T);
	a_tscpu_all_temp[4]=TS_ABB_T;
#else
	a_tscpu_all_temp[0]=MAX(CA7_TS1_T, CA15_TS1_T);
	a_tscpu_all_temp[1]=MAX(CA7_TS2_T, CORE_TS2_T);
	a_tscpu_all_temp[2]=MAX(GPU_TS3_T, CA15_TS3_T);
	a_tscpu_all_temp[3]=MAX(GPU_TS4_T, CORE_TS4_T);
	a_tscpu_all_temp[4]=CORE_TSABB_T;
#endif
	tscpu_met_unlock(&flags);

	if(NULL != g_pThermalSampler)
	{
	   g_pThermalSampler();
	}
	#endif

	ktime = ktime_set(0, g_hr_timer_polling_delay);	
	hrtimer_forward_now(timer, ktime);

#if 0
    tscpu_printk("\n tscpu_update_tempinfo, T=%d,%d,%d,%d,%d,%d,%d,%d,%d\n", CA7_TS1_T, CA15_TS1_T, CA7_TS2_T,
    		CORE_TS2_T, CA15_TS3_T, GPU_TS3_T, GPU_TS4_T, CORE_TS4_T, CORE_TSABB_T);

	printk("\n\n");
	printk("tscpu_update_tempinfo g_max_temp = %d deg\n",g_max_temp);
	printk("Bank 0 : CA7  (TS1 = %d,TS2 = %d)\n",CA7_TS1_T,CA7_TS2_T);
	printk("Bank 1 : CA15 (TS1 = %d,TS3 = %d)\n",CA15_TS1_T,CA15_TS3_T);
	printk("Bank 2 : GPU  (TS3 = %d,TS4 = %d)\n",GPU_TS3_T,GPU_TS4_T);
	printk("Bank 3 : CORE (TS2 = %d,TS4 = %d,TSABB = %d)\n",CORE_TS2_T,CORE_TS4_T,CORE_TSABB_T);

#endif

	return HRTIMER_RESTART;
}

static void tscpu_update_temperature_timer_init(void)
{
	ktime_t ktime;

	tscpu_printk("tscpu_update_temperature_timer_init \n");

    ktime = ktime_set(0, g_hr_timer_polling_delay);	

	hrtimer_init(&ts_tempinfo_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	ts_tempinfo_hrtimer.function = tscpu_update_tempinfo;
	hrtimer_start(&ts_tempinfo_hrtimer,ktime,HRTIMER_MODE_REL);
}
#endif

void tscpu_cancel_thermal_timer(void)
{
	
	
	hrtimer_cancel(&ts_tempinfo_hrtimer);

	
	if (thz_dev)
	    cancel_delayed_work(&(thz_dev->poll_queue));
}
void tscpu_start_thermal_timer(void)
{
	ktime_t ktime;

	
    ktime = ktime_set(0, g_hr_timer_polling_delay);	
    hrtimer_start(&ts_tempinfo_hrtimer,ktime,HRTIMER_MODE_REL);

    
    if (thz_dev != NULL && interval != 0)
	    mod_delayed_work(system_freezable_wq, &(thz_dev->poll_queue), round_jiffies(msecs_to_jiffies(60))); 
}

int thermal_fast_init(void)
{
	UINT32 temp = 0;
	UINT32 cunt = 0;
    

	
	temp = 0xDA1;
    DRV_WriteReg32(PTPSPARE2, (0x00001000 + temp));

    DRV_WriteReg32(TEMPMONCTL1, 1);                
    DRV_WriteReg32(TEMPMONCTL2, 1);                
    DRV_WriteReg32(TEMPAHBPOLL, 1);                

    DRV_WriteReg32(TEMPAHBTO,    0x000000FF);               
    DRV_WriteReg32(TEMPMONIDET0, 0x00000000);               
    DRV_WriteReg32(TEMPMONIDET1, 0x00000000);               

    DRV_WriteReg32(TEMPMSRCTL0, 0x0000000);                 

    DRV_WriteReg32(TEMPADCPNP0, 0x1);                       
    DRV_WriteReg32(TEMPADCPNP1, 0x2);                       
    DRV_WriteReg32(TEMPADCPNP2, 0x3);
    DRV_WriteReg32(TEMPADCPNP3, 0x4);

    DRV_WriteReg32(TEMPPNPMUXADDR, (UINT32)PTPSPARE0_P);    
    DRV_WriteReg32(TEMPADCMUXADDR, (UINT32) PTPSPARE0_P);    
    DRV_WriteReg32(TEMPADCENADDR,  (UINT32) PTPSPARE1_P);     
    DRV_WriteReg32(TEMPADCVALIDADDR,(UINT32) PTPSPARE2_P);  
    DRV_WriteReg32(TEMPADCVOLTADDR, (UINT32) PTPSPARE2_P);   

    DRV_WriteReg32(TEMPRDCTRL, 0x0);                        
    DRV_WriteReg32(TEMPADCVALIDMASK, 0x0000002C);           
    DRV_WriteReg32(TEMPADCVOLTAGESHIFT, 0x0);               
    DRV_WriteReg32(TEMPADCWRITECTRL, 0x3);                  

	DRV_WriteReg32(TEMPMONINT, 0x00000000);                 

    DRV_WriteReg32(TEMPMONCTL0, 0x0000000F);                

	cunt=0;
	temp = DRV_Reg32(TEMPMSR0)& 0x0fff;
    while(temp!=0xDA1 && cunt <20){
		cunt++;
    	
        temp = DRV_Reg32(TEMPMSR0)& 0x0fff;
	}

    cunt=0;
	temp = DRV_Reg32(TEMPMSR1)& 0x0fff;
    while(temp!=0xDA1 &&  cunt <20){
		cunt++;
    	
        temp = DRV_Reg32(TEMPMSR1)& 0x0fff;
	}

	cunt=0;
	temp = DRV_Reg32(TEMPMSR2)& 0x0fff;
    while(temp!=0xDA1 &&  cunt <20){
		cunt++;
    	
        temp = DRV_Reg32(TEMPMSR2)& 0x0fff;
	}

	cunt=0;
	temp = DRV_Reg32(TEMPMSR3)& 0x0fff;
    while(temp!=0xDA1 &&  cunt <20){
		cunt++;
    	
        temp = DRV_Reg32(TEMPMSR3)& 0x0fff;
	}
#if 0
	
	tscpu_printk("===========================================\n");
	tscpu_printk("thermal_interrupt_trigger_test: high temp=0x14\n");
	tscpu_printk("===========================================\n");
    temp = 0xD78;                                            
    DRV_WriteReg32(TEMPSPARE2, (0x00001000 + temp));        



    count = 20;
    while (count--)
    {
    	printk("........");
        udelay(100);
    }

    count = 10;
    temp = DRV_Reg32(TEMPMSR0);
    while ((temp & 0x8000) == 0 && count--){
		printk("\n temp=%x\n",temp);
        temp = DRV_Reg32(TEMPMSR0);
	}
    if ((temp & 0x0FFF) != 0xD78) 
    {
        tscpu_printk("thermal_interrupt_trigger_test: fail, read TEMPMSR0 = 0x%x\n", (temp & 0x0FFF));
    }
    else
        tscpu_printk("thermal_interrupt_trigger_test: pass, read TEMPMSR0 = 0x%x\n", (temp & 0x0FFF));

#endif


#if 0
	count = 10;
    temp = DRV_Reg32(TEMPMSR0);
    

	while (((temp & 0x0FFF) != 0xDA1) && (count--) )
    {
        temp = DRV_Reg32(TEMPMSR0);
        printk("temp=%x,TEMPMSR0=%x, ",temp,DRV_Reg32(TEMPMSR0));
    }

	count = 10;
    temp1 = DRV_Reg32(TEMPMSR1);
    

	while (((temp1 & 0x0FFF) != 0xDA1) && (count--) )
    {
        temp1 = DRV_Reg32(TEMPMSR1);
        printk("temp1=%x,TEMPMSR1=%x, ",temp1,DRV_Reg32(TEMPMSR1));
    }

	count = 10;
    temp2 = DRV_Reg32(TEMPMSR2);
    
	while (((temp2 & 0x0FFF) != 0xDA1) && (count--) )
    {
        temp2 = DRV_Reg32(TEMPMSR2);
        printk("temp2=%x,TEMPMSR2=%x, ",temp2,DRV_Reg32(TEMPMSR2));
    }

    count = 10;
    temp3 = DRV_Reg32(TEMPMSR3);
    
	while (((temp3 & 0x0FFF) != 0xDA1) && (count--) )
    {
        temp3 = DRV_Reg32(TEMPMSR3);
        printk("temp3=%x,TEMPMSR3=%x\n",temp3,DRV_Reg32(TEMPMSR3));
    }


	printk(KERN_CRIT "thermal_DBG  thermal_fast_init TEMPMSR0=%x,TEMPMSR1=%x,TEMPMSR2=%x,TEMPMSR3=%x\n",
        DRV_Reg32(TEMPMSR0),DRV_Reg32(TEMPMSR1),DRV_Reg32(TEMPMSR2),DRV_Reg32(TEMPMSR3));
#endif

	return 0;
}

#ifdef CONFIG_OF
static int get_io_reg_base(void)
{
    struct device_node *node = NULL;
	printk(KERN_CRIT"get_io_reg_base\n");

    node = of_find_compatible_node(NULL, NULL, "mediatek,THERM_CTRL");
    if(node){
		
		thermal_base = of_iomap(node, 0);
		printk("[THERM_CTRL] thermal_base=0x%p\n",thermal_base);
    }
    else{
        	printk("[THERM_CTRL] thermal_base error\n");
    }
	of_property_read_u32(node, "reg", &thermal_phy_base);
	printk("[THERM_CTRL] thermal_base thermal_phy_base=0x%x\n",thermal_phy_base);

	
    thermal_irq_number = irq_of_parse_and_map(node, 0);
    printk("[THERM_CTRL] thermal_irq_number=%d\n",thermal_irq_number);
	if (!thermal_irq_number) {
		printk("[THERM_CTRL] get irqnr failed=%d\n",thermal_irq_number);
		return 0;
	}

#if 1
    node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
    if(node){
		
		tscpu_auxadc_base = of_iomap(node, 0);
		printk("[THERM_CTRL] tscpu_auxadc_base=0x%p\n",tscpu_auxadc_base);
    }
    else{
        	printk("[THERM_CTRL] tscpu_auxadc_base error\n");
    }
	of_property_read_u32(node, "reg", &tscpu_auxadc_phy_base);
	printk("[THERM_CTRL] tscpu_auxadc_phy_base=0x%x\n",tscpu_auxadc_phy_base);
#endif

    node = of_find_compatible_node(NULL, NULL, "mediatek,PERICFG");
    if(node){
		
		tscpu_pericfg_base = of_iomap(node, 0);
		printk("[THERM_CTRL] tscpu_pericfg_base=0x%p\n",tscpu_pericfg_base);
    }
    else{
        	printk("[THERM_CTRL] tscpu_pericfg_base error\n");
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,APMIXED");
    if(node){
		
		tscpu_apmixed_base = of_iomap(node, 0);
		printk("[THERM_CTRL] tscpu_apmixed_base=0x%p\n",tscpu_apmixed_base);
    }
    else{
        	printk("[THERM_CTRL] tscpu_apmixed_base error\n");
    }

	of_property_read_u32(node, "reg", &tscpu_apmixed_phy_base);
	printk("[THERM_CTRL] tscpu_apmixed_phy_base=0x%x\n",tscpu_apmixed_phy_base);
    return 1;
}
#endif

static int __init tscpu_init(void)
{
	int err = 0;
    int temp = 0;
	int cnt=0;
	int wd_api_ret=0;
    struct wd_api *wd_api;

	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtktscpu_dir = NULL;

	printk(KERN_CRIT "tscpu_init\n");

#ifdef CONFIG_OF
	if(get_io_reg_base()==0)
        return 0;
#endif

#if (CONFIG_THERMAL_AEE_RR_REC== 1)
	_mt_thermal_aee_init();
#endif

	
#if 0
	spm_write(SPM_SLEEP_WAKEUP_EVENT_MASK, ~ (1U << 21));
	tscpu_printk("SPM_SLEEP_WAKEUP_EVENT_MASK =0x%08x\n",spm_read(SPM_SLEEP_WAKEUP_EVENT_MASK));
#endif
	wd_api_ret = get_wd_api(&wd_api);
    if(wd_api_ret >=0 ){
		wd_api->wd_thermal_direct_mode_config(WD_REQ_DIS, WD_REQ_RST_MODE);
	}else{
		tscpu_printk("FAILED TO GET WD API\n");
        BUG();
    }

	thermal_cal_prepare();
	thermal_calibration();

	tscpu_reset_thermal();

	temp = DRV_Reg32(TS_CON1);
	tscpu_printk("tscpu_init :TS_CON1=0x%x\n",temp);
	temp &=~(0x00000030); 
	THERMAL_WRAP_WR32(temp, TS_CON1);	                          
	udelay(200);

	tscpu_fast_initial_sw_workaround();

    while(cnt < 50)
	{
		temp = (DRV_Reg32(THAHBST0) >> 16);
		if(cnt>20)
            printk(KERN_CRIT "THAHBST0 = 0x%x,cnt=%d, %d\n", temp,cnt,__LINE__);
        if(temp == 0x0){
            
			thermal_pause_all_periodoc_temp_sensing();
			break;
        }
        udelay(2);
        cnt++;
	}
    
	thermal_disable_all_periodoc_temp_sensing();

	
    thermal_initial_all_bank();

	thermal_release_all_periodoc_temp_sensing();

	read_all_bank_temperature();

#if THERMAL_DRV_FAST_POLL_HRTIMER
	tscpu_update_temperature_timer_init();
#endif

#if MTK_TS_CPU_RT
    {
    	tscpu_printk("mtktscpu_register_thermal creates kthermp\n");
        ktp_thread_handle = kthread_create(ktp_thread, (void *) NULL, "kthermp");
        if (IS_ERR(ktp_thread_handle)) {
			ktp_thread_handle = NULL;
			tscpu_printk("mtktscpu_register_thermal kthermp creation fails\n");
		    goto err_unreg;
		}
		wake_up_process(ktp_thread_handle);
    }
#endif

#ifdef CONFIG_OF
    err= request_irq(thermal_irq_number, thermal_all_bank_interrupt_handler, IRQF_TRIGGER_LOW, THERMAL_NAME, NULL);
	if(err)
		tscpu_printk("tscpu_init IRQ register fail\n");
#else
    err= request_irq(THERM_CTRL_IRQ_BIT_ID, thermal_all_bank_interrupt_handler, IRQF_TRIGGER_LOW, THERMAL_NAME, NULL);
	if(err)
		tscpu_printk("tscpu_init IRQ register fail\n");
#endif

	tscpu_config_all_tc_hw_protect(trip_temp[0], tc_mid_trip);

    err = platform_driver_register(&mtk_thermal_driver);
    if (err)
    {
        tscpu_printk("thermal driver callback register failed..\n");
        return err;
    }

	err = init_cooler();
	if(err)
		return err;

	err = tscpu_register_DVFS_hotplug_cooler();
	if(err){
        tscpu_printk("tscpu_register_DVFS_hotplug_cooler fail\n");
		return err;
	}
	err = tscpu_register_thermal();
	if(err){
        tscpu_printk("tscpu_register_thermal fail\n");
		goto err_unreg;
	}

	mtktscpu_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtktscpu_dir)
	{
		tscpu_printk("[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	}
	else
	{
		entry = proc_create("tzcpu", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}

		entry = proc_create("tzcpu_log", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_log_fops);

		entry = proc_create("thermlmt", S_IRUGO, NULL, &mtktscpu_opp_fops);

        entry = proc_create("tzcpu_cal", S_IRUSR, mtktscpu_dir, &mtktscpu_cal_fops);

        entry = proc_create("tzcpu_read_temperature", S_IRUGO, mtktscpu_dir, &mtktscpu_read_temperature_fops);

        entry = proc_create("tzcpu_set_temperature", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_set_temperature_fops);

        entry = proc_create("tzcpu_talking_flag", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_talking_flag_fops);
#if CPT_ADAPTIVE_AP_COOLER
        entry = proc_create("clatm_setting", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_dtm_setting_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}

        entry = proc_create("clatm_gpu_threshold", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_gpu_threshold_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}
#endif 

#if MTKTSCPU_FAST_POLLING
		entry = proc_create("tzcpu_fastpoll", S_IRUGO | S_IWUSR | S_IWGRP, mtktscpu_dir, &mtktscpu_fastpoll_fops);
		if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}
#endif 

		entry = proc_create("tzcpu_Tj_out_via_HW_pin", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_Tj_out_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}

#if THERMAL_GPIO_OUT_TOGGLE
		entry = proc_create("tzcpu_GPIO_out_monitor", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_GPIO_out_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
		}
#endif

        
		entry = proc_create("clatm", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_atm_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
        

#if THERMAL_HEADROOM
        entry = proc_create("clthp", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_thp_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
#endif

#if CONTINUOUS_TM
        entry = proc_create("clctm", S_IRUGO | S_IWUSR, mtktscpu_dir, &mtktscpu_ctm_fops);
        if (entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
            proc_set_user(entry, 0, 1000);
#else
            entry->gid = 1000;
#endif
        }
#endif
	}

	return 0;

err_unreg:
	tscpu_unregister_DVFS_hotplug_cooler();
	return err;
}

static void __exit tscpu_exit(void)
{
	tscpu_dprintk("tscpu_exit\n");

#if MTK_TS_CPU_RT
	if (ktp_thread_handle)
    	kthread_stop(ktp_thread_handle);
#endif

	tscpu_unregister_thermal();
	tscpu_unregister_DVFS_hotplug_cooler();

#if THERMAL_DRV_FAST_POLL_HRTIMER
	hrtimer_cancel(&ts_tempinfo_hrtimer);
#endif

#if THERMAL_DRV_UPDATE_TEMP_DIRECT_TO_MET
	mt_thermalsampler_registerCB(NULL);
#endif
}

module_init(tscpu_init);
module_exit(tscpu_exit);


