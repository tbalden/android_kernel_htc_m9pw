#include <linux/rtc.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/kernel_stat.h>
#include <linux/workqueue.h>
#include <mach/htc_util.h>
#include <mach/mt_pm_ldo.h>
#include <mach/gpio_const.h>
#include <mach/upmu_common.h>
#include <mach/mt_gpio_core.h>
#include <mach/mtk_thermal_monitor.h>

#define MAX_PID                          32768
#define POWER_PROFILE_POLLING_TIME_S_OFF 10000 
#define POWER_PROFILE_POLLING_TIME_S_ON  60000 
#define NUM_BUSY_PROCESS_CHECK           5

#ifdef CONFIG_VM_EVENT_COUNTERS
#include <linux/mm.h>
#include <linux/vmstat.h>
unsigned long prev_vm_event[NR_VM_EVENT_ITEMS];
#ifdef CONFIG_ZONE_DMA
#define TEXT_FOR_DMA(xx) xx "_dma",
#else
#define TEXT_FOR_DMA(xx)
#endif

#ifdef CONFIG_ZONE_DMA32
#define TEXT_FOR_DMA32(xx) xx "_dma32",
#else
#define TEXT_FOR_DMA32(xx)
#endif

#ifdef CONFIG_HIGHMEM
#define TEXT_FOR_HIGHMEM(xx) xx "_high",
#else
#define TEXT_FOR_HIGHMEM(xx)
#endif
#define TEXTS_FOR_ZONES(xx) TEXT_FOR_DMA(xx) TEXT_FOR_DMA32(xx) xx "_normal", \
					TEXT_FOR_HIGHMEM(xx) xx "_movable",
const char * const vm_event_text[] = {
	"pgpgin",
	"pgpgout",
	"pswpin",
	"pswpout",

	TEXTS_FOR_ZONES("pgalloc")

	"pgfree",
	"pgactivate",
	"pgdeactivate",

	"pgfault",
	"pgmajfault",

	TEXTS_FOR_ZONES("pgrefill")
	TEXTS_FOR_ZONES("pgsteal_kswapd")
	TEXTS_FOR_ZONES("pgsteal_direct")
	TEXTS_FOR_ZONES("pgscan_kswapd")
	TEXTS_FOR_ZONES("pgscan_direct")
	"pgscan_direct_throttle",

#ifdef CONFIG_NUMA
	"zone_reclaim_failed",
#endif
	"pginodesteal",
	"slabs_scanned",
	"kswapd_inodesteal",
	"kswapd_low_wmark_hit_quickly",
	"kswapd_high_wmark_hit_quickly",
	"pageoutrun",
	"allocstall",

	"pgrotated",

#ifdef CONFIG_NUMA_BALANCING
	"numa_pte_updates",
	"numa_huge_pte_updates",
	"numa_hint_faults",
	"numa_hint_faults_local",
	"numa_pages_migrated",
#endif
#ifdef CONFIG_MIGRATION
	"pgmigrate_success",
	"pgmigrate_fail",
#endif
#ifdef CONFIG_COMPACTION
	"compact_migrate_scanned",
	"compact_free_scanned",
	"compact_isolated",
	"compact_stall",
	"compact_fail",
	"compact_success",
#endif

#ifdef CONFIG_HUGETLB_PAGE
	"htlb_buddy_alloc_success",
	"htlb_buddy_alloc_fail",
#endif
	"unevictable_pgs_culled",
	"unevictable_pgs_scanned",
	"unevictable_pgs_rescued",
	"unevictable_pgs_mlocked",
	"unevictable_pgs_munlocked",
	"unevictable_pgs_cleared",
	"unevictable_pgs_stranded",

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	"thp_fault_alloc",
	"thp_fault_fallback",
	"thp_collapse_alloc",
	"thp_collapse_alloc_failed",
	"thp_split",
	"thp_zero_page_alloc",
	"thp_zero_page_alloc_failed",
#endif
};
#endif


struct _htc_kernel_top {
	struct delayed_work dwork;
	int *curr_proc_delta;
	int *curr_proc_pid;
	unsigned int *prev_proc_stat;
	struct task_struct **proc_ptr_array;
	struct kernel_cpustat curr_cpustat;
	struct kernel_cpustat prev_cpustat;
	unsigned long cpustat_time;
	int top_loading_pid[NUM_BUSY_PROCESS_CHECK];
	unsigned long curr_cpu_usage[8][NR_STATS];
	unsigned long prev_cpu_usage[8][NR_STATS];
	spinlock_t lock;
};

static struct workqueue_struct *htc_pm_monitor_wq = NULL;

static const char * const thermal_dev_name[MTK_THERMAL_SENSOR_COUNT] = {
	[MTK_THERMAL_SENSOR_CPU]	= "cpu",
	[MTK_THERMAL_SENSOR_ABB]	= "abb",
	[MTK_THERMAL_SENSOR_PMIC]	= "pmic",
	[MTK_THERMAL_SENSOR_BATTERY]	= "batt",
	[MTK_THERMAL_SENSOR_MD1]	= "md1",
	[MTK_THERMAL_SENSOR_MD2]	= "md2",
	[MTK_THERMAL_SENSOR_WIFI]	= "wifi",
	[MTK_THERMAL_SENSOR_BATTERY2]	= "batt2",
	[MTK_THERMAL_SENSOR_BUCK]	= "buck",
	[MTK_THERMAL_SENSOR_AP]		= "ap",
	[MTK_THERMAL_SENSOR_PCB1]	= "pcb1",
	[MTK_THERMAL_SENSOR_PCB2]	= "pcb2",
	[MTK_THERMAL_SENSOR_SKIN]	= "skin",
	[MTK_THERMAL_SENSOR_XTAL]	= "xtal"
};

static const char * const mt633x_vreg_name[MT65XX_POWER_LDO_DEFAULT] = {
	[MT6331_POWER_BUCK_VDVFS11]     = "mt6331_buck_vdvfs11",
	[MT6331_POWER_BUCK_VDVFS12]     = "mt6331_buck_vdvfs12",
	[MT6331_POWER_BUCK_VDVFS13]     = "mt6331_buck_vdvfs13",
	[MT6331_POWER_BUCK_VDVFS14]     = "mt6331_buck_vdvfs14",
	[MT6331_POWER_BUCK_VCORE2]      = "mt6331_buck_vcore2",
	[MT6331_POWER_BUCK_VIO18]       = "mt6331_buck_vio18",
	[MT6331_POWER_LDO_VTCXO1]       = "mt6331_ldo_vtcxo1",
	[MT6331_POWER_LDO_VTCXO2]       = "mt6331_ldo_vtcxo2",
	[MT6331_POWER_LDO_VAUD32]       = "mt6331_ldo_vaud32",
	[MT6331_POWER_LDO_VAUXA32]      = "mt6331_ldo_vauxa3",
	[MT6331_POWER_LDO_VCAMA]        = "mt6331_ldo_vcama",
	[MT6331_POWER_LDO_VMCH]         = "mt6331_ldo_vmch",
	[MT6331_POWER_LDO_VEMC33]       = "mt6331_ldo_vemc33",
	[MT6331_POWER_LDO_VIO28]        = "mt6331_ldo_vio28",
	[MT6331_POWER_LDO_VMC]          = "mt6331_ldo_vmc",
	[MT6331_POWER_LDO_VCAM_AF]      = "mt6331_ldo_vcam_af",
	[MT6331_POWER_LDO_VGP1]         = "mt6331_ldo_vgp1",
	[MT6331_POWER_LDO_VGP4]         = "mt6331_ldo_vgp4",
	[MT6331_POWER_LDO_VSIM1]        = "mt6331_ldo_vsim1",
	[MT6331_POWER_LDO_VSIM2]        = "mt6331_ldo_vsim2",
	[MT6331_POWER_LDO_VFBB]         = "mt6331_ldo_vfbb",
	[MT6331_POWER_LDO_VRTC]         = "mt6331_ldo_vrtc",
	[MT6331_POWER_LDO_VMIPI]        = "mt6331_ldo_vmipi",
	[MT6331_POWER_LDO_VIBR]         = "mt6331_ldo_vibr",
	[MT6331_POWER_LDO_VDIG18]       = "mt6331_ldo_vdig18",
	[MT6331_POWER_LDO_VCAMD]        = "mt6331_ldo_vcamd",
	[MT6331_POWER_LDO_VUSB10]       = "mt6331_ldo_vusb10",
	[MT6331_POWER_LDO_VCAM_IO]      = "mt6331_ldo_vcam_io",
	[MT6331_POWER_LDO_VSRAM_DVFS1]  = "mt6331_ldo_vsram_dvfs1",
	[MT6331_POWER_LDO_VGP2]         = "mt6331_ldo_vgp2",
	[MT6331_POWER_LDO_VGP3]         = "mt6331_ldo_vgp3",
	[MT6331_POWER_LDO_VBIASN]       = "mt6331_ldo_vbiasn",
	[MT6332_POWER_BUCK_VDVFS2]      = "mt6332_buck_vdvfs2",
	[MT6332_POWER_BUCK_VDRAM]       = "mt6332_buck_vdram",
	[MT6332_POWER_BUCK_VRF18_1]     = "mt6332_buck_vrf18_1",
	[MT6332_POWER_BUCK_VRF18_2]     = "mt6332_buck_vrf18_2",
	[MT6332_POWER_BUCK_VPA]         = "mt6332_buck_vpa",
	[MT6332_POWER_LDO_VBIF28]       = "mt6332_ldo_vbif28",
	[MT6332_POWER_LDO_VAUXB32]      = "mt6332_ldo_vauxb32",
	[MT6332_POWER_LDO_VUSB33]       = "mt6332_ldo_vusb33",
	[MT6332_POWER_LDO_VDIG18]       = "mt6332_ldo_vdig18",
	[MT6332_POWER_LDO_VSRAM_DVFS2]  = "mt6332_ldo_vsram_dvfs2"
};

struct _vregs {
	int vreg_id;
	char vreg_name[32];
};

struct _vregs mtk_vregs[MT65XX_POWER_LDO_DEFAULT];
static char *gpio_sleep_status_info;
static char *vreg_sleep_status_info;
static int slept = 0;
static unsigned int pon_reason = 0xFFFF;
static int son = 0;
static int power_profile_polling_time = 10000;

ssize_t htc_pm_show_wakelocks(char *buf, int size, bool show_active);
extern int da9210_vosel(unsigned long val);
extern unsigned long da9210_vol_get(void);
extern int aee_rr_last_fiq_step(void);

char htc_boot_reason[][16] =
    { "keypad", "usb_chg", "rtc_alarm", "wdt", "reboot", "tool_reboot", "smpl", "unknow", "kpanic", "wdt_sw", "wdt_hw" };
static void htc_show_pon_poff()
{
	char *br_ptr;
	if ((pon_reason == 0xFFFF) &&
		(br_ptr = strstr(saved_command_line, "boot_reason=")) != 0) {
		
		pon_reason = br_ptr[12] - '0';
#ifdef CONFIG_MTK_RAM_CONSOLE
		if (aee_rr_last_fiq_step() != 0)
			pon_reason = 8;
#endif
	} else {
		if (pon_reason <= 10)
			printk("[K] pon: %s, poff: %s\n", htc_boot_reason[pon_reason], "NA");
	}
}

static void htc_show_wakelocks(void)
{
	char wl_buf[256];
	int print_size;
	int buf_size;

	buf_size = sizeof(wl_buf);
	print_size = htc_pm_show_wakelocks(wl_buf, buf_size, true);
	printk("wakeup sources: %s", wl_buf);
	if (print_size >= (buf_size-1))
		printk("Can't print wakelocks due to out of buf size.\n");

	return;
}

static void htc_show_thermal_temp(void)
{
	int i = 0, temp = 0;
	static int count = 0;

	
		for (i = 0; i < MTK_THERMAL_SENSOR_COUNT; i++) {
			temp = mtk_thermal_get_temp(i);
			if (temp == -127000)
				continue;
			printk("[K][THERMAL] Sensor %d (%5s) = %d\n", i, thermal_dev_name[i], temp);
		}
		count = 0;
	

	return;
}

static void sort_cputime_by_pid(int *src, int *pid_pos, int pid_cnt, int *result)
{
    int i = 0, j = 0, k = 0, l = 0;
    int pid_found = 0;

    for (i = 0; i < NUM_BUSY_PROCESS_CHECK; i++) {
        result[i] = 0;
        if (i == 0) {
            for (j = 0; j < pid_cnt; j++) {
                k = pid_pos[j];
                
                if(src[result[i]] < src[k]) {
                    result[i] = k;
                }
            }
        } else {
            for (j = 0; j < pid_cnt; j++) {
                k = pid_pos[j];
                
                for (l = 0; l < i; l++) {
                    
                    if (result[l] == k) {
                        pid_found = 1;
                        break;
                    }
                }
                
                if (pid_found) {
                    pid_found = 0;
                    continue;
                }

                
                if(src[result[i]] < src[k]) {
                    result[i] = k;
                }
            }
        }
    }
}

#ifdef arch_idle_time
static cputime64_t get_idle_time(int cpu)
{
	cputime64_t idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);

	return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
	cputime64_t iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);

	return iowait;
}
#else
static u64 get_idle_time(int cpu)
{
	u64 idle = 0;
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = usecs_to_cputime64(idle_time);

	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait;
	u64 iowait_time = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_time == -1ULL)
		
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = usecs_to_cputime64(iowait_time);

	return iowait;
}
#endif

static void get_all_cpustat(struct _htc_kernel_top *ktop, struct kernel_cpustat *cpu_stat)
{
	int cpu = 0;

	if (!cpu_stat)
		return;

	memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

	for_each_possible_cpu(cpu) {
		cpu_stat->cpustat[CPUTIME_USER] += kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
		cpu_stat->cpustat[CPUTIME_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
		cpu_stat->cpustat[CPUTIME_SYSTEM] += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
		cpu_stat->cpustat[CPUTIME_IDLE] += get_idle_time(cpu);
		cpu_stat->cpustat[CPUTIME_IOWAIT] += get_iowait_time(cpu);
		cpu_stat->cpustat[CPUTIME_IRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		cpu_stat->cpustat[CPUTIME_SOFTIRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
		cpu_stat->cpustat[CPUTIME_STEAL] += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
		cpu_stat->cpustat[CPUTIME_GUEST] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST];
		cpu_stat->cpustat[CPUTIME_GUEST_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST_NICE];
		ktop->curr_cpu_usage[cpu][CPUTIME_USER] = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
		ktop->curr_cpu_usage[cpu][CPUTIME_NICE] = kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
		ktop->curr_cpu_usage[cpu][CPUTIME_SYSTEM] = kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
		ktop->curr_cpu_usage[cpu][CPUTIME_IDLE] = get_idle_time(cpu);
	}

	return;
}

static unsigned long htc_calculate_cpustat_time(struct kernel_cpustat curr_cpustat,
						struct kernel_cpustat prev_cpustat)
{
	unsigned long user_time = 0, system_time = 0, io_time = 0;
	unsigned long irq_time = 0, idle_time = 0;

	user_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_USER] +
					curr_cpustat.cpustat[CPUTIME_NICE]) -
					(prev_cpustat.cpustat[CPUTIME_USER] +
					prev_cpustat.cpustat[CPUTIME_NICE]));
	system_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_SYSTEM] -
					prev_cpustat.cpustat[CPUTIME_SYSTEM]);
	io_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_IOWAIT] -
					prev_cpustat.cpustat[CPUTIME_IOWAIT]);
	irq_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IRQ] +
					curr_cpustat.cpustat[CPUTIME_SOFTIRQ]) -
					(prev_cpustat.cpustat[CPUTIME_IRQ] +
					prev_cpustat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IDLE] > prev_cpustat.cpustat[CPUTIME_IDLE]) ?
					curr_cpustat.cpustat[CPUTIME_IDLE] - prev_cpustat.cpustat[CPUTIME_IDLE] : 0);

	idle_time += (unsigned long) ((curr_cpustat.cpustat[CPUTIME_STEAL] +
					curr_cpustat.cpustat[CPUTIME_GUEST]) -
					(prev_cpustat.cpustat[CPUTIME_STEAL] +
					prev_cpustat.cpustat[CPUTIME_GUEST]));

	return (user_time + system_time + io_time + irq_time + idle_time);
}

static void htc_calc_kernel_top(struct _htc_kernel_top *ktop)
{
	int pid_cnt = 0;
	ulong flags;
	struct task_struct *proc;
	struct task_cputime cputime;

	if(ktop->proc_ptr_array == NULL ||
	   ktop->curr_proc_delta == NULL ||
	   ktop->curr_proc_pid == NULL ||
	   ktop->prev_proc_stat == NULL)
		return;

	spin_lock_irqsave(&ktop->lock, flags);

	
	for_each_process(proc) {
		if (proc->pid < MAX_PID) {
			thread_group_cputime(proc, &cputime);
			ktop->curr_proc_delta[proc->pid] =
				(cputime.utime + cputime.stime) -
				ktop->prev_proc_stat[proc->pid];
			ktop->proc_ptr_array[proc->pid] = proc;

			if (ktop->curr_proc_delta[proc->pid] > 0) {
				ktop->curr_proc_pid[pid_cnt] = proc->pid;
				pid_cnt++;
			}
		}
	}
	sort_cputime_by_pid(ktop->curr_proc_delta, ktop->curr_proc_pid, pid_cnt, ktop->top_loading_pid);

	
	get_all_cpustat(ktop, &ktop->curr_cpustat);
	ktop->cpustat_time = htc_calculate_cpustat_time(ktop->curr_cpustat, ktop->prev_cpustat);

	
	for_each_process(proc) {
		if (proc->pid < MAX_PID) {
			thread_group_cputime(proc, &cputime);
			ktop->prev_proc_stat[proc->pid] = cputime.stime + cputime.utime;
		}
	}
	memcpy(&ktop->prev_cpustat, &ktop->curr_cpustat, sizeof(struct kernel_cpustat));
	spin_unlock_irqrestore(&ktop->lock, flags);

	return;
}

static void htc_show_kernel_top(struct _htc_kernel_top *ktop)
{
	int top_n_pid = 0, i;
	int cpu = 0;
	unsigned long usage = 0, total = 0, idle_time = 0, proc_usage;

	
	printk("[K] CPU Usage\tPID\tName\t\tCPU Time (Total: %lu)\n", ktop->cpustat_time);
	for (i = 0; i < NUM_BUSY_PROCESS_CHECK; i++) {
		if (ktop->cpustat_time > 0) {
			top_n_pid = ktop->top_loading_pid[i];
			proc_usage = ktop->curr_proc_delta[top_n_pid] * 100 / ktop->cpustat_time;
			printk("[K]%8lu%%\t%d\t%s\t\t%d\n",
				proc_usage,
				top_n_pid,
				ktop->proc_ptr_array[top_n_pid]->comm,
				ktop->curr_proc_delta[top_n_pid]);
		}
	}

	
	printk("[K] CPU usage per core: ");
	for_each_possible_cpu(cpu) {
		usage = (ktop->curr_cpu_usage[cpu][CPUTIME_USER] -
			ktop->prev_cpu_usage[cpu][CPUTIME_USER]) +
			(ktop->curr_cpu_usage[cpu][CPUTIME_NICE] -
			ktop->prev_cpu_usage[cpu][CPUTIME_NICE]) +
			(ktop->curr_cpu_usage[cpu][CPUTIME_SYSTEM] -
			ktop->prev_cpu_usage[cpu][CPUTIME_SYSTEM]);
		idle_time = (ktop->curr_cpu_usage[cpu][CPUTIME_IDLE] > ktop->prev_cpu_usage[cpu][CPUTIME_IDLE]) ?
			     ktop->curr_cpu_usage[cpu][CPUTIME_IDLE] - ktop->prev_cpu_usage[cpu][CPUTIME_IDLE] : 0;
		total = usage + idle_time;
		printk("[%lu%%]", usage *100 / total);
		ktop->prev_cpu_usage[cpu][CPUTIME_USER] = ktop->curr_cpu_usage[cpu][CPUTIME_USER];
		ktop->prev_cpu_usage[cpu][CPUTIME_NICE] = ktop->curr_cpu_usage[cpu][CPUTIME_NICE];
		ktop->prev_cpu_usage[cpu][CPUTIME_SYSTEM] = ktop->curr_cpu_usage[cpu][CPUTIME_SYSTEM];
		ktop->prev_cpu_usage[cpu][CPUTIME_IDLE] = ktop->curr_cpu_usage[cpu][CPUTIME_IDLE];
	}
	printk("\n");
	memset(ktop->curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(ktop->proc_ptr_array, 0, sizeof(struct task_struct *) * MAX_PID);
	memset(ktop->curr_proc_pid, 0, sizeof(int) * MAX_PID);

	return;
}

extern char* saved_command_line;
int is_son()
{
	char *p;
	unsigned td_sf = 0;
	size_t sf_len = strlen("td.sf=");
	size_t cmdline_len = strlen(saved_command_line);
	p = saved_command_line;
	for (p = saved_command_line; p < saved_command_line + cmdline_len - sf_len; p++) {
		if (!strncmp(p, "td.sf=", sf_len)) {
			p += sf_len;
			if (*p != '0')
				td_sf = 1;
			break;
		}
	}
	return td_sf;
}
static void htc_pm_monitor_work_func(struct work_struct *work)
{
	struct _htc_kernel_top *ktop = container_of(work, struct _htc_kernel_top,
					dwork.work);
	struct timespec ts;
	struct rtc_time tm;

	unsigned long vm_event[NR_VM_EVENT_ITEMS];
	int i;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	printk("[K][PM] hTC PM Statistic start (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon +1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	
	htc_show_wakelocks();
	
	htc_show_thermal_temp();
	
	htc_calc_kernel_top(ktop);
	htc_show_kernel_top(ktop);
	
	htc_show_pon_poff();

	if(!son){
		all_vm_events(vm_event);
		vm_event[PGPGIN] /= 2;
		
		vm_event[PGPGOUT] /= 2;

		for(i = 0; i < NR_VM_EVENT_ITEMS; i++) {
			if (vm_event[i] - prev_vm_event[i] > 0)
				pr_info("[K] %s = %lu\n", vm_event_text[i], vm_event[i] - prev_vm_event[i]);
		}
		memcpy(prev_vm_event, vm_event, sizeof(unsigned long) * NR_VM_EVENT_ITEMS);
	}

	printk("[K][PM] hTC PM Statistic done\n");
	queue_delayed_work(htc_pm_monitor_wq, &ktop->dwork, msecs_to_jiffies(power_profile_polling_time));
}

static struct dentry *gpio_dbgfs_base;
static struct dentry *vreg_dbgfs_base;
static struct dentry *vreg_dbgfs;
static struct dentry *ptpod_dbgfs_base;

int mt_dump_gpios(struct seq_file *m, int curr_len, char *gpio_buffer)
{
	int idx = 0, len = 0;
	char list_gpio[128];
	char *title_msg = "------------ MTK GPIO -------------";

	if (m) {
		seq_printf(m, "%s\n", title_msg);
	} else {
                printk("%s\n", title_msg);
                curr_len += sprintf(gpio_buffer + curr_len,
				"%s\n", title_msg);
        }

	for (idx = MT_GPIO_BASE_START; idx < MT_GPIO_BASE_MAX; idx++) {
		memset(list_gpio, 0 , sizeof(list_gpio));
		len = 0;
		len += sprintf(list_gpio + len, "GPIO[%3d]: ", idx);
		len += sprintf(list_gpio + len, "[MODE]0x%d, ", mt_get_gpio_mode_base(idx));
		len += sprintf(list_gpio + len, "[DIR]%s, ",
				mt_get_gpio_dir_base(idx) >= 0 ? (mt_get_gpio_dir_base(idx) ? " OUT" : "  IN") : "NULL");
		len += sprintf(list_gpio + len, "[PULL_SEL]%s, ",
				mt_get_gpio_pull_select_base(idx) >= 0 ? (mt_get_gpio_pull_select_base(idx) ? "  UP" : "DOWN") : "NULL");
		len += sprintf(list_gpio + len, "[DIN]0x%d, ", mt_get_gpio_in_base(idx));
		len += sprintf(list_gpio + len, "[DOUT]%s, ",
				mt_get_gpio_out_base(idx) >= 0 ? (mt_get_gpio_out_base(idx) ? "HIGH" : " LOW") : "NULL");
		len += sprintf(list_gpio + len, "[PULL_EN]%s, ",
				mt_get_gpio_pull_enable_base(idx) >= 0 ? (mt_get_gpio_pull_enable_base(idx) ? "  EN" : " DIS"): "NULL");
		len += sprintf(list_gpio + len, "[IES]%s",
				mt_get_gpio_ies_base(idx) >= 0 ? (mt_get_gpio_ies_base(idx) ? "  EN" : " DIS"): "NULL");
#if 0
		len += sprintf(list_gpio + len, "[SMT]0x%d, ", mt_get_gpio_smt_base(idx));
#endif
		list_gpio[127] = '\0';
                if (m) {
                        seq_printf(m, "%s\n", list_gpio);
                } else {
                        printk("%s\n", list_gpio);
                        curr_len += sprintf(gpio_buffer + curr_len, "%s\n", list_gpio);
                }

	}

	return curr_len;
}

int dump_sleep_gpio(void)
{
	slept = 1;
	if (gpio_sleep_status_info)
		mt_dump_gpios(NULL, 0, gpio_sleep_status_info);
}

static int list_sleep_gpios_show(struct seq_file *m, void *unused)
{
	if (slept)
		seq_printf(m, gpio_sleep_status_info);
	else
		seq_printf(m, "Device haven't suspended yet or sleep gpio dump flag doesn't be enabled!\n");
        return 0;
}

static int list_sleep_gpios_open(struct inode *inode, struct file *file)
{
        return single_open(file, list_sleep_gpios_show, inode->i_private);
}

static const struct file_operations list_sleep_gpios_fops = {
        .open           = list_sleep_gpios_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int list_gpios_show(struct seq_file *m, void *unused)
{
        mt_dump_gpios(m, 0, NULL);
        return 0;
}

static int list_gpios_open(struct inode *inode, struct file *file)
{
        return single_open(file, list_gpios_show, inode->i_private);
}

static const struct file_operations list_gpios_fops = {
        .open           = list_gpios_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int vreg_en_get(void *data, u64 *val)
{
	struct _vregs *vreg = data;
	int vreg_id = vreg->vreg_id;

	*val = pmic_ldo_get_status(vreg_id);

	return 0;
}

static int vreg_en_set(void *data, u64 val)
{
	struct _vregs *vreg = data;
	int vreg_id = vreg->vreg_id;

	pmic_ldo_enable(vreg_id, val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vregs_en_fops, vreg_en_get,
			vreg_en_set, "%lld\n");

static int vreg_vol_get(void *data, u64 *val)
{
	struct _vregs *vreg = data;
	int vreg_id = vreg->vreg_id;

	*val = pmic_ldo_get_voltage(vreg_id);

	return 0;
}

static int vreg_vol_set(void *data, u64 val)
{
	struct _vregs *vreg = data;
	int vreg_id = vreg->vreg_id;

	pmic_ldo_vol_sel(vreg_id, val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vregs_vol_fops, vreg_vol_get,
			vreg_vol_set, "%lld\n");

static int ext_buck_vol_get(void *data, u64 *val)
{
	*val = da9210_vol_get();

	return 0;
}

static int ext_buck_vol_set(void *data, u64 val)
{
	
	if (val > 1570000 || val < 300000) {
		printk("%s: not support voltage: %llu\n", __func__, val);
		return -1;
	}
	da9210_vosel(val / 10);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ext_buck_vol_fops, ext_buck_vol_get,
			ext_buck_vol_set, "%lld\n");

extern unsigned long htc_vcore_vol_get(void);
extern int htc_vcore_vol_set(int vcore_uv);

static int vcore_vol_get(void *data, u64 *val)
{
	*val = htc_vcore_vol_get();

	return 0;
}

static int vcore_vol_set(void *data, u64 val)
{
	
	if (val > 1310000 || val < 700000) {
		printk("%s: not support voltage: %llu\n", __func__, val);
		return -1;
	}
	htc_vcore_vol_set(val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcore_vol_fops,vcore_vol_get,
			vcore_vol_set, "%lld\n");

extern unsigned long htc_vgpu_vol_get(void);
extern int htc_vgpu_vol_set(int vgpu_uv);

static int vgpu_vol_get(void *data, u64 *val)
{
	*val = htc_vgpu_vol_get();

	return 0;
}

static int vgpu_vol_set(void *data, u64 val)
{
	
	if (val > 1310000 || val < 700000) {
		printk("%s: not support voltage: %llu\n", __func__, val);
		return -1;
	}
	htc_vgpu_vol_set(val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vgpu_vol_fops,vgpu_vol_get,
			vgpu_vol_set, "%lld\n");

int mt_dump_vregs(struct seq_file *m, int curr_len, char *vreg_buffer)
{
	u32 val = 0;
	int idx = 0, len = 0;
	char list_vreg[128];
	char *title_msg = "------------ MTK VREG -------------";

	if (m) {
		seq_printf(m, "%s\n", title_msg);
	} else {
                printk("%s\n", title_msg);
                curr_len += sprintf(vreg_buffer + curr_len,
				"%s\n", title_msg);
        }

	for (idx = 0; idx < MT65XX_POWER_LDO_DEFAULT; idx++) {
		memset(list_vreg, 0 , sizeof(list_vreg));
		len = 0;
		len += sprintf(list_vreg + len, "%22s: ", mtk_vregs[idx].vreg_name);
		
		val = pmic_ldo_get_status(idx);
		len += sprintf(list_vreg + len, "[ EN] %s, ", val ? "YES" : "NO ");
		
		val = pmic_ldo_get_voltage(idx);
		len += sprintf(list_vreg + len, "[Vol] %7u %s", val, val < 100000 ? "mV" : "uV");

		list_vreg[127] = '\0';
                if (m) {
                        seq_printf(m, "%s\n", list_vreg);
                } else {
                        printk("%s\n", list_vreg);
                        curr_len += sprintf(vreg_buffer + curr_len, "%s\n", list_vreg);
                }
	}

	return curr_len;
}

static int list_vregs_show(struct seq_file *m, void *unused)
{
        mt_dump_vregs(m, 0, NULL);
        return 0;
}

static int list_vregs_open(struct inode *inode, struct file *file)
{
        return single_open(file, list_vregs_show, inode->i_private);
}

static const struct file_operations list_vregs_fops = {
        .open           = list_vregs_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

int dump_sleep_vreg(void)
{
	slept = 1;
	if (vreg_sleep_status_info)
		mt_dump_vregs(NULL, 0, vreg_sleep_status_info);
}

static int list_sleep_vregs_show(struct seq_file *m, void *unused)
{
	if (slept)
		seq_printf(m, vreg_sleep_status_info);
	else
		seq_printf(m, "Device haven't suspended yet or sleep vreg dump flag doesn't be enabled!\n");
        return 0;
}

static int list_sleep_vregs_open(struct inode *inode, struct file *file)
{
        return single_open(file, list_sleep_vregs_show, inode->i_private);
}

static const struct file_operations list_sleep_vregs_fops = {
        .open           = list_sleep_vregs_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int htc_vreg_create_dbgfs(void)
{
	int idx = 0;
	struct _vregs *vreg = NULL;

	
	vreg_dbgfs_base = debugfs_create_dir("htc_vreg", NULL);
	if (!vreg_dbgfs_base)
		return -ENOMEM;

	
	for (idx = 0; idx < MT65XX_POWER_LDO_DEFAULT; idx++) {
		mtk_vregs[idx].vreg_id = idx;
		sprintf(mtk_vregs[idx].vreg_name, "%s", mt633x_vreg_name[idx]);
		vreg_dbgfs = debugfs_create_dir(mtk_vregs[idx].vreg_name, vreg_dbgfs_base);
		if (!vreg_dbgfs)
			return -ENOMEM;
		vreg = &mtk_vregs[idx];
		if (!debugfs_create_file("enable", S_IRUGO, vreg_dbgfs,
					  vreg, &vregs_en_fops))
			return -ENOMEM;
		if (!debugfs_create_file("voltage", S_IRUGO, vreg_dbgfs,
					vreg, &vregs_vol_fops))
			return -ENOMEM;
	}

	
	vreg_dbgfs = debugfs_create_dir("da9210", vreg_dbgfs_base);
	if (!vreg_dbgfs)
		return -ENOMEM;
	if (!debugfs_create_file("voltage", S_IRUGO, vreg_dbgfs,
				NULL, &ext_buck_vol_fops))
		return -ENOMEM;

	
	vreg_dbgfs = debugfs_create_dir("vcore", vreg_dbgfs_base);
	if (!vreg_dbgfs)
		return -ENOMEM;
	if (!debugfs_create_file("voltage", S_IRUGO, vreg_dbgfs,
				NULL, &vcore_vol_fops))
		return -ENOMEM;


	
	vreg_dbgfs = debugfs_create_dir("vgpu", vreg_dbgfs_base);
	if (!vreg_dbgfs)
		return -ENOMEM;
	if (!debugfs_create_file("voltage", S_IRUGO, vreg_dbgfs,
				NULL, &vgpu_vol_fops))
		return -ENOMEM;

	if (!debugfs_create_file("list_vregs", S_IRUGO, vreg_dbgfs_base,
                        NULL, &list_vregs_fops))
                return -ENOMEM;

	if (!debugfs_create_file("list_sleep_vregs", S_IRUGO, vreg_dbgfs_base,
                        NULL, &list_sleep_vregs_fops))
                return -ENOMEM;
	vreg_sleep_status_info = vmalloc(25000);
	if (!vreg_sleep_status_info) {
		pr_err("[PM] vmalloc memory failed in %s\n", __func__);
	}

	return 0;
}

extern int get_ptpod_status(void);
static int ptpod_enable_get(void *data, u64 *val)
{
	*val = get_ptpod_status();

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ptpod_fops, ptpod_enable_get,
			NULL, "%lld\n");

static int htc_ptpod_create_dbgfs(void)
{
	ptpod_dbgfs_base = debugfs_create_dir("htc_ptpod", NULL);
	if (!ptpod_dbgfs_base)
		return -ENOMEM;

	if (!debugfs_create_file("enable", S_IRUGO, ptpod_dbgfs_base,
			NULL, &ptpod_fops))
		return -ENOMEM;
}

static int __init htc_monitor_init(void)
{
	int cpu;
	struct _htc_kernel_top *htc_kernel_top;
	son = is_son();

	if (htc_pm_monitor_wq == NULL)
		
		htc_pm_monitor_wq = create_workqueue("htc_pm_monitor_wq");

	if (!htc_pm_monitor_wq) {
		pr_err("[K] Fail to create htc_pm_monitor_wq\n");
		return -1;
	}

	printk("[K] Success to create htc_pm_monitor_wq.\n");
	htc_kernel_top = vmalloc(sizeof(*htc_kernel_top));
	spin_lock_init(&htc_kernel_top->lock);

	htc_kernel_top->prev_proc_stat = vmalloc(sizeof(unsigned int) * MAX_PID);
	htc_kernel_top->curr_proc_delta = vmalloc(sizeof(int) * MAX_PID);
	htc_kernel_top->proc_ptr_array = vmalloc(sizeof(struct task_struct *) * MAX_PID);
	htc_kernel_top->curr_proc_pid = vmalloc(sizeof(int) * MAX_PID);

	memset(htc_kernel_top->prev_proc_stat, 0, sizeof(unsigned int) * MAX_PID);
	memset(htc_kernel_top->curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(htc_kernel_top->proc_ptr_array, 0, sizeof(struct task_struct *) * MAX_PID);
	memset(htc_kernel_top->curr_proc_pid, 0, sizeof(int) * MAX_PID);

	for_each_possible_cpu(cpu) {
		memset(htc_kernel_top->curr_cpu_usage[cpu], 0, NR_STATS);
		memset(htc_kernel_top->prev_cpu_usage[cpu], 0, NR_STATS);
	}

        power_profile_polling_time = son ? POWER_PROFILE_POLLING_TIME_S_ON : POWER_PROFILE_POLLING_TIME_S_OFF;

	INIT_DELAYED_WORK(&htc_kernel_top->dwork, htc_pm_monitor_work_func);
	queue_delayed_work(htc_pm_monitor_wq, &htc_kernel_top->dwork,
					msecs_to_jiffies(power_profile_polling_time));

	
	gpio_dbgfs_base = debugfs_create_dir("htc_gpio", NULL);
	if (!gpio_dbgfs_base)
		return -ENOMEM;

        if (!debugfs_create_file("list_gpios", S_IRUGO, gpio_dbgfs_base,
                        NULL, &list_gpios_fops))
                return -ENOMEM;

        if (!debugfs_create_file("list_sleep_gpios", S_IRUGO, gpio_dbgfs_base,
                        NULL, &list_sleep_gpios_fops))
                return -ENOMEM;
	gpio_sleep_status_info = vmalloc(25000);
	if (!gpio_sleep_status_info) {
		pr_err("[PM] vmalloc memory failed in %s\n", __func__);
	}

	
	
	htc_vreg_create_dbgfs();
	
	htc_ptpod_create_dbgfs();

	return 0;
}

static void __exit htc_monitor_exit(void)
{
	return;
}

module_init(htc_monitor_init);
module_exit(htc_monitor_exit);

MODULE_DESCRIPTION("HTC Power Utility Profile driver");
MODULE_AUTHOR("Kenny Liu <kenny_liu@htc.com>");
MODULE_LICENSE("GPL");
