/*
 * SMP initialisation and IPI support
 * Based on arch/arm/kernel/smp.c
 *
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cache.h>
#include <linux/profile.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
#include <linux/of.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cpu_ops.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/smp_plat.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/ptrace.h>
#include <mach/wd_api.h>
#include <linux/mt_sched_mon.h>

#define CREATE_TRACE_POINTS
#include <trace/events/ipi.h>
#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <linux/aee.h>
extern void set_cpu_footprint(unsigned cpu, unsigned state);
#endif

struct secondary_data secondary_data;

enum ipi_msg_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
};

static int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (cpu_ops[cpu]->cpu_boot)
		return cpu_ops[cpu]->cpu_boot(cpu);

	return -EOPNOTSUPP;
}

static DECLARE_COMPLETION(cpu_running);

#define mt_reg_sync_writel(v, a)	mt65xx_reg_sync_writel(v, a)

#define mt65xx_reg_sync_writel(v, a) \
        do {    \
            __raw_writel((v), IOMEM((a)));   \
            dsb();  \
        } while (0)

#define REG_READ(addr)   (*(volatile u32 *)(addr))
#define REG_WRITE(addr, value) mt_reg_sync_writel(value, addr)

extern int check_pmic_wrap_init(void);
extern void mt_pwrap_hal_init(void);
extern void pmic_full_reset(void);

int __cpuinit __cpu_up(unsigned int cpu, struct task_struct *idle)
{
	int ret,res;
	int i;
  struct wd_api * wd_api = NULL;
	secondary_data.stack = task_stack_page(idle) + THREAD_START_SP;
	__flush_dcache_area(&secondary_data, sizeof(secondary_data));

	ret = boot_secondary(cpu, idle);
	if (ret == 0) {
		wait_for_completion_timeout(&cpu_running,
					    msecs_to_jiffies(1000));

		if (!cpu_online(cpu)) {
#if 0
			for(i=0x0;i<=4;i++)
			{
			   REG_WRITE(0x10200404 ,((REG_READ(0x10200404 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200408));
			}
			for(i=0x05;i<=0x15;i++)
			{
			   REG_WRITE(0x10200404 ,((REG_READ(0x10200404 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200408));
			}
			for(i=0x20;i<=0x45;i++)
			{
			   REG_WRITE(0x10200404 ,((REG_READ(0x10200404 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200408));
			}
			
			for(i=0x0;i<=4;i++)
			{
			   REG_WRITE(0x10200504  ,((REG_READ(0x10200504 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200508 ));
			}
			for(i=0x05;i<=0x15;i++)
			{
			   REG_WRITE(0x10200504  ,((REG_READ(0x10200504 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200508 ));
			}
			for(i=0x20;i<=0x45;i++)
			{
			   REG_WRITE(0x10200504  ,((REG_READ(0x10200504 )&0xffffff00)|i));
			   pr_crit("Cluster0: Set 8'h%x : 0x%x\n",i,REG_READ(0x10200508 ));
			}
			
			pr_crit("MPx_AXI_CONFIG: REG 0x1020002c : 0x%x\n", REG_READ(0x1020002c));
			pr_crit("MPx_AXI_CONFIG: REG 0x1020022c : 0x%x\n", REG_READ(0x1020022c));
			pr_crit("ACLKEN_DIV: REG 0x10200640 : 0x%x\n", REG_READ(0x10200640));
			pr_crit("CCI: REG 0x10394000  : 0x%x\n", REG_READ(0x10394000));
			pr_crit("CCI: REG 0x10395000  : 0x%x\n", REG_READ(0x10395000));
#endif
			pr_crit("CPU%u: failed to come online\n", cpu);
                        #if 1
		        pr_crit("Trigger WDT RESET\n");
                        res = get_wd_api(&wd_api);
                        if(res) 
                        {
                          pr_crit("get wd api error !!\n");
                        }else {
                          wd_api -> wd_sw_reset(3);  
                        }
                        #endif
                        #if 0
			pr_crit("Trigger PMIC full reset.\n");
                        if(check_pmic_wrap_init())
                        {
                          mt_pwrap_hal_init();
                        }
                        pmic_full_reset();
                        #endif
                   ret = -EIO;
	        }
	} else {
		pr_err("CPU%u: failed to boot: %d\n", cpu, ret);
	}

	secondary_data.stack = NULL;

	return ret;
}

static void __cpuinit smp_store_cpu_info(unsigned int cpuid)
{
	store_cpu_topology(cpuid);
}

asmlinkage void __cpuinit secondary_start_kernel(void)
{
	struct mm_struct *mm = &init_mm;
	unsigned int cpu = smp_processor_id();

	printk("CPU%u: Booted secondary processor\n", cpu);

	atomic_inc(&mm->mm_count);
	current->active_mm = mm;
	cpumask_set_cpu(cpu, mm_cpumask(mm));

	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));

	cpu_set_reserved_ttbr0();
	flush_tlb_all();

	preempt_disable();
	trace_hardirqs_off();

	if (cpu_ops[cpu]->cpu_postboot)
		cpu_ops[cpu]->cpu_postboot();

	set_cpu_online(cpu, true);
	complete(&cpu_running);

	smp_store_cpu_info(cpu);

	notify_cpu_starting(cpu);

	local_dbg_enable();
	local_irq_enable();
	local_fiq_enable();

	cpu_startup_entry(CPUHP_ONLINE);
}

#ifdef CONFIG_HOTPLUG_CPU
static int op_cpu_disable(unsigned int cpu)
{
	if (!cpu_ops[cpu] || !cpu_ops[cpu]->cpu_die)
		return -EOPNOTSUPP;

	if (cpu_ops[cpu]->cpu_disable)
		return cpu_ops[cpu]->cpu_disable(cpu);

	return 0;
}

int __cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();
	int ret;

	ret = op_cpu_disable(cpu);
	if (ret)
		return ret;

	set_cpu_online(cpu, false);

	migrate_irqs();

	clear_tasks_mm_cpumask(cpu);

	return 0;
}

static int op_cpu_kill(unsigned int cpu)
{
	if (!cpu_ops[cpu]->cpu_kill)
		return 1;

	return cpu_ops[cpu]->cpu_kill(cpu);
}

static DECLARE_COMPLETION(cpu_died);

void __cpu_die(unsigned int cpu)
{
	if (!wait_for_completion_timeout(&cpu_died, msecs_to_jiffies(5000))) {
		pr_crit("CPU%u: cpu didn't die\n", cpu);
		return;
	}
	pr_notice("CPU%u: shutdown\n", cpu);

	if (!op_cpu_kill(cpu))
		pr_warn("CPU%d may not have shut down cleanly\n", cpu);
}

void cpu_die(void)
{
	unsigned int cpu = smp_processor_id();

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	set_cpu_footprint(cpu, 0x7);
#endif
	idle_task_exit();

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	set_cpu_footprint(cpu, 0x8);
#endif
	local_irq_disable();

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	set_cpu_footprint(cpu, 0x9);
#endif
	
	complete(&cpu_died);

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	set_cpu_footprint(cpu, 0xA);
#endif
	cpu_ops[cpu]->cpu_die(cpu);

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	set_cpu_footprint(cpu, 0xB);
#endif
	BUG();
}
#endif

void __init smp_cpus_done(unsigned int max_cpus)
{
	unsigned long bogosum = loops_per_jiffy * num_online_cpus();

	pr_info("SMP: Total of %d processors activated (%lu.%02lu BogoMIPS).\n",
		num_online_cpus(), bogosum / (500000/HZ),
		(bogosum / (5000/HZ)) % 100);
}

void __init smp_prepare_boot_cpu(void)
{
	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));
}

void __init smp_init_cpus(void)
{
	struct device_node *dn = NULL;
	unsigned int i, cpu = 1;
	bool bootcpu_valid = false;

	while ((dn = of_find_node_by_type(dn, "cpu"))) {
		const u32 *cell;
		u64 hwid;

		cell = of_get_property(dn, "reg", NULL);
		if (!cell) {
			pr_err("%s: missing reg property\n", dn->full_name);
			goto next;
		}
		hwid = of_read_number(cell, of_n_addr_cells(dn));

		if (hwid & ~MPIDR_HWID_BITMASK) {
			pr_err("%s: invalid reg property\n", dn->full_name);
			goto next;
		}

		for (i = 1; (i < cpu) && (i < NR_CPUS); i++) {
			if (cpu_logical_map(i) == hwid) {
				pr_err("%s: duplicate cpu reg properties in the DT\n",
					dn->full_name);
				goto next;
			}
		}

		if (hwid == cpu_logical_map(0)) {
			if (bootcpu_valid) {
				pr_err("%s: duplicate boot cpu reg property in DT\n",
					dn->full_name);
				goto next;
			}

			bootcpu_valid = true;

			continue;
		}

		if (cpu >= NR_CPUS)
			goto next;

		if (cpu_read_ops(dn, cpu) != 0)
			goto next;

		if (cpu_ops[cpu]->cpu_init(dn, cpu))
			goto next;

		pr_debug("cpu logical map 0x%llx\n", hwid);
		cpu_logical_map(cpu) = hwid;
next:
		cpu++;
	}

	
	if (cpu > NR_CPUS)
		pr_warning("no. of cores (%d) greater than configured maximum of %d - clipping\n",
			   cpu, NR_CPUS);

	if (!bootcpu_valid) {
		pr_err("DT missing boot CPU MPIDR, not enabling secondaries\n");
		return;
	}

	for (i = 0; i < NR_CPUS; i++)
		if (cpu_logical_map(i) != INVALID_HWID)
			set_cpu_possible(i, true);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	int err;
	unsigned int cpu, ncores = num_possible_cpus();

	init_cpu_topology();

	smp_store_cpu_info(smp_processor_id());


	if (max_cpus > ncores)
		max_cpus = ncores;

	
	if (max_cpus <= 1)
		return;

	max_cpus--;
	for_each_possible_cpu(cpu) {
		if (max_cpus == 0)
			break;

		if (cpu == smp_processor_id())
			continue;

		if (!cpu_ops[cpu])
			continue;

		err = cpu_ops[cpu]->cpu_prepare(cpu);
		if (err)
			continue;

		set_cpu_present(cpu, true);
		max_cpus--;
	}
}

static void (*__smp_cross_call)(const struct cpumask *, unsigned int);

void __init set_smp_cross_call(void (*fn)(const struct cpumask *, unsigned int))
{
	__smp_cross_call = fn;
}

static const char *ipi_types[NR_IPI] __tracepoint_string = {
#define S(x,s)	[x] = s
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
};

static void smp_cross_call(const struct cpumask *target, unsigned int ipinr)
{
	trace_ipi_raise(target, ipi_types[ipinr]);
	__smp_cross_call(target, ipinr);
}

void show_ipi_list(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI; i++) {
		seq_printf(p, "%*s%u:%s", prec - 1, "IPI", i,
			   prec >= 4 ? " " : "");
		for_each_present_cpu(cpu)
			seq_printf(p, "%10u ",
				   __get_irq_stat(cpu, ipi_irqs[i]));
		seq_printf(p, "      %s\n", ipi_types[i]);
	}
}

u64 smp_irq_stat_cpu(unsigned int cpu)
{
	u64 sum = 0;
	int i;

	for (i = 0; i < NR_IPI; i++)
		sum += __get_irq_stat(cpu, ipi_irqs[i]);

	return sum;
}

void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	smp_cross_call(mask, IPI_CALL_FUNC);
}

void arch_send_call_function_single_ipi(int cpu)
{
	smp_cross_call(cpumask_of(cpu), IPI_CALL_FUNC_SINGLE);
}

static DEFINE_RAW_SPINLOCK(stop_lock);

static void ipi_cpu_stop(unsigned int cpu)
{
	if (system_state == SYSTEM_BOOTING ||
	    system_state == SYSTEM_RUNNING) {
		raw_spin_lock(&stop_lock);
		pr_crit("CPU%u: stopping\n", cpu);
		dump_stack();
		raw_spin_unlock(&stop_lock);
	}

	set_cpu_online(cpu, false);

	local_fiq_disable();
	local_irq_disable();

	while (1)
		cpu_relax();
}

void handle_IPI(int ipinr, struct pt_regs *regs)
{
	unsigned int cpu = smp_processor_id();
	struct pt_regs *old_regs = set_irq_regs(regs);

	if ((unsigned)ipinr < NR_IPI) {
		trace_ipi_entry(ipi_types[ipinr]);
		__inc_irq_stat(cpu, ipi_irqs[ipinr]);
	}

	switch (ipinr) {
	case IPI_RESCHEDULE:
		scheduler_ipi();
		break;

	case IPI_CALL_FUNC:
		irq_enter();
        mt_trace_ISR_start(ipinr);
		generic_smp_call_function_interrupt();
        mt_trace_ISR_end(ipinr);
		irq_exit();
		break;

	case IPI_CALL_FUNC_SINGLE:
		irq_enter();
        mt_trace_ISR_start(ipinr);
		generic_smp_call_function_single_interrupt();
        mt_trace_ISR_end(ipinr);
		irq_exit();
		break;

	case IPI_CPU_STOP:
		irq_enter();
        mt_trace_ISR_start(ipinr);
		ipi_cpu_stop(cpu);
        mt_trace_ISR_end(ipinr);
		irq_exit();
		break;

	default:
		pr_crit("CPU%u: Unknown IPI message 0x%x\n", cpu, ipinr);
		break;
	}

	if ((unsigned)ipinr < NR_IPI)
		trace_ipi_exit(ipi_types[ipinr]);
	set_irq_regs(old_regs);
}

void smp_send_reschedule(int cpu)
{
	smp_cross_call(cpumask_of(cpu), IPI_RESCHEDULE);
}

void smp_send_stop(void)
{
	unsigned long timeout;

	if (num_online_cpus() > 1) {
		cpumask_t mask;

		cpumask_copy(&mask, cpu_online_mask);
		cpu_clear(smp_processor_id(), mask);

		smp_cross_call(&mask, IPI_CPU_STOP);
	}

	
	timeout = USEC_PER_SEC;
	while (num_online_cpus() > 1 && timeout--)
		udelay(1);

	if (num_online_cpus() > 1)
		pr_warning("SMP: failed to stop secondary CPUs\n");
}

int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}
