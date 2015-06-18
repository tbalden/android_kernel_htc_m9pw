/*
 * Based on arch/arm/kernel/process.c
 *
 * Original Copyright (C) 1995  Linus Torvalds
 * Copyright (C) 1996-2000 Russell King - Converted to ARM.
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

#include <stdarg.h>

#include <linux/compat.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/user.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/elfcore.h>
#include <linux/pm.h>
#include <linux/tick.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/hw_breakpoint.h>
#include <linux/cpuidle.h>
#include <linux/leds.h>
#include <linux/personality.h>
#include <linux/notifier.h>
#include <linux/mtk_ram_console.h>

#include <asm/compat.h>
#include <asm/cacheflush.h>
#include <asm/fpsimd.h>
#include <asm/mmu_context.h>
#include <asm/processor.h>
#include <asm/stacktrace.h>

extern void arch_reset(char mode, const char *cmd);

static void setup_restart(void)
{
	setup_mm_for_reboot();

	
	flush_cache_all();

	
	cpu_cache_off();

	
	flush_cache_all();
}

void soft_restart(unsigned long addr)
{
	setup_restart();
	cpu_reset(addr);
}

void (*pm_power_off)(void);
EXPORT_SYMBOL_GPL(pm_power_off);

void (*arm_pm_restart)(char str, const char *cmd);
EXPORT_SYMBOL_GPL(arm_pm_restart);


void (*arm_pm_idle)(void);	 

static void default_idle(void)
{
	if (arm_pm_idle)
		arm_pm_idle();
	else
		cpu_do_idle();
	local_irq_enable();
}

void arch_cpu_idle_prepare(void)
{
	local_fiq_enable();
}

void arch_cpu_idle_enter(void)
{
	idle_notifier_call_chain(IDLE_START);
	ledtrig_cpu(CPU_LED_IDLE_START);
#ifdef CONFIG_PL310_ERRATA_769419
	wmb();
#endif
}

void arch_cpu_idle_exit(void)
{
	ledtrig_cpu(CPU_LED_IDLE_END);
	idle_notifier_call_chain(IDLE_END);
}

#ifdef CONFIG_HOTPLUG_CPU
void arch_cpu_idle_dead(void)
{
       cpu_die();
}
#endif

void arch_cpu_idle(void)
{
	if (cpuidle_idle_call())
		default_idle();
}

void machine_shutdown(void)
{
#ifdef CONFIG_SMP
	smp_send_stop();
#endif
}

void machine_halt(void)
{
	machine_shutdown();
	while (1);
}

extern int reboot_pid;

#ifdef CONFIG_HTC_DEBUG_POWERDOWN
extern void set_powerdown_pet_time(unsigned long long time);
#endif
void machine_power_off(void)
{
	struct task_struct *tsk;
#ifdef CONFIG_HTC_DEBUG_POWERDOWN
	struct timespec ts;
#endif
	
	local_irq_disable();
	local_fiq_disable();
	
	machine_shutdown();
	if(reboot_pid > 1)
	{
		tsk = find_task_by_vpid(reboot_pid);
		if(tsk == NULL)
			tsk = current;		
		dump_stack();
	}
	else
	{
		tsk = current;
	}

	if(tsk->real_parent)
	{
	 if(tsk->real_parent->real_parent)
	 {
	   printk("machine_shutdown: start, Proess(%s:%d). father %s:%d. grandfather %s:%d.\n",
		tsk->comm, tsk->pid,tsk->real_parent->comm,tsk->real_parent->pid,
		tsk->real_parent->real_parent->comm,tsk->real_parent->real_parent->pid);
	 }
	 else
	 {
	   printk("machine_shutdown: start, Proess(%s:%d). father %s:%d.\n", 
		tsk->comm, tsk->pid,tsk->real_parent->comm,tsk->real_parent->pid);
	 }
	}
	else
	{
	  printk("machine_shutdown: start, Proess(%s:%d)\n", tsk->comm, tsk->pid);	  
	}

#ifdef CONFIG_HTC_DEBUG_POWERDOWN
	getnstimeofday(&ts);
	set_powerdown_pet_time(ts.tv_sec);
#endif

#ifdef CONFIG_MTK_EMMC_SUPPORT 
	
#endif
	if (pm_power_off)
		pm_power_off();
}

void machine_restart(char *cmd)
{
	struct task_struct *tsk;	

	
	local_irq_disable();
	local_fiq_disable();
	
	machine_shutdown();

	if(reboot_pid > 1)
	{
		tsk = find_task_by_vpid(reboot_pid);
		if(tsk == NULL)
			tsk = current;		
		dump_stack();
	}
	else
	{
		tsk = current;
	}

	if(tsk->real_parent)
	{
	 if(tsk->real_parent->real_parent)
	 {
	   printk("machine_shutdown: start, Proess(%s:%d). father %s:%d. grandfather %s:%d.\n",
		tsk->comm, tsk->pid,tsk->real_parent->comm,tsk->real_parent->pid,
		tsk->real_parent->real_parent->comm,tsk->real_parent->real_parent->pid);
	 }
	 else
	 {
	   printk("machine_shutdown: start, Proess(%s:%d). father %s:%d.\n", 
		tsk->comm, tsk->pid,tsk->real_parent->comm,tsk->real_parent->pid);
	 }
	}
	else
	{
	  printk("machine_shutdown: start, Proess(%s:%d)\n", tsk->comm, tsk->pid);	  
	}

	
	if (arm_pm_restart)
		arm_pm_restart('h', cmd);
#ifndef CONFIG_MTK_FPGA
	else
		arch_reset('h', cmd);
#endif
	printk("Reboot failed -- System halted\n");
	while (1);
}

static void show_data(unsigned long addr, int nbytes, const char *name)
{
	int	i, j;
	int	nlines;
	u32	*p;

	if (addr < PAGE_OFFSET || addr > -256UL)
		return;

	printk("\n%s: %#lx:\n", name, addr);

	p = (u32 *)(addr & ~(sizeof(u32) - 1));
	nbytes += (addr & (sizeof(u32) - 1));
	nlines = (nbytes + 31) / 32;


	for (i = 0; i < nlines; i++) {
		printk("%04lx ", (unsigned long)p & 0xffff);
		for (j = 0; j < 8; j++) {
			u32	data;
			if (probe_kernel_address(p, data)) {
				printk(" ********");
			} else {
				printk(" %08x", data);
			}
			++p;
		}
		printk("\n");
	}
}

static void show_extra_register_data(struct pt_regs *regs, int nbytes)
{
	mm_segment_t fs;
	unsigned int i;

	fs = get_fs();
	set_fs(KERNEL_DS);
	show_data(regs->pc - nbytes, nbytes * 2, "PC");
	show_data(regs->regs[30] - nbytes, nbytes * 2, "LR");
	show_data(regs->sp - nbytes, nbytes * 2, "SP");
	for (i = 0; i < 30; i++) {
		char name[4];
		snprintf(name, sizeof(name), "X%u", i);
		show_data(regs->regs[i] - nbytes, nbytes * 2, name);
	}
	set_fs(fs);
}

void __show_regs(struct pt_regs *regs)
{
	int i;

	show_regs_print_info(KERN_DEFAULT);
	print_symbol("PC is at %s\n", instruction_pointer(regs));
	print_symbol("LR is at %s\n", regs->regs[30]);
	printk("pc : [<%016llx>] lr : [<%016llx>] pstate: %08llx\n",
	       regs->pc, regs->regs[30], regs->pstate);
	printk("sp : %016llx\n", regs->sp);
	for (i = 29; i >= 0; i--) {
		printk("x%-2d: %016llx ", i, regs->regs[i]);
		if (i % 2 == 0)
			printk("\n");
	}
	if (!user_mode(regs))
		show_extra_register_data(regs, 128);
	printk("\n");
}

void show_regs(struct pt_regs * regs)
{
	printk("\n");
	__show_regs(regs);
}

void exit_thread(void)
{
}

static void tls_thread_flush(void)
{
	asm ("msr tpidr_el0, xzr");

	if (is_compat_task()) {
		current->thread.tp_value = 0;

		barrier();
		asm ("msr tpidrro_el0, xzr");
	}
}

void flush_thread(void)
{
	fpsimd_flush_thread();
	tls_thread_flush();
	flush_ptrace_hw_breakpoint(current);
}

void release_thread(struct task_struct *dead_task)
{
}

int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src)
{
	fpsimd_preserve_current_state();
	*dst = *src;
	return 0;
}

asmlinkage void ret_from_fork(void) asm("ret_from_fork");

int copy_thread(unsigned long clone_flags, unsigned long stack_start,
		unsigned long stk_sz, struct task_struct *p)
{
	struct pt_regs *childregs = task_pt_regs(p);
	unsigned long tls = p->thread.tp_value;

	memset(&p->thread.cpu_context, 0, sizeof(struct cpu_context));

	if (likely(!(p->flags & PF_KTHREAD))) {
		*childregs = *current_pt_regs();
		childregs->regs[0] = 0;
		if (is_compat_thread(task_thread_info(p))) {
			if (stack_start)
				childregs->compat_sp = stack_start;
		} else {
			asm("mrs %0, tpidr_el0" : "=r" (tls));
			if (stack_start) {
				
				if (stack_start & 15)
					return -EINVAL;
				childregs->sp = stack_start;
			}
		}
		if (clone_flags & CLONE_SETTLS)
			tls = childregs->regs[3];
	} else {
		memset(childregs, 0, sizeof(struct pt_regs));
		childregs->pstate = PSR_MODE_EL1h;
		p->thread.cpu_context.x19 = stack_start;
		p->thread.cpu_context.x20 = stk_sz;
	}
	p->thread.cpu_context.pc = (unsigned long)ret_from_fork;
	p->thread.cpu_context.sp = (unsigned long)childregs;
	p->thread.tp_value = tls;

	ptrace_hw_copy_thread(p);

	return 0;
}

static void tls_thread_switch(struct task_struct *next)
{
	unsigned long tpidr, tpidrro;

	if (!is_compat_task()) {
		asm("mrs %0, tpidr_el0" : "=r" (tpidr));
		current->thread.tp_value = tpidr;
	}

	if (is_compat_thread(task_thread_info(next))) {
		tpidr = 0;
		tpidrro = next->thread.tp_value;
	} else {
		tpidr = next->thread.tp_value;
		tpidrro = 0;
	}

	asm(
	"	msr	tpidr_el0, %0\n"
	"	msr	tpidrro_el0, %1"
	: : "r" (tpidr), "r" (tpidrro));
}

struct task_struct *__switch_to(struct task_struct *prev,
				struct task_struct *next)
{
	struct task_struct *last;

	fpsimd_thread_switch(next);
	tls_thread_switch(next);
	hw_breakpoint_thread_switch(next);
	contextidr_thread_switch(next);

	dsb();

	
	last = cpu_switch_to(prev, next);

	return last;
}

unsigned long get_wchan(struct task_struct *p)
{
	struct stackframe frame;
	int count = 0;
	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;

	frame.fp = thread_saved_fp(p);
	frame.sp = thread_saved_sp(p);
	frame.pc = thread_saved_pc(p);
	do {
		int ret = unwind_frame(&frame);
		if (ret < 0)
			return 0;
		if (!in_sched_functions(frame.pc))
			return frame.pc;
	} while (count ++ < 16);
	return 0;
}

unsigned long arch_align_stack(unsigned long sp)
{
	if (!(current->personality & ADDR_NO_RANDOMIZE) && randomize_va_space)
		sp -= get_random_int() & ~PAGE_MASK;
	return sp & ~0xf;
}

static unsigned long randomize_base(unsigned long base)
{
	unsigned long range_end = base + (STACK_RND_MASK << PAGE_SHIFT) + 1;
	return randomize_range(base, range_end, 0) ? : base;
}

unsigned long arch_randomize_brk(struct mm_struct *mm)
{
	return randomize_base(mm->brk);
}

unsigned long randomize_et_dyn(unsigned long base)
{
	return randomize_base(base);
}
