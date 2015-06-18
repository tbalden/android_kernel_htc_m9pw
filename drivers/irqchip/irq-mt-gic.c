#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/irqchip/mt-gic.h>

#include <asm/irq.h>
#include <asm/exception.h>
#include <asm/smp_plat.h>

#include "irqchip.h"
#include <mach/mt_secure_api.h>


#if defined(CONFIG_HTC_DEBUG_RTB)
#include <linux/msm_rtb.h>
#endif

#include <linux/htc_debug_tools.h>

union gic_base
{
	void __iomem *common_base;
	void __percpu __iomem **percpu_base;
};

struct gic_chip_data
{
	union gic_base dist_base;
	union gic_base cpu_base;
#ifdef CONFIG_CPU_PM
	u32 saved_spi_enable[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(1020, 16)];
	u32 saved_spi_target[DIV_ROUND_UP(1020, 4)];
	u32 __percpu *saved_ppi_enable;
	u32 __percpu *saved_ppi_conf;
#endif
	struct irq_domain *domain;
	unsigned int gic_irqs;
#ifdef CONFIG_GIC_NON_BANKED
	void __iomem *(*get_base)(union gic_base *);
#endif
};


void (*irq_pol_workaround)(phys_addr_t addr, u32 value);



static DEFINE_RAW_SPINLOCK(irq_controller_lock);

#define NR_GIC_CPU_IF 8
static u8 gic_cpu_map[NR_GIC_CPU_IF] __read_mostly;


#ifndef MAX_GIC_NR
#define MAX_GIC_NR	1
#endif

static struct gic_chip_data gic_data[MAX_GIC_NR] __read_mostly;

#ifdef CONFIG_GIC_NON_BANKED
static void __iomem *gic_get_percpu_base(union gic_base *base)
{
	return *__this_cpu_ptr(base->percpu_base);
}

static void __iomem *gic_get_common_base(union gic_base *base)
{
	return base->common_base;
}

static inline void __iomem *gic_data_dist_base(struct gic_chip_data *data)
{
	return data->get_base(&data->dist_base);
}

static inline void __iomem *gic_data_cpu_base(struct gic_chip_data *data)
{
	return data->get_base(&data->cpu_base);
}

static inline void gic_set_base_accessor(struct gic_chip_data *data,
		void __iomem *(*f)(union gic_base *))
{
	data->get_base = f;
}
#else
#define gic_data_dist_base(d)	((d)->dist_base.common_base)
#define gic_data_cpu_base(d)	((d)->cpu_base.common_base)
#define gic_set_base_accessor(d, f)
#endif

static inline void __iomem *gic_dist_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_dist_base(gic_data);
}

static inline void __iomem *gic_cpu_base(struct irq_data *d)
{
	struct gic_chip_data *gic_data = irq_data_get_irq_chip_data(d);
	return gic_data_cpu_base(gic_data);
}

static inline unsigned int gic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static void gic_mask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_CLEAR + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_unmask_irq(struct irq_data *d)
{
	u32 mask = 1 << (gic_irq(d) % 32);

	raw_spin_lock(&irq_controller_lock);
	writel_relaxed(mask, gic_dist_base(d) + GIC_DIST_ENABLE_SET + (gic_irq(d) / 32) * 4);
	raw_spin_unlock(&irq_controller_lock);
}

static void gic_eoi_irq(struct irq_data *d)
{
	writel_relaxed(gic_irq(d), gic_cpu_base(d) + GIC_CPU_EOI);
}


void __iomem *GIC_DIST_BASE;
void __iomem *GIC_CPU_BASE;
void __iomem *INT_POL_CTL0;
phys_addr_t INT_POL_CTL0_phys;

__weak void mt_set_pol_reg(u32 reg_index, u32 value)
{
	writel_relaxed(value, (INT_POL_CTL0 + (reg_index * 4)));
}

void mt_irq_set_polarity(unsigned int irq, unsigned int polarity)
{
	u32 offset, reg_index, value;

	if (irq < 32)
	{
		printk(KERN_CRIT "Fail to set polarity of interrupt %d\n", irq);
		return ;
	}

	offset = (irq - 32) & 0x1F;
	reg_index = (irq - 32) >> 5;

	

	if (polarity == 0)
	{
		
		value = readl_relaxed(IOMEM(INT_POL_CTL0 + (reg_index * 4)));
		value |= (1 << offset);
		
		mt_set_pol_reg(reg_index, value);
	}
	else
	{
		
		value = readl_relaxed(IOMEM(INT_POL_CTL0 + (reg_index * 4)));
		value &= ~(0x1 << offset);
		
		mt_set_pol_reg(reg_index, value);
 	}

	
}

static int gic_set_type(struct irq_data *d, unsigned int type)
{
	void __iomem *base = gic_dist_base(d);
	unsigned int gicirq = gic_irq(d);
	u32 enablemask = 1 << (gicirq % 32);
	u32 enableoff = (gicirq / 32) * 4;
	u32 confmask = 0x2 << ((gicirq % 16) * 2);
	u32 confoff = (gicirq / 16) * 4;
	bool enabled = false;
	u32 val;

	
	if (gicirq < 16)
		return -EINVAL;

	
	

	raw_spin_lock(&irq_controller_lock);

	val = readl_relaxed(base + GIC_DIST_CONFIG + confoff);
	if ((type == IRQ_TYPE_LEVEL_HIGH) || (type == IRQ_TYPE_LEVEL_LOW)) {
		val &= ~confmask;
	} else if ((type == IRQ_TYPE_EDGE_RISING) || (type == IRQ_TYPE_EDGE_FALLING)) {
		val |= confmask;
	} else {
		pr_err("[GIC] not correct trigger type(0x%x)\n", type);
		dump_stack();
		raw_spin_unlock(&irq_controller_lock);
		return -EINVAL;
	}

	if (readl_relaxed(base + GIC_DIST_ENABLE_SET + enableoff) & enablemask)
	{
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_CLEAR + enableoff);
		enabled = true;
	}

	writel_relaxed(val, base + GIC_DIST_CONFIG + confoff);

	if (enabled)
		writel_relaxed(enablemask, base + GIC_DIST_ENABLE_SET + enableoff);

	
	if (type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING))
	{
		mt_irq_set_polarity(gicirq, (type & IRQF_TRIGGER_FALLING) ? 0 : 1);
	}
	else if (type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW))
	{
		mt_irq_set_polarity(gicirq, (type & IRQF_TRIGGER_LOW) ? 0 : 1);
	}

	raw_spin_unlock(&irq_controller_lock);

	return 0;
}

static int gic_retrigger(struct irq_data *d)
{
	
	return 0;
}

#ifdef CONFIG_SMP
static int gic_set_affinity(struct irq_data *d, const struct cpumask *mask_val,
							bool force)
{
	void __iomem *reg = gic_dist_base(d) + GIC_DIST_TARGET + (gic_irq(d) & ~3);
	unsigned int cpu, shift = (gic_irq(d) % 4) * 8;
	u32 val, mask, bit;

	if (!force)
		cpu = cpumask_any_and(mask_val, cpu_online_mask);
	else
		cpu = cpumask_first(mask_val);

	if (cpu >= NR_GIC_CPU_IF || cpu >= nr_cpu_ids)
		return -EINVAL;

	mask = 0xff << shift;
	bit = gic_cpu_map[cpu] << shift;

	raw_spin_lock(&irq_controller_lock);
	val = readl_relaxed(reg) & ~mask;
	writel_relaxed(val | bit, reg);
	raw_spin_unlock(&irq_controller_lock);

	return IRQ_SET_MASK_OK;
}
#endif

#ifdef CONFIG_PM
static int gic_set_wake(struct irq_data *d, unsigned int on)
{
	int ret = -ENXIO;

	return ret;
}

#else
#define gic_set_wake	NULL
#endif

static asmlinkage void __exception_irq_entry gic_handle_irq(struct pt_regs *regs)
{
	u32 irqstat, irqnr;
	struct gic_chip_data *gic = &gic_data[0];
	void __iomem *cpu_base = gic_data_cpu_base(gic);

	do
	{
		irqstat = readl_relaxed(cpu_base + GIC_CPU_INTACK);
		irqnr = irqstat & ~0x1c00;

		if (likely(irqnr > 15 && irqnr < 1021))
		{
			
			unsigned long long timestamp = sched_clock();
			if (irqnr == 30 && smp_processor_id() == 0) {
#if defined(CONFIG_HTC_DEBUG_RTB)
				unsigned long long timestamp_ms = timestamp;
				do_div(timestamp_ms, NSEC_PER_MSEC);
				uncached_logk_pc(LOGK_IRQ, (void *)((uintptr_t)timestamp_ms), (void *)(uintptr_t)irqnr);
#endif	

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
				htc_debug_watchdog_check_pet(timestamp);
#endif
			}
#if defined(CONFIG_HTC_DEBUG_RTB)
			else
			{
				uncached_logk_pc(LOGK_IRQ, (void *)(uintptr_t)htc_debug_get_sched_clock_ms(), (void *)(uintptr_t)irqnr);
			}
#endif
			irqnr = irq_find_mapping(gic->domain, irqnr);
			handle_IRQ(irqnr, regs);
			continue;
		}
		if (irqnr < 16)
		{
#if defined(CONFIG_HTC_DEBUG_RTB)
			uncached_logk_pc(LOGK_IRQ, (void *)(uintptr_t)htc_debug_get_sched_clock_ms(), (void *)(uintptr_t)irqnr);
#endif
			writel_relaxed(irqstat, cpu_base + GIC_CPU_EOI);
#ifdef CONFIG_SMP
			handle_IPI(irqnr, regs);
#endif
			continue;
		}
		break;
	}
	while (1);
}

static void gic_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct gic_chip_data *chip_data = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned int cascade_irq, gic_irq;
	unsigned long status;

	chained_irq_enter(chip, desc);

	raw_spin_lock(&irq_controller_lock);
	status = readl_relaxed(gic_data_cpu_base(chip_data) + GIC_CPU_INTACK);
	raw_spin_unlock(&irq_controller_lock);

	gic_irq = (status & 0x3ff);
	if (gic_irq == 1023)
		goto out;

	cascade_irq = irq_find_mapping(chip_data->domain, gic_irq);
	if (unlikely(gic_irq < 32 || gic_irq > 1020))
		handle_bad_irq(cascade_irq, desc);
	else
		generic_handle_irq(cascade_irq);

out:
	chained_irq_exit(chip, desc);
}

static struct irq_chip gic_chip =
{
	.name			= "GIC",
	.irq_mask		= gic_mask_irq,
	.irq_unmask		= gic_unmask_irq,
	.irq_eoi		= gic_eoi_irq,
	.irq_set_type		= gic_set_type,
	.irq_retrigger		= gic_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= gic_set_affinity,
#endif
	.irq_set_wake		= gic_set_wake,
};

void __init mt_gic_cascade_irq(unsigned int gic_nr, unsigned int irq)
{
	if (gic_nr >= MAX_GIC_NR)
		BUG();
	if (irq_set_handler_data(irq, &gic_data[gic_nr]) != 0)
		BUG();
	irq_set_chained_handler(irq, gic_handle_cascade_irq);
}


static void __init gic_dist_init(struct gic_chip_data *gic)
{
	unsigned int i;
	u32 cpumask;
	unsigned int gic_irqs = gic->gic_irqs;
	void __iomem *base = gic_data_dist_base(gic);

	writel_relaxed(0, base + GIC_DIST_CTRL);

	for (i = 32; i < gic_irqs; i += 16)
		writel_relaxed(0, base + GIC_DIST_CONFIG + i * 4 / 16);

	
	
	cpumask = 1 << smp_processor_id();
	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;
	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);

	for (i = 32; i < gic_irqs; i += 4)
		writel_relaxed(0xa0a0a0a0, base + GIC_DIST_PRI + i * 4 / 4);

	for (i = 32; i < gic_irqs; i += 32)
		writel_relaxed(0xffffffff, base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);

	writel_relaxed(1, base + GIC_DIST_CTRL);
}

static void __cpuinit gic_cpu_init(struct gic_chip_data *gic)
{
	void __iomem *dist_base = gic_data_dist_base(gic);
	void __iomem *base = gic_data_cpu_base(gic);
	unsigned int cpu_mask, cpu = smp_processor_id();
	int i;

	BUG_ON(cpu >= NR_GIC_CPU_IF);
	
	
	cpu_mask = 1 << smp_processor_id();
	gic_cpu_map[cpu] = cpu_mask;

	for (i = 0; i < NR_GIC_CPU_IF; i++)
		if (i != cpu)
			gic_cpu_map[i] &= ~cpu_mask;

	writel_relaxed(0xffff0000, dist_base + GIC_DIST_ENABLE_CLEAR);
	writel_relaxed(0x0000ffff, dist_base + GIC_DIST_ENABLE_SET);

	for (i = 0; i < 32; i += 4)
		writel_relaxed(0xa0a0a0a0, dist_base + GIC_DIST_PRI + i * 4 / 4);

	writel_relaxed(0xf0, base + GIC_CPU_PRIMASK);
	writel_relaxed(1, base + GIC_CPU_CTRL);
}

#ifdef CONFIG_CPU_PM
static void gic_dist_save(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	void __iomem *dist_base;
	int i;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		gic_data[gic_nr].saved_spi_conf[i] =
			readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		gic_data[gic_nr].saved_spi_target[i] =
			readl_relaxed(dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		gic_data[gic_nr].saved_spi_enable[i] =
			readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);
}

static void gic_dist_restore(unsigned int gic_nr)
{
	unsigned int gic_irqs;
	unsigned int i;
	void __iomem *dist_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	gic_irqs = gic_data[gic_nr].gic_irqs;
	dist_base = gic_data_dist_base(&gic_data[gic_nr]);

	if (!dist_base)
		return;

	writel_relaxed(0, dist_base + GIC_DIST_CTRL);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 16); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_conf[i],
					   dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(0xa0a0a0a0,
					   dist_base + GIC_DIST_PRI + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 4); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_target[i],
					   dist_base + GIC_DIST_TARGET + i * 4);

	for (i = 0; i < DIV_ROUND_UP(gic_irqs, 32); i++)
		writel_relaxed(gic_data[gic_nr].saved_spi_enable[i],
					   dist_base + GIC_DIST_ENABLE_SET + i * 4);

	writel_relaxed(1, dist_base + GIC_DIST_CTRL);
}

static void gic_cpu_save(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		ptr[i] = readl_relaxed(dist_base + GIC_DIST_CONFIG + i * 4);

}

static void gic_cpu_restore(unsigned int gic_nr)
{
	int i;
	u32 *ptr;
	void __iomem *dist_base;
	void __iomem *cpu_base;

	if (gic_nr >= MAX_GIC_NR)
		BUG();

	dist_base = gic_data_dist_base(&gic_data[gic_nr]);
	cpu_base = gic_data_cpu_base(&gic_data[gic_nr]);

	if (!dist_base || !cpu_base)
		return;

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_enable);
	for (i = 0; i < DIV_ROUND_UP(32, 32); i++)
		writel_relaxed(ptr[i], dist_base + GIC_DIST_ENABLE_SET + i * 4);

	ptr = __this_cpu_ptr(gic_data[gic_nr].saved_ppi_conf);
	for (i = 0; i < DIV_ROUND_UP(32, 16); i++)
		writel_relaxed(ptr[i], dist_base + GIC_DIST_CONFIG + i * 4);

	for (i = 0; i < DIV_ROUND_UP(32, 4); i++)
		writel_relaxed(0xa0a0a0a0, dist_base + GIC_DIST_PRI + i * 4);

	writel_relaxed(0xf0, cpu_base + GIC_CPU_PRIMASK);
	writel_relaxed(1, cpu_base + GIC_CPU_CTRL);
}

static int gic_notifier(struct notifier_block *self, unsigned long cmd,	void *v)
{
	int i;

	for (i = 0; i < MAX_GIC_NR; i++)
	{
#ifdef CONFIG_GIC_NON_BANKED
		
		if (!gic_data[i].get_base)
			continue;
#endif
		switch (cmd)
		{
		case CPU_PM_ENTER:
			gic_cpu_save(i);
			break;
		case CPU_PM_ENTER_FAILED:
		case CPU_PM_EXIT:
			gic_cpu_restore(i);
			break;
		case CPU_CLUSTER_PM_ENTER:
			gic_dist_save(i);
			break;
		case CPU_CLUSTER_PM_ENTER_FAILED:
		case CPU_CLUSTER_PM_EXIT:
			gic_dist_restore(i);
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block gic_notifier_block =
{
	.notifier_call = gic_notifier,
};

static void __init gic_pm_init(struct gic_chip_data *gic)
{
	gic->saved_ppi_enable = __alloc_percpu(DIV_ROUND_UP(32, 32) * 4,
										   sizeof(u32));
	BUG_ON(!gic->saved_ppi_enable);

	gic->saved_ppi_conf = __alloc_percpu(DIV_ROUND_UP(32, 16) * 4,
										 sizeof(u32));
	BUG_ON(!gic->saved_ppi_conf);

	if (gic == &gic_data[0])
		cpu_pm_register_notifier(&gic_notifier_block);
}
#else
static void __init gic_pm_init(struct gic_chip_data *gic)
{
}
#endif

#ifdef CONFIG_SMP
void mt_gic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	int cpu;
	unsigned long map = 0;

	
	for_each_cpu(cpu, mask)
	map |= gic_cpu_map[cpu];

	dsb();

	
	writel_relaxed(map << 16 | irq, gic_data_dist_base(&gic_data[0]) + GIC_DIST_SOFTINT);
}
#endif

static int gic_irq_domain_map(struct irq_domain *d, unsigned int irq,
							  irq_hw_number_t hw)
{
	if (hw < 32)
	{
		irq_set_percpu_devid(irq);
		irq_set_chip_and_handler(irq, &gic_chip,
								 handle_percpu_devid_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
	}
	else
	{
		irq_set_chip_and_handler(irq, &gic_chip,
								 handle_fasteoi_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}
	irq_set_chip_data(irq, d->host_data);
	return 0;
}

static int gic_irq_domain_xlate(struct irq_domain *d,
								struct device_node *controller,
								const u32 *intspec, unsigned int intsize,
								unsigned long *out_hwirq, unsigned int *out_type)
{
	if (d->of_node != controller)
		return -EINVAL;
	if (intsize < 3)
		return -EINVAL;

	
	*out_hwirq = intspec[1] + 16;

	
	if (!intspec[0])
		*out_hwirq += 16;

	*out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

void mt_gic_register_sgi(unsigned int gic_nr, int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	if (desc)
		desc->irq_data.hwirq = irq;
	irq_set_chip_and_handler(irq, &gic_chip,
							 handle_fasteoi_irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	irq_set_chip_data(irq, &gic_data[gic_nr]);
}

#ifdef CONFIG_SMP
static int __cpuinit gic_secondary_init(struct notifier_block *nfb,
										unsigned long action, void *hcpu)
{
	if (action == CPU_STARTING || action == CPU_STARTING_FROZEN)
		gic_cpu_init(&gic_data[0]);
	return NOTIFY_OK;
}

static struct notifier_block __cpuinitdata gic_cpu_notifier =
{
	.notifier_call = gic_secondary_init,
	.priority = 100,
};
#endif

const struct irq_domain_ops mt_gic_irq_domain_ops =
{
	.map = gic_irq_domain_map,
	.xlate = gic_irq_domain_xlate,
};

void __init mt_gic_init_bases(unsigned int gic_nr, int irq_start,
							  void __iomem *dist_base, void __iomem *cpu_base,
							  u32 percpu_offset, struct device_node *node)
{
	irq_hw_number_t hwirq_base;
	struct gic_chip_data *gic;
	int gic_irqs, irq_base, i;

	BUG_ON(gic_nr >= MAX_GIC_NR);

	gic = &gic_data[gic_nr];
#ifdef CONFIG_GIC_NON_BANKED
	if (percpu_offset)   
	{
		unsigned int cpu;

		gic->dist_base.percpu_base = alloc_percpu(void __iomem *);
		gic->cpu_base.percpu_base = alloc_percpu(void __iomem *);
		if (WARN_ON(!gic->dist_base.percpu_base ||
					!gic->cpu_base.percpu_base))
		{
			free_percpu(gic->dist_base.percpu_base);
			free_percpu(gic->cpu_base.percpu_base);
			return;
		}

		for_each_possible_cpu(cpu)
		{
			unsigned long offset = percpu_offset * cpu_logical_map(cpu);
			*per_cpu_ptr(gic->dist_base.percpu_base, cpu) = dist_base + offset;
			*per_cpu_ptr(gic->cpu_base.percpu_base, cpu) = cpu_base + offset;
		}

		gic_set_base_accessor(gic, gic_get_percpu_base);
	}
	else
#endif
	{
		
		WARN(percpu_offset,
			 "GIC_NON_BANKED not enabled, ignoring %08x offset!",
			 percpu_offset);
		gic->dist_base.common_base = dist_base;
		gic->cpu_base.common_base = cpu_base;
		gic_set_base_accessor(gic, gic_get_common_base);
	}

	for (i = 0; i < NR_GIC_CPU_IF; i++)
		gic_cpu_map[i] = 0xff;

	if (gic_nr == 0 && (irq_start & 31) > 0)
	{
		hwirq_base = 16;
		if (irq_start != -1)
			irq_start = (irq_start & ~31) + 16;
	}
	else
	{
		hwirq_base = 32;
	}

	gic_irqs = readl_relaxed(gic_data_dist_base(gic) + GIC_DIST_CTR) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	gic->gic_irqs = gic_irqs;

	gic_irqs -= hwirq_base; 
	irq_base = irq_alloc_descs(irq_start, 16, gic_irqs, numa_node_id());
	if (IS_ERR_VALUE(irq_base))
	{
		WARN(1, "Cannot allocate irq_descs @ IRQ%d, assuming pre-allocated\n",
			 irq_start);
		irq_base = irq_start;
	}
	gic->domain = irq_domain_add_legacy(node, gic_irqs, irq_base,
										hwirq_base, &mt_gic_irq_domain_ops, gic);
	if (WARN_ON(!gic->domain))
		return;

#ifdef CONFIG_SMP
	set_smp_cross_call(mt_gic_raise_softirq);
	register_cpu_notifier(&gic_cpu_notifier);
#endif

	set_handle_irq(gic_handle_irq);

	gic_dist_init(gic);
	gic_cpu_init(gic);
	gic_pm_init(gic);
}



static spinlock_t irq_lock;

int mt_irq_mask_all(struct mtk_irq_mask *mask)
{
	unsigned long flags;
	void __iomem *dist_base;

	dist_base = gic_data_dist_base(&gic_data[0]);

	if (mask)
	{
		spin_lock_irqsave(&irq_lock, flags);

		mask->mask0 = readl((dist_base + GIC_DIST_ENABLE_SET));
		mask->mask1 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x4));
		mask->mask2 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x8));
		mask->mask3 = readl((dist_base + GIC_DIST_ENABLE_SET + 0xC));
		mask->mask4 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x10));
		mask->mask5 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x14));
		mask->mask6 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x18));
		mask->mask7 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x1C));
		mask->mask8 = readl((dist_base + GIC_DIST_ENABLE_SET + 0x20));

		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x4));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x8));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0xC));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x10));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x14));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x18));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x1C));
		writel(0xFFFFFFFF, (dist_base + GIC_DIST_ENABLE_CLEAR + 0x20));
		dsb();

		spin_unlock_irqrestore(&irq_lock, flags);

		mask->header = IRQ_MASK_HEADER;
		mask->footer = IRQ_MASK_FOOTER;

		return 0;
	}
	else
	{
		return -1;
	}
}

int mt_irq_mask_restore(struct mtk_irq_mask *mask)
{
	unsigned long flags;
	void __iomem *dist_base;

	dist_base = gic_data_dist_base(&gic_data[0]);

	if (!mask)
	{
		return -1;
	}
	if (mask->header != IRQ_MASK_HEADER)
	{
		return -1;
	}
	if (mask->footer != IRQ_MASK_FOOTER)
	{
		return -1;
	}

	spin_lock_irqsave(&irq_lock, flags);

	writel(mask->mask0, (dist_base + GIC_DIST_ENABLE_SET));
	writel(mask->mask1, (dist_base + GIC_DIST_ENABLE_SET + 0x4));
	writel(mask->mask2, (dist_base + GIC_DIST_ENABLE_SET + 0x8));
	writel(mask->mask3, (dist_base + GIC_DIST_ENABLE_SET + 0xC));
	writel(mask->mask4, (dist_base + GIC_DIST_ENABLE_SET + 0x10));
	writel(mask->mask5, (dist_base + GIC_DIST_ENABLE_SET + 0x14));
	writel(mask->mask6, (dist_base + GIC_DIST_ENABLE_SET + 0x18));
	writel(mask->mask7, (dist_base + GIC_DIST_ENABLE_SET + 0x1C));
	writel(mask->mask8, (dist_base + GIC_DIST_ENABLE_SET + 0x20));
	dsb();

	spin_unlock_irqrestore(&irq_lock, flags);

	return 0;
}

void mt_irq_set_pending_for_sleep(unsigned int irq)
{
	void __iomem *dist_base;
	u32 mask = 1 << (irq % 32);

	dist_base = gic_data_dist_base(&gic_data[0]);

	if (irq < 16)
	{
		pr_err("Fail to set a pending on interrupt %d\n", irq);
		return ;
	}

	*(volatile u32 *)(dist_base + GIC_DIST_PENDING_SET + irq / 32 * 4) = mask;
	pr_notice(KERN_CRIT "irq:%d, 0x%p=0x%x\n", irq, dist_base + GIC_DIST_PENDING_SET + irq / 32 * 4,mask);
	dsb();
}

void mt_irq_unmask_for_sleep(unsigned int irq)
{
	void __iomem *dist_base;
	u32 mask = 1 << (irq % 32);

	dist_base = gic_data_dist_base(&gic_data[0]);

	if (irq < 16)
	{
		pr_err(KERN_CRIT "Fail to enable interrupt %d\n", irq);
		return ;
	}

	*(volatile u32 *)(dist_base + GIC_DIST_ENABLE_SET + irq / 32 * 4) = mask;
	dsb();
}

void mt_irq_mask_for_sleep(unsigned int irq)
{
	void __iomem *dist_base;
	u32 mask = 1 << (irq % 32);

	dist_base = gic_data_dist_base(&gic_data[0]);

	if (irq < 16)
	{
		pr_err(KERN_CRIT "Fail to enable interrupt %d\n", irq);
		return ;
	}

	*(volatile u32 *)(dist_base + GIC_DIST_ENABLE_CLEAR + irq / 32 * 4) = mask;
	dsb();
}

void mt_irq_set_sens(unsigned int irq, unsigned int sens)
{
    unsigned long flags;
    u32 config;

    if (irq < 32) {
        pr_err("Fail to set sensitivity of interrupt %d\n", irq);
        return ;
    }

    spin_lock_irqsave(&irq_lock, flags);

    if (sens == MT_EDGE_SENSITIVE) {
        config = readl(GIC_DIST_BASE + GIC_DIST_CONFIG + (irq / 16) * 4);
        config |= (0x2 << (irq % 16) * 2);
        writel(config, GIC_DIST_BASE + GIC_DIST_CONFIG + (irq / 16) * 4);
    } else {
        config = readl(GIC_DIST_BASE + GIC_DIST_CONFIG + (irq / 16) * 4);
        config &= ~(0x2 << (irq % 16) * 2);
        writel(config, GIC_DIST_BASE + GIC_DIST_CONFIG + (irq / 16) * 4);
    }

    spin_unlock_irqrestore(&irq_lock, flags);

    dsb();
}

u32 mt_irq_get_pending(unsigned int irq)
{
        void __iomem *dist_base;
        u32 bit = 1 << (irq % 32);

        dist_base = gic_data_dist_base(&gic_data[0]);

        return ((readl_relaxed(dist_base + GIC_DIST_PENDING_SET + irq / 32 * 4) & bit)? 1 : 0);
}

void mt_irq_dump_status(int irq)
{
	int rc;
	unsigned int result;

	pr_notice("[mt gic dump] irq = %d\n", irq);
	
	rc = mt_secure_call(MTK_SIP_KERNEL_GIC_DUMP, irq, 0, 0);
	if(rc < 0)
	{
		pr_notice("[mt gic dump] not allowed to dump!\n");
		return;
	}	
	
	
	result = rc & 0x1;
	pr_notice("[mt gic dump] enable = %d\n", result);

	
	result = (rc >> 1) & 0x1;
	pr_notice("[mt gic dump] group = %x (0x1:irq,0x0:fiq)\n", result);

	
	result = (rc >> 2) & 0xff;
	pr_notice("[mt gic dump] priority = %x\n", result);
	
	
	result = (rc >> 10) & 0x1;
	pr_notice("[mt gic dump] sensitivity = %x (edge:0x1, level:0x0)\n", result);

	
	result = (rc >> 11) & 0x1;
	pr_notice("[mt gic dump] pending = %x\n", result);

	
	result = (rc >> 12) & 0x1;
	pr_notice("[mt gic dump] active status = %x\n", result);

	
	result = (rc >> 13) & 0x1;
	pr_notice("[mt gic dump] polarity = %x (0x0: high, 0x1:low)\n", result);
	
}


#ifdef CONFIG_OF
static int gic_cnt __initdata;

int __init mt_gic_of_init(struct device_node *node, struct device_node *parent)
{
	void __iomem *cpu_base;
	void __iomem *dist_base;
	void __iomem *pol_base;
	u32 percpu_offset;
	int irq;
	struct resource res;

	if (WARN_ON(!node))
		return -ENODEV;

	spin_lock_init(&irq_lock);

	dist_base = of_iomap(node, 0);
	WARN(!dist_base, "unable to map gic dist registers\n");
	GIC_DIST_BASE = dist_base;

	cpu_base = of_iomap(node, 1);
	WARN(!cpu_base, "unable to map gic cpu registers\n");
	GIC_CPU_BASE = cpu_base;

	pol_base = of_iomap(node, 2);
	WARN(!pol_base, "unable to map pol registers\n");
	INT_POL_CTL0 = pol_base;
	if (of_address_to_resource(node, 2, &res))
	{
		WARN(!pol_base, "unable to map pol registers\n");
	}
	INT_POL_CTL0_phys = res.start;

	if (of_property_read_u32(node, "cpu-offset", &percpu_offset))
		percpu_offset = 0;

	mt_gic_init_bases(gic_cnt, -1, dist_base, cpu_base, percpu_offset, node);

	if (parent)
	{
		irq = irq_of_parse_and_map(node, 0);
		mt_gic_cascade_irq(gic_cnt, irq);
	}
	gic_cnt++;

	
	
	
	return 0;
}
IRQCHIP_DECLARE(mt_gic, "mtk,mt-gic", mt_gic_of_init);

EXPORT_SYMBOL(mt_irq_dump_status);
EXPORT_SYMBOL(mt_irq_set_polarity);
EXPORT_SYMBOL(mt_irq_set_sens);

#endif


