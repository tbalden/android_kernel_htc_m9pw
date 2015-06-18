/*
 * Completely Fair Scheduling (CFS) Class (SCHED_NORMAL/SCHED_BATCH)
 *
 *  Copyright (C) 2007 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 *  Interactivity improvements by Mike Galbraith
 *  (C) 2007 Mike Galbraith <efault@gmx.de>
 *
 *  Various enhancements by Dmitry Adamushko.
 *  (C) 2007 Dmitry Adamushko <dmitry.adamushko@gmail.com>
 *
 *  Group scheduling enhancements by Srivatsa Vaddagiri
 *  Copyright IBM Corporation, 2007
 *  Author: Srivatsa Vaddagiri <vatsa@linux.vnet.ibm.com>
 *
 *  Scaled math optimizations by Thomas Gleixner
 *  Copyright (C) 2007, Thomas Gleixner <tglx@linutronix.de>
 *
 *  Adaptive scheduling granularity, math enhancements by Peter Zijlstra
 *  Copyright (C) 2007 Red Hat, Inc., Peter Zijlstra <pzijlstr@redhat.com>
 */

#include <linux/latencytop.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/profile.h>
#include <linux/interrupt.h>
#include <linux/mempolicy.h>
#include <linux/migrate.h>
#include <linux/task_work.h>

#include <trace/events/sched.h>
#ifdef CONFIG_HMP_VARIABLE_SCALE
#include <linux/sysfs.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
#include <linux/cpufreq.h>
#endif 
#endif 

#include "sched.h"

#include <mtlbprof/mtlbprof.h>


#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
#ifdef CONFIG_LOCAL_TIMERS
unsigned long localtimer_get_counter(void);
#endif
#endif

#ifdef CONFIG_HEVTASK_INTERFACE
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#ifdef CONFIG_KGDB_KDB
#include <linux/kdb.h>
#endif
#endif

unsigned int sysctl_sched_latency = 6000000ULL;
unsigned int normalized_sysctl_sched_latency = 6000000ULL;

enum sched_tunable_scaling sysctl_sched_tunable_scaling
	= SCHED_TUNABLESCALING_LOG;

unsigned int sysctl_sched_min_granularity = 750000ULL;
unsigned int normalized_sysctl_sched_min_granularity = 750000ULL;

static unsigned int sched_nr_latency = 8;

unsigned int sysctl_sched_child_runs_first __read_mostly;

unsigned int sysctl_sched_wakeup_granularity = 1000000UL;
unsigned int normalized_sysctl_sched_wakeup_granularity = 1000000UL;

const_debug unsigned int sysctl_sched_migration_cost = 100000UL;

unsigned int __read_mostly sysctl_sched_shares_window = 10000000UL;

#ifdef CONFIG_CFS_BANDWIDTH
unsigned int sysctl_sched_cfs_bandwidth_slice = 5000UL;
#endif
#if defined (CONFIG_MTK_SCHED_CMP_LAZY_BALANCE) && !defined(CONFIG_HMP_LAZY_BALANCE)
static int need_lazy_balance(int dst_cpu, int src_cpu, struct task_struct *p);
#endif

static int get_update_sysctl_factor(void)
{
	unsigned int cpus = min_t(int, num_online_cpus(), 8);
	unsigned int factor;

	switch (sysctl_sched_tunable_scaling) {
	case SCHED_TUNABLESCALING_NONE:
		factor = 1;
		break;
	case SCHED_TUNABLESCALING_LINEAR:
		factor = cpus;
		break;
	case SCHED_TUNABLESCALING_LOG:
	default:
		factor = 1 + ilog2(cpus);
		break;
	}

	return factor;
}

static void update_sysctl(void)
{
	unsigned int factor = get_update_sysctl_factor();

#define SET_SYSCTL(name) \
	(sysctl_##name = (factor) * normalized_sysctl_##name)
	SET_SYSCTL(sched_min_granularity);
	SET_SYSCTL(sched_latency);
	SET_SYSCTL(sched_wakeup_granularity);
#undef SET_SYSCTL
}

void sched_init_granularity(void)
{
	update_sysctl();
}
#if defined (CONFIG_MTK_SCHED_CMP_PACK_SMALL_TASK) || defined (CONFIG_HMP_PACK_SMALL_TASK)
DEFINE_PER_CPU(int, sd_pack_buddy) = {-1};

#ifdef CONFIG_MTK_SCHED_CMP_PACK_SMALL_TASK 
struct cpumask buddy_cpu_map = {{0}};
#endif

void update_packing_domain(int cpu)
{
	struct sched_domain *sd;
	int id = -1;

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
	pr_info("[PACK] update_packing_domain() CPU%d\n", cpu);
#endif 
	mt_sched_printf("[PACK] update_packing_domain() CPU%d", cpu);

	sd = highest_flag_domain(cpu, SD_SHARE_POWERLINE);
	if (!sd)
	{
		sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd);
	}
	else
		if (cpumask_first(sched_domain_span(sd)) == cpu || !sd->parent)
			sd = sd->parent;

	while (sd) {
		struct sched_group *sg = sd->groups;
		struct sched_group *pack = sg;
		struct sched_group *tmp = sg->next;

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
		pr_info("[PACK]  sd = 0x%08x, flags = %d\n", (unsigned int)sd, sd->flags);
#endif 

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
		pr_info("[PACK]  sg = 0x%08x\n", (unsigned int)sg);
#endif 

		
		if (id == -1)
			id = cpumask_first(sched_domain_span(sd));

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
		pr_info("[PACK]  First cpu in this sd id = %d\n", id);
#endif 

		
		tmp = sd->groups;
		do {
			if (cpumask_test_cpu(id, sched_group_cpus(tmp))) {
				sg = tmp;
				break;
			}
		} while (tmp = tmp->next, tmp != sd->groups);

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
		pr_info("[PACK]  pack = 0x%08x\n", (unsigned int)sg);
#endif 

		pack = sg;
		tmp = sg->next;

		
		
		
		while (tmp != sg && !(sd->flags & SD_LOAD_BALANCE)) {
			if (tmp->sgp->power * sg->group_weight <
					sg->sgp->power * tmp->group_weight) {

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
				pr_info("[PACK]  Now sg power = %u, weight = %u, mask = %lu\n", sg->sgp->power, sg->group_weight, sg->cpumask[0]);
				pr_info("[PACK]  Better sg power = %u, weight = %u, mask = %lu\n", tmp->sgp->power, tmp->group_weight, tmp->cpumask[0]);        
#endif 
      
				pack = tmp;
			}
			tmp = tmp->next;
		}

		
		if (pack != sg) {
			id = cpumask_first(sched_group_cpus(pack));

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
			pr_info("[PACK]  Better sg, first cpu id = %d\n", id);
#endif 

		}

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
		if(sd->parent) {
			pr_info("[PACK]  cpu = %d, id = %d, sd->parent = 0x%08x, flags = %d, SD_LOAD_BALANCE = %d\n", cpu, id, (unsigned int)sd->parent, sd->parent->flags, SD_LOAD_BALANCE);
			pr_info("[PACK]  %d\n", (id != cpu)); 
			pr_info("[PACK]  0x%08x\n", (unsigned int)(sd->parent));  
			pr_info("[PACK]  %d\n", (sd->parent->flags & SD_LOAD_BALANCE));      
		}
		else {
			pr_info("[PACK]  cpu = %d, id = %d, sd->parent = 0x%08x\n", cpu, id, (unsigned int)sd->parent);      
		}
#endif 
        

		
		if ((id != cpu) || 
				((sd->parent) && (sd->parent->flags & SD_LOAD_BALANCE))) {

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
			pr_info("[PACK]  Break\n");
#endif 

			break;
		}
		sd = sd->parent;
	}

#ifdef CONFIG_HMP_PACK_BUDDY_INFO
	pr_info("[PACK] CPU%d packing on CPU%d\n", cpu, id);
#endif 
	mt_sched_printf("[PACK] CPU%d packing on CPU%d", cpu, id);

#ifdef CONFIG_HMP_PACK_SMALL_TASK
	per_cpu(sd_pack_buddy, cpu) = id;
#else 
	if(per_cpu(sd_pack_buddy, cpu) != -1)
		cpu_clear(per_cpu(sd_pack_buddy, cpu), buddy_cpu_map);
	per_cpu(sd_pack_buddy, cpu) = id;
	if(id != -1)
		cpumask_set_cpu(id, &buddy_cpu_map);
#endif
}

#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_USAGE);
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_PERIOD);
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_NR);
DEFINE_PER_CPU(u32, TASK_USGAE);
DEFINE_PER_CPU(u32, TASK_PERIOD);
u32 PACK_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 AVOID_LOAD_BALANCE_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 AVOID_WAKE_UP_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 TASK_PACK_CPU_COUNT[4][NR_CPUS] = {{0}};
u32 PA_ENABLE = 1;
u32 PA_MON_ENABLE = 0;
char PA_MON[4][TASK_COMM_LEN]={{0}};
#endif 

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_USAGE);
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_PERIOD);
DEFINE_PER_CPU(u32, BUDDY_CPU_RQ_NR);
DEFINE_PER_CPU(u32, TASK_USGAE);
DEFINE_PER_CPU(u32, TASK_PERIOD);
u32 PACK_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 AVOID_LOAD_BALANCE_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 AVOID_WAKE_UP_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 HMP_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
u32 PA_ENABLE = 1;
u32 LB_ENABLE = 1;
u32 PA_MON_ENABLE = 0;
char PA_MON[TASK_COMM_LEN];

#ifdef CONFIG_HMP_TRACER
#define POWER_AWARE_ACTIVE_MODULE_PACK_FORM_CPUX_TO_CPUY (0)
#define POWER_AWARE_ACTIVE_MODULE_AVOID_WAKE_UP_FORM_CPUX_TO_CPUY (1)
#define POWER_AWARE_ACTIVE_MODULE_AVOID_BALANCE_FORM_CPUX_TO_CPUY (2)
#define POWER_AWARE_ACTIVE_MODULE_AVOID_FORCE_UP_FORM_CPUX_TO_CPUY (3)
#endif 

#endif 


static inline bool is_buddy_busy(int cpu)
{
#ifdef CONFIG_HMP_PACK_SMALL_TASK
	struct rq *rq;

    if (cpu < 0)
    	return 0;

    rq = cpu_rq(cpu);
#else  
		struct rq *rq = cpu_rq(cpu);
#endif

#if defined (CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER) || defined (CONFIG_HMP_POWER_AWARE_CONTROLLER)
	per_cpu(BUDDY_CPU_RQ_USAGE, cpu) = rq->avg.usage_avg_sum;
	per_cpu(BUDDY_CPU_RQ_PERIOD, cpu) = rq->avg.runnable_avg_period;	
	per_cpu(BUDDY_CPU_RQ_NR, cpu) = rq->nr_running;	
#endif 

	return ((rq->avg.usage_avg_sum << rq->nr_running) >
			rq->avg.runnable_avg_period);

}

static inline bool is_light_task(struct task_struct *p)
{
#if defined (CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER) || defined (CONFIG_HMP_POWER_AWARE_CONTROLLER) 
	per_cpu(TASK_USGAE, task_cpu(p)) = p->se.avg.usage_avg_sum;
	per_cpu(TASK_PERIOD, task_cpu(p)) = p->se.avg.runnable_avg_period;	
#endif 

	
	return ((p->se.avg.usage_avg_sum << 2) < p->se.avg.runnable_avg_period);
}


static int check_pack_buddy(int cpu, struct task_struct *p)
{
#ifdef CONFIG_HMP_PACK_SMALL_TASK
	int buddy;
	
	if(cpu >= NR_CPUS || cpu < 0)
		return false; 
	buddy = per_cpu(sd_pack_buddy, cpu);
#else 
	int buddy = cpu;
#endif

	
	if (buddy == -1)
		return false;

#ifdef CONFIG_HMP_PACK_SMALL_TASK
	if ((buddy == cpu)
	 && ((p->se.avg.usage_avg_sum << 3) < (p->se.avg.runnable_avg_sum * 5)))
		return false;
#endif

	
	if (!cpumask_test_cpu(buddy, tsk_cpus_allowed(p)))
		return false;

	 if (!is_light_task(p) || is_buddy_busy(buddy))
		return false;

	return true;
}
#endif 

#if BITS_PER_LONG == 32
# define WMULT_CONST	(~0UL)
#else
# define WMULT_CONST	(1UL << 32)
#endif

#define WMULT_SHIFT	32

#define SRR(x, y) (((x) + (1UL << ((y) - 1))) >> (y))

static unsigned long
calc_delta_mine(unsigned long delta_exec, unsigned long weight,
		struct load_weight *lw)
{
	u64 tmp;

	if (likely(weight > (1UL << SCHED_LOAD_RESOLUTION)))
		tmp = (u64)delta_exec * scale_load_down(weight);
	else
		tmp = (u64)delta_exec;

	if (!lw->inv_weight) {
		unsigned long w = scale_load_down(lw->weight);

		if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
			lw->inv_weight = 1;
		else if (unlikely(!w))
			lw->inv_weight = WMULT_CONST;
		else
			lw->inv_weight = WMULT_CONST / w;
	}

	if (unlikely(tmp > WMULT_CONST))
		tmp = SRR(SRR(tmp, WMULT_SHIFT/2) * lw->inv_weight,
			WMULT_SHIFT/2);
	else
		tmp = SRR(tmp * lw->inv_weight, WMULT_SHIFT);

	return (unsigned long)min(tmp, (u64)(unsigned long)LONG_MAX);
}


const struct sched_class fair_sched_class;


#ifdef CONFIG_FAIR_GROUP_SCHED

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return cfs_rq->rq;
}

#define entity_is_task(se)	(!se->my_q)

static inline struct task_struct *task_of(struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	WARN_ON_ONCE(!entity_is_task(se));
#endif
	return container_of(se, struct task_struct, se);
}

#define for_each_sched_entity(se) \
		for (; se; se = se->parent)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return p->se.cfs_rq;
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	return se->cfs_rq;
}

static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return grp->my_q;
}

static void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq,
				       int force_update);

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (!cfs_rq->on_list) {
		if (cfs_rq->tg->parent &&
		    cfs_rq->tg->parent->cfs_rq[cpu_of(rq_of(cfs_rq))]->on_list) {
			list_add_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		} else {
			list_add_tail_rcu(&cfs_rq->leaf_cfs_rq_list,
				&rq_of(cfs_rq)->leaf_cfs_rq_list);
		}

		cfs_rq->on_list = 1;
		
		update_cfs_rq_blocked_load(cfs_rq, 0);
	}
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
	if (cfs_rq->on_list) {
		list_del_rcu(&cfs_rq->leaf_cfs_rq_list);
		cfs_rq->on_list = 0;
	}
}

#define for_each_leaf_cfs_rq(rq, cfs_rq) \
	list_for_each_entry_rcu(cfs_rq, &rq->leaf_cfs_rq_list, leaf_cfs_rq_list)

static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	if (se && pse)
	{
		if (se->cfs_rq == pse->cfs_rq)
			return 1;
	}

	return 0;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return se->parent;
}

static inline int depth_se(struct sched_entity *se)
{
	int depth = 0;

	for_each_sched_entity(se)
		depth++;

	return depth;
}

static void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
	int se_depth, pse_depth;


	
	se_depth = depth_se(*se);
	pse_depth = depth_se(*pse);

	while (se_depth > pse_depth) {
		se_depth--;
		*se = parent_entity(*se);
	}

	while (pse_depth > se_depth) {
		pse_depth--;
		*pse = parent_entity(*pse);
	}

	while (!is_same_group(*se, *pse)) {
		*se = parent_entity(*se);
		*pse = parent_entity(*pse);
	}
}

#else	

static inline struct task_struct *task_of(struct sched_entity *se)
{
	return container_of(se, struct task_struct, se);
}

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return container_of(cfs_rq, struct rq, cfs);
}

#define entity_is_task(se)	1

#define for_each_sched_entity(se) \
		for (; se; se = NULL)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
	return &task_rq(p)->cfs;
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct rq *rq = task_rq(p);

	return &rq->cfs;
}

static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return NULL;
}

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

#define for_each_leaf_cfs_rq(rq, cfs_rq) \
		for (cfs_rq = &rq->cfs; cfs_rq; cfs_rq = NULL)

static inline int
is_same_group(struct sched_entity *se, struct sched_entity *pse)
{
	return 1;
}

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return NULL;
}

static inline void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
}

#endif	

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec);


static inline u64 max_vruntime(u64 max_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - max_vruntime);
	if (delta > 0)
		max_vruntime = vruntime;

	return max_vruntime;
}

static inline u64 min_vruntime(u64 min_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vruntime);
	if (delta < 0)
		min_vruntime = vruntime;

	return min_vruntime;
}

static inline int entity_before(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

static void update_min_vruntime(struct cfs_rq *cfs_rq)
{
	u64 vruntime = cfs_rq->min_vruntime;

	if (cfs_rq->curr)
		vruntime = cfs_rq->curr->vruntime;

	if (cfs_rq->rb_leftmost) {
		struct sched_entity *se = rb_entry(cfs_rq->rb_leftmost,
						   struct sched_entity,
						   run_node);

		if (!cfs_rq->curr)
			vruntime = se->vruntime;
		else
			vruntime = min_vruntime(vruntime, se->vruntime);
	}

	
	cfs_rq->min_vruntime = max_vruntime(cfs_rq->min_vruntime, vruntime);
#ifndef CONFIG_64BIT
	smp_wmb();
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
}

static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct rb_node **link = &cfs_rq->tasks_timeline.rb_node;
	struct rb_node *parent = NULL;
	struct sched_entity *entry;
	int leftmost = 1;

	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct sched_entity, run_node);
		if (entity_before(se, entry)) {
			link = &parent->rb_left;
		} else {
			link = &parent->rb_right;
			leftmost = 0;
		}
	}

	if (leftmost)
		cfs_rq->rb_leftmost = &se->run_node;

	rb_link_node(&se->run_node, parent, link);
	rb_insert_color(&se->run_node, &cfs_rq->tasks_timeline);
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->rb_leftmost == &se->run_node) {
		struct rb_node *next_node;

		next_node = rb_next(&se->run_node);
		cfs_rq->rb_leftmost = next_node;
	}

	rb_erase(&se->run_node, &cfs_rq->tasks_timeline);
}

struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *left = cfs_rq->rb_leftmost;

	if (!left)
		return NULL;

	return rb_entry(left, struct sched_entity, run_node);
}

static struct sched_entity *__pick_next_entity(struct sched_entity *se)
{
	struct rb_node *next = rb_next(&se->run_node);

	if (!next)
		return NULL;

	return rb_entry(next, struct sched_entity, run_node);
}

#ifdef CONFIG_SCHED_DEBUG
struct sched_entity *__pick_last_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *last = rb_last(&cfs_rq->tasks_timeline);

	if (!last)
		return NULL;

	return rb_entry(last, struct sched_entity, run_node);
}


int sched_proc_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	int factor = get_update_sysctl_factor();

	if (ret || !write)
		return ret;

	sched_nr_latency = DIV_ROUND_UP(sysctl_sched_latency,
					sysctl_sched_min_granularity);

#define WRT_SYSCTL(name) \
	(normalized_sysctl_##name = sysctl_##name / (factor))
	WRT_SYSCTL(sched_min_granularity);
	WRT_SYSCTL(sched_latency);
	WRT_SYSCTL(sched_wakeup_granularity);
#undef WRT_SYSCTL

	return 0;
}
#endif

static inline unsigned long
calc_delta_fair(unsigned long delta, struct sched_entity *se)
{
	if (unlikely(se->load.weight != NICE_0_LOAD))
		delta = calc_delta_mine(delta, NICE_0_LOAD, &se->load);

	return delta;
}

static u64 __sched_period(unsigned long nr_running)
{
	u64 period = sysctl_sched_latency;
	unsigned long nr_latency = sched_nr_latency;

	if (unlikely(nr_running > nr_latency)) {
		period = sysctl_sched_min_granularity;
		period *= nr_running;
	}

	return period;
}

static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
		slice = calc_delta_mine(slice, se->load.weight, load);
	}
	return slice;
}

static u64 sched_vslice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return calc_delta_fair(sched_slice(cfs_rq, se), se);
}


#ifdef CONFIG_SMP
static inline void __update_task_entity_contrib(struct sched_entity *se);

static long __update_task_entity_ratio(struct sched_entity *se);

#define LOAD_AVG_PERIOD 32
#define LOAD_AVG_MAX 47742 
#define LOAD_AVG_MAX_N 345 
#define LOAD_AVG_VARIABLE_PERIOD 512
static unsigned int init_task_load_period = 4000;

void init_task_runnable_average(struct task_struct *p)
{
	u32 slice;

	p->se.avg.decay_count = 0;
	slice = sched_slice(task_cfs_rq(p), &p->se) >> 10;
	p->se.avg.runnable_avg_sum = (init_task_load_period) ? 0 : slice;
	p->se.avg.runnable_avg_period = (init_task_load_period)?(init_task_load_period):slice;
	__update_task_entity_contrib(&p->se);

#ifdef CONFIG_MTK_SCHED_CMP
	
	p->se.avg.usage_avg_sum =  (init_task_load_period) ? 0 : slice;
#endif
	__update_task_entity_ratio(&p->se);
	trace_sched_task_entity_avg(0, p, &p->se.avg);
}
#else
void init_task_runnable_average(struct task_struct *p)
{
}
#endif

static inline void
__update_curr(struct cfs_rq *cfs_rq, struct sched_entity *curr,
	      unsigned long delta_exec)
{
	unsigned long delta_exec_weighted;

	schedstat_set(curr->statistics.exec_max,
		      max((u64)delta_exec, curr->statistics.exec_max));

	curr->sum_exec_runtime += delta_exec;
	schedstat_add(cfs_rq, exec_clock, delta_exec);
	delta_exec_weighted = calc_delta_fair(delta_exec, curr);

	curr->vruntime += delta_exec_weighted;
	update_min_vruntime(cfs_rq);
}

static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	u64 now = rq_of(cfs_rq)->clock_task;
	unsigned long delta_exec;

	if (unlikely(!curr))
		return;

	delta_exec = (unsigned long)(now - curr->exec_start);
	if (!delta_exec)
		return;

	__update_curr(cfs_rq, curr, delta_exec);
	curr->exec_start = now;

	if (entity_is_task(curr)) {
		struct task_struct *curtask = task_of(curr);

		trace_sched_stat_runtime(curtask, delta_exec, curr->vruntime);
		cpuacct_charge(curtask, delta_exec);
		account_group_exec_runtime(curtask, delta_exec);
	}

	account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline void
update_stats_wait_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_start, rq_of(cfs_rq)->clock);
}

static void update_stats_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se != cfs_rq->curr)
		update_stats_wait_start(cfs_rq, se);
}

static void
update_stats_wait_end(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	schedstat_set(se->statistics.wait_max, max(se->statistics.wait_max,
			rq_of(cfs_rq)->clock - se->statistics.wait_start));
	schedstat_set(se->statistics.wait_count, se->statistics.wait_count + 1);
	schedstat_set(se->statistics.wait_sum, se->statistics.wait_sum +
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
#ifdef CONFIG_SCHEDSTATS
	if (entity_is_task(se)) {
		trace_sched_stat_wait(task_of(se),
			rq_of(cfs_rq)->clock - se->statistics.wait_start);
	}
#endif
	schedstat_set(se->statistics.wait_start, 0);
}

static inline void
update_stats_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (se != cfs_rq->curr)
		update_stats_wait_end(cfs_rq, se);
}

static inline void
update_stats_curr_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	se->exec_start = rq_of(cfs_rq)->clock_task;
}


#ifdef CONFIG_NUMA_BALANCING
unsigned int sysctl_numa_balancing_scan_period_min = 100;
unsigned int sysctl_numa_balancing_scan_period_max = 100*50;
unsigned int sysctl_numa_balancing_scan_period_reset = 100*600;

unsigned int sysctl_numa_balancing_scan_size = 256;

unsigned int sysctl_numa_balancing_scan_delay = 1000;

static void task_numa_placement(struct task_struct *p)
{
	int seq;

	if (!p->mm)	
		return;
	seq = ACCESS_ONCE(p->mm->numa_scan_seq);
	if (p->numa_scan_seq == seq)
		return;
	p->numa_scan_seq = seq;

	
}

void task_numa_fault(int node, int pages, bool migrated)
{
	struct task_struct *p = current;

	if (!sched_feat_numa(NUMA))
		return;

	

        if (!migrated)
		p->numa_scan_period = min(sysctl_numa_balancing_scan_period_max,
			p->numa_scan_period + jiffies_to_msecs(10));

	task_numa_placement(p);
}

static void reset_ptenuma_scan(struct task_struct *p)
{
	ACCESS_ONCE(p->mm->numa_scan_seq)++;
	p->mm->numa_scan_offset = 0;
}

void task_numa_work(struct callback_head *work)
{
	unsigned long migrate, next_scan, now = jiffies;
	struct task_struct *p = current;
	struct mm_struct *mm = p->mm;
	struct vm_area_struct *vma;
	unsigned long start, end;
	long pages;

	WARN_ON_ONCE(p != container_of(work, struct task_struct, numa_work));

	work->next = work; 
	if (p->flags & PF_EXITING)
		return;

	if (mm->first_nid == NUMA_PTE_SCAN_INIT)
		mm->first_nid = numa_node_id();
	if (mm->first_nid != NUMA_PTE_SCAN_ACTIVE) {
		
		if (numa_node_id() == mm->first_nid &&
		    !sched_feat_numa(NUMA_FORCE))
			return;

		mm->first_nid = NUMA_PTE_SCAN_ACTIVE;
	}

	migrate = mm->numa_next_reset;
	if (time_after(now, migrate)) {
		p->numa_scan_period = sysctl_numa_balancing_scan_period_min;
		next_scan = now + msecs_to_jiffies(sysctl_numa_balancing_scan_period_reset);
		xchg(&mm->numa_next_reset, next_scan);
	}

	migrate = mm->numa_next_scan;
	if (time_before(now, migrate))
		return;

	if (p->numa_scan_period == 0)
		p->numa_scan_period = sysctl_numa_balancing_scan_period_min;

	next_scan = now + msecs_to_jiffies(p->numa_scan_period);
	if (cmpxchg(&mm->numa_next_scan, migrate, next_scan) != migrate)
		return;

	if (migrate_ratelimited(numa_node_id()))
		return;

	start = mm->numa_scan_offset;
	pages = sysctl_numa_balancing_scan_size;
	pages <<= 20 - PAGE_SHIFT; 
	if (!pages)
		return;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);
	if (!vma) {
		reset_ptenuma_scan(p);
		start = 0;
		vma = mm->mmap;
	}
	for (; vma; vma = vma->vm_next) {
		if (!vma_migratable(vma))
			continue;

		
		if (vma->vm_end - vma->vm_start < HPAGE_SIZE)
			continue;

		if (!(vma->vm_flags & (VM_READ | VM_EXEC | VM_WRITE)))
			continue;

		do {
			start = max(start, vma->vm_start);
			end = ALIGN(start + (pages << PAGE_SHIFT), HPAGE_SIZE);
			end = min(end, vma->vm_end);
			pages -= change_prot_numa(vma, start, end);

			start = end;
			if (pages <= 0)
				goto out;
		} while (end != vma->vm_end);
	}

out:
	if (vma)
		mm->numa_scan_offset = start;
	else
		reset_ptenuma_scan(p);
	up_read(&mm->mmap_sem);
}

void task_tick_numa(struct rq *rq, struct task_struct *curr)
{
	struct callback_head *work = &curr->numa_work;
	u64 period, now;

	if (!curr->mm || (curr->flags & PF_EXITING) || work->next != work)
		return;

	now = curr->se.sum_exec_runtime;
	period = (u64)curr->numa_scan_period * NSEC_PER_MSEC;

	if (now - curr->node_stamp > period) {
		if (!curr->node_stamp)
			curr->numa_scan_period = sysctl_numa_balancing_scan_period_min;
		curr->node_stamp = now;

		if (!time_before(jiffies, curr->mm->numa_next_scan)) {
			init_task_work(work, task_numa_work); 
			task_work_add(curr, work, true);
		}
	}
}
#else
static void task_tick_numa(struct rq *rq, struct task_struct *curr)
{
}
#endif 

static void
account_entity_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_add(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_add(&rq_of(cfs_rq)->load, se->load.weight);
#ifdef CONFIG_SMP
	if (entity_is_task(se))
		list_add(&se->group_node, &rq_of(cfs_rq)->cfs_tasks);
#endif
	cfs_rq->nr_running++;
}

static void
account_entity_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_sub(&cfs_rq->load, se->load.weight);
	if (!parent_entity(se))
		update_load_sub(&rq_of(cfs_rq)->load, se->load.weight);
	if (entity_is_task(se))
		list_del_init(&se->group_node);
	cfs_rq->nr_running--;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
# ifdef CONFIG_SMP
static inline long calc_tg_weight(struct task_group *tg, struct cfs_rq *cfs_rq)
{
	long tg_weight;

	tg_weight = atomic_long_read(&tg->load_avg);
	tg_weight -= cfs_rq->tg_load_contrib;
	tg_weight += cfs_rq->load.weight;

	return tg_weight;
}

static long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	long tg_weight, load, shares;

	tg_weight = calc_tg_weight(tg, cfs_rq);
	load = cfs_rq->load.weight;

	shares = (tg->shares * load);
	if (tg_weight)
		shares /= tg_weight;

	if (shares < MIN_SHARES)
		shares = MIN_SHARES;
	if (shares > tg->shares)
		shares = tg->shares;

	return shares;
}
# else 
static inline long calc_cfs_shares(struct cfs_rq *cfs_rq, struct task_group *tg)
{
	return tg->shares;
}
# endif 
static void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			    unsigned long weight)
{
	if (se->on_rq) {
		
		if (cfs_rq->curr == se)
			update_curr(cfs_rq);
		account_entity_dequeue(cfs_rq, se);
	}

	update_load_set(&se->load, weight);

	if (se->on_rq)
		account_entity_enqueue(cfs_rq, se);
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq);

static void update_cfs_shares(struct cfs_rq *cfs_rq)
{
	struct task_group *tg;
	struct sched_entity *se;
	long shares;

	tg = cfs_rq->tg;
	se = tg->se[cpu_of(rq_of(cfs_rq))];
	if (!se || throttled_hierarchy(cfs_rq))
		return;
#ifndef CONFIG_SMP
	if (likely(se->load.weight == tg->shares))
		return;
#endif
	shares = calc_cfs_shares(cfs_rq, tg);

	reweight_entity(cfs_rq_of(se), se, shares);
}
#else 
static inline void update_cfs_shares(struct cfs_rq *cfs_rq)
{
}
#endif 

#ifdef CONFIG_SMP

static const u32 runnable_avg_yN_inv[] = {
	0xffffffff, 0xfa83b2da, 0xf5257d14, 0xefe4b99a, 0xeac0c6e6, 0xe5b906e6,
	0xe0ccdeeb, 0xdbfbb796, 0xd744fcc9, 0xd2a81d91, 0xce248c14, 0xc9b9bd85,
	0xc5672a10, 0xc12c4cc9, 0xbd08a39e, 0xb8fbaf46, 0xb504f333, 0xb123f581,
	0xad583ee9, 0xa9a15ab4, 0xa5fed6a9, 0xa2704302, 0x9ef5325f, 0x9b8d39b9,
	0x9837f050, 0x94f4efa8, 0x91c3d373, 0x8ea4398a, 0x8b95c1e3, 0x88980e80,
	0x85aac367, 0x82cd8698,
};

static const u32 runnable_avg_yN_sum[] = {
	    0, 1002, 1982, 2941, 3880, 4798, 5697, 6576, 7437, 8279, 9103,
	 9909,10698,11470,12226,12966,13690,14398,15091,15769,16433,17082,
	17718,18340,18949,19545,20128,20698,21256,21802,22336,22859,23371,
};

static __always_inline u64 decay_load(u64 val, u64 n)
{
	unsigned int local_n;

	if (!n)
		return val;
	else if (unlikely(n > LOAD_AVG_PERIOD * 63))
		return 0;

	
	local_n = n;

	if (unlikely(local_n >= LOAD_AVG_PERIOD)) {
		val >>= local_n / LOAD_AVG_PERIOD;
		local_n %= LOAD_AVG_PERIOD;
	}

	val *= runnable_avg_yN_inv[local_n];
	
	return val >> 32;
}

static u32 __compute_runnable_contrib(u64 n)
{
	u32 contrib = 0;

	if (likely(n <= LOAD_AVG_PERIOD))
		return runnable_avg_yN_sum[n];
	else if (unlikely(n >= LOAD_AVG_MAX_N))
		return LOAD_AVG_MAX;

	
	do {
		contrib /= 2; 
		contrib += runnable_avg_yN_sum[LOAD_AVG_PERIOD];

		n -= LOAD_AVG_PERIOD;
	} while (n > LOAD_AVG_PERIOD);

	contrib = decay_load(contrib, n);
	return contrib + runnable_avg_yN_sum[n];
}

#ifdef CONFIG_HMP_VARIABLE_SCALE

#define HMP_VARIABLE_SCALE_SHIFT 16ULL
struct hmp_global_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
			struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *a, struct attribute *b,
			const char *c, size_t count);
	int *value;
	int (*to_sysfs)(int);
	int (*from_sysfs)(int);
};

#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
#define HMP_DATA_SYSFS_MAX 5
#else
#define HMP_DATA_SYSFS_MAX 4
#endif

struct hmp_data_struct {
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	int freqinvar_load_scale_enabled;
#endif
	int multiplier; 
	struct attribute_group attr_group;
	struct attribute *attributes[HMP_DATA_SYSFS_MAX + 1];
	struct hmp_global_attr attr[HMP_DATA_SYSFS_MAX];
} hmp_data;

static u64 hmp_variable_scale_convert(u64 delta);
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
#define SCHED_FREQSCALE_SHIFT 10
struct cpufreq_extents {
	u32 curr_scale;
	u32 min;
	u32 max;
	u32 flags;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	u32 const_max;
	u32 throttling;
#endif
};
#define SCHED_LOAD_FREQINVAR_SINGLEFREQ 0x01

static struct cpufreq_extents freq_scale[CONFIG_NR_CPUS];
#endif 
#endif 

#ifdef CONFIG_MTK_SCHED_CMP
int get_cluster_id(unsigned int cpu)
{
	return arch_get_cluster_id(cpu);
}

void get_cluster_cpus(struct cpumask *cpus, int cluster_id,
			    bool exclusive_offline)
{
	struct cpumask cls_cpus;

	arch_get_cluster_cpus(&cls_cpus, cluster_id);
	if (exclusive_offline) {
		cpumask_and(cpus, cpu_online_mask, &cls_cpus);
	} else
		cpumask_copy(cpus, &cls_cpus);
}

static int nr_cpus_in_cluster(int cluster_id, bool exclusive_offline)
{
	struct cpumask cls_cpus;
	int nr_cpus;

	arch_get_cluster_cpus(&cls_cpus, cluster_id);
	if (exclusive_offline) {
		struct cpumask online_cpus;
		cpumask_and(&online_cpus, cpu_online_mask, &cls_cpus);
		nr_cpus = cpumask_weight(&online_cpus);
	} else
		nr_cpus = cpumask_weight(&cls_cpus);

	return nr_cpus;
}
#endif 

void sched_get_big_little_cpus(struct cpumask *big, struct cpumask *little)
{
	arch_get_big_little_cpus(big, little);
}
EXPORT_SYMBOL(sched_get_big_little_cpus);

static __init __inline void cmp_cputopo_domain_setup(void)
{
	WARN(smp_processor_id() != 0, "%s is supposed runs on CPU0 "
				      "while kernel init", __func__);
#ifdef CONFIG_MTK_CPU_TOPOLOGY
	pr_debug("[CPUTOPO][%s] build CPU topology and cluster.\n", __func__);
	arch_build_cpu_topology_domain();
#endif
}

#ifdef CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY
static u64 __inline variable_scale_convert(u64 delta)
{
	u64 high = delta >> 32ULL;
	u64 low = delta & 0xffffffffULL;
	low *= LOAD_AVG_VARIABLE_PERIOD;
	high *= LOAD_AVG_VARIABLE_PERIOD;
	return (low >> 16ULL) + (high << (32ULL - 16ULL));
}
#endif

static __always_inline int __update_entity_runnable_avg(u64 now,
							struct sched_avg *sa,
							int runnable,
							int running,
							int cpu)
{
	u64 delta, periods, lru;
	u32 runnable_contrib;
	int delta_w, decayed = 0;
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	u64 scaled_delta;
	u32 scaled_runnable_contrib;
	int scaled_delta_w;
	u32 curr_scale = 1024;
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	u64 scaled_delta;
	u32 scaled_runnable_contrib;
	int scaled_delta_w;
	u32 curr_scale = CPUPOWER_FREQSCALE_DEFAULT;
#endif 

	delta = now - sa->last_runnable_update;
	lru = sa->last_runnable_update;
	if ((s64)delta < 0) {
		sa->last_runnable_update = now;
		return 0;
	}

#ifdef CONFIG_HMP_VARIABLE_SCALE
	delta = hmp_variable_scale_convert(delta);
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	delta = variable_scale_convert(delta);
#endif
	delta >>= 10;
	if (!delta)
		return 0;
	sa->last_runnable_update = now;

#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	WARN(cpu < 0, "[%s] CPU %d < 0 !!!\n", __func__, cpu);
	
	if (cpu >= 0 && cpu < nr_cpu_ids && hmp_data.freqinvar_load_scale_enabled)
		curr_scale = freq_scale[cpu].curr_scale;
	mt_sched_printf("[%s] cpu=%d delta=%llu now=%llu last=%llu curr_scale=%u",
					__func__, cpu, delta, now, lru, curr_scale);
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	WARN(cpu < 0, "[%s] CPU %d < 0 !!!\n", __func__, cpu);
	
	if (cpu >= 0 && cpu < nr_cpu_ids)
		curr_scale = (topology_cpu_capacity(cpu) << CPUPOWER_FREQSCALE_SHIFT)
			/ (topology_max_cpu_capacity(cpu)+1);
	mt_sched_printf("[%s] cpu=%d delta=%llu now=%llu last=%llu curr_scale=%u",
					__func__, cpu, delta, now, lru, curr_scale);
#endif 

	
	delta_w = sa->runnable_avg_period % 1024;
	if (delta + delta_w >= 1024) {
		
		decayed = 1;

		delta_w = 1024 - delta_w;
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
		
		scaled_delta_w = (delta_w * curr_scale)
				>> SCHED_FREQSCALE_SHIFT;
		if (runnable)
			sa->runnable_avg_sum += scaled_delta_w;
		if (running)
			sa->usage_avg_sum += scaled_delta_w;
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
		
		scaled_delta_w = (delta_w * curr_scale)
				>> CPUPOWER_FREQSCALE_SHIFT;
		if (runnable)
			sa->runnable_avg_sum += scaled_delta_w;
		if (running)
			sa->usage_avg_sum += scaled_delta_w;
#else
		if (runnable)
			sa->runnable_avg_sum += delta_w;
		if (running)
			sa->usage_avg_sum += delta_w;
#endif 
		sa->runnable_avg_period += delta_w;

		delta -= delta_w;

		
		periods = delta / 1024;
		delta %= 1024;
		
		sa->runnable_avg_sum = decay_load(sa->runnable_avg_sum,
						  periods + 1);
		sa->runnable_avg_period = decay_load(sa->runnable_avg_period,
						     periods + 1);
		sa->usage_avg_sum = decay_load(sa->usage_avg_sum, periods + 1);
		
		
		runnable_contrib = __compute_runnable_contrib(periods);
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
		scaled_runnable_contrib = (runnable_contrib * curr_scale)
				>> SCHED_FREQSCALE_SHIFT;
		if (runnable)
			sa->runnable_avg_sum += scaled_runnable_contrib;
		if (running)
			sa->usage_avg_sum += scaled_runnable_contrib;
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
		scaled_runnable_contrib = (runnable_contrib * curr_scale)
				>> CPUPOWER_FREQSCALE_SHIFT;
		if (runnable)
			sa->runnable_avg_sum += scaled_runnable_contrib;
		if (running)
			sa->usage_avg_sum += scaled_runnable_contrib;
#else
		if (runnable)
			sa->runnable_avg_sum += runnable_contrib;
		if (running)
			sa->usage_avg_sum += runnable_contrib;
#endif 
		sa->runnable_avg_period += runnable_contrib;
	}

	
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	
	scaled_delta = ((delta * curr_scale) >> SCHED_FREQSCALE_SHIFT);
	if (runnable)
		sa->runnable_avg_sum += scaled_delta;
	if (running)
		sa->usage_avg_sum += scaled_delta;
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	
	scaled_delta = ((delta * curr_scale) >> CPUPOWER_FREQSCALE_SHIFT);
	if (runnable)
		sa->runnable_avg_sum += scaled_delta;
	if (running)
		sa->usage_avg_sum += scaled_delta;
#else
	if (runnable)
		sa->runnable_avg_sum += delta;
	if (running)
		sa->usage_avg_sum += delta;
#endif 
	sa->runnable_avg_period += delta;

	return decayed;
}

static inline u64 __synchronize_entity_decay(struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 decays = atomic64_read(&cfs_rq->decay_counter);

	decays -= se->avg.decay_count;
	if (!decays)
		return 0;

	se->avg.load_avg_contrib = decay_load(se->avg.load_avg_contrib, decays);
	se->avg.decay_count = 0;

	return decays;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static inline void __update_cfs_rq_tg_load_contrib(struct cfs_rq *cfs_rq,
						 int force_update)
{
	struct task_group *tg = cfs_rq->tg;
	long tg_contrib;

	tg_contrib = cfs_rq->runnable_load_avg + cfs_rq->blocked_load_avg;
	tg_contrib -= cfs_rq->tg_load_contrib;

	if (force_update || abs(tg_contrib) > cfs_rq->tg_load_contrib / 8) {
		atomic_long_add(tg_contrib, &tg->load_avg);
		cfs_rq->tg_load_contrib += tg_contrib;
	}
}

static inline void __update_tg_runnable_avg(struct sched_avg *sa,
						  struct cfs_rq *cfs_rq)
{
	struct task_group *tg = cfs_rq->tg;
	long contrib, usage_contrib;

	
	contrib = div_u64(sa->runnable_avg_sum << NICE_0_SHIFT,
			  sa->runnable_avg_period + 1);
	contrib -= cfs_rq->tg_runnable_contrib;

	usage_contrib = div_u64(sa->usage_avg_sum << NICE_0_SHIFT,
			        sa->runnable_avg_period + 1);
	usage_contrib -= cfs_rq->tg_usage_contrib;

	if ((abs(contrib) > cfs_rq->tg_runnable_contrib / 64) ||
	    (abs(usage_contrib) > cfs_rq->tg_usage_contrib / 64)) {
		atomic_add(contrib, &tg->runnable_avg);
		cfs_rq->tg_runnable_contrib += contrib;

		atomic_add(usage_contrib, &tg->usage_avg);
		cfs_rq->tg_usage_contrib += usage_contrib;
	}
}

static inline void __update_group_entity_contrib(struct sched_entity *se)
{
	struct cfs_rq *cfs_rq = group_cfs_rq(se);
	struct task_group *tg = cfs_rq->tg;
	int runnable_avg;

	u64 contrib;

	contrib = cfs_rq->tg_load_contrib * tg->shares;
	se->avg.load_avg_contrib = div_u64(contrib,
				     atomic_long_read(&tg->load_avg) + 1);

	runnable_avg = atomic_read(&tg->runnable_avg);
	if (runnable_avg < NICE_0_LOAD) {
		se->avg.load_avg_contrib *= runnable_avg;
		se->avg.load_avg_contrib >>= NICE_0_SHIFT;
	}
}
#else
static inline void __update_cfs_rq_tg_load_contrib(struct cfs_rq *cfs_rq,
						   int force_update) {}
static inline void __update_tg_runnable_avg(struct sched_avg *sa,
					    struct cfs_rq *cfs_rq) {}
static inline void __update_group_entity_contrib(struct sched_entity *se) {}
#endif

static inline void __update_task_entity_contrib(struct sched_entity *se)
{
	u32 contrib;

	
	contrib = se->avg.runnable_avg_sum * scale_load_down(se->load.weight);
	contrib /= (se->avg.runnable_avg_period + 1);
	se->avg.load_avg_contrib = scale_load(contrib);
}

static long __update_entity_load_avg_contrib(struct sched_entity *se)
{
	long old_contrib = se->avg.load_avg_contrib;

	if (entity_is_task(se)) {
		__update_task_entity_contrib(se);
	} else {
		__update_tg_runnable_avg(&se->avg, group_cfs_rq(se));
		__update_group_entity_contrib(se);
	}

	return se->avg.load_avg_contrib - old_contrib;
}

#if defined(CONFIG_MTK_SCHED_CMP) || defined(CONFIG_SCHED_HMP_ENHANCEMENT)
static long __update_task_entity_ratio(struct sched_entity *se)
{
	long old_ratio = se->avg.load_avg_ratio;
	u32 ratio;

	ratio = se->avg.runnable_avg_sum * scale_load_down(NICE_0_LOAD);
	ratio /= (se->avg.runnable_avg_period + 1);
	se->avg.load_avg_ratio = scale_load(ratio);

	return se->avg.load_avg_ratio - old_ratio;
}
#else
static inline long __update_task_entity_ratio(struct sched_entity *se) { return 0; }
#endif

static inline void subtract_blocked_load_contrib(struct cfs_rq *cfs_rq,
						 long load_contrib)
{
	if (likely(load_contrib < cfs_rq->blocked_load_avg))
		cfs_rq->blocked_load_avg -= load_contrib;
	else
		cfs_rq->blocked_load_avg = 0;
}

#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
unsigned int hmp_up_prio = NICE_TO_PRIO(CONFIG_SCHED_HMP_PRIO_FILTER_VAL);
#endif

#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
#define se_pid(se) ((se != NULL && entity_is_task(se))? \
			container_of(se,struct task_struct,se)->pid:-1)
#define se_load(se) se->avg.load_avg_ratio
#define se_contrib(se) se->avg.load_avg_contrib

#define cfs_pending_load(cpu) cpu_rq(cpu)->cfs.avg.pending_load
#define cfs_load(cpu) cpu_rq(cpu)->cfs.avg.load_avg_ratio
#define cfs_contrib(cpu) cpu_rq(cpu)->cfs.avg.load_avg_contrib

#define cfs_nr_normal_prio(cpu) cpu_rq(cpu)->cfs.avg.nr_normal_prio
#define cfs_nr_pending(cpu) cpu_rq(cpu)->cfs.avg.nr_pending
#define cfs_length(cpu) cpu_rq(cpu)->cfs.h_nr_running
#define rq_length(cpu) (cpu_rq(cpu)->nr_running + cfs_nr_pending(cpu))

#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
#define task_low_priority(prio) ((prio >= hmp_up_prio)?1:0)
#define cfs_nr_dequeuing_low_prio(cpu) \
			cpu_rq(cpu)->cfs.avg.nr_dequeuing_low_prio
#define cfs_reset_nr_dequeuing_low_prio(cpu) \
			(cfs_nr_dequeuing_low_prio(cpu) = 0)
#else
#define task_low_priority(prio) (0)
#define cfs_reset_nr_dequeuing_low_prio(cpu)
#endif 
#endif 

static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq);

#ifdef CONFIG_MTK_SCHED_CMP_TGS
int group_leader_is_empty(struct task_struct *p) {

	struct task_struct *tg = p->group_leader;

	if ((SIGNAL_GROUP_EXIT & p->signal->flags) || (PF_EXITING & p->flags )){
	
	
	
		return 1;
	}

	return 0;
}

static inline void update_tg_info(struct cfs_rq *cfs_rq, struct sched_entity *se, long ratio_delta)
{
	struct task_struct *p = task_of(se);
	struct task_struct *tg = p->group_leader;
	int id;
	unsigned long flags;

	if (group_leader_is_empty(p))
		return;
	id = get_cluster_id(cfs_rq->rq->cpu);
	if (unlikely(WARN_ON(id < 0)))
		return;

	raw_spin_lock_irqsave(&tg->thread_group_info_lock, flags);
	tg->thread_group_info[id].load_avg_ratio += ratio_delta;
	raw_spin_unlock_irqrestore(&tg->thread_group_info_lock, flags);

#ifdef CONFIG_MT_SCHED_INFO
	mt_sched_printf("update_tg_info %d:%s %d:%s %ld %ld %d %d %lu:%lu:%lu update",
	   tg->pid, tg->comm, p->pid, p->comm, 
	   se->avg.load_avg_ratio, ratio_delta,
	   cfs_rq->rq->cpu, id,
	   tg->thread_group_info[id].nr_running,
	   tg->thread_group_info[id].cfs_nr_running,
	   tg->thread_group_info[id].load_avg_ratio);
#endif

}
#endif 

static inline void update_entity_load_avg(struct sched_entity *se,
					  int update_cfs_rq)
{
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	long contrib_delta;
	u64 now;
	long ratio_delta = 0;
	int cpu = -1;   

#if defined(CONFIG_HMP_FREQUENCY_INVARIANT_SCALE)			\
	|| defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	cpu = cfs_rq->rq->cpu;
#endif

	if (entity_is_task(se))
		now = cfs_rq_clock_task(cfs_rq);
	else
		now = cfs_rq_clock_task(group_cfs_rq(se));

	if (!__update_entity_runnable_avg(now, &se->avg, se->on_rq,
				   cfs_rq->curr == se, cpu)) {
#if 0
		if (entity_is_task(se)) {
			ratio_delta = __update_task_entity_ratio(se);
			if (update_cfs_rq)
			{
				cpu = cfs_rq->rq->cpu;
				cpu_rq(cpu)->cfs.avg.load_avg_ratio += ratio_delta;
#ifdef CONFIG_HMP_TRACER
				trace_sched_cfs_load_update(task_of(se),se_load(se),ratio_delta, cpu);
#endif 
			}

			trace_sched_task_entity_avg(2, task_of(se), &se->avg);
#ifdef CONFIG_MTK_SCHED_CMP_TGS
			if (se->on_rq) {
				update_tg_info(cfs_rq, se, ratio_delta);
			}
#endif
		}
#endif
		return;
	}

	contrib_delta = __update_entity_load_avg_contrib(se);

	
	if (entity_is_task(se)) {
		ratio_delta = __update_task_entity_ratio(se);
		trace_sched_task_entity_avg(3, task_of(se), &se->avg);
	}

	if (!update_cfs_rq)
		return;

	if (se->on_rq) {
		cfs_rq->runnable_load_avg += contrib_delta;
		if (entity_is_task(se)) {
			cpu = cfs_rq->rq->cpu;
			cpu_rq(cpu)->cfs.avg.load_avg_ratio += ratio_delta;
			cpu_rq(cpu)->cfs.avg.load_avg_contrib += contrib_delta;
#ifdef CONFIG_HMP_TRACER
			trace_sched_cfs_load_update(task_of(se),se_load(se),ratio_delta,cpu);
#endif 
#ifdef CONFIG_MTK_SCHED_CMP_TGS
			update_tg_info(cfs_rq, se, ratio_delta);
#endif
		}
	}
	else
		subtract_blocked_load_contrib(cfs_rq, -contrib_delta);
}


static void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq, int force_update)
{
	u64 now = cfs_rq_clock_task(cfs_rq) >> 20;
	u64 decays;

	decays = now - cfs_rq->last_decay;
	if (!decays && !force_update)
		return;

	if (atomic_long_read(&cfs_rq->removed_load)) {
		unsigned long removed_load;
		removed_load = atomic_long_xchg(&cfs_rq->removed_load, 0);
		subtract_blocked_load_contrib(cfs_rq, removed_load);
	}

	if (decays) {
		cfs_rq->blocked_load_avg = decay_load(cfs_rq->blocked_load_avg,
						      decays);
		atomic64_add(decays, &cfs_rq->decay_counter);
		cfs_rq->last_decay = now;
	}

	__update_cfs_rq_tg_load_contrib(cfs_rq, force_update);
}

static inline void update_rq_runnable_avg(struct rq *rq, int runnable)
{
	u32 contrib;
	int cpu = -1;	

#if defined(CONFIG_HMP_FREQUENCY_INVARIANT_SCALE)			\
	|| defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	cpu = rq->cpu;
#endif
	__update_entity_runnable_avg(rq->clock_task, &rq->avg, runnable,
			      runnable, cpu);
	__update_tg_runnable_avg(&rq->avg, &rq->cfs);
	contrib = rq->avg.runnable_avg_sum * scale_load_down(1024);
	contrib /= (rq->avg.runnable_avg_period + 1);
	trace_sched_rq_runnable_ratio(cpu_of(rq), scale_load(contrib));
	trace_sched_rq_runnable_load(cpu_of(rq), rq->cfs.runnable_load_avg);
}

static inline void enqueue_entity_load_avg(struct cfs_rq *cfs_rq,
						  struct sched_entity *se,
						  int wakeup)
{
	int cpu = cfs_rq->rq->cpu;

	if (unlikely(se->avg.decay_count <= 0)) {
		se->avg.last_runnable_update = rq_of(cfs_rq)->clock_task;
		if (se->avg.decay_count) {
			se->avg.last_runnable_update -= (-se->avg.decay_count)
							<< 20;
			update_entity_load_avg(se, 0);
			
			se->avg.decay_count = 0;
#ifdef CONFIG_MTK_SCHED_CMP 
		} else {
			if (entity_is_task(se))
				trace_sched_task_entity_avg(1, task_of(se), &se->avg);
#endif
		}
		wakeup = 0;
	} else {
		__synchronize_entity_decay(se);
	}

	
	if (wakeup) {
		subtract_blocked_load_contrib(cfs_rq, se->avg.load_avg_contrib);
		update_entity_load_avg(se, 0);
	}

	cfs_rq->runnable_load_avg += se->avg.load_avg_contrib;
#ifdef CONFIG_MTK_SCHED_CMP_TGS
	if(entity_is_task(se)){
		update_tg_info(cfs_rq, se, se->avg.load_avg_ratio);
	}	
#endif

	if (entity_is_task(se)) {
		cpu_rq(cpu)->cfs.avg.load_avg_contrib += se->avg.load_avg_contrib;
		cpu_rq(cpu)->cfs.avg.load_avg_ratio += se->avg.load_avg_ratio;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
		cfs_nr_pending(cpu) = 0;
		cfs_pending_load(cpu) = 0;
#endif
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
		if(!task_low_priority(task_of(se)->prio))
			cfs_nr_normal_prio(cpu)++;
#endif
#ifdef CONFIG_HMP_TRACER
		trace_sched_cfs_enqueue_task(task_of(se),se_load(se),cpu);
#endif
	}

	
	update_cfs_rq_blocked_load(cfs_rq, !wakeup);
}

static inline void dequeue_entity_load_avg(struct cfs_rq *cfs_rq,
						  struct sched_entity *se,
						  int sleep)
{
	int cpu = cfs_rq->rq->cpu;

	update_entity_load_avg(se, 1);
	
	update_cfs_rq_blocked_load(cfs_rq, !sleep);

	cfs_rq->runnable_load_avg -= se->avg.load_avg_contrib;
#ifdef CONFIG_MTK_SCHED_CMP_TGS
	if(entity_is_task(se)){
		update_tg_info(cfs_rq, se, -se->avg.load_avg_ratio);
	}	
#endif

	if (entity_is_task(se)) {
		cpu_rq(cpu)->cfs.avg.load_avg_contrib -= se->avg.load_avg_contrib;
		cpu_rq(cpu)->cfs.avg.load_avg_ratio -= se->avg.load_avg_ratio;
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
		cfs_reset_nr_dequeuing_low_prio(cpu);
		if(!task_low_priority(task_of(se)->prio))
			cfs_nr_normal_prio(cpu)--;
#endif
#ifdef CONFIG_HMP_TRACER
		trace_sched_cfs_dequeue_task(task_of(se),se_load(se),cpu);
#endif
	}

	if (sleep) {
		cfs_rq->blocked_load_avg += se->avg.load_avg_contrib;
		se->avg.decay_count = atomic64_read(&cfs_rq->decay_counter);
	} 
}

void idle_enter_fair(struct rq *this_rq)
{
	update_rq_runnable_avg(this_rq, 1);
}

void idle_exit_fair(struct rq *this_rq)
{
	update_rq_runnable_avg(this_rq, 0);
}

#else
static inline void update_entity_load_avg(struct sched_entity *se,
					  int update_cfs_rq) {}
static inline void update_rq_runnable_avg(struct rq *rq, int runnable) {}
static inline void enqueue_entity_load_avg(struct cfs_rq *cfs_rq,
					   struct sched_entity *se,
					   int wakeup) {}
static inline void dequeue_entity_load_avg(struct cfs_rq *cfs_rq,
					   struct sched_entity *se,
					   int sleep) {}
static inline void update_cfs_rq_blocked_load(struct cfs_rq *cfs_rq,
					      int force_update) {}
#endif

static void enqueue_sleeper(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHEDSTATS
	struct task_struct *tsk = NULL;

	if (entity_is_task(se))
		tsk = task_of(se);

	if (se->statistics.sleep_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.sleep_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.sleep_max))
			se->statistics.sleep_max = delta;

		se->statistics.sleep_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			account_scheduler_latency(tsk, delta >> 10, 1);
			trace_sched_stat_sleep(tsk, delta);
		}
	}
	if (se->statistics.block_start) {
		u64 delta = rq_of(cfs_rq)->clock - se->statistics.block_start;

		if ((s64)delta < 0)
			delta = 0;

		if (unlikely(delta > se->statistics.block_max))
			se->statistics.block_max = delta;

		se->statistics.block_start = 0;
		se->statistics.sum_sleep_runtime += delta;

		if (tsk) {
			if (tsk->in_iowait) {
				se->statistics.iowait_sum += delta;
				se->statistics.iowait_count++;
				trace_sched_stat_iowait(tsk, delta);
			}

			trace_sched_stat_blocked(tsk, delta);

			if (unlikely(prof_on == SLEEP_PROFILING)) {
				profile_hits(SLEEP_PROFILING,
						(void *)get_wchan(tsk),
						delta >> 20);
			}
			account_scheduler_latency(tsk, delta >> 10, 0);
		}
	}
#endif
}

static void check_spread(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	s64 d = se->vruntime - cfs_rq->min_vruntime;

	if (d < 0)
		d = -d;

	if (d > 3*sysctl_sched_latency)
		schedstat_inc(cfs_rq, nr_spread_over);
#endif
}

static void
place_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int initial)
{
	u64 vruntime = cfs_rq->min_vruntime;

	if (initial && sched_feat(START_DEBIT))
		vruntime += sched_vslice(cfs_rq, se);

	
	if (!initial) {
		unsigned long thresh = sysctl_sched_latency;

		if (sched_feat(GENTLE_FAIR_SLEEPERS))
			thresh >>= 1;

		vruntime -= thresh;
	}

	
	se->vruntime = max_vruntime(se->vruntime, vruntime);
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq);

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	if (!(flags & ENQUEUE_WAKEUP) || (flags & ENQUEUE_WAKING))
		se->vruntime += cfs_rq->min_vruntime;

	update_curr(cfs_rq);
	enqueue_entity_load_avg(cfs_rq, se, flags & ENQUEUE_WAKEUP);
	account_entity_enqueue(cfs_rq, se);
	update_cfs_shares(cfs_rq);

	if (flags & ENQUEUE_WAKEUP) {
		place_entity(cfs_rq, se, 0);
		enqueue_sleeper(cfs_rq, se);
	}

	update_stats_enqueue(cfs_rq, se);
	check_spread(cfs_rq, se);
	if (se != cfs_rq->curr)
		__enqueue_entity(cfs_rq, se);
	se->on_rq = 1;

	if (cfs_rq->nr_running == 1) {
		list_add_leaf_cfs_rq(cfs_rq);
		check_enqueue_throttle(cfs_rq);
	}
}

static void __clear_buddies_last(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->last == se)
			cfs_rq->last = NULL;
		else
			break;
	}
}

static void __clear_buddies_next(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->next == se)
			cfs_rq->next = NULL;
		else
			break;
	}
}

static void __clear_buddies_skip(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->skip == se)
			cfs_rq->skip = NULL;
		else
			break;
	}
}

static void clear_buddies(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->last == se)
		__clear_buddies_last(se);

	if (cfs_rq->next == se)
		__clear_buddies_next(se);

	if (cfs_rq->skip == se)
		__clear_buddies_skip(se);
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	update_curr(cfs_rq);
	dequeue_entity_load_avg(cfs_rq, se, flags & DEQUEUE_SLEEP);

	update_stats_dequeue(cfs_rq, se);
	if (flags & DEQUEUE_SLEEP) {
#ifdef CONFIG_SCHEDSTATS
		if (entity_is_task(se)) {
			struct task_struct *tsk = task_of(se);

			if (tsk->state & TASK_INTERRUPTIBLE)
				se->statistics.sleep_start = rq_of(cfs_rq)->clock;
			if (tsk->state & TASK_UNINTERRUPTIBLE)
				se->statistics.block_start = rq_of(cfs_rq)->clock;
		}
#endif
	}

	clear_buddies(cfs_rq, se);

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, se);
	se->on_rq = 0;
	account_entity_dequeue(cfs_rq, se);

	if (!(flags & DEQUEUE_SLEEP))
		se->vruntime -= cfs_rq->min_vruntime;

	
	return_cfs_rq_runtime(cfs_rq);

	update_min_vruntime(cfs_rq);
	update_cfs_shares(cfs_rq);
}

static void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned long ideal_runtime, delta_exec;
	struct sched_entity *se;
	s64 delta;

	ideal_runtime = sched_slice(cfs_rq, curr);
	delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	if (delta_exec > ideal_runtime) {
		resched_task(rq_of(cfs_rq)->curr);
		clear_buddies(cfs_rq, curr);
		return;
	}

	if (delta_exec < sysctl_sched_min_granularity)
		return;

	se = __pick_first_entity(cfs_rq);
	delta = curr->vruntime - se->vruntime;

	if (delta < 0)
		return;

	if (delta > ideal_runtime)
		resched_task(rq_of(cfs_rq)->curr);
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	
	if (se->on_rq) {
		update_stats_wait_end(cfs_rq, se);
		__dequeue_entity(cfs_rq, se);
		update_entity_load_avg(se, 1);
	}

	update_stats_curr_start(cfs_rq, se);
	cfs_rq->curr = se;
#ifdef CONFIG_SCHEDSTATS
	if (rq_of(cfs_rq)->load.weight >= 2*se->load.weight) {
		se->statistics.slice_max = max(se->statistics.slice_max,
			se->sum_exec_runtime - se->prev_sum_exec_runtime);
	}
#endif
	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se);

static struct sched_entity *pick_next_entity(struct cfs_rq *cfs_rq)
{
	struct sched_entity *se = __pick_first_entity(cfs_rq);
	struct sched_entity *left = se;

	if (cfs_rq->skip == se) {
		struct sched_entity *second = __pick_next_entity(se);
		if (second && wakeup_preempt_entity(second, left) < 1)
			se = second;
	}

	if (cfs_rq->last && wakeup_preempt_entity(cfs_rq->last, left) < 1)
		se = cfs_rq->last;

	if (cfs_rq->next && wakeup_preempt_entity(cfs_rq->next, left) < 1)
		se = cfs_rq->next;

	clear_buddies(cfs_rq, se);

	return se;
}

static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	if (prev->on_rq)
		update_curr(cfs_rq);

	
	check_cfs_rq_runtime(cfs_rq);

	check_spread(cfs_rq, prev);
	if (prev->on_rq) {
		update_stats_wait_start(cfs_rq, prev);
		
		__enqueue_entity(cfs_rq, prev);
		
		update_entity_load_avg(prev, 1);
	}
	cfs_rq->curr = NULL;
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	update_curr(cfs_rq);

	update_entity_load_avg(curr, 1);
	update_cfs_rq_blocked_load(cfs_rq, 1);
	update_cfs_shares(cfs_rq);

#ifdef CONFIG_SCHED_HRTICK
	if (queued) {
		resched_task(rq_of(cfs_rq)->curr);
		return;
	}
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif

	if (cfs_rq->nr_running > 1)
		check_preempt_tick(cfs_rq, curr);
}



#ifdef CONFIG_CFS_BANDWIDTH

#ifdef HAVE_JUMP_LABEL
static struct static_key __cfs_bandwidth_used;

static inline bool cfs_bandwidth_used(void)
{
	return static_key_false(&__cfs_bandwidth_used);
}

void cfs_bandwidth_usage_inc(void)
{
	static_key_slow_inc(&__cfs_bandwidth_used);
}

void cfs_bandwidth_usage_dec(void)
{
	static_key_slow_dec(&__cfs_bandwidth_used);
}
#else 
static bool cfs_bandwidth_used(void)
{
	return true;
}

void cfs_bandwidth_usage_inc(void) {}
void cfs_bandwidth_usage_dec(void) {}
#endif 

static inline u64 default_cfs_period(void)
{
	return 100000000ULL;
}

static inline u64 sched_cfs_bandwidth_slice(void)
{
	return (u64)sysctl_sched_cfs_bandwidth_slice * NSEC_PER_USEC;
}

void __refill_cfs_bandwidth_runtime(struct cfs_bandwidth *cfs_b)
{
	u64 now;

	if (cfs_b->quota == RUNTIME_INF)
		return;

	now = sched_clock_cpu(smp_processor_id());
	cfs_b->runtime = cfs_b->quota;
	cfs_b->runtime_expires = now + ktime_to_ns(cfs_b->period);
}

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return &tg->cfs_bandwidth;
}

static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq)
{
	if (unlikely(cfs_rq->throttle_count))
		return cfs_rq->throttled_clock_task;

	return rq_of(cfs_rq)->clock_task - cfs_rq->throttled_clock_task_time;
}

static int assign_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct task_group *tg = cfs_rq->tg;
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(tg);
	u64 amount = 0, min_amount, expires;

	
	min_amount = sched_cfs_bandwidth_slice() - cfs_rq->runtime_remaining;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota == RUNTIME_INF)
		amount = min_amount;
	else {
		if (!cfs_b->timer_active) {
			__refill_cfs_bandwidth_runtime(cfs_b);
			__start_cfs_bandwidth(cfs_b);
		}

		if (cfs_b->runtime > 0) {
			amount = min(cfs_b->runtime, min_amount);
			cfs_b->runtime -= amount;
			cfs_b->idle = 0;
		}
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	cfs_rq->runtime_remaining += amount;
	if ((s64)(expires - cfs_rq->runtime_expires) > 0)
		cfs_rq->runtime_expires = expires;

	return cfs_rq->runtime_remaining > 0;
}

static void expire_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct rq *rq = rq_of(cfs_rq);

	
	if (likely((s64)(rq->clock - cfs_rq->runtime_expires) < 0))
		return;

	if (cfs_rq->runtime_remaining < 0)
		return;


	if ((s64)(cfs_rq->runtime_expires - cfs_b->runtime_expires) >= 0) {
		
		cfs_rq->runtime_expires += TICK_NSEC;
	} else {
		
		cfs_rq->runtime_remaining = 0;
	}
}

static void __account_cfs_rq_runtime(struct cfs_rq *cfs_rq,
				     unsigned long delta_exec)
{
	
	cfs_rq->runtime_remaining -= delta_exec;
	expire_cfs_rq_runtime(cfs_rq);

	if (likely(cfs_rq->runtime_remaining > 0))
		return;

	if (!assign_cfs_rq_runtime(cfs_rq) && likely(cfs_rq->curr))
		resched_task(rq_of(cfs_rq)->curr);
}

static __always_inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, unsigned long delta_exec)
{
	if (!cfs_bandwidth_used() || !cfs_rq->runtime_enabled)
		return;

	__account_cfs_rq_runtime(cfs_rq, delta_exec);
}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttled;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return cfs_bandwidth_used() && cfs_rq->throttle_count;
}

static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	struct cfs_rq *src_cfs_rq, *dest_cfs_rq;

	src_cfs_rq = tg->cfs_rq[src_cpu];
	dest_cfs_rq = tg->cfs_rq[dest_cpu];

	return throttled_hierarchy(src_cfs_rq) ||
	       throttled_hierarchy(dest_cfs_rq);
}

static int tg_unthrottle_up(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	cfs_rq->throttle_count--;
#ifdef CONFIG_SMP
	if (!cfs_rq->throttle_count) {
		
		cfs_rq->throttled_clock_task_time += rq->clock_task -
					     cfs_rq->throttled_clock_task;
	}
#endif

	return 0;
}

static int tg_throttle_down(struct task_group *tg, void *data)
{
	struct rq *rq = data;
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

	
	if (!cfs_rq->throttle_count)
		cfs_rq->throttled_clock_task = rq->clock_task;
	cfs_rq->throttle_count++;

	return 0;
}

static void throttle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	long task_delta, dequeue = 1;

	se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

	
	rcu_read_lock();
	walk_tg_tree_from(cfs_rq->tg, tg_throttle_down, tg_nop, (void *)rq);
	rcu_read_unlock();

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		struct cfs_rq *qcfs_rq = cfs_rq_of(se);
		
		if (!se->on_rq)
			break;

		if (dequeue)
			dequeue_entity(qcfs_rq, se, DEQUEUE_SLEEP);
		qcfs_rq->h_nr_running -= task_delta;

		if (qcfs_rq->load.weight)
			dequeue = 0;
	}

	if (!se)
		rq->nr_running -= task_delta;

	cfs_rq->throttled = 1;
	cfs_rq->throttled_clock = rq->clock;
	raw_spin_lock(&cfs_b->lock);
	list_add_tail_rcu(&cfs_rq->throttled_list, &cfs_b->throttled_cfs_rq);
	if (!cfs_b->timer_active)
		__start_cfs_bandwidth(cfs_b);
	raw_spin_unlock(&cfs_b->lock);
}

void unthrottle_cfs_rq(struct cfs_rq *cfs_rq)
{
	struct rq *rq = rq_of(cfs_rq);
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	struct sched_entity *se;
	int enqueue = 1;
	long task_delta;

	se = cfs_rq->tg->se[cpu_of(rq)];

	cfs_rq->throttled = 0;
	raw_spin_lock(&cfs_b->lock);
	cfs_b->throttled_time += rq->clock - cfs_rq->throttled_clock;
	list_del_rcu(&cfs_rq->throttled_list);
	raw_spin_unlock(&cfs_b->lock);

	update_rq_clock(rq);
	
	walk_tg_tree_from(cfs_rq->tg, tg_nop, tg_unthrottle_up, (void *)rq);

	if (!cfs_rq->load.weight)
		return;

	task_delta = cfs_rq->h_nr_running;
	for_each_sched_entity(se) {
		if (se->on_rq)
			enqueue = 0;

		cfs_rq = cfs_rq_of(se);
		if (enqueue)
			enqueue_entity(cfs_rq, se, ENQUEUE_WAKEUP);
		cfs_rq->h_nr_running += task_delta;

		if (cfs_rq_throttled(cfs_rq))
			break;
	}

	if (!se)
		rq->nr_running += task_delta;

	
	if (rq->curr == rq->idle && rq->cfs.nr_running)
		resched_task(rq->curr);
}

static u64 distribute_cfs_runtime(struct cfs_bandwidth *cfs_b,
		u64 remaining, u64 expires)
{
	struct cfs_rq *cfs_rq;
	u64 runtime = remaining;

	rcu_read_lock();
	list_for_each_entry_rcu(cfs_rq, &cfs_b->throttled_cfs_rq,
				throttled_list) {
		struct rq *rq = rq_of(cfs_rq);

		raw_spin_lock(&rq->lock);
		if (!cfs_rq_throttled(cfs_rq))
			goto next;

		runtime = -cfs_rq->runtime_remaining + 1;
		if (runtime > remaining)
			runtime = remaining;
		remaining -= runtime;

		cfs_rq->runtime_remaining += runtime;
		cfs_rq->runtime_expires = expires;

		
		if (cfs_rq->runtime_remaining > 0)
			unthrottle_cfs_rq(cfs_rq);

next:
		raw_spin_unlock(&rq->lock);

		if (!remaining)
			break;
	}
	rcu_read_unlock();

	return remaining;
}

static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun)
{
	u64 runtime, runtime_expires;
	int idle = 1, throttled;

	raw_spin_lock(&cfs_b->lock);
	
	if (cfs_b->quota == RUNTIME_INF)
		goto out_unlock;

	throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	
	idle = cfs_b->idle && !throttled;
	cfs_b->nr_periods += overrun;

	
	if (idle)
		goto out_unlock;

	cfs_b->timer_active = 1;

	__refill_cfs_bandwidth_runtime(cfs_b);

	if (!throttled) {
		
		cfs_b->idle = 1;
		goto out_unlock;
	}

	
	cfs_b->nr_throttled += overrun;

	runtime = cfs_b->runtime;
	runtime_expires = cfs_b->runtime_expires;
	cfs_b->runtime = 0;

	while (throttled && runtime > 0) {
		raw_spin_unlock(&cfs_b->lock);
		
		runtime = distribute_cfs_runtime(cfs_b, runtime,
						 runtime_expires);
		raw_spin_lock(&cfs_b->lock);

		throttled = !list_empty(&cfs_b->throttled_cfs_rq);
	}

	
	cfs_b->runtime = runtime;
	cfs_b->idle = 0;
out_unlock:
	if (idle)
		cfs_b->timer_active = 0;
	raw_spin_unlock(&cfs_b->lock);

	return idle;
}

static const u64 min_cfs_rq_runtime = 1 * NSEC_PER_MSEC;
static const u64 min_bandwidth_expiration = 2 * NSEC_PER_MSEC;
static const u64 cfs_bandwidth_slack_period = 5 * NSEC_PER_MSEC;

static int runtime_refresh_within(struct cfs_bandwidth *cfs_b, u64 min_expire)
{
	struct hrtimer *refresh_timer = &cfs_b->period_timer;
	u64 remaining;

	
	if (hrtimer_callback_running(refresh_timer))
		return 1;

	
	remaining = ktime_to_ns(hrtimer_expires_remaining(refresh_timer));
	if (remaining < min_expire)
		return 1;

	return 0;
}

static void start_cfs_slack_bandwidth(struct cfs_bandwidth *cfs_b)
{
	u64 min_left = cfs_bandwidth_slack_period + min_bandwidth_expiration;

	
	if (runtime_refresh_within(cfs_b, min_left))
		return;

	start_bandwidth_timer(&cfs_b->slack_timer,
				ns_to_ktime(cfs_bandwidth_slack_period));
}

static void __return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
	s64 slack_runtime = cfs_rq->runtime_remaining - min_cfs_rq_runtime;

	if (slack_runtime <= 0)
		return;

	raw_spin_lock(&cfs_b->lock);
	if (cfs_b->quota != RUNTIME_INF &&
	    cfs_rq->runtime_expires == cfs_b->runtime_expires) {
		cfs_b->runtime += slack_runtime;

		
		if (cfs_b->runtime > sched_cfs_bandwidth_slice() &&
		    !list_empty(&cfs_b->throttled_cfs_rq))
			start_cfs_slack_bandwidth(cfs_b);
	}
	raw_spin_unlock(&cfs_b->lock);

	
	cfs_rq->runtime_remaining -= slack_runtime;
}

static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (!cfs_rq->runtime_enabled || cfs_rq->nr_running)
		return;

	__return_cfs_rq_runtime(cfs_rq);
}

static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b)
{
	u64 runtime = 0, slice = sched_cfs_bandwidth_slice();
	u64 expires;

	
	raw_spin_lock(&cfs_b->lock);
	if (runtime_refresh_within(cfs_b, min_bandwidth_expiration)) {
		raw_spin_unlock(&cfs_b->lock);
		return;
	}

	if (cfs_b->quota != RUNTIME_INF && cfs_b->runtime > slice) {
		runtime = cfs_b->runtime;
		cfs_b->runtime = 0;
	}
	expires = cfs_b->runtime_expires;
	raw_spin_unlock(&cfs_b->lock);

	if (!runtime)
		return;

	runtime = distribute_cfs_runtime(cfs_b, runtime, expires);

	raw_spin_lock(&cfs_b->lock);
	if (expires == cfs_b->runtime_expires)
		cfs_b->runtime = runtime;
	raw_spin_unlock(&cfs_b->lock);
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	
	if (!cfs_rq->runtime_enabled || cfs_rq->curr)
		return;

	
	if (cfs_rq_throttled(cfs_rq))
		return;

	
	account_cfs_rq_runtime(cfs_rq, 0);
	if (cfs_rq->runtime_remaining <= 0)
		throttle_cfs_rq(cfs_rq);
}

static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	if (!cfs_bandwidth_used())
		return;

	if (likely(!cfs_rq->runtime_enabled || cfs_rq->runtime_remaining > 0))
		return;

	if (cfs_rq_throttled(cfs_rq))
		return;

	throttle_cfs_rq(cfs_rq);
}

static inline u64 default_cfs_period(void);
static int do_sched_cfs_period_timer(struct cfs_bandwidth *cfs_b, int overrun);
static void do_sched_cfs_slack_timer(struct cfs_bandwidth *cfs_b);

static enum hrtimer_restart sched_cfs_slack_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, slack_timer);
	do_sched_cfs_slack_timer(cfs_b);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart sched_cfs_period_timer(struct hrtimer *timer)
{
	struct cfs_bandwidth *cfs_b =
		container_of(timer, struct cfs_bandwidth, period_timer);
	ktime_t now;
	int overrun;
	int idle = 0;

	for (;;) {
		now = hrtimer_cb_get_time(timer);
		overrun = hrtimer_forward(timer, now, cfs_b->period);

		if (!overrun)
			break;

		idle = do_sched_cfs_period_timer(cfs_b, overrun);
	}

	return idle ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	raw_spin_lock_init(&cfs_b->lock);
	cfs_b->runtime = 0;
	cfs_b->quota = RUNTIME_INF;
	cfs_b->period = ns_to_ktime(default_cfs_period());

	INIT_LIST_HEAD(&cfs_b->throttled_cfs_rq);
	hrtimer_init(&cfs_b->period_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->period_timer.function = sched_cfs_period_timer;
	hrtimer_init(&cfs_b->slack_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cfs_b->slack_timer.function = sched_cfs_slack_timer;
}

static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq)
{
	cfs_rq->runtime_enabled = 0;
	INIT_LIST_HEAD(&cfs_rq->throttled_list);
}

void __start_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	while (unlikely(hrtimer_active(&cfs_b->period_timer)) &&
	       hrtimer_try_to_cancel(&cfs_b->period_timer) < 0) {
		
		raw_spin_unlock(&cfs_b->lock);
		cpu_relax();
		raw_spin_lock(&cfs_b->lock);
		
		if (cfs_b->timer_active)
			return;
	}

	cfs_b->timer_active = 1;
	start_bandwidth_timer(&cfs_b->period_timer, cfs_b->period);
}

static void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b)
{
	hrtimer_cancel(&cfs_b->period_timer);
	hrtimer_cancel(&cfs_b->slack_timer);
}

static void __maybe_unused unthrottle_offline_cfs_rqs(struct rq *rq)
{
	struct cfs_rq *cfs_rq;

	for_each_leaf_cfs_rq(rq, cfs_rq) {
		struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);

		if (!cfs_rq->runtime_enabled)
			continue;

		cfs_rq->runtime_remaining = cfs_b->quota;
		if (cfs_rq_throttled(cfs_rq))
			unthrottle_cfs_rq(cfs_rq);
	}
}

#else 
static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq)
{
	return rq_of(cfs_rq)->clock_task;
}

static void account_cfs_rq_runtime(struct cfs_rq *cfs_rq,
				     unsigned long delta_exec) {}
static void check_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
static void check_enqueue_throttle(struct cfs_rq *cfs_rq) {}
static __always_inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_lb_pair(struct task_group *tg,
				    int src_cpu, int dest_cpu)
{
	return 0;
}

void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void init_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}
#endif

static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
{
	return NULL;
}
static inline void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}
static inline void unthrottle_offline_cfs_rqs(struct rq *rq) {}

#endif 


#ifdef CONFIG_SCHED_HRTICK
static void hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	WARN_ON(task_rq(p) != rq);

	if (cfs_rq->nr_running > 1) {
		u64 slice = sched_slice(cfs_rq, se);
		u64 ran = se->sum_exec_runtime - se->prev_sum_exec_runtime;
		s64 delta = slice - ran;

		if (delta < 0) {
			if (rq->curr == p)
				resched_task(p);
			return;
		}

		if (rq->curr != p)
			delta = max_t(s64, 10000LL, delta);

		hrtick_start(rq, delta);
	}
}

static void hrtick_update(struct rq *rq)
{
	struct task_struct *curr = rq->curr;

	if (!hrtick_enabled(rq) || curr->sched_class != &fair_sched_class)
		return;

	if (cfs_rq_of(&curr->se)->nr_running < sched_nr_latency)
		hrtick_start_fair(rq, curr);
}
#else 
static inline void
hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
}

static inline void hrtick_update(struct rq *rq)
{
}
#endif

#if defined(CONFIG_SCHED_HMP) || defined(CONFIG_MTK_SCHED_CMP)

#define HMP_GB (0x1000)
#define HMP_SELECT_RQ (0x2000)
#define HMP_LB (0x4000)
#define HMP_MAX_LOAD (NICE_0_LOAD - 1)


struct clb_env {
	struct clb_stats bstats;
	struct clb_stats lstats;
	int btarget, ltarget;

	struct cpumask *bcpus;
	struct cpumask *lcpus;

	unsigned int flags;
	struct mcheck {
		int status; 
		int result; 
	} mcheck;
};

unsigned long __weak arch_scale_freq_power(struct sched_domain *sd, int cpu);

static void collect_cluster_stats(struct clb_stats *clbs,
            struct cpumask *cluster_cpus, int target)
{
#define HMP_RESOLUTION_SCALING (4)
#define hmp_scale_down(w) ((w) >> HMP_RESOLUTION_SCALING)

	
	int cpu;
	for_each_cpu(cpu, cluster_cpus) {
		if(cpu_online(cpu)) {
			clbs->ncpu ++;
			clbs->ntask += cpu_rq(cpu)->cfs.h_nr_running;
			clbs->load_avg += cpu_rq(cpu)->cfs.avg.load_avg_ratio;
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
			clbs->nr_normal_prio_task += cfs_nr_normal_prio(cpu);
			clbs->nr_dequeuing_low_prio += cfs_nr_dequeuing_low_prio(cpu);
#endif
		}
	}

	if(!clbs->ncpu || NR_CPUS == target || !cpumask_test_cpu(target,cluster_cpus))
		return;

	clbs->cpu_power = (int) arch_scale_freq_power(NULL, target);

	
	clbs->cpu_capacity = HMP_MAX_LOAD;
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	if (hmp_data.freqinvar_load_scale_enabled) {
		cpu = cpumask_any(cluster_cpus);
		if (freq_scale[cpu].throttling == 1){
			clbs->cpu_capacity *= freq_scale[cpu].curr_scale;
		}else {
		clbs->cpu_capacity *= freq_scale[cpu].max;
		}
		clbs->cpu_capacity >>= SCHED_FREQSCALE_SHIFT;

		if (clbs->cpu_capacity > HMP_MAX_LOAD){
			clbs->cpu_capacity = HMP_MAX_LOAD;
		}
	}
#elif defined(CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY)
	if (topology_cpu_inv_power_en()) {
		cpu = cpumask_any(cluster_cpus);
		if (topology_cpu_throttling(cpu))
			clbs->cpu_capacity *=
				(topology_cpu_capacity(cpu) << CPUPOWER_FREQSCALE_SHIFT)
				/ (topology_max_cpu_capacity(cpu)+1);
		else
			clbs->cpu_capacity *= topology_max_cpu_capacity(cpu);
		clbs->cpu_capacity >>= CPUPOWER_FREQSCALE_SHIFT;

		if (clbs->cpu_capacity > HMP_MAX_LOAD){
			clbs->cpu_capacity = HMP_MAX_LOAD;
		}
	}
#endif

	clbs->load_avg /= clbs->ncpu;
	clbs->acap = clbs->cpu_capacity - cpu_rq(target)->cfs.avg.load_avg_ratio;
	clbs->scaled_acap = hmp_scale_down(clbs->acap);
	clbs->scaled_atask = cpu_rq(target)->cfs.h_nr_running * cpu_rq(target)->cfs.avg.load_avg_ratio;
	clbs->scaled_atask = clbs->cpu_capacity - clbs->scaled_atask;
	clbs->scaled_atask = hmp_scale_down(clbs->scaled_atask);

	mt_sched_printf("[%s] cpu/cluster:%d/%02lx load/len:%lu/%u stats:%d,%d,%d,%d,%d,%d,%d,%d\n", __func__,
					target, *cpumask_bits(cluster_cpus),
					cpu_rq(target)->cfs.avg.load_avg_ratio, cpu_rq(target)->cfs.h_nr_running,
					clbs->ncpu, clbs->ntask, clbs->load_avg, clbs->cpu_capacity,
					clbs->acap, clbs->scaled_acap, clbs->scaled_atask, clbs->threshold);
}

#if defined(CONFIG_SCHED_HMP) && defined(USE_HMP_DYNAMIC_THRESHOLD)
static inline void hmp_dynamic_threshold(struct clb_env *clbenv);
#endif

static void adj_threshold(struct clb_env *clbenv)
{
#define TSKLD_SHIFT (2)
#define POSITIVE(x) ((int)(x) < 0 ? 0 : (x))

	int bcpu, lcpu;
	unsigned long b_cap=0, l_cap=0;
	unsigned long b_load=0, l_load=0;
	unsigned long b_task=0, l_task=0;
	int b_nacap, l_nacap, b_natask, l_natask;

#if defined(CONFIG_SCHED_HMP) && defined(USE_HMP_DYNAMIC_THRESHOLD)
	hmp_dynamic_threshold(clbenv);
	return;
#endif

	bcpu = clbenv->btarget;
	lcpu = clbenv->ltarget;
	if (bcpu < nr_cpu_ids) {
		b_load = cpu_rq(bcpu)->cfs.avg.load_avg_ratio;
		b_task = cpu_rq(bcpu)->cfs.h_nr_running;
	}
	if (lcpu < nr_cpu_ids) {
		l_load = cpu_rq(lcpu)->cfs.avg.load_avg_ratio;
		l_task = cpu_rq(lcpu)->cfs.h_nr_running;
	}

#ifdef CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY
	if (bcpu < nr_cpu_ids) {
		b_cap = topology_cpu_capacity(bcpu);
	}
	if (lcpu < nr_cpu_ids) {
		l_cap = topology_cpu_capacity(lcpu);
	}

	b_nacap = POSITIVE(b_cap - b_load);
	b_natask = POSITIVE(b_cap - ((b_task * b_load) >> TSKLD_SHIFT));
	l_nacap = POSITIVE(l_cap - l_load);
	l_natask = POSITIVE(l_cap - ((l_task * l_load) >> TSKLD_SHIFT));
#else 
	b_cap = clbenv->bstats.cpu_power;
	l_cap = clbenv->lstats.cpu_power;
	b_nacap = POSITIVE(clbenv->bstats.scaled_acap *
					   clbenv->bstats.cpu_power / (clbenv->lstats.cpu_power+1));
	b_natask = POSITIVE(clbenv->bstats.scaled_atask *
						clbenv->bstats.cpu_power / (clbenv->lstats.cpu_power+1));
	l_nacap = POSITIVE(clbenv->lstats.scaled_acap);
	l_natask = POSITIVE(clbenv->bstats.scaled_atask);

#endif 

	clbenv->bstats.threshold = HMP_MAX_LOAD - HMP_MAX_LOAD * b_nacap * b_natask /
		((b_nacap + l_nacap) * (b_natask + l_natask)+1);
	clbenv->lstats.threshold = HMP_MAX_LOAD * l_nacap * l_natask /
		((b_nacap + l_nacap) * (b_natask + l_natask)+1);

	mt_sched_printf("[%s]\tup/dl:%4d/%4d L(%d:%4lu,%4lu/%4lu) b(%d:%4lu,%4lu/%4lu)\n", __func__,
					clbenv->bstats.threshold, clbenv->lstats.threshold,
					lcpu, l_load, l_task, l_cap,
					bcpu, b_load, b_task, b_cap);
}

static void sched_update_clbstats(struct clb_env *clbenv)
{
	collect_cluster_stats(&clbenv->bstats, clbenv->bcpus, clbenv->btarget);
	collect_cluster_stats(&clbenv->lstats, clbenv->lcpus, clbenv->ltarget);
	adj_threshold(clbenv);
}
#endif 


#ifdef CONFIG_SCHED_HMP
DEFINE_PER_CPU(struct hmp_domain *, hmp_cpu_domain);
extern struct cpumask hmp_fast_cpu_mask;
extern struct cpumask hmp_slow_cpu_mask;

extern void __init arch_get_hmp_domains(struct list_head *hmp_domains_list);

static int __init hmp_cpu_mask_setup(void)
{
	char buf[64];
	struct hmp_domain *domain;
	struct list_head *pos;
	int dc, cpu;

#if defined(CONFIG_SCHED_HMP_ENHANCEMENT) || \
		defined(CONFIG_MT_RT_SCHED) || defined(CONFIG_MT_RT_SCHED_LOG)
	cpumask_clear(&hmp_fast_cpu_mask);
	cpumask_clear(&hmp_slow_cpu_mask);
#endif

	pr_debug("Initializing HMP scheduler:\n");

	
	arch_get_hmp_domains(&hmp_domains);
	if (list_empty(&hmp_domains)) {
		pr_debug("HMP domain list is empty!\n");
		return 0;
	}

	
	dc = 0;
	list_for_each(pos, &hmp_domains) {
		domain = list_entry(pos, struct hmp_domain, hmp_domains);
		cpulist_scnprintf(buf, 64, &domain->possible_cpus);
		pr_debug("  HMP domain %d: %s\n", dc, buf);

#if defined(CONFIG_SCHED_HMP_ENHANCEMENT) || \
			defined(CONFIG_MT_RT_SCHED) || defined(CONFIG_MT_RT_SCHED_LOG)
		if(cpumask_empty(&hmp_fast_cpu_mask)) {
			cpumask_copy(&hmp_fast_cpu_mask,&domain->possible_cpus);
			for_each_cpu(cpu, &hmp_fast_cpu_mask)
				pr_debug("  HMP fast cpu : %d\n",cpu);
		} else if (cpumask_empty(&hmp_slow_cpu_mask)){
			cpumask_copy(&hmp_slow_cpu_mask,&domain->possible_cpus);
			for_each_cpu(cpu, &hmp_slow_cpu_mask)
				pr_debug("  HMP slow cpu : %d\n",cpu);
		}
#endif

		for_each_cpu_mask(cpu, domain->possible_cpus) {
			per_cpu(hmp_cpu_domain, cpu) = domain;
		}
		dc++;
	}

	return 1;
}

static struct hmp_domain *hmp_get_hmp_domain_for_cpu(int cpu)
{
	struct hmp_domain *domain;
	struct list_head *pos;

	list_for_each(pos, &hmp_domains) {
		domain = list_entry(pos, struct hmp_domain, hmp_domains);
		if(cpumask_test_cpu(cpu, &domain->possible_cpus))
			return domain;
	}
	return NULL;
}

static void hmp_online_cpu(int cpu)
{
	struct hmp_domain *domain = hmp_get_hmp_domain_for_cpu(cpu);

	if(domain)
		cpumask_set_cpu(cpu, &domain->cpus);
}

static void hmp_offline_cpu(int cpu)
{
	struct hmp_domain *domain = hmp_get_hmp_domain_for_cpu(cpu);

	if(domain)
		cpumask_clear_cpu(cpu, &domain->cpus);
}

#ifdef CONFIG_HMP_DYNAMIC_THRESHOLD
unsigned int hmp_up_threshold = 1023;
unsigned int hmp_down_threshold = 0;
#else
unsigned int hmp_up_threshold = 512;
unsigned int hmp_down_threshold = 256;
#endif

unsigned int hmp_next_up_threshold = 4096;
unsigned int hmp_next_down_threshold = 4096;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
#define hmp_last_up_migration(cpu) \
			cpu_rq(cpu)->cfs.avg.hmp_last_up_migration
#define hmp_last_down_migration(cpu) \
			cpu_rq(cpu)->cfs.avg.hmp_last_down_migration
static int hmp_select_task_rq_fair(int sd_flag, struct task_struct *p, 
			int prev_cpu, int new_cpu);
#else
static unsigned int hmp_up_migration(int cpu, int *target_cpu, struct sched_entity *se);
static unsigned int hmp_down_migration(int cpu, struct sched_entity *se);
#endif
static inline unsigned int hmp_domain_min_load(struct hmp_domain *hmpd,
						int *min_cpu);

static inline unsigned int hmp_cpu_is_fastest(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return pos == hmp_domains.next;
}

static inline unsigned int hmp_cpu_is_slowest(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return list_is_last(pos, &hmp_domains);
}

static inline struct hmp_domain *hmp_slower_domain(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return list_entry(pos->next, struct hmp_domain, hmp_domains);
}

static inline struct hmp_domain *hmp_faster_domain(int cpu)
{
	struct list_head *pos;

	pos = &hmp_cpu_domain(cpu)->hmp_domains;
	return list_entry(pos->prev, struct hmp_domain, hmp_domains);
}

static inline unsigned int hmp_select_faster_cpu(struct task_struct *tsk,
							int cpu)
{
	int lowest_cpu=NR_CPUS;
	__always_unused int lowest_ratio = hmp_domain_min_load(hmp_faster_domain(cpu), &lowest_cpu);
	if(lowest_cpu < nr_cpu_ids && cpumask_test_cpu(lowest_cpu,tsk_cpus_allowed(tsk)))
		return lowest_cpu;
	else
		return cpumask_any_and(&hmp_faster_domain(cpu)->cpus,
				tsk_cpus_allowed(tsk));
}

static inline unsigned int hmp_select_slower_cpu(struct task_struct *tsk,
							int cpu)
{
	int lowest_cpu=NR_CPUS;
	__always_unused int lowest_ratio = hmp_domain_min_load(hmp_slower_domain(cpu), &lowest_cpu);
	if(lowest_cpu < nr_cpu_ids && cpumask_test_cpu(lowest_cpu,tsk_cpus_allowed(tsk)))
		return lowest_cpu;
	else
		return cpumask_any_and(&hmp_slower_domain(cpu)->cpus,
				tsk_cpus_allowed(tsk));
}

static inline void hmp_next_up_delay(struct sched_entity *se, int cpu)
{
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	hmp_last_up_migration(cpu) = cfs_rq_clock_task(cfs_rq);
	hmp_last_down_migration(cpu) = 0;
#else
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;

	se->avg.hmp_last_up_migration = cfs_rq_clock_task(cfs_rq);
	se->avg.hmp_last_down_migration = 0;
#endif
}

static inline void hmp_next_down_delay(struct sched_entity *se, int cpu)
{
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	hmp_last_down_migration(cpu) = cfs_rq_clock_task(cfs_rq);
	hmp_last_up_migration(cpu) = 0;
#else
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;

	se->avg.hmp_last_down_migration = cfs_rq_clock_task(cfs_rq);
	se->avg.hmp_last_up_migration = 0;
#endif
}

#ifdef CONFIG_HMP_VARIABLE_SCALE

static u64 hmp_variable_scale_convert(u64 delta)
{
	u64 high = delta >> 32ULL;
	u64 low = delta & 0xffffffffULL;
	low *= hmp_data.multiplier;
	high *= hmp_data.multiplier;
	return (low >> HMP_VARIABLE_SCALE_SHIFT)
			+ (high << (32ULL - HMP_VARIABLE_SCALE_SHIFT));
}

static ssize_t hmp_show(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct hmp_global_attr *hmp_attr =
		container_of(attr, struct hmp_global_attr, attr);
	int temp = *(hmp_attr->value);
	if (hmp_attr->to_sysfs != NULL)
		temp = hmp_attr->to_sysfs(temp);
	ret = sprintf(buf, "%d\n", temp);
	return ret;
}

static ssize_t hmp_store(struct kobject *a, struct attribute *attr,
				const char *buf, size_t count)
{
	int temp;
	ssize_t ret = count;
	struct hmp_global_attr *hmp_attr =
		container_of(attr, struct hmp_global_attr, attr);
	char *str = vmalloc(count + 1);
	if (str == NULL)
		return -ENOMEM;
	memcpy(str, buf, count);
	str[count] = 0;
	if (sscanf(str, "%d", &temp) < 1)
		ret = -EINVAL;
	else {
		if (hmp_attr->from_sysfs != NULL)
			temp = hmp_attr->from_sysfs(temp);
		if (temp < 0)
			ret = -EINVAL;
		else
			*(hmp_attr->value) = temp;
	}
	vfree(str);
	return ret;
}

static int hmp_period_tofrom_sysfs(int value)
{
	return (LOAD_AVG_PERIOD << HMP_VARIABLE_SCALE_SHIFT) / value;
}

static int hmp_theshold_from_sysfs(int value)
{
	if (value > 1024)
		return -1;
	return value;
}
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
static int hmp_freqinvar_from_sysfs(int value)
{
	if (value < 0 || value > 1)
		return -1;
	return value;
}
#endif
static void hmp_attr_add(
	const char *name,
	int *value,
	int (*to_sysfs)(int),
	int (*from_sysfs)(int))
{
	int i = 0;
	while (hmp_data.attributes[i] != NULL) {
		i++;
		if (i >= HMP_DATA_SYSFS_MAX)
			return;
	}
	hmp_data.attr[i].attr.mode = 0644;
	hmp_data.attr[i].show = hmp_show;
	hmp_data.attr[i].store = hmp_store;
	hmp_data.attr[i].attr.name = name;
	hmp_data.attr[i].value = value;
	hmp_data.attr[i].to_sysfs = to_sysfs;
	hmp_data.attr[i].from_sysfs = from_sysfs;
	hmp_data.attributes[i] = &hmp_data.attr[i].attr;
	hmp_data.attributes[i + 1] = NULL;
}

static int hmp_attr_init(void)
{
	int ret;
	memset(&hmp_data, sizeof(hmp_data), 0);
	hmp_data.multiplier = hmp_period_tofrom_sysfs(LOAD_AVG_VARIABLE_PERIOD);

	hmp_attr_add("load_avg_period_ms",
		&hmp_data.multiplier,
		hmp_period_tofrom_sysfs,
		hmp_period_tofrom_sysfs);
	hmp_attr_add("up_threshold",
		&hmp_up_threshold,
		NULL,
		hmp_theshold_from_sysfs);
	hmp_attr_add("down_threshold",
		&hmp_down_threshold,
		NULL,
		hmp_theshold_from_sysfs);
	hmp_attr_add("init_task_load_period",
		&init_task_load_period,
		NULL,
		NULL);
#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	
	hmp_data.freqinvar_load_scale_enabled = 1;
	hmp_attr_add("frequency_invariant_load_scale",
		&hmp_data.freqinvar_load_scale_enabled,
		NULL,
		hmp_freqinvar_from_sysfs);
#endif
	hmp_data.attr_group.name = "hmp";
	hmp_data.attr_group.attrs = hmp_data.attributes;
	ret = sysfs_create_group(kernel_kobj,
		&hmp_data.attr_group);
	return 0;
}
late_initcall(hmp_attr_init);
#endif 

static inline unsigned int hmp_domain_min_load(struct hmp_domain *hmpd,
						int *min_cpu)
{
	int cpu;
	int min_cpu_runnable_temp = NR_CPUS;
	unsigned long min_runnable_load = INT_MAX;
	unsigned long contrib;

	for_each_cpu_mask(cpu, hmpd->cpus) {
		
		contrib = cpu_rq(cpu)->avg.runnable_avg_sum * scale_load_down(1024);
		if (contrib < min_runnable_load) {
			min_runnable_load = contrib;
			min_cpu_runnable_temp = cpu;
		}
	}

	if (min_cpu)
		*min_cpu = min_cpu_runnable_temp;

	
	return min_runnable_load ? min_runnable_load / (LOAD_AVG_MAX + 1) : 0;
}

static inline unsigned int hmp_task_starvation(struct sched_entity *se)
{
	u32 starvation;

	starvation = se->avg.usage_avg_sum * scale_load_down(NICE_0_LOAD);
	starvation /= (se->avg.runnable_avg_sum + 1);

	return scale_load(starvation);
}

static inline unsigned int hmp_offload_down(int cpu, struct sched_entity *se)
{
	int min_usage;
	int dest_cpu = NR_CPUS;

	if (hmp_cpu_is_slowest(cpu))
		return NR_CPUS;

	
	
	min_usage = hmp_domain_min_load(hmp_cpu_domain(cpu), NULL);
	if (min_usage < (NICE_0_LOAD>>1))
		return NR_CPUS;

	
	if (cpu_rq(cpu)->cfs.nr_running < 2)
		return NR_CPUS;

	
	
	if (hmp_task_starvation(se) > 768)
		return NR_CPUS;

	
	min_usage = hmp_domain_min_load(hmp_slower_domain(cpu), &dest_cpu);
	
	if (min_usage > NICE_0_LOAD/2)
		return NR_CPUS;

	if (cpumask_test_cpu(dest_cpu, &hmp_slower_domain(cpu)->cpus))
		return dest_cpu;

	return NR_CPUS;
}
#endif 


#ifdef CONFIG_MTK_SCHED_CMP
unsigned int cmp_up_threshold = 512;
unsigned int cmp_down_threshold = 256;
#endif 

#ifdef CONFIG_MTK_SCHED_CMP_TGS
static void sched_tg_enqueue_fair(struct rq *rq, struct task_struct *p)
{
	int id;
	unsigned long flags;
	struct task_struct *tg = p->group_leader;

	if (group_leader_is_empty(p))
		return;
	id = get_cluster_id(rq->cpu);
	if (unlikely(WARN_ON(id < 0)))
		return;

	raw_spin_lock_irqsave(&tg->thread_group_info_lock, flags);
	tg->thread_group_info[id].cfs_nr_running++;
	raw_spin_unlock_irqrestore(&tg->thread_group_info_lock, flags);
}

static void sched_tg_dequeue_fair(struct rq *rq, struct task_struct *p)
{
	int id;
	unsigned long flags;
	struct task_struct *tg = p->group_leader;

	if (group_leader_is_empty(p))
		return;
	id = get_cluster_id(rq->cpu);
	if (unlikely(WARN_ON(id < 0)))
		return;

	raw_spin_lock_irqsave(&tg->thread_group_info_lock, flags);
	tg->thread_group_info[id].cfs_nr_running--;
	raw_spin_unlock_irqrestore(&tg->thread_group_info_lock, flags);
}

#endif
static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;

	for_each_sched_entity(se) {
		if (se->on_rq)
			break;
		cfs_rq = cfs_rq_of(se);
		enqueue_entity(cfs_rq, se, flags);

		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running++;

		flags = ENQUEUE_WAKEUP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running++;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_shares(cfs_rq);
		update_entity_load_avg(se, 1);
	}

	if (!se) {
		update_rq_runnable_avg(rq, rq->nr_running);
		inc_nr_running(rq);
#ifndef CONFIG_CFS_BANDWIDTH
		BUG_ON(rq->cfs.nr_running > rq->cfs.h_nr_running);
#endif
	}
	hrtick_update(rq);
#ifdef CONFIG_HMP_TRACER
	trace_sched_runqueue_length(rq->cpu,rq->nr_running);
	trace_sched_cfs_length(rq->cpu,rq->cfs.h_nr_running);
#endif
#ifdef CONFIG_MET_SCHED_HMP
	RqLen(rq->cpu,rq->nr_running);
	CfsLen(rq->cpu,rq->cfs.h_nr_running);
#endif

#ifdef CONFIG_MTK_SCHED_CMP_TGS
	sched_tg_enqueue_fair(rq, p);
#endif
}

static void set_next_buddy(struct sched_entity *se);

static void dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se;
	int task_sleep = flags & DEQUEUE_SLEEP;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		dequeue_entity(cfs_rq, se, flags);

		if (cfs_rq_throttled(cfs_rq))
			break;
		cfs_rq->h_nr_running--;

		
		if (cfs_rq->load.weight) {
			if (task_sleep && parent_entity(se))
				set_next_buddy(parent_entity(se));

			
			se = parent_entity(se);
			break;
		}
		flags |= DEQUEUE_SLEEP;
	}

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		cfs_rq->h_nr_running--;

		if (cfs_rq_throttled(cfs_rq))
			break;

		update_cfs_shares(cfs_rq);
		update_entity_load_avg(se, 1);
	}

	if (!se) {
		dec_nr_running(rq);
#ifndef CONFIG_CFS_BANDWIDTH
		BUG_ON(rq->cfs.nr_running > rq->cfs.h_nr_running);
#endif
		update_rq_runnable_avg(rq, 1);
	}
	hrtick_update(rq);
#ifdef CONFIG_HMP_TRACER
	trace_sched_runqueue_length(rq->cpu,rq->nr_running);
	trace_sched_cfs_length(rq->cpu,rq->cfs.h_nr_running);
#endif
#ifdef CONFIG_MET_SCHED_HMP
	RqLen(rq->cpu,rq->nr_running);
	CfsLen(rq->cpu,rq->cfs.h_nr_running);
#endif

#ifdef CONFIG_MTK_SCHED_CMP_TGS
	sched_tg_dequeue_fair(rq, p);
#endif
}

#ifdef CONFIG_SMP
static unsigned long weighted_cpuload(const int cpu)
{
	return cpu_rq(cpu)->cfs.runnable_load_avg;
}

static unsigned long source_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return min(rq->cpu_load[type-1], total);
}

static unsigned long target_load(int cpu, int type)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long total = weighted_cpuload(cpu);

	if (type == 0 || !sched_feat(LB_BIAS))
		return total;

	return max(rq->cpu_load[type-1], total);
}

static unsigned long power_of(int cpu)
{
	return cpu_rq(cpu)->cpu_power;
}

static unsigned long cpu_avg_load_per_task(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long nr_running = ACCESS_ONCE(rq->nr_running);
	unsigned long load_avg = rq->cfs.runnable_load_avg;

	if (nr_running)
		return load_avg / nr_running;

	return 0;
}


static void task_waking_fair(struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	u64 min_vruntime;

#ifndef CONFIG_64BIT
	u64 min_vruntime_copy;

	do {
		min_vruntime_copy = cfs_rq->min_vruntime_copy;
		smp_rmb();
		min_vruntime = cfs_rq->min_vruntime;
	} while (min_vruntime != min_vruntime_copy);
#else
	min_vruntime = cfs_rq->min_vruntime;
#endif

	se->vruntime -= min_vruntime;
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static long effective_load(struct task_group *tg, int cpu, long wl, long wg)
{
	struct sched_entity *se = tg->se[cpu];

	if (!tg->parent)	
		return wl;

	for_each_sched_entity(se) {
		long w, W;

		tg = se->my_q->tg;

		W = wg + calc_tg_weight(tg, se->my_q);

		w = se->my_q->load.weight + wl;

		if (W > 0 && w < W)
			wl = (w * tg->shares) / W;
		else
			wl = tg->shares;

		if (wl < MIN_SHARES)
			wl = MIN_SHARES;

		wl -= se->load.weight;

		wg = 0;
	}

	return wl;
}
#else

static inline unsigned long effective_load(struct task_group *tg, int cpu,
		unsigned long wl, unsigned long wg)
{
	return wl;
}

#endif

static int wake_affine(struct sched_domain *sd, struct task_struct *p, int sync)
{
	s64 this_load, load;
	int idx, this_cpu, prev_cpu;
	unsigned long tl_per_task;
	struct task_group *tg;
	unsigned long weight;
	int balanced;

	idx	  = sd->wake_idx;
	this_cpu  = smp_processor_id();
	prev_cpu  = task_cpu(p);
	load	  = source_load(prev_cpu, idx);
	this_load = target_load(this_cpu, idx);

	if (sync) {
		tg = task_group(current);
		weight = current->se.load.weight;

		this_load += effective_load(tg, this_cpu, -weight, -weight);
		load += effective_load(tg, prev_cpu, 0, -weight);
	}

	tg = task_group(p);
	weight = p->se.load.weight;

	if (this_load > 0) {
		s64 this_eff_load, prev_eff_load;

		this_eff_load = 100;
		this_eff_load *= power_of(prev_cpu);
		this_eff_load *= this_load +
			effective_load(tg, this_cpu, weight, weight);

		prev_eff_load = 100 + (sd->imbalance_pct - 100) / 2;
		prev_eff_load *= power_of(this_cpu);
		prev_eff_load *= load + effective_load(tg, prev_cpu, 0, weight);

		balanced = this_eff_load <= prev_eff_load;
	} else
		balanced = true;

	if (sync && balanced)
		return 1;

	schedstat_inc(p, se.statistics.nr_wakeups_affine_attempts);
	tl_per_task = cpu_avg_load_per_task(this_cpu);

	if (balanced ||
	    (this_load <= load &&
	     this_load + target_load(prev_cpu, idx) <= tl_per_task)) {
		schedstat_inc(sd, ttwu_move_affine);
		schedstat_inc(p, se.statistics.nr_wakeups_affine);

		return 1;
	}
	return 0;
}

static struct sched_group *
find_idlest_group(struct sched_domain *sd, struct task_struct *p,
		  int this_cpu, int load_idx)
{
	struct sched_group *idlest = NULL, *group = sd->groups;
	unsigned long min_load = ULONG_MAX, this_load = 0;
	int imbalance = 100 + (sd->imbalance_pct-100)/2;

	do {
		unsigned long load, avg_load;
		int local_group;
		int i;

		
		if (!cpumask_intersects(sched_group_cpus(group),
					tsk_cpus_allowed(p)))
			continue;

		local_group = cpumask_test_cpu(this_cpu,
					       sched_group_cpus(group));

		
		avg_load = 0;

		for_each_cpu(i, sched_group_cpus(group)) {
			
			if (local_group)
				load = source_load(i, load_idx);
			else
				load = target_load(i, load_idx);

			avg_load += load;

			mt_sched_printf("find_idlest_group cpu=%d avg=%lu",
				i, avg_load);
		}

		
		avg_load = (avg_load * SCHED_POWER_SCALE) / group->sgp->power;

		if (local_group) {
			this_load = avg_load;
			mt_sched_printf("find_idlest_group this_load=%lu",
				this_load);
		} else if (avg_load < min_load) {
			min_load = avg_load;
			idlest = group;
			mt_sched_printf("find_idlest_group min_load=%lu",
				min_load);
		}
	} while (group = group->next, group != sd->groups);

	if (!idlest || 100*this_load < imbalance*min_load){
		mt_sched_printf("find_idlest_group fail this_load=%lu min_load=%lu, imbalance=%d",
			this_load, min_load, imbalance);
		return NULL;
	}
	return idlest;
}

static int
find_idlest_cpu(struct sched_group *group, struct task_struct *p, int this_cpu)
{
	unsigned long load, min_load = ULONG_MAX;
	int idlest = -1;
	int i;

	
	for_each_cpu_and(i, sched_group_cpus(group), tsk_cpus_allowed(p)) {
		load = weighted_cpuload(i);

		if (load < min_load || (load == min_load && i == this_cpu)) {
			min_load = load;
			idlest = i;
		}
	}

	return idlest;
}

static int select_idle_sibling(struct task_struct *p, int target)
{
	struct sched_domain *sd;
	struct sched_group *sg;
	int i = task_cpu(p);

	if (idle_cpu(target))
		return target;

	if (i != target && cpus_share_cache(i, target) && idle_cpu(i))
		return i;

	sd = rcu_dereference(per_cpu(sd_llc, target));
	for_each_lower_domain(sd) {
		sg = sd->groups;
		do {
			if (!cpumask_intersects(sched_group_cpus(sg),
						tsk_cpus_allowed(p)))
				goto next;

			for_each_cpu(i, sched_group_cpus(sg)) {
				if (i == target || !idle_cpu(i))
					goto next;
			}

			target = cpumask_first_and(sched_group_cpus(sg),
					tsk_cpus_allowed(p));
			goto done;
next:
			sg = sg->next;
		} while (sg != sd->groups);
	}
done:
	return target;
}

#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
static int cmp_find_idle_cpu(struct task_struct *p, int clid, int *target)
{
	struct cpumask cls_cpus;
	int j;

	get_cluster_cpus(&cls_cpus, clid, true);
	*target = cpumask_any_and(&cls_cpus, tsk_cpus_allowed(p));
	for_each_cpu(j, &cls_cpus) {
		if (idle_cpu(j) && cpumask_test_cpu(j, tsk_cpus_allowed(p))) {
			*target = j;
			break;
		}
	}
	if (*target >= nr_cpu_ids)
		return 0; 
	mt_sched_printf("wakeup %d %s cpu=%d, max_clid/max_idle_clid=%d",
		p->pid, p->comm, *target, clid);

	return 1;
}

#if !defined(CONFIG_SCHED_HMP)
#define TGS_WAKEUP_EXPERIMENT
#endif
static int cmp_select_task_rq_fair(struct task_struct *p, int sd_flag, int *cpu)
{
	int i, j;
	int max_cnt=0, tskcnt;
	int tgs_clid=-1;
	int idle_cnt, max_idle_cnt=0;
	int in_prev=0, prev_cluster=0;
	struct cpumask cls_cpus;
	int num_cluster;

	num_cluster=arch_get_nr_clusters();
	for(i=0; i< num_cluster; i++) {
		tskcnt= p->group_leader->thread_group_info[i].nr_running;
		idle_cnt = 0;
		get_cluster_cpus(&cls_cpus, i, true);

 		for_each_cpu(j, &cls_cpus) {
#ifdef TGS_WAKEUP_EXPERIMENT
			if (arch_is_big_little()) {
				int bcpu = arch_cpu_is_big(j);
				if (bcpu && p->se.avg.load_avg_ratio >= cmp_up_threshold) {
					in_prev = 0;
					tgs_clid = i;
					mt_sched_printf("[heavy task] wakeup load=%ld up_th=%u pid=%d name=%s cpu=%d, tgs_clid=%d in_prev=%d",
									p->se.avg.load_avg_ratio, cmp_up_threshold, p->pid, p->comm, *cpu, tgs_clid, in_prev);
					goto find_idle_cpu;
				}
				if (!bcpu && p->se.avg.load_avg_ratio < cmp_down_threshold) {
					in_prev = 0;
					tgs_clid = i;
					mt_sched_printf("[light task] wakeup load=%ld down_th=%u pid=%d name=%s cpu=%d, tgs_clid=%d in_prev=%d",
									p->se.avg.load_avg_ratio, cmp_down_threshold, p->pid, p->comm, *cpu, tgs_clid, in_prev);
					goto find_idle_cpu;
				}
			}
#endif
			if (idle_cpu(j))
				idle_cnt++;
		}
		mt_sched_printf("wakeup load=%ld pid=%d name=%s clid=%d idle_cnt=%d tskcnt=%d max_cnt=%d, cls_cpus=%02lx, onlineCPU=%02lx",
						p->se.avg.load_avg_ratio, p->pid, p->comm, i, idle_cnt, tskcnt, max_cnt,
						*cpumask_bits(&cls_cpus), *cpumask_bits(cpu_online_mask));

		if (idle_cnt == 0)
			continue;

		if (i == get_cluster_id(*cpu))
			prev_cluster = 1;

		if (tskcnt > 0) {
			if ( (tskcnt > max_cnt) || ((tskcnt == max_cnt) && prev_cluster)) {
				in_prev = prev_cluster;
				tgs_clid = i;
				max_cnt = tskcnt;
			}
		} else if (0 == max_cnt) {
			if ((idle_cnt > max_idle_cnt) || ((idle_cnt == max_idle_cnt) && prev_cluster)) {
				in_prev = prev_cluster;
				tgs_clid = i ;
				max_idle_cnt = idle_cnt;
			}

		}
		mt_sched_printf("wakeup %d %s i=%d idle_cnt=%d tgs_clid=%d max_cnt=%d max_idle_cnt=%d in_prev=%d",
			p->pid, p->comm, i, idle_cnt, tgs_clid, max_cnt, max_idle_cnt, in_prev);
	}

#ifdef TGS_WAKEUP_EXPERIMENT
find_idle_cpu:
#endif
	mt_sched_printf("wakeup %d %s cpu=%d, tgs_clid=%d in_prev=%d",
		p->pid, p->comm, *cpu, tgs_clid, in_prev);

	if(-1 != tgs_clid && !in_prev && cmp_find_idle_cpu(p, tgs_clid, cpu))
		return 1;

	return 0;
}
#endif

#ifdef CONFIG_MTK_SCHED_TRACERS
#define LB_RESET		0
#define LB_AFFINITY		0x10
#define LB_BUDDY		0x20
#define LB_FORK			0x30
#define LB_CMP_SHIFT	8
#define LB_CMP			0x4000
#define LB_SMP_SHIFT	16
#define LB_SMP			0x500000
#define LB_HMP_SHIFT	24
#define LB_HMP			0x60000000
#endif

static int
select_task_rq_fair(struct task_struct *p, int sd_flag, int wake_flags)
{
	struct sched_domain *tmp, *affine_sd = NULL, *sd = NULL;
	int cpu = smp_processor_id();
	int prev_cpu = task_cpu(p);
	int new_cpu = cpu;
	int want_affine = 0;
	int sync = wake_flags & WF_SYNC;
#if defined(CONFIG_SCHED_HMP) && !defined(CONFIG_SCHED_HMP_ENHANCEMENT)
	int target_cpu = nr_cpu_ids;
#endif
#ifdef CONFIG_MTK_SCHED_TRACERS
	int policy = 0;
#endif
#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
	int cmp_cpu;
	int cmp_cpu_found=0;
#endif
#ifdef CONFIG_MTK_SCHED_CMP_PACK_SMALL_TASK	
	int buddy_cpu = per_cpu(sd_pack_buddy, cpu);
#endif

	if (p->nr_cpus_allowed == 1)
	{
#ifdef CONFIG_MTK_SCHED_TRACERS
		trace_sched_select_task_rq(p, (LB_AFFINITY | prev_cpu), prev_cpu, prev_cpu);
#endif
		return prev_cpu;
	}

#ifdef CONFIG_HMP_PACK_SMALL_TASK
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
	if (check_pack_buddy(cpu, p) && PA_ENABLE) {
		PACK_FROM_CPUX_TO_CPUY_COUNT[cpu][per_cpu(sd_pack_buddy, cpu)]++;

#ifdef CONFIG_HMP_TRACER
		trace_sched_power_aware_active(POWER_AWARE_ACTIVE_MODULE_PACK_FORM_CPUX_TO_CPUY, p->pid, cpu, per_cpu(sd_pack_buddy, cpu));
#endif 
		
		if(PA_MON_ENABLE) {
			if(strcmp(p->comm, PA_MON) == 0 && cpu != per_cpu(sd_pack_buddy, cpu)) {
				printk(KERN_EMERG "[PA] %s PACK From CPU%d to CPU%d\n", p->comm, cpu, per_cpu(sd_pack_buddy, cpu));
				printk(KERN_EMERG "[PA]   Buddy RQ Usage = %u, Period = %u, NR = %u\n", 
														per_cpu(BUDDY_CPU_RQ_USAGE, per_cpu(sd_pack_buddy, cpu)),
														per_cpu(BUDDY_CPU_RQ_PERIOD, per_cpu(sd_pack_buddy, cpu)),
														per_cpu(BUDDY_CPU_RQ_NR, per_cpu(sd_pack_buddy, cpu)));
				printk(KERN_EMERG "[PA]   Task Usage = %u, Period = %u\n", 
														per_cpu(TASK_USGAE, cpu),
														per_cpu(TASK_PERIOD, cpu));
			}
		}
#else 
	if (check_pack_buddy(cpu, p)) {
#endif 
#ifdef CONFIG_MTK_SCHED_TRACERS
		new_cpu = per_cpu(sd_pack_buddy, cpu);
		trace_sched_select_task_rq(p, (LB_BUDDY | new_cpu), prev_cpu, new_cpu);
#endif
		return per_cpu(sd_pack_buddy, cpu);
	}
#elif defined (CONFIG_MTK_SCHED_CMP_PACK_SMALL_TASK)
#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER
	if (PA_ENABLE && (sd_flag & SD_BALANCE_WAKE) && (check_pack_buddy(buddy_cpu, p))) {
#else
	if ((sd_flag & SD_BALANCE_WAKE) && (check_pack_buddy(buddy_cpu, p))) {
#endif
		struct thread_group_info_t *src_tginfo, *dst_tginfo;
		src_tginfo = &p->group_leader->thread_group_info[get_cluster_id(prev_cpu)]; 
		dst_tginfo = &p->group_leader->thread_group_info[get_cluster_id(buddy_cpu)];
		if((get_cluster_id(prev_cpu) == get_cluster_id(buddy_cpu)) ||  
			(src_tginfo->nr_running < dst_tginfo->nr_running))
		{
#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER
			PACK_FROM_CPUX_TO_CPUY_COUNT[cpu][buddy_cpu]++;
			mt_sched_printf("[PA]pid=%d, Pack to CPU%d(CPU%d's buddy)\n", p->pid,buddy_cpu,cpu);
			if(PA_MON_ENABLE) {
				u8 i=0;
				for(i=0;i<4; i++) {
					if(strcmp(p->comm, &PA_MON[i][0]) == 0) {
						TASK_PACK_CPU_COUNT[i][buddy_cpu]++;
						printk(KERN_EMERG "[PA] %s PACK to CPU%d(CPU%d's buddy), pre(cpu%d)\n", p->comm, buddy_cpu,cpu, prev_cpu);
						printk(KERN_EMERG "[PA]   Buddy RQ Usage = %u, Period = %u, NR = %u\n", 
																per_cpu(BUDDY_CPU_RQ_USAGE, buddy_cpu),
																per_cpu(BUDDY_CPU_RQ_PERIOD, buddy_cpu),
																per_cpu(BUDDY_CPU_RQ_NR, buddy_cpu));
						printk(KERN_EMERG "[PA]   Task Usage = %u, Period = %u\n", 
																per_cpu(TASK_USGAE, cpu),
																per_cpu(TASK_PERIOD, cpu));
						break;										
					}
				}
			}	
#endif 
#ifdef CONFIG_MTK_SCHED_TRACERS
			trace_sched_select_task_rq(p, (LB_BUDDY | buddy_cpu), prev_cpu, buddy_cpu);
#endif
			return buddy_cpu;
		}		
	}
#endif 

#ifdef CONFIG_SCHED_HMP
	
	if (p->mm && (sd_flag & SD_BALANCE_FORK)) {
		if(hmp_cpu_is_fastest(prev_cpu)) {
			struct hmp_domain *hmpdom = list_entry(&hmp_cpu_domain(prev_cpu)->hmp_domains, struct hmp_domain, hmp_domains);
			__always_unused int lowest_ratio = hmp_domain_min_load(hmpdom, &new_cpu);
			if(new_cpu < nr_cpu_ids && cpumask_test_cpu(new_cpu,tsk_cpus_allowed(p)))
			{
#ifdef CONFIG_MTK_SCHED_TRACERS
				trace_sched_select_task_rq(p, (LB_FORK | new_cpu), prev_cpu, new_cpu);
#endif
				return new_cpu;
			}
			else
			{
				new_cpu = cpumask_any_and(&hmp_faster_domain(cpu)->cpus,
						tsk_cpus_allowed(p));
				if(new_cpu < nr_cpu_ids)
				{
#ifdef CONFIG_MTK_SCHED_TRACERS
					trace_sched_select_task_rq(p, (LB_FORK | new_cpu), prev_cpu, new_cpu);
#endif
					return new_cpu;
				}
			}
		} else {
			new_cpu = hmp_select_faster_cpu(p, prev_cpu);
			if (new_cpu < nr_cpu_ids)
			{
#ifdef CONFIG_MTK_SCHED_TRACERS
				trace_sched_select_task_rq(p, (LB_FORK | new_cpu), prev_cpu, new_cpu);
#endif
				return new_cpu;
			}
		}
		
		if (new_cpu >= nr_cpu_ids)
			new_cpu = cpu;
	}
#endif

	if (sd_flag & SD_BALANCE_WAKE) {
		if (cpumask_test_cpu(cpu, tsk_cpus_allowed(p)))
			want_affine = 1;
		new_cpu = prev_cpu;
	}

#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
	cmp_cpu = prev_cpu;
	cmp_cpu_found = cmp_select_task_rq_fair(p, sd_flag, &cmp_cpu);
	if (cmp_cpu_found && (cmp_cpu < nr_cpu_ids)) {
		cpu = cmp_cpu;
		new_cpu = cmp_cpu;
#ifdef CONFIG_MTK_SCHED_TRACERS
		policy |= (new_cpu << LB_CMP_SHIFT);
		policy |= LB_CMP;
#endif
		mt_sched_printf("wakeup %d %s sd_flag=%x cmp_cpu_found=%d, cpu=%d, want_affine=%d ",
			p->pid, p->comm, sd_flag, cmp_cpu_found, cpu, want_affine);
		goto cmp_found;
	}
#endif
	rcu_read_lock();
	for_each_domain(cpu, tmp) {
		mt_sched_printf("wakeup %d %s tmp->flags=%x, cpu=%d, prev_cpu=%d, new_cpu=%d",
			p->pid, p->comm, tmp->flags, cpu, prev_cpu, new_cpu); 

		if (!(tmp->flags & SD_LOAD_BALANCE))
			continue;

		if (want_affine && (tmp->flags & SD_WAKE_AFFINE) &&
		    cpumask_test_cpu(prev_cpu, sched_domain_span(tmp))) {
			affine_sd = tmp;
			break;
		}

		if (tmp->flags & sd_flag)
			sd = tmp;
	}

	if (affine_sd) {
		if (cpu != prev_cpu && wake_affine(affine_sd, p, sync))
			prev_cpu = cpu;

		new_cpu = select_idle_sibling(p, prev_cpu);
		goto unlock;
	}

	mt_sched_printf("wakeup %d %s sd=%p", p->pid, p->comm, sd);

	while (sd) {
		int load_idx = sd->forkexec_idx;
		struct sched_group *group;
		int weight;

		mt_sched_printf("wakeup %d %s find_idlest_group cpu=%d sd->flags=%x sd_flag=%x",
			p->pid, p->comm, cpu, sd->flags, sd_flag);

		if (!(sd->flags & sd_flag)) {
			sd = sd->child;
			continue;
		}

		if (sd_flag & SD_BALANCE_WAKE)
			load_idx = sd->wake_idx;

		mt_sched_printf("wakeup %d %s find_idlest_group cpu=%d",
			p->pid, p->comm, cpu);
		group = find_idlest_group(sd, p, cpu, load_idx);
		if (!group) {
			sd = sd->child;
			mt_sched_printf("wakeup %d %s find_idlest_group child", 
				p->pid, p->comm);
			continue;
		}

		new_cpu = find_idlest_cpu(group, p, cpu);
		if (new_cpu == -1 || new_cpu == cpu) {
			
			sd = sd->child;
			mt_sched_printf("wakeup %d %s find_idlest_cpu sd->child=%p",
				p->pid, p->comm, sd);
			continue;
		}

		
		mt_sched_printf("wakeup %d %s find_idlest_cpu cpu=%d sd=%p",
				p->pid, p->comm, new_cpu, sd);
		cpu = new_cpu;
		weight = sd->span_weight;
		sd = NULL;
		for_each_domain(cpu, tmp) {
			if (weight <= tmp->span_weight)
				break;
			if (tmp->flags & sd_flag)
				sd = tmp;
			mt_sched_printf("wakeup %d %s sd=%p weight=%d, tmp->span_weight=%d", 
				p->pid, p->comm, sd, weight, tmp->span_weight);
		}
		
	}

#ifdef CONFIG_MTK_SCHED_TRACERS
	policy |= (new_cpu << LB_SMP_SHIFT);
	policy |= LB_SMP;
#endif

unlock:
	rcu_read_unlock();
	mt_sched_printf("wakeup %d %s new_cpu=%x", p->pid, p->comm, new_cpu);

#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
cmp_found:
#endif

#ifdef CONFIG_SCHED_HMP
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	new_cpu = hmp_select_task_rq_fair(sd_flag, p, prev_cpu, new_cpu);
#ifdef CONFIG_MTK_SCHED_TRACERS
	policy |= (new_cpu << LB_HMP_SHIFT);
	policy |= LB_HMP;
#endif

#else
	if (hmp_up_migration(prev_cpu, &target_cpu, &p->se)) {
		new_cpu = hmp_select_faster_cpu(p, prev_cpu);
		hmp_next_up_delay(&p->se, new_cpu);
		trace_sched_hmp_migrate(p, new_cpu, 0);
		return new_cpu;
	}
	if (hmp_down_migration(prev_cpu, &p->se)) {
		new_cpu = hmp_select_slower_cpu(p, prev_cpu);
		hmp_next_down_delay(&p->se, new_cpu);
		trace_sched_hmp_migrate(p, new_cpu, 0);
		return new_cpu;
	}
	
	if (!cpumask_test_cpu(new_cpu, &hmp_cpu_domain(prev_cpu)->cpus))
		return prev_cpu;
#endif 
#endif 

#ifdef CONFIG_MTK_SCHED_TRACERS
	trace_sched_select_task_rq(p, policy, prev_cpu, new_cpu);
#endif

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
	if(PA_MON_ENABLE) {
		if(strcmp(p->comm, PA_MON) == 0 && cpu != new_cpu) {
			printk(KERN_EMERG "[PA] %s Select From CPU%d to CPU%d\n", p->comm, cpu, new_cpu);
		}
	}
#endif 

	return new_cpu;
}

static void
migrate_task_rq_fair(struct task_struct *p, int next_cpu)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	if (se->avg.decay_count) {
		se->avg.decay_count = -__synchronize_entity_decay(se);
		atomic_long_add(se->avg.load_avg_contrib,
						&cfs_rq->removed_load);
	}
}
#endif 

static unsigned long
wakeup_gran(struct sched_entity *curr, struct sched_entity *se)
{
	unsigned long gran = sysctl_sched_wakeup_granularity;

	return calc_delta_fair(gran, se);
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se)
{
	s64 gran, vdiff = curr->vruntime - se->vruntime;

	if (vdiff <= 0)
		return -1;

	gran = wakeup_gran(curr, se);
	if (vdiff > gran)
		return 1;

	return 0;
}

static void set_last_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->last = se;
}

static void set_next_buddy(struct sched_entity *se)
{
	if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
		return;

	for_each_sched_entity(se)
		cfs_rq_of(se)->next = se;
}

static void set_skip_buddy(struct sched_entity *se)
{
	for_each_sched_entity(se)
		cfs_rq_of(se)->skip = se;
}

static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = rq->curr;
	struct sched_entity *se = &curr->se, *pse = &p->se;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	int scale = cfs_rq->nr_running >= sched_nr_latency;
	int next_buddy_marked = 0;

	if (unlikely(se == pse))
		return;

	if (unlikely(throttled_hierarchy(cfs_rq_of(pse))))
		return;

	if (sched_feat(NEXT_BUDDY) && scale && !(wake_flags & WF_FORK)) {
		set_next_buddy(pse);
		next_buddy_marked = 1;
	}

	if (test_tsk_need_resched(curr))
		return;

	
	if (unlikely(curr->policy == SCHED_IDLE) &&
	    likely(p->policy != SCHED_IDLE))
		goto preempt;

	if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
		return;

	find_matching_se(&se, &pse);
	update_curr(cfs_rq_of(se));
	BUG_ON(!pse);
	if (wakeup_preempt_entity(se, pse) == 1) {
		if (!next_buddy_marked)
			set_next_buddy(pse);
		goto preempt;
	}

	return;

preempt:
	resched_task(curr);
	if (unlikely(!se->on_rq || curr == rq->idle))
		return;

	if (sched_feat(LAST_BUDDY) && scale && entity_is_task(se))
		set_last_buddy(se);
}

static struct task_struct *pick_next_task_fair(struct rq *rq)
{
	struct task_struct *p;
	struct cfs_rq *cfs_rq = &rq->cfs;
	struct sched_entity *se;

	
	if (!cfs_rq->nr_running || !cfs_rq->h_nr_running)
		return NULL;

	do {
		se = pick_next_entity(cfs_rq);
		set_next_entity(cfs_rq, se);
		cfs_rq = group_cfs_rq(se);
	} while (cfs_rq);

	p = task_of(se);
	if (hrtick_enabled(rq))
		hrtick_start_fair(rq, p);

	return p;
}

static void put_prev_task_fair(struct rq *rq, struct task_struct *prev)
{
	struct sched_entity *se = &prev->se;
	struct cfs_rq *cfs_rq;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		put_prev_entity(cfs_rq, se);
	}
}

static void yield_task_fair(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	struct sched_entity *se = &curr->se;

	if (unlikely(rq->nr_running == 1))
		return;

	clear_buddies(cfs_rq, se);

	if (curr->policy != SCHED_BATCH) {
		update_rq_clock(rq);
		update_curr(cfs_rq);
		 rq->skip_clock_update = 1;
	}

	set_skip_buddy(se);
}

static bool yield_to_task_fair(struct rq *rq, struct task_struct *p, bool preempt)
{
	struct sched_entity *se = &p->se;

	
	if (!se->on_rq || throttled_hierarchy(cfs_rq_of(se)))
		return false;

	
	set_next_buddy(se);

	yield_task_fair(rq);

	return true;
}

#ifdef CONFIG_SMP
 

static unsigned long __read_mostly max_load_balance_interval = HZ/10;

#define LBF_ALL_PINNED	0x01
#define LBF_NEED_BREAK	0x02
#define LBF_SOME_PINNED 0x04

struct lb_env {
	struct sched_domain	*sd;

	struct rq		*src_rq;
	int			src_cpu;

	int			dst_cpu;
	struct rq		*dst_rq;

	struct cpumask		*dst_grpmask;
	int			new_dst_cpu;
	enum cpu_idle_type	idle;
	long			imbalance;
	
	struct cpumask		*cpus;

	unsigned int		flags;

	unsigned int		loop;
	unsigned int		loop_break;
	unsigned int		loop_max;
#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
	int			mt_check_cache_in_idle;
#endif 	
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
	unsigned int 		fail_reason;
#endif	
};

static void move_task(struct task_struct *p, struct lb_env *env)
{
	deactivate_task(env->src_rq, p, 0);
	set_task_cpu(p, env->dst_cpu);
	activate_task(env->dst_rq, p, 0);
	check_preempt_curr(env->dst_rq, p, 0);

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
	if(PA_MON_ENABLE) {
		if(strcmp(p->comm, PA_MON) == 0) {
			printk(KERN_EMERG "[PA] %s Balance From CPU%d to CPU%d\n", p->comm, env->src_rq->cpu, env->dst_rq->cpu);
		}
	}
#endif 
		
}

#if defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT)
static int
task_hot(struct task_struct *p, u64 now, struct sched_domain *sd, int mt_check_cache_in_idle)
#else
static int
task_hot(struct task_struct *p, u64 now, struct sched_domain *sd)
#endif
{
	s64 delta;

	if (p->sched_class != &fair_sched_class)
		return 0;

	if (unlikely(p->policy == SCHED_IDLE))
		return 0;

#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
	if (!mt_check_cache_in_idle){
		if ( !this_rq()->nr_running && (task_rq(p)->nr_running >= 2) )
			return 0;
	}
#endif 	 
	if (sched_feat(CACHE_HOT_BUDDY) && this_rq()->nr_running &&
			(&p->se == cfs_rq_of(&p->se)->next ||
			 &p->se == cfs_rq_of(&p->se)->last))
		return 1;

	if (sysctl_sched_migration_cost == -1)
		return 1;
	if (sysctl_sched_migration_cost == 0)
		return 0;

	delta = now - p->se.exec_start;

	return delta < (s64)sysctl_sched_migration_cost;
}

static
int can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot = 0;
	if (throttled_lb_pair(task_group(p), env->src_cpu, env->dst_cpu))
		return 0;

	if (!cpumask_test_cpu(env->dst_cpu, tsk_cpus_allowed(p))) {
		int cpu;

		schedstat_inc(p, se.statistics.nr_failed_migrations_affine);
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_AFFINITY);
		if(mt_lbprof_lt (env->sd->mt_lbprof_nr_balance_failed, MT_LBPROF_NR_BALANCED_FAILED_UPPER_BOUND)){
			char strings[128]="";
			snprintf(strings, 128, "%d:balance fail:affinity:%d:%d:%s:0x%lu"
				, env->dst_cpu, env->src_cpu, p->pid, p->comm, p->cpus_allowed.bits[0]);
			trace_sched_lbprof_log(strings);
		}
#endif		

		if (!env->dst_grpmask || (env->flags & LBF_SOME_PINNED))
			return 0;

		
		for_each_cpu_and(cpu, env->dst_grpmask, env->cpus) {
			if (cpumask_test_cpu(cpu, tsk_cpus_allowed(p))) {
				env->flags |= LBF_SOME_PINNED;
				env->new_dst_cpu = cpu;
				break;
			}
		}

		return 0;
	}

	
	env->flags &= ~LBF_ALL_PINNED;

	if (task_running(env->src_rq, p)) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_running);
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_RUNNING);
		if( mt_lbprof_lt (env->sd->mt_lbprof_nr_balance_failed, MT_LBPROF_NR_BALANCED_FAILED_UPPER_BOUND)){
			char strings[128]="";
			snprintf(strings, 128, "%d:balance fail:running:%d:%d:%s"
				, env->dst_cpu, env->src_cpu, p->pid, p->comm);
			trace_sched_lbprof_log(strings);
		}
#endif		
		return 0;
	}

#if defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT)
	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd, env->mt_check_cache_in_idle);
#else
	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd);
#endif 
	if (!tsk_cache_hot ||
		env->sd->nr_balance_failed > env->sd->cache_nice_tries) {

		if (tsk_cache_hot) {
			schedstat_inc(env->sd, lb_hot_gained[env->idle]);
			schedstat_inc(p, se.statistics.nr_forced_migrations);
		}

		return 1;
	}

	schedstat_inc(p, se.statistics.nr_failed_migrations_hot);
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
	mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_CACHEHOT);
	if(mt_lbprof_lt (env->sd->mt_lbprof_nr_balance_failed, MT_LBPROF_NR_BALANCED_FAILED_UPPER_BOUND)){
		char strings[128]="";
		snprintf(strings, 128, "%d:balance fail:cache hot:%d:%d:%s"
			, env->dst_cpu, env->src_cpu, p->pid, p->comm);
		trace_sched_lbprof_log(strings);
	}
#endif		
	return 0;
}

static int move_one_task(struct lb_env *env)
{
	struct task_struct *p, *n;
#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
	env->mt_check_cache_in_idle = 1;
#endif
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
	mt_lbprof_stat_set(env->fail_reason, MT_LBPROF_NO_TRIGGER);
#endif

	list_for_each_entry_safe(p, n, &env->src_rq->cfs_tasks, se.group_node) {
#if defined (CONFIG_MTK_SCHED_CMP_LAZY_BALANCE) && !defined(CONFIG_HMP_LAZY_BALANCE)
		if(need_lazy_balance(env->dst_cpu, env->src_cpu, p))
			continue;
#endif			
		if (!can_migrate_task(p, env))
			continue;

		move_task(p, env);
		schedstat_inc(env->sd, lb_gained[env->idle]);
		return 1;
	}
	return 0;
}

static unsigned long task_h_load(struct task_struct *p);

static const unsigned int sched_nr_migrate_break = 32;

#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
	#define over_imbalance(lw, im) \
		(((lw)/2 > (im)) && \
        ((env->mt_check_cache_in_idle==1) || \
         (env->src_rq->rt.rt_nr_running==0) || \
         (pulled>0)))
#else
	#define over_imbalance(lw, im) (((lw) / 2) > (im))
#endif

static int move_tasks(struct lb_env *env)
{
	struct list_head *tasks = &env->src_rq->cfs_tasks;
	struct task_struct *p;
	unsigned long load;
	int pulled = 0;

	if (env->imbalance <= 0)
		return 0;

	mt_sched_printf("move_tasks start ");

	while (!list_empty(tasks)) {
		p = list_first_entry(tasks, struct task_struct, se.group_node);

		env->loop++;
		
		if (env->loop > env->loop_max)
			break;

		
		if (env->loop > env->loop_break) {
			env->loop_break += sched_nr_migrate_break;
			env->flags |= LBF_NEED_BREAK;
			break;
		}
#if defined (CONFIG_MTK_SCHED_CMP_LAZY_BALANCE) && !defined(CONFIG_HMP_LAZY_BALANCE)
		if(need_lazy_balance(env->dst_cpu, env->src_cpu, p))	
			goto next;
#endif
		if (!can_migrate_task(p, env))
			goto next;

		load = task_h_load(p);

		if (sched_feat(LB_MIN) && load < 16 && !env->sd->nr_balance_failed)
			goto next;

		if (over_imbalance(load, env->imbalance))
			{
			goto next;
			}

		move_task(p, env);
		pulled++;
		env->imbalance -= load;

#ifdef CONFIG_PREEMPT
		if (env->idle == CPU_NEWLY_IDLE)
			break;
#endif

		if (env->imbalance <= 0)
			break;

		continue;
next:
		list_move_tail(&p->se.group_node, tasks);
	}

	schedstat_add(env->sd, lb_gained[env->idle], pulled);

	mt_sched_printf("move_tasks end");

	return pulled;
}

#ifdef CONFIG_MTK_SCHED_CMP
#ifdef CONFIG_MTK_SCHED_CMP_TGS
static int cmp_can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	struct sched_domain *sd = env->sd;

	BUG_ON(sd == NULL);

	if (!(sd->flags & SD_BALANCE_TG))
		return 0;

	if (arch_is_multi_cluster()) {
		int src_clid, dst_clid;
		int src_nr_cpus;
		struct thread_group_info_t *src_tginfo, *dst_tginfo;

		src_clid = get_cluster_id(env->src_cpu);
		dst_clid = get_cluster_id(env->dst_cpu);
		BUG_ON(dst_clid == -1 || src_clid == -1);
		BUG_ON(p == NULL || p->group_leader == NULL);
		src_tginfo = &p->group_leader->thread_group_info[src_clid];
		dst_tginfo = &p->group_leader->thread_group_info[dst_clid];
		src_nr_cpus = nr_cpus_in_cluster(src_clid, false);

#ifdef CONFIG_MT_SCHED_INFO
		mt_sched_printf("check rule0: pid=%d comm=%s load=%ld src:clid=%d tginfo->nr_running=%ld nr_cpus=%d load_avg_ratio=%ld",
			p->pid, p->comm, p->se.avg.load_avg_ratio,
			src_clid, src_tginfo->nr_running, src_nr_cpus,
			src_tginfo->load_avg_ratio);
#endif
#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
		if ( (!thread_group_empty(p)) &&
			 (src_tginfo->nr_running <= src_nr_cpus) &&
			 (src_tginfo->nr_running > dst_tginfo->nr_running)){
			mt_sched_printf("hit ruleA: bypass pid=%d comm=%s src:nr_running=%lu nr_cpus=%d dst:nr_running=%lu",
				p->pid, p->comm, src_tginfo->nr_running, src_nr_cpus, dst_tginfo->nr_running);
			return 0;
		}
#endif
	}
	return 1;
}

static int need_migrate_task_immediately(struct task_struct *p,
           struct lb_env *env, struct clb_env *clbenv)
{
	struct sched_domain *sd = env->sd;

	BUG_ON(sd == NULL);

	if (arch_is_big_little()) {
		mt_sched_printf("[%s] b.L arch", __func__);
#ifdef CONFIG_MT_SCHED_INFO
		mt_sched_printf("check rule0: pid=%d comm=%s src=%d dst=%d p->prio=%d p->se.avg.load_avg_ratio=%ld",
			p->pid, p->comm, env->src_cpu, env->dst_cpu, p->prio, p->se.avg.load_avg_ratio);
#endif
		
		if (arch_cpu_is_little(env->src_cpu) && arch_cpu_is_big(env->dst_cpu)) {
			BUG_ON(env->src_cpu != clbenv->ltarget);
			if (p->se.avg.load_avg_ratio >= clbenv->bstats.threshold)
 				return 1;

		
		} else if (arch_cpu_is_big(env->src_cpu) && arch_cpu_is_little(env->dst_cpu)) {
			BUG_ON(env->src_cpu != clbenv->btarget);
			if (p->se.avg.load_avg_ratio < clbenv->lstats.threshold)
				return 1;
		}
		return 0;
	}

	if (arch_is_multi_cluster() && (sd->flags & SD_BALANCE_TG)) {
		int src_clid, dst_clid;
		int src_nr_cpus;
		struct thread_group_info_t *src_tginfo, *dst_tginfo;

		src_clid = get_cluster_id(env->src_cpu);
		dst_clid = get_cluster_id(env->dst_cpu);
		BUG_ON(dst_clid == -1 || src_clid == -1);
		BUG_ON(p == NULL || p->group_leader == NULL);
		src_tginfo = &p->group_leader->thread_group_info[src_clid];
		dst_tginfo = &p->group_leader->thread_group_info[dst_clid];
		src_nr_cpus = nr_cpus_in_cluster(src_clid, false);
		mt_sched_printf("[%s] L.L arch", __func__);

		if ((p->se.avg.load_avg_ratio*4  >=  NICE_0_LOAD*3) &&
			src_tginfo->nr_running > src_nr_cpus &&
			src_tginfo->load_avg_ratio*10 > NICE_0_LOAD*src_nr_cpus*9) {
			
			
			return 1;
		}
	}

	return 0;
}
#endif

static int cmp_move_tasks(struct sched_domain *sd, struct lb_env *env)
{
	struct list_head *tasks = &env->src_rq->cfs_tasks;
	struct task_struct *p;
	unsigned long load = 0;
	int pulled = 0;

	long tg_load_move, other_load_move;
	struct list_head tg_tasks, other_tasks;
	int src_clid, dst_clid;
#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
	struct cpumask tmp, *cpus = &tmp;
#endif
#ifdef MTK_QUICK 
	int flag = 0;
#endif
	struct clb_env clbenv;
	struct cpumask srcmask, dstmask;

	if (env->imbalance <= 0)
		return 0;

	other_load_move = env->imbalance;
	INIT_LIST_HEAD(&other_tasks);

		tg_load_move = env->imbalance;
		INIT_LIST_HEAD(&tg_tasks);
		src_clid = get_cluster_id(env->src_cpu);
		dst_clid = get_cluster_id(env->dst_cpu);
		BUG_ON(dst_clid == -1 || src_clid == -1);

#ifdef CONFIG_MTK_SCHED_CMP_TGS_WAKEUP
		get_cluster_cpus(cpus, src_clid, true);
#endif
		mt_sched_printf("move_tasks_tg start: src:cpu=%d clid=%d runnable_load=%lu dst:cpu=%d clid=%d runnable_load=%lu imbalance=%ld curr->on_rq=%d",
			env->src_cpu, src_clid, cpu_rq(env->src_cpu)->cfs.runnable_load_avg,
			env->dst_cpu, dst_clid, cpu_rq(env->dst_cpu)->cfs.runnable_load_avg,
			env->imbalance, env->dst_rq->curr->on_rq);

	mt_sched_printf("max=%d busiest->nr_running=%d",
		env->loop_max, cpu_rq(env->src_cpu)->nr_running);

	if (arch_is_big_little()) {
		get_cluster_cpus(&srcmask, src_clid, true);
		get_cluster_cpus(&dstmask, dst_clid, true);
		memset(&clbenv, 0, sizeof(clbenv));
		clbenv.flags |= HMP_LB;
		clbenv.ltarget = arch_cpu_is_little(env->src_cpu) ? env->src_cpu : env->dst_cpu;
		clbenv.btarget = arch_cpu_is_big(env->src_cpu) ? env->src_cpu : env->dst_cpu;
		clbenv.lcpus = arch_cpu_is_little(env->src_cpu) ? &srcmask : &dstmask;
		clbenv.bcpus = arch_cpu_is_big(env->src_cpu) ? &srcmask : &dstmask;
		sched_update_clbstats(&clbenv);
	}

	while (!list_empty(tasks)) {
		struct thread_group_info_t *src_tginfo, *dst_tginfo;

		p = list_first_entry(tasks, struct task_struct, se.group_node);

#ifdef CONFIG_MT_SCHED_INFO
		mt_sched_printf("check: pid=%d comm=%s load_avg_contrib=%lu h_load=%lu runnable_load_avg=%lu loop=%d, env->imbalance=%ld tg_load_move=%ld",
			p->pid, p->comm, p->se.avg.load_avg_contrib,
			task_cfs_rq(p)->h_load, task_cfs_rq(p)->runnable_load_avg,
			env->loop, env->imbalance, tg_load_move);
#endif
		env->loop++;
		
		if (env->loop > env->loop_max)
			break;

#if 0 
		
		if (env->loop > env->loop_break) {
			env->loop_break += sched_nr_migrate_break;
			env->flags |= LBF_NEED_BREAK;
			break;
		}
#endif
		BUG_ON(p == NULL || p->group_leader == NULL);
		src_tginfo = &p->group_leader->thread_group_info[src_clid];
		dst_tginfo = &p->group_leader->thread_group_info[dst_clid];

		
		if (!can_migrate_task(p, env)) {
			mt_sched_printf("can not migrate: pid=%d comm=%s",
				p->pid, p->comm);
			goto next;
		}

		load = task_h_load(p);

		if (sched_feat(LB_MIN) && load < 16 && !env->sd->nr_balance_failed) {
			mt_sched_printf("can not migrate: pid=%d comm=%s sched_feat",
				p->pid, p->comm );
			goto next;
		}

		if (over_imbalance(load, env->imbalance)) {
			mt_sched_printf("can not migrate: pid=%d comm=%s load=%ld imbalance=%ld",
				p->pid, p->comm, load, env->imbalance );
			goto next;
		}

		
		if (need_migrate_task_immediately(p, env, &clbenv)) {
			pulled++;
			env->imbalance -= load;
			tg_load_move -= load;
			other_load_move -= load;
			mt_sched_printf("hit rule0: pid=%d comm=%s load=%ld imbalance=%ld tg_imbalance=%ld other_load_move=%ld",
				p->pid, p->comm, load, env->imbalance, tg_load_move, other_load_move);
			move_task(p, env);
			if (env->imbalance <= 0)
				break;
			continue;
		}

		
		if (!cmp_can_migrate_task(p, env))
			goto next;

		if (sd->flags & SD_BALANCE_TG){
			if (over_imbalance(load, tg_load_move)) {
				mt_sched_printf("can not migrate: pid=%d comm=%s load=%ld imbalance=%ld",
					p->pid, p->comm, load, tg_load_move );
				goto next;
			}

#ifdef MTK_QUICK 
			if (candidate_load_move <= 0) {
				mt_sched_printf("check: pid=%d comm=%s candidate_load_move=%d",
					p->pid, p->comm, candidate_load_move);
				goto next;
			}
#endif

			
#ifdef CONFIG_MT_SCHED_INFO
			mt_sched_printf("check rule1: pid=%d p->comm=%s thread_group_cnt=%lu thread_group_empty(p)=%d",
				p->pid, p->comm, 
				p->group_leader->thread_group_info[0].nr_running + 
				p->group_leader->thread_group_info[1].nr_running, 
				thread_group_empty(p));
#endif

			if (thread_group_empty(p)) {
				list_move_tail(&p->se.group_node, &tg_tasks);
				tg_load_move -= load;
				other_load_move -= load;
				mt_sched_printf("hit rule1: pid=%d p->comm=%s load=%ld tg_imbalance=%ld",
				   p->pid, p->comm, load, tg_load_move);
				continue;
			}

			
#ifdef CONFIG_MT_SCHED_INFO
			mt_sched_printf("check rule2: pid=%d p->comm=%s %ld, %ld, %ld, %ld, %ld",
				p->pid, p->comm, src_tginfo->nr_running, src_tginfo->cfs_nr_running, dst_tginfo->nr_running, 
				p->se.avg.load_avg_ratio, src_tginfo->load_avg_ratio);
#endif
			if ((src_tginfo->nr_running < dst_tginfo->nr_running) &&
			   ((p->se.avg.load_avg_ratio * src_tginfo->cfs_nr_running) <= 
			    src_tginfo->load_avg_ratio)) {
				list_move_tail(&p->se.group_node, &tg_tasks);
				tg_load_move -= load;
				other_load_move -= load;
				mt_sched_printf("hit rule2: pid=%d p->comm=%s load=%ld tg_imbalance=%ld",
				   p->pid, p->comm, load, tg_load_move);
				continue;
			}

			if (over_imbalance(load, other_load_move))
				goto next;

			list_move_tail(&p->se.group_node, &other_tasks);
			other_load_move -= load;
			continue;
		}else{
			list_move_tail(&p->se.group_node, &other_tasks);
			other_load_move -= load;
			continue;
		}

	
#if defined (CONFIG_MTK_SCHED_CMP_LAZY_BALANCE) && !defined(CONFIG_HMP_LAZY_BALANCE)
		if(need_lazy_balance(env->dst_cpu, env->src_cpu, p))
			goto next;
#endif

next:
		
		list_move_tail(&p->se.group_node, tasks);
	} 

	if ( sd->flags & SD_BALANCE_TG){
		while (!list_empty(&tg_tasks)) {
			p = list_first_entry(&tg_tasks, struct task_struct, se.group_node);
			list_move_tail(&p->se.group_node, tasks);

			if (env->imbalance > 0) {
				load = task_h_load(p);
				if (over_imbalance(load, env->imbalance)){
					mt_sched_printf("overload rule1,2: pid=%d p->comm=%s load=%ld imbalance=%ld",
			   	   	   p->pid, p->comm, load, env->imbalance);
#ifdef MTK_QUICK 

					flag=1;
#endif
					continue;
				}

				move_task(p, env);
				env->imbalance -= load;
				pulled++;

				mt_sched_printf("migrate hit rule1,2: pid=%d p->comm=%s load=%ld imbalance=%ld",
			   	   p->pid, p->comm, load, env->imbalance);
			}
		}
	}

	mt_sched_printf("move_tasks_tg finish rule migrate");

	while (!list_empty(&other_tasks)) {
		p = list_first_entry(&other_tasks, struct task_struct, se.group_node);
		list_move_tail(&p->se.group_node, tasks);

#ifdef MTK_QUICK
		if (!flag && (env->imbalance > 0)) {
#else
		if (env->imbalance > 0) {
#endif
			load = task_h_load(p);

			if (over_imbalance(load, env->imbalance)){
				mt_sched_printf("overload others: pid=%d p->comm=%s load=%ld imbalance=%ld",
		   	   	   p->pid, p->comm, load, env->imbalance);
				continue;
			}

			move_task(p, env);
			env->imbalance -= load;
			pulled++;

			mt_sched_printf("migrate others: pid=%d p->comm=%s load=%ld imbalance=%ld",
		  	   p->pid, p->comm, load, env->imbalance);
		}
	}

	schedstat_add(env->sd, lb_gained[env->idle], pulled);

 	mt_sched_printf("move_tasks_tg finish pulled=%d imbalance=%ld", pulled, env->imbalance);

	return pulled;
}

#endif 


#if defined (CONFIG_MTK_SCHED_CMP_LAZY_BALANCE) && !defined(CONFIG_HMP_LAZY_BALANCE)
static int need_lazy_balance(int dst_cpu, int src_cpu, struct task_struct *p)
{
	
#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER		
		if ( PA_ENABLE && cpumask_test_cpu(src_cpu, &buddy_cpu_map) &&
			!is_buddy_busy(src_cpu) && is_light_task(p)) {
#else
		if (cpumask_test_cpu(src_cpu, &buddy_cpu_map) &&
			!is_buddy_busy(src_cpu) && is_light_task(p)) {
#endif
#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER
			unsigned int i;
			AVOID_LOAD_BALANCE_FROM_CPUX_TO_CPUY_COUNT[src_cpu][dst_cpu]++;
			mt_sched_printf("[PA]pid=%d, Lazy balance from CPU%d to CPU%d\n)\n", p->pid, src_cpu, dst_cpu);
			for(i=0;i<4;i++) {
				if(PA_MON_ENABLE && (strcmp(p->comm, &PA_MON[i][0]) == 0)) {
					printk(KERN_EMERG "[PA] %s Lazy balance from CPU%d to CPU%d\n", p->comm, src_cpu, dst_cpu);
	
	
	
	
	
	
	
				}
			}
#endif		
			return 1;
		}
		else
			return 0;
}
#endif
#ifdef CONFIG_FAIR_GROUP_SCHED
static void __update_blocked_averages_cpu(struct task_group *tg, int cpu)
{
	struct sched_entity *se = tg->se[cpu];
	struct cfs_rq *cfs_rq = tg->cfs_rq[cpu];

	
	if (throttled_hierarchy(cfs_rq))
		return;

	update_cfs_rq_blocked_load(cfs_rq, 1);

	if (se) {
		update_entity_load_avg(se, 1);
		if (!se->avg.runnable_avg_sum && !cfs_rq->nr_running)
			list_del_leaf_cfs_rq(cfs_rq);
	} else {
		struct rq *rq = rq_of(cfs_rq);
		update_rq_runnable_avg(rq, rq->nr_running);
	}
}

static void update_blocked_averages(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	struct cfs_rq *cfs_rq;
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);
	update_rq_clock(rq);
	for_each_leaf_cfs_rq(rq, cfs_rq) {
		__update_blocked_averages_cpu(cfs_rq->tg, rq->cpu);
	}

	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

static int tg_load_down(struct task_group *tg, void *data)
{
	unsigned long load;
	long cpu = (long)data;

	if (!tg->parent) {
		load = cpu_rq(cpu)->cfs.runnable_load_avg;
	} else {
		load = tg->parent->cfs_rq[cpu]->h_load;
		load = div64_ul(load * tg->se[cpu]->avg.load_avg_contrib,
				tg->parent->cfs_rq[cpu]->runnable_load_avg + 1);
	}

	tg->cfs_rq[cpu]->h_load = load;

	return 0;
}

static void update_h_load(long cpu)
{
	rcu_read_lock();
	walk_tg_tree(tg_load_down, tg_nop, (void *)cpu);
	rcu_read_unlock();
}

static unsigned long task_h_load(struct task_struct *p)
{
	struct cfs_rq *cfs_rq = task_cfs_rq(p);

	return div64_ul(p->se.avg.load_avg_contrib * cfs_rq->h_load,
			cfs_rq->runnable_load_avg + 1);
}
#else
static inline void update_blocked_averages(int cpu)
{
}

static inline void update_h_load(long cpu)
{
}

static unsigned long task_h_load(struct task_struct *p)
{
	return p->se.avg.load_avg_contrib;
}
#endif

struct sd_lb_stats {
	struct sched_group *busiest; 
	struct sched_group *this;  
	unsigned long total_load;  
	unsigned long total_pwr;   
	unsigned long avg_load;	   

	
	unsigned long this_load;
	unsigned long this_load_per_task;
	unsigned long this_nr_running;
	unsigned long this_has_capacity;
	unsigned int  this_idle_cpus;

	
	unsigned int  busiest_idle_cpus;
	unsigned long max_load;
	unsigned long busiest_load_per_task;
	unsigned long busiest_nr_running;
	unsigned long busiest_group_capacity;
	unsigned long busiest_has_capacity;
	unsigned int  busiest_group_weight;

	int group_imb; 
};

struct sg_lb_stats {
	unsigned long avg_load; 
	unsigned long group_load; 
	unsigned long sum_nr_running; 
	unsigned long sum_weighted_load; 
	unsigned long group_capacity;
	unsigned long idle_cpus;
	unsigned long group_weight;
	int group_imb; 
	int group_has_capacity; 
};

static inline int get_sd_load_idx(struct sched_domain *sd,
					enum cpu_idle_type idle)
{
	int load_idx;

	switch (idle) {
	case CPU_NOT_IDLE:
		load_idx = sd->busy_idx;
		break;

	case CPU_NEWLY_IDLE:
		load_idx = sd->newidle_idx;
		break;
	default:
		load_idx = sd->idle_idx;
		break;
	}

	return load_idx;
}

static unsigned long default_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return SCHED_POWER_SCALE;
}

unsigned long __weak arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return default_scale_freq_power(sd, cpu);
}

static unsigned long default_scale_smt_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long smt_gain = sd->smt_gain;

	smt_gain /= weight;

	return smt_gain;
}

unsigned long __weak arch_scale_smt_power(struct sched_domain *sd, int cpu)
{
	return default_scale_smt_power(sd, cpu);
}

static unsigned long scale_rt_power(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	u64 total, available, age_stamp, avg;

	age_stamp = ACCESS_ONCE(rq->age_stamp);
	avg = ACCESS_ONCE(rq->rt_avg);

	total = sched_avg_period() + (rq->clock - age_stamp);

	if (unlikely(total < avg)) {
		
		available = 0;
	} else {
		available = total - avg;
	}

	if (unlikely((s64)total < SCHED_POWER_SCALE))
		total = SCHED_POWER_SCALE;

	total >>= SCHED_POWER_SHIFT;

	return div_u64(available, total);
}

static void update_cpu_power(struct sched_domain *sd, int cpu)
{
	unsigned long weight = sd->span_weight;
	unsigned long power = SCHED_POWER_SCALE;
	struct sched_group *sdg = sd->groups;

	if ((sd->flags & SD_SHARE_CPUPOWER) && weight > 1) {
		if (sched_feat(ARCH_POWER))
			power *= arch_scale_smt_power(sd, cpu);
		else
			power *= default_scale_smt_power(sd, cpu);

		power >>= SCHED_POWER_SHIFT;
	}

	sdg->sgp->power_orig = power;

	if (sched_feat(ARCH_POWER))
		power *= arch_scale_freq_power(sd, cpu);
	else
		power *= default_scale_freq_power(sd, cpu);

	power >>= SCHED_POWER_SHIFT;

	power *= scale_rt_power(cpu);
	power >>= SCHED_POWER_SHIFT;

	if (!power)
		power = 1;

	cpu_rq(cpu)->cpu_power = power;
	sdg->sgp->power = power;
}

void update_group_power(struct sched_domain *sd, int cpu)
{
	struct sched_domain *child = sd->child;
	struct sched_group *group, *sdg = sd->groups;
	unsigned long power;
	unsigned long interval;

	interval = msecs_to_jiffies(sd->balance_interval);
	interval = clamp(interval, 1UL, max_load_balance_interval);
	sdg->sgp->next_update = jiffies + interval;

	if (!child) {
		update_cpu_power(sd, cpu);
		return;
	}

	power = 0;

	if (child->flags & SD_OVERLAP) {

		for_each_cpu(cpu, sched_group_cpus(sdg))
			power += power_of(cpu);
	} else  {
 

		group = child->groups;
		do {
			power += group->sgp->power;
			group = group->next;
		} while (group != child->groups);
	}

	sdg->sgp->power_orig = sdg->sgp->power = power;
}

static inline int
fix_small_capacity(struct sched_domain *sd, struct sched_group *group)
{
	if (!(sd->flags & SD_SHARE_CPUPOWER))
		return 0;

	if (group->sgp->power * 32 > group->sgp->power_orig * 29)
		return 1;

	return 0;
}

static inline void update_sg_lb_stats(struct lb_env *env,
			struct sched_group *group, int load_idx,
			int local_group, int *balance, struct sg_lb_stats *sgs)
{
	unsigned long nr_running, max_nr_running, min_nr_running;
	unsigned long load, max_cpu_load, min_cpu_load;
	unsigned int balance_cpu = -1, first_idle_cpu = 0;
	unsigned long avg_load_per_task = 0;
	int i;

	if (local_group)
		balance_cpu = group_balance_cpu(group);

	
	max_cpu_load = 0;
	min_cpu_load = ~0UL;
	max_nr_running = 0;
	min_nr_running = ~0UL;

	for_each_cpu_and(i, sched_group_cpus(group), env->cpus) {
		struct rq *rq = cpu_rq(i);

		nr_running = rq->nr_running;

		
		if (local_group) {
			if (idle_cpu(i) && !first_idle_cpu &&
					cpumask_test_cpu(i, sched_group_mask(group))) {
				first_idle_cpu = 1;
				balance_cpu = i;
			}

			load = target_load(i, load_idx);
		} else {
			load = source_load(i, load_idx);
			if (load > max_cpu_load)
				max_cpu_load = load;
			if (min_cpu_load > load)
				min_cpu_load = load;

			if (nr_running > max_nr_running)
				max_nr_running = nr_running;
			if (min_nr_running > nr_running)
				min_nr_running = nr_running;

#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER  
			if((load_idx > 0) && (load == cpu_rq(i)->cpu_load[load_idx-1]))
				mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_HISTORY);
#endif
		}

		sgs->group_load += load;
		sgs->sum_nr_running += nr_running;
		sgs->sum_weighted_load += weighted_cpuload(i);
		if (idle_cpu(i))
			sgs->idle_cpus++;
	}

	if (local_group) {
		if (env->idle != CPU_NEWLY_IDLE) {
			if (balance_cpu != env->dst_cpu) {
				*balance = 0;
				return;
			}
			update_group_power(env->sd, env->dst_cpu);
		} else if (time_after_eq(jiffies, group->sgp->next_update))
			update_group_power(env->sd, env->dst_cpu);
	}

	
	sgs->avg_load = (sgs->group_load*SCHED_POWER_SCALE) / group->sgp->power;

	if (sgs->sum_nr_running)
		avg_load_per_task = sgs->sum_weighted_load / sgs->sum_nr_running;

	if ((max_cpu_load - min_cpu_load) >= avg_load_per_task &&
	    (max_nr_running - min_nr_running) > 1)
		sgs->group_imb = 1;

	sgs->group_capacity = DIV_ROUND_CLOSEST(group->sgp->power,
						SCHED_POWER_SCALE);
	if (!sgs->group_capacity)
		sgs->group_capacity = fix_small_capacity(env->sd, group);
	sgs->group_weight = group->group_weight;

	if (sgs->group_capacity > sgs->sum_nr_running)
		sgs->group_has_capacity = 1;
}

static bool update_sd_pick_busiest(struct lb_env *env,
				   struct sd_lb_stats *sds,
				   struct sched_group *sg,
				   struct sg_lb_stats *sgs)
{
	if (sgs->avg_load <= sds->max_load) {
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_PICK_BUSIEST_FAIL_1);
		return false;
	}		

	if (sgs->sum_nr_running > sgs->group_capacity)
		return true;

	if (sgs->group_imb)
		return true;

	if ((env->sd->flags & SD_ASYM_PACKING) && sgs->sum_nr_running &&
	    env->dst_cpu < group_first_cpu(sg)) {
		if (!sds->busiest)
			return true;

		if (group_first_cpu(sds->busiest) > group_first_cpu(sg))
			return true;
	}

	mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_PICK_BUSIEST_FAIL_2);
	return false;
}

static inline void update_sd_lb_stats(struct lb_env *env,
					int *balance, struct sd_lb_stats *sds)
{
	struct sched_domain *child = env->sd->child;
	struct sched_group *sg = env->sd->groups;
	struct sg_lb_stats sgs;
	int load_idx, prefer_sibling = 0;

	if (child && child->flags & SD_PREFER_SIBLING)
		prefer_sibling = 1;

	load_idx = get_sd_load_idx(env->sd, env->idle);

	do {
		int local_group;

		local_group = cpumask_test_cpu(env->dst_cpu, sched_group_cpus(sg));
		memset(&sgs, 0, sizeof(sgs));
		update_sg_lb_stats(env, sg, load_idx, local_group, balance, &sgs);

		if (local_group && !(*balance))
			return;

		sds->total_load += sgs.group_load;
		sds->total_pwr += sg->sgp->power;

		if (prefer_sibling && !local_group && sds->this_has_capacity)
			sgs.group_capacity = min(sgs.group_capacity, 1UL);

		if (local_group) {
			sds->this_load = sgs.avg_load;
			sds->this = sg;
			sds->this_nr_running = sgs.sum_nr_running;
			sds->this_load_per_task = sgs.sum_weighted_load;
			sds->this_has_capacity = sgs.group_has_capacity;
			sds->this_idle_cpus = sgs.idle_cpus;
		} else if (update_sd_pick_busiest(env, sds, sg, &sgs)) {
			sds->max_load = sgs.avg_load;
			sds->busiest = sg;
			sds->busiest_nr_running = sgs.sum_nr_running;
			sds->busiest_idle_cpus = sgs.idle_cpus;
			sds->busiest_group_capacity = sgs.group_capacity;
			sds->busiest_load_per_task = sgs.sum_weighted_load;
			sds->busiest_has_capacity = sgs.group_has_capacity;
			sds->busiest_group_weight = sgs.group_weight;
			sds->group_imb = sgs.group_imb;
		}

		sg = sg->next;
	} while (sg != env->sd->groups);
}

static int check_asym_packing(struct lb_env *env, struct sd_lb_stats *sds)
{
	int busiest_cpu;

	if (!(env->sd->flags & SD_ASYM_PACKING))
		return 0;

	if (!sds->busiest)
		return 0;

	busiest_cpu = group_first_cpu(sds->busiest);
	if (env->dst_cpu > busiest_cpu)
		return 0;

	env->imbalance = DIV_ROUND_CLOSEST(
		sds->max_load * sds->busiest->sgp->power, SCHED_POWER_SCALE);

	return 1;
}

static inline
void fix_small_imbalance(struct lb_env *env, struct sd_lb_stats *sds)
{
	unsigned long tmp, pwr_now = 0, pwr_move = 0;
	unsigned int imbn = 2;
	unsigned long scaled_busy_load_per_task;

	if (sds->this_nr_running) {
		sds->this_load_per_task /= sds->this_nr_running;
		if (sds->busiest_load_per_task >
				sds->this_load_per_task)
			imbn = 1;
	} else {
		sds->this_load_per_task =
			cpu_avg_load_per_task(env->dst_cpu);
	}

	scaled_busy_load_per_task = sds->busiest_load_per_task
					 * SCHED_POWER_SCALE;
	scaled_busy_load_per_task /= sds->busiest->sgp->power;

	if (sds->max_load - sds->this_load + scaled_busy_load_per_task >=
			(scaled_busy_load_per_task * imbn)) {
		env->imbalance = sds->busiest_load_per_task;
		return;
	}


	pwr_now += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load);
	pwr_now += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load);
	pwr_now /= SCHED_POWER_SCALE;

	
	tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
		sds->busiest->sgp->power;
	if (sds->max_load > tmp)
		pwr_move += sds->busiest->sgp->power *
			min(sds->busiest_load_per_task, sds->max_load - tmp);

	
	if (sds->max_load * sds->busiest->sgp->power <
		sds->busiest_load_per_task * SCHED_POWER_SCALE)
		tmp = (sds->max_load * sds->busiest->sgp->power) /
			sds->this->sgp->power;
	else
		tmp = (sds->busiest_load_per_task * SCHED_POWER_SCALE) /
			sds->this->sgp->power;
	pwr_move += sds->this->sgp->power *
			min(sds->this_load_per_task, sds->this_load + tmp);
	pwr_move /= SCHED_POWER_SCALE;

	
	if (pwr_move > pwr_now)
		env->imbalance = sds->busiest_load_per_task;
}

static inline void calculate_imbalance(struct lb_env *env, struct sd_lb_stats *sds)
{
	unsigned long max_pull, load_above_capacity = ~0UL;

	sds->busiest_load_per_task /= sds->busiest_nr_running;
	if (sds->group_imb) {
		sds->busiest_load_per_task =
			min(sds->busiest_load_per_task, sds->avg_load);
	}

	if (sds->max_load < sds->avg_load) {
		env->imbalance = 0;
		return fix_small_imbalance(env, sds);
	}

	if (!sds->group_imb) {
		load_above_capacity = (sds->busiest_nr_running -
						sds->busiest_group_capacity);

		load_above_capacity *= (SCHED_LOAD_SCALE * SCHED_POWER_SCALE);

		load_above_capacity /= sds->busiest->sgp->power;
	}

	max_pull = min(sds->max_load - sds->avg_load, load_above_capacity);

	
	env->imbalance = min(max_pull * sds->busiest->sgp->power,
		(sds->avg_load - sds->this_load) * sds->this->sgp->power)
			/ SCHED_POWER_SCALE;

	if (env->imbalance < sds->busiest_load_per_task)
		return fix_small_imbalance(env, sds);

}


static struct sched_group *
find_busiest_group(struct lb_env *env, int *balance)
{
	struct sd_lb_stats sds;

	memset(&sds, 0, sizeof(sds));

	update_sd_lb_stats(env, balance, &sds);

	if (!(*balance)){
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_BALANCE);
		goto ret;
	}

	if ((env->idle == CPU_IDLE || env->idle == CPU_NEWLY_IDLE) &&
	    check_asym_packing(env, &sds))
		return sds.busiest;

	
	if (!sds.busiest || sds.busiest_nr_running == 0){
		if(!sds.busiest){
			mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_NOBUSYG_BUSIEST_NO_TASK);
		}else{
			mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_NOBUSYG_NO_BUSIEST);
		}		
		goto out_balanced;
	}

	sds.avg_load = (SCHED_POWER_SCALE * sds.total_load) / sds.total_pwr;

	if (sds.group_imb)
		goto force_balance;

	
	if (env->idle == CPU_NEWLY_IDLE && sds.this_has_capacity &&
			!sds.busiest_has_capacity)
		goto force_balance;

	if (sds.this_load >= sds.max_load){
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_NOBUSYG_NO_LARGER_THAN);		
		goto out_balanced;
	}

	if (sds.this_load >= sds.avg_load){
		mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_NOBUSYG_NO_LARGER_THAN);
		goto out_balanced;
	}

	if (env->idle == CPU_IDLE) {
		if ((sds.this_idle_cpus <= sds.busiest_idle_cpus + 1) &&
		    sds.busiest_nr_running <= sds.busiest_group_weight)
			goto out_balanced;
	} else {
		if (100 * sds.max_load <= env->sd->imbalance_pct * sds.this_load){
			mt_lbprof_stat_or(env->fail_reason, MT_LBPROF_NOBUSYG_CHECK_FAIL);
			goto out_balanced;
		}
	}

force_balance:
	
	calculate_imbalance(env, &sds);
	return sds.busiest;

out_balanced:
ret:
	env->imbalance = 0;
	return NULL;
}

static struct rq *find_busiest_queue(struct lb_env *env,
				     struct sched_group *group)
{
	struct rq *busiest = NULL, *rq;
	unsigned long max_load = 0;
	int i;

	for_each_cpu(i, sched_group_cpus(group)) {
		unsigned long power = power_of(i);
		unsigned long capacity = DIV_ROUND_CLOSEST(power,
							   SCHED_POWER_SCALE);
		unsigned long wl;

		if (!capacity)
			capacity = fix_small_capacity(env->sd, group);

		if (!cpumask_test_cpu(i, env->cpus))
			continue;

		rq = cpu_rq(i);
		wl = weighted_cpuload(i);

		if (capacity && rq->nr_running == 1 && wl > env->imbalance)
			continue;

		wl = (wl * SCHED_POWER_SCALE) / power;

		if (wl > max_load) {
			max_load = wl;
			busiest = rq;
		}
	}

	return busiest;
}

#define MAX_PINNED_INTERVAL	512

DEFINE_PER_CPU(cpumask_var_t, load_balance_mask);

static int need_active_balance(struct lb_env *env)
{
	struct sched_domain *sd = env->sd;

	if (env->idle == CPU_NEWLY_IDLE) {

		if ((sd->flags & SD_ASYM_PACKING) && env->src_cpu > env->dst_cpu)
			return 1;
	}

	return unlikely(sd->nr_balance_failed > sd->cache_nice_tries+2);
}

static int active_load_balance_cpu_stop(void *data);

static int load_balance(int this_cpu, struct rq *this_rq,
			struct sched_domain *sd, enum cpu_idle_type idle,
			int *balance)
{
	int ld_moved, cur_ld_moved, active_balance = 0;
	struct sched_group *group;
	struct rq *busiest;
	unsigned long flags;
	struct cpumask *cpus = __get_cpu_var(load_balance_mask);

	struct lb_env env = {
		.sd		= sd,
		.dst_cpu	= this_cpu,
		.dst_rq		= this_rq,
		.dst_grpmask    = sched_group_cpus(sd->groups),
		.idle		= idle,
		.loop_break	= sched_nr_migrate_break,
		.cpus		= cpus,
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
		.fail_reason= MT_LBPROF_NO_TRIGGER,
#endif
	};

	if (idle == CPU_NEWLY_IDLE)
		env.dst_grpmask = NULL;

	cpumask_copy(cpus, cpu_active_mask);

	schedstat_inc(sd, lb_count[idle]);

redo:
	group = find_busiest_group(&env, balance);

	if (*balance == 0)
		goto out_balanced;

	if (!group) {
		schedstat_inc(sd, lb_nobusyg[idle]);
		if(mt_lbprof_test(env.fail_reason, MT_LBPROF_HISTORY)){
			int tmp_cpu;
			for_each_cpu(tmp_cpu, cpu_possible_mask){
				if (tmp_cpu == this_rq->cpu)
					continue;
				mt_lbprof_update_state(tmp_cpu, MT_LBPROF_BALANCE_FAIL_STATE);
			}
		}		
		goto out_balanced;
	}

	busiest = find_busiest_queue(&env, group);
	if (!busiest) {
		schedstat_inc(sd, lb_nobusyq[idle]);
		mt_lbprof_stat_or(env.fail_reason, MT_LBPROF_NOBUSYQ);
		goto out_balanced;
	}

#ifdef CONFIG_HMP_LAZY_BALANCE

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
	if (PA_ENABLE && LB_ENABLE) {
#endif 
 
		if (per_cpu(sd_pack_buddy, this_cpu) == busiest->cpu && !is_buddy_busy(per_cpu(sd_pack_buddy, this_cpu))) {

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
			AVOID_LOAD_BALANCE_FROM_CPUX_TO_CPUY_COUNT[this_cpu][busiest->cpu]++;

#ifdef CONFIG_HMP_TRACER
			trace_sched_power_aware_active(POWER_AWARE_ACTIVE_MODULE_AVOID_BALANCE_FORM_CPUX_TO_CPUY, 0, this_cpu, busiest->cpu);
#endif 

#endif 

			schedstat_inc(sd, lb_nobusyq[idle]);
			goto out_balanced;		
		}

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER    
	}
#endif 

#endif 

	BUG_ON(busiest == env.dst_rq);

	schedstat_add(sd, lb_imbalance[idle], env.imbalance);

	ld_moved = 0;
	if (busiest->nr_running > 1) {
		env.flags |= LBF_ALL_PINNED;
		env.src_cpu   = busiest->cpu;
		env.src_rq    = busiest;
		env.loop_max  = min(sysctl_sched_nr_migrate, busiest->nr_running);
#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
		env.mt_check_cache_in_idle = 1;
#endif

		update_h_load(env.src_cpu);
more_balance:
		local_irq_save(flags);
		double_rq_lock(env.dst_rq, busiest);
#ifdef CONFIG_MTK_SCHED_CMP
		env.loop_max	= min_t(unsigned long, sysctl_sched_nr_migrate, busiest->nr_running);
		mt_sched_printf("1 env.loop_max=%d, busiest->nr_running=%d src=%d, dst=%d, cpus_share_cache=%d",
			env.loop_max, busiest->nr_running, env.src_cpu, env.dst_cpu, cpus_share_cache(env.src_cpu, env.dst_cpu));
#endif 
#ifdef CONFIG_MTK_SCHED_CMP
		if (!cpus_share_cache(env.src_cpu, env.dst_cpu))
			cur_ld_moved = cmp_move_tasks(sd, &env);
		else
			cur_ld_moved = move_tasks(&env);
#else 
		cur_ld_moved = move_tasks(&env);
#endif 
		ld_moved += cur_ld_moved;
		double_rq_unlock(env.dst_rq, busiest);
		local_irq_restore(flags);

		if (cur_ld_moved && env.dst_cpu != smp_processor_id())
			resched_cpu(env.dst_cpu);

		if (env.flags & LBF_NEED_BREAK) {
			env.flags &= ~LBF_NEED_BREAK;
			goto more_balance;
		}

		if ((env.flags & LBF_SOME_PINNED) && env.imbalance > 0) {

			env.dst_rq	 = cpu_rq(env.new_dst_cpu);
			env.dst_cpu	 = env.new_dst_cpu;
			env.flags	&= ~LBF_SOME_PINNED;
			env.loop	 = 0;
			env.loop_break	 = sched_nr_migrate_break;

			
			cpumask_clear_cpu(env.dst_cpu, env.cpus);

			goto more_balance;
		}

		
		if (unlikely(env.flags & LBF_ALL_PINNED)) {
			mt_lbprof_update_state(busiest->cpu, MT_LBPROF_ALLPINNED);
			cpumask_clear_cpu(cpu_of(busiest), cpus);
			if (!cpumask_empty(cpus)) {
				env.loop = 0;
				env.loop_break = sched_nr_migrate_break;
				goto redo;
			}
			goto out_balanced;
		}
		
#ifdef CONFIG_MT_LOAD_BALANCE_ENHANCEMENT
		
		
		if (!ld_moved && ((CPU_NEWLY_IDLE == idle) || (CPU_IDLE == idle) ) ) {
#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
			mt_lbprof_stat_set(env.fail_reason, MT_LBPROF_DO_LB);
#endif
			env.mt_check_cache_in_idle = 0;	
			env.loop = 0;
			local_irq_save(flags);
			double_rq_lock(env.dst_rq, busiest);
#ifdef CONFIG_MTK_SCHED_CMP
			env.loop_max	= min_t(unsigned long, sysctl_sched_nr_migrate, busiest->nr_running);
			mt_sched_printf("2 env.loop_max=%d, busiest->nr_running=%d",
			env.loop_max, busiest->nr_running);
#endif 
			if (!env.loop)
				update_h_load(env.src_cpu);				
#ifdef CONFIG_MTK_SCHED_CMP_TGS
			if (!cpus_share_cache(env.src_cpu, env.dst_cpu)) 
				ld_moved = cmp_move_tasks(sd, &env);
			else{
				ld_moved = move_tasks(&env);
			}
#else 
			ld_moved = move_tasks(&env);
#endif 
			double_rq_unlock(env.dst_rq, busiest);
			local_irq_restore(flags);

			if (ld_moved && this_cpu != smp_processor_id())
				resched_cpu(this_cpu);			
		}
#endif		
	}

	if (!ld_moved) {
		schedstat_inc(sd, lb_failed[idle]);
		mt_lbprof_stat_or(env.fail_reason, MT_LBPROF_FAILED);
		if ( mt_lbprof_test(env.fail_reason, MT_LBPROF_AFFINITY) ) {
			mt_lbprof_update_state(busiest->cpu, MT_LBPROF_FAILURE_STATE);
		}else if ( mt_lbprof_test(env.fail_reason, MT_LBPROF_CACHEHOT) ) {
			mt_lbprof_update_state(busiest->cpu, MT_LBPROF_FAILURE_STATE);
		}

		if (idle != CPU_NEWLY_IDLE)
			sd->nr_balance_failed++;
		mt_lbprof_stat_inc(sd, mt_lbprof_nr_balance_failed);

		if (need_active_balance(&env)) {
			raw_spin_lock_irqsave(&busiest->lock, flags);

			if (!cpumask_test_cpu(this_cpu,
					tsk_cpus_allowed(busiest->curr))) {
				raw_spin_unlock_irqrestore(&busiest->lock,
							    flags);
				env.flags |= LBF_ALL_PINNED;
				goto out_one_pinned;
			}

			if (!busiest->active_balance) {
				busiest->active_balance = 1;
				busiest->push_cpu = this_cpu;
				active_balance = 1;
			}
			raw_spin_unlock_irqrestore(&busiest->lock, flags);

			if (active_balance) {
				stop_one_cpu_nowait(cpu_of(busiest),
					active_load_balance_cpu_stop, busiest,
					&busiest->active_balance_work);
			}

			sd->nr_balance_failed = sd->cache_nice_tries+1;
		}
	} else
		sd->nr_balance_failed = 0;

	if (likely(!active_balance)) {
		
		sd->balance_interval = sd->min_interval;
	} else {
		if (sd->balance_interval < sd->max_interval)
			sd->balance_interval *= 2;
	}

	goto out;

out_balanced:
	schedstat_inc(sd, lb_balanced[idle]);

	sd->nr_balance_failed = 0;
	mt_lbprof_stat_set(sd->mt_lbprof_nr_balance_failed, 0);

out_one_pinned:
	
	if (((env.flags & LBF_ALL_PINNED) &&
			sd->balance_interval < MAX_PINNED_INTERVAL) ||
			(sd->balance_interval < sd->max_interval))
		sd->balance_interval *= 2;

	ld_moved = 0;
out:
	if (ld_moved){
		mt_lbprof_stat_or(env.fail_reason, MT_LBPROF_SUCCESS);
		mt_lbprof_stat_set(sd->mt_lbprof_nr_balance_failed, 0);
	}	

#ifdef CONFIG_MT_LOAD_BALANCE_PROFILER
	if( CPU_NEWLY_IDLE == idle){
		char strings[128]="";
		snprintf(strings, 128, "%d:idle balance:%d:0x%x ", this_cpu, ld_moved, env.fail_reason);
		mt_lbprof_rqinfo(strings);
		trace_sched_lbprof_log(strings);
	}else{
		char strings[128]="";
		snprintf(strings, 128, "%d:periodic balance:%d:0x%x ", this_cpu, ld_moved, env.fail_reason);
		mt_lbprof_rqinfo(strings);
		trace_sched_lbprof_log(strings);
	}
#endif

	return ld_moved;
}

void idle_balance(int this_cpu, struct rq *this_rq)
{
	struct sched_domain *sd;
	int pulled_task = 0;
	unsigned long next_balance = jiffies + HZ;
#if (defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT) || defined(CONFIG_MT_LOAD_BALANCE_PROFILER)) && defined(CONFIG_LOCAL_TIMERS)
	unsigned long counter = 0;
#endif

	this_rq->idle_stamp = this_rq->clock;

	mt_lbprof_update_state_has_lock(this_cpu, MT_LBPROF_UPDATE_STATE);
#if defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT) && defined(CONFIG_LOCAL_TIMERS)
	counter = localtimer_get_counter();
	if ( counter >= 260000 )  
		goto must_do;
	if ( time_before(jiffies + 2, this_rq->next_balance) )	
		goto must_do;
#endif

	if (this_rq->avg_idle < sysctl_sched_migration_cost){
#if defined(CONFIG_MT_LOAD_BALANCE_PROFILER)
		char strings[128]="";
		mt_lbprof_update_state_has_lock(this_cpu, MT_LBPROF_ALLOW_UNBLANCE_STATE);
	#ifdef CONFIG_LOCAL_TIMERS
		snprintf(strings, 128, "%d:idle balance bypass: %llu %lu ", this_cpu, this_rq->avg_idle, counter);
	#else
		snprintf(strings, 128, "%d:idle balance bypass: %llu ", this_cpu, this_rq->avg_idle);
	#endif
		mt_lbprof_rqinfo(strings);
		trace_sched_lbprof_log(strings);
#endif		
		return;
	}

#if defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT) && defined(CONFIG_LOCAL_TIMERS)
	must_do:
#endif

	raw_spin_unlock(&this_rq->lock);

	mt_lbprof_update_status();
	update_blocked_averages(this_cpu);
	rcu_read_lock();
	for_each_domain(this_cpu, sd) {
		unsigned long interval;
		int balance = 1;

		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		if (sd->flags & SD_BALANCE_NEWIDLE) {
			
			pulled_task = load_balance(this_cpu, this_rq,
						   sd, CPU_NEWLY_IDLE, &balance);
		}

		interval = msecs_to_jiffies(sd->balance_interval);
		if (time_after(next_balance, sd->last_balance + interval))
			next_balance = sd->last_balance + interval;
		if (pulled_task) {
			this_rq->idle_stamp = 0;
			break;
		}
	}
	rcu_read_unlock();

	raw_spin_lock(&this_rq->lock);

	if (pulled_task || time_after(jiffies, this_rq->next_balance)) {
		this_rq->next_balance = next_balance;
	}
}

static int active_load_balance_cpu_stop(void *data)
{
	struct rq *busiest_rq = data;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd;

	raw_spin_lock_irq(&busiest_rq->lock);

	
	if (unlikely(busiest_cpu != smp_processor_id() ||
		     !busiest_rq->active_balance))
		goto out_unlock;

	
	if (busiest_rq->nr_running <= 1)
		goto out_unlock;

	BUG_ON(busiest_rq == target_rq);

	
	double_lock_balance(busiest_rq, target_rq);

	
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if ((sd->flags & SD_LOAD_BALANCE) &&
		    cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
				break;
	}

	if (likely(sd)) {
		struct lb_env env = {
			.sd		= sd,
			.dst_cpu	= target_cpu,
			.dst_rq		= target_rq,
			.src_cpu	= busiest_rq->cpu,
			.src_rq		= busiest_rq,
			.idle		= CPU_IDLE,
		};

		schedstat_inc(sd, alb_count);

		if (move_one_task(&env))
			schedstat_inc(sd, alb_pushed);
		else
			schedstat_inc(sd, alb_failed);
	}
	rcu_read_unlock();
	double_unlock_balance(busiest_rq, target_rq);
out_unlock:
	busiest_rq->active_balance = 0;
	raw_spin_unlock_irq(&busiest_rq->lock);
	return 0;
}

#ifdef CONFIG_NO_HZ_COMMON
static struct {
	cpumask_var_t idle_cpus_mask;
	atomic_t nr_cpus;
	unsigned long next_balance;     
} nohz ____cacheline_aligned;


static inline int find_new_ilb(int call_cpu)
{
#ifdef CONFIG_HMP_PACK_SMALL_TASK 

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER

	struct sched_domain *sd;
	
	int ilb_new = nr_cpu_ids;
	
	int ilb_return = 0;
	
	int ilb = cpumask_first(nohz.idle_cpus_mask);

	
	if(PA_ENABLE)
	{
		int buddy = per_cpu(sd_pack_buddy, call_cpu);

		if (buddy != -1)
			for_each_domain(buddy, sd) {
				if (sd->flags & SD_SHARE_CPUPOWER)
					continue;

				ilb_new = cpumask_first_and(sched_domain_span(sd),
						nohz.idle_cpus_mask);

				if (ilb_new < nr_cpu_ids)
					break;
									
			}
	}

	if (ilb < nr_cpu_ids && idle_cpu(ilb)) {
		ilb_return = 1;
	}

	if (ilb_new < nr_cpu_ids) {
		if (idle_cpu(ilb_new)) {
			if(PA_ENABLE && ilb_return && ilb_new != ilb) {	
				AVOID_WAKE_UP_FROM_CPUX_TO_CPUY_COUNT[call_cpu][ilb]++;

#ifdef CONFIG_HMP_TRACER
				trace_sched_power_aware_active(POWER_AWARE_ACTIVE_MODULE_AVOID_WAKE_UP_FORM_CPUX_TO_CPUY, 0, call_cpu, ilb);
#endif 

			}
			return ilb_new;			
		}
	}
	
	if(ilb_return) {
		return ilb;
	}
			
	return nr_cpu_ids;
  
#else   	

	struct sched_domain *sd;
	int ilb = cpumask_first(nohz.idle_cpus_mask);
	int buddy = per_cpu(sd_pack_buddy, call_cpu);

	if (buddy != -1)
		for_each_domain(buddy, sd) {
			if (sd->flags & SD_SHARE_CPUPOWER)
				continue;

			ilb = cpumask_first_and(sched_domain_span(sd),
					nohz.idle_cpus_mask);

			if (ilb < nr_cpu_ids)
				break;
		}

	if (ilb < nr_cpu_ids && idle_cpu(ilb))
		return ilb;

	return nr_cpu_ids;

#endif  	

#else  

	int ilb = cpumask_first(nohz.idle_cpus_mask);
#ifdef CONFIG_MTK_SCHED_CMP_TGS
	
	int new_ilb;
	struct cpumask tmp;
	
	get_cluster_cpus(&tmp, get_cluster_id(call_cpu), true);
	new_ilb = cpumask_first_and(nohz.idle_cpus_mask, &tmp);
	if (new_ilb < nr_cpu_ids && idle_cpu(new_ilb))
	{
#ifdef CONFIG_MTK_SCHED_CMP_POWER_AWARE_CONTROLLER
		if(new_ilb != ilb)
		{
			mt_sched_printf("[PA]find_new_ilb(cpu%x), new_ilb = %d, ilb = %d\n", call_cpu, new_ilb, ilb);
			AVOID_WAKE_UP_FROM_CPUX_TO_CPUY_COUNT[call_cpu][ilb]++;
		}
#endif
		return new_ilb;
	}
#endif 

	if (ilb < nr_cpu_ids && idle_cpu(ilb))
		return ilb;

	return nr_cpu_ids;

#endif 

}


static void nohz_balancer_kick(int cpu)
{
	int ilb_cpu;

	nohz.next_balance++;

	ilb_cpu = find_new_ilb(cpu);

	if (ilb_cpu >= nr_cpu_ids)
		return;

	if (test_and_set_bit(NOHZ_BALANCE_KICK, nohz_flags(ilb_cpu)))
		return;
	smp_send_reschedule(ilb_cpu);
	return;
}

static inline void nohz_balance_exit_idle(int cpu)
{
	if (unlikely(test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))) {
		cpumask_clear_cpu(cpu, nohz.idle_cpus_mask);
		atomic_dec(&nohz.nr_cpus);
		clear_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
	}
}

static inline void set_cpu_sd_state_busy(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	rcu_read_lock();
	sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd);

	if (!sd || !sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 0;

	for (; sd; sd = sd->parent)
		atomic_inc(&sd->groups->sgp->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

void set_cpu_sd_state_idle(void)
{
	struct sched_domain *sd;
	int cpu = smp_processor_id();

	rcu_read_lock();
	sd = rcu_dereference_check_sched_domain(cpu_rq(cpu)->sd);

	if (!sd || sd->nohz_idle)
		goto unlock;
	sd->nohz_idle = 1;

	for (; sd; sd = sd->parent)
		atomic_dec(&sd->groups->sgp->nr_busy_cpus);
unlock:
	rcu_read_unlock();
}

void nohz_balance_enter_idle(int cpu)
{
	if (!cpu_active(cpu))
		return;

	if (test_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu)))
		return;

	cpumask_set_cpu(cpu, nohz.idle_cpus_mask);
	atomic_inc(&nohz.nr_cpus);
	set_bit(NOHZ_TICK_STOPPED, nohz_flags(cpu));
}

static int __cpuinit sched_ilb_notifier(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DYING:
		nohz_balance_exit_idle(smp_processor_id());
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}
#endif

static DEFINE_SPINLOCK(balancing);

void update_max_interval(void)
{
	max_load_balance_interval = HZ*num_online_cpus()/10;
}

static void rebalance_domains(int cpu, enum cpu_idle_type idle)
{
	int balance = 1;
	struct rq *rq = cpu_rq(cpu);
	unsigned long interval;
	struct sched_domain *sd;
	
	unsigned long next_balance = jiffies + 60*HZ;
	int update_next_balance = 0;
	int need_serialize;

	update_blocked_averages(cpu);

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (!(sd->flags & SD_LOAD_BALANCE))
			continue;

		interval = sd->balance_interval;
		if (idle != CPU_IDLE)
			interval *= sd->busy_factor;

		
		interval = msecs_to_jiffies(interval);
		interval = clamp(interval, 1UL, max_load_balance_interval);

		need_serialize = sd->flags & SD_SERIALIZE;

		if (need_serialize) {
			if (!spin_trylock(&balancing))
				goto out;
		}

		if (time_after_eq(jiffies, sd->last_balance + interval)) {
			if (load_balance(cpu, rq, sd, idle, &balance)) {
				idle = idle_cpu(cpu) ? CPU_IDLE : CPU_NOT_IDLE;
			}
			sd->last_balance = jiffies;
		}
		if (need_serialize)
			spin_unlock(&balancing);
out:
		if (time_after(next_balance, sd->last_balance + interval)) {
			next_balance = sd->last_balance + interval;
			update_next_balance = 1;
		}

		if (!balance)
			break;
	}
	rcu_read_unlock();

	if (likely(update_next_balance))
		rq->next_balance = next_balance;
}

#ifdef CONFIG_NO_HZ_COMMON
static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle)
{
	struct rq *this_rq = cpu_rq(this_cpu);
	struct rq *rq;
	int balance_cpu;

	if (idle != CPU_IDLE ||
	    !test_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu)))
		goto end;

	for_each_cpu(balance_cpu, nohz.idle_cpus_mask) {
		if (balance_cpu == this_cpu || !idle_cpu(balance_cpu))
			continue;

		if (need_resched())
			break;

		rq = cpu_rq(balance_cpu);

		raw_spin_lock_irq(&rq->lock);
		update_rq_clock(rq);
		update_idle_cpu_load(rq);
		raw_spin_unlock_irq(&rq->lock);

		rebalance_domains(balance_cpu, CPU_IDLE);

		if (time_after(this_rq->next_balance, rq->next_balance))
			this_rq->next_balance = rq->next_balance;
	}
	nohz.next_balance = this_rq->next_balance;
end:
	clear_bit(NOHZ_BALANCE_KICK, nohz_flags(this_cpu));
}

static inline int nohz_kick_needed(struct rq *rq, int cpu)
{
	unsigned long now = jiffies;
	struct sched_domain *sd;

	if (unlikely(idle_cpu(cpu)))
		return 0;

	set_cpu_sd_state_busy();
	nohz_balance_exit_idle(cpu);

	if (likely(!atomic_read(&nohz.nr_cpus)))
		return 0;

	if (time_before(now, nohz.next_balance))
		return 0;

#ifdef CONFIG_SCHED_HMP
	if (cpumask_first_and(nohz.idle_cpus_mask,
			&((struct hmp_domain *)hmp_cpu_domain(cpu))->cpus) >= nr_cpu_ids)
		return 0;
#endif

	if (rq->nr_running >= 2)
		goto need_kick;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		struct sched_group *sg = sd->groups;
		struct sched_group_power *sgp = sg->sgp;
		int nr_busy = atomic_read(&sgp->nr_busy_cpus);

		if (sd->flags & SD_SHARE_PKG_RESOURCES && nr_busy > 1)
			goto need_kick_unlock;

		if (sd->flags & SD_ASYM_PACKING && nr_busy != sg->group_weight
		    && (cpumask_first_and(nohz.idle_cpus_mask,
					  sched_domain_span(sd)) < cpu))
			goto need_kick_unlock;

		if (!(sd->flags & (SD_SHARE_PKG_RESOURCES | SD_ASYM_PACKING)))
			break;
	}
	rcu_read_unlock();
	return 0;

need_kick_unlock:
	rcu_read_unlock();
need_kick:
	return 1;
}
#else
static void nohz_idle_balance(int this_cpu, enum cpu_idle_type idle) { }
#endif

#ifdef CONFIG_SCHED_HMP
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT


static int hmp_up_stable(int cpu);
static int hmp_down_stable(int cpu);
static unsigned int hmp_up_migration(int cpu, int *target_cpu, struct sched_entity *se,
                    struct clb_env *clbenv);
static unsigned int hmp_down_migration(int cpu, int *target_cpu, struct sched_entity *se,
                    struct clb_env *clbenv);

#define hmp_caller_is_gb(caller) ((HMP_GB == caller)?1:0)

#define hmp_cpu_is_fast(cpu) cpumask_test_cpu(cpu,&hmp_fast_cpu_mask)
#define hmp_cpu_is_slow(cpu) cpumask_test_cpu(cpu,&hmp_slow_cpu_mask)
#define hmp_cpu_stable(cpu) (hmp_cpu_is_fast(cpu)? \
			hmp_up_stable(cpu):hmp_down_stable(cpu))

#define hmp_inc(v) ((v) + 1)
#define hmp_dec(v) ((v) - 1)
#define hmp_pos(v) ((v) < (0) ? (0) : (v))

#define task_created(f) ((SD_BALANCE_EXEC == f || SD_BALANCE_FORK == f)?1:0)
#define task_cpus_allowed(mask,p) cpumask_intersects(mask,tsk_cpus_allowed(p))
#define task_slow_cpu_allowed(p) task_cpus_allowed(&hmp_slow_cpu_mask,p)
#define task_fast_cpu_allowed(p) task_cpus_allowed(&hmp_fast_cpu_mask,p)


static int hmp_up_stable(int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now = cfs_rq_clock_task(cfs_rq);
	if (((now - hmp_last_up_migration(cpu)) >> 10) < hmp_next_up_threshold)
		return 0;
	return 1;
}

static int hmp_down_stable(int cpu)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now = cfs_rq_clock_task(cfs_rq);
	if (((now - hmp_last_down_migration(cpu)) >> 10) < hmp_next_down_threshold)
		return 0;
	return 1;
}

static unsigned int hmp_select_cpu(unsigned int caller, struct task_struct *p,
			struct cpumask *mask, int prev)
{
	int curr = 0;
	int target = NR_CPUS;
	unsigned long curr_wload = 0;
	unsigned long target_wload = 0;
	struct cpumask srcp;
	cpumask_and(&srcp, cpu_online_mask, mask);
	target = cpumask_any_and(&srcp, tsk_cpus_allowed(p));
	if (NR_CPUS == target)
		goto out;
		
	target_wload = hmp_inc(cfs_load(target));
	target_wload += cfs_pending_load(target);
	target_wload *= rq_length(target);
	for_each_cpu(curr, mask) {
		
		if(!cpu_online(curr) || !cpumask_test_cpu(curr, tsk_cpus_allowed(p)))
			continue;

		
		if(hmp_caller_is_gb(caller) && !hmp_cpu_stable(curr))
			continue;

		curr_wload = hmp_inc(cfs_load(curr));
		curr_wload += cfs_pending_load(curr);
		curr_wload *= rq_length(curr);
		if(curr_wload < target_wload) {
			target_wload = curr_wload;
			target = curr;
		} else if(curr_wload == target_wload && curr == prev) {
			target = curr;
		}
	}

out:
	return target;
}

 
static int hmp_select_task_rq_fair(int sd_flag, struct task_struct *p,
			int prev_cpu, int new_cpu) 
{
#ifdef CONFIG_HMP_TASK_ASSIGNMENT
	int step = 0;
	struct sched_entity *se = &p->se;
	int B_target = NR_CPUS;
	int L_target = NR_CPUS;
	struct clb_env clbenv;
 
#ifdef CONFIG_HMP_TRACER
	int cpu = 0;
	for_each_online_cpu(cpu)
		trace_sched_cfs_runnable_load(cpu,cfs_load(cpu),cfs_length(cpu));
#endif

	
	if (prev_cpu >= NR_CPUS)
		return new_cpu;

	if (num_online_cpus() == 1)
		goto out;
	B_target = hmp_select_cpu(HMP_SELECT_RQ,p,&hmp_fast_cpu_mask,prev_cpu);
	L_target = hmp_select_cpu(HMP_SELECT_RQ,p,&hmp_slow_cpu_mask,prev_cpu);

#ifdef CONFIG_HMP_DISCARD_CFS_SELECTION_RESULT
	if(NR_CPUS == B_target && NR_CPUS == L_target)
		goto out;
	if(NR_CPUS == B_target)
		goto select_slow;
	if(NR_CPUS == L_target)
		goto select_fast;
#else
	if(NR_CPUS == B_target || NR_CPUS == L_target)
		goto out;
#endif

	step = 1;
	if (task_created(sd_flag) && !task_low_priority(p->prio)) {
		if (!rq_length(B_target))
			goto select_fast;
		if (!rq_length(L_target))
			goto select_slow;
	}
	memset(&clbenv, 0, sizeof(clbenv));
	clbenv.flags |= HMP_SELECT_RQ;
	clbenv.lcpus = &hmp_slow_cpu_mask;
	clbenv.bcpus = &hmp_fast_cpu_mask;
	clbenv.ltarget = L_target;
	clbenv.btarget = B_target;
	sched_update_clbstats(&clbenv);
	step = 2;
	if (hmp_up_migration(L_target, &B_target, se, &clbenv))
		goto select_fast;
	step = 3;
	if (hmp_down_migration(B_target, &L_target, se, &clbenv))
		goto select_slow;
	step = 4;
	if (hmp_cpu_is_slow(prev_cpu))
		goto select_slow;
	goto select_fast;

select_fast:
	new_cpu = B_target;
	goto out;
select_slow:
	new_cpu = L_target;
	goto out;

out:

	
	if (new_cpu >= nr_cpu_ids)
	{
		
		new_cpu = prev_cpu;
	}

	cfs_nr_pending(new_cpu)++;
	cfs_pending_load(new_cpu) += se_load(se);
#ifdef CONFIG_HMP_TRACER
	trace_sched_hmp_load(clbenv.bstats.load_avg, clbenv.lstats.load_avg);
	trace_sched_hmp_select_task_rq(p,step,sd_flag,prev_cpu,new_cpu,
			se_load(se),&clbenv.bstats,&clbenv.lstats);
#endif
#ifdef CONFIG_MET_SCHED_HMP
	HmpLoad(clbenv.bstats.load_avg, clbenv.lstats.load_avg);
#endif
#endif 
	return new_cpu;
}




#define HMP_RATIO(v) ((v)*17/10)

#define hmp_fast_cpu_has_spare_cycles(B,cpu_load) (cpu_load < \
			(HMP_RATIO(B->cpu_capacity) - (B->cpu_capacity >> 2)))

#define hmp_task_fast_cpu_afford(B,se,cpu) (B->acap > 0 \
			&& hmp_fast_cpu_has_spare_cycles(B,se_load(se) + cfs_load(cpu)))

#define hmp_fast_cpu_oversubscribed(caller,B,se,cpu) \
			(hmp_caller_is_gb(caller)? \
			!hmp_fast_cpu_has_spare_cycles(B,cfs_load(cpu)): \
			!hmp_task_fast_cpu_afford(B,se,cpu))

#define hmp_task_slow_cpu_afford(L,se) \
			(L->acap > 0 && L->acap >= se_load(se))

#define hmp_low_prio_task_up_rejected(p,B,L) \
			(task_low_priority(p->prio) && \
			(B->ntask >= B->ncpu || 0 != L->nr_normal_prio_task) && \
			 (p->se.avg.load_avg_ratio < 800))

#define hmp_low_prio_task_down_allowed(p,B,L) \
			(task_low_priority(p->prio) && !B->nr_dequeuing_low_prio && \
			B->ntask >= B->ncpu && 0 != L->nr_normal_prio_task && \
			(p->se.avg.load_avg_ratio < 800))

#define HMP_BIG_NOT_OVERSUBSCRIBED           (0x01)
#define HMP_BIG_CAPACITY_INSUFFICIENT        (0x02)
#define HMP_LITTLE_CAPACITY_INSUFFICIENT     (0x04)
#define HMP_LOW_PRIORITY_FILTER              (0x08)
#define HMP_BIG_BUSY_LITTLE_IDLE             (0x10)
#define HMP_BIG_IDLE                         (0x20)
#define HMP_MIGRATION_APPROVED              (0x100)
#define HMP_TASK_UP_MIGRATION               (0x200)
#define HMP_TASK_DOWN_MIGRATION             (0x400)

#ifdef CONFIG_HMP_TRACER
struct hmp_statisic hmp_stats;
#endif

static inline void hmp_dynamic_threshold(struct clb_env *clbenv)
{
	struct clb_stats *L = &clbenv->lstats;
	struct clb_stats *B = &clbenv->bstats;
	unsigned int hmp_threshold_diff = hmp_up_threshold - hmp_down_threshold;
	unsigned int B_normalized_acap = hmp_pos(HMP_RATIO(B->scaled_acap));
	unsigned int B_normalized_atask = hmp_pos(HMP_RATIO(B->scaled_atask));
	unsigned int L_normalized_acap = hmp_pos(L->scaled_acap);
	unsigned int L_normalized_atask = hmp_pos(L->scaled_atask);

#ifdef CONFIG_HMP_DYNAMIC_THRESHOLD
	L->threshold = hmp_threshold_diff;
	L->threshold *= hmp_inc(L_normalized_acap) * hmp_inc(L_normalized_atask);
	L->threshold /= hmp_inc(B_normalized_acap + L_normalized_acap);
	L->threshold /= hmp_inc(B_normalized_atask + L_normalized_atask);
	L->threshold = hmp_down_threshold + L->threshold;

	B->threshold = hmp_threshold_diff;
	B->threshold *= hmp_inc(B_normalized_acap) * hmp_inc(B_normalized_atask);
	B->threshold /= hmp_inc(B_normalized_acap + L_normalized_acap);
	B->threshold /= hmp_inc(B_normalized_atask + L_normalized_atask);
	B->threshold = hmp_up_threshold - B->threshold;
#else 
	clbenv->lstats.threshold = hmp_down_threshold; 
	clbenv->bstats.threshold = hmp_up_threshold; 
#endif 

	mt_sched_printf("[%s]\tup/dl:%4d/%4d bcpu(%d):%d/%d, lcpu(%d):%d/%d\n", __func__,
					B->threshold, L->threshold,
					clbenv->btarget, clbenv->bstats.cpu_capacity, clbenv->bstats.cpu_power,
					clbenv->ltarget, clbenv->lstats.cpu_capacity, clbenv->lstats.cpu_power);
}

static unsigned int hmp_up_migration(int cpu, int *target_cpu, struct sched_entity *se,
                                     struct clb_env *clbenv)
{
	struct task_struct *p = task_of(se);
	struct clb_stats *L, *B;
	struct mcheck *check;
	int curr_cpu = cpu;
	unsigned int caller = clbenv->flags;

	L = &clbenv->lstats;
	B = &clbenv->bstats;
	check = &clbenv->mcheck;

	check->status = clbenv->flags;
	check->status |= HMP_TASK_UP_MIGRATION;
	check->result = 0;
	
	if (!L->ncpu || !B->ncpu
		|| cpumask_test_cpu(curr_cpu, clbenv->bcpus)
		|| !cpumask_intersects(clbenv->bcpus, tsk_cpus_allowed(p)))
		goto out;

	if (!hmp_up_stable(*target_cpu))
		goto out;

	
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	if (hmp_low_prio_task_up_rejected(p,B,L)) {
		check->status |= HMP_LOW_PRIORITY_FILTER;	
		goto trace;
	}
#endif

	
	if (rq_length(*target_cpu)==0)
	{
		check->status |= HMP_BIG_IDLE;
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
		goto trace;
	}

	if (!hmp_task_fast_cpu_afford(B,se,*target_cpu)) {
		check->status |= HMP_BIG_CAPACITY_INSUFFICIENT;
		goto trace;
	}

	if (se_load(se) > B->threshold) {
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
	}

trace:
#ifdef CONFIG_HMP_TRACER
	if(check->result && hmp_caller_is_gb(caller))
		hmp_stats.nr_force_up++;
	trace_sched_hmp_stats(&hmp_stats);
	trace_sched_dynamic_threshold(task_of(se),B->threshold,check->status,
			curr_cpu,*target_cpu,se_load(se),B,L);
#endif
#ifdef CONFIG_MET_SCHED_HMP
	TaskTh(B->threshold,L->threshold);
	HmpStat(&hmp_stats);
#endif
out:
	return check->result;
}

static unsigned int hmp_down_migration(int cpu, int *target_cpu, struct sched_entity *se,
                                       struct clb_env *clbenv)
{
	struct task_struct *p = task_of(se);
	struct clb_stats *L, *B;
	struct mcheck *check;
	int curr_cpu = cpu;
	unsigned int caller = clbenv->flags;

	L = &clbenv->lstats;
	B = &clbenv->bstats;
	check = &clbenv->mcheck;

	check->status = caller;
	check->status |= HMP_TASK_DOWN_MIGRATION;
	check->result = 0;

	if (!L->ncpu || !B->ncpu
		|| cpumask_test_cpu(curr_cpu, clbenv->lcpus)
		|| !cpumask_intersects(clbenv->lcpus, tsk_cpus_allowed(p)))
		goto out;

	if (!hmp_down_stable(*target_cpu))
		goto out;

	
	if (rq_length(*target_cpu)==0 && caller == HMP_SELECT_RQ && rq_length(curr_cpu)>0)
	{
		check->status |= HMP_BIG_BUSY_LITTLE_IDLE;
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
		goto trace;
	}

	
#ifdef CONFIG_SCHED_HMP_PRIO_FILTER	
	if (hmp_low_prio_task_down_allowed(p,B,L)) {
		cfs_nr_dequeuing_low_prio(curr_cpu)++;
		check->status |= HMP_LOW_PRIORITY_FILTER;
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
		goto trace;
	}
#endif

	if (!hmp_fast_cpu_oversubscribed(caller,B,se,curr_cpu)) {
		check->status |= HMP_BIG_NOT_OVERSUBSCRIBED;
		goto trace;
	}

 	if (!hmp_task_slow_cpu_afford(L,se)) {
  		check->status |= HMP_LITTLE_CAPACITY_INSUFFICIENT;
  		goto trace;
	}

		 
	if (L->threshold >= se_load(se)) {
		check->status |= HMP_MIGRATION_APPROVED;
		check->result = 1;
	}

trace:
#ifdef CONFIG_HMP_TRACER
	if (check->result && hmp_caller_is_gb(caller))
		hmp_stats.nr_force_down++;
	trace_sched_hmp_stats(&hmp_stats);
	trace_sched_dynamic_threshold(task_of(se),L->threshold,check->status,
			curr_cpu,*target_cpu,se_load(se),B,L);
#endif
#ifdef CONFIG_MET_SCHED_HMP
	TaskTh(B->threshold,L->threshold);
	HmpStat(&hmp_stats);
#endif
out:
	return check->result;
}
#else 
static unsigned int hmp_up_migration(int cpu, int *target_cpu, struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now;

	if (target_cpu)
		*target_cpu = NR_CPUS;

	if (hmp_cpu_is_fastest(cpu))
		return 0;

#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	
	if (p->prio >= hmp_up_prio)
		return 0;
#endif
	if (se->avg.load_avg_ratio < hmp_up_threshold)
		return 0;

	
	now = cfs_rq_clock_task(cfs_rq);
	if (((now - se->avg.hmp_last_up_migration) >> 10)
					< hmp_next_up_threshold)
		return 0;

	
	if (hmp_domain_min_load(hmp_faster_domain(cpu), target_cpu)
			> NICE_0_LOAD-64)
		return 0;

	if (cpumask_intersects(&hmp_faster_domain(cpu)->cpus,
			tsk_cpus_allowed(p)))
		return 1;

	return 0;
}

static unsigned int hmp_down_migration(int cpu, struct sched_entity *se)
{
	struct task_struct *p = task_of(se);
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	u64 now;

	if (hmp_cpu_is_slowest(cpu))
		return 0;

#ifdef CONFIG_SCHED_HMP_PRIO_FILTER
	
	if ((p->prio >= hmp_up_prio) &&
		cpumask_intersects(&hmp_slower_domain(cpu)->cpus,
					tsk_cpus_allowed(p))) {
		return 1;
	}
#endif

	
	now = cfs_rq_clock_task(cfs_rq);
	if (((now - se->avg.hmp_last_down_migration) >> 10)
					< hmp_next_down_threshold)
		return 0;

	if (cpumask_intersects(&hmp_slower_domain(cpu)->cpus,
					tsk_cpus_allowed(p))
		&& se->avg.load_avg_ratio < hmp_down_threshold) {
		return 1;
	}
	return 0;
}
#endif 

static int hmp_can_migrate_task(struct task_struct *p, struct lb_env *env)
{
	int tsk_cache_hot = 0;

	if (!cpumask_test_cpu(env->dst_cpu, tsk_cpus_allowed(p))) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_affine);
		return 0;
	}
	env->flags &= ~LBF_ALL_PINNED;

	if (task_running(env->src_rq, p)) {
		schedstat_inc(p, se.statistics.nr_failed_migrations_running);
		return 0;
	}


#if defined(CONFIG_MT_LOAD_BALANCE_ENHANCEMENT)
	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd, env->mt_check_cache_in_idle);
#else
	tsk_cache_hot = task_hot(p, env->src_rq->clock_task, env->sd);
#endif
	if (!tsk_cache_hot ||
		env->sd->nr_balance_failed > env->sd->cache_nice_tries) {
#ifdef CONFIG_SCHEDSTATS
		if (tsk_cache_hot) {
			schedstat_inc(env->sd, lb_hot_gained[env->idle]);
			schedstat_inc(p, se.statistics.nr_forced_migrations);
		}
#endif
		return 1;
	}

	return 1;
}

static int move_specific_task(struct lb_env *env, struct task_struct *pm)
{
	struct task_struct *p, *n;

	list_for_each_entry_safe(p, n, &env->src_rq->cfs_tasks, se.group_node) {
	if (throttled_lb_pair(task_group(p), env->src_rq->cpu,
				env->dst_cpu))
		continue;

		if (!hmp_can_migrate_task(p, env))
			continue;
		
		if (p != pm)
			continue;

		move_task(p, env);
		schedstat_inc(env->sd, lb_gained[env->idle]);
		return 1;
	}
	return 0;
}

static int hmp_active_task_migration_cpu_stop(void *data)
{
	struct rq *busiest_rq = data;
	struct task_struct *p = busiest_rq->migrate_task;
	int busiest_cpu = cpu_of(busiest_rq);
	int target_cpu = busiest_rq->push_cpu;
	struct rq *target_rq = cpu_rq(target_cpu);
	struct sched_domain *sd;

	raw_spin_lock_irq(&busiest_rq->lock);
	
	if (unlikely(busiest_cpu != smp_processor_id() ||
		!busiest_rq->active_balance)) {
		goto out_unlock;
	}
	
	if (busiest_rq->nr_running <= 1)
		goto out_unlock;
	
	if (task_rq(p) != busiest_rq)
		goto out_unlock;
	BUG_ON(busiest_rq == target_rq);

	
	double_lock_balance(busiest_rq, target_rq);

	
	rcu_read_lock();
	for_each_domain(target_cpu, sd) {
		if (cpumask_test_cpu(busiest_cpu, sched_domain_span(sd)))
			break;
	}

	if (likely(sd)) {
		struct lb_env env = {
			.sd		= sd,
			.dst_cpu	= target_cpu,
			.dst_rq		= target_rq,
			.src_cpu	= busiest_rq->cpu,
			.src_rq		= busiest_rq,
			.idle		= CPU_IDLE,
		};

		schedstat_inc(sd, alb_count);

		if (move_specific_task(&env, p))
			schedstat_inc(sd, alb_pushed);
		else
			schedstat_inc(sd, alb_failed);
	}
	rcu_read_unlock();
	double_unlock_balance(busiest_rq, target_rq);
out_unlock:
	busiest_rq->active_balance = 0;
	raw_spin_unlock_irq(&busiest_rq->lock);
	return 0;
}

static DEFINE_SPINLOCK(hmp_force_migration);
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT

#ifdef CONFIG_HMP_GLOBAL_BALANCE
static void hmp_force_down_migration(int this_cpu)
{
	int curr_cpu, target_cpu;
	struct sched_entity *se;
	struct rq *target;
	unsigned long flags;
	unsigned int force;
	struct task_struct *p;
	struct clb_env clbenv;

	
	for_each_cpu(curr_cpu, &hmp_fast_cpu_mask) {
		
		if(!cpu_online(curr_cpu))
			continue;

		force = 0;
		target = cpu_rq(curr_cpu);
		raw_spin_lock_irqsave(&target->lock, flags);
		se = target->cfs.curr;
		if (!se) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

		
		if (!entity_is_task(se)) {
			struct cfs_rq *cfs_rq;
			cfs_rq = group_cfs_rq(se);
			while (cfs_rq) {
				se = cfs_rq->curr;
				cfs_rq = group_cfs_rq(se);
			}
		}

		p = task_of(se);
		target_cpu = hmp_select_cpu(HMP_GB,p,&hmp_slow_cpu_mask,-1);
		if(NR_CPUS == target_cpu) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

		
		memset(&clbenv, 0, sizeof(clbenv));
		clbenv.flags |= HMP_GB;
		clbenv.btarget = curr_cpu;
		clbenv.ltarget = target_cpu;
		clbenv.lcpus = &hmp_slow_cpu_mask;
		clbenv.bcpus = &hmp_fast_cpu_mask;
		sched_update_clbstats(&clbenv);

		
		if (!target->active_balance && 
			hmp_down_migration(curr_cpu, &target_cpu, se, &clbenv)) {
			target->active_balance = 1;
			target->push_cpu = target_cpu;
			target->migrate_task = p;
			force = 1;
			trace_sched_hmp_migrate(p, target->push_cpu, 1);
			hmp_next_down_delay(&p->se, target->push_cpu);
		}
		raw_spin_unlock_irqrestore(&target->lock, flags);
		if (force) {
			stop_one_cpu_nowait(cpu_of(target),
				hmp_active_task_migration_cpu_stop,
				target, &target->active_balance_work);
		}
	}
}
#endif 
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER 
u32 AVOID_FORCE_UP_MIGRATION_FROM_CPUX_TO_CPUY_COUNT[NR_CPUS][NR_CPUS];
#endif 

static void hmp_force_up_migration(int this_cpu)
{
	int curr_cpu, target_cpu;
	struct sched_entity *se;
	struct rq *target;
	unsigned long flags;
	unsigned int force;
	struct task_struct *p;
	struct clb_env clbenv;
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER 
	int push_cpu;
#endif

	if (!spin_trylock(&hmp_force_migration))
		return;

#ifdef CONFIG_HMP_TRACER
	for_each_online_cpu(curr_cpu)
		trace_sched_cfs_runnable_load(curr_cpu,cfs_load(curr_cpu),
			cfs_length(curr_cpu));
#endif

	
	for_each_cpu(curr_cpu, &hmp_slow_cpu_mask) {
		
		if(!cpu_online(curr_cpu))
			continue;

		force = 0;
		target = cpu_rq(curr_cpu);
		raw_spin_lock_irqsave(&target->lock, flags);
		se = target->cfs.curr;
		if (!se) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

		
		if (!entity_is_task(se)) {
			struct cfs_rq *cfs_rq;
			cfs_rq = group_cfs_rq(se);
			while (cfs_rq) {
				se = cfs_rq->curr;
				cfs_rq = group_cfs_rq(se);
			}
		}
                
		p = task_of(se);
		target_cpu = hmp_select_cpu(HMP_GB,p,&hmp_fast_cpu_mask,-1);
		if(NR_CPUS == target_cpu) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}

			
		memset(&clbenv, 0, sizeof(clbenv));
		clbenv.flags |= HMP_GB;
		clbenv.ltarget = curr_cpu;
		clbenv.btarget = target_cpu;
		clbenv.lcpus = &hmp_slow_cpu_mask;
		clbenv.bcpus = &hmp_fast_cpu_mask;
		sched_update_clbstats(&clbenv);

#ifdef CONFIG_HMP_LAZY_BALANCE  
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
		if (PA_ENABLE && LB_ENABLE) {
#endif 
			if (is_light_task(p) && !is_buddy_busy(per_cpu(sd_pack_buddy, curr_cpu))) {
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER  
				push_cpu = hmp_select_cpu(HMP_GB,p,&hmp_fast_cpu_mask,-1);
				if (hmp_cpu_is_fast(push_cpu)) {
					AVOID_FORCE_UP_MIGRATION_FROM_CPUX_TO_CPUY_COUNT[curr_cpu][push_cpu]++;
#ifdef CONFIG_HMP_TRACER
					trace_sched_power_aware_active(POWER_AWARE_ACTIVE_MODULE_AVOID_FORCE_UP_FORM_CPUX_TO_CPUY, p->pid, curr_cpu, push_cpu);
#endif 
				}
#endif 
				goto out_force_up;
			}
#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER         
		}
#endif 
#endif 

		
		if (!target->active_balance &&
			hmp_up_migration(curr_cpu, &target_cpu, se, &clbenv)) {
			target->active_balance = 1;
			target->push_cpu = target_cpu;
			target->migrate_task = p;
			force = 1;
			trace_sched_hmp_migrate(p, target->push_cpu, 1);
			hmp_next_up_delay(&p->se, target->push_cpu);
		}

#ifdef CONFIG_HMP_LAZY_BALANCE
out_force_up:    
#endif 

		raw_spin_unlock_irqrestore(&target->lock, flags);
		if (force) {
			stop_one_cpu_nowait(cpu_of(target),
				hmp_active_task_migration_cpu_stop,
				target, &target->active_balance_work);
		}
	}

#ifdef CONFIG_HMP_GLOBAL_BALANCE
	hmp_force_down_migration(this_cpu);
#endif
#ifdef CONFIG_HMP_TRACER
	trace_sched_hmp_load(clbenv.bstats.load_avg, clbenv.lstats.load_avg);
#endif
	spin_unlock(&hmp_force_migration);
}
#else 
static void hmp_force_up_migration(int this_cpu)
{
	int cpu, target_cpu;
	struct sched_entity *curr;
	struct rq *target;
	unsigned long flags;
	unsigned int force;
	struct task_struct *p;

	if (!spin_trylock(&hmp_force_migration))
		return;
	for_each_online_cpu(cpu) {
		force = 0;
		target = cpu_rq(cpu);
		raw_spin_lock_irqsave(&target->lock, flags);
		curr = target->cfs.curr;
		if (!curr) {
			raw_spin_unlock_irqrestore(&target->lock, flags);
			continue;
		}
		if (!entity_is_task(curr)) {
			struct cfs_rq *cfs_rq;

			cfs_rq = group_cfs_rq(curr);
			while (cfs_rq) {
				curr = cfs_rq->curr;
				cfs_rq = group_cfs_rq(curr);
			}
		}
		p = task_of(curr);
		if (hmp_up_migration(cpu, &target_cpu, curr)) {
			if (!target->active_balance) {
				target->active_balance = 1;
				target->push_cpu = target_cpu;
				target->migrate_task = p;
				force = 1;
				trace_sched_hmp_migrate(p, target->push_cpu, 1);
				hmp_next_up_delay(&p->se, target->push_cpu);
			}
		}
		if (!force && !target->active_balance) {
			target->push_cpu = hmp_offload_down(cpu, curr);
			if (target->push_cpu < NR_CPUS) {
				target->active_balance = 1;
				target->migrate_task = p;
				force = 1;
				trace_sched_hmp_migrate(p, target->push_cpu, 2);
				hmp_next_down_delay(&p->se, target->push_cpu);
			}
		}
		raw_spin_unlock_irqrestore(&target->lock, flags);
		if (force)
			stop_one_cpu_nowait(cpu_of(target),
				hmp_active_task_migration_cpu_stop,
				target, &target->active_balance_work);
	}
	spin_unlock(&hmp_force_migration);
}
#endif 
#else
static void hmp_force_up_migration(int this_cpu) { }
#endif 

static void run_rebalance_domains(struct softirq_action *h)
{
	int this_cpu = smp_processor_id();
	struct rq *this_rq = cpu_rq(this_cpu);
	enum cpu_idle_type idle = this_rq->idle_balance ?
						CPU_IDLE : CPU_NOT_IDLE;

	hmp_force_up_migration(this_cpu);

	rebalance_domains(this_cpu, idle);

	nohz_idle_balance(this_cpu, idle);
}

static inline int on_null_domain(int cpu)
{
	return !rcu_dereference_sched(cpu_rq(cpu)->sd);
}

void trigger_load_balance(struct rq *rq, int cpu)
{
	
	if (time_after_eq(jiffies, rq->next_balance) &&
	    likely(!on_null_domain(cpu)))
		raise_softirq(SCHED_SOFTIRQ);
#ifdef CONFIG_NO_HZ_COMMON
	if (nohz_kick_needed(rq, cpu) && likely(!on_null_domain(cpu)))
		nohz_balancer_kick(cpu);
#endif
}

static void rq_online_fair(struct rq *rq)
{
#ifdef CONFIG_SCHED_HMP
	hmp_online_cpu(rq->cpu);
#endif
	update_sysctl();
}

static void rq_offline_fair(struct rq *rq)
{
#ifdef CONFIG_SCHED_HMP
	hmp_offline_cpu(rq->cpu);
#endif
	update_sysctl();

	
	unthrottle_offline_cfs_rqs(rq);
}

#endif 

static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &curr->se;

	for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		entity_tick(cfs_rq, se, queued);
	}

	if (sched_feat_numa(NUMA))
		task_tick_numa(rq, curr);

	update_rq_runnable_avg(rq, 1);
}

static void task_fork_fair(struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = &p->se, *curr;
	int this_cpu = smp_processor_id();
	struct rq *rq = this_rq();
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);

	update_rq_clock(rq);

	cfs_rq = task_cfs_rq(current);
	curr = cfs_rq->curr;

	rcu_read_lock();
	__set_task_cpu(p, this_cpu);
	rcu_read_unlock();

	update_curr(cfs_rq);

	if (curr)
		se->vruntime = curr->vruntime;
	place_entity(cfs_rq, se, 1);

	if (sysctl_sched_child_runs_first && curr && entity_before(curr, se)) {
		swap(curr->vruntime, se->vruntime);
		resched_task(rq->curr);
	}

	se->vruntime -= cfs_rq->min_vruntime;

	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio)
{
	if (!p->se.on_rq)
		return;

	if (rq->curr == p) {
		if (p->prio > oldprio)
			resched_task(rq->curr);
	} else
		check_preempt_curr(rq, p, 0);
}

static void switched_from_fair(struct rq *rq, struct task_struct *p)
{
	struct sched_entity *se = &p->se;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);

	if (!p->on_rq && p->state != TASK_RUNNING) {
		place_entity(cfs_rq, se, 0);
		se->vruntime -= cfs_rq->min_vruntime;
	}

#ifdef CONFIG_SMP
	if (p->se.avg.decay_count) {
		struct cfs_rq *cfs_rq = cfs_rq_of(&p->se);
		__synchronize_entity_decay(&p->se);
		subtract_blocked_load_contrib(cfs_rq,
				p->se.avg.load_avg_contrib);
	}
#endif
}

static void switched_to_fair(struct rq *rq, struct task_struct *p)
{
	if (!p->se.on_rq)
		return;

	if (rq->curr == p)
		resched_task(rq->curr);
	else{
		if (rq == task_rq(p)) {
			check_preempt_curr(rq, p, 0);
		}
	}
}

static void set_curr_task_fair(struct rq *rq)
{
	struct sched_entity *se = &rq->curr->se;

	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);

		set_next_entity(cfs_rq, se);
		
		account_cfs_rq_runtime(cfs_rq, 0);
	}
}

void init_cfs_rq(struct cfs_rq *cfs_rq)
{
	cfs_rq->tasks_timeline = RB_ROOT;
	cfs_rq->min_vruntime = (u64)(-(1LL << 20));
#ifndef CONFIG_64BIT
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
#ifdef CONFIG_SMP
	atomic64_set(&cfs_rq->decay_counter, 1);
	atomic_long_set(&cfs_rq->removed_load, 0);
#endif
}

#ifdef CONFIG_FAIR_GROUP_SCHED
static void task_move_group_fair(struct task_struct *p, int on_rq)
{
	struct cfs_rq *cfs_rq;
	if (!on_rq && (!p->se.sum_exec_runtime || p->state == TASK_WAKING))
		on_rq = 1;

	if (!on_rq)
		p->se.vruntime -= cfs_rq_of(&p->se)->min_vruntime;
	set_task_rq(p, task_cpu(p));
	if (!on_rq) {
		cfs_rq = cfs_rq_of(&p->se);
		p->se.vruntime += cfs_rq->min_vruntime;
#ifdef CONFIG_SMP
		p->se.avg.decay_count = atomic64_read(&cfs_rq->decay_counter);
		cfs_rq->blocked_load_avg += p->se.avg.load_avg_contrib;
#endif
	}
}

void free_fair_sched_group(struct task_group *tg)
{
	int i;

	destroy_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		if (tg->cfs_rq)
			kfree(tg->cfs_rq[i]);
		if (tg->se)
			kfree(tg->se[i]);
	}

	kfree(tg->cfs_rq);
	kfree(tg->se);
}

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se;
	int i;

	tg->cfs_rq = kzalloc(sizeof(cfs_rq) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->cfs_rq)
		goto err;
	tg->se = kzalloc(sizeof(se) * nr_cpu_ids, GFP_KERNEL);
	if (!tg->se)
		goto err;

	tg->shares = NICE_0_LOAD;

	init_cfs_bandwidth(tg_cfs_bandwidth(tg));

	for_each_possible_cpu(i) {
		cfs_rq = kzalloc_node(sizeof(struct cfs_rq),
				      GFP_KERNEL, cpu_to_node(i));
		if (!cfs_rq)
			goto err;

		se = kzalloc_node(sizeof(struct sched_entity),
				  GFP_KERNEL, cpu_to_node(i));
		if (!se)
			goto err_free_rq;

		init_cfs_rq(cfs_rq);
		init_tg_cfs_entry(tg, cfs_rq, se, i, parent->se[i]);
	}

	return 1;

err_free_rq:
	kfree(cfs_rq);
err:
	return 0;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	if (!tg->cfs_rq[cpu]->on_list)
		return;

	raw_spin_lock_irqsave(&rq->lock, flags);
	list_del_leaf_cfs_rq(tg->cfs_rq[cpu]);
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

void init_tg_cfs_entry(struct task_group *tg, struct cfs_rq *cfs_rq,
			struct sched_entity *se, int cpu,
			struct sched_entity *parent)
{
	struct rq *rq = cpu_rq(cpu);

	cfs_rq->tg = tg;
	cfs_rq->rq = rq;
	init_cfs_rq_runtime(cfs_rq);

	tg->cfs_rq[cpu] = cfs_rq;
	tg->se[cpu] = se;

	
	if (!se)
		return;

	if (!parent)
		se->cfs_rq = &rq->cfs;
	else
		se->cfs_rq = parent->my_q;

	se->my_q = cfs_rq;
	
	update_load_set(&se->load, NICE_0_LOAD);
	se->parent = parent;
}

static DEFINE_MUTEX(shares_mutex);

int sched_group_set_shares(struct task_group *tg, unsigned long shares)
{
	int i;
	unsigned long flags;

	if (!tg->se[0])
		return -EINVAL;

	shares = clamp(shares, scale_load(MIN_SHARES), scale_load(MAX_SHARES));

	mutex_lock(&shares_mutex);
	if (tg->shares == shares)
		goto done;

	tg->shares = shares;
	for_each_possible_cpu(i) {
		struct rq *rq = cpu_rq(i);
		struct sched_entity *se;

		se = tg->se[i];
		
		raw_spin_lock_irqsave(&rq->lock, flags);
		for_each_sched_entity(se)
			update_cfs_shares(group_cfs_rq(se));
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

done:
	mutex_unlock(&shares_mutex);
	return 0;
}
#else 

void free_fair_sched_group(struct task_group *tg) { }

int alloc_fair_sched_group(struct task_group *tg, struct task_group *parent)
{
	return 1;
}

void unregister_fair_sched_group(struct task_group *tg, int cpu) { }

#endif 


static unsigned int get_rr_interval_fair(struct rq *rq, struct task_struct *task)
{
	struct sched_entity *se = &task->se;
	unsigned int rr_interval = 0;

	if (rq->cfs.load.weight)
		rr_interval = NS_TO_JIFFIES(sched_slice(cfs_rq_of(se), se));

	return rr_interval;
}

const struct sched_class fair_sched_class = {
	.next			= &idle_sched_class,
	.enqueue_task		= enqueue_task_fair,
	.dequeue_task		= dequeue_task_fair,
	.yield_task		= yield_task_fair,
	.yield_to_task		= yield_to_task_fair,

	.check_preempt_curr	= check_preempt_wakeup,

	.pick_next_task		= pick_next_task_fair,
	.put_prev_task		= put_prev_task_fair,

#ifdef CONFIG_SMP
	.select_task_rq		= select_task_rq_fair,
	.migrate_task_rq	= migrate_task_rq_fair,

	.rq_online		= rq_online_fair,
	.rq_offline		= rq_offline_fair,

	.task_waking		= task_waking_fair,
#endif

	.set_curr_task          = set_curr_task_fair,
	.task_tick		= task_tick_fair,
	.task_fork		= task_fork_fair,

	.prio_changed		= prio_changed_fair,
	.switched_from		= switched_from_fair,
	.switched_to		= switched_to_fair,

	.get_rr_interval	= get_rr_interval_fair,

#ifdef CONFIG_FAIR_GROUP_SCHED
	.task_move_group	= task_move_group_fair,
#endif
};

#ifdef CONFIG_SCHED_DEBUG
void print_cfs_stats(struct seq_file *m, int cpu)
{
	struct cfs_rq *cfs_rq;

	rcu_read_lock();
	for_each_leaf_cfs_rq(cpu_rq(cpu), cfs_rq)
		print_cfs_rq(m, cpu, cfs_rq);
	rcu_read_unlock();
}
#endif

__init void init_sched_fair_class(void)
{
#ifdef CONFIG_SMP
	open_softirq(SCHED_SOFTIRQ, run_rebalance_domains);

#ifdef CONFIG_NO_HZ_COMMON
	nohz.next_balance = jiffies;
	zalloc_cpumask_var(&nohz.idle_cpus_mask, GFP_NOWAIT);
	cpu_notifier(sched_ilb_notifier, 0);
#endif

	cmp_cputopo_domain_setup();
#ifdef CONFIG_SCHED_HMP
	hmp_cpu_mask_setup();
#endif
#endif 
}

#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
static u32 cpufreq_calc_scale(u32 min, u32 max, u32 curr)
{
	u32 result = curr / max;
	return result;
}

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
DEFINE_PER_CPU(u32, FREQ_CPU);
#endif 

static int cpufreq_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	int cpu = freq->cpu;
	struct cpufreq_extents *extents;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	struct cpumask* mask;
	int id;
#endif

	if (freq->flags & CPUFREQ_CONST_LOOPS)
		return NOTIFY_OK;

	if (val != CPUFREQ_POSTCHANGE)
		return NOTIFY_OK;

	
	if (!hmp_data.freqinvar_load_scale_enabled) {
		freq_scale[cpu].curr_scale = 1024;
		return NOTIFY_OK;
	}

	extents = &freq_scale[cpu];
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	if (extents->max < extents->const_max){
		extents->throttling=1;
	}
	else {
		extents->throttling=0;
	}	
#endif
	if (extents->flags & SCHED_LOAD_FREQINVAR_SINGLEFREQ) {
		extents->curr_scale = 1024;
	} else {
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
		extents->curr_scale = cpufreq_calc_scale(extents->min,
				extents->const_max, freq->new);
#else
		extents->curr_scale = cpufreq_calc_scale(extents->min,
				extents->max, freq->new);
#endif
	}

#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	mask = arch_cpu_is_big(cpu)?&hmp_fast_cpu_mask:&hmp_slow_cpu_mask;
	for_each_cpu(id, mask) 
		freq_scale[id].curr_scale = extents->curr_scale;
#endif

#if NR_CPUS == 4
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
	switch (cpu) {
	case 0:
	case 2:
		(extents + 1)->curr_scale = extents->curr_scale;
		break;

	case 1:
	case 3:
		(extents - 1)->curr_scale = extents->curr_scale;
		break;

	default:

		break;
	}
#endif
#endif

#ifdef CONFIG_HMP_POWER_AWARE_CONTROLLER
	per_cpu(FREQ_CPU, cpu) = freq->new;
#endif 
	return NOTIFY_OK;
}

static int cpufreq_policy_callback(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	struct cpufreq_extents *extents;
	int cpu, singleFreq = 0;
	static const char performance_governor[] = "performance";
	static const char powersave_governor[] = "powersave";

	if (event == CPUFREQ_START)
		return 0;

	if (event != CPUFREQ_INCOMPATIBLE)
		return 0;

	if (!strncmp(policy->governor->name, performance_governor,
			strlen(performance_governor)) ||
		!strncmp(policy->governor->name, powersave_governor,
				strlen(powersave_governor)))
		singleFreq = 1;

	for_each_cpu(cpu, policy->cpus) {
		extents = &freq_scale[cpu];
		extents->max = policy->max >> SCHED_FREQSCALE_SHIFT;
		extents->min = policy->min >> SCHED_FREQSCALE_SHIFT;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
		extents->const_max = policy->cpuinfo.max_freq >> SCHED_FREQSCALE_SHIFT;
#endif
		if (!hmp_data.freqinvar_load_scale_enabled) {
			extents->curr_scale = 1024;
		} else if (singleFreq) {
			extents->flags |= SCHED_LOAD_FREQINVAR_SINGLEFREQ;
			extents->curr_scale = 1024;
		} else {
			extents->flags &= ~SCHED_LOAD_FREQINVAR_SINGLEFREQ;
#ifdef CONFIG_SCHED_HMP_ENHANCEMENT
			extents->curr_scale = cpufreq_calc_scale(extents->min,
					extents->const_max, policy->cur);
#else
			extents->curr_scale = cpufreq_calc_scale(extents->min,
					extents->max, policy->cur);
#endif
		}
	}

	return 0;
}

static struct notifier_block cpufreq_notifier = {
	.notifier_call  = cpufreq_callback,
};
static struct notifier_block cpufreq_policy_notifier = {
	.notifier_call  = cpufreq_policy_callback,
};

static int __init register_sched_cpufreq_notifier(void)
{
	int ret = 0;

	
	for (ret = 0; ret < CONFIG_NR_CPUS; ret++) {
		
		freq_scale[ret].max = 1024;
		freq_scale[ret].min = 1024;
		freq_scale[ret].curr_scale = 1024;
	}

	pr_info("sched: registering cpufreq notifiers for scale-invariant loads\n");
	ret = cpufreq_register_notifier(&cpufreq_policy_notifier,
			CPUFREQ_POLICY_NOTIFIER);

	if (ret != -EINVAL)
		ret = cpufreq_register_notifier(&cpufreq_notifier,
			CPUFREQ_TRANSITION_NOTIFIER);

	return ret;
}

core_initcall(register_sched_cpufreq_notifier);
#endif 

#ifdef CONFIG_HEVTASK_INTERFACE
#ifndef CONFIG_KGDB_KDB
#define SEQ_printf(m, x...)			\
	do {						\
		if (m)					\
		seq_printf(m, x);		\
		else					\
		printk(x);			\
	} while (0)
#else
#define SEQ_printf(m, x...)			\
	do {						\
		if (m)					\
		seq_printf(m, x);		\
		else if (__get_cpu_var(kdb_in_use) == 1)		\
		kdb_printf(x);			\
		else						\
		printk(x);				\
	} while (0)
#endif

static int task_detect_show(struct seq_file *m, void *v)
{
	struct task_struct *p;
	unsigned long flags;
	unsigned int i;

#ifdef CONFIG_HMP_FREQUENCY_INVARIANT_SCALE
	for(i=0;i<NR_CPUS;i++){
		SEQ_printf(m,"%5d ",freq_scale[i].curr_scale);
	}
#endif

	SEQ_printf(m, "\n%lu\n ",jiffies_to_cputime(jiffies));

	for(i=0;i<NR_CPUS;i++){		
		raw_spin_lock_irqsave(&cpu_rq(i)->lock,flags);     
		if(cpu_online(i)){
			list_for_each_entry(p,&cpu_rq(i)->cfs_tasks,se.group_node){
				SEQ_printf(m, "%lu %5d %5d %lu (%15s)\n ",
						p->se.avg.load_avg_ratio,p->pid,task_cpu(p),
						(p->utime+p->stime),p->comm);
			}            
		}
		raw_spin_unlock_irqrestore(&cpu_rq(i)->lock,flags);

	}

	return 0;
}

static int task_detect_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, task_detect_show, NULL);
}

static const struct file_operations task_detect_fops = {
	.open		= task_detect_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init init_task_detect_procfs(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("task_detect", 0444, NULL, &task_detect_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

__initcall(init_task_detect_procfs);
#endif 
