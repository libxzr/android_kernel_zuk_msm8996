/*
 * Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/moduleparam.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/kthread.h>

static unsigned int use_input_evts_with_hi_slvt_detect;


/* Maximum number to clusters that this module will manage*/
static unsigned int num_clusters;
struct cluster {
	cpumask_var_t cpus;
	/* Number of CPUs to maintain online */
	int max_cpu_request;
	/* To track CPUs that the module decides to offline */
	cpumask_var_t offlined_cpus;
	/* stats for load detection */
	/* IO */
	u64 last_io_check_ts;
	unsigned int iowait_enter_cycle_cnt;
	unsigned int iowait_exit_cycle_cnt;
	spinlock_t iowait_lock;
	unsigned int cur_io_busy;
	bool io_change;
	/* CPU */
	unsigned int mode;
	bool mode_change;
	u64 last_mode_check_ts;
	unsigned int single_enter_cycle_cnt;
	unsigned int single_exit_cycle_cnt;
	unsigned int multi_enter_cycle_cnt;
	unsigned int multi_exit_cycle_cnt;
	spinlock_t mode_lock;
	/* Perf Cluster Peak Loads */
	unsigned int perf_cl_peak;
	u64 last_perf_cl_check_ts;
	bool perf_cl_detect_state_change;
	unsigned int perf_cl_peak_enter_cycle_cnt;
	unsigned int perf_cl_peak_exit_cycle_cnt;
	spinlock_t perf_cl_peak_lock;
	/* Tunables */
	unsigned int single_enter_load;
	unsigned int pcpu_multi_enter_load;
	unsigned int perf_cl_peak_enter_load;
	unsigned int single_exit_load;
	unsigned int pcpu_multi_exit_load;
	unsigned int perf_cl_peak_exit_load;
	unsigned int single_enter_cycles;
	unsigned int single_exit_cycles;
	unsigned int multi_enter_cycles;
	unsigned int multi_exit_cycles;
	unsigned int perf_cl_peak_enter_cycles;
	unsigned int perf_cl_peak_exit_cycles;
	unsigned int current_freq;
	spinlock_t timer_lock;
	unsigned int timer_rate;
	struct timer_list mode_exit_timer;
	struct timer_list perf_cl_peak_mode_exit_timer;
};

struct input_events {
	unsigned int evt_x_cnt;
	unsigned int evt_y_cnt;
	unsigned int evt_pres_cnt;
	unsigned int evt_dist_cnt;
};

struct trig_thr {
	unsigned int pwr_cl_trigger_threshold;
	unsigned int perf_cl_trigger_threshold;
	unsigned int ip_evt_threshold;
};
static struct cluster **managed_clusters;
static bool clusters_inited;
static struct trig_thr thr;
/* Work to evaluate the onlining/offlining CPUs */
struct delayed_work evaluate_hotplug_work;

/* To handle cpufreq min/max request */
struct cpu_status {
	unsigned int min;
	unsigned int max;
};
static DEFINE_PER_CPU(struct cpu_status, cpu_stats);

static int init_events_group(void);

#ifdef CONFIG_SCHED_HMP
static DEFINE_PER_CPU(unsigned int, cpu_power_cost);
#endif // CONFIG_SCHED_HMP

struct load_stats {
	u64 last_wallclock;
	/* IO wait related */
	u64 last_iowait;
	unsigned int last_iopercent;
	/* CPU load related */
	unsigned int cpu_load;
	/*CPU Freq*/
	unsigned int freq;
};

struct events {
	spinlock_t cpu_hotplug_lock;
	bool cpu_hotplug;
	bool init_success;
};
static struct events events_group;

#define LAST_UPDATE_TOL		USEC_PER_MSEC

/* Bitmask to keep track of the workloads being detected */
static unsigned int workload_detect;
#define IO_DETECT	1
#define MODE_DETECT	2
#define PERF_CL_PEAK_DETECT	4


/* IOwait related tunables */
static unsigned int io_enter_cycles = 4;
static unsigned int io_exit_cycles = 4;
static u64 iowait_ceiling_pct = 25;
static u64 iowait_floor_pct = 8;
#define LAST_IO_CHECK_TOL	(3 * USEC_PER_MSEC)

/* CPU workload detection related */
#define NO_MODE		(0)
#define SINGLE		(1)
#define MULTI		(2)
#define MIXED		(3)
#define PERF_CL_PEAK		(4)
#define DEF_SINGLE_ENT		90
#define DEF_PCPU_MULTI_ENT	85
#define DEF_PERF_CL_PEAK_ENT	80
#define DEF_SINGLE_EX		60
#define DEF_PCPU_MULTI_EX	50
#define DEF_PERF_CL_PEAK_EX		70
#define DEF_SINGLE_ENTER_CYCLE	4
#define DEF_SINGLE_EXIT_CYCLE	4
#define DEF_MULTI_ENTER_CYCLE	4
#define DEF_MULTI_EXIT_CYCLE	4
#define DEF_PERF_CL_PEAK_ENTER_CYCLE	100
#define DEF_PERF_CL_PEAK_EXIT_CYCLE	20
#define LAST_LD_CHECK_TOL	(2 * USEC_PER_MSEC)
#define CLUSTER_0_THRESHOLD_FREQ	147000
#define CLUSTER_1_THRESHOLD_FREQ	190000
#define INPUT_EVENT_CNT_THRESHOLD	15
#define MAX_LENGTH_CPU_STRING	256

static int set_num_clusters(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_num_clusters(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", num_clusters);
}

static const struct kernel_param_ops param_ops_num_clusters = {
	.set = set_num_clusters,
	.get = get_num_clusters,
};
device_param_cb(num_clusters, &param_ops_num_clusters, NULL, 0644);

static int set_max_cpus(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_max_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:", managed_clusters[i]->max_cpu_request);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_max_cpus = {
	.set = set_max_cpus,
	.get = get_max_cpus,
};

#ifdef CONFIG_MSM_PERFORMANCE_HOTPLUG_ON
device_param_cb(max_cpus, &param_ops_max_cpus, NULL, 0644);
#endif

static int set_managed_cpus(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_managed_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0, total_cnt = 0;
	char tmp[MAX_LENGTH_CPU_STRING] = "";

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++) {
		cnt = cpumap_print_to_pagebuf(true, buf,
						managed_clusters[i]->cpus);
		if ((i + 1) < num_clusters &&
		    (total_cnt + cnt + 1) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			tmp[cnt-1] = ':';
			tmp[cnt] = '\0';
			total_cnt += cnt;
		} else if ((i + 1) == num_clusters &&
			   (total_cnt + cnt) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			total_cnt += cnt;
		} else {
			pr_err("invalid string for managed_cpu:%s%s\n", tmp,
				buf);
			break;
		}
	}
	snprintf(buf, PAGE_SIZE, "%s", tmp);
	return total_cnt;
}

static const struct kernel_param_ops param_ops_managed_cpus = {
	.set = set_managed_cpus,
	.get = get_managed_cpus,
};
device_param_cb(managed_cpus, &param_ops_managed_cpus, NULL, 0644);

/* Read-only node: To display all the online managed CPUs */
static int get_managed_online_cpus(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0, total_cnt = 0;
	char tmp[MAX_LENGTH_CPU_STRING] = "";
	struct cpumask tmp_mask;
	struct cluster *i_cl;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++) {
		i_cl = managed_clusters[i];

		cpumask_clear(&tmp_mask);
		cpumask_complement(&tmp_mask, i_cl->offlined_cpus);
		cpumask_and(&tmp_mask, i_cl->cpus, &tmp_mask);

		cnt = cpumap_print_to_pagebuf(true, buf, &tmp_mask);
		if ((i + 1) < num_clusters &&
		    (total_cnt + cnt + 1) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			tmp[cnt-1] = ':';
			tmp[cnt] = '\0';
			total_cnt += cnt;
		} else if ((i + 1) == num_clusters &&
			   (total_cnt + cnt) <= MAX_LENGTH_CPU_STRING) {
			snprintf(tmp + total_cnt, cnt, "%s", buf);
			total_cnt += cnt;
		} else {
			pr_err("invalid string for managed_cpu:%s%s\n", tmp,
				buf);
			break;
		}
	}
	snprintf(buf, PAGE_SIZE, "%s", tmp);
	return total_cnt;
}

static const struct kernel_param_ops param_ops_managed_online_cpus = {
	.get = get_managed_online_cpus,
};

#ifdef CONFIG_MSM_PERFORMANCE_HOTPLUG_ON
device_param_cb(managed_online_cpus, &param_ops_managed_online_cpus,
							NULL, 0444);
#endif
/*
 * Userspace sends cpu#:min_freq_value to vote for min_freq_value as the new
 * scaling_min. To withdraw its vote it needs to enter cpu#:0
 */
static int set_cpu_min_freq(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_cpu_min_freq(char *buf, const struct kernel_param *kp)
{
	int cnt = 0, cpu;

	for_each_present_cpu(cpu) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu, per_cpu(cpu_stats, cpu).min);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

static const struct kernel_param_ops param_ops_cpu_min_freq = {
	.set = set_cpu_min_freq,
	.get = get_cpu_min_freq,
};
module_param_cb(cpu_min_freq, &param_ops_cpu_min_freq, NULL, 0644);

/*
 * Userspace sends cpu#:max_freq_value to vote for max_freq_value as the new
 * scaling_max. To withdraw its vote it needs to enter cpu#:UINT_MAX
 */
static int set_cpu_max_freq(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_cpu_max_freq(char *buf, const struct kernel_param *kp)
{
	int cnt = 0, cpu;

	for_each_present_cpu(cpu) {
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%d:%u ", cpu, per_cpu(cpu_stats, cpu).max);
	}
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "\n");
	return cnt;
}

static const struct kernel_param_ops param_ops_cpu_max_freq = {
	.set = set_cpu_max_freq,
	.get = get_cpu_max_freq,
};
module_param_cb(cpu_max_freq, &param_ops_cpu_max_freq, NULL, 0644);

static int set_ip_evt_trigger_threshold(const char *buf,
		const struct kernel_param *kp)
{
	return 0;
}

static int get_ip_evt_trigger_threshold(char *buf,
		const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", thr.ip_evt_threshold);
}

static const struct kernel_param_ops param_ops_ip_evt_trig_thr = {
	.set = set_ip_evt_trigger_threshold,
	.get = get_ip_evt_trigger_threshold,
};
device_param_cb(ip_evt_trig_thr, &param_ops_ip_evt_trig_thr, NULL, 0644);


static int set_perf_cl_trigger_threshold(const char *buf,
		 const struct kernel_param *kp)
{
	return 0;
}

static int get_perf_cl_trigger_threshold(char *buf,
		const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", thr.perf_cl_trigger_threshold);
}

static const struct kernel_param_ops param_ops_perf_trig_thr = {
	.set = set_perf_cl_trigger_threshold,
	.get = get_perf_cl_trigger_threshold,
};
device_param_cb(perf_cl_trig_thr, &param_ops_perf_trig_thr, NULL, 0644);


static int set_pwr_cl_trigger_threshold(const char *buf,
		const struct kernel_param *kp)
{
	return 0;
}

static int get_pwr_cl_trigger_threshold(char *buf,
		const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", thr.pwr_cl_trigger_threshold);
}

static const struct kernel_param_ops param_ops_pwr_trig_thr = {
	.set = set_pwr_cl_trigger_threshold,
	.get = get_pwr_cl_trigger_threshold,
};
device_param_cb(pwr_cl_trig_thr, &param_ops_pwr_trig_thr, NULL, 0644);

static int set_single_enter_load(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_single_enter_load(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%u:", managed_clusters[i]->single_enter_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_single_enter_load = {
	.set = set_single_enter_load,
	.get = get_single_enter_load,
};
device_param_cb(single_enter_load, &param_ops_single_enter_load, NULL, 0644);

static int set_single_exit_load(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_single_exit_load(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%u:", managed_clusters[i]->single_exit_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_single_exit_load = {
	.set = set_single_exit_load,
	.get = get_single_exit_load,
};
device_param_cb(single_exit_load, &param_ops_single_exit_load, NULL, 0644);

static int set_pcpu_multi_enter_load(const char *buf,
					const struct kernel_param *kp)
{

	return 0;
}

static int get_pcpu_multi_enter_load(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"%u:", managed_clusters[i]->pcpu_multi_enter_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_pcpu_multi_enter_load = {
	.set = set_pcpu_multi_enter_load,
	.get = get_pcpu_multi_enter_load,
};
device_param_cb(pcpu_multi_enter_load, &param_ops_pcpu_multi_enter_load,
								NULL, 0644);

static int set_pcpu_multi_exit_load(const char *buf,
						const struct kernel_param *kp)
{

	return 0;
}

static int get_pcpu_multi_exit_load(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"%u:", managed_clusters[i]->pcpu_multi_exit_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_pcpu_multi_exit_load = {
	.set = set_pcpu_multi_exit_load,
	.get = get_pcpu_multi_exit_load,
};
device_param_cb(pcpu_multi_exit_load, &param_ops_pcpu_multi_exit_load,
		NULL, 0644);
static int set_perf_cl_peak_enter_load(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_perf_cl_peak_enter_load(char *buf,
				const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"%u:", managed_clusters[i]->perf_cl_peak_enter_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_perf_cl_peak_enter_load = {
	.set = set_perf_cl_peak_enter_load,
	.get = get_perf_cl_peak_enter_load,
};
device_param_cb(perf_cl_peak_enter_load, &param_ops_perf_cl_peak_enter_load,
		 NULL, 0644);

static int set_perf_cl_peak_exit_load(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_perf_cl_peak_exit_load(char *buf,
				const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"%u:", managed_clusters[i]->perf_cl_peak_exit_load);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_perf_cl_peak_exit_load = {
	.set = set_perf_cl_peak_exit_load,
	.get = get_perf_cl_peak_exit_load,
};
device_param_cb(perf_cl_peak_exit_load, &param_ops_perf_cl_peak_exit_load,
		 NULL, 0644);

static int set_perf_cl_peak_enter_cycles(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_perf_cl_peak_enter_cycles(char *buf,
				const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%u:",
				managed_clusters[i]->perf_cl_peak_enter_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_perf_cl_peak_enter_cycles = {
	.set = set_perf_cl_peak_enter_cycles,
	.get = get_perf_cl_peak_enter_cycles,
};
device_param_cb(perf_cl_peak_enter_cycles, &param_ops_perf_cl_peak_enter_cycles,
		NULL, 0644);


static int set_perf_cl_peak_exit_cycles(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_perf_cl_peak_exit_cycles(char *buf,
			const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
			"%u:", managed_clusters[i]->perf_cl_peak_exit_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_perf_cl_peak_exit_cycles = {
	.set = set_perf_cl_peak_exit_cycles,
	.get = get_perf_cl_peak_exit_cycles,
};
device_param_cb(perf_cl_peak_exit_cycles, &param_ops_perf_cl_peak_exit_cycles,
		 NULL, 0644);


static int set_single_enter_cycles(const char *buf,
				const struct kernel_param *kp)
{
	return 0;
}

static int get_single_enter_cycles(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "%u:",
				managed_clusters[i]->single_enter_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_single_enter_cycles = {
	.set = set_single_enter_cycles,
	.get = get_single_enter_cycles,
};
device_param_cb(single_enter_cycles, &param_ops_single_enter_cycles,
		NULL, 0644);


static int set_single_exit_cycles(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_single_exit_cycles(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%u:", managed_clusters[i]->single_exit_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_single_exit_cycles = {
	.set = set_single_exit_cycles,
	.get = get_single_exit_cycles,
};
device_param_cb(single_exit_cycles, &param_ops_single_exit_cycles, NULL, 0644);

static int set_multi_enter_cycles(const char *buf,
				const struct kernel_param *kp)
{

	return 0;
}

static int get_multi_enter_cycles(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%u:", managed_clusters[i]->multi_enter_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_multi_enter_cycles = {
	.set = set_multi_enter_cycles,
	.get = get_multi_enter_cycles,
};
device_param_cb(multi_enter_cycles, &param_ops_multi_enter_cycles, NULL, 0644);

static int set_multi_exit_cycles(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_multi_exit_cycles(char *buf, const struct kernel_param *kp)
{
	int i, cnt = 0;

	if (!clusters_inited)
		return cnt;

	for (i = 0; i < num_clusters; i++)
		cnt += snprintf(buf + cnt, PAGE_SIZE - cnt,
				"%u:", managed_clusters[i]->multi_exit_cycles);
	cnt--;
	cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, " ");
	return cnt;
}

static const struct kernel_param_ops param_ops_multi_exit_cycles = {
	.set = set_multi_exit_cycles,
	.get = get_multi_exit_cycles,
};
device_param_cb(multi_exit_cycles, &param_ops_multi_exit_cycles, NULL, 0644);

static int set_io_enter_cycles(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_io_enter_cycles(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", io_enter_cycles);
}

static const struct kernel_param_ops param_ops_io_enter_cycles = {
	.set = set_io_enter_cycles,
	.get = get_io_enter_cycles,
};
device_param_cb(io_enter_cycles, &param_ops_io_enter_cycles, NULL, 0644);

static int set_io_exit_cycles(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_io_exit_cycles(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", io_exit_cycles);
}

static const struct kernel_param_ops param_ops_io_exit_cycles = {
	.set = set_io_exit_cycles,
	.get = get_io_exit_cycles,
};
device_param_cb(io_exit_cycles, &param_ops_io_exit_cycles, NULL, 0644);

static int set_iowait_floor_pct(const char *buf, const struct kernel_param *kp)
{

	return 0;
}

static int get_iowait_floor_pct(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%llu", iowait_floor_pct);
}

static const struct kernel_param_ops param_ops_iowait_floor_pct = {
	.set = set_iowait_floor_pct,
	.get = get_iowait_floor_pct,
};
device_param_cb(iowait_floor_pct, &param_ops_iowait_floor_pct, NULL, 0644);

static int set_iowait_ceiling_pct(const char *buf,
						const struct kernel_param *kp)
{

	return 0;
}

static int get_iowait_ceiling_pct(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%llu", iowait_ceiling_pct);
}

static const struct kernel_param_ops param_ops_iowait_ceiling_pct = {
	.set = set_iowait_ceiling_pct,
	.get = get_iowait_ceiling_pct,
};
device_param_cb(iowait_ceiling_pct, &param_ops_iowait_ceiling_pct, NULL, 0644);

static int set_workload_detect(const char *buf, const struct kernel_param *kp)
{
	return 0;
}

static int get_workload_detect(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", workload_detect);
}

static const struct kernel_param_ops param_ops_workload_detect = {
	.set = set_workload_detect,
	.get = get_workload_detect,
};
device_param_cb(workload_detect, &param_ops_workload_detect, NULL, 0644);


static int set_input_evts_with_hi_slvt_detect(const char *buf,
					const struct kernel_param *kp)
{
	return 0;
}

static int get_input_evts_with_hi_slvt_detect(char *buf,
					const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u",
			use_input_evts_with_hi_slvt_detect);
}

static const struct kernel_param_ops param_ops_ip_evts_with_hi_slvt_detect = {
	.set = set_input_evts_with_hi_slvt_detect,
	.get = get_input_evts_with_hi_slvt_detect,
};
device_param_cb(input_evts_with_hi_slvt_detect,
	&param_ops_ip_evts_with_hi_slvt_detect, NULL, 0644);

/* CPU Hotplug */
static struct kobject *events_kobj;

static ssize_t show_cpu_hotplug(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "\n");
}
static struct kobj_attribute cpu_hotplug_attr =
__ATTR(cpu_hotplug, 0444, show_cpu_hotplug, NULL);

static struct attribute *events_attrs[] = {
	&cpu_hotplug_attr.attr,
	NULL,
};

static struct attribute_group events_attr_group = {
	.attrs = events_attrs,
};

static int init_events_group(void)
{
	int ret;
	struct kobject *module_kobj;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		return -ENOENT;
	}

	events_kobj = kobject_create_and_add("events", module_kobj);
	if (!events_kobj) {
		return -ENOMEM;
	}

	ret = sysfs_create_group(events_kobj, &events_attr_group);
	if (ret) {
		return ret;
	}

	events_group.init_success = true;

	return 0;
}

static int __init msm_performance_init(void)
{

	init_events_group();

	return 0;
}
late_initcall(msm_performance_init);
