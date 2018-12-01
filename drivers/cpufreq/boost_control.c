/*
 * Copyright (C) 2018, LibXZR <xzr467706992@163.com>
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

#include <linux/moduleparam.h>

bool enable_fb_boost=false;
module_param_named(enable_fb_boost, enable_fb_boost, bool, 0644);

bool enable_cpu_boost=true;
module_param_named(enable_cpu_boost, enable_cpu_boost, bool, 0644);

u32 input_boost_freq_perf=CONFIG_INPUT_BOOST_FREQ_PERF;
module_param_named(input_boost_freq_perf, input_boost_freq_perf, int, 0644);

u32 input_boost_freq_lp=CONFIG_INPUT_BOOST_FREQ_LP;
module_param_named(input_boost_freq_lp, input_boost_freq_lp, int, 0644);

u32 wake_boost_duration=CONFIG_WAKE_BOOST_DURATION_MS;
module_param_named(cpu_wake_boost_duration, wake_boost_duration, int, 0644);

u32 cpubw_boost_freq=CONFIG_DEVFREQ_MSM_CPUBW_BOOST_FREQ;
module_param_named(cpubw_boost_freq, cpubw_boost_freq, int, 0644);

u32 cpubw_wake_boost_duration=CONFIG_DEVFREQ_WAKE_BOOST_DURATION_MS;
module_param_named(cpubw_wake_boost_duration, cpubw_wake_boost_duration, int, 0644);
