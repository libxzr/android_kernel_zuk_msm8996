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
#ifndef _BOOST_CONTROL_H_
#define _BOOST_CONTROL_H_

extern bool enable_fb_boost;
extern u32 input_boost_freq_perf;
extern u32 input_boost_freq_lp;
extern u32 wake_boost_duration;
extern unsigned long cpubw_boost_freq;
extern u32 cpubw_wake_boost_duration;
#ifdef CONFIG_DYNAMIC_STUNE_BOOST
extern u16 dynamic_stune_boost;
#endif
extern u16 app_launch_boost_ms;
extern u16 lmk_boost_ms;
#endif
