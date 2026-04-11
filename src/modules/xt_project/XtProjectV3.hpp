/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <sys/ioctl.h>
#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/xt_project_start.h>
#include <uORB/topics/xt_project_stop.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_setpoint_orignal.h>
#include <uORB/topics/xt_dronecan_keyvalue.h>

using namespace time_literals;

class XtProject : public ModuleBase<XtProject>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	XtProject();
	~XtProject() override = default;

	//PX4启动模块固定需要
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Publication<xt_project_stop_s>             _xt_pro_stop_pub{ORB_ID(xt_project_stop)};
	uORB::Publication<manual_control_setpoint_s>     _mc_setpoint_pub{ORB_ID(manual_control_setpoint)};

	uORB::Subscription                               _xt_pro_start_sub{ORB_ID(xt_project_start)};
	uORB::Subscription                               _xt_keyvalue_sub{ORB_ID(xt_dronecan_keyvalue)};
	uORB::SubscriptionCallbackWorkItem               _mc_setpoint_sub{this,ORB_ID(manual_control_setpoint_orignal)};

	manual_control_setpoint_s                       _mc_setpoint;
	manual_control_setpoint_orignal_s               _mc_setpoint_org;
	xt_project_stop_s                               _xt_pro_stop;
	xt_project_start_s                              _xt_pro_start;
	xt_dronecan_keyvalue_s                          _xt_keyvalue;

	hrt_abstime                                     now_time{0};
	hrt_abstime                                     start_time{0};

	uint16_t                                        pwm_out{1000};
	float                                           weight_total{0.0f};
	uint16_t                                        sum{0};
	bool                                            inMission{false};

	/* 为避免与遥控器可能使用的AUX1/2冲突，使用AUX3控制PWM输出 */
	void publish_manual_control_setpoint(uint16_t pwm);

	/* 处理重量传感器数据 */
	float deal_with_weight_sensor();

	/* 此处枚举与uavcan_keyvaluebridge中保持一致 */
	enum key_type
	{
		KEY_DISTANCE = 0,
		KEY_WEIGHT = 1
	};
};
