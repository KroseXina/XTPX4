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
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <commander/px4_custom_mode.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

//在实时参与控制解算的系统中还需调用 public px4::ScheduledWorkItem
class XtProject : public ModuleBase<XtProject>, public ModuleParams
{
public:
	XtProject();
	~XtProject() override;

	//PX4启动模块固定需要
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase 在实时参与控制解算的系统中不调用*/
	static XtProject *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() 在实时参与控制解算的系统中使用Run()*/
	void run() override;

private:
	//话题发布者
	uORB::Publication<offboard_control_mode_s>  _offboard_control_mode_puber{ORB_ID(offboard_control_mode)};  //offboard模式控制（位置/速度etc.）
	uORB::Publication<trajectory_setpoint_s>    _trajectory_setpoint_puber{ORB_ID(trajectory_setpoint)};      //根据模式发布的目标数据
	uORB::Publication<vehicle_command_s>        _vehicle_command_puber{ORB_ID(vehicle_command)};              //PX4模式控制

	//话题订阅者
	//uORB::SubscriptionInterval                  _parameter_update_suber{ORB_ID(parameter_update), 1_s};
	uORB::Subscription                          _local_pos_suber{ORB_ID(vehicle_local_position)};           //获取当前无人机位置、信息、加速度etc.(NED)
	uORB::Subscription                          _attitude_suber{ORB_ID(vehicle_attitude)};                  //获取当前无人机姿态

	vehicle_local_position_s                    _local_position;
	vehicle_attitude_s                          _attitude;
	float                                       _yaw;
	u_int64_t                                   start_time = 0;

	//函数模块
	void publish_offboard_control_mode(bool pos,bool vel,bool acc,bool att,bool br);
	void publish_trajectory_setpoint(float x,float y,float z,float vx,float vy,float vz,float yaw);
	void publish_vehicle_command(uint16_t command, float param1, float param2);

	//飞行圆形轨迹测试
	float _heading_first;
	bool isCircling = false;
	int radius = 5;    //半径5m
	float omega = 0.2; //角速度0.2rad/s
	float theta = 0;
};
