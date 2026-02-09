/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

 /**
 * @file XtProjectV2.cpp
 *
 * @brief 投喂机飞控自主实现投料，依据输入信息控制某路PWM输出
 *
 * @version v0:测试新建module,进入offboard模式发布位置信息使无人机按指定路径飞行
 *          v1:测试新module随px4自启动，使用外部指令(ROS2模拟遥控器)，使用一条新建的msg消息
 *          v2:测试控制某一路PWM输出(发布manual_control_setpoint进行控制，适合手动模式)
 *
 * @author 巡天科技
 */

#include "XtProjectV2.hpp"


XtProject::XtProject():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	PX4_INFO("Hello XT!");
	start_time = hrt_absolute_time();
}

bool XtProject::init()
{
	//订阅消息每次更新，触发一次Run
	if (!_mc_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void XtProject::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	_time = hrt_absolute_time();
	uint32_t pwm_sin = 1500 + 500*sin((_time - start_time)*1e-7);
	publish_manual_control_setpoint(pwm_sin);
}

int XtProject::task_spawn(int argc, char *argv[])
{
	XtProject *instance = new XtProject();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int XtProject::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int XtProject::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("xt_project", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

void XtProject::publish_manual_control_setpoint(uint16_t pwm)
{
	float tem = (static_cast<float>(pwm) - 1500.0f)/500.0f;

	if(_mc_setpoint_sub.update(&_mc_setpoint_org))
	{
		memcpy(&_mc_setpoint,&_mc_setpoint_org,sizeof(_mc_setpoint));
		_mc_setpoint.timestamp = hrt_absolute_time();
		_mc_setpoint.aux3 = tem;

		_mc_setpoint_pub.publish(_mc_setpoint);
	}

}

extern "C" __EXPORT int xt_project_main(int argc, char *argv[])
{
	return XtProject::main(argc, argv);
}

