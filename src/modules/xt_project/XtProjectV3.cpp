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
 * @file XtProjectV3.cpp
 *
 * @brief 投喂机飞控自主实现投料，依据输入信息控制某路PWM输出
 *
 * @version v0:测试新建module,进入offboard模式发布位置信息使无人机按指定路径飞行
 *          v1:测试新module随px4自启动，使用外部指令(ROS2模拟遥控器)，使用一条新建的msg消息
 *          v2:测试控制某一路PWM输出(发布manual_control_setpoint进行控制，暂不考虑遥控器断开的情况)
 *          v3:测试在mission模式中，航点到达后依据本模块指令控制悬停时间
 *
 * @author 巡天科技
 */

#include "XtProjectV3.hpp"


XtProject::XtProject():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	PX4_INFO("Hello XT!");
	_xt_pro_stop.missionstop = false;
	weight_total = deal_with_weight_sensor();
}

bool XtProject::init()
{
	//订阅消息每次更新，触发一次Run
	if (!_mc_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
#ifdef __PX4_POSIX
	//在仿真环境中，没有rc输入，周期性执行Run，50Hz
	ScheduleOnInterval(20_ms);
#endif
	return true;
}

void XtProject::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* 接入称重传感器数据，赋值到_xt_pro_stop.weight */
	_xt_pro_stop.weight = deal_with_weight_sensor();

	/* 判断飞机是否达到了悬停点，并可以开始任务(该话题从mission_block中发布，频率为25Hz) */
	if(_xt_pro_start_sub.update(&_xt_pro_start))
	{
		if(_xt_pro_start.missionstart)
		{
			/* 此处产生任务指令，打开舵机、测量料重etc..
			*  需要排出的weight数据来自QGC输入的时间值 */
			if(sum >= 125)
			{
				_xt_pro_stop.missionstop = true;
				pwm_out = 1000;
				sum = 0;
				PX4_INFO("XT: put weight %.2f kg finished.", (double)_xt_pro_start.weight);
			}
			else
			{
				_xt_pro_stop.missionstop = false;
				pwm_out = 2000;
				sum ++;
			}
			//TODO:考虑先关闭PWM输出，等待1s后再继续任务
			/*
			if((weight_total - _xt_pro_stop.weight) > (_xt_pro_start.weight - 0.1f)) //预留0.1kg余量
			{
				pwm_out = 1000;
				if(sum >= 25)
				{
					_xt_pro_stop.missionstop = true;
					weight_total = deal_with_weight_sensor();
					sum = 0;
				}
				else
				{
					_xt_pro_stop.missionstop = false;
					sum ++;
				}
			}
			else
			{
				_xt_pro_stop.missionstop = false;
				pwm_out = 2000；
			}
			*/
		}
		else
		_xt_pro_stop.missionstop = false;
	}

	_xt_pro_stop.timestamp = hrt_absolute_time();
	_xt_pro_stop_pub.publish(_xt_pro_stop);

	publish_manual_control_setpoint(pwm_out);
}

void XtProject::publish_manual_control_setpoint(uint16_t pwm)
{
	float tem = (static_cast<float>(pwm) - 1500.0f)/500.0f;

	if(_mc_setpoint_sub.update(&_mc_setpoint_org))
	{
		memcpy(&_mc_setpoint,&_mc_setpoint_org,sizeof(_mc_setpoint));

		_mc_setpoint.aux3 = tem;

		_mc_setpoint_pub.publish(_mc_setpoint);
	}

}

float XtProject::deal_with_weight_sensor()
{//TODO:
	return 100.0f;
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

extern "C" __EXPORT int xt_project_main(int argc, char *argv[])
{
	return XtProject::main(argc, argv);
}

