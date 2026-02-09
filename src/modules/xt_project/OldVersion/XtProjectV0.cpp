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
 * @file XtProjectV0.cpp
 *
 * @brief 投喂机飞控自主实现投料，依据输入信息控制某路PWM输出
 *
 * @version v0:测试新建module,进入offboard模式发布位置信息使无人机按指定路径飞行
 *
 * @author 巡天科技
 */

#include "XtProjectV0.hpp"


XtProject::XtProject():
	ModuleParams(nullptr)
{
	PX4_INFO("Hello XT!");
}

XtProject::~XtProject()
{
	//清除轨迹、控制等信息，返回position模式
	PX4_INFO("Cleaning up and returning control...");

	//在当前位置悬停，安全起见多发几次
	for(int i=0;i<10;++i)
	{
		publish_trajectory_setpoint(_local_position.x,_local_position.y,_local_position.z,0,0,0,_yaw);
		usleep(20000);
	}
	//清除offboard控制标志
	publish_offboard_control_mode(0,0,0,0,0);

	//切换到position模式
	publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);
	usleep(500000);

	//如果高度很低，自动上锁
	if(_local_position.z > -0.5f)
		publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, NAN);

	PX4_INFO("Return to position mode.");
}

void XtProject::run()
{
	//start_time = hrt_absolute_time();  //hrt_absolute_time()得到的结果是微秒

	//切模式前持续发布offboard消息(等待1s)
	for(int i=0;i<50;++i)
	{
		publish_offboard_control_mode(1,1,0,0,0);
		usleep(20000);
	}

	if(_attitude_suber.updated())
	{
		_attitude_suber.copy(&_attitude);
		matrix::Quatf _q(_attitude.q);  //获取四元数
		matrix::Eulerf _euler(_q);      //转为欧拉角
		_heading_first = _euler.psi();  //偏航，弧度
	}

	//进入offboard
	publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
	//解锁
	publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, NAN);

	while(!should_exit())
	{
		//local position update
		if(_local_pos_suber.updated())
			_local_pos_suber.copy(&_local_position);
		//odometroy update
		if(_attitude_suber.updated())
		{
			_attitude_suber.copy(&_attitude);
			matrix::Quatf _q(_attitude.q);  //获取四元数
			matrix::Eulerf _euler(_q);      //转为欧拉角
			_yaw = _euler.psi();            //偏航，弧度
		}

		publish_offboard_control_mode(1,1,0,0,0);
		if(_local_position.z > -4.8f)
		{
			_heading_first = _yaw;
			publish_trajectory_setpoint(_local_position.x,_local_position.y,-5,NAN,NAN,NAN,_heading_first);
			start_time = hrt_absolute_time();  //hrt_absolute_time()得到的结果是微秒。从画轨迹开始计时
		}
		else
		{
			float _t = (hrt_absolute_time() - start_time)*1e-6;
			theta = omega * _t;
			//顺时针方向画圆
			publish_trajectory_setpoint(NAN,NAN,-5,(radius * omega * cos(_heading_first+theta)),
			                                	(radius * omega * sin(_heading_first+theta)),
								0,(_heading_first+theta));
		}

		usleep(20000); //50Hz
	}

}

int XtProject::task_spawn(int argc, char *argv[])
{
	XtProject *instance = new XtProject();

	if (!instance)
	{
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}
	_object.store(instance);
	_task_id = px4_task_spawn_cmd("xtproject",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(3000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);
	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

XtProject *XtProject::instantiate(int argc, char *argv[])
{
	XtProject *instance = new XtProject();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
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

void XtProject::publish_offboard_control_mode(bool pos,bool vel,bool acc,bool att,bool br)
{
	offboard_control_mode_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.position = pos;
	msg.velocity = vel;
	msg.acceleration = acc;
	msg.attitude = att;
	msg.body_rate = br;

	_offboard_control_mode_puber.publish(msg);
}

void XtProject::publish_trajectory_setpoint(float x,float y,float z,float vx,float vy,float vz,float yaw)
{
	trajectory_setpoint_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	msg.yaw = yaw;

	_trajectory_setpoint_puber.publish(msg);

}

void XtProject::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	vehicle_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = false;

	_vehicle_command_puber.publish(msg);
}

extern "C" __EXPORT int xt_project_main(int argc, char *argv[])
{
	return XtProject::main(argc, argv);
}

