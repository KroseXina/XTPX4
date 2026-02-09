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
 * @file XtProjectV1.cpp
 *
 * @brief 投喂机飞控自主实现投料，依据输入信息控制某路PWM输出
 *
 * @version v0:测试新建module,进入offboard模式发布位置信息使无人机按指定路径飞行
 *          v1:测试新module随px4自启动，使用外部指令(ROS2模拟遥控器)，使用一条新建的msg消息
 *
 * @author 巡天科技
 */

#include "XtProjectV1.hpp"


XtProject::XtProject():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	PX4_INFO("Hello XT!");
	_xt_pro.command = 0; //初始化指令为0
}

bool XtProject::init()
{
	//订阅消息每次更新，触发一次Run
	if (!_xt_pro_suber.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	//周期性执行Run，25Hz
	ScheduleOnInterval(40_ms);

	return true;
}

void XtProject::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	//更新外部指令
	if(_xt_pro_suber.updated())
		_xt_pro_suber.copy(&_xt_pro);

	//更新位置信息
	if(_local_pos_suber.updated())
	_local_pos_suber.copy(&_local_position);

	//更新姿态信息
	if(_attitude_suber.updated())
	{
		_attitude_suber.copy(&_attitude);
		matrix::Quatf _q(_attitude.q);  //获取四元数
		matrix::Eulerf _euler(_q);      //转为欧拉角
		_yaw = _euler.psi();            //偏航，弧度
	}

	switch(_xt_pro.command)
	{
		case 0:break;

		case 1:  //进入offboard，开始轨迹跟踪
			//检查当前状态
			if (_vehicle_status_suber.updated())
			{
				_vehicle_status_suber.copy(&_vehicle_status);
				//若还未解锁，先解锁
				if(_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED)
					publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, NAN);
				//若还未进入offboard，先进入offboard
				if(_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
				{
					for(int i=0;i<10;++i)
					{
						publish_offboard_control_mode(1,1,0,0,0);
						usleep(100);
					}
					publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
				}

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
			break;

		case 9:	//在当前位置悬停
			publish_offboard_control_mode(1,1,0,0,0);
			publish_trajectory_setpoint(_local_position.x,_local_position.y,_local_position.z,0,0,0,_yaw);
			break;

		case 10: //降落
			if(_local_position.z<-0.5f)
			{
				publish_offboard_control_mode(1,1,0,0,0);
				publish_trajectory_setpoint(_local_position.x,_local_position.y,NAN,0,0,0.5,_yaw);
			}
			else
			{
				//清除offboard控制标志
				publish_offboard_control_mode(0,0,0,0,0);
				//切换到position模式
				publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);
			}
			break;

		default:break;
	}
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

