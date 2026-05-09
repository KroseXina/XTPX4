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
 * @file XtPro.cpp
 *
 * @brief 投喂机飞控自动规划航线，执行舵机PWM输出控制
 *        与机载计算机端通信，获取目标航点信息，获取视觉降落辅助信息
 * 	  需要通信的msg内容：本模块与mission模块，使用loiter_time，更改time逻辑为重量触发，重量数据从keyvalue获取
 *			  xt_main_in:bool是否到达,float需要投放的重量
 *			  xt_main_out:float当前料重,bool是否投料完成,bool是否开始进行降落（控制相机打开）,进入offboard
 *  	  与机载机通信，需要添加到uxrce
 *
 * @version v1.0
 *
 * @author 巡天科技
 */

#include "XtPro.hpp"

ModuleBase::Descriptor XtPro::desc{task_spawn, custom_command, print_usage};

XtPro::XtPro() : ModuleParams(nullptr)
{
	PX4_INFO("Hello XT!");
}

XtPro::~XtPro()
{

}

void XtPro::run()
{
	_xt_out.current_weight = 0.0f;
	_xt_out.put_finish = false;
	_xt_out.switch_to_offboard = false;

	constexpr hrt_abstime PUT_TIMEOUT = 5_s;
	hrt_abstime PUT_STARTTIME{0};
	float WEIGHT_EPS{0.0f};

	_total_weight = get_current_weight();

	while (!should_exit())
	{
		//维持xt_main_out的发布
		_xt_out.timestamp = hrt_absolute_time();
		_xt_out_pub.publish(_xt_out);

		//获取料重
		_xt_out.current_weight = get_current_weight();

		//更新transponder report,并维护_targets列表
		if(_transponder_report_sub.updated())
		{
			transponder_report_s msg{};
			bool found = false;
			_transponder_report_sub.copy(&msg);
			if(msg.icao_address == 0) continue;
			for(int i=0;i<_target_count;++i)
			{
				if(_targets[i].icao_address == msg.icao_address)
				{
					_targets[i] = msg;
					found = true;
					break;
				}
			}

			if(!found && _target_count < MAX_TARGET)
			{
				_targets[_target_count++] = msg;
			}
			//为_targets排序
			for (int i = 0; i < _target_count - 1; ++i)
			{
				for (int j = i + 1; j < _target_count; ++j)
				{
					if (_targets[j].icao_address < _targets[i].icao_address)
					{
						auto tmp = _targets[i];
						_targets[i] = _targets[j];
						_targets[j] = tmp;
					}
				}
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);
		_global_pos_sub.update(&_global_pos);

		//是否进入mission
		if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
		{
			_vehicle_in_mission = true;

			if(_xt_in_sub.update(&_xt_in))
			{
				if(_xt_in.wp_reached)
				{
					WEIGHT_EPS = math::min(_xt_in.target_weight * 0.5f,1.0f);

					if(!_putting)
					{
						_total_weight = _xt_out.current_weight;
						//打开放料控制开关-->对应设置的PeripheralActuatorControls，目前选择actuator 1
						publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR,1.0f,NAN);
						_putting = true;
						PUT_STARTTIME = hrt_absolute_time();
					}
					if((_total_weight - _xt_out.current_weight) >= (_xt_in.target_weight - WEIGHT_EPS)) //留下余量
					{
						_xt_out.put_finish = true;
					}
					//投料异常/无料保护
					if(hrt_absolute_time() - PUT_STARTTIME >= PUT_TIMEOUT)
					{
						if(fabsf(_total_weight - _xt_out.current_weight) < WEIGHT_EPS)
							_xt_out.put_finish = true;
					}
				}
				else
				{
					if(_putting)
					{
						publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR,-1.0f,NAN);
						_putting = false;
					}
					_xt_out.put_finish = false;
				}
			}
		}

		//最后一个航点完成
		if(_mission_result_sub.update(&_mission_result))
		{
			if(_vehicle_in_mission && _mission_result.finished)
			{
				_vehicle_in_mission = false;
				_xt_out.switch_to_offboard = true;
			}
		}

		if(_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED)
			_xt_out.switch_to_offboard = false;

		//生成航线,只有在系统就绪并且未解锁时可以执行
		if(_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED && _global_pos.lat_lon_valid)
		{
#ifdef __PX4_POSIX
			//虚拟环境,启动后生成一次航线
			if(!_mission_created)
			{
				create_mission();
				PX4_INFO("target count %d",_target_count);
				_mission_created = true;
			}
#else
			//真实飞机使用遥控器指令生成航线
			bool rc_trigger_now = false;
			if(_input_rc_sub.update(&_input_rc))
			{
				const int channel = 7; //使用遥控器第8通道
				if(channel < _input_rc.channel_count)
					rc_trigger_now = (_input_rc.values[channel] > 1700);
				//边沿触发
				if(rc_trigger_now && !_xt_mission_valid)
					create_mission();

				_xt_mission_valid = rc_trigger_now;
			}
#endif
		}

		px4_usleep(200000);  //最高5Hz执行
	}
}


void XtPro::create_mission()
{
	//1.清理旧的mission
	mission_s mission_old{};
	_dataman_client.readSync(DM_KEY_MISSION_STATE,
				0, reinterpret_cast<uint8_t *>(&mission_old),
				sizeof(mission_s));

	mission_item_s empty_item{};
	for(int i=0;i<math::max(mission_old.count,(uint16_t)(_target_count + 2));i++)
	{
		_dataman_client.writeSync(
			DM_KEY_WAYPOINTS_OFFBOARD_0,
			i,
			reinterpret_cast<uint8_t *>(&empty_item),
			sizeof(mission_item_s)
			);
	}

	//2.设置当前位置为起飞点
	mission_item_s takeoff{};
	takeoff.nav_cmd = NAV_CMD_TAKEOFF;
	takeoff.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;

	takeoff.lat = _global_pos.lat;
	takeoff.lon = _global_pos.lon;
	takeoff.yaw = NAN;
	takeoff.altitude = 5.0f;
	takeoff.altitude_is_relative = true;

	takeoff.autocontinue = true;
	takeoff.acceptance_radius = 0.5f;

	_dataman_client.writeSync(
		DM_KEY_WAYPOINTS_OFFBOARD_0,
		0,
		reinterpret_cast<uint8_t *>(&takeoff),
		sizeof(mission_item_s)
		);

	uint16_t wp_index = 1;
	if(_target_count > 0)
	{
		//3.将targets航点写入dataman
		for (int i = 0; i < _target_count; ++i)
		{
			mission_item_s mission_item{};
			mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;

			mission_item.lat = _targets[i].lat;
			mission_item.lon = _targets[i].lon;
			mission_item.yaw = NAN;
			mission_item.altitude = 5.0f;
			mission_item.altitude_is_relative = true;

			mission_item.time_inside = 5.0f;

			mission_item.autocontinue = true;
			mission_item.acceptance_radius = 0.5f;

			_dataman_client.writeSync(
				DM_KEY_WAYPOINTS_OFFBOARD_0,
				wp_index ++,
				reinterpret_cast<uint8_t *>(&mission_item),
				sizeof(mission_item_s)
				);
		}

		//4.将当前位置设置为最后一个航点
		mission_item_s home{};
		home.nav_cmd = NAV_CMD_WAYPOINT;
		home.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;

		home.lat = _global_pos.lat;
		home.lon = _global_pos.lon;
		home.yaw = NAN;
		home.altitude = 5.0f;
		home.altitude_is_relative = true;

		home.autocontinue = true;
		home.acceptance_radius = 0.5f;

		_dataman_client.writeSync(
			DM_KEY_WAYPOINTS_OFFBOARD_0,
			wp_index ++,
			reinterpret_cast<uint8_t *>(&home),
			sizeof(mission_item_s)
			);
	}

	//5.生成整体mission信息，写入dataman，发布话题触发navigator更新
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();

	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.fence_dataman_id   = DM_KEY_FENCE_POINTS_0;
	mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;

	mission.count = wp_index;
	mission.current_seq = -1;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_id = mission_old.mission_id + 1;
	mission.geofence_id = 0;
	mission.safe_points_id = 0;

	if(!_dataman_client.writeSync(
		DM_KEY_MISSION_STATE,
		0,
		reinterpret_cast<uint8_t *>(&mission),
		sizeof(mission_s)))
	{
		PX4_ERR("Mission state write failed");
		return;
	}

	_mission_pub.publish(mission);
}

float XtPro::get_current_weight()
{
	xt_dronecan_keyvalue_s msg{};
	if(_keyvalue_sub.update(&msg))
	{
		if(msg.key == xt_dronecan_keyvalue_s::KEY_TYPE_WEIGHT)
			return msg.value;
	}

	return 0.0f;
}

void XtPro::publish_vehicle_command(uint16_t command, float param1, float param2)
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

	_vehicle_cmd_pub.publish(msg);
}

int XtPro::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return XtPro::instantiate(ac, av);
	}, argc, argv);
}

int XtPro::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("xtpro",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_NAVIGATION,
					  PX4_STACK_ADJUSTED(3000),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

XtPro *XtPro::instantiate(int argc, char *argv[])
{
	XtPro *instance = new XtPro();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int XtPro::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int XtPro::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("xt_pro", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int xt_pro_main(int argc, char *argv[])
{
	return ModuleBase::main(XtPro::desc, argc, argv);
}
