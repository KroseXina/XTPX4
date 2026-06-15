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
	if(_fd >= 0)
		::close(_fd);
}

void XtPro::run()
{
	_xt_out.current_weight = 0.0f;
	_xt_out.put_finish = false;
	_xt_out.switch_to_offboard = false;

	constexpr hrt_abstime PUT_TIMEOUT = 5_s;
	hrt_abstime PUT_STARTTIME{0};
	float WEIGHT_EPS{0.0f};

	static float total_weight = get_current_weight();

#ifndef __PX4_POSIX
	if(!open_uart()) return;
	uint8_t buffer[128];

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
#endif

	while (!should_exit())
	{
#ifndef __PX4_POSIX
		int ret = px4_poll(fds, 1, 200); //200ms超时，最快5Hz执行该模块

		//处理串口
		if(ret > 0 && (fds[0].revents & POLLIN))
		{
			int n = ::read(_fd,buffer,sizeof(buffer));
			if(n > 0)
				//收到数据，拼帧
				handle_receive_data(buffer,n);
		}

		//处理信标
		if(_rec_struct_vaild)
		{
			publish_transponder_report(_rec_struct.node_id,_rec_struct.lat,_rec_struct.lon);
			_rec_struct_vaild = false;
		}
#else
		px4_usleep(200000);

#endif
		//维持xt_main_out的发布
		_xt_out.timestamp = hrt_absolute_time();
		_xt_out_pub.publish(_xt_out);

		//获取料重
		_xt_out.current_weight = get_current_weight();

		_vehicle_status_sub.update(&_vehicle_status);
		_global_pos_sub.update(&_global_pos);

		//是否进入mission
		static bool vehicle_in_mission = false;
		static bool putting = false;
		if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
		{
			vehicle_in_mission = true;

			if(_xt_in_sub.update(&_xt_in))
			{
				if(_xt_in.wp_reached)
				{
					WEIGHT_EPS = math::min(_xt_in.target_weight * 0.5f,1.0f);

					if(!putting)
					{
						total_weight = _xt_out.current_weight;
						//打开放料控制开关-->对应设置的PeripheralActuatorControls，目前选择actuator 1
						publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR,1.0f,NAN);
						putting = true;
						PUT_STARTTIME = hrt_absolute_time();
					}
					if((total_weight - _xt_out.current_weight) >= (_xt_in.target_weight - WEIGHT_EPS)) //留下余量
					{
						_xt_out.put_finish = true;
					}
					//投料异常/无料保护
					if(hrt_absolute_time() - PUT_STARTTIME >= PUT_TIMEOUT)
					{
						if(fabsf(total_weight - _xt_out.current_weight) < WEIGHT_EPS)
							_xt_out.put_finish = true;
					}
				}
				else
				{
					if(putting)
					{
						publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR,-1.0f,NAN);
						putting = false;
					}
					_xt_out.put_finish = false;
				}
			}
		}

		//最后一个航点完成
		if(_mission_result_sub.update(&_mission_result))
		{
			if(vehicle_in_mission && _mission_result.finished)
			{
				vehicle_in_mission = false;
				_xt_out.switch_to_offboard = true;
				_mission_finished = true;
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
			static bool rc_trigger_last = false;
			bool rc_trigger_now = false;
			if(_input_rc_sub.update(&_input_rc))
			{
				const int channel = 7; //使用遥控器第8通道
				if(channel < _input_rc.channel_count)
					rc_trigger_now = (_input_rc.values[channel] > 1700);
				//边沿触发
				if(rc_trigger_now && !rc_trigger_last)
					create_mission();

				rc_trigger_last = rc_trigger_now;
			}
#endif

			//根据id自动起飞执行投喂
			if(_params_update_sub.updated())
			{
				parameter_update_s p{};
				_params_update_sub.copy(&p);
				updateParams();
			}
			uint8_t xt_id = _param_id.get();
			//当id>0时才执行起飞以及发送任务完成的信息，并且按照n->n+1->n+2的顺序起飞
			if(xt_id > 0)
			{
				if(_rec_mission_end)
					do_recv_mission_end(xt_id);

				if(_mission_finished)
					do_send_mission_end(xt_id);
			}

		}
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

void XtPro::xt_do_mission()
{
	//确认目前有mission存在
	mission_s mission{};
	if(!_mission_sub.copy(&mission) || mission.count < 1)
	{
		PX4_WARN("XT: NO Valid Mission!");
		return;
	}
	//解锁
	publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 21196.f);
	//执行任务
	publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_MISSION_START,0,NAN);
}

//暂时固定使用GPS2口
bool XtPro::open_uart()
{
	_fd = ::open(CONFIG_BOARD_SERIAL_GPS2,O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(_fd < 0)
	{
		PX4_ERR("Open lora uart failed.");
		return false;
	}

	struct termios config;
	tcgetattr(_fd,&config);
	cfmakeraw(&config);

	//设置波特率  115200
	cfsetispeed(&config,B115200);
	cfsetospeed(&config,B115200);

	//8N1
	config.c_cflag &= ~PARENB;
    	config.c_cflag &= ~CSTOPB;
    	config.c_cflag &= ~CSIZE;
    	config.c_cflag |= CS8;
   	config.c_cflag |= (CLOCAL | CREAD);
    	config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    	config.c_iflag &= ~(IXON | IXOFF | IXANY);
    	config.c_oflag &= ~OPOST;

    	tcsetattr(_fd, TCSANOW, &config);

    	return true;
}

void XtPro::handle_receive_data(uint8_t *data, int len)
{
	static parse_state parse = WAIT_HEAD1;
	static uint8_t payload[64];
	static uint8_t index = 0;
	static uint8_t length = 0;

	uint16_t recv_rcr = 0;
	uint16_t cal_crc;

	for(int i=0;i<len;i++)
	{
		uint8_t byte = data[i];
		switch (parse)
		{
		case WAIT_HEAD1:
			if(byte == 0xAA)
				parse = WAIT_HEAD2;
			break;
		case WAIT_HEAD2:
			if(byte == 0x55) //是lora定位信息
				parse = WAIT_LENGTH;
			else if(byte == 0xFF)  //是mission通知信息
				parse = WAIT_ID;
			else
				parse = WAIT_HEAD1;
			break;
        	case WAIT_LENGTH:
            		length = byte;
            		index = 0;

            		if (length == 0 || length > sizeof(payload))
                		parse = WAIT_HEAD1;
            		else
                		parse = WAIT_PAYLOAD;
            		break;
		case WAIT_PAYLOAD:
			payload[index++] = byte;
			if(index >= length)
				parse = WAIT_CRC1;
			break;
		case WAIT_CRC1:
			recv_rcr = byte;
			parse = WAIT_CRC2;
			break;
		case WAIT_CRC2:
			recv_rcr |= ((uint16_t)byte << 8);
			//收到完整一帧，进行校验
			cal_crc = crc_ccitt(payload,length);
			if(cal_crc == recv_rcr)
			{
				//通过校验
				_rec_struct.node_id = payload[0];
				_rec_struct.lat = (int32_t)payload[1]
						|((int32_t)payload[2] << 8)
						|((int32_t)payload[3] << 16)
						|((int32_t)payload[4] << 24);
				_rec_struct.lon = (int32_t)payload[5]
						|((int32_t)payload[6] << 8)
						|((int32_t)payload[7] << 16)
						|((int32_t)payload[8] << 24);

				_rec_struct_vaild = true;
			}
			parse = WAIT_HEAD1;
			break;
		case WAIT_ID:
			_rec_id = byte;
			parse = WAIT_BOOL;
			break;
		case WAIT_BOOL:
			if(byte == 0x01)
				_rec_mission_end = true;
			parse = WAIT_HEAD1;
			break;
		}
	}
}

uint16_t XtPro::crc_ccitt(const uint8_t *data, uint8_t len)
{
	uint16_t crc = 0xFFFF;
	for(int i=0;i<len;i++)
	{
		crc ^= (uint16_t)data[i] << 8;
		for(int j=0;j<8;j++)
		{
			if(crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc;
}

//发布transponder_report，并维护targets列表
void XtPro::publish_transponder_report(uint8_t node_id,int32_t lat,int32_t lon)
{
	transponder_report_s msg{};

	msg.timestamp = hrt_absolute_time();
	msg.icao_address = 1000 + node_id;
	snprintf(msg.callsign,sizeof(msg.callsign),"XT_%02d",node_id);
	msg.lat = static_cast<double>(lat) / 1e7;  //transponder_report_s中为double类型.degree
	msg.lon = static_cast<double>(lon) / 1e7;
	msg.altitude = 0;
	msg.emitter_type = transponder_report_s::ADSB_EMITTER_TYPE_UAV;
	msg.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
		transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
                transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN |
		transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;

	_transponder_report_pub.publish(msg);

	bool found = false;
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

//处理收到任务完成的信息
void XtPro::do_recv_mission_end(const uint8_t& id)
{
	//只在前一个id结束mission时，本id才执行，其它就忽略
	if((id == 1) || _rec_id != (id - 1))
		return;

	static uint16_t count = 0;
	//等待至少5s后再执行起飞（主循环5Hz执行）
	count ++;
	if(count > 25)
	{
		count = 0;
		_rec_mission_end = false;
		xt_do_mission();
	}
}

//发送任务完成信息
void XtPro::do_send_mission_end(const uint8_t& id)
{
	static uint16_t count = 0;
	static uint16_t total_count = 0;
	count ++;
	if(count > 5)
	{
	count = 0;
	total_count ++;

#ifndef __PX4_POSIX
	//避免接收失败，持续发布5次，1Hz发布
	uint8_t send_buf[4];
	send_buf[0] = 0xAA;
	send_buf[1] = 0xFF;
	send_buf[2] = id;
	send_buf[3] = 0x01;
	::write(_fd, send_buf, sizeof(send_buf));

#else
	PX4_INFO("XT: send mission end.");

#endif
	}
	if(total_count > 5)
	{
		total_count = 0;
		_mission_finished = false;
	}
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
