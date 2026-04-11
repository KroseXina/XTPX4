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
 * @file XtLora.cpp
 *
 * @brief 投喂机识别远端lora模块的定位数据，上传到QGC进行显示
 *        此模块不需要强周期运行，靠poll读取串口
 *
 * @version V0：新建module,读取串口数据，并向QGC发布ADSB_Vehicle mavlink消息以实现显示
 * 		ADSB_Vehicle对应的msg是transponder_report
 * 		根据经纬度信息生成waypoint
 *
 * @author 巡天科技
 */

#include "XtLora.hpp"

XtLora::XtLora():
	ModuleParams(nullptr)
{
	PX4_INFO("Xt lora bridge start.");
}

XtLora::~XtLora()
{
	if(_fd >= 0)
		::close(_fd);
}

void XtLora::run()
{
#ifdef __PX4_POSIX
	// 仿真环境中，测试创建目标点及生成航点
	// 仅在xt_lora start时生成一次
	if(_global_pos_sub.copy(&_global_pos) && _global_pos.lat_lon_valid)
	{
		double lat_s = _global_pos.lat;
		double lon_s = _global_pos.lon;
		publish_transponder_report(1,(lat_s + 0.0005)*1e7,(lon_s + 0.0005)*1e7);
		publish_transponder_report(2,(lat_s + 0.0005)*1e7,(lon_s - 0.0005)*1e7);
		usleep(100000);
		create_waypoint();
		if(_vehicle_status_sub.update(&_vehicle_sta))
		{
			// 若还未解锁，先解锁
			if(_vehicle_sta.arming_state == vehicle_status_s::ARMING_STATE_DISARMED)
				publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, NAN, NAN);
			// 进入mission
			publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_MISSION_START, 0, NAN, NAN);
		}
		usleep(500000); //等待vehicle_status话题更新
		if(_vehicle_status_sub.update(&_vehicle_sta) && (_vehicle_sta.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION))
			_vehicle_in_mission = true;

	}

	while (!should_exit())
	{
		if(_vehicle_status_sub.update(&_vehicle_sta) && _vehicle_land_sub.update(&_vehicle_land) && _vehicle_in_mission)
		{
			if((_vehicle_sta.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) && _vehicle_land.landed)
			{
				PX4_INFO("Xtlora: Mission complete.");
				_vehicle_in_mission = false;
			}
		}
		usleep(200000);
	}


#else
	if(!open_uart())
		return;
	//PX4_INFO("Open /dev/ttyS2 success.");
	if(_vehicle_status_sub.update(&_vehicle_sta))
	{
		_sys_id = _vehicle_sta.system_id;
	}

	uint8_t buffer[128];

	while(!should_exit())
	{
		//poll属于事件消费型，最好写在循环内部，保证每次都new
		struct pollfd fds{};
		fds.fd = _fd;
		fds.events = POLLIN;

		int ret = px4_poll(&fds, 1, 500); //500ms超时

		if(ret > 0 && (fds.revents & POLLIN))
		{
			int n = ::read(_fd,buffer,sizeof(buffer));
			if(n > 0)
				//收到数据，拼帧
				handle_receive_data(buffer,n);

			if(_rec_struct_vaild)
			{
				publish_transponder_report(_rec_struct.node_id,_rec_struct.lat,_rec_struct.lon);
				_rec_struct_vaild = false;

				//物理环境，依靠遥控器输入触发waypoint生成
				bool rc_trigger_now = false;

				if(_input_rc_sub.update(&_input_rc))
				{
					const int channel = 7; //使用遥控器第8通道
					if(channel < _input_rc.channel_count)
						rc_trigger_now = (_input_rc.values[channel] > 1700);  //开关高位

					//上升沿检测
					if(rc_trigger_now && !waypoint_valid)
						create_waypoint();

					waypoint_valid = rc_trigger_now;
				}
			}

			// 前一个飞机结束mission后，自动执行mission
			if(_rec_mission_end)
			{
				create_waypoint();
				if(_vehicle_status_sub.update(&_vehicle_sta))
				{
					// 若还未解锁，先解锁
					if(_vehicle_sta.arming_state == vehicle_status_s::ARMING_STATE_DISARMED)
						publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, NAN, NAN);
					// 进入mission
					publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_MISSION_START, 0, NAN, NAN);
				}
				_rec_mission_end = false;
			}
		}

		_vehicle_status_sub.update(&_vehicle_sta);
		_vehicle_land_sub.update(&_vehicle_land);
		_mission_result_sub.update(&_mission_result);

		// 判断飞机进入了mission并且已经起飞
		if((_vehicle_sta.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) && !_vehicle_land.landed)
			_vehicle_in_mission = true;

		uint8_t mission_end = 0x00;
		// 判断飞机完成了最后一个seq，并已经降落(防止中途切换模式并降落导致错误触发)
		if(_vehicle_in_mission && (_mission_result.seq_total > 0) &&
			(_mission_result.seq_current >= (_mission_result.seq_total - 1)) && _vehicle_land.landed)
		{
			// mission已经结束
			mission_end = 0x01;
			_vehicle_in_mission = false;
		}

		// 与lora模块保持通信
		uint8_t send_buf[4];
		send_buf[0] = 0xAA;
		send_buf[1] = 0xFF;
		send_buf[2] = _sys_id;
		send_buf[3] = mission_end;
		::write(_fd,send_buf,sizeof(send_buf));
	}

	::close(_fd);
#endif
}

bool XtLora::open_uart()
{
	//尝试打开串口,ttyS2对应GPS2/uart4
	_fd = ::open("/dev/ttyS2",O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(_fd < 0)
	{
		PX4_ERR("Open ttyS2 failed.");
		return false;
	}

	struct termios _config;
	tcgetattr(_fd,&_config);

	//设置波特率  115200
	cfsetispeed(&_config,B115200);
	cfsetospeed(&_config,B115200);

	//8N1
	_config.c_cflag &= ~PARENB;
    	_config.c_cflag &= ~CSTOPB;
    	_config.c_cflag &= ~CSIZE;
    	_config.c_cflag |= CS8;
   	_config.c_cflag |= (CLOCAL | CREAD);
    	_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    	_config.c_iflag &= ~(IXON | IXOFF | IXANY);
    	_config.c_oflag &= ~OPOST;

    	tcsetattr(_fd, TCSANOW, &_config);

    	return true;
}

void XtLora::handle_receive_data(uint8_t *data, int len)
{
	uint16_t cal_crc;
	for(int i=0;i<len;i++)
	{
		uint8_t byte = data[i];
		switch (_parse_state)
		{
		case WAIT_HEAD1:
			if(byte == 0xAA)
				_parse_state = WAIT_HEAD2;
			break;
		case WAIT_HEAD2:
			if(byte == 0x55) //是lora定位信息
				_parse_state = WAIT_LENGTH;
			else if(byte == 0xFF)  //是mission通知信息
				_parse_state = WAIT_ID;
			else
				_parse_state = WAIT_HEAD1;
			break;
        	case WAIT_LENGTH:
            		_length = byte;
            		_index = 0;

            		if (_length == 0 || _length > sizeof(_payload))
                		_parse_state = WAIT_HEAD1;
            		else
                		_parse_state = WAIT_PAYLOAD;
            		break;
		case WAIT_PAYLOAD:
			_payload[_index++] = byte;
			if(_index >= _length)
				_parse_state = WAIT_CRC1;
			break;
		case WAIT_CRC1:
			_recv_rcr = byte;
			_parse_state = WAIT_CRC2;
			break;
		case WAIT_CRC2:
			_recv_rcr |= ((uint16_t)byte << 8);
			//收到完整一帧，进行校验
			cal_crc = crc_ccitt(_payload,_length);
			if(cal_crc == _recv_rcr)
			{
				//通过校验
				_rec_struct.node_id = _payload[0];
				_rec_struct.lat = (int32_t)_payload[1]
						|((int32_t)_payload[2] << 8)
						|((int32_t)_payload[3] << 16)
						|((int32_t)_payload[4] << 24);
				_rec_struct.lon = (int32_t)_payload[5]
						|((int32_t)_payload[6] << 8)
						|((int32_t)_payload[7] << 16)
						|((int32_t)_payload[8] << 24);

				_rec_struct_vaild = true;
			}
			_parse_state = WAIT_HEAD1;
			break;
		case WAIT_ID:
			if(byte == (_sys_id - 1))  // 仅接收前一个id的信息
				_parse_state = WAIT_BOOL;
			else
				_parse_state = WAIT_HEAD1;
			break;
		case WAIT_BOOL:
			if(byte == 0x01)
				_rec_mission_end = true;
			_parse_state = WAIT_HEAD1;
			break;
		}
	}
}

uint16_t XtLora::crc_ccitt(const uint8_t *data, uint8_t len)
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

void XtLora::publish_transponder_report(uint8_t node_id,int32_t lat,int32_t lon)
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

	//每次发布transponder_report,维护targets列表，用以生成航点
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

	//为targets列表排序
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

void XtLora::create_waypoint()
{
	//清理旧的mission信息
	mission_s mission_old{};
	_dataman_client.readSync(DM_KEY_MISSION_STATE,
				0, reinterpret_cast<uint8_t *>(&mission_old),
				sizeof(mission_s));
	if(mission_old.count > _target_count)
	{
		mission_item_s empty_item{};
		for(int i=_target_count;i<mission_old.count;i++)
		{
			_dataman_client.writeSync(
				DM_KEY_WAYPOINTS_OFFBOARD_0,
				i,
				reinterpret_cast<uint8_t *>(&empty_item),
				sizeof(mission_item_s)
				);
		}
	}

	//获取当前位置信息作为起飞点
	if(!_global_pos_sub.copy(&_global_pos) || !_global_pos.lat_lon_valid)
		return;
	mission_item_s takeoff{};
	takeoff.nav_cmd = NAV_CMD_TAKEOFF;
	takeoff.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;

	takeoff.lat = _global_pos.lat;
	takeoff.lon = _global_pos.lon;
	takeoff.yaw = NAN;
	takeoff.altitude = 5.0f;
	takeoff.altitude_is_relative = true;

	takeoff.autocontinue = true;
	takeoff.acceptance_radius = 1.0f;

	_dataman_client.writeSync(
		DM_KEY_WAYPOINTS_OFFBOARD_0,
		0,
		reinterpret_cast<uint8_t *>(&takeoff),
		sizeof(mission_item_s)
	);

	//将各个航点信息写入dataman
	for (int i = 0; i < _target_count; ++i)
	{
		mission_item_s mission_item{};
		mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		mission_item.frame = NAV_FRAME_GLOBAL_RELATIVE_ALT;

		mission_item.lat = _targets[i].lat;
		mission_item.lon = _targets[i].lon;
		mission_item.yaw = NAN;
		mission_item.altitude = 5.0f;
		mission_item.altitude_is_relative = true;

		mission_item.autocontinue = true;
		mission_item.acceptance_radius = 1.0f;

		bool success = _dataman_client.writeSync(
		DM_KEY_WAYPOINTS_OFFBOARD_0,
		i + 1,
		reinterpret_cast<uint8_t *>(&mission_item),
		sizeof(mission_item_s)
		);

		if (!success)
		{
			PX4_ERR("Dataman write failed at %d", i);
			return;
		}
	}

	//将当前位置作为返航点
	mission_item_s land{};
	land.nav_cmd = NAV_CMD_RETURN_TO_LAUNCH;
	land.frame = NAV_FRAME_MISSION;

	land.lat = NAN;
	land.lon = NAN;
	land.yaw = NAN;
	land.altitude = NAN;

	land.autocontinue = true;
	land.acceptance_radius = NAN;

	_dataman_client.writeSync(
		DM_KEY_WAYPOINTS_OFFBOARD_0,
		_target_count + 1,
		reinterpret_cast<uint8_t *>(&land),
		sizeof(mission_item_s)
	);


	//将mission信息写入dataman，发布话题以触发navigator更新
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();

	mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission.fence_dataman_id   = DM_KEY_FENCE_POINTS_0;
	mission.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;

	mission.count = _target_count + 2;
	mission.current_seq = -1;
	mission.land_start_index = -1;
	mission.land_index = -1;
	mission.mission_id = hrt_absolute_time();
	mission.geofence_id = 0;
	mission.safe_points_id = 0;

	bool state_write = _dataman_client.writeSync(
		DM_KEY_MISSION_STATE,
		0,
		reinterpret_cast<uint8_t *>(&mission),
		sizeof(mission_s)
	);

	if (!state_write)
	{
		PX4_ERR("Mission state write failed");
		return;
	}

	usleep(100000);
	_mission_pub.publish(mission);

}

void XtLora::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	vehicle_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = false;

	_vehicle_command_pub.publish(msg);
}

int XtLora::task_spawn(int argc, char *argv[])
{
	XtLora *instance = new XtLora();

	if (!instance)
	{
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}
	_object.store(instance);
	_task_id = px4_task_spawn_cmd("xtlora",
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

XtLora *XtLora::instantiate(int argc, char *argv[])
{
	XtLora *instance = new XtLora();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int XtLora::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int XtLora::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("xt_lora", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}


extern "C" __EXPORT int xt_lora_main(int argc, char *argv[])
{
	return XtLora::main(argc, argv);
}
