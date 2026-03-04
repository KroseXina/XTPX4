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
//#ifdef __PX4_POSIX
//在仿真环境下，仅测试发布transponder_report并在QGC显示的功能
	while (!should_exit())
	{
		add_vehicle.timestamp = hrt_absolute_time();
		add_vehicle.icao_address = 10001;
		if(_gps_sub.update(&vehicle_gps))
		{
			add_vehicle.lat = vehicle_gps.latitude_deg + 0.01;
			add_vehicle.lon = vehicle_gps.longitude_deg + 0.01;
		}
		add_vehicle.altitude = 0.2;
		add_vehicle.heading = 0;
		add_vehicle.hor_velocity = 0;
		add_vehicle.ver_velocity = 0;
		add_vehicle.emitter_type = 0;
		add_vehicle.tslc = 1;
		add_vehicle.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;;

		_transponder_report_pub.publish(add_vehicle);

		usleep(1000000);
	}


//#else
#if 0
	if(!open_uart())
		return;

	PX4_INFO("Open /dev/ttyS4 success.");

	uint8_t buffer[128];

	while(!should_exit())
	{
		//poll属于事件消费型，最好写在循环内部，保证每次都new
		struct pollfd fds{};
		fds.fd = _fd;
		fds.events = POLLIN;

		int ret = px4_poll(&fds, 1, 200); //200ms超时

		if(ret > 0 && (fds.revents & POLLIN))
		{
			int n = ::read(_fd,buffer,sizeof(buffer));
			if(n > 0)
				//收到数据，拼帧
				handle_receive_data(buffer,n);
			if(_rec_struct_vaild)
			{
				PX4_INFO("receive data: node--%d, lat--%d, lon--%d",
					_rec_struct.node_id,_rec_struct.lat,_rec_struct.lon);

				add_vehicle.timestamp = hrt_absolute_time();
				add_vehicle.icao_address = 1000 + _rec_struct.node_id;
				add_vehicle.lat = _rec_struct.lat / 1e7;
				add_vehicle.lon = _rec_struct.lon / 1e7;
				add_vehicle.altitude = 0.2;
				add_vehicle.heading = 0;
				add_vehicle.hor_velocity = 0;
				add_vehicle.ver_velocity = 0;
				add_vehicle.emitter_type = 0;
				add_vehicle.tslc = 1;
				add_vehicle.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;

				_transponder_report_pub.publish(add_vehicle);

				_rec_struct_vaild = false;
			}
		}
	}

	::close(_fd);
#endif
}

bool XtLora::open_uart()
{
	//尝试打开串口
	_fd = ::open("/dev/ttyS4",O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(_fd < 0)
	{
		PX4_ERR("Open ttyS4 failed.");
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
			if(byte == 0x55)
				_parse_state = WAIT_LENGTH;
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
			uint16_t cal_crc = crc_ccitt(_payload,_length);
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
