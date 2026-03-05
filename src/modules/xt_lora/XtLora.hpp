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
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <containers/Array.hpp>
#include <termios.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <dataman_client/DatamanClient.hpp>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/sensor_gps.h>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/input_rc.h>

using namespace time_literals;

class XtLora : public ModuleBase<XtLora>, public ModuleParams
{
public:
	XtLora();
	~XtLora() override;

	//PX4启动模块固定需要
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase 在实时参与控制解算的系统中不调用*/
	static XtLora *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() 在实时参与控制解算的系统中使用Run()*/
	void run() override;

private:
	int   _fd      {-1};

	/* 与lora模块的通信协议定义(uint8_t)：0xAA 0x55 length payload CRC_L CRC_H*/
	/* lora传输的payload结构体,长度为1+4+4=9字节，整帧长度为14字节 */
	struct lora_struct
	{
		uint8_t node_id;
	   	int32_t lat; //[degE7] Latitude
	   	int32_t lon; //[degE7] Longitude
	};

	/* 定义状态机进行拼包 */
	enum parse_state
	{
		WAIT_HEAD1,
		WAIT_HEAD2,
		WAIT_LENGTH,
		WAIT_PAYLOAD,
		WAIT_CRC1,
		WAIT_CRC2
	};

	/* 用以接收串口数据 */
	lora_struct _rec_struct;
	bool _rec_struct_vaild {false};
	parse_state _parse_state {WAIT_HEAD1};
	uint8_t _length{0};
	uint8_t _payload[64];
	uint8_t _index{0};
	uint16_t _recv_rcr;

	/* 用以生成航点 */
	DatamanClient _dataman_client{};
	static constexpr int MAX_TARGET = 20;
	px4::Array<transponder_report_s,MAX_TARGET> _targets{};
	int _target_count{0};
	bool waypoint_valid {false};

	/* 话题订阅及发布 */
	uORB::Publication<transponder_report_s> _transponder_report_pub{ORB_ID(transponder_report)};
	uORB::Publication<mission_s>            _mission_pub{ORB_ID(mission)};
	uORB::Subscription                      _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription                      _gps_sub{ORB_ID(sensor_gps)};

	input_rc_s               _input_rc;
	sensor_gps_s             vehicle_gps;

	/* 函数区域 */
	bool  open_uart();
	void  handle_receive_data(uint8_t *data, int len);
	uint16_t crc_ccitt(const uint8_t *data, uint8_t len);
	// 发布adsb vehicle，在qgc界面生成定位模型；同时维护targets列表
	void publish_transponder_report(uint8_t node_id,int32_t lat,int32_t lon);
	// 根据目标经纬度信息，生成航点/任务信息
	void create_waypoint();
};
