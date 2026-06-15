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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <dataman_client/DatamanClient.hpp>
#include <navigator/navigation.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <containers/Array.hpp>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/xt_main_in.h>
#include <uORB/topics/xt_main_out.h>
#include <uORB/topics/xt_dronecan_keyvalue.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class XtPro : public ModuleBase, public ModuleParams
{
public:
	static Descriptor desc;

	XtPro();
	~XtPro() override;

	//PX4启动模块固定需要
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int run_trampoline(int argc, char *argv[]);

	/** @see ModuleBase 在实时参与控制解算的系统中不调用*/
	static XtPro *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	int _fd{-1};

	uORB::Publication<mission_s>            _mission_pub{ORB_ID(mission)};
	uORB::Publication<xt_main_out_s>        _xt_out_pub{ORB_ID(xt_main_out)};
	uORB::Publication<vehicle_command_s>    _vehicle_cmd_pub{ORB_ID(vehicle_command)};
	uORB::Publication<transponder_report_s> _transponder_report_pub{ORB_ID(transponder_report)};

	uORB::Subscription                _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription                _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription                _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription                _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription                _xt_in_sub{ORB_ID(xt_main_in)};
	uORB::Subscription                _keyvalue_sub{ORB_ID(xt_dronecan_keyvalue)};
	uORB::Subscription                _mission_sub{ORB_ID(mission)};
	uORB::Subscription                _params_update_sub{ORB_ID(parameter_update)};

	input_rc_s                        _input_rc;
	vehicle_global_position_s         _global_pos;
	mission_result_s                  _mission_result;
	vehicle_status_s                  _vehicle_status;
	xt_main_in_s                      _xt_in;
	xt_main_out_s                     _xt_out;

	/* 串口通信定义 */
		/* 与lora模块的通信协议定义(uint8_t)：0xAA 0x55 length payload CRC_L CRC_H*/
	/* lora传输的payload结构体,长度为1+4+4=9字节，整帧长度为14字节 */
	/* 传递mission已完成的通信协议定义(uint8_t): 0XAA 0xFF id bool */
	/* payload为0x00或0x01，整帧长度4字节 */
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
		WAIT_CRC2,
		WAIT_ID,
		WAIT_BOOL
	};

	/* 用以接收串口数据 */
	lora_struct _rec_struct;
	uint8_t _rec_id{0};
	bool _rec_struct_vaild {false};
	bool _rec_mission_end {false};
	bool _mission_finished {false};

	/* 用以生成航点 */
	DatamanClient _dataman_client{};
	static constexpr int MAX_TARGET = 50;
	px4::Array<transponder_report_s,MAX_TARGET> _targets{};
	int _target_count{0};

	bool open_uart();
	void handle_receive_data(uint8_t *data, int len);
	uint16_t crc_ccitt(const uint8_t *data, uint8_t len);
	void publish_transponder_report(uint8_t node_id,int32_t lat,int32_t lon);

	//根据现有的target，生成任务
	void create_mission();

	//获取料重，来自于DroneCAN的keyvalue
	float get_current_weight();

	void publish_vehicle_command(uint16_t command, float param1, float param2);
	void xt_do_mission();
	void do_recv_mission_end(const uint8_t& id);
	void do_send_mission_end(const uint8_t& id);

	DEFINE_PARAMETERS(
	(ParamInt<px4::params::XT_ID>) _param_id
	)

	//虚拟环境下使用变量
#ifdef __PX4_POSIX
	bool _mission_created{false};
#endif

};
