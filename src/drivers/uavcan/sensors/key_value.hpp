/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "sensor_bridge.hpp"
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uORB/Publication.hpp>
#include <drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/topics/xt_dronecan_keyvalue.h>


class UavcanKeyValueBridge : public UavcanSensorBridgeBase
{
public:
	UavcanKeyValueBridge(uavcan::INode &node);

	const char *get_name() const override {return "keyvalue";}

	int init() override;

private:
	void keyvalue_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg);
	int init_driver(uavcan_bridge::Channel *channel) override;

	typedef uavcan::MethodBinder < UavcanKeyValueBridge *,
		void (UavcanKeyValueBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &) >
		KeyValueBinder;

	uavcan::Subscriber<uavcan::protocol::debug::KeyValue, KeyValueBinder> _sub_keyvalue;

	uORB::Publication<xt_dronecan_keyvalue_s> _dronecan_keyvalue_pub{ORB_ID(xt_dronecan_keyvalue)};

	PX4Rangefinder	_px4_rangefinder;
};
