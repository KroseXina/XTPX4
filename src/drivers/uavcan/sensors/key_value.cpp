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

#include "key_value.hpp"

UavcanKeyValueBridge::UavcanKeyValueBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_key_value", ORB_ID(xt_dronecan_keyvalue),nullptr),
	_sub_keyvalue(node),
	_px4_rangefinder(1,0)
{ }

int UavcanKeyValueBridge::init()
{
	int res = _sub_keyvalue.start(KeyValueBinder(this, &UavcanKeyValueBridge::keyvalue_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanKeyValueBridge::keyvalue_cb(const
	uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg)
{
	if(strcmp(msg.key.c_str(),"fw") == 0)
	{
		//uav21r雷达发布距离信息
		_px4_rangefinder.set_device_id(9);
		_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
		_px4_rangefinder.set_min_distance(1.5f);
		_px4_rangefinder.set_max_distance(27.0f);
		_px4_rangefinder.set_hfov(math::radians(30.0f));
		_px4_rangefinder.set_vfov(math::radians(10.0f));
		_px4_rangefinder.set_orientation(distance_sensor_s::ROTATION_FORWARD_FACING);

		hrt_abstime timestamp = hrt_absolute_time();
		_px4_rangefinder.update(timestamp, msg.value);
	}
	else
	{
		//其余can设备发布xt_dronecan_keyvalue
		xt_dronecan_keyvalue_s keyvalue;
		keyvalue.timestamp = hrt_absolute_time();
		keyvalue.value = msg.value;
		keyvalue.key = xt_dronecan_keyvalue_s::KEY_TYPE_WEIGHT;

		_dronecan_keyvalue_pub.publish(keyvalue);
	}
}

int UavcanKeyValueBridge::init_driver(uavcan_bridge::Channel *channel)
{
	return PX4_OK;
}
