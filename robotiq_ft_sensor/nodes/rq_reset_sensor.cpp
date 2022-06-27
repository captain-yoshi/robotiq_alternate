/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Jean-Philippe Roberge <ros@robotiq.com>
 */

#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include "robotiq_ft_sensor/rq_sensor_state.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"



int main(int argc, char **argv)
{
        ros::init(argc, argv, "robotiq_reset_sensor");
	ros::NodeHandle n;

robotiq_ft_sensor::sensor_accessor srv_acc_;
ros::ServiceClient fts_srv_;

	fts_srv_ = n.serviceClient<robotiq_ft_sensor::sensor_accessor>("/robotiq_ft_sensor_acc");
	
ROS_INFO("Waiting for server existance");
fts_srv_.waitForExistence();
ROS_INFO("Found server");

	srv_acc_.request.command_id = srv_acc_.request.COMMAND_SET_ZERO;
	if(fts_srv_.call(srv_acc_)) {
		ROS_INFO("Reset completed");
	} else {
ROS_ERROR("Reset FAILED");
}


}
