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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/windspeed.h>
#include <uORB/topics/custom_airspeed_estimation.h>

__EXPORT int custom_airspeed_estimation_test_main(int argc, char *argv[]);

int custom_airspeed_estimation_test_main(int argc, char *argv[])
{
    // Subscriptions
    int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    int _vehicle_rate_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    int _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int _vehicle_local_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int _windspeed_sub = orb_subscribe(ORB_ID(windspeed));
    int _airspeed_sub = orb_subscribe(ORB_ID(custom_airspeed_estimation));
    /* limit the update rate to 1000 Hz */
    orb_set_interval(_actuator_controls_sub, 1);
    orb_set_interval(_vehicle_rate_sub, 1);
    orb_set_interval(_vehicle_attitude_sub, 1);
    orb_set_interval(_vehicle_local_sub, 1);
    orb_set_interval(_windspeed_sub, 1);
    orb_set_interval(_airspeed_sub, 1);

    const hrt_abstime now = hrt_absolute_time();
    int index_updated_act=0;
    int index_updated_rate=0;
    int index_updated_att=0;
    int index_updated_local=0;
    int index_updated_wind=0;
    int index_updated_cst=0;
    while (hrt_absolute_time()-now<1000000) {
            bool updated_act;
            orb_check(_actuator_controls_sub, &updated_act);
            if(updated_act){
                struct actuator_controls_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(actuator_controls_0), _actuator_controls_sub, &raw);
                index_updated_act++;
            }
            bool updated_rate;
            orb_check(_vehicle_rate_sub, &updated_rate);
            if(updated_rate){
                struct vehicle_angular_velocity_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_rate_sub, &raw);
                index_updated_rate++;
            }
            bool updated_att;
            orb_check(_vehicle_attitude_sub, &updated_att);
            if(updated_att){
                struct vehicle_attitude_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &raw);
                index_updated_att++;
            }
            bool updated_local;
            orb_check(_vehicle_local_sub, &updated_local);
            if(updated_local){
                struct vehicle_local_position_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_sub, &raw);
                index_updated_local++;
            }
            bool updated_wind;
            orb_check(_windspeed_sub, &updated_wind);
            if(updated_wind){
                struct windspeed_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(windspeed), _windspeed_sub, &raw);
                index_updated_wind++;
            }
            bool updated_cst;
            orb_check(_airspeed_sub, &updated_cst);
            if(updated_cst){
                struct custom_airspeed_estimation_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(custom_airspeed_estimation), _airspeed_sub, &raw);
                index_updated_cst++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates times in 1 second, \n actuator_controls: %d \n vehicle_rate: %d \n vehicle_attitude: %d \n vehicle_local: %d \n windspeed: %d \n custom_airspeed_estimation: %d",index_updated_act,index_updated_rate,index_updated_att,index_updated_local,index_updated_wind,index_updated_cst);

    return 0;
}
