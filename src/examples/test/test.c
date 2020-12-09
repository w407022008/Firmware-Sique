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
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/windspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/custom_airspeed_estimation.h>

__EXPORT int test_main(int argc, char *argv[]);

int test_main(int argc, char *argv[])
{
    /* subscribe to sensor_combined topic */
    int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    int _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs),1);
    int _vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    int _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int _vehicle_acceleration_sub = orb_subscribe(ORB_ID(vehicle_acceleration));
    int _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    int _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int _anemometer_sub = orb_subscribe(ORB_ID(windspeed));
    int _battery_status_sub = orb_subscribe(ORB_ID(battery_status));
    int _custom_airspeed_estimation_sub = orb_subscribe(ORB_ID(custom_airspeed_estimation));
    /* limit the update rate to 1000 Hz */
    orb_set_interval(_actuator_controls_sub, 1);
    orb_set_interval(_actuator_outputs_sub, 1);
    orb_set_interval(_vehicle_angular_velocity_sub, 1);
    orb_set_interval(_vehicle_attitude_sub, 1);
    orb_set_interval(_vehicle_acceleration_sub, 1);
    orb_set_interval(_sensor_combined_sub, 1);
    orb_set_interval(_vehicle_local_position_sub, 1);
    orb_set_interval(_anemometer_sub, 1);
    orb_set_interval(_battery_status_sub, 1);
    orb_set_interval(_custom_airspeed_estimation_sub, 1);

    const hrt_abstime now = hrt_absolute_time();
    int index_updated=0;
    while (hrt_absolute_time()-now<1000000) {
            bool updated;
            orb_check(_actuator_controls_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct actuator_controls_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(actuator_controls), _actuator_controls_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of actuator_controls %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<2000000) {
            bool updated;
            orb_check(_actuator_outputs_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct actuator_outputs_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of actuator_outputs %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<3000000) {
            bool updated;
            orb_check(_vehicle_angular_velocity_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct vehicle_angular_velocity_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_angular_velocity_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of vehicle_angular_velocity %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<4000000) {
            bool updated;
            orb_check(_anemometer_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct windspeed_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(windspeed), _anemometer_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of windspeed %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<5000000) {
            bool updated;
            orb_check(_vehicle_attitude_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct vehicle_attitude_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of vehicle_attitude %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<6000000) {
            bool updated;
            orb_check(_vehicle_acceleration_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct vehicle_acceleration_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_acceleration), _vehicle_acceleration_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of vehicle_acceleration %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<7000000) {
            bool updated;
            orb_check(_sensor_combined_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct sensor_combined_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of sensor_combined %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<8000000) {
            bool updated;
            orb_check(_vehicle_local_position_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct vehicle_local_position_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of vehicle_local_position %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<9000000) {
            bool updated;
            orb_check(_battery_status_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct battery_status_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(battery_status), _battery_status_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of battery_status %d times in 1 second",index_updated);

    index_updated=0;
    while (hrt_absolute_time()-now<10000000) {
            bool updated;
            orb_check(_custom_airspeed_estimation_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct custom_airspeed_estimation_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(custom_airspeed_estimation), _custom_airspeed_estimation_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates of custom_airspeed_estimation %d times in 1 second",index_updated);

    return 0;
}
