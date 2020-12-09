/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "custom_airspeed_estimation.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

AirspeedEstimation::AirspeedEstimation()
	: ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	//parameters_update();
	_cst_airspeed_est_pub.advertise();
}

AirspeedEstimation::~AirspeedEstimation()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	_cst_airspeed_est_pub.unadvertise();
}

void AirspeedEstimation::parameters_update()
{
}

bool AirspeedEstimation::init()
{
        ScheduleOnInterval(5000_us); // 5000 us interval, 200 Hz rate

	return true;
}


void AirspeedEstimation::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	/*if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_update();
	}*/
	
	custom_airspeed_estimation_s &report = _cst_airspeed_est_pub.get();

	if (_cst_airspeed_est_sub.updated()){
		custom_airspeed_estimation_s last_data;
		_cst_airspeed_est_sub.update(&last_data);

		report.timestamp_sample_ctl = last_data.timestamp_sample_ctl;
		for(int i=0;i<4;i++){
			report.control[i] = last_data.control[i];
		}

		report.timestamp_sample_pwm = last_data.timestamp_sample_pwm;
		for(int i=0;i<4;i++){
			report.output[i] = last_data.output[i];
		}

		report.timestamp_sample_att = last_data.timestamp_sample_att;
		for(int i=0;i<4;i++){
			report.vehicle_attitude_quaternion_ned[i] = last_data.vehicle_attitude_quaternion_ned[i];
		}

                report.timestamp_sample_imu = last_data.timestamp_sample_imu;
                for(int i=0;i<3;i++){
                        report.vehicle_angular_velocity_xyz[i] = last_data.vehicle_angular_velocity_xyz[i];
                        report.vehicle_acceleration_xyz[i] = last_data.vehicle_acceleration_xyz[i];
                }

                report.timestamp_sample_bias = last_data.timestamp_sample_bias;
                for(int i=0;i<3;i++){
                        report.gyro_bias[i] = last_data.gyro_bias[i];
                        report.accel_bias[i] = last_data.accel_bias[i];
                }

		report.timestamp_sample_ground_truth = last_data.timestamp_sample_ground_truth;
		report.ground_truth_acceleration_ned[0] = last_data.ground_truth_acceleration_ned[0];
		report.ground_truth_acceleration_ned[1] = last_data.ground_truth_acceleration_ned[1];
		report.ground_truth_acceleration_ned[2] = last_data.ground_truth_acceleration_ned[2];
		report.ground_truth_velocity_ned[0] = last_data.ground_truth_velocity_ned[0];
		report.ground_truth_velocity_ned[1] = last_data.ground_truth_velocity_ned[1];
		report.ground_truth_velocity_ned[2] = last_data.ground_truth_velocity_ned[2];

		report.timestamp_sample_wind = last_data.timestamp_sample_wind;
		report.windspeed_x = last_data.windspeed_x;

		report.timestamp_sample_bat = last_data.timestamp_sample_bat;
		report.battery_scale = last_data.battery_scale;
	}

        if (_battery_sub.updated()){
                battery_status_s battery_status;
                _battery_sub.update(&battery_status);
                report.timestamp_sample_bat = battery_status.timestamp;
                report.battery_scale = battery_status.scale;
        }

        if (_vehicle_local_sub.updated()){
                vehicle_local_position_s vehicle_ground_truth;
                _vehicle_local_sub.update(&vehicle_ground_truth);
                report.timestamp_sample_ground_truth = vehicle_ground_truth.timestamp;
                report.ground_truth_acceleration_ned[0] = vehicle_ground_truth.ax;
                report.ground_truth_acceleration_ned[1] = vehicle_ground_truth.ay;
                report.ground_truth_acceleration_ned[2] = vehicle_ground_truth.az;
                report.ground_truth_velocity_ned[0] = vehicle_ground_truth.vx;
                report.ground_truth_velocity_ned[1] = vehicle_ground_truth.vy;
                report.ground_truth_velocity_ned[2] = vehicle_ground_truth.vz;
        }

	if (_vehicle_attitude_sub.updated()){
		vehicle_attitude_s vehicle_attitude;
		_vehicle_attitude_sub.update(&vehicle_attitude);
		report.timestamp_sample_att = vehicle_attitude.timestamp;
		for(int i=0;i<4;i++){
			report.vehicle_attitude_quaternion_ned[i] = vehicle_attitude.q[i];
		}
	}

//        if (_vehicle_acceleration_sub.updated()){
//		vehicle_acceleration_s vehicle_acceleration;
//		_vehicle_acceleration_sub.update(&vehicle_acceleration);
//		report.timestamp_sample_acc = vehicle_acceleration.timestamp_sample;
//		for(int i=0;i<3;i++){
//			report.vehicle_acceleration_xyz[i] = vehicle_acceleration.xyz[i];
//		}
//	}

	if (_windspeed_sub.updated()){
		windspeed_s windspeed;
		_windspeed_sub.update(&windspeed);
		report.timestamp_sample_wind = windspeed.timestamp;
		report.windspeed_x = windspeed.measurement_windspeed_x_m_s;
	}

//        if (_vehicle_rate_sub.updated()){
//                vehicle_angular_velocity_s vehicle_angular_velocity;
//                _vehicle_rate_sub.update(&vehicle_angular_velocity);
//                report.timestamp_sample_rate = vehicle_angular_velocity.timestamp_sample;
//                for(int i=0;i<3;i++){
//                        report.vehicle_angular_velocity_xyz[i] = vehicle_angular_velocity.xyz[i];
//                }
//        }

        if (_actuator_controls_sub.updated()){
                actuator_controls_s actuator_controls;
                _actuator_controls_sub.update(&actuator_controls);
                report.timestamp_sample_ctl = actuator_controls.timestamp;
                for(int i=0;i<4;i++){
                        report.control[i] = actuator_controls.control[i];
                }
        }

        if (_actuator_outputs_sub.updated()){
                actuator_outputs_s actuator_outputs;
                _actuator_outputs_sub.update(&actuator_outputs);
                report.timestamp_sample_pwm = actuator_outputs.timestamp;
                for(int i=0;i<4;i++){
                        report.output[i] = actuator_outputs.output[i];
                }
        }

        if (_estimator_sensor_bias_sub.updated()){
                estimator_sensor_bias_s estimator_sensor_bias;
                _estimator_sensor_bias_sub.update(&estimator_sensor_bias);
                report.timestamp_sample_bias = estimator_sensor_bias.timestamp;
                for(int i=0;i<3;i++){
                        report.gyro_bias[i] = estimator_sensor_bias.gyro_bias[i];
                        report.accel_bias[i] = estimator_sensor_bias.accel_bias[i];
                }
        }

        if (_vehicle_sensor_combined_sub.updated()){
                sensor_combined_s sensor_combined;
                _vehicle_sensor_combined_sub.update(&sensor_combined);
                report.timestamp_sample_imu = sensor_combined.timestamp;
                for(int i=0;i<3;i++){
                        report.vehicle_angular_velocity_xyz[i] = sensor_combined.gyro_rad[i];
                        report.vehicle_acceleration_xyz[i] = sensor_combined.accelerometer_m_s2[i];
                }
        }

	report.timestamp = hrt_absolute_time();
	_cst_airspeed_est_pub.update();

	perf_end(_loop_perf);
}

int AirspeedEstimation::task_spawn(int argc, char *argv[])
{
	AirspeedEstimation *instance = new AirspeedEstimation();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		} else {
			PX4_ERR("init failed");
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AirspeedEstimation::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AirspeedEstimation::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AirspeedEstimation::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start 

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custome_airspeed_estimation", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int custom_airspeed_estimation_main(int argc, char *argv[])
{
	return AirspeedEstimation::main(argc, argv);
}
