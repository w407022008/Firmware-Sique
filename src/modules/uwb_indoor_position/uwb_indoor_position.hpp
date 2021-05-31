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

#include <matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/actuator_outputs.h>
//#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/vehicle_acceleration.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/windspeed.h>
//#include <uORB/topics/battery_status.h>
//#include <uORB/topics/estimator_sensor_bias.h>
//#include <uORB/topics/vehicle_air_data.h>

#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/uwb_msg.h>

extern "C" __EXPORT int uwb_indoor_position_main(int argc, char *argv[]);


class UWBIndoorPosition : public ModuleBase<UWBIndoorPosition>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
        UWBIndoorPosition();

        ~UWBIndoorPosition() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
        void parameters_update(bool force);

        matrix::Dcmf _R_NED_to_EV;

        int uwb_tag_num;                                    ///< number of landmark
        static constexpr uint8_t uwb_tag_num_max{12};		///< max number of landmark
        static constexpr uint8_t dim{9};                    ///< number of UKF states
        float p_p = 1.0f;
        float p_v = 0.1f;
        float p_a = 1.0f;
        float q_p = 0.0f;
        float q_v = 0.0f;
        float q_a = 1.5f;
        float r = 1.0f;

        float alpha = 0.01f;
        float ki = 1.0f;
        float beta = 2.0f;
        float lambda = alpha*alpha*(dim+ki)-dim;

        float dt = 0.1;

        float Tag_pos[3*12];
        float *State[dim];
        float *sigma[dim*(2*dim+1)];
        float *VarP[dim*dim];
        float VarQ[dim*dim];

        bool is_positive_definit;

        /*
         * ukf
         */
        void ukf_update(float dist[]);
        void Predict(float wm[], float w[]);
        void Measure(float Z[]);
        bool check_healthy() {return is_positive_definit;}



        DEFINE_PARAMETERS(
                (ParamFloat<px4::params::UWB_TAG_0_X>) _param_tag_0_x,
                (ParamFloat<px4::params::UWB_TAG_0_Y>) _param_tag_0_y,
                (ParamFloat<px4::params::UWB_TAG_0_Z>) _param_tag_0_z,
                (ParamFloat<px4::params::UWB_TAG_1_X>) _param_tag_1_x,
                (ParamFloat<px4::params::UWB_TAG_1_Y>) _param_tag_1_y,
                (ParamFloat<px4::params::UWB_TAG_1_Z>) _param_tag_1_z,
                (ParamFloat<px4::params::UWB_TAG_2_X>) _param_tag_2_x,
                (ParamFloat<px4::params::UWB_TAG_2_Y>) _param_tag_2_y,
                (ParamFloat<px4::params::UWB_TAG_2_Z>) _param_tag_2_z,
                (ParamFloat<px4::params::UWB_TAG_3_X>) _param_tag_3_x,
                (ParamFloat<px4::params::UWB_TAG_3_Y>) _param_tag_3_y,
                (ParamFloat<px4::params::UWB_TAG_3_Z>) _param_tag_3_z,
                (ParamFloat<px4::params::UWB_TAG_4_X>) _param_tag_4_x,
                (ParamFloat<px4::params::UWB_TAG_4_Y>) _param_tag_4_y,
                (ParamFloat<px4::params::UWB_TAG_4_Z>) _param_tag_4_z,
                (ParamFloat<px4::params::UWB_TAG_5_X>) _param_tag_5_x,
                (ParamFloat<px4::params::UWB_TAG_5_Y>) _param_tag_5_y,
                (ParamFloat<px4::params::UWB_TAG_5_Z>) _param_tag_5_z,
                (ParamFloat<px4::params::UWB_TAG_6_X>) _param_tag_6_x,
                (ParamFloat<px4::params::UWB_TAG_6_Y>) _param_tag_6_y,
                (ParamFloat<px4::params::UWB_TAG_6_Z>) _param_tag_6_z,
                (ParamFloat<px4::params::UWB_TAG_7_X>) _param_tag_7_x,
                (ParamFloat<px4::params::UWB_TAG_7_Y>) _param_tag_7_y,
                (ParamFloat<px4::params::UWB_TAG_7_Z>) _param_tag_7_z,
                (ParamFloat<px4::params::UWB_TAG_8_X>) _param_tag_8_x,
                (ParamFloat<px4::params::UWB_TAG_8_Y>) _param_tag_8_y,
                (ParamFloat<px4::params::UWB_TAG_8_Z>) _param_tag_8_z,
                (ParamFloat<px4::params::UWB_TAG_9_X>) _param_tag_9_x,
                (ParamFloat<px4::params::UWB_TAG_9_Y>) _param_tag_9_y,
                (ParamFloat<px4::params::UWB_TAG_9_Z>) _param_tag_9_z,
                (ParamFloat<px4::params::UWB_TAG_10_X>) _param_tag_10_x,
                (ParamFloat<px4::params::UWB_TAG_10_Y>) _param_tag_10_y,
                (ParamFloat<px4::params::UWB_TAG_10_Z>) _param_tag_10_z,
                (ParamFloat<px4::params::UWB_TAG_11_X>) _param_tag_11_x,
                (ParamFloat<px4::params::UWB_TAG_11_Y>) _param_tag_11_y,
                (ParamFloat<px4::params::UWB_TAG_11_Z>) _param_tag_11_z,
                (ParamInt<px4::params::SENS_UWB_TAG_NUM>) _param_uwb_tag_num,
                (ParamFloat<px4::params::MPC_MAN_NED_SET>) _param_yaw_offset /**< rotate the NED frame along Z-axis descripted in NED frame */
        );

	// Subscriptions
        uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};
//        uORB::Subscription 	_actuator_controls_sub{ORB_ID(actuator_controls_0)};
//        uORB::Subscription 	_actuator_outputs_sub{ORB_ID(actuator_outputs),1};
//        uORB::Subscription 	_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
//        uORB::Subscription 	_vehicle_rate_sub{ORB_ID(vehicle_angular_velocity)};
//        uORB::Subscription 	_vehicle_sensor_combined_sub{ORB_ID(sensor_combined)};
        uORB::Subscription 	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
//        uORB::Subscription 	_vehicle_local_sub{ORB_ID(vehicle_local_position)};
//        uORB::Subscription 	_vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
//        uORB::Subscription 	_windspeed_sub{ORB_ID(windspeed)};
//        uORB::Subscription 	_battery_sub{ORB_ID(battery_status)};
	uORB::Subscription 	_uwb_msg_sub{ORB_ID(uwb_msg)};
//        uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};

	// Publications
	uORB::PublicationData<vehicle_odometry_s>	_uwb_odom_pub{ORB_ID(vehicle_uwb_odometry)};			/**< rate setpoint publication */

    perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": uwb_odometry_cycle")};
    perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": uwb_odometry_interval")};
};

