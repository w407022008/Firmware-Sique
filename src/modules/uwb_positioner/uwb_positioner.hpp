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

extern "C" __EXPORT int uwb_positioner_main(int argc, char *argv[]);


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

        matrix::Dcmf _R_EV_to_NED;

        int test{0};
        int ukf_mode{1};
        int initWithAnalytical{1};
        bool inited{false};
        int uwb_tag_num;                                    ///< number of landmark
        static constexpr uint8_t uwb_tag_num_max{12};		///< max number of landmark
        static constexpr uint8_t dim{9};                    ///< number of UKF states
        const double p_p = 0.1f;
        const double p_v = 0.1f;
        const double p_a = 1.0f;
        const double q_p = 0.1f;
        const double q_v = 0.5f;
        const double q_a = 5.0f;
        const double r = 0.1f;

        const double alpha = 0.8f;
        const double ki = 3-dim;
        const double beta = 2.0f;
        const double lambda = alpha*alpha*(dim+ki)-dim;

        const double dt = 0.1; // 10Hz

        double Tag_pos[3*uwb_tag_num_max];
        double State[dim];
        double Wm[2*dim+1];
        double W[(2*dim+1)*(2*dim+1)];
        double sigma[dim*(2*dim+1)];
        double P[dim*dim];
        double Q[dim];

        bool is_positive_definit = true;
        bool p_z_inv_able = true;
        bool P_is_good = true;

        /*
         * ukf
         */
        void Predict();
        void ukf_update(double dist[]);
        void trilateration(double dist[]);
        bool check_healthy() {return (is_positive_definit & p_z_inv_able & P_is_good);}
        void reset();



        DEFINE_PARAMETERS(
                (ParamInt<px4::params::UWB_INIT_ANA>) _param_init,
                (ParamInt<px4::params::UWB_EST_UKF>) _param_mode,
                (ParamInt<px4::params::UWB_OUTPUT_NED>) _param_output_in_ned,
                (ParamInt<px4::params::UWB_TEST>) _param_test,
                (ParamInt<px4::params::UWB_OUTDOOR_FLY>) _param_flying_outdoor,
                (ParamFloat<px4::params::UWB_TAG_IND_0_X>) _param_indoor_tag_0_x,
                (ParamFloat<px4::params::UWB_TAG_IND_0_Y>) _param_indoor_tag_0_y,
                (ParamFloat<px4::params::UWB_TAG_IND_0_Z>) _param_indoor_tag_0_z,
                (ParamFloat<px4::params::UWB_TAG_IND_1_X>) _param_indoor_tag_1_x,
                (ParamFloat<px4::params::UWB_TAG_IND_1_Y>) _param_indoor_tag_1_y,
                (ParamFloat<px4::params::UWB_TAG_IND_1_Z>) _param_indoor_tag_1_z,
                (ParamFloat<px4::params::UWB_TAG_IND_2_X>) _param_indoor_tag_2_x,
                (ParamFloat<px4::params::UWB_TAG_IND_2_Y>) _param_indoor_tag_2_y,
                (ParamFloat<px4::params::UWB_TAG_IND_2_Z>) _param_indoor_tag_2_z,
                (ParamFloat<px4::params::UWB_TAG_IND_3_X>) _param_indoor_tag_3_x,
                (ParamFloat<px4::params::UWB_TAG_IND_3_Y>) _param_indoor_tag_3_y,
                (ParamFloat<px4::params::UWB_TAG_IND_3_Z>) _param_indoor_tag_3_z,
                (ParamFloat<px4::params::UWB_TAG_IND_4_X>) _param_indoor_tag_4_x,
                (ParamFloat<px4::params::UWB_TAG_IND_4_Y>) _param_indoor_tag_4_y,
                (ParamFloat<px4::params::UWB_TAG_IND_4_Z>) _param_indoor_tag_4_z,
                (ParamFloat<px4::params::UWB_TAG_IND_5_X>) _param_indoor_tag_5_x,
                (ParamFloat<px4::params::UWB_TAG_IND_5_Y>) _param_indoor_tag_5_y,
                (ParamFloat<px4::params::UWB_TAG_IND_5_Z>) _param_indoor_tag_5_z,
                (ParamFloat<px4::params::UWB_TAG_IND_6_X>) _param_indoor_tag_6_x,
                (ParamFloat<px4::params::UWB_TAG_IND_6_Y>) _param_indoor_tag_6_y,
                (ParamFloat<px4::params::UWB_TAG_IND_6_Z>) _param_indoor_tag_6_z,
                (ParamFloat<px4::params::UWB_TAG_IND_7_X>) _param_indoor_tag_7_x,
                (ParamFloat<px4::params::UWB_TAG_IND_7_Y>) _param_indoor_tag_7_y,
                (ParamFloat<px4::params::UWB_TAG_IND_7_Z>) _param_indoor_tag_7_z,
                (ParamFloat<px4::params::UWB_TAG_IND_8_X>) _param_indoor_tag_8_x,
                (ParamFloat<px4::params::UWB_TAG_IND_8_Y>) _param_indoor_tag_8_y,
                (ParamFloat<px4::params::UWB_TAG_IND_8_Z>) _param_indoor_tag_8_z,
                (ParamFloat<px4::params::UWB_TAG_IND_9_X>) _param_indoor_tag_9_x,
                (ParamFloat<px4::params::UWB_TAG_IND_9_Y>) _param_indoor_tag_9_y,
                (ParamFloat<px4::params::UWB_TAG_IND_9_Z>) _param_indoor_tag_9_z,
                (ParamFloat<px4::params::UWB_TAG_IND_10_X>) _param_indoor_tag_10_x,
                (ParamFloat<px4::params::UWB_TAG_IND_10_Y>) _param_indoor_tag_10_y,
                (ParamFloat<px4::params::UWB_TAG_IND_10_Z>) _param_indoor_tag_10_z,
                (ParamFloat<px4::params::UWB_TAG_IND_11_X>) _param_indoor_tag_11_x,
                (ParamFloat<px4::params::UWB_TAG_IND_11_Y>) _param_indoor_tag_11_y,
                (ParamFloat<px4::params::UWB_TAG_IND_11_Z>) _param_indoor_tag_11_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_0_X>) _param_outdoor_tag_0_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_0_Y>) _param_outdoor_tag_0_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_0_Z>) _param_outdoor_tag_0_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_1_X>) _param_outdoor_tag_1_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_1_Y>) _param_outdoor_tag_1_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_1_Z>) _param_outdoor_tag_1_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_2_X>) _param_outdoor_tag_2_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_2_Y>) _param_outdoor_tag_2_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_2_Z>) _param_outdoor_tag_2_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_3_X>) _param_outdoor_tag_3_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_3_Y>) _param_outdoor_tag_3_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_3_Z>) _param_outdoor_tag_3_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_4_X>) _param_outdoor_tag_4_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_4_Y>) _param_outdoor_tag_4_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_4_Z>) _param_outdoor_tag_4_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_5_X>) _param_outdoor_tag_5_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_5_Y>) _param_outdoor_tag_5_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_5_Z>) _param_outdoor_tag_5_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_6_X>) _param_outdoor_tag_6_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_6_Y>) _param_outdoor_tag_6_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_6_Z>) _param_outdoor_tag_6_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_7_X>) _param_outdoor_tag_7_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_7_Y>) _param_outdoor_tag_7_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_7_Z>) _param_outdoor_tag_7_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_8_X>) _param_outdoor_tag_8_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_8_Y>) _param_outdoor_tag_8_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_8_Z>) _param_outdoor_tag_8_z,
                (ParamFloat<px4::params::UWB_TAG_OUTD_9_X>) _param_outdoor_tag_9_x,
                (ParamFloat<px4::params::UWB_TAG_OUTD_9_Y>) _param_outdoor_tag_9_y,
                (ParamFloat<px4::params::UWB_TAG_OUTD_9_Z>) _param_outdoor_tag_9_z,
                (ParamFloat<px4::params::UWB_TAG_OUT_10_X>) _param_outdoor_tag_10_x,
                (ParamFloat<px4::params::UWB_TAG_OUT_10_Y>) _param_outdoor_tag_10_y,
                (ParamFloat<px4::params::UWB_TAG_OUT_10_Z>) _param_outdoor_tag_10_z,
                (ParamFloat<px4::params::UWB_TAG_OUT_11_X>) _param_outdoor_tag_11_x,
                (ParamFloat<px4::params::UWB_TAG_OUT_11_Y>) _param_outdoor_tag_11_y,
                (ParamFloat<px4::params::UWB_TAG_OUT_11_Z>) _param_outdoor_tag_11_z,
                (ParamInt<px4::params::SENS_UWB_TAG_NUM>) _param_uwb_tag_num,
                (ParamFloat<px4::params::MPC_MAN_NED_SET>) _param_yaw_offset /**< rotate the NED frame along Down-axis descripted in NED frame */
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

