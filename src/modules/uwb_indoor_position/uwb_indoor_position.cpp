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

#include "uwb_indoor_position.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

UWBIndoorPosition::UWBIndoorPosition()
	: ModuleParams(nullptr),
        ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
        // fetch initial parameter values
        parameters_update(true);
        _uwb_odom_pub.advertise();
}

UWBIndoorPosition::~UWBIndoorPosition()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
        _uwb_odom_pub.unadvertise();
}

void UWBIndoorPosition::parameters_update(bool force)
{
    // Check if parameters have changed
    if (_parameter_update_sub.updated() || force) {
            // clear update
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);

            ModuleParams::updateParams();

            _R_NED_to_EV = matrix::Eulerf(0.0f, 0.0f, -math::radians(_param_yaw_offset.get())); // refer to FlightTaskManualAltitude.cpp Fun:_rotateIntoHeadingFrame() -> R_Local_to(in)_NED

            uwb_tag_num = _param_uwb_tag_num.get();

            Tag_pos[0] = _param_tag_0_x.get();
            Tag_pos[12] = _param_tag_0_y.get();
            Tag_pos[24] = _param_tag_0_z.get();
            Tag_pos[1] = _param_tag_1_x.get();
            Tag_pos[13] = _param_tag_1_y.get();
            Tag_pos[25] = _param_tag_1_z.get();
            Tag_pos[2] = _param_tag_2_x.get();
            Tag_pos[14] = _param_tag_2_y.get();
            Tag_pos[26] = _param_tag_2_z.get();
            Tag_pos[3] = _param_tag_3_x.get();
            Tag_pos[15] = _param_tag_3_y.get();
            Tag_pos[27] = _param_tag_3_z.get();
            Tag_pos[4] = _param_tag_4_x.get();
            Tag_pos[16] = _param_tag_4_y.get();
            Tag_pos[28] = _param_tag_4_z.get();
            Tag_pos[5] = _param_tag_5_x.get();
            Tag_pos[17] = _param_tag_5_y.get();
            Tag_pos[29] = _param_tag_5_z.get();
            Tag_pos[6] = _param_tag_6_x.get();
            Tag_pos[18] = _param_tag_6_y.get();
            Tag_pos[30] = _param_tag_6_z.get();
            Tag_pos[7] = _param_tag_7_x.get();
            Tag_pos[19] = _param_tag_7_y.get();
            Tag_pos[31] = _param_tag_7_z.get();
            Tag_pos[8] = _param_tag_8_x.get();
            Tag_pos[20] = _param_tag_8_y.get();
            Tag_pos[32] = _param_tag_8_z.get();
            Tag_pos[9] = _param_tag_9_x.get();
            Tag_pos[21] = _param_tag_9_y.get();
            Tag_pos[33] = _param_tag_9_z.get();
            Tag_pos[10] = _param_tag_10_x.get();
            Tag_pos[22] = _param_tag_10_y.get();
            Tag_pos[34] = _param_tag_10_z.get();
            Tag_pos[11] = _param_tag_11_x.get();
            Tag_pos[23] = _param_tag_11_y.get();
            Tag_pos[35] = _param_tag_11_z.get();
    }

}

bool UWBIndoorPosition::init()
{
        ScheduleOnInterval(50000_us); // 50000 us interval, 20 Hz rate, (UWB real rate 10Hz)

        for(int i=0;i<3;i++){
            *VarP[i*dim+i] = p_p*p_p;
            *VarP[(i+3)*dim+(i+3)] = p_v*p_v;
            *VarP[(i+6)*dim+(i+6)] = p_a*p_a;
        }
        for(int i=0;i<3;i++){
            VarQ[i*dim+i] = q_p*q_p;
            VarQ[(i+3)*dim+(i+3)] = q_v*q_v;
            VarQ[(i+6)*dim+(i+6)] = q_a*q_a;
        }
	return true;
}

float vec_sqr_dist(float A[], float B[], uint8_t n){
    float sum=0.0f;
    for(int i=0;i<n;i++)
        sum += (A[i]-B[i])*(A[i]-B[i]);
    return (float) sqrt(sum);
}

void UWBIndoorPosition::Predict(float wm[], float w[])
{
    /* Sigma Points Weight */
    for(int i=0;i<2*dim+1;i++){
        for(int j=0;j<2*dim+1;j++){
            for(int k=0;k<2*dim+1;k++){
                float c_ik, c_jk;
                c_ik = (i==k) ?
                            1 - wm[i] :
                            - wm[i];
                c_jk = (j==k) ?
                            1 - wm[j] :
                            - wm[j];

                w[i*(2*dim+1)+j] += (k==0) ?
                                     c_ik * (lambda/(dim+lambda) + 1-alpha*alpha+beta) * c_jk :
                                     c_ik * wm[k] * c_jk;
            }
        }
    }

    /* Cholesky Decomposition */
    is_positive_definit = true;
    float *L;
    L = new float[dim * dim];

    for(int i=0;i<dim;i++)
        for(int j=0;j<=i;j++){
            float sum = 0.0f;
            for(int k=0; k<j; k++)
                sum += L[i*dim+k] * L[j*dim+k];

            if(*VarP[i*dim+i] - sum < 1e-6f){
                L[i*dim+j] = 1e-6f;
                is_positive_definit = false;
            }else{
                L[i*dim+j] = (i == j)?
                            (float)sqrt(*VarP[i*dim+i] - sum) :
                            (1.0f / L[j*dim+j] * (*VarP[i*dim+i] - sum));
            }
        }

    memcpy(VarP, L, dim * dim * sizeof(float));

    float A[dim*dim];
    float foo =(float)sqrt(dim+lambda);
    for(int i=0;i<dim;i++)
        for(int j=0;j<dim;j++)
            A[i*dim+j] = *VarP[i*dim+j]*foo;

    /* Sigma Points */
    for(int i=0;i<dim;i++){
        sigma[i*(2*dim+1)] = State[i];
        for(int j=0;j<dim;j++){
            *sigma[i*(2*dim+1)+j+1] = *State[i] + A[i*dim+j];
            *sigma[i*(2*dim+1)+j+1+dim] = *State[i] - A[i*dim+j];
        }
    }

    /* Predict */
    for(int i=0;i<2*dim+1;i++){
        *sigma[i] += *sigma[3*(2*dim+1)+i]*dt + *sigma[6*(2*dim+1)+i]*dt*dt;
        *sigma[(2*dim+1)+i] += *sigma[4*(2*dim+1)+i]*dt + *sigma[7*(2*dim+1)+i]*dt*dt;
        *sigma[2*(2*dim+1)+i] += *sigma[5*(2*dim+1)+i]*dt + *sigma[8*(2*dim+1)+i]*dt*dt;
        *sigma[3*(2*dim+1)+i] += *sigma[6*(2*dim+1)+i]*dt;
        *sigma[4*(2*dim+1)+i] += *sigma[7*(2*dim+1)+i]*dt;
        *sigma[5*(2*dim+1)+i] += *sigma[8*(2*dim+1)+i]*dt;
    }
}

void UWBIndoorPosition::Measure(float Z[])
{
    /* Measurement */
    for(int i=0;i<2*dim+1;i++){
        for(int j=0;j<uwb_tag_num;j++){
            float A[3], B[3];
            for(int k=0;k<3;k++){
                A[k] = *sigma[k*(2*dim+1)+i];
                B[k] = Tag_pos[k*uwb_tag_num+j];
            }
            Z[j*(2*dim+1)+i] = vec_sqr_dist(A,B,3);
        }
    }
}

void UWBIndoorPosition::ukf_update(float dist[])
{
    /* Weights */
    float wm[2*dim+1];
    wm[0] = lambda/(dim+lambda);
    for(int i=1;i<2*dim+1;i++)
        wm[i] = 0.5f/(dim+lambda);

    float w[(2*dim+1)*(2*dim+1)];

    /* Predict */
    Predict(wm, w);

    /* Measurement */
    if (uwb_tag_num_max<uwb_tag_num){
        // debug only first uwb_tag_num_max landmark will be considered
        uwb_tag_num = uwb_tag_num_max;
    }
    float *_dist[uwb_tag_num];
    for(int i=0;i<uwb_tag_num;i++)
        *_dist[i] = dist[i];

    float z[uwb_tag_num*(2*dim+1)];

    Measure(z);

    float *Z_bar = mat_mul(z, wm, uwb_tag_num, 2*dim+1, 1);

    float *zw = mat_mul(z, w, uwb_tag_num, 2*dim+1, 2*dim+1);
    float *z_trans = mat_transpose(z,uwb_tag_num, 2*dim+1);
    float *p_z =mat_mul(zw, z_trans, uwb_tag_num, 2*dim+1, uwb_tag_num);
    delete[] zw;
    for(int i=0;i<uwb_tag_num;i++)
        p_z[i*uwb_tag_num+i] += r*r;

    /* Matrix K */
    float p_z_inv[uwb_tag_num*uwb_tag_num];
    if(mat_inverse(p_z,p_z_inv,uwb_tag_num)){
        float *a = mat_mul(*sigma, w, dim, 2*dim+1, 2*dim+1);
        float *b = mat_mul(a, z_trans, dim, 2*dim+1, uwb_tag_num);
        float *k = mat_mul(b, p_z_inv, dim, uwb_tag_num, uwb_tag_num);
        delete[] a;
        delete[] b;
        delete[] z_trans;

        /* Update */
        float y[uwb_tag_num];
        for(int i=0;i<uwb_tag_num;i++)
            y[i] = *_dist[i] - Z_bar[i];
        float *k_delta = mat_mul(k, y, dim, uwb_tag_num, 1);
        float *State_bar = mat_mul(*sigma, wm, dim, 2*dim+1, 1);

        for(int i=0;i<dim;i++)
            *State[i] = State_bar[i] + k_delta[i];

        delete[] State_bar;
        delete[] k_delta;
        delete[] Z_bar;

        float *k_trans = mat_transpose(k,dim, uwb_tag_num);
        float *k_p_z = mat_mul(k, p_z, dim, uwb_tag_num, uwb_tag_num);
        float *kpk = mat_mul(k_p_z, k_trans, dim, uwb_tag_num, dim);
        delete[] k;
        delete[] k_trans;
        delete[] k_p_z;
        delete[] p_z;
        float *sigma_w = mat_mul(*sigma, w, dim, 2*dim+1, 2*dim+1);
        float *sigma_trans = mat_transpose(*sigma,dim, 2*dim+1);
        float *VarP_bar = mat_mul(sigma_w, sigma_trans, dim, 2*dim+1, dim);

        for(int i=0;i<dim;i++)
            for(int j=0;j<dim;j++)
                *VarP[i*dim+j] = VarP_bar[i*dim+j] + VarQ[i*dim+j] - kpk[i*dim+j];

        delete[] kpk;
        delete[] sigma_w;
        delete[] sigma_trans;
        delete[] VarP_bar;
    }
}

void UWBIndoorPosition::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);



        vehicle_odometry_s &visual_odom = _uwb_odom_pub.get();

        matrix::Quatf q_ev;
//        float_t phi, theta, psi;

        uwb_msg_s uwb_msg;
        bool updated = _uwb_msg_sub.update(&uwb_msg);

        if (updated){
                parameters_update(false);

                float dist[uwb_tag_num];

                visual_odom.timestamp = hrt_absolute_time();
                visual_odom.timestamp_sample = uwb_msg.timestamp;

                for(int i=0;i<uwb_tag_num;i++){
                    dist[i] = uwb_msg.distance[i];
                }

                if (_vehicle_attitude_sub.updated()){
                        vehicle_attitude_s vehicle_attitude;
                        _vehicle_attitude_sub.update(&vehicle_attitude);
                        matrix::Dcmf Body_in_NED = matrix::Dcmf(vehicle_attitude.q);
                        matrix::Dcmf Body_in_EV_Local_Frame = _R_NED_to_EV * Body_in_NED;
                        q_ev = matrix::Quatf(Body_in_EV_Local_Frame);
//                        phi = matrix::Eulerf(q).phi();
//                        theta = matrix::Eulerf(q).theta();
//                        psi = matrix::Eulerf(q).psi();
                }

                ukf_update(dist);

                if(!check_healthy()){

                }

                visual_odom.x = *State[0];
                visual_odom.y = *State[1];
                visual_odom.z = *State[2];
                q_ev.copyTo(visual_odom.q);

                visual_odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED; // need to enable rotate external vision in EKF2_AID_MASK

                visual_odom.pose_covariance[0] = *VarP[0];
                visual_odom.pose_covariance[1] = *VarP[1];
                visual_odom.pose_covariance[2] = *VarP[2];
                visual_odom.pose_covariance[6] = *VarP[dim+1];
                visual_odom.pose_covariance[7] = *VarP[dim+2];
                visual_odom.pose_covariance[11] = *VarP[2*dim+2];
                visual_odom.pose_covariance[15] = 1.0f;
                visual_odom.pose_covariance[18] = 1.0f;
                visual_odom.pose_covariance[20] = 1.0f;

                visual_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD; // need to enable rotate external vision in EKF2_AID_MASK
                visual_odom.vx = *State[3];
                visual_odom.vy = *State[4];
                visual_odom.vz = *State[5];
                visual_odom.rollspeed = NAN;
                visual_odom.pitchspeed = NAN;
                visual_odom.yawspeed = NAN;

                visual_odom.velocity_covariance[0] = *VarP[3*dim+3];
                visual_odom.velocity_covariance[1] = *VarP[3*dim+4];
                visual_odom.velocity_covariance[2] = *VarP[3*dim+5];
                visual_odom.velocity_covariance[6] = *VarP[4*dim+4];
                visual_odom.velocity_covariance[7] = *VarP[4*dim+5];
                visual_odom.velocity_covariance[11] = *VarP[5*dim+5];
                visual_odom.velocity_covariance[15] = NAN;
                visual_odom.velocity_covariance[18] = NAN;
                visual_odom.velocity_covariance[20] = NAN;

                _uwb_odom_pub.update();

        }

	perf_end(_loop_perf);
}

int UWBIndoorPosition::task_spawn(int argc, char *argv[])
{
        UWBIndoorPosition *instance = new UWBIndoorPosition();

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

int UWBIndoorPosition::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int UWBIndoorPosition::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UWBIndoorPosition::print_usage(const char *reason)
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

        PRINT_MODULE_USAGE_NAME("uwb_indoor_position", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uwb_indoor_position_main(int argc, char *argv[])
{
        return UWBIndoorPosition::main(argc, argv);
}
