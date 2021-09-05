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

#include "uwb_positioner.hpp"

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

            // _R_EV_to_NED: Rotation matrix from descripted in frame EV to descripted in frame NED
            // _R_EV_in_NED(==_R_EV_to_NED): frame EV space matrix descripted in frame NED, which looks same as _R_EV_to_NED
            // NED -> EV_frame_FRD -> EV_frame_FLU
            if(_param_output_in_ned.get())
            	_R_EV_to_NED = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, math::radians(_param_yaw_offset.get()) + (float)M_PI/2.0f)) * matrix::Dcmf(matrix::Eulerf(M_PI, 0.0f, 0.0f)); // refer to FlightTaskManualAltitude.cpp Fun:_rotateIntoHeadingFrame() -> R_Local_to(in)_NED

            uwb_tag_num = _param_uwb_tag_num.get();
            if (uwb_tag_num_max<uwb_tag_num){
                // debug only first uwb_tag_num_max landmark will be considered
                uwb_tag_num = uwb_tag_num_max;
            }

            initWithAnalytical = _param_init.get();
            ukf_mode = _param_mode.get();
            test = _param_test.get();
            pos_z = _param_pos_z.get();
            if(_param_flying_outdoor.get()){
                Tag_pos[0] = _param_outdoor_tag_0_x.get();
                Tag_pos[12] = _param_outdoor_tag_0_y.get();
                Tag_pos[24] = _param_outdoor_tag_0_z.get();
                Tag_pos[1] = _param_outdoor_tag_1_x.get();
                Tag_pos[13] = _param_outdoor_tag_1_y.get();
                Tag_pos[25] = _param_outdoor_tag_1_z.get();
                Tag_pos[2] = _param_outdoor_tag_2_x.get();
                Tag_pos[14] = _param_outdoor_tag_2_y.get();
                Tag_pos[26] = _param_outdoor_tag_2_z.get();
                Tag_pos[3] = _param_outdoor_tag_3_x.get();
                Tag_pos[15] = _param_outdoor_tag_3_y.get();
                Tag_pos[27] = _param_outdoor_tag_3_z.get();
                Tag_pos[4] = _param_outdoor_tag_4_x.get();
                Tag_pos[16] = _param_outdoor_tag_4_y.get();
                Tag_pos[28] = _param_outdoor_tag_4_z.get();
                Tag_pos[5] = _param_outdoor_tag_5_x.get();
                Tag_pos[17] = _param_outdoor_tag_5_y.get();
                Tag_pos[29] = _param_outdoor_tag_5_z.get();
                Tag_pos[6] = _param_outdoor_tag_6_x.get();
                Tag_pos[18] = _param_outdoor_tag_6_y.get();
                Tag_pos[30] = _param_outdoor_tag_6_z.get();
                Tag_pos[7] = _param_outdoor_tag_7_x.get();
                Tag_pos[19] = _param_outdoor_tag_7_y.get();
                Tag_pos[31] = _param_outdoor_tag_7_z.get();
                Tag_pos[8] = _param_outdoor_tag_8_x.get();
                Tag_pos[20] = _param_outdoor_tag_8_y.get();
                Tag_pos[32] = _param_outdoor_tag_8_z.get();
                Tag_pos[9] = _param_outdoor_tag_9_x.get();
                Tag_pos[21] = _param_outdoor_tag_9_y.get();
                Tag_pos[33] = _param_outdoor_tag_9_z.get();
                Tag_pos[10] = _param_outdoor_tag_10_x.get();
                Tag_pos[22] = _param_outdoor_tag_10_y.get();
                Tag_pos[34] = _param_outdoor_tag_10_z.get();
                Tag_pos[11] = _param_outdoor_tag_11_x.get();
                Tag_pos[23] = _param_outdoor_tag_11_y.get();
                Tag_pos[35] = _param_outdoor_tag_11_z.get();
            } else {
                Tag_pos[0] = _param_indoor_tag_0_x.get();
                Tag_pos[12] = _param_indoor_tag_0_y.get();
                Tag_pos[24] = _param_indoor_tag_0_z.get();
                Tag_pos[1] = _param_indoor_tag_1_x.get();
                Tag_pos[13] = _param_indoor_tag_1_y.get();
                Tag_pos[25] = _param_indoor_tag_1_z.get();
                Tag_pos[2] = _param_indoor_tag_2_x.get();
                Tag_pos[14] = _param_indoor_tag_2_y.get();
                Tag_pos[26] = _param_indoor_tag_2_z.get();
                Tag_pos[3] = _param_indoor_tag_3_x.get();
                Tag_pos[15] = _param_indoor_tag_3_y.get();
                Tag_pos[27] = _param_indoor_tag_3_z.get();
                Tag_pos[4] = _param_indoor_tag_4_x.get();
                Tag_pos[16] = _param_indoor_tag_4_y.get();
                Tag_pos[28] = _param_indoor_tag_4_z.get();
                Tag_pos[5] = _param_indoor_tag_5_x.get();
                Tag_pos[17] = _param_indoor_tag_5_y.get();
                Tag_pos[29] = _param_indoor_tag_5_z.get();
                Tag_pos[6] = _param_indoor_tag_6_x.get();
                Tag_pos[18] = _param_indoor_tag_6_y.get();
                Tag_pos[30] = _param_indoor_tag_6_z.get();
                Tag_pos[7] = _param_indoor_tag_7_x.get();
                Tag_pos[19] = _param_indoor_tag_7_y.get();
                Tag_pos[31] = _param_indoor_tag_7_z.get();
                Tag_pos[8] = _param_indoor_tag_8_x.get();
                Tag_pos[20] = _param_indoor_tag_8_y.get();
                Tag_pos[32] = _param_indoor_tag_8_z.get();
                Tag_pos[9] = _param_indoor_tag_9_x.get();
                Tag_pos[21] = _param_indoor_tag_9_y.get();
                Tag_pos[33] = _param_indoor_tag_9_z.get();
                Tag_pos[10] = _param_indoor_tag_10_x.get();
                Tag_pos[22] = _param_indoor_tag_10_y.get();
                Tag_pos[34] = _param_indoor_tag_10_z.get();
                Tag_pos[11] = _param_indoor_tag_11_x.get();
                Tag_pos[23] = _param_indoor_tag_11_y.get();
                Tag_pos[35] = _param_indoor_tag_11_z.get();
            }
    }

}

double vec_sqr_dist(double A[], double B[], uint8_t n){
    double sum=0.0;
    for(int i=0;i<n;i++)
        sum += (A[i]-B[i])*(A[i]-B[i]);
    return  sqrt(sum);
}

int combination(int up, int down){
    double n=1;
    for(int i=0;i<up;i++){
        n *= down;
        n /= i+1;
        down -= 1;
    }
        return (int)n;
}

bool UWBIndoorPosition::init()
{
        ScheduleOnInterval(50000_us); // 50000 us interval, 20 Hz rate, (UWB real rate 10Hz)

        /* State */
        for(int i=0;i<3;i++){
            State[i] = 0.0f;
            if(dim >= 6) State[i+3] = 0.0f;
            if(dim >= 9) State[i+6] = 0.0f;
        }

        /* Covariance matrix P */
        for(int i=0;i<dim;i++)
            for(int j=0;j<dim;j++)
                P[i*dim+j] = 0.0f;
//        memset(&P, 0.0f, dim*dim * sizeof(double));
        for(int i=0;i<3;i++){
            P[i*dim+i] = p_p*p_p;
            if(dim >= 6) P[(i+3)*dim+(i+3)] = p_v*p_v;
            if(dim >= 9) P[(i+6)*dim+(i+6)] = p_a*p_a;
        }

        /* Covariance matrix Q */
        for(int i=0;i<3;i++){
            Q[i] = q_p*q_p;
            if(dim >= 6) Q[i+3] = q_v*q_v;
            if(dim >= 9) Q[i+6] = q_a*q_a;
        }

        /* Sigma Points Weight */
        Wm[0] = lambda/(dim+lambda);
        for(int i=1;i<2*dim+1;i++)
            Wm[i] = 0.5/(dim+lambda);

        for(int i=0;i<2*dim+1;i++){
            for(int j=0;j<2*dim+1;j++){
                W[i*(2*dim+1)+j] = 0.0f;
                for(int k=0;k<2*dim+1;k++){
                    double c_ik, c_kj;
                    c_ik = (i==k) ?
                                1 - Wm[i] :
                                - Wm[i];
                    c_kj = (j==k) ?
                                1 - Wm[j] :
                                - Wm[j];

                    W[i*(2*dim+1)+j] += (k==0) ?
                                         c_ik * (lambda/(dim+lambda) + 1-alpha*alpha+beta) * c_kj :
                                         c_ik * Wm[k] * c_kj;
                }
            }
        }
        return true;
}

void UWBIndoorPosition::trilateration(double dist[])
{
    /************  buffer  *****************
    * tag_1 tag_2 tag_3 vote_tag_1 vote_tag_2 vote_tag_3 vote_tag_4 vote_tag_5 vote_tag_6 vote_tag_7 vote_tag_8 vote_tag_9
    * candidat_point_1 value candidat_point_2 value candidat_distance
    */
    int num = combination(3,uwb_tag_num);
    double *buffer = new double[num*21];
    memset(buffer, 0.0, num*21 * sizeof(double));
    int l = 0;
    for(int i=0;i<uwb_tag_num;i++)
        for(int j=i+1;j<uwb_tag_num;j++)
            for(int k=j+1;k<uwb_tag_num;k++){
                buffer[l*21+0] = i;
                buffer[l*21+1] = j;
                buffer[l*21+2] = k;
                int col = 3;
                for(int idx=0;idx<uwb_tag_num;idx++)
                        if(idx != i && idx != j && idx != k){
                                buffer[l*21+col] = idx;
                                col += 1;
                        }
                l += 1;
            }

    for(int i=0;i<num;i++){
        // sphere (a,b,c) with radius r
        double a_1=Tag_pos[0*uwb_tag_num_max+(int)buffer[i*21+0]];
        double b_1=Tag_pos[1*uwb_tag_num_max+(int)buffer[i*21+0]];
        double c_1=Tag_pos[2*uwb_tag_num_max+(int)buffer[i*21+0]];
        double r_1=dist[(int)buffer[i*21+0]];
        double a_2=Tag_pos[0*uwb_tag_num_max+(int)buffer[i*21+1]];
        double b_2=Tag_pos[1*uwb_tag_num_max+(int)buffer[i*21+1]];
        double c_2=Tag_pos[2*uwb_tag_num_max+(int)buffer[i*21+1]];
        double r_2=dist[(int)buffer[i*21+1]];
        double a_3=Tag_pos[0*uwb_tag_num_max+(int)buffer[i*21+2]];
        double b_3=Tag_pos[1*uwb_tag_num_max+(int)buffer[i*21+2]];
        double c_3=Tag_pos[2*uwb_tag_num_max+(int)buffer[i*21+2]];
        double r_3=dist[(int)buffer[i*21+2]];
        // Alternative Parameters
        double alpha_1 = 2*(a_2-a_1);
        double alpha_2 = 2*(a_3-a_1);
        double beta_1 = 2*(b_2-b_1);
        double beta_2 = 2*(b_3-b_1);
        double gamma_1 = 2*(c_2-c_1);
        double gamma_2 = 2*(c_3-c_1);
        double theta_1 = (r_1*r_1-r_2*r_2)+(a_2*a_2-a_1*a_1)+(b_2*b_2-b_1*b_1)+(c_2*c_2-c_1*c_1);
        double theta_2 = (r_1*r_1-r_3*r_3)+(a_3*a_3-a_1*a_1)+(b_3*b_3-b_1*b_1)+(c_3*c_3-c_1*c_1);
        // Normal vector of the spherical center plane
        double t_1 = beta_1*gamma_2-gamma_1*beta_2;
        double t_2 = gamma_1*alpha_2-alpha_1*gamma_2;
        double t_3 = alpha_1*beta_2-beta_1*alpha_2;

        double x_0;
        double y_0;
        double z_0 ;
        if(1/t_1 != (double) NAN){
            // (t_1==0) means that x=0 assumption failure
            // point at the intersection line of the three spheres
            if(1/gamma_1!=(double) NAN){
                x_0 = 0;
                y_0 = (theta_1*gamma_2-theta_2*gamma_1)/(beta_1*gamma_2-beta_2*gamma_1);
                z_0 = (theta_1 - (theta_1*gamma_2-theta_2*gamma_1)/(beta_1*gamma_2-beta_2*gamma_1) * beta_1)/gamma_1;
            }else if(1/gamma_2!=(double) NAN){
                x_0 = 0;
                y_0 = (theta_1*gamma_2-theta_2*gamma_1)/(beta_1*gamma_2-beta_2*gamma_1);
                z_0 = (theta_2 - (theta_1*gamma_2-theta_2*gamma_1)/(beta_1*gamma_2-beta_2*gamma_1) * beta_2)/gamma_2;
            }else
                continue;
        }else if(1/t_2 != (double) NAN){
            // (t_2==0) means that y=0 assumption failure
            // point at the intersection line of the three spheres
            if(1/alpha_1!=(double) NAN){
                x_0 = (theta_1 - (theta_1*alpha_2-theta_2*alpha_1)/(gamma_1*alpha_2-gamma_2*alpha_1) * gamma_1)/alpha_1;
                y_0 = 0;
                z_0 = (theta_1*alpha_2-theta_2*alpha_1)/(gamma_1*alpha_2-gamma_2*alpha_1);
            }else if(1/alpha_2!=(double) NAN){
                x_0 = (theta_2 - (theta_1*alpha_2-theta_2*alpha_1)/(gamma_1*alpha_2-gamma_2*alpha_1) * gamma_2)/alpha_2;
                y_0 = 0;
                z_0 = (theta_1*alpha_2-theta_2*alpha_1)/(gamma_1*alpha_2-gamma_2*alpha_1);
            }else
                continue;
        }else if(1/t_3 != (double) NAN){
            // (t_3==0) means that z=0 assumption failure
            // point at the intersection line of the three spheres
            if(1/alpha_1!=(double) NAN){
                x_0 = (theta_1 - (theta_1*alpha_2-theta_2*alpha_1)/(beta_1*alpha_2-beta_2*alpha_1) * beta_1)/alpha_1;
                y_0 = (theta_1*alpha_2-theta_2*alpha_1)/(beta_1*alpha_2-beta_2*alpha_1);
                z_0 = 0;
            }else if(1/alpha_2!=(double) NAN){
                x_0 = (theta_2 - (theta_1*alpha_2-theta_2*alpha_1)/(beta_1*alpha_2-beta_2*alpha_1) * beta_2)/alpha_2;
                y_0 = (theta_1*alpha_2-theta_2*alpha_1)/(beta_1*alpha_2-beta_2*alpha_1);
                z_0 = 0;
            }else
                continue;
        }else
            // t_1==0&&t_2==0&&t_3==0 means that the intersection planes of spheres 1-2 and spheres 1-3 are parallel, which means also sphere 2,3 overlap or centrosymmetric
            continue;

        // Intersection points parameters
        double a = t_1*t_1+t_2*t_2+t_3*t_3; // a == 0 if t_1,t_2,t_3 == 0
        double b = 2*(t_1*(x_0-a_1)+t_2*(y_0-b_1)+t_3*(z_0-c_1));
        double c = (x_0-a_1)*(x_0-a_1)+(y_0-b_1)*(y_0-b_1)+(z_0-c_1)*(z_0-c_1)-r_1*r_1;
        double judg = b*b-4*a*c;
        if (judg<0)
            judg = 0;
        else
            judg = sqrt(judg);
        double tau_1 = (-b + judg)/(2*a);
        double tau_2 = (-b - judg)/(2*a);
        // Intersection points
        double point_1[3] = {t_1*tau_1+x_0,t_2*tau_1+y_0,t_3*tau_1+z_0};
        buffer[i*21+12] = point_1[0];
        buffer[i*21+13] = point_1[1];
        buffer[i*21+14] = point_1[2];
        buffer[i*21+15] = 0;
        double point_2[3] = {t_1*tau_2+x_0,t_2*tau_2+y_0,t_3*tau_2+z_0};
        buffer[i*21+16] = point_2[0];
        buffer[i*21+17] = point_2[1];
        buffer[i*21+18] = point_2[2];
        buffer[i*21+19] = 0;
        // vote
        for(int j=3;j<uwb_tag_num;j++){
            double point_vote[3] = {Tag_pos[0*uwb_tag_num_max+(int)buffer[i*21+j]],Tag_pos[1*uwb_tag_num_max+(int)buffer[i*21+j]],Tag_pos[2*uwb_tag_num_max+(int)buffer[i*21+j]]};
            if(abs(vec_sqr_dist(point_vote,point_1,3)-dist[(int)buffer[i*21+j]]) > abs(vec_sqr_dist(point_vote,point_2,3)-dist[(int)buffer[i*21+j]]))
                buffer[i*21+19] += 1;
            else
                buffer[i*21+15] += 1;
        }
        // distance
        buffer[i*21+20]=vec_sqr_dist(point_1,point_2,3);
    }

    /************ collect ****************/
    double x[3] = {0,0,0};
    int weight_total=0;
    for(int i=0;i<num;i++){
        if(buffer[i*21+15]>=(uwb_tag_num-2)/2){
            x[0] += buffer[i*21+12]*buffer[i*21+15];
            x[1] += buffer[i*21+13]*buffer[i*21+15];
            x[2] += buffer[i*21+14]*buffer[i*21+15];
            weight_total += buffer[i*21+15];
        }else if(buffer[i*21+19]>=(uwb_tag_num-2)/2){
            x[0] += buffer[i*21+16]*buffer[i*21+19];
            x[1] += buffer[i*21+17]*buffer[i*21+19];
            x[2] += buffer[i*21+18]*buffer[i*21+19];
            weight_total += buffer[i*21+19];
        }
    }
    
    if(weight_total != 0){
		x[0] /= weight_total;
		x[1] /= weight_total;
		x[2] /= weight_total;
	}

    /************ update ****************/
    if(inited){
        State[3] = 0.6*State[3] + 0.4*0.8*(x[0]-State[0])/dt;
        State[4] = 0.6*State[4] + 0.4*0.8*(x[1]-State[1])/dt;
        State[5] = 0.6*State[5] + 0.4*0.8*(x[2]-State[2])/dt;
    }else{
        State[3] = 0;
        State[4] = 0;
        State[5] = 0;
        inited = true;
    }

    State[0] = 0.2*State[0] + 0.8*x[0];
    State[1] = 0.2*State[1] + 0.8*x[1];
    State[2] = 0.5*State[2] + 0.5*x[2];

    P[0] = p_p*p_p;
    P[dim+1] = p_p*p_p;
    P[2*dim+2] = p_p*p_p;
    P[3*dim+3] = q_v*q_v;
    P[4*dim+4] = q_v*q_v;
    P[5*dim+5] = q_v*q_v;

    delete[] buffer;
}

void UWBIndoorPosition::Predict(){
    /* Cholesky Decomposition */
    double A[dim*dim];
//    memset(&A, 0.0f, dim*dim * sizeof(double));
    for(int i=0;i<dim;i++)
        for(int j=0;j<dim;j++)
            A[i*dim+j] = 0.0;

    for (int i = 0; i < dim; i++)
        for (int j = 0; j < (i+1); j++) {
            double s = 0;
            for (int k = 0; k < j; k++)
                s += A[i * dim + k] * A[j * dim + k];

            if(i == j)
                if(P[i * dim + i] < s){
                    is_positive_definit = false;
                    A[i * dim + j] = 0.0;
                }else
                    A[i * dim + j] = sqrt(P[i * dim + i] - s);
            else
                A[i * dim + j] = 1.0 / A[j * dim + j] * (P[i * dim + j] - s);
            A[i * dim + j] = (i == j) ?
                           sqrt(P[i * dim + i] - s) :
                           (1.0 / A[j * dim + j] * (P[i * dim + j] - s));
        }

//    for(int i=0;i<dim;i++)
//        for(int j=0;j<dim;j++)
//            P[i*dim+j] = A[i*dim+j];
//    memcpy(&P, &A, dim * dim * sizeof(double));

    double foo = sqrt(dim+lambda);
//    for(int i=0;i<dim;i++)
//        for(int j=0;j<dim;j++)
//            A[i*dim+j] *= foo;

    /* Sigma Points */
    for(int i=0;i<dim;i++){
        sigma[i*(2*dim+1)] = State[i];
        for(int j=0;j<dim;j++){
            sigma[i*(2*dim+1)+j+1] = State[i] + A[i*dim+j] * foo;
            sigma[i*(2*dim+1)+j+1+dim] = State[i] - A[i*dim+j] * foo;
        }
    }

    /* Sigma Predict */
    if(dim == 9)
        for(int i=0;i<2*dim+1;i++){
            sigma[i]                += sigma[3*(2*dim+1)+i]*dt + sigma[6*(2*dim+1)+i]*dt*dt/2;
            sigma[(2*dim+1)+i]      += sigma[4*(2*dim+1)+i]*dt + sigma[7*(2*dim+1)+i]*dt*dt/2;
            sigma[2*(2*dim+1)+i]    += sigma[5*(2*dim+1)+i]*dt + sigma[8*(2*dim+1)+i]*dt*dt/2;
            sigma[3*(2*dim+1)+i]    += sigma[6*(2*dim+1)+i]*dt;
            sigma[4*(2*dim+1)+i]    += sigma[7*(2*dim+1)+i]*dt;
            sigma[5*(2*dim+1)+i]    += sigma[8*(2*dim+1)+i]*dt;
        }
    else if(dim == 6)
        for(int i=0;i<2*dim+1;i++){
            sigma[i]                += sigma[3*(2*dim+1)+i]*dt;
            sigma[(2*dim+1)+i]      += sigma[4*(2*dim+1)+i]*dt;
            sigma[2*(2*dim+1)+i]    += sigma[5*(2*dim+1)+i]*dt;
        }

}

void UWBIndoorPosition::ukf_update(double dist[]){
    /**************** Predict *******************/
    Predict();
    /***************** Measurement ********************/
    double Z[uwb_tag_num*(2*dim+1)];
    for(int i=0;i<uwb_tag_num;i++){
        for(int j=0;j<2*dim+1;j++){
            double sum=0.0;
            for(int k=0;k<3;k++){
                sum += (sigma[k*(2*dim+1)+j]-Tag_pos[k*uwb_tag_num_max+i])*(sigma[k*(2*dim+1)+j]-Tag_pos[k*uwb_tag_num_max+i]);
            }
            Z[i*(2*dim+1)+j] = sqrt(sum);
        }
    }

    double *K = new double[dim*uwb_tag_num];
    double *p_z_inv = new double[uwb_tag_num*uwb_tag_num];
    double *P_z = new double[uwb_tag_num*uwb_tag_num];

    // P_z = Z*W*Z' + R
    double *Z_trans = new double[(2*dim+1)*uwb_tag_num];
    for (uint8_t i = 0; i < uwb_tag_num; i++)
        for (uint8_t j = 0; j < (2*dim+1); j++)
                Z_trans[j * uwb_tag_num + i] = Z[i * (2*dim+1) + j];

    double *ZW = new double[uwb_tag_num*(2*dim+1)];
    for (uint8_t i = 0; i < uwb_tag_num; i++)
        for (uint8_t j = 0; j < (2*dim+1); j++){
                ZW[i * (2*dim+1) + j] = 0.0;
                for(uint8_t k=0;k<2*dim+1;k++)
                    ZW[i * (2*dim+1) + j] += Z[i*(2*dim+1)+k] * W[k*(2*dim+1)+j];
        }

    for (uint8_t i = 0; i < uwb_tag_num; i++)
        for (uint8_t j = 0; j < uwb_tag_num; j++){
                P_z[i * uwb_tag_num + j] = 0.0;
                for(uint8_t k=0;k<2*dim+1;k++)
                    P_z[i * uwb_tag_num + j] += ZW[i*(2*dim+1)+k] * Z_trans[k*uwb_tag_num+j];
        }
//    mat_transpose(Z_trans, Z, uwb_tag_num, 2*dim+1);
//    mat_mul(ZW, Z, W, uwb_tag_num, 2*dim+1, 2*dim+1);
//    mat_mul(P_z, ZW, Z_trans, uwb_tag_num, 2*dim+1, uwb_tag_num);

    delete[] ZW;
    ZW = NULL;

    for(int i=0;i<uwb_tag_num;i++)
        P_z[i*uwb_tag_num+i] += r*r;

    /***************** Matrix K ******************/
//    p_z_inv_able = mat_inverse(P_z,p_z_inv,uwb_tag_num);

    double a[64];
    for(int i=0;i<uwb_tag_num;i++)
        for(int j=0;j<uwb_tag_num;j++)
            a[i*uwb_tag_num+j] = P_z[i*uwb_tag_num+j];
    /* Augmenting Identity Matrix */
    for(int i=0;i<uwb_tag_num;i++)
         for(int j=0;j<uwb_tag_num;j++)
              p_z_inv[i*uwb_tag_num+j] = i==j? 1.0: 0.0;

    /* Applying Gauss Jordan Elimination */
    for(int i=0;i<uwb_tag_num;i++){
        if(1/a[i*uwb_tag_num+i] == (double) NAN){
            p_z_inv_able = false;
            break;
        }else
            for(int j=0;j<uwb_tag_num;j++)
                if(i!=j){
                    double ratio = a[j*uwb_tag_num+i]/a[i*uwb_tag_num+i];
                    for(int k=0;k<uwb_tag_num;k++){
                        p_z_inv[j*uwb_tag_num+k] -= ratio*p_z_inv[i*uwb_tag_num+k];
                        a[j*uwb_tag_num+k] -= ratio*a[i*uwb_tag_num+k];
                    }
                }
    }

    if(p_z_inv_able){
        /* Row Operation to Make Principal Diagonal to 1 */
        for(int i=0;i<uwb_tag_num;i++)
                for(int j=0;j<uwb_tag_num;j++)
                        p_z_inv[i*uwb_tag_num+j] /= a[i*uwb_tag_num+i];

        // K = Y*W*Z'/P_z
        double *YW = new double[dim*(2*dim+1)];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < 2*dim+1; j++){
                    YW[i * (2*dim+1) + j] = 0.0;
                    for(uint8_t k=0;k<2*dim+1;k++)
                        YW[i * (2*dim+1) + j] += sigma[i*(2*dim+1)+k] * W[k*(2*dim+1)+j];
            }
        double *YWZ = new double[dim*uwb_tag_num];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < uwb_tag_num; j++){
                    YWZ[i * uwb_tag_num + j] = 0.0;
                    for(uint8_t k=0;k<2*dim+1;k++)
                        YWZ[i * uwb_tag_num + j] += YW[i*(2*dim+1)+k] * Z_trans[k*uwb_tag_num+j];
            }
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < uwb_tag_num; j++){
                    K[i * uwb_tag_num + j] = 0.0;
                    for(uint8_t k=0;k<uwb_tag_num;k++)
                        K[i * uwb_tag_num + j] += YWZ[i*uwb_tag_num+k] * p_z_inv[k*uwb_tag_num+j];
            }
        delete[] YWZ;
        YWZ = NULL;
//        mat_mul(YW, sigma, W, dim, 2*dim+1, 2*dim+1);
//        mat_mul(K, YW, Z_trans, dim, 2*dim+1, uwb_tag_num);
//        mat_mul(K, K, p_z_inv, dim, uwb_tag_num, uwb_tag_num);

        /*************** Update State *****************
         *
         * State = State_bar + K*(z-z_bar)
         * State_bar = Y*Wm
         * z_bar = z*Wm
         *
         */
        double *State_bar = new double[dim];
        for (uint8_t i = 0; i < dim; i++){
            State_bar[i] = 0.0;
            for(uint8_t k=0;k<2*dim+1;k++)
                State_bar[i] += sigma[i*(2*dim+1)+k] * Wm[k];
        }

        double *z_bar = new double[uwb_tag_num];
        for (uint8_t i = 0; i < uwb_tag_num; i++){
            z_bar[i] = 0.0;
            for(uint8_t k=0;k<2*dim+1;k++)
                z_bar[i] += Z[i*(2*dim+1)+k] * Wm[k];
        }

        double *delta = new double[uwb_tag_num];
        for (uint8_t i = 0; i < uwb_tag_num; i++)
            delta[i] = dist[i] - z_bar[i];

        double *Kdelta = new double[dim];
        for (uint8_t i = 0; i < dim; i++){
            Kdelta[i] = 0.0;
            for(uint8_t k=0;k<uwb_tag_num;k++)
                Kdelta[i] += K[i*uwb_tag_num+k] * delta[k];
        }

        for (uint8_t i = 0; i < dim; i++)
                    State[i] = State_bar[i] + Kdelta[i];

//        double *S_update = new double[dim];
//        for (uint8_t i = 0; i < dim; i++)
//                    S_update[i] = State_bar[i] + Kdelta[i];

//        mat_mul(State_bar, sigma, Wm, dim, 2*dim+1, 1);
//        mat_mul(z_bar, Z, Wm, uwb_tag_num, 2*dim+1, 1);
//        mat_plus(delta, dist,z_bar,uwb_tag_num,1,false);
//        mat_mul(Kdelta, K, delta, dim, uwb_tag_num, 1);
//        mat_plus(S_update, State_bar, Kdelta, dim, 1, true);
//        memcpy(&State, S_update, dim * sizeof(double));
//        delete[] S_update;
//        S_update = NULL;

        delete[] State_bar;
        State_bar = NULL;
        delete[] z_bar;
        z_bar = NULL;
        delete[] delta;
        delta = NULL;
        delete[] Kdelta;
        Kdelta = NULL;

        /* Update Covariance
         *
         * P = P_bar - K*P_z*K'
         * P_bar = Y*W*Y' + Q
         *
         */
        double *Y_trans = new double[(2*dim+1)*dim];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < (2*dim+1); j++)
                    Y_trans[j * dim + i] = sigma[i * (2*dim+1) + j];

        double * P_bar = new double[dim*dim];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < dim; j++){
                    P_bar[i * dim + j] = 0.0;
                    for(uint8_t k=0;k<2*dim+1;k++)
                        P_bar[i * dim + j] += YW[i*(2*dim+1)+k] * Y_trans[k*dim+j];
            }

//        mat_transpose(Y_trans, sigma,dim, 2*dim+1);
//        mat_mul(YWY, YW, Y_trans, dim, 2*dim+1, dim);
//        memcpy(&P, YWY, dim * dim * sizeof(double));

        delete[] YW;
        YW = NULL;
        delete[] Y_trans;
        Y_trans = NULL;

        for(int i=0;i<dim;i++)
                P_bar[i*dim+i] += Q[i];

        double *K_trans = new double[dim*uwb_tag_num];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < uwb_tag_num; j++)
                    K_trans[j * dim + i] = K[i * uwb_tag_num + j];

        double *KPz = new double[dim*uwb_tag_num];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < uwb_tag_num; j++){
                    KPz[i * uwb_tag_num + j] = 0.0;
                    for(uint8_t k=0;k<uwb_tag_num;k++)
                        KPz[i * uwb_tag_num + j] += K[i*uwb_tag_num+k] * P_z[k*uwb_tag_num+j];
            }

        double *KPK = new double[dim*dim];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < dim; j++){
                    KPK[i * dim + j] = 0.0;
                    for(uint8_t k=0;k<uwb_tag_num;k++)
                        KPK[i * dim + j] += KPz[i*uwb_tag_num+k] * K_trans[k*dim+j];
            }

//        double *P_update = new double[dim*dim];
        for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < dim; j++)
                    P[i * dim + j] = P_bar[i*dim+j] - KPK[i*dim+j];

//        mat_transpose(K_trans, K,dim, uwb_tag_num);
//        mat_mul(KPz, K, P_z, dim, uwb_tag_num, uwb_tag_num);
//        mat_mul(KPK, KPz, K_trans, dim, uwb_tag_num, dim);
//        mat_plus(P_update, P, KPK, dim,dim,false);

//        memcpy(&P, P_update, dim * dim * sizeof(double));

        delete[] P_bar;
        P_bar = NULL;
        delete[] K_trans;
        K_trans = NULL;
        delete[] KPz;
        KPz = NULL;
        delete[] KPK;
        KPK = NULL;
//        delete[] P_update;
//        P_update = NULL;
    }

    delete[] K;
    delete[] p_z_inv;
    delete[] P_z;
    delete[] Z_trans;
    Z_trans = NULL;
    K = NULL;
    p_z_inv = NULL;
    P_z = NULL;

    if(is_positive_definit && p_z_inv_able){
        for (uint8_t i = 0; i < dim; i++) {
                for (uint8_t j = 0; j < dim; j++) {
                        if (P[i * dim + j] == (double) NAN) {
                                P_is_good = false;
                        }
                }
        }
    }
}

void UWBIndoorPosition::reset()
{
    for (uint8_t i = 0; i < dim; i++)
        State[i] = 0.0;

    for (uint8_t i = 0; i < dim; i++)
            for (uint8_t j = 0; j < dim; j++)
                    P[i * dim + j] = 0.0;

    for(int i=0;i<3;i++){
        P[i*dim+i] = p_p*p_p;
        if(dim>=6) P[(i+3)*dim+(i+3)] = p_v*p_v;
        if(dim>=9) P[(i+6)*dim+(i+6)] = p_a*p_a;
    }

    inited = false;
}

void UWBIndoorPosition::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

        perf_begin(_loop_perf);
        parameters_update(false);

        vehicle_odometry_s &visual_odom = _uwb_odom_pub.get();
        visual_odom.timestamp = hrt_absolute_time();

        uwb_msg_s uwb_msg;
        bool updated_uwb = _uwb_msg_sub.update(&uwb_msg);
        vehicle_attitude_s vehicle_attitude;
        bool updated_att = _vehicle_attitude_sub.update(&vehicle_attitude);

        if (updated_uwb || updated_att){

                double dist[uwb_tag_num];

                if(test){
                    double virtual_state[6] {0,0,0,0,0,0};
                    /* virtual measurements */
                    for(int j=0;j<uwb_tag_num;j++){
                            double sum=0.0;
                            for(int k=0;k<3;k++){
                                    sum += (virtual_state[k]-Tag_pos[k*uwb_tag_num_max+j]) * (virtual_state[k]-Tag_pos[k*uwb_tag_num_max+j]);
                            }
                            dist[j] = sqrt(sum) + 0.1*(2*static_cast<double>(rand()) / static_cast<double>(RAND_MAX)-1);
                    }

                    if(ukf_mode == 1)
                        ukf_update(dist);
                    else if(ukf_mode == 0)
                        trilateration(dist);

                    /* virtual state update */
                    virtual_state[0] += virtual_state[3]*dt;
                    virtual_state[1] += virtual_state[4]*dt;
                    virtual_state[2] += virtual_state[5]*dt;
                    virtual_state[3]  = 0.4*cos(virtual_state[2]-virtual_state[1]);
                    virtual_state[4]  = -0.4*sin(virtual_state[0])+virtual_state[5];
                    virtual_state[5]  = 0.1*sin(virtual_state[0])*cos(virtual_state[1]);

                }else if(updated_uwb){
                    visual_odom.timestamp_sample = uwb_msg.timestamp - 130000; // 130ms delay refered to mocap

                    for(int i=0;i<uwb_tag_num;i++){
                        dist[i] = uwb_msg.distance[i];
                    }

                    if(ukf_mode == 1){
                        if(initWithAnalytical && !inited){
                            trilateration(dist);
                        }

                        ukf_update(dist);

                        if(!check_healthy()){
                            PX4_ERR("chol decomposition faild! reset covariance matrix P");
                            reset();
                            trilateration(dist);
                            is_positive_definit = true;
                            p_z_inv_able = true;
                            P_is_good = true;
                        }

                    }else if(ukf_mode == 0)
                        trilateration(dist);
                }else
                    return;

                matrix::Quatf q_ev{vehicle_attitude.q};
//                q_ev.copyTo(visual_odom.q_offset); // body in NED
//                double phi, theta, psi;
//                matrix::Dcmf Body_in_NED = matrix::Dcmf(q_ev);
//                matrix::Dcmf Body_in_EV_Local_Frame = _R_EV_to_NED.transpose() * Body_in_NED; // body in EV_frame
//                q_ev = matrix::Quatf(Body_in_NED);
                q_ev.copyTo(visual_odom.q); // body in EV
//                phi = matrix::Eulerf(q).phi();
//                theta = matrix::Eulerf(q).theta();
//                psi = matrix::Eulerf(q).psi();


                matrix::Vector3f _position(State[0],State[1],State[2]);
                _position = _R_EV_to_NED * _position;
                visual_odom.x = _position(0);
                visual_odom.y = _position(1);
                visual_odom.z = _position(2)-pos_z;

                visual_odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED; // Do not need to enable rotate external vision in EKF2_AID_MASK

                visual_odom.pose_covariance[0] = (float)P[0];
                visual_odom.pose_covariance[1] = (float)P[1];
                visual_odom.pose_covariance[2] = (float)P[2];

                visual_odom.pose_covariance[6] = (float)P[dim+1];
                visual_odom.pose_covariance[7] = (float)P[dim+2];

                visual_odom.pose_covariance[11] = (float)P[2*dim+2];

                visual_odom.pose_covariance[15] = NAN;
                visual_odom.pose_covariance[18] = NAN;
                visual_odom.pose_covariance[20] = NAN;

                visual_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED; // DO not need to enable rotate external vision in EKF2_AID_MASK
                matrix::Vector3f _vel(State[3],State[4],State[5]);
                _vel = _R_EV_to_NED * _vel;
                visual_odom.vx = _vel(0);
                visual_odom.vy = _vel(1);
                visual_odom.vz = _vel(2);
                visual_odom.rollspeed = NAN;
                visual_odom.pitchspeed = NAN;
                visual_odom.yawspeed = NAN;
                if(!p_z_inv_able)
                    visual_odom.rollspeed = 0;
                if(!P_is_good)
                    visual_odom.pitchspeed = 0;
                if(!is_positive_definit)
                    visual_odom.yawspeed = 0;


                visual_odom.velocity_covariance[0] = (float)P[3*dim+3];
                visual_odom.velocity_covariance[1] = (float)P[3*dim+4];
                visual_odom.velocity_covariance[2] = (float)P[3*dim+5];

                visual_odom.velocity_covariance[6] = (float)P[4*dim+4];
                visual_odom.velocity_covariance[7] = (float)P[4*dim+5];

                visual_odom.velocity_covariance[11] = (float)P[5*dim+5];

                visual_odom.velocity_covariance[15] = NAN;
                visual_odom.velocity_covariance[18] = NAN;
                visual_odom.velocity_covariance[20] = NAN;

                _uwb_odom_pub.update();
                perf_count(_loop_interval_perf);
        }

        updated_uwb = false;
        updated_att = false;
	perf_end(_loop_perf);
}

int UWBIndoorPosition::task_spawn(int argc, char *argv[])
{
        UWBIndoorPosition *instance = new UWBIndoorPosition();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
                        PX4_INFO("uwb indoor position started");
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

    PRINT_MODULE_USAGE_NAME("uwb_positioner", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uwb_positioner_main(int argc, char *argv[])
{
        return UWBIndoorPosition::main(argc, argv);
}
