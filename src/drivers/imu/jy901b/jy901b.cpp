/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "jy901b.hpp"
#include <lib/parameters/param.h>

#include <fcntl.h>

JY901B::JY901B(const char *port) :
        ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
        _jy901b_msg_pub.advertise();

	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

JY901B::~JY901B()
{
        _jy901b_msg_pub.unadvertise();

	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
JY901B::init()
{
        int32_t hw_model = 1; // only one model so far...
        param_get(param_find("SENS_EN_JY901B"), &hw_model);
        if(!hw_model)
        {
                PX4_ERR("invalid HW model %d.", hw_model);
                return -1;
        }

        start();

        return PX4_OK;
}

int
JY901B::collect()
{
        perf_begin(_sample_perf);
        jy901b_msg_s report{};
        bool acc_valid  = false;
        bool gyro_valid =false;
        bool baro_valid = false;
        bool q_valid = false;
        uint8_t data = 0x00;
        uint8_t cnt =0;
        // Read from the sensor UART buffer.
        while(::read(_file_descriptor, &data, 1)>0)
        {
                if(data != 0x55) {continue;}
                ::read(_file_descriptor, &data, 1);
                if(data != 0x51 && data != 0x52 && data != 0x53 && data != 0x56 && data != 0x59) {continue;}
                _linebuf[0] = 0x55;
                _linebuf[1] = data;
                ::read(_file_descriptor, &_linebuf[2], 9);
                switch(data)
                {
                        case 0x51:
                                for(int i=0;i<11;i++)
                                        acc_linebuf[i] = _linebuf[i];
                                memcpy(&stcAcc,&_linebuf[2],8);
                                report.acc[0] = (float)stcAcc.a[0]/32768.0f*16.0f;
                                report.acc[1] = (float)stcAcc.a[1]/32768.0f*16.0f;
                                report.acc[2] = (float)stcAcc.a[2]/32768.0f*16.0f;
                                report.temp = (float)stcAcc.T/100.0f;
                                acc_valid = true;id_acc = cnt;
                                break;
                        case 0x52:
                                for(int i=0;i<11;i++)
                                        gyro_linebuf[i] = _linebuf[i];
                                memcpy(&stcGyro,&_linebuf[2],8);
                                report.gyro[0] = (float)stcGyro.w[0]/32768.0f*16.0f;
                                report.gyro[1] = (float)stcGyro.w[1]/32768.0f*16.0f;
                                report.gyro[2] = (float)stcGyro.w[2]/32768.0f*16.0f;
                                gyro_valid = true;id_gyro = cnt;
                                break;
                        case 0x53:
                                for(int i=0;i<11;i++)
                                        euler_linebuf[i] = _linebuf[i];
                                memcpy(&stcAngle,&euler_linebuf[2],8);
                                report.euler[0] = (float)stcAngle.Angle[0]/32768.0f*180.0f;
                                report.euler[1] = (float)stcAngle.Angle[1]/32768.0f*180.0f;
                                report.euler[2] = (float)stcAngle.Angle[2]/32768.0f*180.0f;
                                q_valid = true;id_euler = cnt;
                                break;
                        case 0x56:
                                for(int i=0;i<11;i++)
                                        baro_linebuf[i] = _linebuf[i];
                                memcpy(&stcPress,&_linebuf[2],8);
                                report.baro = stcPress.lPressure;
                                baro_valid = true;id_baro = cnt;
                                break;
                        case 0x59:
                                for(int i=0;i<11;i++)
                                        q_linebuf[i] = _linebuf[i];
                                memcpy(&stcQ,&_linebuf[2],8);
                                report.q[0] = (float)stcQ.q[0]/32768.0f;
                                report.q[1] = (float)stcQ.q[1]/32768.0f;
                                report.q[2] = (float)stcQ.q[2]/32768.0f;
                                report.q[3] = (float)stcQ.q[3]/32768.0f;
                                q_valid = true;id_q = cnt;
                                break;
                        default:
                                for(int i=0;i<11;i++)
                                        oth_linebuf[i] = _linebuf[i];
                                perf_count(_comms_errors);
                                break;
                }
                cnt++;
        }

        if (acc_valid && gyro_valid && baro_valid && q_valid) {
                // publish most recent valid measurement from buffer
                report.timestamp = hrt_absolute_time();
                _jy901b_msg_pub.publish(report);

                perf_end(_sample_perf);
                return PX4_OK;
        }
        return -EAGAIN;
}

int
JY901B::open_serial_port(const speed_t speed)
{

        // File descriptor initialized?
        if (_file_descriptor > 0) {

                // PX4_INFO("serial port already open");
                return PX4_OK;
        }
        // Configure port flags for read/write, non-controlling, non-blocking.
        int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

        // Open the serial port.
        _file_descriptor = ::open(_port, flags);

        if (_file_descriptor < 0) {

                PX4_ERR("open failed (%i)", errno);
                return PX4_ERROR;
        }

        termios uart_config = {};

        // Store the current port configuration. attributes.
        tcgetattr(_file_descriptor, &uart_config);

        // Clear ONLCR flag (which appends a CR for every LF).
        uart_config.c_oflag &= ~ONLCR;

        // No parity, one stop bit.
        uart_config.c_cflag &= ~(CSTOPB | PARENB);

        // Set the input baud rate in the uart_config struct.
        int termios_state = cfsetispeed(&uart_config, speed);

        if (termios_state < 0) {

                PX4_ERR("CFG: %d ISPD", termios_state);
                ::close(_file_descriptor);
                return PX4_ERROR;
        }

        // Set the output baud rate in the uart_config struct.
        termios_state = cfsetospeed(&uart_config, speed);

        if (termios_state < 0) {

                PX4_ERR("CFG: %d OSPD", termios_state);
                ::close(_file_descriptor);
                return PX4_ERROR;
        }

        // Apply the modified port attributes.
        termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

        if (termios_state < 0) {

                PX4_ERR("baud %d ATTR", termios_state);
                ::close(_file_descriptor);
                return PX4_ERROR;
        }

        PX4_INFO("successfully opened UART port %s", _port);
        return PX4_OK;
}

void
JY901B::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(_MEASURE_INTERVAL);

    PX4_INFO("driver started");
}

void
JY901B::stop()
{
    // Clear the work queue schedule.
    ScheduleClear();

    // Ensure the serial port is closed.
    ::close(_file_descriptor);
}

void
JY901B::Run()
{
    // Ensure the serial port is open.
    open_serial_port();

    // Perform collection.
    collect();
}

void
JY901B::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
        perf_print_counter(_comms_errors);
        PX4_INFO("id_acc:%d",id_acc);
        PX4_INFO("id_gyro:%d",id_gyro);
        PX4_INFO("id_euler:%d",id_euler);
        PX4_INFO("id_baro:%d",id_baro);
        PX4_INFO("id_q:%d",id_q);
        // PX4_INFO("_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", _linebuf[0],_linebuf[1],_linebuf[2],_linebuf[3],_linebuf[4],_linebuf[5],_linebuf[6],_linebuf[7],_linebuf[8],_linebuf[9],_linebuf[10]);
        PX4_INFO("acc_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", acc_linebuf[0],acc_linebuf[1],acc_linebuf[2],acc_linebuf[3],acc_linebuf[4],acc_linebuf[5],acc_linebuf[6],acc_linebuf[7],acc_linebuf[8],acc_linebuf[9],acc_linebuf[10]);
        PX4_INFO("gyro_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", gyro_linebuf[0],gyro_linebuf[1],gyro_linebuf[2],gyro_linebuf[3],gyro_linebuf[4],gyro_linebuf[5],gyro_linebuf[6],gyro_linebuf[7],gyro_linebuf[8],gyro_linebuf[9],gyro_linebuf[10]);
        PX4_INFO("euler_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", euler_linebuf[0],euler_linebuf[1],euler_linebuf[2],euler_linebuf[3],euler_linebuf[4],euler_linebuf[5],euler_linebuf[6],euler_linebuf[7],euler_linebuf[8],euler_linebuf[9],euler_linebuf[10]);
        PX4_INFO("baro_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", baro_linebuf[0],baro_linebuf[1],baro_linebuf[2],baro_linebuf[3],baro_linebuf[4],baro_linebuf[5],baro_linebuf[6],baro_linebuf[7],baro_linebuf[8],baro_linebuf[9],baro_linebuf[10]);
        PX4_INFO("q_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", q_linebuf[0],q_linebuf[1],q_linebuf[2],q_linebuf[3],q_linebuf[4],q_linebuf[5],q_linebuf[6],q_linebuf[7],q_linebuf[8],q_linebuf[9],q_linebuf[10]);
        PX4_INFO("oth_linebuf:\t%x %x %x %x %x %x %x %x %x %x %x", oth_linebuf[0],oth_linebuf[1],oth_linebuf[2],oth_linebuf[3],oth_linebuf[4],oth_linebuf[5],oth_linebuf[6],oth_linebuf[7],oth_linebuf[8],oth_linebuf[9],oth_linebuf[10]);
        // PX4_INFO("acc:\t%f\t%f\t%f", double(report.acc[0]),double(report.acc[1]),double(report.acc[2]));
        // PX4_INFO("gyro:\t%f\t%f\t%f", double(report.gyro[0]),double(report.gyro[1]),double(report.gyro[2]));
        // PX4_INFO("baro:\t%ld", long(report.baro));
        // PX4_INFO("euler:\t%f\t%f\t%f", double(report.euler[0]),double(report.euler[1]),double(report.euler[2]));
        // PX4_INFO("quaternion:\t%f\t%f\t%f\t%f", double(report.q[0]),double(report.q[1]),double(report.q[2]),double(report.q[3]));
}
