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

#include "arduino.hpp"
#include <lib/parameters/param.h>

#include <fcntl.h>

ARDUINO::ARDUINO(const char *port) :
        ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
        _arduino_msg_pub.advertise();

	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

ARDUINO::~ARDUINO()
{
        _arduino_msg_pub.unadvertise();

	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
ARDUINO::init()
{
        int32_t hw_model = 1; // only one model so far...
        param_get(param_find("SENS_EN_ARDUINO"), &hw_model);
        if(hw_model){ // ARDUINO (12 Hz)
            param_get(param_find("SENS_EN_AIRFLOW"), &en_airflow);
            param_get(param_find("SENS_EN_FORCE"), &en_force);
        } else{
            PX4_ERR("invalid HW model %d.", hw_model);
            return -1;
        }

        start();

        return PX4_OK;
}

int
ARDUINO::collect()
{
        perf_begin(_sample_perf);
        _valid_airflow = false;
        // Read from the sensor UART buffer.
        const hrt_abstime timestamp_sample = hrt_absolute_time();

        uint8_t data;
        ::read(_file_descriptor, &data, 1);
        if(data==START_FLAG){
            _linebuf[0]=START_FLAG;
            ::read(_file_descriptor, &_linebuf[1], 34);
        }else{
            return PX4_OK;
        }

        // airflow sensor
        if(en_airflow){
            for(int k=0;k<2*airflow_sensor_num;k++){
                int val = 0;
                int sign = 1;
                bool good = 1;
                for(int i=1+8*k; i<5+8*k; i++){
                    val *= 16;
                    if(_linebuf[i]=='n' || _linebuf[i]=='p'){
                        if(_linebuf[i]=='n')
                            sign = -1;
                    }else if(_linebuf[i]>='0' && _linebuf[i]<='9')
                      val += _linebuf[i]-'0';
                    else if(_linebuf[i]>='a' && _linebuf[i]<='f')
                      val += _linebuf[i]-'a'+10;
                    else if(_linebuf[i]>='A' && _linebuf[i]<='F')
                      val += _linebuf[i]-'A'+10;
                    else{
                      good=0;
                    }
                }

                if(good){
                    report.airflow_sensor_num = airflow_sensor_num;
                    report.current[k] = static_cast<float>(sign*val) / 1000.0f;
                    val = 0;
                    sign = 1;
                    for(int i=5+8*k; i<9+8*k; i++){
                        val *= 16;
                        if(_linebuf[i]=='n' || _linebuf[i]=='p'){
                            if(_linebuf[i]=='n')
                                sign = -1;
                        }else if(_linebuf[i]>='0' && _linebuf[i]<='9')
                          val += _linebuf[i]-'0';
                        else if(_linebuf[i]>='a' && _linebuf[i]<='f')
                          val += _linebuf[i]-'a'+10;
                        else if(_linebuf[i]>='A' && _linebuf[i]<='F')
                          val += _linebuf[i]-'A'+10;
                        else{
                          good=0;
                        }

                        if(good){
                            report.voltage[k] = static_cast<float>(sign*val) / 1000.0f;
                            _valid_airflow = true;
                        }
                    }
                }
            }

        }

        // force sensor
        if(en_force){

        }

        while(::read(_file_descriptor, &data, 1)>0);

        if (_valid_airflow || _valid_force) {
            // publish most recent valid measurement from buffer
            report.timestamp = timestamp_sample;

            _arduino_msg_pub.publish(report);

            perf_end(_sample_perf);

            return PX4_OK;
        }
        return -EAGAIN;
}

int
ARDUINO::open_serial_port(const speed_t speed)
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
ARDUINO::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(_MEASURE_INTERVAL);

    PX4_INFO("driver started");
}

void
ARDUINO::stop()
{
    // Clear the work queue schedule.
    ScheduleClear();

    // Ensure the serial port is closed.
    ::close(_file_descriptor);
}

void
ARDUINO::Run()
{
    // Ensure the serial port is open.
    open_serial_port();

    // Perform collection.
    collect();
}

void
ARDUINO::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
        perf_print_counter(_comms_errors);
        for(int i=0;i<10;i++)
            PX4_INFO("buffer:\t%s", _linebuf);
//            if (report.timestamp != 0)
//                print_message(report);

}
