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

#include "ychiot.hpp"
#include <lib/parameters/param.h>

#include <fcntl.h>

YCHIOT::YCHIOT(const char *port) :
        ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
        _uwb_msg_pub.advertise();

	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

YCHIOT::~YCHIOT()
{
        _uwb_msg_pub.unadvertise();

	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
YCHIOT::init()
{
        int32_t hw_model = 1; // only one model so far...
        param_get(param_find("SENS_EN_YCHIOT"), &hw_model);
        switch (hw_model) {
            case 1: // YCHIOT (12 Hz)
                // set landmark position
                param_get(param_find("SENS_UWB_TAG_NUM"), &tag_num);
                break;

            default:
                PX4_ERR("invalid HW model %d.", hw_model);
                return -1;
        }

        start();

        return PX4_OK;
}

int
YCHIOT::collect()
{
        perf_begin(_sample_perf);
        distance_mm = -1;
        _valid = false;
        // Read from the sensor UART buffer.
        const hrt_abstime timestamp_sample = hrt_absolute_time();

        uint8_t data;
        ::read(_file_descriptor, &data, 1);
        if(data==START_FRAME_DIGIT2){
            _linebuf[0]=START_FRAME_DIGIT1;
            _linebuf[1]=START_FRAME_DIGIT2;
            report.buffer[0]=START_FRAME_DIGIT1;
            report.buffer[1]=START_FRAME_DIGIT2;
            ::read(_file_descriptor, &_linebuf[2], 34);
        }else{
            return PX4_OK;
        }

        for(int k=0;k<tag_num;k++){
            int val = 0;
            bool good = 1;
            for(int i=2+4*k; i<6+4*k; i++){
                val *= 16;
                if(_linebuf[i]>='0' && _linebuf[i]<='9')
                  val += _linebuf[i]-'0';
                else if(_linebuf[i]>='a' && _linebuf[i]<='f')
                  val += _linebuf[i]-'a'+10;
                else if(_linebuf[i]>='A' && _linebuf[i]<='F')
                  val += _linebuf[i]-'A'+10;
                else{
                  good=0;
                }
                report.buffer[i] = _linebuf[i];
            }

            if(good){
                report.distance[k] = static_cast<float>(val) / 1000.0f;
                _valid = true;
            }
        }

        while(::read(_file_descriptor, &data, 1)>0);

        if (_valid || distance_mm>0) {
            // publish most recent valid measurement from buffer
            report.timestamp = timestamp_sample;

            _uwb_msg_pub.publish(report);

            perf_end(_sample_perf);

            return PX4_OK;
        }
        return -EAGAIN;
}

int
YCHIOT::data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH], YCHIOT_PARSE_STATE &state, int &dist, int &tag, int &anchor)
{
        switch (state) {
        case YCHIOT_PARSE_STATE::DIGIT_1:
            if (check_byte == START_FRAME_DIGIT1) {
                    state = YCHIOT_PARSE_STATE::DIGIT_1;

            } else if (check_byte == START_FRAME_DIGIT3) {
                state = YCHIOT_PARSE_STATE::DIGIT_1;

            } else if (check_byte == START_FRAME_DIGIT4) {
                state = YCHIOT_PARSE_STATE::DIGIT_1;

            } else if (check_byte == START_FRAME_DIGIT2) {
                    state = YCHIOT_PARSE_STATE::DIGIT_2;

            }

            break;

        case YCHIOT_PARSE_STATE::DIGIT_2:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_0;
                parserbuf[2] = check_byte; // Data_0

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_0:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_1;
                parserbuf[3] = check_byte; // Data_1

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_1:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_2;
                parserbuf[4] = check_byte; // Data_2

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_2:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_3;
                parserbuf[5] = check_byte; // Data_3

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_3:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_4;
                parserbuf[6] = check_byte; // Data_4

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_4:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_5;
                parserbuf[7] = check_byte; // Data_5

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_5:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_6;
                parserbuf[8] = check_byte; // Data_6

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_6:
                state = YCHIOT_PARSE_STATE::STATE2_GOT_DIST_7;
                parserbuf[9] = check_byte; // Data_7

                break;

        case YCHIOT_PARSE_STATE::STATE2_GOT_DIST_7:
                state = YCHIOT_PARSE_STATE::STATE3_GOT_TAG_NUM;
                parserbuf[10] = check_byte; // tag_index

                break;

        case YCHIOT_PARSE_STATE::STATE3_GOT_TAG_NUM:
                state = YCHIOT_PARSE_STATE::STATE4_GOT_ANCHOR_NUM;
                parserbuf[11] = check_byte; // anchor_index

                break;

        case YCHIOT_PARSE_STATE::STATE4_GOT_ANCHOR_NUM:
                // Here, reset state to `NOT-STARTED` no matter crc ok or not
                state = YCHIOT_PARSE_STATE::DIGIT_1;

                int val = 0;
                bool good = 1;
                for(int i=6; i<10; i++){
                    val *= 16;
                    if(parserbuf[i]>='0' && parserbuf[i]<='9')
                      val += parserbuf[i]-'0';
                    else if(parserbuf[i]>='a' && parserbuf[i]<='f')
                      val += parserbuf[i]-'a'+10;
                    else if(parserbuf[i]>='A' && parserbuf[i]<='F')
                      val += parserbuf[i]-'A'+10;
                    else{
                      good=0;
                      break;
                    }
                }

                if(good && parserbuf[10]>='0' && parserbuf[10]<='9' && parserbuf[11]>='0' && parserbuf[11]<='9'){
                    tag = parserbuf[10]-'0';
                    anchor = parserbuf[11]-'0';
                } else {
                    good=0;
                }

                if(good){
                    dist = val;
                    state = YCHIOT_PARSE_STATE::DIGIT_1;
                    return PX4_OK;
                }
                break;
        }

#ifdef YCHIOT_DEBUG
        printf("state: YCHIOT_PARSE_STATE%s\n", parser_state[state]);
#endif

        return PX4_ERROR;
}

int
YCHIOT::open_serial_port(const speed_t speed)
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
YCHIOT::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(_MEASURE_INTERVAL);

    PX4_INFO("driver started");
}

void
YCHIOT::stop()
{
    // Clear the work queue schedule.
    ScheduleClear();

    // Ensure the serial port is closed.
    ::close(_file_descriptor);
}

void
YCHIOT::Run()
{
    // Ensure the serial port is open.
    open_serial_port();

    // Perform collection.
    collect();
}

void
YCHIOT::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
        perf_print_counter(_comms_errors);
        for(int i=0;i<10;i++)
            PX4_INFO("buffer:\t%s", _linebuf);
//            if (report.timestamp != 0)
//                print_message(report);

}
