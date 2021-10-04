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

/**
 * @file arduino.cpp
 * @author Ze Wang <wang.ze@isir.upmc.fr>
 *
 * Driver for the Arduino UART Communication
 */

#pragma once

#include <termios.h>
#include <string.h>
#include <stdlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_arduino.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/arduino_msg.h>

#define ARDUINO_DEFAULT_PORT	"/dev/ttyS3"

#ifdef ARDUINO_DEBUG
#include <stdio.h>

const char *parser_state[] = {

};
#endif

using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t _MEASURE_INTERVAL{10_ms};	// 10ms default sensor conversion time.

/* Frame start delimiter */
static constexpr unsigned char START_FLAG{'s'};

static constexpr uint8_t PARSER_BUF_LENGTH{36};

class ARDUINO : public px4::ScheduledWorkItem
{
public:
    /**
     * Default Constructor
     * @param port The serial port to open for communicating with the sensor.
     */
        ARDUINO(const char *port);

    /** Virtual destructor */
        virtual ~ARDUINO();

    /**
     * Method : init()
     * This method initializes the general driver for arduino .
     */
	int init();

    /**
     * Diagnostics - print some basic information about the driver.
     */
	void print_info();

private:
    // Data Format for Aruidno UART Communication
    // ===============================
    // 9 bytes total per message:
    // 1) m (1b)
    // 2) c (1b)
    // 3) Dist (8b)
    // 4) Tag (1b)
    // 5) Anchor (1b)

//    enum class ARDUINO_PARSE_STATE {
//    };

    /**
     * Reads data from serial UART and places it into a buffer.
     */
	int collect();

    /**
     * Opens and configures the UART serial communications port.
     * @param speed The baudrate (speed) to configure the serial UART port.
     */
    int open_serial_port(const speed_t speed = B115200);

    /**
     * Perform a reading cycle; collect from the previous measurement
     * and start a new one.
     */
	void Run() override;

    /**
     * Initialise the automatic measurement state machine and start it.
     * @note This function is called at open and error time.  It might make sense
     *       to make it more aggressive about resetting the bus in case of errors.
     */
	void start();

    /**
     * Stops the automatic measurement state machine.
     */
	void stop();

    char _port[20] {};

    int _file_descriptor{-1};

    uint8_t _linebuf[PARSER_BUF_LENGTH] {};

    int32_t en_airflow = 0;
    int32_t en_force = 0;

    int32_t airflow_sensor_num=3; // default: 3; max: 8

    bool _valid_airflow = false;
    bool _valid_force = false;

    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

    arduino_msg_s report{};
    uORB::Publication<arduino_msg_s> _arduino_msg_pub{ORB_ID(arduino_msg)};
};
