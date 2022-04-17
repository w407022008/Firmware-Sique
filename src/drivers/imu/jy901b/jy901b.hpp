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
 * @file jy901b.cpp
 * @author Ze Wang <wang.ze@isir.upmc.fr>
 *
 * Driver for the JY901B IMU series
 */

#pragma once

#include <termios.h>
#include <string.h>
#include <stdlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_jy901b.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/jy901b_msg.h>

#define JY901B_DEFAULT_PORT	"/dev/ttyS3"


using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t _MEASURE_INTERVAL{5000};	// 5ms default sensor conversion time.


class JY901B : public px4::ScheduledWorkItem
{
public:
    /**
     * Default Constructor
     * @param port The serial port to open for communicating with the sensor.
     */
	JY901B(const char *port);

    /** Virtual destructor */
	virtual ~JY901B();

    /**
     * Method : init()
     * This method initializes the general driver for a jy901b sensor.
     */
	int init();

    /**
     * Diagnostics - print some basic information about the driver.
     */
	void print_info();

private:

    /**
     * Reads data from serial UART and places it into a buffer.
     */
	int collect();

    /**
     * Opens and configures the UART serial communications port.
     * @param speed The baudrate (speed) to configure the serial UART port.
     */
    int open_serial_port(const speed_t speed = B921600);

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

    struct STime
    {
        unsigned char ucYear;
        unsigned char ucMonth;
        unsigned char ucDay;
        unsigned char ucHour;
        unsigned char ucMinute;
        unsigned char ucSecond;
        unsigned short usMiliSecond;
    } stcTime;
    struct SAcc
    {
        short a[3];
        short T;
    } stcAcc;
    struct SGyro
    {
        short w[3];
        short T;
    } stcGyro;
    struct SAngle
    {
        short Angle[3];
        short T;
    } stcAngle;
    struct SMag
    {
        short h[3];
        short T;
    } stcMag;

    struct SDStatus
    {
        short sDStatus[4];
    } stcDStatus;

    struct SPress
    {
        long lPressure;
        long lAltitude;
    } stcPress;

    struct SLonLat
    {
        long lLon;
        long lLat;
    } stcLonLat;

    struct SGPSV
    {
        short sGPSHeight;
        short sGPSYaw;
        long lGPSVelocity;
    } stcGPSV;
    struct SQ
    {
        short q[4];
    } stcQ;

    uint8_t _linebuf[11] {};
    uint8_t acc_linebuf[11] {};
    uint8_t gyro_linebuf[11] {};
    uint8_t euler_linebuf[11] {};
    uint8_t baro_linebuf[11] {};
    uint8_t q_linebuf[11] {};
    uint8_t oth_linebuf[11] {};

    uint8_t id_acc, id_gyro, id_euler, id_baro, id_q;

    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

    uORB::Publication<jy901b_msg_s> _jy901b_msg_pub{ORB_ID(jy901b_msg)};
};
