/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file sfm3000.cpp
 * @author Ze WANG
 *
 * Driver for the SFM3000 Anemometer connected via I2C.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/anemometer/PX4Anemometer.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

using namespace time_literals;

/* Configuration Constants */
#define TCA_ADDR						0x70
#define SFM_BASEADDR                 	0x40 // 7-bit address.

/* Device limits */
#define SFM_MEASUREMENT_INTERVAL         1000 // us

#define ANEMOMETER_MAX_SENSORS           	3	// Maximum number of sensors on bus
#define TCA9578A_MAX_CHANAL					8	// Maximum number of chanals of multiplexer TCA9578A

class sfm3000 : public device::I2C, public I2CSPIDriver<sfm3000>
{
public:
    sfm3000(I2CSPIBusOption bus_option, const int bus, const uint16_t address, const uint8_t rotation, int bus_frequency);
    ~sfm3000() override;

    static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance);
    static void print_usage();

    virtual int init() override;
    void print_status() override;

    void RunImpl();
protected:

    virtual int probe() override;

private:
    void start();
    int collect();
    int measure();

    /**
    * Test whether the device supported by the driver is present at a
    * specific address.
    *
    * @param address The I2C bus address to probe.
    * @return True if the device is present.
    */
    int probe_address(const uint8_t address);


    PX4Anemometer _px4_anemometer;

    float offset{32000.0f};
    float scale{140.0f};
    float slm2ms{0.0546808f};

    /**
     * Gets the current sensor rotation value.
     */
    int get_sensor_rotation(const size_t index);

    size_t	_sensor_count{0};
    uint8_t _sensor_chanal[ANEMOMETER_MAX_SENSORS];
    uint8_t _sensor_rotations[ANEMOMETER_MAX_SENSORS];

    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comm_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
