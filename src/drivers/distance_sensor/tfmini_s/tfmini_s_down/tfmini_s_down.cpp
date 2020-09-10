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

#include "tfmini_s_down.hpp"

#include <lib/parameters/param.h>

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

	return crc & 0xFF;
}

tfmini_s_down::tfmini_s_down(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency) :
	I2C(DRV_DIST_DEVTYPE_TFMINI_S_D, MODULE_NAME, bus, TFMINI_S_D_ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
        _px4_rangefinder(get_device_id(), rotation)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

tfmini_s_down::~tfmini_s_down()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int tfmini_s_down::collect()
{
	if (!_collect_phase) {
		return measure();
	}

	perf_begin(_sample_perf);

	// Transfer data from the bus.
	uint8_t val[9] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	set_device_address(TFMINI_S_D_ADDR);
	int ret_val = transfer(nullptr, 0, &val[0], 9);

	if (ret_val < 0) {
		PX4_ERR("error reading from sensor: 0x%02x", TFMINI_S_D_ADDR);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret_val;
	}
	if (val[0] != 0x59 || val[1] != 0x59){
		PX4_ERR("error reading from sensor: 0x%02X 0x%02X", val[0], val[1]);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return measure();
	}
	uint16_t distance_cm = (val[3] << 8) | val[2];
	float distance_m = static_cast<float>(distance_cm) * 1e-2f;
	int strength = (val[5] << 8) | val[4];
	//uint16_t temperature = ((val[7] << 8) | val[6]) / 8 - 256;

	// PX4_ERR("distance %d cm", distance_cm);
	// Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
	// Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
	uint8_t signal_quality = 100 * strength / 65535.0f;

	// Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
	if (distance_m < TFMINI_S_MIN_DISTANCE)
		 signal_quality = 0;
	if((strength == -1) || (strength == -2))
		 signal_quality = strength;

	if (crc8(val, 8) == val[8]) {
		 _px4_rangefinder.update(timestamp_sample, distance_m,signal_quality);
		 //PX4_INFO("Tested data as %f m",double(distance_m));
	}

	// Next phase is measurement.
	_collect_phase = false;

	perf_count(_sample_perf);
	perf_end(_sample_perf);

	return PX4_OK;
}

int tfmini_s_down::init()
{
	int32_t hw_enable = 0;
	param_get(param_find("SENS_EN_TFMINI_D"), &hw_enable);

	if (hw_enable){
		int32_t address = TFMINI_S_D_ADDR;
		set_device_address(address);
		if (I2C::init() != OK) {
		PX4_DEBUG("initialisation failed at i2c address 0x%02x",address);
		return PX4_ERROR;

		} else {
			PX4_DEBUG("Enabled the sensor at 0x%02x",address);
			// Assume minimum and maximum possible distances acros Evo family
			_px4_rangefinder.set_min_distance(TFMINI_S_MIN_DISTANCE);
			_px4_rangefinder.set_max_distance(TFMINI_S_MAX_DISTANCE);
		}
		return PX4_OK;
	}else{
		PX4_WARN("Disabled");
		return PX4_ERROR;
	}

}

int tfmini_s_down::measure()
{
	// Send the command to begin a measurement.
	const uint8_t cmd[] = {0x5A,0x05,0x00,0x01,0x60};
	int ret_val = transfer(&cmd[0], 5, nullptr, 0);

	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	_collect_phase = true;
	return PX4_OK;
}

int tfmini_s_down::probe()
{
	return measure();
}

void tfmini_s_down::RunImpl()
{
	// Perform data collection.
	collect();
}

void tfmini_s_down::start()
{
	_collect_phase = false;

	// Schedule the driver to run on a set interval
	ScheduleOnInterval(TFMINI_S_MEASUREMENT_INTERVAL);
}

void tfmini_s_down::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
