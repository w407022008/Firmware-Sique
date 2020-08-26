/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file tfmini_s.cpp
 *
 * Driver for the downward facing TFmini-S connected via I2C.
 *
 * @author Ze WANG
 */


#include <px4_platform_common/defines.h>
#include <drivers/drv_device.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>

#include <lib/parameters/param.h>


/*
 * TFmini-S internal constants and data structures.
 */
#define TFMINI_S_DOWNWARD_I2C_ADDR        0x10
#define TFMINI_S_FORWARD_I2C_ADDR             0x11
#define TFMINI_S_LEFT_I2C_ADDR                        0x12
#define TFMINI_S_RIGHT_I2C_ADDR	                    0x13
#define TFMINI_S_UPWARD_I2C_ADDR                0x14
#define TFMINI_S_BACKWARD_I2C_ADDR           0x15
#define TFMINI_S_DEFAULT_BUS_SPEED            400000

/* The datasheet gives 1000Hz maximum measurement rate, but it's not true according to tech support from Benewake*/
#define TFMINI_S_CONVERSION_INTERVAL	(1000) /* microseconds */

/*
 * Resolution & Limits according to datasheet
 */
#define TFMINI_S_RESOLUTION_M		(0.001)
#define TFMINI_S_MAX_DISTANCE_M		(12.00f)
#define TFMINI_S_MIN_DISTANCE_M		(0.10f)
#define TFMINI_S_FOV_DEG		(1.0f)

/* Hardware definitions */


class tfmini_s : public device::I2C, public I2CSPIDriver<tfmini_s>
{
public:
	tfmini_s(I2CSPIBusOption bus_option, int bus_number, int bus_frequency);
	virtual ~tfmini_s();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int     init();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void		start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void            RunImpl();

private:
	int		probe() override;

	void            print_status() override;

	PX4Rangefinder _px4_rangefinder;

	unsigned        _measure_interval{TFMINI_S_CONVERSION_INTERVAL};
	int32_t            DEVICE_I2C_ADDR{0xff};

	bool        _collect_phase{false};

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _range_errors;
	perf_counter_t      _conf_errors;

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int         measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int         collect();
};

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

	return crc & 0xFF;
}

tfmini_s::tfmini_s(I2CSPIBusOption bus_option, int bus_number, int bus_frequency) :
	I2C(DRV_DIST_DEVTYPE_TFMINI_S, MODULE_NAME, bus_number, TFMINI_S_DOWNWARD_I2C_ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus_number, TFMINI_S_DOWNWARD_I2C_ADDR),
	_px4_rangefinder(get_device_id(), distance_sensor_s::ROTATION_DOWNWARD_FACING),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err"))
{
	DEVICE_I2C_ADDR = TFMINI_S_DOWNWARD_I2C_ADDR;
}

tfmini_s::~tfmini_s()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int tfmini_s::init()
{
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_TFMINI_S"), &hw_model);

	switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("SENS_EN_TFMINI_S set as 0");
		return PX4_ERROR;

	case 1: // Enable
		if (I2C::init() != OK) {
		PX4_DEBUG("Initialisation failed at i2c address 0x%02x",DEVICE_I2C_ADDR);
		return PX4_ERROR;

		} else {
			PX4_DEBUG("Enabled the sensor at 0x%02x",DEVICE_I2C_ADDR);
			_px4_rangefinder.set_fov(TFMINI_S_FOV_DEG);
			_px4_rangefinder.set_max_distance(TFMINI_S_MAX_DISTANCE_M);
			_px4_rangefinder.set_min_distance(TFMINI_S_MIN_DISTANCE_M);
		}
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void tfmini_s::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}


int tfmini_s::probe()
{
	return measure();
}

void tfmini_s::RunImpl()
{
	/* collection phase? */
	if (_collect_phase) {




		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;




		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > TFMINI_S_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - TFMINI_S_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(TFMINI_S_CONVERSION_INTERVAL);
}

int tfmini_s::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	set_device_address(DEVICE_I2C_ADDR);
	//const uint8_t cmd[] = {0x5A,0x05,0x00,0x01,0x60};//  output unit: cm
	const uint8_t cmd[] = {0x5A,0x05,0x00,0x06,0x65};//  output unit: mm
	int ret = transfer(&cmd[0], 5, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int tfmini_s::collect()
{
	uint8_t val[9] {};

	struct {
		int16_t     distance_mm, strength, temperature;
	} report{};

	int ret;

	perf_begin(_sample_perf);



	/* get measurements from the device */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint8_t cmd = DEVICE_I2C_ADDR;
	ret =  transfer(&cmd, 1, &val[0], 9);

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("I2C read error");
		perf_end(_sample_perf);
		return ret;
	}

	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers
	 */
	if (val[0] != 0x59 || val[1] != 0x59 ) {
		perf_count(_range_errors);
		DEVICE_DEBUG("data/status read error");
		perf_end(_sample_perf);
		return ret;
	}

	/* swap the data we just received */
	report.distance_mm = (val[3] << 8) | val[2];
	report.strength = (val[5] << 8) | val[4];
	report.temperature = ((val[7] << 8) | val[6]) / 8 - 256;

	/*
	 * raw outputs
	 */
	float distance_m = static_cast<float>(report.distance_mm) * 1e-3f;
	uint8_t signal_quality = 100 * report.strength / 65535.0f;
	//uint16_t temperature = report.temperature;

	if (distance_m < TFMINI_S_MIN_DISTANCE_M)
		 signal_quality = 0;
	if((report.strength == -1) || (report.strength == -2))
		 signal_quality = report.strength;

	if (crc8(val, 8) == val[8]) {
		 _px4_rangefinder.update(timestamp_sample, distance_m,signal_quality);
	}
	ret = OK;
	return ret;
}


void tfmini_s::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u interval\n", _measure_interval);
}

I2CSPIDriverBase *
tfmini_s::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance)
{
	tfmini_s *interface = new tfmini_s(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	interface->start();

	return interface;
}

void tfmini_s::print_usage()
{
	PRINT_MODULE_USAGE_NAME("tfmini_s", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tfmini_s_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = tfmini_s;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = TFMINI_S_DEFAULT_BUS_SPEED;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.orientation = atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINI_S);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return 1;
}
