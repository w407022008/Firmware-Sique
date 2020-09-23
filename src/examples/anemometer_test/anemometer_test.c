/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file PX4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/windspeed.h>

__EXPORT int anemometer_test_main(int argc, char *argv[]);

int anemometer_test_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
        int _anemometer_sub = orb_subscribe(ORB_ID(windspeed));         		/**< anemometer subscription*/
//	int airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	/* limit the update rate to 50 Hz */
        orb_set_interval(_anemometer_sub, 20);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
                { .fd = _anemometer_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for(int i=0;i < 20; i++) {
                /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
                int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
                                struct windspeed_s raw;
				/* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(windspeed), _anemometer_sub, &raw);
                                PX4_INFO("anemometer measurement:\t (%8.4f, %8.4f, %8.4f)",(double)raw.measurement_windspeed_x_m_s, (double)raw.measurement_windspeed_y_m_s, (double)raw.measurement_windspeed_z_m_s);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
                                */
                        }

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */

//			struct airspeed_s raw;
//			orb_copy(ORB_ID(airspeed), airspeed_sub, &raw);
//			att.true_airspeed_m_s = sqrt(pow(raw.airspeed_body_x_m_s,2) + pow(raw.airspeed_body_y_m_s,2));
//			orb_publish(ORB_ID(airspeed), att_pub, &att);
		}
	}

	PX4_INFO("exiting");

	return 0;
}