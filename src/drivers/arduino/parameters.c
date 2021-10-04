/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * Arduino UART Communication
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(SENS_EN_ARDUINO, 1);

/**
 * Airflow Sensors
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(SENS_EN_AIRFLOW, 1);

/**
 * Force Sensors
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(SENS_EN_FORCE, 0);

/**
 * Airflow Sensor 0 Orientation
 *
 * This parameter defines the queue number of Airflow sensor with FORWARD-FACING, e.g: sensor 0x40 0x41 placed along x-axis, and 0x40 is in front of 0x41, set it therefore as 1.
 *
 * @reboot_required true
 * @min 0
 * @max 6
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Direction of wind from I2c Address at 0x40 -> 0x41
 * @value 2 Direction of wind from I2c Address at 0x41 -> 0x40
 * @value 3 Direction of wind from I2c Address at 0x42 -> 0x43
 * @value 4 Direction of wind from I2c Address at 0x43 -> 0x42
 * @value 5 Direction of wind from I2c Address at 0x44 -> 0x45
 * @value 6 Direction of wind from I2c Address at 0x45 -> 0x44
 */
PARAM_DEFINE_INT32(AIRFLOW_ROT_X, 2);

/**
 * Airflow Sensor 1 Orientation
 *
 * This parameter defines the queue number of Airflow sensor with RIGHT_FACING, e.g: sensor 0x40 0x41 placed along x-axis, and 0x40 is in front of 0x41, set it therefore as 1.
 *
 * @reboot_required true
 * @min 0
 * @max 6
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Direction of wind from I2c Address at 0x40 -> 0x41
 * @value 2 Direction of wind from I2c Address at 0x41 -> 0x40
 * @value 3 Direction of wind from I2c Address at 0x42 -> 0x43
 * @value 4 Direction of wind from I2c Address at 0x43 -> 0x42
 * @value 5 Direction of wind from I2c Address at 0x44 -> 0x45
 * @value 6 Direction of wind from I2c Address at 0x45 -> 0x44
 */
PARAM_DEFINE_INT32(AIRFLOW_ROT_Y, 4);

/**
 * Airflow Sensor 2 Orientation
 *
 * This parameter defines the queue number of Airflow sensor with DOWNWARD-FACING, e.g: sensor 0x40 0x41 placed along x-axis, and 0x40 is in front of 0x41, set it therefore as 1.
 *
 * @reboot_required true
 * @min 0
 * @max 6
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Direction of wind from I2c Address at 0x40 -> 0x41
 * @value 2 Direction of wind from I2c Address at 0x41 -> 0x40
 * @value 3 Direction of wind from I2c Address at 0x42 -> 0x43
 * @value 4 Direction of wind from I2c Address at 0x43 -> 0x42
 * @value 5 Direction of wind from I2c Address at 0x44 -> 0x45
 * @value 6 Direction of wind from I2c Address at 0x45 -> 0x44
 */
PARAM_DEFINE_INT32(AIRFLOW_ROT_Z, 6);
