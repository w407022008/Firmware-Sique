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
 * TCA9578A I2C Expander
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enable
 */
PARAM_DEFINE_INT32(SENS_EN_TCA9578A, 1);

/**
 * SENSIRION SFM Low Pressure DropDigital Flow Meter (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 SFM3000
 */
PARAM_DEFINE_INT32(SENS_EN_SFM3000, 1);

/**
 * SFM Sensor
 *
 * This parameter defines the number of SFM sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 *
 * @value 0 No sensor
 * @value 1 One sensor
 * @value 2 Two sensors
 * @value 3 Three sensors
 */
PARAM_DEFINE_INT32(SFM_NUM, 3);

/**
 * SFM Sensor 0 Rotation
 *
 * This parameter defines the queue number of SFM sensor with FORWARD-FACING, defualt: the First one
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 First sensor which connects to the multiplexer
 * @value 2 Second sensor which connects to the multiplexer
 * @value 3 Third sensor which connects to the multiplexer
 */
PARAM_DEFINE_INT32(SFM_ROT_X, 1);

/**
 * SFM Sensor 1 Rotation
 *
 * This parameter defines the queue number of SFM sensor with RIGHT_FACING, defualt: the Second one
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 First sensor which connects to the multiplexer
 * @value 2 Second sensor which connects to the multiplexer
 * @value 3 Third sensor which connects to the multiplexer
 */
PARAM_DEFINE_INT32(SFM_ROT_Y, 2);

/**
 * SFM Sensor 2 Rotation
 *
 * This parameter defines the queue number of SFM sensor with DOWNWARD-FACING, defualt: the Third one
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 First sensor which connects to the multiplexer
 * @value 2 Second sensor which connects to the multiplexer
 * @value 3 Third sensor which connects to the multiplexer
 */
PARAM_DEFINE_INT32(SFM_ROT_Z, 3);
