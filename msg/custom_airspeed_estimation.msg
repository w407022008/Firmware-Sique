uint64 timestamp				# time since system start (microseconds)

# float32[4] control				# Roll, Pitch, Yaw, Throttle
# uint64 timestamp_sample_ctl			# timestamp of the data sample on which this message is based (microseconds)

uint64 timestamp_sample_pwm			# timestamp of the data sample on which this message is based (microseconds)
float32[4] output				# output data, in natural output units PWM

uint64 timestamp_sample_imu			# timestamp of the data sample on which this message is based (microseconds)
float32[3] vehicle_acceleration_xyz		# Bias corrected acceleration (including gravity) in body axis (in m/s^2)
float32[3] vehicle_angular_velocity_xyz		# Bias corrected angular velocity about X, Y, Z body axis (in rad/s)

uint64 timestamp_sample_bias			# timestamp of the data sample on which this message is based (microseconds)
float32[3] gyro_bias				# gyroscope in-run bias in body frame (rad/s)
float32[3] accel_bias				# accelerometer in-run bias in body frame (m/s^2)

float32[3] vehicle_attitude_ned			# Euler rotation from XYZ body frame to NED earth frame, phi theta psi.
uint64 timestamp_sample_att 			# timestamp of the data sample on which this message is based (microseconds)

uint64 timestamp_sample_ground_truth		# timestamp of the data sample on which this message is based (microseconds)
float32[3] ground_truth_velocity_ned		# Velocity in NED earth-fixed frame, (metres/sec)
float32[3] ground_truth_velocity_xyz		# Velocity in Body frame, (metres/sec)
float32[3] ground_truth_acceleration_ned	# Acceleration in NED earth-fixed frame, (metres/sec^2)
float32[3] ground_truth_acceleration_xyz	# Acceleration in XYZ Body frame, (metres/sec^2)

float32 windspeed_x				# Airspeed measurement along x-axis in body frame, (metres/sec)
uint64 timestamp_sample_wind 			# timestamp of the data sample on which this message is based (microseconds)

float32 battery_scale					# Power scaling factor, >= 1, or -1 if unknown
uint64 timestamp_sample_bat 			# timestamp of the data sample on which this message is based (microseconds)

# TOPICS custom_airspeed_estimation
