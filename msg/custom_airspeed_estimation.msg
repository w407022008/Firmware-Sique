uint64 timestamp				# time since system start (microseconds)

float32[4] control				# Roll, Pitch, Yaw, Throttle
uint64 timestamp_sample_ctl			# timestamp of the data sample on which this message is based (microseconds)

float32[3] vehicle_angular_velocity_xyz		# Bias corrected angular velocity about X, Y, Z body axis (in rad/s)
uint64 timestamp_sample_rate 			# timestamp of the data sample on which this message is based (microseconds)

float32[4] vehicle_attitude_quaternion_ned	# Quaternion rotation from XYZ body frame to NED earth frame.
uint64 timestamp_sample_att 			# timestamp of the data sample on which this message is based (microseconds)

uint64 timestamp_sample_local 			# timestamp of the data sample on which this message is based (microseconds)
float32[3] vehicle_acceleration_ned		# Acceleration (without gravity) in NED earth-fixed frame (in m/s^2)
float32[3] vehicle_velocity_ned			# Velocity in NED earth-fixed frame, (metres/sec)

float32 windspeed_x				# Airspeed measurement along x-axis in body frame, (metres/sec)
uint64 timestamp_sample_wind 			# timestamp of the data sample on which this message is based (microseconds)

# TOPICS custom_airspeed_estimation
