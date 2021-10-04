uint64 timestamp				# time since system start (microseconds)

# Airflow Sensors Measurements
uint8 airflow_sensor_num                # the number of airflow sensor
float32[16] current             # airflow sensor current measurement
float32[16] voltage             # airflow sensor voltage measurement
float32[16] power               # airflow sensor power measurement

# Force Sensors Measurements
float32[4] force                # force sensor force measurement

# TOPICS arduino_msg
