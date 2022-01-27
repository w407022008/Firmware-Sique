# this message contains the predicted state and the updated state of ekf2, 
# and error integrated for output.


uint64 timestamp			 # time since system start (microseconds)

float32[3] pre_vel
float32[3] pre_pos
float32[3] est_vel
float32[3] est_pos

float32 vel_gain
float32 pos_gain

float32[3] vel_err
float32[3] pos_err
float32[3] vel_err_integ
float32[3] pos_err_integ

float32[3] vel_correction
float32[3] pos_correction

float32[3] buff_oldest
float32[3] buff_newest
