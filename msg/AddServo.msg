# Provides joint state msg for various types of joints
float64[3] gains #Gains PID
float64[4] limits #Integral lower and upper limits, cmd lower and upper limits
uint32 joint_ind #Index of joint
int8 control_type #Control type can be position Control or velocity control
#int8 POSITION_CONTROL=0
#int8 VELOCITY_CONTROL=1
