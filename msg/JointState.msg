# Provides joint state msg for various types of joints
geometry_msgs/Vector3 angle #Angles for each axis
geometry_msgs/Vector3 vel_angle #Velocity of joint for each axis
bool joint_type   #Type of Joint
uint32 joint_ind #Name of the joint
#bool REVOLUTE=1
#bool REVOLUTE2=2
#bool PRISMATIC=3
#bool UNIVERSAL=4
#bool BALL=5
#bool SCREW=6
