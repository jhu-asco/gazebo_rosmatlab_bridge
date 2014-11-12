uint32[] body_ind                          # Gazebo body to apply wrench (linear force and torque)
float64[] wrench														#Wrenches are provided as nx6 matrix easier to pass
#string reference_frame                    # wrench is defined in the reference frame of this entity
#geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame #Not implemented
#geometry_msgs/Wrench[] wrench               # wrench applied to the origin of the body
