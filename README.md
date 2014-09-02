Bridge Node for providing topics to a general robot from gazebo to ros
---------------------------------------------------------------------
	Topics which are already being provided as topics by gazebo_ros_api are not provided again. First we take care of applying wrenches and torques on joints. Since it is a  service in ros_api and it is not supported rosmatlab we provide a topic for doing the same.

	A class is provided in Matlab(matlab_rosClient) for accessing ros topics provided by this bridge node. Using the class you can set efforts to joints, apply body wrenches and many other things. [TODO Example programs in Matlab]
Architecture:
--------------
Matlab side -> (rosmatlab node)
Subscribed topics:
* link_states (gazebo_msgs/LinkStates)
* model_states (gazebo_msgs/ModelStates)
* Sensor Data (Odometry, Imu, LaserScan etc)

Published topics:
* apply_bodywrench (geometry_msgs/WrenchStamped)
* apply_jointeffort (geometry_msgs/Vector3Stamped)
* set_modelstate (gazebo_msgs/ModelState)
* run_simulation (std_msgs/Float64) # Duration
* reset world
* clear_bodywrench (std_msgs/String)

Ros side ->
Subscribed topics:
* apply_bodywrench (geometry_msgs/WrenchStamped)
* apply_jointeffort (geometry_msgs/Vector3Stamped)
* set_modelstate (gazebo_msgs/ModelState)
* run_simulation (std_msgs/Duration)
* reset_world (std_msgs/Empty)
* clear_bodywrench (std_msgs/String)

Client :
* set_model_state (gazebo_msgs/SetModelState.srv)
* apply_body_wrench (gazebo_msgs/ApplyBodyWrench.srv)
* apply_joint_effort (gazebo_msgs/ApplyJointEffort.srv)
* pause_physics
* reset_world
* clear_body_wrench(gazebo_msgs/BodyRequest.srv)
* clear_joint_forces(gazebo_msgsJointRequest.srv)

World Files:
--------------
World files provide scenes that can be used to test algorithms in matlab. To view them in matlab, use the basic.launch file and change the argument world_name according to your needs.
For example:
											 roslaunch gazebo_rosmatlab_bridge basic.launch 
will launch the world file specified in the launch file.
