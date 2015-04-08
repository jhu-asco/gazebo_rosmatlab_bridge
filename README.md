Gazebo Matlab Bridge using Ros serialization
---------------------------------------------------------------------
This package provides a gazebo plugin and a mex interface for fast communication between matlab and Gazebo. The mex interface provides fast access to link and joint states; easy application of link and joint efforts; way to set model and joint states. Various matlab examples have been provided in MATLAB
Installation:
--------------
Prerequisite installations:
*	ros-hydro                   Install Ubuntu 12.04: sudo apt-get install ros-hydro-desktop
* Gazebo 1.9                  Install Ubuntu 12.04: sudo apt-get install ros-hydro-gazebo-ros
* ros-hydro-gazebo-packages   Install Ubuntu 12.04: sudo apt-get install ros-hydro-gazebo-ros-pkgs

Although the code has not been tested on Ubuntu 14.04, it can be tested by installing ros-indigo-desktop, ros-indigo-gazebo-ros and ros-indigo-gazebo-ros-pkgs in a similar manner.

Package installation. Copy the package in catkin workspace, run the setup_script.bash with the arguments as MATLAB_ROOT and ROS_WORKSPACE.a
Example Usage: source setup_script /usr/local/MATLAB/R2014a ~/hydro_workspace

Further Documentation:
-------------------
Lookup __docs/documentation.pdf__ and __docs/matlab_documentation__ folder
