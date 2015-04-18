Gazebo Matlab Bridge using Ros serialization
---------------------------------------------------------------------
This package provides a gazebo plugin and a mex interface for fast communication between matlab and Gazebo. The mex interface provides fast access to link and joint states; easy application of link and joint efforts; way to set model and joint states. Various matlab examples have been provided in MATLAB

Installation:
--------------
Prerequisite installations:
*	ros-hydro/indigo
* Gazebo 1.9/2.x
* ros-hydro-gazebo-packages 
* PQP (Optional)

Install hydro for ubuntu 12.04 and indigo for ubuntu 14.04.

Instructions for setting ROS:
----------------------------
For Hydro/Indigo installation follow instructions [here](http://wiki.ros.org/hydro/Installation/Ubuntu)/[here](http://wiki.ros.org/indigo/Installation/Ubuntu); Choose the Desktop-Full Install (Recommended) version when installing ROS.

Then to setup a catkin workspace follow the instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Once the environment is setup run the following command:

echo "source ~/catkin_ws/devel/setup.bash">>~/.bashrc

Once the environment is setup, install the prerequisites:

For hydro:
sudo apt-get install ros-hydro-gazebo-ros ros-hydro-gazebo-ros-pkgs
For Indigo:
sudo apt-get install ros-indigo-gazebo-ros ros-indigo-gazebo-ros-pkgs

Instructions for Installing PQP:
---------------------------------
Download PQP from  [here](http://gamma.cs.unc.edu/SSV/); Untar the file and run make in the folder. If you are running a 64 bit operating system you should add the following lines to the Makefile :

CFLAGS    = -O2 -I.

__Add these lines here for x86_64 systems__

CFLAGS += -fPIC
CPPFLAGS += -fPIC

Once compiled copy the files into your system using the following commands:

sudo cp -r include /usr/local/include/PQP
sudo cp lib/libPQP.a /usr/local/lib/libPQP.a
sudo ldconfig

Package installation:
--------------------------
Go into catkin workspace (~/catkin_ws for above instructions)/src folder:
cd ~/catkin_ws/src

Clone the package from git:
git clone https://git.lcsr.jhu.edu/ggarime1/gazebo_rosmatlab_bridge.git


Run the setup_script.bash with the arguments as MATLAB_ROOT(Directory where MATLAB is installed) and ROS_WORKSPACE(~/catkin_ws if you have followed instructions above)

Example Usage: source setup_script /usr/local/MATLAB/R2014a ~/catkin_ws

After installing the main components, if pqp is installed, you can run 
source pqp_setup.bash 

to setup collision checking module.

Further Documentation:
-------------------
Lookup __docs/documentation.pdf__ and __docs/matlab_documentation__ folder

Notes:
---------------------
Compile PQP with -fPIC under x86_64 systems since it produces static libraries
