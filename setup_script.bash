#!/bin/bash
if [[ "$#" -ne 2 ]] 
	then
	echo "Usage: source setup_script.bash [MATLAB ROOT DIRECTORY: For example /usr/local/matlab/r2014a] [ROS_WORKSPACE: For example ~/hydro_workspace]"
	exit
fi
PKG_FULL=`rospack find gazebo_rosmatlab_bridge`
echo $PKG_FULL
echo "export MATLAB_ROOT=$1">>~/.bashrc #Should be modified for each machine
echo "export GAZEBO_MODEL_PATH=$PKG_FULL/models">>~/.bashrc 
source ~/.bashrc
source $2/devel/setup.bash
roscd
cd ..
catkin_make
cd devel/lib
ln -s $PWD/mex_mmap.mexa64 $PKG_FULL/matlab_scripts/mex_mmap.mexa64 
