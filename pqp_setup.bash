#!/bin/bash
PKG_FULL=`rospack find gazebo_rosmatlab_bridge`
echo $PKG_FULL
roscd
cd ..
catkin_make --pkg gazebo_rosmatlab_bridge
cd devel/lib
ln -s $PWD/mex_pqp.mexa64 $PKG_FULL/matlab_scripts/mex_pqp.mexa64 
cd $PKG_FULL
