Gazebo Matlab Bridge using Ros serialization
---------------------------------------------------------------------
This package provides a gazebo plugin and a mex interface for fast communication between matlab and Gazebo. The mex interface provides fast access to link and joint states; easy application of link and joint efforts; way to set model and joint states. Various matlab examples have been provided in MATLAB
Installation:
--------------
Prerequisite installations:
*	ros-hydro
* Gazebo
* ros-hydro-gazebo-packages

Package installation. Copy the package in catkin workspace, run the setup_script.bash with the arguments as MATLAB_ROOT and ROS_WORKSPACE.a
Example Usage: setup_script /usr/local/MATLAB/R2014a ~/hydro_workspace
Usage:
--------
To use the bridge, first run the gazebo with one of the world files provided in "worlds" folder. This should load an example model in gazebo with the required plugin. Then you can run one of the matlab scripts provided to control the models. Some example pairs of world files and corresponding matlab scripts are provided below:
* scene1_obstacle, scene1: arm_shooting.m, arm_closedloop.m
* scene2: car_shooting.m, sopt_car.m
* scene3: wam_shooting.m
* scene4: quadcopter_shooting.m

Additional scenes are provided for future applications. To load the world file with gazebo, you can run either of these commands:
	roslaunch gazebo_rosmatlab_bridge basic.launch scene:=scene1 #scene1 can be replaced by any of the scenes above
  ///Goto the  worlds folder and run:
	gazebo scene1.world #Will load the world file directly

Mex Documentaion (Created from the source file):
-----------------------
 This file provides a mex interface for communicating with Gazebo. This file should be used along with the world plugin gazebo_rosmatlab_bridge 
	 to complete the connection with gazebo. This interface works by calling string arguments for completing specific actions. 
	 The supported string arguments and example usages are provided below:
* NEW: This creates the bridge. You should always cleanup the class by calling DELETE after you are done with your simulation. It provides a pointer to be stored and passed on later.
	
				 MATLAB USAGE: A = mex_mmap('new');
	
	
* DELETE: This closes the bridge properly. It should be provided with the pointer created using NEW
	
				MATLAB USAGE: mex_mmap('delete',A); %A is the pointer created in above step
	

* RESET: This resets the world to its initial state. Useful for resetting the simulation after every sample.
	
				MATLAB USAGE: mex_mmap('reset',A); %A is the stored pointer
	
	
* AVAILABLENAMES: This provides the available link and joint names from the link. The indices of the links and joints are used below to configure the links and joints
	
				MATLAB USAGE: [Link_Names, Joint_Names] = mex_mmap('availablenames',A);
	
	
* CONFIGUREPHYSICS: Configure the physics engine in Gazebo to run faster/slower according to your simulation requirements; 	
	
				MATLAB USAGE: mex_mmap('configurephysics', A, 0.001,2000); 
	
				EXPLANATION: This runs the physics engine twice the real time speed with a physics timestep of 1 millisecond. The arguments taken are the physics engine timestep 
				Physics engine update frequency: The frequency at which the internal physics engine step is called
	
	
* SETMODELSTATE: This sets the complete state of model using Model Pose (x,y,z, qw,qx,qy,qz, body_vx,body_vy,body_vz,body_wx,body_wy,body_wz)[13x1] in world frame and using 
				Joint states(Joint angles, Joint Velocities)[2xn matrix]. 
	     #TODO The Joint Velocities are not supported yet. Gazebo somehow does not set the joint velocities and should be replaced with link twists somehow
	
				MATLAB USAGE I: mex_mmap('setmodelstate',h.Mex_data,'Airbotwith2dofarm',[0 0 0 1 0 0 0 0 0 0 0 0 0],...
																uint32(0:1),[0 pi;0 0]);
				MATLAB USAGE II: mex_mmap('setmodelstate',h.Mex_data,'Airbotwith2dofarm',[0 0 0 1 0 0 0 0 0 0 0 0 0]);
	
				Explanation: The 4th argument is the model pose in world frame. The fifth argument is joint indices as uint32 and starting index with 0. The final argument is the actual
				joint angles and joint velocities.
				
	
* SETJOINTSTATE: This sets a single joint angle and velocity(#TODO Velocities not working properly). NOTE: This should not be used for moving multiple joints and setmodelstate 
			should be used instead.
	
			MATLAB USAGE: mex_mmap('setjointstate', A, uint32(0) ,[1,0]);
	
			EXPLANATION: This takes in the zero based joint index as a single scalar which is uint32. The Joint State is just the angle and velocity. 
			
	
* RUNSIMULATION: This function runs the gazebo physics engine for specified number of steps and provides back the Link and Joint States at the steps specified
			
			MATLAB USAGE: [LinkData, JointData] =  mex_mmap('runsimulation', A, JOINTIDS, JOINT_CONTROLS, ...
																												LINKIDS, LINK_WRENCHES, STEPS);
			EXPLANATION:
			A: 								Stored Pointer 
			STEPS:						Vector[Nx1 or 1xN]  providing number of steps for physics engine. It should start with zero and should be increasing and the last element provides the number of 
  											steps the physics engine should take
		  JOINTIDS:					zero based joint index vector for which controls are provided [nx1 or 1xn]. The indices are obtained from available names described above.
		  JOINT_CONTROLS:		Matrix of [2x(nxN)] with each [2xn] matrix showing the Joint efforts at that step. The controls are constant in between two steps.#TODO Add interpolation techniques
			LINKIDS:					zero based link index vector for which link wrenches are provided [nx1 or 1xn]. The indices are obtained from available names described above.
			LINK_WRENCHES:		The Link wrenches are provided as [6x(nxN)]. Each [6xn] matrix provides wrenches for the set of links at that step.
			
			If you do not want to apply jointids or joint controls you can replace them by empty matrix. This applies to link ids and wrenches too
	 	Example System: Double Pendulum where you want to control the two joints indexed 0, 1. We want to run the simulation for 1 second 	
			(Assuming physics timestep is 0.001 this is 1000 steps) with state data every 0.5 seconds. Also we assume that the joint effort is 0.1 for both joints in first half of the trajectory
			and -0.1 in second half of the trajectory. This is called from matlab as follows.
			[LinkData,JointData] = mex_mmap('runsimulation',A, uint32([0 1]), [0.1 -0.1; 0.1 -0.1], ...
																			[], [], [0 500 1000]);
	  NOTE: For more examples look at the examples from matlab_rosClient folder of the package

KNOWN BUGS:
----------
There are some known bugs with this interface that need to be fixed.
* You can run the MATLAB scripts only when Gazebo is running. If not it will get stuck in some infinite loops and matlab should be forced to closed down
* Some times mex interface throws an error (Stream not ready to be read). In this case, a temporary solution is to restart gazebo. This is caused by the memory maps not syncing properly and to be fixed
