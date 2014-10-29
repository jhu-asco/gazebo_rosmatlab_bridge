function mex_mmap
%MEX_MMAP This provides bridge between gazebo and matlab
% mex_mmap provides a bridge between gazebo and matlab The first argument
% provided to the function decides the action commited by the mex function:
% Available First arguments and corresponding documentation is as follows:
%
% 1. "new" : INPUT [ARG1("new")] OUTPUT [Stored Data] 
% -> This creates a new bridge with gazebo and provides stored data which
% needs to be passed for completing any of the actions below.
%
% 2. "delete" : INPUT [ARG1("delete"), ARG2(Stored Data)] OUTPUT [NONE] 
% -> Delete action deletes the bridge and closes it properly. It requires
% the stored data provided by the function when creating the class in the
% action above
%
% 3. "readlinkjoint": INPUT [ARG1("readlinkjoint") ARG2(Stored Data)
% ARG3(2x1 Cell with ids of links and joints to read)] OUTPUT: [Link
% Data[13xnlinkid]; Joint Data[6xnjointid]; Time(optional)] -> Reads the
% link and joint states. The link state is 13x1 vector
% [x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz] and the joint state is 6x1 vector
% with 3 axis angles and 3 angular velocities for those axis. The actual
% values depend on the type of joint. For example a revolute joint has
% rotation about only one axis and hence angle1 and velocity1 are only
% useful.
%
% 4. "readtime": INPUT [ARG1("readtime") ARG2(Stored Data)] OUTPUT [Current
% time in sec] 
% -> Returns the simulation Time in seconds
%
% 5. "setjointeffort": INPUT [ARG1("setjointeffort") ARG2(Stored Data)
% ARG3(1xn Jointids) ARG4(1xn Efforts)] OUTPUT [NONE] 
% -> Sets the joint effort (Torque) for the ids specified.
%
% 6. "setwrench": INPUT [ARG1("setwrench") ARG2(Stored Data) ARG3(nx1 Link
% ids) ARG4(6xn Wrenches)] OUTPUT [NONE] 
% -> Sets the body wrench for the specified link ids. The wrench is in
% inertial frame and applied at COM of the link. The wrench is 6x1
% vector[fx, fy, fz, torque_x, torque_y,
% torque_z]
%
% 7. "setjointstate": INPUT [ARG1("setjointstate") ARG2(Stored Data)
% ARG3(joint index) ARG4(6x1 state)] OUTPUT [NONE] 
% -> Sets the joint state i.e 3 axis angles and 3 axis angular velocities
% based on the type of joint and the data given. For example a revolute
% joint is set to specified 1st component angle and 4th component of
% angular velocity
%
% 8. "setmodelstate": INPUT [ARG1("setmodelstate") ARG2(Stored Data)
% ARG3(model name) ARG4(13x1 model state) ARG5(reference_frame(Optional))]
% OUTPUT [NONE] 
%-> Set the model to a different state. The state of model is the same as a
%link [x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz];
%
% 9. "setgazebostate": INPUT [ARG1("setmodelstate") ARG2(Stored Data)
% ARG3(status)] OUPTUT[NONE] 
% -> This allows for starting and pausing gazebo from matlab. The status is
% a string which can be one of  "start pause reset"
%
% 10."availablenames": INPUT [ARG1("availablenames") ARG2(Stored Data)]
% OUTPUT[Array1(Link Names) Array2(JointNames)] 
% ->Provides the available link and joint names. These indices are used to
% query link/joint states and for other actions mentioned above.
%
% 11."loadcamera": INPUT [ARG1("loadcamera") ARG2(Stored Data)
% ARG3(CameraTopic)] OUTPUT NONE 
% ->Creates a camera bridge between gazebo and matlab. The topic name is
% specified in the plugin for matlab for camera
%
% 12."readimage": INPUT [ARG1("readimage") ARG2(Stored Data)
% ARG3(CameraTopic)] OUTPUT [IMAGE(nx1) vector] 
% ->Reads an image from matlab camera. The image output is nx1 vector of
% rgb pixel data. arranged as [r1g1b1r2g2b2...rngnbn] for n pixels of the
% image.
%
% Author: Gowtham Garimella (ggarime1@jhu.edu) 
%Base Sample MEX code written by Fang Liu (leoliuf@gmail.com).


