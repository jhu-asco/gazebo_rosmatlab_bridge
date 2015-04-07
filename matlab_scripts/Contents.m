% MATLAB_ROSCLIENT
% Programs for controlling different Gazebo models from MATLAB
%
% Closed Loop Controllers:
%   arm_closedloop                   - Example for closed loop PID control of Double Pendulum 2 link arm
%   arm_computedtorquelaw            - Controls 2 link arm using Computed Torque Control law
%   car_closedloop                   - Example of trajectory optimization for an rccar using feedback linearization of kinematic model
%   quad_bs                          - Demonstration of nonlinear trajectory tracking control of a quadrotor system using backstepping controller
%   wam_servo_control                - wam_servo_control Control WAM arm using servos.
% Optimization Examples:
%   arm_shooting                     - Example of trajectory optimization for 2 link arm using GN shooting
%   car_shooting                     - Example of trajectory optimization for a rccar using GN shooting
%   quadcopter_shooting              - Example of trajectory optimization for a quadrotor using GN shooting
%   sopt_car                         - Optimization of rccar
%   wam_shooting                     - Example of trajectory optimization for a wam arm using GN shooting
%   
% Setup files for future use:
%   AuvSetup                         - AuvSetup Sets up the auv vehicle to be controlled by Matlab
%   satellite_test                   - Satellite_test Applies forces to thrusters to 
%   servo_verification               - Setups servos for different systems
%
% Helper Classes:
%   GazeboMatlabSimulator            - Matlab client which handles the communication with Gazebo.
%   MarkerInfo                       - MarkerInfo Provides information about color, id etc of the line being published
%   MatlabLinkInput                  - MatlabLinkInput Creates a struct for storing link data force and torque
%   MatlabRigidBodyState             - Convenience class which stores position, orientation etc of Rigid body state  
%   mex_mmap                         - Provides bridge between gazebo and MATLAB (Mex library)
%   sopt7                            - 
% Helper Functions:
%   quat2mat                         - converts quaternion of form qw + qx*i + qy*j + qz*k to 3x3
%   quat2rpy                         - Converts the quaternion in the form(q0 + q1 i + q2 j + q3 k into the roll pitch yaw
%   quatinv                          - Performs Quaternion inverse for q being a nx4 matrix
%   quatmultiply                     - Performs Quaternion multiplication of q1*q2. q1 and q2 are nx4 matrices
%   quatrotate                       - rotate a given vector using a quaternion to provide a rotated point
%   rpy2quat                         - Converts rpy body 321 sequence (yaw pitch roll) to quaternion
%   wcov                             - Weighted Covariance Matrix
%   draw_rb                          - draw a rigid body at a given state
