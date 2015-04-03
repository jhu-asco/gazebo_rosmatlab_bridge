/**This file is part of GAZEBO_ROSMATLAB_BRIDGE.
 *
 * mex_mmap_interface.cpp is a free mex file: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GAZEBO_ROSMATLAB_BRIDGE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the package gazebo_rosmatlab_bridge.  If not, see <http://www.gnu.org/licenses/>.
 *
 *********************************************************************
 *
 * This file provides a mex interface for communicating with Gazebo. This file should be used along with the world plugin gazebo_rosmatlab_bridge 
 * to complete the connection with gazebo. This interface works by calling string arguments for completing specific actions. 
 * The supported string arguments and example usages are provided below:
 *    -> NEW: This creates the bridge. You should always cleanup the class by calling DELETE after you are done with your simulation. It provides a pointer
 *       to be stored and passed on later.
 *
 *       MATLAB USAGE: A = mex_mmap('new');
 *
 *
 *    -> DELETE: This closes the bridge properly. It should be provided with the pointer created using NEW
 *
 *      MATLAB USAGE: mex_mmap('delete',A); %A is the pointer created in above step
 *
 *
 *    -> RESET: This resets the world to its initial state. Useful for resetting the simulation after every sample.
 *
 *      MATLAB USAGE: mex_mmap('reset',A); %A is the stored pointer
 *
 *
 *    -> AVAILABLENAMES: This provides the available link and joint names from the link. The indices of the links and joints are used below to configure the links and joints
 *
 *      MATLAB USAGE: [Link_Names, Joint_Names] = mex_mmap('availablenames',A);
 *
 *
 *    -> CONFIGUREPHYSICS: Configure the physics engine in Gazebo to run faster/slower according to your simulation requirements;   
 *
 *      MATLAB USAGE: mex_mmap('configurephysics', A, 0.001,2000); 
 *
 *      EXPLANATION: This runs the physics engine twice the real time speed with a physics timestep of 1 millisecond. The arguments taken are the physics engine timestep 
 *      Physics engine update frequency: The frequency at which the internal physics engine step is called
 *
 *
 *    -> SETMODELSTATE: This sets the complete state of model using Model Pose (x,y,z, qw,qx,qy,qz, body_vx,body_vy,body_vz,body_wx,body_wy,body_wz)[13x1] in world frame and using 
 *      Joint states(Joint angles, Joint Velocities)[2xn matrix]. 
 *     #TODO The Joint Velocities are not supported yet. Gazebo somehow does not set the joint velocities and should be replaced with link twists somehow
 *
 *      MATLAB USAGE I: mex_mmap('setmodelstate',h.Mex_data,'Airbotwith2dofarm',[0 0 0 1 0 0 0 0 0 0 0 0 0],...
 *                              uint32(0:1),[0 pi;0 0]);
 *      MATLAB USAGE II: mex_mmap('setmodelstate',h.Mex_data,'Airbotwith2dofarm',[0 0 0 1 0 0 0 0 0 0 0 0 0]);
 *
 *      Explanation: The 4th argument is the model pose in world frame. The fifth argument is joint indices as uint32 and starting index with 0. The final argument is the actual
 *      joint angles and joint velocities.
 *      
 *
 *    -> SETJOINTSTATE: This sets a single joint angle and velocity(#TODO Velocities not working properly). NOTE: This should not be used for moving multiple joints and setmodelstate 
 *    should be used instead.
 *
 *    MATLAB USAGE: mex_mmap('setjointstate', A, uint32(0) ,[1,0]);
 *
 *    EXPLANATION: This takes in the zero based joint index as a single scalar which is uint32. The Joint State is just the angle and velocity. 
 *    
 *
 *    -> RUNSIMULATION: This function runs the gazebo physics engine for specified number of steps and provides back the Link and Joint States at the steps specified
 *    
 *    MATLAB USAGE: [LinkData, JointData] =  mex_mmap('runsimulation', A, JOINTIDS, JOINT_CONTROLS, ...
 *                                                      LINKIDS, LINK_WRENCHES, STEPS);
 *    EXPLANATION:
 *    A:                Stored Pointer 
 *    STEPS:            Vector[Nx1 or 1xN]  providing number of steps for physics engine. It should start with zero and should be increasing and the last element provides the number of 
 *                      steps the physics engine should take
 *    JOINTIDS:         zero based joint index vector for which controls are provided [nx1 or 1xn]. The indices are obtained from available names described above.
 *    JOINT_CONTROLS:   Matrix of [2x(nxN)] with each [2xn] matrix showing the Joint efforts at that step. The controls are constant in between two steps.#TODO Add interpolation techniques
 *    LINKIDS:          zero based link index vector for which link wrenches are provided [nx1 or 1xn]. The indices are obtained from available names described above.
 *    LINK_WRENCHES:    The Link wrenches are provided as [6x(nxN)]. Each [6xn] matrix provides wrenches for the set of links at that step.
 *    
 *    If you do not want to apply jointids or joint controls you can replace them by empty matrix. This applies to link ids and wrenches too
 *  Example System: Double Pendulum where you want to control the two joints indexed 0, 1. We want to run the simulation for 1 second   
 *    (Assuming physics timestep is 0.001 this is 1000 steps) with state data every 0.5 seconds. Also we assume that the joint effort is 0.1 for both joints in first half of the trajectory
 *    and -0.1 in second half of the trajectory. This is called from matlab as follows.
 *    [LinkData,JointData] = mex_mmap('runsimulation',A, uint32([0 1]), [0.1 -0.1; 0.1 -0.1], ...
 *                                    [], [], [0 500 1000]);
 *  NOTE: For more examples look at the examples from matlab_rosClient folder of the package
 *
 *
 *
**/

/* system header */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MEX header */
#include <mex.h> 
#include "matrix.h"
#include <gazebo_rosmatlab_bridge/mex_utils.h>


//ROS and messages
#include <ros/ros.h>
#include <gazebo_rosmatlab_bridge/mmap.h>
#include <gazebo_rosmatlab_bridge/shared_mmap.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include "gazebo_msgs/ModelState.h"

#include <gazebo_rosmatlab_bridge/JointEfforts.h>
#include <gazebo_rosmatlab_bridge/JointState.h>
#include <gazebo_rosmatlab_bridge/JointStates.h>
#include "gazebo_rosmatlab_bridge/AvailableNames.h"
#include "gazebo_rosmatlab_bridge/AddServo.h"
#include "gazebo_rosmatlab_bridge/PhysicsEngineConfig.h"
#include "gazebo_rosmatlab_bridge/RunSimulation.h"
#include <gazebo_rosmatlab_bridge/BodyWrenches.h>
#include <gazebo_rosmatlab_bridge/CompleteModelState.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <vector>

/*#define REVOLUTE 1
#define REVOLUTE2 2
#define PRISMATIC 3
#define UNIVERSAL 4
#define BALL 5
#define SCREW 6
 */


//Useful Links:
//http://www.mathworks.com/matlabcentral/answers/uploaded_files/1750/simplefunction.cpp

using namespace std;

/** This class is used for storing data that needs to be saved even after returning the mex function.
 *   This essentially contains the shared memory streams to configure gazebo.
 */
class Data_struct
{
  public:
    gazebo_rosmatlab_bridge::AvailableNames names;//Available Link and Joint names

    Mmap<gazebo_rosmatlab_bridge::CompleteModelState> memout1;//Shared memory for resetting model to any place
    Mmap<gazebo_rosmatlab_bridge::JointState> memout2;//Shared memory for setting single joint state
    Mmap<gazebo_rosmatlab_bridge::RunSimulation> memout3; //Shared memory for sending simulation request
    Mmap<std_msgs::String> memout4;//Shared memory for sending reset request to gazebo
    Mmap<gazebo_rosmatlab_bridge::PhysicsEngineConfig> memout5;//Memory map for sending physics engine rate and timestep
    Mmap<gazebo_rosmatlab_bridge::AddServo> memout6;//Memory map for sending physics engine rate and timestep
    Mmap<visualization_msgs::Marker> memout7;//Memory map for sending trajectories for drawing in gazebo

    Mmap<gazebo_msgs::LinkStates> memin1;//Receive Link States
    Mmap<gazebo_rosmatlab_bridge::JointStates> memin2; //Receive Joint States
    Mmap<gazebo_rosmatlab_bridge::AvailableNames> memin3;//Receive Link and Joint names

    Data_struct():memout1("/tmp/in_setmodelstate.tmp", 0)//Will make a readonly mode #TODO
                  ,memout2("/tmp/in_setjointstate.tmp", 0)  
                  ,memout3("/tmp/in_simulationreq.tmp", 0)
                  ,memout4("/tmp/in_stringreq.tmp", 0)
                  ,memout5("/tmp/in_physicsconfig.tmp",0)
                  ,memout6("/tmp/in_attachservo.tmp",0)
                  ,memout7("/tmp/in_publishtrajectory.tmp",0)
                  ,memin1("/tmp/out_linkstates.tmp", 0)
                  ,memin2("/tmp/out_jointstates.tmp", 0)
                  ,memin3("/tmp/out_names.tmp", 0)
  {
  }
};

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
  char cmd[64];
  if(nrhs >= 1)
  {
    if(mxGetString(prhs[0], cmd, sizeof(cmd)))
      mexErrMsgTxt("First input should be a command string less than 64 characters long.");

    if (!strcmp("new", cmd)) 
    { 
      //printf("1");
      Data_struct *storage = new Data_struct();
      //printf("2");
      if(nlhs != 1)
      {
        mexErrMsgTxt("There should be only one output for storing data in new");
        return;
      }
      //Read link and model data once to get the available names
      bool result_readstatus = false;
      for(int count = 0; count < 10; count++)
      {
        if(storage->memin3.Status()== 128)
        {
          result_readstatus = true;
          break;
        }
        usleep(1000);//Wait for 1 milli second
      }

      if(!result_readstatus)
      {
        plhs[0] = mxCreateDoubleScalar(0);
        printf("Status: %d",storage->memin3.Status());
        mexErrMsgTxt("Stream not ready to be read");
      }
      //printf("storage->memin3.Status(): %d",storage->memin3.Status());
      storage->memin3.Read(storage->names);

      plhs[0] = convertPtr2Mat<Data_struct>(storage); 
      return;
    }
    else if(!strcmp("delete",cmd))
    {
      if(nrhs != 2)
      {
        mexErrMsgTxt("To delete need to pass the storage data");
        return;
      }
      Data_struct *storage = convertMat2Ptr<Data_struct>(prhs[1]);
      destroyObject<Data_struct>(prhs[1]);
      return;
    }
    else if(!strcmp("setmodelstate",cmd))
    {
      if(nlhs != 0 && nrhs <= 4)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, model_name, data[13x1] Jointinds JointStates[2xn]} and output none; Jointids and JointStates are optional");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      gazebo_rosmatlab_bridge::CompleteModelState completemodelstate_msg;
      gazebo_msgs::ModelState &modelstate_msg = completemodelstate_msg.model_state;
      char modelname[64];
      char ref_name[64];
      mxGetString(prhs[2], modelname, sizeof(modelname));
      modelstate_msg.model_name = std::string(modelname);
      if(mxGetNumberOfElements(prhs[3]) > 0)
      {
        double *modeldata = mxGetPr(prhs[3]);
        modelstate_msg.pose.position.x = modeldata[0]; modelstate_msg.pose.position.y = modeldata[1]; modelstate_msg.pose.position.z = modeldata[2];
        modelstate_msg.pose.orientation.w = modeldata[3]; 
        modelstate_msg.pose.orientation.x = modeldata[4]; modelstate_msg.pose.orientation.y = modeldata[5]; modelstate_msg.pose.orientation.z = modeldata[6];
        modelstate_msg.twist.linear.x = modeldata[7]; modelstate_msg.twist.linear.y = modeldata[8]; modelstate_msg.twist.linear.z = modeldata[9]; 
        modelstate_msg.twist.angular.x = modeldata[10]; modelstate_msg.twist.angular.y = modeldata[11]; modelstate_msg.twist.angular.z = modeldata[12];
        completemodelstate_msg.change_modelstate = true;
      }
      else
      {
        completemodelstate_msg.change_modelstate = false;
        modelstate_msg.pose.orientation.w  = 1;//Just to avoid giving an invalid pose although it is not used
      }
      /*if(nrhs >= 5)
        {
        mxGetString(prhs[4], ref_name, sizeof(ref_name));
        }
       */
      if(nrhs == 6)
      {
        int nofjoints = mxGetNumberOfElements(prhs[4]);
        if(nofjoints > 0)
        {
          uint32_t *jointinds = (uint32_t*)mxGetData(prhs[4]);
          double *jointdataptr = mxGetPr(prhs[5]);
          for(int count =0;count < nofjoints;count++)
          {
            gazebo_rosmatlab_bridge::JointState jointstate_msg;
            jointstate_msg.joint_ind = jointinds[count];
            jointstate_msg.angle.x = jointdataptr[2*count];
            jointstate_msg.vel_angle.x = jointdataptr[2*count+1];//Actually velocity does not work
            completemodelstate_msg.joint_states.push_back(jointstate_msg);
          }
        }
      }
      while(!d->memout1.Write(completemodelstate_msg));//Write the topic until it succeeds
    }
    else if(!strcmp("setjointstate",cmd))//#TODO Modify this to accept joint states and on the other side too
    {
      if(nlhs != 0 && nrhs == 4) 
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, joint_index, data[2x1]} and output none");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      gazebo_rosmatlab_bridge::JointState jointstatemsg;
      uint32_t *index = (uint32_t*)mxGetData(prhs[2]);
      jointstatemsg.joint_ind = index[0];
      jointstatemsg.joint_type = d->names.joint_types[index[0] -1];
      //DEBUG:
      //printf("Joint INFO: %d\t%d",jointstatemsg.joint_ind, jointstatemsg.joint_type);
      double *data = mxGetPr(prhs[3]);
      jointstatemsg.angle.x = data[0];
      jointstatemsg.angle.y = 0;
      jointstatemsg.angle.z = 0;
      jointstatemsg.vel_angle.x = data[1];
      jointstatemsg.vel_angle.y = 0;
      jointstatemsg.vel_angle.z = 0;
      while(!d->memout2.Write(jointstatemsg));//Write the topic until it succeeds
    }
    else if(!strcmp("publishtrajectory",cmd))
    {
      if(nlhs != 0 && (nrhs>=3) &&(nrhs <=6) )
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, data[3xn], id, modifyoradd(0/1)/delete(2), rgba[4x1]} and output none");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      visualization_msgs::Marker trajectory;//Create trajectory to publish
      trajectory.type = trajectory.LINE_STRIP;//Type of trajectory is lines

      int nofelems = mxGetNumberOfElements(prhs[2]);
      if(nofelems > 0)
      {
        double *data = mxGetPr(prhs[2]);
        assert(mxGetM(prhs[2]) == 3);
        int nofcols = mxGetN(prhs[2]);
        geometry_msgs::Point currentpoint;
        printf("nofcols: %d\n",nofcols);
        for(int count = 0;count < nofcols; count++)
        {
          currentpoint.x = data[(count*3)];
          currentpoint.y = data[3*count+1];
          currentpoint.z = data[3*count+2];
          trajectory.points.push_back(currentpoint);
          printf("currentpoint: %f\t%f\t%f\n",currentpoint.x,currentpoint.y,currentpoint.z);
        }
      }
      if(nrhs >= 4)
      {
        trajectory.id = round(mxGetScalar(prhs[3]));
      }
      if(nrhs >= 5)
      {
        trajectory.action = round(mxGetScalar(prhs[4]));//Set the action for the trajectory
      }
      if(nrhs >= 6)
      {
        double *data = mxGetPr(prhs[5]);
        trajectory.color.r  = round(data[0]);
        trajectory.color.g  = round(data[1]);
        trajectory.color.b  = round(data[2]);
        trajectory.color.a  = round(data[3]);
      }
      else
      {
        trajectory.color.r  = 1.0;
        trajectory.color.g  = 0.0;
        trajectory.color.b  = 0.0;
        trajectory.color.a  = 0.0;
      }
      printf("\nDone Loading points\n");
      while(!d->memout7.Write(trajectory));//Write the topic until it succeeds
    }
    else if(!strcmp("reset",cmd))
    {
      if(nlhs != 0 && nrhs != 2)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data} and output none");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      std_msgs::String req_msg;
      req_msg.data = "worldreset";
      while(!d->memout4.Write(req_msg));//Wait till its written
    }
    else if(!strcmp("runsimulation",cmd))
    {
      if(nlhs != 2 && nrhs != 7)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, jointids[nx1], JointEfforts[1xnxn1], linkids[nx1], LinkWrenches[6xnxn1] steps} and two output arguments");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      gazebo_rosmatlab_bridge::RunSimulation simulation_req;

      int nofjoints = mxGetNumberOfElements(prhs[2]);
      if(nofjoints > 0)//There are joint efforts to apply
      {
        uint32_t *jointinds = (uint32_t*)mxGetData(prhs[2]);
        simulation_req.jointefforts.joint_ind.assign(jointinds, jointinds + nofjoints);
        double *efforts = mxGetPr(prhs[3]);
        int nofeffort_elems = mxGetNumberOfElements(prhs[3]);
        //printf("Nof Elems: %d\t%d\n",nofjoints, nofeffort_elems);
        simulation_req.jointefforts.effort.assign(efforts,efforts+nofeffort_elems);
      }
      int noflinks = mxGetNumberOfElements(prhs[4]);
      if(noflinks > 0)//There are link wrenches to apply
      {
        uint32_t *linkinds = (uint32_t*)mxGetData(prhs[4]);
        simulation_req.bodywrenches.body_ind.assign(linkinds, linkinds + noflinks);
        double *wrenches = mxGetPr(prhs[5]);
        int nofwrench_elems = mxGetNumberOfElements(prhs[5]);
        //printf("Nof Wrench Elems: %d\t%d\n",noflinks, nofwrench_elems);
        simulation_req.bodywrenches.wrench.assign(wrenches,wrenches+nofwrench_elems);
      }
      //simulation_req.simulation_steps = uint32_t(mxGetScalar(prhs[]));

      int nofsteps = mxGetNumberOfElements(prhs[6]);
      //printf("nofcontrols: %d",nofsteps-1);
      if(nofsteps > 0)
      {
        uint32_t *ctrlsteps = (uint32_t*)mxGetData(prhs[6]);
        simulation_req.steps.assign(ctrlsteps, ctrlsteps+nofsteps);
      }
      else
      {
        mexErrMsgTxt("No steps provided");
      }
      //int nofcontrols = nofsteps-1;


      gazebo_msgs::LinkStates linkdata;
      gazebo_rosmatlab_bridge::JointStates jointdata;
      if(d->memin1.Status() == 128)
        d->memin1.Read(linkdata);
      if(d->memin2.Status() == 128)
        d->memin2.Read(jointdata); //Remove any stray messages

      while(!d->memout3.Write(simulation_req));//Write Simulation Request
      while(d->memin1.Status() != 128);//Wait till its ready to be read
      d->memin1.Read(linkdata);
      while(d->memin2.Status() != 128);//Wait till its ready to be read
      d->memin2.Read(jointdata);
      //printf("2");
      //Create Lhs
      int out_noflinks = d->names.link_names.size();
      //printf("Out Nof Links: %d",out_noflinks);
      if (out_noflinks > 0)
      {
        /*mwSize dims[3];
          dims[0] = 13;
          dims[1] = out_noflinks;
          dims[3] = simulation_req.nofcontrols;
          plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS,mxREAL);*/
        plhs[0] = mxCreateDoubleMatrix(13,out_noflinks*nofsteps,mxREAL);
        double *pointer = mxGetPr(plhs[0]);//Since this is a double array
        //Get the data:
        for(int count = 0;count < (out_noflinks*nofsteps); count++)
        {
          geometry_msgs::Pose& pose = linkdata.pose[count];
          geometry_msgs::Twist& twist = linkdata.twist[count];
          pointer[13*count+0] = pose.position.x; pointer[13*count+1] = pose.position.y; pointer[13*count+2] = pose.position.z;
          pointer[13*count+3] = pose.orientation.w; pointer[13*count+4] = pose.orientation.x; pointer[13*count+5] = pose.orientation.y; pointer[13*count+6] = pose.orientation.z;
          pointer[13*count+7] = twist.linear.x; pointer[13*count+8] = twist.linear.y; pointer[13*count+9] = twist.linear.z;
          pointer[13*count+10] = twist.angular.x; pointer[13*count+11] = twist.angular.y; pointer[13*count+12] = twist.angular.z;
        }
      }
      else
      {
        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);//Empty Matrix
      }
      int out_nofjoints = d->names.joint_names.size();//#TODO check to make sure these numbers are positive and non zero
      //printf("Out Nof Joints: %d",out_nofjoints);
      //printf("Nof joint efforts: %lu\t",(jointdata.angle.size()));
      if (out_nofjoints > 0)
      {
        /*mwSize dims[3];
          dims[0] = 6;
          dims[1] = out_nofjoints;
          dims[3] = simulation_req.nofcontrols;
          plhs[1] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS,mxREAL);
         */
        plhs[1] = mxCreateDoubleMatrix(2,out_nofjoints*nofsteps,mxREAL);
        double *pointer = mxGetPr(plhs[1]);//Since this is a double array
        int step = 0;
        //Get the data:
        for(int count = 0;count < (out_nofjoints*nofsteps); count++)
        {
          geometry_msgs::Vector3& angle  = jointdata.angle[count];
          geometry_msgs::Vector3& vel_angle = jointdata.vel_angle[count];
          pointer[2*count+0] = angle.x;
          pointer[2*count+1] = vel_angle.x;
        }
      }
      else
      {
        plhs[1] = mxCreateDoubleMatrix(0,0,mxREAL);//Empty Matrix
      }
    }
    else if(!strcmp("availablenames",cmd))  
    {
      if(nlhs != 2 && nrhs != 2)
      {
        mexErrMsgTxt("Produces available names For Links, Joints and Models");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      int noflinks = d->names.link_names.size();
      int nofjoints = d->names.joint_names.size();//#TODO check to make sure these numbers are positive and non zero
      plhs[0] = mxCreateCellMatrix(1,noflinks);//For Links Joints Models
      plhs[1] = mxCreateCellMatrix(1,nofjoints);//For Links Joints Models
      for(int count = 0; count < noflinks; count++)
      {
        mxArray* string = mxCreateString(d->names.link_names[count].c_str());
        mxSetCell(plhs[0],count,string);
      }
      for(int count = 0; count < nofjoints; count++)
      {
        mxArray* string = mxCreateString(d->names.joint_names[count].c_str());
        mxSetCell(plhs[1],count,string);
      }
    }
    else if(!strcmp("configurephysics",cmd))  
    {
      if(nlhs != 0 && nrhs != 4)
      {
        mexErrMsgTxt("Configure Physics Engine with rate and time step");
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

      gazebo_rosmatlab_bridge::PhysicsEngineConfig config_msg;
      config_msg.timestep = mxGetScalar(prhs[2]);
      config_msg.realtimerate = mxGetScalar(prhs[3]);

      while(!d->memout5.Write(config_msg));//Write the topic until it succeeds
    }
    else if(!strcmp("attachservo",cmd))
    {
      if(nlhs != 0 && nrhs < 3)
      {
        mexErrMsgTxt("Have to send args as {cmd, Stored_Data, jointid, PID Gains[3x1], Limits[4x1], control_type}");
      }
      gazebo_rosmatlab_bridge::AddServo servo_msg;

      servo_msg.joint_ind = uint32_t(round(mxGetScalar(prhs[2])));

      //Set PID Gains
      if(nrhs >=4 && mxIsDouble(prhs[3]))//Add length check #TODO
      {
        double *ptr = mxGetPr(prhs[3]);
        servo_msg.gains[0] = ptr[0];
        servo_msg.gains[1] = ptr[1];
        servo_msg.gains[2] = ptr[2];
      }
      else
      {
        servo_msg.gains[0] = 20.0;
        servo_msg.gains[1] = 1.0;
        servo_msg.gains[2] = 20.0;
      }

      //Set Limits
      if(nrhs >=5 && mxIsDouble(prhs[4]))//Add length check #TODO
      {
        double *ptr = mxGetPr(prhs[4]);
        servo_msg.limits[0] = ptr[0];
        servo_msg.limits[1] = ptr[1];
        servo_msg.limits[2] = ptr[2];
        servo_msg.limits[3] = ptr[3];
      }
      else
      {
        servo_msg.limits[0] = 100;
        servo_msg.limits[1] = -100;
        servo_msg.limits[2] = 100;
        servo_msg.limits[3] = -100;
      }
      if(nrhs >= 6 && mxIsDouble(prhs[5]))
      {
        servo_msg.control_type = int8_t(mxGetScalar(prhs[5]));
      }
      else
      {
        servo_msg.control_type = 0;//Position control usual one
      }
      Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
      while(!d->memout6.Write(servo_msg));//Write the topic until it succeeds
    }
  }
  else
  {
    mexPrintf("\n No rhs arguments\n");
    return;
  }
}
