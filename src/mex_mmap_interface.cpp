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
#include "gazebo_rosmatlab_bridge/PhysicsEngineConfig.h"
#include "gazebo_rosmatlab_bridge/RunSimulation.h"
#include <gazebo_rosmatlab_bridge/BodyWrenches.h>
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

class Data_struct//Stores data
{
	public:
		gazebo_rosmatlab_bridge::AvailableNames names;
		gazebo_msgs::LinkStates linkdata;
		gazebo_rosmatlab_bridge::JointStates jointdata;
		gazebo_msgs::ModelStates modeldata;
		//ros::Time timestamp_link;

		Mmap<gazebo_msgs::ModelState> memout1;//Shared memory for resetting model to any place
		Mmap<gazebo_rosmatlab_bridge::JointState> memout2;
		Mmap<gazebo_rosmatlab_bridge::RunSimulation> memout3;
		Mmap<std_msgs::String> memout4;//Shared memory for sending reset req
		Mmap<gazebo_rosmatlab_bridge::PhysicsEngineConfig> memout5;//Memory map for sending physics engine rate and timestep

		Mmap<gazebo_msgs::LinkStates> memin1;
		Mmap<gazebo_rosmatlab_bridge::JointStates> memin2;
		Mmap<gazebo_rosmatlab_bridge::AvailableNames> memin3;

		Data_struct():memout1("/tmp/in_setmodelstate.tmp", 20000)//Will make a readonly mode #TODO
									,memout2("/tmp/in_setjointstate.tmp",5000)	
												 ,memout3("/tmp/in_simulationreq.tmp", 5000)
												 ,memout4("/tmp/in_stringreq.tmp", 500)
												 ,memout5("/tmp/in_physicsconfig.tmp",500)
														,memin1("/tmp/out_linkstates.tmp", 50000)
															,memin2("/tmp/out_jointstates.tmp", 50000)
															 ,memin3("/tmp/out_names.tmp",5000)
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
				while(storage->memin3.Status()!= 128);//Wait till the stream is ready to be read
				printf("storage->memin3.Status(): %d",storage->memin3.Status());
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
					mexErrMsgTxt("Have to send args as {cmd, Stored_Data, model_name, data[13x1] reference_frame{Optional}} and output none");
				}
				Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
				gazebo_msgs::ModelState modelstate_msg;
				char modelname[64];
				char ref_name[64];
				mxGetString(prhs[2], modelname, sizeof(modelname));
				double *modeldata = mxGetPr(prhs[3]);
				modelstate_msg.model_name = std::string(modelname);
				modelstate_msg.pose.position.x = modeldata[0]; modelstate_msg.pose.position.y = modeldata[1]; modelstate_msg.pose.position.z = modeldata[2];
				modelstate_msg.pose.orientation.w = modeldata[3]; 
				modelstate_msg.pose.orientation.x = modeldata[4]; modelstate_msg.pose.orientation.y = modeldata[5]; modelstate_msg.pose.orientation.z = modeldata[6];
				modelstate_msg.twist.linear.x = modeldata[7]; modelstate_msg.twist.linear.y = modeldata[8]; modelstate_msg.twist.linear.z = modeldata[9]; 
				modelstate_msg.twist.angular.x = modeldata[10]; modelstate_msg.twist.angular.y = modeldata[11]; modelstate_msg.twist.angular.z = modeldata[12];
				if(nrhs == 5)
				{
					mxGetString(prhs[3], ref_name, sizeof(ref_name));
				}
				while(!d->memout1.Write(modelstate_msg));//Write the topic until it succeeds
			}
			else if(!strcmp("setjointstate",cmd))//#TODO Modify this to accept joint states and on the other side too
			{
				if(nlhs != 0 && nrhs == 4) 
				{
					mexErrMsgTxt("Have to send args as {cmd, Stored_Data, joint_index, data[6x1]} and output none");
				}
				Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
				gazebo_rosmatlab_bridge::JointState jointstatemsg;
				int index = int(mxGetScalar(prhs[2]));
				jointstatemsg.joint_ind = (index - 1);
				jointstatemsg.joint_type = d->names.joint_types[index-1];
				//DEBUG:
				//printf("Joint INFO: %d\t%d",jointstatemsg.joint_ind, jointstatemsg.joint_type);
				double *data = mxGetPr(prhs[3]);
				jointstatemsg.angle.x = data[0];
				jointstatemsg.angle.y = data[1];
				jointstatemsg.angle.z = data[2];
				jointstatemsg.vel_angle.x = data[3];
				jointstatemsg.vel_angle.y = data[4];
				jointstatemsg.vel_angle.z = data[5];
				while(!d->memout2.Write(jointstatemsg));//Write the topic until it succeeds
			}
			else if(!strcmp("stringreq",cmd))
			{
				if(nlhs != 0 && nrhs != 3)
				{
					mexErrMsgTxt("Have to send args as {cmd, Stored_Data, String_Req} and output none");
				}
				Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
				char stringreq[64];
				mxGetString(prhs[2], stringreq, sizeof(stringreq));
				std_msgs::String req_msg;
				req_msg.data = std::string(stringreq);
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
					printf("Nof Elems: %d\t%d\n",nofjoints, nofeffort_elems);
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
					plhs[1] = mxCreateDoubleMatrix(6,out_nofjoints*nofsteps,mxREAL);
					double *pointer = mxGetPr(plhs[1]);//Since this is a double array
					int step = 0;
					//Get the data:
					for(int count = 0;count < (out_nofjoints*nofsteps); count++)
					{
						geometry_msgs::Vector3& angle  = jointdata.angle[count];
						geometry_msgs::Vector3& vel_angle = jointdata.vel_angle[count];
						pointer[6*count+0] = angle.x;
						pointer[6*count+1] = angle.y;
						pointer[6*count+2] = angle.z;

						pointer[6*count+3] = vel_angle.x;
						pointer[6*count+4] = vel_angle.y;
						pointer[6*count+5] = vel_angle.z;
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
		}
		else
		{
			mexPrintf("\n No rhs arguments\n");
			return;
		}
}
