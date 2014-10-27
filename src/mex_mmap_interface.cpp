/************************************************************************
 Base Sample MEX code written by Fang Liu (leoliuf@gmail.com).

 Modified by: Gowtham Garimella(garimella.gowtham74@gmail.com)
************************************************************************/

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
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>

#include <gazebo_rosmatlab_bridge/JointEfforts.h>
#include <gazebo_rosmatlab_bridge/BodyWrenches.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <vector>

//Useful Links:
//http://www.mathworks.com/matlabcentral/answers/uploaded_files/1750/simplefunction.cpp


using namespace std;

class Data_struct//Stores data
{
	public:
		gazebo_msgs::LinkStates linkdata;
		gazebo_msgs::ModelStates modeldata;
		ros::Time timestamp_link;

		Mmap<gazebo_rosmatlab_bridge::JointEfforts> memout1;
		Mmap<gazebo_rosmatlab_bridge::BodyWrenches> memout2;
		Mmap<gazebo_msgs::LinkStates> memin1;
		Mmap<std_msgs::Empty> memin3;

		Data_struct():memout1("/tmp/in_jointefforts.tmp", 5000)//Will make a readonly mode #TODO
								  ,memout2("/tmp/in_bodywrenches.tmp",5000)	
									,memin1("/tmp/out_linkstates.tmp", 5000)
									,memin3("/tmp/out_time.tmp",50)
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
			Data_struct *storage = new Data_struct();
			if(nlhs != 1)
			{
				mexErrMsgTxt("There should be only one output for storing data in new");
				return;
			}
			//Read link and model data once to get the available names
			while(storage->memin1.Status()!= 128);//Wait till the stream is reastoragey to be reastorage
			storage->memin1.Read(storage->linkdata,storage->timestamp_link.sec, storage->timestamp_link.nsec);

			/*while(storage->memin2.Status() != 128);//Wait till it is ready to be read can add timeouts here #TODO
			storage->memin2.Read(storage->modeldata,storage->timestamp_model.sec,storage->timestamp_model.nsec);
			*/

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
		else if(!strcmp("readlinkjoint",cmd))
		{
			if((nrhs != 3) && (nlhs < 3))
			{
				mexErrMsgTxt("Expected input: cmd Stored_data {id_link; id_joint}; Output: Link_data; Joint_data Time(Optional)");
				return;
			}

			if(!mxIsCell(prhs[2]))
				mexErrMsgTxt("Third arg, left hand side args need to be double");

			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

			while(d->memin1.Status()!= 128);//Wait till the stream is ready to be read
			d->memin1.Read(d->linkdata,d->timestamp_link.sec, d->timestamp_link.nsec);

			double * link_ind = mxGetPr(mxGetCell(prhs[2],0));
			int nlinkind = mxGetNumberOfElements(mxGetCell(prhs[2],0));
			double * joint_ind = mxGetPr(mxGetCell(prhs[2],1));
			int njointind = mxGetNumberOfElements(mxGetCell(prhs[2],1));

			//Create lhs data:
			plhs[0] = mxCreateDoubleMatrix(13,nlinkind,mxREAL);
			double *lhs1 = mxGetPr(plhs[0]);

			plhs[1] = mxCreateDoubleMatrix(6,njointind,mxREAL);
			double *lhs2 = mxGetPr(plhs[1]);

			for(int count = 0; count < nlinkind; count++)
			{
				geometry_msgs::Pose &pose = d->linkdata.pose[int(link_ind[count])-1];
				geometry_msgs::Twist &twist = d->linkdata.twist[int(link_ind[count])-1];

				lhs1[13*count+0] = pose.position.x; lhs1[13*count+1] = pose.position.y; lhs1[13*count+2] = pose.position.z;
				lhs1[13*count+3] = pose.orientation.w; lhs1[13*count+4] = pose.orientation.x; lhs1[13*count+5] = pose.orientation.y; lhs1[13*count+6] = pose.orientation.z;
				lhs1[13*count+7] = twist.linear.x; lhs1[13*count+8] = twist.linear.y; lhs1[13*count+9] = twist.linear.z;
				lhs1[13*count+10] = twist.angular.x; lhs1[13*count+11] = twist.angular.y; lhs1[13*count+12] = twist.angular.z;
			}

			for(int count = 0;count <njointind; count++)
			{
				geometry_msgs::Twist &twist = d->linkdata.twist[int(joint_ind[count])-1];

				//printf("Joint Name: %s",d->linkdata.name[int(joint_ind[count])-1].c_str());//#DEBUG
				lhs2[6*count+0] = twist.angular.x; //We only use revolute or 1DOF joints. Easily extendable to other joints
				lhs2[6*count+1] = twist.angular.y;
				lhs2[6*count+2] = twist.angular.z;
				lhs2[6*count+3] = twist.linear.x; 
				lhs2[6*count+4] = twist.linear.y; 
				lhs2[6*count+5] = twist.linear.z; 
			}


			if(nlhs == 3)
			{
				plhs[2] = mxCreateDoubleScalar(d->timestamp_link.toSec());
			}
		}		
		else if(!strcmp("readtime",cmd))
		{
			if(nlhs !=1 && nrhs != 2)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data} and 1 output argument {Current time in sec}");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

			std_msgs::Empty emptymsg;
			ros::Time simtime;

			while(d->memin3.Status()!= 128);//Wait till the stream is ready to be read
			d->memin3.Read(emptymsg,simtime.sec, simtime.nsec);

			plhs[0] = mxCreateDoubleScalar(simtime.toSec());
		}
		else if(!strcmp("seteffort",cmd))
		{
			if(nlhs != 0 && nrhs != 4)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, ids, Efforts} and no output arguments");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			gazebo_rosmatlab_bridge::JointEfforts je;//Message to pass on


			if(mxGetNumberOfElements(prhs[2]) != mxGetNumberOfElements(prhs[3]))
			{
				mexErrMsgTxt("Number of Ids and Efforts do not match");
				return;
			}
			int nofelems = mxGetNumberOfElements(prhs[2]);

			double *pointer1 = mxGetPr(prhs[2]);
			double *pointer2 = mxGetPr(prhs[3]);
			if((d->linkdata.name.size() == 0))// No Link Data
			{
				mexErrMsgTxt("No available Link names");
			}
			for(int count = 0;count < nofelems; count++)
			{
				je.joint_names.push_back(d->linkdata.name[int(pointer1[count])-1].substr(7));
				je.effort.push_back(pointer2[count]);
				//printf("Joint Name: %s",je.joint_names[count].c_str());//#DEBUG
			}
			while(!d->memout1.Write(je));//Write the topic until it succeeds
		}
		else if(!strcmp("setwrench",cmd))
		{
			if(nlhs != 0 && nrhs != 4)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, ids[1xn], Wrenches[6xn]} and no output arguments");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			gazebo_rosmatlab_bridge::BodyWrenches bw;//Message to pass on

			if(mxGetNumberOfElements(prhs[2]) != mxGetM(prhs[3]))
			{
				mexErrMsgTxt("Number of Ids and Wrenches do not match");
				return;
			}

			int nofelems = mxGetNumberOfElements(prhs[2]);

			double *pointer1 = mxGetPr(prhs[2]);
			double *pointer2 = mxGetPr(prhs[3]);
			for(int count = 0;count < nofelems; count++)
			{
				bw.body_name.push_back(d->linkdata.name[int(pointer1[count])-1]);//Can add more checks to see if pointer is valid or not
				//printf("Link Name: %s",bw.body_name[count].c_str());//#DEBUG
			}
			bw.wrench.assign(pointer2,pointer2+mxGetNumberOfElements(prhs[3]));//Assign array directly Avoiding writing for loop ourselves
			while(!d->memout2.Write(bw));//Write the topic until it succeeds
		}	
		else if(!strcmp("loadimage",cmd))
		{
			//Create mmap based on topic name
		}
		else if(!strcmp("readimage",cmd))
		{
			//Read Latest Image
		}
		else if(!strcmp("availablenames",cmd))	
		{
			if(nlhs != 2 && nrhs != 2)
			{
				mexErrMsgTxt("Produces available names For Links, Joints and Models");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			int noflinksandjoints = d->linkdata.name.size();
			int nofmodels = d->modeldata.name.size();//#TODO check to make sure these numbers are positive and non zero
			plhs[0] = mxCreateCellMatrix(1,noflinksandjoints);//For Links Joints Models
			plhs[1] = mxCreateCellMatrix(1,nofmodels);//For Links Joints Models
			for(int count = 0; count < noflinksandjoints; count++)
			{
				mxArray* string = mxCreateString(d->linkdata.name[count].c_str());
				mxSetCell(plhs[0],count,string);
			}
			for(int count = 0; count < nofmodels; count++)
			{
				mxArray* string = mxCreateString(d->modeldata.name[count].c_str());
				mxSetCell(plhs[1],count,string);
			}
		}
	}
	else
	{
		mexPrintf("\n No rhs arguments\n");
		return;
	}
}
/*else if(!strcmp("readmodel",cmd))
		{
			if((nrhs != 3) && (nlhs <= 2))
			{
				mexErrMsgTxt("Expected input: cmd Stored_data {id_model}; Output: Model_data Time(Optional)");
				return;
			}

			if(!mxIsDouble(prhs[2]))
				mexErrMsgTxt("Third arg, left hand side args need to be double");


			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

			while(d->memin2.Status() != 128);//Wait till it is ready to be read
			d->memin2.Read(d->modeldata,d->timestamp_model.sec,d->timestamp_model.nsec);

			//Read rhs data:
			double *model_ind = mxGetPr(prhs[2]);
			int ninds = mxGetNumberOfElements(prhs[2]);


			//Create lhs data:
			plhs[0] = mxCreateDoubleMatrix(13,ninds,mxREAL);
			double *lhs1 = mxGetPr(plhs[0]);

			for(int count = 0;count < ninds; count++)
			{
				geometry_msgs::Pose &pose = d->modeldata.pose[int(model_ind[count])-1];
				geometry_msgs::Twist &twist = d->modeldata.twist[int(model_ind[count])-1];

				lhs1[13*count+0] = pose.position.x; lhs1[13*count+1] = pose.position.y; lhs1[13*count+2] = pose.position.z;
				lhs1[13*count+3] = pose.orientation.w; lhs1[13*count+4] = pose.orientation.x; lhs1[13*count+5] = pose.orientation.y; lhs1[13*count+6] = pose.orientation.z;
				lhs1[13*count+7] = twist.linear.x; lhs1[13*count+8] = twist.linear.y; lhs1[13*count+9] = twist.linear.z;
				lhs1[13*count+10] = twist.angular.x; lhs1[13*count+11] = twist.angular.y; lhs1[13*count+12] = twist.angular.z;
			}
			if(nlhs == 2)
			{
				plhs[1] = mxCreateDoubleScalar(d->timestamp_model.toSec());//Time
			}
		}
		*/
