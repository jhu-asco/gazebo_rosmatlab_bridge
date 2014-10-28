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
#include <gazebo_rosmatlab_bridge/shared_mmap.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include "gazebo_msgs/ModelState.h"

#include <gazebo_rosmatlab_bridge/JointEfforts.h>
#include <gazebo_rosmatlab_bridge/JointState.h>
#include <gazebo_rosmatlab_bridge/JointStates.h>
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
		gazebo_msgs::LinkStates linkdata;
		gazebo_rosmatlab_bridge::JointStates jointdata;
		gazebo_msgs::ModelStates modeldata;
		ros::Time timestamp_link;
		mwSize image_nofelems;
		//mwSize imagesize_[3];
		uint32_t imagesize[3];
		bool ispaused;//State of gazebo whether it is paused or running

		Mmap<gazebo_rosmatlab_bridge::JointEfforts> memout1;
		Mmap<gazebo_rosmatlab_bridge::BodyWrenches> memout2;
		Mmap<gazebo_msgs::ModelState> memout3;//Shared memory for resetting model to any place
		Mmap<std_msgs::String> memout4;
		Mmap<gazebo_rosmatlab_bridge::JointState> memout5;

		Mmap<gazebo_msgs::LinkStates> memin1;
		Mmap<gazebo_rosmatlab_bridge::JointStates> memin2;
		Mmap<std_msgs::Empty> memin3;
		map<string, boost::shared_ptr<SharedMmap<unsigned char> > > imageMap;//Shared memory for sharing image Data

		Data_struct():memout1("/tmp/in_jointefforts.tmp", 5000)//Will make a readonly mode #TODO
								  ,memout2("/tmp/in_bodywrenches.tmp",5000)	
									,memout3("/tmp/in_setmodelstate.tmp", 5000)
									,memout4("/tmp/in_gazebocontrol.tmp", 5000)
									,memout5("/tmp/in_setjointstate.tmp", 5000)
									,memin1("/tmp/out_linkstates.tmp", 5000)
									,memin2("/tmp/out_jointstates.tmp", 5000)
									,memin3("/tmp/out_time.tmp",50)
									,ispaused(false)
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
			while(storage->memin1.Status()!= 128);//Wait till the stream is ready to be read
			storage->memin1.Read(storage->linkdata,storage->timestamp_link.sec, storage->timestamp_link.nsec);
			while(storage->memin2.Status()!= 128);//Wait till the stream is ready to be read
			storage->memin2.Read(storage->jointdata);

			std_msgs::String stringmsg;
			stringmsg.data = "pause";
			while(!storage->memout4.Write(stringmsg));
			storage->ispaused = true;//World is paused

			stringmsg.data = "reset";
			while(!storage->memout4.Write(stringmsg));

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
				mexErrMsgTxt("Third arg, left hand side args need to be cell");

			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);

			if(!d->ispaused)//If gazebo is not paused
			{
				while(d->memin1.Status()!= 128);//Wait till the stream is ready to be read
				d->memin1.Read(d->linkdata,d->timestamp_link.sec, d->timestamp_link.nsec);
				while(d->memin2.Status()!= 128);//Wait till the stream is ready to be read
				d->memin2.Read(d->jointdata);
			}

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
				geometry_msgs::Vector3 &angle = d->jointdata.angle[int(joint_ind[count])-1];
				geometry_msgs::Vector3 &vel_angle = d->jointdata.vel_angle[int(joint_ind[count])-1];
				lhs2[6*count+0] = angle.x;
				lhs2[6*count+1] = angle.y;
				lhs2[6*count+2] = angle.z;

				lhs2[6*count+3] = vel_angle.x;
				lhs2[6*count+4] = vel_angle.y;
				lhs2[6*count+5] = vel_angle.z;
				//printf("Joint Name: %s",d->jointdata.name[int(joint_ind[count])-1].c_str());//#DEBUG
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

			if(!d->ispaused)//If gazebo is not paused
			{
				while(d->memin3.Status()!= 128);//Wait till the stream is ready to be read
				d->memin3.Read(emptymsg,simtime.sec, simtime.nsec);
			}

			plhs[0] = mxCreateDoubleScalar(simtime.toSec());
		}
		else if(!strcmp("setjointeffort",cmd))
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
			if((d->jointdata.joint_name.size() == 0))// No Link Data
			{
				mexErrMsgTxt("No available Joint names");
			}
			for(int count = 0;count < nofelems; count++)
			{
				je.joint_names.push_back(d->jointdata.joint_name[int(pointer1[count])-1]);
				je.effort.push_back(pointer2[count]);
				//printf("Joint Name: %s",je.joint_names[count].c_str());//#DEBUG
			}
			if(!d->ispaused)//If gazebo is not paused
			{
				while(!d->memout1.Write(je));//Write the topic until it succeeds
			}
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
			if(!d->ispaused)//If gazebo is not paused
			{
				while(!d->memout2.Write(bw));//Write the topic until it succeeds
			}
		}	
		else if(!strcmp("loadcamera",cmd))
		{
			//Create mmap based on topic name
			if(nlhs != 0 && nrhs != 2)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, topic_name} and no output arguments");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			char camerammapname[64];
			mxGetString(prhs[2], camerammapname, sizeof(camerammapname));
			std::string mmap_location = std::string("/tmp/"+std::string(camerammapname)+".tmp"); 
			printf("Mmap location: %s",mmap_location.c_str());
			boost::shared_ptr<SharedMmap<unsigned char> > map1;
			map1.reset(new SharedMmap<unsigned char>(mmap_location.c_str(),0));//Read mode when the bytes are 0
			d->imageMap[std::string(camerammapname)] = map1; 
			if(!d->ispaused)//If gazebo is not paused
			{
				while(!map1->Readnofelems(d->imagesize));
				d->image_nofelems = (d->imagesize[0])*(d->imagesize[1])*(d->imagesize[2]);
				printf("Image Info: %d\t%d\t%d\n",d->imagesize[0], d->imagesize[1], d->imagesize[2]);
			}
		}
		else if(!strcmp("readimage",cmd))
		{
			//Read Latest Image
			if(nlhs != 1 && nrhs != 2)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, topic_name} and output is the image matrix");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			char camerammapname[64];
			mxGetString(prhs[2], camerammapname, sizeof(camerammapname));
			boost::shared_ptr<SharedMmap<unsigned char> > map1 = d->imageMap[std::string(camerammapname)];
			if(!map1)
			{
				mexErrMsgTxt("Cannot find the specified camera topic");
			}
			//Create lhs:
			plhs[0] = mxCreateNumericMatrix(d->image_nofelems,1, mxUINT8_CLASS, mxREAL);
			uint8_t *dest = (uint8_t *)mxGetData(plhs[0]); 
			ros::Time imageTime;
			if(!d->ispaused)
			{
				while(!map1->Read((unsigned char*)dest, d->imagesize, imageTime.sec, imageTime.nsec));//Wait till u read the data
			}
			//printf("Image Info: %f\n",imageTime.toSec());
			//Can actually set the image Data to CData in the image handle and not send any matrix out
		}
		else if(!strcmp("setjointstate",cmd))//#TODO Modify this to accept joint states and on the other side too
		{
			if(nlhs != 0 && nrhs == 4) 
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, joint_index, data[6x1]} and output is the image matrix");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			gazebo_rosmatlab_bridge::JointState jointstatemsg;
			int index = int(mxGetScalar(prhs[2]));
			jointstatemsg.joint_name = d->jointdata.joint_name[index-1];
			jointstatemsg.joint_type = d->jointdata.joint_type[index-1];//Fixed joint type for now #TODO Introduce Joint data separate from Link data and use the joint type specified in there
			//DEBUG:
			//printf("Joint INFO: %s\t%d",jointstatemsg.joint_name.c_str(), jointstatemsg.joint_type);
			double *data = mxGetPr(prhs[3]);
			jointstatemsg.angle.x = data[0];
			jointstatemsg.angle.y = data[1];
			jointstatemsg.angle.z = data[2];
			jointstatemsg.vel_angle.x = data[3];
			jointstatemsg.vel_angle.y = data[4];
			jointstatemsg.vel_angle.z = data[5];
			while(!d->memout5.Write(jointstatemsg));//Write the topic until it succeeds
		}
		else if(!strcmp("setmodelstate",cmd))
		{
			if(nlhs != 0 && nrhs <= 4)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, model_name, data[13x1] reference_frame{Optional}} and output is the image matrix");
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
			while(!d->memout3.Write(modelstate_msg));//Write the topic until it succeeds
		}
		else if(!strcmp("setgazebostate",cmd))
		{
			if(nlhs != 0 && nrhs == 2)
			{
				mexErrMsgTxt("Have to send args as {cmd, Stored_Data, status} and output is the image matrix");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			char status_msg[64];
			mxGetString(prhs[2], status_msg, sizeof(status_msg));
			std_msgs::String statusmsg;
			statusmsg.data = std::string(status_msg);
			if(!strcmp(status_msg, "pause"))
				d->ispaused = true;//Set the state of gazebo to paused
			if(!strcmp(status_msg, "start"))
				d->ispaused = false;//Set the state of gazebo to paused
			while(!d->memout4.Write(statusmsg));//Write status
		}
		else if(!strcmp("availablenames",cmd))	
		{
			if(nlhs != 2 && nrhs != 2)
			{
				mexErrMsgTxt("Produces available names For Links, Joints and Models");
			}
			Data_struct *d = convertMat2Ptr<Data_struct>(prhs[1]);
			int noflinks = d->linkdata.name.size();
			int nofjoints = d->jointdata.joint_name.size();//#TODO check to make sure these numbers are positive and non zero
			plhs[0] = mxCreateCellMatrix(1,noflinks);//For Links Joints Models
			plhs[1] = mxCreateCellMatrix(1,nofjoints);//For Links Joints Models
			for(int count = 0; count < noflinks; count++)
			{
				mxArray* string = mxCreateString(d->linkdata.name[count].c_str());
				mxSetCell(plhs[0],count,string);
			}
			for(int count = 0; count < nofjoints; count++)
			{
				mxArray* string = mxCreateString(d->jointdata.joint_name[count].c_str());
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
