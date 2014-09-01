#include <ros/ros.h>
#include <iostream>
//Services:
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/JointRequest.h>
#include <gazebo_msgs/BodyRequest.h>
#include <std_srvs/Empty.h>
//Messages
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>

using namespace std;

/// Publishers:

/// Subscribers:
ros::Subscriber bodywrench_subscriber; // TODO Documentation
ros::Subscriber jointeffort_subscriber;
ros::Subscriber modelstate_subscriber;
ros::Subscriber runningsim_subscriber;
ros::Subscriber resetworld_subscriber;
ros::Subscriber clearbodywrench_subscriber;

/// Service Clients:
ros::ServiceClient bodywrench_client;
ros::ServiceClient jointeffort_client;
ros::ServiceClient modelstate_client;
ros::ServiceClient pausephysics_client;
ros::ServiceClient unpausephysics_client;
ros::ServiceClient resetworld_client;
ros::ServiceClient clearbodywrench_client;
ros::ServiceClient clearjointeffort_client;

//Timer:
ros::Timer runsimulation_timer;

//Timer Function:
void StopPhysics(const ros::TimerEvent &event)
{
	std_srvs::Empty empty_packet;
	if(pausephysics_client.call(empty_packet))
	{
		ROS_INFO("Stopping Simulation: %f", ros::Time::now().toSec());
	}
	else
	{
		ROS_ERROR("Failed to call pause physics service");
	}
}
//Callback Functions
void BodyWrenchCallback(const geometry_msgs::WrenchStamped &bodywrenchstamped)
{
	gazebo_msgs::ApplyBodyWrench bodywrench_packet;
	bodywrench_packet.request.body_name = bodywrenchstamped.header.frame_id;
	bodywrench_packet.request.wrench = bodywrenchstamped.wrench;
	//By Default the wrench is applied in the inertial frame and always applied to the origin of the body.
	//Also we apply the wrench permanently until it is cleared
	//If we want to use full power of the service, then have to write a custom message TODO in future
	bodywrench_packet.request.start_time = bodywrenchstamped.header.stamp;
	bodywrench_packet.request.duration = ros::Duration(-1);//Keeps on applying the wrench till you clear it with a new msg

	if(bodywrench_client.call(bodywrench_packet))
	{
		ROS_INFO("Applied Body wrench with success: %d",bodywrench_packet.response.success);
		ROS_INFO("Status: %s",bodywrench_packet.response.status_message.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call bodywrench service");
	}
}

void JointEffortCallback(const geometry_msgs::Vector3Stamped &effortstamped)
{
	//First Clear exist joint effort before applying new one
	/*gazebo_msgs::JointRequest clearjointreq;
	clearjointreq.request.joint_name = effortstamped.header.frame_id;
	if(clearjointeffort_client.call(clearjointreq))
	{
		ROS_INFO("Cleared Existing Joint effort");
	}
	else
	{
		ROS_ERROR("Failed to call Clear Joint Effort service");
	}
	*/
	gazebo_msgs::ApplyJointEffort jointeffort_packet;
	jointeffort_packet.request.joint_name = effortstamped.header.frame_id;//This cannot be used as of now since there is no ros time in matlab
	jointeffort_packet.request.effort = effortstamped.vector.x;//X is the Effort value
	jointeffort_packet.request.start_time = effortstamped.header.stamp;//X is the Effort value
	jointeffort_packet.request.duration = ros::Duration(effortstamped.vector.y);//Y is the duration 
	//Z is not used as of now
	if(jointeffort_client.call(jointeffort_packet))
	{
		ROS_INFO("Applied JointEffort with success: %d",jointeffort_packet.response.success);
		ROS_INFO("Status: %s",jointeffort_packet.response.status_message.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call Jointeffort service");
	}
}

void ModelStateCallback(const gazebo_msgs::ModelState &modelstate)
{
	gazebo_msgs::SetModelState modelstate_packet;
	modelstate_packet.request.model_state = modelstate;
	if(modelstate_client.call(modelstate_packet))
	{
		ROS_INFO("Set Model State with success: %d",modelstate_packet.response.success);
		ROS_INFO("Status: %s",modelstate_packet.response.status_message.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call SetModelState service");
	}
}

void RunSimCallback(const std_msgs::Duration &duration)
{
	//Unpause the physics:
	std_srvs::Empty empty_packet;
	if(unpausephysics_client.call(empty_packet))
	{
		ROS_INFO("Starting Simulation: %f", ros::Time::now().toSec());
	}
	else
	{
		ROS_ERROR("Failed to call unpause physics service");
	}
	//Start oneshot timer for the required duration
	runsimulation_timer.setPeriod(duration.data);
	runsimulation_timer.start();
	//Simulation will be stopped when timer maxes out
}

void ResetWorldCallback(const std_msgs::Empty &emptymsg)
{
	std_srvs::Empty empty_packet;
	if(resetworld_client.call(empty_packet))
	{
		ROS_INFO("Reset World");
	}
	else
	{
		ROS_ERROR("Failed to call reset world service");
	}
}
void ClearBodyWrenchCallback(const std_msgs::String &bodyname)
{
	gazebo_msgs::BodyRequest clearbodyreq;
	clearbodyreq.request.body_name = bodyname.data;
	if(clearbodywrench_client.call(clearbodyreq))
	{
		ROS_INFO("Cleared Existing Body Wrench");
	}
	else
	{
		ROS_ERROR("Failed to call Clear Body Wrench service");
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "gazebo_rosmatlab_bridge");

	ros::NodeHandle nh("~");

	//Declare the subscribers and clients:

	bodywrench_subscriber = nh.subscribe("apply_bodywrench",100, BodyWrenchCallback);
	jointeffort_subscriber = nh.subscribe("apply_jointeffort",100, JointEffortCallback);
	modelstate_subscriber = nh.subscribe("set_modelstate",100, ModelStateCallback);
	runningsim_subscriber = nh.subscribe("run_simulation",100, RunSimCallback);
	resetworld_subscriber = nh.subscribe("reset_world",100, ResetWorldCallback);
	clearbodywrench_subscriber = nh.subscribe("clear_bodywrench",100,ClearBodyWrenchCallback);

	bodywrench_client  = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
	jointeffort_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	modelstate_client  = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	pausephysics_client= nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	unpausephysics_client= nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	resetworld_client  = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
	clearjointeffort_client = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");
	clearbodywrench_client = nh.serviceClient<gazebo_msgs::BodyRequest>("/gazebo/clear_body_wrenches");

	//Declare Timer:
	runsimulation_timer =  nh.createTimer(ros::Duration(0), StopPhysics, true);//One shot timer
	//This timer also stops physics as soon as the node starts

	//Spin 
	ros::spin();
	return 0;
}
