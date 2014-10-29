#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/msgs/msgs.hh"

#include <unistd.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

//Memcopy Topics
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"

#include "gazebo_rosmatlab_bridge/JointEfforts.h"
#include "gazebo_rosmatlab_bridge/BodyWrenches.h"
#include "gazebo_rosmatlab_bridge/JointState.h"
#include "gazebo_rosmatlab_bridge/JointStates.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include <gazebo_rosmatlab_bridge/mmap.h> //To create and use a shared memory

#define REVOLUTE 1
#define REVOLUTE2 2
#define PRISMATIC 3
#define UNIVERSAL 4
#define BALL 5
#define SCREW 6

using namespace std;

namespace gazebo
{
	class GazeboMatlabPlugin : public WorldPlugin
	{
		private:
		physics::WorldPtr world;//World Pointer for accessing correspoding links and models
		map<string, physics::JointPtr> pub_joints;
		map<string, physics::LinkPtr> pub_links;//Smaller map of links and joints used by us
		boost::shared_ptr<boost::thread> recv_thread_;


		/*vector<physics::JointPtr> pub_joints;//Joints for which data is published or are controlled
		vector<physics::LinkPtr> pub_links;//Links for which datat is published or are controlled
		vector<string> link_strings;//Storing the link names for searching later
		vector<string> joint_strings;
		*/

		Mmap<gazebo_msgs::LinkStates> memout1;//Shared memory for directly sending Link Data
		Mmap<gazebo_rosmatlab_bridge::JointStates> memout2;//Shared memory for directly sending joint data
		Mmap<std_msgs::Empty> memout3;//Shared memory for directly sending the sim time
		Mmap<gazebo_rosmatlab_bridge::JointEfforts> memin1;//Shared memory for directly receiving joint efforts
		Mmap<gazebo_rosmatlab_bridge::BodyWrenches> memin2;//Shared memory for directly sending Model Data
		Mmap<gazebo_msgs::ModelState> memin3;//Shared memory for resetting model to any place
		Mmap<std_msgs::String> memin4;
		Mmap<gazebo_rosmatlab_bridge::JointState> memin5;//Shared memory for resetting joint to any angle and vel

		//Memout messages
		gazebo_rosmatlab_bridge::JointEfforts jointeffort_msg;//Joint Effort message received from client
		gazebo_rosmatlab_bridge::BodyWrenches bodywrenches_msg;//Joint Effort message received from client

    event::ConnectionPtr updateConnection1;
    event::ConnectionPtr updateConnection2;
    event::ConnectionPtr updateConnection3;
    event::ConnectionPtr updateConnection4;

		/*sensors::RaySensorPtr hokuyo_sensor;
		sensors::ImuSensorPtr imu_sensor;
		sensors::DepthCameraSensorPtr kinect_sensor;
		*/
		void publishlinksandjoints()
		{
			if(memout1.Status() == 0)//Only publish data when stream is ready to be written
			{
				gazebo_msgs::LinkStates link_states;

				// fill link_states
				for(std::map<string, physics::LinkPtr>::iterator it = pub_links.begin(); it!= pub_links.end(); ++it)
				{
					physics::LinkPtr body = it->second;
					if (body)
					{
						link_states.name.push_back(it->first);
						geometry_msgs::Pose pose;
						gazebo::math::Pose  body_pose = body->GetWorldPose(); // - myBody->GetCoMPose();
						gazebo::math::Vector3 pos = body_pose.pos;
						gazebo::math::Quaternion rot = body_pose.rot;
						pose.position.x = pos.x;
						pose.position.y = pos.y;
						pose.position.z = pos.z;
						pose.orientation.w = rot.w;
						pose.orientation.x = rot.x;
						pose.orientation.y = rot.y;
						pose.orientation.z = rot.z;
						link_states.pose.push_back(pose);
						gazebo::math::Vector3 linear_vel  = body->GetWorldLinearVel();
						gazebo::math::Vector3 angular_vel = body->GetWorldAngularVel();
						geometry_msgs::Twist twist;
						twist.linear.x = linear_vel.x;
						twist.linear.y = linear_vel.y;
						twist.linear.z = linear_vel.z;
						twist.angular.x = angular_vel.x;
						twist.angular.y = angular_vel.y;
						twist.angular.z = angular_vel.z;
						link_states.twist.push_back(twist);
					}
				}

				if(world)
				{
					common::Time current_time = world->GetSimTime();
					bool status = memout1.Write( link_states, (uint32_t)current_time.sec, (uint32_t)current_time.nsec);
					//gzdbg<<"Writing Link States: "<<status<<endl;//#DEBUG
				}
			}
			if(memout2.Status() == 0)//Only publish data when stream is ready to be written
			{
				gazebo_rosmatlab_bridge::JointStates joint_states;
				for(std::map<string, physics::JointPtr>::iterator it = pub_joints.begin(); it!= pub_joints.end(); ++it)
				{
					gazebo::physics::JointPtr joint = it->second;
					if(joint) //If it is a joint
					{
						/* We use the joint name to specify the type of joint: [NOT IMPLEMENTED NOW ]
							 REVOLUTE  = 1;
							 REVOLUTE2 = 2;
							 PRISMATIC = 3;
							 UNIVERSAL = 4;
							 BALL      = 5;
							 SCREW     = 6;
						 */
						//std::string jointname = joint->GetScopedName();
						geometry_msgs::Vector3 angle;
						geometry_msgs::Vector3 vel_angle;
						uint8_t joint_type;
						if (joint->HasType(physics::Base::HINGE_JOINT))
						{
							joint_type = REVOLUTE;
							vel_angle.x = joint->GetVelocity(0);
							angle.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::HINGE2_JOINT))
						{
							joint_type = REVOLUTE2;
							vel_angle.x = joint->GetVelocity(0);
							vel_angle.y = joint->GetVelocity(1);
							angle.x = joint->GetAngle(0).Radian();
							angle.y = joint->GetAngle(1).Radian();
						}
						else if (joint->HasType(physics::Base::BALL_JOINT))
						{
							joint_type = BALL;
							vel_angle.x = joint->GetVelocity(0);
							vel_angle.y = joint->GetVelocity(1);
							vel_angle.z = joint->GetVelocity(2);

							angle.x = joint->GetAngle(0).Radian();
							angle.y = joint->GetAngle(1).Radian();
							angle.z = joint->GetAngle(2).Radian();
						}
						else if (joint->HasType(physics::Base::SLIDER_JOINT))
						{
							joint_type = PRISMATIC;
							vel_angle.x = joint->GetVelocity(0);
							angle.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::SCREW_JOINT))
						{
							joint_type = SCREW;
							vel_angle.x = joint->GetVelocity(0);
							angle.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::UNIVERSAL_JOINT))
						{
							joint_type = UNIVERSAL;
							vel_angle.x = joint->GetVelocity(0);
							vel_angle.y = joint->GetVelocity(1);
							angle.x = joint->GetAngle(0).Radian();
							angle.y = joint->GetAngle(1).Radian();
						}
						joint_states.angle.push_back(angle);
						joint_states.vel_angle.push_back(vel_angle);
						joint_states.joint_type.push_back(joint_type);
						joint_states.joint_name.push_back(it->first);
					}
				}
				if(world)
				{
					bool status = memout2.Write( joint_states);
					//gzdbg<<"Wrote Joint Data"<<endl;
				}
			}
		}
		void setlinkwrenches()
		{
			memin2.Read(bodywrenches_msg);//Reading the message dont care about time stamp
			for(int count = 0; count < bodywrenches_msg.body_name.size();count++)
			{
				if(6*bodywrenches_msg.body_name.size() != bodywrenches_msg.wrench.size())
				{
					gzdbg<<"Sizes of bodywrenches names and wrench size do not match"<<bodywrenches_msg.body_name.size()<<"\t"<<bodywrenches_msg.wrench.size()<<std::endl;
					return;
				}
				//gzdbg<<"Verifying "<<jointeffort_msg.joint_names[count]<<std::endl;//#DEBUG
				gazebo::physics::LinkPtr link = pub_links[bodywrenches_msg.body_name[count]];
				if(link)//Found joint
				{
					//gzdbg<<"Link Found"<<bodywrenches_msg.body_name[count]<<std::endl;//#DEBUG
					gazebo::math::Vector3 target_force(bodywrenches_msg.wrench[6*count],bodywrenches_msg.wrench[6*count+1],bodywrenches_msg.wrench[6*count+2]);
					gazebo::math::Vector3 target_torque(bodywrenches_msg.wrench[6*count+3],bodywrenches_msg.wrench[6*count+4],bodywrenches_msg.wrench[6*count+5]);
					link->SetForce(target_force);
					link->SetTorque(target_torque);
				}
			}
		}
		void setjointefforts()
		{
			memin1.Read(jointeffort_msg);//Reading the message dont care about time stamp 
			for(int count = 0; count < jointeffort_msg.joint_names.size();count++)
			{
				physics::JointPtr joint = pub_joints[jointeffort_msg.joint_names[count]];;
				if(joint)//Found joint
				{
					//gzdbg<<"Joint Found"<<jointeffort_msg.joint_names[count]<<std::endl;//#DEBUG
					joint->SetForce(0,jointeffort_msg.effort[count]);
				}
			}
		}

		void publishsimtime()
		{
			gazebo::common::Time currentTime = world->GetSimTime();
			std_msgs::Empty emptymsg;
			memout3.Write(emptymsg, currentTime.sec, currentTime.nsec);
		}

		bool setjointstate(const gazebo_rosmatlab_bridge::JointState &joint_state)
		{
			physics::JointPtr joint = pub_joints[joint_state.joint_name];;

			if(joint) //If it is a joint
			{
				switch(joint_state.joint_type)
				{
					case PRISMATIC :
					case SCREW :
					case REVOLUTE :
						//gzdbg<<"Joint Angle: "<<joint_state.angle.x<<std::endl;
						joint->SetAngle(0,joint_state.angle.x);
						joint->SetVelocity(0,joint_state.vel_angle.x);
						break;
					case UNIVERSAL :
					case REVOLUTE2 :
						joint->SetAngle(0,joint_state.angle.x);
						joint->SetAngle(1,joint_state.angle.y);
						joint->SetVelocity(0,joint_state.vel_angle.x);
						joint->SetVelocity(1,joint_state.vel_angle.y);
						break;
					case BALL :
						joint->SetAngle(0,joint_state.angle.x);
						joint->SetAngle(1,joint_state.angle.y);
						joint->SetAngle(2,joint_state.angle.z);
						joint->SetVelocity(0,joint_state.vel_angle.x);
						joint->SetVelocity(1,joint_state.vel_angle.y);
						joint->SetVelocity(2,joint_state.vel_angle.z);
						break;
				}
			}
		}


		bool setmodelstate(gazebo_msgs::ModelState &model_state)
		{
			gazebo::math::Vector3 target_pos(model_state.pose.position.x,model_state.pose.position.y,model_state.pose.position.z);
			gazebo::math::Quaternion target_rot(model_state.pose.orientation.w,model_state.pose.orientation.x,model_state.pose.orientation.y,model_state.pose.orientation.z);
			target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
			gazebo::math::Pose target_pose(target_pos,target_rot);
			gazebo::math::Vector3 target_pos_dot(model_state.twist.linear.x,model_state.twist.linear.y,model_state.twist.linear.z);
			gazebo::math::Vector3 target_rot_dot(model_state.twist.angular.x,model_state.twist.angular.y,model_state.twist.angular.z);

			gazebo::physics::ModelPtr model = world->GetModel(model_state.model_name);
			if (!model)
			{
				gzdbg<<"Updating ModelState: model ["<<model_state.model_name<<"] does not exist";
				return false;
			}
			else
			{
				bool is_paused = world->IsPaused();
				world->SetPaused(true);
				model->SetWorldPose(target_pose);
				world->SetPaused(is_paused);

				// set model velocity
				model->SetLinearVel(target_pos_dot);
				model->SetAngularVel(target_rot_dot);

				return true;
			}
		}
		public: 
		GazeboMatlabPlugin() :WorldPlugin(),
		memout1("/tmp/out_linkstates.tmp", 5000),//#TODO add a buffering system to the memout (Not needed right now)
		memout2("/tmp/out_jointstates.tmp", 5000),
		memout3("/tmp/out_time.tmp", 50),
		memin1("/tmp/in_jointefforts.tmp", 5000),
		memin2("/tmp/in_bodywrenches.tmp", 5000),
		memin3("/tmp/in_setmodelstate.tmp", 5000),
		memin4("/tmp/in_gazebocontrol.tmp", 5000),
		memin5("/tmp/in_setjointstate.tmp", 5000)
		{
		}

		void ReceivingThread()
		{
			gazebo_msgs::ModelState modelstate_msg;
			std_msgs::String string_msg;
			gazebo_rosmatlab_bridge::JointState jointstate_msg;

			//This thread is used control gazebo from matlab:
			while(world)// Do this as long as world is valid (Gazebo is alive)
			{
				if(memin3.Read(modelstate_msg))
				{
					setmodelstate(modelstate_msg);
				}

				if(memin4.Read(string_msg))
				{
					if(!strcmp(string_msg.data.c_str(),"reset"))//Can also make this integers in a message
					{
						world->Reset();
					}
					else if(!strcmp(string_msg.data.c_str(),"pause"))
					{
						world->SetPaused(true);
					}
					else if(!strcmp(string_msg.data.c_str(),"start"))
					{
						world->SetPaused(false);
					}
				}

				if(memin5.Read(jointstate_msg))
				{
					setjointstate(jointstate_msg);
				}
				usleep(1000);//1Khz checking
			}
		}
		
		//Gets called after world has been loaded
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
			//Store ptr for later use;
			world = _world;

			string worldname = _world->GetName();
			ROS_INFO("Worldname %s",worldname.c_str());


			//pause the world
			//ROS_INFO("Pausing World...");
			//world->SetPaused(true);

			gzdbg<<"Loading Links..."<<endl;
			if(_sdf->HasElement("links"))
			{
				string link_string = _sdf->GetElement("links")->Get<string>();
				gzdbg<<link_string<<std::endl;//#DEBUG

				stringstream ss(link_string);//Create a stream from the link strings
				while(ss.good())
				{
					string substr;
					getline(ss,substr,';');//String delimited by semicolon
					//	 joint = boost::dynamic_pointer_cast<gazebo::physics::Joint>(world_->GetByName(jointeffort_msg.joint_names[count]));//Can simplify this by storing Pointers #TODO
					physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(world->GetEntity(substr));
					if(!link)
					{
						gzdbg<<"Failed to load link: "<<substr<<endl;
						continue;
					}
					pub_links[substr] = link;
				}
			}
			else
			{
				gzdbg<<"Cannot Find links to load"<<endl;
			}

			gzdbg<<"Loading Joints..."<<endl;
			if(_sdf->HasElement("joints"))
			{
				string joint_string = _sdf->GetElement("joints")->Get<string>();
				gzdbg<<joint_string<<std::endl;//#DEBUG

				stringstream ss1(joint_string);//Create a stream from the link strings
				while(ss1.good())
				{
					string substr;
					getline(ss1,substr,';');//String delimited by semicolon
					physics::JointPtr joint = boost::dynamic_pointer_cast<gazebo::physics::Joint>(world->GetByName(substr));
					if(!joint)
					{
						gzdbg<<"Failed to load: "<<substr<<endl;
						continue;
					}
					pub_joints[substr] = joint;
				}
			}
			else
			{
				gzdbg<<"Cannot Find joints to load"<<endl;
			}

			/*hokuyo_sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::get_sensor(worldname + "::" + modelname + "::hokuyo::link::laser"));
				imu_sensor = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor(worldname + "::" + modelname + "::imu::link::imu_sensor"));
				kinect_sensor = boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensors::get_sensor(worldname + "::" + modelname + "::kinect::link::camera"));

				if(!hokuyo_sensor)
				ROS_WARN("Cannot find hokuyo laser scanner");

				if(!imu_sensor)
				ROS_WARN("Cannot find imu");

				if(!kinect_sensor)
				ROS_WARN("Cannot find kinect");
			 */
			//Create a Callback for Worldupdate used for inputting forces
			this->updateConnection1 = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&GazeboMatlabPlugin::publishlinksandjoints, this));
			this->updateConnection2 = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&GazeboMatlabPlugin::setlinkwrenches, this));
			this->updateConnection3 = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&GazeboMatlabPlugin::setjointefforts, this));
			this->updateConnection4 = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&GazeboMatlabPlugin::publishsimtime, this));
			recv_thread_.reset(new boost::thread(boost::bind(&GazeboMatlabPlugin::ReceivingThread, this)));
		}
	};
	GZ_REGISTER_WORLD_PLUGIN(GazeboMatlabPlugin)
}
