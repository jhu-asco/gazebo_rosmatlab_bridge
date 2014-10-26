#include <gazebo/gazebo.hh>
#include <common/common.hh>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <transport/transport.hh>
#include "gazebo/msgs/msgs.hh"

#include <unistd.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

//Memcopy Topics
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"

#include "gazebo_rosmatlab_bridge/JointEfforts.h"
#include "gazebo_rosmatlab_bridge/BodyWrenches.h"
#include "std_msgs/Empty.h"

#include <gazebo_rosmatlab_bridge/mmap.h> //To create and use a shared memory

using namespace std;

namespace gazebo
{
	class GazeboMatlabPlugin : public WorldPlugin
	{
		private:
		physics::WorldPtr world;//World Pointer for accessing correspoding links and models
		map<string, physics::JointPtr> pub_joints;
		map<string, physics::LinkPtr> pub_links;//Smaller map of links and joints used by us

		/*vector<physics::JointPtr> pub_joints;//Joints for which data is published or are controlled
		vector<physics::LinkPtr> pub_links;//Links for which datat is published or are controlled
		vector<string> link_strings;//Storing the link names for searching later
		vector<string> joint_strings;
		*/

		Mmap<gazebo_msgs::LinkStates> memout1;//Shared memory for directly sending Link Data
		//Mmap<gazebo_msgs::ModelStates> memout2;//Shared memory for directly sending Model Data
		Mmap<std_msgs::Empty> memout3;//Shared memory for directly sending the sim time
		Mmap<gazebo_rosmatlab_bridge::JointEfforts> memin1;//Shared memory for directly receiving joint efforts
		Mmap<gazebo_rosmatlab_bridge::BodyWrenches> memin2;//Shared memory for directly sending Model Data

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
						geometry_msgs::Pose pose;//Dummy nothing in there right now
						geometry_msgs::Twist twist;
						if (joint->HasType(physics::Base::HINGE_JOINT))
						{
							//	jointname += "_REVOLUTE";
							twist.linear.x = joint->GetVelocity(0);
							twist.angular.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::HINGE2_JOINT))
						{
							//	jointname += "_REVOLUTE2";
							twist.linear.x = joint->GetVelocity(0);
							twist.linear.y = joint->GetVelocity(1);
							twist.angular.x = joint->GetAngle(0).Radian();
							twist.angular.y = joint->GetAngle(1).Radian();
						}
						else if (joint->HasType(physics::Base::BALL_JOINT))
						{
							//jointname += "_BALL";
							twist.linear.x = joint->GetVelocity(0);
							twist.linear.y = joint->GetVelocity(1);
							twist.linear.z = joint->GetVelocity(2);

							twist.angular.x = joint->GetAngle(0).Radian();
							twist.angular.y = joint->GetAngle(1).Radian();
							twist.angular.z = joint->GetAngle(2).Radian();
						}
						else if (joint->HasType(physics::Base::SLIDER_JOINT))
						{
							//jointname += "_PRISMATIC";
							twist.linear.x = joint->GetVelocity(0);
							twist.angular.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::SCREW_JOINT))
						{
							//jointname += "_SCREW";
							twist.linear.x = joint->GetVelocity(0);
							twist.angular.x = joint->GetAngle(0).Radian();
						}
						else if (joint->HasType(physics::Base::UNIVERSAL_JOINT))
						{
							//jointname += "_UNIVERSAL";
							twist.linear.x = joint->GetVelocity(0);
							twist.linear.y = joint->GetVelocity(1);
							twist.angular.x = joint->GetAngle(0).Radian();
							twist.angular.y = joint->GetAngle(1).Radian();
						}
						link_states.name.push_back(std::string("JOINT::") + it->first);//Prepend JOINT to separate links and joints for user end
						link_states.twist.push_back(twist);
						link_states.pose.push_back(pose);//Pushing back into the list
					}
				}

				if(world)
				{
					common::Time current_time = world->GetSimTime();
					bool status = memout1.Write( link_states, (uint32_t)current_time.sec, (uint32_t)current_time.nsec);
					//gzdbg<<"Writing Link States: "<<status<<endl;//#DEBUG
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

		public: 
		GazeboMatlabPlugin() :WorldPlugin(),
		memout1("/tmp/out_linkstates.tmp", 5000),//#TODO add a buffering system to the memout (Not needed right now)
		//memout2("/tmp/out_modelstates.tmp", 5000),
		memout3("/tmp/out_time.tmp", 50),
		memin1("/tmp/in_jointefforts.tmp", 5000),
		memin2("/tmp/in_bodywrenches.tmp", 5000)
		{
		}
		/*~GazeboMatlabPlugin()
		{
		}
		*/
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
		}
	};
	GZ_REGISTER_WORLD_PLUGIN(GazeboMatlabPlugin)
}
