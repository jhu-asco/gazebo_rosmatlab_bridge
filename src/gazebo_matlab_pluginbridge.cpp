/**This file is part of GAZEBO_ROSMATLAB_BRIDGE.
 *
 * gazebo_matlab_pluginbridge.cpp is a free gazebo world plugin: you can redistribute it and/or modify
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
 **/
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/msgs/msgs.hh"

#include <unistd.h>
#include <iostream>
#include <vector>
//#include <algorithm>
#include <ctime>

#include <ros/ros.h>

//Memcopy Topics
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
//#include "gazebo_msgs/ModelState.h"

#include "gazebo_rosmatlab_bridge/JointEfforts.h"
#include "gazebo_rosmatlab_bridge/BodyWrenches.h"
#include "gazebo_rosmatlab_bridge/JointState.h"
#include "gazebo_rosmatlab_bridge/PhysicsEngineConfig.h"
#include "gazebo_rosmatlab_bridge/JointStates.h"
#include "gazebo_rosmatlab_bridge/AvailableNames.h"
#include "gazebo_rosmatlab_bridge/RunSimulation.h"
#include "gazebo_rosmatlab_bridge/CompleteModelState.h"

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
      physics::PhysicsEnginePtr physicsEngine;
      vector<physics::JointPtr> joints;
      vector<physics::LinkPtr> links;//Smaller vector of links and joints used by us

      vector<string> names_links;
      vector<string> names_joints;
      vector<uint8_t> joint_types;

      boost::shared_ptr<boost::thread> recv_thread_;

      Mmap<gazebo_msgs::LinkStates> memout1;//Shared memory for directly sending Link Data
      Mmap<gazebo_rosmatlab_bridge::JointStates> memout2;//Shared memory for directly sending joint data
      Mmap<gazebo_rosmatlab_bridge::AvailableNames> memout3;//Shared memory for directly sending joint data

      Mmap<gazebo_rosmatlab_bridge::CompleteModelState> memin1;//Shared memory for setting Model State
      Mmap<gazebo_rosmatlab_bridge::JointState> memin2;//Shared memory for setting Joint State
      Mmap<gazebo_rosmatlab_bridge::RunSimulation> memin3;//Shared memory for sending simulation req
      Mmap<std_msgs::String> memin4;//Shared memory for sending reset req
      Mmap<gazebo_rosmatlab_bridge::PhysicsEngineConfig> memin5;//Receiving physics config msgs

      //Memout messages
      gazebo_rosmatlab_bridge::JointStates joint_states;
      gazebo_rosmatlab_bridge::RunSimulation simulation_req;
      gazebo_msgs::LinkStates link_states;
      gazebo::common::Time prevcollection_time;

      int stepcount;//Step count for world Update
      int physxcount;//Number of Physics Iterations done
      //int interpolation_steps;//Number of steps to keep the control constant over %TODO Introduce some bezier or other kind of interpolation

      event::ConnectionPtr updateConnection1;
      bool updatingjoint;
      gazebo::common::Time worldupdate_time;
      //std::clock_t    start_worldupdate;

      inline void FindLinkandJointStatus()
      {
        // fill link_states
        for(int count1 = 0; count1 < links.size(); count1++)
        {
          gazebo::physics::LinkPtr body = links[count1];
          if (body)
          {
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
        link_states.name = names_links;
        //Fill Joint States
        for(int count1 = 0; count1 < joints.size(); count1++)
        {
          gazebo::physics::JointPtr joint = joints[count1];
          if(joint) //If it is a joint
          {
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
          }
        }
        joint_states.joint_name = names_joints;
      }


      bool SetJointState(const gazebo_rosmatlab_bridge::JointState &joint_state)
      {

        physics::JointPtr joint = joints[joint_state.joint_ind];;

        if(joint) //If it is a joint
        {
          switch(joint_state.joint_type)
          {
            case PRISMATIC :
            case SCREW :
            case REVOLUTE :
              //gzdbg<<"Joint Angle: "<<joint_state.angle.x<<"\t"<<joint_state.vel_angle.x<<std::endl;
              joint->SetAngle(0,joint_state.angle.x);
              joint->SetVelocity(0,joint_state.vel_angle.x);
              //gzdbg<<"Joint Angle after setting: "<<joint->GetAngle(0).Radian()<<"\t"<<joint->GetVelocity(0)<<endl;

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
        /*if(world)
          {
          updatingjoint = true;//Stops the callbacks in updateworld until joint is updated
          world->StepWorld(1);//Step Once Check why it is breaking
          }
         */
        //usleep(5000);//1ms sleep (Check if this is enough)
      }


      bool SetModelState(gazebo_rosmatlab_bridge::CompleteModelState &completemodel_state)
      {
        gazebo_msgs::ModelState &model_state = completemodel_state.model_state;
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
          //Set Model and Joint States:
          //gazebo_rosmatlab_bridge::JointStates &joint_states = completemodelstate.joint_states;
          std::map<std::string, double> jointpositions;

          bool is_paused = world->IsPaused();
          world->SetPaused(true);
          if(completemodel_state.change_modelstate)
          {
            model->SetWorldPose(target_pose);
          }

          //gzdbg<<"Number of states: "<<completemodel_state.joint_states.size()<<endl;
          for(int count = 0;count < completemodel_state.joint_states.size();count++)
          {
            gazebo_rosmatlab_bridge::JointState &jointstate = completemodel_state.joint_states[count];
            jointpositions[names_joints[jointstate.joint_ind]] = jointstate.angle.x;//Only useful for x, y, z
            //  gzdbg<<"Joint Positions: "<<names_joints[jointstate.joint_ind]<<"\t"<<jointstate.angle.x<<endl;
          }
          model->SetJointPositions(jointpositions);

          world->SetPaused(is_paused);

          // set model velocity
          if(completemodel_state.change_modelstate)
          {
            model->SetLinearVel(target_pos_dot);
            model->SetAngularVel(target_rot_dot);
          }

          /*
          //Set Joint Velocities:
          for(int count = 0; count < completemodel_state.joint_states.size();count++)
          {
          uint32_t index = completemodel_state.joint_states[count].joint_ind;
          joints[index]->SetVelocity(0,completemodel_state.joint_states[count].vel_angle.x);//Set Joint Velocity
          joints[index]->Update();
          gzdbg<<"joints[index]: "<<index<<"\t"<<joints[index]->GetName()<<endl;
          gzdbg<<"Joint Velocities: "<<completemodel_state.joint_states[count].vel_angle.x<<"\t"<<joints[index]->GetVelocity(0)<<endl;
          }
           */

          return true;
        }
      }

      inline void transformWrench( gazebo::math::Vector3 &target_force, gazebo::math::Vector3 &target_torque,
          gazebo::math::Vector3 reference_force, gazebo::math::Vector3 reference_torque,
          gazebo::math::Pose target_to_reference )
      {
        // rotate force into target frame
        target_force = target_to_reference.rot.RotateVector(reference_force);
        // rotate torque into target frame
        target_torque = target_to_reference.rot.RotateVector(reference_torque);

        // target force is the refence force rotated by the target->reference transform
        target_torque = target_torque + target_to_reference.pos.Cross(target_force);
      }

      void WorldUpdate()
      {
        //gzdbg<<"U1"<<endl;
        if(this->updatingjoint == true)
        {
          this->updatingjoint = false;
          return;
        }

        //gzdbg<<"Time :"<<(gazebo::common::Time::GetWallTime()-worldupdate_time)<<endl;
        worldupdate_time = gazebo::common::Time::GetWallTime();

        //This  applies the body wrenches to the bodies specified from message
        gazebo_rosmatlab_bridge::BodyWrenches &bodywrenches_msg = simulation_req.bodywrenches;
        int nofbodies = bodywrenches_msg.body_ind.size();
        //gzdbg<<"Nof Bodies: "<<nofbodies<<endl;
        for(int count = 0; count < nofbodies;count++)
        {
          //gzdbg<<"Verifying "<<jointeffort_msg.joint_names[count]<<std::endl;//#DEBUG
          gazebo::physics::LinkPtr link = links[bodywrenches_msg.body_ind[count]];
          if(link)//Found joint
          {
            //gzdbg<<"Link Found"<<bodywrenches_msg.body_name[count]<<std::endl;//#DEBUG
            gazebo::math::Vector3 reference_force(bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))],bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))+1],bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))+2]);
            gazebo::math::Vector3 reference_torque(bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))+3],bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))+4],bodywrenches_msg.wrench[6*(count+nofbodies*(stepcount-1))+5]);
            gazebo::math::Pose target_to_reference = link->GetWorldPose();
            // rotate force into target frame
            gazebo::math::Vector3 target_force = target_to_reference.rot.RotateVector(reference_force);
            // rotate torque into target frame
            gazebo::math::Vector3 target_torque = target_to_reference.rot.RotateVector(reference_torque);
            //transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
            link->SetForce(target_force);
            link->SetTorque(target_torque);
          }
        }
        gazebo_rosmatlab_bridge::JointEfforts &jointeffort_msg = simulation_req.jointefforts;
        //This applies joint efforts to the specified message
        int nofjoints = jointeffort_msg.joint_ind.size();
        for(int count = 0; count < nofjoints; count++)
        {
          //  gzdbg<<"Joint Index: "<<jointeffort_msg.joint_ind[count]<<endl;
          physics::JointPtr joint = joints[jointeffort_msg.joint_ind[count]];;
          if(joint)//Found joint
          {
            //gzdbg<<"Joint Found"<<jointeffort_msg.joint_names[count]<<std::endl;//#DEBUG
            joint->SetForce(0,jointeffort_msg.effort[count + nofjoints*(stepcount-1)]);
            /*
               if(physxcount == simulation_req.steps[stepcount])
               {
               gzdbg<<"Joint Effort: "<<jointeffort_msg.effort[count+nofjoints*(stepcount-1)]<<endl;
               }
             */
          }
        }
        //gazebo::common::Time currentTime = world->GetSimTime();
        if(physxcount == simulation_req.steps[stepcount])
        {
          FindLinkandJointStatus();//For now collecting states at same rate as ctrl
          //gzdbg<<"Step: "<<physxcount<<" "<<joint_states.angle[2*stepcount].x<<" "<<joint_states.angle[2*stepcount+1].x<<" "<<joint_states.vel_angle[2*stepcount].x<<" "<<joint_states.vel_angle[2*stepcount+1].x<<endl;
          stepcount++;
        }
        physxcount++;
      }
      /*
         void findgradient(std::string &model_name)
         {
      //First pause the world
      world->SetPaused(true);
      //set internal flag so that it does not read from stream while updating world
      internalcontrol  = true;
      //Read Nominal Control to apply
      gazebo_rosmatlab_bridge::JointEfforts nominaljointeffort_msg;//Joint Effort message received from client
      gazebo_rosmatlab_bridge::BodyWrenches nominalbodywrenches_msg;//Joint Effort message received from client
      memin2.Read(nominalbodywrenches_msg);//Reading the message dont care about time stamp
      memin1.Read(nominaljointeffort_msg);//Reading the message dont care about time stamp 
      gazebo_msgs::LinkStates link_states;
      gazebo_rosmatlab_bridge::JointStates joint_states;
      //Store Model State:
      gazebo::physics::ModelPtr model = world->GetModel(model_name);
      gazebo::physics::ModelState model_state(model);
      //forloop
      //Perturb and copy the control  to jointeffort_msg and bodywrenches_msg
      //Run the world by 1 step
      world->StepWorld(1);
      //Evaluate Link and joint states
      findlinkandjointstatus(link_states, joint_states);
      //Find the Gradient based on reading link and joint states
      //Reset Model to how it was
      model->SetState(model_state);
      //endforloop

      //Alternative to this is to run stepworld by N steps needed for computing finitedifferences
      //Then based on the count of the stepworld, the control is perturbed and model is reset inside the WorldUpdateStep.
      //For this we should have only one WorldUpdatestep where we collect link data and then reset the model to nominal state
      //Apply Perturbed control and find the new linkandjoint states
      }
       */
    public: 
      GazeboMatlabPlugin() :WorldPlugin(), stepcount(0),physxcount(0), updatingjoint(false),
      memout1("/tmp/out_linkstates.tmp", 50000),//#TODO add a buffering system to the memout (Not needed right now)
      memout2("/tmp/out_jointstates.tmp", 50000),//#TODO add a buffering system to the memout (Not needed right now)
      memout3("/tmp/out_names.tmp", 5000),//#TODO add a buffering system to the memout (Not needed right now)
      memin1("/tmp/in_setmodelstate.tmp", 5000),
      memin2("/tmp/in_setjointstate.tmp", 5000),
      memin3("/tmp/in_simulationreq.tmp", 5000),
      memin4("/tmp/in_stringreq.tmp",500),
      memin5("/tmp/in_physicsconfig.tmp",500)
    {
    }

      void ReceivingThread()
      {
        gazebo_rosmatlab_bridge::CompleteModelState modelstate_msg;
        std_msgs::String string_msg;
        gazebo_rosmatlab_bridge::PhysicsEngineConfig physics_msg;
        gazebo_rosmatlab_bridge::JointState jointstate_msg;
        gazebo_rosmatlab_bridge::AvailableNames names;
        names.link_names = names_links;
        names.joint_names = names_joints;
        names.joint_types = joint_types;
        //if(physicsEngine)
        //names.physics_timestep = physicsEngine->GetMaxStepSize();//Send the physics time step back to matlab

        //This thread is used control gazebo from matlab:
        while(world)// Do this as long as world is valid (Gazebo is alive)
        {
          if(memin1.Read(modelstate_msg))
          {
            SetModelState(modelstate_msg);
          }

          if(memin2.Read(jointstate_msg))
          {
            SetJointState(jointstate_msg);
          }

          if(memin3.Read(simulation_req))
          {
            physxcount = 0;
            stepcount = 0;
            joint_states = gazebo_rosmatlab_bridge::JointStates();
            link_states = gazebo_msgs::LinkStates();//Set them back to 0 for next simulation
            uint32_t nofsteps = simulation_req.steps[simulation_req.steps.size()-1];//starts with 0
            //x0 state
            FindLinkandJointStatus();//For now collecting states at same rate as ctrl
            //Assuming 2 joints
            //  gzdbg<<"Step: "<<physxcount<<" "<<joint_states.angle[2*stepcount].x<<" "<<joint_states.angle[2*stepcount+1].x<<" "<<joint_states.vel_angle[2*stepcount].x<<" "<<joint_states.vel_angle[2*stepcount+1].x<<endl;

            stepcount = 1;///Sets stepcount to 1 to start the loop

            gazebo_rosmatlab_bridge::JointEfforts &jointeffort_msg = simulation_req.jointefforts;
            int nofjoints = jointeffort_msg.joint_ind.size();
            //gzdbg<<std::setprecision(10)<<endl;
            //This applies joint efforts to the specified message
            //gzdbg<<"NofJoints: "<<nofjoints<<endl;
            //#DEBUG:
            /*
               for(int count=0;count < simulation_req.steps.size()-1;count++)
               {
               for(int count1 = 0;count1 < nofjoints;count1++)
               gzdbg<<"Joint Efforts: "<<jointeffort_msg.effort[count1+nofjoints*count]<<endl;
               }
             */
            //world->SetSimTime(gazebo::common::Time(0.0));//Set Sim time to 0
            std::clock_t    start;
            start = std::clock();
            //start_worldupdate = std::clock();
            //gzdbg<<"Time :"<<gazebo::common::Time::GetWallTime()<<endl;
            world->StepWorld(nofsteps);
            //world->SetPaused(false);
            //world->Step(nofsteps);
            //gzdbg << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000000) << " microsec" << std::endl;
            //Final State xf
            FindLinkandJointStatus();
            //gzdbg<<"Step: "<<physxcount<<" "<<joint_states.angle[2*stepcount].x<<" "<<joint_states.angle[2*stepcount+1].x<<" "<<joint_states.vel_angle[2*stepcount].x<<" "<<joint_states.vel_angle[2*stepcount+1].x<<endl;
            //gzdbg<<"Joint Effort: "<<jointeffort_msg.effort[nofjoints*(stepcount-1)]<<endl;//Final Joint Effort
            //gzdbg<<"Joint Effort: "<<jointeffort_msg.effort[nofjoints*(stepcount-1)+1]<<endl;
            //After stepping send the data back:
            while(!memout1.Write(link_states));
            //gzdbg<<"Link State Info: "<<link_states.pose.size()<<endl;
            while(!memout2.Write(joint_states));
            //gzdbg<<"Joint State Info: "<<joint_states.angle.size()<<endl;
          }
          if(memin4.Read(string_msg))
          {
            if(string_msg.data == "worldreset")
            {
              //  gzdbg<<"Restting world"<<endl;
              if(world)
                world->Reset();
            }
          }
          if(memin5.Read(physics_msg))
          {
            if(physicsEngine)
            {
              physicsEngine->SetRealTimeUpdateRate(physics_msg.realtimerate);
              physicsEngine->SetMaxStepSize(physics_msg.timestep);
              gzdbg<<"TimeStep: "<<physics_msg.timestep<<"\t Real Time Update Rate: "<<physics_msg.realtimerate<<endl;
            }
          }
          if(memout3.Write(names))
          {
            gzdbg<<"Wrote Names"<<endl;
          }
          //memout3.Write(names);//Write the available Names if someone is ready to read
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

        //PhysicsEngine 
        if(_world->GetEnablePhysicsEngine ())
        {
          ROS_INFO("Physics engine enabled");
          physicsEngine = _world->GetPhysicsEngine();
          /*if(_sdf->HasElement("rate"))
            {
            double physics_rate = _sdf->GetElement("rate")->Get<double>();
            physicsEngine->SetRealTimeUpdateRate(physics_rate);
            }
            if(_sdf->HasElement("stepsize"))
            {
            double stepsize = _sdf->GetElement("stepsize")->Get<double>();
            physicsEngine->SetMaxStepSize(stepsize);
            }
           */
        }

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
            //   joint = boost::dynamic_pointer_cast<gazebo::physics::Joint>(world_->GetByName(jointeffort_msg.joint_names[count]));//Can simplify this by storing Pointers #TODO
            physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(world->GetEntity(substr));
            if(!link)
            {
              gzdbg<<"Failed to load link: "<<substr<<endl;
              continue;
            }
            this->links.push_back(link);
            this->names_links.push_back(substr);
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
            this->joints.push_back(joint);
            this->names_joints.push_back(substr);
            //Based on type of joint specify the joint types
            uint8_t joint_type;
            if (joint->HasType(physics::Base::HINGE_JOINT))
            {
              joint_type = REVOLUTE;
            }
            else if (joint->HasType(physics::Base::HINGE2_JOINT))
            {
              joint_type = REVOLUTE2;
            }
            else if (joint->HasType(physics::Base::BALL_JOINT))
            {
              joint_type = BALL;
            }
            else if (joint->HasType(physics::Base::SLIDER_JOINT))
            {
              joint_type = PRISMATIC;
            }
            else if (joint->HasType(physics::Base::SCREW_JOINT))
            {
              joint_type = SCREW;
            }
            else if (joint->HasType(physics::Base::UNIVERSAL_JOINT))
            {
              joint_type = UNIVERSAL;
            }
            this->joint_types.push_back(joint_type);
          }
        }
        else
        {
          gzdbg<<"Cannot Find joints to load"<<endl;
        }
        world->SetPaused(true);//Pause the world

        //Create a Callback for Worldupdate used for inputting forces
        this->updateConnection1 = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboMatlabPlugin::WorldUpdate, this));
        recv_thread_.reset(new boost::thread(boost::bind(&GazeboMatlabPlugin::ReceivingThread, this)));
      }
  };
  GZ_REGISTER_WORLD_PLUGIN(GazeboMatlabPlugin)
}
