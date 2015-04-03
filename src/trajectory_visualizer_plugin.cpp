#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
//#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <visualization_msgs/Marker.h>

#include <gazebo_rosmatlab_bridge/mmap.h> //To create and use a shared memory

//#include "gazebo/gui/GuiEvents.hh"//For getting event of key press
#include "gazebo/msgs/msgs.hh"
#include <unistd.h>
#include <iostream>
#include <vector>
#define N 500 


/* Creating a Visual plugin for plotting the COM of the link of interest */
using namespace std;

namespace gazebo
{
  namespace rendering
  {
    class TrajectoryVisualizingPlugin : public VisualPlugin
    {
      typedef const boost::shared_ptr<msgs::Quaternion const> ConstQuaternionPtr;
      typedef std::map<int, DynamicLines *> LineMap;

      private:
      rendering::VisualPtr visual;
      LineMap linemap;
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;
      math::Vector3 previousposition;
      Mmap<visualization_msgs::Marker> memin1;//Shared memory for setting Model State
      visualization_msgs::Marker inputmsg;
  //		rendering::ArrowVisual axis;

      protected:
      void UpdateChild()
      {
        if(memin1.Read(inputmsg))
        {
          assert(this->visual);
          //Make sure input msg is a point strip
          if(inputmsg.type == inputmsg.LINE_STRIP)
          {
            //Create a line:
            LineMap::iterator it = linemap.find(inputmsg.id);
            DynamicLines *line;
            if(it != linemap.end())
            {
              //Line already exists;
              line = it->second;
              if(inputmsg.action == 2)//If asked to delete the line then just delete and return
              {
                if(line->GetPointCount() <= 2)//2 is because of the workaround Check if 1 can be done will be more intuitive
                {
                  gzdbg<<"Deleting existing line"<<endl;
                  this->visual->DeleteDynamicLine(line);//Delete that line
                  linemap.erase(it);//Remove this iterator from map
                }
                else
                {
                  gzdbg<<"Cannot delete the line before calling with Null trajectory"<<endl;
                }
                return;
              }

              if(inputmsg.action == 0)
                line->Clear();//Clear all the points if we are modifying the existing line rather than adding new points to the line
            }
            else
            {
              if(inputmsg.action == 2)//If asked to delete the line then no need to create one
              {
                return;
              }
              //No line exist create one:
              line = this->visual->CreateDynamicLine(RENDERING_LINE_STRIP);
              linemap[inputmsg.id] =  line;

              Ogre::String matName = "rgb" + Ogre::StringConverter::toString(inputmsg.color.r)+Ogre::StringConverter::toString(inputmsg.color.g)+Ogre::StringConverter::toString(inputmsg.color.b)+Ogre::StringConverter::toString(inputmsg.color.a);
              Ogre::MaterialPtr materialPtr = Ogre::MaterialManager::getSingleton().getByName(matName);
              if(materialPtr.isNull())
              {
                gzdbg<<"Creating New Material"<<endl;
                materialPtr = Ogre::MaterialManager::getSingleton().create(matName, 
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                Ogre::ColourValue color(inputmsg.color.r, inputmsg.color.g, inputmsg.color.b, inputmsg.color.a);
                materialPtr->setAmbient(color);
                materialPtr->setDiffuse(color);
                //Ogre::Pass
                //materialPtr->setPointSize(200.0);//Thickness of line
              }
              line->setMaterial(matName);
                            
              //line->SetDiffuse(common::Color(inputmsg.color.r, inputmsg.color.g, inputmsg.color.b, inputmsg.color.a));
              /*if(inputmsg.color.r == 1)
                line->setMaterial("Gazebo/Red");
              else if(inputmsg.color.b == 1)
                line->setMaterial("Gazebo/Blue");
              else if(inputmsg.color.g == 1)
                line->setMaterial("Gazebo/Green");
              else
                line->setMaterial("Gazebo/Red");//Default
                */

              line->setVisibilityFlags(GZ_VISIBILITY_GUI);
              this->visual->SetVisible(true);
            }

            if(inputmsg.points.size() == 0)
            {
              math::Vector3 pos(0,0,0);
              line->AddPoint(pos);//Add two points
              line->AddPoint(pos);//Add two points at same position and really this is a bug in Gazebo and this is a workaround for that
            }

            //Iterate through the points and add them to the line strip:
            for(int count = 0; count < inputmsg.points.size(); count++)
            {
              math::Vector3 pos(inputmsg.points[count].x, inputmsg.points[count].y, inputmsg.points[count].z);
              line->AddPoint(pos);
              //cout<<"pos["<<count<<"]: "<<pos.x<<"\t"<<pos.y<<"\t"<<pos.z<<"\t"<<endl;
            }
          }
        }
        /*//Add Position of the visual to the line:
          math::Pose currpose = this->visual->GetWorldPose();
        //math::Vector3 position = this->visual->GetPosition();
        if((currpose.pos - previousposition).GetLength() > 0.01)
        {
        //New position is atleast of 1cm from previous position
        line->AddPoint(currpose.pos);
        gzdbg<<"Position: "<<currpose.pos.x<<"\t"<<currpose.pos.y<<"\t"<<currpose.pos.z<<"\t"<<endl;
        previousposition = currpose.pos;//Update previous pos
        }
         */
      }

      public: 

      TrajectoryVisualizingPlugin() :VisualPlugin()
                              , previousposition(0,0,0)
                              ,memin1("/tmp/in_publishtrajectory.tmp",50000)
      {
        //printf("Hello World3!\n");
      }

      // Destructor
  /*		~VisualPluginTutorial()
      {
        // Finalize the visualizer
        node->Fini();
        visual->Fini();
      }
      */


      void Load(rendering::VisualPtr visual_, sdf::ElementPtr _sdf)
      {
        //Store ptr for later use;
        visual = visual_;
        //Create shared memory for receiving request to render trajectories:

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectRender(
            boost::bind(&TrajectoryVisualizingPlugin::UpdateChild, this));
        gzdbg<<"Done Loading Plugin!"<<endl;
      }

      /*virtual void Reset()
      {
        if(line)
          line->Clear();
        gzdbg<<"Reset Plugin"<<endl;
      }
      */
    };
    GZ_REGISTER_VISUAL_PLUGIN(TrajectoryVisualizingPlugin)
  }
}
