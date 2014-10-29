#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
//#include <transport/transport.hh>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo/plugins/CameraPlugin.hh>

#include "gazebo/msgs/msgs.hh"

#include <unistd.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>

//Memcopy Topics will have a different type of memcpy

#include <gazebo_rosmatlab_bridge/shared_mmap.h> //To create and use a shared memory

using namespace std;

namespace gazebo
{
	class MatlabCameraPlugin : public CameraPlugin
	{
		private:
		boost::shared_ptr<SharedMmap<unsigned char> > imageMap;//Shared memory for sharing image Data
		uint32_t imagesize[3];
		public: 
		MatlabCameraPlugin() :CameraPlugin()
		{
		}
		//Gets called after world has been loaded
		void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
		{
			CameraPlugin::Load(_parent, _sdf);
			std::string camerammapname = _sdf->GetElement("CameraMmapTopicName")->Get<string>();
			std::string mmap_location = std::string("/tmp/"+camerammapname+".tmp"); 
			imagesize[0] = this->height;
			imagesize[1] = this->width;
			imagesize[2] = this->depth;
			//imagesize[2] = this->depth;
			gzdbg<<"Mmap_location: "<<mmap_location<<std::endl;
			imageMap.reset(new SharedMmap<unsigned char>(mmap_location.c_str(),(imagesize[0]*imagesize[1]*imagesize[2])));//#TODO Buffer based on size of image
		}
		//Update the 
    protected: 
		//Overriding Parent Camera Plugin to transport the camera data
		virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
		{
//			gzdbg<<"Depth: "<<_depth<<std::endl;
 			common::Time update_time =  this->parentSensor->GetLastUpdateTime(); 
			imageMap->Write(_image,imagesize,update_time.sec, update_time.nsec);//Write to shared memory
				//gzdbg<<"Update_time: "<<update_time.sec<<"\t"<<update_time.nsec<<std::endl;
		}
	};
	GZ_REGISTER_SENSOR_PLUGIN(MatlabCameraPlugin)
}
