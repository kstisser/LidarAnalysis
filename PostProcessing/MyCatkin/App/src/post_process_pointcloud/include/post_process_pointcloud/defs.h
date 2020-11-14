#include <string>
#include <ros/console.h>

namespace Topics
{
   std::string Sensor0Prefix = "/device_0/sensor_0";
   std::string Sensor1Prefix = "/device_0/sensor_1";

   std::string CameraImageTopic = Sensor1Prefix + "/Color_0/image/data";
   std::string IRimageTopic = Sensor0Prefix + "/Infrared_0/image/data";
   std::string PointCloud = Sensor0Prefix + "/Depth_0/image/data";

   std::string NumPointsAndColor = "/numPointsAndColor";
}

namespace Scenarios
{
   enum Size
   {
      MM40,
      MM80,
      MM160,
      NAsize
   };

   enum ObjectTemperature
   {
      C40,
      C45,
      C50,
      Ambient,
      NAtemp
   };

   enum ObjectColor
   {
      Black,
      RoseGold,
      Blue,
      Silver,
      White
   };

   enum Distance
   {
      FT2,
      FT4,
      FT6
   };

   float getDistanceMeters(Distance distance)
   {
      if(distance == Distance::FT2)
      {
         return 0.6096;
      }
      if(distance == Distance::FT4)
      {
         return 1.2192;
      }
      if(distance == Distance::FT6)
      {
         return 1.8288;
      }
      std::string d = std::to_string(distance);
      ROS_ERROR_STREAM("Didn't find lookup value!" << d);
      return 1;
   }

   struct scenario
   {
      Size size;
      ObjectTemperature temperature;
      ObjectColor color;
      Distance distance;
   };
}