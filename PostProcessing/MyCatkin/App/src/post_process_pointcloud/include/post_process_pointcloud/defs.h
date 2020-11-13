#include <string>

namespace Topics
{
   std::string Sensor0Prefix = "/device_0/sensor_0";
   std::string Sensor1Prefix = "/device_0/sensor_1";
   std::string CameraImageTopic = Sensor1Prefix + "/Color_0/image/data";
   std::string IRimageTopic = Sensor0Prefix + "/Infrared_0/image/data";
   std::string PointCloud = Sensor0Prefix + "/Depth_0/image/data";
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

   struct scenario
   {
      Size size;
      ObjectTemperature temperature;
      ObjectColor color;
      Distance distance;
   };
}