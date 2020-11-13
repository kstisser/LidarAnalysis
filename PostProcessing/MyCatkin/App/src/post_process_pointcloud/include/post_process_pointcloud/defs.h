#include <string>

namespace Topics
{
   std::string Sensor0Prefix = "/device_0/sensor_0";
   std::string Sensor1Prefix = "/device_0/sensor_1";
   std::string CameraImageTopic = Sensor1Prefix + "/Color_0/image/data";
   std::string IRimageTopic = Sensor0Prefix + "/Infrared_0/image/data";
   std::string PointCloud = Sensor0Prefix + "/Depth_0/image/data";
}