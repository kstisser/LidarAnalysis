#include <string>
#include <ros/console.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

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
      Ambient
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

   struct Scenario
   {
      Size size;
      ObjectTemperature temperature;
      ObjectColor color;
      Distance distance;
   };
}


namespace SensorSpecs
{
   struct Sensor
   {
      int rowResolution_depth;
      int columnResolution_depth;
      int rowResolution_camera;
      int columnResolution_camera;      
      float angularFieldOfViewWidth_deg;
      float angularFieldOfViewHeight_deg;
      float minRange_m;
      float maxRange_m; 
   };

   Sensor getIntelLidar()
   {
      Sensor IntelLidarL515;
      IntelLidarL515.rowResolution_depth = 480;
      IntelLidarL515.columnResolution_depth = 640;
      IntelLidarL515.rowResolution_camera = 1080;
      IntelLidarL515.columnResolution_camera = 1920;
      IntelLidarL515.angularFieldOfViewWidth_deg = 55;
      IntelLidarL515.angularFieldOfViewHeight_deg = 70;
      IntelLidarL515.minRange_m = 0.25;
      IntelLidarL515.maxRange_m = 9.0;
      return IntelLidarL515;
   }
}

namespace Supportive
{
   struct ObjectSize
   {
      float width;
      float height;
   };

   struct PixelCount
   {
      int column;
      int row;
   };

   struct ObjectExpectations
   {
      ObjectSize size_m;
      PixelCount pixelCount_depth;
      PixelCount pixelCount_camera;
      float distanceValue_m;
      cv::Mat convolutionKernel_depth;
      cv::Mat convolutionKernel_camera;
   };

   ObjectSize getSizeMeters(Scenarios::Size s)
   {
      ObjectSize size_m;
      if(s == Scenarios::MM40)
      {
         size_m.width = 0.04;
         size_m.height = 0.04;
      }
      else if(s == Scenarios::MM80)
      {
         size_m.width = 0.08;
         size_m.height = 0.08;
      }
      else if(s == Scenarios::MM160)
      {
         size_m.width = 0.16;
         size_m.height = 0.16;
      }
      else //otherwise, it's the hand warmer size for this experiment
      {
         size_m.width = 0.057;
         size_m.height = 0.108;
      }
      return size_m;
   }

   float getDistanceMeters(Scenarios::Distance distance)
   {
      if(distance == Scenarios::Distance::FT2)
      {
         return 0.6096;
      }
      if(distance == Scenarios::Distance::FT4)
      {
         return 1.2192;
      }
      if(distance == Scenarios::Distance::FT6)
      {
         return 1.8288;
      }
      std::string d = std::to_string(distance);
      ROS_ERROR_STREAM("Didn't find lookup value!" << d);
      return 1;
   }

   /**
    * 640 x 480- IR and Depth Lidar resolution
    * 1920 x 1080- Camera pixel resolution
    */
   void setPixelCounts(ObjectExpectations &expectations, SensorSpecs::Sensor sensor)
   {
      //this is the distance in meters the range of the sensor is expected to cover at the expected distance in height
      float angleHeight = sensor.angularFieldOfViewHeight_deg/2.0;
      ROS_INFO_STREAM("Tan: " << (tan(angleHeight)));
      float metersCoveredHeight = std::abs(2.0 * (tan(angleHeight)) * expectations.distanceValue_m);
      ROS_INFO_STREAM("Meters covered height: " << metersCoveredHeight << ", angleHeight" << angleHeight << ", distance: " << expectations.distanceValue_m);
      //now we need to convert based on resolution to expected pixels it will cover rounded to the nearest int
      float depthRatio = 1.0/float(sensor.rowResolution_depth);
      float cameraRatio = 1.0/float(sensor.rowResolution_camera);
      ROS_INFO_STREAM("Ratios, depth: " << depthRatio << ", camera: " << cameraRatio);
      ROS_INFO_STREAM("Before multiplying by size, depth: " << (metersCoveredHeight * depthRatio));
      expectations.pixelCount_depth.row = expectations.size_m.height/(metersCoveredHeight * (1.0/float(sensor.rowResolution_depth)));
      expectations.pixelCount_camera.row = expectations.size_m.height/(metersCoveredHeight * (1.0/float(sensor.rowResolution_camera)));

      //this is the distance in meters the range of the sensor is expected to cover at the expected distance in width
      float angleWidth = sensor.angularFieldOfViewWidth_deg/2.0;
      ROS_INFO_STREAM("Tan2: " << (tan(angleWidth)));
      float metersCoveredWidth = std::abs(2.0 * (tan(angleWidth)) * expectations.distanceValue_m);
      ROS_INFO_STREAM("Meters covered width: " << metersCoveredWidth << ", angle width: " << angleWidth << ", distance: " << expectations.distanceValue_m);
      //now we need to convert based on resolution to expected pixels it will cover rounded to the nearest int
      expectations.pixelCount_depth.column = expectations.size_m.width/(metersCoveredWidth * (1.0/float(sensor.columnResolution_depth)));
      expectations.pixelCount_camera.column = expectations.size_m.width/(metersCoveredWidth * (1.0/float(sensor.columnResolution_camera)));

      ROS_INFO_STREAM("Depth- Expecting to cover pixel width: " << expectations.pixelCount_depth.column << ", height: " << expectations.pixelCount_depth.row);
      ROS_INFO_STREAM("Camera- Expecting to cover pixel width: " << expectations.pixelCount_camera.column << ", height: " << expectations.pixelCount_camera.row);
   }

   ObjectExpectations getSizeExpectation(Scenarios::Scenario scenario, SensorSpecs::Sensor sensor)
   {
      ROS_INFO("Getting size expectations");
      ObjectExpectations expectations;
      expectations.distanceValue_m = getDistanceMeters(scenario.distance);
      ROS_INFO_STREAM("Distance: " << expectations.distanceValue_m << " after lookup: " << scenario.distance);
      expectations.size_m = getSizeMeters(scenario.size);
      ROS_INFO_STREAM("Size: " << expectations.size_m.width << ", " << expectations.size_m.height);
      setPixelCounts(expectations, sensor);
      ROS_INFO("Set pixel counts");
      expectations.convolutionKernel_depth = cv::Mat(expectations.pixelCount_depth.row, expectations.pixelCount_depth.column, CV_32F, expectations.distanceValue_m);
      expectations.convolutionKernel_camera = cv::Mat(expectations.pixelCount_camera.row, expectations.pixelCount_camera.column, CV_32F, expectations.distanceValue_m);
      cv::Size depthKernelSize = expectations.convolutionKernel_depth.size();
      cv::Size cameraKernelSize = expectations.convolutionKernel_camera.size();
      ROS_INFO_STREAM("Depth kernel size: " << depthKernelSize.height << ", width: " << depthKernelSize.width);
      ROS_INFO_STREAM("Camera kernel size: " << cameraKernelSize.height << ", width: " << cameraKernelSize.width);
      return expectations;
   }   
}