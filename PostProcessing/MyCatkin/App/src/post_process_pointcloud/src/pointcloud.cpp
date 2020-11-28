#include "pointcloud.h"
#include <pcl/segmentation/extract_clusters.h>
#include <librealsense2/rs.hpp>
#include "realsense_interface.h"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

namespace PC
{
   /**
    * @class PostProcessPointCloud
    * @brief This class handles time synchronizing incoming point cloud and image data, 
    *        it segments all data not tied to the expected object, and outputs a number of
    *        points and average pixel color
    * 
    * @param handle This is a ROS node handle which allows message communication
    */
   PostProcessPointcloud::PostProcessPointcloud( ros::NodeHandle *handle, std::string filename, Scenarios::Scenario scenario, Supportive::PixelPoints lidarPoints, Supportive::PixelPoints cameraPoints ):
      _mHandle( handle ),
      _mImHandle( *handle )
   {
      this->_mScenario = scenario;
      this->_mLidarPoints = lidarPoints;
      this->_mCameraPoints = cameraPoints;

      this->_mPointNumColor_pub = handle->advertise<post_process_pointcloud::PointCountAndColor>(Topics::NumPointsAndColor, 1);

      //initialize all output image publishers
      this->_mLaplacianPub = _mImHandle.advertise("/Laplacian", 1);
      this->_mLapFilterResultPub = _mImHandle.advertise("/LapFilterResult", 1);

      Supportive::ObjectExpectations expectations = Supportive::getSizeExpectation(this->_mScenario, SensorSpecs::getIntelLidar());
      std::vector<std::vector<float>> framesOfDistancePoints;
      std::vector<int> numberOfPoints;
      float tolerance = 0.15; //6 inches
      RealsenseInterface::getDepthDistances(filename, lidarPoints.upperLeft, lidarPoints.lowerRight, framesOfDistancePoints, numberOfPoints, expectations.distanceValue_m, tolerance);

      //write out data to a file
      std::string jsonPath = filename.substr(0, filename.find_last_of("\\/")) + "/lidarPoints.json";

      nlohmann::json jsonData;
      nlohmann::json jArrayFramePoints;
      for(int n : numberOfPoints)
      {
         jArrayFramePoints.push_back(n);
      }
      cv::Scalar color = Supportive::getColor(Scenarios::ObjectColor(scenario.color));
      jsonData["objectColor"]["b"] = color[0];
      jsonData["objectColor"]["g"] = color[1];
      jsonData["objectColor"]["r"] = color[2];
      jsonData["actualDistance"] = Supportive::getDistanceMeters(Scenarios::Distance(scenario.distance));
      Supportive::ObjectSize osize = Supportive::getSizeMeters(Scenarios::Size(scenario.size));
      jsonData["objectSize"]["height"] = osize.height;
      jsonData["objectSize"]["width"] = osize.width;
      jsonData["objectTemperature"] = Supportive::getTemperatureCelcius(Scenarios::ObjectTemperature(scenario.temperature));
      jsonData["pointNumberByFrame"] = jArrayFramePoints;
      //jsonData["framePoints"] = framesOfDistancePoints;

      std::ofstream file("lidarPoints.json");
      file << jsonData;
      ROS_INFO_STREAM("Writing data to JSON at lidarPoints.json");
   }

   /**
    * @brief This function was originally made to handle time synchronizing the three messages from camera, IR image, and point cloud when 
    *        a recorded bag file is played. Then, the image would be convolved with a kernel based on the size, shape, and location of the 
    *        expected object, expecting a spike where it exists in the image. This would be used for segmenting that from the image, then 
    *        we can see how many points contribute to the object and publish it. This function is not currently used, as I couldn't get the
    *        depth value from the image once it was received. librealsense serialization was needed. It is left for future development.
    * 
    * @param cameraImage- Image from the Intel lidar camera
    * @param IRimage- Image from the IR part of the Intel lidar camera
    * @param depthImage- Point Cloud from the Intel lidar
    */
   void PostProcessPointcloud::callback( const sensor_msgs::ImageConstPtr &cameraImage, 
                                         const sensor_msgs::ImageConstPtr &IRimage,
                                         const sensor_msgs::ImageConstPtr &depthImage )
   {
      ROS_INFO("Entering callback");
      post_process_pointcloud::PointCountAndColor pCountColorMsg;
      pCountColorMsg.header = depthImage->header;

      ROS_INFO("Converting to opencv images");

      //convert to opencv image
      cv::Mat cameraImage_cv = ( cv_bridge::toCvCopy( cameraImage, cameraImage->encoding ) )->image;
      cv::Mat IRimage_cv = ( cv_bridge::toCvCopy( IRimage, IRimage->encoding ) )->image;
      cv::Mat depthImage_cv = ( cv_bridge::toCvCopy( depthImage, depthImage->encoding ) )->image;

      /*cv::Vec3b color = cameraImage_cv.at<cv::Vec3b>(712,581);
      ROS_INFO_STREAM("Rose gold color: (BGR) " << color[0] << ", " << color[1] << ", " << color[2]);
      color = cameraImage_cv.at<cv::Vec3b>(638,576);
      ROS_INFO_STREAM("Blue color: (BGR) " << color[0] << ", " << color[1] << ", " << color[2]);
      color = cameraImage_cv.at<cv::Vec3b>(795,592);
      ROS_INFO_STREAM("Silver color: (BGR) " << color[0] << ", " << color[1] << ", " << color[2]);
      */

      //segment
      Supportive::ObjectExpectations expectations = Supportive::getSizeExpectation(this->_mScenario, SensorSpecs::getIntelLidar());

      /* 
      //find largest value in depth image
      double minVal,maxVal; 
      cv::Point minLoc, maxLoc; 
      minMaxLoc( depthImage_cv, &minVal, &maxVal, &minLoc, &maxLoc );

      //make left side and top of kernel show the largest value to better correlate when convolved
      int kernelHeight = expectations.convolutionKernel_depth.size().height;
      cv::Mat addedColumn(kernelHeight, 1, CV_32F, cv::Scalar::all(maxVal));
      int kernelWidth = expectations.convolutionKernel_depth.size().width;
      cv::Mat addedRow(1, kernelWidth, CV_32F, cv::Scalar::all(maxVal));
      addedColumn.copyTo(expectations.convolutionKernel_depth.colRange(1,1).rowRange(1, expectations.convolutionKernel_depth.rows - 1));
      addedRow.copyTo(expectations.convolutionKernel_depth.colRange(1,expectations.convolutionKernel_depth.cols - 1).rowRange(1,1));

      //convolve each image to find where the expected object is
      ROS_INFO("Getting convolved depth image");
      //ROS_INFO_STREAM(expectations.convolutionKernel_depth);
      cv::Mat depthConvolvedImage;
      cv::filter2D(depthImage_cv, depthConvolvedImage, -1, expectations.convolutionKernel_depth, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      double minValue_depth, maxValue_depth;
      cv::Point minLocation_depth, maxLocation_depth;
      cv::minMaxLoc(depthConvolvedImage, &minValue_depth, &maxValue_depth, &minLocation_depth, &maxLocation_depth);
      ROS_INFO_STREAM("Max convolved depth: " << maxValue_depth);
      */

      /*ROS_INFO("Getting convolved camera image");
      //ROS_INFO_STREAM(expectations.convolutionKernel_camera);
      cv::Mat cameraConvolvedImage;
      cv::filter2D(cameraImage_cv, cameraConvolvedImage, -1, expectations.convolutionKernel_camera, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      double minValue_camera, maxValue_camera;
      cv::Point minLocation_camera, maxLocation_camera;
      cv::minMaxLoc(cameraConvolvedImage, &minValue_camera, &maxValue_camera, &minLocation_camera, &maxLocation_camera);
      ROS_INFO_STREAM("Max convolved camera: " << maxValue_camera);*/
      
      //publish images for debugging visualization
      sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage( std_msgs::Header(), depthImage->encoding, depthImage_cv ).toImageMsg();
      depthMsg->header.stamp = ros::Time::now(); 
      this->_mLaplacianPub.publish(depthMsg); 
      
      cv::Mat squareDrawnImg = depthImage_cv.clone();  
      cv::rectangle(squareDrawnImg, this->_mLidarPoints.upperLeft, this->_mLidarPoints.lowerRight, cv::Scalar(50, 200, 50));
      sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage( std_msgs::Header(), depthImage->encoding, squareDrawnImg ).toImageMsg();
      imgMsg->header.stamp = ros::Time::now(); 
      this->_mLapFilterResultPub.publish(imgMsg);

      cv::Mat subImgSquare = depthImage_cv(cv::Range(this->_mLidarPoints.upperLeft.x, this->_mLidarPoints.lowerRight.x), cv::Range(this->_mLidarPoints.upperLeft.y, this->_mLidarPoints.lowerRight.y));

      //find which segmentation aligns with the expected distance

      //publish segmented section for debugging




      //get number of points aligned with segmented section
      expectations.minDistanceBuffer = expectations.distanceValue_m - BUFFERREGION_m;
      expectations.maxDistanceBuffer = expectations.distanceValue_m + BUFFERREGION_m;

      //ROS_WARN_STREAM(depthImage_cv);
      int pixelsInDistanceRegion;
      for(int row = this->_mLidarPoints.upperLeft.x; row <= this->_mLidarPoints.lowerRight.x; row++)
      {
         for(int column = this->_mLidarPoints.upperLeft.y; column <= this->_mLidarPoints.lowerRight.y; column++)
         {
            //trying to see what format will show the correct value for mono16- none of these seem to, will come back in future
            cv::Scalar depth = depthImage_cv.at<uchar>(row,column);
            float d1 = depthImage_cv.at<uchar>(row,column);
            int16_t d2 = depthImage_cv.at<uchar>(row,column);
            uint16_t d3 = depthImage_cv.at<uchar>(row,column);
            float d4 = depthImage_cv.at<char>(row,column);
            int16_t d5 = depthImage_cv.at<char>(row,column);
            uint16_t d6 = depthImage_cv.at<char>(row,column);
            float d7 = depthImage_cv.at<float>(row,column);
            int16_t d8 = depthImage_cv.at<float>(row,column);
            uint16_t d9 = depthImage_cv.at<float>(row,column);
            float d10 = depthImage_cv.at<ushort>(row,column);
            int16_t d11 = depthImage_cv.at<ushort>(row,column);
            uint16_t d12 = depthImage_cv.at<ushort>(row,column);
            
            ROS_INFO_STREAM(depth);
            ROS_INFO_STREAM(d1 << ", " << d2 << ", " << d3 << ", " << d4 << ", " << d5 << ", " << d6 << ", " << d7 << ", " << d8 << ", " << d9);
            ROS_INFO_STREAM(d10 << ", " << d11 << ", " << d12);
            //ROS_INFO_STREAM("IR char value: " << IRdata);
            //ROS_INFO_STREAM("Min buffer: " << expectations.minDistanceBuffer << ", Max buffer: " << expectations.maxDistanceBuffer);
            /*if((depth > expectations.minDistanceBuffer) && (depth < expectations.maxDistanceBuffer))
            {
               pixelsInDistanceRegion++;
            }*/
         }
      }
      pCountColorMsg.numberOfPixels = pixelsInDistanceRegion;
      ROS_WARN_STREAM("Found " << pixelsInDistanceRegion << " pixels in the region!");

      //get average pixel color of segmented section
      double red = 0;
      double green = 0;
      double blue = 0;
      int numPixelsUsed = 0;
      for(int row = this->_mCameraPoints.upperLeft.x; row <= this->_mCameraPoints.lowerRight.x; row++)
      {
         for(int column = this->_mCameraPoints.upperLeft.y; column <= this->_mCameraPoints.lowerRight.y; column++)
         {
            cv::Vec3b color = cameraImage_cv.at<cv::Vec3b>(row,column);
            if((red + green + blue) > 20)
            {
               blue += color[0];
               green += color[1];
               red += color[2];
               numPixelsUsed++;
            }
            else
            {
               //ROS_INFO_STREAM("Black!");
            }
         }
      }
      //ROS_INFO_STREAM("Color totals: " << red << ", " << blue << ", " << green << ". Pixel total: " << numPixelsUsed);
      blue = (numPixelsUsed == 0) ? 0 : int(blue/numPixelsUsed);
      green = (numPixelsUsed == 0) ? 0 : int(green/numPixelsUsed);
      red = (numPixelsUsed == 0) ? 0 : int(red/numPixelsUsed);
      pCountColorMsg.averagePixelColor[0] = red;
      pCountColorMsg.averagePixelColor[1] = green;
      pCountColorMsg.averagePixelColor[2] = blue;

      //ROS_WARN_STREAM("Average RGB: " << red << ", " << green << ", " << blue);

      //publish number of points and average pixel color
      this->_mPointNumColor_pub.publish(pCountColorMsg);
   }
}


int main( int argc, char * *argv )
{
   ROS_INFO( "Initializing post processing pointcloud" );
   ros::init( argc, argv, "PostProcessingPointcloud", ros::init_options::NoRosout );
   ros::NodeHandle handle;
   
   Scenarios::Scenario scenario;
   scenario.size = Scenarios::Size::HANDWARMER;
   scenario.temperature = Scenarios::ObjectTemperature::Ambient;
   scenario.color = Scenarios::ObjectColor::Blue;
   scenario.distance = Scenarios::Distance::FT6;

   ROS_INFO_STREAM("Size: " << scenario.size);
   ROS_INFO_STREAM("Temperature: " << scenario.temperature);
   ROS_INFO_STREAM("Color: " << scenario.color);
   ROS_INFO_STREAM("Distance input: " << scenario.distance);

   Supportive::PixelPoints depthPoints;
   depthPoints.upperLeft = cv::Point(385,249);
   depthPoints.lowerRight = cv::Point(405,291);

   Supportive::PixelPoints cameraPoints;
   cameraPoints.upperLeft = cv::Point(647,420);
   cameraPoints.lowerRight = cv::Point(690,481);

   std::string filename = "lidar.bag";

   PC::PostProcessPointcloud pppNode(&handle, filename, scenario, depthPoints, cameraPoints);

   //message_filters::Subscriber<sensor_msgs::Image> cameraImage_sub( handle, Topics::CameraImageTopic, 10 );
   //message_filters::Subscriber<sensor_msgs::Image> IRimage_sub( handle, Topics::IRimageTopic, 10 );
   //message_filters::Subscriber<sensor_msgs::Image> pointCloud_sub( handle, Topics::PointCloud, 10 );

   //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> imageSyncPolicy;
   //message_filters::Synchronizer<imageSyncPolicy> imageMessageSync( imageSyncPolicy( 10 ), cameraImage_sub, IRimage_sub, pointCloud_sub );
   //imageMessageSync.registerCallback( boost::bind( &PC::PostProcessPointcloud::callback, &pppNode, _1, _2, _3) );

   //ros::MultiThreadedSpinner spinner( 6 );
   //spinner.spin();
   return 0;  //should never get here
}