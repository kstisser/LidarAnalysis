#include "pointcloud.h"
#include <pcl/segmentation/extract_clusters.h>

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
   PostProcessPointcloud::PostProcessPointcloud( ros::NodeHandle *handle, Scenarios::Scenario scenario ):
      _mHandle( handle ),
      _mImHandle( *handle )
   {
      this->_mScenario = scenario;

      this->_mPointNumColor_pub = handle->advertise<post_process_pointcloud::PointCountAndColor>(Topics::NumPointsAndColor, 1);

      //initialize all output image publishers
      this->_mLaplacianPub = _mImHandle.advertise("/Laplacian", 1);
      this->_mLapFilterResultPub = _mImHandle.advertise("/LapFilterResult", 1);
   }

   /**
    * @brief This function handles time synchronizing the three messages from camera, IR image, and point cloud
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

      // float distanceAway_m = float(depthImage_cv.at<uchar>(cv::Point(327,269)))/10.0;
      // ROS_INFO_STREAM("Point cloud:");
      // ROS_INFO_STREAM(std::to_string(distanceAway_m));
      // ROS_INFO_STREAM("IR image:");
      // ROS_INFO_STREAM(std::to_string(IRimage_cv.at<uchar>(cv::Point(327,269))));
      // ROS_INFO_STREAM("Camera:");
      // //ROS_INFO_STREAM(std::to_string(cameraImage_cv.at<Vec3b>(cv::Point(666,454))));

      //segment
      Supportive::ObjectExpectations expectations = Supportive::getSizeExpectation(this->_mScenario, SensorSpecs::getIntelLidar());
      //convolve each image to find where the expected object is
      ROS_INFO("Getting convolved depth image");
      //ROS_INFO_STREAM(expectations.convolutionKernel_depth);
      cv::Mat depthConvolvedImage;
      cv::filter2D(depthImage_cv, depthConvolvedImage, -1, expectations.convolutionKernel_depth, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      double minValue_depth, maxValue_depth;
      cv::Point minLocation_depth, maxLocation_depth;
      cv::minMaxLoc(depthConvolvedImage, &minValue_depth, &maxValue_depth, &minLocation_depth, &maxLocation_depth);
      ROS_INFO_STREAM("Max convolved depth: " << maxValue_depth);

      /*ROS_INFO("Getting convolved camera image");
      //ROS_INFO_STREAM(expectations.convolutionKernel_camera);
      cv::Mat cameraConvolvedImage;
      cv::filter2D(cameraImage_cv, cameraConvolvedImage, -1, expectations.convolutionKernel_camera, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
      double minValue_camera, maxValue_camera;
      cv::Point minLocation_camera, maxLocation_camera;
      cv::minMaxLoc(cameraConvolvedImage, &minValue_camera, &maxValue_camera, &minLocation_camera, &maxLocation_camera);
      ROS_INFO_STREAM("Max convolved camera: " << maxValue_camera);*/

      //publish images for debugging visualization
      sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage( std_msgs::Header(), depthImage->encoding, depthConvolvedImage ).toImageMsg();
      depthMsg->header.stamp = ros::Time::now(); 
      this->_mLaplacianPub.publish(depthMsg); 

      /*sensor_msgs::ImagePtr cameraMsg = cv_bridge::CvImage( std_msgs::Header(), depthImage->encoding, cameraConvolvedImage ).toImageMsg();
      cameraMsg->header.stamp = ros::Time::now(); 
      this->_mLapFilterResultPub.publish(cameraMsg);*/

      //find which segmentation aligns with the expected distance

      //publish segmented section for debugging

      //get number of points aligned with segmented section

      //get average pixel color of segmented section

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
   scenario.size = Scenarios::Size::MM80;
   scenario.temperature = Scenarios::ObjectTemperature::Ambient;
   scenario.color = Scenarios::ObjectColor::White;
   scenario.distance = Scenarios::Distance::FT4;
   /*std::string temp;
   int tempInt;
   handle.getParam("size", temp);
   ROS_INFO_STREAM("Got size: " << temp);
   tempInt = stoi(temp);
   scenario.size = static_cast<Scenarios::Size>(tempInt);
   handle.getParam("objectTemperature", temp);
   ROS_INFO_STREAM("Got object temperature: " << temp);
   tempInt = stoi(temp);
   scenario.temperature = static_cast<Scenarios::ObjectTemperature>(tempInt);
   handle.getParam("objectColor", temp);
   tempInt = stoi(temp);
   scenario.color = static_cast<Scenarios::ObjectColor>(tempInt);
   handle.getParam("distance", temp);
   tempInt = stoi(temp);
   scenario.distance = static_cast<Scenarios::Distance>(tempInt);*/
   ROS_INFO_STREAM("Size: " << scenario.size);
   ROS_INFO_STREAM("Temperature: " << scenario.temperature);
   ROS_INFO_STREAM("Color: " << scenario.color);
   ROS_INFO_STREAM("Distance input: " << scenario.distance);

   PC::PostProcessPointcloud pppNode(&handle, scenario);

   message_filters::Subscriber<sensor_msgs::Image> cameraImage_sub( handle, Topics::CameraImageTopic, 10 );
   message_filters::Subscriber<sensor_msgs::Image> IRimage_sub( handle, Topics::IRimageTopic, 10 );
   message_filters::Subscriber<sensor_msgs::Image> pointCloud_sub( handle, Topics::PointCloud, 10 );

   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> imageSyncPolicy;
   message_filters::Synchronizer<imageSyncPolicy> imageMessageSync( imageSyncPolicy( 10 ), cameraImage_sub, IRimage_sub, pointCloud_sub );
   imageMessageSync.registerCallback( boost::bind( &PC::PostProcessPointcloud::callback, &pppNode, _1, _2, _3) );

   ros::MultiThreadedSpinner spinner( 6 );
   spinner.spin();
   return 0;  //should never get here
}