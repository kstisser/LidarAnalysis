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
   PostProcessPointcloud::PostProcessPointcloud( ros::NodeHandle *handle ):
      _mHandle( handle )
   {
   }

   /**
    * @brief This function handles time synchronizing the three messages from camera, IR image, and point cloud
    * 
    * @param cameraImage- Image from the Intel lidar camera
    * @param IRimage- Image from the IR part of the Intel lidar camera
    * @param pointCloud- Point Cloud from the Intel lidar
    */
   void PostProcessPointcloud::callback( const sensor_msgs::ImageConstPtr &cameraImage, 
                                         const sensor_msgs::ImageConstPtr &IRimage,
                                         const sensor_msgs::ImageConstPtr &pointCloud )
   {

   }
}

int main( int argc, char * *argv )
{
   ROS_INFO( "Initializing post processing pointcloud" );
   ros::init( argc, argv, "PostProcessingPointcloud", ros::init_options::NoRosout );
   ros::NodeHandle handle;
   PC::PostProcessPointcloud pppNode(&handle);

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