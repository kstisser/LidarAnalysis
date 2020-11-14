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
      this->_mPointNumColor_pub = handle->advertise<post_process_pointcloud::PointCountAndColor>(Topics::NumPointsAndColor, 1);
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
      post_process_pointcloud::PointCountAndColor pCountColorMsg;
      pCountColorMsg.header = pointCloud->header;

      pcl::PointCloud<pcl::PointXYZRGB> cloud;

      //convert to opencv image
      cv::Mat cameraImage_cv = ( cv_bridge::toCvCopy( cameraImage, cameraImage->encoding ) )->image;
      cv::Mat IRimage_cv = ( cv_bridge::toCvCopy( IRimage, IRimage->encoding ) )->image;
      cv::Mat pointCloud_cv = ( cv_bridge::toCvCopy( pointCloud, pointCloud->encoding ) )->image;

      //segment


      //find which segmentation aligns with the expected distance

      //publish segmented section for debugging

      //get number of points aligned with segmented section

      //get average pixel color of segmented section

      //publish number of points and average pixel color
      this->_mPointNumColor_pub.publish(pCountColorMsg);
   }

/*
   //You can get your disparity map using StereoSGBM, then your depth map using reprojectImageTo3D. Once you have your depth map, you can reconstruct your 3D scene using something like this
   //ref: https://answers.ros.org/question/237565/how-to-convert-sensor_msgsimage-to-pointcloudpclrgb/
   void PostProcessPointcloud::convertImgToPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, sensor_msgs::ImageConstPtr depth)
   {
      //Reconstruct PointCloud with the depthmap points
      for (int i = 0; i < depth->rows; ++i)
      {
         for (int j = 0; j < depth->cols; ++j)
         {
            pcl::PointXYZRGB p;

            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = depth->at<Vec3f>(i,j)[0];
            p.y = -(depth->at<Vec3f>(i,j)[1]);
            p.z = -(depth->at<Vec3f>(i,j)[2]);

            //Coloring the point with the corrispondent point in the rectified image
            p.r = static_cast<uint8_t>(color.at<Vec3b>(i,j)[2]);
            p.g = static_cast<uint8_t>(color.at<Vec3b>(i,j)[1]);
            p.b = static_cast<uint8_t>(color.at<Vec3b>(i,j)[0]);

            //Insert point in the cloud, cutting the points that are too distant
            if(( abs( p.x ) < 500 )&&( abs( p.y ) < 200 )&&( abs( p.z ) < 500 ))
               cloud.points.push_back(p);
         }
      }
      cloud.width = (int) cloud->points.size();
      cloud.height = 1;
   }*/
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