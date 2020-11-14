#ifndef POSTPROCESSPOINTCLOUD_H
#define POSTPROCESSPOINTCLOUD_H

#include "defs.h"

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "post_process_pointcloud/PointCountAndColor.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace PC
{
   class PostProcessPointcloud
   {
      public:
         PostProcessPointcloud( ros::NodeHandle *handle );
         ~PostProcessPointcloud() = default;

         void callback( const sensor_msgs::ImageConstPtr &cameraImage,
                        const sensor_msgs::ImageConstPtr &IRimage,
                        const sensor_msgs::ImageConstPtr &pointCloud );

      private:
         ros::NodeHandle *_mHandle;
         ros::Publisher _mPointNumColor_pub;

         std::string _mCameraTopic;
         std::string _mIRimageTopic;
         std::string _mPointCloudTopic;

         //void convertImgToPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, sensor_msgs::ImageConstPtr depth);

      protected:

   };
}
#endif //POSTPROCESSPOINTCLOUD_H