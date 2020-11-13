#ifndef POSTPROCESSPOINTCLOUD_H
#define POSTPROCESSPOINTCLOUD_H

#include "defs.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>

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

         std::string _mCameraTopic;
         std::string _mIRimageTopic;
         std::string _mPointCloudTopic;

      protected:

   };
}
#endif //POSTPROCESSPOINTCLOUD_H