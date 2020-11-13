#ifndef POSTPROCESSPOINTCLOUD_H
#define POSTPROCESSPOINTCLOUD_H

#include <ros/ros.h>
#include <ros/console.h>

class PostProcessPointcloud
{
   public:
      PostProcessPointcloud( ros::NodeHandle *handle );
      ~PostProcessPointcloud() = default;

   private:
      ros::NodeHandle *_mHandle;

   protected:

};
#endif //POSTPROCESSPOINTCLOUD_H