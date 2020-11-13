#include "pointcloud.h"

PostProcessPointcloud::PostProcessPointcloud( ros::NodeHandle *handle ):
   _mHandle( handle )
{}

int main( int argc, char * *argv )
{
   ROS_INFO( "Initializing post processing pointcloud" );
   ros::init( argc, argv, "PostProcessingPointcloud", ros::init_options::NoRosout );
   ros::NodeHandle handle;

   ros::MultiThreadedSpinner spinner( 6 );
   spinner.spin();
   return 0;  //should never get here
}