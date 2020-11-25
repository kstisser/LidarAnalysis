#ifndef REALSENSE_INTERFACE_H
#define REALSENSE_INTERFACE_H

#include <memory>
#include <vector> 
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
/* Include the librealsense C header files */
#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/h/rs_config.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are for L515 Lidar                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           640               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          0                 // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace PC
{
   class RealsenseInterface
   {
      public:
         RealsenseInterface();
         ~RealsenseInterface() = default;

         //returns all distances to the rectangle & number of points through reference
         static void getDepthDistances(std::string filename, cv::Point upperLeft, cv::Point lowerRight, std::vector<std::vector<float>> &framesOfDistancePoints, std::vector<int> &numberOfPoints, float expectedDistance_m, float tolerance);

      private:
         rs2_error* e = 0;
         rs2_context* ctx;
         static void check_error(rs2_error* e);
         //std::shared_ptr<rs2_context> _mCtx_sptr;

      protected:

   };
}

#endif //REALSENSE_INTERFACE_H