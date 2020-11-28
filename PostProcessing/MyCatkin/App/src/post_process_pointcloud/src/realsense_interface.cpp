#include "realsense_interface.h"
#include <ros/console.h>
#include <iostream>


namespace PC
{
   RealsenseInterface::RealsenseInterface(){}

   //note- used librealsense rs-distance example as reference for this
   void RealsenseInterface::getDepthDistances(std::string filename, cv::Point upperLeft, cv::Point lowerRight, std::vector<std::vector<float>> &framesOfDistancePoints, std::vector<int> &numberOfPoints, float expectedDistance_m, float tolerance)
   {
      rs2_error* e = 0;

      // Create a context object. This object owns the handles to all connected realsense devices.
      // The returned object should be released with rs2_delete_context(...)
      rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
      check_error(e);

      // Create a config instance, used to specify hardware configuration
      // The retunred object should be released with rs2_delete_config(...)
      rs2_config* config = rs2_create_config(&e);
      check_error(e);

      // Create a pipeline to configure, start and stop camera streaming
      // The returned object should be released with rs2_delete_pipeline(...)
      rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
      check_error(e);


      // Request a specific configuration
      rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
      check_error(e);

      //Added this to look up from file- rs-distance had other code that was removed for reading from the device itself
      rs2_config_enable_device_from_file(config, filename.c_str(), &e);
      check_error(e);
      //make it so when finishing it does not repeat
      rs2_config_enable_device_from_file_repeat_option(config, filename.c_str(), false, &e);

      // Start the pipeline streaming
      // The retunred object should be released with rs2_delete_pipeline_profile(...)
      rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
      if (e)
      {
         printf("The connected device doesn't support depth streaming!\n");
         exit(EXIT_FAILURE);
      }

      try
      {
         bool readWholeFile = false;
         int frameCount = 0;
         //get depth from stream, return a vector of vectors of depth data. Each internal vector is a frame's worth of data
         while (!readWholeFile)
         {
            // This call waits until a new composite_frame is available
            // composite_frame holds a set of frames. It is used to prevent frame drops
            // The returned object should be released with rs2_release_frame(...)
            rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, 15000, &e);
            if(e)
            {
               readWholeFile = true;
               break;
            }
            check_error(e);
            frameCount++;

            // Returns the number of frames embedded within the composite frame
            int num_of_frames = rs2_embedded_frames_count(frames, &e);
            check_error(e);

            int i;
            for (i = 0; i < num_of_frames; ++i)
            {
               //reducing to 10 hz, as lidar runs at 30hz and other light/temp/humidity sensors run at 10 hz
               if( (i % 3) == 0)
               {
                  std::vector<float> frameVec;
                  int framePointCount = 0;

                  // The returned object should be released with rs2_release_frame(...)
                  rs2_frame* frame = rs2_extract_frame(frames, i, &e);
                  check_error(e);

                  // Check if the given frame can be extended to depth frame interface
                  // Accept only depth frames and skip other frames
                  if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
                     continue;

                  // Get the depth frame's dimensions
                  int width = rs2_get_frame_width(frame, &e);
                  check_error(e);
                  int height = rs2_get_frame_height(frame, &e);
                  check_error(e);

                  for(int row = upperLeft.x; row < lowerRight.x; row++)
                  {
                     for(int column = upperLeft.y; column < lowerRight.y; column++)
                     {
                        // Query the distance from the camera to the object in the center of the image
                        float dist_to_point = rs2_depth_frame_get_distance(frame, row, column, &e);
                        //check_error(e);

                        frameVec.push_back(dist_to_point);

                        //add to frame's point count
                        if( (dist_to_point > (expectedDistance_m - tolerance)) && (dist_to_point < (expectedDistance_m + tolerance)))
                        {
                           framePointCount++;
                           //ROS_INFO_STREAM("Accepted distance: " << dist_to_point << " meters away at expected: " << expectedDistance_m);
                        }
                        else
                        {
                           //ROS_INFO_STREAM("Rejected distance: " << dist_to_point << " meters away at expected: " << expectedDistance_m);
                        }
                     }
                  }
                  ROS_INFO_STREAM("Frame point count: " << framePointCount << " with frame count: " << frameCount);
                  rs2_release_frame(frame);
                  framesOfDistancePoints.push_back(frameVec);
                  numberOfPoints.push_back(framePointCount);
               }
            }
            rs2_release_frame(frames);
         }
      }
      catch(const std::exception& e)
      {
         std::cerr << e.what() << '\n';
         ROS_INFO("Probably finished reading file, no worries");
      } 

      std::cout << "Exiting interface function";

      // Stop the pipeline streaming
      rs2_pipeline_stop(pipeline, &e);
      check_error(e);

      // Release resources
      rs2_delete_pipeline_profile(pipeline_profile);
      rs2_delete_config(config);
      rs2_delete_pipeline(pipeline);
      rs2_delete_context(ctx);      
   }

   /* Function calls to librealsense may raise errors of type rs_error*/
   void RealsenseInterface::check_error(rs2_error* e)
   {
      if (e)
      {
         printf("rs_error was raised when calling %s(%s):\n", rs2_get_failed_function(e), rs2_get_failed_args(e));
         printf("    %s\n", rs2_get_error_message(e));
         //exit(EXIT_FAILURE);
      }
   }   
}