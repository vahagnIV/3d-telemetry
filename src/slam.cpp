//
// Created by vahagn on 08/09/20.
//

#include "slam.h"
#include <opencv2/opencv.hpp>

namespace gago {
Slam::Slam(const std::shared_ptr<IFeatureExtractor> & feature_extractor) {

}

void Slam::Init() {

  pipeline_.start();
  while (true) {
    // Block program until frames arrive
    rs2::frameset frames = pipeline_.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color_frame =  frames.get_color_frame();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();
    cv::Mat depth_mat(height, width, CV_16UC1, (void *) depth.get_data());
    cv::Mat color_mat(height, width, CV_8UC3, (void *) color_frame.get_data());
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);

//    cv::normalize(depth_mat, depth_mat, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("depth", depth_mat*20);
    cv::imshow("color", color_mat);
    cv::waitKey(1);


    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);
    // Print the distance

  }
}

}
