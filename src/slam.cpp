//
// Created by vahagn on 08/09/20.
//

#include "slam.h"

namespace gago {
Slam::Slam(const std::shared_ptr<IFeatureExtractor> & feature_extractor) {

}

void Slam::Init() {
  pipeline_.start();
  while (true)
  {
    // Block program until frames arrive
    rs2::frameset frames = pipeline_.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();

    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);
    // Print the distance

  }
}

}
