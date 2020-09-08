//
// Created by vahagn on 08/09/20.
//

#ifndef INC_3D_TELEMETRY_SLAM_H
#define INC_3D_TELEMETRY_SLAM_H
#include <ifeature_extractor.h>
#include <memory>
#include <librealsense2/rs.hpp>
namespace gago {

class Slam {
 public:
  Slam(const std::shared_ptr<IFeatureExtractor> & feature_extractor);
  void Init();
 private:
  std::shared_ptr<IFeatureExtractor> feature_extractor_;
  rs2::pipeline pipeline_;
};

}

#endif //INC_3D_TELEMETRY_SLAM_H
