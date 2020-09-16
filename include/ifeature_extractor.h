//
// Created by vahagn on 08/09/20.
//

#ifndef INC_3D_TELEMETRY_IFEATURE_EXTRACTOR_H
#define INC_3D_TELEMETRY_IFEATURE_EXTRACTOR_H
#include <vector>
#include <Eigen/Dense>

namespace gago {

template<typename TLandmarkDescriptor, typename TCoord>
class IFeatureExtractor {
 public:
  virtual void Extract(uint8_t *rgb,
                       int w,
                       int h,
                       Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_descriptors,
                       Eigen::Matrix<TCoord, Eigen::Dynamic, 2> & out_keypoints) = 0;
};

}

#endif //INC_3D_TELEMETRY_IFEATURE_EXTRACTOR_H
