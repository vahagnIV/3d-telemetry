//
// Created by vahagn on 08/09/20.
//

#ifndef INC_3D_TELEMETRY_SIFT_FEATURE_EXTRACTOR_H
#define INC_3D_TELEMETRY_SIFT_FEATURE_EXTRACTOR_H
#include "open_cv_feature_extractor.h"
namespace gago {
class SiftFeatureExtractor : public OpenCVFeatureExtractor {
 public:
  SiftFeatureExtractor(size_t max_count = std::numeric_limits<size_t>::max());

};
}

#endif //INC_3D_TELEMETRY_SIFT_FEATURE_EXTRACTOR_H
