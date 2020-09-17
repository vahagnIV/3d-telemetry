//
// Created by vahagn on 17/09/20.
//

#ifndef INC_3D_TELEMETRY_SURF_FEATURE_EXTRACTOR_H
#define INC_3D_TELEMETRY_SURF_FEATURE_EXTRACTOR_H
#include "open_cv_feature_extractor.h"

namespace gago {

class SurfFeatureExtractor : public OpenCVFeatureExtractor {
 public:
  SurfFeatureExtractor(size_t max_count);
};

}

#endif //INC_3D_TELEMETRY_SURF_FEATURE_EXTRACTOR_H
