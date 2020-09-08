//
// Created by vahagn on 08/09/20.
//

#ifndef INC_3D_TELEMETRY_FEATURE_EXTRACTOR_FACTORY_H
#define INC_3D_TELEMETRY_FEATURE_EXTRACTOR_FACTORY_H
#include <ifeature_extractor.h>
#include <feature_type.h>
#include <memory>
namespace gago {
class FeatureExtractorFactory {
 public:
  static std::shared_ptr<IFeatureExtractor> Create(FeatureType type);

};
}

#endif //INC_3D_TELEMETRY_FEATURE_EXTRACTOR_FACTORY_H
