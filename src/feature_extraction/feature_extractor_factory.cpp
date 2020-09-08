//
// Created by vahagn on 08/09/20.
//

#include "feature_extractor_factory.h"
#include "sift_feature_extractor.h"
namespace gago {
std::shared_ptr<IFeatureExtractor> FeatureExtractorFactory::Create(FeatureType type) {
  switch (type) {
    case FeatureType::SIFT:
      return std::make_shared<SiftFeatureExtractor>();
    default:
      return nullptr;
  }

}
}
