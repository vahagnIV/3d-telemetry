#include <iostream>
#include "slam.h"
#include <feature_extractor_factory.h>
int main() {
  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<gago::IFeatureExtractor>
      feature_extractor = gago::FeatureExtractorFactory::Create(gago::FeatureType::SIFT);
  gago::Slam slam(feature_extractor);
  slam.Init();


  return 0;
}