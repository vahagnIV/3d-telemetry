#include <iostream>
#include "extended_kalman_filter.h"
#include <feature_extractor_factory.h>
int main() {
  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<gago::IFeatureExtractor>
      feature_extractor = gago::FeatureExtractorFactory::Create(gago::FeatureType::SIFT);
  gago::ExtendedKalmanFilter<float, 3> ekf;
  ekf.ComputeEvolution(nullptr);



  return 0;
}