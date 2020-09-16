#include <iostream>
#include "extended_kalman_filter.h"
#include <feature_extractor_factory.h>
#include <velocity_2d_motion_model.h>

template<class Derived>
void ModifyBlock(Eigen::MatrixBase<Derived> && block){
  // A stupid example
  block += Eigen::Matrix<double, 3, 3>::Identity();
}

int main() {


  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<gago::IFeatureExtractor>
      feature_extractor = gago::FeatureExtractorFactory::Create(gago::FeatureType::SIFT);
  gago::ExtendedKalmanFilter<float, 3> ekf;

  std::shared_ptr<gago::Velocity2DMotionModel<float>>
      motion_model = std::make_shared<gago::Velocity2DMotionModel<float>>(100, 10);
  ekf.ComputeEvolution(motion_model, 152354878.00012);

  return 0;
}