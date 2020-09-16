#include <iostream>
#include "extended_kalman_filter.h"
#include <feature_extractor_factory.h>
#include <velocity_2d_motion_model.h>

void ModifyBlock(Eigen::Matrix<double, 3, 3> & block){
  // A stupid example
  block += Eigen::Matrix<double, 3, 3>::Identity();
}

int main() {

  const int M = 10;
  const int N = 20;

  Eigen::Matrix<double, M, N> matrix;
  // Initialize matrix
  Eigen::Matrix<double, 3, 3> & b = matrix(std::vector<int>{1,3,7}, std::vector<int>{0,2,4});
  //ModifyBlock(matrix(std::vector<int>{1,3,7}, std::vector<int>{0,2,4}));
  return 0;


  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<gago::IFeatureExtractor>
      feature_extractor = gago::FeatureExtractorFactory::Create(gago::FeatureType::SIFT);
  gago::ExtendedKalmanFilter<double, 3> ekf;

  std::shared_ptr<gago::Velocity2DMotionModel<double>>
      motion_model = std::make_shared<gago::Velocity2DMotionModel<double>>(100, 10);
  ekf.ComputeEvolution(motion_model, 152354878.00012);

  return 0;
}