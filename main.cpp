#include <iostream>
#include "extended_kalman_filter.h"

#include <velocity_2d_motion_model.h>
#include <rgbd_sensor_model.h>
#include "src/feature_extraction/sift_feature_extractor.h"
#include "src/feature_extraction/surf_feature_extractor.h"

template<class Derived>
void ModifyBlock(Eigen::MatrixBase<Derived> && block) {
  // A stupid example
  block += Eigen::Matrix<double, 3, 3>::Identity();
}

int main() {

  std::cout << "Hello, World!" << std::endl;
  std::shared_ptr<gago::IFeatureExtractor<float, float>>
      extractor = std::make_shared<gago::SiftFeatureExtractor>(16);

  cv::Mat image = cv::imread("/home/vahagn/Pictures/anna_hakobyan.jpg");
  Eigen::Matrix<float , Eigen::Dynamic, Eigen::Dynamic> descriptors;
  Eigen::Matrix<float, Eigen::Dynamic, 2> keypoints;
  extractor->Extract(image.data, image.cols, image.rows, descriptors, keypoints);

  std::cout << keypoints << std::endl;

  std::shared_ptr<gago::IMotionModel<gago::Velocity2dMeasurement>>
      motion_model = std::make_shared<gago::Velocity2DMotionModel<>>();

  std::shared_ptr<gago::ISensorModel<gago::RGBDData, gago::RgbdLandmarks>>
      sensor_model = std::make_shared<gago::RGBDSensorModel<>>(extractor);

  gago::ExtendedKalmanFilter<gago::Velocity2dMeasurement, gago::RGBDData, gago::RgbdLandmarks> ekf(motion_model, sensor_model);

  //ekf.ComputeEvolution(motion_model, 152354878.00012);

  return 0;
}