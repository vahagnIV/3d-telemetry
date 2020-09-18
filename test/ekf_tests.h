//
// Created by vahagn on 18/09/20.
//

#ifndef INC_3D_TELEMETRY_EKF_TESTS_H
#define INC_3D_TELEMETRY_EKF_TESTS_H
#include "gtest/gtest.h"
#include "mock/sensor_mock.h"
#include <extended_kalman_filter.h>

namespace gago {
namespace test {

class EKFTests : public ::testing::Test {
 public:
  typedef void * TMotionModelMeasurement;
  typedef void * TSensorModelMeasurement;
  typedef Eigen::Matrix<float, Eigen::Dynamic, 5> TLandmark;
  typedef  mock::MockSensorModel<TSensorModelMeasurement, TLandmark> TSensorModel;
  EKFTests(): mock_model_(std::make_shared<TSensorModel>()), filter_(nullptr, mock_model_){}
  std::shared_ptr<TSensorModel> mock_model_;
  ExtendedKalmanFilter<TMotionModelMeasurement, TSensorModelMeasurement, TLandmark> filter_;


};

}
}

#endif //INC_3D_TELEMETRY_EKF_TESTS_H
