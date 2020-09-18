//
// Created by vahagn on 18/09/20.
//

#include "ekf_tests.h"

using ::testing::Return;
using ::testing::SetArgReferee;
using ::testing::DoAll;

namespace gago {
namespace test {
/*
 *               0 const TSensorMeasurement & sensor_measurement,
                 1 double sensor_measurement_time,
                 2 double last_measurement_time,
                 3 TLandmarkDescriptor & current_landmark_descriptors,
                 4 CoordType & current_coordinates,
                 5 SliceType & out_matched_landmark_indices,
                 6 SQuareType & out_Q,
                 7 ZHType & out_z,
                 8 ZHType & out_h,
                 9 SQuareType & out_jacobian,
                 10 CoordType & new_coordinates
 * */
TEST_F(EKFTests, KalmanCorrectlyProcessesSensorUpdate) {
  const int landmark_dim = 5;
  const int first_landmark_count = 100;

  Eigen::Matrix<float, Eigen::Dynamic, landmark_dim>
      landmark_descriptors = Eigen::Matrix<float, first_landmark_count, landmark_dim>::Random();

  Eigen::Matrix<float, Eigen::Dynamic, 3> new_coordinates = Eigen::Matrix<float, first_landmark_count, 3>::Random();

  Eigen::Array<int, Eigen::Dynamic, 1> matched_indices;

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Q = Eigen::Matrix<float, 6, 6>::Zero(),
      H = Eigen::Matrix<float, 6, 6>::Identity();

  Eigen::Matrix<float, Eigen::Dynamic, 1> z = Eigen::Matrix<float, 6, 1>::Zero();
  Eigen::Matrix<float, Eigen::Dynamic, 1> h = Eigen::Matrix<float, 6, 1>::Zero();

  EXPECT_CALL(*mock_model_, UpdateSensor).WillOnce(DoAll(SetArgReferee<3>(landmark_descriptors),
                                                         SetArgReferee<5>(matched_indices),
                                                         SetArgReferee<6>(Q),
                                                         SetArgReferee<7>(z),
                                                         SetArgReferee<8>(h),
                                                         SetArgReferee<9>(H),
                                                         SetArgReferee<10>(new_coordinates)));

  filter_.UpdateSensorModelMeasurement(nullptr, 0.5);

}

}
}