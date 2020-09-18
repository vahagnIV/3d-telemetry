//
// Created by vahagn on 18/09/20.
//

#ifndef INC_3D_TELEMETRY_SENSOR_MOCK_H
#define INC_3D_TELEMETRY_SENSOR_MOCK_H
#include <isensor_model.h>
#include "gmock/gmock.h"
#include <Eigen/Dense>

namespace gago {
namespace test {
namespace mock {



template<typename TSensorMeasurement, typename TLandmarkDescriptor, typename TData=float, int Dim = 3>
class MockSensorModel : public ISensorModel<TSensorMeasurement, TLandmarkDescriptor, TData, Dim> {
 public:
  typedef Eigen::Matrix<TData, Eigen::Dynamic, Dim> CoordType;
  typedef Eigen::Matrix<TData, Eigen::Dynamic, 1> ZHType;
  typedef Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> SQuareType;
  typedef Eigen::Array<int, Eigen::Dynamic, 1> SliceType;
  typedef Eigen::Matrix<TData, Eigen::Dynamic, 1> TState;
  MOCK_METHOD(void,
              UpdateSensor,
              (   const TSensorMeasurement & sensor_measurement,
                  double sensor_measurement_time,
                  double last_measurement_time,
                  TLandmarkDescriptor & current_landmark_descriptors,
                  const TState & current_coordinates,
                  SliceType & out_matched_landmark_indices,
                  SQuareType & out_Q,
                  ZHType & out_z,
                  ZHType & out_h,
                  SQuareType & out_jacobian,
                  CoordType & new_coordinates),
              (override));
};


//class MockSensorModel : public ISensorModel<float, float, float, 3> {
// public:
//  MOCK_METHOD(void,
//              UpdateSensor,
//              (const float & sensor_measurement,
//                  double sensor_measurement_time,
//                  double last_measurement_time,
//                  float & current_landmark_descriptors,
//                  Eigen::Matrix<float, Eigen::Dynamic, 3> & current_coordinates,
//                  Eigen::Array<int, Eigen::Dynamic, 1> & out_matched_landmark_indices,
//                  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_Q,
//                  Eigen::Matrix<float, Eigen::Dynamic, 1> & out_z,
//                  Eigen::Matrix<float, Eigen::Dynamic, 1> & out_h,
//                  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_jacobian,
//                  float & new_landmark_descriptors,
//                  Eigen::Matrix<float, Eigen::Dynamic, 3> & new_coordinates),
//              (override));
//};

}
}
}

#endif //INC_3D_TELEMETRY_SENSOR_MOCK_H
