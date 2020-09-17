//
// Created by vahagn on 16/09/20.
//

#ifndef INC_3D_TELEMETRY_ISENSOR_MODEL_H
#define INC_3D_TELEMETRY_ISENSOR_MODEL_H
#include <Eigen/Dense>

namespace gago {

template<typename TSensorMeasurement,
    typename TLandmarkDescriptor,
    typename TData = float, int Dim = 3>
class ISensorModel {
 public:

  typedef Eigen::Matrix<TData, Eigen::Dynamic, Dim> TCoordinateSet;

  virtual void UpdateSensor(const TSensorMeasurement & sensor_measurement,
                            double sensor_measurement_time,
                            double last_measurement_time,
                            TLandmarkDescriptor & current_landmark_descriptors,
                            TCoordinateSet & current_coordinates,
                            Eigen::Array<int, Eigen::Dynamic, 1> & out_matched_landmark_indices,
                            Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> & out_Q,
                            Eigen::Matrix<TData, Eigen::Dynamic, 1> & out_z,
                            Eigen::Matrix<TData, Eigen::Dynamic, 1> & out_h,
                            Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> & out_jacobian,
                            TLandmarkDescriptor & new_landmark_descriptors,
                            TCoordinateSet & new_coordinates) = 0;
};

}

#endif //INC_3D_TELEMETRY_ISENSOR_MODEL_H
