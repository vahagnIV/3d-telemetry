//
// Created by vahagn on 15/09/20.
//

#ifndef INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
#define INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
#include <imotion_model.h>
#include <isensor_model.h>

namespace gago {

template<
    typename TMotionModelMeasurement,
    typename TSensorModelMeasurement,
    typename TLandmarkDescriptor,
    typename TData = float,
    int StateDim = 6,
    int LandmarkDim = 3>

class ExtendedKalmanFilter {
 public:
  ExtendedKalmanFilter(const std::shared_ptr<IMotionModel<TMotionModelMeasurement, TData, StateDim >> & motion_model,
                       const std::shared_ptr<ISensorModel<TSensorModelMeasurement,
                                                          TLandmarkDescriptor,
                                                          TData,
                                                          LandmarkDim>> & sensor_model) :
      sigma_(Eigen::Matrix<TData,
                           StateDim,
                           StateDim>::Zero()),
      state_vector_(Eigen::Matrix<TData,
                                  StateDim,
                                  1>::Zero()),
      motion_model_(motion_model), sensor_model_(sensor_model) {}

  void UpdateMotionModel(TMotionModelMeasurement & measurement, double time) {
    Eigen::Matrix<TData, StateDim, StateDim> jacobian, r;
    motion_model_->PredictNewStateVector(measurement, time,
                                         state_vector_,
                                         jacobian,
                                         r,
                                         last_state_update_time_);
    auto && sigma_x = sigma_(Eigen::seq(0, StateDim - 1), Eigen::seq(0, StateDim - 1));
    auto && sigma_xm = sigma_(Eigen::seq(0, StateDim - 1), Eigen::seq(StateDim, sigma_.cols() - 1));
    auto && sigma_mx = sigma_(Eigen::seq(StateDim, sigma_.rows() - 1), Eigen::seq(0, StateDim - 1));
    sigma_x = jacobian.transpose() * sigma_x * jacobian + r;
    sigma_xm = jacobian * sigma_xm;
    sigma_mx = sigma_xm.transpose();
    last_state_update_time_ = time;
  }

  void UpdateSensorModelMeasurement(const TSensorModelMeasurement & measurement, double time) {

    Eigen::Array<int, Eigen::Dynamic, 1> matched_landmark_indices;
    Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> Q;
    Eigen::Matrix<TData, Eigen::Dynamic, 1> z;
    Eigen::Matrix<TData, Eigen::Dynamic, 1> h;
    Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> H;

    typename ISensorModel<TSensorModelMeasurement, TLandmarkDescriptor, TData, LandmarkDim>::TCoordinateSet
        new_coordinates;

    sensor_model_->UpdateSensor(measurement,
                                time,
                                0,
                                current_landmark_descriptors_,
                                state_vector_,
                                matched_landmark_indices,
                                Q,
                                z,
                                h,
                                H,
                                new_coordinates);

    assert(matched_landmark_indices.rows() % LandmarkDim == 0);

    Eigen::Array<int, Eigen::Dynamic, 1> matched_landmark_indices_main(StateDim + matched_landmark_indices.rows());
    for (int i = 0; i < StateDim; ++i)
      matched_landmark_indices_main(i, 0) = i;

    matched_landmark_indices_main(Eigen::seq(StateDim, StateDim + matched_landmark_indices.rows() - 1), Eigen::all) =
        matched_landmark_indices;
    auto && sigma_slice = sigma_(matched_landmark_indices_main, matched_landmark_indices_main);
    auto && HT = H.transpose();
    auto K = sigma_slice * HT * (H * sigma_slice * HT + Q).inverse();
    state_vector_(matched_landmark_indices_main) += K * (z - h);
    sigma_slice = (-K * H
        + Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic>::Identity(sigma_slice.rows(), sigma_slice.cols()))
        * sigma_slice;
    sigma_(Eigen::seq(0, matched_landmark_indices_main.rows() - 1),
           Eigen::seq(0, matched_landmark_indices_main.rows() - 1)) =
        sigma_slice;

    long new_size = matched_landmark_indices_main.rows() + new_coordinates.size();
    sigma_.conservativeResize(new_size, new_size);

    sigma_(Eigen::all, Eigen::seq(sigma_slice.cols(), new_size - 1)) =
        Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic>::Zero(sigma_.rows(), new_size - sigma_slice.cols());

    sigma_(Eigen::seq(sigma_slice.rows(), new_size - 1),
           Eigen::seq(0, sigma_slice.cols() - 1)) =
        Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic>::Zero(new_size - sigma_slice.rows(), sigma_slice.cols());

    state_vector_.conservativeResize(new_size);
    state_vector_(Eigen::seq(matched_landmark_indices_main.rows(), new_size - 1)) = new_coordinates.reshaped(new_coordinates.size() , 1);


  }
 private:

//  void


  Eigen::Matrix<TData, Eigen::Dynamic, 1> state_vector_;
  Eigen::Matrix<TData, Eigen::Dynamic, Eigen::Dynamic> sigma_;
  std::shared_ptr<IMotionModel<TMotionModelMeasurement, TData, StateDim >> motion_model_;
  std::shared_ptr<ISensorModel<TSensorModelMeasurement, TLandmarkDescriptor, TData, LandmarkDim>> sensor_model_;
  TLandmarkDescriptor current_landmark_descriptors_;
  double last_state_update_time_;

};

}
#endif //INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
