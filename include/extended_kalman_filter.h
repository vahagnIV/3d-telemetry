//
// Created by vahagn on 15/09/20.
//

#ifndef INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
#define INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
#include <imotion_model.h>

namespace gago {

template<typename TData, int SelfDim>
class ExtendedKalmanFilter {
 public:
  ExtendedKalmanFilter() : sigma_(Eigen::Matrix<TData, SelfDim, SelfDim>::Zero()),
                           state_vector_(Eigen::Matrix<TData, SelfDim, 1>::Zero()) {}
  void ComputeEvolution(const std::shared_ptr<IMotionModel<TData, SelfDim>> &motion_model, double measurement_time) {
    Eigen::Matrix<TData, SelfDim, SelfDim> jacobian, r;
    const Eigen::Matrix<TData, SelfDim, 1> & mu = state_vector_.topRows(SelfDim);
    std::cout << state_vector_ << std::endl;
    //motion_model->GetJacobianAndError(mu, jacobian, r, last_state_update_time_, measurement_time);
    std::cout << state_vector_ << std::endl;

  }
 private:
  Eigen::Matrix<TData, SelfDim, 1> state_vector_;
  Eigen::Matrix<TData, SelfDim, SelfDim> sigma_;
  double last_state_update_time_;

};

}
#endif //INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
