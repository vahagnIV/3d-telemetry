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
  void ComputeEvolution(const std::shared_ptr<IMotionModel<TData, SelfDim>> & motion_model, double measurement_time) {
    std::cout << "Sigma_before" << std::endl << sigma_ << std::endl;
    Predict(motion_model, measurement_time);
    std::cout << "Sigma_asfter" << std::endl << sigma_ << std::endl;

  }
 private:

//  void

  void Predict(const std::shared_ptr<IMotionModel<TData, SelfDim>> & motion_model, double measurement_time) {
    Eigen::Matrix<TData, SelfDim, SelfDim> jacobian, r;
    motion_model->GetJacobianAndError(state_vector_, jacobian, r, last_state_update_time_, measurement_time);
    auto && sigma_x = sigma_(Eigen::seq(0, SelfDim-1), Eigen::seq(0, SelfDim-1));
    auto && sigma_xm = sigma_(Eigen::seq(0, SelfDim-1), Eigen::seq(SelfDim , sigma_.cols() - 1));
    auto && sigma_mx = sigma_(Eigen::seq(SelfDim , sigma_.rows()-1), Eigen::seq(0, SelfDim-1));
    sigma_x = jacobian.transpose() * sigma_x * jacobian + r;
    sigma_xm = jacobian * sigma_xm;
    sigma_mx = sigma_xm.transpose();
  }

  Eigen::Matrix<TData, SelfDim, 1> state_vector_;
  Eigen::Matrix<TData, SelfDim, SelfDim> sigma_;
  double last_state_update_time_;

};

}
#endif //INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
