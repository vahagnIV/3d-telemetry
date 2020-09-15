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
  void ComputeEvolution(const std::shared_ptr<IMotionModel<TData, SelfDim>> & motion_model) {
    Eigen::Matrix<TData, SelfDim, SelfDim> jacobian, r;
    motion_model->GetJacobianAndError(state_vector_, jacobian, r);

  }
 private:
  Eigen::Matrix<TData, SelfDim, 1> state_vector_;
  Eigen::Matrix<TData, SelfDim, SelfDim> sigma_;

};

}
#endif //INC_3D_TELEMETRY_EXTENDED_KALMAN_FILTER_H
