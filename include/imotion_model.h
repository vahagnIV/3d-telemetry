//
// Created by vahagn on 15/09/20.
//

#ifndef INC_3D_TELEMETRY_IMOTION_MODEL_H
#define INC_3D_TELEMETRY_IMOTION_MODEL_H
#include <memory>
#include <Eigen/Dense>

namespace gago {

template<typename TData, int Dim>
class IMotionModel {
 public:
  virtual void GetJacobianAndError(Eigen::Matrix<TData, Dim, 1> &self_state,
                                   Eigen::Matrix<TData, Dim, Dim> &jacobian,
                                   Eigen::Matrix<TData, Dim, Dim> &r,
                                   double last_state_update_time,
                                   double current_time) = 0;
};

}
#endif //INC_3D_TELEMETRY_IMOTION_MODEL_H
