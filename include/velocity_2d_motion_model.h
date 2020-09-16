//
// Created by vahagn on 9/15/20.
//

#ifndef INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_
#define INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_

#include <vector>

#include "imotion_model.h"

namespace gago {

namespace internal {

struct __vel_measurement {
  double t;
  double v;
  double omega;
};

}

template<typename TData>
class Velocity2DMotionModel : public IMotionModel<TData, 3> {
 public:
  Velocity2DMotionModel(int measurement_frequency, int time_depth)
      : last_measurement_index_(measurement_frequency * time_depth - 1),
        measurements_(measurement_frequency * time_depth) {}

  void AppendMeasurement(double v, double omega, double t) {
    last_measurement_index_ = (last_measurement_index_ + 1) % measurements_.size();
    measurements_[last_measurement_index_].v = v;
    measurements_[last_measurement_index_].omega = omega;
    measurements_[last_measurement_index_].t = t;
  }

  void GetJacobianAndError(Eigen::Matrix<TData, 3, 1> &self_state,
                           Eigen::Matrix<TData, 3, 3> &jacobian,
                           Eigen::Matrix<TData, 3, 3> &r,
                           double last_state_update_time,
                           double current_time) override {

    // TODO: implement
    self_state(0, 0) = self_state(1, 0) = self_state(2, 0) = 1;
    jacobian = Eigen::Matrix<TData, 3, 3>::Identity();
    r = Eigen::Matrix<TData, 3, 3>::Identity();
    std::cout << self_state << std::endl;

  }
 private:
  std::vector<gago::internal::__vel_measurement> measurements_;
  size_t last_measurement_index_;

};

}

#endif //INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_
