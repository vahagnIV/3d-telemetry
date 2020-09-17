//
// Created by vahagn on 9/15/20.
//

#ifndef INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_
#define INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_

#include <vector>
#include <Eigen/Eigen>

#include "imotion_model.h"

namespace gago {

struct Velocity2dMeasurement {
  double v;
  double omega;
};

template<typename TData=float>
class Velocity2DMotionModel : public IMotionModel<Velocity2dMeasurement, TData> {
 public:
  Velocity2DMotionModel() {}

  void PredictNewStateVector(const Velocity2dMeasurement & measurement,
                             double current_time,
                             Eigen::Matrix<TData, Eigen::Dynamic, 1> & state,
                             Eigen::Matrix<TData, 6, 6> & jacobian,
                             Eigen::Matrix<TData, 6, 6> & r,
                             double last_state_update_time) override {

    // TODO: implement
    state(Eigen::seq(0, 5), 0) = Eigen::Matrix<TData, 5, 1>::Ones();
    jacobian = Eigen::Matrix<TData, 6, 6>::Identity();
    r = Eigen::Matrix<TData, 6, 6>::Identity() * 2;
//    std::cout << self_state << std::endl;
  }

 private:

};

}

#endif //INC_3D_TELEMETRY_INCLUDE_VELOCITY_2D_MOTION_MODEL_H_
