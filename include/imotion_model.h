//
// Created by vahagn on 15/09/20.
//

#ifndef INC_3D_TELEMETRY_IMOTION_MODEL_H
#define INC_3D_TELEMETRY_IMOTION_MODEL_H
#include <memory>
#include <Eigen/Dense>

namespace gago {

template<typename TMotionModelMeasurement, typename TData = float, int StateDim = 6>
class IMotionModel {
 public:

  virtual void PredictNewStateVector(const TMotionModelMeasurement & measurement,
                                     double current_time,
                                     Eigen::Matrix<TData, Eigen::Dynamic, 1> & state,
                                     Eigen::Matrix<TData, StateDim, StateDim> & jacobian,
                                     Eigen::Matrix<TData, StateDim, StateDim> & r,
                                     double last_state_update_time) = 0;
};

}
#endif //INC_3D_TELEMETRY_IMOTION_MODEL_H
