//
// Created by vahagn on 14/09/20.
//

#ifndef INC_3D_TELEMETRY_ISTATE_H
#define INC_3D_TELEMETRY_ISTATE_H
#include <Eigen/Dense>

namespace gago {


template<typename Type, int Dim>
using SelfState = Eigen::Matrix<Type, Eigen::Dynamic, 1, Eigen::ColMajor>;



}

#endif //INC_3D_TELEMETRY_ISTATE_H
