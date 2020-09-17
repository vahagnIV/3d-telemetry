//
// Created by vahagn on 16/09/20.
//

#ifndef INC_3D_TELEMETRY_RGBD_SENSOR_MODEL_H
#define INC_3D_TELEMETRY_RGBD_SENSOR_MODEL_H
#include <isensor_model.h>
#include <ifeature_extractor.h>
#include <opencv2/opencv.hpp>
namespace gago {

struct RGBDData{

};

typedef IFeatureExtractor<float, float> TFeatureExtractor;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> RgbdLandmarks;

template <int StateDim = 6>
 class RGBDSensorModel : public ISensorModel<RGBDData, RgbdLandmarks> {
 public:
  RGBDSensorModel(const std::shared_ptr<TFeatureExtractor> & feature_extractor)
      : feature_extractor_(feature_extractor) {

  }

   void UpdateSensor(const RGBDData & sensor_measurement,
                     double sensor_measurement_time,
                     double last_measurement_time,
                     RgbdLandmarks & current_landmark_descriptors,
                     TCoordinateSet & current_coordinates,
                     Eigen::Array<int, Eigen::Dynamic, 1> & out_matched_landmark_indices,
                     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_Q,
                     Eigen::Matrix<float, Eigen::Dynamic, 1> & out_z,
                     Eigen::Matrix<float, Eigen::Dynamic, 1> & out_h,
                     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_jacobian,
                     RgbdLandmarks & new_landmark_descriptors,
                     TCoordinateSet & new_coordinates){



  }
  private:
  std::shared_ptr<TFeatureExtractor> feature_extractor_;

};

}

#endif //INC_3D_TELEMETRY_RGBD_SENSOR_MODEL_H
