//
// Created by vahagn on 08/09/20.
//

#ifndef INC_3D_TELEMETRY_OPEN_CV_FEATURE_EXTRACTOR_H
#define INC_3D_TELEMETRY_OPEN_CV_FEATURE_EXTRACTOR_H
#include "ifeature_extractor.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace gago {
class OpenCVFeatureExtractor : public IFeatureExtractor<float, float> {
 public:
  OpenCVFeatureExtractor(size_t max_count, cv::Ptr<cv::Feature2D> ptr) : max_count_(max_count), ptr_(ptr) {}
  void Extract(uint8_t *rgb,
               int w,
               int h,
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_descriptors,
               Eigen::Matrix<float, -1, 2> & out_keypoints) override;
 private:
  virtual void OpencvExtract(const cv::Mat & image,
                             std::vector<cv::KeyPoint> & out_keypoints,
                             cv::Mat & out_descriptors);
  cv::Ptr<cv::Feature2D> ptr_;
  size_t max_count_;

};
}

#endif //INC_3D_TELEMETRY_OPEN_CV_FEATURE_EXTRACTOR_H
