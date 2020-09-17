//
// Created by vahagn on 08/09/20.
//

#include "open_cv_feature_extractor.h"

namespace gago {
void OpenCVFeatureExtractor::Extract(uint8_t *rgb, int w,
                                     int h,
                                     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & out_descriptors,
                                     Eigen::Matrix<float, Eigen::Dynamic, 2> & out_keypoints) {
  cv::Mat image(h, w, CV_8UC3, (void *) rgb);
  std::vector<cv::KeyPoint> cv_kps;
  cv::Mat descriptors;
  OpencvExtract(image, cv_kps, descriptors);
  out_descriptors = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>((float *)descriptors.data,descriptors.rows, descriptors.cols);

  out_keypoints.resize(cv_kps.size(), Eigen::NoChange );
  for (int i = 0; i < cv_kps.size(); ++i) {
    out_keypoints(i,0) = cv_kps[i].pt.x;
    out_keypoints(i,1) = cv_kps[i].pt.y;
  }


}

void OpenCVFeatureExtractor::OpencvExtract(const cv::Mat & image,
                                         std::vector<cv::KeyPoint> & out_keypoints,
                                         cv::Mat & out_descriptors) {

  cv::Mat d;
  ptr_->detect(image, out_keypoints);
  out_keypoints.resize(std::min(out_keypoints.size(), max_count_));

  ptr_->compute(image,
                out_keypoints,
                out_descriptors);

}
}