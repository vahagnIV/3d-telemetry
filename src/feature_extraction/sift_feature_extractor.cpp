//
// Created by vahagn on 08/09/20.
//

#include "sift_feature_extractor.h"
#include <opencv2/xfeatures2d.hpp>

namespace gago {

SiftFeatureExtractor::SiftFeatureExtractor(size_t max_count) : OpenCVFeatureExtractor(max_count, cv::SIFT::create()) {
}


}