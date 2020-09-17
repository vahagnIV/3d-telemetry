//
// Created by vahagn on 17/09/20.
//

#include "surf_feature_extractor.h"
#include <opencv2/xfeatures2d.hpp>

namespace gago{

SurfFeatureExtractor::SurfFeatureExtractor(size_t max_count): OpenCVFeatureExtractor(max_count,cv::xfeatures2d::SURF::create()) {

}

}