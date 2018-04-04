#include "gvio/feature2d/stereo_tracker.hpp"

namespace gvio {

StereoTracker::StereoTracker() { this->hello; }

StereoTracker::~StereoTracker() {}

int StereoTracker::update(const cv::Mat &img_cur) { return 0; }

} // namespace gvio
