#ifndef GVIO_UTIL_OPENCV_HPP
#define GVIO_UTIL_OPENCV_HPP
/**
 * @file opencv.hpp
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace gvio {

bool cvMatIsEqual(const cv::Mat m1, const cv::Mat m2);

} // namespace gvio
#endif
