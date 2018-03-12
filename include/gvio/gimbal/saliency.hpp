/**
 * @file
 * @defgroup gimbal gimbal
 */
#ifndef GVIO_GIMBAL_SALIENCY_HPP
#define GVIO_GIMBAL_SALIENCY_HPP

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
// #include <opencv2/saliency/saliencyBaseClasses.hpp>
// #include <opencv2/saliency/saliencySpecializedClasses.hpp>

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class Saliency {
public:
  // cv::Ptr<cv::saliency::Saliency> detector;

  Saliency();
  virtual ~Saliency();

  /**
   * Detect
   *
   * @param frame Image frame
   * @param map Saliency map
   * @returns 0 for success, -1 for failure
   */
  int detect(const cv::Mat &frame);
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_SALIENCY_HPP
