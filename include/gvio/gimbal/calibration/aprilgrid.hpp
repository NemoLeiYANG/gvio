/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_APRILGRID_HPP
#define GVIO_GIMBAL_APRILGRID_HPP

#include <apriltags_mit/TagDetector.h>
#include <apriltags_mit/Tag36h11.h>

#include "gvio/util/util.hpp"

namespace gvio {

class AprilGrid {
public:
  MatX grid_points;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  AprilTags::TagDetector detector =
      AprilTags::TagDetector(AprilTags::tagCodes36h11);

  AprilGrid();
  ~AprilGrid();
  int extractCorners(const cv::Mat &image_gray);
};

} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_APRILGRID_HPP
