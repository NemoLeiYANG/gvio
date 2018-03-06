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
  int tag_rows = 0;         ///< Number of tags in y-dir
  int tag_cols = 0;         ///< Number of tags in x-dir
  double tag_size = 0.0;    ///< Size of a tag [m]
  double tag_spacing = 0.0; ///< Space between tags [m]

  // Construct an AprilGrid calibration target
  //
  // Corner ordering in AprilGrid.object_points:
  //
  //   12-----13  14-----15
  //   | TAG 3 |  | TAG 4 |
  //   8-------9  10-----11
  //   4-------5  6-------7
  // y | TAG 1 |  | TAG 2 |
  // ^ 0-------1  2-------3
  // |-->x
  MatX object_points;

  /// AprilGrid detector
  AprilTags::TagDetector detector =
      AprilTags::TagDetector(AprilTags::tagCodes36h11);

  AprilGrid();
  AprilGrid(const int rows,
            const int cols,
            const double tag_size,
            const double tag_spacing);
  virtual ~AprilGrid();

  /**
   * Detect
   *
   * @param image Input image
   * @return 0 for success, -1 for failure
   */
  int detect(cv::Mat &image);

  int solvePnP(const std::map<int, std::vector<Vec2>> &tags);

  /**
   * Extract tags
   *
   * @param image Input image
   * @return 0 for success, -1 for failure
   */
  int extractTags(cv::Mat &image, std::map<int, std::vector<Vec2>> &tags);
};

} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_APRILGRID_HPP
