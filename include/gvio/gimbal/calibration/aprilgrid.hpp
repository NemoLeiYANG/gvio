/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_APRILGRID_HPP
#define GVIO_GIMBAL_APRILGRID_HPP

#include <apriltags_mit/TagDetector.h>
#include <apriltags_mit/Tag36h11.h>

#include "gvio/util/util.hpp"
#include "gvio/camera/distortion.hpp"

namespace gvio {

class AprilGrid {
public:
  int tag_rows = 0;         ///< Number of tags in y-dir
  int tag_cols = 0;         ///< Number of tags in x-dir
  double tag_size = 0.0;    ///< Size of a tag [m]
  double tag_spacing = 0.0; ///< Space between tags [m]

  // AprilGrid grid_points:
  //
  //   12-----13  14-----15
  //   | TAG 3 |  | TAG 4 |
  //   8-------9  10-----11
  //   4-------5  6-------7
  // y | TAG 1 |  | TAG 2 |
  // ^ 0-------1  2-------3
  // |-->x
  MatX grid_points;

  // AprilGrid tag coordinates:
  //
  //   12-----13  14-----15
  //   | (0,1) |  | (1,1) |
  //   8-------9  10-----11
  //   4-------5  6-------7
  // y | (0,0) |  | (1,0) |
  // ^ 0-------1  2-------3
  // |-->x
  std::map<int, std::pair<int, int>> tag_coordinates;

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

  /**
   * Extract tags
   *
   * @param image Input image
   * @param tags Observed AprilGrid tags
   * @return 0 for success, -1 for failure
   */
  int extractTags(cv::Mat &image,
                  std::map<int, std::vector<cv::Point2f>> &tags);

  /**
   * Form object points
   *
   * @param tags Observed AprilGrid tags
   * @returns Object points for SolvePnP
   */
  std::vector<cv::Point3f>
  formObjectPoints(const std::map<int, std::vector<cv::Point2f>> &tags);

  /**
   * Form image points
   *
   * @param tags Observed AprilGrid tags
   * @returns Image points observed for SolvePnP
   */
  std::vector<cv::Point2f>
  formImagePoints(const std::map<int, std::vector<cv::Point2f>> &tags);

  /**
   * SolvePnP
   *
   * @param tags Observed AprilGrid tags
   * @param K Camera intrinsics matrix K
   * @param grid_points AprilGrid tag corners in 3D
   *
   * @return 0 for success, -1 for failure
   */
  int solvePnP(const std::map<int, std::vector<cv::Point2f>> &tags,
               const cv::Mat &K,
               MatX &grid_points);
};

} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_APRILGRID_HPP
