/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_GMS_MATCHER_HPP
#define GVIO_FEATURE2D_GMS_MATCHER_HPP

//
// BSD 3-Clause License
//
// Copyright (c) 2017, JiaWang Bian
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// - Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

#ifdef USE_GPU
#include <opencv2/cudafeatures2d.hpp>
using cv::cuda::GpuMat;
#endif

inline cv::Mat draw_inliers(const cv::Mat &src1,
                            const cv::Mat &src2,
                            const std::vector<cv::KeyPoint> &kpt1,
                            const std::vector<cv::KeyPoint> &kpt2,
                            const std::vector<cv::DMatch> &inlier,
                            int type = 1) {
  const int height = std::max(src1.rows, src2.rows);
  const int width = src1.cols + src2.cols;
  cv::Mat output(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  src1.copyTo(output(cv::Rect(0, 0, src1.cols, src1.rows)));
  src2.copyTo(output(cv::Rect(src1.cols, 0, src2.cols, src2.rows)));

  if (type == 1) {
    for (size_t i = 0; i < inlier.size(); i++) {
      cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
      cv::Point2f right =
          (kpt2[inlier[i].trainIdx].pt + cv::Point2f((float) src1.cols, 0.f));
      cv::line(output, left, right, cv::Scalar(0, 255, 255));
    }
  } else if (type == 2) {
    for (size_t i = 0; i < inlier.size(); i++) {
      cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
      cv::Point2f right =
          (kpt2[inlier[i].trainIdx].pt + cv::Point2f((float) src1.cols, 0.f));
      cv::line(output, left, right, cv::Scalar(255, 0, 0));
    }

    for (size_t i = 0; i < inlier.size(); i++) {
      cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
      cv::Point2f right =
          (kpt2[inlier[i].trainIdx].pt + cv::Point2f((float) src1.cols, 0.f));
      cv::circle(output, left, 1, cv::Scalar(0, 255, 255), 2);
      cv::circle(output, right, 1, cv::Scalar(0, 255, 0), 2);
    }
  }

  return output;
}

class GMSMatcher {
public:
  int threshold_factor = 10; ///< Threshold factor

  cv::Ptr<cv::DescriptorMatcher> bf_matcher; ///< Brute-force Matcher
  std::vector<cv::Point2f> points1;          ///< Normalized points 1
  std::vector<cv::Point2f> points2;          ///< Normalized points 2
  std::vector<std::pair<int, int>> matches;  ///< Matches
  size_t nb_matches = 0;                     ///< Number of Matches
  cv::Size grid_size_left;                   ///< Grid size left
  cv::Size grid_size_right;                  ///< Grid size right
  int grid_number_left;                      ///< Grid number left
  int grid_number_right;                     ///< Grid number right

  // x : left grid idx
  // y : right grid idx
  // value : how many matches from idx_left to idx_right
  cv::Mat motion_statistics;

  ///< Number of points left per cell
  std::vector<int> nb_points_per_cell;

  // Index  : grid_idx_left
  // Value   : grid_idx_right
  std::vector<int> cell_pairs;

  // Every Matches has a cell-pair
  // first  : grid_idx_left
  // second : grid_idx_right
  std::vector<std::pair<int, int>> match_pairs;

  // Inlier Mask for output
  std::vector<bool> inliers_mask;

  // Grid neighbor
  cv::Mat grid_neighbor_left;
  cv::Mat grid_neighbor_right;

  // 8 possible rotation and each one is 3 X 3
  const int rotation_patterns[8][9] = {{1, 2, 3, 4, 5, 6, 7, 8, 9},
                                       {4, 1, 2, 7, 5, 3, 8, 9, 6},
                                       {7, 4, 1, 8, 5, 2, 9, 6, 3},
                                       {8, 7, 4, 9, 5, 1, 6, 3, 2},
                                       {9, 8, 7, 6, 5, 4, 3, 2, 1},
                                       {6, 9, 8, 3, 5, 7, 2, 1, 4},
                                       {3, 6, 9, 2, 5, 8, 1, 4, 7},
                                       {2, 3, 6, 1, 5, 9, 4, 7, 8}};

  // 5 level scales
  const double scale_ratios[5] = {1.0,
                                  1.0 / 2,
                                  1.0 / sqrt(2.0),
                                  sqrt(2.0),
                                  2.0};

  GMSMatcher();
  ~GMSMatcher();

  /**
   * Normalize Key Points to range (0 - 1)
   *
   * @param kp Keypoints
   * @param img_size Image size
   * @returns Normalized keypoints
   */
  std::vector<cv::Point2f> normalizePoints(const std::vector<cv::KeyPoint> &kp,
                                           const cv::Size &img_size);

  /**
   * Normalize Key Points to range (0 - 1)
   *
   * @param kp Keypoints
   * @param img_size Image size
   * @returns Normalized keypoints
   */
  std::vector<cv::Point2f> normalizePoints(const std::vector<cv::Point2f> &kp,
                                           const cv::Size &size);

  /**
   * Convert OpenCV DMatch to `std::pair<int, int>`
   *
   * @param vDMatches Input matches
   * @param vMatches Output matches
   */
  void convertMatches(const std::vector<cv::DMatch> &vDMatches,
                      std::vector<std::pair<int, int>> &vMatches);

  /**
   * Get grid index left
   */
  int getGridIndexLeft(const cv::Point2f &pt, const int type);

  /**
   * Get grid index right
   */
  int getGridIndexRight(const cv::Point2f &pt);

  /**
   * Assign match pairs
   */
  void assignMatchPairs(const int grid_type);

  /**
   * Verify cell pairs
   */
  void verifyCellPairs(const int rotation_type);

  /**
   * Get neighbor 9
   */
  std::vector<int> getNB9(const int idx, const cv::Size &grid_size);

  /**
   * Initialize neighbors
   */
  void initNeighbors(cv::Mat &neighbor, const cv::Size &grid_size);

  /**
   * Set scale
   */
  void setScale(const int scale);

  /**
   * Run
   *
   * @param rotation_type Rotation type
   */
  int run(const int rotation_type);

  /**
   * Get inlier mask
   *
   * @param with_scale With scale
   * @param with_rotation With rotation
   * @returns Inlier mask
   */
  std::vector<bool> getInlierMask(const bool with_scale = false,
                                  const bool with_rotation = false);

  /**
   * Match
   *
   * @param kp1 Keypoints from camera 1
   * @param desc1 Descriptors of keypoints detected in camera 1
   * @param kp2 Keypoints from camera 2
   * @param desc2 Descriptors of keypoints detected in camera 2
   * @param img_size Image size
   * @param matches Matches
   */
  int match(const std::vector<cv::KeyPoint> &kp1,
            const cv::Mat &desc1,
            const std::vector<cv::KeyPoint> &kp2,
            const cv::Mat &desc2,
            const cv::Size &img_size,
            std::vector<cv::DMatch> &matches);

  /**
   * Match
   *
   * @param kp1 Keypoints from camera 1
   * @param kp2 Keypoints from camera 2
   * @param img_size Image size
   * @param matches Matches
   */
  int match(const std::vector<cv::Point2f> &kp1,
            const std::vector<cv::Point2f> &kp2,
            const cv::Size &img_size,
            std::vector<bool> &matches);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_GMS_MATCHER_HPP
