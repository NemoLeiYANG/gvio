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

/* GMS Matcher threshold */
#define THRESH_FACTOR 6

// 8 possible rotation and each one is 3 X 3
// clang-format off
const int mRotationPatterns[8][9] = {
  1, 2, 3,
  4, 5, 6,
  7, 8, 9,

  4, 1, 2,
  7, 5, 3,
  8, 9, 6,

  7, 4, 1,
  8, 5, 2,
  9, 6, 3,

  8, 7, 4,
  9, 5, 1,
  6, 3, 2,

  9, 8, 7,
  6, 5, 4,
  3, 2, 1,

  6, 9, 8,
  3, 5, 7,
  2, 1, 4,

  3, 6, 9,
  2, 5, 8,
  1, 4, 7,

  2, 3, 6,
  1, 5, 9,
  4, 7, 8
};
// clang-format on

// 5 level scales
const double mScaleRatios[5] = {1.0, 1.0 / 2, 1.0 / sqrt(2.0), sqrt(2.0), 2.0};

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
  cv::Ptr<cv::DescriptorMatcher> bf_matcher;  ///< Brute-force Matcher
  std::vector<cv::Point2f> mvP1, mvP2;        ///< Normalized points
  std::vector<std::pair<int, int>> mvMatches; ///< Matches
  size_t mNumberMatches = 0;                  ///< Number of Matches
  cv::Size mGridSizeLeft, mGridSizeRight;     ///< Grid sizes
  int mGridNumberLeft, mGridNumberRight;

  // x : left grid idx
  // y : right grid idx
  // value : how many matches from idx_left to idx_right
  cv::Mat mMotionStatistics;

  std::vector<int>
      mNumberPointsInPerCellLeft; ///< Number of points left per cell

  // Inldex  : grid_idx_left
  // Value   : grid_idx_right
  std::vector<int> mCellPairs;

  // Every Matches has a cell-pair
  // first  : grid_idx_left
  // second : grid_idx_right
  std::vector<std::pair<int, int>> mvMatchPairs;

  // Inlier Mask for output
  std::vector<bool> mvbInlierMask;

  // Grid neighbor
  cv::Mat mGridNeighborLeft;
  cv::Mat mGridNeighborRight;

  GMSMatcher() {
#ifdef USG_CUDA
    bf_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
#else
    bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
#endif
  }
  ~GMSMatcher() {
  }

  /**
   * Normalize Key Points to Range(0 - 1)
   */
  std::vector<cv::Point2f> normalizePoints(const std::vector<cv::KeyPoint> &kp,
                                           const cv::Size &size);

  /**
   * Convert OpenCV DMatch to `std::pair<int, int>`
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
  void assignMatchPairs(const int GridType);

  /**
   * Verify cell pairs
   */
  void verifyCellPairs(const int RotationType);

  /**
   * Get neighbor 9
   */
  std::vector<int> getNB9(const int idx, const cv::Size &GridSize);

  /**
   * Initialize neighbors
   */
  void initNeighbors(cv::Mat &neighbor, const cv::Size &GridSize);

  /**
   * Set scale
   */
  void setScale(const int Scale);

  /**
   * Run
   */
  int run(const int RotationType);

  /**
   * Get inlier mask
   */
  std::vector<bool> getInlierMask(const bool WithScale = false,
                                  const bool WithRotation = false);

  /**
   * Match
   */
  int match(const std::vector<cv::KeyPoint> &kp1,
            const cv::Mat &des1,
            const std::vector<cv::KeyPoint> &kp2,
            const cv::Mat &des2,
            const cv::Size &img_size,
            std::vector<cv::DMatch> &matches);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_GMS_MATCHER_HPP
