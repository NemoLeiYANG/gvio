#ifndef GVIO_FEATURE2D_GMS_MATCHER_HPP
#define GVIO_FEATURE2D_GMS_MATCHER_HPP
/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, JiaWang Bian
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * - Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>

#ifdef USE_GPU
#include <opencv2/cudafeatures2d.hpp>
using cv::cuda::GpuMat;
#endif

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

inline cv::Mat DrawInlier(cv::Mat &src1,
                          cv::Mat &src2,
                          std::vector<cv::KeyPoint> &kpt1,
                          std::vector<cv::KeyPoint> &kpt2,
                          std::vector<cv::DMatch> &inlier,
                          int type) {
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

inline void imresize(cv::Mat &src, int height) {
  double ratio = src.rows * 1.0 / height;
  int width = static_cast<int>(src.cols * 1.0 / ratio);
  resize(src, src, cv::Size(width, height));
}

class GMSMatcher {
public:
  // OpenCV Keypoints & Correspond Image Size & Nearest Neighbor Matches
  GMSMatcher(const std::vector<cv::KeyPoint> &vkp1,
             const cv::Size size1,
             const std::vector<cv::KeyPoint> &vkp2,
             const cv::Size size2,
             const std::vector<cv::DMatch> &vDMatches) {
    // Input initialize
    NormalizePoints(vkp1, size1, mvP1);
    NormalizePoints(vkp2, size2, mvP2);
    mNumberMatches = vDMatches.size();
    ConvertMatches(vDMatches, mvMatches);

    // Grid initialize
    mGridSizeLeft = cv::Size(20, 20);
    mGridNumberLeft = mGridSizeLeft.width * mGridSizeLeft.height;

    // Initialize the neihbor of left grid
    mGridNeighborLeft = cv::Mat::zeros(mGridNumberLeft, 9, CV_32SC1);
    InitalizeNeighbors(mGridNeighborLeft, mGridSizeLeft);
  };
  ~GMSMatcher(){};

  // Get Inlier Mask
  // Return number of inliers
  int GetInlierMask(std::vector<bool> &vbInliers,
                    bool WithScale = false,
                    bool WithRotation = false);

private:
  // Normalized Points
  std::vector<cv::Point2f> mvP1, mvP2;

  // Matches
  std::vector<std::pair<int, int>> mvMatches;

  // Number of Matches
  size_t mNumberMatches;

  // Grid cv::Size
  cv::Size mGridSizeLeft, mGridSizeRight;
  int mGridNumberLeft;
  int mGridNumberRight;

  // x    : left grid idx
  // y      :  right grid idx
  // value  : how many matches from idx_left to idx_right
  cv::Mat mMotionStatistics;

  // Number of points left per cell
  std::vector<int> mNumberPointsInPerCellLeft;

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

  // Normalize Key Points to Range(0 - 1)
  void NormalizePoints(const std::vector<cv::KeyPoint> &kp,
                       const cv::Size &size,
                       std::vector<cv::Point2f> &npts) {
    const size_t numP = kp.size();
    const int width = size.width;
    const int height = size.height;
    npts.resize(numP);

    for (size_t i = 0; i < numP; i++) {
      npts[i].x = kp[i].pt.x / width;
      npts[i].y = kp[i].pt.y / height;
    }
  }

  // Convert OpenCV DMatch to Match (pair<int, int>)
  void ConvertMatches(const std::vector<cv::DMatch> &vDMatches,
                      std::vector<std::pair<int, int>> &vMatches) {
    vMatches.resize(mNumberMatches);
    for (size_t i = 0; i < mNumberMatches; i++) {
      vMatches[i] =
          std::pair<int, int>(vDMatches[i].queryIdx, vDMatches[i].trainIdx);
    }
  }

  int GetGridIndexLeft(const cv::Point2f &pt, int type) {
    int x = 0, y = 0;

    if (type == 1) {
      x = floor(pt.x * mGridSizeLeft.width);
      y = floor(pt.y * mGridSizeLeft.height);
    }

    if (type == 2) {
      x = floor(pt.x * mGridSizeLeft.width + 0.5);
      y = floor(pt.y * mGridSizeLeft.height);
    }

    if (type == 3) {
      x = floor(pt.x * mGridSizeLeft.width);
      y = floor(pt.y * mGridSizeLeft.height + 0.5);
    }

    if (type == 4) {
      x = floor(pt.x * mGridSizeLeft.width + 0.5);
      y = floor(pt.y * mGridSizeLeft.height + 0.5);
    }

    if (x >= mGridSizeLeft.width || y >= mGridSizeLeft.height) {
      return -1;
    }

    return x + y * mGridSizeLeft.width;
  }

  int GetGridIndexRight(const cv::Point2f &pt) {
    int x = floor(pt.x * mGridSizeRight.width);
    int y = floor(pt.y * mGridSizeRight.height);

    return x + y * mGridSizeRight.width;
  }

  void AssignMatchPairs(int GridType);

  void VerifyCellPairs(int RotationType);

  // Get Neighbor 9
  std::vector<int> GetNB9(const int idx, const cv::Size &GridSize) {
    std::vector<int> NB9(9, -1);

    int idx_x = idx % GridSize.width;
    int idx_y = idx / GridSize.width;

    for (int yi = -1; yi <= 1; yi++) {
      for (int xi = -1; xi <= 1; xi++) {
        int idx_xx = idx_x + xi;
        int idx_yy = idx_y + yi;

        if (idx_xx < 0 || idx_xx >= GridSize.width || idx_yy < 0 ||
            idx_yy >= GridSize.height)
          continue;

        NB9[xi + 4 + yi * 3] = idx_xx + idx_yy * GridSize.width;
      }
    }
    return NB9;
  }

  void InitalizeNeighbors(cv::Mat &neighbor, const cv::Size &GridSize) {
    for (int i = 0; i < neighbor.rows; i++) {
      std::vector<int> NB9 = GetNB9(i, GridSize);
      int *data = neighbor.ptr<int>(i);
      memcpy(data, &NB9[0], sizeof(int) * 9);
    }
  }

  void SetScale(int Scale) {
    // Set Scale
    mGridSizeRight.width = mGridSizeLeft.width * mScaleRatios[Scale];
    mGridSizeRight.height = mGridSizeLeft.height * mScaleRatios[Scale];
    mGridNumberRight = mGridSizeRight.width * mGridSizeRight.height;

    // Initialize the neihbor of right grid
    mGridNeighborRight = cv::Mat::zeros(mGridNumberRight, 9, CV_32SC1);
    InitalizeNeighbors(mGridNeighborRight, mGridSizeRight);
  }

  int run(int RotationType);
};

#endif // GVIO_FEATURE2D_GMS_MATCHER_HPP
