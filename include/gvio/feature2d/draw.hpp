/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_DRAW_HPP
#define GVIO_FEATURE2D_DRAW_HPP

#include "gvio/feature2d/feature.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
  * Draw tracks
  *
  * @param img_cur Current image frame
  * @param p0 Previous corners
  * @param p1 Current corners
  * @param status Corners status
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
  * Draw tracks
  *
  * @param img_cur Current image frame
  * @param p0 Previous corners
  * @param p1 Current corners
  * @param status Corners status
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
  * Draw matches
  *
  * @param img0 Image frame 0
  * @param img1 Image frame 1
  * @param k0 Previous keypoints
  * @param k1 Current keypoints
  * @param matches Feature matches
  * @returns Image with feature matches between frame 0 and 1
  */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status);

/**
  * Draw matches
  *
  * @param img0 Previous image frame
  * @param img1 Current image frame
  * @param k0 Previous keypoints
  * @param k1 Current keypoints
  * @param matches Feature matches
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
  * Draw features
  *
  * @param image Image frame
  * @param features List of features
  * @returns Features image
  */
cv::Mat draw_features(const cv::Mat &image, const Features features);

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_DRAW_HPP
