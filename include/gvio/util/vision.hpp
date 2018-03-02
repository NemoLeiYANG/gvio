/**
 * @file
 * @defgroup vision vision
 * @ingroup util
 */
#ifndef GVIO_UTIL_VISION_HPP
#define GVIO_UTIL_VISION_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/math.hpp"

namespace gvio {
/**
 * @addtogroup vision
 * @{
 */

/**
 * Compare `cv::Mat` whether they are equal
 *
 * @param m1 First matrix
 * @param m2 Second matrix
 * @returns true or false
 */
bool is_equal(const cv::Mat &m1, const cv::Mat &m2);

/**
 * Convert cv::Mat to Eigen::Matrix
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const cv::Mat &x, MatX &y);

/**
 * Convert Eigen::Matrix to cv::Mat
 *
 * @param x Input matrix
 * @param y Output matrix
 */
void convert(const MatX &x, cv::Mat &y);

/**
 * Convert x to homogenous coordinates
 *
 * @param x Input vector
 * @return Output vector in homogeneous coordinates
 */
Vec3 homogeneous(const Vec2 &x);

/**
 * Convert x to homogenous coordinates
 *
 * @param x Input vector
 * @return Output vector in homogeneous coordinates
 */
Vec4 homogeneous(const Vec3 &x);

/**
 * Normalize vector of
 */
Vec2 normalize(const Vec2 &x);

/** @} group vision */
} // namespace gvio
#endif // GVIO_UTIL_VISION_HPP
