/**
 * @file
 * @ingroup camera
 */
#ifndef GVIO_CAMERA_DISTORTION_HPP
#define GVIO_CAMERA_DISTORTION_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Distort 3D points with the radial-tangential distortion model
 *
 * Source:
 * http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 */
MatX radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const MatX &points);

/**
 * Distort 3D points with the equi-distant distortion model
 */
MatX equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const MatX &points);

/**
 * Distort a single 3D point with the equi-distant distortion model
 */
Vec2 equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const Vec3 &point);

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_DISTORTION_HPP
