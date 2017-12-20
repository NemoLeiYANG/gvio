/**
 * @file
 * @defgroup euler euler
 * @ingroup util
 */
#ifndef GVIO_UTIL_EULER_HPP
#define GVIO_UTIL_EULER_HPP

#include "gvio/util/math.hpp"

namespace gvio {
/**
 * @addtogroup euler
 * @{
 */

/**
  * Rotation matrix around x-axis (counter-clockwise, right-handed)
  *
  * @param theta Angle in radians
  * @returns Rotation matrix
  */
Mat3 rotx(const double theta);

/**
  * Rotation matrix around y-axis (counter-clockwise, right-handed)
  *
  * @param theta Angle in radians
  * @returns Rotation matrix
  */
Mat3 roty(const double theta);

/**
  * Rotation matrix around z-axis (counter-clockwise, right-handed)
  *
  * @param theta Angle in radians
  * @returns Rotation matrix
  */
Mat3 rotz(const double theta);

/**
 * Convert euler sequence 123 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @param euler Euler angle (roll, pitch, yaw)
 * @returns Rotation matrix
 */
Mat3 euler123ToRot(const Vec3 &euler);

/**
 * Convert euler sequence 321 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @param euler Euler angle (roll, pitch, yaw)
 * @returns Rotation matrix
 */
Mat3 euler321ToRot(const Vec3 &euler);

/** @} group data */
} // namespace gvio
#endif // GVIO_UTIL_EULER_HPP
