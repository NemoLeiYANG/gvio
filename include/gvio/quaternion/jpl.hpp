/**
 * @file
 * @ingroup quaternion
 */
#ifndef GVIO_QUATERNION_JPL_HPP
#define GVIO_QUATERNION_JPL_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup quaternion
 * @{
 */

/**
 * Quaternion norm
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Norm of quaternion
 */
double quatnorm(const Vec4 &q);

/**
 * Quaternion normalize
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Normalized quaternion
 */
Vec4 quatnormalize(const Vec4 &q);

/**
 * Quaternion conjugate
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Conjugate of quaternion
 */
Vec4 quatconj(const Vec4 &q);

/**
 * Quaternion multiply
 *
 * Page 3. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param p Quaternion in (x, y, z, w)
 * @param q Quaternion in (x, y, z, w)
 * @returns Product of quaternions p and q
 */
Vec4 quatmul(const Vec4 &p, const Vec4 &q);

/**
 * Quaternion to Rotation Matrix
 *
 * Page 9. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Rotation matrix
 */
Mat3 quat2rot(const Vec4 &q);

/**
 * Quaternion left-compound
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Quaternion left-compound
 */
Mat4 quatlcomp(const Vec4 &q);

/**
 * Quaternion right-compound
 *
 * Page 4. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
 * Kalman filter for 3D attitude estimation." University of Minnesota,
 * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Quaternion right-compound
 */
Mat4 quatrcomp(const Vec4 &q);

/**
 * Quaternion to Euler-angles
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Euler-angles
 */
Vec3 quat2euler(const Vec4 &q);

/**
 * Euler-angles to Quaternion
 *
 * @param rpy Euler-angles in (roll, pitch, yaw)
 * @returns Quaternion
 */
Vec4 euler2quat(const Vec3 &rpy);

/**
 * Quaternion to Rotation Matrix
 *
 * @param q Quaternion in (x, y, z, w)
 * @returns Rotation matrix
 */
Mat3 C(const Vec4 &q);

/**
 * Omega function
 *
 * @param w Angular velocity
 * @returns Differential form of an angular velocity
 */
Mat4 Omega(const Vec3 &w);

/** @} group quaternion */
} // namespace gvio
#endif // GVIO_QUATERNION_JPL_HPP
