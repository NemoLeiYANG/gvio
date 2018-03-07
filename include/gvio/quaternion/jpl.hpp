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
 * Skew
 *
 * @param vector Quaternion
 * @param matrix_const Skew matrix
 */
template <typename Derived, typename OtherDerived>
void skew(const Eigen::MatrixBase<Derived> &vector,
          Eigen::MatrixBase<OtherDerived> const &matrix_const);

/**
 * Signed quaternion product
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param product_const Quaternion product
 */
template <typename Derived1, typename Derived2, typename Derived3>
void signed_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                    const Eigen::MatrixBase<Derived2> &q2,
                    const Eigen::MatrixBase<Derived3> &product_const);

/**
 * Positive quaternion product
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param product_const Quaternion product
 */
template <typename Derived1, typename Derived2, typename Derived3>
void positive_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                      const Eigen::MatrixBase<Derived2> &q2,
                      const Eigen::MatrixBase<Derived3> &product_const);

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
 * Convert the vector part of a quaternion to a full quaternion.
 *
 * This function is useful to convert delta quaternion which is usually a 3x1
 * vector to a full quaternion.  For more details, check Section 3.2 Kalman
 * Filter Update of Equation (217) and (218) in "Indirect Kalman Filter for 3D
 * Attitude Estimation" by Trawny and Roumeliotis.
 *
 * @param dtheta Small angle vector to form full quaternion with
 * @returns Small angle Quaternion
 */
Vec4 quatsmallangle(const Vec3 &dtheta);

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
 * Rotation matrix to Quaternion
 */
Vec4 rot2quat(const Mat3 &rot);

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

/**
 * Quaternion Zeroth order integration
 *
 * Page 12-13 of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman
 * filter for 3D attitude estimation." University of Minnesota, Dept. of Comp.
 * Sci. & Eng., Tech. Rep 2 (2005): 2005.
 *
 * @param q Quaternion in (x, y, z, w)
 * @param w Angular velocity
 * @param dt Time difference (s)
 *
 * @returns Zeroth order integrated quaternion
 */
Vec4 quatzoi(const Vec4 &q, const Vec3 &w, const double dt);

/** @} group quaternion */
} // namespace gvio

#include "impl/jpl.hpp"

#endif // GVIO_QUATERNION_JPL_HPP
