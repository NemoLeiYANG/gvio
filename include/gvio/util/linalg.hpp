/**
 * @file
 * @defgroup linalg linalg
 * @ingroup util
 */
#ifndef GVIO_UTIL_LINALG_HPP
#define GVIO_UTIL_LINALG_HPP

#include "gvio/util/math.hpp"

namespace gvio {
/**
 * @addtogroup linalg
 * @{
 */

/**
 * Zeros-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Zeros matrix
 */
MatX zeros(const int rows, const int cols);

/**
 * Zeros square matrix
 *
 * @param size Square size of matrix
 * @returns Zeros matrix
 */
MatX zeros(const int size);

/**
 * Identity-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Identity matrix
 */
MatX I(const int rows, const int cols);

/**
 * Identity square matrix
 *
 * @param size Square size of matrix
 * @returns Identity square matrix
 */
MatX I(const int size);

/**
 * Ones-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Ones square matrix
 */
MatX ones(const int rows, const int cols);

/**
 * Ones square matrix
 *
 * @param size Square size of matrix
 * @returns Ones square matrix
 */
MatX ones(const int size);

/**
 * Horizontally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX hstack(const MatX &A, const MatX &B);

/**
 * Vertically stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX vstack(const MatX &A, const MatX &B);

/**
 * Diagonally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX dstack(const MatX &A, const MatX &B);

/**
 * Skew symmetric-matrix
 *
 * @param w Input vector
 * @returns Skew symmetric matrix
 */
Mat3 skew(const Vec3 &w);

/**
 * Skew symmetric-matrix squared
 *
 * @param w Input vector
 * @returns Skew symmetric matrix squared
 */
Mat3 skewsq(const Vec3 &w);

/**
 * Enforce Positive Semi-Definite
 *
 * @param A Input matrix
 * @returns Positive semi-definite matrix
 */
MatX enforce_psd(const MatX &A);

/**
 * Null-space of A
 *
 * @param A Input matrix
 * @returns Null space of A
 */
MatX nullspace(const MatX &A);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix R and translation vector t.
 *
 * @param R Rotation matrix
 * @param t Translation vector
 * @return T Transformation matrix
 */
Mat4 transformation_matrix(const Mat3 R, const Vec3 t);

/** @} group linalg */
} // namepsace gvio
#endif // GVIO_UTIL_LINALG_HPP
