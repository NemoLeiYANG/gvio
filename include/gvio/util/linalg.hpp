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
 */
MatX zeros(const int rows, const int cols);

/**
 * Zeros square matrix
 */
MatX zeros(const int size);

/**
 * Identity-matrix
 */
MatX I(const int rows, const int cols);

/**
 * Identity square matrix
 */
MatX I(const int size);

/**
 * Ones-matrix
 */
MatX ones(const int rows, const int cols);

/**
 * Ones square matrix
 */
MatX ones(const int size);

/**
 * Skew symmetric-matrix
 */
Mat3 skew(const Vec3 &w);

/**
 * Skew symmetric-matrix squared
 */
Mat3 skewsq(const Vec3 &w);

/**
 * Enforce Positive Semi-Definite
 */
MatX enforce_psd(const MatX &A);

/** @} group linalg */
} // namepsace gvio
#endif // GVIO_UTIL_LINALG_HPP
