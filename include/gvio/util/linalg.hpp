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
MatX zeros(const double rows, const double cols);

/**
 * Zeros square matrix
 */
MatX zeros(const double size);

/**
 * Identity-matrix
 */
MatX I(const double rows, const double cols);

/**
 * Identity square matrix
 */
MatX I(const double size);

/**
 * Ones-matrix
 */
MatX ones(const double rows, const double cols);

/**
 * Ones square matrix
 */
MatX ones(const double size);

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
