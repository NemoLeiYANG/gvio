/**
 * @file
 * @defgroup stats stats
 * @ingroup util
 */
#ifndef GVIO_UTILS_STATS_HPP
#define GVIO_UTILS_STATS_HPP

#include "gvio/util/math.hpp"

namespace gvio {
/**
 * @addtogroup stats
 * @{
 */

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

/** @} group stats */
} // namespace gvio
#endif
