#ifndef GVIO_UTILS_STATS_HPP
#define GVIO_UTILS_STATS_HPP

#include "gvio/util/math.hpp"

namespace gvio {

int linreg(std::vector<Eigen::Vector2d> pts, double *m, double *b, double *r);

} // namespace gvio
#endif
