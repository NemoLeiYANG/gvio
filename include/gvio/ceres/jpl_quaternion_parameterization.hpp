/**
 * @file
 * @ingroup ceres
 */
#ifndef GVIO_CERES_JPL_QUATERNION_HPP
#define GVIO_CERES_JPL_QUATERNION_HPP

#include <ceres/ceres.h>

#include "gvio/quaternion/jpl.hpp"

namespace gvio {
/**
 * @addtogroup ceres
 * @{
 */

/**
 * JPL quaternion parameterization
 */
class JPLQuaternionParameterization : public ceres::LocalParameterization {
public:
  virtual ~JPLQuaternionParameterization() {}
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};

/** @} group ceres */
} // namespace gvio
#endif // GVIO_CERES_JPL_QUATERNION_HPP
