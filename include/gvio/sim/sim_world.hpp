/**
 * @file
 * @defgroup sim sim
 */
#ifndef GVIO_SIM_SIM_WORLD_HPP
#define GVIO_SIM_SIM_WORLD_HPP

#include "gvio/util/util.hpp"
#include "gvio/sim/twowheel.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

struct feature_bounds {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
};

class SimWorld {
public:
  double t = 0.0;
  double dt = 0.0;
  double t_end = 0.0;
  TwoWheelRobot robot;

  SimWorld() {}
  virtual ~SimWorld() {}

  int configure(const double dt);
  MatX create3DFeatures(const struct feature_bounds &bounds,
                        const size_t nb_features);
  MatX create3DFeaturePerimeter(const Vec3 &origin,
                                const Vec3 &dimensions,
                                const size_t nb_features);
  int step();
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_SIM_WORLD_HPP
