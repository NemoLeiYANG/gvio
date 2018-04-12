/**
 * @file
 * @ingroup calibration
 */
#ifndef GVIO_CALIBRATION_SIMULATION_HPP
#define GVIO_CALIBRATION_SIMULATION_HPP

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/calibration/calib_data.hpp"
#include "gvio/calibration/calib_params.hpp"

namespace gvio {
/**
 * @addtogroup calibration
 * @{
 */

class GimbalSimulation {
public:
  CalibData data;
  CalibParams params;

  GimbalSimulation();
  virtual ~GimbalSimulation();
};

/** @} group calibration */
} // namespace gvio
#endif // GVIO_CALIBRATION_SIMULATION_HPP
