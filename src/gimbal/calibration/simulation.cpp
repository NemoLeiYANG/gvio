/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_SIMULATION_HPP
#define GVIO_GIMBAL_CALIBRATION_SIMULATION_HPP

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/calib_data.hpp"
#include "gvio/gimbal/calibration/calib_params.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class GimbalSimulation {
public:
  CalibData data;
  CalibParams params;

  GimbalSimulation();
  virtual ~GimbalSimulation();
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_SIMULATION_HPP
