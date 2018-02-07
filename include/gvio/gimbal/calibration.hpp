/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_HPP
#define GVIO_GIMBAL_CALIBRATION_HPP

#include <string>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class ExtrinsicsDataLoader {
public:
  ExtrinsicsDataLoader();
  virtual ~ExtrinsicsDataLoader();

  /**
   * Load data
   *
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &data_dir,
           const std::string &static_cam_dir,
           const std::string &gimbal_cam_dir);
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_HPP
