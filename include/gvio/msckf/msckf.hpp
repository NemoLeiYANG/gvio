/**
 * @file
 * @defgroup msckf msckf
 */
#ifndef GVIO_MSCKF_MSCKF_HPP
#define GVIO_MSCKF_MSCKF_HPP

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"

#include "gvio/estimation/camera_state.hpp"
#include "gvio/estimation/imu_state.hpp"
#include "gvio/estimation/msckf.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

struct MSCKF {
  std::vector<CameraState> cam_states;
  IMUState imu_state;

  MSCKF() {}

  void augmentState();
  void getTrackCameraStates(const FeatureTrack &track);
  void P();
  void N();
  void H();
  void predictionUpdate();
  void calTrackResiduals();
  void measurementUpdate();
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_MSCKF_HPP
