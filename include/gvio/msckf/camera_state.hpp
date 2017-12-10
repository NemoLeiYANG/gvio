#ifndef GVIO_MSCKF_CAMERA_STATE_HPP
#define GVIO_MSCKF_CAMERA_STATE_HPP

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"

namespace gvio {

/**
 * Camera State
 */
struct CameraState {
  const int size = 6;      ///< Size of state vector
  int frame_id = -1;       ///< Camera frame id
  Vec3 p_G = zeros(3, 1);  ///< Position of camera in Global frame
  Vec4 q_CG = zeros(4, 1); ///< Orientation of camera in Global frame

  CameraState() {}

  /**
   * Correct camera state
   * @param dx Correction vector
   */
  void correct(const VecX &dx);

  /**
   * Set frame id
   * @param frame_id Frame ID
   */
  void setFrameID(const int frame_id);
};

} // namespace gvio
#endif // GVIO_MSCKF_CAMERA_STATE_HPP
