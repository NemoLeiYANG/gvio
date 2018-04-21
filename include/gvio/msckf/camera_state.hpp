/**
 * @file
 * @ingroup msckf
 */
#ifndef GVIO_MSCKF_CAMERA_STATE_HPP
#define GVIO_MSCKF_CAMERA_STATE_HPP

#include <vector>

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"
#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Camera state
 */
class CameraState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int size = 6;            ///< Size of state vector
  FrameID frame_id = -1;                ///< Camera frame id
  Vec3 p_G = zeros(3, 1);               ///< Position in Global frame
  Vec4 q_CG = Vec4{0.0, 0.0, 0.0, 1.0}; ///< Orientation in Global frame

  CameraState() {}
  CameraState(const Vec3 &p_G, const Vec4 &q_CG) : p_G{p_G}, q_CG{q_CG} {}
  CameraState(const FrameID frame_id, const Vec3 &p_G, const Vec4 &q_CG)
      : frame_id{frame_id}, p_G{p_G}, q_CG{q_CG} {}

  /**
   * Correct camera state
   * @param dx Correction vector
   */
  void correct(const VecX &dx);

  /**
   * Set frame id
   * @param frame_id Frame ID
   */
  void setFrameID(const FrameID &frame_id);
};

/**
 * Camera states
 */
using CameraStates = std::vector<CameraState>;

/**
 * Get camera states the feature track was observed in
 *
 * @param cam_states Camera states
 * @param track Feature track
 * @returns Camera states where feature track was observed
 */
CameraStates get_track_camera_states(const CameraStates &cam_states,
                                     const FeatureTrack &track);

/**
 * Camera states to CSV file
 *
 * @param states Camera states
 * @param output_path Output path
 */
int save_camera_states(const CameraStates &states,
                       const std::string &output_path);

/**
  * CameraState to string
  */
std::ostream &operator<<(std::ostream &os, const CameraState &state);

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_CAMERA_STATE_HPP
