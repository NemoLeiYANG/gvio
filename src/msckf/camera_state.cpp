#include "gvio/msckf/camera_state.hpp"

namespace gvio {

void CameraState::correct(const VecX &dx) {
  // Split dx into its own components
  const Vec3 dtheta_CG = dx.block(0, 0, 3, 1);
  const Vec3 dp_G = dx.block(3, 0, 3, 1);

  // Time derivative of quaternion (small angle approx)
  Vec4 dq_CG = zeros(4, 1);
  const double norm = 0.5 * dtheta_CG.transpose() * dtheta_CG;
  if (norm > 1.0) {
    dq_CG.block(0, 0, 3, 1) = dtheta_CG;
    dq_CG(3) = 1.0;
    dq_CG = dq_CG / sqrt(1.0 + norm);
  } else {
    dq_CG.block(0, 0, 3, 1) = dtheta_CG;
    dq_CG(3) = sqrt(1.0 - norm);
  }
  dq_CG = quatnormalize(dq_CG);

  // Correct camera state
  this->q_CG = quatlcomp(dq_CG) * this->q_CG;
  this->p_G = this->p_G + dp_G;
}

void CameraState::setFrameID(const FrameID &frame_id) {
  this->frame_id = frame_id;
}

} // namespace gvio
