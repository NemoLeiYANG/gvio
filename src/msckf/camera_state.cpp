#include "gvio/msckf/camera_state.hpp"

namespace gvio {

void CameraState::correct(const VecX &dx) {
  // Split dx into its own components
  const Vec3 dtheta_CG = dx.segment(0, 3);
  const Vec3 dp_G = dx.segment(3, 3);

  // Time derivative of quaternion (small angle approx)
  const Vec4 dq_CG = quatsmallangle(dtheta_CG);

  // Correct camera state
  this->q_CG = quatnormalize(quatlcomp(dq_CG) * this->q_CG);
  this->p_G = this->p_G + dp_G;
}

void CameraState::setFrameID(const FrameID &frame_id) {
  this->frame_id = frame_id;
}

int save_camera_states(const CameraStates &states,
                       const std::string &output_path) {
  // Setup output file
  std::ofstream output_file(output_path);
  if (output_file.good() == false) {
    LOG_ERROR("Failed to open file for output [%s]", output_path.c_str());
    return -1;
  }

  // Output states
  for (auto state : states) {
    output_file << state.p_G(0) << ",";
    output_file << state.p_G(1) << ",";
    output_file << state.p_G(2) << ",";

    output_file << state.q_CG(0) << ",";
    output_file << state.q_CG(1) << ",";
    output_file << state.q_CG(2) << ",";
    output_file << state.q_CG(3) << std::endl;
  }

  return 0;
}

} // namespace gvio
