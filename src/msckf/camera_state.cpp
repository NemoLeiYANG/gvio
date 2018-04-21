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

CameraStates get_track_camera_states(const CameraStates &cam_states,
                                     const FeatureTrack &track) {
  const size_t N = cam_states.size();
  assert(N != 0);

  // Calculate camera states where feature was observed
  const FrameID fstart = track.frame_start;
  const FrameID fend = track.frame_end;
  const FrameID cstart = fstart - cam_states[0].frame_id;
  const FrameID cend = N - (cam_states.back().frame_id - fend);

  // Copy camera states
  auto first = cam_states.begin() + cstart;
  auto last = cam_states.begin() + cend;
  CameraStates track_cam_states{first, last};
  assert(track_cam_states.front().frame_id == track.frame_start);
  assert(track_cam_states.back().frame_id == track.frame_end);

  return track_cam_states;
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

std::ostream &operator<<(std::ostream &os, const CameraState &state) {
  os << "camera_state.frame_id: " << state.frame_id << std::endl;
  os << "camera_state.p_G: " << state.p_G.transpose() << std::endl;
  os << "camera_state.q_CG: " << state.q_CG.transpose() << std::endl;
  return os;
}

} // namespace gvio
