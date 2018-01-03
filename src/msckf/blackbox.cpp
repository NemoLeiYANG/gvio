#include "gvio/msckf/blackbox.hpp"

namespace gvio {

BlackBox::BlackBox() {
}

BlackBox::~BlackBox() {
  if (this->output_file.good()) {
    this->output_file.close();
  }
}

int BlackBox::configure(const std::string &output_path) {
  // Setup file
  this->output_file.open(output_path);
  if (this->output_file.good() == false) {
    LOG_ERROR("Failed to open file for recording [%s]", output_path.c_str());
    return -1;
  }

  // Write header
  const std::string header = "x,y,z,vx,vy,vz,r,p,y";
  this->output_file << header << std::endl;

  return 0;
}

int BlackBox::record(const MSCKF &msckf) {
  // Position
  this->output_file << msckf.imu_state.p_G(0) << ",";
  this->output_file << msckf.imu_state.p_G(1) << ",";
  this->output_file << msckf.imu_state.p_G(2) << ",";

  // Velocity
  this->output_file << msckf.imu_state.v_G(0) << ",";
  this->output_file << msckf.imu_state.v_G(1) << ",";
  this->output_file << msckf.imu_state.v_G(2) << ",";

  // Roll, pitch and yaw
  Vec3 rpy_G = quat2euler(msckf.imu_state.q_IG);
  this->output_file << rpy_G(0) << ",";
  this->output_file << rpy_G(1) << ",";
  this->output_file << rpy_G(2) << std::endl;

  return 0;
}

} // namespace gvio
