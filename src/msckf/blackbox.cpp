#include "gvio/msckf/blackbox.hpp"

namespace gvio {

BlackBox::BlackBox() {}

BlackBox::~BlackBox() {
  if (this->est_file.good()) {
    this->est_file.close();
  }
}

int BlackBox::configure(const std::string &output_path,
                        const std::string &base_name) {
  // Estimation file
  this->est_file.open(output_path + "/" + base_name + "_est.dat");
  if (this->est_file.good() == false) {
    LOG_ERROR("Failed to open estimate file for recording [%s]",
              output_path.c_str());
    return -1;
  }

  // Measurement file
  this->mea_file.open(output_path + "/" + base_name + "_mea.dat");
  if (this->mea_file.good() == false) {
    LOG_ERROR("Failed to open measurement file for recording [%s]",
              output_path.c_str());
    return -1;
  }

  // Ground truth file
  this->gnd_file.open(output_path + "/" + base_name + "_gnd.dat");
  if (this->gnd_file.good() == false) {
    LOG_ERROR("Failed to open ground truth file for recording [%s]",
              output_path.c_str());
    return -1;
  }

  // Write header
  // clang-format off
  const std::string est_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  this->est_file << est_header << std::endl;

  const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
  this->mea_file << mea_header << std::endl;

  const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  this->gnd_file << gnd_header << std::endl;
  // clang-format on

  return 0;
}

int BlackBox::record(const double time,
                     const MSCKF &msckf,
                     const Vec3 &measurement_a_B,
                     const Vec3 &measurement_w_B,
                     const Vec3 &ground_truth_p_G,
                     const Vec3 &ground_truth_v_G,
                     const Vec3 &ground_truth_rpy_G) {
  // Estimated
  // -- Time
  this->est_file << time << ",";
  // -- Position
  this->est_file << msckf.imu_state.p_G(0) << ",";
  this->est_file << msckf.imu_state.p_G(1) << ",";
  this->est_file << msckf.imu_state.p_G(2) << ",";
  // -- Velocity
  this->est_file << msckf.imu_state.v_G(0) << ",";
  this->est_file << msckf.imu_state.v_G(1) << ",";
  this->est_file << msckf.imu_state.v_G(2) << ",";
  // -- Roll, pitch and yaw
  Vec3 rpy_G = quat2euler(msckf.imu_state.q_IG);
  this->est_file << rpy_G(0) << ",";
  this->est_file << rpy_G(1) << ",";
  this->est_file << rpy_G(2) << std::endl;

  // Measurements
  // -- Time
  this->mea_file << time << ",";
  // -- Acceleration
  this->mea_file << measurement_a_B(0) << ",";
  this->mea_file << measurement_a_B(1) << ",";
  this->mea_file << measurement_a_B(2) << ",";
  // -- Angular velocity
  this->mea_file << measurement_w_B(0) << ",";
  this->mea_file << measurement_w_B(1) << ",";
  this->mea_file << measurement_w_B(2) << std::endl;

  // Ground truth
  // -- Time
  this->gnd_file << time << ",";
  // -- Position
  this->gnd_file << ground_truth_p_G(0) << ",";
  this->gnd_file << ground_truth_p_G(1) << ",";
  this->gnd_file << ground_truth_p_G(2) << ",";
  // -- Velocity
  this->gnd_file << ground_truth_v_G(0) << ",";
  this->gnd_file << ground_truth_v_G(1) << ",";
  this->gnd_file << ground_truth_v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->gnd_file << ground_truth_rpy_G(0) << ",";
  this->gnd_file << ground_truth_rpy_G(1) << ",";
  this->gnd_file << ground_truth_rpy_G(2) << std::endl;

  return 0;
}

} // namespace gvio