#include "gvio/gimbal/calibration/calib_params.hpp"

namespace gvio {

CalibParams::CalibParams() {}

CalibParams::~CalibParams() {
  if (this->tau_s != nullptr)
    free(this->tau_s);

  if (this->tau_d != nullptr)
    free(this->tau_d);

  if (this->w1 != nullptr)
    free(this->w1);

  if (this->w2 != nullptr)
    free(this->w2);

  if (this->theta1_offset != nullptr)
    free(this->theta1_offset);

  if (this->theta2_offset != nullptr)
    free(this->theta2_offset);

  if (this->Lambda1 != nullptr)
    free(this->Lambda1);

  if (this->Lambda2 != nullptr)
    free(this->Lambda2);
}

int CalibParams::load(const std::string &camchain_file,
                      const std::string &joint_file) {
  // Parse camchain file
  if (this->camchain.load(3, camchain_file) != 0) {
    LOG_ERROR("Failed to load camchain file [%s]!", camchain_file.c_str());
    return -1;
  }

  // Load joint angles file
  MatX joint_data;
  if (csv2mat(joint_file, false, joint_data) != 0) {
    LOG_ERROR("Failed to load joint angles file [%s]!", joint_file.c_str());
    return -1;
  }
  this->nb_measurements = joint_data.rows();

  // Initialize optimization params
  this->tau_s = vec2array(this->camchain.tau_s);
  this->tau_d = vec2array(this->camchain.tau_d);
  this->w1 = vec2array(this->camchain.w1);
  this->w2 = vec2array(this->camchain.w2);

  this->theta1_offset = (double *) malloc(sizeof(double) * 1);
  this->theta2_offset = (double *) malloc(sizeof(double) * 1);
  *this->theta1_offset = this->camchain.theta1_offset;
  *this->theta2_offset = this->camchain.theta2_offset;

  this->Lambda1 = vec2array(joint_data.col(0));
  this->Lambda2 = vec2array(joint_data.col(1));

  return 0;
}

std::ostream &operator<<(std::ostream &os, const CalibParams &params) {
  os << "tau_s: " << array2str(params.tau_s, 6) << std::endl;
  os << "tau_d: " << array2str(params.tau_d, 6) << std::endl;
  os << "w1: " << array2str(params.w1, 3) << std::endl;
  os << "w2: " << array2str(params.w2, 3) << std::endl;
  // os << "Lamba 1 and 2: " << std::endl;
  // for (int i = 0; i < params.nb_measurements; i++) {
  //   os << params.Lambda1[i] << " " << params.Lambda2[i] << std::endl;
  // }

  return os;
}

} // namespace gvio
