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

  if (this->Lambda1 != nullptr)
    free(this->Lambda1);

  if (this->Lambda2 != nullptr)
    free(this->Lambda2);
}

int CalibParams::load(const std::string &config_file,
                      const std::string &joint_file) {
  // Parse config file
  ConfigParser parser;
  VecX tau_s, tau_d;
  Vec3 w1, w2;
  parser.addParam("tau_s", &tau_s);
  parser.addParam("tau_d", &tau_d);
  parser.addParam("w1", &w1);
  parser.addParam("w2", &w2);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
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
  this->tau_s = vec2array(tau_s);
  this->tau_d = vec2array(tau_d);
  this->w1 = vec2array(w1);
  this->w2 = vec2array(w2);
  this->Lambda1 = vec2array(joint_data.col(0));
  this->Lambda2 = vec2array(joint_data.col(1));

  return 0;
}

std::ostream &operator<<(std::ostream &os, const CalibParams &params) {
  os << "tau_s: " << array2str(params.tau_s, 6) << std::endl;
  os << "tau_d: " << array2str(params.tau_d, 6) << std::endl;
  os << "w1: " << array2str(params.w1, 3) << std::endl;
  os << "w2: " << array2str(params.w2, 3) << std::endl;
  os << "Lamba 1 and 2: " << std::endl;
  for (int i = 0; i < params.nb_measurements; i++) {
    os << params.Lambda1[i] << " " << params.Lambda2[i] << std::endl;
  }

  return os;
}

} // namespace gvio
