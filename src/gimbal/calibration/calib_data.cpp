#include "gvio/gimbal/calibration/calib_data.hpp"

namespace gvio {

int CalibData::load(const std::string &data_dir) {
  // Load joint data
  const std::string joint_filepath = data_dir + "/joint.csv";
  if (csv2mat(joint_filepath, true, this->joint_data) != 0) {
    LOG_ERROR("Failed to load joint data [%s]!", joint_filepath.c_str());
    return -1;
  }

  // Load data
  this->nb_measurements = this->joint_data.rows();
  for (int i = 0; i < this->nb_measurements; i++) {
    const std::string set_path = data_dir + "/set" + std::to_string(i);
    std::cout << "Loading measurement set " << i << " ";
    std::cout << "[" << set_path << "]" << std::endl;

    // P_s_i
    MatX P_s_i;
    const std::string P_s_fp = set_path + "/P_s";
    if (csv2mat(P_s_fp, true, P_s_i) != 0) {
      LOG_ERROR("Failed to P_s data [%s]!", P_s_fp.c_str());
      return -1;
    }

    // P_d_i
    MatX P_d_i;
    const std::string P_d_fp = set_path + "/P_d";
    if (csv2mat(P_d_fp, true, P_d_i) != 0) {
      LOG_ERROR("Failed to P_d data [%s]!", P_d_fp.c_str());
      return -1;
    }

    // Q_s_i
    MatX Q_s_i;
    const std::string Q_s_fp = set_path + "/Q_s";
    if (csv2mat(Q_s_fp, true, Q_s_i) != 0) {
      LOG_ERROR("Failed to Q_s data [%s]!", Q_s_fp.c_str());
      return -1;
    }

    // Q_d_i
    MatX Q_d_i;
    const std::string Q_d_fp = set_path + "/Q_d";
    if (csv2mat(Q_d_fp, true, Q_d_i) != 0) {
      LOG_ERROR("Failed to Q_d data [%s]!", Q_d_fp.c_str());
      return -1;
    }

    // Store it
    this->P_s.emplace_back(P_s_i);
    this->P_d.emplace_back(P_d_i);
    this->Q_s.emplace_back(Q_s_i);
    this->Q_d.emplace_back(Q_d_i);
  }

  return 0;
}

} // namespace gvio
