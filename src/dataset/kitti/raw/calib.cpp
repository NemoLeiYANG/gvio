#include "gvio/dataset/kitti/raw/calib.hpp"

namespace gvio {

int CalibCamToCam::load(const std::string &file_path) {
  std::ifstream file(file_path.c_str());

  // Check file
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      this->calib_time = parseString(line);
      continue;

    } else if (token == "corner_dist") {
      this->corner_dist = parseDouble(line);
      continue;
    }

    // Parse rectification variables
    const double cam_id = std::stod(token.substr(token.length() - 1));
    const bool rect_var = (token.find("rect") != std::string::npos);
    if (rect_var) {
      if (token[0] == 'S') {
        this->S_rect[cam_id] = parseVec2(line);
      } else if (token[0] == 'R') {
        this->R_rect[cam_id] = parseMat3(line);
      } else if (token[0] == 'P') {
        this->P_rect[cam_id] = parseMat34(line);
      }

      continue;
    }

    // Parse calibration variables
    if (token[0] == 'S') {
      this->S[cam_id] = parseVec2(line);
    } else if (token[0] == 'K') {
      this->K[cam_id] = parseMat3(line);
    } else if (token[0] == 'D') {
      this->D[cam_id] = parseVecX(line);
    } else if (token[0] == 'R') {
      this->R[cam_id] = parseMat3(line);
    } else if (token[0] == 'T') {
      this->T[cam_id] = parseVec3(line);
    }
  }

  this->ok = true;
  return 0;
}

int CalibIMUToVelo::load(const std::string &file_path) {
  std::ifstream file(file_path.c_str());

  // Check file
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      this->calib_time = parseString(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      this->R = parseMat3(line);
    } else if (token[0] == 'T') {
      this->t = parseVec3(line);
    }
  }

  this->ok = true;
  return 0;
}

int CalibVeloToCam::load(const std::string &file_path) {
  std::ifstream file(file_path.c_str());

  // Check file
  if (file.good() != true) {
    return -1;
  }

  // Parse calibration file
  std::string line;
  while (std::getline(file, line)) {
    // Parse token
    std::istringstream iss(line);
    std::string token;
    iss >> token;
    token = strip_end(token, ":");

    // Parse calibration time and corner distance
    if (token == "calib_time") {
      this->calib_time = parseString(line);
      continue;
    }

    // Parse calibration variables
    if (token[0] == 'R') {
      this->R = parseMat3(line);
    } else if (token[0] == 'T') {
      this->t = parseVec3(line);
    } else if (token == "delta_f") {
      this->df = parseVec2(line);
    } else if (token == "delta_c") {
      this->dc = parseVec2(line);
    }
  }

  this->ok = true;
  return 0;
}

} // namespace gvio
