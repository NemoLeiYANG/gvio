#include "gvio/kitti/raw.hpp"

namespace gvio {

std::string parseString(const std::string &line) {
  return strip(line.substr(line.find(":") + 1, line.size() - 1));
}

double parseDouble(const std::string &line) {
  const std::string str_val = parseString(line);
  const double val = atof(str_val.c_str());
  return val;
}

std::vector<double> parseArray(const std::string &line) {
  // Setup
  std::string s = parseString(line);
  std::string delimiter = " ";

  // Extract tokens
  size_t pos = 0;
  std::string token;
  std::vector<double> values;

  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    values.push_back(std::stod(token));
    s.erase(0, pos + delimiter.length());
  }

  // Extract last token
  token = s.substr(0, pos);
  values.push_back(std::stod(token));

  return values;
}

Vec2 parseVec2(const std::string &line) {
  const std::vector<double> values = parseArray(line);
  return Vec2(values[0], values[1]);
}

Vec3 parseVec3(const std::string &line) {
  const std::vector<double> values = parseArray(line);
  return Vec3(values[0], values[1], values[2]);
}

VecX parseVecX(const std::string &line) {
  const std::vector<double> values = parseArray(line);

  // Form the vector
  VecX vec;
  vec.resize(values.size());
  for (size_t i = 0; i < values.size(); i++) {
    vec(i) = values[i];
  }

  return vec;
}

Mat3 parseMat3(const std::string &line) {
  const std::vector<double> values = parseArray(line);

  // Form the matrix
  Mat3 mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

Mat34 parseMat34(const std::string &line) {
  const std::vector<double> values = parseArray(line);

  // Form the matrix
  Mat34 mat;
  int index = 0;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 4; j++) {
      mat(i, j) = values[index];
      index++;
    }
  }

  return mat;
}

int CalibCamToCam::load(const std::string &file_path) {
  std::ifstream file(file_path.c_str());

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

int CalibIMUToCam::load(const std::string &file_path) {
  std::ifstream file(file_path.c_str());

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

int RawDataset::loadCalibrations() {
  const std::string base_dir = this->raw_dir + "/" + this->date;
  this->calib_cam_to_cam.load(base_dir + "/calib_cam_to_cam.txt");
  this->calib_imu_to_cam.load(base_dir + "/calib_imu_to_cam.txt");
  this->calib_velo_to_cam.load(base_dir + "/calib_velo_to_cam.txt");

  return 0;
}

int RawDataset::load() {
  // Pre-check
  const std::string dataset_path = this->raw_dir + "/" + this->date;
  if (dir_exists(dataset_path) == false) {
    LOG_ERROR("Raw dataset path not found! [%s]", this->raw_dir.c_str());
    return -1;
  }

  // Load calibrations
  if (this->loadCalibrations() != 0) {
    LOG_ERROR("Failed to load calibrations!");
    return -2;
  }

  return 0;
}

} // namespace gvio
