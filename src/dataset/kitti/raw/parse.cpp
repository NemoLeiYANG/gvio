#include "gvio/dataset/kitti/raw/parse.hpp"

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

} // namespace gvio
