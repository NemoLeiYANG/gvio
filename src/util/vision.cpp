#include "gvio/util/vision.hpp"

namespace gvio {

bool is_equal(const cv::Mat &m1, const cv::Mat &m2) {
  cv::Mat diff;

  // pre-check
  if (m1.empty() && m2.empty()) {
    return true;
  }

  // check dimensions
  if (m1.cols != m2.cols) {
    return false;
  } else if (m1.rows != m2.rows) {
    return false;
  } else if (m1.dims != m2.dims) {
    return false;
  }

  // check matrix elements
  cv::compare(m1, m2, diff, cv::CMP_NE);

  return cv::countNonZero(diff) ? false : true;
}

void convert(const cv::Mat &x, MatX &y) {
  y.resize(x.rows, x.cols);

  for (int i = 0; i < x.rows; i++) {
    for (int j = 0; j < x.cols; j++) {
      y(i, j) = x.at<double>(i, j);
    }
  }
}

void convert(const MatX &x, cv::Mat &y) {
  y = cv::Mat(x.rows(), x.cols(), cv::DataType<double>::type);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      y.at<double>(i, j) = x(i, j);
    }
  }
}

MatX convert(const cv::Mat &x) {
  MatX y;
  convert(x, y);
  return y;
}

cv::Mat convert(const MatX &x) {
  cv::Mat y;
  convert(x, y);
  return y;
}

Vec3 homogeneous(const Vec2 &x) { return Vec3{x(0), x(1), 1.0}; }

Vec4 homogeneous(const Vec3 &x) { return Vec4{x(0), x(1), x(2), 1.0}; }

} // namespace gvio
