#include "gvio/util/linalg.hpp"

namespace gvio {

MatX zeros(const double rows, const double cols) {
  return MatX::Zero(rows, cols);
}

MatX zeros(const double size) { return MatX::Zero(size, size); }

MatX I(const double rows, const double cols) {
  return MatX::Identity(rows, cols);
}

MatX I(const double size) { return MatX::Identity(size, size); }

MatX ones(const double rows, const double cols) {
  return 1.0 * MatX::Identity(rows, cols);
}

MatX ones(const double size) { return 1.0 * MatX::Identity(size, size); }

Mat3 skew(const Vec3 &w) {
  Mat3 S;
  // clang-format off
  S << 0.0, -w(2), w(1),
       w(2), 0.0, -w(0),
       -w(1), w(0), 0.0;
  // clang-format on
  return S;
}

Mat3 skewsq(const Vec3 &w) {
  Mat3 SS = (w * w.transpose()) - pow(w.norm(), 2) * I(3);
  return SS;
}

MatX enforce_psd(const MatX &A) {
  MatX A_psd;

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i == j) {
        A_psd(i, j) = std::fabs(A(i, j));
      } else {
        const double x = 0.5 * (A(i, j) + A(j, i));
        A_psd(i, j) = x;
        A_psd(j, i) = x;
      }
    }
  }

  return A_psd;
}

} // namepsace gvio
