#include "gvio/util/linalg.hpp"

namespace gvio {

MatX zeros(const int rows, const int cols) { return MatX::Zero(rows, cols); }

MatX zeros(const int size) { return MatX::Zero(size, size); }

MatX I(const int rows, const int cols) { return MatX::Identity(rows, cols); }

MatX I(const int size) { return MatX::Identity(size, size); }

MatX ones(const int rows, const int cols) {
  MatX A{rows, cols};
  A.fill(1.0);
  return A;
}

MatX ones(const int size) { return ones(size, size); }

MatX hstack(const MatX &A, const MatX &B) {
  MatX C(A.rows(), A.cols() + B.cols());
  C << A, B;
  return C;
}

MatX vstack(const MatX &A, const MatX &B) {
  MatX C(A.rows() + B.rows(), A.cols());
  C << A, B;
  return C;
}

MatX dstack(const MatX &A, const MatX &B) {
  MatX C = zeros(A.rows() + B.rows(), A.cols() + B.cols());
  C.block(0, 0, A.rows(), A.cols()) = A;
  C.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
  return C;
}

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

  A_psd.resize(A.rows(), A.cols());

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

MatX nullspace(const MatX &A) {
  Eigen::FullPivLU<MatX> lu(A);
  MatX A_null_space = lu.kernel();
  return A_null_space;
}

} // namepsace gvio
