#include "gvio/munit.h"
#include "gvio/util/linalg.hpp"

namespace gvio {

int test_zeros() {
  MatX A = zeros(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      MU_CHECK_FLOAT(0.0, A(i, j));
    }
  }

  return 0;
}

int test_I() {
  MatX A = I(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i != j) {
        MU_CHECK_FLOAT(0.0, A(i, j));
      } else {
        MU_CHECK_FLOAT(1.0, A(i, j));
      }
    }
  }

  return 0;
}

int test_ones() {
  MatX A = ones(2, 2);

  MU_CHECK_EQ(A.rows(), 2);
  MU_CHECK_EQ(A.cols(), 2);

  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (i != j) {
        MU_CHECK_FLOAT(0.0, A(i, j));
      } else {
        MU_CHECK_FLOAT(1.0, A(i, j));
      }
    }
  }

  return 0;
}

int test_hstack() {
  Mat2 A;
  A.fill(1);

  Mat2 B;
  B.fill(2);

  MatX C = hstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(2, C.rows());
  MU_CHECK_EQ(4, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(B));

  return 0;
}

int test_vstack() {
  Mat2 A;
  A.fill(1);

  Mat2 B;
  B.fill(2);

  MatX C = vstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(4, C.rows());
  MU_CHECK_EQ(2, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(B));

  return 0;
}

int test_dstack() {
  Mat2 A;
  A.fill(1);

  Mat2 B;
  B.fill(2);

  MatX C = dstack(A, B);
  std::cout << C << std::endl;

  MU_CHECK_EQ(4, C.rows());
  MU_CHECK_EQ(4, C.cols());
  MU_CHECK(C.block(0, 0, 2, 2).isApprox(A));
  MU_CHECK(C.block(0, 2, 2, 2).isApprox(zeros(2, 2)));
  MU_CHECK(C.block(2, 2, 2, 2).isApprox(B));
  MU_CHECK(C.block(2, 0, 2, 2).isApprox(zeros(2, 2)));

  return 0;
}

int test_skew() {
  Vec3 v{1.0, 2.0, 3.0};
  Mat3 X = skew(v);

  Mat3 X_expected;
  // clang-format off
  X_expected << 0.0, -3.0, 2.0,
                3.0, 0.0, -1.0,
                -2.0, 1.0, 0.0;
  // clang-format on

  MU_CHECK(X_expected.isApprox(X));

  return 0;
}

int test_skewsq() { return 0; }

int test_enforce_psd() {
  Mat3 A;
  // clang-format off
  A << 0.0, -3.0, 2.0,
       3.0, 0.0, -1.0,
       -2.0, 1.0, 0.0;
  // clang-format on

  Mat3 B = enforce_psd(A);
  for (int i = 0; i < B.rows(); i++) {
    for (int j = 0; j < B.cols(); j++) {
      MU_CHECK(B(i, j) >= 0);
    }
  }

  return 0;
}

int test_nullspace() {
  Mat3 A;
  // clang-format off
  A << 1.0, 2.0, 3.0,
       1.0, 2.0, 3.0,
       1.0, 2.0, 3.0;
  // clang-format on

  MatX B = nullspace(A);
  std::cout << B << std::endl;
  std::cout << A * B << std::endl;

  MatX C = A * B;
  for (int i = 0; i < C.rows(); i++) {
    for (int j = 0; j < C.cols(); j++) {
      MU_CHECK_FLOAT(0.0, C(i, j));
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_I);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_hstack);
  MU_ADD_TEST(test_vstack);
  MU_ADD_TEST(test_dstack);
  MU_ADD_TEST(test_skew);
  MU_ADD_TEST(test_skewsq);
  MU_ADD_TEST(test_enforce_psd);
  MU_ADD_TEST(test_nullspace);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
