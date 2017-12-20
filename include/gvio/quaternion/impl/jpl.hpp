#include "gvio/util/util.hpp"

namespace gvio {

template <typename Derived, typename OtherDerived>
void skew(const Eigen::MatrixBase<Derived> &vector,
          Eigen::MatrixBase<OtherDerived> const &matrix_const) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(OtherDerived, 3, 3);

  typedef typename OtherDerived::Scalar Scalar;

  Eigen::MatrixBase<OtherDerived> &matrix =
      const_cast<Eigen::MatrixBase<OtherDerived> &>(matrix_const);

  Scalar zero = static_cast<Scalar>(0.0);

  // clang-format off
  matrix.derived() << zero, -vector[2], vector[1],
                      vector[2], zero, -vector[0],
                      -vector[1], vector[0], zero;
  // clang-format on
}

template <typename Derived1, typename Derived2, typename Derived3>
void signed_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                    const Eigen::MatrixBase<Derived2> &q2,
                    const Eigen::MatrixBase<Derived3> &product_const) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived3, 4);

  typedef typename Derived1::Scalar Scalar;

  // Construct left multiplication matrix.
  Eigen::Matrix<Scalar, 4, 4> matrix_L;
  Eigen::Matrix<Scalar, 3, 3> skew_q1;
  skew(q1.head(3), skew_q1);
  matrix_L.block(0, 0, 3, 3) =
      q1.derived()(3) * Eigen::Matrix3d::Identity().cast<Scalar>() - skew_q1;
  matrix_L.block(3, 0, 1, 3) = -q1.derived().head(3).transpose();
  matrix_L.block(0, 3, 4, 1) = q1.derived();

  Eigen::MatrixBase<Derived3> &product =
      const_cast<Eigen::MatrixBase<Derived3> &>(product_const);

  product.derived().resize(4, 1);
  product.derived() = matrix_L * q2;
  product.derived().normalize();
}

template <typename Derived1, typename Derived2, typename Derived3>
void positive_quatmul(const Eigen::MatrixBase<Derived1> &q1,
                      const Eigen::MatrixBase<Derived2> &q2,
                      const Eigen::MatrixBase<Derived3> &product_const) {
  signed_quatmul(q1, q2, product_const);

  // Make sure the scalar part is positive.
  typedef typename Derived3::Scalar Scalar;
  Eigen::MatrixBase<Derived3> &product =
      const_cast<Eigen::MatrixBase<Derived3> &>(product_const);
  if (product(3) < static_cast<Scalar>(0.)) {
    product = -product;
  }
}

} // namespace gvio
