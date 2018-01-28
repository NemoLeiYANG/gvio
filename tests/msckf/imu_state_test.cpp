#include "gvio/munit.hpp"
#include "gvio/msckf/imu_state.hpp"

namespace gvio {

int test_IMUState_constructor() {
  IMUState imu_state;

  MU_CHECK_EQ(15, imu_state.size);

  MU_CHECK(Vec4(0.0, 0.0, 0.0, 1.0).isApprox(imu_state.q_IG));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.b_g));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.v_G));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.b_a));
  MU_CHECK(zeros(3, 1).isApprox(imu_state.p_G));

  MU_CHECK(zeros(3, 1).isApprox(imu_state.w_G));
  MU_CHECK(Vec3(0.0, 0.0, -9.81).isApprox(imu_state.g_G));

  // MU_CHECK(zeros(1, 1).isApprox(imu_state.P));
  // MU_CHECK(zeros(1, 1).isApprox(imu_state.Q));
  // MU_CHECK(zeros(1, 1).isApprox(imu_state.Phi));

  return 0;
}

int test_IMUState_F() {
  IMUState imu_state;

  const Vec3 w_hat = Vec3{1.0, 2.0, 3.0};
  const Vec4 q_hat = Vec4{0.0, 0.0, 0.0, 1.0};
  const Vec3 a_hat = Vec3{1.0, 2.0, 3.0};
  const Vec3 w_G = Vec3{0.1, 0.1, 0.1};

  const MatX F = imu_state.F(w_hat, q_hat, a_hat, w_G);
  std::cout << F << std::endl;

  return 0;
}

int test_IMUState_G() {
  IMUState imu_state;

  const Vec4 q_hat = Vec4{0.0, 0.0, 0.0, 1.0};
  const MatX G = imu_state.G(q_hat);
  std::cout << G << std::endl;

  return 0;
}

int test_IMUState_update() {
  IMUState imu_state;

  const Vec3 a_m{0.0, 0.0, 0.0};
  const Vec3 w_m{0.0, 0.0, 0.0};
  const double dt = 0.1;

  imu_state.update(a_m, w_m, dt);
  MU_CHECK(imu_state.P.maxCoeff() < 1.0);

  return 0;
}

int test_IMUState_correct() {
  IMUState imu_state;

  const Vec3 dtheta_IG{0.0, 0.0, 0.0};
  const Vec3 db_g{0.0, 0.0, 0.0};
  const Vec3 dv_G{0.0, 0.0, 0.0};
  const Vec3 db_a{0.0, 0.0, 0.0};
  const Vec3 dp_G{0.0, 0.0, 0.0};

  VecX dx;
  dx.resize(15, 1);
  dx.block(0, 0, 3, 1) = dtheta_IG;
  dx.block(3, 0, 3, 1) = db_g;
  dx.block(6, 0, 3, 1) = dv_G;
  dx.block(9, 0, 3, 1) = db_a;
  dx.block(12, 0, 3, 1) = dp_G;

  imu_state.correct(dx);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_IMUState_constructor);
  MU_ADD_TEST(test_IMUState_F);
  MU_ADD_TEST(test_IMUState_G);
  MU_ADD_TEST(test_IMUState_update);
  MU_ADD_TEST(test_IMUState_correct);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
