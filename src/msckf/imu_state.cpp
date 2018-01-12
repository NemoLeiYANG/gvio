#include "gvio/msckf/imu_state.hpp"

namespace gvio {

MatX IMUState::F(const Vec3 &w_hat,
                 const Vec4 &q_hat,
                 const Vec3 &a_hat,
                 const Vec3 &w_G) {
  MatX F = zeros(15);
  // -- First row --
  F.block(0, 0, 3, 3) = -skew(w_hat);
  F.block(0, 3, 3, 3) = -ones(3);
  // -- Third Row --
  F.block(6, 0, 3, 3) = -C(q_hat).transpose() * skew(a_hat);
  F.block(6, 6, 3, 3) = -2.0 * skew(w_G);
  F.block(6, 9, 3, 3) = -C(q_hat).transpose();
  F.block(6, 12, 3, 3) = -skewsq(w_G);
  // -- Fifth Row --
  F.block(12, 6, 3, 3) = ones(3);

  return F;
}

MatX IMUState::G(const Vec4 &q_hat) {
  MatX G = zeros(15, 12);
  // -- First row --
  G.block(0, 0, 3, 3) = -ones(3);
  // -- Second row --
  G.block(3, 3, 3, 3) = ones(3);
  // -- Third row --
  G.block(6, 6, 3, 3) = -C(q_hat).transpose();
  // -- Fourth row --
  G.block(9, 9, 3, 3) = ones(3);

  return G;
}

MatX IMUState::J(const Vec4 &cam_q_CI,
                 const Vec3 &cam_p_IC,
                 const Vec4 &q_hat_IG,
                 const int N) {
  const Mat3 C_CI = C(cam_q_CI);
  const Mat3 C_IG = C(q_hat_IG);

  MatX J = zeros(6, 15 + 6 * N);
  // -- First row --
  J.block(0, 0, 3, 3) = C_CI;
  // -- Second row --
  J.block(3, 0, 3, 3) = skew(C_IG.transpose() * cam_p_IC);
  J.block(3, 12, 3, 3) = I(3);

  return J;
}

void IMUState::update(const Vec3 &a_m, const Vec3 &w_m, const double dt) {
  // Calculate new accel and gyro estimates
  const Vec3 a_hat = a_m;
  const Vec3 w_hat = w_m;

  // Propagate IMU states
  // clang-format off
  // -- Orientation
  this->q_IG = this->q_IG + 0.5 * Omega(w_hat) * q_IG * dt;
  this->q_IG = quatnormalize(q_IG);
  // -- Gyro bias
  this->b_g = this->b_g + zeros(3, 1);
  // -- Velocity
  this->v_G = this->v_G + (C(this->q_IG).transpose() * a_hat - 2 * skew(this->w_G) * this->v_G - skewsq(this->w_G) * this->p_G + this->g_G) * dt;
  // -- Accel bias
  this->b_a = this->b_a + zeros(3, 1);
  // -- Position
  this->p_G = this->p_G + v_G * dt;
  // clang-format on

  // Build the jacobians F and G
  const MatX F = this->F(w_hat, this->q_IG, a_hat, w_G);
  const MatX G = this->G(this->q_IG);

  // Update covariance
  this->Phi = I(this->size) + F * dt;
  this->P = Phi * P * Phi.transpose() + (G * this->Q * G.transpose()) * dt;
  this->P = enforce_psd(P);
}

void IMUState::correct(const VecX &dx) {
  // Split dx into its own components
  const Vec3 dtheta_IG = dx.block(0, 0, 3, 1);
  const Vec3 db_g = dx.block(3, 0, 3, 1);
  const Vec3 dv_G = dx.block(6, 0, 3, 1);
  const Vec3 db_a = dx.block(9, 0, 3, 1);
  const Vec3 dp_G = dx.block(12, 0, 3, 1);

  // Time derivative of quaternion (small angle approx)
  Vec4 dq_IG = zeros(4, 1);
  const double norm = 0.5 * dtheta_IG.transpose() * dtheta_IG;
  if (norm > 1.0) {
    dq_IG.block(0, 0, 3, 1) = dtheta_IG;
    dq_IG(3) = 1.0;
    dq_IG = dq_IG / sqrt(1.0 + norm);
  } else {
    dq_IG.block(0, 0, 3, 1) = dtheta_IG;
    dq_IG(3) = sqrt(1.0 - norm);
  }
  dq_IG = quatnormalize(dq_IG);

  // // Correct IMU state
  this->q_IG = quatlcomp(dq_IG) * this->q_IG;
  this->b_g = this->b_g + db_g;
  this->v_G = this->v_G + dv_G;
  this->b_a = this->b_a + db_a;
  this->p_G = this->p_G + dp_G;
}

} // namespace gvio
