#include "gvio/msckf/imu_state.hpp"

namespace gvio {

IMUState::IMUState() {}

IMUState::IMUState(const IMUStateConfig &config) {
  // clang-format off
  // Set estimate covariance matrix
  VecX init_var = zeros(15, 1);
  init_var << config.q_init_var,
              config.bg_init_var,
              config.v_init_var,
              config.ba_init_var,
              config.p_init_var;
  this->P = init_var.asDiagonal();

  // Set noise covariance matrix
  VecX n_imu = zeros(12, 1);
  n_imu << config.w_var,
           config.dbg_var,
           config.a_var,
           config.dba_var;
  this->Q = n_imu.asDiagonal();
  // clang-format on
}

MatX IMUState::F(const Vec3 &w_hat,
                 const Vec4 &q_hat,
                 const Vec3 &a_hat,
                 const Vec3 &w_G) {
  MatX F = zeros(15);
  // -- First row block --
  F.block(0, 0, 3, 3) = -skew(w_hat);
  F.block(0, 3, 3, 3) = -I(3);
  // -- Third Row block --
  F.block(6, 0, 3, 3) = -C(q_hat).transpose() * skew(a_hat);
  F.block(6, 6, 3, 3) = -2.0 * skew(w_G);
  F.block(6, 9, 3, 3) = -C(q_hat).transpose();
  F.block(6, 12, 3, 3) = -skewsq(w_G);
  // -- Fifth Row block --
  F.block(12, 6, 3, 3) = I(3);

  return F;
}

MatX IMUState::G(const Vec4 &q_hat) {
  MatX G = zeros(15, 12);
  // -- First row block --
  G.block(0, 0, 3, 3) = -I(3);
  // -- Second row block --
  G.block(3, 3, 3, 3) = I(3);
  // -- Third row block --
  G.block(6, 6, 3, 3) = -C(q_hat).transpose();
  // -- Fourth row block --
  G.block(9, 9, 3, 3) = I(3);

  return G;
}

void IMUState::update(const Vec3 &a_m, const Vec3 &w_m, const double dt) {
  // Calculate new accel and gyro estimates
  const Vec3 a_hat = a_m - this->b_a;
  const Vec3 w_hat = w_m - this->b_g - C(this->q_IG) * this->w_G;
  // const Vec3 a_hat = a_m;
  // const Vec3 w_hat = w_m;

  // Build the transition F and input G matrices
  const MatX F = this->F(w_hat, this->q_IG, a_hat, this->w_G);
  const MatX G = this->G(this->q_IG);

  // Propagate IMU states
  // clang-format off
  // -- Orientation
  this->q_IG += 0.5 * Omega(w_hat) * q_IG * dt;
  this->q_IG = quatnormalize(this->q_IG);
  // -- Velocity
  this->v_G += (C(this->q_IG).transpose() * a_hat - 2 * skew(this->w_G) * this->v_G - skewsq(this->w_G) * this->p_G + this->g_G) * dt;
  // -- Position
  this->p_G += v_G * dt;
  // clang-format on

  // Update covariance
  // clang-format off
  this->Phi = I(this->size) + F * dt;
  this->P = Phi * this->P * Phi.transpose() + (G * this->Q * G.transpose()) * dt;
  this->P = enforce_psd(P);
  // clang-format on
}

void IMUState::correct(const VecX &dx) {
  // Split dx into its own components
  const Vec3 dtheta_IG = dx.segment(0, 3);
  const Vec3 db_g = dx.segment(3, 3);
  const Vec3 dv_G = dx.segment(6, 3);
  const Vec3 db_a = dx.segment(9, 3);
  const Vec3 dp_G = dx.segment(12, 3);

  // Time derivative of quaternion (small angle approx)
  const Vec4 dq_IG = quatsmallangle(dtheta_IG);

  // Correct IMU state
  this->q_IG = quatnormalize(quatlcomp(dq_IG) * this->q_IG);
  this->b_g = this->b_g + db_g;
  this->v_G = this->v_G + dv_G;
  this->b_a = this->b_a + db_a;
  this->p_G = this->p_G + dp_G;
}

} // namespace gvio
