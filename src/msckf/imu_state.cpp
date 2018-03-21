#include "gvio/msckf/imu_state.hpp"

namespace gvio {

IMUState::IMUState() {
  Vec3 w_hat{0.0, 0.0, 0.0};
  Vec4 q_hat{0.0, 0.0, 0.0, 1.0};
  Vec3 a_hat{0.0, 0.0, 0.0};
  Vec3 w_G{0.0, 0.0, 0.0};
  // this->Phi = I(this->size);
}

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

  Vec3 w_hat{0.0, 0.0, 0.0};
  Vec4 q_hat{0.0, 0.0, 0.0, 1.0};
  Vec3 a_hat{0.0, 0.0, 0.0};
  Vec3 w_G{0.0, 0.0, 0.0};
  // this->Phi = I(this->size) + this->F(w_hat, q_hat, a_hat, w_G) * 0.1;
  // this->Phi = I(this->size);
}

MatX IMUState::F(const Vec3 &w_hat, const Vec4 &q_hat, const Vec3 &a_hat) {
  MatX F = zeros(15);
  // -- First row block --
  F.block(0, 0, 3, 3) = -skew(w_hat);
  F.block(0, 3, 3, 3) = -I(3);
  // -- Third Row block --
  F.block(6, 0, 3, 3) = -C(q_hat).transpose() * skew(a_hat);
  F.block(6, 9, 3, 3) = -C(q_hat).transpose();
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
  const Vec3 w_hat = w_m - this->b_g;

  // Build the transition F and input G matrices
  const MatX F = this->F(w_hat, this->q_IG, a_hat);
  const MatX G = this->G(this->q_IG);

  // Propagate IMU states
  // clang-format off
  if (this->rk4) {
    // Quaternion zeroth order integration
    const Vec4 dq_dt = quatzoi(this->q_IG, w_hat, dt);
    const Vec4 dq_dt2 = quatzoi(this->q_IG, w_hat, dt);
    const Mat3 dR_dt_transpose = C(dq_dt).transpose();
    const Mat3 dR_dt2_transpose = C(dq_dt2).transpose();

    // 4th order Runge-Kutta
    // -- k1 = f(tn, yn)
    const Vec3 k1_v_dot = C(this->q_IG).transpose() * a_hat + this->g_G;
    const Vec3 k1_p_dot = this->v_G;
    // -- k2 = f(tn + dt / 2, yn + k1 * dt / 2)
    const Vec3 k1_v = this->v_G + k1_v_dot * dt / 2.0;
    const Vec3 k2_v_dot = dR_dt2_transpose * a_hat + this->g_G;
    const Vec3 k2_p_dot = k1_v;
    // -- k3 = f(tn + dt / 2, yn + k2 * dt / 2)
    const Vec3 k2_v = this->v_G + k2_v_dot * dt / 2;
    const Vec3 k3_v_dot = dR_dt2_transpose * a_hat + this->g_G;
    const Vec3 k3_p_dot = k2_v;
    // -- k4 = f(tn + dt, yn + k3)
    const Vec3 k3_v = this->v_G + k3_v_dot * dt;
    const Vec3 k4_v_dot = dR_dt_transpose * a_hat + this->g_G;
    const Vec3 k4_p_dot = k3_v;
    // -- yn + 1 = yn + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    this->q_IG = quatnormalize(dq_dt);
    this->v_G = this->v_G + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    this->p_G = this->p_G + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

  } else {
    // -- Orientation
    this->q_IG += 0.5 * Omega(w_hat) * this->q_IG * dt;
    this->q_IG = quatnormalize(this->q_IG);
    // -- Velocity
    this->v_G += (C(this->q_IG).transpose() * a_hat + this->g_G) * dt;
    // -- Position
    this->p_G += v_G * dt;

  }
  // clang-format on

  // Update covariance
  // clang-format off
  // -- Approximate matrix exponential to the 3rd order using the power series,
  //    which can be considered to be accurate enough assuming dt is within
  //    0.01s.
  const MatX F_dt = F * dt;
  const MatX F_dt_sq = F_dt * F_dt;
  const MatX F_dt_cube = F_dt_sq * F_dt;
  this->Phi = I(this->size) + F_dt + 0.5 * F_dt_sq + (1.0 / 6.0) * F_dt_cube;
  // -- Update
  this->P = Phi * this->P * Phi.transpose() + (G * this->Q * G.transpose()) * dt;
  this->P = enforce_psd(P);
  // clang-format on

  // TODO: Modify transition matrix according to OC-EKF
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
