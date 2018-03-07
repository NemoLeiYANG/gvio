#include "gvio/quaternion/jpl.hpp"

namespace gvio {

double quatnorm(const Vec4 &q) {
  const double sum = pow(q(0), 2) + pow(q(1), 2) + pow(q(2), 2) + pow(q(3), 2);
  const double norm = sqrt(sum);
  return norm;
}

Vec4 quatnormalize(const Vec4 &q) {
  const double norm = quatnorm(q);
  return Vec4{q(0) / norm, q(1) / norm, q(2) / norm, q(3) / norm};
}

Vec4 quatconj(const Vec4 &q) { return Vec4{-q(0), -q(1), -q(2), q(3)}; }

Vec4 quatmul(const Vec4 &p, const Vec4 &q) {
  return Vec4{q(3) * p(0) + q(2) * p(1) - q(1) * p(2) + q(0) * p(3),
              -q(2) * p(0) + q(3) * p(1) + q(0) * p(2) + q(1) * p(3),
              q(1) * p(0) - q(0) * p(1) + q(3) * p(2) + q(2) * p(3),
              -q(0) * p(0) - q(1) * p(1) - q(2) * p(2) + q(3) * p(3)};
}

Mat3 quat2rot(const Vec4 &q) {
  const double q1 = q(0);
  const double q2 = q(1);
  const double q3 = q(2);
  const double q4 = q(3);

  const double R11 = 1.0 - 2.0 * pow(q2, 2.0) - 2.0 * pow(q3, 2.0);
  const double R12 = 2.0 * (q1 * q2 + q3 * q4);
  const double R13 = 2.0 * (q1 * q3 - q2 * q4);

  const double R21 = 2.0 * (q1 * q2 - q3 * q4);
  const double R22 = 1.0 - 2.0 * pow(q1, 2.0) - 2.0 * pow(q3, 2.0);
  const double R23 = 2.0 * (q2 * q3 + q1 * q4);

  const double R31 = 2.0 * (q1 * q3 + q2 * q4);
  const double R32 = 2.0 * (q2 * q3 - q1 * q4);
  const double R33 = 1.0 - 2.0 * pow(q1, 2.0) - 2.0 * pow(q2, 2.0);

  Mat3 R;
  // clang-format off
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on
  return R;
}

Mat4 quatlcomp(const Vec4 &q) {
  const double q1 = q(0);
  const double q2 = q(1);
  const double q3 = q(2);
  const double q4 = q(3);

  const Vec3 vector{q1, q2, q3};
  const double scalar = q4;

  Mat4 A;
  A.block(0, 0, 3, 3) = scalar * I(3) - skew(vector);
  A.block(0, 3, 3, 1) = vector;
  A.block(3, 0, 1, 3) = -vector.transpose();
  A(3, 3) = scalar;

  return A;
}

Mat4 quatrcomp(const Vec4 &q) {
  const double q1 = q(0);
  const double q2 = q(1);
  const double q3 = q(2);
  const double q4 = q(3);

  const Vec3 vector{q1, q2, q3};
  const double scalar = q4;

  Mat4 A;
  A.block(0, 0, 3, 3) = scalar * MatX::Identity(3, 3) + skew(vector);
  A.block(0, 3, 3, 1) = vector;
  A.block(3, 0, 1, 3) = -vector.transpose();
  A(3, 3) = scalar;

  return A;
}

Vec4 quatsmallangle(const Vec3 &dtheta) {
  const Vec3 dq = dtheta / 2.0;
  const double dq_square_norm = dq.squaredNorm();

  Vec4 q;
  if (dq_square_norm <= 1.0) {
    q.head<3>() = dq;
    q(3) = std::sqrt(1.0 - dq_square_norm);
  } else {
    q.head<3>() = dq;
    q(3) = 1;
    q = q / std::sqrt(1.0 + dq_square_norm);
  }

  return q;
}

Vec3 quat2euler(const Vec4 &q) {
  const double x = q(0);
  const double y = q(1);
  const double z = q(2);
  const double w = q(3);

  const double ysqr = y * y;

  const double t0 = 2.0 * (w * x + y * z);
  const double t1 = 1.0 - 2.0 * (x * x + ysqr);
  const double X = atan2(t0, t1);

  double t2 = 2.0 * (w * y - z * x);
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;
  const double Y = asin(t2);

  const double t3 = 2.0 * (w * z + x * y);
  const double t4 = 1.0 - 2.0 * (ysqr + z * z);
  const double Z = atan2(t3, t4);

  return Vec3{X, Y, Z};
}

Vec4 euler2quat(const Vec3 &rpy) {
  const double roll = rpy(0);
  const double pitch = rpy(1);
  const double yaw = rpy(2);

  const double cy = cos(yaw * 0.5);
  const double sy = sin(yaw * 0.5);
  const double cr = cos(roll * 0.5);
  const double sr = sin(roll * 0.5);
  const double cp = cos(pitch * 0.5);
  const double sp = sin(pitch * 0.5);

  const Vec4 q{cy * sr * cp - sy * cr * sp,
               cy * cr * sp + sy * sr * cp,
               sy * cr * cp - cy * sr * sp,
               cy * cr * cp + sy * sr * sp};

  return quatnormalize(q);
}

Vec4 rot2quat(const Mat3 &R) {
  Vec4 q{0.0, 0.0, 0.0, 0.0};

  double T = R.trace();
  if ((R(0, 0) > T) && (R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
    q(0) = sqrt((1.0 + (2 * R(0, 0)) - T) / 4.0);
    q(1) = (1.0 / (4.0 * q(0))) * (R(0, 1) + R(1, 0));
    q(2) = (1.0 / (4.0 * q(0))) * (R(0, 2) + R(2, 0));
    q(3) = (1.0 / (4.0 * q(0))) * (R(1, 2) - R(2, 1));
  } else if ((R(1, 1) > T) && (R(1, 1) > R(0, 0)) && (R(1, 1) > R(2, 2))) {
    q(1) = sqrt((1.0 + (2 * R(1, 1)) - T) / 4.0);
    q(0) = (1.0 / (4.0 * q(1))) * (R(0, 1) + R(1, 0));
    q(2) = (1.0 / (4.0 * q(1))) * (R(1, 2) + R(2, 1));
    q(3) = (1.0 / (4.0 * q(1))) * (R(2, 0) - R(0, 2));
  } else if ((R(2, 2) > T) && (R(2, 2) > R(0, 0)) && (R(2, 2) > R(1, 1))) {
    q(2) = sqrt((1.0 + (2 * R(2, 2)) - T) / 4.0);
    q(0) = (1.0 / (4.0 * q(2))) * (R(0, 2) + R(2, 0));
    q(1) = (1.0 / (4.0 * q(2))) * (R(1, 2) + R(2, 1));
    q(3) = (1.0 / (4.0 * q(2))) * (R(0, 1) - R(1, 0));
  } else {
    q(3) = sqrt((1.0 + T) / 4.0);
    q(0) = (1.0 / (4.0 * q(3))) * (R(1, 2) - R(2, 1));
    q(1) = (1.0 / (4.0 * q(3))) * (R(2, 0) - R(0, 2));
    q(2) = (1.0 / (4.0 * q(3))) * (R(0, 1) - R(1, 0));
  }
  if (q(3) < 0.0) {
    q = -q;
  }
  q.normalize();

  return q;
}

Mat3 C(const Vec4 &q) {
  Mat4 A = quatrcomp(q).transpose() * quatlcomp(q);
  const Mat3 R = A.block(0, 0, 3, 3);
  return R;
}

Mat4 Omega(const Vec3 &w) {
  Mat4 Om;

  Om.block(0, 0, 3, 3) = -skew(w);
  Om.block(0, 3, 3, 1) = w;
  Om.block(3, 0, 1, 3) = -w.transpose();
  Om(3, 3) = 0.0;

  return Om;
}

Vec4 quatzoi(const Vec4 &q, const Vec3 &w, const double dt) {
  Vec4 dqdt;

  if (w.norm() > 1e-5) {
    dqdt = (cos(w.norm() * dt * 0.5) * I(4) +
            1 / w.norm() * sin(w.norm() * dt * 0.5) * Omega(w)) *
           q;
  } else {
    dqdt = I(4) + dt * 0.5 * Omega(w);
  }

  return dqdt;
}

} // namespace gvio
