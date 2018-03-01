#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {

GimbalCalibResidual::GimbalCalibResidual() {}

GimbalCalibResidual::GimbalCalibResidual(const Vec3 &P_s,
                                         const Vec3 &P_d,
                                         const Vec2 &Q_s,
                                         const Vec2 &Q_d,
                                         const Mat3 &K_s,
                                         const Mat3 &K_d) {
  // Observed 3d point in static camera
  this->P_s[0] = P_s(0);
  this->P_s[1] = P_s(1);
  this->P_s[2] = P_s(2);

  // Observed 3d point in dynamic camera
  this->P_d[0] = P_d(0);
  this->P_d[1] = P_d(1);
  this->P_d[2] = P_d(2);

  // Observed pixel in static camera
  this->Q_s[0] = Q_s(0);
  this->Q_s[1] = Q_s(1);

  // Observed pixel in dynamic camera
  this->Q_d[0] = Q_d(0);
  this->Q_d[1] = Q_d(1);

  // Static camera intrinsics
  this->fx_s = K_s(0, 0);
  this->fy_s = K_s(1, 1);
  this->cx_s = K_s(0, 2);
  this->cy_s = K_s(1, 2);

  // Dynamic camera intrinsics
  this->fx_d = K_d(0, 0);
  this->fy_d = K_d(1, 1);
  this->cx_d = K_d(0, 2);
  this->cy_d = K_d(1, 2);
}

} // namespace gvio
