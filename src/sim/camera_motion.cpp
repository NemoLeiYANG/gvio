#include "gvio/sim/camera_motion.hpp"

namespace gvio {

CameraMotion::CameraMotion() {}

CameraMotion::CameraMotion(const std::vector<Vec3> &control_points,
                           const double max_time)
    : control_points{control_points}, max_time{max_time} {
  this->update(0.0);
}

CameraMotion::~CameraMotion() {}

void CameraMotion::update(const double time) {
  // Note: t is not time, its a Bezier curve parameter
  const double t = time / this->max_time;

  // Calculate pos, vel and accel on the Bezier curve at t
  this->p_G = bezier(this->control_points, t);
  this->v_G = bezier_derivative(this->control_points, t, 1);
  this->a_G = bezier_derivative(this->control_points, t, 2);

  // Calculate attitude
  // const Vec3 tangent = bezier_tangent(this->control_points, t);
  // const double roll = 0.0;
  // const double pitch = asin(tangent(2));
  // const double yaw = atan2(tangent(1), tangent(0));
  const double roll = 0.0;
  const double pitch = 0.0;
  const double yaw = 0.0;
  this->rpy_G << roll, pitch, yaw;

  // Calculate IMU measurements
  const Mat3 R_BG = euler321ToRot(this->rpy_G);
  this->a_B = R_BG * this->a_G;
  this->w_B << 0.0, 0.0, 0.0;

  // std::cout << this->a_G.transpose() << std::endl;
  // std::cout << this->a_B.transpose() << std::endl;
  // std::cout << std::endl;
}

std::ostream &operator<<(std::ostream &os, const CameraMotion &camera_motion) {
  os << "camera_motion:" << std::endl;
  os << "\t p_G: " << camera_motion.p_G.transpose() << std::endl;
  os << "\t v_G: " << camera_motion.v_G.transpose() << std::endl;
  os << "\t a_G: " << camera_motion.a_G.transpose() << std::endl;
  os << "\t rpy_G: " << camera_motion.rpy_G.transpose() << std::endl;
  os << "\t a_B: " << camera_motion.a_B.transpose() << std::endl;
  os << "\t w_B: " << camera_motion.w_B.transpose() << std::endl;
  return os;
}

} // namespace gvio
