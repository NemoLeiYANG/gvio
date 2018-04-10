#include "gvio/sim/camera_motion.hpp"

namespace gvio {

CameraMotion::CameraMotion() {}

CameraMotion::CameraMotion(const std::vector<Vec3> &pos_points,
                           const std::vector<Vec3> &att_points,
                           const double max_time)
    : pos_points{pos_points}, att_points{att_points}, max_time{max_time} {
  this->update(0.0);
}

CameraMotion::~CameraMotion() {}

void CameraMotion::update(const double time) {
  // Note: t is not time, its a Bezier curve parameter
  const double t = time / this->max_time;

  // Calculate pos, vel and accel on the Bezier curve at t
  this->p_G = bezier(this->pos_points, t);
  this->v_G = bezier_derivative(this->pos_points, t, 1);
  this->a_G = bezier_derivative(this->pos_points, t, 2);

  // Calculate attitude
  this->rpy_G = bezier(this->att_points, t);
  this->w_G = bezier_derivative(this->att_points, t, 1);

  // Calculate IMU measurements
  const Mat3 R_BG = euler321ToRot(this->rpy_G);
  const Vec3 gravity{0.0, 0.0, 9.81};
  this->a_B = R_BG * (this->a_G + gravity);
  this->w_B = R_BG * this->w_G;
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
