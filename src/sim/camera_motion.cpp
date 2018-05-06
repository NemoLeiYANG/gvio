#include "gvio/sim/camera_motion.hpp"

namespace gvio {

CameraMotion::CameraMotion() {}

CameraMotion::CameraMotion(const std::vector<Vec3> &pos_points,
                           const std::vector<Vec3> &att_points,
                           const int max_steps)
    : pos_points{pos_points}, att_points{att_points}, max_steps{max_steps} {
  this->update();
  this->time_index = 0.0;
  this->dt = 1.0 / this->max_steps;
}

CameraMotion::~CameraMotion() {}

int CameraMotion::update() {
  assert(this->time_index > 1.0);

  // Calculate pos, vel and accel on the Bezier curve at t
  this->p_G = bezier(this->pos_points, this->time_index);
  this->v_G = bezier_derivative(this->pos_points, this->time_index, 1);
  this->a_G = bezier_derivative(this->pos_points, this->time_index, 2);

  // Calculate attitude
  this->rpy_G = bezier(this->att_points, this->time_index);
  this->w_G = bezier_derivative(this->att_points, this->time_index, 1);

  // Calculate IMU measurements
  const Mat3 R_BG = euler123ToRot(this->rpy_G);
  const Vec3 gravity{0.0, 0.0, 9.81};
  this->a_B = R_BG * (this->a_G + gravity);
  this->w_B = R_BG * this->w_G;

  // Update time
  this->time_index += this->dt;
  if (this->time_index > 1.0) {
    return 1;
  }

  return 0;
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
