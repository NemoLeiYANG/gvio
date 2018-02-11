#include "gvio/gimbal/gimbal.hpp"

namespace gvio {

int Gimbal::configure(const std::string &config_file) {
  // Parse config file
  ConfigParser parser;
  std::string device_path;
  parser.addParam("device_path", &device_path);
  parser.addParam("enable_tracking", &this->enable_tracking);
  parser.addParam("gimbal_limits.roll_min", &this->roll_limits[0]);
  parser.addParam("gimbal_limits.roll_max", &this->roll_limits[1]);
  parser.addParam("gimbal_limits.pitch_min", &this->pitch_limits[0]);
  parser.addParam("gimbal_limits.pitch_max", &this->pitch_limits[1]);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // SBGC
  this->sbgc = SBGC(device_path);
  if (this->sbgc.connect() != 0 || this->sbgc.on() != 0) {
    LOG_ERROR("Failed to connect or switch on SBGC!");
    return -2;
  }

  this->configured = true;
  return 0;
}

int Gimbal::on() { return this->sbgc.on(); }

int Gimbal::off() { return this->sbgc.off(); }

int Gimbal::trackTarget(const Vec3 &target_P) {
  // Pre-check
  if (this->enable_tracking == false) {
    return 0;
  }

  // Calculate roll pitch yaw setpoints
  // Note: setpoints are assuming Gimbal are in FLU
  // FLU: (x - forward, y - left, z - up)
  const double dist = target_P.norm();
  this->setpoints(0) = asin(target_P(1) / dist); // Roll setpoint
  this->setpoints(1) = asin(target_P(0) / dist); // Pitch setpoint
  this->setpoints(2) = 0.0; // Yaw setpoint - unsupported at the moment

  return 0;
}

int Gimbal::updateGimbalStates() {
  int retval = this->sbgc.getRealtimeData4();
  if (retval != 0) {
    LOG_ERROR("Failed to get data from SBGC!");
    return -1;
  }

  // retval = this->sbgc.getAnglesExt();
  // if (retval != 0) {
  //   return -1;
  // }

  // Convert from G's to m/s^2
  const double k_gravity = 9.80665;
  this->imu_accel(0) = this->sbgc.data.accel(0) * k_gravity;
  this->imu_accel(1) = this->sbgc.data.accel(1) * k_gravity;
  this->imu_accel(2) = this->sbgc.data.accel(2) * k_gravity;

  this->imu_gyro(0) = deg2rad(this->sbgc.data.gyro(0));
  this->imu_gyro(1) = deg2rad(this->sbgc.data.gyro(1));
  this->imu_gyro(2) = deg2rad(this->sbgc.data.gyro(2));

  this->camera_angles(0) = deg2rad(this->sbgc.data.camera_angles(0));
  this->camera_angles(1) = deg2rad(this->sbgc.data.camera_angles(1));
  this->camera_angles(2) = deg2rad(this->sbgc.data.camera_angles(2));

  this->frame_angles(0) = deg2rad(this->sbgc.data.frame_angles(0));
  this->frame_angles(1) = deg2rad(this->sbgc.data.frame_angles(1));
  this->frame_angles(2) = deg2rad(this->sbgc.data.frame_angles(2));

  this->encoder_angles(0) = deg2rad(this->sbgc.data.encoder_angles(0));
  this->encoder_angles(1) = deg2rad(this->sbgc.data.encoder_angles(1));
  this->encoder_angles(2) = deg2rad(this->sbgc.data.encoder_angles(2));

  return 0;
}

int Gimbal::setAngle(const double roll, const double pitch) {
  this->setpoints(0) = roll * 180 / M_PI;
  this->setpoints(1) = pitch * 180 / M_PI;
  this->setpoints(2) = 0.0 * 180 / M_PI;

  return this->sbgc.setAngle(this->setpoints(0),
                             this->setpoints(1),
                             this->setpoints(2));
}

void Gimbal::printSetpoints() {
  std::cout << "roll setpoint: " << this->setpoints(0) << "\t";
  std::cout << "pitch setpoint: " << this->setpoints(1) << "\t";
  std::cout << "target: [";
  std::cout << "x: " << this->target_P(0) << "\t";
  std::cout << "y: " << this->target_P(1) << "\t";
  std::cout << "z: " << this->target_P(2);
  std::cout << "]" << std::endl;
}

} // namespace gvio
