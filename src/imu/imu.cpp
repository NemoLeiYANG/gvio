#include "gvio/imu/imu.hpp"

namespace gvio {
/**
 * @addtogroup imu
 * @{
 */

// void MPU6050::accelerometerCalcAngle() {
//   float x;
//   float y;
//   float z;
//
//   // setup
//   x = this->accel.x;
//   y = this->accel.y;
//   z = this->accel.z;
//
//   // calculate pitch and roll
//   this->accel.pitch = (atan(x / sqrt(pow(y, 2) + pow(z, 2)))) * 180 / M_PI;
//   this->accel.roll = (atan(y / sqrt(pow(x, 2) + pow(z, 2)))) * 180 / M_PI;
// }
//
// void MPU6050::gyroscopeCalcAngle(float dt) {
//   this->gyro.roll = (this->gyro.x * dt) + this->roll;
//   this->gyro.pitch = (this->gyro.y * dt) + this->pitch;
// }

// Vec2 complementary_filter(const Gyroscope gyro, const Accelerometer accel,
// const double dt) {
//   // complimentary filter
//   this->accelerometerCalcAngle();
//   this->pitch = (0.98 * this->gyro.pitch) + (0.02 * this->accel.pitch);
//   this->roll = (0.98 * this->gyro.roll) + (0.02 * this->accel.roll);
//   this->gyroscopeCalcAngle(dt);
// }

// int MPU6050::calibrate() {
//   // Let it stablize for a while first
//   LOG_INFO("calibrating mpu6050");
//   for (int16_t i = 0; i < 50; i++) {
//     this->getData();
//   }
//
//   // Calculate offset
//   for (int i = 0; i < 50; i++) {
//     this->getData();
//
//     this->accel.offset_x += this->accel.raw_x;
//     this->accel.offset_y += this->accel.raw_y;
//     this->accel.offset_z += this->accel.raw_z;
//
//     this->accel.offset_x = this->accel.offset_x / 2.0;
//     this->accel.offset_y = this->accel.offset_y / 2.0;
//     this->accel.offset_z = this->accel.offset_z / 2.0;
//
//     this->gyro.offset_x += this->gyro.raw_x;
//     this->gyro.offset_y += this->gyro.raw_y;
//     this->gyro.offset_z += this->gyro.raw_z;
//
//     this->gyro.offset_x = this->gyro.offset_x / 2.0;
//     this->gyro.offset_y = this->gyro.offset_y / 2.0;
//     this->gyro.offset_z = this->gyro.offset_z / 2.0;
//   }
//
//   return 0;
// }

void IMU::recordHeader(FILE *output_file) {
  fprintf(output_file, "gyro.x,");
  fprintf(output_file, "gyro.y,");
  fprintf(output_file, "gyro.z,");

  fprintf(output_file, "accel.x,");
  fprintf(output_file, "accel.y,");
  fprintf(output_file, "accel.z\n");
}

void IMU::recordData(FILE *output_file) {
  fprintf(output_file, "%f,", this->gyro.x);
  fprintf(output_file, "%f,", this->gyro.y);
  fprintf(output_file, "%f,", this->gyro.z);

  fprintf(output_file, "%f,", this->accel.x);
  fprintf(output_file, "%f,", this->accel.y);
  fprintf(output_file, "%f\n", this->accel.z);
}

int IMU::record(std::string output_path, int nb_samples) {
  // setup
  FILE *output_file = fopen(output_path.c_str(), "w");
  this->recordHeader(output_file);

  // record
  for (int i = 0; i < nb_samples; i++) {
    // get data
    int retval = this->getData();
    if (retval == -1) {
      LOG_ERROR("failed to obtain data from IMU!");
      return -1;
    }

    // record data
    this->recordData(output_file);
    if (retval == -1) {
      LOG_ERROR("failed to record IMU data!");
      return -1;
    }
  }

  // clean up
  fclose(output_file);

  return 0;
}

/** @} group imu */
} // namespace gvio
