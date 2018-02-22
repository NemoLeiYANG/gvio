#include "gvio/imu/imu_base.hpp"

namespace gvio {

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

void IMUBase::calibrate() {
  // Let it stablize for a while first
  LOG_INFO("Calibrating IMU - DO NOT MOVE!");
  sleep(1);

  // Calculate offset
  double ax_sum, ay_sum, az_sum = 0.0;
  double gx_sum, gy_sum, gz_sum = 0.0;
  const double nb_samples = 10000;
  for (int i = 0; i < (int) nb_samples; i++) {
    this->getData();
    ax_sum += this->accel.x;
    ay_sum += this->accel.y;
    az_sum += this->accel.z;
    gx_sum += this->gyro.x;
    gy_sum += this->gyro.y;
    gz_sum += this->gyro.z;
  }

  // this->accel.offset_x = ax_sum / nb_samples;
  // this->accel.offset_y = ay_sum / nb_samples;
  // this->accel.offset_z = (az_sum / nb_samples) - 9.81;

  this->gyro.offset_x = gx_sum / nb_samples;
  this->gyro.offset_y = gy_sum / nb_samples;
  this->gyro.offset_z = gz_sum / nb_samples;

  std::cout << "accel offsets: ";
  std::cout << this->accel.offset_x << "\t";
  std::cout << this->accel.offset_y << "\t";
  std::cout << this->accel.offset_z << std::endl;
  std::cout << "gyro offsets: ";
  std::cout << this->gyro.offset_x << "\t";
  std::cout << this->gyro.offset_y << "\t";
  std::cout << this->gyro.offset_z << std::endl;

  LOG_INFO("Finished calibrating IMU!");
}

void IMUBase::recordHeader(FILE *output_file) {
  fprintf(output_file, "gyro.x,");
  fprintf(output_file, "gyro.y,");
  fprintf(output_file, "gyro.z,");

  fprintf(output_file, "accel.x,");
  fprintf(output_file, "accel.y,");
  fprintf(output_file, "accel.z\n");
}

void IMUBase::recordData(FILE *output_file) {
  fprintf(output_file, "%f,", this->gyro.x);
  fprintf(output_file, "%f,", this->gyro.y);
  fprintf(output_file, "%f,", this->gyro.z);

  fprintf(output_file, "%f,", this->accel.x);
  fprintf(output_file, "%f,", this->accel.y);
  fprintf(output_file, "%f\n", this->accel.z);
}

int IMUBase::record(std::string output_path, int nb_samples) {
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

} // namespace gvio
