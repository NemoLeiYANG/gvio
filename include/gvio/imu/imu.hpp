/**
 * @file
 * @defgroup imu imu
 */
#ifndef GVIO_IMU_IMU_HPP
#define GVIO_IMU_IMU_HPP

#include <stdio.h>
#include <stdint.h>
#include <string>

#include "gvio/util/util.hpp"
#include "gvio/driver/i2c.hpp"

namespace gvio {
/**
 * @addtogroup imu
 * @{
 */

struct Gyroscope {
  float sensitivity = 0.0f;

  int16_t raw_x = 0.0f;
  int16_t raw_y = 0.0f;
  int16_t raw_z = 0.0f;

  float offset_x = 0.0f;
  float offset_y = 0.0f;
  float offset_z = 0.0f;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float pitch = 0.0f;
  float roll = 0.0f;

  Gyroscope() {}
};

struct Accelerometer {
  float sensitivity = 0.0f;

  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  float offset_x = 0.0f;
  float offset_y = 0.0f;
  float offset_z = 0.0f;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float pitch = 0.0f;
  float roll = 0.0f;

  Accelerometer() {}
};

struct IMU {
  Gyroscope gyro;
  Accelerometer accel;
  I2C i2c;

  IMU() {}

  /**
   * Ping
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int ping();

  /**
   * Get data
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int getData();

  /**
   * Record header
   */
  void recordHeader(FILE *output_file);

  /**
   * Record data
   */
  void recordData(FILE *output_file);

  /**
   * Record
   */
  int record(std::string output_path, int nb_samples);
};

/** @} group imu */
} // namespace gvio
#endif
