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

struct GyroData {
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

  GyroData() {}
};

struct AccelData {
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

  AccelData() {}
};

struct IMU {
  GyroData gyro;
  AccelData accel;
  I2C i2c;

  IMU() {}

  /**
   * Ping
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int ping() = 0;

  /**
   * Get data
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int getData() = 0;

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
