#include "gvio/imu/mpu9250.hpp"

namespace gvio {

int MPU9250::configure() {
  // setup
  this->i2c = I2C();
  this->i2c.setSlave(MPU9250_ADDRESS);

  // this->last_updated = clock();

  return 0;
}

int MPU9250::ping() {
  char data;

  // print mpu9250 address
  this->i2c.setSlave(MPU9250_ADDRESS);
  this->i2c.readByte(MPU9250_WHO_AM_I, &data);
  printf("MPU9250 ADDRESS: 0x%02X\n", data);

  return data;
}

int MPU9250::setGyroScale(int8_t scale) {
  char data;
  uint8_t retval;

  // pre-check
  if (scale > 3 || scale < 0) {
    return -2;
  }

  // set gyro scale
  data = scale << 3;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.writeByte(MPU9250_GYRO_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU9250::getGyroScale() {
  char data;
  uint8_t retval;

  // get gyro scale
  data = 0x00;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.readByte(MPU9250_GYRO_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // get gyro scale bytes
  data = (data >> 3) & 0b00000011;

  return data;
}

int MPU9250::setAccelScale(int8_t scale) {
  char data;
  uint8_t retval;

  // pre-check
  if (scale > 3 || scale < 0) {
    return -2;
  }

  // set accel scale
  data = scale << 3;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.writeByte(MPU9250_ACCEL_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU9250::getAccelScale() {
  char data;
  uint8_t retval;

  // get accel scale
  data = 0x00;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.readByte(MPU9250_ACCEL_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // get accel scale bytes
  data = (data >> 3) & 0b00000011;

  return data;
}

int MPU9250::setAccelFchoice(int8_t fchoice) {
  char data;
  uint8_t retval;

  // set accel scale
  data = fchoice << 3;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.writeByte(MPU9250_ACCEL_CONFIG2, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU9250::getAccelFchoice() {
  char data;
  uint8_t retval;

  // get accel scale
  data = 0x00;
  this->i2c.setSlave(MPU9250_ADDRESS);
  retval = this->i2c.readByte(MPU9250_ACCEL_CONFIG2, &data);
  if (retval != 0) {
    return -1;
  }

  // get accel scale bytes
  data = (data >> 3) & 0b00000001;

  return data;
}

} // namespace gvio
