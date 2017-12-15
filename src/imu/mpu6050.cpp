#include "gvio/imu/mpu6050.hpp"

namespace gvio {

int MPU6050::configure() {
  int8_t retval;

  // setup
  this->i2c = I2C();
  this->i2c.setup();
  this->i2c.setSlave(MPU6050_ADDRESS);

  // set dplf
  this->setDPLF(6);
  retval = this->getDPLF();
  if (retval > 7 || retval < 0) {
    return -1;
  } else {
    this->dplf_config = retval;
    LOG_INFO("dplf config: %d", this->dplf_config);
  }

  // set power management register
  this->i2c.writeByte(MPU6050_RA_PWR_MGMT_1, 0x00);

  // get gyro range
  this->setGyroRange(0);
  retval = this->getGyroRange();
  if (retval == 0) {
    this->gyro.sensitivity = 131.0;
  } else if (retval == 1) {
    this->gyro.sensitivity = 65.5;
  } else if (retval == 2) {
    this->gyro.sensitivity = 32.8;
  } else if (retval == 3) {
    this->gyro.sensitivity = 16.4;
  } else {
    return -2;
  }

  // get accel range
  this->setAccelRange(0);
  retval = this->getAccelRange();
  if (retval == 0) {
    this->accel.sensitivity = 16384.0;
  } else if (retval == 1) {
    this->accel.sensitivity = 8192.0;
  } else if (retval == 2) {
    this->accel.sensitivity = 4096.0;
  } else if (retval == 3) {
    this->accel.sensitivity = 2048.0;
  } else {
    return -3;
  }

  // get sample rate
  this->sample_rate = this->getSampleRate();

  return 0;
}

int MPU6050::ping() {
  char buf;

  // print mpu6050 address
  this->i2c.setSlave(MPU6050_ADDRESS);
  this->i2c.readByte(MPU6050_RA_WHO_AM_I, &buf);
  printf("MPU6050 ADDRESS: 0x%02X\n", buf);

  return 0;
}

int MPU6050::getData() {
  // read this data
  char raw_data[14];
  memset(raw_data, '\0', 14);
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.readBytes(MPU6050_RA_ACCEL_XOUT_H, raw_data, 14);
  if (retval != 0) {
    return -1;
  }

  // accelerometer
  this->accel.raw_x = (raw_data[0] << 8) | (raw_data[1]);
  this->accel.raw_y = (raw_data[2] << 8) | (raw_data[3]);
  this->accel.raw_z = (raw_data[4] << 8) | (raw_data[5]);

  this->accel.raw_x -= this->accel.offset_x;
  this->accel.raw_y -= this->accel.offset_y;
  this->accel.raw_z -= this->accel.offset_z;

  this->accel.x = this->accel.raw_x / this->accel.sensitivity;
  this->accel.y = this->accel.raw_y / this->accel.sensitivity;
  this->accel.z = this->accel.raw_z / this->accel.sensitivity;

  // temperature
  const int8_t raw_temp = (raw_data[6] << 8) | (raw_data[7]);
  this->temperature = raw_temp / 340.0 + 36.53;

  // gyroscope
  this->gyro.raw_x = (raw_data[8] << 8) | (raw_data[9]);
  this->gyro.raw_y = (raw_data[10] << 8) | (raw_data[11]);
  this->gyro.raw_z = (raw_data[12] << 8) | (raw_data[13]);

  this->gyro.raw_x -= this->gyro.offset_x;
  this->gyro.raw_y -= this->gyro.offset_y;
  this->gyro.raw_z -= this->gyro.offset_z;

  this->gyro.x = this->gyro.raw_x / this->gyro.sensitivity;
  this->gyro.y = this->gyro.raw_y / this->gyro.sensitivity;
  this->gyro.z = this->gyro.raw_z / this->gyro.sensitivity;

  // set last_updated
  this->last_updated = clock();

  return 0;
}

int MPU6050::setDPLF(const int setting) {
  /*
      DPLF_CFG    Accelerometer
      ----------------------------------------
                  Bandwidth(Hz) | Delay(ms)
      0           260             0
      1           184             2.0
      2           94              3.0
      3           44              4.9
      4           21              8.5
      5           10              13.8
      6           5               19.0
      7           RESERVED        RESERVED


      DPLF_CFG    Gyroscope
      ----------------------------------------------
                  Bandwidth(Hz) | Delay(ms) | Fs(kHz)
      0           256             0.98        8
      1           188             1.9         1
      2           98              2.8         1
      3           42              4.8         1
      4           20              8.3         1
      5           10              13.4        1
      6           5               18.5        1
      7           RESERVED        RESERVED    8
  */

  // Check setting range
  if (setting > 7 || setting < 0) {
    return -2;
  }

  // Set DPLF
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.writeByte(MPU6050_RA_CONFIG, (char) setting);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU6050::getDPLF() {
  // get dplf config
  char data[1] = {0x00};
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.readBytes(MPU6050_RA_CONFIG, data, 1);
  if (retval != 0) {
    return -1;
  }

  LOG_INFO("GOT DPLF: %d", data[0]);
  data[0] = data[0] & 0b00000111;

  return data[0];
}

int MPU6050::setSampleRateDiv(const int div) {
  // Set sample rate divider
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.writeByte(MPU6050_RA_SMPLRT_DIV, div);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU6050::getSampleRateDiv() {
  // Get sample rate
  this->i2c.setSlave(MPU6050_ADDRESS);
  char data = 0;
  int retval = this->i2c.readByte(MPU6050_RA_SMPLRT_DIV, &data);
  if (retval != 0) {
    return -1;
  }

  return data;
}

int MPU6050::getSampleRate() {
  // Get sample rate divider
  uint16_t sample_div = 0;
  int rate_div = this->getSampleRateDiv();
  if (rate_div != -1 || rate_div != -2) {
    sample_div = (float) rate_div;
  } else {
    return -1;
  }

  // Get gyro sample rate
  uint16_t gyro_rate = 0;
  uint8_t dlpf_cfg = this->getSampleRateDiv();
  if (dlpf_cfg == 0 || dlpf_cfg == 7) {
    gyro_rate = 8000;
  } else if (dlpf_cfg >= 1 || dlpf_cfg <= 6) {
    gyro_rate = 1000;
  } else {
    return -2;
  }

  // calculate sample rate
  return gyro_rate / (1 + sample_div);
}

int MPU6050::setGyroRange(const int range) {
  // pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // set sample rate
  char data = range << 3;
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.writeByte(MPU6050_RA_GYRO_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU6050::getGyroRange() {
  // get gyro config
  char data = 0x00;
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.readByte(MPU6050_RA_GYRO_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // get gyro range bytes
  data = (data >> 3) & 0b00000011;

  return data;
}

int MPU6050::setAccelRange(const int range) {
  // pre-check
  if (range > 3 || range < 0) {
    return -2;
  }

  // set sample rate
  char data = range << 3;
  this->i2c.setSlave(MPU6050_ADDRESS);
  int retval = this->i2c.writeByte(MPU6050_RA_ACCEL_CONFIG, data);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int MPU6050::getAccelRange() {
  char data;
  int retval;

  // get accel config
  data = 0x00;
  this->i2c.setSlave(MPU6050_ADDRESS);
  retval = this->i2c.readByte(MPU6050_RA_ACCEL_CONFIG, &data);
  if (retval != 0) {
    return -1;
  }

  // get accel range bytes
  data = (data >> 3) & 0b00000011;

  return data;
}

void MPU6050::info() {
  printf("gyro sensitivity: %f\n", this->gyro.sensitivity);
  printf("gyro offset_x: %f\n", this->gyro.offset_x);
  printf("gyro offset_y: %f\n", this->gyro.offset_y);
  printf("gyro offset_z: %f\n", this->gyro.offset_z);
  printf("\n");
  printf("accel sensitivity: %f\n", this->accel.sensitivity);
  printf("accel offset_x: %f\n", this->accel.offset_x);
  printf("accel offset_y: %f\n", this->accel.offset_y);
  printf("accel offset_z: %f\n", this->accel.offset_z);
  printf("\n");
  printf("sample rate: %f\n", this->sample_rate);
  printf("\n");
}

void MPU6050::print() {
  printf("gyro_x: %f\n", this->gyro.x);
  printf("gyro_y: %f\n", this->gyro.y);
  printf("gyro_z: %f\n", this->gyro.z);
  printf("accel x: %f\n", this->accel.x);
  printf("accel y: %f\n", this->accel.y);
  printf("accel z: %f\n", this->accel.z);
  printf("temp: %f\n", this->temperature);
}

} // namespace gvio
