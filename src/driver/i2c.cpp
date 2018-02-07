#include "gvio/driver/i2c.hpp"

namespace gvio {

int I2C::setup() {
  int fd;
  int adapter_nr;
  char filename[20];

  // Setup
  adapter_nr = 1; // probably dynamically determined
  memset(filename, '\0', sizeof(char) * 20);
  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);

  // Open i2c connection
  fd = open(filename, O_RDWR);
  if (fd < 0) {
    LOG_INFO("Failed to open i2c connection!");
    return -1;
  } else {
    this->fd = fd;
  }

  return 0;
}

int I2C::setSlave(const char slave_addr) {
  return ioctl(this->fd, I2C_SLAVE, slave_addr);
}

int I2C::readByte(const char reg_addr, char *data) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(this->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(this->fd, data, 1) != 1) {
    return -2;
  }

  return 0;
}

int I2C::readBytes(const char reg_addr, char *data, size_t length) {
  char buf[1];

  buf[0] = reg_addr;
  if (write(this->fd, buf, 1) != 1) {
    return -1;
  }

  if (read(this->fd, data, length) != (int) length) {
    return -2;
  }

  return 0;
}

int I2C::writeByte(const char reg_addr, const char byte) {
  char buf[2];

  buf[0] = reg_addr;
  buf[1] = byte;
  if (write(this->fd, buf, 2) != 1) {
    return -1;
  }

  return 0;
}

int I2C::writeRawByte(const char byte) {
  if (write(this->fd, &byte, 1) != 1) {
    return -1;
  }

  return 0;
}

int I2C::writeBytes(const char reg_addr,
                    const char *data,
                    const size_t length) {
  int i;
  char buf[I2C_BUF_MAX];

  // Create buf
  memset(buf, '\0', sizeof(char) * I2C_BUF_MAX);
  buf[0] = reg_addr;
  for (i = 1; i < (int) length + 1; i++) {
    buf[i] = data[i];
  }

  // Write bytes
  if (write(this->fd, buf, length + 1) != 1) {
    return -1;
  }

  return 0;
}

} // namespace gvio
