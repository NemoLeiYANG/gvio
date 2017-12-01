#ifndef GVIO_DRIVERS_I2C_HPP
#define GVIO_DRIVERS_I2C_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

#include "gvio/util/log.hpp"

namespace gvio {

// ERROR MESSAGES
#define I2C_INIT_FAILED "failed to initialize I2C!"

// DEFINES
#define I2C_BUF_MAX 1024

class I2C {
public:
  int fd;

  I2C() : fd(-1) {}
  ~I2C() {
    if (this->fd != -1) {
      close(this->fd);
    }
  }

  int setup();
  int setSlave(char slave_addr);
  int readBytes(char reg_addr, char *data, size_t length);
  int readByte(char reg_addr, char *data);
  int writeByte(char reg_addr, char byte);
  int writeRawByte(char byte);
  int writeBytes(char reg_addr, char *data, size_t length);
};

} // namespace gvio
#endif
