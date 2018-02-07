/**
 * @file
 * @ingroup driver
 */
#ifndef GVIO_DRIVER_I2C_HPP
#define GVIO_DRIVER_I2C_HPP

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
/**
 * @addtogroup driver
 * @{
 */

// ERROR MESSAGES
#define I2C_INIT_FAILED "failed to initialize I2C!"

// DEFINES
#define I2C_BUF_MAX 1024

class I2C {
public:
  int fd = -1;

  I2C() {}
  ~I2C() {
    if (this->fd != -1) {
      close(this->fd);
    }
  }

  int setup();
  int setSlave(const char slave_addr);
  int readBytes(const char reg_addr, char *data, size_t length);
  int readByte(const char reg_addr, char *data);
  int writeByte(const char reg_addr, const char byte);
  int writeRawByte(const char byte);
  int writeBytes(const char reg_addr, const char *data, const size_t length);
};

/** @} group driver */
} // namespace gvio
#endif
