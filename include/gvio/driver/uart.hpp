/**
 * @file
 * @ingroup driver driver
 */
#ifndef GVIO_DRIVERS_UART_HPP
#define GVIO_DRIVERS_UART_HPP

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

namespace gvio {
/**
 * @addtogroup driver
 * @{
 */

class UART {
public:
  bool connected;
  int connection;
  std::string port;
  int speed;
  int parity;
  int blocking;

  UART(const std::string &port, int speed)
      : connected{false}, connection{-1}, port{port}, speed{speed}, parity{0},
        blocking{1} {}

  int connect();
  int disconnect();
  int setInterfaceAttributes(int speed, int parity);
  void setBlocking(int blocking);
};

int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);

/** @} group driver */
} // namespace gvio
#endif
