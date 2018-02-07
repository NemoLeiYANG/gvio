/**
 * @file
 * @ingroup driver
 */
#ifndef GVIO_DRIVER_UART_HPP
#define GVIO_DRIVER_UART_HPP

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
  bool connected = false;
  int connection = -1;
  std::string port;
  int speed;
  int parity = 0;
  int blocking = 1;

  UART(const std::string &port, const int speed) : port{port}, speed{speed} {}

  int connect();
  int disconnect();
  int setInterfaceAttributes(const int speed, const int parity);
  void setBlocking(const int blocking);
};

int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);

/** @} group driver */
} // namespace gvio
#endif // GVIO_DRIVER_UART_HPP
