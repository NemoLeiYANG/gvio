#include <stdlib.h>
#include "gvio/gvio.hpp"

using namespace gvio;

void print_usage() {
  std::cout << "Usage: pwm <Frequency (Hz)>" << std::endl;
  std::cout << "Example: pwm 40" << std::endl;
}

int main(const int argc, const char *argv[]) {
  // Parse CLI args
  if (argc != 2) {
    print_usage();
    exit(-1);
  }
  const int freq = strtod(argv[1], NULL);

  // Set PWM driver frequency
  PCA9685 pwm_driver;
  pwm_driver.configure(freq);
  pwm_driver.setAllPWM(4096 / 2);

  return 0;
}
