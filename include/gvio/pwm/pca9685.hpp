#ifndef GVIO_PWM_PCA9685_HPP
#define GVIO_PWM_PCA9685_HPP

#include <math.h>
#include <unistd.h>

#include "gvio/driver/i2c.hpp"

namespace gvio {

// REGISTERS
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADD1 0x02
#define PCA9685_SUBADD2 0x03
#define PCA9685_SUBADD3 0x04
#define PCA9685_ALLCALLADR 0x05
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_LED1_ON_L 0x0A
#define PCA9685_LED1_ON_H 0x0B
#define PCA9685_LED1_OFF_L 0x0C
#define PCA9685_LED1_OFF_H 0x0D
#define PCA9685_LED2_ON_L 0x0E
#define PCA9685_LED2_ON_H 0x0F
#define PCA9685_LED2_OFF_L 0x10
#define PCA9685_LED2_OFF_H 0x11
#define PCA9685_LED3_ON_L 0x12
#define PCA9685_LED3_ON_H 0x13
#define PCA9685_LED3_OFF_L 0x14
#define PCA9685_LED3_OFF_H 0x15
#define PCA9685_LED4_ON_L 0x16
#define PCA9685_LED4_ON_H 0x17
#define PCA9685_LED4_OFF_L 0x18
#define PCA9685_LED4_OFF_H 0x19
#define PCA9685_LED5_ON_L 0x1A
#define PCA9685_LED5_ON_H 0x1B
#define PCA9685_LED5_OFF_L 0x1C
#define PCA9685_LED5_OFF_H 0x1D
#define PCA9685_LED6_ON_L 0x1E
#define PCA9685_LED6_ON_H 0x1F
#define PCA9685_LED6_OFF_L 0x20
#define PCA9685_LED6_OFF_H 0x21
#define PCA9685_LED7_ON_L 0x22
#define PCA9685_LED7_ON_H 0x23
#define PCA9685_LED7_OFF_L 0x24
#define PCA9685_LED7_OFF_H 0x25
#define PCA9685_LED8_ON_L 0x26
#define PCA9685_LED8_ON_H 0x27
#define PCA9685_LED8_OFF_L 0x28
#define PCA9685_LED8_OFF_H 0x29
#define PCA9685_LED9_ON_L 0x2A
#define PCA9685_LED9_ON_H 0x2B
#define PCA9685_LED9_OFF_L 0x2C
#define PCA9685_LED9_OFF_H 0x2D
#define PCA9685_LED10_ON_L 0x2E
#define PCA9685_LED10_ON_H 0x2F
#define PCA9685_LED10_OFF_L 0x30
#define PCA9685_LED10_OFF_H 0x31
#define PCA9685_LED11_ON_L 0x32
#define PCA9685_LED11_ON_H 0x33
#define PCA9685_LED11_OFF_L 0x34
#define PCA9685_LED11_OFF_H 0x35
#define PCA9685_LED12_ON_L 0x36
#define PCA9685_LED12_ON_H 0x37
#define PCA9685_LED12_OFF_L 0x38
#define PCA9685_LED12_OFF_H 0x39
#define PCA9685_LED13_ON_L 0x3A
#define PCA9685_LED13_ON_H 0x3B
#define PCA9685_LED13_OFF_L 0x3C
#define PCA9685_LED13_OFF_H 0x3D
#define PCA9685_LED14_ON_L 0x3E
#define PCA9685_LED14_ON_H 0x3F
#define PCA9685_LED14_OFF_L 0x40
#define PCA9685_LED14_OFF_H 0x41
#define PCA9685_LED15_ON_L 0x42
#define PCA9685_LED15_ON_H 0x43
#define PCA9685_LED15_OFF_L 0x44
#define PCA9685_LED15_OFF_H 0x45
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRE_SCALE 0xFE
#define PCA9685_TEST_MODE 0xFF

// OTHER
#define PCA9685_I2C_ADDR 0x40
#define PCA9685_WAIT_MS 5

class PCA9685 {
public:
  I2C i2c;

  PCA9685() {}

  /**
   * Configure
   *
   * @param freq Frequency (Hz)
   * @returns 0 for success, -1 for failure
   */
  int configure(const int freq);

  /**
   * Set PWM frequency
   *
   * @param freq Frequency (Hz)
   * @returns 0 for success, -1 for failure
   */
  void setPWMFrequency(const int freq);

  /**
   * Set PWM on a specific channel
   *
   * @param channel
   * @param off
   * @returns 0 for success, -1 for failure
   */
  void setPWM(const int8_t channel, const int16_t off);

  /**
   * Set PWM on all channels
   *
   * @param off
   * @returns 0 for success, -1 for failure
   */
  void setAllPWM(const int16_t off);

  /**
   * Reset PCA9685
   */
  void reset();
};

} // namespace gvio
#endif // GVIO_PWM_PCA9685_HPP
