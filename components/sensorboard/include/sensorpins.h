#ifndef _LOSPINS_H_
#define _LOSPINS_H_
//
// GPIO pins and GPIO expander pins
//    Use these instead of GPIO #s
//    !!!IMPORTANT!!! See losboard.c for detailed documentation
//      of all GPIOs and GPIO expander GPIOs.
//
#include "bwboardio.h"

// GPIO pins attached directly to ESP32
// !!!PAY ATTENTION!!! These are ESP32 *GPIO number* (in range GPIO 0 to 39)
//   and *not* CPU physical pin number on the IC


#if LOSDOORMON_MB_HW_VERSION >= 1
  //// REV2 and REV1 common GPIOs ////
  // WARNING: GPIO 16 and 17 are not available on WROVER chips

  // REV1: We have to alter I2C GPIOs on the WROVER-KIT VB like this:
  // * Disconnect R_IO5 from LCD FET:  Remove R164 5.1K(1%).   
  //   And add a 10k pullup between I05 and 3.3V (did this on JP4)
  // * IO13 is pulled up(10k) and down(10k):  Remove R50 (pulldown)
  // * IO14 was pulled up by 10k twice:  Remove R55.
  // * OK as is: IO15 pulled up by 10k once
  //  

  #define SENSORPIN_I2C_SCL_0         5
  #define SENSORPIN_I2C_SDA_0        13
  #define SENSORPIN_I2C_SDA_1        14
  #define SENSORPIN_I2C_SCL_1        15

#endif

#endif // not _LOSPINS_H_
