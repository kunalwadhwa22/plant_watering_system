#ifndef _BWBOARDIO_H_
#define _BWBOARDIO_H_
//
// Losant Board Definitions here
// This file contains all HW definitions that are board dependent (clocks,
// GPIOs, I2C, SPI, etc)
// Notice that these defines include CPU information and board information
//    consolidated in one place as BWBOARD_ definitions.
//
// For board level software definitions see bwboard.h
//
// For definitions included in #define below, you must include

// REV 1: REV2 simulation system.  We only used this setup to prove out WROVER.  Not recommended because it doesn't have enough GPIOs to do 3 ToF XSHUT.
//        ESP32 code runs on ESP-WROVER-KIT V4.1 with ESP32-WROVER-B
//        This board is also called ESP32-WROVER-KIT VB
//        Use same hardware as REV0 but GPIOs match REV 2 (where possible)
//        No ethernet

#define LOSDOORMON_MB_HW_VERSION         1

//////// GPIO
  // There are one group with 35 pins.
  // There are 35 pins per group (need at least 6 bits to store pin #)
  #define BWBOARD_GPIO_GROUP_TOTAL              1
  #define BWBOARD_GPIO_BITS_PER_GROUP           6

  // There are 1 GPIO expander
  // The CTRL GPIO expander is 16-bits
  #define BWBOARD_GPIOEXP_TOTAL                 1
    #define BWBOARD_GPIOEXP_CTRL                  0
    #define BWBOARD_GPIOEXP_16BIT_START           BWBOARD_GPIOEXP_CTRL0
  #define BWBOARD_GPIOEXP_PINS_PER_EXP          16

// PAY ATTENTION: I2C, SERIAL, SPI are implemented by SERCOM HW in Atmel.
//    There are only SERCOM_INST_NUM total for all serial comm. 
//    Therefore you can't have 6 I2C and 6 SPI.
//    
//////// I2C
// BUSID is the CPU I2C bus number: 0 to BWBOARD_I2C_BUSID_MAX
  #define BWBOARD_I2C_BUSID_TOTAL         I2C_NUM_MAX
  #define BWBOARD_I2C_BUSID_MAX           (BWBOARD_I2C_BUSID_TOTAL-1)

  // Choose standard 100kHz
  #define BWBOARD_I2C0_BUS_CLOCK_RATE_HZ     (400 * 1000)
  #define BWBOARD_I2C1_BUS_CLOCK_RATE_HZ     (100 * 1000)

  #define BWBOARD_I2C_PORTNUM_GRIDEYE                           0

  #define BWBOARD_I2C_PORTNUM_MLX90640                          1

  #define BWBOARD_I2C_PORTNUM_ADS1015                           1


  // 7-bit I2C address of Grideye AMG8833 (AD_SELECT pin is high)
  #define BWBOARD_I2C_AMG8333_SLAVE_ADDR                   0x69

  #define BWBOARD_I2C_MLX90640_SLAVE_ADDR                  0X33

  #define BWBOARD_I2C_ADS1015_SLAVE_ADDR                   0x49


//////// SERIAL UART/USART
// BUSID is the CPU serial bus number: 0 to BWBOARD_SERIAL_BUSID_MAX
//
  #define BWBOARD_SERIAL_BUSID_TOTAL      UART_NUM_MAX
  #define BWBOARD_SERIAL_BUSID_MAX        (BWBOARD_SERIAL_BUSID_TOTAL-1)

  // Atmel defines max clock (also know as baud rate or bit rate)
  // IMPROVE: Depends on system clock
  #define BWBOARD_SERIAL_CLOCK_HZ_MAX     115200

  // UART0: Reserved UART for MCU stdio (stdin/stdout/stderr) and comms with PC
  #define BWBOARD_SERIAL_BUSID_STDIO           UART_NUM_0

  // UART1: UART for communicating with STM32 using AT commands
  #define BWBOARD_SERIAL_BUSID_STM32           UART_NUM_1
  #define BWBOARD_SERIAL_BUSID_STM32_BAUD      9600


//////// SPI
// BUSID is the CPU SPI bus number: 0 to BWBOARD_SPI_BUSID_MAX
// Each CPU SPI has chip selects: 0 to BWBOARD_SPI_CHIPSELECT_MAX
//
  #define BWBOARD_SPI_BUSID_TOTAL         (VSPI_HOST+1)
  #define BWBOARD_SPI_BUSID_MAX           (BWBOARD_SPI_BUSID_TOTAL-1)

  #define BWBOARD_SPI_CLOCK_HZ_MAX        ((8 * 1000000) / 2)
  #define BWBOARD_SPI_CLOCK_HZ_MIN        ((8 * 1000000) / 256)


//////// Watchdog
// If defined, we use a watchdog timer.  If not defined, we don't use it.
// IMPROVE: Nobody uses this #define
#define USE_WATCHDOG_TIMER

#ifdef USE_WATCHDOG_TIMER
// 4 second watchdog.   This value is ignored.  See BWWDTCreate() comments below
#define BWBOARD_WDT_TIMEOUT_MS      (4*1000)
#endif // USE_WATCHDOG_TIMER

#endif // not _BWBOARDIO_H_
