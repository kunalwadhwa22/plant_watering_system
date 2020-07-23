//
// This initializes the Losant Door Monitor main board
//
#include <stdio.h>
#include <string.h>

#include "common/bwdelay.h"
#include "common/bwtime.h"
#include "log/bwlog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"

// Must be before bwgpio/bwgpioexpander
#include "bwboardio.h"

#include "driver/bwgpio.h"
#include "driver/bwgpioexpander.h"
#include "driver/bwi2cdriver.h"
#include "driver/bwhwinfo.h"
#include "driver/bwtimer.h"
#include "driver/bwwdt.h"

#include "sensorpins.h"
#include "sensorboard.h"

#define SENSOR_APP_NAME   "Sensor Application"

static const BWSWVersion firmwareVersion = 1;

BoardGlobal gBoardGlobal;

void LosBoardInit(void)
{
  // Enable Watchdog Timer: BWBOARD_WDT_TIMEOUT_MS is ignored because
  //   esp-idf handles it automatically in FreeRTOS
  // Do this as early as possible!
  BWWDTCreate(0, BWBOARD_WDT_TIMEOUT_MS, 0);

  memset(&gBoardGlobal, 0, sizeof(gBoardGlobal));
    gBoardGlobal.pFirmwareVersion = &firmwareVersion;

  // Change RSSI pattern to display reset reason (with RSSI3 LED on)
  BWHWResetReason reason = BWHWResetReasonGet();

  // Now that pCommandSerial is initialized, output something
  // IMPROVE: Add time/date for debug purposes
  BWLogInfo("%s: BWBoardInit reason: %d ", SENSOR_APP_NAME, (int)reason);

  //// bwhal initialization
  BWDelayInit();

  BWGpioInit(false); // we don't use GPIO interrupt feature

  // Create timer driver for HW instance 0 (which is RTC TIMER_0)
  // This timer is NOT used by the BWTime module (not passed to BWTimeInit())
  gBoardGlobal.pTimerDriver = BWTimerCreate(0); 

  BWTimeInit(NULL);


  return true; // SUCCESS
}
