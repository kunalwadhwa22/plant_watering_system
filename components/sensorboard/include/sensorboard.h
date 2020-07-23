#ifndef _LOSBOARD_H_
#define _LOSBOARD_H_
//
// Losant Door Monitor mainboard high-level definitions.
//    For hardware definitions see bwboardio.h
//
#include <stdbool.h>
#include <stdint.h>

#include "bwboardio.h"

#include "common/bwtypes.h"

// Initialize the board and all its hardware and drivers
void LosBoardInit(void);

// Globals created by BWBoardInit for application to use
typedef struct BoardGlobal {
  const BWSWVersion *pFirmwareVersion;
  struct BWTimerDriver *pTimerDriver;
} BoardGlobal;
extern BoardGlobal gBoardGlobal;

// Take mutex for i2cBusId
// i2cBusId: the I2C bus to lock with mutex.  
//    Starts at 0, max is (BWBOARD_I2C_BUSID_TOTAL-1)
// msToWait: Number of ms to wait for semaphore.  Use 0 for "forever".
// RETURNS: true if result of xSemaphoreTakeRecursive is pdTRUE, false otherwise
#if LOSDOORMON_MB_HW_VERSION >= 2
bool LosBoardI2cMutexTake(BWHWInst i2cBusId, BWTimeMS msToWait);
#else // LOSDOORMON_MB_HW_VERSION < 2
#define LosBoardI2cMutexTake(i2cBusId, msToWait) \
  do { } while(0)
#endif // LOSDOORMON_MB_HW_VERSION < 2

// Give mutex for i2cBusId
// i2cBusId: the I2C bus to lock with mutex.  
//    Starts at 0, max is (BWBOARD_I2C_BUSID_TOTAL-1)
// RETURNS: true if result of xSemaphoreGiveRecursive is pdTRUE, false otherwise
#if LOSDOORMON_MB_HW_VERSION >= 2
bool LosBoardI2cMutexGive(BWHWInst i2cBusId);
#else // LOSDOORMON_MB_HW_VERSION < 2
#define LosBoardI2cMutexGive(i2cBusId) \
  do { } while(0)
#endif // LOSDOORMON_MB_HW_VERSION < 2

#if LOSDOORMON_MB_HW_VERSION >= 2
// Initialize all pin directions on GPIO expander, and set all outputs to 0
// RETURNS: true on success, false otherwise
bool LosBoardInitGpioExpPins(void);
#endif // LOSDOORMON_MB_HW_VERSION >= 2

#endif // not _LOSBOARD_H_
