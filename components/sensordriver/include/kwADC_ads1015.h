#ifndef KW_ADC_ADS1015_H_
#define KW_ADC_ADS1015_H_

#include "stdint.h"
#include "stdbool.h"

// Door open close define 
// Define States 
// ideally make it enum
#define DOOR_OPEN_INSIDE        0
#define DOOR_AJAR_INSIDE        1
#define DOOR_CLOSE              2
#define DOOR_AJAR_OUTSIDE       3
#define DOOR_OPEN_OUTSIDE       4
#define DOOR_INVALID_STATE      5

#define UPPER_THRESHOLD         2500
#define LOWER_THRESHOLD         500

// These are the states the magnetic sensor can be in 
// 0 means it has value less than 500
// 1 means it has value between 500 and 2500
// 2 means it has value above 2500
#define STATE_LOW       0
#define STATE_MIDDLE    1
#define STATE_HIGH      2

// Now there can be combinations of the above states for 2 sensors
// Dual State
#define S1_LOW_S2_LOW           0
#define S1_LOW_S2_MIDDLE        1
#define S1_LOW_S2_HIGH          2
#define S1_MIDDLE_S2_LOW        3
#define S1_MIDDLE_S2_MIDDLE     4
#define S1_MIDDLE_S2_HIGH       5
#define S1_HIGH_S2_LOW          6
#define S1_HIGH_S2_MIDDLE       7
#define S1_HIGH_S2_HIGH         8
#define NO_STATE                9

// API Header File of ADS1015 12-bit ADC with i2c interface and PGA

bool ads1015Init();

uint16_t ads1015ReadConfigRegister();

uint16_t ads1015ReadConversionRegister();

int16_t ads1015GetMilliVoltsReading();

int16_t ads1015ReadChannel(uint8_t channel);

uint8_t ads1015GetDoorOpenClose(uint16_t sensorValue1, uint16_t sensorValue2);

#endif // not KW_ADC_ADS1015_H_