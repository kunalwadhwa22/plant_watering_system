#ifndef WATER_PUMP_H_
#define WATER_PUMP_H_

#include<stdbool.h>
#include<stdint.h>

// system has 3 water pump

// defines for output to each water pump
#define PUMP_IO_1   18
#define PUMP_IO_2   19
#define PUMP_IO_3   20
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<PUMP_IO_1) | (1ULL<<PUMP_IO_2) | (1ULL<<PUMP_IO_3))

#define HIGH  1
#define LOW   0

#define CAPACITIVE_TYPE   0
#define RESISTIVE_TYPE    1

#define SENSOR_1_TYPE     CAPACITIVE_TYPE
#define SENSOR_2_TYPE     CAPACITIVE_TYPE
#define SENSOR_3_TYPE     RESISTIVE_TYPE
#define SENSOR_4_TYPE     RESISTIVE_TYPE

// todo : set good values
#define RESISTIVE_LOWER_THRESHOLD   100
#define RESISTIVE_UPPER_THRESHOLD   200

// todo : set good values
#define CAPACITIVE_LOWER_THRESHOLD    1000
#define CAPACITIVE_HIGHER_THRESHOLD   2000

// init the gpio related to water pump
void water_pump_init();

// switch on a particular pump
bool water_pump_on(uint8_t pump_number);

// switch off a particular pump
bool water_pump_off(uint8_t pump_number);

// logic to switch on/off the pump depending on the moisture level
void water_pump_logic(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif //