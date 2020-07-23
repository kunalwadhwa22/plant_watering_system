#include "water_pump.h"
#include <stdio.h>
#include "driver/gpio.h"

static bool sensor_1_type = SENSOR_1_TYPE;
static bool sensor_2_type = SENSOR_2_TYPE;
static bool sensor_3_type = SENSOR_3_TYPE;
static bool sensor_4_type = SENSOR_4_TYPE;


// init the gpio related to water pump
void water_pump_init()
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
  return;
}

void water_pump_logic(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  // sensor 1
  if(sensor_1_type == CAPACITIVE_TYPE)
  {
    if(m1 < CAPACITIVE_LOWER_THRESHOLD)
    {
      // pump associated with m1 is switched on
    }
    if(m1 > CAPACITIVE_HIGHER_THRESHOLD)
    {
      // pump associated with m1 is switched off
    }
  }
  else if(sensor_1_type == RESISTIVE_TYPE)
  {
    if(m1 < RESISTIVE_LOWER_THRESHOLD)
    {
      // pump associated with m1 is switched on
    }
    if(m1 > RESISTIVE_UPPER_THRESHOLD)
    {
      // pump associated with m1 is switched on
    }
  }

    // sensor 2
  if(sensor_2_type == CAPACITIVE_TYPE)
  {
    if(m2 < CAPACITIVE_LOWER_THRESHOLD)
    {
      // pump associated with m2 is switched on
    }
    if(m2 > CAPACITIVE_HIGHER_THRESHOLD)
    {
      // pump associated with m2 is switched off
    }
  }
  else if(sensor_2_type == RESISTIVE_TYPE)
  {
    if(m2 < RESISTIVE_LOWER_THRESHOLD)
    {
      // pump associated with m2 is switched on
    }
    if(m2 > RESISTIVE_UPPER_THRESHOLD)
    {
      // pump associated with m2 is switched on
    }
  }


  // sensor 3
  if(sensor_3_type == CAPACITIVE_TYPE)
  {
    if(m3 < CAPACITIVE_LOWER_THRESHOLD)
    {
      // pump associated with m3 is switched on
    }
    if(m3 > CAPACITIVE_HIGHER_THRESHOLD)
    {
      // pump associated with m3 is switched off
    }
  }
  else if(sensor_3_type == RESISTIVE_TYPE)
  {
    if(m3 < RESISTIVE_LOWER_THRESHOLD)
    {
      // pump associated with m3 is switched on
    }
    if(m3 > RESISTIVE_UPPER_THRESHOLD)
    {
      // pump associated with m3 is switched on
    }
  }


  // sensor 4
  if(sensor_4_type == CAPACITIVE_TYPE)
  {
    if(m4 < CAPACITIVE_LOWER_THRESHOLD)
    {
      // pump associated with m4 is switched on
    }
    if(m4 > CAPACITIVE_HIGHER_THRESHOLD)
    {
      // pump associated with m4 is switched off
    }
  }
  else if(sensor_4_type == RESISTIVE_TYPE)
  {
    if(m4 < RESISTIVE_LOWER_THRESHOLD)
    {
      // pump associated with m4 is switched on
    }
    if(m4 > RESISTIVE_UPPER_THRESHOLD)
    {
      // pump associated with m4 is switched on
    }
  }

}

bool water_pump_on(uint8_t pump_number)
{
  switch (pump_number)
  {
  case 1:
    gpio_set_level(PUMP_IO_1, HIGH);
    break;
  
  case 2:
    gpio_set_level(PUMP_IO_2, HIGH);
    break;
  
  case 3:
    gpio_set_level(PUMP_IO_3, HIGH);
    break;
  
  default:
    break;
  }
  return 1;
}

bool water_pump_off(uint8_t pump_number)
{
  switch (pump_number)
  {
  case 1:
    gpio_set_level(PUMP_IO_1, LOW);
    break;
  
  case 2:
    gpio_set_level(PUMP_IO_2, LOW);
    break;
  
  case 3:
    gpio_set_level(PUMP_IO_3, LOW);
    break;
  
  default:
    break;
  }
  return 1;
}