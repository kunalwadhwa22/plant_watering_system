#include "kwADC_ads1015.h"

#include <stdio.h>
#include "driver/bwi2c.h"
#include "driver/bwi2cdriver.h"
#include "log/bwlog.h"

#include "common/bwassert.h"
#include "common/bwversion.h"
#include "common/bwdelay.h"
#include "common/bwtime.h"
#include "driver/bwwdt.h"

#include "log/bwlog.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bwboardio.h"
#include "sensorboard.h"
#include "sensorpins.h"

#include "math.h"

// API Implementation of ADS1015 12-bit ADC with i2c interface and PGA

#define AIN0_GND_ENABLE_MASK        0x0040
#define AIN0_GND_ENABLE_CLEAR       0xFF8F

#define AIN1_GND_ENABLE_MASK        0x0050
#define AIN1_GND_ENABLE_CLEAR       0xFF8F

#define AIN2_GND_ENABLE_MASK        0x0060
#define AIN2_GND_ENABLE_CLEAR       0xFF8F

#define AIN3_GND_ENABLE_MASK        0x0070
#define AIN3_GND_ENABLE_CLEAR       0xFF8F

#define PGA_LEVEL_4096_MASK         0x0002
#define PGA_LEVEL_4096_CLEAR        0xFFF1

#define MODE_SELECT_CONT_MASK       0x0000
#define MODE_SELECT_CONT_CLEAR      0xFFFE

#define DATA_RATE_3300_MASK         0xE000
#define DATA_RATE_3300_CLEAR        0x1FFF

#define CONVERSION_REGISTER_ADDRESS     0x00
#define CONFIG_REGISTER_ADDRESS         0X01
#define LO_THRESHOLD_REGISTER_ADDRESS   0x02
#define HI_THRESHOLD_REGISTER_ADDRESS   0X03

static uint16_t ain0_gnd_enable_mask = AIN0_GND_ENABLE_MASK;
static uint16_t ain0_gnd_enable_clear = AIN0_GND_ENABLE_CLEAR;

static uint16_t ain1_gnd_enable_mask = AIN1_GND_ENABLE_MASK;
static uint16_t ain1_gnd_enable_clear = AIN1_GND_ENABLE_CLEAR;

static uint16_t ain2_gnd_enable_mask = AIN2_GND_ENABLE_MASK;
static uint16_t ain2_gnd_enable_clear = AIN2_GND_ENABLE_CLEAR;

static uint16_t ain3_gnd_enable_mask = AIN3_GND_ENABLE_MASK;
static uint16_t ain3_gnd_enable_clear = AIN3_GND_ENABLE_CLEAR;

static uint16_t pga_level_4096_mask = PGA_LEVEL_4096_MASK;
static uint16_t pga_level_4096_clear = PGA_LEVEL_4096_CLEAR;

static uint16_t mode_select_cont_mask = MODE_SELECT_CONT_MASK;
static uint16_t mode_select_cont_clear = MODE_SELECT_CONT_CLEAR;

static uint16_t data_rate_3300_clear = DATA_RATE_3300_CLEAR;
static uint16_t data_rate_3300_mask = DATA_RATE_3300_MASK;

static uint8_t conversion_register_address = CONVERSION_REGISTER_ADDRESS;
static uint8_t config_register_address = CONFIG_REGISTER_ADDRESS;
static uint8_t lo_threshold_register_address = LO_THRESHOLD_REGISTER_ADDRESS;
static uint8_t hi_threshold_register_address = HI_THRESHOLD_REGISTER_ADDRESS;

static BWBus *m_pComm;

static bool ads1015WriteConfigRegister(uint16_t value);
static uint8_t ads1015GetDualStates(uint16_t sensorValue1, uint16_t sensorValue2);
static uint8_t ads1015GetSingleState(uint16_t sensorValue);

static uint8_t previousDualState = NO_STATE;
static uint8_t currentDualState = NO_STATE;

static uint8_t previousDoorState = DOOR_INVALID_STATE;
static uint8_t currentDoorState = DOOR_INVALID_STATE;

uint8_t ads1015GetDoorOpenClose(uint16_t sensorValue1, uint16_t sensorValue2)
{
    previousDualState = currentDualState;
    previousDoorState = currentDoorState;
    currentDualState = ads1015GetDualStates(sensorValue1, sensorValue2);
    printf("DUAL STATE %d \t", currentDualState);

    switch (currentDualState)
    {
    case S1_LOW_S2_LOW:
        {
            currentDoorState = DOOR_CLOSE;
        }
        break;

    case S1_LOW_S2_MIDDLE:
        {
            currentDoorState = DOOR_AJAR_INSIDE;
        }
        break;

    case S1_LOW_S2_HIGH:
        {
            printf("Error State S1_LOW_S2_HIGH\n");
        }
        break;

    case S1_MIDDLE_S2_LOW:
        {
            currentDoorState = DOOR_AJAR_OUTSIDE;
        }        
        break;

    case S1_MIDDLE_S2_MIDDLE:
        {
            if(previousDoorState == DOOR_AJAR_INSIDE)
            {
                currentDoorState = DOOR_OPEN_INSIDE;
            }
            else if(previousDoorState == DOOR_AJAR_OUTSIDE)
            {
                currentDoorState = DOOR_OPEN_OUTSIDE;
            }
        }
        break;

    case S1_MIDDLE_S2_HIGH:
        {
            currentDoorState = DOOR_AJAR_OUTSIDE;
        }
        break;

    case S1_HIGH_S2_LOW:
        {
            printf("Error State S1_HIGH_S2_LOW\n");
        }
        break;

    case S1_HIGH_S2_MIDDLE:
        {
            currentDoorState = DOOR_AJAR_INSIDE;
        }
        break;

    case S1_HIGH_S2_HIGH:
        {
            printf("Error State S1_HIGH_S2_HIGH\n");
        }
        break;

    case NO_STATE:
        {
            printf("Error State NO_STATE");
        }
        break;

    default:
        break;
    }

    return currentDoorState;
}


static uint8_t ads1015GetSingleState(uint16_t sensorValue)
{
    if(sensorValue > UPPER_THRESHOLD)
    {
        return STATE_HIGH;
    }
    else if(sensorValue < LOWER_THRESHOLD)
    {
        return STATE_LOW;
    }
    return STATE_MIDDLE;
}

static uint8_t ads1015GetDualStates(uint16_t sensorValue1, uint16_t sensorValue2)
{
    uint8_t dualState = NO_STATE;
    uint8_t singleStateSensor1 = ads1015GetSingleState(sensorValue1);
    uint8_t singleStateSensor2 = ads1015GetSingleState(sensorValue2);

    switch (singleStateSensor1)
    {
    case STATE_LOW:
        {
            if(singleStateSensor2 == STATE_LOW)
            {
                return S1_LOW_S2_LOW;
            }
            else if(singleStateSensor2 == STATE_MIDDLE)
            {
                return S1_LOW_S2_MIDDLE;
            }
            else
            {
                return S1_LOW_S2_HIGH;
            }
        }
        break;

    case STATE_MIDDLE:
        {
            if(singleStateSensor2 == STATE_LOW)
            {
                return S1_MIDDLE_S2_LOW;
            }
            else if(singleStateSensor2 == STATE_MIDDLE)
            {
                return S1_MIDDLE_S2_MIDDLE;
            }
            else
            {
                return S1_MIDDLE_S2_HIGH;
            }
        }
        break;

    case STATE_HIGH:
        {
            if(singleStateSensor2 == STATE_LOW)
            {
                return S1_HIGH_S2_LOW;
            }
            else if(singleStateSensor2 == STATE_MIDDLE)
            {
                return S1_HIGH_S2_MIDDLE;
            }
            else
            {
                return S1_HIGH_S2_HIGH;
            }
        }
        break;

    default:
        break;
    }

    return dualState;
}


bool ads1015Init()
{
    BWBusMode i2cBusModeWithSubAddr = { 1, 0 }; // I2C: using 0-byte subaddress / no subaddress
    BWDelayMS(1000);
    m_pComm = BWSerialI2CCreate(BWBOARD_I2C_PORTNUM_ADS1015,
                                    SENSORPIN_I2C_SCL_1, SENSORPIN_I2C_SDA_1,
                                    BWBOARD_I2C_ADS1015_SLAVE_ADDR, 
                                    &i2cBusModeWithSubAddr,
                                    BWBOARD_I2C1_BUS_CLOCK_RATE_HZ,
                                    1000,0);
    
    if(!m_pComm)
    {
        printf("m_pcomm failed\n");
        return false;
    }

    uint16_t config_register = ads1015ReadConfigRegister();

    printf("Config register contents 0x%x \n", config_register);

    // contents of the config register should be 0x8385 in the start

    uint16_t config_register_new =  config_register & (ain1_gnd_enable_clear & pga_level_4096_clear
                                & mode_select_cont_clear & data_rate_3300_clear);
    config_register_new = config_register_new | ain1_gnd_enable_mask | pga_level_4096_mask
                                | mode_select_cont_mask | data_rate_3300_mask;

    printf("Config register to write : 0x%4x \n", config_register_new);

    // write the new config register 
    
    ads1015WriteConfigRegister(config_register_new);

    config_register = 0;
    BWBusMasterRead16(m_pComm, 0x01, &config_register);
    printf("Config register contents 0x%x \n", config_register);

    // read from the conversion register once

    uint16_t conversion_register = 0;

    conversion_register = ads1015ReadConversionRegister();

    printf("Conversion register contents 0x%x \n", conversion_register);

    printf("milliVolts from the ADC %d \n", ads1015GetMilliVoltsReading());
    
    return true;
}

uint16_t ads1015ReadConfigRegister()
{
    uint16_t config_register_contents;
    BWBusXferLength bytesReadConfigReg = BWBusMasterRead16(m_pComm, config_register_address, &config_register_contents);
    if(bytesReadConfigReg != 2)
    {
        return 0;
    }
    return config_register_contents;
}

uint16_t ads1015ReadConversionRegister()
{
    uint16_t conversion_register_contents;
    BWBusXferLength bytesReadCoversionReg = BWBusMasterRead16(m_pComm, conversion_register_address, &conversion_register_contents);
    if(bytesReadCoversionReg != 2)
    {
        return 0;
    }
    return conversion_register_contents;
}

int16_t ads1015GetMilliVoltsReading()
{
    int16_t milliVolts = 0;
    uint16_t conversion_register = ads1015ReadConversionRegister(); 
    uint16_t conversion_register_endianness_corrected; 
    conversion_register_endianness_corrected = (conversion_register << 8) | (conversion_register >> 8);
    int8_t sign = 1; 
    if((conversion_register_endianness_corrected & 0x8000) == 0x8000)
    {
        sign = -1;
    }
    conversion_register_endianness_corrected = conversion_register_endianness_corrected >> 4;
    conversion_register_endianness_corrected = conversion_register_endianness_corrected & 0x07FF;
    milliVolts = conversion_register_endianness_corrected * 2;
    milliVolts = milliVolts * sign;
    // read the config register to see if the register is ready or not
    
    return milliVolts;
}

int16_t ads1015ReadChannel(uint8_t channel)
{
    uint16_t config_register_new = 0;
    uint16_t config_register = ads1015ReadConfigRegister();

    switch (channel)
    {
    case 0:
        //Set Config register to desire value
        config_register_new =  config_register & (ain0_gnd_enable_clear & pga_level_4096_clear
                                & mode_select_cont_clear & data_rate_3300_clear);
        config_register_new = config_register_new | ain0_gnd_enable_mask | pga_level_4096_mask
                                | mode_select_cont_mask | data_rate_3300_mask;
        // write the new config register value to the device
        ads1015WriteConfigRegister(config_register_new);
        break;
    case 1:
        //Set Config register to desire value
        config_register_new =  config_register & (ain1_gnd_enable_clear & pga_level_4096_clear
                                & mode_select_cont_clear & data_rate_3300_clear);
        config_register_new = config_register_new | ain1_gnd_enable_mask | pga_level_4096_mask
                                | mode_select_cont_mask | data_rate_3300_mask;
        // write the new config register value to the device
        ads1015WriteConfigRegister(config_register_new);
        break;
     case 2:
        //Set Config register to desire value
        config_register_new =  config_register & (ain2_gnd_enable_clear & pga_level_4096_clear
                                & mode_select_cont_clear & data_rate_3300_clear);
        config_register_new = config_register_new | ain2_gnd_enable_mask | pga_level_4096_mask
                                | mode_select_cont_mask | data_rate_3300_mask;
        // write the new config register value to the device
        ads1015WriteConfigRegister(config_register_new);
        break;
    case 3:
        //Set Config register to desire value
        config_register_new =  config_register & (ain3_gnd_enable_clear & pga_level_4096_clear
                                & mode_select_cont_clear & data_rate_3300_clear);
        config_register_new = config_register_new | ain3_gnd_enable_mask | pga_level_4096_mask
                                | mode_select_cont_mask | data_rate_3300_mask;
        // write the new config register value to the device
        ads1015WriteConfigRegister(config_register_new);
        break;
    default:
        break;
    }

    // printf("Channel : %d \t Config Old: 0x%x \t Config New: 0x%x \t", channel, config_register, config_register_new);

    BWDelayMS(1);

    return ads1015GetMilliVoltsReading();
}

static bool ads1015WriteConfigRegister(uint16_t value)
{
    BWBusXferLength bytesWritternConfigRegister = BWBusMasterWrite16(m_pComm, config_register_address, value);
  
    if (bytesWritternConfigRegister != 3) 
    {
        printf("refresh rate block fail , bytes written : %d\n", bytesWritternConfigRegister);
        return false; // FAIL
    }
    return true;
}