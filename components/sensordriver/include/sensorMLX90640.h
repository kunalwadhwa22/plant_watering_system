#ifndef SENSOR_MLX90640_H_
#define SENSOR_MLX90640_H_

// Driver header file for MLX90640 - 32 x 24 - Thermal Sensor

#include "stdint.h"
#include "stdbool.h"

#define SCALEALPHA 0.000001

#define LEFT_START              0
#define LEFT_END                256
#define MIDDLE_START            256
#define MIDDLE_END              512
#define RIGHT_START             512
#define RIGHT_END               768

typedef struct
{
        int16_t kVdd;
        int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        uint16_t vPTAT25;
        float alphaPTAT;
        int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        uint8_t resolutionEE;
        uint8_t calibrationModeEE;
        float KsTa;
        float ksTo[5];
        int16_t ct[5];
        uint16_t alpha[768];    
        uint8_t alphaScale;
        int16_t offset[768];    
        int8_t kta[768];
        uint8_t ktaScale;    
        int8_t kv[768];
        uint8_t kvScale;
        float cpAlpha[2];
        int16_t cpOffset[2];
        float ilChessC[3]; 
        uint16_t brokenPixels[5];
        uint16_t outlierPixels[5];  
} paramsMLX90640;


// Status register bitwise definitions here

// Configuration register-1 bitwise definitions here

void SensorRunLoopMLX90640(void);

// Function to initialize the sensor
// Sensor address 0x33
// RETURNS : true if successfully initialized
//            false if failed initialization
bool SensorInitMLX90640(void);

// Function to read from the sensor

// Function to set the refresh rate of the sensor
// This function currently sets the refresh rate in the RAM
// So the settings will erase after the device reboots
void SensorSetRefreshRateMLX90640(uint8_t refreshRate);

int16_t SensorGetRefreshRateMLX90640(void);

// Function to read the control register 1 of the sensor
// Control register is a 16 bit register stored at the address 0x800D
// RETURNS : Endianess corrected contents if the register
uint16_t SensorGetControlRegister1MLX90640(void);

// Function to Check if the sensor is ready to give frame data
// the status register (global in the .c file) is re-read from the device if
//      'reReadStatusRegister' is passed true
// if not it will just use the value in the current variable to check the 
//      status if the availability of new data
// RETURNS : true if sensor is ready with new data
//          : false if not ready
bool SensorIsNewDataReadyMLX90640(bool reReadStatusRegister);

// Function to read the status register of the sensor
// The status register is a 16 bit register located at the address 0x8000
// RETURNS : the endianess corrected contents of the status register
uint16_t SensorGetStatusRegisterMLX90640(void);

bool SensorGetFrameMLX90640(uint16_t * frame);

// Function to get the current frame that was read from the sensor
//      0 means 0 sub-page
//      1 means 1 sub-page
//      any other value is melexis reserved and should not be seen
uint8_t SensorGetCurrentSubFrameMLX90640(bool reReadStatusRegister);

// function to calculate Vdd from frameData and extracted parameters
float SensorGetVddMLX90640(uint16_t *frameData, paramsMLX90640 *params);
float SensorGetTaMLX90640(uint16_t *frameData, paramsMLX90640 *params);
void SensorCalculateObjectTemperatureMLX90640(uint16_t *frameData, paramsMLX90640 *params, float emissivity, float tr, float *result);


// EEPROM Functions 

// Function to dump all the EEPROM contents to the arguement pointer/array
bool SensorDumpEEMLX90640(uint16_t * m_eeData);

int SensorExtractParametersMLX90640(uint16_t *eeData, paramsMLX90640 *mlx90640);


// helpers

// Function to change endianess of a uint16_t variable
uint16_t ChangeEndianessUnsignedInteger16(uint16_t input);

// Function to change the endianess of the whole frame
void ChangeEndianessArrayUnsignedInteger16(uint16_t * frame, uint16_t length);

// Function to print the frame in a 32 x 24 line format
void SensorPrintFrameMLX90640(uint16_t * frame, uint8_t rows, uint8_t columns);

void SensorPrintToFrameMLX90640(float * frame, uint8_t rows, uint8_t columns);

void SensorPrintToFrameBinaryMLX90640(float * frame, uint8_t rows, uint8_t columns, uint16_t threshold);

void PrintDesiredData(void);

// mathematical functions for the thermal sensor

float GetFrameTotal(uint16_t startPixel, uint16_t endPixel, float * frame);

float GetFrameAverage(uint16_t startPixel, uint16_t endPixel, float * frame);

float GetFrameTotalOfPixelsAboveThreshold(uint16_t startPixel, uint16_t endPixel, float * frame, float threshold);




#endif