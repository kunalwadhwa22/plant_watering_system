#ifndef _LOSANTGRIDEYE_H_
#define _LOSANTGRIDEYE_H_

// Header file for Losant Grid Eye implementation
//
// To do a general read for the gridEye
//      initialize the sensor and I2C handle            :   losantGridEyeInit()
//      read the frame                                  :   losantGridEyeGetFrame()
//  all other APIs are to perform applications over the Grid Obtained
//
// Note 1 : In the Functions that use a 'start' and 'end' value, 
// It is assumed that 'start' value is less than (equal to) 'end' 

// Note: It is assumed that people walk through the rows along the columns

#include "common/bwtypes.h"
#include "driver/bwbus.h"
#include "stdint.h"


#define GRID_START      	0
#define GRID_END	        64

#define THERM_ADDRESS 0x0E
#define DATA_OFFSET 0x80

#define NORMAL_WIDE_DOOR    0
#define EXTRA_WIDE_DOOR     1
#define DOOR_WIDTH_TYPE     NORMAL_WIDE_DOOR

// Defines for the trimmed array
// We eliminate the topmost and the bottommost columns in our current implementation

//    Left Region                               Left Region 
//   x x x x x x x x                         0 x x x x x x 0 
//   x x x x x x x x                         0 x x x x x x 0 
//   x x x x x x x x                         0 x x x x x x 0 
//   x x x x x x x x         ----->          0 x x x x x x 0 
//   x x x x x x x x         ----->          0 x x x x x x 0 
//   x x x x x x x x                         0 x x x x x x 0 
//   x x x x x x x x                         0 x x x x x x 0 
//   x x x x x x x x                         0 x x x x x x 0 
//    Right Region                             Right Region


#if DOOR_WIDTH_TYPE == NORMAL_WIDE_DOOR
#define TRIMMED_TOP_LEFT_X          0      
#define TRIMMED_TOP_LEFT_Y          1      
#define TRIMMED_BOTTOM_RIGHT_X      7          
#define TRIMMED_BOTTOM_RIGHT_Y      6      
#endif

#if DOOR_WIDTH_TYPE == EXTRA_WIDE_DOOR
#define TRIMMED_TOP_LEFT_X          0      
#define TRIMMED_TOP_LEFT_Y          0      
#define TRIMMED_BOTTOM_RIGHT_X      7          
#define TRIMMED_BOTTOM_RIGHT_Y      7      
#endif

#define TRIMMED_X_RANGE (TRIMMED_BOTTOM_RIGHT_X - TRIMMED_TOP_LEFT_X + 1)
#define TRIMMED_Y_RANGE (TRIMMED_BOTTOM_RIGHT_Y - TRIMMED_TOP_LEFT_Y + 1)

// Function to Initialize Grid Eye module
// RETURNS: 1 on success
bool losantGridEyeInit(void);

// Function to get the next Grid Eye frame
uint16_t * losantGridEyeGetFrame(void);

// Function to print the current frame values
void losantGridEyePrintFrame(void);

// Function to see if the reading are stable
bool losantGridEyeGetStable(void);

// Funtion to get the number of rows between the 'start' and 'end' pixel in the frame
// RETURNS the integer roundoff of rows
uint32_t losantGridEyeGetRows(uint32_t start, uint32_t end);

// Takes the frame 'average' , 'start' and 'end' pixels as inputs
// sets the value of ROI_UPPER_THRESHOLD 
// RETURNS: ROI_UPPER_THRESHOLD
uint32_t losantGridEyeGetRoiUpperThreshold(float average, uint32_t start, uint32_t end);

// Takes the frame 'average' , 'start' and 'end' pixels as inputs
// sets the value of ROI_LOWER_THRESHOLD
// RETURNS : ROI_LOWER_THRESHOLD 
uint32_t losantGridEyeGetRoiLowerThreshold(float average, uint32_t start, uint32_t end);

// Method to get the current state
// TODO: Need to get this function in the upper algorithm layers
uint32_t losantGridEyeGetState(uint8_t top, uint8_t mid, uint8_t bot);

// Function to calculate the average value of the pixels starting from 'start' ending at 'end'
// RETURNS : float 'AVERAGE'
float losantGridEyeGetFrameAverage(uint32_t start, uint32_t end);

// Function to get the 'position' of the Maximum valued pixel
// Range starts from 'start' and ends at 'end'
uint32_t losantGridEyeGetFrameMaxPosition(uint32_t start, uint32_t end);

// Function to get the Maximum value from 'start' pixel to the 'end' pixel in the frame
uint16_t losantGridEyeGetFrameMaxValue(uint32_t start, uint32_t end);

// RETURNS: total sum of all pixel values from 'start' to 'end', inclusive
uint64_t losantGridEyeGetFrameTotal(uint32_t start, uint32_t end);

//RETURNS : the temperature variable
uint16_t losantGridEyeGetTherm(void);

// Function to detect if there are heat signatures greater than a Threshold
// Threshold is defined as 'BLOB_THRESHOLD' 
// RETURNS : average blob intensity around max
uint32_t losantGridEyeDetectHeatBlobs(void);

// Function to detect if there were reading errors in some pixels
// Replaces them with an aggregate value
void losantGridEyeMitigateReadingErrors(void);

// Function to get the total of a particular column 
// at a particular side of the frame
// Can be used to determine if a person was walking through the side
// So that the system is still able to count even if the ToF misses the data
// Takes the column number as an input. ranging between 0 to 7
// RETURNS : the total at the column
uint32_t losantGridEyeGetColumnTotal(uint8_t column);

// Function to get trimmed frame
// Returns a rectangular frame given the top left and the bottom right coordinates
// TODO: give example
// Note : TopLeftX < BottomLeftX and TopLeftY < BottomLeftY 
//       Otherwise the function will return invalid results, or a NULL matrix
uint16_t * LosGridEyeGetTrimmedFrame(void);

// Function to print the trimmed frame
void LosGridEyePrintTrimmedFrame(void);

// Function to get the number of rows in a trimmed grid
uint8_t LosGridEyeGetTrimmedRows(uint32_t start, uint32_t end);

// Function to get the regional upper threshold for a trimmed grid
uint32_t LosGridEyeGetRoiUpperThresholdTrimmed(float average, uint32_t start, uint32_t end);

// Function to get a regional Lower threshold for a trimmed grid
uint32_t LosGridEyeGetRoiLowerThresholdTrimmed(float average, uint32_t start, uint32_t end);

// Function to get the total of a region defined by start and end in the trimmed frame
uint32_t LosGridEyeGetTrimmedFrameTotal(uint32_t start, uint32_t end);

// Function to get the average of a region defined by start and end in the trimmed frame
float LosGridEyeGetTrimmedFrameAverage(uint32_t start, uint32_t end);

// Function to give the total of the difference between the average pixel reading and pixels that are threshold 'pixelThreshold'
// above average 'pixelAverage'
float LosGridEyeGetPixelDifference(uint32_t start, uint32_t end, uint16_t pixelThreshold, float pixelAverage);

#endif // not _LOSANTGRIDEYE_H_
