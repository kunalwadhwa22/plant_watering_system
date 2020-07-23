/*
File to Get frames and perform basic Functions on a gridEye 'grid'
Consists of basic methods like getting total, max, averages for regions
Also has logic to set Upper and Lower Thresholds on the gridEye frame for Counting People
 */

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

#include "sensorGridEye.h"

float AVERAGE;
float AVERAGE_TRIMMED;
uint32_t ROI_UPPER_THRESHOLD;        // Upper Threshold value to change the roi (state) value
uint32_t ROI_LOWER_THRESHOLD;        // Lower Threshold value to change the roi (state) value

uint32_t Roi_Upper_Threshold_Trimmed; // Upper Th. value to change the roi (state) value for a trimmed grid
uint32_t Roi_Lower_Threshold_Trimmed; // Lower Th. value to change the roi (state) value for a trimmed grid

uint32_t THRESHOLD_UPPER_ROW = 12;		// Threshold upper for a row of 8 pixels
uint32_t THRESHOLD_LOWER_ROW = 9;		  // Threshold lower for a row of 8 pixels

// Upper and lower thresholds for trimmed frames
// TODO: Modify the thresholds so that they are calculated using factors like height etc
uint32_t Threshold_Upper_Row_Trimmed = 14;  // Threshold Upper for a single row of 'trimmed' grid
uint32_t Threshold_Lower_Row_Trimmed = 10;    // Threshold Lower for a single row of 'trimmed' grid

static BWBus *m_pComm;
//static BWBus pComm = 0xD2;
//static bool m_bigEndian;

uint16_t frame;
uint16_t frame_1[64];
uint16_t trimmedFrame[(TRIMMED_X_RANGE) * (TRIMMED_Y_RANGE)];
uint16_t temp;  // variable to which THERM_ADDRESS value is stored
uint16_t tempFrame[8][8];
uint16_t frame2D[8][8];
uint16_t columnTotal[8]; // total of each column (side ways)

uint16_t pixelThresholdLower = 20;
uint16_t pixelThresholUpper = 200;

bool losantGridEyeInit()
{
  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexTake(BWBOARD_I2C_PORTNUM_GRIDEYE, 0);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2

  // Initializing I2C handle
  BWBusMode i2cBusModeWithSubAddr = { 1, 0 }; // I2C: using 1-byte subaddress
  m_pComm = BWSerialI2CCreate(BWBOARD_I2C_PORTNUM_GRIDEYE,
                                SENSORPIN_I2C_SCL_0, SENSORPIN_I2C_SDA_0,
                                BWBOARD_I2C_AMG8333_SLAVE_ADDR, 
                                &i2cBusModeWithSubAddr,
                                BWBOARD_I2C0_BUS_CLOCK_RATE_HZ,
                                1000,0);
  
  if(!m_pComm)
  {
    printf("m_pcomm failed\n");
    #if LOSDOORMON_MB_HW_VERSION >= 2
    LosBoardI2cMutexGive(BWBOARD_I2C_PORTNUM_GRIDEYE);
    #endif // LOSDOORMON_MB_HW_VERSION >= 2
    return false;
  }
  
  // TODO : define what the inputs mean
  uint8_t input_1 = 0x3F;
  uint8_t input_2 = 0x00;
  uint8_t input_3 = 0x00;
  
  // Setting modes
  // TODO : define what modes are being set exactly

  BWBusXferLength bytesWritten1 = BWBusMasterWrite8(m_pComm, 0x01, input_1);
  
  if (bytesWritten1 != 2) {
    printf("Write at block 1 fail, %d\n", bytesWritten1);
    //return false; // FAIL
  }
  
  BWBusXferLength bytesWritten2 = BWBusMasterWrite8(m_pComm, 0x00, input_2);
  
   
  if (bytesWritten2 != 2) {
    printf("Write at block 2 fail\n");
    //return false; // FAIL
  }
  
  BWBusXferLength bytesWritten3 = BWBusMasterWrite8(m_pComm, 0x02, input_3);
  
  if (bytesWritten3 != 2) {
    printf("Write at block 3 fail\n");
    //return false; // FAIL
  }
  
  // Reading the first frame
  for(int i=0; i<64; i++)
  {
    BWBusMasterRead16(m_pComm, DATA_OFFSET+i+i, &frame);
    frame_1[i] = frame;
  }

  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexGive(BWBOARD_I2C_PORTNUM_GRIDEYE);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2

  // Take a stable reading here and set the AVERAGE value 
  while(!losantGridEyeGetStable())
  {
  }
  return true;
}

uint16_t * losantGridEyeGetFrame(void)
{

  int k=0;

  uint32_t localColumnTotal;    // variable to store local column totals after each read

  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexTake(BWBOARD_I2C_PORTNUM_GRIDEYE, 0);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2

  // reading the I2C lines 64 times for each pixel
  for(int i=0; i<8; i++)
  {
    // Initializing the column totals to 0 after reading every column
    localColumnTotal = 0;


    for(int j=0; j<8; j++)
    {
      BWBusMasterRead16(m_pComm, DATA_OFFSET+k+k, &frame);
      tempFrame[i][j] = frame;
      localColumnTotal += tempFrame[i][j];
      k++;
    }
    columnTotal[i] = localColumnTotal;
  }

  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexGive(BWBOARD_I2C_PORTNUM_GRIDEYE);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2
  
  // Normalizing any misreads from the grid Eye
  // Some pixels always get a very abrupt misread.
  // They are replaced by the average value of the frame.
  losantGridEyeMitigateReadingErrors();
  
  // Not
  // Taking the transpose to rectify the matrix according to the orientation0
  // TODO : implement an orientation input to see if this is necessary in every installation

  // To take the transpose, replace i,j by j,i
  k=0;
  for(int i=0; i<8; i++)
  {
    for(int j=0; j<8; j++)
    {
      frame_1[k] = tempFrame[i][j];
      frame2D[i][j] = tempFrame[i][j];
      k++;
    }
  }

  return frame_1;
}

uint16_t * LosGridEyeGetTrimmedFrame(void)
{
  uint8_t topLeftX = TRIMMED_TOP_LEFT_X;
  uint8_t bottomRightX = TRIMMED_BOTTOM_RIGHT_X;
  uint8_t topLeftY = TRIMMED_TOP_LEFT_Y;
  uint8_t bottomRightY = TRIMMED_BOTTOM_RIGHT_Y;
    
  if(topLeftX >= bottomRightX || topLeftY > bottomRightY)
  {
    BWLogErr("Error\n");
    return NULL;
  }

  uint8_t xLimit = bottomRightX - topLeftX + 1;
  uint8_t yLimit = bottomRightY - topLeftY + 1;
  int k=0;
  for(int i=0; i<64; i++)
  {
    if((i/8 >= topLeftX) && (i/8 <= bottomRightX) && (i%8 >= topLeftY) && (i%8 <= bottomRightY))
    {
      trimmedFrame[k] = frame_1[i];
      k++;
    }
  }

  return trimmedFrame;
}

void losantGridEyePrintFrame(void)
{
  for(int i=0; i<64; i++)
  {
    printf("%d ",frame_1[i]);
    if((i+1)%8==0)
    {
      printf("\n");
    }
  }
  printf("\n");
}

void LosGridEyePrintTrimmedFrame(void)
{
  for(int i=0; i< TRIMMED_X_RANGE * TRIMMED_Y_RANGE; i++)
  {
    printf("%d ", trimmedFrame[i]);
    if((i+1) % TRIMMED_Y_RANGE == 0)
    {
      printf("\n");
    }
  }
  printf("\n");
}

uint16_t losantGridEyeGetTherm(void)
{
  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexTake(BWBOARD_I2C_PORTNUM_GRIDEYE, 0);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2

  BWBusMasterRead16(m_pComm, THERM_ADDRESS, &temp);

  #if LOSDOORMON_MB_HW_VERSION >= 2
  LosBoardI2cMutexGive(BWBOARD_I2C_PORTNUM_GRIDEYE);
  #endif // LOSDOORMON_MB_HW_VERSION >= 2

  return temp;
}

bool losantGridEyeGetStable(void)
{
  int new_average=0;
	int previous_average=-1;
  // check the integer values of average to see if they repeat 
  // over 2 back to back iterations to check if they are stable
	while(previous_average!=new_average)
	{
		losantGridEyeGetFrame();
		previous_average=new_average;
		new_average=losantGridEyeGetFrameAverage(GRID_START, GRID_END);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
  AVERAGE = new_average;
	BWLogInfo("Average pixel Intensity for Grid Eye is %d \n", (int)AVERAGE);
	return true;
}

uint32_t losantGridEyeGetRows(uint32_t start, uint32_t end)
{
  uint32_t rows;
  rows = (end - start)/8;
  return rows;
}

uint8_t LosGridEyeGetTrimmedRows(uint32_t start, uint32_t end)
{
  uint8_t trimmedRows;
  trimmedRows = (end - start)/TRIMMED_Y_RANGE;
  return trimmedRows;
}

uint32_t losantGridEyeGetRoiUpperThreshold(float average, uint32_t start, uint32_t end)
{
  // Multiply the regional average over all the pixels
  ROI_UPPER_THRESHOLD = average * (end - start);
  
  // Add the product in the previous iteration to the product of 
  //    number of rows in the region and threshold for each row
  ROI_UPPER_THRESHOLD = ROI_UPPER_THRESHOLD + (LosGridEyeGetTrimmedRows(start, end) * THRESHOLD_UPPER_ROW);
	return ROI_UPPER_THRESHOLD;
}

uint32_t LosGridEyeGetRoiUpperThresholdTrimmed(float average, uint32_t start, uint32_t end)
{
  // Multiply the regional average over all the pixels
  Roi_Upper_Threshold_Trimmed = average * (end - start);
  
  // Add the product in the previous iteration to the product of 
  //    number of rows in the region and threshold for each row
  Roi_Upper_Threshold_Trimmed = Roi_Upper_Threshold_Trimmed + (LosGridEyeGetTrimmedRows(start, end) * Threshold_Upper_Row_Trimmed);
	return Roi_Upper_Threshold_Trimmed;
}

uint32_t losantGridEyeGetRoiLowerThreshold(float average, uint32_t start, uint32_t end)
{
  // Multiply the regional average over all the pixels
  ROI_LOWER_THRESHOLD = average * (end - start);
  
  // Add the product in the previous iteration to the product of 
  //    number of rows in the region and threshold for each row
  ROI_LOWER_THRESHOLD = ROI_LOWER_THRESHOLD + (losantGridEyeGetRows(start, end) * THRESHOLD_LOWER_ROW);
	return ROI_LOWER_THRESHOLD;
}

uint32_t LosGridEyeGetRoiLowerThresholdTrimmed(float average, uint32_t start, uint32_t end)
{
  Roi_Lower_Threshold_Trimmed = average * (end - start);
  
  // Add the product in the previous iteration to the product of 
  //    number of rows in the region and threshold for each row
  Roi_Lower_Threshold_Trimmed = Roi_Lower_Threshold_Trimmed + (losantGridEyeGetRows(start, end) * Threshold_Lower_Row_Trimmed);
	return Roi_Lower_Threshold_Trimmed;
}

float losantGridEyeGetFrameAverage(uint32_t start, uint32_t end)
{
  float average;
	average = losantGridEyeGetFrameTotal(start,end);
  average = average/(end-start);
	return average;
}

float LosGridEyeGetTrimmedFrameAverage(uint32_t start, uint32_t end)
{
  float trimmedAverage;
  trimmedAverage = LosGridEyeGetTrimmedFrameTotal(start,end);
  trimmedAverage = trimmedAverage/(end-start);
  return trimmedAverage;
}

uint32_t losantGridEyeGetFrameMaxPosition(uint32_t start, uint32_t end)
{
  uint32_t max_position = start;
  // iterate over all pixels to get the position of the max pixel
	for(int i=0; i<64; i++)
  {
    if(frame_1[max_position] < frame_1[i])
    {
      max_position = i;
    }
  }
	return max_position;  
}

uint16_t losantGridEyeGetFrameMaxValue(uint32_t start, uint32_t end)
{
  uint32_t max = 0;
  // iterate over all pixels to get the pixel with the max value
	for(int i=0; i<64; i++)
  {
    if(frame_1[i] > max)
    {
      max = frame_1[i];
    }
  }
	return max; 
}

// Adds all the rows and columns from start to end
 // total is *not* an average
uint64_t losantGridEyeGetFrameTotal(uint32_t start, uint32_t end)
{
  uint64_t total = 0;
  // iterate from start to end to get the total over the pixels
	for(int i=start; i<end; i++)
  {
    total += frame_1[i];
  }
	return total; 
}

uint32_t LosGridEyeGetTrimmedFrameTotal(uint32_t start, uint32_t end)
{
    uint64_t trimmedTotal = 0;
    for(int i = start; i < end; i++)
    {
      trimmedTotal += trimmedFrame[i];
    }
    return trimmedTotal;
}

uint32_t losantGridEyeDetectHeatBlobs(void)
{
  // get the position of the maximum intensity
  uint32_t myMaxPosition = losantGridEyeGetFrameMaxPosition(GRID_START, GRID_END);
  uint8_t countOfFramesAdded = 0;
  int16_t xAxis = 0, yAxis = 0;
  
  // translate the position from 1D to 2D
  xAxis = myMaxPosition / 8;
  yAxis = myMaxPosition % 8;
  uint32_t blobTotal = 0;
  uint32_t averageBlobIntensity;

  // Iterate for the nearest 8 pixels (or 5 if on the edge) to
  // get the total around the maximum
  // If the pixel is on the edge, the 3 pixels which are virtually
  // out of frame are ignored
  for(int i = xAxis - 1; i <= xAxis + 1; i++)
  {
    for(int j = yAxis - 1; j <= yAxis + 1; j++)
    {
      if(j >= 0 && j <= 7 && i >= 0 && i <=7)
      {
        countOfFramesAdded++;
        blobTotal += frame2D[i][j];
      }
    }
  }

  // get the average intensity around the pixel of maximum intensity
  averageBlobIntensity = blobTotal/countOfFramesAdded;
  return averageBlobIntensity;
}

void losantGridEyeMitigateReadingErrors(void)
{
  // Iterate over the frame
  // replace the pixel by the average value of the grid if there is a misread
  // misread is an obviously high/low value
  for(int i=0; i<64; i++)
  {
    if(frame_1[i] < pixelThresholdLower || frame_1[i] > pixelThresholUpper)
    {
      frame_1[i] = AVERAGE;
    } 
  }
}

uint32_t losantGridEyeGetColumnTotal(uint8_t column)
{
  return columnTotal[column ];
}

// API that gives the difference between Total and regional Threshold
//  gives the difference of the calculation
float LosGridEyeGetPixelDifference(uint32_t start, uint32_t end, uint16_t pixelThreshold, float pixelAverage)
{
  float totalUp = 0;
  for(int i=start; i<end; i++)
  {
    if(trimmedFrame[i] > (pixelAverage + pixelThreshold))
    {
      totalUp = totalUp + trimmedFrame[i] - pixelAverage;
    }
  }
  if(totalUp < (3 * pixelThreshold))
    return 0;
  return totalUp;
}

