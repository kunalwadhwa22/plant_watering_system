#include "sensorMLX90640.h"

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

// Register definitions
#define STATUS_REGISTER_ADDRESS     0x8000
#define CONTROL_REGISTER_1_ADDRESS   0x800D

#define NUMBER_OF_PIXELS  832

uint16_t statusRegister;    // global to store status register for MLX90640
uint16_t controlRegister1;   // global to store config register 1 for MLX90640

uint16_t statusRegisterAddress = STATUS_REGISTER_ADDRESS;
uint16_t controlRegister1Address = CONTROL_REGISTER_1_ADDRESS;

uint16_t numberOfPixels = NUMBER_OF_PIXELS;
uint16_t pixel1[NUMBER_OF_PIXELS];

// Note : The endianess of the register is in place with the endianess in the datasheet
// ESP32 is opposite in that case. 
// Therefore the directly read values are reversed and need correction

static uint16_t frame_0[832];
static uint16_t eeData[832];
static float mlx90640To[768];

static paramsMLX90640 mlx90640;

static BWBus *m_pComm;
static float Ta;


// Static / internal declarations

void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *mlx90640);
int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);  
float GetMedian(float *values, int n);
int IsPixelBad(uint16_t pixel,paramsMLX90640 *params);

// Static declarations end

void PrintDesiredData(void)
{
  SensorPrintToFrameMLX90640(mlx90640To,32,24);
}

void SensorRunLoopMLX90640(void)
{

  // 1

  //SensorGetFrameMLX90640(frame_0);
  SensorGetFrameMLX90640(frame_0);
  ChangeEndianessArrayUnsignedInteger16(frame_0, 32*26);

  // SensorDumpEEMLX90640(eeData);
  // ChangeEndianessArrayUnsignedInteger16(eeData, 32*26);
  // SensorExtractParametersMLX90640(eeData, &mlx90640);

  Ta = SensorGetTaMLX90640(frame_0, &mlx90640);
  //printf("Ta from the sensor : %f \n", Ta);
  SensorCalculateObjectTemperatureMLX90640(frame_0, &mlx90640, 0.95, Ta, mlx90640To);
  //SensorPrintToFrameBinaryMLX90640(mlx90640To,32,24,30);
  //SensorPrintToFrameMLX90640(mlx90640To,32,24);

  //2

  SensorGetFrameMLX90640(frame_0);
  ChangeEndianessArrayUnsignedInteger16(frame_0, 32*26);

  // SensorDumpEEMLX90640(eeData);
  // ChangeEndianessArrayUnsignedInteger16(eeData, 32*26);
  // SensorExtractParametersMLX90640(eeData, &mlx90640);

  Ta = SensorGetTaMLX90640(frame_0, &mlx90640);
  //printf("Ta from the sensor : %f \n", Ta);
  SensorCalculateObjectTemperatureMLX90640(frame_0, &mlx90640, 0.95, Ta, mlx90640To);

  // SensorGetFrameMLX90640(frame_0);
  // ChangeEndianessArrayUnsignedInteger16(frame_0, 32*26);

  // // SensorDumpEEMLX90640(eeData);
  // // ChangeEndianessArrayUnsignedInteger16(eeData, 32*26);
  // // SensorExtractParametersMLX90640(eeData, &mlx90640);

  // Ta = SensorGetTaMLX90640(frame_0, &mlx90640);
  // //printf("Ta from the sensor : %f \n", Ta);
  // SensorCalculateObjectTemperatureMLX90640(frame_0, &mlx90640, 0.95, Ta, mlx90640To);
  // //SensorPrintToFrameBinaryMLX90640(mlx90640To,32,24,30);
  // SensorPrintToFrameMLX90640(mlx90640To,32,24);


  // // get average left
  // float leftAverage =  GetFrameAverage(LEFT_START, LEFT_END, mlx90640To);

  // // get average middle
  // float middleAverage = GetFrameAverage(MIDDLE_START, MIDDLE_END, mlx90640To);

  // // get average right
  // float rightAverage = GetFrameAverage(RIGHT_START, RIGHT_END, mlx90640To);

  // float leftAboveThTotal = GetFrameTotalOfPixelsAboveThreshold(LEFT_START, LEFT_END, mlx90640To, 31);

  // float middleAboveThTotal = GetFrameTotalOfPixelsAboveThreshold(MIDDLE_START, MIDDLE_END, mlx90640To, 31);

  // float rightAboveThTotal = GetFrameTotalOfPixelsAboveThreshold(RIGHT_START, RIGHT_END, mlx90640To, 31);
    
  // printf("%f, %f, %f, \t %f, %f, %f \r\n", leftAverage, middleAverage, rightAverage,
  //       leftAboveThTotal, middleAboveThTotal, rightAboveThTotal);

}



bool SensorInitMLX90640(void)
{
  BWBusMode i2cBusModeWithSubAddr = { 2, 0 }; // I2C: using 2-byte subaddress
  BWDelayMS(1000);
  m_pComm = BWSerialI2CCreate(BWBOARD_I2C_PORTNUM_MLX90640,
                                SENSORPIN_I2C_SCL_1, SENSORPIN_I2C_SDA_1,
                                BWBOARD_I2C_MLX90640_SLAVE_ADDR, 
                                &i2cBusModeWithSubAddr,
                                400000,
                                1000,0);
  
  if(!m_pComm)
  {
    printf("m_pcomm failed\n");
    return false;
  }

  BWDelayMS(1000);

  controlRegister1 = SensorGetControlRegister1MLX90640();
  printf("\n Control Register Contents : %x \n ", controlRegister1);

  printf("Refresh Rate using function : %d\n" ,SensorGetRefreshRateMLX90640());

  SensorSetRefreshRateMLX90640(0x0007);

  controlRegister1 = SensorGetControlRegister1MLX90640();
  printf("New Control Register Contents : %x \n ", controlRegister1);

  printf("New Refresh Rate using function : %d\n" ,SensorGetRefreshRateMLX90640());

  BWDelayMS(1000);

  // getting EEPROM readings, change endianess and print
  SensorDumpEEMLX90640(eeData);
  ChangeEndianessArrayUnsignedInteger16(eeData, 32*26);
  SensorPrintFrameMLX90640(eeData, 32, 26);
  SensorExtractParametersMLX90640(eeData, &mlx90640);
   Ta = SensorGetTaMLX90640(frame_0, &mlx90640);


  for(int i = 0; i < 10; i++)
  {
    SensorGetFrameMLX90640(frame_0);
    SensorGetFrameMLX90640(frame_0);
    //BWDelayMS(1000);
  }

  return true;
}

bool SensorGetFrameMLX90640(uint16_t * frame)
{
  // check if the sensor has updated
  bool dataReady = false;

  while(!dataReady)
  {
    dataReady = SensorIsNewDataReadyMLX90640(true);
  }

  // allow the sensor to overwrite RAM
  uint8_t count = 0;
  //while(count < 5 && dataReady != 0)
  //{
    uint16_t dataToWrite = 0x1000;

    BWBusXferLength bytesWrittenRAMOverwrite = BWBusMasterWrite16(m_pComm, statusRegisterAddress, dataToWrite);
    
    if (bytesWrittenRAMOverwrite != 4) {
      printf("RAM Overwrite write failed, bytes written : %d\n", bytesWrittenRAMOverwrite);
      //return false; // FAIL
    }

    // read from the sensor
    // BWBusMasterRead(m_pComm, 0x0400, frameToRead, 832 * 2);

    //BWBusMasterRead16(m_pComm, 0x0400, &pixel1);

    // BWBusMasterRead(m_pComm, 0x0400, &frame_0[pixelLower], 832 * 2);         // pixels 0 - 127
    // pixelLower += i2cHardwareBufferLimitWords;

    BWBusMasterRead(m_pComm, 0x0400, &frame[0], 832 * 2);         // pixels 0 - 127

    // BWBusMasterRead(m_pComm, 0x0480, &frame_0[pixelLower], 128 * 2);    // pixels 128 - 255
    // pixelLower += i2cHardwareBufferLimitWords;

    // BWBusMasterRead(m_pComm, 0x0500, &frame_0[pixelLower], 128 * 2);    // pixels 256 - 383
    // pixelLower += i2cHardwareBufferLimitWords;

    // BWBusMasterRead(m_pComm, 0x0580, &frame_0[pixelLower], 128 * 2);    // pixels 384 - 511
    // pixelLower += i2cHardwareBufferLimitWords;

    // BWBusMasterRead(m_pComm, 0x0600, &frame_0[pixelLower], 128 * 2);    // pixels 512 - 639
    // pixelLower += i2cHardwareBufferLimitWords;

    // BWBusMasterRead(m_pComm, 0x0680, &frame_0[pixelLower], 128 * 2);    // pixels 640 - 767
    // pixelLower += i2cHardwareBufferLimitWords;

    // BWBusMasterRead(m_pComm, 0x0700, &frame_0[pixelLower], 64 * 2);    // calibration pixels
    // pixelLower += i2cHardwareBufferLimitWords;

    // printf("Frame read pixel 1 : %d \n", frameToRead[0]);

    count++;
    dataReady = SensorIsNewDataReadyMLX90640(true);
  //}

  if(count < 4)
  {
    //printf("Successful\n");
  }
  // see what sub-frame was polled
  //printf("Sub page polled : %d \n", SensorGetCurrentSubFrameMLX90640(false));

  // if the sub frame was toggled, we were able to read

  // temporary print
  // SensorPrintFrameMLX90640();

  return false;
}


void SensorSetRefreshRateMLX90640(uint8_t refreshRate)
{
  uint16_t controlRegister1;
  uint16_t value;

  value = (refreshRate & 0x07) << 7;

  BWBusXferLength bytesReadRefreshRate = BWBusMasterRead16(m_pComm, 0x800D, &controlRegister1);

  if (bytesReadRefreshRate != 2) {
    printf("Reading error \n");
    return;
  }

  uint16_t controlRegister2 = 0;

  controlRegister2 = ((controlRegister1 << 8) & 0xff00) | ((controlRegister1 >> 8) & 0x00ff);

  value = (controlRegister2 & 0xFC7F) | value;

  uint16_t value_to_write = 0;

  value_to_write = ((value << 8) & 0xff00) | ((value >> 8) & 0x00ff);
    /// writing refresh rate 

  BWBusXferLength bytesWrittenRefreshRate = BWBusMasterWrite16(m_pComm, 0x800D, value_to_write);
  
  if (bytesWrittenRefreshRate != 4) {
    printf("refresh rate block fail , bytes written : %d\n", bytesWrittenRefreshRate);
    //return false; // FAIL
  }
}

int16_t SensorGetRefreshRateMLX90640(void)
{
  uint16_t refreshRate = 0;

  BWBusXferLength bytesReadRefreshRate = BWBusMasterRead16(m_pComm, 0x800D, &refreshRate);

  refreshRate = ChangeEndianessUnsignedInteger16(refreshRate);

  if (bytesReadRefreshRate != 2) {
    printf("Refresh Rate Reading error \n");
    return -1; // FAIL
  }

  uint16_t rr = 0;

  rr = (refreshRate & 0x0380) >> 7;
  printf("Refresh rate read Out of the sensor is %d \n", rr);

  return rr;
}

bool SensorIsNewDataReadyMLX90640(bool reReadStatusRegister)
{

  if(reReadStatusRegister)
  {
    statusRegister = SensorGetStatusRegisterMLX90640();
  }

  if((statusRegister & 0x0008) != 0)
    return true;

  return false;
}


uint16_t SensorGetControlRegister1MLX90640(void)
{
  uint16_t localControlRegister1 = 0x0000;
  
  BWBusXferLength bytesReadRefreshRate = BWBusMasterRead16(m_pComm, controlRegister1Address, &localControlRegister1);

  if (bytesReadRefreshRate != 2) {
    printf("Reading error \n");
    return 0x0000;
  }

  localControlRegister1 = ChangeEndianessUnsignedInteger16(localControlRegister1);
  return localControlRegister1;
}

uint16_t SensorGetStatusRegisterMLX90640(void)
{
  uint16_t localStatusRegister = 0x0000;
  
  // reading the status register address
  BWBusXferLength bytesStatusRegister = BWBusMasterRead16(m_pComm, statusRegisterAddress, &localStatusRegister);

  if (bytesStatusRegister != 2) {
    printf("Status register reading error \n");
  }

  localStatusRegister = ChangeEndianessUnsignedInteger16(localStatusRegister);

  //printf("Status Register Contents %4x\n", localStatusRegister);

  return localStatusRegister;
}

uint8_t SensorGetCurrentSubFrameMLX90640(bool reReadStatusRegister)
{
  if(reReadStatusRegister)
  {
    statusRegister = SensorGetStatusRegisterMLX90640();  
  }

  uint8_t subFramePolled = (statusRegister & 0x0007);
  return subFramePolled;
}

void SensorPrintFrameMLX90640(uint16_t * frame, uint8_t rows, uint8_t columns)
{
  int k = 0;
  for(int i = 0; i < columns; i++)
  {
    for(int j = 0; j < rows; j++)
    {
      printf("%4x\t", frame[k]);
      k++;
    }
    printf("\n");
  }  
  printf("\n\n");
}

void SensorPrintToFrameMLX90640(float * frame, uint8_t rows, uint8_t columns)
{
  printf("Data:\r\n");
  int k = 0;
  for(int i = 0; i < columns; i++)
  {
    for(int j = 0; j < rows; j++)
    {
      printf("%2.0f ", frame[k]);
      k++;
    }
    printf("\n");
  }  
  printf("\r\n\n");
}

void SensorPrintToFrameBinaryMLX90640(float * frame, uint8_t rows, uint8_t columns, uint16_t threshold)
{
  int k = 0;
  for(int i = 0; i < columns; i++)
  {
    for(int j = 0; j < rows; j++)
    {
      if(frame[k] < threshold)
        printf("- ");
      else
        printf("* ");
      k++;
    }
    printf("\n");
  }  
  printf("\n\n");
}

///////////////////

bool SensorDumpEEMLX90640(uint16_t * m_eeData)
{
  BWBusMasterRead(m_pComm, 0x2400, &m_eeData[0], 832 * 2);         // pixels 0 - 127
  return true;
}

int SensorExtractParametersMLX90640(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int error = 0;
    
    ExtractVDDParameters(eeData, mlx90640);
    printf(" KVDD %d \n" , mlx90640->kVdd);
    printf(" Vdd25 %d \n" , mlx90640->vdd25); 
    ExtractPTATParameters(eeData, mlx90640);
    ExtractGainParameters(eeData, mlx90640);
    ExtractTgcParameters(eeData, mlx90640);
    ExtractResolutionParameters(eeData, mlx90640);
    ExtractKsTaParameters(eeData, mlx90640);
    ExtractKsToParameters(eeData, mlx90640);
    ExtractCPParameters(eeData, mlx90640);
    ExtractAlphaParameters(eeData, mlx90640);
    ExtractOffsetParameters(eeData, mlx90640);
    ExtractKtaPixelParameters(eeData, mlx90640);
    ExtractKvPixelParameters(eeData, mlx90640);
    ExtractCILCParameters(eeData, mlx90640);
    error = ExtractDeviatingPixels(eeData, mlx90640);  
    
    return error;

}


float SensorGetVddMLX90640(uint16_t *frameData, paramsMLX90640 *params)
{
    float vdd;
    float resolutionCorrection;

    int resolutionRAM;    
    
    vdd = frameData[810];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolutionRAM = (controlRegister1 & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)params->resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - params->vdd25) / params->kVdd + 3.3;
    
    return vdd;
}

float SensorGetTaMLX90640(uint16_t *frameData, paramsMLX90640 *params)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = SensorGetVddMLX90640(frameData, params);
    
    ptat = frameData[800];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptatArt = frameData[768];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + params->KvPTAT * (vdd - 3.3)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25;
    
    return ta;
}

void SensorCalculateObjectTemperatureMLX90640(uint16_t *frameData, paramsMLX90640 *params, float emissivity, float tr, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = statusRegister & 0x0001;
    vdd = SensorGetVddMLX90640(frameData, params);
    ta = SensorGetTaMLX90640(frameData, params);
    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    alphaScale = pow(2,(double)params->alphaScale);
    
    alphaCorrR[0] = 1 / (1 + params->ksTo[0] * 40);
    alphaCorrR[1] = 1 ;
    alphaCorrR[2] = (1 + params->ksTo[1] * params->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + params->ksTo[2] * (params->ct[3] - params->ct[2]));
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[778];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- To calculation -------------------------------------    
    mode = (controlRegister1 & 0x1000) >> 5;
    
    irDataCP[0] = frameData[776];  
    irDataCP[1] = frameData[808];
    for( int i = 0; i < 2; i++)
    {
        if(irDataCP[i] > 32767)
        {
            irDataCP[i] = irDataCP[i] - 65536;
        }
        irDataCP[i] = irDataCP[i] * gain;
    }
    irDataCP[0] = irDataCP[0] - params->cpOffset[0] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    if( mode ==  params->calibrationModeEE)
    {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }
    else
    {
      irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }

    for( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
    {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2; 
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2); 
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        
        if(mode == 0)
        {
          pattern = ilPattern; 
        }
        else 
        {
          pattern = chessPattern; 
        }               
        
        if(pattern == (statusRegister & 0x0001))
        {    
            irData = frameData[pixelNumber];
            if(irData > 32767)
            {
                irData = irData - 65536;
            }
            irData = irData * gain;
            
            kta = params->kta[pixelNumber]/ktaScale;
            kv = params->kv[pixelNumber]/kvScale;
            irData = irData - params->offset[pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));
            
            if(mode !=  params->calibrationModeEE)
            {
              irData = irData + params->ilChessC[2] * (2 * ilPattern - 1) - params->ilChessC[1] * conversionPattern; 
            }                       
    
            irData = irData - params->tgc * irDataCP[subPage];
            irData = irData / emissivity;
            
            alphaCompensated = SCALEALPHA*alphaScale/params->alpha[pixelNumber];
            alphaCompensated = alphaCompensated*(1 + params->KsTa * (ta - 25));
                        
            Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            Sx = sqrt(sqrt(Sx)) * params->ksTo[1];            
            
            To = sqrt(sqrt(irData/(alphaCompensated * (1 - params->ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;                     
                    
            if(To < params->ct[1])
            {
                range = 0;
            }
            else if(To < params->ct[2])   
            {
                range = 1;            
            }   
            else if(To < params->ct[3])
            {
                range = 2;            
            }
            else
            {
                range = 3;            
            }      
            
            To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15;
                        
            result[pixelNumber] = To;
        }
    }
}


// Helper to change endianness
uint16_t ChangeEndianessUnsignedInteger16(uint16_t input)
{
  uint16_t output;
  output = ((input << 8) & 0xff00) | ((input >> 8) & 0x00ff);
  return output;
}

void ChangeEndianessArrayUnsignedInteger16(uint16_t * frame, uint16_t length)
{
  // change endianess of the data
  for(int i = 0; i < length; i++)
  {
    frame[i] = ChangeEndianessUnsignedInteger16(frame[i]);
  }
}

// Static / In file declared functions definitions

void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[51];
    
    kVdd = (eeData[51] & 0xFF00) >> 8;
    if(kVdd > 127)
    {
        kVdd = kVdd - 256;
    }
    kVdd = 32 * kVdd;
    vdd25 = eeData[51] & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;
    
    mlx90640->kVdd = kVdd;
    mlx90640->vdd25 = vdd25; 
}

//------------------------------------------------------------------------------

void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = (eeData[50] & 0xFC00) >> 10;
    if(KvPTAT > 31)
    {
        KvPTAT = KvPTAT - 64;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData[50] & 0x03FF;
    if(KtPTAT > 511)
    {
        KtPTAT = KtPTAT - 1024;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = eeData[49];
    
    alphaPTAT = (eeData[16] & 0xF000) / pow(2, (double)14) + 8.0f;
    
    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;    
    mlx90640->vPTAT25 = vPTAT25;
    mlx90640->alphaPTAT = alphaPTAT;   
}

//------------------------------------------------------------------------------

void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int16_t gainEE;
    
    gainEE = eeData[48];
    if(gainEE > 32767)
    {
        gainEE = gainEE -65536;
    }
    
    mlx90640->gainEE = gainEE;    
}

//------------------------------------------------------------------------------

void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float tgc;
    tgc = eeData[60] & 0x00FF;
    if(tgc > 127)
    {
        tgc = tgc - 256;
    }
    tgc = tgc / 32.0f;
    
    mlx90640->tgc = tgc;        
}

//------------------------------------------------------------------------------

void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    uint8_t resolutionEE;
    resolutionEE = (eeData[56] & 0x3000) >> 12;    
    
    mlx90640->resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float KsTa;
    KsTa = (eeData[60] & 0xFF00) >> 8;
    if(KsTa > 127)
    {
        KsTa = KsTa -256;
    }
    KsTa = KsTa / 8192.0f;
    
    mlx90640->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int KsToScale;
    int8_t step;
    
    step = ((eeData[63] & 0x3000) >> 12) * 10;
    
    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = (eeData[63] & 0x00F0) >> 4;
    mlx90640->ct[3] = (eeData[63] & 0x0F00) >> 8;    
    
    mlx90640->ct[2] = mlx90640->ct[2]*step;
    mlx90640->ct[3] = mlx90640->ct[2] + mlx90640->ct[3]*step;
    mlx90640->ct[4] = 400;
    
    KsToScale = (eeData[63] & 0x000F) + 8;
    KsToScale = 1 << KsToScale;
    
    mlx90640->ksTo[0] = eeData[61] & 0x00FF;
    mlx90640->ksTo[1] = (eeData[61] & 0xFF00) >> 8;
    mlx90640->ksTo[2] = eeData[62] & 0x00FF;
    mlx90640->ksTo[3] = (eeData[62] & 0xFF00) >> 8;      
    
    for(int i = 0; i < 4; i++)
    {
        if(mlx90640->ksTo[i] > 127)
        {
            mlx90640->ksTo[i] = mlx90640->ksTo[i] - 256;
        }
        mlx90640->ksTo[i] = mlx90640->ksTo[i] / KsToScale;
    } 
    
    mlx90640->ksTo[4] = -0.0002;
}

//------------------------------------------------------------------------------

void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    float alphaTemp[768];
    float temp;
    

    accRemScale = eeData[32] & 0x000F;
    accColumnScale = (eeData[32] & 0x00F0) >> 4;
    accRowScale = (eeData[32] & 0x0F00) >> 8;
    alphaScale = ((eeData[32] & 0xF000) >> 12) + 30;
    alphaRef = eeData[33];
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        accRow[p + 0] = (eeData[34 + i] & 0x000F);
        accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
        accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
        accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (accRow[i] > 7)
        {
            accRow[i] = accRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        accColumn[p + 0] = (eeData[40 + i] & 0x000F);
        accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
        accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
        accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (accColumn[i] > 7)
        {
            accColumn[i] = accColumn[i] - 16;
        }
    }

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
            if (alphaTemp[p] > 31)
            {
                alphaTemp[p] = alphaTemp[p] - 64;
            }
            alphaTemp[p] = alphaTemp[p]*(1 << accRemScale);
            alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alphaTemp[p]);
            alphaTemp[p] = alphaTemp[p] / pow(2,(double)alphaScale);
            alphaTemp[p] = alphaTemp[p] - mlx90640->tgc * (mlx90640->cpAlpha[0] + mlx90640->cpAlpha[1])/2;
            alphaTemp[p] = SCALEALPHA/alphaTemp[p];
        }
    }
    
    temp = alphaTemp[0];
    for(int i = 1; i < 768; i++)
    {
        if (alphaTemp[i] > temp)
        {
            temp = alphaTemp[i];
        }
    }
    
    alphaScale = 0;
    while(temp < 32768)
    {
        temp = temp*2;
        alphaScale = alphaScale + 1;
    } 
    
    for(int i = 0; i < 768; i++)
    {
        temp = alphaTemp[i] * pow(2,(double)alphaScale);        
        mlx90640->alpha[i] = (temp + 0.5);        
        
    } 
    
    mlx90640->alphaScale = alphaScale;      
   
}

//------------------------------------------------------------------------------

void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;
    

    occRemScale = (eeData[16] & 0x000F);
    occColumnScale = (eeData[16] & 0x00F0) >> 4;
    occRowScale = (eeData[16] & 0x0F00) >> 8;
    offsetRef = eeData[17];
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        occRow[p + 0] = (eeData[18 + i] & 0x000F);
        occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
        occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
        occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (occRow[i] > 7)
        {
            occRow[i] = occRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        occColumn[p + 0] = (eeData[24 + i] & 0x000F);
        occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
        occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
        occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (occColumn[i] > 7)
        {
            occColumn[i] = occColumn[i] - 16;
        }
    }

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            mlx90640->offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
            if (mlx90640->offset[p] > 31)
            {
                mlx90640->offset[p] = mlx90640->offset[p] - 64;
            }
            mlx90640->offset[p] = mlx90640->offset[p]*(1 << occRemScale);
            mlx90640->offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + mlx90640->offset[p]);
        }
    }
}

//------------------------------------------------------------------------------

void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int p = 0;
    int8_t KtaRC[4];
    int8_t KtaRoCo;
    int8_t KtaRoCe;
    int8_t KtaReCo;
    int8_t KtaReCe;
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    float ktaTemp[768];
    float temp;
    
    KtaRoCo = (eeData[54] & 0xFF00) >> 8;
    if (KtaRoCo > 127)
    {
        KtaRoCo = KtaRoCo - 256;
    }
    KtaRC[0] = KtaRoCo;
    
    KtaReCo = (eeData[54] & 0x00FF);
    if (KtaReCo > 127)
    {
        KtaReCo = KtaReCo - 256;
    }
    KtaRC[2] = KtaReCo;
      
    KtaRoCe = (eeData[55] & 0xFF00) >> 8;
    if (KtaRoCe > 127)
    {
        KtaRoCe = KtaRoCe - 256;
    }
    KtaRC[1] = KtaRoCe;
      
    KtaReCe = (eeData[55] & 0x00FF);
    if (KtaReCe > 127)
    {
        KtaReCe = KtaReCe - 256;
    }
    KtaRC[3] = KtaReCe;
  
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
    ktaScale2 = (eeData[56] & 0x000F);

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
            if (ktaTemp[p] > 3)
            {
                ktaTemp[p] = ktaTemp[p] - 8;
            }
            ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
            ktaTemp[p] = KtaRC[split] + ktaTemp[p];
            ktaTemp[p] = ktaTemp[p] / pow(2,(double)ktaScale1);
            //ktaTemp[p] = ktaTemp[p] * mlx90640->offset[p];
        }
    }
    
    temp = fabs(ktaTemp[0]);
    for(int i = 1; i < 768; i++)
    {
        if (fabs(ktaTemp[i]) > temp)
        {
            temp = fabs(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        ktaScale1 = ktaScale1 + 1;
    }    
     
    for(int i = 0; i < 768; i++)
    {
        temp = ktaTemp[i] * pow(2,(double)ktaScale1);
        if (temp < 0)
        {
            mlx90640->kta[i] = (temp - 0.5);
        }
        else
        {
            mlx90640->kta[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90640->ktaScale = ktaScale1;           
}


//------------------------------------------------------------------------------

void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;
    float kvTemp[768];
    float temp;

    KvRoCo = (eeData[52] & 0xF000) >> 12;
    if (KvRoCo > 7)
    {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;
    
    KvReCo = (eeData[52] & 0x0F00) >> 8;
    if (KvReCo > 7)
    {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;
      
    KvRoCe = (eeData[52] & 0x00F0) >> 4;
    if (KvRoCe > 7)
    {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;
      
    KvReCe = (eeData[52] & 0x000F);
    if (KvReCe > 7)
    {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;
  
    kvScale = (eeData[56] & 0x0F00) >> 8;


    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            kvTemp[p] = KvT[split];
            kvTemp[p] = kvTemp[p] / pow(2,(double)kvScale);
            //kvTemp[p] = kvTemp[p] * mlx90640->offset[p];
        }
    }
    
    temp = fabs(kvTemp[0]);
    for(int i = 1; i < 768; i++)
    {
        if (fabs(kvTemp[i]) > temp)
        {
            temp = fabs(kvTemp[i]);
        }
    }
    
    kvScale = 0;
    while(temp < 64)
    {
        temp = temp*2;
        kvScale = kvScale + 1;
    }    
     
    for(int i = 0; i < 768; i++)
    {
        temp = kvTemp[i] * pow(2,(double)kvScale);
        if (temp < 0)
        {
            mlx90640->kv[i] = (temp - 0.5);
        }
        else
        {
            mlx90640->kv[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90640->kvScale = kvScale;        
}

//------------------------------------------------------------------------------

void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;
    
    offsetSP[0] = (eeData[58] & 0x03FF);
    if (offsetSP[0] > 511)
    {
        offsetSP[0] = offsetSP[0] - 1024;
    }
    
    offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
    if (offsetSP[1] > 31)
    {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0]; 
    
    alphaSP[0] = (eeData[57] & 0x03FF);
    if (alphaSP[0] > 511)
    {
        alphaSP[0] = alphaSP[0] - 1024;
    }
    alphaSP[0] = alphaSP[0] /  pow(2,(double)alphaScale);
    
    alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
    if (alphaSP[1] > 31)
    {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0];
    
    cpKta = (eeData[59] & 0x00FF);
    if (cpKta > 127)
    {
        cpKta = cpKta - 256;
    }
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;    
    mlx90640->cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = (eeData[59] & 0xFF00) >> 8;
    if (cpKv > 127)
    {
        cpKv = cpKv - 256;
    }
    kvScale = (eeData[56] & 0x0F00) >> 8;
    mlx90640->cpKv = cpKv / pow(2,(double)kvScale);
       
    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];  
}

//------------------------------------------------------------------------------

void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;
    
    calibrationModeEE = (eeData[10] & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (eeData[53] & 0x003F);
    if (ilChessC[0] > 31)
    {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;
    
    ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
    if (ilChessC[1] > 15)
    {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;
    
    ilChessC[2] = (eeData[53] & 0xF800) >> 11;
    if (ilChessC[2] > 15)
    {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;
    
    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
}

//------------------------------------------------------------------------------

int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;
    int i;
    
    for(pixCnt = 0; pixCnt<5; pixCnt++)
    {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }
        
    pixCnt = 0;    
    while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
    {
        if(eeData[pixCnt+64] == 0)
        {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        else if((eeData[pixCnt+64] & 0x0001) != 0)
        {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt = outlierPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
        
    } 
    
    if(brokenPixCnt > 4)  
    {
        warn = -3;
    }         
    else if(outlierPixCnt > 4)  
    {
        warn = -4;
    }
    else if((brokenPixCnt + outlierPixCnt) > 4)  
    {
        warn = -5;
    } 
    else
    {
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<brokenPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->brokenPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }
        
        for(pixCnt=0; pixCnt<outlierPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->outlierPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        } 
        
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=0; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }    
        
    }    
    
    
    return warn;
       
}

//------------------------------------------------------------------------------

 int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
 {
     int pixPosDif;
     
     pixPosDif = pix1 - pix2;
     if(pixPosDif > -34 && pixPosDif < -30)
     {
         return -6;
     } 
     if(pixPosDif > -2 && pixPosDif < 2)
     {
         return -6;
     } 
     if(pixPosDif > 30 && pixPosDif < 34)
     {
         return -6;
     }
     
     return 0;    
 }
 
//------------------------------------------------------------------------------
 
float GetMedian(float *values, int n)
 {
    float temp;
    
    for(int i=0; i<n-1; i++)
    {
        for(int j=i+1; j<n; j++)
        {
            if(values[j] < values[i]) 
            {                
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
    
    if(n%2==0) 
    {
        return ((values[n/2] + values[n/2 - 1]) / 2.0);
        
    } 
    else 
    {
        return values[n/2];
    }
    
 }           

//------------------------------------------------------------------------------

int IsPixelBad(uint16_t pixel,paramsMLX90640 *params)
{
    for(int i=0; i<5; i++)
    {
        if(pixel == params->outlierPixels[i] || pixel == params->brokenPixels[i])
        {
            return 1;
        }    
    }   
    
    return 0;     
}     


// mathematical functions for the thermal sensor

float GetFrameTotal(uint16_t startPixel, uint16_t endPixel, float * frame)
{
  float total = 0;
  for(uint32_t i = startPixel; i < endPixel; i++)
  {
    total += frame[i];
  }
  return total;
}

float GetFrameAverage(uint16_t startPixel, uint16_t endPixel, float * frame)
{
  if(endPixel == startPixel)
  {
    return -1; // error
  }
  float average = GetFrameTotal(startPixel, endPixel, frame);
  average = average / (endPixel - startPixel);
  if(average > 10 && average < 50)
    return average;
  return -1;
}

float GetFrameTotalOfPixelsAboveThreshold(uint16_t startPixel, uint16_t endPixel, float * frame, float threshold)
{
  float aboveThTotal = 0;
  for(int i = startPixel; i < endPixel; i++)
  {
    if(frame[i] > threshold && frame[i] > 10 && frame[i] < 50)
    {
      aboveThTotal += frame[i];
    }
  }
  return aboveThTotal;
}

