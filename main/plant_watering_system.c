/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "sensorMLX90640.h"
#include "kwADC_ads1015.h"
#include "water_pump.h"

bool initDone = 0;

void MoistureSensorTask()
{
  bool init = false;
  printf("Sensor Code Init\n");

  // init ADC
  if(ads1015Init())
  {
    printf("Init successful\n");
    init = true;
  }
  else
  {
    printf("Init failed\n");
  }
  
  while(1)
  {
    if(init)
    {
      uint16_t moisture_sensor_value_0 = ads1015ReadChannel(0);
      uint16_t moisture_sensor_value_1 = ads1015ReadChannel(1);
      uint16_t moisture_sensor_value_2 = ads1015ReadChannel(2);
      uint16_t moisture_sensor_value_3 = ads1015ReadChannel(3);
      
      water_pump_logic(moisture_sensor_value_0, moisture_sensor_value_1, moisture_sensor_value_2, moisture_sensor_value_3);
      
      printf("%d\t", moisture_sensor_value_0);
      printf("%d\t", moisture_sensor_value_1);
      printf("%d\t", moisture_sensor_value_0);
      printf("%d\t", moisture_sensor_value_1);
      vTaskDelay(10);
      printf("\n");
    }
  }
}

void ADS1015TestTask()
{
  bool init = false;
  printf("Sensor Code Init\n");

  // init ADC
  if(ads1015Init())
  {
    printf("Init successful\n");
    init = true;
  }
  else
  {
    printf("Init failed\n");
  }
  
  while(1)
  {
    // run something
    if(init)
    {
      uint16_t sensorValue1 = ads1015ReadChannel(0);
      uint16_t sensorValue2 = ads1015ReadChannel(1);
      printf("%d, \t ", sensorValue1);
      printf("%d \t ", sensorValue2);
      uint8_t doorState = ads1015GetDoorOpenClose(sensorValue1, sensorValue2);
      switch (doorState)
      {
      case DOOR_OPEN_INSIDE:
        printf("DOOR_OPEN_INSIDE \t %d \t", doorState);
        break;
      case DOOR_AJAR_INSIDE:
        printf("DOOR_AJAR_INSIDE \t %d \t", doorState);
        break;
      case DOOR_CLOSE:
        printf("DOOR_CLOSE \t %d \t", doorState);
        break;
      case DOOR_AJAR_OUTSIDE:
        printf("DOOR_AJAR_OUTSIDE \t %d \t", doorState);
        break;
      case DOOR_OPEN_OUTSIDE:
        printf("DOOR_OPEN_OUTSIDE \t %d \t", doorState);
        break;
      case DOOR_INVALID_STATE:
        printf("DOOR_INVALID_STATE \t %d \t", doorState);
        break;
      default:
        break;
      }
      printf("\n");
    }
  }
}

void MLX90640TestTask()
{
  printf("Initializing MLX90640 Thermal Sensor");
  if(SensorInitMLX90640())
  {
    printf("Initialization Successful\n");
  }
  else
  {
    printf("Initialization Failure\n");
  }

  static uint16_t frame_1[832];

  while(1)
  {
    SensorRunLoopMLX90640();
    initDone = 1;
  }
}

void printTask()
{
  while(!initDone)
  {
    vTaskDelay(1000);
  }
  while(initDone)
  {
    PrintDesiredData();
    vTaskDelay(1);
  }
}


void helloWorldTask()
{
    while(1)
    {
        printf("hello World\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // MLX90640 Test Task
    //xTaskCreatePinnedToCore(MLX90640TestTask, "MLX90640TestTask", 4096 * 2, NULL, 10, NULL, 1);

    // // MLX90640 Test Task
    //xTaskCreatePinnedToCore(printTask, "printTask", 4096, NULL, 10, NULL, 0);

    // ADS1015 Test Task
    xTaskCreatePinnedToCore(MoistureSensorTask, "MoistureSensorTask", 4096, NULL, 10, NULL, 0);

    // MLX90640 Test Task
    //xTaskCreatePinnedToCore(helloWorldTask, "helloWorldTask", 4096, NULL, 10, NULL, 1);


    // printf("Hello world!\n");

    // /* Print chip information */
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
    //         chip_info.cores,
    //         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
    //         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    // printf("silicon revision %d, ", chip_info.revision);

    // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
    //         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    fflush(stdout);
    // esp_restart();
}
