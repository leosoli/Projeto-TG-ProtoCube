/*
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
*/

/**
 * @file main.c
 * @brief Brief description of your modifications.
 *
 * Detailed description of the modifications and improvements made to the
 * original code by Simon M. Werner.
 *
 * Modifications:
 * - Removed library AHRS, since it is no longer the goal of this work
 * - Added library to deal with the UART initialization and writing proccesses
 * - Removed axis transformation for the magnetometer readings since 
 *   accelerometer and gyroscope values should be in the magnetometer reference
 * - Adjusted calibration parameters to meet the MPU-9250 tested
 * - Delivery of data acquired through UART in a 5 Hz rate
 *
 * Acknowledgments:
 * - Thank you to Simon M. Werner for the original code.
 *
 * License:
 * - This modified code is also licensed under the Apache License, Version 2.0.
 */



#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "libs/uart.h"

static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal = {
    .mag_offset = {.x = -13.234375, .y = 80.871094, .z = -38.414062},
    .mag_scale = {.x = 0.980424, .y = 1.006865, .z = 1.013324},
    .accel_offset = {.x = 0.039083, .y = 0.070368, .z = 0.136644},
    .accel_scale_lo = {.x = 1.016390, .y = 1.036406, .z = 1.069427},
    .accel_scale_hi = {.x = -0.978780, .y = -0.962060, .z = -0.925534},

    .gyro_bias_offset = {.x = -0.947105, .y = 2.625738, .z = 0.588435}
    };



static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = y;
  v->y = x;
  v->z = -z;
}


void run_imu(void)
{

  // Inicialização do barramento I2C e verificações iniciais do estado de funcionamento do MPU-9250 utilizando os parâmetros de calibração fornecidos
  i2c_mpu9250_init(&cal);

  // Inicialização da UART para envio dos dados adquiridos
  init_uart();

  uint64_t i = 0;
  while (true)
  {
    vector_t va, vg, vm;

    // Aquisição das medições do Acelerômetro, giroscópio e magnetômetro.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transformação dos eixos dos valores adquiridos para a mesma orientação dos eixos do magnetômetro
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);


    // Impressão a cada 10 iterações
    if (i++ % 10 == 0)
    {
            
      ESP_LOGI(TAG, "accel: x %2.3f, y %2.3f, z %2.3f (g)", va.x, va.y, va.z);
      ESP_LOGI(TAG, "gyro: x %2.3f, y %2.3f, z %2.3f (degrees/s)", vg.x, vg.y, vg.z);
      ESP_LOGI(TAG, "mag: x %2.3f, y %2.3f, z %2.3f (uT)", vm.x, vm.y, vm.z);

      ESP_LOGI(TAG, "data:%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f", va.x, va.y, va.z, vg.x, vg.y, vg.z, vm.x, vm.y, vm.z);

      // Atribuir memória para a mensagem string
      size_t message_size = snprintf(NULL, 0, "data:%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f\n", va.x, va.y, va.z, vg.x, vg.y, vg.z, vm.x, vm.y, vm.z) + 1;
      char* message = (char*)malloc(message_size);

      // Gerar a string da mensagem e guardá-la na memória reservada
      snprintf(message, message_size, "data:%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f\n", va.x, va.y, va.z, vg.x, vg.y, vg.z, vm.x, vm.y, vm.z);

      // Envio da mensagem atraves da UART
      send_uart(message);

      // Liberacao da memoria alocada
      free(message);


      send_uart("\n");



      // Intervalo de amostragem
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    pause();
  }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
  ESP_LOGI("main-calibrate", "A calibração foi concluída com sucesso.\n");
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

void app_main(void)
{
  // start i2c task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}