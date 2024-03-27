/******************************************************************************
 * @file     vio_iENBL.c
 * @brief    Virtual I/O implementation for board iENBL
 * @version  V2.0.0
 * @date     17. October 2023
 ******************************************************************************/
/*
 * Copyright (c) 2020-2023 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! \page vio_iENBL Physical I/O Mapping
The table below lists the physical I/O mapping of this CMSIS-Driver VIO implementation.
Virtual Resource  | Variable       | Physical Resource on iEMBL                     |
:-----------------|:---------------|:-----------------------------------------------|
vioBUTTON0        | vioSignalIn.0  | MCU_BTN1_INT  ('I' button)                     |
vioBUTTON1        | vioSignalIn.1  | MCU_BTN0_WKUP ('II" button)                    |
vioLED0           | vioSignalOut.0 | LED1_PWM_R (Red LED, front panel)              |
vioLED1           | vioSignalOut.1 | LED1_PWM_G (Green LED, front panel)            |
vioLED2           | vioSignalOut.2 | LED1_PWM_B (Blue LED, front panel)             |
vioLED3           | vioSignalOut.3 | LED0_PWM_R (Red LED, charger)                  |
*/

/* History:
 *  Version 2.0.0
 *    Updated to API 1.0.0
 *  Version 1.0.0
 *    Initial release
 */

#include <string.h>
#include "cmsis_vio.h"

#include "RTE_Components.h"                 // Component selection
#include CMSIS_device_header

#if !defined CMSIS_VOUT || !defined CMSIS_VIN
// Add user includes here:
#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"
#endif

// VIO input, output definitions
#define VIO_VALUE_NUM           3U          // Number of values

// VIO input, output variables
__USED uint32_t vioSignalIn;                // Memory for incoming signal
__USED uint32_t vioSignalOut;               // Memory for outgoing signal
__USED int32_t  vioValue[VIO_VALUE_NUM];    // Memory for value used in vioGetValue/vioSetValue

#if !defined CMSIS_VOUT
// Add global user types, variables, functions here:

#endif

#if !defined CMSIS_VIN
// Add global user types, variables, functions here:

#endif

// Initialize test input, output.
void vioInit (void) {
#if !defined CMSIS_VOUT
// Add user variables here:
  GPIO_InitTypeDef io_out;
#endif
#if !defined CMSIS_VIN
// Add user variables here:
  GPIO_InitTypeDef io_in;
#endif

  vioSignalIn  = 0U;
  vioSignalOut = 0U;

  memset(vioValue, 0, sizeof(vioValue));

#if !defined CMSIS_VOUT
  // Initialize LEDs pins
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  io_out.Mode  = GPIO_MODE_OUTPUT_OD;
  io_out.Pull  = GPIO_NOPULL;
  io_out.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // LED0_PWM_R
  io_out.Pin   = GPIO_PIN_4;
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOE, &io_out);

  // LED1_PWM_R
  io_out.Pin   = GPIO_PIN_12;
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOD, &io_out);

  // LED1_PWM_B
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  io_out.Pin   = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &io_out);

  // LED1_PWM_G
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  io_out.Pin   = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOD, &io_out);
  
#endif

#if !defined CMSIS_VIN
  // Initialize buttons pins
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  io_in.Mode  = GPIO_MODE_INPUT;
  io_in.Pull  = GPIO_NOPULL;
  io_in.Speed = GPIO_SPEED_FREQ_LOW;

  //MCU_BTN0_WKUP
  io_in.Pin   = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &io_in);

  //MCU_BTN1_INT
  io_in.Pin   = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOA, &io_in);
#endif
}

// Set signal output.
void vioSetSignal (uint32_t mask, uint32_t signal) {
#if !defined CMSIS_VOUT
// Add user variables here:

#endif

  vioSignalOut &= ~mask;
  vioSignalOut |=  mask & signal;

#if !defined CMSIS_VOUT
  // Output signals to LEDs

  if (mask & vioLED0) {
    /* Red LED */
    if (signal & vioLED0) {
      // LED On
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    } else {
      // LED Off
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    }
  }

  if (mask & vioLED1) {
    /* Green LED */
    if (signal & vioLED1) {
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    }
  }

  if (mask & vioLED2) {
    /* Blue LED */
    if (signal & vioLED2) {
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin (GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    }
  }

  if (mask & vioLED3) {
    /* Red LED (charger) */
    if (signal & vioLED3) {
      HAL_GPIO_WritePin (GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin (GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    }
  }
#endif
}

// Get signal input.
uint32_t vioGetSignal (uint32_t mask) {
  uint32_t signal;
#if !defined CMSIS_VIN
// Add user variables here:

#endif

#if !defined CMSIS_VIN
  // Get input signals from buttons
  if (mask & vioBUTTON0) {
    /* Button 'I' */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET) {
      vioSignalIn |=  vioBUTTON0;
    } else {
      vioSignalIn &= ~vioBUTTON0;
    }
  }

  if (mask & vioBUTTON1) {
    /* Button 'II' */
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET) {
      vioSignalIn |=  vioBUTTON1;
    } else {
      vioSignalIn &= ~vioBUTTON1;
    }
  }
#endif

  signal = vioSignalIn & mask;

  return signal;
}

// Set value output.
void vioSetValue (uint32_t id, int32_t value) {
  uint32_t index = id;
#if !defined CMSIS_VOUT
// Add user variables here:

#endif

  if (index >= VIO_VALUE_NUM) {
    return;                             /* return in case of out-of-range index */
  }

  vioValue[index] = value;

#if !defined CMSIS_VOUT
// Add user code here:

#endif
}

// Get value input.
int32_t vioGetValue (uint32_t id) {
  uint32_t index = id;
  int32_t  value = 0;
#if !defined CMSIS_VIN
// Add user variables here:

#endif

  if (index >= VIO_VALUE_NUM) {
    return value;                       /* return default in case of out-of-range index */
  }

#if !defined CMSIS_VIN
// Add user code here:

//   vioValue[index] = ...;
#endif

  value = vioValue[index];

  return value;
}
