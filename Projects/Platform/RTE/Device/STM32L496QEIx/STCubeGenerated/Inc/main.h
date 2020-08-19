/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void app_initialize (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CELL_UART_TX_Pin GPIO_PIN_5
#define CELL_UART_TX_GPIO_Port GPIOD
#define CELL_PWR_EN_Pin GPIO_PIN_4
#define CELL_PWR_EN_GPIO_Port GPIOB
#define CELL_UART_RX_Pin GPIO_PIN_15
#define CELL_UART_RX_GPIO_Port GPIOA
#define LED0_PWM_R_Pin GPIO_PIN_4
#define LED0_PWM_R_GPIO_Port GPIOE
#define CELL_SPI_NSS_Pin GPIO_PIN_9
#define CELL_SPI_NSS_GPIO_Port GPIOB
#define CELL_PWRKEY_Pin GPIO_PIN_6
#define CELL_PWRKEY_GPIO_Port GPIOB
#define PWR_IO2_EN_Pin GPIO_PIN_6
#define PWR_IO2_EN_GPIO_Port GPIOD
#define CELL_UART_RTS_Pin GPIO_PIN_4
#define CELL_UART_RTS_GPIO_Port GPIOD
#define CELL_UART_CTS_Pin GPIO_PIN_3
#define CELL_UART_CTS_GPIO_Port GPIOD
#define USIM_DETECT_Pin GPIO_PIN_1
#define USIM_DETECT_GPIO_Port GPIOD
#define SD_CLK_Pin GPIO_PIN_12
#define SD_CLK_GPIO_Port GPIOC
#define GNSS_UART_TX_Pin GPIO_PIN_10
#define GNSS_UART_TX_GPIO_Port GPIOC
#define PWR_SENS_EN_Pin GPIO_PIN_0
#define PWR_SENS_EN_GPIO_Port GPIOE
#define BLE_GPIO2_Pin GPIO_PIN_5
#define BLE_GPIO2_GPIO_Port GPIOB
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define CELL_NRST_Pin GPIO_PIN_0
#define CELL_NRST_GPIO_Port GPIOD
#define GNSS_UART_RX_Pin GPIO_PIN_11
#define GNSS_UART_RX_GPIO_Port GPIOC
#define RTC_IN_Pin GPIO_PIN_14
#define RTC_IN_GPIO_Port GPIOC
#define MCU_BTN0_WKUP_Pin GPIO_PIN_6
#define MCU_BTN0_WKUP_GPIO_Port GPIOE
#define CELL_INT_Pin GPIO_PIN_2
#define CELL_INT_GPIO_Port GPIOF
#define WIFI_UART_RX_Pin GPIO_PIN_10
#define WIFI_UART_RX_GPIO_Port GPIOG
#define WIFI_UART_TX_Pin GPIO_PIN_9
#define WIFI_UART_TX_GPIO_Port GPIOG
#define MCU_BTN1_INT_Pin GPIO_PIN_9
#define MCU_BTN1_INT_GPIO_Port GPIOA
#define BLE_GPIO1_Pin GPIO_PIN_8
#define BLE_GPIO1_GPIO_Port GPIOA
#define SD_ON_Pin GPIO_PIN_9
#define SD_ON_GPIO_Port GPIOC
#define RTC_OUT_Pin GPIO_PIN_15
#define RTC_OUT_GPIO_Port GPIOC
#define GNSS_VBCKP_Pin GPIO_PIN_3
#define GNSS_VBCKP_GPIO_Port GPIOF
#define SD_D0_Pin GPIO_PIN_8
#define SD_D0_GPIO_Port GPIOC
#define CELL_RADIO_STATE_Pin GPIO_PIN_7
#define CELL_RADIO_STATE_GPIO_Port GPIOC
#define HOST_SLEEP_IND_Pin GPIO_PIN_4
#define HOST_SLEEP_IND_GPIO_Port GPIOF
#define PWR_GNSS_EN_Pin GPIO_PIN_5
#define PWR_GNSS_EN_GPIO_Port GPIOF
#define WIFI_BOOT_Pin GPIO_PIN_11
#define WIFI_BOOT_GPIO_Port GPIOG
#define CELL_FLIGHTMODE_EN_Pin GPIO_PIN_1
#define CELL_FLIGHTMODE_EN_GPIO_Port GPIOG
#define EXT_UART_TX_Pin GPIO_PIN_7
#define EXT_UART_TX_GPIO_Port GPIOG
#define LED1_PWM_G_Pin GPIO_PIN_14
#define LED1_PWM_G_GPIO_Port GPIOD
#define LED1_PWM_B_Pin GPIO_PIN_13
#define LED1_PWM_B_GPIO_Port GPIOD
#define CELL_GPIO1_Pin GPIO_PIN_4
#define CELL_GPIO1_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define EXT_UART_XR_Pin GPIO_PIN_8
#define EXT_UART_XR_GPIO_Port GPIOG
#define CELL_I2C_SCL_Pin GPIO_PIN_14
#define CELL_I2C_SCL_GPIO_Port GPIOF
#define CELL_I2C_SDA_Pin GPIO_PIN_15
#define CELL_I2C_SDA_GPIO_Port GPIOF
#define LED1_PWM_R_Pin GPIO_PIN_12
#define LED1_PWM_R_GPIO_Port GPIOD
#define PCM_OUT_Pin GPIO_PIN_11
#define PCM_OUT_GPIO_Port GPIOD
#define PCM_CLK_Pin GPIO_PIN_10
#define PCM_CLK_GPIO_Port GPIOD
#define WIFI_CHIP_EN_Pin GPIO_PIN_15
#define WIFI_CHIP_EN_GPIO_Port GPIOG
#define BLE_nRST_Pin GPIO_PIN_3
#define BLE_nRST_GPIO_Port GPIOC
#define BLE_WKUP_Pin GPIO_PIN_2
#define BLE_WKUP_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define BLE_UART_TX_Pin GPIO_PIN_4
#define BLE_UART_TX_GPIO_Port GPIOC
#define WIFI_RSTn_Pin GPIO_PIN_11
#define WIFI_RSTn_GPIO_Port GPIOF
#define USB_SW_CTRL_Pin GPIO_PIN_9
#define USB_SW_CTRL_GPIO_Port GPIOD
#define BLE_WIFI_SW_CTRL_Pin GPIO_PIN_8
#define BLE_WIFI_SW_CTRL_GPIO_Port GPIOD
#define GNSS_MISC_Pin GPIO_PIN_15
#define GNSS_MISC_GPIO_Port GPIOB
#define BLE_GPIO0_Pin GPIO_PIN_13
#define BLE_GPIO0_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define CELL_WKUP_Pin GPIO_PIN_5
#define CELL_WKUP_GPIO_Port GPIOC
#define CELL_GPIO2_Pin GPIO_PIN_2
#define CELL_GPIO2_GPIO_Port GPIOB
#define CELL_GPIO3_Pin GPIO_PIN_8
#define CELL_GPIO3_GPIO_Port GPIOE
#define GNSS_TIMEPULSE_Pin GPIO_PIN_12
#define GNSS_TIMEPULSE_GPIO_Port GPIOE
#define BLE_WIFI_SW_EN_Pin GPIO_PIN_10
#define BLE_WIFI_SW_EN_GPIO_Port GPIOB
#define BLE_UART_RX_Pin GPIO_PIN_11
#define BLE_UART_RX_GPIO_Port GPIOB
#define PCM_SYNC_Pin GPIO_PIN_12
#define PCM_SYNC_GPIO_Port GPIOB
#define CELL_GPIO0_Pin GPIO_PIN_1
#define CELL_GPIO0_GPIO_Port GPIOA
#define MEM_SPI_NSS_Pin GPIO_PIN_0
#define MEM_SPI_NSS_GPIO_Port GPIOB
#define CELL_GPIO4_Pin GPIO_PIN_7
#define CELL_GPIO4_GPIO_Port GPIOE
#define CELL_GPIO5_Pin GPIO_PIN_9
#define CELL_GPIO5_GPIO_Port GPIOE
#define PWR_WIFI_EN_Pin GPIO_PIN_11
#define PWR_WIFI_EN_GPIO_Port GPIOE
#define GNSS_EXT_INT_Pin GPIO_PIN_13
#define GNSS_EXT_INT_GPIO_Port GPIOE
#define BLE_BOOT_Pin GPIO_PIN_14
#define BLE_BOOT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
