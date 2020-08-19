Platform project
================

The **Platform** project configures the hardware of the evaluation board
and is a CMSIS-RTOS2 based software template that can be further expanded.

RTX5 Real-Time Operating System
-------------------------------
The [RTX5 RTOS](https://arm-software.github.io/CMSIS_5/RTOS2/html/rtx5_impl.html) 
implements the resource management. It is configured with the following settings:

- Global Dynamic Memory size: 24000 bytes
- Default Thread Stack size: 3072 bytes

Flex iENBL Target Board
-----------------------
The Board layer contains the following configured interface drivers:

**CMSIS-Driver USART1** routed to ESP8285 (U601):
 - RX: WIFI_UART_RX (PG10) (USART1_RX)
 - TX: WIFI_UART_TX (PG9)  (USART1_TX)

**CMSIS-Driver USART2** routed to BG96 (U304):
 - RX: CELL_UART_RX (PA15) (USART2_RX)
 - TX: CELL_UART_TX (PD5)  (USART2_TX)
 - CTS: CELL_UART_CTS (PD3) (USART2_CTS)
 - RTS: CELL_UART_RTS (PD4) (USART2_RTS)

**CMSIS-Driver USART3** routed to BlueNRG-2 (U701):
 - RX: BLE_UART_RX (PB11) (USART3_RX)
 - TX: BLE_UART_TX (PC4)  (USART3_TX)

**CMSIS-Driver USART4** routed to ZOE-M8G (U404):
 - RX: GNSS_UART_RX (PC11) (UART4_RX)
 - TX: GNSS_UART_TX (PC10) (UART4_TX)

**CMSIS-Driver MCI0** routed to Micro SD transceiver (U901):
 - D0: SD_D0 (PC8)    (SDMMC1_D0)
 - CMD: SD_CMD (PD2)  (SDMMC1_CMD)
 - CLK: SD_CLK (PC12) (SDMMC1_CK)

**CMSIS-Driver SPI1** routed to W25Q16FWUXIE (U203) and BG96 (U304):
 - SCK:  SPI1_SCK (PA5) (SPI1_SCK)
 - MISO: SPI1_MISO (PA6) (SPI1_MISO)
 - MOSI: SPI1_MOSI (PA7) (SPI1_MOSI)

**CMSIS-Driver I2C4** routed to BG96 (U304):
 - SDA: CELL_I2C_SDA (PF15) (I2C4_SDA)
 - SCL: CELL_I2C_SCL (PF14) (I2C4_SCL)

**CMSIS-Driver VIO** with the following board hardware mapping:
 - vioBUTTON0: MCU_BTN1_INT (PA9)
 - vioBUTTON1: MCU_BTN0_WKUP (PE6)
 - vioLED0: LED1_PWM_R (PD12)
 - vioLED1: LED1_PWM_G (PD14)
 - vioLED2: LED1_PWM_B (PD13)
 - vioLED3: LED0_PWM_R (PE4)

**STDIO** routed to ITM port (CMSIS-DAP)

**GPIO** pins routed to various devices:
 - output: PWR_IO2_EN (PD6)
 - output: PWR_WIFI_EN (PE11)
 - output: PWR_GNSS_EN (PF5)
 - output: PWR_SENS_EN (PE0)
 - output: CELL_PWR_EN (PB4)
 - output: WIFI_CHIP_EN (PG15)
 - output: WIFI_RSTn (PF11)
 - output: WIFI_BOOT (PG11)
 - output: SD_ON (PC9)
 - output: MEM_SPI_NSS (PB0) (SPI1_NSS)
 - output: CELL_SPI_NSS (PB9) (SPI1_NSS)

The board configuration can be modified using 
[STM32CubeMX](https://www.keil.com/stmicroelectronics-stm32) 
and is stored in the file `STCubeGenerated.ioc`.
