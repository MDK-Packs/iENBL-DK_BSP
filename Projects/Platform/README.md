Flex iENBL Platform Project
---------------------------

- This board layer is configured using [*STM32CubeMX*](https://www.st.com/en/development-tools/stm32cubemx.html)
- The board configuration is stored in the file `./RTE/Device/STM32L496QEIx/STCubeGenerated/STCubeGenerated.ioc`.
- [Setup with classic Keil MDK](https://www.keil.com/pack/doc/STM32Cube/html/index.html)
- It is a [CMSIS-RTOS v2](https://arm-software.github.io/CMSIS-RTX/latest/index.html) based software template that can be further expanded.

## Keil RTX5 Real-Time Operating System

The [Keil RTX5 RTOS](https://arm-software.github.io/CMSIS-RTX/latest/cre_rtx_proj.html)
implements the resource management. It is configured with the following settings:

| Memory setting           | Size
|:-------------------------|:----------------------------------------
|Global Dynamic Memory size| 24000 bytes
|Default Thread Stack size | 3072 bytes

## Flex iENBL Target Board

The board layer contains the following configured interface drivers:

### System Configuration

| System Component        | Setting
|:------------------------|:----------------------------------------
| Device                  | STM32L496QEIx
| Heap                    | 64 kB (configured in startup file)
| Stack (MSP)             |  1 kB (configured in startup file)

### Clock Configuration

| Clock                   | Setting
|:------------------------|:----------------------------------------
| HCLK                    | 80 MHz
| FCLK                    | 80 MHz
| APB1                    | 80 MHz
| APB2                    | 80 MHz
| To USARTs               | 80 MHz

### GPIO Configuration and Usage

| GPIO         | Signal            | GPIO Settings                                                    | Usage/Label
|:-------------|:------------------|:-----------------------------------------------------------------|:-----
| PA1          |                   | Input mode                                                       | CELL_GPIO0
| PA2          |                   | Input mode                                                       | BLE_WKUP  
| PA4          |                   | Input mode                                                       | CELL_GPIO1
| PA5          | SPI1_SCK          | Alternate Function Push Pull, Very High speed                    | SPI1_SCK
| PA6          | SPI1_MISO         | Alternate Function Push Pull, Very High speed                    | SPI1_MISO
| PA7          | SPI1_MOSI         | Alternate Function Push Pull, Very High speed                    | SPI1_MOSI
| PA8          |                   | Input mode                                                       | BLE_GPIO1   
| PA9          |                   | Input mode                                                       | MCU_BTN1_INT
| PA15 (JTDI)  | USART2_RX         | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_UART_RX
| PB0          |                   | Output Push Pull, Low output level, Low speed                    | MEM_SPI_NSS   
| PB2          |                   | Input mode                                                       | CELL_GPIO2    
| PB4 (NJTRST) |                   | Output Push Pull, Low output level, Low speed                    | CELL_PWR_EN   
| PB5          |                   | Input mode                                                       | BLE_GPIO2     
| PB6          |                   | Output Push Pull, Low output level, Low speed, Disable Fast Mode | CELL_PWRKEY   
| PB9          |                   | Output Push Pull, Low output level, Low speed, Disable Fast Mode | CELL_SPI_NSS  
| PB10         |                   | Output Push Pull, Low output level, Low speed                    | BLE_WIFI_SW_EN
| PB11         | USART3_RX         | Alternate Function Open Drain, Pull-up, Very High speed          | BLE_UART_RX
| PB12         |                   | Output Push Pull, Low output level, Low speed                    | PCM_SYNC
| PB13         |                   | Input mode                                                       | BLE_GPIO0
| PB15         |                   | Input mode                                                       | GNSS_MISC
| PC3          |                   | Output Push Pull, Low output level, Low speed                    | BLE_nRST
| PC4          | USART3_TX         | Alternate Function Open Drain, Pull-up, Very High speed          | BLE_UART_TX
| PC5          |                   | Input mode                                                       | CELL_WKUP         
| PC7          |                   | Input mode                                                       | CELL_RADIO_STATE  
| PC8          | SDMMC1_D0         | Alternate Function Push Pull, Very High speed                    | SD_D0
| PC9          |                   | Output Push Pull, Low output level, Low speed                    | SD_ON
| PC10         | UART4_TX          | Alternate Function Open Drain, Pull-up, Very High speed          | GNSS_UART_TX
| PC11         | UART4_RX          | Alternate Function Open Drain, Pull-up, Very High speed          | GNSS_UART_RX
| PC12         | SDMMC1_CK         | Alternate Function Push Pull, Very High speed                    | SD_CLK
| PD0          |                   | Output Push Pull, Low output level, Low speed                    | CELL_NRST
| PD1          |                   | Input mode                                                       | USIM_DETECT
| PD2          | SDMMC1_CMD        | Alternate Function Push Pull, Very High speed                    | SD_CMD
| PD3          | USART2_CTS        | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_UART_CTS
| PD4          | USART2_RTS        | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_UART_RTS
| PD5          | USART2_TX         | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_UART_TX
| PD6          |                   | Output Push Pull, Low output level, Low speed                    | PWR_IO2_EN        
| PD8          |                   | Output Push Pull, Low output level, Low speed                    | BLE_WIFI_SW_CTRL  
| PD10         |                   | Input mode                                                       | PCM_CLK           
| PD11         |                   | Output Push Pull, Low output level, Low speed                    | PCM_OUT           
| PD12         |                   | Output Open Drain, Low output level, Low speed                   | LED1_PWM_R        
| PD13         |                   | Output Open Drain, Low output level, Low speed                   | LED1_PWM_G        
| PD14         |                   | Output Open Drain, Low output level, Low speed                   | LED1_PWM_B        
| PE0          |                   | Output Push Pull, Low output level, Low speed                    | PWR_SENS_EN       
| PE4          |                   | Output Open Drain, Low output level, Low speed                   | LED0_PWM_R        
| PE6          |                   | Input mode                                                       | MCU_BTN0_WKUP     
| PE7          |                   | Input mode                                                       | CELL_GPIO4        
| PE8          |                   | Input mode                                                       | CELL_GPIO3        
| PE9          |                   | Input mode                                                       | CELL_GPIO5        
| PE11         |                   | Output Push Pull, Low output level, Low speed                    | PWR_WIFI_EN       
| PE12         |                   | Input mode                                                       | GNSS_TIMEPULSE    
| PE13         |                   | Output Push Pull, Low output level, Low speed                    | GNSS_EXT_INT      
| PE14         |                   | Output Push Pull, Low output level, Low speed                    | BLE_BOOT          
| PF2          |                   | Input mode                                                       | CELL_INT          
| PF3          |                   | Output Push Pull, Low output level, Low speed                    | GNSS_VBCKP        
| PF4          |                   | Input mode                                                       | HOST_SLEEP_IND    
| PF5          |                   | Output Push Pull, Low output level, Low speed                    | PWR_GNSS_EN       
| PF11         |                   | Output Push Pull, Low output level, Low speed                    | WIFI_RSTn         
| PF14         | I2C4_SCL          | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_I2C_SCL
| PF15         | I2C4_SDA          | Alternate Function Open Drain, Pull-up, Very High speed          | CELL_I2C_SDA
| PG1          |                   | Output Push Pull, Low output level, Low speed                    | CELL_FLIGHTMODE_EN
| PG11         |                   | Output Open Drain, Low output level, Low speed                   | WIFI_BOOT   
| PG15         |                   | Output Push Pull, Low output level, Low speed                    | WIFI_CHIP_EN

### NVIC Configuration

 - Priority Group = 4 bits for preemption priority 0 bits for subpriority

| NVIC Interrupt                          | Preempt Priority | Code Generation
|:----------------------------------------|:-----------------|:---------------
| Non maskable interrupt                  | 0                | Generate IRQ handler
| Hard fault interrupt                    | 0                | Generate IRQ handler
| Memory Management fault                 | 0                | Generate IRQ handler
| Prefetch fault, memory access fault     | 0                | Generate IRQ handler
| Undefined instruction or illegal state  | 0                | Generate IRQ handler
| System service call via SWI instruction | 0                | none
| Debug monitor                           | 0                | Generate IRQ handler
| Pendable request for system service     | 0                | none
| Time base: System tick timer            | 0                | none
| SPI1 global                             | 0                | Generate IRQ handler, Call HAL handler
| USART1 global                           | 0                | Generate IRQ handler, Call HAL handler
| USART2 global                           | 0                | Generate IRQ handler, Call HAL handler
| USART3 global                           | 0                | Generate IRQ handler, Call HAL handler
| SDMMC1 global                           | 0                | none
| UART4 global                            | 0                | Generate IRQ handler, Call HAL handler
| DAM2 channel4 global                    | 0                | Generate IRQ handler, Call HAL handler
| DAM2 channel5 global                    | 0                | Generate IRQ handler, Call HAL handler
| I2C4 event                              | 0                | Generate IRQ handler, Call HAL handler
| I2C4 error                              | 0                | Generate IRQ handler, Call HAL handler


### Connectivity Peripherals Configuration

| Peripheral   | Mode / Settings                                                                          | IRQ | DMA                                                   
|:-------------|:-----------------------------------------------------------------------------------------|:----|:------------------------------------------------------
| I2C4         | I2C, Standard Mode, Do Not Generate Function Call                                        | yes | no                                                    
| SDMMC1       | SD 1 bit, Do Not Generate Function Call                                                  | yes | SDMMC1_RX = DMA2 Channel 5, SDMMC1_TX = DMA2 Channel 4
| SPI1         | Full-Duplex Master, Hardware NSS Signal=Disable, Do Not Generate Function Call           | yes | no                                                    
| UART4        | Asynchronous, Hardware Flow Control=Disable, 115200/8/N/1, Do Not Generate Function Call | yes | no                                                    
| USART1       | Asynchronous, Hardware Flow Control=Disable, 115200/8/N/1, Do Not Generate Function Call | yes | no                                                    
| USART2       | Asynchronous, Hardware Flow Control=CTS/RTS, 115200/8/N/1, Do Not Generate Function Call | yes | no                                                    
| USART3       | Asynchronous, Hardware Flow Control=Disable, 115200/8/N/1, Do Not Generate Function Call | yes | no                                                     

**STDIO** routed to ITM port (CMSIS-DAP)

### CMSIS-Driver Mapping

| CMSIS-Driver | Peripheral   | Routed to
|:-------------|:-------------|----------
| I2C4         | I2C4         | BG96 (U304)
| MCI0         | SDMMC1       | Micro SD transceiver (U901)
| SPI1         | SPI1         | W25Q16FWUXIE (U203) and BG96 (U304)
| USART1       | USART1       | BG96 (U304)
| USART2       | USART2       | BlueNRG-2 (U701)
| USART3       | USART3       | ZOE-M8G (U404)
| USART4       | UART4        | ESP8285 (U601)

| CMSIS-Driver VIO  | Physical board hardware
|:------------------|:-----------------------
| vioBUTTON0        | MCU_BTN1_INT (PA9)
| vioBUTTON1        | MCU_BTN0_WKUP (PE6)
| vioLED0           | LED1_PWM_R (PD12)
| vioLED1           | LED1_PWM_G (PD14)
| vioLED2           | LED1_PWM_B (PD13)
| vioLED3           | LED0_PWM_R (PE4)