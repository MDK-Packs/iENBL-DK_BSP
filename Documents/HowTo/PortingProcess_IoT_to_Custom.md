Porting process from B-L475E-IOT01A_BSP to iENBL
================================================

**Start building application from an existing project**

Open existing Platform project
- Open "Options for Target ..." dialog and select Device tab:
  - Change selected device from STM32L475VGTx to STM32L496QEIx and click OK

Manage Run-Time Environment window opens
- Expand "Board Support" group and deselect all components
- Expand "CMSIS Driver" group
- Expand "VIO (API)" group and deselect Board component
- Click OK and close uVision

Using File Explorer navigate to project root ./RTE/Device and delete folder STM32L475VGTx.

Open uVision again and load the same project.

Open "Manage Run-Time Environment" window
- Expand "Device" group
- Expand "STM32Cube Framework (API)", select STM32CubeMX checkbox and Launch STM32CubeMX

STM32CubeMX Application opens

**Configure device using STM32CubeMX**
- Open Pinout & Configuration and configure peripherals under System Core, Connectivity etc.
- Under System Core select NVIC and open Code generation tab:
  - Clear Generate IRQ handler checkbox for "System service call...", "Pendable request..." and "Time base..."
- Open Clock Configuration and configure system and peripheral clocks
- Open Project Manager and select Code Generator:
  - Under STM32Cube MCU packages and embedded software packs select Add necessary library files...
- Click GENERATE CODE.
- Click Close when Code Generation notification dialog opens.
- You can leave STM32CubeMX application open.
- Return to uVision and click the OK button to close "Manage Run-Time Environment" window.

**Add source code to the project**

Open source file containing the main application entry function:
- Add the following includes:
```
#include "cmsis_os2.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_EventRecorder)
#include "EventRecorder.h"
#endif
```

-Add the following code into the main function:
- Configure system clocks and update system clock variable (add following code snippet):
```
  /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
```
- Initialize Event Recorder (add following code snippet):
```
#if defined(RTE_Compiler_EventRecorder) && \
    (defined(__MICROLIB) || \
    !(defined(RTE_CMSIS_RTOS2_RTX5) || defined(RTE_CMSIS_RTOS2_FreeRTOS)))
  EventRecorderInitialize(EventRecordAll, 1U);
#endif
```
- Start CMSIS-RTOS2 kernel (add following code snippet):
```
  osKernelInitialize();                         /* Initialize CMSIS-RTOS2 */
  app_initialize();                             /* Initialize application */
  osKernelStart();                              /* Start thread execution */
```

**Remove unnecessary files and components**

In uVision Project tree windows select "Board IO" group and remove it.

Open "Manage Run-Time Environment" window and reconfigure C library stdio retarget:
Reconfigure C library stdio retarget:
- Expand Compiler group (under Software Component column)
- Expand I/O group
- STDERR component: select ITM variant
- STDIN component: select ITM variant
- STDOUT component: select ITM variant

Change WiFi module driver (Platform should not have WiFi driver pre-selected anyway):
- Expand CMSIS Driver group (under Software Component column)
- Expand "WiFi (API)" group:
  - Deselect ISM43362 and select ESP8266 (ESP8285 compatible)

**Add or modify CMSIS drivers**

Open "Manage Run-Time Environment" window:
- Expand CMSIS Driver group (under Software Component column)
- Expand and select/deselect the required driver (I2C, MCI, SPI, USART, VIO, WiFi)
- Resolve dependencies:
  - Click Resolve button
  - Click OK to close the window

Ensure that CMSIS Drivers are configured as specified:
- Documentation is accessible via Manage Run-Time Environment, Device::STM32Cube Framework(API) see the Description column
  (refer to chapter: STM32L4 CMSIS-Drivers Configuration documentation).

**Configure project target**

Open Options for Target dialog:
- Select compiler, configure debugger (i.e. trace setup for ITM)

Locate Event Recorder in uninitialized memory (https://www.keil.com/pack/doc/compiler/EventRecorder/html/er_use.html#place_uninit_memory)

Ensure that stack and heap settings in device startup are as required.

Optionally:
- In uVision Project tree window update location to or remove file STCubeGenerated.ioc
 (STM32CubeMX configuration file).