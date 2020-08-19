Create Platform Project From Scratch
====================================

**Create uVision Project**
- Select device

**Add and configure software components**

Open "Manage Run-Time Environment" window and configure as follows:
Configure CMSIS components:
- Expand CMSIS group (under Software Component column)
- Select CORE component
- Expand RTOS2 (API) group
- Select RTOS2 component (FreeRTOS, Keil RTX5)

Configure CMSIS-Driver components:
- Expand CMSIS Driver group (under Software Component column)
- Expand and select the required driver (I2C, MCI, SPI, USART, VIO, WiFi)

Configure Event Recorder:
- Expand Compiler group (under Software Component column)
- Enable Event Recorder checkbox (DAP variant)

Configure C library stdio retarget:
- Expand Compiler group (under Software Component column)
- Expand I/O group
- Enable STDERR checkbox and select variant (ITM for iENBL)
- Enable STDIN checkbox and select variant (ITM for iENBL)
- Enable STDOUT checkbox and select variant (ITM for iENBL)

Add device startup:
- Expand Device group (under Software Component column)
- Select Startup component

Resolve dependencies:
- Click Resolve button

STMicroelectronics specific:
- Expand STM32Cube Framework(API) group
- Launch STM32CubeMX (this might not work when clicked for the first time?? Could take several clicks!)

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
Open (or create) source file containing the main application entry function:
- Add the following includes:
```
#include "cmsis_os2.h"
#include "RTE_Components.h"
#ifdef    RTE_VIO_BOARD
#include "cmsis_vio.h"
#endif
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
- Initialize peripherals (GPIO, DMA, etc...)

- Initialize Event Recorder (add following code snippet):
```
#if defined(RTE_Compiler_EventRecorder) && \
    (defined(__MICROLIB) || \
    !(defined(RTE_CMSIS_RTOS2_RTX5) || defined(RTE_CMSIS_RTOS2_FreeRTOS)))
  EventRecorderInitialize(EventRecordAll, 1U);
#endif
```
- Initialize CMSIS-VIO (add following code snippet):
```
#ifdef RTE_VIO_BOARD
  vioInit();
#endif
```
- Start CMSIS-RTOS2 kernel (add following code snippet):
```
  osKernelInitialize();                         /* Initialize CMSIS-RTOS2 */
  app_initialize();                             /* Initialize application */
  osKernelStart();                              /* Start thread execution */
```
Create application first thread:
- Create new source file and add the following code:
```
#include "cmsis_os2.h"
static const osThreadAttr_t app_main_attr = {
  .stack_size = 4096U
};
static void app_main (void *argument) {
  (void)argument;

  for (;;) {}
}
void app_initialize (void) {
  osThreadNew(app_main, NULL, &app_main_attr);
}
```
Right click on a Source Group in the uVision Project window and select "Add Existing Files to..."
- Navigate to ./RTE/Device/STM32L496QEIx/STCubeGenerated/ and add STCubeGenerated.ioc as Text Document file

**Configure project target**

Open Options for Target dialog:
- Select compiler, configure debugger (i.e. trace setup for ITM)

Locate Event Recorder in uninitialized memory (https://www.keil.com/pack/doc/compiler/EventRecorder/html/er_use.html#place_uninit_memory)

STMicroelectronics specific:
- Ensure that CMSIS Drivers are configured as specified:
  - Documentation is accessible via Manage Run-Time Environment, Device::STM32Cube Framework(API) see the Description column.
    (refer to chapter: STM32L4 CMSIS-Drivers Configuration documentation)

**Add and configure Board layer**

uVision bug: Keyboard shortcut (Alt+F7) does not work if you click on software component and press Alt+F7.
             But it does work if you click on software component and right click on it
             Using uVision 5.31.0.0 (release version)

Create layers:
- Create layer named RTOS and add:
  - Description: RTOS layer
  - Keywords: RTOS
  - Category: RTOS
  - License: (BSD 3-Clause, Apache 2.0)

- Open "Manage Project Items" window (Project->Manage->Project Items)
- Select Project Info/Layer tab
- Create layer named "Board" and add:
  - Description: (Board layer for...)
  - Keywords: Board, (etc.)
  - Category: Board
  - License: (BSD 3-Clause, Apache 2.0)
  - Enable "Target" checkbox

**Assign layers to components**

In uVision, in Project window, select appropriate group and right click on it to open "Options for Component Class" (Alt+F7):

- Source groups, source files, documents:
  - Under Memory Assignment, open Layer dropbox and select appropriate layer (RTOS, Board)

- CMSIS component:
  - Select CMSIS::CORE and under Settings->Layer select Board
  - Select component listed under CMSIS::RTOS2 (API) and under Settings->Layer select RTOS

- CMSIS Driver component:
  - For any listed component: select it and under Settings->Layer select Board

- Compiler component:
  - Select Compiler::Event Recorder and under Settings->Layer select Board
  - Select Compiler::I/O::STDERR/STDIO/STDOUT and under Settings->Layer select Board

- Device component:
- Select Device::Startup and under Settings->Layer select Board
- Repeat for any other listed component

**Export layer info**

In uVision, select Project->Export->Save Project to CPRJ format
