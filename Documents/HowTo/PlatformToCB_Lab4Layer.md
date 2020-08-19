Create board layer from Platform project to fit CB_Lab4Layer
============================================================

1. Open Platform project in uVision and export it to CPRJ format.
2. Open bash shell and extract layers.
3. Copy .\Layer\Board folder into .\CB_Lab4Layer\layer\Board\
4. Rename Board folder into iENBL (to get .\CB_Lab4Layer\layer\Board\iENBL)
5. Create new file named "board_define" and add the following content: export ARDUINO_USART_NUMBER=1
6. Take README.md from the Platform project folder and copy it into .\CB_Lab4Layer\layer\Board\iENBL
7. Rename copied README.md into "layer.Board.md"
8. Optional (for consistency): Open file "layer.Board.md" and delete text till "Flex iENBL TargetBoard" and save the file.

Folder structure in .\CB_Lab4Layer\layer\Board\iENBL must be as follows:
```
.\RTE
Board.clayer
board_define
layer.Board.md
Platform.iENBL.sct
```
Navigate into folder .\CB_Lab4Layer\build\

- Create new file List.txt and add the combinations listed below into it, save and close.
```
App=Blinky Board=iENBL RTOS=RTX
App=Blinky Board=iENBL RTOS=FreeRTOS
App=AWS_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=RTX
App=AWS_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=FreeRTOS
App=Azure_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=RTX
App=Azure_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=FreeRTOS
App=Google_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=RTX
App=Google_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=FreeRTOS
App=Paho_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=RTX
App=Paho_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=FreeRTOS
App=Watson_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=RTX
App=Watson_MQTT_Demo Board=iENBL Module=ESP8266 Socket=WiFi RTOS=FreeRTOS
```
- Open bash shell and execute: $./gen_proj_list.sh List.txt --layer=../layer

All listed projects must be generated successfuly.

Tested projects (verified to work as expected):
- Blinky using RTX and FreeRTOS
- All cloud connectors using RTX (FreeRTOS variants are not tested)
