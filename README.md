

**SE4130 Real-Time Embedded Systems**

**Course Project, Spring 2021**

**Team #3 - Team Nuclear Football**

**Tim Collier, Austin FitzGerald, Brandon Krcmar**


## Hardware
- 1x [STM32F413ZH Discovery Kit](https://www.st.com/en/evaluation-tools/32f413hdiscovery.html)
- 1x [Robot Smart Car Board Starter Kit](https://www.amazon.com/gp/product/B07YCCSW7W)
	- This includes a chassis, four motors, four wheels, a battery housing, an L298N motor driver, and mounting fasteners. We do not recommend purchasing this exact kit because the motors are of poor quality and the L298N board that we received did not work.
- 16x 20cm jumper wires
- 6x AA batteries (for powering the motors)
- 1x 5V power bank (for powering the microcontroller)
- WiFi capable computer or phone
### Hardware Instructions
1. Solder a separate wire on to each motor terminal. Take note of the polarity for each wire.
2. Splice the wires for both pairs of motors such that there are only four input wires (left positive/negative and right positive/negative). The motors on each side of the car (left/right) must be driven as a single unit due to the solo motor driver.
3. Cut the barrel connector off of the battery housing and strip the ends. This will power the motors.
4. Assemble the car per the kit instructions. The fasteners must be adequately tightened so that the motors stay aligned straight. 
5. Mount the L298N, battery housing, and microcontroller such that they are secure but accessible. Our mounting strategy was poor so we cannot recommend much here.
6. Connect wires.
	1. 
### Hardware Lessons Learned
- It would have been wise to wire a switch between the battery housing and L298N. Without one we must disconnect the jumper cable when not using the car in order to save batteries.
- A single motor driver, paired with the 6 AA batteries, cannot adequately power all four motors. We should have chosen a lighter battery, more powerful motors, and possibly two motor drivers.
- Splices should be soldered and wires should be trimmed to acceptable lengths.

## Software
 - Appropriate STM32 drivers
 - STM32CubeIDE v1.4.0
 - Node.js v12+
	- npm v6+
### STM32CubeIDE Setup
1. File->New->STM32 Project From An Existing...
2. Choose the .ioc file from this project
3. Set the compiler include paths as shown below

MCU GCC Compiler Include Paths:
 - ../USB_HOST/App
 - ../Drivers/BSP/STM32F413H-Discovery
 - ../USB_HOST/Target
 - ../Core/Inc
 - ../Drivers/STM32F4xx_HAL_Driver/Inc
 - ../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
 - ../Middlewares/ST/STM32_USB_Host_Library/Core/Inc
 - ../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc
 - ../Drivers/CMSIS/Device/ST/STM32F4xx/Include
 - ../Drivers/CMSIS/Include
 - ../Middlewares/Third_Party/FreeRTOS/Source/include
 - ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
 -  ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
### Node.js Setup
1. Open a terminal and navigate to the /web directory of the project
2. Run `npm install` to install the required packages (as specified in package.json)
### Running The Software
These steps should be the same no matter which platform the controller server is running on. It is easiest on PC, but has been tested on an unrooted Android phone as well.
1. Deploy the project onto the microcontroller from STM32CubeIDE.
	- The microcontroller display should read "AP Created."
2. Connect to the newly created access point "STM32F413H-DISCO", password is "12345678".
	- The microcontroller display should read "AP client connected." and then "TCP server created."
3. Launch the controller web server. In the project /web directory, run `node index.js` in a terminal. In a web browser, connect to the newly created local web server at localhost:8080.
	- You have 60 seconds to start the web server.
	- The server console should read "Listening on *:8080", then "Web client connected.", then "Connected to TCP server."
	- The microcontroller display should read "TCP client connected."
5. Use the red draggable circle to control the car.
	- You should see the x,y coordinates change on the webpage, which indicates that the microcontroller has received that data. Those x,y coordinates will also be displayed on the server console and microcontroller display.
	- The connection strength (RSSI) will only be shown on the microcontroller display.
### Noticing Errors
If a fault occurs during operation, the car will not continue moving. All faults related to the access point and TCP server are handled and the user should be notified which fault occurs by means of a message on the microcontroller display.

Setting up the TCP connection can be finicky. If the messages described for step 3 are not displayed correctly, then please restart the software from step 1.
