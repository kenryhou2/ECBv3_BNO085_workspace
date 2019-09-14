## PSOC_EBC3.1_WS

Tested build environment:
```Windows 10 64-bits #PSOC Creator 4.2```

# Hardware

- ECB v3.1 board: Version3 rev 1 of the Embedded Controller Board(ECB), a small form factor swiss army knife for bio-robotics applications.
- MAT6 board: ECB for MAT6 6 legged robo with force sensing, and BNO085 IMU integration on the feet. 
- CY8CKIT-002 PSoC® MiniProg3 Program and Debug Kit.
- MicroUSB cable

# Software
- C programming Language.
- PSoC Creator Schematic GUI
- Serial terminal (Arduino Serial monitor and serial plotter works well)

# Projects
- P00_Blink_LED: (Working) Basic IO toggle test
- P01_Bootloader: (Working) Bootloader project for the ECBv3 board, please link any biotloadable project the bin/hex file generated to this folder.
- P02_Blink_LED_Bootloadable: (Working) Test project of using USBUART based bootloader to flash the ECB without MiniProg.
- P03_USBUART_Control_LED: (Working) Basic test for bi-directional USBUART communication. Using keyboard inputs, LED RGB values will adjust to new values.
- P04_a_CAN_TX_Test
- P04b_b_CAN_RX_Test
- P05_IMU_BNO085_Test: Original adaptation of IMU code from PSoC4 to PSoC5LP. Non working.
- P06_IMU_BNO085_WIP: (Working) Work in progress adaptation of IMU code from PSoC4 to PSoC5LP with I2C communication between PSoC and IMU. Successful implementation and configurable outputs for biorobotics applications. Current data output mode: Outputs 6 axis rotation vector, linear accelration, and gyroscope values at ~200 Hz, in quaternion, linear acceleration (xyz), and angular velocity (wx,wy,wz) format respectively.
- P07_IMU_BNO085_SPI: SPI implementation of P06, goal is to obtain ~6x faster output with SPI communication between IMU and PSoC. Not yet working! and ECBv3 Board needs more hardware modifications like reset and interrupt pins, along with slave select and other SPI specific pins pulled to the right polarity.
- P08_IMU_BNO085_MAT6: (Working) Adaptation of IMU code to another ECB, this time with a different PSoC processor. Hardware pins are changed and performs same functionality as P06. **There is one significant hardware difference in this project where the enable pin of the 3.3V voltage regulator was not pulled high.** Therefore we manually pulled it high through the code, and called it ENABLE_3V_PIN.
- P09_MAT6_Bootloader: (Working) Bootloader project for MAT6 ECB.
- P10_IMU_BNO085_SensTileECB: (Working) Adaptation of P06 for the sensortile like BNO085 IMU attachment to the ECBv3 board. Essentially the sensortile is an offboard attachment PCB with just the IMU. 
- P10_IMU_BNO085_SensTileEigen: Same as the SensTileECB version except pinouts are to the Eigenboard. Actually haven't tested it.

# P06 Project Details.

## Quick Start
- Using the P06 Code, one can successfully program the ECBv3 board to output three sensor fields: Orientation, linear acceleration, angular velocity in the form of 10 floats at a rate of 200 Hz from the PSoC. 

1. **Setup**
	1. Obtain code by pulling from this remote master. Folder should contain a PSoC Creator workspace with projects specified above. 
	2. Once the workspace is open, you will find a directory of projects on the left hand side of the IDE. Right click on P06_IMUBNO085_WIP and select `Set as Active Project`.
	3. Within `main.c`, configure the use modes for the board by commenting or uncommenting the following flags.
		- `#define USBUART_MODE` is used anytime there is a direct USB serial connection from the board to an external device, in our case, a CPU. Note: that when USBUART_MODE is defined, **debugging cannot be used**. If you debug with this mode activated, the code will hang forever in a while loop waiting for a USB port to be enumerated which can't happen during debugging. Use USBUART_MODE for printing out information whether that is for its primary purpose of outputting IMU data as floats, or outputting strings.
		- `#define DATA_OUTPUT_MODE` indicates that the PSoC is outputting IMU data as **floats** rather than strings. In this mode, the `printOut()` function is disabled, meaning there are no string outputs from the PSoC. This is the intended mode of use for the Blaser or MAT6 projects where the 10 floats of IMU information are needed through USB serial connection. **An important note:** In order for the PSoC to successfully output IMU float data, **both USBUART_MODE and DATA_OUTPUT_MODE need to be active.** Solely DATA_OUTPUT_MODE activated will result in no USB serial capability by the PSoC, which is useful in debug mode when one wants to see the procedure of outputting float data step by step.
		- If DATA_OUTPUT_MODE is undefined, there is an implicit mode of outputting strings. In this mode, printOut() function will work and can print any helper messages to the serial output. Like with DATA_OUTPUT_MODE, **this mode requires USBUART_MODE to be able to serially output strings to terminal.**

2. **Program**
	1. After configuring the outputs, the board must be programmed. The method of programming is determined by the hardware in two possible configurations. The first is traditionally with the miniprog3 programmer/debugger through SWD connection. The second is through bootloading the corresponding application/project onto the board. Check the `TopDesign.cysch` file for a Bootloadable component. If it is enabled, then bootloader is configured, which should be as default for this project.
	2. If the bootloadable is enabled, plug the board directly to the computer with USB and open the bootloader host under <Tools> in the taskbar. Further instructions to using the bootloader host can be found in the [Bootloader User Guide](https://docs.google.com/document/d/1NsbHpMEDuHHZEE9elAJRFjD2x9ydBso8VCzAN2paOsE/edit).
	3. If bootloadable is not enabled, use the miniprog3 and select program under <Debug> in the taskbar. You may need to select the device, port acquire, and then program. 

3. **Use**

If the bootloadable is enabled, there will be a brief ~3 Second bootloader program being run indicated by a rapidly blinking blue LED. Once the bootloader program has finished, the IMU application will run and output according to the configured output modes from the Setup.
	While the program is running there are serveral LED indicators for the status of the IMU.
	1. At the beginning of the IMU application, the IMU is initializing and setting up a USB connection. While the board is doing this it periodically activates the red LED in addition to the blue and green LEDs. The RGB LED will look like it is blinking between red and orange. 
	2. As soon as a USB serial connection is established with an external computer, the Red LED will cease to activate leaving only the blue and green LEDs to link periodically. 
	3. After a certain period of time, the Red LED should begin to periodically activate again, meaning the IMU has been calibrated onboard the PCB. If the Red light has not lit up after the initialization, it means the IMU is still dynamically calibrating  and the environment may not be constant enough for the IMU. This will result in less accurate outputs.
	**Note:** If the bootloadable is not enabled, the IMU will proceed directly to the IMU initialization stage, skipping the blue LED blink stage.

To view the outputs, a script needs to be created to parse the 10 output floats in the following encoding: [float i, float j, float k, float r, float x, float y, float z, float wx, float wy, float wz]. The first four floats are for a quaternion for orientation, the next three floats are linear acceleration in x, y, z axes, and the last three floats are angular velocities along x, y, z axes.

## IMU Code Full Procedure Description
- Relevent File Brief Descriptions:
	- Main.c: (User generated code) Contains **high level** functions for initializing hardware, configuring sensor settings, obtaining data repeatedly from IMU, and printing IMU information. Outline of code that runs through series of stages described below.
	- Sh2_hal_psoc5.c (User generated code) Provides interface that adapts SH-2 API to a particular sensor hub. In our case, the BNO085 IMU. Contains **low level** functions for communicating between the PSoC and the sensorhub/IMU with SHTP protocol. This includes, reading/writing with I2C, creating timestamps on microsecond scale, and opening/closing connection between the PSoC and the sensorhub. 
	- sh2.c: (Library code) Contains **mid level** public SH2-API functions that involve sending/reading commands to the IMU sensorhub. These commands include getting product ID, enabling calibration, taring the sensorhub, etc. Sh2 API functions use the low level functions from the user generated Sh2.hal_psoc5.c to pass information between the IMU and PSoC.  
	- shtp.c: (Library code) Specifies the protocols to parse and format data coming to and from the sensorhub. **Note:** sh2 and shtp are intertwined in the process to interact with the IMU. You can think of the SH-2 as the software represented device that we're communicating to and SHTP as the protocol to communicate to the SH-2.
	**Note**: information on the specifics of SHTP and SH-2 are found in the datasheets listed below in Useful Links. Or here:  
- [SH2 Datasheet](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- [SH2 specifics of SHTP Datasheet](https://www.hillcrestlabs.com/downloads/sh-2-shtp-reference-manual)
- [SHTP Datasheet](https://cdn.sparkfun.com/assets/7/6/9/3/c/Sensor-Hub-Transport-Protocol-v1.7.pdf)
- In addition, although I believe it is outdated, there is an SH-2 user guide provided here with some more information on the SH-2 API. Warning: some of the API functions are not accurate to the ones we use, for example, sh2_initialize(). But they use a lot of the structs that are still consistent with our implementation.
- [Outdated SH2 User Guide](https://github.com/hcrest/bno080-driver/blob/master/UserGuide.pdf)

### Chronological Timeline of IMU Data Reading Process
1. Initialization:
- The program starts by starting all hardware not associated with the IMU. This includes PWM, Buzzer,and USBFS components. 
- Then the IMU setup function is called. This creates a Hardware Abstraction Layer (HAL) variable and initializes it by creating pointers to specific HAL functions within the SH2_hal_psoc5.c file.
- The function then proceeds to open an instance of the HAL variable with the SH2_open() function within sh2.c.
- The Sh2_open function performs a variety of actions including setting up callback functions, setting up important initialization flags, such as **pSh2->resetComplete and pSh2->advertDone**, and calling the library defined SHTP_open() function. The most important thing to notice about the SHTP_open() function during initialization is that it is watching for an SHTP advertisement packet coming from the IMU. During initialization, the program sets a flag for Advert Requested, which can be seen in the SHTP_open() function. - The SHTP advertisement packet, like all transfers from the IMU is preceded by an SHTP header, and contains the necessary information to configure the SHTP communication protocols to be able to receive useful data from the IMU. From observation, the SHTP advertisement packet should be around 276 bytes. Specifically, it contains channel assignments. For more information on the advertisement packet, take a look at the SHTP datasheet, and SH2 Specifics of SHTP Datasheet. In addition, the SHTP advertisement will be outputted from the IMU once it senses that its reset pin has been activated. Therefore, the active reset pin is necessary for this IMU.
- The SHTP open() function, in addition, calls the user generated SH2_i2c_HAL_open() function.
- The SH2_i2c_HAL_open() initializes IMU hardware components such as the timer, I2C bus, and interrupt handler. The function also manually resets the IMU active low, which prompts an advertisement packet. It is also important to note the significance of the IMU interrupt here as well. The IMU will pull the interrupt pin active low when it needs attention. This is the case when it is ready to output data, such as the SHTP advertisement packet or other IMU data. The interrupt is caught as a falling edge and then a boolean flag, Attention_needed is set true. The IMU interrupt pin being toggled is a good indicator that the IMU is not broken and can communicate with the MCU.
- When the SH2_i2c_Hal_open() function returns, the SHTP_open() function finishes and also returns back to SH2_open().
- The rest of the SH2_open() function is spent servicing the IMU in order to read this advertisement packet in a while loop. There is a time-out condition where if the IMU doesn't send the advertisement in time or at all, the time-out will occur and the IMU will not be properly initialized. The program must service the IMU so that the entire advertisement packet is read. This initilization servicing is performed by the shtp_service() function. It ultimately calls the user generated sh2_i2c_hal_read() function and also applies the proper callbacks to the recieved information to parse the data for each transfer.
-if the advertisement packet successfully transfers, the MCU will now have a clear idea of how to communicate with the IMU.
-At the end of the Sh2_open() function, it will return SH2_OK, and the flags, **pSh2->resetComplete and pSh2->advertDone should be true.** If this is not the situation, the initialization did not occur successfully.
- Otherwise, the IMU should now be ready to be configured, serviced to output data, and recieve commands.

2. Configuration and Calibration
- Now that the IMU has been setup correctly, we can configure it with SH-2 API commands to obtain different kinds of data at varying rates.
- These actions are performed in the start_reports() and enableCal() functions.
-start_reports() is where the user enables the sensor fields for output. There are numerous options all defined in the BNO085 Datasheet and SH-2 Reference manual/datasheet. For our default program, we are using the SH2_GYRO_INTEGRATED_RV and SH2_ACCELEROMETER. The Gyro integrated rotation vector has the capability to output a 6 axis dependent quaternion in the form of 4 floats, i, j, and real. It also can output gyroscope angular velocity data in the form of 3 floats, wx, wy, and wz. This gives angular velocity along the 3 respective axes. The accelerometer sensor field outputs linear accelerometer data including gravity in float form wrt three axes: x, y, z. These data outputs will be saved into a float buffer array during servicing.
- The next configuration step is to set up a configure variable to set for the enabled sensors. The config struct contains member variables such as sensitivity, wakeupEnable, and report_rate. 
- **A very important factor to note is that the output rate by the IMU will drop significantly for each enabled sensor.** This makes sense as the IMU has to work n times as hard for n sensors enabled, Thus theoretically decreasing the output rate by a factor of n. In addition, when configuring the sensor report_interval, there needs to be a significant difference in the values since the IMU will get overwhelmed by sensor data and the rate at which it will output will drop even more significantly than n times. This is just from observation through trial and error. For the specific setup, the GyroIntegrated_RV is set to a report interval of 5000 us or 200 Hz and the Accelerometer is set to an interval of 1000us of frequency of 1 KHz. 
- In addition, certain sensor fields are made of fused sensors from the raw gyroscope, raw accelerometer, or raw magnetometer. They undergo a fusing process and more processing to obtain a special output such as the rotation vector. This leads it to have a greater actual report period in general when compared to the individual accelerometer report interval.
- The next step is calibration, which is performed in enableCal(). There are parameters to set which type of raw sensor to calibrate and by default they are all set to true. Then, the enableCal performs an Sh2 API call to perform dynamic calibration on each raw sensor. The dynamic calibration procedure is explained in the BNO085 datasheet as well as in the [IMU calibration datasheet](https://www.hillcrestlabs.com/downloads/bno080-sensor-calibration-procedure)


3. Servicing
- Servicing is the act of reading and parsing our requested sensor field reports from the IMU.
- Sensor data arrives from the IMU to the MCU in the form of I2C reads which are organized in sensor events with reports. Each sensor type has its own report ID with sensor data in the payload. 
- The main functions involved with servicing include SensorHandler(), SH2_service, SHTP_service(), and checkCal();
- SH2_service is called repeatedly which calls SHTP_service(), and essentially performs an I2C read into a payload buffer which is parsed byte by byte. The cursor then encounters a specific report and initiates a callback function call to deal with the report.
- Sensor Handler is one of the callback functions where it takes the sensor event passed in and decodes the report into useful data, such as our floats for output. The sensor values are then placed in a transmit buffer which is used as the vessel for output.
- It's useful to visualize the flow of data in a series pipeline with an I2C transfer containing an SHTP header with a bunch of sensor events containing sensor reports with sensor field data. The exact composition of each transfer is not known exactly, but the composition of each I2C transfer from the IMU is not unifromly distributed with the enabled sensor field data. It makes sense that the sensor events are in the order of first arrival, so usually fused sensor fields have less sensor events than accelerometer sensor events in an I2C transfer. In other words, we would receive sensor events from fused sensors less frequently.
- One interesting observation is that the gyroscope sensor event rate behaves strangely in comparison to the accelerometer sensor event rate or the fused sensor event rates. However, the raw gyroscope sensor event rate runs at an expected rate. I've contacted HillCrest Labs with about this discrepancy but they have not replied, and will update this readme once they do. 

3. Printing Data and Output Efficiency
- If enabling different sensor reports, there is an issue that not each sensor event will arrive at the same rate due to the nature of having different sensors and fused sensors. The IMU compensates for this by sending repeated values for the sensor until the event is updated. 
- Note that the output can be defined by both frequency and uniqueness. In order to obtain the best output from the IMU, there needs to be a high frequency and unique values.
- This is implemented by placing booleans "got_mag, got_rot, etc." as flag indicators to detect when new data is read in and is reset when output.
- Factors that affect output efficiency include the bit rate of our I2C reads, number of sensors enabled, and sensor report interval congiguration. 
- Finally, after each iteration of Sh2_service, the status bits of each sensor report are read that indicate accuracy of the specific sensor by a bitwise function. The accuracy bits are represented by two bits ranging from values [0 - 3]  where 0 is least accurate, and 3 is most accurate. Accuracy is dependent on the  dynamic calibration, so as the calibration sequence proceeds in a stable environment, the accuracy bits should tend to up 3. Once the accuracy bit is a value of 2 or greater for an arbitrary number of cycles, the program deems calibration is complete and saves the environment calibration configurations to flash. Therefore when the IMU starts up again in a similar environment, the calibration process should be tuned much more quickly.


# Debugging Tips
- **pSh2->resetComplete and pSh2->advertDone**: These flags are indicators of the initialization process. If both true, the advertisement packet has been successfully read in by the MCU.
- **SH-2 API functions return an integer specifying status**: The status return value of 0 means SH2_OK and the function successfully executed. A negative return value is an error. Additional details are within sh2_err.h.
- **IMU Interrupt**: The IMU interrupt pin is good indicator for the IMU communicating properly. Once global interrupts are enabled on in the program after a restart, the IMU should pull the interrupt pin active low because it is about to output an unsolicited advertisement packet. Then, the interrupt pin activates every time it prepares to output data. 
- Timer Firing pin for output rates: After obtaining data and printing out, the timer firing pin shouldtoggle, and it was used along with a logic analyzer for determining output data rates.
- Disabling DATA_OUTPUT_MODE for string outputs:
-Logic Analyzer or Oscilloscope on I2C pins: Viewing the I2C reads is a good indicator on the status of communication between the MCU and the IMU.
- LEDs

# Useful Links
- [BNO085 IMU Datasheet](https://www.hillcrestlabs.com/downloads/bno080-datasheet)
- [SH2 Datasheet](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- [SH2 specifics of SHTP Datasheet](https://www.hillcrestlabs.com/downloads/sh-2-shtp-reference-manual)
- [SHTP Datasheet](https://cdn.sparkfun.com/assets/7/6/9/3/c/Sensor-Hub-Transport-Protocol-v1.7.pdf)
- [STM BNO085 Implementation](https://github.com/hcrest/sh2-demo-nucleo)
- [Arduino BNO085 Implementation](https://github.com/sparkfun/Qwiic_IMU_BNO080/blob/master/Firmware/Tester/Tester.ino)
- [Bootloader and Bootloadable User Guide](https://docs.google.com/document/d/1NsbHpMEDuHHZEE9elAJRFjD2x9ydBso8VCzAN2paOsE/edit)
- [Miniprog 3 User Guide](https://www.cypress.com/file/44091/download)
- [IMU calibration datasheet](https://www.hillcrestlabs.com/downloads/bno080-sensor-calibration-procedure)
- [ECBv3 Schematic](https://drive.google.com/open?id=15yXZ_82Z9C4lrHsHq1oSJScUXaHmO8ll)

# Personal Annotations on Datasheets 
- I've added some personal notes on the datasheets in the Datasheet_Annotations folder. These may or may not help.

