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

- P00_Blink_LED: Basic IO toggle test
- P01_Bootloader: Bootloader project for the ECBv3 board, please link any biotloadable project the bin/hex file generated to this folder.
- P02_Blink_LED_Bootloadable: Test project of using USBUART based bootloader to flash the ECB without MiniProg.
- P03_USBUART_Control_LED: Basic test for bi-directional USBUART communication. Using keyboard inputs, LED RGB values will adjust to new values.
- P04_a_CAN_TX_Test
- P04b_b_CAN_RX_Test
-P05_IMU_BNO085_Test: Original adaptation of IMU code from PSoC4 to PSoC5LP. Non working.
-P06_IMU_BNO085_WIP: Work in progress adaptation of IMU code from PSoC4 to PSoC5LP with I2C communication between PSoC and IMU. Successful implementation and configurable outputs for biorobotics applications. Current data output mode: Outputs 6 axis rotation vector, linear accelration, and gyroscope values at ~200 Hz, in quaternion, linear acceleration (xyz), and angular velocity (wx,wy,wz) format respectively.
-P07_IMU_BNO085_SPI: SPI implementation of P06, goal is to obtain ~6x faster output with SPI communication between IMU and PSoC. Not yet working! and ECBv3 Board needs more hardware modifications like reset and interrupt pins, along with slave select and other SPI specific pins pulled to the right polarity.
-P08_IMU_BNO085_MAT6: Adaptation of IMU code to another ECB, this time with a different PSoC processor. Hardware pins are changed and performs same functionality as P06. **There is one significant hardware difference in this project where the enable pin of the 3.3V voltage regulator was not pulled high.** Therefore we manually pulled it high through the code, and called it ENABLE_3V_PIN.
-P09_MAT6_Bootloader: Bootloader project for MAT6 ECB.

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

###Chronological Timeline of IMU Data Reading Process
1. Initialization:
- The program starts by starting all hardware not associated with the IMU. This includes PWM, Buzzer,and USBFS components. 
- Then the IMU setup function is called. This creates a Hardware Abstraction Layer (HAL) variable and initializes it by creating pointers to specific HAL functions within the SH2_hal_psoc5.c file.
- The function then proceeds to open an instance of the HAL variable with the SH2_open() function within sh2.c.
- The Sh2_open function performs a variety of actions including setting up callback functions, setting up important initialization flags, such as **resetComplete and advertDone**, and calling the library defined SHTP_open() function. The most important thing to notice about the SHTP_open() function during initialization is that it is watching for an SHTP advertisement packet coming from the IMU. During initialization, the program sets a flag for Advert Requested, which can be seen in the SHTP_open() function. - The SHTP advertisement packet, like all transfers from the IMU is preceded by an SHTP header, and contains the necessary information to configure the SHTP communication protocols to be able to receive useful data from the IMU. From observation, the SHTP advertisement packet should be around 276 bytes. Specifically, it contains channel assignments. For more information on the advertisement packet, take a look at the SHTP datasheet, and SH2 Specifics of SHTP Datasheet. In addition, the SHTP advertisement will be outputted from the IMU once it senses that its reset pin has been activated. Therefore, the active reset pin is necessary for this IMU.
- The SHTP open() function, in addition, calls the user generated SH2_i2c_HAL_open() function.
- The SH2_i2c_HAL_open() initializes IMU hardware components such as the timer, I2C bus, and interrupt andler. The function also manually resets the IMU active low, which prompts an advertisement packet.
- When the SH2_Hal_open() function returns, the SHTP_open() function finishes and also returns back to SH2_open().
- The rest of the SH2_open() function is spent servicing the IMU in order to read this advertisement packet in a while loop. There is a time-out condition where if the IMU doesn't send the advertisement in time or at all, the time-out will occur and the IMU will not be properly initialized. The program must service the IMU so that the entire advertisement packet is read. This initilization servicing is performed by the shtp_service() function. It ultimately calls the user generated sh2_i2c_hal_read() function and also applies the proper callbacks to the recieved information to parse the data for each transfer.
-if the advertisement packet successfully transfers, the MCU will now have a clear idea of how to communicate with the IMU.
-At the end of the Sh2_open() function, it will return SH2_OK, and the flags, pSh2->resetComplete and pSh2->advertDone should be true. If this is not the situation, the initialization did not occur successfully.
- Otherwise, the IMU should now be ready to be serviced and output data and recieve commands. 
- The SH2_HAL_open() function 	
2. Servicing
3. Printing Data

# Useful Links
- [BNO085 IMU Datasheet](https://www.hillcrestlabs.com/downloads/bno080-datasheet)
- [SH2 Datasheet](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- [SH2 specifics of SHTP Datasheet](https://www.hillcrestlabs.com/downloads/sh-2-shtp-reference-manual)
- [SHTP Datasheet](https://cdn.sparkfun.com/assets/7/6/9/3/c/Sensor-Hub-Transport-Protocol-v1.7.pdf)
- [STM BNO085 Implementation](https://github.com/hcrest/sh2-demo-nucleo)
- [Arduino BNO085 Implementation](https://github.com/sparkfun/Qwiic_IMU_BNO080/blob/master/Firmware/Tester/Tester.ino)
- [Bootloader and Bootloadable User Guide](https://docs.google.com/document/d/1NsbHpMEDuHHZEE9elAJRFjD2x9ydBso8VCzAN2paOsE/edit)
- [Miniprog 3 User Guide](https://www.cypress.com/file/44091/download)


