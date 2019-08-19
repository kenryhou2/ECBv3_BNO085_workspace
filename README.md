## PSOC_EBC3.1_WS

Tested build environment:
```Windows 10 64-bits #PSOC Creator 4.2```

#Hardware

- ECB v3.1 board: Version3 rev 1 of the Embedded Controller Board(ECB), a small form factor swiss army knife for bio-robotics applications.
- MAT6 board: ECB for MAT6 6 legged robo with force sensing, and BNO085 IMU integration on the feet. 
- CY8CKIT-002 PSoC® MiniProg3 Program and Debug Kit.
- MicroUSB cable

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
-P08_IMU_BNO085_MAT6: Adaptation of IMU code to another ECB, this time with a different PSoC processor. Hardware pins are changed and performs same functionality as P06.
-P09_MAT6_Bootloader: Bootloader project for MAT6 ECB.

##P06 Project Details.

#Quick Start
- 

#Full Procedure Description
- Initialization
- Servicing
- Printing Data

