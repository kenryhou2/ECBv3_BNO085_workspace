## PSOC_EBC3.1_WS

Tested build environment:
```Windows 10 64-bits #PSOC Creator 4.2```

- ECB v3.1 board: Version3 rev 1 of the Embedded Controller Board(ECB), a small form factor swiss army knife for bio-robotics applications.
- CY8CKIT-002 PSoC® MiniProg3 Program and Debug Kit.
- MicroUSB cable

# Projects

- P00_Blink_LED: Basic IO toggle test
- P01_Bootloader: Bootloader project, please link any bootloadable project the bin/hex file generated to this folder.
- P02_Blink_LED_Bootloadable: Test project of using USBUART based bootloader to flash the ECB without MiniProg.
- P02_USBUART_Control_LED: Basic test for bi-directional USBUART communication.