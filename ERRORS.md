xPack Open On-Chip Debugger 0.12.0-01004-g9ea7f3d64-dirty (2023-01-30-15:04)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
debug_level: 1

srst_only separate srst_nogate srst_open_drain connect_deassert_srst

Error: libusb_open() failed with LIBUSB_ERROR_NOT_FOUND
Error: open failed
in procedure 'program'
** OpenOCD init failed **
shutdown command invoked

*** [upload] Error 1

Processing genericSTM32F401RE (platform: ststm32; board: nucleo_f401re; framework: cmsis)
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Verbose mode can be enabled via `-v, --verbose` option
CONFIGURATION: https://docs.platformio.org/page/boards/ststm32/nucleo_f401re.html
PLATFORM: ST STM32 (19.2.0) > ST Nucleo F401RE
HARDWARE: STM32F401RET6 84MHz, 96KB RAM, 512KB Flash
DEBUG: Current (stlink) On-board (stlink) External (blackmagic, cmsis-dap, jlink)
PACKAGES: 
 - framework-cmsis @ 2.50501.200527 (5.5.1) 
 - framework-cmsis-stm32f4 @ 2.6.11 
 - tool-dfuutil @ 1.11.0 
 - tool-dfuutil-arduino @ 1.11.0 
 - tool-ldscripts-ststm32 @ 0.2.0 
 - tool-openocd @ 3.1200.0 (12.0) 
 - tool-stm32flash @ 0.7.0
 - toolchain-gccarmnoneeabi @ 1.70201.0 (7.2.1)
LDF: Library Dependency Finder -> https://bit.ly/configure-pio-ldf
LDF Modes: Finder ~ chain, Compatibility ~ soft
Found 0 compatible libraries
Scanning dependencies...
No dependencies
Building in release mode
Checking size .pio\build\genericSTM32F401RE\firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [          ]   0.0% (used 28 bytes from 98304 bytes)
Flash: [          ]   0.1% (used 472 bytes from 524288 bytes)
Configuring upload protocol...
AVAILABLE: blackmagic, cmsis-dap, jlink, mbed, stlink
CURRENT: upload_protocol = stlink
Uploading .pio\build\genericSTM32F401RE\firmware.elf
xPack Open On-Chip Debugger 0.12.0-01004-g9ea7f3d64-dirty (2023-01-30-15:04)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
debug_level: 1

srst_only separate srst_nogate srst_open_drain connect_deassert_srst

[stm32f4x.cpu] halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x080001d4 msp: 0x20018000
** Programming Started **
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
shutdown command invoked
--- Terminal on COM7 | 9600 8-N-1
--- Available filters and text transformations: colorize, debug, default, direct, hexlify, log2file, nocontrol, printable, send_on_enter, time
--- More details at https://bit.ly/pio-monitor-filters
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H


Program
 received signal SIGINT, Interrupt.
0x08000370 in i2c1_read_bytes (dev_addr=dev_addr@entry=83 'S', reg=reg@entry=0 '\000', buf=buf@entry=0x20017f6f "", len=len@entry=1 '\001') at src\main.c:162
162	    while (!(I2C1->SR1 & I2C_SR1_SB));
When running the debugger, the program seems to hang on this line in i2c1_read_bytes function. Can you explain what is going on here? 

Linking .pio\build\genericSTM32F401RE\firmware.elf
c:/users/u36604/.platformio/packages/toolchain-gccarmnoneeabi/bin/../lib/gcc/arm-none-eabi/7.2.1/../../../../arm-none-eabi/bin/ld.exe: cannot find -lC:\Users\U36604\Projects\Smart Spindle\_printf_float
collect2.exe: error: ld returned 1 exit status
*** [.pio\build\genericSTM32F401RE\firmware.elf] Error 1
===============================================================

