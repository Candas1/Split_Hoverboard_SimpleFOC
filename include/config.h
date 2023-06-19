// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

// LAYOUT_2_X is used in defines.h
#define LAYOUT_2_0	// https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
//#define LAYOUT_2_1	// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
//#define LAYOUT_2_2	// 2023/05/11 only MASTER and TEST_SPEED: motor is spinning but needs a push to startup :-/
//#define LAYOUT_2_4	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/3
//#define LAYOUT_2_5	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/11
//#define LAYOUT_2_NONE	// for testing the binary upload with no harmful pin definitions

// SEGGER RTT Debug
//#define DEBUG_STLINK rtt              // Uncomment to enable DEBUG over stlink dongle

// log debug output over master/slave uart
#define DEBUG_UART  Serial2
#ifdef DEBUG_UART
  #define DEBUG_UART_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

#define TIME_SEND           500         // [ms] Sending time interval


#endif //  CONFIG_H
