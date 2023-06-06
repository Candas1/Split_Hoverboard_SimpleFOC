// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

// LAYOUT_2_X is used in defines.h
//#define LAYOUT_2_0	// https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
//#define LAYOUT_2_1	// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
//#define LAYOUT_2_2	// 2023/05/11 only MASTER and TEST_SPEED: motor is spinning but needs a push to startup :-/
//#define LAYOUT_2_4	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/3
#define LAYOUT_2_5	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/11

// SEGGER RTT Debug
//#define DEBUG_SERIAL rtt              // Uncomment to enable DEBUG
#ifdef DEBUG_SERIAL
  #undef Serial
  #define Serial rtt
#endif

// Hover serial protocol
//#define HOVER_SERIAL                    // Send commands to the mainboard and receive feedback
#ifdef HOVER_SERIAL
  #define HOVER_SERIAL_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

#define TIME_SEND           100         // [ms] Sending time interval


#endif //  CONFIG_H
