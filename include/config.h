// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H


#if defined(PLATFORMIO)
  #ifdef STM32F103RC
    #define HOVER_GEN   1
    #define HOVER_LAYOUT	0

    // too late here, has to be a build flag #define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH false

    // call motor.initFOC() without parameters to auto align sensor and copy values from debug log
    #define MOTOR_zero_electric_offset  2.09  
    #define MOTOR_sensor_direction  Direction::CCW

    // log debug output over master/slave uart
    //#define DEBUG_UART  Serial
    //HardwareSerial oSerialSteer(PB11, PB10);  // short cable 5VT EFeru USART3 GPIO Configuration
    //#define DEBUG_UART oSerialSteer
    //#define DEBUG_STLINK rtt
  #else
    #define HOVER_GEN   2
    #define HOVER_LAYOUT	0

    // call motor.initFOC() without parameters to auto align sensor and copy values from debug log
    #define MOTOR_zero_electric_offset  2.09
    #define MOTOR_sensor_direction  Direction::CCW

    // SEGGER RTT Debug
    #define DEBUG_STLINK rtt              // Uncomment to enable DEBUG over stlink dongle

  #endif
#else
  // LAYOUT_x_y is used in defines.h
  #define HOVER_GEN   2
  #define HOVER_LAYOUT 0
  //  2_0	// https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
  //  2_1	// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
  //  2_2	// 2023/05/11 only MASTER and TEST_SPEED: motor is spinning but needs a push to startup :-/
  //  2_4	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/3
  //  2_5	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/11

  //  1_0	// old Gen 1 boards with two motors

    // call motor.initFOC() without parameters to auto align sensor and copy values from debug log
    #define MOTOR_zero_electric_offset  2.09
    #define MOTOR_sensor_direction  Direction::CCW

    // SEGGER RTT Debug
    #define DEBUG_STLINK rtt              // Uncomment to enable DEBUG over stlink dongle
    // log debug output over master/slave uart
    //#define DEBUG_UART  Serial2

#endif


#define BLDC_POLE_PAIRS   15    // all hoverboard motors have 15 pole pairs ?
#define BAT_CELLS         7     // battery number of cells. mostly 10 = 10s = 36V. Sometimes 7 = 7s = 25V

#ifdef DEBUG_UART
  #define DEBUG_UART_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

#define TIME_SEND           10000         // [ms] Sending time interval


#endif //  CONFIG_H