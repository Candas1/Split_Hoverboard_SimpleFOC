// Gen 2.x	start with this file to add a new layout :-)

#define TODO_PIN	PB15	// B15 is not accessibla on the smaller QFN32 32 pin MCU version

// LED defines
#define LED_GREEN 	TODO_PIN
#define LED_ORANGE 	TODO_PIN
#define LED_RED 	TODO_PIN

#define UPPER_LED_PIN TODO_PIN
#define LOWER_LED_PIN TODO_PIN


// Hall sensor defines
#define HALL_A_PIN TODO_PIN
#define HALL_B_PIN TODO_PIN
#define HALL_C_PIN TODO_PIN


// Brushless Control DC (BLDC) defines
// Channel G
#define BLDC_GH_PIN TODO_PIN
#define BLDC_GL_PIN TODO_PIN
// Channel B
#define BLDC_BH_PIN TODO_PIN
#define BLDC_BL_PIN TODO_PIN
// Channel Y
#define BLDC_YH_PIN TODO_PIN
#define BLDC_YL_PIN TODO_PIN

// BLDC low-side current sense pins (R_ds)
#define BLDC_CUR_Rds 0.008	// R_ds of the low side n-Chnannel mosfets
#define BLDC_CUR_Gain 50.0	// gain of the op-amp to amplify voltage_drain-source
#define BLDC_CUR_G_PIN _NC	// simpleFOC can handle 2 or 3 current sense pins
#define BLDC_CUR_B_PIN TODO_PIN
#define BLDC_CUR_Y_PIN TODO_PIN


// Self hold defines
// Set output high to keep the latch ON. Set low to turn the board OFF
#define SELF_HOLD_PIN TODO_PIN

// Button defines
// Usually used as INPUT but could be used as analog input if available on the pin to detect button press with low voltage batteries  
#define BUTTON_PIN TODO_PIN
