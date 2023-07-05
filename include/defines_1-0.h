// Gen 2.x	start with this file to add a new layout :-)

#define TODO_PIN	PB7	// left motor hall sensor w

// LED defines
#define LED_GREEN 	PB5		// left motor hall sensor u
#define LED_ORANGE 	PB6		// left motor hall sensor v
#define LED_RED 	PB2		// LED_PIN

#define UPPER_LED_PIN TODO_PIN
#define LOWER_LED_PIN TODO_PIN


// Hall sensor defines
#define HALL_A_PIN PC10	// right motor hall sensor u
#define HALL_B_PIN PC11	// right motor hall sensor v
#define HALL_C_PIN PC12	// right motor hall sensor w


// Brushless Control DC (BLDC) defines
// Channel G
#define BLDC_GH_PIN PA8		// right motor u
#define BLDC_GL_PIN PB13
// Channel B
#define BLDC_BH_PIN PA9		// right motor v
#define BLDC_BL_PIN PB14
// Channel Y
#define BLDC_YH_PIN PA10	// right motor w
#define BLDC_YL_PIN PB15

// BLDC low-side current sense pins (R_ds)
#define BLDC_CUR_Rds 0.008	// R_ds of the low side n-Chnannel mosfets
#define BLDC_CUR_Gain 50.0	// gain of the op-amp to amplify voltage_drain-source
#define BLDC_CUR_G_PIN PC4	//	RIGHT_U_CUR_PIN
#define BLDC_CUR_B_PIN PC5	//	RIGHT_V_CUR_PIN
#define BLDC_CUR_Y_PIN _NC


// Self hold defines
// Set output high to keep the latch ON. Set low to turn the board OFF
#define SELF_HOLD_PIN PA5	// OFF_PIN

// Button defines
// Usually used as INPUT but could be used as analog input if available on the pin to detect button press with low voltage batteries  
#define BUTTON_PIN PA1	// BUTTON_PIN
