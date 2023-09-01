// Gen 2.0

// LED defines
#define LED_GREEN PA15
#define LED_ORANGE PA12
#define LED_RED PB3

#define UPPER_LED_PIN PA1
#define LOWER_LED_PIN PA0


// Hall sensor defines
#define HALL_A_PIN PB11
#define HALL_B_PIN PF1
#define HALL_C_PIN PC14


// Brushless Control DC (BLDC) defines
// Channel G
#define BLDC_GH_PIN PA10
#define BLDC_GL_PIN PB15
// Channel B
#define BLDC_BH_PIN PA9
#define BLDC_BL_PIN PB14
// Channel Y
#define BLDC_YH_PIN PA8
#define BLDC_YL_PIN PB13

// BLDC low-side current sense pins (R_ds)
#define BLDC_CUR_Rds 0.008	// R_ds of the low side n-Chnannel mosfets
#define BLDC_CUR_Gain -2.5	// gain of the op-amp to amplify voltage_drain-source
#define BLDC_CUR_G_PIN PB1	
#define BLDC_CUR_B_PIN PB0
#define BLDC_CUR_Y_PIN _NC  // simpleFOC can handle 2 or 3 current sense pins

// Self hold defines
// Set output high to keep the latch ON. Set low to turn the board OFF
#define SELF_HOLD_PIN PB2

// Button defines
// Usually used as INPUT but could be used as analog input if available on the pin to detect button press with low voltage batteries  
#define BUTTON_PIN PC15

#define VBAT PA4