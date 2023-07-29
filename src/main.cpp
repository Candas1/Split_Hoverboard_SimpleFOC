#include <Arduino.h>
#include <Config.h>
#include <Defines.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/smoothing/SmoothingSensor.h>

#ifdef DEBUG_STLINK
  #include <RTTStream.h>
  RTTStream rtt;
#endif

/*
#ifdef DEBUG_UART // use Serial1 with differen rx/tx pins as DEBUG_UART
  HardwareSerial oSerialSteer(PB7, PB6,0);  // 1 = uart_index = Serial2 ; 0 = uart_index = Serial1
  #undef DEBUG_UART
  #define DEBUG_UART oSerialSteer
#endif
*/

// HallSensor(int hallA, int hallB , int hallC , int pp)  = HallSensor A, B and C pins , pp = pole pairs
HallSensor sensor = HallSensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, BLDC_POLE_PAIRS);

// Interrupt routine initialization
void doA()  { sensor.handleA(); } // channel callbacks
void doB()  { sensor.handleB(); }
void doC()  { sensor.handleC(); }

// BLDC motor & driver instance
//BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS); // int pp,  float R = NOT_SET [Ohm], float KV = NOT_SET [rpm/V], float L = NOT_SET [H]
//BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS, 0.1664, 16.0, 0.00036858);
BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS, NOT_SET, NOT_SET, 0.00036858); 
BLDCDriver6PWM driver = BLDCDriver6PWM( BLDC_BH_PIN,BLDC_BL_PIN,  BLDC_GH_PIN,BLDC_GL_PIN,  BLDC_YH_PIN,BLDC_YL_PIN );

// instantiate the smoothing sensor, providing the real sensor as a constructor argument
SmoothingSensor smooth = SmoothingSensor(sensor, motor);

// shunt resistor value , gain value,  pins phase A,B,C
LowsideCurrentSense current_sense = LowsideCurrentSense(BLDC_CUR_Rds, BLDC_CUR_Gain, BLDC_CUR_G_PIN, BLDC_CUR_B_PIN, BLDC_CUR_Y_PIN);


class CIO   // little helper class to demonstrate object oriented programming
{	
private:
	int m_iPin;
	int m_iType;  // OUTPUT, INPUT, INPUT_PULLUP, INPUT_PULLDOWN
public:
	CIO(int iPin, int iType);
	void Init();
	void Set(bool bOn = true);
	bool Get(void);
};
CIO::CIO(int iPin, int iType=INPUT){	m_iPin = iPin;  m_iType = iType;}
void CIO::Init(){  pinMode(m_iPin, m_iType);  }	
void CIO::Set(bool bOn){ digitalWrite(m_iPin,bOn); }	
bool CIO::Get(void){  return digitalRead(m_iPin); }


CIO oKeepOn = CIO(SELF_HOLD_PIN,OUTPUT);
CIO oOnOff = CIO(BUTTON_PIN);

CIO oLedGreen   = CIO(LED_GREEN,OUTPUT);
CIO oLedOrange  = CIO(LED_ORANGE,OUTPUT);
CIO oLedRed     = CIO(LED_RED,OUTPUT);

CIO aoLed[5] = {oLedGreen, oLedOrange, oLedRed, CIO(UPPER_LED_PIN,OUTPUT), CIO(LOWER_LED_PIN,OUTPUT) };
#define LED_Count 5   // reduce to test led pins..

CIO aoHall[3] = {CIO(HALL_A_PIN), CIO(HALL_B_PIN), CIO(HALL_C_PIN) };
#define HALL_Count 3

CIO aoBLDC[6] = { CIO(BLDC_BH_PIN), CIO(BLDC_BL_PIN), CIO(BLDC_GH_PIN), CIO(BLDC_GL_PIN), CIO(BLDC_YH_PIN), CIO(BLDC_YL_PIN) };

void Blink(int iBlinks, CIO& oLed = oLedRed)
{
  for (int j=0; j<iBlinks; j++)
  {
    if (j)  delay(100);
    oLed.Set(HIGH);
    delay(100);
    oLed.Set(LOW);
  }
}


unsigned long iLoopStart = 0;   // time setup() finishes and loop() starts

Commander command = Commander(rtt);

float target = 0;

void doTarget(char* cmd) { command.scalar(&target, cmd); }
void doPhaseCorrection(char* cmd) {
  float phase_correction;
  command.scalar(&phase_correction, cmd);
  smooth.phase_correction = phase_correction;
}

void doZero(char* cmd) {
  float zero_angle;
  command.scalar(&zero_angle, cmd);
  motor.zero_electric_angle = zero_angle;
}
void enableSmoothing(char* cmd) {
  float enable;
  command.scalar(&enable, cmd);
  motor.linkSensor(enable == 0 ? (Sensor*)&sensor : (Sensor*)&smooth);
}

void doLPF(char* cmd) {
  float LPF;
  command.scalar(&LPF, cmd);
  motor.LPF_velocity.Tf = LPF;
}

// ########################## SETUP ##########################
void setup()
{
  #ifdef DEBUG_UART
    DEBUG_UART.begin(DEBUG_UART_BAUD);
    SimpleFOCDebug::enable(&DEBUG_UART);
    motor.useMonitoring(DEBUG_UART);
  #endif
  //Serial2.begin(DEBUG_UART_BAUD); // when using Serial1 as DEBUG_UART

  #ifdef DEBUG_STLINK
    SimpleFOCDebug::enable(&rtt);
    motor.useMonitoring(rtt);
  #endif

  oKeepOn.Init();
  oKeepOn.Set(true);  // now we can release the on button :-)
  oOnOff.Init();

  OUTN("Split Hoverboards with C++ SimpleFOC :-)")

  for (int i=0; i<LED_Count; i++) 
  {
    aoLed[i].Init();
    aoLed[i].Set(HIGH);
    delay(100);
    aoLed[i].Set(LOW);
  }

  // For time measurement with the oscilloscope
  pinMode(PB6,OUTPUT);
  pinMode(PB7,OUTPUT);

  for (int i=0; i<HALL_Count; i++)  aoHall[i].Init();

  
  sensor.init();  // initialize sensor hardware
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  
  // set SmoothingSensor phase correction for hall sensors
  smooth.phase_correction = -_PI_6;
  motor.linkSensor(&smooth); // link the motor to the smoothing sensor

  driver.voltage_power_supply = 26; // 3.6 * BAT_CELLS; // power supply voltage [V]
  motor.voltage_limit = driver.voltage_power_supply * 0.58; // should be half the power supply
  if (driver.init())
  {
    driver.enable();
    Blink(2,oLedOrange);
  }
  else
  {
    Blink(10);
    for(int i=0; i<6; i++)  aoBLDC[i].Init();  // set back to input to free the blocked motor
    return; // cancel simpleFOC setup
  }

  motor.linkDriver(&driver);  // link driver
  motor.voltage_sensor_align  = 2;                            // aligning voltage
  motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
  motor.controller            = MotionControlType::torque;    // set motion control loop to be used
  motor.torque_controller     = TorqueControlType::voltage;
  //motor.motion_downsample = 100;

  // velocity low pass filtering time constant
  motor.PID_velocity.P  = 0.6f;
  motor.PID_velocity.I  = 5.0f;
  motor.LPF_velocity.Tf = 0.05f;

/*  
  if (current_sense.init())
  {
    motor.linkCurrentSense(&current_sense);
    Blink(3,oLedOrange);
  }
  else
  {
    Blink(5);
    OUTN("current_sense.init() failed.")
    return;   // cancel simpleFOC setup
  }
*/

  // initialize motor
  motor.sensor_direction=Direction::CCW;
  motor.zero_electric_angle = 2.09;     // use the real value!
  motor.init();
  motor.initFOC(); // Start FOC without alignment
  

  // add target command T
  command.add('T', doTarget, "target voltage");
  // add smoothing enable/disable command E (send E0 to use hall sensor alone, or E1 to use smoothing)
  command.add('E', enableSmoothing, "enable smoothing");
  command.add('P', doPhaseCorrection, "phase correction");
  command.add('Z', doZero, "zero electrical angle");
  command.add('F', doLPF, "LPF");

  Blink(3,oLedGreen);
}

unsigned long iTimeSend = 0;
float fSpeed;

void loop()
{
  
  if (motor.enabled)  // set by successful motor.init() at the end of setup()
  {
    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    GPIO_BOP(GPIOB) = (uint32_t)GPIO_PIN_6; 
    motor.loopFOC();
    GPIO_BC(GPIOB) = (uint32_t)GPIO_PIN_6;

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    //fSpeed = (motor.voltage_limit) * (ABS((float)(((millis()-iLoopStart)/10 + 250) % 1000) - 500) - 250)/250;
    
    GPIO_BOP(GPIOB) = (uint32_t)GPIO_PIN_7;
    motor.move(target);
    GPIO_BC(GPIOB) = (uint32_t)GPIO_PIN_7;

  }

  // user communication
  command.run();

  if (oOnOff.Get()) oKeepOn.Set(false);

  /*
  DEBUG(
    if (current_sense.initialized)
    {
      PhaseCurrent_s currents = current_sense.getPhaseCurrents();
      float current_magnitude = current_sense.getDCCurrent();
      OUT2T("mA",current_magnitude*1000)  // milli Amps
      OUT2T("B mA",currents.b*1000)  // milli Amps
      OUT2T("C mA",currents.c*1000)  // milli Amps
    }
  )
  */
}
