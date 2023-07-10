#include <Arduino.h>
#include <Config.h>
#include <Defines.h>
#include <SimpleFOC.h>

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
BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS); // int pp,  float R = NOT_SET [Ohm], float KV = NOT_SET [rpm/V], float L = NOT_SET [H]
//BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS, 0.1664, 16.0, 0.00036858);  
BLDCDriver6PWM driver = BLDCDriver6PWM( BLDC_BH_PIN,BLDC_BL_PIN,  BLDC_GH_PIN,BLDC_GL_PIN,  BLDC_YH_PIN,BLDC_YL_PIN );

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

  for (int i=0; i<HALL_Count; i++)  aoHall[i].Init();

  
  sensor.init();  // initialize sensor hardware
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
   
  motor.linkSensor(&sensor);  // link the motor to the sensor

  driver.voltage_power_supply = 3.6 * BAT_CELLS; // power supply voltage [V]
  motor.voltage_limit = driver.voltage_power_supply/2; // should be half the power supply
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
  motor.voltage_sensor_align  = 1;                            // aligning voltage
  motor.foc_modulation        = FOCModulationType::Trapezoid_120;
  //motor.foc_modulation        = FOCModulationType::SinePWM;   // choose FOC modulation (optional)
  //motor.foc_modulation        = FOCModulationType::SpaceVectorPWM;
  motor.controller            = MotionControlType::torque;    // set motion control loop to be used
  motor.torque_controller     = TorqueControlType::voltage;
  
/*  
  // succeeds but motor.initFOC(2.09,Direction::CCW)  hangs 
  // motor.initFOC() will turn motor forward and backward a bit and then also hangs
  // We probably need a SimpleFOC/src/current_sense/hardware_specific/gd32/gd32_mcu.cpp
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
  motor.init();
  motor.initFOC(2.09,Direction::CCW); // Start FOC without alignment
  //motor.initFOC(NOT_SET,Direction::CW);
  //motor.initFOC();// align sensor and start FOC

  Blink(3,oLedGreen);

  iLoopStart = millis();    // this will at least take 1 ms;
  OUT2N("setup needed ms",iLoopStart)
}

unsigned long iTimeSend = 0;
long iMicrosLast = 0;
long iMicrosMax = 0;
float fSpeedMax = 0;
float fSpeedMin = 0;
float fSpeedAvgMax = 13;
float fSpeedAvgMin = -13;

unsigned int iFOC = 0;

unsigned long iOptimize = 0;
float fOptimize = 0;
/*
float fZero_Step = 0.005;
int iZero_Direction = 1;
*/
void loop()
{
  long iMicrosNow = _micros();
  long iMicros = iMicrosNow - iMicrosLast;
  if (iMicrosMax < iMicros) iMicrosMax = iMicros;
  iMicrosLast = iMicrosNow;

  float fSpeed;
  if (motor.enabled)  // set by successful motor.init() at the end of setup()
  {
    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz 
    motor.loopFOC();

    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    //float fSpeed = (motor.voltage_limit)  * (ABS(	(float)(((millis()-iLoopStart)/50 + 100) % 400) - 200) - 100)/100;
    fSpeed = (motor.voltage_limit) * (ABS((float)(((millis()-iLoopStart)/10 + 250) % 1000) - 500) - 250)/250;
    //fSpeed = 3.0;
    motor.move(fSpeed);
  }

  iFOC++;
  float fVelocity = sensor.getVelocity();
  if (fSpeedMax < fVelocity) fSpeedMax = fVelocity;
  if (fSpeedMin > fVelocity) fSpeedMin = fVelocity;
  if (-fSpeed > driver.voltage_limit)  // will be clamped and therefore be constant
    fSpeedAvgMax = 0.99 * fSpeedAvgMax + 0.01 * fVelocity;
  else if (fSpeed > driver.voltage_limit)  // will be clamped and therefore be constant
    fSpeedAvgMin = 0.99 * fSpeedAvgMin + 0.01 * fVelocity;

  unsigned long iNow = millis();

  //aoLed[0].Set((iNow%1000) < 500);
  //aoLed[1].Set(aoHall[0].Get());
  //aoLed[0].Set(aoHall[0].Get() ? (iNow%200) < 100 : (iNow%1000) < 500);

  unsigned int iTime = (iNow/1000)%12;
  if (iTime < 5)
    for (int i=0; i<HALL_Count; i++)  aoLed[i].Set( aoHall[i].Get() );
  else if (iTime < 6 || iTime >= 11)
    for (int i=0; i<LED_Count; i++)  aoLed[i].Set( (iNow%200) < 100 );
  else if (iTime < 11)
  {
    int iPos = ((int)ABS(5*sensor.getAngle())) % HALL_Count;
    for (int i=0; i<HALL_Count; i++)  aoLed[i].Set(i==iPos);
  }

  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;


  if (oOnOff.Get()) oKeepOn.Set(false);



  DEBUG( 
    //OUT2T("SystemCoreClock",SystemCoreClock ) 
    OUT2T(fSpeedMin , fSpeedMax)
    OUT2T(fSpeedAvgMin , fSpeedAvgMax)
    OUT2T(abs(fSpeedMin+fSpeedMax), abs(fSpeedAvgMin+fSpeedAvgMax))
    //OUT2T("fLinAdd%",sensor.fLinAdd*100)
    OUT2T(fOptimize*1000, motor.zero_electric_angle*1000)
    OUT2T( iMicros , iMicrosMax )
    //OUT2T( 1000.0f/iMicros , 1000.0f/(float)iMicrosMax )

/*
    //for (int i=0; i<HALL_Count; i++)  OUT2T(i,aoHall[i].Get())
    OUT2T("angle",sensor.getAngle()) 
    OUT2T("speed",sensor.getVelocity())

    OUT2T(sensor.fAngleOrgLast , sensor.fAngleOrg)
    OUT2T(sensor.fAngleLinLast , sensor.fAngleLin)
    //OUT2T("fAngleNew" , sensor.fAngleLagrange)
    OUT2T(sensor.aiAngle[0] , sensor.aiTime[0]-sensor.aiTime[1])
    OUT2T(sensor.iTimeSinceOld , sensor.iTimeSince) // sensor.fGradient1*1000
    //OUT2T(sensor.fGradient1Last*1000,sensor.fGradient1*1000)
*/
    if (current_sense.initialized)
    {
      PhaseCurrent_s currents = current_sense.getPhaseCurrents();
      float current_magnitude = current_sense.getDCCurrent();
      OUT2T("mA",current_magnitude*1000)  // milli Amps
      OUT2T("B mA",currents.b*1000)  // milli Amps
      OUT2T("C mA",currents.c*1000)  // milli Amps
    }
    OUTN(iFOC)
  )

  //fOptimize = -0.2  * (ABS(	(float)((iOptimize++ + 20) % 80) - 40) - 20)/20;
  //motor.zero_electric_angle = 2.09 + fOptimize;
  //sensor.fLinAdd = 0.5 - 0.5  * (ABS(	(float)((iOptimize++ + 20) % 80) - 40) - 20)/20;


  iMicrosMax = fSpeedMax = fSpeedMin = iFOC = 0;
  
  //Serial2.println("test of the master/slave uart rx/tx PA3/PA2"); // when using Serial1 as DEBUG_UART

}
