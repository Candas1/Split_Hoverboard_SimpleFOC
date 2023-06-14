#include <Arduino.h>
#include <Config.h>
#include <Defines.h>
#include <SimpleFOC.h>

/*
#ifdef DEBUG_UART // use Serial1 with differen rx/tx pins as DEBUG_UART
  HardwareSerial oSerialSteer(PB7, PB6,0);  // 1 = uart_index = Serial2 ; 0 = uart_index = Serial1
  #undef DEBUG_UART
  #define DEBUG_UART oSerialSteer
#endif
*/

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, 15);

// Interrupt routine initialization
// channel A and B callbacks
void doA()
{
  sensor.handleA();
  //OUT("A")
}
void doB()
{
  sensor.handleB();
  //OUT("B")
}
void doC()
{
  sensor.handleC();
  //OUT("C")
}

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(15);
BLDCDriver6PWM driver = BLDCDriver6PWM( BLDC_BH_PIN,BLDC_BL_PIN,  BLDC_GH_PIN,BLDC_GL_PIN,  BLDC_YH_PIN,BLDC_YL_PIN );
//LowPassFilter filter = LowPassFilter(0.01);

#ifdef DEBUG_STLINK
  #include <RTTStream.h>
  RTTStream rtt;
#endif


class CIO
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

//CIO oPB6   = CIO(PB6,OUTPUT);


CIO oKeepOn = CIO(SELF_HOLD_PIN,OUTPUT);
CIO oOnOff = CIO(BUTTON_PIN);

CIO oLedGreen   = CIO(LED_GREEN,OUTPUT);
CIO oLedOrange  = CIO(LED_ORANGE,OUTPUT);
CIO oLedRed     = CIO(LED_RED,OUTPUT);

CIO aoLed[5] = {oLedGreen, oLedOrange, oLedRed, CIO(UPPER_LED_PIN,OUTPUT), CIO(LOWER_LED_PIN,OUTPUT) };
#define LED_Count 5

CIO aoHall[3] = {CIO(HALL_A_PIN), CIO(HALL_B_PIN), CIO(HALL_C_PIN) };
#define HALL_Count 3

CIO aoBLDC[6] = {CIO(BLDC_BH_PIN),CIO(BLDC_BL_PIN),CIO(BLDC_GH_PIN),CIO(BLDC_GL_PIN),CIO(BLDC_YH_PIN),CIO(BLDC_YL_PIN)};

void LedError(int iError)
{
  for (int j=0; j<iError; j++)
  {
    if (j)  delay(100);
    oLedRed.Set(HIGH);
    delay(100);
    oLedRed.Set(LOW);
  }

}

unsigned long iI2c = 0;
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() 
{
  iI2c++;
  OUTLN("an i2c request")

  //SerialWrite(Wire,oC);
  Wire.write(1);
  Wire.write(2);
  Wire.write(3);  // CRC8((byte*)&oC,sizeof oC)
}

void receiveEvent(int iReceived) 
{
  iI2c++;
  OUT2T("\ni2c received",iReceived)

  char temp[iReceived+2];
  temp[iReceived] = 0;
  for (int i=0; i<iReceived; i++) temp[i] = Wire.read();
  OUTLN(temp);

}

// ########################## SETUP ##########################
void setup()
{

  int iRX0old = PA10;
  int iTX0old = PA9;

  int iRX0new = PB7;
  int iTX0new = PB6;

  int iPinRX0 = RX0;
  int iPinTX0 = TX0;

  oKeepOn.Init();
  oKeepOn.Set(true);  // now we can release the on button :-)
  oOnOff.Init();

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event


  #ifdef DEBUG_UART
    DEBUG_UART.begin(DEBUG_UART_BAUD);
  #endif
  //Serial2.begin(DEBUG_UART_BAUD); // when using Serial1 as DEBUG_UART

  OUTLN("Split Hoverboards with C++ SimpleFOC :-)")

  //oPB6.Init();
  
  for (int i=0; i<LED_Count; i++) 
  {
    aoLed[i].Init();
    aoLed[i].Set(HIGH);
    delay(100);
    aoLed[i].Set(LOW);
  }


  for (int i=0; i<HALL_Count; i++)  aoHall[i].Init();

  // init5ialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

   // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 30;
  driver.voltage_limit = 3;
  if (!driver.init())
  {
    LedError(10);
    for(int i=0; i<6; i++)  aoBLDC[i].Init();  // set back to input to free the blocked motor
  }

  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 0.5;
  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::Trapezoid_120;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.target = 1;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

}

unsigned long iTimeSend = 0;
void loop()
{
  sensor.update();

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move();

  unsigned long iNow = millis();

  //oPB6.Set((iNow%100) < 50);

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

  #ifdef SERIALDEBUGXX
    if (SERIALDEBUG.available()) 
    {
      while (SERIALDEBUG.available() )
      {
        SERIALDEBUG.write(SERIALDEBUG.read());
      }
    }
  #endif



  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;


  if (oOnOff.Get()) oKeepOn.Set(false);

  #ifdef SERIALDEBUG
    if (SERIALDEBUG.available()) 
    {
      while (SERIALDEBUG.available() )
      {
        SERIALDEBUG.write(SERIALDEBUG.read());
      }
      SERIALDEBUG.println();
    }
  #endif

  DEBUG( 
    OUT2T("GD32",iNow) 
    for (int i=0; i<HALL_Count; i++)  OUT2T(i,aoHall[i].Get())
    OUT2T("angle",sensor.getAngle()) 
    OUT2T("speed",sensor.getVelocity())
    OUT2("iI2c",iI2c);
    OUTLN()
  )

  //Serial2.println("test of the master/slave uart rx/tx PA3/PA2"); // when using Serial1 as DEBUG_UART
}