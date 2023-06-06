#include <Arduino.h>
#include <Config.h>
#include <Defines.h>
#include <SimpleFOC.h>

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
  //Serial2.print("A");
}
void doB()
{
  sensor.handleB();
  //Serial2.print("B");
}
void doC()
{
  sensor.handleC();
  //Serial2.print("C");
}


BLDCDriver6PWM driver = BLDCDriver6PWM( BLDC_BH_PIN,BLDC_BL_PIN,  BLDC_GH_PIN,BLDC_GL_PIN,  BLDC_YH_PIN,BLDC_YL_PIN );


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

CIO oKeepOn = CIO(SELF_HOLD_PIN,OUTPUT);
CIO oOnOff = CIO(BUTTON_PIN);

CIO oLedGreen   = CIO(LED_GREEN,OUTPUT);
CIO oLedOrange  = CIO(LED_ORANGE,OUTPUT);
CIO oLedRed     = CIO(LED_RED,OUTPUT);

CIO aoLed[5] = {oLedGreen, oLedOrange, oLedRed, CIO(UPPER_LED_PIN,OUTPUT), CIO(LOWER_LED_PIN,OUTPUT) };
#define LED_Count 3

CIO aoHall[3] = {CIO(HALL_A_PIN), CIO(HALL_B_PIN), CIO(HALL_C_PIN) };
#define HALL_Count 3

void LedError(int iError)
{
  for (int j=0; j<iError; j++)
  {
    if (j)  delay(100);
    oLedRed.Set(HIGH);
    //for (int i=0; i<LED_Count; i++) aoLed[i].Set(HIGH);
    delay(100);
    //for (int i=0; i<LED_Count; i++) aoLed[i].Set(HIGH);
    oLedRed.Set(LOW);
  }

}

// ########################## SETUP ##########################
void setup()
{
  oKeepOn.Init();
  oKeepOn.Set(true);  // now we can release the on button :-)
  oOnOff.Init();

  #ifdef DEBUG_UART
    DEBUG_UART.begin(DEBUG_UART_BAUD);
  #endif

  OUTLN("Split Hoverboards with C++ SimpleFOC :-)")

  
  for (int i=0; i<LED_Count; i++) 
  {
    aoLed[i].Init();
    aoLed[i].Set(HIGH);
    delay(300);
    aoLed[i].Set(LOW);
  }

  for (int i=0; i<HALL_Count; i++)  aoHall[i].Init();

  // init5ialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);

  if (!driver.init())  LedError(10);


}

unsigned long iTimeSend = 0;
void loop()
{
  sensor.update();

  unsigned long iNow = millis();

  //aoLed[0].Set((iNow%1000) < 500);
  //aoLed[1].Set(aoHall[0].Get());
  //aoLed[0].Set(aoHall[0].Get() ? (iNow%200) < 100 : (iNow%1000) < 500);

  unsigned int iTime = (iNow/1000)%12;
  if (iTime < 5)
    for (int i=0; i<LED_Count; i++)  aoLed[i].Set( aoHall[i].Get() );
  else if (iTime < 6 || iTime >= 11)
    for (int i=0; i<LED_Count; i++)  aoLed[i].Set( (iNow%200) < 100 );
  else if (iTime < 11)
  {
    int iPos = ((int)ABS(5*sensor.getAngle())) % HALL_Count;
    for (int i=0; i<LED_Count; i++)  aoLed[i].Set(i==iPos);
  }



  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;


  if (oOnOff.Get()) oKeepOn.Set(false);

  DEBUG( 
    for (int i=0; i<HALL_Count; i++)  OUT2T(i,aoHall[i].Get())
    OUT2T("angle",sensor.getAngle()) 
    OUT2("speed",sensor.getVelocity())
    OUTLN()
  )
}