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

#ifdef DEBUG_SERIAL
  #include <RTTStream.h>
  RTTStream rtt;
#endif


class CIO
{	
private:
	int m_iPin;
public:
	CIO(int iPin);
	void Setup(int iType = INPUT_PULLUP);
	void Set(bool bOn = true);
	bool Get(void);
};

CIO::CIO(int iPin){	m_iPin = iPin;  }
void CIO::Setup(int iType){  pinMode(m_iPin, iType);  }	
void CIO::Set(bool bOn){ digitalWrite(m_iPin,bOn); }	
bool CIO::Get(void){  return digitalRead(m_iPin); }


CIO oLedGreen   = CIO(LED_GREEN);
CIO oLedOrange  = CIO(LED_ORANGE);
CIO oLedRed     = CIO(LED_RED);

CIO aoLed[5] = {oLedGreen, oLedOrange, oLedRed, CIO(UPPER_LED_PIN), CIO(LOWER_LED_PIN) };
#define LED_Count 3

CIO aoHall[3] = {CIO(HALL_A_PIN), CIO(HALL_B_PIN), CIO(HALL_C_PIN) };
#define HALL_Count 3


// ########################## SETUP ##########################
void setup(){
  
  for (int i=0; i<LED_Count; i++) 
  {
    aoLed[i].Setup(OUTPUT);
    aoLed[i].Set(HIGH);
    delay(500);
    aoLed[i].Set(LOW);
  }

  for (int i=0; i<HALL_Count; i++)  aoHall[i].Setup(INPUT);

  // init5ialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
    //robo using HOVER_SERIAL instead of DEBUG_SERIAL=ST-Link
    Serial2.println("Hoverboard Serial v1.0, this is HOVER_SERIAL");
  #endif

  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Hoverboard Serial v1.0");
  #endif

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

  #ifdef HOVER_SERIAL //robo using HOVER_SERIAL instead of DEBUG_SERIAL=ST-Link
    for (int i=0; i<HALL_Count; i++)  {Serial2.print(aoHall[i].Get()); Serial2.print("  ");}
    //Serial2.print(sensor.getAngle());Serial2.print("  "); Serial2.print(sensor.getVelocity());Serial2.print("  ");
    Serial2.println();
  #endif
}