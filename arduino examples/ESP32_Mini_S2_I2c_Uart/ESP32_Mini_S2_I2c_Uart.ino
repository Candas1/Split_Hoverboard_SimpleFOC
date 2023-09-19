#include <Wire.h>
#define I2C_DEV_ADDR 8


#define pin_LED 15

void setup() 
{
  pinMode(pin_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("ESP32 Mini S2 SerialPassthrough :-)");
  Serial1.begin(115200, SERIAL_8N1, 16, 18);  //, pin_RX, pin_TX
  //Serial1.begin(115200, SERIAL_8N1, 18, 16);  //, pin_RX, pin_TX

  Wire.begin(37,39);  // ESP32 S2 Mini
}

unsigned long iNow = 0;
unsigned long iTimeSend = 0;
#define TIME_SEND 500
void loop() 
{
  iNow = millis();
  digitalWrite(pin_LED, (iNow%500) < 200 ? HIGH : LOW);   // turn the LED on (HIGH is the voltage level)

  //int i = ( iNow /32)  % 256; Serial.println(i);
  
  if (Serial.available())  // If anything comes in Serial (USB),
  {        
    Serial1.write(Serial.read());  // read it and send it out Serial1
  }

  if (Serial1.available()) // If anything comes in Serial1
  {       
    Serial.write(Serial1.read());  // read it and send it out Serial (USB)
  }

  if (Wire.available()) // not yet working
  {
    Serial.print("I2c available: ");Serial.println(Wire.available());
  }
  
  
  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;

  Serial1.print("hello from ESP32 S2 Mini");

  //Wire.requestFrom(I2C_DEV_ADDR, 3);

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.print("i2c message");
  byte error = Wire.endTransmission();
  if (error != 0) Serial.println("i2c failed");  
}
