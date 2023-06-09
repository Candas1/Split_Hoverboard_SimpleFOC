 
#define pin_LED 15

void setup() 
{
  pinMode(pin_LED, OUTPUT);
  Serial.begin(115200);
  Serial.println("ESP32 Mini S2 SerialPassthrough :-)");
  Serial1.begin(115200, SERIAL_8N1, 16, 18);  //, pin_RX, pin_TX
}

unsigned long iNow = 0;
unsigned long iTimeSend = 0;
#define TIME_SEND 15

unsigned int iCounter = 0;
void loop() 
{
  iNow = millis();
  digitalWrite(pin_LED, (iNow%1000) < 200 ? HIGH : LOW);   // turn the LED on (HIGH is the voltage level)

  //int i = ( iNow /32)  % 256; Serial.println(i);
  
  if (Serial.available())  // If anything comes in Serial (USB),
  {        
    Serial1.write(Serial.read());  // read it and send it out Serial1
  }

  if (Serial1.available()) // If anything comes in Serial1
  {       
    char c = Serial1.read();
    Serial.write(c);  // read it and send it out Serial (USB)
    //if (c == 10){Serial.print(iNow); Serial.print(":\t");}
  }

  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;

  Serial1.print(iCounter++);
  Serial1.print("_");
  Serial1.print(iNow);
  Serial1.print("\t");
}
