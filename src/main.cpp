#include <Arduino.h>

void setup()
{

  pinMode(PA2,OUTPUT); // For time measurement with the oscilloscope
  pinMode(PA5,OUTPUT); // Latch

  digitalWrite(PA5,true); // Latch High
 
}

float battery_voltage1,battery_voltage2,battery_voltage3;

void loop()
{  

    GPIOA->BSRR = GPIO_BSRR_BS2; // Output High
    battery_voltage1 = analogRead(PC2_ALT0);
    GPIOA->BRR = GPIO_BRR_BR2; // Output Low
    
    GPIOA->BSRR = GPIO_BSRR_BS2; // Output High
    battery_voltage2 = analogRead(PC2_ALT1);
    GPIOA->BRR = GPIO_BRR_BR2; // Output Low

    GPIOA->BSRR = GPIO_BSRR_BS2; // Output High
    battery_voltage3 = analogRead(PC2_ALT2);
    GPIOA->BRR = GPIO_BRR_BR2; // Output Low

}
