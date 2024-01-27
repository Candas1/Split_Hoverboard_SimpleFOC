#include <Arduino.h>
#include <config.h>
#include <defines.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <voltage/GenericVoltageSense.h>
#include <helper.h>


#ifdef SENSORLESS
#include <encoders/flux_observer/FluxObserverSensor.h>
#else
#include <encoders/smoothing/SmoothingSensor.h>
#endif


#ifdef DEBUG_STLINK
  #include <RTTStream.h>
  RTTStream rtt;
#endif

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS, 0.1664, 17.0, 0.00036858);
BLDCDriver6PWM driver = BLDCDriver6PWM( BLDC_GH_PIN,BLDC_GL_PIN, BLDC_BH_PIN,BLDC_BL_PIN, BLDC_YH_PIN,BLDC_YL_PIN );

#ifdef SENSORLESS 
  FluxObserverSensor fluxsensor = FluxObserverSensor(motor);
#else
  HallSensor sensor = HallSensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN, BLDC_POLE_PAIRS);
  // Interrupt routine initialization
  void doA()  { sensor.handleA(); } // channel callbacks
  void doB()  { sensor.handleB(); }
  void doC()  { sensor.handleC(); }

  // instantiate the smoothing sensor, providing the real sensor as a constructor argument
  SmoothingSensor smooth = SmoothingSensor(sensor, motor);
#endif

// shunt resistor value , gain value,  pins phase A,B,C
LowsideCurrentSense current_sense = LowsideCurrentSense(BLDC_CUR_Rds, BLDC_CUR_Gain, BLDC_CUR_G_PIN, BLDC_CUR_B_PIN, BLDC_CUR_Y_PIN);

#ifdef VBAT // This won't work yet on STM32
GenericVoltageSense battery = GenericVoltageSense(VBAT,BATTERY_GAIN,0,0.5);
float battery_voltage,vref;
#endif

CIO oKeepOn = CIO(SELF_HOLD_PIN,OUTPUT); // For holding the latch after start-up
CIO oOnOff  = CIO(BUTTON_PIN); // ON/OFF button, not used yet


// Only on splitboards
CIO oLedGreen   = CIO(LED_GREEN,OUTPUT);
CIO oLedOrange  = CIO(LED_ORANGE,OUTPUT);
CIO oLedRed     = CIO(LED_RED,OUTPUT);
CIO oLedUp      = CIO(UPPER_LED_PIN,OUTPUT);
CIO oLedLow     = CIO(LOWER_LED_PIN,OUTPUT);

CIO aoLed[5] = {oLedGreen, oLedOrange, oLedRed, oLedUp, oLedLow };
#define LED_Count 5   // reduce to test led pins..

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

#ifdef DEBUG_STLINK
  Commander command = Commander(SERIALDEBUG);
#endif

float target = 0;

#ifdef DEBUG_STLINK
void doTarget(char* cmd) { command.scalar(&target, cmd); }
#endif

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

  // For time measurement with the oscilloscope
  pinMode(PA2,OUTPUT);
  pinMode(PA3,OUTPUT);
  
  #ifdef SENSORLESS
    motor.linkSensor(&fluxsensor); // link the motor to the smoothing sensor
  #else
    sensor.init();  // initialize sensor hardware
    sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable

    // set SmoothingSensor phase correction for hall sensors
    smooth.phase_correction = -_PI_6;
    motor.linkSensor(&smooth); // link the motor to the smoothing sensor
  #endif
  
  driver.voltage_power_supply = 26;
  driver.pwm_frequency = 16000;
  driver.dead_zone = 0.01;

  if (driver.init()){
    driver.enable();
  }
  else{
    return; // cancel simpleFOC setup
  }

  // link the driver to the current sense
  current_sense.linkDriver(&driver);

  motor.linkDriver(&driver);  // link driver
  motor.voltage_sensor_align  = 2;                            // aligning voltage
  motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
  motor.controller            = MotionControlType::torque;    // set motion control loop to be used
  motor.torque_controller     = TorqueControlType::foc_current;
  motor.current_limit         = 1;
  
  // Limit the voltage to have enough low side ON time for phase current sampling
  driver.voltage_limit = driver.voltage_power_supply * 0.95;

  // Limit the voltage depending on the motion control type and torque control type
  if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
    if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
      // When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
      motor.voltage_limit = driver.voltage_power_supply * 0.58f;
    }else{
      motor.voltage_limit = driver.voltage_power_supply * 0.58f;
    }
  }else{
    // For openloop angle and velocity modes, use very small limit
    motor.voltage_limit = driver.voltage_power_supply * 0.07f;
  }

  #ifdef VBAT // This won't work yet on STM32
  battery.init(12);
  battery.update();
  battery_voltage = battery.getVoltage();
  #endif

  // motor init
  motor.init();
  current_sense.skip_align = true;
  if (current_sense.init()){  
    motor.linkCurrentSense(&current_sense);
  }
  else{
    OUTN("current_sense.init() failed.")
    return;   // cancel simpleFOC setup
  }
  

  // initialize motor
  #ifdef SENSORLESS
    motor.sensor_direction= Direction::CW;
    motor.zero_electric_angle = 0;     // use the real value!
  #else  
    motor.sensor_direction= MOTOR_sensor_direction;
    motor.zero_electric_angle = MOTOR_zero_electric_offset;     // use the real value!
  #endif

  motor.initFOC();

  #ifdef DEBUG_STLINK
  // add target command T
  command.add('t', doTarget, "target voltage");
  #endif

  Blink(3,oLedGreen);
}

LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect
PhaseCurrent_s currents;

void loop()
{
  // Limit the voltage depending on the motion control type and torque control type
  if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
    if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
      // When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
      motor.voltage_limit = driver.voltage_power_supply * 0.58f;
    }else{
      motor.voltage_limit = driver.voltage_power_supply * 0.58f;
    }
  }else{
    // For openloop angle and velocity modes, use very small limit
    motor.voltage_limit = driver.voltage_power_supply * 0.07f;
  } 

  if (motor.enabled){  // set by successful motor.init() at the end of setup()
    //GPIO_BOP(GPIOA) = (uint32_t)GPIO_PIN_3; // For GD32
    //GPIOA->BSRR = GPIO_BSRR_BS2; // For STM32
    motor.loopFOC();
    //GPIOA->BRR = GPIO_BRR_BR2; // For STM32
    //GPIO_BC(GPIOA) = (uint32_t)GPIO_PIN_3; // For GD32    
    motor.move(LPF_target(target));
  }

  if (current_sense.initialized){
		currents = current_sense.getPhaseCurrents();
  }

  #ifdef DEBUG_STLINK
    // user communication
    command.run();
  #endif

  #ifdef VBAT // This won't work yet on STM32
  battery.update();
  battery_voltage = battery.getVoltage();
  //battery_current = (motor.current.d * motor.voltage.d + motor.current.q * motor.voltage.q) / battery_voltage;
  #endif 
}