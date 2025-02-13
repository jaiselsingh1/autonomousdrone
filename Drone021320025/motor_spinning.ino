/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai
 * Email : thanakorn.khamvilai@psu.edu
 *
 * EndCopyright
 ***/

/*
 Motors/ESCs Spinning Direction:
  - Front Right -> Pin 3,  CCW
  - Back Right  -> Pin 5,  CW
  - Back Left   -> Pin 9, CCW
  - Front Left  -> Pin 10, CW
*/

#include "wifi.h"
#include "rc_pilot.h"
#include "datalink.h"
#include "motors.h"

Motors motors;
RC_PILOT rc;

unsigned long previousMillis = 0;
const long interval = 500;
uint16_t pwm[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};
uint16_t MotorDataGCS[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};

void setup() {
  // initialize wifi
  WifiSetup();

  // initialize rc
  rc.init();

  // initialize motors/servos
  motors.init();
  // motors.calibrate();

  // initialize peripherals 
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  rc.update();
  
  if(rc.rc_in.AUX2 > 1500)
  {
    pwm[0] = rc.rc_in.THR;
    pwm[1] = rc.rc_in.THR;
    pwm[2] = rc.rc_in.THR;
    pwm[3] = rc.rc_in.THR;
  }
  else{
    pwm[0] = MotorDataGCS[0];
    pwm[1] = MotorDataGCS[1];
    pwm[2] = MotorDataGCS[2];
    pwm[3] = MotorDataGCS[3];
  } 
  motors.update(pwm);
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    rc.print(); // should be commented out for flight
  }

  // read/write datalink msg
  readDatalink( &Udp );
  //writeAutopilotDels( &Udp );
  writeRcChannels( &Udp , &rc );
}
