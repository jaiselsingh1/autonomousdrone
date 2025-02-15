/*
 Motors/ESCs Spinning Direction:
 - Front Right -> Pin 3, CCW
 - Back Right -> Pin 5, CW
 - Back Left -> Pin 9, CCW
 - Front Left -> Pin 10, CW
*/
#include "wifi.h"
#include "rc_pilot.h"
#include "datalink.h"
#include "motors.h"
#include "controller.h"
#include "mixer.h"
#include <Arduino.h>



Motors motors;
RC_PILOT rc;
Controller controller;
Mixer mixer;

unsigned long previousMillis = 0;
const long interval = 500;
uint16_t pwm[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};
uint16_t MotorDataGCS[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};

// PID Gains 
float kp[3] = {1.0, 1.0, 1.0};  // Roll, Pitch, Yaw
float ki[3] = {0.0, 0.0, 0.0};
float kd[3] = {0.1, 0.1, 0.1};

void setup() {
    // initialize rc
    rc.init();
    
    // initialize motors/servos
    motors.init();
    // motors.calibrate();
    
    // Initialize controller with gains
    controller.setPIDGains(kp, ki, kd);
    
    // initialize peripherals
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect
    }
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    unsigned long currentMillis = millis();
    float dt = (currentMillis - previousMillis) / 1000.0f; // Convert to seconds
    
    rc.update();
    
    if(rc.rc_in.AUX > 1500) {
        // Manual RC control mode
        float throttle = map(rc.rc_in.THR, MIN_PWM_OUT, MAX_PWM_OUT, -1.0, 1.0);
        float roll = map(rc.rc_in.ROLL, MIN_PWM_OUT, MAX_PWM_OUT, -1.0, 1.0);
        float pitch = map(rc.rc_in.PITCH, MIN_PWM_OUT, MAX_PWM_OUT, -1.0, 1.0);
        float yaw = map(rc.rc_in.YAW, MIN_PWM_OUT, MAX_PWM_OUT, -1.0, 1.0);
        
        // Mix directly from RC inputs
        mixer.mix(throttle, roll, pitch, yaw, pwm);
    } else {
        // Autonomous/GCS mode
        float c_delf = 0, c_delm0 = 0, c_delm1 = 0, c_delm2 = 0;
        
        // Update controller with current state
        controller.updateControl(dt, 0,  // phiCmd
                               0, 0, 0,  // posDes_x, y, z
                               0, 0, 0,  // nav_p_x, y, z
                               0, 0, 0,  // nav_v_x, y, z
                               0, 0, 0,  // nav_w_x, y, z
                               0, 0, 0,  // nav_phi, theta, psi
                               c_delf, c_delm0, c_delm1, c_delm2);
        
        // Mix controller outputs
        mixer.mix(c_delf, c_delm0, c_delm1, c_delm2, pwm);
    }
    
    motors.update(pwm);
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        rc.print(); // should be commented out for flight
    }
    
    // read/write datalink msg
    readDatalink(&Udp);
    writeRcChannels(&Udp, &rc);
}