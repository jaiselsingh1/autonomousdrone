/*
 Motors/ESCs Spinning Direction:
 - Front Right -> Pin 3, CCW motor 1
 - Back Right -> Pin 5, CW motor 2
 - Back Left -> Pin 9, CCW motor 3
 - Front Left -> Pin 10, CW motor 4
*/
#include "wifi.h"
#include "rc_pilot.h"
#include "datalink.h"
#include "motors.h"
#include "controller.h"
#include "mixer.h"
#include <Arduino.h>
float manualMap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



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
    if (rc.rc_in.AUX < 1500){
        if(rc.rc_in.AUX2 > 1200) {
            // Manual RC control mode
            float throttle = manualMap(rc.rc_in.THR, 1000, 2000, -1.0, 1.0);
            float roll = manualMap(rc.rc_in.ROLL, 1000, 2000, -1.0, 1.0);
            float pitch = manualMap(rc.rc_in.PITCH, 1000, 2000, -1.0, 1.0);
            float yaw = manualMap(rc.rc_in.YAW, 1000, 2000, -1.0, 1.0);

            // Serial.print(rc.rc_in.THR) ;  Serial.print(", "); 
            // Serial.print(throttle) ;  Serial.print(", "); 
            // Serial.print(pitch) ;  Serial.print(", "); 
            // Serial.print(", \n"); 
            
            // Mix directly from RC inputs
            mixer.mix(throttle, roll, pitch, yaw, pwm);
            //Serial.println("MANUAL");
            if (rc.rc_in.AUX2 > 1700){
                //Serial.println("Drop Payload") ; 
            }

        } else {
            // Autonomous/GCS mode
            float c_delf = 0, c_delm0 = 0, c_delm1 = 0, c_delm2 = 0;
            
            // Update controller with current state
            //controller.updateControl(dt, 0,  // phiCmd
            //                       0, 0, 0,  // posDes_x, y, z
            //                       0, 0, 0,  // nav_p_x, y, z
            //                       0, 0, 0,  // nav_v_x, y, z
            //                       0, 0, 0,  // nav_w_x, y, z
            //                       0, 0, 0,  // nav_phi, theta, psi
            //                      c_delf, c_delm0, c_delm1, c_delm2);
            
            // Mix controller outputs
            mixer.mix(c_delf, c_delm0, c_delm1, c_delm2, pwm);
            //Serial.println("AUTO");
        }  
    }else{ 
        // Autonomous/GCS mode
        pwm[0] = MIN_PWM_OUT ;
        pwm[1] = MIN_PWM_OUT ;
        pwm[2] = MIN_PWM_OUT ;
        pwm[3] = MIN_PWM_OUT ; 
        
        Serial.println("KILLED");
        Serial.print(pwm[0]) ; Serial.print(",") ; 
        Serial.print(pwm[1]) ; Serial.print(",") ; 
        Serial.print(pwm[2]) ; Serial.print(",") ; 
        Serial.print(pwm[3]) ; 
    }
    
    motors.update(pwm);
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        // rc.print(); // should be commented out for flight
    }
    
    // read/write datalink msg
    readDatalink(&Udp);
    writeRcChannels(&Udp, &rc);
}