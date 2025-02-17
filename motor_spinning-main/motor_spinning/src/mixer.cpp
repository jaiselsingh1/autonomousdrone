#include "mixer.h"
#include <algorithm>
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))


Mixer::Mixer() {}
Mixer::~Mixer() {}

float Mixer::scaleThrottle(float input) {
    // Scale from [-1,1] to [MIN_PWM_OUT,MAX_PWM_OUT]
    return input * SCALE_THROTTLE + OFFSET_THROTTLE;
}

float Mixer::scaleCommand(float input) {
    // Scale from [-1,1] to [-500,500]
    return input * 500.0f;
}

void Mixer::mix(float throttle, float roll, float pitch, float yaw, 
                uint16_t pwm_out[NUM_MOTORS]) {

                    
    // Manual bias elimination
    float thr_pwm = scaleThrottle(throttle);
    float roll_pwm = scaleCommand(roll);
    float pitch_pwm = scaleCommand(pitch);
    float yaw_pwm = scaleCommand(yaw);

    // Mixing matrix (matches your reference implementation)
    pwm_out[0] = LIMIT(thr_pwm - roll_pwm - pitch_pwm + yaw_pwm,MIN_PWM_OUT,MAX_PWM_OUT);  // Front right CW
    pwm_out[1] = LIMIT(thr_pwm - roll_pwm + pitch_pwm - yaw_pwm,MIN_PWM_OUT,MAX_PWM_OUT);   // Back right CCW
    pwm_out[2] = LIMIT(thr_pwm + roll_pwm + pitch_pwm + yaw_pwm,MIN_PWM_OUT,MAX_PWM_OUT);   // Back left CW
    pwm_out[3] = LIMIT(thr_pwm + roll_pwm - pitch_pwm - yaw_pwm,MIN_PWM_OUT,MAX_PWM_OUT);   // Front left CCW

    if ((pwm_out[0] < 910) && (pwm_out[2] < 910) && (pwm_out[3] < 910) && (pwm_out[1] < 950)) {
        pwm_out[1] = 900;
    }


    // Serial.print(pitch_pwm) ;  Serial.print(", "); 
    // Serial.print(thr_pwm) ;  Serial.print(", "); 
    // Serial.print(roll_pwm) ;  Serial.print(", "); 
    // Serial.print(yaw_pwm) ;  Serial.print(", \n"); 
    
    // Serial.print(pwm_out[0]) ;  Serial.print(", "); 
    // Serial.print(pwm_out[1]) ;  Serial.print(", "); 
    // Serial.print(pwm_out[2]) ;  Serial.print(", "); 
    // Serial.print(pwm_out[3]) ;  Serial.print(", \n"); 


    // Serial.print(throttle) ;  Serial.print(", "); 
    // Serial.print(roll) ;  Serial.print(", "); 
    // Serial.print(pitch) ;  Serial.print(", "); 
    // Serial.print(", \n"); 



    // Constrain outputs
    // for(uint8_t i = 0; i < NUM_MOTORS; i++) {
    //     pwm_out[i] = std::max(std::min(pwm_out[i], (uint16_t)MAX_PWM_OUT), (uint16_t)MIN_PWM_OUT);
    // }
}