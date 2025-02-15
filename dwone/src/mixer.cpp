#include "mixer.h"
#include <algorithm>

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
    float thr_pwm = scaleThrottle(throttle);
    float roll_pwm = scaleCommand(roll);
    float pitch_pwm = scaleCommand(pitch);
    float yaw_pwm = scaleCommand(yaw);

    // Mixing matrix (matches your reference implementation)
    pwm_out[0] = thr_pwm - roll_pwm + pitch_pwm + yaw_pwm;  // Front right CW
    pwm_out[1] = thr_pwm - roll_pwm - pitch_pwm - yaw_pwm;  // Back right CCW
    pwm_out[2] = thr_pwm + roll_pwm - pitch_pwm + yaw_pwm;  // Back left CW
    pwm_out[3] = thr_pwm + roll_pwm + pitch_pwm - yaw_pwm;  // Front left CCW

    // Constrain outputs
    for(uint8_t i = 0; i < NUM_MOTORS; i++) {
        pwm_out[i] = std::max(std::min(pwm_out[i], (uint16_t)MAX_PWM_OUT), (uint16_t)MIN_PWM_OUT);
    }
}