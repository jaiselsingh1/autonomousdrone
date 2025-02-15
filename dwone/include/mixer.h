#ifndef AERSP_MIXER_H
#define AERSP_MIXER_H

#include "motors.h"

class Mixer {
public:
    Mixer();
    ~Mixer();

    // Takes control inputs and converts to motor PWM values
    void mix(float throttle, float roll, float pitch, float yaw, 
             uint16_t pwm_out[NUM_MOTORS]);

private:
    static constexpr float SCALE_THROTTLE = (MAX_PWM_OUT - MIN_PWM_OUT) / 2.0f;
    static constexpr float OFFSET_THROTTLE = (MAX_PWM_OUT + MIN_PWM_OUT) / 2.0f;

    // Helper functions
    float scaleThrottle(float input);
    float scaleCommand(float input);
};

#endif