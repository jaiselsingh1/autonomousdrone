#ifndef AERSP_CONTROLLER_H
#define AERSP_CONTROLLER_H

#include <cmath>

class Controller {
public:
    Controller();
    ~Controller();

    void updateControl(const float dt, const float phiCmd,
                      const float posDes_x, const float posDes_y, const float posDes_z,
                      const float nav_p_x, const float nav_p_y, const float nav_p_z,
                      const float nav_v_x, const float nav_v_y, const float nav_v_z,
                      const float nav_w_x, const float nav_w_y, const float nav_w_z,
                      const float nav_phi, const float nav_theta, const float nav_psi,
                      float& c_delf, float& c_delm0, float& c_delm1, float& c_delm2);

    // Getters/setters for PID gains
    void setPIDGains(float kp[3], float ki[3], float kd[3]);
    void setOuterLoopGains(float kp[3], float ki[3], float kd[3]);

private:
    // PID gains for attitude control
    float KP[3];  // Roll, Pitch, Yaw
    float KI[3];
    float KD[3];

    // Outer loop gains for position control
    float KPol[3];  // X, Y, Z
    float KIol[3];
    float KDol[3];

    // State variables
    float integralPos[3];
    float integralAngle[3];
    float posError[3];
    float posError_Hdg[3];
    float velError_Hdg[3];

    float phiCmd, thetaCmd, psiCmd;

    // Helper functions
    double hmodRad(double h);
};

#endif