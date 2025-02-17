#include "controller.h"
#include <cmath>
#include "rc_pilot.h"
#include "sensors.h"

#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

extern RC_PILOT rc;
extern Sensors sens;


#ifndef C_PI
#define C_PI 3.14159265358979323846264338327950288419716939937511
#endif



Controller::Controller() {
    // Initialize arrays to zero
    for(int i = 0; i < 3; i++) {
        KP[i] = KI[i] = KD[i] = 0;
        KPol[i] = KIol[i] = KDol[i] = 0;
        integralPos[i] = integralAngle[i] = 0;
        posError[i] = posError_Hdg[i] = velError_Hdg[i] = 0;
    }
    phiCmd = thetaCmd = psiCmd = 0;
}

Controller::~Controller() {}

double Controller::hmodRad(double h) {
    double dh;
    int i = (h > 0) ? (int)(h / (2 * C_PI) + 0.5) : (int)(h / (2 * C_PI) - 0.5);
    dh = h - C_PI * 2 * i;
    return dh;
}





void Controller::updateControl(const float dt, const float phiCmd,
                             const float posDes_x, const float posDes_y, const float posDes_z,
                             const float nav_p_x, const float nav_p_y, const float nav_p_z,
                             const float nav_v_x, const float nav_v_y, const float nav_v_z,
                             const float nav_w_x, const float nav_w_y, const float nav_w_z,
                             const float nav_phi, const float nav_theta, const float nav_psi,
                             float& c_delf, float& c_delm0, float& c_delm1, float& c_delm2) {
    
    // Calculate position errors
    posError[0] = posDes_x - nav_p_x;  // Error North
    posError[1] = posDes_y - nav_p_y;  // Error East

    // Coordinate transformation
    posError_Hdg[0] = posError[0] * cos(nav_psi) + posError[1] * sin(nav_psi);
    posError_Hdg[1] = -posError[0] * sin(nav_psi) + posError[1] * cos(nav_psi);

    // Update integral terms
    for(int i = 0; i < 3; i++) {
        integralPos[i] += posError_Hdg[i] * dt;
    }
    
    // Calculate velocity errors in heading frame
    velError_Hdg[0] = nav_v_x * cos(nav_psi) + nav_v_y * sin(nav_psi);
    velError_Hdg[1] = -nav_v_x * sin(nav_psi) + nav_v_y * cos(nav_psi);

    // Position controller
    this->phiCmd = KPol[1] * posError_Hdg[1] - KDol[1] * velError_Hdg[1] + KIol[1] * integralPos[1];
    this->thetaCmd = -(KPol[0] * posError_Hdg[0] - KDol[0] * velError_Hdg[0] + KIol[0] * integralPos[0]);

    // Altitude controller (negative because positive is down)
    c_delf = - KPol[2] * (posDes_z - nav_p_z)
             - KDol[2] * (0 - nav_v_z)
             - KIol[2] * integralPos[2];

    // Update attitude integrals
    for(int i = 0; i < 3; i++) {
        integralAngle[i] += dt * (i == 0 ? (this->phiCmd - nav_phi) :
                                 i == 1 ? (this->thetaCmd - nav_theta) :
                                         (this->psiCmd - nav_psi));
    }

    // Attitude controller
    c_delm0 = KP[0] * (this->phiCmd - nav_phi) + KD[0] * (0 - nav_w_x) + KI[0] * integralAngle[0];
    c_delm1 = KP[1] * (this->thetaCmd - nav_theta) + KD[1] * (0 - nav_w_y) + KI[1] * integralAngle[1];
    c_delm2 = KP[2] * hmodRad(this->psiCmd - nav_psi) + KD[2] * (0 - nav_w_z) + KI[2] * integralAngle[2];
}

void Controller::setPIDGains(float kp[3], float ki[3], float kd[3]) {
    for(int i = 0; i < 3; i++) {
        KP[i] = kp[i];
        KI[i] = ki[i];
        KD[i] = kd[i];
    }
}

void Controller::setOuterLoopGains(float kp[3], float ki[3], float kd[3]) {
    for(int i = 0; i < 3; i++) {
        KPol[i] = kp[i];
        KIol[i] = ki[i];
        KDol[i] = kd[i];
    }
}