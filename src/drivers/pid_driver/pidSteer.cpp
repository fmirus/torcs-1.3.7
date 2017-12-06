#include "pidSteer.h"

#include <cmath>
#include <iostream>

PidSteer::PidSteer() {}

PidSteer::PidSteer(float dt, float Kp, float Kd, float Ki, float G1, float G2, float max)
    : _dt(dt), _Kp(Kp), _Kd(Kd), _Ki(Ki), _G1(G1), _G2(G2), _preError(0), _integral(0), _max(max) {}

float PidSteer::step(float setpoint1, float setpoint2, float currentVal1, float currentVal2) {

    // Calculate error
    float error1 = 1 * (setpoint1 - currentVal1);
    float error2 = 1 * (setpoint2 - currentVal2);
    // std::cout << "1: " << error1 << ", 2: " << error2 << std::endl;
    float error = _G1 * error1 + _G2 * error2;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _preError) / _dt;
    float Dout = _Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;
    std::cout << "P: " << Pout << ", I: " << Iout << " D: " << Dout << std::endl;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < -1 * _max)
        output = -1 * _max;

    // Save error to previous error
    _preError = error;

    return output;
}
