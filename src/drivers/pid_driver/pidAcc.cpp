#include "pidAcc.h"

#include <iostream>
#include <cmath>


PidAcc::PidAcc(){}

PidAcc::PidAcc( float dt, float Kp, float Kd, float Ki ) :
    _dt(dt),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _preError(0),
    _integral(0)
{
}


float PidAcc::step( float setpoint, float currentVal, float maxAcc)
{

    // Calculate error
    float error = setpoint - currentVal;

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
    // std::cout << "P: " << Pout << ", I: " << Iout << " D: " << Dout << std::endl;

    // Restrict to max/min
    if( output > maxAcc )
        output = maxAcc;
    else if( output < -1 * maxAcc )
        output = -1 * maxAcc;

    // Save error to previous error
    _preError = error;

    return output;
}
