#ifndef PID_STEER_H
#define PID_STEER_H

class PidSteer {
  public:
    PidSteer();
    // G1 and G2 are they weights for the 2 set points
    PidSteer(float dt, float Kp, float Kd, float Ki, float G1, float G2, float max);

    // Calculate new steering value
    float step(float setpoint1, float setpoint2, float currentVal1, float currentVal2);

  private:
    float _dt, _Kp, _Kd, _Ki, _G1, _G2, _preError, _integral, _max;
};

#endif // PID_STEER_H
