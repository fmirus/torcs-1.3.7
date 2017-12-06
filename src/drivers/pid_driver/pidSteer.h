#ifndef PID_STEER_H
#define PID_STEER_H

class PidSteer {
  public:
    PidSteer();
    // PidSteer(float dt, float Kp, float Kd, float Ki, float max);
    PidSteer(float dt, float Kp, float Kd, float Ki, float G1, float G2, float max);

    // float step(float setPoint, float currentVal);
    float step(float setpoint1, float setpoint2, float currentVal1, float currentVal2);

  private:
    float _dt, _Kp, _Kd, _Ki, _G1, _G2, _preError, _integral, _max;
};

#endif // PID_STEER_H
