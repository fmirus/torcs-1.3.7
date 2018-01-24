#ifndef PID_ACC_H
#define PID_ACC_H

class PidAcc{
    public:
        PidAcc();
        PidAcc(float dt, float Kp, float Kd, float Ki);

        // Calculate new acceleration value
        float step(float setPoint, float currentVal, float maxAcc);

    private:
        float _dt, _Kp, _Kd, _Ki, _preError, _integral;
};


#endif // PID_ACC_H
