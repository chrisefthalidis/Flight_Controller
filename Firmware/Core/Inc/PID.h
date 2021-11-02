#ifndef PID_H
#define PID_H

class PID
{
private:
    float prev_measurement;
    float error, prev_error;
    float kp, ki, kd;
    float p, i, d;
    float int_max, int_min;
    float max, min;
    float command;
    float tau;
    float Ts;

public:
    void Initialise(float object_kp, float object_ki, float object_kd, float object_max, float object_min);
    float Control(float setpoint, float measurement, float derivative);
};

#endif
