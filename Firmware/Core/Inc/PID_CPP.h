#ifndef PID_CPP_H
#define PID_CPP_H

class PID
{
private:
    float prev_measurement,
        error, prev_error,
        kp, ki, kd,
        p, i, d,
        int_max, int_min,
        max, min,
        command,
        tau, Ts;

public:
    void Initialise(float object_kp, float object_ki, float object_kd, float object_max, float object_min);
    float Control(float setpoint, float measurement, float derivative);
};

#endif
