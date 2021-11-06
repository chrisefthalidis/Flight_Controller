#ifndef PID_C_H
#define PID_C_H

typedef struct
{

    float prev_measurement,
        error, prev_error,
        kp, ki, kd,
        p, i, d,
        int_max, int_min,
        max, min,
        command,
        tau, Ts;

} PID;

void PID_Initialise(PID *controller, float object_kp, float object_ki, float object_kd, float object_max, float object_min);
float PID_Control(PID *controller, float setpoint, float measurement, float derivative);

#endif
