#include "PID_CPP.h"

void PID::Initialise(float object_kp, float object_ki, float object_kd, float object_max, float object_min)
{
    prev_measurement = 0.f,
    error = 0.f, prev_error = 0.f,
    kp = object_kp, ki = object_ki, kd = object_kd,
    p = 0.f, i = 0.f, d = 0.f,
    int_max = 0, int_min = 0,
    max = object_max, min = object_min,
    command = 0.f,
    tau = 0.02f, Ts = 0.01f;
}

float PID::Control(float setpoint, float measurement, float derivative)
{
    // Error calculation
    error = setpoint - measurement;

    // PID implementation
    p = kp * error;
    i = ki * 0.5f * (error + prev_error) * Ts + i;
    d = -kd * derivative;
    // d = -(2 * kd * (measurement - prev_measurement) + (2 * tau - Ts) * d) / (2 * tau + Ts);

    // Anti-windup limit calculation
    if (max > p)
        int_max = max - p;
    else
        int_max = 0.f;

    if (min < p)
        int_min = min + p;
    else
        int_min = 0.f;

    // Anti-windup implementation
    if (i > int_max)
        i = int_max;
    else if (i < int_min)
        i = int_min;

    // Control command
    command = p + i + d;

    if (command > max)
        command = max;
    else if (command < min)
        command = min;

    prev_error = error;
    prev_measurement = measurement;

    return command;
}
