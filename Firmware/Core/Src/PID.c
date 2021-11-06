#include "PID_C.h"

void PID_Initialise(PID *controller, float object_kp, float object_ki, float object_kd, float object_max, float object_min)
{
    controller->prev_measurement = 0.f,
    controller->error = 0.f, controller->prev_error = 0.f,
    controller->kp = object_kp, controller->ki = object_ki, controller->kd = object_kd,
    controller->p = 0.f, controller->i = 0.f, controller->d = 0.f,
    controller->int_max = 0, controller->int_min = 0,
    controller->max = object_max, controller->min = object_min,
    controller->command = 0.f,
    controller->tau = 0.02f, controller->Ts = 0.01f;
}

float PID_Control(PID *controller, float setpoint, float measurement, float derivative)
{
    // Error calculation
    controller->error = setpoint - measurement;

    // PID implementation
    controller->p = controller->kp * controller->error;
    controller->i = controller->ki * 0.5f * (controller->error + controller->prev_error) * controller->Ts + controller->i;
    controller->d = -controller->kd * derivative;
    // d = -(2 * kd * (measurement - prev_measurement) + (2 * tau - Ts) * d) / (2 * tau + Ts);

    // Anti-windup limit calculation
    if (controller->max > controller->p)
        controller->int_max = controller->max - controller->p;
    else
        controller->int_max = 0.f;

    if (controller->min < controller->p)
        controller->int_min = controller->min + controller->p;
    else
        controller->int_min = 0.f;

    // Anti-windup implementation
    if (controller->i > controller->int_max)
        controller->i = controller->int_max;
    else if (controller->i < controller->int_min)
        controller->i = controller->int_min;

    // Control command
    controller->command = controller->p + controller->i + controller->d;

    if (controller->command > controller->max)
        controller->command = controller->max;
    else if (controller->command < controller->min)
        controller->command = controller->min;

    controller->prev_error = controller->error;
    controller->prev_measurement = measurement;

    return controller->command;
}