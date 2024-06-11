/*
 * pid.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_PID_H_
#define CORE_INC_PID_H_

/**
 * @file pid.h
 * @brief Provides a PID controller implementation.
 *
 * This header file defines a PID (Proportional-Integral-Derivative) controller
 * structure and functions to initialize and update the PID controller.
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param integral Integral term
 * @param prev_error Previous error
 *
 * @function PID_Init Initializes the PID controller with the given parameters.
 * @function PID_Update Updates the PID controller with the current value and
 * returns the control output.
 */

#include <stdio.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd);

float PID_Update(PID *pid, float current_value, float setpoint, float dt);

void PID_set_constants(PID *pid, float kp, float ki, float kd);

#endif /* CORE_INC_PID_H_ */
