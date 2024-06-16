/*
 * stabilize_mode.h
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 *
 */

#ifndef CORE_INC_STABILIZE_MODE_H_
#define CORE_INC_STABILIZE_MODE_H_

#include "pid.h"
#include "navigation.h"
#include "arm_math.h"
#include <stdint.h>

#define PID_NUMBER 4

extern PID pids[PID_NUMBER];

/**
 * @brief Initializes the PID controllers with the given proportional, integral, and derivative gain values.
 *
 * @param kps Array of proportional gain values for each PID controller.
 * @param kis Array of integral gain values for each PID controller.
 * @param kds Array of derivative gain values for each PID controller.
 */
void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER]);

/**
 * @brief Calculates the PWM output values using the PID controllers.
 *
 * @param input_values Array of input values for the PID controllers.
 * @param pwm_output Array to store the calculated PWM output values.
 * @param setpoints Array of setpoint values for the PID controllers.
 * @param current_values Array of current values for the PID controllers.
 * @param integration_intervals Array of integration interval values for the PID controllers.
 * @param orientation_quaternion Pointer to the current orientation quaternion.
 * @return eventual error code
 */
uint8_t calculate_pwm_with_pid(float input_values[6], uint16_t pwm_output[8], float setpoints[PID_NUMBER], float current_values[PID_NUMBER], float integration_intervals[PID_NUMBER], Quaternion *orientation_quaternion);

#endif /* CORE_INC_STABILIZE_MODE_H_ */
