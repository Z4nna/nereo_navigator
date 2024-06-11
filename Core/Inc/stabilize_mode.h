/*
 * stabilize_mode.h
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_STABILIZE_MODE_H_
#define CORE_INC_STABILIZE_MODE_H_

#include "pid.h"
#include "navigation.h"
#include "arm_math.h"
#include <stdint.h>

#define PID_NUMBER 4

// PIDs instances
extern PID pids[PID_NUMBER];

/*
 * Inits PIDS controller, setting their constants
 */
void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER]);

/*
 *
 */
uint8_t calculate_pwm_with_pid(float input_values[6], uint16_t pwm_output[8], float setpoints[PID_NUMBER], float current_values[PID_NUMBER], float integration_intervals[PID_NUMBER], Quaternion *orientation_quaternion);

#endif /* CORE_INC_STABILIZE_MODE_H_ */
