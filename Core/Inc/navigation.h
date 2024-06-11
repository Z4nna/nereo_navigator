/*
 * navigation.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_NAVIGATION_H_
#define CORE_INC_NAVIGATION_H_


#include "interpolations.h"
#include "mixing_matrix.h"
#include "arm_math.h"

typedef enum {
	PWM_IDLE = 1500,
	PWM_MAX = 1900,
	PWM_MIN = 1100,
} PwmValues;

typedef struct {
	float w;
	float a;
	float b;
	float c;
} Quaternion;

uint8_t calculate_pwm(float joystick_input[6], uint16_t pwm_output[8]);

void invert_quaternion(Quaternion * q, Quaternion * q_inv);

void multiply_quaternions(Quaternion* q1, Quaternion* q2, Quaternion* qResult);

#endif /* CORE_INC_NAVIGATION_H_ */
