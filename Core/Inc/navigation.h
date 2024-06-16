/**
 * @file navigation.h
 * @brief Provides functions and data types for navigation-related calculations.
 *
 * This header file defines enumerations and data structures for handling quaternions and
 * calculating PWM values from joystick inputs. It also declares functions for inverting
 * quaternions and multiplying quaternions.
 *
 * @author michelecarenini
 * @date May 23, 2024
 */
#ifndef CORE_INC_NAVIGATION_H_
#define CORE_INC_NAVIGATION_H_

#include "interpolations.h"
#include "mixing_matrix.h"
#include "arm_math.h"

/**
 * @brief Enumeration of PWM value constants.
 */
typedef enum
{
	PWM_IDLE = 1500,
	PWM_MAX = 1900,
	PWM_MIN = 1100,
} PwmValues;

/**
 * @brief Data structure representing a quaternion.
 */
typedef struct
{
	float w; ///< Scalar component of the quaternion
	float a; ///< Vector component x of the quaternion
	float b; ///< Vector component y of the quaternion
	float c; ///< Vector component z of the quaternion
} Quaternion;

/**
 * @brief Calculates PWM output values from joystick input.
 *
 * @param joystick_input
 * @param pwm_output
 * @return eventual error code
 */
uint8_t calculate_pwm(float joystick_input[6], uint16_t pwm_output[8]);

/**
 * @brief Inverts a quaternion.
 *
 * @param q Pointer to the input quaternion.
 * @param q_inv Pointer to the output inverted quaternion.
 */
void invert_quaternion(Quaternion *q, Quaternion *q_inv);

/**
 * @brief Multiplies two quaternions.
 *
 * @param q1 Pointer to the first quaternion.
 * @param q2 Pointer to the second quaternion.
 * @param qResult Pointer to the output quaternion result.
 */
void multiply_quaternions(Quaternion *q1, Quaternion *q2, Quaternion *qResult);

#endif /* CORE_INC_NAVIGATION_H_ */
