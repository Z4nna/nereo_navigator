/*
 * navigation.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#include "navigation.h"

// error codes
enum {
	OK,
	MAT_MULT_ERROR,
};

/**
 * @brief Calculates the PWM output values based on the joystick input.
 *
 * This function takes the joystick input values, normalizes them, and then
 * multiplies them with a fixed mixing matrix to generate the PWM output values.
 * The PWM output values are then normalized and mapped to the range of PWM_MIN
 * to PWM_MAX.
 *
 * @param joystick_input An array of 6 float values representing the joystick input.
 * @param pwm_output An array of 8 uint16_t values to store the calculated PWM output.
 * @return OK if the calculation was successful, MAT_MULT_ERROR if the matrix
 * multiplication failed.
 */
uint8_t calculate_pwm(const float in_joystick_input[6], uint16_t pwm_output[8])
{
	float joystick_input[6];
	for(uint8_t i = 0; i < 6; i++)
	{
		joystick_input[i] = in_joystick_input[i];
	}

    normalize_vector(in_joystick_input, joystick_input, 6);

    float f_pwm_output[8];

    __attribute__((aligned(4))) float pwm_output_8_1[8][1] = {0};

    arm_matrix_instance_f32 fixed_mixing_matrix_instance;
    arm_matrix_instance_f32 joystick_input_instance;
    arm_matrix_instance_f32 pwm_output_instance;

    arm_mat_init_f32(&fixed_mixing_matrix_instance, 8, 6, (float *)FIXED_MIXING_MATRIX);
    arm_mat_init_f32(&joystick_input_instance, 6, 1, (float *)joystick_input);
    arm_mat_init_f32(&pwm_output_instance, 8, 1, (float *)pwm_output_8_1);

    if (arm_mat_mult_f32(&fixed_mixing_matrix_instance, &joystick_input_instance, &pwm_output_instance) != ARM_MATH_SUCCESS)
    {
        return MAT_MULT_ERROR;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        f_pwm_output[i] = pwm_output_instance.pData[i];
    }

    // normalize pwm_output and map to 1100 ~ 1900
    normalize_vector(f_pwm_output, f_pwm_output, 8);
    for (uint8_t i = 0; i < 8; i++)
    {
        // pwm_output[i][0] = symmetric_quadratic_interpolation(pwm_output[i][0], 1, PWM_MAX);
        pwm_output[i] = (int)linear_interpolation(f_pwm_output[i], -1, 1, PWM_MIN, PWM_MAX);
    }
    return OK;
}

void invert_quaternion(const Quaternion * q, Quaternion * q_inv)
{
	arm_quaternion_inverse_f32((float*)q, (float*)q_inv, 1);
}

void multiply_quaternions(const Quaternion* q1, const Quaternion* q2, Quaternion* qResult)
{
    // Perform the quaternion multiplication
    arm_quaternion_product_single_f32((float*)q1, (float*)q2, (float*)qResult);
}
