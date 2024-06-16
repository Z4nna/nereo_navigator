/*
 * stabilize_mode.c
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 */


#include "stabilize_mode.h"

// tolerance: if a joystick input (in [-1,1]) is < TOLERANCE, it is considered as 0
const double TOLERANCE = 0.05;

// PIDs controllers, respectively for z, pitch, roll, yaw
PID pids[4] = {0};

void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER])
{
    for(uint8_t i = 0; i < PID_NUMBER; i++)
    {
        PID_Init(&pids[i], kps[i], kis[i], kds[i]);
    }
}

// The order for 4-elements arrays is: z, pitch, roll, yaw
uint8_t calculate_pwm_with_pid(float input_values[6], uint16_t pwm_output[8], float setpoints[4], float current_values[4], float integration_intervals[4], Quaternion *orientation_quaternion)
{
	float pitch_pid_feedback = PID_Update(&pids[1], current_values[1], setpoints[1], integration_intervals[1]);
	float roll_pid_feedback = PID_Update(&pids[2], current_values[2], setpoints[2], integration_intervals[2]);
	float yaw_pid_feedback = PID_Update(&pids[3], current_values[3], setpoints[3], integration_intervals[3]);

	/* **************
	 * Depth Hold
	*/
	// The z axis we can get measures of is in the fixed-body-frame:
	// we need to convert the output of the PID to the body frame in order to modify the input, in order to achieve the desired depth hold.
	float z_out = PID_Update(&pids[0], current_values[0], setpoints[0], integration_intervals[0]);

	// Applies the inverse rotation of the body-frame from the fixed-body-frame ( described by the orientation quaternion ),
	// in order to compute the coordinates of the z_out vector with respect to the body-frame
	Quaternion z_out_q;
	z_out_q.w = 0;
	z_out_q.a = 0;
	z_out_q.b = 0;
	z_out_q.c = z_out;
	Quaternion q_inv = {0};
	invert_quaternion(orientation_quaternion, &q_inv);
	
	// applies the inverse rotation to the z_out_q vector
	Quaternion intermediate_result = {0};
	Quaternion z_out_body_frame = {0};
	multiply_quaternions(&q_inv, &z_out_q, &intermediate_result);
	multiply_quaternions(&intermediate_result, orientation_quaternion, &z_out_body_frame);

	// apply the feedback on x y z axis if and only if either the feedback is approx 0, or the input value by the user is approx 0.
	// This condition must be met for every axis value
	uint8_t y_condition = fabsf(z_out_body_frame.b) < TOLERANCE || fabsf(input_values[0] < TOLERANCE);
	uint8_t x_condition = fabsf(z_out_body_frame.a) < TOLERANCE || fabsf(input_values[1] < TOLERANCE);
	uint8_t z_condition = fabsf(z_out_body_frame.c) < TOLERANCE || fabsf(input_values[2] < TOLERANCE);

	if (x_condition && y_condition && z_condition)
	{
		input_values[0] += z_out_body_frame.b;
		input_values[1] += z_out_body_frame.a;
		input_values[2] += z_out_body_frame.c;
	}

	// pitch
	if (fabsf(pitch_pid_feedback) < TOLERANCE || fabsf(input_values[3] < TOLERANCE))
	{
		input_values[3] += pitch_pid_feedback;
	}
	// roll
	if (fabsf(roll_pid_feedback) < TOLERANCE || fabsf(input_values[4] < TOLERANCE))
	{
		input_values[4] += roll_pid_feedback;
	}
	// yaw
	if (fabsf(yaw_pid_feedback) < TOLERANCE || fabsf(input_values[5] < TOLERANCE))
	{
		input_values[5] += yaw_pid_feedback;
		return 1;
	}
	return 0;
}

