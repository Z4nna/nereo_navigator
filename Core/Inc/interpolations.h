/*
 * interpolations.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_INTERPOLATIONS_H_
#define CORE_INC_INTERPOLATIONS_H_

#include <stdint.h>
#include <math.h>

float linear_interpolation(float input_value, float input_min, float input_max, float output_min, float output_max);

float symmetric_quadratic_interpolation(float input_value, float input_max, float output_max);

void normalize_vector(const float *input_array, float *output_array, uint8_t size);

#endif /* CORE_INC_INTERPOLATIONS_H_ */
