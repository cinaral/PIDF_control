/*
 * pid_control
 *
 * MIT License
 *
 * Copyright (c) 2022 cinaral
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef DERIVATIVE_FILTER_HPP_CINARAL_220925_0348
#define DERIVATIVE_FILTER_HPP_CINARAL_220925_0348

#include "types.hpp"

namespace pid_control
{
//* Realizable derivative filter
//*
//*              s
//* DF(s) = -----------
//*          T_f s + 1
//*
//* x -> DF(s) -> y
template <size_t X_DIM, size_t Y_DIM>
inline void
derivative(const Real_T T_sample, const Real_T T_filter, const Real_T (&x)[X_DIM], Real_T (&y)[Y_DIM])
{
	static bool has_initialized = false;
	static Real_T x_prev[X_DIM];
	static Real_T y_prev[X_DIM];

	
	const Real_T coef_LHS = (2. * T_filter + T_sample);
	const Real_T coef_x_prev = -2. / coef_LHS;
	const Real_T coef_x = 2. / coef_LHS;
	const Real_T coef_y_prev = -(2. * T_filter - T_sample) / coef_LHS;

	if (!has_initialized) {
		for (size_t i = 0; i < Y_DIM; ++i) {
			x_prev[i] = 0;
			y_prev[i] = 0;
		}
		has_initialized = true;
		return;
	}

	for (size_t i = 0; i < Y_DIM; ++i) {
		y[i] = coef_x_prev * x_prev[i] + coef_x * x[i] - coef_y_prev * y[i];
	}

	for (size_t i = 0; i < X_DIM; ++i) {
		x_prev[i] = x[i];
	}
}
} // namespace pid_control

#endif
