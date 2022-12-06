/*
 * pid_control
 *  
 * MIT License
 * 
 * Copyright (c) 2022 Cinar, A. L.
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

#ifndef PIDF_FILTER_HPP_CINARAL_220925_0347
#define PIDF_FILTER_HPP_CINARAL_220925_0347

#include "types.hpp"

namespace pid_control
{
//* Proportional-Integral-Derivative (Filtered) filter
//*                                  s
//* PIDF(s) = K_p + K_i s + K_d -----------
//*                              T_f s + 1
//*
//* x -> PIDF(s) -> y
// template <size_t Y_DIM>
// void
// PIDF(const Real_T T_s, const Real_T T_f, const Real_T K_p[], const Real_T K_i[], const Real_T K_d[],
//      const Real_T x_prev[], const Real_T x[], const Real_T x_next[], const Real_T y_prev[], const Real_T y[],
//      Real_T y_next[])
//{
//	const Real_T coef_LHS = 2. * (2. * T_f + T_s);

//	for (size_t i = 0; i < Y_DIM; ++i) {
//		const Real_T coef_x_prev =
//		    (2. * K_p[i] * (2. * T_f - T_s) + K_i[i] * T_s * (T_s - 2. * T_f) + 4. * K_d[i]) / coef_LHS;
//		const Real_T coef_x = (-8. * K_p[i] * T_f + 2. * K_i[i] * T_s * T_s - 8. * K_d[i]) / coef_LHS;
//		const Real_T coef_x_next =
//		    (2. * K_p[i] * (2. * T_f + T_s) + K_i[i] * T_s * (T_s + 2. * T_f) + 4. * K_d[i]) / coef_LHS;
//		const Real_T coef_y_prev = (4. * T_f - T_s) / coef_LHS;
//		const Real_T coef_y = (-8. * T_f) / coef_LHS;

//		y_next[i] = coef_x_prev * x_prev[i] + coef_x * x[i] + coef_x_next * x_next[i] -
//		    coef_y_prev * y_prev[i] - coef_y * y[i];
//	}
//}
} // namespace pid_control

#endif
