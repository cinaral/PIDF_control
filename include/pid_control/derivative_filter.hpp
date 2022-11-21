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
//* Derivative (Filtered) filter
//*
//*              s
//* DF(s) = -----------
//*          T_f s + 1
//*
//* x -> DF(s) -> y
template <size_t Y_DIM>
inline void
DF(const Real_T T_s, const Real_T T_f, const Real_T x[], const Real_T x_next[], const Real_T y[], Real_T y_next[])
{
	const Real_T coef_LHS = (2. * T_f + T_s);
	const Real_T coef_x = -2. / coef_LHS;
	const Real_T coef_x_next = 2. / coef_LHS;
	const Real_T coef_y = -(2. * T_f - T_s) / coef_LHS;

	for (size_t i = 0; i < Y_DIM; ++i) {
		y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
	}
}
//* This isn't faster, the compiler is smart enough to not repeat the same calc provided that consts are passed:
// template <size_t Y_DIM> class PDF
//{
//   private:
//	const Real_T T_f, T_s;
//	const Real_T K_p[Y_DIM];
//	const Real_T K_d[Y_DIM];
//	const Real_T coef_LHS = (2. * T_f + T_s);
//	const Real_T coef_y = -(2. * T_f - T_s) / coef_LHS;
//	const Real_T coef_x = -(K_p[0] * (2. * T_f - T_s) + 2. * K_d[0]) / coef_LHS;
//	const Real_T coef_x_next = (K_p[0] * (2. * T_f + T_s) + 2. * K_d[0]) / coef_LHS;
//  public:
//	PDF(const Real_T T_s, const Real_T T_f, const Real_T K_p[], const Real_T K_d[]) : T_f(T_f), T_s(T_s),
// K_p{K_p[0]}, K_d{K_d[0]}{};
//	void
//	filter(const Real_T x[], const Real_T x_next[], const Real_T y[], Real_T y_next[])
//	{
//		for (size_t i = 0; i < Y_DIM; ++i) {
//			y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
//		}
//	}
//};
} // namespace pidf_control

#endif
