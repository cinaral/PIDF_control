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

#ifndef FILTER_HPP_CINARAL_221129_2345
#define FILTER_HPP_CINARAL_221129_2345

#include "matrix_op/row_operations.hpp"
#include "types.hpp"

namespace pid_control
{
//* Realizable filter
//*
//*              s
//* F(s) = -----------
//*          T_f s + 1
//*
//* in -> F(s) -> out
template <size_t IN_DIM, size_t OUT_DIM, size_t ORDER>
inline void
derivative(const Real_T coef_in_prev[IN_DIM * ORDER], const Real_T T_filter,
           const Real_T (&in)[IN_DIM], Real_T (&out)[OUT_DIM])
{
	static bool has_initialized = false;
	static Real_T x[IN_DIM * ORDER];
	static Real_T y[OUT_DIM * ORDER];

	matrix_op::replace_row<IN_DIM, ORDER>(0, in, x);
	matrix_op::replace_row<OUT_DIM, ORDER>(0, out, y);

	if (!has_initialized) {
		Real_T in_prev[IN_DIM];
		Real_T out_prev[OUT_DIM];

		for (size_t i = 0; i < IN_DIM; ++i) {
			in_prev[i] = 0.;
		}

		for (size_t i = 0; i < OUT_DIM; ++i) {
			out_prev[i] = 0.;
		}
		matrix_op::replace_row<IN_DIM, ORDER>(1, in_prev, x);
		matrix_op::replace_row<OUT_DIM, ORDER>(1, out_prev, y);
		has_initialized = true;
	}

	for (size_t i = 0; i < IN_DIM * ORDER; ++i) {
		y[i] = coef_x[i] * x[i] - coef_y[i] * y[i];
	}
	matrix_op::replace_row<IN_DIM, ORDER>(1, in, x);
	matrix_op::replace_row<OUT_DIM, ORDER>(1, out, y);
}
} // namespace pid_control

#endif
