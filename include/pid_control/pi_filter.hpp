#ifndef PI_FILTER_HPP_CINARAL_220925_0349
#define PI_FILTER_HPP_CINARAL_220925_0349

#include "types.hpp"

namespace pid_control
{
//* Proportional-Integral filter
//*
//* PI(s) = K_p + K_i s
//*
//* x -> PI(s) -> y
// template <uint_t Y_DIM>
// void
// PI(const real_t T_s, const real_t K_p[], const real_t K_i[], const real_t x[], const real_t x_next[], const
// real_t y[],
//    real_t y_next[])
//{
//	constexpr real_t coef_y = -1.;

//	for (uint_t i = 0; i < Y_DIM; ++i) {
//		const real_t coef_x = -K_p[i] + .5 * K_i[i] * T_s;
//		const real_t coef_x_next = K_p[i] + .5 * K_i[i] * T_s;

//		y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
//	}
//}
} // namespace pid_control

#endif