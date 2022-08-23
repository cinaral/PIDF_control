#ifndef PIDF_HPP_CINARAL_220822_0805
#define PIDF_HPP_CINARAL_220822_0805

#include "types.hpp"

namespace pidf
{

//* Derivative (Filtered) filter
//*
//*              s
//* DF(s) = -----------
//*          T_f s + 1
//*
//* x -> DF(s) -> y
template <uint_t Y_DIM>
void
DF(const real_t T_s, const real_t T_f, const real_t x[], const real_t x_next[], const real_t y[], real_t y_next[])
{
	const real_t coef_LHS = (2. * T_f + T_s);
	const real_t coef_x = -2. / coef_LHS;
	const real_t coef_x_next = 2. / coef_LHS;
	const real_t coef_y = -(2. * T_f - T_s) / coef_LHS;

	for (uint_t i = 0; i < Y_DIM; ++i) {
		y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
	}
}

//* This isn't faster, the compiler is smart enough to not repeat the same calc provided that consts are passed:
//template <uint_t Y_DIM> class DF
//{
//   private:
// const real_t T_f, T_s;
// const real_t coef_LHS = (2. * T_f + T_s);
// const real_t coef_x = -2. / coef_LHS;
// const real_t coef_x_next = 2. / coef_LHS;
// const real_t coef_y = -(2. * T_f - T_s) / coef_LHS;

//  public:
//    DF(const real_t T_s, const real_t T_f) : T_f(T_f), T_s(T_s){};

//    void
//    filter(const real_t x[], const real_t x_next[], const real_t y[], real_t y_next[])
//    {
//        for (uint_t i = 0; i < Y_DIM; ++i) {
//            y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
//        }
//    }
//};

//* Proportional-Derivative (Filtered) filter
//*                         s
//* PDF(s) = K_p + K_d -----------
//*                     T_f s + 1
//*
//* x -> PDF(s) -> y
template <uint_t Y_DIM>
void
PDF(const real_t T_s, const real_t T_f, const real_t K_p[], const real_t K_d[], const real_t x[], const real_t x_next[],
    const real_t y[], real_t y_next[])
{
	const real_t coef_LHS = (2. * T_f + T_s);
	const real_t coef_y = -(2. * T_f - T_s) / coef_LHS;

	for (uint_t i = 0; i < Y_DIM; ++i) {
		const real_t coef_x = -(K_p[i] * (2. * T_f - T_s) + 2. * K_d[i]) / coef_LHS;
		const real_t coef_x_next = (K_p[i] * (2. * T_f + T_s) + 2. * K_d[i]) / coef_LHS;

		y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
	}
}

//! NEEDS INTERNAL STATE
////* Proportional-Integral filter
////*
////* PI(s) = K_p + K_i s
////*
////* x -> PI(s) -> y
//template <uint_t Y_DIM>
//void
//PI(const real_t T_s, const real_t K_p[], const real_t K_i[], const real_t x[], const real_t x_next[], const real_t y[],
//   real_t y_next[])
//{
//	constexpr real_t coef_y = -1.;

//	for (uint_t i = 0; i < Y_DIM; ++i) {
//		const real_t coef_x = -K_p[i] + .5 * K_i[i] * T_s;
//		const real_t coef_x_next = K_p[i] + .5 * K_i[i] * T_s;

//		y_next[i] = coef_x * x[i] + coef_x_next * x_next[i] - coef_y * y[i];
//	}
//}

//! NEEDS INTERNAL STATE
////* Proportional-Integral-Derivative (Filtered) filter
////*                                  s
////* PIDF(s) = K_p + K_i s + K_d -----------
////*                              T_f s + 1
////*
////* x -> PIDF(s) -> y
//template <uint_t Y_DIM>
//void
//PIDF(const real_t T_s, const real_t T_f, const real_t K_p[], const real_t K_i[], const real_t K_d[],
//     const real_t x_prev[], const real_t x[], const real_t x_next[], const real_t y_prev[], const real_t y[],
//     real_t y_next[])
//{
//	const real_t coef_LHS = 2. * (2. * T_f + T_s);

//	for (uint_t i = 0; i < Y_DIM; ++i) {
//		const real_t coef_x_prev =
//		    (2. * K_p[i] * (2. * T_f - T_s) + K_i[i] * T_s * (T_s - 2. * T_f) + 4. * K_d[i]) / coef_LHS;
//		const real_t coef_x = (-8. * K_p[i] * T_f + 2. * K_i[i] * T_s * T_s - 8. * K_d[i]) / coef_LHS;
//		const real_t coef_x_next =
//		    (2. * K_p[i] * (2. * T_f + T_s) + K_i[i] * T_s * (T_s + 2. * T_f) + 4. * K_d[i]) / coef_LHS;
//		const real_t coef_y_prev = (4. * T_f - T_s) / coef_LHS;
//		const real_t coef_y = (-8. * T_f) / coef_LHS;

//		y_next[i] = coef_x_prev * x_prev[i] + coef_x * x[i] + coef_x_next * x_next[i] -
//		    coef_y_prev * y_prev[i] - coef_y * y[i];
//	}
//}

} // namespace pidf

#endif