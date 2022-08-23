//* requires input data for verification
//* test__pidf_PDF.m can generate it in ./dat if you have MATLAB (see README.md)
//* then copy to ./test/dat or use ./scripts/update_test_data.sh

#include "matrix.hpp"
#include "matrix_rw.hpp"
#include "pidf.hpp"
#include "rk4_solver.hpp"
#include <cmath>

//********
//* setup
//********
const std::string dat_dir = "../../dat";
const std::string test_dat_dir = "../../test/dat";
const std::string test_name = "test-pidf-PDF";
const std::string dir_prefix = dat_dir + "/" + test_name + "-";
const std::string test_dat_prefix = test_dat_dir + "/" + test_name + "-";
const std::string t_arr_fname = "t_arr.dat";
const std::string x_arr_fname = "x_arr.dat";
const std::string x_arr_chk_fname = "x_arr_chk.dat";
const std::string u_arr_fname = "u_arr.dat";

constexpr uint_t t_dim = 1e3;
constexpr uint_t x_dim = 3;
constexpr uint_t u_dim = 1;
constexpr real_t t0 = 0;
constexpr real_t x0[x_dim] = {0};
constexpr real_t h = 1. / (t_dim - 1);
constexpr real_t T_f = h * 1e1;
constexpr real_t K_p[u_dim] = {6.};
constexpr real_t K_d[u_dim] = {.2};
constexpr real_t R = 1; //* unit step
#ifdef __USE_SINGLE_PRECISION__
constexpr real_t error_thres = 1e-5;
#else
constexpr real_t error_thres = 1e-13;
#endif

//* Crouzet 89830012:
constexpr real_t A[x_dim * x_dim] = {0., 1., 0., 0., -3.038760, 496.1240, 0., -37.64706, -823.5294};
constexpr real_t B[x_dim * u_dim] = {0., 0., 588.2353};

real_t t;
real_t x[x_dim];
real_t t_arr[t_dim];
real_t x_arr[t_dim * x_dim];
real_t x_arr_chk[t_dim * x_dim];
real_t u_arr[t_dim * u_dim];
real_t u_arr_chk[t_dim * u_dim];

real_t u[u_dim] = {0};
real_t error[u_dim] = {0};

//* dt__x = A*x + B*x
void
control_fun(const real_t x_next[], real_t u_next[])
{
	real_t error_next[u_dim] = {R - x_next[0]};

	pidf::PDF<u_dim>(h, T_f, K_p, K_d, error, error_next, u, u_next);

	error[0] = error_next[0];
	u[0] = u_next[0];
}

void
ode_fun(const real_t, const real_t x[], const uint_t i, real_t dt__x[])
{
	real_t temp0[x_dim];
	real_t temp1[x_dim];

	const real_t *u = matrix::select_row<u_dim>(i, u_arr);

	matrix::right_multiply<x_dim, x_dim>(A, x, temp0);
	matrix::right_multiply<x_dim, u_dim>(B, u, temp1);
	matrix::sum<x_dim>(temp0, temp1, dt__x);
}

int
main()
{
	//*****************
	//* read test data
	//*****************
	matrix_rw::read<t_dim, x_dim>(test_dat_prefix + x_arr_chk_fname, x_arr_chk);

	//*******
	//* test
	//*******
	// rk4_solver::cum_loop<ode_fun, t_dim, x_dim>(t0, x0, h, t_arr, x_arr);

	real_t x[x_dim] = {0};
	matrix::replace_row<x_dim>(0, x0, x); //* initialize x
	real_t t = t0;                        //* initialize t

	t_arr[0] = t;
	matrix::replace_row<x_dim>(0, x, x_arr);

	for (uint_t i = 0; i < t_dim - 1; ++i) {
		real_t u_next[u_dim];
		control_fun(x, u_next);
		u_arr[i] = u_next[0];

		rk4_solver::step<ode_fun, x_dim>(t, x, h, i, x); //* update x to the next x

		t = t0 + (i + 1) * h; //* update t to the next t

		t_arr[i + 1] = t;
		matrix::replace_row<x_dim>(i + 1, x, x_arr);
	}

	//******************
	//* write test data
	//******************
	matrix_rw::write<t_dim, 1>(dir_prefix + t_arr_fname, t_arr);
	matrix_rw::write<t_dim, x_dim>(dir_prefix + x_arr_fname, x_arr);
	matrix_rw::write<t_dim, u_dim>(dir_prefix + u_arr_fname, u_arr);

	//*********
	//* verify
	//*********
	real_t max_error = 0.;

	for (uint_t i = 0; i < t_dim; ++i) {
		const real_t *x_ = matrix::select_row<x_dim>(i, x_arr);
		const real_t *x_chk_ = matrix::select_row<x_dim>(i, x_arr_chk);

		for (uint_t j = 0; j < x_dim; ++j) {
			real_t error = std::abs(x_[j] - x_chk_[j]);
			if (error > max_error) {
				max_error = error;
			}
		}
	}

	if (max_error < error_thres) {
		return 0;
	} else {
		return 1;
	}
}
