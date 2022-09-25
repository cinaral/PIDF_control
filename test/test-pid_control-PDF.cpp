#include "matrix_op.hpp"
#include "matrix_rw.hpp"
#include "pid_control.hpp"
#include "rk4_solver.hpp"

using real_t = pid_control::real_t;
using uint_t = pid_control::uint_t;

//* setup
const std::string dat_dir = "../dat";
const std::string ref_dat_dir = "../../test/reference_dat";
const std::string test_name = "test-pid_control-PDF";
const std::string dat_prefix = dat_dir + "/" + test_name + "-";
const std::string ref_dat_prefix = ref_dat_dir + "/" + test_name + "-";
const std::string t_arr_fname = "t_arr.dat";
const std::string x_arr_fname = "x_arr.dat";
const std::string x_arr_chk_fname = "x_arr_chk.dat";
const std::string u_arr_fname = "u_arr.dat";
const std::string u_arr_chk_fname = "u_arr_chk.dat";

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
constexpr real_t error_thres = 1e-4;
#else
constexpr real_t error_thres = 1e-11;
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

	pid_control::PDF<u_dim>(h, T_f, K_p, K_d, error, error_next, u, u_next);

	error[0] = error_next[0];
	u[0] = u_next[0];
}

struct Dynamics {
	void
	ode_fun(const real_t, const real_t (&x)[x_dim], const uint_t i, real_t (&dt__x)[x_dim])
	{
		real_t temp0[x_dim];
		real_t temp1[x_dim];

		const real_t(&u)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr);

		matrix_op::right_multiply<x_dim, x_dim>(A, x, temp0);
		matrix_op::right_multiply<x_dim, u_dim>(B, u, temp1);
		matrix_op::sum<x_dim>(temp0, temp1, dt__x);
	}
};
Dynamics dyn;

int
main()
{
	//* read reference data
	matrix_rw::read<t_dim, x_dim>(ref_dat_prefix + x_arr_chk_fname, x_arr_chk);
	matrix_rw::read<t_dim, u_dim>(ref_dat_prefix + u_arr_chk_fname, u_arr_chk);

	//* test
	real_t x[x_dim];
	matrix_op::replace_row<1, x_dim>(0, x0, x); //* initialize x
	real_t t = t0;                              //* initialize t

	t_arr[0] = t;
	matrix_op::replace_row<t_dim, x_dim>(0, x, x_arr);

	for (uint_t i = 0; i < t_dim - 1; ++i) {
		real_t u_next[u_dim];
		control_fun(x, u_next);
		u_arr[i] = u_next[0];

		rk4_solver::step<Dynamics, x_dim>(dyn, &Dynamics::ode_fun, t, x, h, i, x); //* update x to the next x

		t = t0 + (i + 1) * h; //* update t to the next t

		t_arr[i + 1] = t;
		matrix_op::replace_row<t_dim, x_dim>(i + 1, x, x_arr);
	}

	//* write test data
	matrix_rw::write<t_dim, 1>(dat_prefix + t_arr_fname, t_arr);
	matrix_rw::write<t_dim, x_dim>(dat_prefix + x_arr_fname, x_arr);
	matrix_rw::write<t_dim, u_dim>(dat_prefix + u_arr_fname, u_arr);

	//* verify
	real_t max_x_error = 0.;
	real_t max_u_error = 0.;

	for (uint_t i = 0; i < t_dim; ++i) {
		const real_t (&x_)[x_dim] = *matrix_op::select_row<t_dim, x_dim>(i, x_arr);
		const real_t (&x_chk_)[x_dim] = *matrix_op::select_row<t_dim, x_dim>(i, x_arr_chk);

		for (uint_t j = 0; j < x_dim; ++j) {
			real_t x_error = std::abs(x_[j] - x_chk_[j]);
			if (x_error > max_x_error) {
				max_x_error = x_error;
			}
		}

		const real_t (&u_)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr);
		const real_t (&u_chk_)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr_chk);

		real_t u_error = std::abs(u_[0] - u_chk_[0]);
		if (u_error > max_u_error) {
			max_u_error = u_error;
		}
	}

	if (max_x_error < error_thres && max_u_error < error_thres) {
		return 0;
	} else {
		return 1;
	}
}
