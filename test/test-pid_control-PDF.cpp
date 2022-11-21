#include "matrix_op.hpp"
#include "matrix_rw.hpp"
#include "pid_control.hpp"
#include "rk4_solver.hpp"

using Real_T = pid_control::Real_T;
using size_t = pid_control::size_t;

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

constexpr size_t sample_freq = 1e3;
constexpr Real_T h = 1. / sample_freq;
constexpr Real_T t0 = 0;
constexpr Real_T tf = 1;
constexpr size_t t_dim = sample_freq*(tf - t0) + 1;
constexpr size_t x_dim = 3;
constexpr size_t u_dim = 1;
constexpr Real_T x0[x_dim] = {0,0,0};
constexpr Real_T T_f = h * 1e1;
constexpr Real_T K_p[u_dim] = {6.};
constexpr Real_T K_d[u_dim] = {.2};
constexpr Real_T R = 1; //* unit step

#ifdef __USE_SINGLE_PRECISION__
constexpr Real_T error_thres = 1e-4;
#else
constexpr Real_T error_thres = 1e-11;
#endif

//* Crouzet 89830012:
constexpr Real_T A[x_dim * x_dim] = {0., 1., 0., 0., -3.038760, 496.1240, 0., -37.64706, -823.5294};
constexpr Real_T B[x_dim * u_dim] = {0., 0., 588.2353};

Real_T u[u_dim] = {0};
Real_T u_arr[t_dim * u_dim];
Real_T error[u_dim] = {0};

void
control_fun(const Real_T x_next[], Real_T u_next[])
{
	Real_T error_next[u_dim] = {R - x_next[0]};

	pid_control::PDF<u_dim>(h, T_f, K_p, K_d, error, error_next, u, u_next);

	error[0] = error_next[0];
	u[0] = u_next[0];
}

struct Dynamics {
	//* dt__x = A*x + B*x
	void
	ode_fun(const Real_T, const Real_T (&x)[x_dim], const size_t i, Real_T (&dt__x)[x_dim])
	{
		Real_T temp0[x_dim];
		Real_T temp1[x_dim];

		const Real_T(&u)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr);

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
	Real_T x_arr_chk[t_dim * x_dim];
	Real_T u_arr_chk[t_dim * u_dim];
	matrix_rw::read<t_dim, x_dim>(ref_dat_prefix + x_arr_chk_fname, x_arr_chk);
	matrix_rw::read<t_dim, u_dim>(ref_dat_prefix + u_arr_chk_fname, u_arr_chk);

	//* test
	Real_T t_arr[t_dim];

	Real_T x[x_dim];
	matrix_op::replace_row<1, x_dim>(0, x0, x); //* initialize x
	Real_T t = t0;                              //* initialize t
	t_arr[0] = t;

	Real_T x_arr[t_dim * x_dim];
	matrix_op::replace_row<t_dim, x_dim>(0, x, x_arr);

	for (size_t i = 0; i < t_dim - 1; ++i) {
		Real_T u_next[u_dim];
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
	Real_T max_x_error = 0.;
	Real_T max_u_error = 0.;

	for (size_t i = 0; i < t_dim; ++i) {
		const Real_T(&x_)[x_dim] = *matrix_op::select_row<t_dim, x_dim>(i, x_arr);
		const Real_T(&x_chk_)[x_dim] = *matrix_op::select_row<t_dim, x_dim>(i, x_arr_chk);

		for (size_t j = 0; j < x_dim; ++j) {
			const Real_T x_error = std::abs(x_[j] - x_chk_[j]);
			if (x_error > max_x_error) {
				max_x_error = x_error;
			}
		}

		const Real_T(&u_)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr);
		const Real_T(&u_chk_)[u_dim] = *matrix_op::select_row<t_dim, u_dim>(i, u_arr_chk);

		const Real_T u_error = std::abs(u_[0] - u_chk_[0]);
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
