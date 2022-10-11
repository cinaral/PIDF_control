#include "matrix_rw.hpp"
#include "pid_control.hpp"
#include <cmath>

using real_t = pid_control::real_t;
using uint_t = pid_control::uint_t;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//* setup
const std::string dat_dir = "../dat";
const std::string ref_dat_dir = "../../test/reference_dat";
const std::string test_name = "test-pid_control-DF";
const std::string dat_prefix = dat_dir + "/" + test_name + "-";
const std::string ref_dat_prefix = ref_dat_dir + "/" + test_name + "-";
const std::string t_arr_fname = "t_arr.dat";
const std::string dt__x_arr_fname = "dt__x_arr.dat";

constexpr uint_t sample_freq = 1e5;
constexpr real_t h = 1. / sample_freq;
constexpr real_t t0 = 0;
constexpr real_t tf = 1;
constexpr uint_t t_dim = sample_freq*(tf - t0) + 1;
constexpr uint_t x_dim = 1;
constexpr real_t sine_freq = 3.;
constexpr real_t T_f = h * 1e1;

#ifdef __USE_SINGLE_PRECISION__
constexpr real_t error_thres = 1e-1;
#else
constexpr real_t error_thres = 1e-1;
#endif

int
main()
{
	//* test
	real_t x[x_dim] = {0};
	real_t dt__x[x_dim] = {0};
	real_t x_next[x_dim];
	real_t t_arr[t_dim];
	real_t dt__x_arr[t_dim * x_dim] = {0};

	for (uint_t i = 0; i < t_dim; ++i) {
		t_arr[i] = i * h;
		x_next[0] = sin(t_arr[i] * 2. * M_PI * sine_freq);

		pid_control::DF<x_dim>(h, T_f, x, x_next, dt__x, dt__x_arr + i);

		//* next x, dt__x becomes previous
		x[0] = x_next[0];
		dt__x[0] = dt__x_arr[i];
	}

	//* write test data
	matrix_rw::write<t_dim, 1>(dat_prefix + t_arr_fname, t_arr);
	matrix_rw::write<t_dim, x_dim>(dat_prefix + dt__x_arr_fname, dt__x_arr);

	//* verify
	real_t max_error = 0.;

	for (uint_t i = 1e4; i < t_dim; ++i) {

		const real_t error = std::abs(dt__x_arr[i] - 2 * M_PI * sine_freq * cos(t_arr[i] * 2 * M_PI * sine_freq));
		if (error > max_error) {
			max_error = error;
		}
	}

	if (max_error < error_thres) {
		return 0;
	} else {
		return 1;
	}
}
