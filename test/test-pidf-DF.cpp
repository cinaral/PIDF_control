#include "matrix_rw.hpp"

//********
//* setup
//********
const std::string dat_dir = "../../dat";
const std::string test_name = "test-pidf-DF";
const std::string dir_prefix = dat_dir + "/" + test_name + "-";
const std::string x_arr_fname = "x_arr.dat";

constexpr real_t PI = 3.1415926535897932;
constexpr real_t f = 5.;
#ifdef __USE_SINGLE_PRECISION__
constexpr real_t error_thres = 1e-5;
#else
constexpr real_t error_thres = 1e-9;
#endif

int
main()
{
	//*******
	//* test
	//*******

	sin(t_arr[i] * 2 * PI * f);

	//******************
	//* write test data
	//******************

	//*********
	//* verify
	//*********
	real_t max_error = 0.;

	// for (uint_t i = 0; i < t_dim; ++i) {

	//	real_t error = std::abs(x_[0] - sin(t_arr[i] * 2 * PI * f));
	//	if (error > max_error) {
	//		max_error = error;
	//	}
	//}

	if (max_error < error_thres) {
		return 0;
	} else {
		return 1;
	}
}