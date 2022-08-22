#ifndef MATRIX_HPP_CINARAL_220822_1110
#define MATRIX_HPP_CINARAL_220822_1110

//* basic matrix operations

#include "types.hpp"

namespace matrix
{

//* selects a row from an N_ROW by M_COL matrix
template <uint_t M_COL>
static const real_t *
select_row(const uint_t row_idx, const real_t arr[])
{
	const real_t *row_ptr = arr + row_idx * M_COL;
	return row_ptr;
}

//* replaces a row of an N_ROW by M_COL matrix
template <uint_t M_COL>
static void
replace_row(const uint_t row_idx, const real_t sel[], real_t arr[])
{
	//* faster than std::memmove(arr + row_idx * M_COL, sel, M_COL * sizeof(real_t));
	//* identical to: *arr + row_idx * M_COL + i = *sel + i
	for (uint_t i = 0; i < M_COL; ++i) {
		arr[row_idx * M_COL + i] = sel[i];
	}
}

} // namespace matrix

#endif