#ifndef TYPES_HPP_CINARAL_220925_0344
#define TYPES_HPP_CINARAL_220925_0344

namespace pid_control
{
using uint_t = unsigned long long int;
#ifdef __USE_SINGLE_PRECISION__
using real_t = float;
#else
using real_t = double;
#endif
} // namespace pidf_control

#endif