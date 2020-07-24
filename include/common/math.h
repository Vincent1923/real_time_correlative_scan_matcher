#ifndef COMMON_MATH_H_
#define COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "port.h"

namespace common {

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

}  // namespace common

#endif  // COMMON_MATH_H_
