#ifndef OPW_UTILITIES_H
#define OPW_UTILITIES_H

#include <cmath>
#include <opw_kinematics/opw_kinematics.h>

namespace opw_kinematics
{
template <typename T>
inline bool isValid(const std::array<T, 6>& qs)
{
  return std::isfinite(qs[0]) && std::isfinite(qs[1]) && std::isfinite(qs[2]) && std::isfinite(qs[3]) &&
         std::isfinite(qs[4]) && std::isfinite(qs[5]);
}

template <typename T>
inline void harmonizeTowardZero(std::array<T, 6>& qs)
{
  const static auto pi = T(M_PI);
  const static auto two_pi = T(2.0 * M_PI);

  for (auto& q : qs)
  {
    if (q > pi)
      q -= two_pi;
    else if (q < -pi)
      q += two_pi;
  }
}

}  // namespace opw_kinematics

#endif  // OPW_UTILITIES_H
