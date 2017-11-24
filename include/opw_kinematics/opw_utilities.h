#ifndef OPW_UTILITIES_H
#define OPW_UTILITIES_H

#include <cmath>

namespace opw_kinematics
{

template <typename T>
inline bool isValid(const T* qs)
{
  return std::isfinite(qs[0]) && std::isfinite(qs[1]) && std::isfinite(qs[2]) && std::isfinite(qs[3]) &&
         std::isfinite(qs[4]) && std::isfinite(qs[5]);
}

template <typename T>
inline void harmonizeTowardZero(T* qs)
{
  const static T pi = T(M_PI);
  const static T two_pi = T(2.0 * M_PI);

  for (int i = 0; i < 6; i++) // TODO: Unroll manually?
  {
    if (qs[i] > pi) qs[i] -= two_pi;
    else if (qs[i] < -pi) qs[i] += two_pi;
  }
}

}

#endif // OPW_UTILITIES_H
