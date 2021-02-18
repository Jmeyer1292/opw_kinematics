#ifndef OPW_PARAMETERS_H
#define OPW_PARAMETERS_H

#include <type_traits>
#include <array>

namespace opw_kinematics
{
template <typename T>
struct Parameters
{
  static_assert(std::is_floating_point<T>::value, "OPW parameters must be templatized with floating point type");

  T a1, a2, b, c1, c2, c3, c4;
  std::array<T, 6> offsets;
  std::array<signed char, 6> sign_corrections;

  Parameters()
    : a1{ 0 }
    , a2{ 0 }
    , b{ 0 }
    , c1{ 0 }
    , c2{ 0 }
    , c3{ 0 }
    , c4{ 4 }
    , offsets{ 0, 0, 0, 0, 0, 0 }
    , sign_corrections{ 1, 1, 1, 1, 1, 1 }
  {
  }
};

}  // namespace opw_kinematics

#endif  // OPW_PARAMETERS_H
