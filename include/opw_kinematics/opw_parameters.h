#ifndef OPW_PARAMETERS_H
#define OPW_PARAMETERS_H

#include <type_traits>

namespace opw_kinematics
{

template <typename T>
struct Parameters
{
//  static_assert(std::is_floating_point<T>::value,
//                "OPW parameters must be templatized with floating point type");

  T a1, a2, b, c1, c2, c3, c4;
  T offsets[6];
  signed char sign_corrections[6];

  Parameters()
    : a1{static_cast<T>(0.0)}, a2{static_cast<T>(0.0)}, b{static_cast<T>(0.0)}, c1{static_cast<T>(0.0)}, c2{static_cast<T>(0.0)}, c3{static_cast<T>(0.0)}, c4{static_cast<T>(0.0)},
      offsets{static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0)},
      sign_corrections{1, 1, 1, 1, 1, 1}
  {}
};

}

#endif // OPW_PARAMETERS_H
