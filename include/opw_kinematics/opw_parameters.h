#ifndef OPW_PARAMETERS_H
#define OPW_PARAMETERS_H

#include <type_traits>

namespace opw_kinematics
{

template <typename T>
struct Parameters
{
  static_assert(std::is_floating_point<T>::value,
                "OPW parameters must be templatized with floating point type");

  T a1, a2, b, c1, c2, c3, c4;
};

}

#endif // OPW_PARAMETERS_H
