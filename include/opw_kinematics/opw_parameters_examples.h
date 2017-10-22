#ifndef OPW_PARAMETER_EXAMPLES_H
#define OPW_PARAMETER_EXAMPLES_H

#include "opw_kinematics/opw_kinematics.h"

namespace opw_kinematics
{

template <typename T>
Parameters<T> makeIrb2400_10()
{
  return {T(.100), T(-0.135), T(0), T(.615), T(.705), T(.755), T(.085)};
}

template <typename T>
Parameters<T> makeFanucR2000iB_200R()
{
  return {T(.720), T(-0.225), T(0), T(.600), T(1.075), T(1.280), T(0.235)};
}

template <typename T>
Parameters<T> makeKukaKR6R700sixx()
{
  return {T(.025), T(-0.035), T(0), T(.400), T(0.315), T(0.365), T(0.080)};
}


} // ns opw_kinematics

#endif // OPW_PARAMETER_EXAMPLES_H
