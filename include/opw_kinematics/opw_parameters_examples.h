#ifndef OPW_PARAMETER_EXAMPLES_H
#define OPW_PARAMETER_EXAMPLES_H

#include "opw_kinematics/opw_kinematics.h"

namespace opw_kinematics
{

template <typename T>
Parameters<T> makeIrb2400_10()
{
  Parameters<T> p;
  p.a1 = T(0.100);
  p.a2 = T(-0.135);
  p.b =  T(0.000);
  p.c1 = T(0.615);
  p.c2 = T(0.705);
  p.c3 = T(0.755);
  p.c4 = T(0.085);

  p.offsets[2] = -M_PI / 2.0;

  return p;
}



} // ns opw_kinematics

#endif // OPW_PARAMETER_EXAMPLES_H
