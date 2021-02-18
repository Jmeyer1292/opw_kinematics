#ifndef OPW_PARAMETER_EXAMPLES_H
#define OPW_PARAMETER_EXAMPLES_H

#include "opw_kinematics/opw_kinematics.h"
#include <opw_kinematics/opw_macros.h>

namespace opw_kinematics
{
template <typename T>
Parameters<T> makeIrb2400_10()
{
  Parameters<T> p;
  p.a1 = T(0.100);
  p.a2 = T(-0.135);
  p.b = T(0.000);
  p.c1 = T(0.615);
  p.c2 = T(0.705);
  p.c3 = T(0.755);
  p.c4 = T(0.085);

  p.offsets[2] = static_cast<T>(-M_PI / 2.0);

  return p;
}

template <typename T>
DEPRECATED("UN-TESTED")
Parameters<T> makeFanucR2000iB_200R()
{
  Parameters<T> p;
  p.a1 = T(0.720);
  p.a2 = T(-0.225);
  p.b = T(0.000);
  p.c1 = T(0.600);
  p.c2 = T(1.075);
  p.c3 = T(1.280);
  p.c4 = T(0.235);

  // WARNING: This is a guess! I don't know the offets.
  p.offsets[2] = static_cast<T>(-M_PI / 2.0);

  return p;
}

template <typename T>
Parameters<T> makeKukaKR6_R700_sixx()
{
  Parameters<T> p;
  p.a1 = T(0.025);
  p.a2 = T(-0.035);
  p.b = T(0.000);
  p.c1 = T(0.400);
  p.c2 = T(0.315);
  p.c3 = T(0.365);
  p.c4 = T(0.080);

  p.offsets[1] = static_cast<T>(-M_PI / 2.0);
  p.sign_corrections[0] = -1;
  p.sign_corrections[3] = -1;
  p.sign_corrections[5] = -1;

  return p;
}

template <typename T>
DEPRECATED("UN-TESTED")
Parameters<T> makeStaubliTX40()
{
  Parameters<T> p;
  p.a1 = T(0.000);
  p.a2 = T(0.000);
  p.b = T(0.035);
  p.c1 = T(0.320);
  p.c2 = T(0.225);
  p.c3 = T(0.225);
  p.c4 = T(0.065);

  // WARNING: This is a guess! I don't know the offets.
  p.offsets[2] = static_cast<T>(-M_PI / 2.0);

  return p;
}

}  // namespace opw_kinematics

#endif  // OPW_PARAMETER_EXAMPLES_H
