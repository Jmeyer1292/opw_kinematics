#ifndef OPW_IO_H
#define OPW_IO_H

#include <iostream>
#include "opw_kinematics/opw_parameters.h"

namespace opw_kinematics
{

template <typename T>
std::ostream& operator<<(std::ostream& os, const Parameters<T>& params)
{
  os << params.a1 << " " << params.a2 << " " << params.b << " " <<
        params.c1 << " " << params.c2 << " " << params.c3 << " " <<
        params.c4 << "\n";
  os << "Offsets = [" << params.offsets[0] << " " << params.offsets[1] <<
        " " << params.offsets[2] << " " << params.offsets[3] << " " <<
        params.offsets[4] << " " << params.offsets[5] << "]\n";
  return os;
}

}

#endif // OPW_IO_H
