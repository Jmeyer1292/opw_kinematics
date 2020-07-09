#ifndef OPW_IO_H
#define OPW_IO_H

#include <iostream>
#include "opw_kinematics/opw_parameters.h"  // IWYU pragma: export

namespace opw_kinematics
{
template <typename T>
std::ostream& operator<<(std::ostream& os, const Parameters<T>& params)
{
  os << "Distances: [" << params.a1 << " " << params.a2 << " " << params.b << " " << params.c1 << " " << params.c2
     << " " << params.c3 << " " << params.c4 << "]\n";
  os << "Offsets = [";
  for (std::size_t i = 0; i < 6; ++i)
  {
    os << params.offsets[i] << " ";
  }
  os << "]\nSign_corrections = [";
  for (std::size_t i = 0; i < 6; ++i)
  {
    os << static_cast<int>(params.sign_corrections[i]) << " ";
  }
  os << "]";
  return os;
}

}  // namespace opw_kinematics

#endif  // OPW_IO_H
