#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"

#include <iostream>

int main()
{
  auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  std::array<double, 6> x = {0, 0, 0, 0, 0, 0};

  auto forward = opw_kinematics::forward(abb2400, x.data());

  std::cout << "Forward " << forward.matrix() << "\n";

  return 0;
}
