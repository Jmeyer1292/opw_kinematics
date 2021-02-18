#include <iomanip>
#include <iostream>
#include <array>

// IWYU pragma: no_include "opw_kinematics/opw_kinematics_impl.h"
#include "opw_kinematics/opw_utilities.h"  // IWYU pragma: keep
#include "opw_kinematics/opw_parameters_examples.h"

void printResults(const opw_kinematics::Solutions<double>& sols)
{
  std::cout << std::setprecision(5) << std::fixed;
  for (const auto& s : sols)
  {
    for (std::size_t i = 0; i < 6; ++i)
      std::cout << s[i] << "   ";

    std::cout << "\n";
  }
}

int main()
{
  const auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  auto pose = opw_kinematics::Transform<double>::Identity();
  pose.translation() = Eigen::Vector3d(0.7, 0.2, 0);

  opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(abb2400, pose);
  opw_kinematics::isValid(sols[0]);

  for (auto& s : sols)
  {
    if (opw_kinematics::isValid(s))
      opw_kinematics::harmonizeTowardZero(s);
  }

  printResults(sols);

  for (std::size_t i = 0; i < 8; ++i)
  {
    std::cout << i << ": " << opw_kinematics::isValid(sols[i]) << "\n";
    std::cout << i << ":\n" << opw_kinematics::forward(abb2400, sols[i]).matrix() << "\n";
  }

  return 0;
}
