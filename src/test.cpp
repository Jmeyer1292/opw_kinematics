#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"
#include "opw_kinematics/opw_parameters_examples.h"
#include "opw_kinematics/opw_io.h"

#include <iomanip>

void printResults(const std::array<double, 6 * 8>& sols)
{
  std::cout <<  std::setprecision(5) << std::fixed;
  for (int i = 0; i < 8; ++i)
  {
    for (int j = 0; j < 6; ++j)
      std::cout << sols[i * 6 + j] << "   ";
    std::cout << "\n";
  }
}

int main()
{
  const auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  auto pose = opw_kinematics::Transform<double>::Identity();
  pose.translation() = Eigen::Vector3d(0.7, 0.2, 0);

  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(abb2400, pose, sols.data());

  opw_kinematics::isValid(sols.data());

  for (int i = 0; i < 8; ++i)
  {
    opw_kinematics::harmonizeTowardZero(&sols[i * 6]);
  }

  printResults(sols);

  for (int i = 0; i < 8; ++i)
  {
    std::cout << i << ": " << opw_kinematics::isValid(&sols[i * 6]) << "\n";
    std::cout << i << ":\n" << opw_kinematics::forward(abb2400, &sols[i * 6]).matrix() << "\n";
  }

  return 0;
}
