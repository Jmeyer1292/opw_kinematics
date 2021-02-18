#include <opw_kinematics/opw_macros.h>
OPW_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>  // IWYU pragma: keep
#include <cstdlib>        // for abs, size_t
#include <array>          // for array
#include <cmath>          // for fmod, M_PI, isnan
#include <iomanip>        // for operator<<, setp...
#include <iostream>       // for operator<<, basi...
#include <vector>         // for vector
#include "ikfast.h"
OPW_IGNORE_WARNINGS_POP

#include "opw_kinematics/opw_kinematics.h"           // IWYU pragma: keep
#include "opw_kinematics/opw_utilities.h"            // IWYU pragma: keep
#include "opw_kinematics/opw_parameters_examples.h"  // for makeIrb2400_10

using Transform = opw_kinematics::Transform<double>;
using Solutions = opw_kinematics::Solutions<double>;

struct PoseGenerator
{
  PoseGenerator() { index = 0; }

  Transform operator()()
  {
    const static int width = 100;
    int y = index % width;
    int z = index / width;

    Transform p = Transform::Identity();
    p.translation() = Eigen::Vector3d(0.75, y * 0.01 - 0.5, 0.5 + z * 0.01);

    index++;

    return p;
  }

  int index;
};

void solveIKFast(const Transform& p, ikfast::IkSolutionList<double>& sols)
{
  double eetrans[3];
  double eerot[9];
  double* pfree = nullptr;

  eetrans[0] = p.translation().x();
  eetrans[1] = p.translation().y();
  eetrans[2] = p.translation().z();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      eerot[i * 3 + j] = p.matrix()(i, j);
    }
  }

  ComputeIk(eetrans, eerot, pfree, sols);
}

Solutions solveOPW(const opw_kinematics::Parameters<double>& param, const Transform& p)
{
  return opw_kinematics::inverse(param, p);
}

std::size_t countValidSolutions(const Solutions& opw)
{
  std::size_t count = 0;
  for (const auto& s : opw)
  {
    if (opw_kinematics::isValid(s))
      count++;
  }
  return count;
}

/*!
 * \brief normalize_angle_positive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
static inline double normalize_angle_positive(double angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
static inline double normalize_angle(double angle)
{
  double a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

/*!
 * \function
 * \brief shortest_angular_distance
 *
 * Given 2 angles, this returns the shortest angular
 * difference.  The inputs and ouputs are of course radians.
 *
 * The result
 * would always be -pi <= result <= pi.  Adding the result
 * to "from" will always get you an equivelent angle to "to".
 */

static inline double shortest_angular_distance(double from, double to) { return normalize_angle(to - from); }

bool findSolInSet(const std::array<double, 6>& s, const Solutions& opw)
{
  for (std::size_t i = 0; i < 8; ++i)
  {
    bool is_same = true;
    for (std::size_t j = 0; j < 6; ++j)
    {
      double value = opw[i][j];
      //      if (value > M_PI) value -= 2*M_PI;
      //      if (value < -M_PI) value += 2*M_PI;
      double diff = std::abs(shortest_angular_distance(s[j], value));
      if (diff > 1e-6)
      {
        is_same = false;
        break;
      }
    }
    if (is_same)
      return true;
  }
  return false;
}

void printResults(const Solutions& sols)
{
  std::cout << std::setprecision(5) << std::fixed;
  for (const auto& s : sols)
  {
    for (std::size_t i = 0; i < 6; ++i)
      std::cout << s[i] << "   ";
    std::cout << "\n";
  }
}

void compare(ikfast::IkSolutionList<double>& ikf, Solutions& opw)
{
  auto num_ikf_sols = ikf.GetNumSolutions();
  auto num_opw_sols = countValidSolutions(opw);

  bool equal_num = num_ikf_sols == num_opw_sols;
  EXPECT_TRUE(equal_num);

  if (!equal_num)
    return;

  // lets compare solutions
  std::array<double, 6> v;
  for (decltype(num_ikf_sols) i = 0; i < num_ikf_sols; ++i)
  {
    ikf.GetSolution(i).GetSolution(v.data(), nullptr);
    const bool found_sol_in_opw = findSolInSet(v, opw);
    EXPECT_TRUE(found_sol_in_opw);
    if (!found_sol_in_opw)
    {
      std::cout << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5] << "\n";

      printResults(opw);
    }
  }
}

TEST(ikfast_to_opw, similar_solutions)  // NOLINT
{
  // IK-Fast is implicitly instantiated

  // Make OPW model
  const auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  // Create a pose generator
  PoseGenerator gen;

  for (int i = 0; i < 40000; ++i)
  {
    Transform p = gen();

    // Solve with IKFast
    ikfast::IkSolutionList<double> ikf_sols;
    solveIKFast(p, ikf_sols);

    // Solve with OPW
    Solutions opw_sols = solveOPW(abb2400, p);

    // Compare
    compare(ikf_sols, opw_sols);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
