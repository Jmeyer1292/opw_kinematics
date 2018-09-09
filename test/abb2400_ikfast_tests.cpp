#include <gtest/gtest.h>

#include "ikfast.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"

using Transform = opw_kinematics::Transform<double>;

struct PoseGenerator
{
  PoseGenerator()
  {
    index = 0;
  }

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
      eerot[i * 3 + j] = p.matrix()(i,j);
    }
  }

  ComputeIk(eetrans, eerot, pfree, sols);
}

void solveOPW(const opw_kinematics::Parameters<double>& param, const Transform& p,
              std::array<double, 6 * 8>& sols)
{
  opw_kinematics::inverse(param, p, sols.data());
}

size_t countValidSolutions(const std::array<double, 6 * 8>& opw)
{
  std::size_t count = 0;
  for (int i = 0; i < 8; ++i)
  {
    bool is_valid = true;
    for (int j = 0; j < 6; ++j)
    {
      if (std::isnan(opw[i * 6 + j]))
      {
        is_valid = false;
        break;
      }
    }
    if (is_valid) count++;
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
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
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
    a -= 2.0 *M_PI;
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

static inline double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to-from);
}


bool findSolInSet(const std::vector<double>& s, const std::array<double, 6 * 8>& opw)
{
  for (int i = 0; i < 8; ++i)
  {
    bool is_same = true;
    for (int j = 0; j < 6; ++j)
    {
      double value = opw[i*6 + j];
//      if (value > M_PI) value -= 2*M_PI;
//      if (value < -M_PI) value += 2*M_PI;
      double diff = std::abs(shortest_angular_distance(s[j], value));
      if (diff > 1e-6)
      {
        is_same = false;
        break;
      }
    }
    if (is_same) return true;
  }
  return false;
}

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

void compare(ikfast::IkSolutionList<double>& ikf, std::array<double, 6 * 8>& opw)
{
  auto num_ikf_sols = ikf.GetNumSolutions();
  auto num_opw_sols = countValidSolutions(opw);

  bool equal_num = num_ikf_sols == num_opw_sols;
  EXPECT_TRUE(equal_num);

  if (!equal_num) return;

  // lets compare solutions
  std::vector<double> v (6);
  for (decltype(num_ikf_sols) i = 0; i < num_ikf_sols; ++i)
  {
    ikf.GetSolution(i).GetSolution(v.data(), nullptr);
    const bool found_sol_in_opw = findSolInSet(v, opw);
    EXPECT_TRUE(found_sol_in_opw);
    if (!found_sol_in_opw)
    {
      std::cout << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " "
                << v[5] << "\n";

      printResults(opw);
    }
  }

}

TEST(ikfast_to_opw, similar_solutions)
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
    std::array<double, 6 * 8> opw_sols;
    solveOPW(abb2400, p, opw_sols);

    // Compare
    compare(ikf_sols, opw_sols);
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
