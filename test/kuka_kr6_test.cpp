#include <array>
#include <gtest/gtest.h>
#include <random>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h" // for makeIrb2400_10<double>()
#include "opw_kinematics/opw_utilities.h"           // for optional checking

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Affine>;

template <typename T>
void comparePoses(const Transform<T> & Ta, const Transform<T> & Tb)
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      EXPECT_NEAR(Ra(i, j), Rb(i, j), 1e-6);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], 1e-6);
  EXPECT_NEAR(pa[1], pb[1], 1e-6);
  EXPECT_NEAR(pa[2], pb[2], 1e-6);
}

template <typename T>
void getRandomJointValues(T * q)
{
  static std::default_random_engine en{42}; // random engine
  static std::uniform_real_distribution<T> rg{-M_PI, M_PI}; // random generator

  q[0] = rg(en);
  q[1] = rg(en);
  q[2] = rg(en);
  q[3] = rg(en);
  q[4] = rg(en);
  q[5] = rg(en);
}

TEST(kuka_kr6, random_reachable_poses_double)
{
  const int number_of_tests = 10;
  const auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  Eigen::Affine3d pose;
  double q_rand[6];

  for (int j = 0; j < number_of_tests; ++j)
  {
    // create a reachable pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(abb2400, q_rand);

    // Solve Inverse kinematics
    std::array<double, 6 * 8> sols;
    opw_kinematics::inverse(abb2400, pose, sols.data());

    // check all valid solutions using forward kinematics
    for (int i = 0; i < 8; ++i)
    {
      if (opw_kinematics::isValid(&sols[6 * i]))
      {
        // Forward kinematics of a solution should result in the same pose
        Eigen::Affine3d forward_pose = opw_kinematics::forward(abb2400, &sols[6 * i]);
        comparePoses(forward_pose, pose);
      }
    }
  }
}

TEST(kuka_kr6, random_reachable_poses_float)
{
  const int number_of_tests = 10;
  const auto abb2400 = opw_kinematics::makeIrb2400_10<float>();

  Eigen::Affine3f pose;
  float q_rand[6];

  for (int j = 0; j < number_of_tests; ++j)
  {
    // create a pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(abb2400, q_rand);

    // Inverse kinematics
    std::array<float, 6 * 8> sols; // You could also use a std::vector or c-array of the appropriate size (6*8)
    opw_kinematics::inverse(abb2400, pose, sols.data());

    // check all valid solutions using forward kinematics
    for (int i = 0; i < 8; ++i)
    {
      if (opw_kinematics::isValid(&sols[6 * i]))
      {
        // Forward kinematics
        Eigen::Affine3f forward_pose = opw_kinematics::forward(abb2400, &sols[6 * i]);
        comparePoses(forward_pose, pose);
      }
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}