#include <array>
#include <gtest/gtest.h>
#include <random>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"  // for makeIrb2400_10<double>()
#include "opw_kinematics/opw_utilities.h"            // for optional checking

const double TOLERANCE_DOUBLE = 1e-10;
const float TOLERANCE_FLOAT = 1e-5;

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Affine>;

template <typename T>
void comparePoses(const Transform<T>& Ta, const Transform<T>& Tb, double tolerance)
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      EXPECT_NEAR(Ra(i, j), Rb(i, j), tolerance);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], tolerance);
  EXPECT_NEAR(pa[1], pb[1], tolerance);
  EXPECT_NEAR(pa[2], pb[2], tolerance);
}

template <typename T>
void getRandomJointValues(T* q)
{
  static std::default_random_engine en{ 42 };                  // random engine
  static std::uniform_real_distribution<T> rg{ -M_PI, M_PI };  // random generator

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
  const auto kukaKR6_R700_sixx = opw_kinematics::makeKukaKR6_R700_sixx<double>();

  Eigen::Affine3d pose, forward_pose;
  double q_rand[6];
  std::array<double, 6 * 8> sols;

  for (int j = 0; j < number_of_tests; ++j)
  {
    // create a reachable pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(kukaKR6_R700_sixx, q_rand);

    // Solve Inverse kinematics
    opw_kinematics::inverse(kukaKR6_R700_sixx, pose, sols.data());

    // check all valid solutions using forward kinematics
    for (int i = 0; i < 8; ++i)
    {
      if (opw_kinematics::isValid(&sols[6 * i]))
      {
        // Forward kinematics of a solution should result in the same pose
        forward_pose = opw_kinematics::forward(kukaKR6_R700_sixx, &sols[6 * i]);
        comparePoses(forward_pose, pose, TOLERANCE_DOUBLE);
      }
    }
  }
}

TEST(kuka_kr6, random_reachable_poses_float)
{
  const int number_of_tests = 10;
  const auto kukaKR6_R700_sixx = opw_kinematics::makeKukaKR6_R700_sixx<float>();

  Eigen::Affine3f pose, forward_pose;
  float q_rand[6];
  std::array<float, 6 * 8> sols;

  for (int j = 0; j < number_of_tests; ++j)
  {
    // create a pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(kukaKR6_R700_sixx, q_rand);

    // Inverse kinematics
    opw_kinematics::inverse(kukaKR6_R700_sixx, pose, sols.data());

    // check all valid solutions using forward kinematics
    for (int i = 0; i < 8; ++i)
    {
      if (opw_kinematics::isValid(&sols[6 * i]))
      {
        // Forward kinematics
        forward_pose = opw_kinematics::forward(kukaKR6_R700_sixx, &sols[6 * i]);
        comparePoses(forward_pose, pose, TOLERANCE_FLOAT);
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}