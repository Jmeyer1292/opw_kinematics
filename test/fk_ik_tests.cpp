// Author: JeroenDM (Github)
// With minor edits by Jonathan Meyer

#include <array>
#include <gtest/gtest.h>
#include <random>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"
#include "opw_kinematics/opw_utilities.h"


// Structure used for setting sampler parameters based on floating point type
template <typename T>
struct TestTolerance;

template<>
struct TestTolerance<float> {
  static constexpr float TOLERANCE = 1e-4f;
};

template<>
struct TestTolerance<double> {
  static constexpr double TOLERANCE = 1e-10;
};

// Helper Functions for generating and testing random poses in cartesian/joint space
using opw_kinematics::Transform;

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
  static std::uniform_real_distribution<T> rg{ T(-M_PI), T(M_PI) };  // random generator

  q[0] = rg(en);
  q[1] = rg(en);
  q[2] = rg(en);
  q[3] = rg(en);
  q[4] = rg(en);
  q[5] = rg(en);
}

// The test itself
template <typename T>
void runRandomReachablePosesTest(const opw_kinematics::Parameters<T>& params)
{
  const static unsigned int number_of_tests = 1000;

  Transform<T> pose, forward_pose;
  T q_rand[6];
  std::array<T, 6 * 8> sols;

  for (unsigned j = 0; j < number_of_tests; ++j)
  {
    // create a reachable pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(params, q_rand);

    // Solve Inverse kinematics
    opw_kinematics::inverse(params, pose, sols.data());

    // check all valid solutions using forward kinematics
    for (unsigned i = 0; i < 8; ++i)
    {
      if (opw_kinematics::isValid(&sols[6 * i]))
      {
        // Forward kinematics of a solution should result in the same pose
        forward_pose = opw_kinematics::forward(params, &sols[6 * i]);
        comparePoses(forward_pose, pose, TestTolerance<T>::TOLERANCE);
      }
    }
  }
}

TEST(kuka_kr6, random_reachable_poses_double)
{
  const auto kukaKR6_R700_sixx = opw_kinematics::makeKukaKR6_R700_sixx<double>();
  runRandomReachablePosesTest(kukaKR6_R700_sixx);
}

TEST(kuka_kr6, random_reachable_poses_float)
{
  const auto kukaKR6_R700_sixx = opw_kinematics::makeKukaKR6_R700_sixx<float>();
  runRandomReachablePosesTest(kukaKR6_R700_sixx);
}

TEST(abb_2400, random_reachable_poses_double)
{
  const auto params = opw_kinematics::makeIrb2400_10<double>();
  runRandomReachablePosesTest(params);
}

TEST(abb_2400, random_reachable_poses_float)
{
  const auto params= opw_kinematics::makeIrb2400_10<float>();
  runRandomReachablePosesTest(params);
}

TEST(fanuc_r2000, random_reachable_poses_double)
{
  const auto params = opw_kinematics::makeFanucR2000iB_200R<double>();
  runRandomReachablePosesTest(params);
}

TEST(fanuc_r2000, random_reachable_poses_float)
{
  const auto params= opw_kinematics::makeFanucR2000iB_200R<float>();
  runRandomReachablePosesTest(params);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
