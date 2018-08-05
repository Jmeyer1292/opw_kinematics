// Author: Jeroen De Maeyer (JeroenDM @ Github)
// With edits by Jonathan Meyer

#include <array>
#include <chrono>
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
  static constexpr float TOLERANCE = 5e-4f;
  static constexpr const char* const NAME = "float";
};

template<>
struct TestTolerance<double> {
  static constexpr double TOLERANCE = 1e-10;
  static constexpr const char* const NAME = "double";
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

// Generate random joint pose -> Solve FK -> Solve IK -> Verify that new FK matches
// the original.
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

// Generate random joint poses, time how long it takes to solve FK for all of them
// Then time how long it takes to solve IK for all of them
template <typename T>
void runThroughputTests(const opw_kinematics::Parameters<T>& params)
{
  const static unsigned int number_of_tests = 10000;

  std::vector<Transform<T>, Eigen::aligned_allocator<Transform<T>>> poses;
  poses.resize(number_of_tests);

  {
    // Generate N random joint poses
    std::vector<std::array<T, 6>> random_joint_values;
    random_joint_values.resize(number_of_tests);

    for (std::size_t i = 0; i < number_of_tests; ++i)
    {
      getRandomJointValues(random_joint_values[i].data());
    }

    // Time the solving of FK
    const auto fk_start_tm = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < number_of_tests; ++i)
    {
      poses[i] = opw_kinematics::forward(params, random_joint_values[i].data());
    }
    const auto fk_end_tm = std::chrono::steady_clock::now();

    // Report FK timing
    const auto fk_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(fk_end_tm - fk_start_tm).count();
    std::cout << "Forward Kinematics " << TestTolerance<T>::NAME << " generated " << number_of_tests
              << " poses in " << fk_dt_us << " us\n";
    std::cout << "Average us per fk solve: " << static_cast<double>(fk_dt_us) / number_of_tests << "\n";
  }

  std::array<T, 6*8> ik_sol_space;
  const auto ik_start_tm = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < number_of_tests; ++i)
  {
    opw_kinematics::inverse(params, poses[i], ik_sol_space.data());
  }
  const auto ik_end_tm = std::chrono::steady_clock::now();
  // Report FK timing
  const auto ik_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(ik_end_tm - ik_start_tm).count();
  std::cout << "Inverse Kinematics " << TestTolerance<T>::NAME << " generated " << number_of_tests
            << " poses in " << ik_dt_us << " us\n";
  std::cout << "Average us per ik solve: " << static_cast<double>(ik_dt_us) / number_of_tests << "\n";
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

TEST(kuka_kr6, throughput_tests_float)
{
  const auto params = opw_kinematics::makeKukaKR6_R700_sixx<float>();
  runThroughputTests(params);
}

TEST(kuka_kr6, throughput_tests_double)
{
  const auto params = opw_kinematics::makeKukaKR6_R700_sixx<double>();
  runThroughputTests(params);
}

TEST(abb_2400, throughput_tests_float)
{
  const auto params = opw_kinematics::makeIrb2400_10<float>();
  runThroughputTests(params);
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

TEST(abb_2400, throughput_tests_double)
{
  const auto params = opw_kinematics::makeIrb2400_10<double>();
  runThroughputTests(params);
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

TEST(fanuc_r2000, throughput_tests_float)
{
  const auto params = opw_kinematics::makeFanucR2000iB_200R<float>();
  runThroughputTests(params);
}

TEST(fanuc_r2000, throughput_tests_double)
{
  const auto params = opw_kinematics::makeFanucR2000iB_200R<double>();
  runThroughputTests(params);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
