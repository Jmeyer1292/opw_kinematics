#include <opw_kinematics/opw_macros.h>
OPW_IGNORE_WARNINGS_PUSH
#include <gtest/gtest-message.h>    // for Message
#include <gtest/gtest-test-part.h>  // for TestPartResult
#include <gtest/gtest.h>            // IWYU pragma: keep
#include <Eigen/Core>               // IWYU pragma: keep
#include <vector>
#include <array>
OPW_IGNORE_WARNINGS_POP

#include "opw_kinematics/opw_kinematics.h"  // IWYU pragma: keep
#include "opw_kinematics/opw_parameters_examples.h"
#include "opw_kinematics/opw_utilities.h"

const double TOLERANCE = 1e-5;  // absolute tolerance for EXPECT_NEAR checks

using opw_kinematics::Solutions;
using opw_kinematics::Transform;

/** @brief Compare every element of two eigen Isometry3 poses.
 */
template <typename T>
void comparePoses(const Transform<T>& Ta, const Transform<T>& Tb)
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
  EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
  EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}

TEST(kuka_kr6, forward_kinematics)  // NOLINT
{
  const auto kuka = opw_kinematics::makeKukaKR6_R700_sixx<float>();

  std::array<float, 6> joint_values = { 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f };
  Transform<float> forward_pose = opw_kinematics::forward(kuka, joint_values);

  // Compare with copied results from forward kinematics using MoveIt!
  Transform<float> actual_pose;
  actual_pose.matrix() << -0.5965795f, 0.000371195f, 0.8025539f, 0.0f, -0.2724458f, 0.9405218f, -0.202958f, 0.0f,
      -0.7548948f, -0.3397331f, -0.5609949f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
  actual_pose.translation() << 0.7341169f, -0.1520347f, 0.182639f;

  comparePoses(forward_pose, actual_pose);
}

TEST(kuka_kr6, inverse_kinematics)  // NOLINT
{
  // this test assumes that the forward kinematics for joint values
  // {0.2, 0.2, 0.2, 0.2, 0.2, 0.2} are correct

  const auto kuka = opw_kinematics::makeKukaKR6_R700_sixx<float>();

  std::array<float, 6> joint_values = { 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f };
  Transform<float> forward_pose = opw_kinematics::forward(kuka, joint_values);

  Solutions<float> sols = opw_kinematics::inverse(kuka, forward_pose);

  Transform<float> pose;
  for (const auto& s : sols)
  {
    if (opw_kinematics::isValid(s))
    {
      // Forward kinematics of a solution should result in the same pose
      pose = opw_kinematics::forward(kuka, s);
      comparePoses(forward_pose, pose);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
