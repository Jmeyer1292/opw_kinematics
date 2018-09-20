#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"
#include "opw_kinematics/opw_utilities.h"

const double TOLERANCE = 1e-5; // absolute tolerance for EXPECT_NEAR checks

using opw_kinematics::Transform;

/** @brief Compare every element of two eigen Isometry3 poses.
 */
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
      EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
  EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
  EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}

TEST(kuka_kr6, forward_kinematics)
{  
  const auto kuka = opw_kinematics::makeKukaKR6_R700_sixx<float>();

  std::vector<float> joint_values = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
  Transform<float> forward_pose = opw_kinematics::forward(kuka, &joint_values[0]);

  // Compare with copied results from forward kinematics using MoveIt!
  Transform<float> actual_pose;
  actual_pose.matrix()  << -0.5965795, 0.000371195,   0.8025539, 0,
     -0.2724458,   0.9405218,   -0.202958, 0,
     -0.7548948,  -0.3397331,  -0.5609949, 0,
      0        ,   0        ,   0        , 1;
  actual_pose.translation() << 0.7341169, -0.1520347, 0.182639;

  comparePoses(forward_pose, actual_pose);
}

TEST(kuka_kr6, inverse_kinematics)
{
  // this test assumes that the forward kinematics for joint values
  // {0.2, 0.2, 0.2, 0.2, 0.2, 0.2} are correct

  const auto kuka = opw_kinematics::makeKukaKR6_R700_sixx<float>();

  std::vector<float> joint_values = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
  Transform<float> forward_pose = opw_kinematics::forward(kuka, &joint_values[0]);

  std::array<float, 6 * 8> sols;
  opw_kinematics::inverse(kuka, forward_pose, sols.data());

  Transform<float> pose;
  for (int i = 0; i < 8; ++i)
  {
    if (opw_kinematics::isValid(&sols[6 * i]))
    {
      // Forward kinematics of a solution should result in the same pose
      pose = opw_kinematics::forward(kuka, &sols[6 * i]);
      comparePoses(forward_pose, pose);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
