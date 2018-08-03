#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_parameters_examples.h"

const double TOLERANCE = 1e-6; // absolute tolerance for EXPECT_NEAR checks

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Affine>;

/** @brief Compare every element of two eigen affine3 poses.
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

  std::vector<float> joint_values = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  Eigen::Affine3f forward_pose = opw_kinematics::forward(kuka, &joint_values[0]);

  // Compare with copied results from forward kinematics using MoveIt!
  Eigen::Affine3f actual_pose;
  actual_pose.matrix()  << -0.5965795, 0.000371195,   0.8025539, 0,
     -0.2724458,   0.9405218,   -0.202958, 0,
     -0.7548948,  -0.3397331,  -0.5609949, 0,
      0        ,   0        ,   0        , 1;
  actual_pose.translation() << 0.7341169, -0.1520347, 0.182639;

  comparePoses(forward_pose, actual_pose);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}