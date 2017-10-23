# Intro
A simple, analytical inverse kinematic library for industrial robots with parallel bases and
spherical wrists. Based on the paper `An Analytical Solution of the Inverse Kinematics Problem
of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist` by
Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.

This package is still under heavy development (23 October 2017) and I encourage you to check back soon.

# Purpose
This package is meant to provide a simpler alternative to IK-Fast based solutions in situations
where one has an industrial robot with a parallel base and spherical wrist. This configuration
is extremely common in industrial robots.

The kinematics are parameterized by 7 primary values taken directly from the robot's spec sheet
and a set of joint-zero offsets. Given this structure, no other setup is required.

# Parameters

TBD

a1, a2, b, c1, c2, c3, c4

See the paper at the moment. Description to follow.

# Example

```c++

#include "opw_kinematics/opw_kinematics.h"
#include <array>

int main()
{
  const auto abb2400 = opw_kinematics::makeIrb2400_10<double>();

  // Inverse kinematics
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() = Eigen::Vector3d(1.3, 0.2, 0);

  // Up to 8 solutions exist
  // Currently NaN indicates a solution did not exist
  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(abb2400, pose, sols.data());

  // Forward kinematics
  Eigen::Affine3d forward_pose = opw_kinematics::forward(abb2400, &sols[6 * 0]);

  return 0;
}

```

