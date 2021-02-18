#ifndef OPW_KINEMATICS_H
#define OPW_KINEMATICS_H

#include <array>                            // IWYU pragma: keep
#include <Eigen/Dense>                      // IWYU pragma: keep
#include "opw_kinematics/opw_parameters.h"  // IWYU pragma: export

namespace opw_kinematics
{
/**
 * Typedef equivalent to Eigen::Isometry3d for T = double and Eigen::Isometry3f for
 * T = float.
 */
template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Isometry>;

/**
 * Typedef equivalent to std::array<std::array<double, 6>, 8> for T = double and std::array<std::array<float, 6>, 8> for
 * T = float.
 */
template <typename T>
using Solutions = std::array<std::array<T, 6>, 8>;

/**
 * @brief Computes up to 8 kinematically unique joint solutions that put the tool
 *        flange of the robot described by @e params at the pose given by @e pose.
 *        Results are stored in @e out, a 6x8 array of T. See out's description
 *        for more details.
 * @param pose The pose of the tool flange for which you want joint solutions for
 * @param params The robot for which you want to solve.
 * @return A array of 8 to an array of 6 elements.
 *         ALL 8 SOLUTIONS ARE ALWAYS WRITTEN, EVEN IF THEY CONTAIN NANS.
 *         You must check in a subsequent call if you have a solution. The plus
 *         side is that the first solution should be from the same
 *         configuraton each time.
 */
template <typename T>
Solutions<T> inverse(const Parameters<T>& params, const Transform<T>& pose) noexcept;

/**
 * @brief Computes the tool pose of the robot described by @e when said robot
 *        has the joint positions given by @e qs, a 6 element array of type T.
 * @param p The robot for which you want to solve forward kinematics.
 * @param qs The joint pose of the robot which you want to know the tool pose of.
 * @return The flange pose.
 */
template <typename T>
Transform<T> forward(const Parameters<T>& p, const std::array<T, 6>& qs) noexcept;

#include "opw_kinematics/opw_kinematics_impl.h"  // IWYU pragma: export

}  // end namespace opw_kinematics

#endif  // OPW_KINEMATICS_H
