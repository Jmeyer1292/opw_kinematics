#ifndef OPW_KINEMATICS_H
#define OPW_KINEMATICS_H

#include <Eigen/Dense>
#include "opw_kinematics/opw_parameters.h"

namespace opw_kinematics
{

/**
 * Typedef equivalent to Eigen::Isometry3d for T = double and Eigen::Isometry3f for
 * T = float.
 */
template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Isometry>;

/**
 * @brief Computes up to 8 kinematically unique joint solutions that put the tool
 *        flange of the robot described by @e params at the pose given by @e pose.
 *        Results are stored in @e out, a 6x8 array of T. See out's description
 *        for more details.
 * @param pose The pose of the tool flange for which you want joint solutions for
 * @param params The robot for which you want to solve.
 * @param out A pointer to an array of 48 elements (6x8). Values 0-5 will have the
 *            joint values of the first solution, 6-11 will have the second and so
 *            on. ALL 8 SOLUTIONS ARE ALWAYS WRITTEN, EVEN IF THEY CONTAIN NANS.
 *            You must check in a subsequent call if you have a solution. The plus
 *            side is that the first solution should be from the same
 *            configuraton each time.
 */
template <typename T>
void inverse(const Parameters<T>& params, const Transform<T>& pose, T* out) noexcept;

/**
 * @brief Computes the tool pose of the robot described by @e when said robot
 *        has the joint positions given by @e qs, a 6 element array of type T.
 * @param p The robot for which you want to solve forward kinematics.
 * @param qs The joint pose of the robot which you want to know the tool pose of.
 * @return The flange pose.
 */
template <typename T>
Transform<T> forward(const Parameters<T>& p, const T* qs) noexcept;

#include "opw_kinematics/opw_kinematics_impl.h"

} // end namespace opw_kinematics

#endif // OPW_KINEMATICS_H
