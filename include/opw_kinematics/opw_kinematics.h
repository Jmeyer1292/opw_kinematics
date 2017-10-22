#ifndef OPW_KINEMATICS_H
#define OPW_KINEMATICS_H

#include <Eigen/Dense>
#include "opw_kinematics/opw_parameters.h"

namespace opw_kinematics
{


/**
 * @brief inverse
 * @param pose
 * @param params
 * @param out
 */
template <typename T>
void inverse(const Eigen::Affine3d& pose, const Parameters<T>& params, T* out);


/**
 * @brief forward
 * @param p
 * @param q
 * @return
 */
template <typename T>
Eigen::Affine3d forward(const Parameters<T>& p, const T* q);




#include "opw_kinematics/opw_kinematics_impl.h"

} // end namespace opw_kinematics

#endif // OPW_KINEMATICS_H

