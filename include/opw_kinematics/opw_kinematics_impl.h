#ifndef OPW_KINEMATICS_IMPL_H
#define OPW_KINEMATICS_IMPL_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace opw_kinematics
{
template <typename T>
bool comparePoses(const Eigen::Transform<T, 3, Eigen::Isometry>& Ta,
                  const Eigen::Transform<T, 3, Eigen::Isometry>& Tb,
                  T tolerance)
{
  T translation_distance = (Ta.translation() - Tb.translation()).norm();
  T angular_distance = Eigen::Quaternion<T>(Ta.linear()).angularDistance(Eigen::Quaternion<T>(Tb.linear()));

  if (std::abs(translation_distance) > tolerance)
  {
    std::cout << "Translation Error: " << translation_distance << std::endl;
    return false;
  }

  if (std::abs(angular_distance) > tolerance)
  {
    std::cout << "Angular Error: " << angular_distance << std::endl;
    return false;
  }

  return true;
}

template <typename T>
Transform<T> forward(const Parameters<T>& p, const std::array<T, 6>& qs) noexcept
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  std::array<T, 6> q;
  q[0] = qs[0] * p.sign_corrections[0] - p.offsets[0];
  q[1] = qs[1] * p.sign_corrections[1] - p.offsets[1];
  q[2] = qs[2] * p.sign_corrections[2] - p.offsets[2];
  q[3] = qs[3] * p.sign_corrections[3] - p.offsets[3];
  q[4] = qs[4] * p.sign_corrections[4] - p.offsets[4];
  q[5] = qs[5] * p.sign_corrections[5] - p.offsets[5];

  T psi3 = std::atan2(p.a2, p.c3);
  T k = std::sqrt(p.a2 * p.a2 + p.c3 * p.c3);

  T cx1 = p.c2 * std::sin(q[1]) + k * std::sin(q[1] + q[2] + psi3) + p.a1;
  T cy1 = p.b;
  T cz1 = p.c2 * std::cos(q[1]) + k * std::cos(q[1] + q[2] + psi3);

  T cx0 = cx1 * std::cos(q[0]) - cy1 * std::sin(q[0]);
  T cy0 = cx1 * std::sin(q[0]) + cy1 * std::cos(q[0]);
  T cz0 = cz1 + p.c1;

  T s1 = std::sin(q[0]);
  T s2 = std::sin(q[1]);
  T s3 = std::sin(q[2]);
  T s4 = std::sin(q[3]);
  T s5 = std::sin(q[4]);
  T s6 = std::sin(q[5]);

  T c1 = std::cos(q[0]);
  T c2 = std::cos(q[1]);
  T c3 = std::cos(q[2]);
  T c4 = std::cos(q[3]);
  T c5 = std::cos(q[4]);
  T c6 = std::cos(q[5]);

  Matrix r_0c;
  r_0c(0, 0) = c1 * c2 * c3 - c1 * s2 * s3;
  r_0c(0, 1) = -s1;
  r_0c(0, 2) = c1 * c2 * s3 + c1 * s2 * c3;

  r_0c(1, 0) = s1 * c2 * c3 - s1 * s2 * s3;
  r_0c(1, 1) = c1;
  r_0c(1, 2) = s1 * c2 * s3 + s1 * s2 * c3;

  r_0c(2, 0) = -s2 * c3 - c2 * s3;
  r_0c(2, 1) = 0;
  r_0c(2, 2) = -s2 * s3 + c2 * c3;

  Matrix r_ce;
  r_ce(0, 0) = c4 * c5 * c6 - s4 * s6;
  r_ce(0, 1) = -c4 * c5 * s6 - s4 * c6;
  r_ce(0, 2) = c4 * s5;

  r_ce(1, 0) = s4 * c5 * c6 + c4 * s6;
  r_ce(1, 1) = -s4 * c5 * s6 + c4 * c6;
  r_ce(1, 2) = s4 * s5;

  r_ce(2, 0) = -s5 * c6;
  r_ce(2, 1) = s5 * s6;
  r_ce(2, 2) = c5;

  Matrix r_oe = r_0c * r_ce;

  // Note: do not use auto here, leads to lazy evaluation which
  // seems to be buggy on at least some setups and uses uninitialized data
  Vector u = Vector(cx0, cy0, cz0) + p.c4 * r_oe * Vector::UnitZ();

  Transform<T> i;
  i.setIdentity();
  i.translation() = u;
  i.linear() = r_oe;

  return i;
}

template <typename T>
Solutions<T> inverse(const Parameters<T>& params, const Transform<T>& pose) noexcept
{
  using Vector = Eigen::Matrix<T, 3, 1>;
  using Matrix3 = Eigen::Matrix<T, 3, 3>;

  // The container for solutions
  Solutions<T> solutions;

  // Adjust to wrist center
  const auto& matrix = pose.linear();
  Vector c = pose.translation() - params.c4 * pose.linear() * Vector::UnitZ();

  T nx1 = std::sqrt(c.x() * c.x() + c.y() * c.y() - params.b * params.b) - params.a1;

  // Compute theta1_i, theta1_ii
  T tmp1 = std::atan2(c.y(), c.x());
  T tmp2 = std::atan2(params.b, nx1 + params.a1);
  T theta1_i = tmp1 - tmp2;
  T theta1_ii = tmp1 + tmp2 - T(M_PI);

  // theta2 i through iv
  T tmp3 = (c.z() - params.c1);
  T s1_2 = nx1 * nx1 + tmp3 * tmp3;

  T tmp4 = nx1 + T(2.0) * params.a1;
  T s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
  T kappa_2 = params.a2 * params.a2 + params.c3 * params.c3;

  T c2_2 = params.c2 * params.c2;

  T tmp5 = s1_2 + c2_2 - kappa_2;

  T s1 = std::sqrt(s1_2);
  T s2 = std::sqrt(s2_2);

  T tmp13 = std::acos(tmp5 / (T(2.0) * s1 * params.c2));
  T tmp14 = std::atan2(nx1, c.z() - params.c1);
  T theta2_i = -tmp13 + tmp14;
  T theta2_ii = tmp13 + tmp14;

  T tmp6 = s2_2 + c2_2 - kappa_2;

  T tmp15 = std::acos(tmp6 / (T(2.0) * s2 * params.c2));
  T tmp16 = std::atan2(nx1 + T(2.0) * params.a1, c.z() - params.c1);
  T theta2_iii = -tmp15 - tmp16;
  T theta2_iv = tmp15 - tmp16;

  // theta3
  T tmp7 = s1_2 - c2_2 - kappa_2;
  T tmp8 = s2_2 - c2_2 - kappa_2;
  T tmp9 = T(2) * params.c2 * std::sqrt(kappa_2);
  T tmp10 = std::atan2(params.a2, params.c3);
  T tmp11 = std::acos(tmp7 / tmp9);

  T theta3_i = tmp11 - tmp10;
  T theta3_ii = -tmp11 - tmp10;

  T tmp12 = std::acos(tmp8 / tmp9);
  T theta3_iii = tmp12 - tmp10;
  T theta3_iv = -tmp12 - tmp10;

  // Now for the orientation part...
  std::array<T, 4> s23;
  std::array<T, 4> c23;
  std::array<T, 4> sin1;
  std::array<T, 4> cos1;

  sin1[0] = std::sin(theta1_i);
  sin1[1] = std::sin(theta1_i);
  sin1[2] = std::sin(theta1_ii);  // ???
  sin1[3] = std::sin(theta1_ii);

  cos1[0] = std::cos(theta1_i);
  cos1[1] = std::cos(theta1_i);
  cos1[2] = std::cos(theta1_ii);  // ???
  cos1[3] = std::cos(theta1_ii);

  s23[0] = std::sin(theta2_i + theta3_i);
  s23[1] = std::sin(theta2_ii + theta3_ii);
  s23[2] = std::sin(theta2_iii + theta3_iii);
  s23[3] = std::sin(theta2_iv + theta3_iv);

  c23[0] = std::cos(theta2_i + theta3_i);
  c23[1] = std::cos(theta2_ii + theta3_ii);
  c23[2] = std::cos(theta2_iii + theta3_iii);
  c23[3] = std::cos(theta2_iv + theta3_iv);

  std::array<T, 4> m;
  m[0] = matrix(0, 2) * s23[0] * cos1[0] + matrix(1, 2) * s23[0] * sin1[0] + matrix(2, 2) * c23[0];
  m[1] = matrix(0, 2) * s23[1] * cos1[1] + matrix(1, 2) * s23[1] * sin1[1] + matrix(2, 2) * c23[1];
  m[2] = matrix(0, 2) * s23[2] * cos1[2] + matrix(1, 2) * s23[2] * sin1[2] + matrix(2, 2) * c23[2];
  m[3] = matrix(0, 2) * s23[3] * cos1[3] + matrix(1, 2) * s23[3] * sin1[3] + matrix(2, 2) * c23[3];

  T theta5_i = std::atan2(std::sqrt(1 - m[0] * m[0]), m[0]);
  T theta5_ii = std::atan2(std::sqrt(1 - m[1] * m[1]), m[1]);
  T theta5_iii = std::atan2(std::sqrt(1 - m[2] * m[2]), m[2]);
  T theta5_iv = std::atan2(std::sqrt(1 - m[3] * m[3]), m[3]);

  T theta5_v = -theta5_i;
  T theta5_vi = -theta5_ii;
  T theta5_vii = -theta5_iii;
  T theta5_viii = -theta5_iv;

  // The derived equations in the paper are geometric and break down when joint 5 is equal to zero.
  // When joint 5 is equal to zeros the values passed to std::atan2 are both zero which is undefined.
  // This can result in significant rotation error up to PI between joint 4 and joint 6 each.
  // In the paper it defines an equation for Rc which is the rotation matrix of joint 5 relative to the base.
  // If you set joint 4 to zero and joint 5 is zero it results in the matrix below where the z-axis is the
  // same as the provided pose. Next, it takes the poses x-axis and projects it onto Rc and then leverage
  // std::atan2 to calculate the joint 6 angle.
  T zero_threshold = static_cast<T>(1e-6);
  T theta4_i, theta6_i;
  if (std::abs(theta5_i) < zero_threshold)
  {
    theta4_i = 0;
    Vector xe(matrix(0, 0), matrix(1, 0), matrix(2, 0));
    Matrix3 Rc;
    Rc.col(1) = Vector(-std::sin(theta1_i), std::cos(theta1_i), 0);  // yc
    Rc.col(2) = matrix.col(2);                                       // zc and ze are equal
    Rc.col(0) = Rc.col(1).cross(Rc.col(2));                          // xc
    Vector xec = Rc.transpose() * xe;
    theta6_i = std::atan2(xec(1), xec(0));
  }
  else
  {
    T theta4_iy = matrix(1, 2) * cos1[0] - matrix(0, 2) * sin1[0];
    T theta4_ix = matrix(0, 2) * c23[0] * cos1[0] + matrix(1, 2) * c23[0] * sin1[0] - matrix(2, 2) * s23[0];
    theta4_i = std::atan2(theta4_iy, theta4_ix);

    T theta6_iy = matrix(0, 1) * s23[0] * cos1[0] + matrix(1, 1) * s23[0] * sin1[0] + matrix(2, 1) * c23[0];
    T theta6_ix = -matrix(0, 0) * s23[0] * cos1[0] - matrix(1, 0) * s23[0] * sin1[0] - matrix(2, 0) * c23[0];
    theta6_i = std::atan2(theta6_iy, theta6_ix);
  }

  T theta4_ii, theta6_ii;
  if (std::abs(theta5_ii) < zero_threshold)
  {
    theta4_ii = 0;
    Vector xe(matrix(0, 0), matrix(1, 0), matrix(2, 0));
    Matrix3 Rc;
    Rc.col(1) = Vector(-std::sin(theta1_i), std::cos(theta1_i), 0);  // yc
    Rc.col(2) = matrix.col(2);                                       // zc and ze are equal
    Rc.col(0) = Rc.col(1).cross(Rc.col(2));                          // xc
    Vector xec = Rc.transpose() * xe;
    theta6_ii = std::atan2(xec(1), xec(0));
  }
  else
  {
    T theta4_iiy = matrix(1, 2) * cos1[1] - matrix(0, 2) * sin1[1];
    T theta4_iix = matrix(0, 2) * c23[1] * cos1[1] + matrix(1, 2) * c23[1] * sin1[1] - matrix(2, 2) * s23[1];
    theta4_ii = std::atan2(theta4_iiy, theta4_iix);

    T theta6_iiy = matrix(0, 1) * s23[1] * cos1[1] + matrix(1, 1) * s23[1] * sin1[1] + matrix(2, 1) * c23[1];
    T theta6_iix = -matrix(0, 0) * s23[1] * cos1[1] - matrix(1, 0) * s23[1] * sin1[1] - matrix(2, 0) * c23[1];
    theta6_ii = std::atan2(theta6_iiy, theta6_iix);
  }

  T theta4_iii, theta6_iii;
  if (std::abs(theta5_iii) < zero_threshold)
  {
    theta4_iii = 0;
    Vector xe(matrix(0, 0), matrix(1, 0), matrix(2, 0));
    Matrix3 Rc;
    Rc.col(1) = Vector(-std::sin(theta1_ii), std::cos(theta1_ii), 0);  // yc
    Rc.col(2) = matrix.col(2);                                         // zc and ze are equal
    Rc.col(0) = Rc.col(1).cross(Rc.col(2));                            // xc
    Vector xec = Rc.transpose() * xe;
    theta6_iii = std::atan2(xec(1), xec(0));
  }
  else
  {
    T theta4_iiiy = matrix(1, 2) * cos1[2] - matrix(0, 2) * sin1[2];
    T theta4_iiix = matrix(0, 2) * c23[2] * cos1[2] + matrix(1, 2) * c23[2] * sin1[2] - matrix(2, 2) * s23[2];
    theta4_iii = std::atan2(theta4_iiiy, theta4_iiix);

    T theta6_iiiy = matrix(0, 1) * s23[2] * cos1[2] + matrix(1, 1) * s23[2] * sin1[2] + matrix(2, 1) * c23[2];
    T theta6_iiix = -matrix(0, 0) * s23[2] * cos1[2] - matrix(1, 0) * s23[2] * sin1[2] - matrix(2, 0) * c23[2];
    theta6_iii = std::atan2(theta6_iiiy, theta6_iiix);
  }

  T theta4_iv, theta6_iv;
  if (std::abs(theta5_iv) < zero_threshold)
  {
    theta4_iv = 0;
    Vector xe(matrix(0, 0), matrix(1, 0), matrix(2, 0));
    Matrix3 Rc;
    Rc.col(1) = Vector(-std::sin(theta1_ii), std::cos(theta1_ii), 0);  // yc
    Rc.col(2) = matrix.col(2);                                         // zc and ze are equal
    Rc.col(0) = Rc.col(1).cross(Rc.col(2));                            // xc
    Vector xec = Rc.transpose() * xe;
    theta6_iv = std::atan2(xec(1), xec(0));
  }
  else
  {
    T theta4_ivy = matrix(1, 2) * cos1[3] - matrix(0, 2) * sin1[3];
    T theta4_ivx = matrix(0, 2) * c23[3] * cos1[3] + matrix(1, 2) * c23[3] * sin1[3] - matrix(2, 2) * s23[3];
    theta4_iv = std::atan2(theta4_ivy, theta4_ivx);

    T theta6_ivy = matrix(0, 1) * s23[3] * cos1[3] + matrix(1, 1) * s23[3] * sin1[3] + matrix(2, 1) * c23[3];
    T theta6_ivx = -matrix(0, 0) * s23[3] * cos1[3] - matrix(1, 0) * s23[3] * sin1[3] - matrix(2, 0) * c23[3];
    theta6_iv = std::atan2(theta6_ivy, theta6_ivx);
  }

  T theta4_v = theta4_i + T(M_PI);
  T theta4_vi = theta4_ii + T(M_PI);
  T theta4_vii = theta4_iii + T(M_PI);
  T theta4_viii = theta4_iv + T(M_PI);

  T theta6_v = theta6_i - T(M_PI);
  T theta6_vi = theta6_ii - T(M_PI);
  T theta6_vii = theta6_iii - T(M_PI);
  T theta6_viii = theta6_iv - T(M_PI);

  Eigen::Map<const Eigen::Array<T, 6, 1>> offsets(params.offsets.data());
  Eigen::Array<T, 6, 1> signs;
  signs[0] = params.sign_corrections[0];
  signs[1] = params.sign_corrections[1];
  signs[2] = params.sign_corrections[2];
  signs[3] = params.sign_corrections[3];
  signs[4] = params.sign_corrections[4];
  signs[5] = params.sign_corrections[5];

  Eigen::Array<T, 6, 8> theta;
  // clang-format off
  theta << theta1_i, theta1_i,  theta1_ii,  theta1_ii, theta1_i, theta1_i,  theta1_ii,  theta1_ii,
           theta2_i, theta2_ii, theta2_iii, theta2_iv, theta2_i, theta2_ii, theta2_iii, theta2_iv,
           theta3_i, theta3_ii, theta3_iii, theta3_iv, theta3_i, theta3_ii, theta3_iii, theta3_iv,
           theta4_i, theta4_ii, theta4_iii, theta4_iv, theta4_v, theta4_vi, theta4_vii, theta4_viii,
           theta5_i, theta5_ii, theta5_iii, theta5_iv, theta5_v, theta5_vi, theta5_vii, theta5_viii,
           theta6_i, theta6_ii, theta6_iii, theta6_iv, theta6_v, theta6_vi, theta6_vii, theta6_viii;
  // clang-format on

  // Perform the computation
  Eigen::Map<Eigen::Array<T, 6, 8>> output(solutions[0].data());
  output = (theta.colwise() + offsets).colwise() * signs;

  // If debug check
#ifndef NDEBUG
  Eigen::IOFormat heavy_fmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  for (std::size_t i = 0; i < solutions.size(); ++i)
  {
    if (std::isfinite(solutions[i][0]) && std::isfinite(solutions[i][1]) && std::isfinite(solutions[i][2]) &&
        std::isfinite(solutions[i][3]) && std::isfinite(solutions[i][4]) && std::isfinite(solutions[i][5]))
    {
      Transform<T> check_pose = forward<T>(params, solutions[i]);
      if (!comparePoses<T>(pose, check_pose, T(1e-3)))
      {
        double t{ 0 };
        if (i == 0)
          t = theta5_i;
        else if (i == 1)
          t = theta5_ii;
        else if (i == 2)
          t = theta5_iii;
        else if (i == 3)
          t = theta5_iv;
        else if (i == 4)
          t = theta5_v;
        else if (i == 5)
          t = theta5_vi;
        else if (i == 6)
          t = theta5_vii;
        else if (i == 7)
          t = theta5_viii;

        std::cout << "*********************************" << std::endl
                  << "********** Pose Failure *********" << std::endl
                  << "*********************************" << std::endl
                  << "Theta 5 value: " << t << std::endl
                  << "Determinant: " << pose.matrix().determinant() << std::endl
                  << "Pose:" << std::endl
                  << pose.matrix().format(heavy_fmt) << std::endl;
      }
    }
  }
#endif

  return solutions;
}
}  // namespace opw_kinematics
#endif
