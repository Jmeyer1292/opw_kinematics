#ifndef OPW_KINEMATICS_IMPL_H
#define OPW_KINEMATICS_IMPL_H

#include <cmath>
#include <Eigen/Dense>

template <typename T>
Solutions<T> inverse(const Parameters<T>& params, const Transform<T>& pose) noexcept
{
  using Vector = Eigen::Matrix<T, 3, 1>;

  // The container for solutions
  Solutions<T> solutions;

  // Adjust to wrist center
  Vector c = pose.translation() - params.c4 * pose.linear() * Vector::UnitZ();
  const auto& matrix = pose.matrix();

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
  T theta2_i = -std::acos(tmp5 / (T(2.0) * s1 * params.c2)) + std::atan2(nx1, c.z() - params.c1);
  T theta2_ii = std::acos(tmp5 / (T(2.0) * s1 * params.c2)) + std::atan2(nx1, c.z() - params.c1);

  T tmp6 = s2_2 + c2_2 - kappa_2;

  T theta2_iii = -std::acos(tmp6 / (T(2.0) * s2 * params.c2)) - std::atan2(nx1 + T(2.0) * params.a1, c.z() - params.c1);
  T theta2_iv = std::acos(tmp6 / (T(2.0) * s2 * params.c2)) - std::atan2(nx1 + T(2.0) * params.a1, c.z() - params.c1);

  // theta3
  T tmp7 = s1_2 - c2_2 - kappa_2;
  T tmp8 = s2_2 - c2_2 - kappa_2;
  T tmp9 = T(2) * params.c2 * std::sqrt(kappa_2);
  T theta3_i = std::acos(tmp7 / tmp9) - std::atan2(params.a2, params.c3);

  T theta3_ii = -std::acos(tmp7 / tmp9) - std::atan2(params.a2, params.c3);

  T theta3_iii = std::acos(tmp8 / tmp9) - std::atan2(params.a2, params.c3);
  T theta3_iv = -std::acos(tmp8 / tmp9) - std::atan2(params.a2, params.c3);

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

  T theta4_i = std::atan2(matrix(1, 2) * cos1[0] - matrix(0, 2) * sin1[0],
                          matrix(0, 2) * c23[0] * cos1[0] + matrix(1, 2) * c23[0] * sin1[0] - matrix(2, 2) * s23[0]);

  T theta4_ii = std::atan2(matrix(1, 2) * cos1[1] - matrix(0, 2) * sin1[1],
                           matrix(0, 2) * c23[1] * cos1[1] + matrix(1, 2) * c23[1] * sin1[1] - matrix(2, 2) * s23[1]);

  T theta4_iii = std::atan2(matrix(1, 2) * cos1[2] - matrix(0, 2) * sin1[2],
                            matrix(0, 2) * c23[2] * cos1[2] + matrix(1, 2) * c23[2] * sin1[2] - matrix(2, 2) * s23[2]);

  T theta4_iv = std::atan2(matrix(1, 2) * cos1[3] - matrix(0, 2) * sin1[3],
                           matrix(0, 2) * c23[3] * cos1[3] + matrix(1, 2) * c23[3] * sin1[3] - matrix(2, 2) * s23[3]);

  T theta4_v = theta4_i + T(M_PI);
  T theta4_vi = theta4_ii + T(M_PI);
  T theta4_vii = theta4_iii + T(M_PI);
  T theta4_viii = theta4_iv + T(M_PI);

  T theta5_i = std::atan2(std::sqrt(1 - m[0] * m[0]), m[0]);
  T theta5_ii = std::atan2(std::sqrt(1 - m[1] * m[1]), m[1]);
  T theta5_iii = std::atan2(std::sqrt(1 - m[2] * m[2]), m[2]);
  T theta5_iv = std::atan2(std::sqrt(1 - m[3] * m[3]), m[3]);

  T theta5_v = -theta5_i;
  T theta5_vi = -theta5_ii;
  T theta5_vii = -theta5_iii;
  T theta5_viii = -theta5_iv;

  T theta6_i = std::atan2(matrix(0, 1) * s23[0] * cos1[0] + matrix(1, 1) * s23[0] * sin1[0] + matrix(2, 1) * c23[0],
                          -matrix(0, 0) * s23[0] * cos1[0] - matrix(1, 0) * s23[0] * sin1[0] - matrix(2, 0) * c23[0]);

  T theta6_ii = std::atan2(matrix(0, 1) * s23[1] * cos1[1] + matrix(1, 1) * s23[1] * sin1[1] + matrix(2, 1) * c23[1],
                           -matrix(0, 0) * s23[1] * cos1[1] - matrix(1, 0) * s23[1] * sin1[1] - matrix(2, 0) * c23[1]);

  T theta6_iii = std::atan2(matrix(0, 1) * s23[2] * cos1[2] + matrix(1, 1) * s23[2] * sin1[2] + matrix(2, 1) * c23[2],
                            -matrix(0, 0) * s23[2] * cos1[2] - matrix(1, 0) * s23[2] * sin1[2] - matrix(2, 0) * c23[2]);

  T theta6_iv = std::atan2(matrix(0, 1) * s23[3] * cos1[3] + matrix(1, 1) * s23[3] * sin1[3] + matrix(2, 1) * c23[3],
                           -matrix(0, 0) * s23[3] * cos1[3] - matrix(1, 0) * s23[3] * sin1[3] - matrix(2, 0) * c23[3]);

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

  return solutions;
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

  // Note: do not use auto here, leads to lazy evalutation which
  // seems to be buggy on at least some setups and uses uninitialized data
  Vector u = Vector(cx0, cy0, cz0) + p.c4 * r_oe * Vector::UnitZ();

  Transform<T> i;
  i.setIdentity();
  i.translation() = u;
  i.linear() = r_oe;

  return i;
}

#endif
