#ifndef OPW_KINEMATICS_IMPL_H
#define OPW_KINEMATICS_IMPL_H

template <typename T>
void inverse(const Parameters<T>& params, const Transform<T>& pose, T* out) noexcept
{
  using Vector = Eigen::Matrix<T, 3, 1>;

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
  T theta2_i = -std::acos(tmp5 / (T(2.0) * s1 * params.c2)) +
                    std::atan2(nx1, c.z() - params.c1);
  T theta2_ii = std::acos(tmp5 / (T(2.0) * s1 * params.c2)) +
                     std::atan2(nx1, c.z() - params.c1);

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
  T s23[4];
  T c23[4];
  T sin1[4];
  T cos1[4];

  sin1[0] = sin(theta1_i);
  sin1[1] = sin(theta1_i);
  sin1[2] = sin(theta1_ii); // ???
  sin1[3] = sin(theta1_ii);

  cos1[0] = cos(theta1_i);
  cos1[1] = cos(theta1_i);
  cos1[2] = cos(theta1_ii); // ???
  cos1[3] = cos(theta1_ii);

  s23[0] = sin(theta2_i + theta3_i);
  s23[1] = sin(theta2_ii + theta3_ii);
  s23[2] = sin(theta2_iii + theta3_iii);
  s23[3] = sin(theta2_iv + theta3_iv);

  c23[0] = cos(theta2_i + theta3_i);
  c23[1] = cos(theta2_ii + theta3_ii);
  c23[2] = cos(theta2_iii + theta3_iii);
  c23[3] = cos(theta2_iv + theta3_iv);

  T m[4];
  m[0] = matrix(0,2) * s23[0] * cos1[0] + matrix(1,2) * s23[0] * sin1[0] + matrix(2,2) * c23[0];
  m[1] = matrix(0,2) * s23[1] * cos1[1] + matrix(1,2) * s23[1] * sin1[1] + matrix(2,2) * c23[1];
  m[2] = matrix(0,2) * s23[2] * cos1[2] + matrix(1,2) * s23[2] * sin1[2] + matrix(2,2) * c23[2];
  m[3] = matrix(0,2) * s23[3] * cos1[3] + matrix(1,2) * s23[3] * sin1[3] + matrix(2,2) * c23[3];

  T theta4_i = std::atan2(matrix(1,2) * cos1[0] - matrix(0,2) * sin1[0],
                   matrix(0,2) * c23[0] * cos1[0] + matrix(1,2) * c23[0] * sin1[0] - matrix(2,2) * s23[0]);

  T theta4_ii = std::atan2(matrix(1,2) * cos1[1] - matrix(0,2) * sin1[1],
                   matrix(0,2) * c23[1] * cos1[1] + matrix(1,2) * c23[1] * sin1[1] - matrix(2,2) * s23[1]);

  T theta4_iii = std::atan2(matrix(1,2) * cos1[2] - matrix(0,2) * sin1[2],
                   matrix(0,2) * c23[2] * cos1[2] + matrix(1,2) * c23[2] * sin1[2] - matrix(2,2) * s23[2]);

  T theta4_iv = std::atan2(matrix(1,2) * cos1[3] - matrix(0,2) * sin1[3],
                   matrix(0,2) * c23[3] * cos1[3] + matrix(1,2) * c23[3] * sin1[3] - matrix(2,2) * s23[3]);

  T theta4_v = theta4_i + T(M_PI);
  T theta4_vi = theta4_ii + T(M_PI);
  T theta4_vii = theta4_iii + T(M_PI);
  T theta4_viii = theta4_iv + T(M_PI);

  T theta5_i = std::atan2(sqrt(1 - m[0] * m[0]), m[0]);
  T theta5_ii = std::atan2(sqrt(1 - m[1] * m[1]), m[1]);
  T theta5_iii = std::atan2(sqrt(1 - m[2] * m[2]), m[2]);
  T theta5_iv = std::atan2(sqrt(1 - m[3] * m[3]), m[3]);

  T theta5_v = -theta5_i;
  T theta5_vi = -theta5_ii;
  T theta5_vii = -theta5_iii;
  T theta5_viii = -theta5_iv;

  T theta6_i = std::atan2(matrix(0,1) * s23[0] * cos1[0] + matrix(1,1) * s23[0] * sin1[0] + matrix(2,1) * c23[0],
                          -matrix(0,0) * s23[0] * cos1[0] - matrix(1, 0) * s23[0] * sin1[0] - matrix(2,0) * c23[0]);

  T theta6_ii = std::atan2(matrix(0,1) * s23[1] * cos1[1] + matrix(1,1) * s23[1] * sin1[1] + matrix(2,1) * c23[1],
                          -matrix(0,0) * s23[1] * cos1[1] - matrix(1, 0) * s23[1] * sin1[1] - matrix(2,0) * c23[1]);

  T theta6_iii = std::atan2(matrix(0,1) * s23[2] * cos1[2] + matrix(1,1) * s23[2] * sin1[2] + matrix(2,1) * c23[2],
                          -matrix(0,0) * s23[2] * cos1[2] - matrix(1, 0) * s23[2] * sin1[2] - matrix(2,0) * c23[2]);

  T theta6_iv = std::atan2(matrix(0,1) * s23[3] * cos1[3] + matrix(1,1) * s23[3] * sin1[3] + matrix(2,1) * c23[3],
                          -matrix(0,0) * s23[3] * cos1[3] - matrix(1, 0) * s23[3] * sin1[3] - matrix(2,0) * c23[3]);

  T theta6_v = theta6_i - T(M_PI);
  T theta6_vi = theta6_ii - T(M_PI);
  T theta6_vii = theta6_iii - T(M_PI);
  T theta6_viii = theta6_iv - T(M_PI);

  out[6 * 0 + 0] = (theta1_i + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 0 + 1] = (theta2_i + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 0 + 2] = (theta3_i + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 0 + 3] = (theta4_i + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 0 + 4] = (theta5_i + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 0 + 5] = (theta6_i + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 1 + 0] = (theta1_i  + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 1 + 1] = (theta2_ii + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 1 + 2] = (theta3_ii + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 1 + 3] = (theta4_ii + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 1 + 4] = (theta5_ii + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 1 + 5] = (theta6_ii + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 2 + 0] = (theta1_ii  + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 2 + 1] = (theta2_iii + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 2 + 2] = (theta3_iii + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 2 + 3] = (theta4_iii + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 2 + 4] = (theta5_iii + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 2 + 5] = (theta6_iii + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 3 + 0] = (theta1_ii + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 3 + 1] = (theta2_iv + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 3 + 2] = (theta3_iv + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 3 + 3] = (theta4_iv + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 3 + 4] = (theta5_iv + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 3 + 5] = (theta6_iv + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 4 + 0] = (theta1_i + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 4 + 1] = (theta2_i + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 4 + 2] = (theta3_i + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 4 + 3] = (theta4_v + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 4 + 4] = (theta5_v + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 4 + 5] = (theta6_v + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 5 + 0] = (theta1_i  + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 5 + 1] = (theta2_ii + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 5 + 2] = (theta3_ii + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 5 + 3] = (theta4_vi + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 5 + 4] = (theta5_vi + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 5 + 5] = (theta6_vi + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 6 + 0] = (theta1_ii  + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 6 + 1] = (theta2_iii + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 6 + 2] = (theta3_iii + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 6 + 3] = (theta4_vii + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 6 + 4] = (theta5_vii + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 6 + 5] = (theta6_vii + params.offsets[5]) * params.sign_corrections[5];

  out[6 * 7 + 0] = (theta1_ii   + params.offsets[0]) * params.sign_corrections[0];
  out[6 * 7 + 1] = (theta2_iv   + params.offsets[1]) * params.sign_corrections[1];
  out[6 * 7 + 2] = (theta3_iv   + params.offsets[2]) * params.sign_corrections[2];
  out[6 * 7 + 3] = (theta4_viii + params.offsets[3]) * params.sign_corrections[3];
  out[6 * 7 + 4] = (theta5_viii + params.offsets[4]) * params.sign_corrections[4];
  out[6 * 7 + 5] = (theta6_viii + params.offsets[5]) * params.sign_corrections[5];
}

template <typename T>
Transform<T> forward(const Parameters<T>& p, const T* qs) noexcept
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  T q[6];
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
  r_0c(0,0) = c1 * c2 * c3- c1 * s2 * s3;
  r_0c(0,1) = -s1;
  r_0c(0,2) = c1*c2*s3+c1*s2*c3;

  r_0c(1,0) = s1*c2*c3-s1*s2*s3;
  r_0c(1,1) = c1;
  r_0c(1,2) = s1*c2*s3+s1*s2*c3;

  r_0c(2,0) = -s2*c3-c2*s3;
  r_0c(2,1) = 0;
  r_0c(2,2) = -s2*s3+c2*c3;

  Matrix r_ce;
  r_ce(0,0) = c4*c5*c6-s4*s6;
  r_ce(0,1) = -c4*c5*s6-s4*c6;
  r_ce(0,2) = c4 * s5;

  r_ce(1,0) = s4*c5*c6+c4*s6;
  r_ce(1,1) = -s4*c5*s6+c4*c6;
  r_ce(1,2) = s4*s5;

  r_ce(2,0) = -s5*c6;
  r_ce(2,1) =  s5*s6 ;
  r_ce(2,2) = c5;

  Matrix r_oe = r_0c * r_ce;

  //Note: do not use auto here, leads to lazy evalutation which
  // seems to be buggy on at least some setups and uses uninitialized data
  Vector u = Vector(cx0, cy0, cz0) + p.c4 * r_oe * Vector::UnitZ();

  Transform<T> i;
  i.translation() = u;
  i.linear() = r_oe;

  return i;
}

#endif
