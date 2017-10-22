#ifndef OPW_KINEMATICS_IMPL_H
#define OPW_KINEMATICS_IMPL_H

template <typename T>
void inverse(const Eigen::Affine3d& pose, const Parameters<T>& params, T* out)
{
  // Adjust to wrist center
  Eigen::Vector3d c = pose.translation() - params.c4 * pose.linear() * Eigen::Vector3d::UnitZ();
  const auto& matrix = pose.matrix();

  double nx1 = std::sqrt(c.x() * c.x() + c.y() * c.y() - params.b * params.b) - params.a1;

  // Compute theta1_i, theta1_ii
  double tmp1 = atan2(c.y(), c.x());
  double tmp2 = atan2(params.b, nx1 + params.a1);
  double theta1_i = tmp1 - tmp2;
  double theta1_ii = tmp1 + tmp2 - M_PI;

  // theta2 i through iv
  double tmp3 = (c.z() - params.c1);
  double s1_2 = nx1 * nx1 + tmp3 * tmp3;

  double tmp4 = nx1 + 2.0 * params.a1;
  double s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
  double kappa_2 = params.a2 * params.a2 + params.c3 * params.c3;

  double c2_2 = params.c2 * params.c2;

  double tmp5 = s1_2 + c2_2 - kappa_2;

  double s1 = std::sqrt(s1_2);
  double s2 = std::sqrt(s2_2);
  double theta2_i = -std::acos(tmp5 / (2.0 * s1 * params.c2)) +
                    atan2(nx1, c.z() - params.c1);
  double theta2_ii = std::acos(tmp5 / (2.0 * s1 * params.c2)) +
                     atan2(nx1, c.z() - params.c1);

  double tmp6 = s2_2 + c2_2 - kappa_2;

  double theta2_iii = -std::acos(tmp6 / (2.0 * s2 * params.c2)) - atan2(nx1 + 2.0 * params.a1, c.z() - params.c1);
  double theta2_iv = std::acos(tmp6 / (2.0 * s2 * params.c2)) - atan2(nx1 + 2.0 * params.a1, c.z() - params.c1);

  // theta3
  double tmp7 = s1_2 - c2_2 - kappa_2;
  double tmp8 = s2_2 - c2_2 - kappa_2;
  double tmp9 = 2 * params.c2 * std::sqrt(kappa_2);
  double theta3_i = std::acos(tmp7 / tmp9) - atan2(params.a2, params.c3);

  double theta3i = acos( (s1_2 - c2_2 - kappa_2) / (2*params.c2*std::sqrt(kappa_2)) ) - atan2(params.a2, params.c3);


  double theta3_ii = -std::acos(tmp7 / tmp9) - atan2(params.a2, params.c3);

  double theta3_iii = std::acos(tmp8 / tmp9) - atan2(params.a2, params.c3);
  double theta3_iv = -std::acos(tmp8 / tmp9) - atan2(params.a2, params.c3);

  // Now for the orientation part...
  double s23[4];
  double c23[4];
  double sin1[4];
  double cos1[4];

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

  double m[4];
  m[0] = matrix(0,2) * s23[0] * cos1[0] + matrix(1,2) * s23[0] * sin1[0] + matrix(2,2) * c23[0];
  m[1] = matrix(0,2) * s23[1] * cos1[1] + matrix(1,2) * s23[1] * sin1[1] + matrix(2,2) * c23[1];
  m[2] = matrix(0,2) * s23[2] * cos1[2] + matrix(1,2) * s23[2] * sin1[2] + matrix(2,2) * c23[2];
  m[3] = matrix(0,2) * s23[3] * cos1[3] + matrix(1,2) * s23[3] * sin1[3] + matrix(2,2) * c23[3];

  double theta4_i = atan2(matrix(1,2) * cos1[0] - matrix(0,2) * sin1[0],
                   matrix(0,2) * c23[0] * cos1[0] + matrix(1,2) * c23[0] * sin1[0] - matrix(2,2) * s23[0]);

  double theta4_ii = atan2(matrix(1,2) * cos1[1] - matrix(0,2) * sin1[1],
                   matrix(0,2) * c23[1] * cos1[1] + matrix(1,2) * c23[1] * sin1[1] - matrix(2,2) * s23[1]);

  double theta4_iii = atan2(matrix(1,2) * cos1[2] - matrix(0,2) * sin1[2],
                   matrix(0,2) * c23[2] * cos1[2] + matrix(1,2) * c23[2] * sin1[2] - matrix(2,2) * s23[2]);

  double theta4_iv = atan2(matrix(1,2) * cos1[3] - matrix(0,2) * sin1[3],
                   matrix(0,2) * c23[3] * cos1[3] + matrix(1,2) * c23[3] * sin1[3] - matrix(2,2) * s23[3]);

  double theta4_v = theta4_i + M_PI;
  double theta4_vi = theta4_ii + M_PI;
  double theta4_vii = theta4_iii + M_PI;
  double theta4_viii = theta4_iv + M_PI;

  double theta5_i = atan2(sqrt(1 - m[0] * m[0]), m[0]);
  double theta5_ii = atan2(sqrt(1 - m[1] * m[1]), m[1]);
  double theta5_iii = atan2(sqrt(1 - m[2] * m[2]), m[2]);
  double theta5_iv = atan2(sqrt(1 - m[3] * m[3]), m[3]);

  double theta5_v = -theta5_i;
  double theta5_vi = -theta5_ii;
  double theta5_vii = -theta5_iii;
  double theta5_viii = -theta5_iv;

  double theta6_i = atan2(matrix(0,1) * s23[0] * cos1[0] + matrix(1,1) * s23[0] * sin1[0] + matrix(2,1) * c23[0],
                          -matrix(0,0) * s23[0] * cos1[0] - matrix(1, 0) * s23[0] * sin1[0] - matrix(2,0) * c23[0]);

  double theta6_ii = atan2(matrix(0,1) * s23[1] * cos1[1] + matrix(1,1) * s23[1] * sin1[1] + matrix(2,1) * c23[1],
                          -matrix(0,0) * s23[1] * cos1[1] - matrix(1, 0) * s23[1] * sin1[1] - matrix(2,0) * c23[1]);

  double theta6_iii = atan2(matrix(0,1) * s23[2] * cos1[2] + matrix(1,1) * s23[2] * sin1[2] + matrix(2,1) * c23[2],
                          -matrix(0,0) * s23[2] * cos1[2] - matrix(1, 0) * s23[2] * sin1[2] - matrix(2,0) * c23[2]);

  double theta6_iv = atan2(matrix(0,1) * s23[3] * cos1[3] + matrix(1,1) * s23[3] * sin1[3] + matrix(2,1) * c23[3],
                          -matrix(0,0) * s23[3] * cos1[3] - matrix(1, 0) * s23[3] * sin1[3] - matrix(2,0) * c23[3]);

  double theta6_v = theta6_i - M_PI;
  double theta6_vi = theta6_ii - M_PI;
  double theta6_vii = theta6_iii - M_PI;
  double theta6_viii = theta6_iv - M_PI;

  out[6 * 0 + 0] = theta1_i;
  out[6 * 0 + 1] = theta2_i;
  out[6 * 0 + 2] = theta3_i - 0;
  out[6 * 0 + 3] = theta4_i;
  out[6 * 0 + 4] = theta5_i;
  out[6 * 0 + 5] = theta6_i;

  out[6 * 1 + 0] = theta1_i;
  out[6 * 1 + 1] = theta2_ii;
  out[6 * 1 + 2] = theta3_ii  - 0;
  out[6 * 1 + 3] = theta4_ii;
  out[6 * 1 + 4] = theta5_ii;
  out[6 * 1 + 5] = theta6_ii;

  out[6 * 2 + 0] = theta1_ii;
  out[6 * 2 + 1] = theta2_iii;
  out[6 * 2 + 2] = theta3_iii - 0;
  out[6 * 2 + 3] = theta4_iii;
  out[6 * 2 + 4] = theta5_iii;
  out[6 * 2 + 5] = theta6_iii;

  out[6 * 3 + 0] = theta1_ii;
  out[6 * 3 + 1] = theta2_iv;
  out[6 * 3 + 2] = theta3_iv - 0;
  out[6 * 3 + 3] = theta4_iv;
  out[6 * 3 + 4] = theta5_iv;
  out[6 * 3 + 5] = theta6_iv;

  out[6 * 4 + 0] = theta1_i;
  out[6 * 4 + 1] = theta2_i;
  out[6 * 4 + 2] = theta3_i - 0;
  out[6 * 4 + 3] = theta4_v;
  out[6 * 4 + 4] = theta5_v;
  out[6 * 4 + 5] = theta6_v;

  out[6 * 5 + 0] = theta1_i;
  out[6 * 5 + 1] = theta2_ii;
  out[6 * 5 + 2] = theta3_ii - 0;
  out[6 * 5 + 3] = theta4_vi;
  out[6 * 5 + 4] = theta5_vi;
  out[6 * 5 + 5] = theta6_vi;

  out[6 * 6 + 0] = theta1_ii;
  out[6 * 6 + 1] = theta2_iii;
  out[6 * 6 + 2] = theta3_iii - 0;
  out[6 * 6 + 3] = theta4_vii;
  out[6 * 6 + 4] = theta5_vii;
  out[6 * 6 + 5] = theta6_vii;

  out[6 * 7 + 0] = theta1_ii;
  out[6 * 7 + 1] = theta2_iv;
  out[6 * 7 + 2] = theta3_iv - 0;
  out[6 * 7 + 3] = theta4_viii;
  out[6 * 7 + 4] = theta5_viii;
  out[6 * 7 + 5] = theta6_viii;
  return;
}

template <typename T>
Eigen::Affine3d forward(const Parameters<T>& p, const T* q)
{
  Eigen::Affine3d i;
  auto& m = i.matrix();

  double psi3 = atan2(p.a2, p.c3);
  double k = sqrt(p.a2 * p.a2 + p.c3 * p.c3);

  double cx1 = p.c2 * sin(q[1]) + k * sin(q[1] + (q[2] + double(0)) + psi3) + p.a1;
  double cy1 = p.b;
  double cz1 = p.c2 * cos(q[1]) + k * cos(q[1] + (q[2] + double(0)) + psi3);

  double cx0 = cx1 * cos(q[0]) - cy1 * sin(q[0]);
  double cy0 = cx1 * sin(q[0]) + cy1 * cos(q[0]);
  double cz0 = cz1 + p.c1;

  double s1 = sin(q[0]);
  double s2 = sin(q[1]);
  double s3 = sin((q[2] + double(0)));
  double s4 = sin(q[3]);
  double s5 = sin(q[4]);
  double s6 = sin(q[5]);

  double c1 = cos(q[0]);
  double c2 = cos(q[1]);
  double c3 = cos((q[2] + double(0)));
  double c4 = cos(q[3]);
  double c5 = cos(q[4]);
  double c6 = cos(q[5]);

  Eigen::Matrix3d r_0c;
  r_0c(0,0) = c1 * c2 * c3- c1 * s2 * s3;
  r_0c(0,1) = -s1;
  r_0c(0,2) = c1*c2*s3+c1*s2*c3;

  r_0c(1,0) = s1*c2*c3-s1*s2*s3;
  r_0c(1,1) = c1;
  r_0c(1,2) = s1*c2*s3+s1*s2*c3;

  r_0c(2,0) = -s2*c3-c2*s3;
  r_0c(2,1) = 0;
  r_0c(2,2) = -s2*s3+c2*c3;

  Eigen::Matrix3d r_ce;
  r_ce(0,0) = c4*c5*c6-s4*s6;
  r_ce(0,1) = -c4*c5*s6-s4*c6;
  r_ce(0,2) = c4 * s5;

  r_ce(1,0) = s4*c5*c6+c4*s6;
  r_ce(1,1) = -s4*c5*s6+c4*c6;
  r_ce(1,2) = s4*s5;

  r_ce(2,0) = -s5*c6;
  r_ce(2,1) =  s5*s6 ;
  r_ce(2,2) = c5;

  Eigen::Matrix3d r_oe = r_0c * r_ce;

  auto u = Eigen::Vector3d(cx0, cy0, cz0) + p.c4 * r_oe * Eigen::Vector3d::UnitZ();

  i.translation() = u;
  i.linear() = r_oe;

  return i;
}

#endif
