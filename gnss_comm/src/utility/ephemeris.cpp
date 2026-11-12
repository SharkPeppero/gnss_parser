/**
 * This file is part of gnss_comm.
 *
 * Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
 * Author: CAO Shaozu (shaozu.cao@gmail.com)
 *
 * gnss_comm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * gnss_comm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gnss_comm. If not, see <http://www.gnu.org/licenses/>.
 *
 * As many of the utility functions are adapted from RTKLIB,
 * the license for those part of code is claimed as follows:
 *
 * The RTKLIB software package is distributed under the following BSD 2-clause
 * license (http://opensource.org/licenses/BSD-2-Clause) and additional two
 * exclusive clauses. Users are permitted to develop, produce or sell their own
 * non-commercial or commercial products utilizing, linking or including RTKLIB as
 * long as they comply with the license.
 *
 *         Copyright (c) 2007-2020, T. Takasu, All rights reserved.
 */

#include "gnss_comm/constants/ephemeris.hpp"
#include "gnss_comm/constants/physical.hpp"
#include "gnss_comm/constants/system.hpp"
#include "gnss_comm/constants/time.hpp"
#include "gnss_comm/utility/ephemeris_utility.hpp"
#include "gnss_comm/utility/satellite_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <cmath>
#include <glog/logging.h>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
double Kepler(const double mk, const double es) {
  double e = mk;
  double ek = 1e6;
  size_t num_iter = 0;
  while (num_iter < MAX_ITER_KEPLER && fabs(e - ek) > EPSILON_KEPLER) {
    ek = e;
    e -= (e - es * sin(e) - mk) / (1.0 - es * cos(e));
    ++num_iter;
  }
  if (num_iter == MAX_ITER_KEPLER)
    LOG(WARNING) << "Kepler equation reached max number iteration";

  return e; // 返回本次更新后的偏近点角，而不是上一轮结果
}

double eph2svdt(const gtime_t &curr_time, const EphemPtr ephem_ptr) {
  double dt = time_diff(curr_time, ephem_ptr->toc);

  for (size_t i = 0; i < 2; ++i)
    dt -= ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;

  double satellite_clock_bias_s = ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;
  return satellite_clock_bias_s;
}

Eigen::Vector3d eph2pos(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *satellite_clock_bias_s) {
  Eigen::Vector3d sv_pos;
  double tk = time_diff(curr_time, ephem_ptr->toe);

  if (tk > WEEK_SECONDS / 2)
    tk -= WEEK_SECONDS;
  else if (tk < -WEEK_SECONDS / 2)
    tk += WEEK_SECONDS;

  // LOG(INFO) << "tk is " << std::setprecision(10) << tk;

  if (std::abs(tk) > EPH_VALID_SECONDS)
    LOG(WARNING) << "Ephemeris is not valid anymore";

  uint32_t prn;
  uint32_t sys = satsys(ephem_ptr->sat, &prn);
  double mu = MU, earth_omg = EARTH_OMG_GPS;
  switch (sys) {
  case SYS_GPS:
    mu = MU_GPS;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_QZS:
    mu = MU_GPS;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_GAL:
    mu = MU;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_GLO:
    mu = MU;
    earth_omg = EARTH_OMG_GLO;
    break;
  case SYS_BDS:
    mu = MU;
    earth_omg = EARTH_OMG_BDS;
    break;
  }

  // LOG(INFO) << "SYS: " << sys << ", prn: " << prn << ", curr_time is "
  //           << std::setprecision(10) << curr_time.time
  //           << ", and toe is " << ephem_ptr->toe.time;

  double n0 = sqrt(mu / pow(ephem_ptr->a, 3)); // mean motion angular velocity (rad/sec)
  double n = n0 + ephem_ptr->delta_n;          // corrected mean motion
  double Mk = ephem_ptr->m0 + n * tk;
  double Ek = Kepler(Mk, ephem_ptr->e); // solve Kepler equation for eccentric anomaly
  double sin_Ek = sin(Ek), cos_Ek = cos(Ek);
  double vk = atan2(sqrt(1 - ephem_ptr->e * ephem_ptr->e) * sin_Ek, cos_Ek - ephem_ptr->e);
  double phi = vk + ephem_ptr->omg;
  double cos_2phi = cos(2 * phi);
  double sin_2phi = sin(2 * phi);

  double delta_uk = ephem_ptr->cus * sin_2phi + ephem_ptr->cuc * cos_2phi; // latitude correction
  double delta_rk = ephem_ptr->crs * sin_2phi + ephem_ptr->crc * cos_2phi; // radius correction
  double delta_ik = ephem_ptr->cis * sin_2phi + ephem_ptr->cic * cos_2phi; // correction to inclination
  double uk = phi + delta_uk;                                              // corrected latitude
  double rk = ephem_ptr->a * (1 - ephem_ptr->e * cos_Ek) + delta_rk;       // corrected radius
  double ik = ephem_ptr->i0 + ephem_ptr->i_dot * tk + delta_ik;            // corrected inclination
  double sin_ik = sin(ik), cos_ik = cos(ik);

  double xk_prime = rk * cos(uk); // x position in orbital plane
  double yk_prime = rk * sin(uk); // y position in orbital plane

  double toe_tow = (sys == SYS_BDS ? time2bdt(time_add(ephem_ptr->toe, -BDT_GPST_OFFSET), NULL) : ephem_ptr->toe_tow);

  if (sys == SYS_BDS && IS_BDS_GEO(prn)) // BDS GEO satellite
  {
    double OMG_k = ephem_ptr->omg0 + ephem_ptr->omg_dot * tk - earth_omg * toe_tow;
    double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
    double xg = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
    double yg = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
    double zg = yk_prime * sin_ik;
    double sin_o = sin(earth_omg * tk), cos_o = cos(earth_omg * tk);
    sv_pos.x() = xg * cos_o + yg * sin_o * COS_N5 + zg * sin_o * SIN_N5;
    sv_pos.y() = -xg * sin_o + yg * cos_o * COS_N5 + zg * cos_o * SIN_N5;
    sv_pos.z() = -yg * SIN_N5 + zg * COS_N5;
  } else {
    double OMG_k = ephem_ptr->omg0 + (ephem_ptr->omg_dot - earth_omg) * tk - earth_omg * toe_tow;
    double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
    sv_pos.x() = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
    sv_pos.y() = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
    sv_pos.z() = yk_prime * sin_ik;
  }

  double dt = time_diff(curr_time, ephem_ptr->toc);
  double computed_satellite_clock_bias_s = ephem_ptr->af0 + ephem_ptr->af1 * dt + ephem_ptr->af2 * dt * dt;
  // relativity correction
  computed_satellite_clock_bias_s -= 2.0 * sqrt(mu * ephem_ptr->a) * ephem_ptr->e * sin_Ek / LIGHT_SPEED / LIGHT_SPEED;

  if (satellite_clock_bias_s)
    *satellite_clock_bias_s = computed_satellite_clock_bias_s;

  return sv_pos;
}

Eigen::Vector3d eph2vel(const gtime_t &curr_time, const EphemPtr ephem_ptr, double *satellite_clock_drift_sps) {
  Eigen::Vector3d sv_vel;
  sv_vel.setZero();
  double tk = time_diff(curr_time, ephem_ptr->toe);

  if (tk > WEEK_SECONDS / 2)
    tk -= WEEK_SECONDS;
  else if (tk < -WEEK_SECONDS / 2)
    tk += WEEK_SECONDS;

  // LOG(INFO) << "tk is " << std::setprecision(10) << tk;

  if (std::abs(tk) > EPH_VALID_SECONDS)
    LOG(WARNING) << "Ephemeris is not valid anymore";

  uint32_t prn;
  uint32_t sys = satsys(ephem_ptr->sat, &prn);
  double mu = MU, earth_omg = EARTH_OMG_GPS;
  switch (sys) {
  case SYS_GPS:
    mu = MU_GPS;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_QZS:
    mu = MU_GPS;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_GAL:
    mu = MU;
    earth_omg = EARTH_OMG_GPS;
    break;
  case SYS_GLO:
    mu = MU;
    earth_omg = EARTH_OMG_GLO;
    break;
  case SYS_BDS:
    mu = MU;
    earth_omg = EARTH_OMG_BDS;
    break;
  }
  double n0 = sqrt(mu / pow(ephem_ptr->a, 3)); // mean motion angular velocity (rad/sec)
  double n = n0 + ephem_ptr->delta_n;          // corrected mean motion
  double Mk = ephem_ptr->m0 + n * tk;
  double Ek = Kepler(Mk, ephem_ptr->e); // solve Kepler equation for eccentric anomaly
  double sin_Ek = sin(Ek), cos_Ek = cos(Ek);
  double Ek_dot = n / (1 - ephem_ptr->e * cos_Ek);
  double vk_dot = sqrt(1 - ephem_ptr->e * ephem_ptr->e) * Ek_dot / (1 - ephem_ptr->e * cos_Ek);
  double vk = atan2(sqrt(1 - ephem_ptr->e * ephem_ptr->e) * sin_Ek, cos_Ek - ephem_ptr->e);
  double phi = vk + ephem_ptr->omg;
  double cos_2phi = cos(2 * phi);
  double sin_2phi = sin(2 * phi);

  double delta_uk_dot = 2 * vk_dot * (ephem_ptr->cus * cos_2phi - ephem_ptr->cuc * sin_2phi);
  double delta_rk_dot = 2 * vk_dot * (ephem_ptr->crs * cos_2phi - ephem_ptr->crc * sin_2phi);
  double delta_ik_dot = 2 * vk_dot * (ephem_ptr->cis * cos_2phi - ephem_ptr->cic * sin_2phi);

  double uk_dot = vk_dot + delta_uk_dot;
  double rk_dot = ephem_ptr->a * ephem_ptr->e * Ek_dot * sin_Ek + delta_rk_dot;
  double ik_dot = ephem_ptr->i_dot + delta_ik_dot;

  double delta_uk = ephem_ptr->cus * sin_2phi + ephem_ptr->cuc * cos_2phi; // latitude correction
  double delta_rk = ephem_ptr->crs * sin_2phi + ephem_ptr->crc * cos_2phi; // radius correction
  double delta_ik = ephem_ptr->cis * sin_2phi + ephem_ptr->cic * cos_2phi; // correction to inclination
  double uk = phi + delta_uk;                                              // corrected latitude
  double rk = ephem_ptr->a * (1 - ephem_ptr->e * cos_Ek) + delta_rk;       // corrected radius
  double ik = ephem_ptr->i0 + ephem_ptr->i_dot * tk + delta_ik;            // corrected inclination
  double sin_ik = sin(ik), cos_ik = cos(ik);

  double sin_uk = sin(uk), cos_uk = cos(uk);
  double xk_prime = rk * cos(uk); // x position in orbital plane
  double yk_prime = rk * sin(uk); // y position in orbital plane
  double xk_prime_dot = rk_dot * cos_uk - rk * uk_dot * sin_uk;
  double yk_prime_dot = rk_dot * sin_uk + rk * uk_dot * cos_uk;

  double toe_tow = (sys == SYS_BDS ? time2bdt(time_add(ephem_ptr->toe, -BDT_GPST_OFFSET), NULL) : ephem_ptr->toe_tow);

  if (sys == SYS_BDS && IS_BDS_GEO(prn)) // BDS GEO satellite
  {
    double OMG_k = ephem_ptr->omg0 + ephem_ptr->omg_dot * tk - earth_omg * toe_tow;
    double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
    double OMGk_dot = ephem_ptr->omg_dot;
    double term1 = xk_prime_dot - yk_prime * OMGk_dot * cos_ik;
    double term2 = xk_prime * OMGk_dot + yk_prime_dot * cos_ik - yk_prime * ik_dot * sin_ik;
    double xg = xk_prime * cos_OMG_k - yk_prime * cos_ik * sin_OMG_k;
    double yg = xk_prime * sin_OMG_k + yk_prime * cos_ik * cos_OMG_k;
    double zg = yk_prime * sin_ik;
    double xg_dot = term1 * cos_OMG_k - term2 * sin_OMG_k;
    double yg_dot = term1 * sin_OMG_k + term2 * cos_OMG_k;
    double zg_dot = yk_prime_dot * sin_ik + yk_prime * ik_dot * cos_ik;
    double sin_o = sin(earth_omg * tk), cos_o = cos(earth_omg * tk);
    double sin_o_dot = earth_omg * cos_o, cos_o_dot = -earth_omg * sin_o;

    sv_vel.x() = xg_dot * cos_o + xg * cos_o_dot + yg_dot * sin_o * COS_N5 + yg * sin_o_dot * COS_N5 + zg_dot * sin_o * SIN_N5 + zg * sin_o_dot * SIN_N5;
    sv_vel.y() = -xg_dot * sin_o - xg * sin_o_dot + yg_dot * cos_o * COS_N5 + yg * cos_o_dot * COS_N5 + zg_dot * cos_o * SIN_N5 + zg * cos_o_dot * SIN_N5;
    sv_vel.z() = -yg_dot * SIN_N5 + zg_dot * COS_N5;
  } else {
    double OMG_k = ephem_ptr->omg0 + (ephem_ptr->omg_dot - earth_omg) * tk - earth_omg * toe_tow;
    double sin_OMG_k = sin(OMG_k), cos_OMG_k = cos(OMG_k);
    double OMGk_dot = ephem_ptr->omg_dot - earth_omg;
    double term1 = xk_prime_dot - yk_prime * OMGk_dot * cos_ik;
    double term2 = xk_prime * OMGk_dot + yk_prime_dot * cos_ik - yk_prime * ik_dot * sin_ik;
    sv_vel.x() = term1 * cos_OMG_k - term2 * sin_OMG_k;
    sv_vel.y() = term1 * sin_OMG_k + term2 * cos_OMG_k;
    sv_vel.z() = yk_prime_dot * sin_ik + yk_prime * ik_dot * cos_ik;
  }

  double dt = time_diff(curr_time, ephem_ptr->toc);
  double computed_satellite_clock_drift_sps = ephem_ptr->af1 + 2.0 * ephem_ptr->af2 * dt;
  // relativity correction
  computed_satellite_clock_drift_sps -= 2.0 * sqrt(mu * ephem_ptr->a) * ephem_ptr->e * cos_Ek * Ek_dot / LIGHT_SPEED / LIGHT_SPEED;

  if (satellite_clock_drift_sps)
    *satellite_clock_drift_sps = computed_satellite_clock_drift_sps;

  return sv_vel;
}

void deq(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc, Eigen::Vector3d &pos_dot, Eigen::Vector3d &vel_dot) {
  double r2 = pos.squaredNorm(), r3 = r2 * sqrt(r2);
  double omg2 = EARTH_OMG_GLO * EARTH_OMG_GLO;

  if (r2 <= 0.0) {
    pos_dot.setZero();
    vel_dot.setZero();
    return;
  }
  /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
  double a = 1.5 * J2_GLO * MU_GLO * EARTH_SEMI_MAJOR_GLO * EARTH_SEMI_MAJOR_GLO / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
  double b = 5.0 * pos.z() * pos.z() / r2;                                                  /* 5*z^2/r^2 */
  double c = -MU_GLO / r3 - a * (1.0 - b);                                                  /* -mu/r^3-a(1-b) */
  pos_dot.x() = vel.x();
  pos_dot.y() = vel.y();
  pos_dot.z() = vel.z();
  vel_dot.x() = (c + omg2) * pos.x() + 2.0 * EARTH_OMG_GLO * vel.y() + acc.x();
  vel_dot.y() = (c + omg2) * pos.y() - 2.0 * EARTH_OMG_GLO * vel.x() + acc.y();
  vel_dot.z() = (c - 2.0 * a) * pos.z() + acc.z();
}

void glo_orbit(double dt, Eigen::Vector3d &pos, Eigen::Vector3d &vel, const Eigen::Vector3d &acc) {
  Eigen::Vector3d pos_dot1, pos_dot2, pos_dot3, pos_dot4;
  Eigen::Vector3d vel_dot1, vel_dot2, vel_dot3, vel_dot4;
  Eigen::Vector3d next_pos, next_vel;

  deq(pos, vel, acc, pos_dot1, vel_dot1);
  next_pos = pos + 0.5 * pos_dot1 * dt;
  next_vel = vel + 0.5 * vel_dot1 * dt;
  deq(next_pos, next_vel, acc, pos_dot2, vel_dot2);
  next_pos = pos + 0.5 * pos_dot2 * dt;
  next_vel = vel + 0.5 * vel_dot2 * dt;
  deq(next_pos, next_vel, acc, pos_dot3, vel_dot3);
  next_pos = pos + pos_dot3 * dt;
  next_vel = vel + vel_dot3 * dt;
  deq(next_pos, next_vel, acc, pos_dot4, vel_dot4);
  pos += (pos_dot1 + 2.0 * pos_dot2 + 2.0 * pos_dot3 + pos_dot4) * dt / 6.0;
  vel += (vel_dot1 + 2.0 * vel_dot2 + 2.0 * vel_dot3 + vel_dot4) * dt / 6.0;
}

double geph2svdt(const gtime_t &curr_time, const GloEphemPtr geph_ptr) {
  double dt = time_diff(curr_time, geph_ptr->toe);
  if (dt > WEEK_SECONDS / 2)
    dt -= WEEK_SECONDS;
  else if (dt < -WEEK_SECONDS / 2)
    dt += WEEK_SECONDS;

  for (size_t i = 0; i < 2; ++i)
    dt -= -geph_ptr->tau_n + geph_ptr->gamma * dt;
  double satellite_clock_bias_s = -geph_ptr->tau_n + geph_ptr->gamma * dt;
  return satellite_clock_bias_s;
}

Eigen::Vector3d geph2pos(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *satellite_clock_bias_s) {
  Eigen::Vector3d sv_pos, sv_vel, sv_acc;
  sv_pos << geph_ptr->pos[0], geph_ptr->pos[1], geph_ptr->pos[2];
  sv_vel << geph_ptr->vel[0], geph_ptr->vel[1], geph_ptr->vel[2];
  sv_acc << geph_ptr->acc[0], geph_ptr->acc[1], geph_ptr->acc[2];

  double dt = time_diff(curr_time, geph_ptr->toe);
  if (std::abs(dt) > GLO_EPH_VALID_SECONDS)
    LOG(WARNING) << "GLONASS ephemeris is not valid anymore";
  double computed_satellite_clock_bias_s = -geph_ptr->tau_n + geph_ptr->gamma * dt;
  if (satellite_clock_bias_s)
    *satellite_clock_bias_s = computed_satellite_clock_bias_s;

  for (double tt = dt < 0.0 ? -TSTEP : TSTEP; fabs(dt) > 1e-9; dt -= tt) {
    if (fabs(dt) < TSTEP)
      tt = dt;
    glo_orbit(tt, sv_pos, sv_vel, sv_acc);
  }
  return sv_pos;
}

Eigen::Vector3d geph2vel(const gtime_t &curr_time, const GloEphemPtr geph_ptr, double *satellite_clock_drift_sps) {
  Eigen::Vector3d sv_pos, sv_vel, sv_acc;
  sv_pos << geph_ptr->pos[0], geph_ptr->pos[1], geph_ptr->pos[2];
  sv_vel << geph_ptr->vel[0], geph_ptr->vel[1], geph_ptr->vel[2];
  sv_acc << geph_ptr->acc[0], geph_ptr->acc[1], geph_ptr->acc[2];

  double dt = time_diff(curr_time, geph_ptr->toe);
  if (satellite_clock_drift_sps)
    *satellite_clock_drift_sps = geph_ptr->gamma;

  for (double tt = dt < 0.0 ? -TSTEP : TSTEP; fabs(dt) > 1e-9; dt -= tt) {
    if (fabs(dt) < TSTEP)
      tt = dt;
    glo_orbit(tt, sv_pos, sv_vel, sv_acc);
  }
  return sv_vel;
}

} // namespace gnss_comm
