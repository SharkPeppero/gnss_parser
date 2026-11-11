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

#include "gnss_comm/constants/physical.hpp"
#include "gnss_comm/utility/coordinate_utility.hpp"
#include <cmath>
#include <glog/logging.h>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
Eigen::Vector3d geo2ecef(const Eigen::Vector3d &lla) {
  Eigen::Vector3d xyz;
  const double cos_lat = std::cos(lla(0) * D2R);
  const double sin_lat = std::sin(lla(0) * D2R);
  const double N = EARTH_SEMI_MAJOR / std::sqrt(1 - EARTH_ECCE_2 * sin_lat * sin_lat);
  xyz.x() = (N + lla(2)) * cos_lat * std::cos(lla(1) * D2R);
  xyz.y() = (N + lla(2)) * cos_lat * std::sin(lla(1) * D2R);
  xyz.z() = (N * (1 - EARTH_ECCE_2) + lla(2)) * sin_lat;
  return xyz;
}

Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz) {
  Eigen::Vector3d lla = Eigen::Vector3d::Zero();
  if (xyz.x() == 0 && xyz.y() == 0) {
    LOG(ERROR) << "LLA coordinate is not defined if x = 0 and y = 0";
    return lla;
  }

  double e2 = EARTH_ECCE_2;
  double a = EARTH_SEMI_MAJOR;
  double a2 = a * a;
  double b2 = a2 * (1 - e2);
  double b = sqrt(b2);
  double ep2 = (a2 - b2) / b2;
  double p = xyz.head<2>().norm();

  // two sides and hypotenuse of right angle triangle with one angle = theta:
  double s1 = xyz.z() * a;
  double s2 = p * b;
  double h = sqrt(s1 * s1 + s2 * s2);
  double sin_theta = s1 / h;
  double cos_theta = s2 / h;

  // two sides and hypotenuse of right angle triangle with one angle = lat:
  s1 = xyz.z() + ep2 * b * pow(sin_theta, 3);
  s2 = p - a * e2 * pow(cos_theta, 3);
  h = sqrt(s1 * s1 + s2 * s2);
  double tan_lat = s1 / s2;
  double sin_lat = s1 / h;
  double cos_lat = s2 / h;
  double lat = atan(tan_lat);
  double lat_deg = lat * R2D;

  double N = a2 * pow((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat), -0.5);
  double altM = p / cos_lat - N;

  double lon = atan2(xyz.y(), xyz.x());
  double lon_deg = lon * R2D;
  lla << lat_deg, lon_deg, altM;
  return lla;
}

Eigen::Vector3d ecef2enu(const Eigen::Vector3d &ref_lla, const Eigen::Vector3d &v_ecef) {
  double lat = ref_lla.x() * D2R, lon = ref_lla.y() * D2R;
  double sin_lat = sin(lat), cos_lat = cos(lat);
  double sin_lon = sin(lon), cos_lon = cos(lon);
  Eigen::Matrix3d R_enu_ecef;
  R_enu_ecef << -sin_lon, cos_lon, 0,                  //
      -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, //
      cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;   //
  return (R_enu_ecef * v_ecef);
}

Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo) {
  double lat = ref_geo.x() * D2R, lon = ref_geo.y() * D2R;
  double sin_lat = sin(lat), cos_lat = cos(lat);
  double sin_lon = sin(lon), cos_lon = cos(lon);
  Eigen::Matrix3d R_ecef_enu;
  R_ecef_enu << -sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon, //
      cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon,            //
      0, cos_lat, sin_lat;
  return R_ecef_enu;
}

Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef) { return geo2rotation(ecef2geo(ref_ecef)); }

void sat_azel(const Eigen::Vector3d &rev_pos, const Eigen::Vector3d &sat_pos, double *azel) {
  if (!azel)
    return;
  Eigen::Vector3d rev_lla = ecef2geo(rev_pos);
  Eigen::Vector3d rev2sat_ecef = (sat_pos - rev_pos).normalized();
  Eigen::Vector3d rev2sat_enu = ecef2enu(rev_lla, rev2sat_ecef);
  azel[0] = rev2sat_ecef.head<2>().norm() < 1e-12 ? 0.0 : atan2(rev2sat_enu.x(), rev2sat_enu.y());
  azel[0] += (azel[0] < 0 ? 2 * M_PI : 0);
  azel[1] = asin(rev2sat_enu.z());
}

} // namespace gnss_comm
