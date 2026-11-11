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
#include "gnss_comm/utility/atmosphere_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <cmath>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
    static double interpc(const double coef[], double lat)
    {
        int i=(int)(lat/15.0);
        if (i<1) return coef[0]; else if (i>4) return coef[4];
        return coef[i-1]*(1.0-lat/15.0+i)+coef[i]*(lat/15.0-i);
    }

    /* elevation in radius */
    static double mapf(double el, double a, double b, double c)
    {
        double sinel=sin(el);
        return (1.0+a/(1.0+b/(1.0+c)))/(sinel+(a/(sinel+b/(sinel+c))));
    }

    /* calculate troposphere hydrostatic/wet mapping functions  ------------------------------
    * calculate troposphere hydrostatic/wet mapping functions by Niell model
    * ref: Global mapping functions for the atmosphere delay at radio wavelengths
    * args   : gtime_t time                 I   time (gpst)
    * args   : Eigen::Vector3d rev_lla      I   receiver position in geodetic coordinate
    *          double*         azel         I   satellite azimuth/elevation angle in radius
    *          double*         mapfh        IO  hydrostatic mapping coefficient
    * return : delay expressed by light travel distance (m)
    *-----------------------------------------------------------------------------*/
    static void nmf(gtime_t time, const Eigen::Vector3d rev_lla, const double azel[],
                    double *mapfh, double *mapfw)
    {
        /* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
        const double coef[][5]={
            { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
            { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
            { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

            { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
            { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
            { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

            { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
            { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
            { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
        };
        const double aht[]={ 2.53E-5, 5.49E-3, 1.14E-3}; /* height correction */

        double ah[3],aw[3],dm,el=azel[1],lat=rev_lla.x(),hgt=rev_lla.z();
        int i;

        if (el<=0.0) {
            if (mapfw) *mapfw=0.0;
            if (mapfh) *mapfh=0.0;
            return;
        }
        /* year from doy 28, added half a year for southern latitudes */
        double y=(time2doy(time)-28.0)/365.25+(lat<0.0?0.5:0.0);

        double cosy=cos(2.0*M_PI*y);
        lat=fabs(lat);

        for (i=0;i<3;i++) {
            ah[i]=interpc(coef[i  ],lat)-interpc(coef[i+3],lat)*cosy;
            aw[i]=interpc(coef[i+6],lat);
        }
        /* ellipsoidal height is used instead of height above sea level */
        dm=(1.0/sin(el)-mapf(el,aht[0],aht[1],aht[2]))*hgt/1E3;

        if (mapfw) *mapfw=mapf(el,aw[0],aw[1],aw[2]);
        if (mapfh) *mapfh=mapf(el,ah[0],ah[1],ah[2])+dm;
    }

    double calculate_trop_delay(gtime_t time, const Eigen::Vector3d &rev_lla, const double *azel)
    {
        // compute tropospheric delay by standard atmosphere and saastamoinen model
        const double temp0=15.0; /* temparature at sea level */
        const double humi =0.7;  /* relative humidity */

        if (rev_lla.z()<-100.0||1E4<rev_lla.z()||azel[1]<=0) return 0.0;

        /* standard atmosphere */
        double hgt=rev_lla.z()<0.0?0.0:rev_lla.z();

        double pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
        double temp=temp0-6.5E-3*hgt+273.16;
        double e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));

        /* saastamoninen model */
        double zhd=0.0022768*pres/(1.0-0.00266*cos(2.0*rev_lla.x()*D2R)-0.00028*hgt/1E3);
        double zwd=0.002277*(1255.0/temp+0.05)*e;

        double mapfh = 0.0, mapfw = 0.0;
        nmf(time, rev_lla, azel, &mapfh, &mapfw);
        return (mapfh*zhd + mapfw*zwd);
    }

    double calculate_ion_delay(gtime_t t, const std::vector<double> &ion_parameters,
                               const Eigen::Vector3d &rev_lla, const double *azel)
    {
        if (ion_parameters.empty())         return 0.0;
        if (rev_lla.z()<-1E3||azel[1]<=0)   return 0.0;

        /* earth centered angle (semi-circle) */
        double psi=0.0137/(azel[1]/M_PI+0.11)-0.022;

        /* subionospheric latitude/longitude (semi-circle) */
        double phi=rev_lla.x()/180.0+psi*cos(azel[0]);
        if      (phi> 0.416) phi= 0.416;
        else if (phi<-0.416) phi=-0.416;
        double lam=rev_lla.y()/180.0+psi*sin(azel[0])/cos(phi*M_PI);

        /* geomagnetic latitude (semi-circle) */
        phi+=0.064*cos((lam-1.617)*M_PI);

        /* local time (s) */
        uint32_t week = 0;
        double tt=43200.0*lam+time2gpst(t,&week);
        tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */

        /* slant factor */
        double f=1.0+16.0*pow(0.53-azel[1]/M_PI,3.0);

        /* ionospheric delay */
        double amp=ion_parameters[0]+phi*(ion_parameters[1]+phi*(ion_parameters[2]+phi*ion_parameters[3]));
        double per=ion_parameters[4]+phi*(ion_parameters[5]+phi*(ion_parameters[6]+phi*ion_parameters[7]));
        amp=amp<    0.0?    0.0:amp;
        per=per<72000.0?72000.0:per;
        double x=2.0*M_PI*(tt-50400.0)/per;

        return LIGHT_SPEED*f*(fabs(x)<1.57?5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)):5E-9);
    }


} // namespace gnss_comm
