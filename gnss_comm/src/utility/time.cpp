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

#include "gnss_comm/constants/time.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <cmath>
#include <cstddef>
#include <glog/logging.h>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
    const static double gpst0[] = {1980,1,6,0,0,0}; /* gps time reference */
    const static double gst0 [] = {1999,8,22,0,0,0}; /* galileo system time reference */
    const static double bdt0 [] = {2006,1,1,0,0,0}; /* beidou time reference */
    gtime_t epoch2time(const double *ep)
    {
        const int doy[] = {1,32,60,91,121,152,182,213,244,274,305,335};
        gtime_t time = {0};
        int days, sec, year=(int)ep[0], mon=(int)ep[1], day=(int)ep[2];

        if (year < 1970 || year > 2099 || mon < 1 || mon > 12) return time;

        /* leap year if year%4==0 in 1901-2099 */
        days = (year-1970)*365 + (year-1969)/4 + doy[mon-1] + day-2 + (year%4==0&&mon>=3?1:0);
        sec = (int)floor(ep[5]);
        time.time = (time_t)days*86400 + (int)ep[3]*3600 + (int)ep[4]*60 + sec;
        time.sec = ep[5] - sec;
        return time;
    }
    /* time to calendar day/time ---------------------------------------------------
    * convert gtime_t struct to calendar day/time
    * args   : gtime_t t        I   gtime_t struct
    *          double *ep       O   day/time {year,month,day,hour,min,sec}
    * return : none
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    void time2epoch(gtime_t t, double *ep)
    {
        const int mday[] = { /* # of days in a month */
            31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
            31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
        };
        int days, sec, mon, day;

        /* leap year if year%4==0 in 1901-2099 */
        days = (int)(t.time / 86400);
        sec = (int)(t.time - (time_t)days*86400);
        for (day=days%1461, mon=0; mon<48; mon++) {
            if (day >= mday[mon]) day -= mday[mon]; else break;
        }
        ep[0] = 1970 + days/1461*4 + mon/12;
        ep[1] = mon%12 + 1;
        ep[2] = day + 1;
        ep[3] = sec / 3600;
        ep[4] = sec%3600 / 60;
        ep[5] = sec%60 + t.sec;
    }

    /* gps time to time ------------------------------------------------------------
    * convert week and tow in gps time to gtime_t struct
    * args   : uint32_t    week      I   week number in gps time
    *          double      tow       I   time of week in gps time (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2time(uint32_t week, double tow)
    {
        gtime_t t = epoch2time(gpst0);
        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to gps time ------------------------------------------------------------
    * convert gtime_t struct to week and tow in gps time
    * args   : gtime_t      t        I   gtime_t struct
    *          uint32_t    *week     IO  week number in gps time (NULL: no output)
    * return : time of week in gps time (s)
    *-----------------------------------------------------------------------------*/
    double time2gpst(gtime_t t, uint32_t *week)
    {
        gtime_t t0 = epoch2time(gpst0);
        time_t sec = t.time - t0.time;
        uint32_t w = (uint32_t)(sec / (86400*7));

        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }
    /* galileo system time to time -------------------------------------------------
    * convert week and tow in galileo system time (gst) to gtime_t struct
    * args   : int    week      I   week number in gst
    *          double tow       I   time of week in gst (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t gst2time(int week, double tow)
    {
        gtime_t t = epoch2time(gst0);

        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to galileo system time -------------------------------------------------
    * convert gtime_t struct to week and tow in galileo system time (gst)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in gst (NULL: no output)
    * return : time of week in gst (s)
    *-----------------------------------------------------------------------------*/
    double time2gst(gtime_t t, int *week)
    {
        gtime_t t0 = epoch2time(gst0);
        time_t sec = t.time-t0.time;
        int w = (int)(sec / (86400*7));

        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }
    /* beidou time (bdt) to time ---------------------------------------------------
    * convert week and tow in beidou time (bdt) to gtime_t struct
    * args   : int    week      I   week number in bdt
    *          double tow       I   time of week in bdt (s)
    * return : gtime_t struct
    *-----------------------------------------------------------------------------*/
    gtime_t bdt2time(int week, double tow)
    {
        gtime_t t = epoch2time(bdt0);

        if (tow < -1E9 || tow > 1E9) tow = 0.0;
        t.time += 86400*7*week + (int)tow;
        t.sec = tow - (int)tow;
        return t;
    }
    /* time to beidouo time (bdt) --------------------------------------------------
    * convert gtime_t struct to week and tow in beidou time (bdt)
    * args   : gtime_t t        I   gtime_t struct
    *          int    *week     IO  week number in bdt (NULL: no output)
    * return : time of week in bdt (s)
    *-----------------------------------------------------------------------------*/
    double time2bdt(gtime_t t, int *week)
    {
        gtime_t t0 = epoch2time(bdt0);
        time_t sec = t.time - t0.time;
        int w = (int)(sec / (86400*7));

        if (week) *week = w;
        return (double)(sec - w*86400*7) + t.sec;
    }

    /* gpstime to utc --------------------------------------------------------------
    * convert gpstime to utc considering leap seconds
    * args   : gtime_t t        I   time expressed in gpstime
    * return : time expressed in utc
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t gpst2utc(gtime_t t)
    {
        gtime_t tu;
        int i;

        for (i=0;leaps[i][0]>0;i++)
        {
            tu=time_add(t,leaps[i][6]);
            if (time_diff(tu,epoch2time(leaps[i]))>=0.0) return tu;
        }
        return t;
    }

    /* utc to gpstime --------------------------------------------------------------
    * convert utc to gpstime considering leap seconds
    * args   : gtime_t t        I   time expressed in utc
    * return : time expressed in gpstime
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    gtime_t utc2gpst(gtime_t t)
    {
        int i;

        for (i=0;leaps[i][0]>0;i++)
        {
            if (time_diff(t,epoch2time(leaps[i]))>=0.0) return time_add(t,-leaps[i][6]);
        }
        return t;
    }

    double julian_day(std::vector<double> datetime)
    {
        LOG_IF(FATAL, datetime.size() != 6) << "datetime should contain 6 fields";
        LOG_IF(FATAL, datetime[0] < 1901 || datetime[0] > 2099) << datetime[0] << " should within range 1900 ~ 2100";

        if (datetime[1] <= 2)
        {
            datetime[1] += 12;
            datetime[0] -= 1;
        }
        double julian_day = floor(365.25*datetime[0]) + floor(30.6001*(datetime[1]+1)) -
                            15 + 1720996.5 + datetime[2] + datetime[3] / 24 +
                            datetime[4] / 60 / 24 + datetime[5] / 3600 / 24;
        return julian_day;
    }

    uint32_t leap_seconds_from_GPS_epoch(std::vector<double> datetime)
    {
        gtime_t given_t = epoch2time(&(datetime[0]));
        for (size_t i = 0; leaps[i][0] > 0; ++i)
        {
            gtime_t tu = time_add(given_t, leaps[i][6]);
            if (time_diff(tu, epoch2time(leaps[i])) >= 0.0)
                return static_cast<uint32_t>(-leaps[i][6]);
        }
        return static_cast<uint32_t>(-1);
    }

    double time2doy(gtime_t time)
    {
        double ep[6];
        time2epoch(time, ep);
        ep[1] = ep[2] = 1.0;
        ep[3] = ep[4] = ep[5] = 0.0;
        return time_diff(time, epoch2time(ep))/86400.0 + 1.0;
    }

    double time_diff(gtime_t t1, gtime_t t2)
    {
        return difftime(t1.time,t2.time)+t1.sec-t2.sec;
    }

    gtime_t time_add(gtime_t t, double sec)
    {
        t.sec += sec;
        double tt = floor(t.sec);
        t.time += static_cast<int>(tt);
        t.sec -= tt;
        return t;
    }

    double time2sec(gtime_t time)
    {
        return static_cast<double>(time.time) + time.sec;
    }

    gtime_t sec2time(const double sec)
    {
        gtime_t time;
        time.time = floor(sec);
        time.sec = sec - time.time;
        return time;
    }


} // namespace gnss_comm
