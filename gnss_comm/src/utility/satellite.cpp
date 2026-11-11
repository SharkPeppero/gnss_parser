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

#include "gnss_comm/constants/frequency.hpp"
#include "gnss_comm/constants/system.hpp"
#include "gnss_comm/utility/satellite_utility.hpp"
#include <glog/logging.h>
#include <iomanip>
#include <sstream>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
    uint32_t sat_no(uint32_t sys, uint32_t prn) {
        if (prn == 0) return 0;
        switch (sys) {
            case SYS_GPS:
                if (prn < MIN_PRN_GPS || prn > MAX_PRN_GPS) return 0;
                return prn - MIN_PRN_GPS + 1;
            case SYS_GLO:
                if (prn < MIN_PRN_GLO || prn > MAX_PRN_GLO) return 0;
                return N_SAT_GPS + prn - MIN_PRN_GLO + 1;
            case SYS_GAL:
                if (prn < MIN_PRN_GAL || prn > MAX_PRN_GAL) return 0;
                return N_SAT_GPS + N_SAT_GLO + prn - MIN_PRN_GAL + 1;
            case SYS_BDS:
                if (prn < MIN_PRN_BDS || prn > MAX_PRN_BDS) return 0;
                return N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + prn - MIN_PRN_BDS + 1;
            case SYS_QZS:
                if (prn < MIN_PRN_QZS || prn > MAX_PRN_QZS) return 0;
                return N_SAT_GPS + N_SAT_GLO + N_SAT_GAL + N_SAT_BDS + prn - MIN_PRN_QZS + 1;
        }
        return 0;
    }

    /* satellite number to satellite system ----------------------------------------
    * convert satellite number to satellite system
    * args   : uint32_t    sat       I   satellite number (1-MAXSAT)
    *          uint32_t    *prn      IO  satellite prn/slot number (NULL: no output)
    * return : satellite system (SYS_GPS,SYS_GLO,...)
    *-----------------------------------------------------------------------------*/
    uint32_t satsys(uint32_t sat, uint32_t *prn)
    {
        uint32_t sys = SYS_NONE;
        if (sat <= 0 || sat > MAX_SAT) sat = 0;
        else if (sat <= N_SAT_GPS) {
            sys = SYS_GPS; sat += MIN_PRN_GPS-1;
        }
        else if ((sat-=N_SAT_GPS) <= N_SAT_GLO) {
            sys = SYS_GLO; sat += MIN_PRN_GLO-1;
        }
        else if ((sat-=N_SAT_GLO) <= N_SAT_GAL) {
            sys = SYS_GAL; sat += MIN_PRN_GAL-1;
        }
        else if ((sat-=N_SAT_GAL) <= N_SAT_BDS) {
            sys = SYS_BDS; sat += MIN_PRN_BDS-1;
        }
        else if ((sat-=N_SAT_BDS) <= N_SAT_QZS) {
            sys = SYS_QZS; sat += MIN_PRN_QZS-1;
        }
        else sat = 0;
        if (prn) *prn = sat;
        return sys;
    }

    std::string sat2str(uint32_t sat_no)
    {
        std::stringstream ss;
        uint32_t prn = 0;
        uint32_t sys = satsys(sat_no, &prn);
        switch (sys)
        {
            case SYS_GPS:
                ss << "G" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_GLO:
                ss << "R" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_BDS:
                ss << "C" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_GAL:
                ss << "E" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_SBS:
                ss << "S" << std::setw(2) << std::setfill('0') << prn;
                break;
            case SYS_QZS:
                ss << "J" << std::setw(2) << std::setfill('0') << prn;
                break;
            default:    // current not support other system
                LOG(WARNING) << "currently not support satelite system id " << sys;
                break;
        }
        return ss.str();
    }

    double L1_freq(const ObsPtr &obs, int *l1_idx)
    {
        const uint32_t sys = satsys(obs->sat, NULL);
        double freq_min = -1.0;
        double freq_max = -1.0;
        if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS)
            freq_min = freq_max = FREQ1;
        else if (sys == SYS_BDS)
            freq_min = freq_max = FREQ1_BDS;
        else if (sys == SYS_GLO)
        {
            freq_min = FREQ1_GLO - 7 * DFRQ1_GLO;
            freq_max = FREQ1_GLO + 6 * DFRQ1_GLO;
        }

        if (l1_idx != NULL)
            *l1_idx = -1;

        for (uint32_t i = 0; i < obs->freqs.size(); ++i)
        {
            if (obs->freqs[i] >= freq_min && obs->freqs[i] <= freq_max)
            {
                if (l1_idx != NULL)
                    *l1_idx = static_cast<int>(i);
                return obs->freqs[i];
            }
        }

        return -1.0;
    }

    uint32_t str2sat(const std::string &sat_str)
    {
        if (sat_str.size() < 3)
        {
            LOG(ERROR) << "Invalid RINEX satellite identifier " << sat_str;
            return 0;
        }
        const uint32_t prn = std::stoul(sat_str.substr(1, 2));
        switch (sat_str.at(0))
        {
            case 'G':
                return sat_no(SYS_GPS, prn);
            case 'R':
                return sat_no(SYS_GLO, prn);
            case 'C':
                return sat_no(SYS_BDS, prn);
            case 'E':
                return sat_no(SYS_GAL, prn);
            case 'S':
                return sat_no(SYS_SBS, prn);
            case 'J':
                return sat_no(SYS_QZS, prn);
        }
        return 0;
    }


} // namespace gnss_comm
