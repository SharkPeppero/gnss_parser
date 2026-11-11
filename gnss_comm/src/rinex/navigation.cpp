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
*/

#include "gnss_comm/constants/system.hpp"
#include "gnss_comm/constants/time.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include "gnss_comm/utility/satellite_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <cmath>
#include <fstream>
#include <glog/logging.h>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
    static double str2double(const std::string &num_str)
    {
        // replace character 'D' with 'e'
        size_t D_pos = num_str.find("D");
        std::string tmp_str = num_str;
        if (D_pos != std::string::npos)
            tmp_str = tmp_str.replace(D_pos, 1, "e");
        return std::stod(tmp_str);
    }

    static GloEphemPtr rinex_line2glo_ephem(const std::vector<std::string> &ephem_lines, const uint32_t gpst_leap_seconds)
    {
        LOG_IF(FATAL, ephem_lines.size() != 4) << "GLO ephemeris record should contain 4 lines";
        LOG_IF(FATAL, ephem_lines[0].at(0) != 'R') << "Not a valid GLO ephemeris record";
        GloEphemPtr glo_ephem(new GloEphem());

        uint32_t prn = static_cast<uint32_t>(std::stoi(ephem_lines[0].substr(1, 2)));
        glo_ephem->sat = sat_no(SYS_GLO, prn);
        double epoch[6];
        epoch[0] = static_cast<double>(std::stoi(ephem_lines[0].substr(4, 4)));
        epoch[1] = static_cast<double>(std::stoi(ephem_lines[0].substr(9, 2)));
        epoch[2] = static_cast<double>(std::stoi(ephem_lines[0].substr(12, 2)));
        epoch[3] = static_cast<double>(std::stoi(ephem_lines[0].substr(15, 2)));
        epoch[4] = static_cast<double>(std::stoi(ephem_lines[0].substr(18, 2)));
        epoch[5] = static_cast<double>(std::stoi(ephem_lines[0].substr(21, 2)));
        glo_ephem->toe = epoch2time(epoch);
        glo_ephem->toe.time += gpst_leap_seconds;
        glo_ephem->tau_n = -1.0 * str2double(ephem_lines[0].substr(23, 19));
        glo_ephem->gamma = str2double(ephem_lines[0].substr(42, 19));

        // the second line
        glo_ephem->pos[0] = str2double(ephem_lines[1].substr(4, 19)) * 1e3;
        glo_ephem->vel[0] = str2double(ephem_lines[1].substr(23, 19)) * 1e3;
        glo_ephem->acc[0] = str2double(ephem_lines[1].substr(42, 19)) * 1e3;
        glo_ephem->health = static_cast<uint32_t>(str2double(ephem_lines[1].substr(61, 19)));

        // the third line
        glo_ephem->pos[1] = str2double(ephem_lines[2].substr(4, 19)) * 1e3;
        glo_ephem->vel[1] = str2double(ephem_lines[2].substr(23, 19)) * 1e3;
        glo_ephem->acc[1] = str2double(ephem_lines[2].substr(42, 19)) * 1e3;
        glo_ephem->freqo  = static_cast<int>(str2double(ephem_lines[2].substr(61, 19)));

        // the forth line
        glo_ephem->pos[2] = str2double(ephem_lines[3].substr(4, 19)) * 1e3;
        glo_ephem->vel[2] = str2double(ephem_lines[3].substr(23, 19)) * 1e3;
        glo_ephem->acc[2] = str2double(ephem_lines[3].substr(42, 19)) * 1e3;
        glo_ephem->age  = static_cast<uint32_t>(str2double(ephem_lines[3].substr(61, 19)));

        return glo_ephem;
    }

    static EphemPtr rinex_line2ephem(const std::vector<std::string> &ephem_lines)
    {
        LOG_IF(FATAL, ephem_lines.size() != 8) << "Ephemeris record should contain 8 lines";
        uint32_t sat_sys = SYS_NONE;
        if      (ephem_lines[0].at(0) == 'G')    sat_sys = SYS_GPS;
        else if (ephem_lines[0].at(0) == 'C')    sat_sys = SYS_BDS;
        else if (ephem_lines[0].at(0) == 'E')    sat_sys = SYS_GAL;
        else if (ephem_lines[0].at(0) == 'J')    sat_sys = SYS_QZS;
        LOG_IF(FATAL, sat_sys == SYS_NONE) << "Satellite system is not supported: " << ephem_lines[0].at(0);

        EphemPtr ephem(new Ephem());
        uint32_t prn = static_cast<uint32_t>(std::stoi(ephem_lines[0].substr(1, 2)));
        ephem->sat = sat_no(sat_sys, prn);
        double epoch[6];
        epoch[0] = static_cast<double>(std::stoi(ephem_lines[0].substr(4, 4)));
        epoch[1] = static_cast<double>(std::stoi(ephem_lines[0].substr(9, 2)));
        epoch[2] = static_cast<double>(std::stoi(ephem_lines[0].substr(12, 2)));
        epoch[3] = static_cast<double>(std::stoi(ephem_lines[0].substr(15, 2)));
        epoch[4] = static_cast<double>(std::stoi(ephem_lines[0].substr(18, 2)));
        epoch[5] = static_cast<double>(std::stoi(ephem_lines[0].substr(21, 2)));
        ephem->toc = epoch2time(epoch);
        if (sat_sys == SYS_BDS)     ephem->toc.time += BDT_GPST_OFFSET;     // BDS-GPS time correction
        ephem->af0 = str2double(ephem_lines[0].substr(23, 19));
        ephem->af1 = str2double(ephem_lines[0].substr(42, 19));
        ephem->af2 = str2double(ephem_lines[0].substr(61, 19));

        // the second line
        if (sat_sys == SYS_GPS || sat_sys == SYS_QZS)
            ephem->iode  = str2double(ephem_lines[1].substr(4, 19));
        ephem->crs       = str2double(ephem_lines[1].substr(23, 19));
        ephem->delta_n   = str2double(ephem_lines[1].substr(42, 19));
        ephem->m0        = str2double(ephem_lines[1].substr(61, 19));

        // the third line
        ephem->cuc = str2double(ephem_lines[2].substr(4, 19));
        ephem->e = str2double(ephem_lines[2].substr(23, 19));
        ephem->cus = str2double(ephem_lines[2].substr(42, 19));
        double sqrt_A = str2double(ephem_lines[2].substr(61, 19));
        ephem->a = sqrt_A * sqrt_A;

        // the forth line
        ephem->toe_tow = str2double(ephem_lines[3].substr(4, 19));
        ephem->cic = str2double(ephem_lines[3].substr(23, 19));
        ephem->omg0 = str2double(ephem_lines[3].substr(42, 19));
        ephem->cis = str2double(ephem_lines[3].substr(61, 19));

        // the fifth line
        ephem->i0 = str2double(ephem_lines[4].substr(4, 19));
        ephem->crc = str2double(ephem_lines[4].substr(23, 19));
        ephem->omg = str2double(ephem_lines[4].substr(42, 19));
        ephem->omg_dot = str2double(ephem_lines[4].substr(61, 19));

        // the sixth line
        ephem->i_dot = str2double(ephem_lines[5].substr(4, 19));
        if  (sat_sys == SYS_GAL)
        {
            uint32_t ephe_source = static_cast<uint32_t>(str2double(ephem_lines[5].substr(23, 19)));
            if (!(ephe_source & 0x01))
            {
                // LOG(ERROR) << "not contain I/NAV E1-b info, skip this ephemeris";
                return ephem;   // only parse I/NAV E1-b ephemeris
            }
        }
        ephem->week = static_cast<uint32_t>(str2double(ephem_lines[5].substr(42, 19)));
        if (sat_sys == SYS_GPS || sat_sys == SYS_GAL || sat_sys == SYS_QZS)     ephem->toe = gpst2time(ephem->week, ephem->toe_tow);
        else if (sat_sys == SYS_BDS)                      ephem->toe = bdt2time(ephem->week, ephem->toe_tow+BDT_GPST_OFFSET);
        // if (sat_sys == SYS_GAL)     ephem->toe = gst2time(ephem->week, ephem->toe_tow);

        // the seventh line
        ephem->ura = str2double(ephem_lines[6].substr(4, 19));
        ephem->health = static_cast<uint32_t>(str2double(ephem_lines[6].substr(23, 19)));
        ephem->tgd[0] = str2double(ephem_lines[6].substr(42, 19));
        if (sat_sys == SYS_BDS || sat_sys == SYS_GAL)
            ephem->tgd[1] = str2double(ephem_lines[6].substr(61, 19));
        if (sat_sys == SYS_GPS || sat_sys == SYS_QZS)     ephem->iodc = str2double(ephem_lines[6].substr(61, 19));

        // the eighth line
        double ttr_tow = str2double(ephem_lines[7].substr(4, 19));
        // GAL week = GST week + 1024 + rollover, already align with GPS week!!!
        if      (sat_sys == SYS_GPS || sat_sys == SYS_GAL || sat_sys == SYS_QZS)   ephem->ttr = gpst2time(ephem->week, ttr_tow);
        else if (sat_sys == SYS_BDS)   ephem->ttr = bdt2time(ephem->week, ttr_tow+BDT_GPST_OFFSET);

        // convert time system to parameter GPST
        if (sat_sys == SYS_BDS)
        {
            uint32_t week = 0;
            ephem->toe_tow = time2gpst(ephem->toe, &week);
            ephem->week = week;
        }

        return ephem;
    }

    void rinex2ephems(const std::string &rinex_filepath, std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem)
    {
        uint32_t gpst_leap_seconds = static_cast<uint32_t>(-1);
        std::ifstream ephem_file(rinex_filepath);
        std::string line;
        while(std::getline(ephem_file, line))
        {
            if (line.find("RINEX VERSION / TYPE") != std::string::npos && line.find("3.04") == std::string::npos)
            {
                LOG(ERROR) << "Only RINEX 3.04 is supported for observation file";
                return;
            }
            else if (line.find("LEAP SECONDS") != std::string::npos && line.find("BDS") == std::string::npos)
                gpst_leap_seconds = static_cast<uint32_t>(std::stoi(line.substr(4, 6)));
            else if (line.find("END OF HEADER") != std::string::npos)
                break;
        }
        LOG_IF(FATAL, gpst_leap_seconds == static_cast<uint32_t>(-1)) << "No leap second record found";

        while(std::getline(ephem_file, line))
        {
            if (line.at(0) == 'G' || line.at(0) == 'C' || line.at(0) == 'E' || line.at(0) == 'J')
            {
                std::vector<std::string> ephem_lines;
                ephem_lines.push_back(line);
                for (size_t i = 0; i < 7; ++i)
                {
                    std::getline(ephem_file, line);
                    ephem_lines.push_back(line);
                }
                EphemPtr ephem = rinex_line2ephem(ephem_lines);
                if (!ephem || ephem->ttr.time == 0)  continue;
                if (sat2ephem.count(ephem->sat) == 0)
                    sat2ephem.emplace(ephem->sat, std::vector<EphemBasePtr>());
                sat2ephem.at(ephem->sat).push_back(ephem);
            }
            else if (line.at(0) == 'R')
            {
                std::vector<std::string> ephem_lines;
                ephem_lines.push_back(line);
                for (size_t i = 0; i < 3; ++i)
                {
                    std::getline(ephem_file, line);
                    ephem_lines.push_back(line);
                }
                GloEphemPtr glo_ephem = rinex_line2glo_ephem(ephem_lines, gpst_leap_seconds);
                if (sat2ephem.count(glo_ephem->sat) == 0)
                    sat2ephem.emplace(glo_ephem->sat, std::vector<EphemBasePtr>());
                sat2ephem.at(glo_ephem->sat).push_back(glo_ephem);
            }
        }
    }

    void rinex2iono_params(const std::string &rinex_filepath, std::vector<double> &iono_params)
    {
        iono_params.resize(8);
        std::ifstream file(rinex_filepath);
        std::string line;

        // check first line, mainly RINEX version
        if (!(std::getline(file, line) && line.find("RINEX VERSION")  != std::string::npos
                && line.find("3.04") != std::string::npos))
        {
            LOG(ERROR) << "Only RINEX 3.04 is supported";
            return;
        }

        bool find_alpha = false, find_beta = false;
        while(std::getline(file, line))
        {
            if (line.find("IONOSPHERIC CORR") != std::string::npos && line.find("GPSA") != std::string::npos)
            {

                // parse ion alpha value
                for (size_t i = 0; i < 4; ++i)
                {
                    std::string value_str = line.substr(i*12+5, 12);
                    iono_params[i] = str2double(value_str);
                }
                find_alpha = true;
            }
            else if (line.find("IONOSPHERIC CORR") != std::string::npos && line.find("GPSB") != std::string::npos)
            {
                // parse ion beta value
                for (size_t i = 0; i < 4; ++i)
                {
                    std::string value_str = line.substr(i*12+5, 12);
                    iono_params[i+4] = str2double(value_str);
                }
                find_beta = true;
            }

            if(find_alpha && find_beta)
                break;
        }
        file.close();
    }


} // namespace gnss_comm
