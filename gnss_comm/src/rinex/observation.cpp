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

#include "gnss_comm/constants/frequency.hpp"
#include "gnss_comm/constants/system.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include "gnss_comm/utility/satellite_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <glog/logging.h>
#include <iterator>
#include <map>
#include <sstream>

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

    static ObsPtr rinex_line2obs(const std::string rinex_str,
        const std::map<uint8_t, std::vector<std::string>> &sys2type)
    {
        ObsPtr obs;
        uint8_t sys_char = rinex_str.at(0);
        if (char2sys.count(sys_char) == 0)   return obs;
        obs.reset(new Obs());
        uint32_t sys = char2sys.at(sys_char);
        uint32_t prn = static_cast<uint32_t>(std::stoi(rinex_str.substr(1, 2)));
        obs->sat = sat_no(sys, prn);
        std::map<double, uint32_t> freq2idx;
        std::vector<std::string> date_types = sys2type.at(sys_char);
        uint32_t line_offset = 3;
        for (auto type : date_types)
        {
            std::string field_str = rinex_str.substr(line_offset, 14);
            line_offset += 14 + 2;
            if (field_str.find_first_not_of(' ') == std::string::npos)  continue;
            const double field_value = stod(field_str);
            std::stringstream ss;
            ss << sys_char << type.at(1);
            const double freq = type2freq.at(ss.str());
            uint32_t freq_idx = static_cast<uint32_t>(-1);
            if (freq2idx.count(freq) == 0)
            {
                obs->freqs.push_back(freq);
                freq_idx = obs->freqs.size()-1;
                freq2idx.emplace(freq, freq_idx);
            }
            else
            {
                freq_idx = freq2idx.at(freq);
            }

            if (type.at(0) == 'L')
                obs->cp[freq_idx] = field_value;
            else if (type.at(0) == 'C')
                obs->psr[freq_idx] = field_value;
            else if (type.at(0) == 'D')
                obs->dopp[freq_idx] = field_value;
            else if (type.at(0) == 'S')
                obs->cn0[freq_idx] = field_value;
            else
                LOG(FATAL) << "Unrecognized measurement type " << type.at(0);
        }
        // fill in other fields
        uint32_t num_freqs = obs->freqs.size();
        LOG_IF(FATAL, num_freqs < obs->cn0.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->cn0), num_freqs-obs->cn0.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->lli.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->lli), num_freqs-obs->lli.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->code.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->code), num_freqs-obs->code.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->psr.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->psr), num_freqs-obs->psr.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->psr_std.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->psr_std), num_freqs-obs->psr_std.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->cp.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->cp), num_freqs-obs->cp.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->cp_std.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->cp_std), num_freqs-obs->cp_std.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->dopp.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->dopp), num_freqs-obs->dopp.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->dopp_std.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->dopp_std), num_freqs-obs->dopp_std.size(), 0);
        LOG_IF(FATAL, num_freqs < obs->status.size()) << "Suspicious observation field.\n";
        std::fill_n(std::back_inserter(obs->status), num_freqs-obs->status.size(), 0x0F);

        return obs;
    }

    // TODO: GLONASS slot number
    void rinex2obs(const std::string &rinex_filepath, std::vector<std::vector<ObsPtr>> &rinex_meas)
    {
        std::ifstream obs_file(rinex_filepath);
        std::string rinex_line;

        // parse header
        std::map<uint8_t, std::vector<std::string>> sys2type;
        uint8_t sys_char = 0;
        while (std::getline(obs_file, rinex_line))
        {
            if (rinex_line.find("RINEX VERSION / TYPE") != std::string::npos && rinex_line.find("3.04") == std::string::npos)
            {
                LOG(ERROR) << "Only RINEX 3.04 is supported for observation file";
                return;
            }
            else if (rinex_line.find("SYS / # / OBS TYPES") != std::string::npos)
            {
                if (rinex_line.at(0) != ' ')
                {
                    sys_char = rinex_line.at(0);
                    sys2type.emplace(sys_char, std::vector<std::string>());
                }
                for (size_t i = 0; i < 13; ++i)
                    if (rinex_line.substr(7+4*i, 3) != "   ")
                        sys2type.at(sys_char).emplace_back(rinex_line.substr(7+4*i, 3));
            }
            else if (rinex_line.find("END OF HEADER") != std::string::npos)  break;
        }

        while (std::getline(obs_file, rinex_line))
        {
            LOG_IF(FATAL, rinex_line.at(0) != '>') << "Invalid Observation record " << rinex_line;
            LOG_IF(FATAL, rinex_line.at(31) != '0') << "Invalid Epoch data " << rinex_line.at(31)-48;
            std::vector<double> epoch_time;
            epoch_time.emplace_back(std::stod(rinex_line.substr(2, 4)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(7, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(10, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(13, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(16, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(18, 11)));
            gtime_t obs_time = epoch2time(&(epoch_time[0]));
            const int num_obs = std::stoi(rinex_line.substr(32, 3));
            std::vector<ObsPtr> meas;
            for (int i = 0; i < num_obs; ++i)
            {
                LOG_IF(FATAL, !std::getline(obs_file, rinex_line)) << "Incomplete RINEX file";
                ObsPtr obs = rinex_line2obs(rinex_line, sys2type);
                if (!obs || obs->freqs.empty())  continue;
                obs->time = obs_time;
                meas.emplace_back(obs);
            }
            rinex_meas.emplace_back(meas);
        }
    }

    int obs_index(const uint32_t sys, const double freq)
    {
        if (sys == SYS_GPS || sys == SYS_QZS)
        {
            if (freq == FREQ1)          return 0;
            else if (freq == FREQ2)     return 1;
        }
        else if (sys == SYS_GLO)
        {
            if (freq <= (FREQ1_GLO+6*DFRQ1_GLO))        return 0;
            else if (freq <= (FREQ2_GLO+6*DFRQ2_GLO))   return 1;
        }
        else if (sys == SYS_GAL)
        {
            if (freq == FREQ1)          return 0;
            else if (freq == FREQ7)     return 1;
        }
        else if (sys == SYS_BDS)
        {
            if (freq == FREQ1_BDS)          return 0;
            else if (freq == FREQ2_BDS)     return 1;
        }
        return -1;
    }

    double index_freq(const ObsPtr &obs, const int band_idx, int &freq_idx)
    {
        freq_idx = -1;      // set frequency index of `obs->freqs` invalid
        const uint32_t sys = satsys(obs->sat, NULL);

        // calculate target band frequency
        double freq_min = 0;
        double freq_max = 0;
        if ((sys == SYS_GPS || sys == SYS_QZS) && band_idx == 0)
        {
            freq_min = freq_max = FREQ1;
        }
        else if ((sys == SYS_GPS || sys == SYS_QZS) && band_idx == 1)
        {
            freq_min = freq_max = FREQ2;
        }
        else if (sys == SYS_GLO && band_idx == 0)
        {
            freq_min = FREQ1_GLO-7*DFRQ1_GLO;
            freq_max = FREQ1_GLO+6*DFRQ1_GLO;
        }
        else if (sys == SYS_GLO && band_idx == 1)
        {
            freq_min = FREQ2_GLO-7*DFRQ2_GLO;
            freq_max = FREQ2_GLO+6*DFRQ2_GLO;
        }
        else if (sys == SYS_GAL && band_idx == 0)
        {
            freq_min = freq_max = FREQ1;
        }
        else if (sys == SYS_GAL && band_idx == 1)
        {
            freq_min = freq_max = FREQ7;
        }
        else if (sys == SYS_BDS && band_idx == 0)
        {
            freq_min = freq_max = FREQ1_BDS;
        }
        else if (sys == SYS_BDS && band_idx == 1)
        {
            freq_min = freq_max = FREQ2_BDS;
        }

        // check if target frequency exists or not
        for (uint32_t i = 0; i < obs->freqs.size(); ++i)
        {
            if (obs->freqs[i] >= freq_min && obs->freqs[i] <= freq_max)
            {
                freq_idx = static_cast<int>(i);
                return obs->freqs[i];
            }
        }

        return -1.0;
    }

    std::map<uint32_t, int> get_glo_freqo(const std::vector<std::vector<ObsPtr>> &gnss_meas)
    {
        std::map<uint32_t, int> sat2freqo;
        for (auto &meas : gnss_meas)
        {
            for (auto &obs : meas)
            {
                const uint32_t sys = satsys(obs->sat, NULL);
                if (sys != SYS_GLO)     continue;

                for (double freq : obs->freqs)
                {
                    if (freq > FREQ1_GLO-8*DFRQ1_GLO && freq < FREQ1_GLO+7*DFRQ1_GLO)
                    {
                        const int freqo = round((freq-FREQ1_GLO)/DFRQ1_GLO);
                        sat2freqo.emplace(obs->sat, freqo);
                    }
                    else if (freq > FREQ2_GLO-8*DFRQ2_GLO && freq < FREQ2_GLO+7*DFRQ2_GLO)
                    {
                        const int freqo = round((freq-FREQ2_GLO)/DFRQ2_GLO);
                        sat2freqo.emplace(obs->sat, freqo);
                    }
                }
            }
        }
        return sat2freqo;
    }

    void obs2rinex(const std::string &rinex_filepath,
        const std::vector<std::vector<ObsPtr>> &gnss_meas)
    {
        FILE *fp = fopen(rinex_filepath.c_str(), "w");
        fprintf(fp, "%9.2f%-11s%-20s%-20s%-20s\n", 3.04, "", "OBSERVATION DATA",
            "M: Mixed", "RINEX VERSION / TYPE");
        std::time_t time_ptr;
        time_ptr = time(NULL);
        tm *tm_utc = gmtime(&time_ptr);
        char date_str[256];
        sprintf(date_str, "%4d%02d%02d %02d%02d%02d UTC", tm_utc->tm_year+1900, tm_utc->tm_mon+1,
            tm_utc->tm_mday, tm_utc->tm_hour, tm_utc->tm_min, tm_utc->tm_sec);
        fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n","gnss_comm obs2rinex", "", date_str,
            "PGM / RUN BY / DATE");
        fprintf(fp, "%-60.60s%-20s\n", "", "MARKER NAME");
        fprintf(fp, "%-20.20s%-40.40s%-20s\n", "", "", "MARKER NUMBER");
        fprintf(fp, "%-20.20s%-40.40s%-20s\n", "", "", "MARKER TYPE");
        fprintf(fp, "%-20.20s%-40.40s%-20s\n", "", "", "OBSERVER / AGENCY");
        fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", "", "", "", "REC # / TYPE / VERS");
        fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", "", "", "", "ANT # / TYPE");
        // ignore approximate position
        fprintf(fp, "%-14.14s%-14.14s%-14.14s%-18s%-20s\n", "", "", "", "", "APPROX POSITION XYZ");
        fprintf(fp, "%-14.14s%-14.14s%-14.14s%-18s%-20s\n", "", "", "", "", "ANTENNA: DELTA H/E/N");

        // observation type, hard code here
        // only support dual-frequency record without the knowledge of signal type
        fprintf(fp, "%c  %3d %3s %3s %3s %3s %3s %3s %3s %3s%20s  %-20s\n", 'G', 8, "C1C", "L1C",
            "D1C", "S1C", "C2S", "L2S", "D2S", "S2S", "", "SYS / # / OBS TYPES ");
        fprintf(fp, "%c  %3d %3s %3s %3s %3s %3s %3s %3s %3s%20s  %-20s\n", 'R', 8, "C1C", "L1C",
            "D1C", "S1C", "C2C", "L2C", "D2C", "S2C", "", "SYS / # / OBS TYPES ");
        fprintf(fp, "%c  %3d %3s %3s %3s %3s %3s %3s %3s %3s%20s  %-20s\n", 'E', 8, "C1C", "L1C",
            "D1C", "S1C", "C7Q", "L7Q", "D7Q", "S7Q", "", "SYS / # / OBS TYPES ");
        fprintf(fp, "%c  %3d %3s %3s %3s %3s %3s %3s %3s %3s%20s  %-20s\n", 'C', 8, "C2I", "L2I",
            "D2I", "S2I", "C7I", "L7I", "D7I", "S7I", "", "SYS / # / OBS TYPES ");
        fprintf(fp, "%c  %3d %3s %3s %3s %3s %3s %3s %3s %3s%20s  %-20s\n", 'J', 8, "C1C", "L1C",
            "D1C", "S1C", "C2L", "L2L", "D2L", "S2L", "", "SYS / # / OBS TYPES ");

        if (!gnss_meas.empty())
        {
            int front_idx = -1;
            while(gnss_meas[++front_idx].empty());
            const gtime_t start_time = gnss_meas[front_idx].front()->time;
            double ep[6];
            time2epoch(start_time, ep);
            fprintf(fp, "  %04.0f%6.0f%6.0f%6.0f%6.0f%13.7f     %-12s%-20s\n", ep[0],
                ep[1], ep[2], ep[3], ep[4], ep[5], "GPS", "TIME OF FIRST OBS");
            int back_idx = static_cast<int>(gnss_meas.size());
            while(gnss_meas[--back_idx].empty());
            const gtime_t end_time = gnss_meas[back_idx].front()->time;
            time2epoch(end_time, ep);
            fprintf(fp, "  %04.0f%6.0f%6.0f%6.0f%6.0f%13.7f     %-12s%-20s\n", ep[0],
                ep[1], ep[2], ep[3], ep[4], ep[5], "GPS", "TIME OF LAST OBS");
        }
        fprintf(fp,"%c %-58s%-20s\n",'G',"","SYS / PHASE SHIFT");
        fprintf(fp,"%c %-58s%-20s\n",'R',"","SYS / PHASE SHIFT");
        fprintf(fp,"%c %-58s%-20s\n",'E',"","SYS / PHASE SHIFT");
        fprintf(fp,"%c %-58s%-20s\n",'C',"","SYS / PHASE SHIFT");
        fprintf(fp,"%c %-58s%-20s\n",'J',"","SYS / PHASE SHIFT");

        // add GLONASS SLOT / FRQ #
        std::map<uint32_t, int> glo_sat2freqo = get_glo_freqo(gnss_meas);
        auto glo_sat2freqo_it = glo_sat2freqo.begin();
        const int num_glo_sats = static_cast<int>(glo_sat2freqo.size());
        for (int i = 0; i < (num_glo_sats<=0?1:(num_glo_sats-1)/8+1); ++i)
        {
            if (i == 0)
                fprintf(fp, "%3d", num_glo_sats);
            else
                fprintf(fp, "%3s", "");
            for (int j = 0; j < 8; ++j)
            {
                if (i*8+j < num_glo_sats)
                {
                    fprintf(fp, " %3s %2d", sat2str(glo_sat2freqo_it->first).c_str(), glo_sat2freqo_it->second);
                    ++ glo_sat2freqo_it;
                }
                else
                {
                    fprintf(fp, " %6s", "");
                }
            }
            fprintf(fp, " %-20s\n", "GLONASS SLOT / FRQ #");
        }
        fprintf(fp," C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000        GLONASS COD/PHS/BIS \n");
        fprintf(fp,"%-60.60s%-20s\n","","END OF HEADER");

        // write data
        for (auto &meas : gnss_meas)
        {
            if (meas.empty())         continue;
            double ep[6];
            time2epoch(meas[0]->time, ep);
            fprintf(fp, "> %04.0f %2.0f %2.0f %2.0f %2.0f%11.7f  %d%3d%21s\n",
                ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], 0, static_cast<int>(meas.size()), "");

            std::vector<ObsPtr> sorted_meas(meas);
            std::sort(sorted_meas.begin(), sorted_meas.end(), [](ObsPtr o1, ObsPtr o2) {
                return o1->sat < o2->sat;
            });

            for (auto &obs : sorted_meas)
            {
                std::string sat_str = sat2str(obs->sat);
                fprintf(fp, "%3s", sat_str.c_str());
                for (int k = 0; k < 2; ++k)
                {
                    int freq_idx = -1;
                    const double freq = index_freq(obs, k, freq_idx);
                    if (freq < 0)
                    {
                        fprintf(fp, "%16s%16s%16s%16s", "", "", "", "");
                    }
                    else
                    {
                        fprintf(fp, "%14.3f%2s", obs->psr[freq_idx], "");
                        fprintf(fp, "%14.3f%2s", obs->cp[freq_idx], "");
                        fprintf(fp, "%14.3f%2s", obs->dopp[freq_idx], "");
                        fprintf(fp, "%14.3f%2s", obs->cn0[freq_idx], "");
                    }
                }
                fprintf(fp, "\n");
            }
        }

        fclose(fp);

    }

} // namespace gnss_comm
