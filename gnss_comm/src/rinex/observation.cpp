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
#include "gnss_comm/constants/observation.hpp"
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

    // RINEX 观测码（频段+属性）到 CODE_* 的映射，下标即 CODE_* 值。
    static const char *const code_strs[MAXCODE+1] = {
        "",   "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E",
        "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P",
        "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X",
        "6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8I", "8Q", "8X",
        "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q"};

    uint8_t rinex_obs_code(const std::string &band_attr)
    {
        for (uint8_t i = 1; i <= MAXCODE; ++i)
            if (band_attr == code_strs[i])  return i;
        return CODE_NONE;
    }

    // 同一频段有多个信号时的选择优先级（越靠前越优先），尽量与 u-blox 流动站的信号一致。
    int rinex_attr_rank(const uint8_t sys_char, const char band, const char attr)
    {
        static const std::map<std::string, std::string> pris = {
            {"G1", "CPWYMNSL"}, {"G2", "LSXPWYMNDC"}, {"G5", "QIX"},
            {"R1", "CP"}, {"R2", "CP"}, {"R3", "IQX"},
            {"E1", "CBXAZ"}, {"E5", "QIX"}, {"E6", "CBXAZ"}, {"E7", "QIX"}, {"E8", "QIX"},
            {"C1", "IQX"}, {"C2", "IQX"}, {"C6", "IQX"}, {"C7", "IQX"},
            {"J1", "CSLXZ"}, {"J2", "LSX"}, {"J5", "QIX"}, {"J6", "SLX"}};
        const auto it = pris.find(std::string{static_cast<char>(sys_char), band});
        const size_t pos = it == pris.end() ? std::string::npos : it->second.find(attr);
        return pos == std::string::npos ? 100 : static_cast<int>(pos);
    }

    // 频段对应的载波频率，未知频段返回 -1；GLONASS 按频道号修正，RINEX 3.02 及以前北斗 "1" 频段为 B1I。
    static double band_freq(const uint8_t sys_char, const char band, const uint32_t prn,
        const double version, const std::map<uint32_t, int> &glo_freqo)
    {
        if (sys_char == 'C' && band == '1' && version < 3.025)  return FREQ1_BDS;
        const auto freq_it = type2freq.find(std::string{static_cast<char>(sys_char), band});
        if (freq_it == type2freq.end())  return -1.0;
        if (sys_char == 'R' && (band == '1' || band == '2'))
        {
            const auto it = glo_freqo.find(prn);
            if (it == glo_freqo.end())  return freq_it->second;   // 头文件缺少频道号时退化为基准频率
            return freq_it->second + it->second * (band == '1' ? DFRQ1_GLO : DFRQ2_GLO);
        }
        return freq_it->second;
    }

    static ObsPtr rinex_line2obs(const std::string &rinex_str,
        const std::map<uint8_t, std::vector<std::string>> &sys2type, const double version,
        const std::map<uint32_t, int> &glo_freqo)
    {
        ObsPtr obs;
        uint8_t sys_char = rinex_str.at(0);
        if (char2sys.count(sys_char) == 0 || sys2type.count(sys_char) == 0)   return obs;
        uint32_t sys = char2sys.at(sys_char);
        uint32_t prn = static_cast<uint32_t>(std::stoi(rinex_str.substr(1, 2)));
        const std::vector<std::string> &data_types = sys2type.at(sys_char);

        // 每个字段为 14 位数值 + 1 位 LLI + 1 位信号强度，行尾空白可能被截掉。
        std::vector<double> values(data_types.size(), 0);
        std::vector<uint8_t> llis(data_types.size(), 0);
        std::vector<bool> has_value(data_types.size(), false);
        for (size_t i = 0; i < data_types.size(); ++i)
        {
            const size_t offset = 3 + 16*i;
            if (offset >= rinex_str.size())  break;
            const std::string field_str = rinex_str.substr(offset, 14);
            if (field_str.find_first_not_of(' ') == std::string::npos)  continue;
            values[i] = std::stod(field_str);
            has_value[i] = true;
            if (offset+14 < rinex_str.size() && rinex_str.at(offset+14) >= '0' && rinex_str.at(offset+14) <= '9')
                llis[i] = static_cast<uint8_t>(rinex_str.at(offset+14) - '0');
        }

        // 每个频段只保留一个信号：在有伪距或载波的属性中取优先级最高者，避免 L2S/L2W 等互相覆盖。
        std::map<char, char> band2attr;
        for (size_t i = 0; i < data_types.size(); ++i)
        {
            const char kind = data_types[i].at(0), band = data_types[i].at(1), attr = data_types[i].at(2);
            if (!has_value[i] || (kind != 'C' && kind != 'L'))  continue;
            const auto it = band2attr.find(band);
            if (it == band2attr.end() || rinex_attr_rank(sys_char, band, attr) < rinex_attr_rank(sys_char, band, it->second))
                band2attr[band] = attr;
        }

        obs.reset(new Obs());
        obs->sat = sat_no(sys, prn);
        for (const auto &band_attr : band2attr)
        {
            const double freq = band_freq(sys_char, band_attr.first, prn, version, glo_freqo);
            if (freq <= 0)  continue;
            double psr = 0, cp = 0, dopp = 0, cn0 = 0;
            uint8_t lli = 0;
            for (size_t i = 0; i < data_types.size(); ++i)
            {
                if (!has_value[i] || data_types[i].at(1) != band_attr.first || data_types[i].at(2) != band_attr.second)
                    continue;
                switch (data_types[i].at(0))
                {
                    case 'C': psr = values[i]; break;
                    case 'L': cp = values[i]; lli = llis[i]; break;
                    case 'D': dopp = values[i]; break;
                    case 'S': cn0 = values[i]; break;
                    default: break;
                }
            }
            obs->freqs.push_back(freq);
            obs->code.push_back(rinex_obs_code(std::string{band_attr.first, band_attr.second}));
            obs->psr.push_back(psr);
            obs->psr_std.push_back(0);
            obs->cp.push_back(cp);
            obs->cp_std.push_back(0);
            obs->dopp.push_back(dopp);
            obs->dopp_std.push_back(0);
            obs->cn0.push_back(cn0);
            obs->lli.push_back(lli & 0x03); // RINEX bit2 为跟踪属性，不是统一 LLI。
            // RINEX LLI bit1 表示半周模糊度，不能无条件认为半周已解决。
            obs->status.push_back((psr != 0 ? PSEUDORANGE_VALID : 0) |
                                  (cp != 0 ? CARRIER_PHASE_VALID | HALF_CYCLE_KNOWN |
                                   ((lli & 0x02) ? 0 : HALF_CYCLE_RESOLVED) : 0));
        }
        return obs;
    }

    void rinex2obs(const std::string &rinex_filepath, std::vector<std::vector<ObsPtr>> &rinex_meas)
    {
        std::ifstream obs_file(rinex_filepath);
        std::string rinex_line;

        // parse header
        std::map<uint8_t, std::vector<std::string>> sys2type;
        std::map<uint32_t, int> glo_freqo;   // GLONASS PRN -> 频道号
        double version = 0;
        uint8_t sys_char = 0;
        while (std::getline(obs_file, rinex_line))
        {
            if (rinex_line.find("CRINEX VERS") != std::string::npos)
            {
                LOG(ERROR) << "Hatanaka compressed RINEX, decompress with crx2rnx first: " << rinex_filepath;
                return;
            }
            else if (rinex_line.find("RINEX VERSION / TYPE") != std::string::npos)
            {
                version = std::stod(rinex_line.substr(0, 9));
                if (version < 3.0 || version >= 4.0)
                {
                    LOG(ERROR) << "Only RINEX 3.xx is supported for observation file, got " << version;
                    return;
                }
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
            else if (rinex_line.find("GLONASS SLOT / FRQ #") != std::string::npos)
            {
                for (size_t i = 0; i < 8; ++i)
                {
                    const std::string slot = rinex_line.substr(4+7*i, 3);
                    if (slot.at(0) != 'R')  continue;
                    glo_freqo[std::stoi(slot.substr(1, 2))] = std::stoi(rinex_line.substr(8+7*i, 2));
                }
            }
            else if (rinex_line.find("END OF HEADER") != std::string::npos)  break;
        }
        LOG_IF(WARNING, sys2type.count('R') && glo_freqo.empty())
            << "No GLONASS SLOT / FRQ # in header, GLONASS uses nominal frequency";

        while (std::getline(obs_file, rinex_line))
        {
            if (rinex_line.empty())  continue;
            LOG_IF(FATAL, rinex_line.at(0) != '>') << "Invalid Observation record " << rinex_line;
            const int epoch_flag = rinex_line.at(31) - '0';
            const int num_obs = std::stoi(rinex_line.substr(32, 3));
            // 0 正常、1 断电后恢复，均为观测记录；2~5 事件后跟 num_obs 行头信息，6 为周跳记录，跳过。
            if (epoch_flag > 1)
            {
                for (int i = 0; i < num_obs && std::getline(obs_file, rinex_line); ++i) {}
                continue;
            }
            std::vector<double> epoch_time;
            epoch_time.emplace_back(std::stod(rinex_line.substr(2, 4)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(7, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(10, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(13, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(16, 2)));
            epoch_time.emplace_back(std::stod(rinex_line.substr(18, 11)));
            gtime_t obs_time = epoch2time(&(epoch_time[0]));
            std::vector<ObsPtr> meas;
            for (int i = 0; i < num_obs; ++i)
            {
                LOG_IF(FATAL, !std::getline(obs_file, rinex_line)) << "Incomplete RINEX file";
                ObsPtr obs = rinex_line2obs(rinex_line, sys2type, version, glo_freqo);
                if (!obs || obs->freqs.empty())  continue;
                obs->time = obs_time;
                meas.emplace_back(obs);
            }
            rinex_meas.emplace_back(meas);
        }
    }

    bool rinex2obs_header(const std::string &rinex_filepath, RinexObsHeader &header)
    {
        std::ifstream obs_file(rinex_filepath);
        std::string rinex_line;
        while (std::getline(obs_file, rinex_line))
        {
            if (rinex_line.size() <= 60)  continue;
            const std::string label = rinex_line.substr(60);
            if (label.find("MARKER NAME") == 0)
            {
                header.marker_name = rinex_line.substr(0, 60);
                header.marker_name.erase(header.marker_name.find_last_not_of(' ') + 1);
            }
            else if (label.find("ANT # / TYPE") == 0)
            {
                header.antenna_type = rinex_line.substr(20, 20);
            }
            else if (label.find("APPROX POSITION XYZ") == 0)
            {
                header.approx_pos = Eigen::Vector3d(std::stod(rinex_line.substr(0, 14)),
                    std::stod(rinex_line.substr(14, 14)), std::stod(rinex_line.substr(28, 14)));
            }
            else if (label.find("ANTENNA: DELTA H/E/N") == 0)
            {
                header.antenna_delta_hen = Eigen::Vector3d(std::stod(rinex_line.substr(0, 14)),
                    std::stod(rinex_line.substr(14, 14)), std::stod(rinex_line.substr(28, 14)));
            }
            else if (label.find("END OF HEADER") == 0)
            {
                return true;
            }
        }
        return false;
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
