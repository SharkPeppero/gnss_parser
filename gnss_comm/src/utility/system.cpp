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


#include "gnss_comm/utility/system.hpp"
#include <cstdio>
#include <stdexcept>

namespace gnss_comm {

// 算法实现从原文件迁移；接口含义与单位见对应头文件。
    std::string exec(const std::string &cmd)
    {
        char buffer[128];
        std::string result = "";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("popen() failed!");
        try
        {
            while (fgets(buffer, sizeof buffer, pipe) != NULL)
            {
                result += buffer;
            }
        }
        catch (...)
        {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;

    }

} // namespace gnss_comm
