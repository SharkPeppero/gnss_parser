//
// Created by yjh on 2026/11/12.
//
// RTCM 3.x MSM 解码，字段定义与比例因子参考 RTCM 10403.3 及 RTKLIB rtcm3.c。

#include "gnss_comm/rtcm/rtcm_conversion.hpp"
#include "gnss_comm/constants/frequency.hpp"
#include "gnss_comm/constants/system.hpp"
#include "gnss_comm/constants/time.hpp"
#include "gnss_comm/rinex/rinex_conversion.hpp"
#include "gnss_comm/utility/satellite_utility.hpp"
#include "gnss_comm/utility/time_utility.hpp"
#include <cmath>
#include <fstream>
#include <glog/logging.h>
#include <iterator>
#include <limits>
#include <map>

namespace gnss_comm {

namespace {

constexpr double RANGE_MS = LIGHT_SPEED * 0.001; // 1 ms 光行距离，m
constexpr double P2_10 = 9.765625e-4;
constexpr double P2_24 = 5.960464477539063e-08;
constexpr double P2_29 = 1.862645149230957e-09;
constexpr double P2_31 = 4.656612873077393e-10;

// MSM 信号 ID（1~32）到 RINEX 频段+属性，下标为 ID−1；北斗按 RINEX 3.03+ 约定，"2I" 为 B1I。
const char *const msm_sig_gps[32] = {"",   "1C", "1P", "1W", "",   "",   "",   "2C", "2P", "2W", "",   "",   "",   "",   "2S", "2L",
                                     "2X", "",   "",   "",   "",   "5I", "5Q", "5X", "",   "",   "",   "",   "",   "1S", "1L", "1X"};
const char *const msm_sig_glo[32] = {"", "1C", "1P", "", "", "", "", "2C", "2P", "", "", "", "", "", "", "",
                                     "", "",   "",   "", "", "", "", "",   "",   "", "", "", "", "", "", ""};
const char *const msm_sig_gal[32] = {"",   "1C", "1A", "1B", "1X", "1Z", "",   "6C", "6A", "6B", "6X", "6Z", "", "7I", "7Q", "7X",
                                     "",   "8I", "8Q", "8X", "",   "5I", "5Q", "5X", "",   "",   "",   "",   "", "",   "",   ""};
const char *const msm_sig_qzs[32] = {"",   "1C", "",   "",   "",   "",   "",   "",   "6S", "6L", "6X", "",   "",   "",   "2S", "2L",
                                     "2X", "",   "",   "",   "",   "5I", "5Q", "5X", "",   "",   "",   "",   "",   "1S", "1L", "1X"};
const char *const msm_sig_bds[32] = {"",   "2I", "2Q", "2X", "",   "",   "",   "6I", "6Q", "6X", "",   "",   "",   "7I", "7Q", "7X",
                                     "",   "",   "",   "",   "",   "5D", "5P", "5X", "",   "",   "",   "",   "",   "1D", "1P", "1X"};

uint32_t getbitu(const uint8_t *buff, const uint32_t pos, const uint32_t len) {
  uint32_t bits = 0;
  for (uint32_t i = pos; i < pos + len; ++i)
    bits = (bits << 1) | ((buff[i / 8] >> (7 - i % 8)) & 1u);
  return bits;
}

int32_t getbits(const uint8_t *buff, const uint32_t pos, const uint32_t len) {
  const uint32_t bits = getbitu(buff, pos, len);
  if (len == 0 || len >= 32 || !(bits & (1u << (len - 1))))
    return static_cast<int32_t>(bits);
  return static_cast<int32_t>(bits | (~0u << len));
}

// 38 位有符号数（1005/1006 的坐标字段）
double getbits38(const uint8_t *buff, const uint32_t pos) { return static_cast<double>(getbits(buff, pos, 32)) * 64.0 + getbitu(buff, pos + 32, 6); }

uint32_t crc24q(const uint8_t *buff, const size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(buff[i]) << 16;
    for (int b = 0; b < 8; ++b) {
      crc <<= 1;
      if (crc & 0x1000000)
        crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

// 周内秒 → GPST，周数取离 ref 最近的一周。
gtime_t adjust_week(const gtime_t &ref, double tow) {
  uint32_t week = 0;
  const double ref_tow = time2gpst(ref, &week);
  if (tow < ref_tow - 0.5 * WEEK_SECONDS)
    tow += WEEK_SECONDS;
  else if (tow > ref_tow + 0.5 * WEEK_SECONDS)
    tow -= WEEK_SECONDS;
  return time_add(gpst2time(week, 0), tow);
}

// GLONASS 日内秒（GLONASS 时 = UTC + 3 h）→ GPST，日期取离 ref 最近的一天。
gtime_t adjust_glo_tod(const gtime_t &ref, double tod) {
  const gtime_t ref_glot = time_add(gpst2utc(ref), UTC_GLOT_OFFSET);
  uint32_t week = 0;
  double tow = time2gpst(ref_glot, &week);
  const double ref_tod = std::fmod(tow, 86400.0);
  tow -= ref_tod;
  if (tod < ref_tod - 43200.0)
    tod += 86400.0;
  else if (tod > ref_tod + 43200.0)
    tod -= 86400.0;
  return utc2gpst(time_add(gpst2time(week, 0), tow + tod - UTC_GLOT_OFFSET));
}

class RtcmDecoder {
public:
  RtcmDecoder(const gtime_t &ref_time, std::vector<std::vector<ObsPtr>> &epochs, RtcmStation &station) : time_(ref_time), epochs_(epochs), station_(station) {}

  // buff 从帧头 0xD3 开始，len_bits 为帧头+数据的位数（不含 CRC）。
  void decode(const uint8_t *buff, const uint32_t len_bits) {
    if (len_bits < 24 + 12)
      return;
    const uint32_t type = getbitu(buff, 24, 12);
    const uint32_t group = type / 10, msm = type % 10;
    if (type == 1005 || type == 1006)
      decode_station(buff, len_bits, type == 1006);
    else if (type == 1007 || type == 1008 || type == 1033)
      decode_antenna(buff, len_bits);
    else if ((group == 107 || group == 108 || group == 109 || group == 111 || group == 112) && msm >= 4 && msm <= 7)
      decode_msm(buff, len_bits, group, msm);
  }

  // 把当前历元写入 epochs。
  void flush() {
    if (current_.empty())
      return;
    std::vector<ObsPtr> meas;
    for (const auto &sat_sigs : current_) {
      ObsPtr obs(new Obs());
      obs->time = current_time_;
      obs->sat = sat_sigs.first;
      for (const auto &band_sig : sat_sigs.second) {
        const Signal &sig = band_sig.second;
        obs->freqs.push_back(sig.freq);
        obs->code.push_back(sig.code);
        obs->psr.push_back(sig.psr);
        obs->psr_std.push_back(0);
        obs->cp.push_back(sig.cp);
        obs->cp_std.push_back(0);
        obs->dopp.push_back(0);
        obs->dopp_std.push_back(0);
        obs->cn0.push_back(sig.cn0);
        obs->lli.push_back(sig.lli);
        // 与 RINEX 读取一致：bit0 伪距有效，bit1 载波有效，半周已解决时再置 bit2/bit3。
        obs->status.push_back((sig.psr != 0 ? 0x01 : 0x00) | (sig.cp != 0 ? (sig.half ? 0x02 : 0x0E) : 0x00));
      }
      meas.push_back(obs);
    }
    epochs_.push_back(meas);
    current_.clear();
  }

private:
  struct Signal {
    char attr{};
    int rank{};
    uint8_t code{};
    double freq{}, psr{}, cp{}, cn0{};
    uint8_t lli{};
    bool half{};
  };

  void decode_station(const uint8_t *buff, const uint32_t len_bits, const bool with_height) {
    uint32_t i = 24 + 12;
    if (i + 140 + (with_height ? 16 : 0) > len_bits)
      return;
    station_.station_id = getbitu(buff, i, 12);
    i += 12 + 6 + 4; // 测站 ID、ITRF 年份、GPS/GLO/GAL 指示与参考站指示
    Eigen::Vector3d arp;
    arp.x() = getbits38(buff, i) * 1e-4;
    i += 38 + 2; // 2 位：单接收机振荡器指示、保留
    arp.y() = getbits38(buff, i) * 1e-4;
    i += 38 + 2; // 2 位：保留（1005 为四分之一周指示）
    arp.z() = getbits38(buff, i) * 1e-4;
    i += 38;
    station_.arp_ecef = arp;
    station_.antenna_height_m = with_height ? getbitu(buff, i, 16) * 1e-4 : 0.0;
    station_.has_position = true;
  }

  void decode_antenna(const uint8_t *buff, const uint32_t len_bits) {
    uint32_t i = 24 + 12;
    if (i + 20 > len_bits)
      return;
    station_.station_id = getbitu(buff, i, 12);
    i += 12;
    const uint32_t n = getbitu(buff, i, 8);
    i += 8;
    if (n == 0 || i + 8 * n > len_bits)
      return;
    std::string descriptor;
    for (uint32_t j = 0; j < n; ++j, i += 8)
      descriptor.push_back(static_cast<char>(getbitu(buff, i, 8)));
    station_.antenna_type = descriptor;
  }

  void decode_msm(const uint8_t *buff, const uint32_t len_bits, const uint32_t group, const uint32_t msm) {
    uint32_t sys = SYS_GPS;
    char sys_char = 'G';
    const char *const *sig_table = msm_sig_gps;
    switch (group) {
    case 108: sys = SYS_GLO, sys_char = 'R', sig_table = msm_sig_glo; break;
    case 109: sys = SYS_GAL, sys_char = 'E', sig_table = msm_sig_gal; break;
    case 111: sys = SYS_QZS, sys_char = 'J', sig_table = msm_sig_qzs; break;
    case 112: sys = SYS_BDS, sys_char = 'C', sig_table = msm_sig_bds; break;
    default: break;
    }

    // 头部：测站 ID 12、历元时间 30、多电文标志 1、IODS 3、保留 7、时钟校正 2、外部时钟 2、平滑 1、平滑间隔 3、卫星掩码 64、信号掩码 32
    uint32_t i = 24 + 12;
    if (i + 157 > len_bits)
      return;
    i += 12;
    gtime_t time;
    if (sys == SYS_GLO) {
      i += 3; // 星期
      time = adjust_glo_tod(time_, getbitu(buff, i, 27) * 0.001);
      i += 27;
    } else {
      double tow = getbitu(buff, i, 30) * 0.001;
      i += 30;
      if (sys == SYS_BDS)
        tow += BDT_GPST_OFFSET;
      time = adjust_week(time_, tow);
    }
    const bool more = getbitu(buff, i, 1);
    i += 1 + 3 + 7 + 2 + 2 + 1 + 3;
    std::vector<uint32_t> sats, sigs;
    for (uint32_t j = 1; j <= 64; ++j)
      if (getbitu(buff, i++, 1))
        sats.push_back(j);
    for (uint32_t j = 1; j <= 32; ++j)
      if (getbitu(buff, i++, 1))
        sigs.push_back(j);
    const size_t nsat = sats.size(), nsig = sigs.size();
    if (nsat * nsig > 64 || i + nsat * nsig > len_bits)
      return;
    std::vector<bool> cell_mask(nsat * nsig);
    size_t ncell = 0;
    for (size_t j = 0; j < cell_mask.size(); ++j) {
      cell_mask[j] = getbitu(buff, i++, 1);
      ncell += cell_mask[j];
    }

    const bool ext = msm == 5 || msm == 7; // 含扩展卫星信息与相位距离变化率
    const bool high = msm == 6 || msm == 7; // 高精度伪距/载波与扩展锁定时间
    const uint32_t sat_bits = ext ? 36 : 18;
    const uint32_t cell_bits = (high ? 65 : 48) + (ext ? 15 : 0);
    if (i + nsat * sat_bits + ncell * cell_bits > len_bits)
      return;

    // 卫星数据：粗略距离整毫秒（255 无效）、扩展信息、粗略距离毫秒余数、粗略相位距离变化率
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> rough(nsat, nan);
    std::vector<uint32_t> ex_info(nsat, 15);
    for (size_t j = 0; j < nsat; ++j, i += 8) {
      const uint32_t ms = getbitu(buff, i, 8);
      if (ms != 255)
        rough[j] = ms * RANGE_MS;
    }
    if (ext)
      for (size_t j = 0; j < nsat; ++j, i += 4)
        ex_info[j] = getbitu(buff, i, 4);
    for (size_t j = 0; j < nsat; ++j, i += 10)
      rough[j] += getbitu(buff, i, 10) * P2_10 * RANGE_MS;
    if (ext)
      i += 14 * nsat;

    // 信号数据：每个字段依次覆盖全部单元
    std::vector<double> fine_psr(ncell, nan), fine_cp(ncell, nan), cn0(ncell);
    std::vector<uint32_t> lock(ncell);
    std::vector<bool> half(ncell);
    for (size_t c = 0; c < ncell; ++c) {
      const int32_t v = high ? getbits(buff, i, 20) : getbits(buff, i, 15);
      if (v != (high ? -524288 : -16384))
        fine_psr[c] = v * (high ? P2_29 : P2_24) * RANGE_MS;
      i += high ? 20 : 15;
    }
    for (size_t c = 0; c < ncell; ++c) {
      const int32_t v = high ? getbits(buff, i, 24) : getbits(buff, i, 22);
      if (v != (high ? -8388608 : -2097152))
        fine_cp[c] = v * (high ? P2_31 : P2_29) * RANGE_MS;
      i += high ? 24 : 22;
    }
    for (size_t c = 0; c < ncell; ++c, i += high ? 10 : 4)
      lock[c] = getbitu(buff, i, high ? 10 : 4);
    for (size_t c = 0; c < ncell; ++c, ++i)
      half[c] = getbitu(buff, i, 1);
    for (size_t c = 0; c < ncell; ++c, i += high ? 10 : 6)
      cn0[c] = high ? getbitu(buff, i, 10) * 0.0625 : getbitu(buff, i, 6);

    // 同一历元的其它星座电文时间相同（均已换算为 GPST）；时间变化说明上一历元缺少结束标志。
    if (!current_.empty() && std::abs(time_diff(time, current_time_)) > 1e-3)
      flush();
    current_time_ = time;
    time_ = time;

    size_t c = 0;
    for (size_t j = 0; j < nsat; ++j) {
      const uint32_t prn = sats[j];
      if (sys == SYS_GLO && ext && ex_info[j] <= 13)
        glo_channel_[prn] = static_cast<int>(ex_info[j]) - 7;
      const uint32_t sat = sat_no(sys, prn);
      const auto channel = glo_channel_.find(prn);
      for (size_t k = 0; k < nsig; ++k) {
        if (!cell_mask[j * nsig + k])
          continue;
        const size_t cell = c++;
        const std::string band_attr = sig_table[sigs[k] - 1];
        if (sat == 0 || band_attr.empty() || std::isnan(rough[j]))
          continue;
        const char band = band_attr[0], attr = band_attr[1];
        const auto freq_it = type2freq.find(std::string{sys_char, band});
        if (freq_it == type2freq.end())
          continue;
        double freq = freq_it->second;
        if (sys == SYS_GLO && (band == '1' || band == '2')) {
          if (channel == glo_channel_.end())
            continue;
          freq += channel->second * (band == '1' ? DFRQ1_GLO : DFRQ2_GLO);
        }

        Signal sig;
        sig.attr = attr;
        sig.rank = rinex_attr_rank(sys_char, band, attr);
        sig.code = rinex_obs_code(band_attr);
        sig.freq = freq;
        sig.psr = std::isnan(fine_psr[cell]) ? 0.0 : rough[j] + fine_psr[cell];
        sig.cp = std::isnan(fine_cp[cell]) ? 0.0 : (rough[j] + fine_cp[cell]) * freq / LIGHT_SPEED;
        sig.cn0 = cn0[cell];
        sig.half = half[cell];
        // 锁定时间指示器减小、或连续为 0 视为失锁（同 RTKLIB）；bit1 为半周未解决（同 RINEX LLI）。
        const auto key = std::make_pair(sat, sig.code);
        const auto prev = lock_.find(key);
        const bool slip = prev == lock_.end() ? lock[cell] == 0 : (lock[cell] == 0 && prev->second == 0) || lock[cell] < prev->second;
        lock_[key] = lock[cell];
        sig.lli = (slip ? 0x01 : 0x00) | (sig.half ? 0x02 : 0x00);

        auto &bands = current_[sat];
        const auto it = bands.find(band);
        if (it == bands.end() || sig.rank < it->second.rank)
          bands[band] = sig;
      }
    }
    if (!more)
      flush();
  }

  gtime_t time_; // 最近一次解出的 MSM 历元，用于推算下一电文的周数/日期
  std::vector<std::vector<ObsPtr>> &epochs_;
  RtcmStation &station_;
  gtime_t current_time_{};
  std::map<uint32_t, std::map<char, Signal>> current_; // 当前历元：卫星 → 频段 → 选中信号
  std::map<std::pair<uint32_t, uint8_t>, uint32_t> lock_; // (卫星, CODE_*) → 上一次的锁定时间指示器
  std::map<uint32_t, int> glo_channel_;                   // GLONASS 槽号 → 频道号
};

} // namespace

bool rtcm2obs(const std::string &rtcm_filepath, const gtime_t &ref_time, std::vector<std::vector<ObsPtr>> &epochs, RtcmStation &station) {
  std::ifstream file(rtcm_filepath, std::ios::binary);
  if (!file) {
    LOG(ERROR) << "Cannot open RTCM file " << rtcm_filepath;
    return false;
  }
  const std::vector<uint8_t> buf((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  RtcmDecoder decoder(ref_time, epochs, station);
  const size_t before = epochs.size();
  size_t frames = 0, crc_errors = 0;
  size_t i = 0;
  // 帧结构：0xD3、6 位保留 + 10 位长度、数据、CRC-24Q
  while (i + 6 <= buf.size()) {
    if (buf[i] != 0xD3 || (buf[i + 1] & 0xFC) != 0) {
      ++i;
      continue;
    }
    const size_t len = (static_cast<size_t>(buf[i + 1] & 0x03) << 8) | buf[i + 2];
    if (i + len + 6 > buf.size())
      break;
    if (crc24q(&buf[i], len + 3) != getbitu(&buf[i], static_cast<uint32_t>((len + 3) * 8), 24)) {
      ++crc_errors;
      ++i;
      continue;
    }
    decoder.decode(&buf[i], static_cast<uint32_t>((len + 3) * 8));
    ++frames;
    i += len + 6;
  }
  decoder.flush();
  LOG(INFO) << "RTCM " << rtcm_filepath << ": " << frames << " frames, " << epochs.size() - before << " epochs, " << crc_errors << " CRC mismatches";
  return epochs.size() > before;
}

} // namespace gnss_comm
