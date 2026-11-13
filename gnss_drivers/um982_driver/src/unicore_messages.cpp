#include "um982_driver/unicore_messages.hpp"
#include <gnss_comm/constants/system.hpp>
#include <gnss_comm/constants/observation.hpp>
#include <gnss_comm/constants/frequency.hpp>
#include <gnss_comm/utility/satellite_utility.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
using Fields = std::vector<std::string>;
constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();
Fields split(const std::string& text)
{
    Fields fields;
    std::stringstream stream(text);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}
double number(const Fields& f, size_t i) { return std::stod(f.at(i)); }
unsigned integer(const Fields& f, size_t i) { return std::stoul(f.at(i)); }
bool flag(const Fields& f, size_t i) { return f.at(i) == "TRUE" || f.at(i) == "1"; }
void require(const Fields& f, size_t n)
{
    if (f.size() < n) throw std::runtime_error("incomplete Unicore payload");
}
gnss_msg::msg::GnssTimeMsg gps_time(int week, double tow)
{
    while (tow < 0) { --week; tow += 604800; }
    while (tow >= 604800) { ++week; tow -= 604800; }
    gnss_msg::msg::GnssTimeMsg result;
    result.week = week;
    result.tow = tow;
    return result;
}
gnss_msg::msg::GnssTimeMsg near_time(const gnss_msg::msg::GnssTimeMsg& reference, double tow)
{
    int week = reference.week;
    if (tow - reference.tow > 302400) --week;
    if (tow - reference.tow < -302400) ++week;
    return gps_time(week, tow);
}
struct Signal { double frequency; uint8_t code; };
Signal signal(unsigned system, unsigned id, int channel, uint32_t state)
{
    if (system == SYS_GPS || system == SYS_QZS)
    {
        switch (id)
        {
            case 0: return {FREQ1, CODE_L1C};
            case 1: if (system == SYS_QZS) return {FREQ1, CODE_L1Z}; break;
            case 3: return {FREQ1, CODE_L1L};
            case 4: if (system == SYS_QZS) return {FREQ1, CODE_L1E}; break;
            case 11: return {FREQ1, CODE_L1S};
            case 6: return {FREQ5, CODE_L5I};
            case 14: return {FREQ5, CODE_L5Q};
            case 9: return {FREQ2, static_cast<uint8_t>((state & (1u << 26)) ? CODE_L2L : CODE_L2W)};
            case 17: return {FREQ2, CODE_L2L};
            case 21: if (system == SYS_QZS) return {FREQ6, CODE_L6S}; break;
            case 27: if (system == SYS_QZS) return {FREQ6, CODE_L6L}; break;
        }
    }
    if (system == SYS_GLO)
    {
        switch (id)
        {
            case 0: return {FREQ1_GLO + channel * DFRQ1_GLO, CODE_L1C};
            case 5: return {FREQ2_GLO + channel * DFRQ2_GLO, CODE_L2C};
            case 6: return {FREQ3_GLO, CODE_L3I};
            case 7: return {FREQ3_GLO, CODE_L3Q};
        }
    }
    if (system == SYS_GAL)
    {
        switch (id)
        {
            case 1: return {FREQ1, CODE_L1B};
            case 2: return {FREQ1, CODE_L1C};
            case 12: return {FREQ5, CODE_L5Q};
            case 17: return {FREQ7, CODE_L7Q};
            case 18: return {FREQ6, CODE_L6B};
            case 22: return {FREQ6, CODE_L6C};
        }
    }
    if (system == SYS_BDS)
    {
        switch (id)
        {
            case 0: return {FREQ1_BDS, CODE_L2I};
            case 4: return {FREQ1_BDS, CODE_L2Q};
            case 17: return {FREQ2_BDS, CODE_L7I};
            case 5: return {FREQ2_BDS, CODE_L7Q};
            case 21: return {FREQ3_BDS, CODE_L6I};
            case 6: return {FREQ3_BDS, CODE_L6Q};
            // The project's CODE_* set has no B1C/B2a/B2b codes. Preserve the
            // measured frequency with CODE_NONE rather than mislabel the signal.
            case 8: case 23: return {FREQ1, CODE_NONE};
            case 12: case 28: return {FREQ5, CODE_NONE};
            case 13: return {FREQ7, CODE_NONE};
        }
    }
    return {0, CODE_NONE};
}
double sisa(unsigned index)
{
    if (index <= 49) return index * 0.01;
    if (index <= 74) return 0.5 + (index - 50) * 0.02;
    if (index <= 99) return 1.0 + (index - 75) * 0.04;
    if (index <= 125) return 2.0 + (index - 100) * 0.16;
    return kUnknown;
}
}

UnicoreMessages::UnicoreMessages(rclcpp::Node& node, const std::string& frame)
    : node_(node), frame_(frame)
{
    obs_ = node.create_publisher<gnss_msg::msg::GnssMeasMsg>("~/obs", 10);
    const auto nav_qos = rclcpp::QoS(256).reliable().transient_local();
    ephem_ = node.create_publisher<gnss_msg::msg::GnssEphemMsg>("~/ephem", nav_qos);
    glo_ephem_ = node.create_publisher<gnss_msg::msg::GnssGloEphemMsg>("~/glo_ephem", nav_qos);
    for (const auto& entry : std::map<std::string, std::string>{
        {"GPSIONA", "gps"}, {"BDSIONA", "bds"}, {"BD3IONA", "bd3"}, {"GALIONA", "gal"}})
        iono_[entry.first] = node.create_publisher<gnss_msg::msg::StampedFloat64Array>(
            "~/iono/" + entry.second, rclcpp::QoS(1).reliable().transient_local());
}

void UnicoreMessages::parse(const std::string& line)
{
    try
    {
        const auto star = line.find('*');
        const auto separator = line.find(';');
        if (line.empty() || line.front() != '#' || star == std::string::npos ||
            separator == std::string::npos || separator > star || star + 9 != line.size()) return;
        uint32_t crc = 0;
        for (size_t i = 1; i < star; ++i)
        {
            crc ^= static_cast<unsigned char>(line[i]);
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
        }
        if (crc != std::stoul(line.substr(star + 1), nullptr, 16))
            throw std::runtime_error("Unicore CRC32 mismatch");
        const auto h = split(line.substr(1, separator - 1));
        require(h, 10);
        // The N4 ASCII header reports GPST week and milliseconds, even for BDS logs.
        if (h[3] != "FINE") return;
        if (h[2] != "GPS") throw std::runtime_error("Only GPS TimeRef is supported; configure GPST output");
        const auto time = gps_time(integer(h, 4), number(h, 5) * 0.001);
        const auto fields = split(line.substr(separator + 1, star - separator - 1));
        if (h[0] == "OBSVMA") observations(fields, time);
        else if (iono_.count(h[0])) ionosphere(h[0], fields, h);
        else if (h[0] == "GPSEPHA" || h[0] == "BDSEPHA" || h[0] == "BD3EPHA" ||
                 h[0] == "GALEPHA" || h[0] == "GLOEPHA" || h[0] == "QZSSEPHA")
            ephemeris(h[0], fields, time);
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                            "Unicore log skipped: %s", e.what());
    }
}

void UnicoreMessages::observations(const Fields& f, const gnss_msg::msg::GnssTimeMsg& time)
{
    require(f, 1);
    const unsigned count = integer(f, 0);
    if (count > 2048 || f.size() != 1 + count * 11) throw std::runtime_error("OBSVM record count mismatch");
    gnss_msg::msg::GnssMeasMsg epoch;
    std::map<unsigned, size_t> satellites;
    const double stamp = time.week * 604800.0 + time.tow;
    for (unsigned record = 0; record < count; ++record)
    {
        const size_t i = 1 + record * 11;
        const uint32_t state = std::stoul(f.at(i + 10), nullptr, 16);
        const unsigned system_id = (state >> 16) & 7;
        static constexpr unsigned systems[] = {SYS_GPS, SYS_GLO, SYS_SBS, SYS_GAL, SYS_BDS, SYS_QZS, SYS_IRN, SYS_NONE};
        const unsigned system = systems[system_id];
        unsigned prn = integer(f, i + 1);
        if (system == SYS_GLO) prn = prn >= 38 ? prn - 37 : 0;
        if (system == SYS_QZS) prn = prn >= 193 ? prn - 192 : 0;
        const unsigned sat = gnss_comm::sat_no(system, prn);
        const unsigned signal_id = (state >> 21) & 31;
        const auto sig = signal(system, signal_id, static_cast<int>(integer(f, i)) - 7, state);
        if (!sat || sig.frequency == 0)
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10000,
                                "Unsupported observation system=%u PRN=%u signal=%u", system_id, prn, signal_id);
            continue;
        }
        const bool psr_valid = state & (1u << 12);
        const bool cp_valid = state & (1u << 10);
        if (!psr_valid && !cp_valid) continue;
        if (!satellites.count(sat))
        {
            satellites[sat] = epoch.meas.size();
            gnss_msg::msg::GnssObsMsg obs;
            obs.sat = sat;
            obs.time = time;
            epoch.meas.push_back(obs);
        }
        auto& obs = epoch.meas[satellites.at(sat)];
        const double duration = number(f, i + 9);
        const auto key = std::make_pair(sat, signal_id);
        const auto previous = locks_.find(key);
        bool slip = !cp_valid;
        if (previous != locks_.end())
        {
            const double dt = stamp - previous->second.time;
            slip = slip || !previous->second.valid || dt <= 0 ||
                duration < previous->second.duration || duration + 0.05 < dt;
        }
        locks_[key] = {stamp, duration, cp_valid};
        obs.freqs.push_back(sig.frequency);
        obs.code.push_back(sig.code);
        obs.cn0.push_back(number(f, i + 7) * 0.01);
        obs.lli.push_back(slip ? 1 : 0);
        obs.psr.push_back(psr_valid ? number(f, i + 2) : kUnknown);
        obs.psr_std.push_back(number(f, i + 4) * 0.01);
        // Unicore ADR uses the opposite sign to RINEX/UBX carrier phase.
        obs.cp.push_back(cp_valid ? -number(f, i + 3) : kUnknown);
        obs.cp_std.push_back(number(f, i + 5) * 0.0001);
        obs.dopp.push_back(number(f, i + 6));
        obs.dopp_std.push_back(kUnknown); // OBSVM does not provide Doppler uncertainty.
        obs.status.push_back((psr_valid ? 1 : 0) | (cp_valid ? 2 : 0));
    }
    if (!epoch.meas.empty()) obs_->publish(epoch);
}

void UnicoreMessages::ionosphere(const std::string& name, const Fields& f, const Fields& h)
{
    const size_t size = name == "BD3IONA" ? 9 : 8;
    require(f, size);
    gnss_msg::msg::StampedFloat64Array message;
    message.header.frame_id = frame_;
    // Receiver GPST converted to UTC using the leap seconds in this log header.
    const double unix_seconds = 315964800.0 + integer(h, 4) * 604800.0 +
        number(h, 5) * 0.001 - number(h, 8);
    message.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(unix_seconds * 1e9)));
    // GPS/BDS: alpha0..3,beta0..3. BD3: BDGIM A1..9.
    // GAL: ai0..2,SF1..5 (NeQuick); never feed these to a Klobuchar solver.
    for (size_t i = 0; i < size; ++i) message.data.push_back(number(f, i));
    iono_.at(name)->publish(message);
}

void UnicoreMessages::ephemeris(const std::string& name, const Fields& f,
                                const gnss_msg::msg::GnssTimeMsg& time)
{
    if (name == "GLOEPHA")
    {
        require(f, 29);
        gnss_msg::msg::GnssGloEphemMsg e;
        const unsigned slot = integer(f, 0);
        e.sat = gnss_comm::sat_no(SYS_GLO, slot >= 38 ? slot - 37 : 0);
        e.freqo = static_cast<int>(integer(f, 1)) - 7;
        e.ttr = time;
        e.toe = gps_time(integer(f, 4), number(f, 5) * 0.001);
        e.iode = integer(f, 10); e.health = integer(f, 11);
        e.pos_x = number(f, 12); e.pos_y = number(f, 13); e.pos_z = number(f, 14);
        e.vel_x = number(f, 15); e.vel_y = number(f, 16); e.vel_z = number(f, 17);
        e.acc_x = number(f, 18); e.acc_y = number(f, 19); e.acc_z = number(f, 20);
        e.tau_n = number(f, 21); e.delta_tau_n = number(f, 22); e.gamma = number(f, 23);
        static constexpr double accuracy[] = {1, 2, 2.5, 4, 5, 7, 10, 12, 14, 16, 32, 64, 128, 256, 512};
        const unsigned ft = integer(f, 26);
        e.ura = ft < 15 ? accuracy[ft] : kUnknown;
        e.age = integer(f, 27);
        if (e.sat) glo_ephem_->publish(e);
        return;
    }
    gnss_msg::msg::GnssEphemMsg e;
    e.ttr = time;
    if (name == "GALEPHA")
    {
        require(f, 38);
        e.sat = gnss_comm::sat_no(SYS_GAL, integer(f, 0));
        const bool inav = flag(f, 2), fnav = flag(f, 1);
        if (!inav && !fnav) return;
        e.code = inav ? 517 : 258; // RINEX Galileo data-source bits, including clock reference.
        // RINEX/RTKLIB Galileo SV health: E1 DVS/HS, E5a DVS/HS, E5b DVS/HS.
        e.health = integer(f, 6) | (integer(f, 3) << 1) |
                   (integer(f, 7) << 3) | (integer(f, 4) << 4) |
                   (integer(f, 8) << 6) | (integer(f, 5) << 7);
        e.ura = sisa(integer(f, 9));
        e.iode = e.iodc = integer(f, 11);
        e.toe = near_time(time, number(f, 12));
        e.a = std::pow(number(f, 13), 2);
        e.delta_n = number(f, 14); e.m0 = number(f, 15); e.e = number(f, 16); e.omg = number(f, 17);
        e.cuc = number(f, 18); e.cus = number(f, 19); e.crc = number(f, 20); e.crs = number(f, 21);
        e.cic = number(f, 22); e.cis = number(f, 23); e.i0 = number(f, 24); e.i_dot = number(f, 25);
        e.omg0 = number(f, 26); e.omg_dot = number(f, 27);
        const size_t clock = inav ? 32 : 28;
        e.toc = near_time(e.toe, number(f, clock));
        e.af0 = number(f, clock + 1); e.af1 = number(f, clock + 2); e.af2 = number(f, clock + 3);
        e.tgd0 = number(f, 36); e.tgd1 = number(f, 37);
    }
    else if (name == "BD3EPHA")
    {
        // Build7160/7676 omit the two B2b TGD fields (N4 table 7-66).
        if (f.size() != 43 && f.size() != 45)
            throw std::runtime_error("Unsupported BD3EPH field count");
        const size_t clock = f.size() == 43 ? 32 : 34;
        e.sat = gnss_comm::sat_no(SYS_BDS, integer(f, 0));
        e.health = integer(f, 1); e.iode = integer(f, 4); e.iodc = integer(f, 5);
        e.toe = gps_time(integer(f, 7), number(f, 9));
        e.ttr = gps_time(integer(f, 6), number(f, 8));
        const unsigned type = integer(f, 2);
        if (type < 1 || type > 3) throw std::runtime_error("Unknown BD3 satellite type");
        // BDS B1C/B2a ICD: A = Aref + DeltaA, not sqrt(A).
        e.a = (type == 3 ? 27906100.0 : 42162200.0) + number(f, 10);
        e.a_dot = number(f, 11); e.delta_n = number(f, 12); e.n_dot = number(f, 13);
        e.m0 = number(f, 14); e.e = number(f, 15); e.omg = number(f, 16);
        e.cuc = number(f, 17); e.cus = number(f, 18); e.crc = number(f, 19); e.crs = number(f, 20);
        e.cic = number(f, 21); e.cis = number(f, 22); e.i0 = number(f, 23); e.i_dot = number(f, 24);
        e.omg0 = number(f, 25); e.omg_dot = number(f, 26); e.toc = near_time(e.toe, number(f, 27));
        e.tgd0 = number(f, 28); e.tgd1 = number(f, 29); // B1Cp/B2ap, not legacy B1I/B2I TGD.
        e.af0 = number(f, clock); e.af1 = number(f, clock + 1); e.af2 = number(f, clock + 2);
        e.code = 0x100u | integer(f, f.size() - 1); // Driver source tag: CNAV + FreqType (0=B1C,1=B2a,2=B2b).
        e.ura = kUnknown; // SISMAI/SISAI are indices, not a directly supplied range sigma.
    }
    else
    {
        const bool bds = name == "BDSEPHA";
        require(f, bds ? 33 : 32);
        unsigned prn = integer(f, 0);
        unsigned system = bds ? SYS_BDS : (name == "QZSSEPHA" ? SYS_QZS : SYS_GPS);
        if (name == "GPSEPHA" && prn >= 33 && prn <= 42) { system = SYS_QZS; prn -= 32; }
        e.sat = gnss_comm::sat_no(system, prn);
        e.health = integer(f, 2); e.iode = integer(f, 3);
        // N4 already converts BDS weeks/times to GPST: do not add 1356 weeks/14 seconds.
        e.toe = gps_time(integer(f, 5), number(f, 7));
        e.ttr = near_time(time, number(f, 1));
        e.a = number(f, 8); e.delta_n = number(f, 9); e.m0 = number(f, 10);
        e.e = number(f, 11); e.omg = number(f, 12);
        e.cuc = number(f, 13); e.cus = number(f, 14); e.crc = number(f, 15); e.crs = number(f, 16);
        e.cic = number(f, 17); e.cis = number(f, 18); e.i0 = number(f, 19); e.i_dot = number(f, 20);
        e.omg0 = number(f, 21); e.omg_dot = number(f, 22); e.iodc = integer(f, 23);
        e.toc = near_time(e.toe, number(f, 24)); e.tgd0 = number(f, 25);
        const size_t clock = bds ? 27 : 26;
        e.tgd1 = bds ? number(f, 26) : 0;
        e.af0 = number(f, clock); e.af1 = number(f, clock + 1); e.af2 = number(f, clock + 2);
        e.ura = std::sqrt(number(f, bds ? 32 : 31)); // N4 supplies variance; gnss_msg expects sigma.
    }
    e.week = e.toe.week;
    e.toe_tow = e.toe.tow;
    if (e.sat) ephem_->publish(e);
}
