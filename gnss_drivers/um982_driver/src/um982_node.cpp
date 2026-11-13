#include <rclcpp/rclcpp.hpp>
#include "um982_driver/unicore_messages.hpp"
#include "um982_driver/rtcm_writer.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>
#include <curl/curl.h>
#include <poll.h>
#include <cerrno>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class Um982 : public rclcpp::Node
{
public:
    Um982() : Node("um982_driver")
    {
        const auto path = declare_parameter<std::string>("config_file",
                                                         ament_index_cpp::get_package_share_directory("um982_driver") +
                                                         "/config/serial_setting.yaml");
        RCLCPP_INFO(get_logger(), "Loading config: %s", path.c_str());
        config_ = YAML::LoadFile(path);
        frame_ = config_["frame_id"].as<std::string>("gnss");
        unicore_ = std::make_unique<UnicoreMessages>(*this, frame_);
        gga_pub_ = create_publisher<std_msgs::msg::String>("~/gga", 10);
        rmc_pub_ = create_publisher<std_msgs::msg::String>("~/rmc", 10);
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 10);
        port_.setPort(config_["serial_port"].as<std::string>());
        port_.setBaudrate(config_["baud_rate"].as<unsigned>());
        auto timeout = serial::Timeout::simpleTimeout(200);
        port_.setTimeout(timeout);
        port_.open();
        for (const auto& command : config_["um982_cfg_cmd"])
        {
            write(command.as<std::string>() + "\r\n");
            std::this_thread::sleep_for(100ms);
        }
        timer_ = create_wall_timer(10ms, [this] { read(); });
        if (config_["ntrip_module"] && config_["ntrip_module"]["enabled"].as<bool>(false))
        {
            const auto n = config_["ntrip_module"];
            const auto host = n["server_address"].as<std::string>();
            auto mountpoint = n["amount_point"].as<std::string>();
            if (!mountpoint.empty() && mountpoint.front() == '/') mountpoint.erase(0, 1);
            authority_ = host + ":" + std::to_string(n["port"].as<unsigned>());
            mountpoint_ = mountpoint;
            url_ = "http://" + authority_ + "/" + mountpoint_;
            username_ = n["username"].as<std::string>("");
            password_ = n["password"].as<std::string>("");
            const auto recording = n["recording"];
            if (recording && recording["enabled"].as<bool>(false))
                rtcm_writer_ = std::make_unique<RtcmWriter>(*this,
                    recording["directory"].as<std::string>("rtcm_logs"));
            ntrip_ = std::thread([this] { corrections(); });
        }
        else
        {
            RCLCPP_INFO(get_logger(), "NTRIP disabled: ntrip_module.enabled is false or missing");
        }
        RCLCPP_INFO(get_logger(), "UM982 serial open: %s", port_.getPort().c_str());
    }

    ~Um982() override
    {
        stop_ = true;
        if (ntrip_.joinable()) ntrip_.join();
        // Stop the producer first, then drain and flush the recording queue.
        rtcm_writer_.reset();
    }

private:
    void write(const std::string& data)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (port_.write(data) != data.size()) throw std::runtime_error("Short serial write");
    }

    void read()
    {
        try
        {
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                const auto count = port_.available();
                if (count) buffer_ += port_.read(count);
            }
            size_t end;
            while ((end = buffer_.find('\n')) != std::string::npos)
            {
                auto line = buffer_.substr(0, end);
                buffer_.erase(0, end + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                try { publish(line); }
                catch (const std::exception&)
                {
                    /* Ignore incomplete numeric NMEA fields. */
                }
            }
            // A multi-frequency OBSVMA epoch can exceed 16 KiB.
            if (buffer_.size() > 262144) buffer_.clear();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Serial: %s", e.what());
        }
    }

    static double degrees(const std::string& value, const std::string& hemisphere)
    {
        const double v = std::stod(value);
        const double d = std::floor(v / 100.0);
        return (d + (v - d * 100.0) / 60.0) *
            ((hemisphere == "S" || hemisphere == "W") ? -1.0 : 1.0);
    }

    void publish(const std::string& line)
    {
        if (!line.empty() && line.front() == '#')
        {
            unicore_->parse(line);
            return;
        }
        if (line.size() < 10 || line.front() != '$') return;
        const auto star = line.find('*');
        if (star == std::string::npos || star + 3 != line.size()) return;
        unsigned checksum = 0;
        for (size_t i = 1; i < star; ++i) checksum ^= static_cast<unsigned char>(line[i]);
        if (checksum != std::stoul(line.substr(star + 1), nullptr, 16)) return;
        std::vector<std::string> fields;
        std::stringstream stream(line.substr(0, star));
        std::string field;
        while (std::getline(stream, field, ',')) fields.push_back(field);
        const auto type = fields[0].substr(3);
        std_msgs::msg::String raw;
        raw.data = line;
        if (type == "RMC")
        {
            rmc_pub_->publish(raw);
            return;
        }
        if (type != "GGA" || fields.size() < 13) return;
        gga_pub_->publish(raw);
        const int quality = std::stoi(fields[6]);
        if (quality > 0 && quality <= 5)
        {
            std::lock_guard<std::mutex> lock(gga_mutex_);
            gga_ = line;
            gga_time_ = std::chrono::steady_clock::now();
        }
        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = now(); // ROS receipt time; receiver UTC remains in raw NMEA.
        fix.header.frame_id = frame_;
        fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
            sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
            sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
            sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        fix.latitude = fix.longitude = fix.altitude = std::numeric_limits<double>::quiet_NaN();
        if (quality > 0 && quality <= 5)
        {
            fix.latitude = degrees(fields[2], fields[3]);
            fix.longitude = degrees(fields[4], fields[5]);
            if (!fields[9].empty() && !fields[11].empty())
                fix.altitude = std::stod(fields[9]) + std::stod(fields[11]);
            fix.status.status = quality == 1
                                    ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                    : sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        }
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        fix_pub_->publish(fix);
    }

    static size_t receive(char* data, size_t size, size_t count, void* context)
    {
        auto* self = static_cast<Um982*>(context);
        try
        {
            self->received_bytes_ += size * count;
            auto& buffer = self->rtcm_;
            buffer.append(data, size * count);
            while (!buffer.empty())
            {
                const auto start = buffer.find(static_cast<char>(0xd3));
                if (start == std::string::npos)
                {
                    buffer.clear();
                    break;
                }
                buffer.erase(0, start);
                if (buffer.size() < 3) break;
                const auto b1 = static_cast<unsigned char>(buffer[1]);
                if (b1 & 0xfc)
                {
                    buffer.erase(0, 1);
                    continue;
                }
                const size_t length = ((b1 & 3) << 8) +
                    static_cast<unsigned char>(buffer[2]) + 6;
                if (buffer.size() < length) break;
                unsigned crc = 0;
                for (size_t i = 0; i < length; ++i)
                {
                    crc ^= static_cast<unsigned char>(buffer[i]) << 16;
                    for (int bit = 0; bit < 8; ++bit)
                    {
                        crc <<= 1;
                        if (crc & 0x1000000) crc ^= 0x1864cfb;
                    }
                }
                if (crc == 0)
                {
                    ++self->received_frames_;
                    const unsigned type = length >= 8 ?
                        (static_cast<unsigned char>(buffer[3]) << 4) |
                        (static_cast<unsigned char>(buffer[4]) >> 4) : 0;
                    if (self->received_frames_ == 1)
                        RCLCPP_INFO(self->get_logger(), "First valid RTCM3 received: type=%u, bytes=%zu", type, length);
                    if (self->rtcm_writer_)
                        self->rtcm_writer_->enqueue(buffer.substr(0, length), self->ntrip_session_, type);
                    self->write(buffer.substr(0, length));
                    ++self->written_frames_;
                    self->written_bytes_ += length;
                    const auto now = std::chrono::steady_clock::now();
                    if (self->written_frames_ == 1 || now - self->last_rtcm_log_ >= 5s)
                    {
                        RCLCPP_INFO(self->get_logger(),
                            "RTCM stream: received=%zu bytes, valid=%zu frames; serial write OK=%zu frames/%zu bytes, latest type=%u",
                            self->received_bytes_, self->received_frames_, self->written_frames_, self->written_bytes_, type);
                        self->last_rtcm_log_ = now;
                    }
                    buffer.erase(0, length);
                }
                else buffer.erase(0, 1);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(self->get_logger(), "RTCM serial write: %s", e.what());
            return 0;
        }
        return size * count;
    }

    std::string latest_gga()
    {
        std::lock_guard<std::mutex> lock(gga_mutex_);
        if (gga_.empty() || std::chrono::steady_clock::now() - gga_time_ >= 10s) return {};
        return gga_ + "\r\n";
    }

    static std::string base64(const std::string& input)
    {
        static constexpr char alphabet[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string output;
        unsigned value = 0;
        int bits = -6;
        for (unsigned char c : input)
        {
            value = (value << 8) | c;
            bits += 8;
            while (bits >= 0)
            {
                output += alphabet[(value >> bits) & 63];
                bits -= 6;
            }
        }
        if (bits > -6) output += alphabet[((value << 8) >> (bits + 8)) & 63];
        while (output.size() % 4) output += '=';
        return output;
    }

    void send_ntrip(CURL* connection, curl_socket_t socket, const std::string& data)
    {
        size_t offset = 0;
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (offset < data.size())
        {
            if (stop_ || !rclcpp::ok()) throw std::runtime_error("NTRIP stopped");
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("NTRIP send timeout");
            size_t sent = 0;
            const auto result = curl_easy_send(connection, data.data() + offset,
                                               data.size() - offset, &sent);
            if (result == CURLE_AGAIN)
            {
                pollfd descriptor{socket, POLLOUT, 0};
                if (poll(&descriptor, 1, 200) < 0 && errno != EINTR)
                    throw std::runtime_error("NTRIP socket poll failed");
                continue;
            }
            if (result != CURLE_OK) throw std::runtime_error(curl_easy_strerror(result));
            if (sent == 0) throw std::runtime_error("NTRIP socket closed during send");
            offset += sent;
        }
    }

    void corrections()
    {
        // Use libcurl only to establish the transport (including configured proxies).
        // Parse the v1 response ourselves and send GGA on the same connection.
        while (!stop_ && rclcpp::ok())
        {
            if (latest_gga().empty())
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                    "NTRIP waiting for a fresh valid GGA");
                std::this_thread::sleep_for(200ms);
                continue;
            }
            rtcm_.clear();
            ++ntrip_session_;
            received_bytes_ = received_frames_ = written_frames_ = written_bytes_ = 0;
            std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> connection(
                curl_easy_init(), &curl_easy_cleanup);
            if (!connection)
            {
                RCLCPP_ERROR(get_logger(), "NTRIP curl initialization failed");
                return;
            }
            auto* curl = connection.get();
            try
            {
                RCLCPP_INFO(get_logger(), "NTRIP connecting: %s", url_.c_str());
                curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
                curl_easy_setopt(curl, CURLOPT_CONNECT_ONLY, 1L);
                curl_easy_setopt(curl, CURLOPT_HTTPPROXYTUNNEL, 1L);
                curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
                curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
                const auto result = curl_easy_perform(curl);
                if (result != CURLE_OK) throw std::runtime_error(curl_easy_strerror(result));
                curl_socket_t socket = CURL_SOCKET_BAD;
                curl_easy_getinfo(curl, CURLINFO_ACTIVESOCKET, &socket);
                if (socket == CURL_SOCKET_BAD) throw std::runtime_error("NTRIP has no connected socket");
                send_ntrip(curl, socket,
                    "GET /" + mountpoint_ + " HTTP/1.0\r\nHost: " + authority_ +
                    "\r\nUser-Agent: NTRIP um982_driver/1.0\r\nAuthorization: Basic " +
                    base64(username_ + ":" + password_) + "\r\n\r\n");

                std::string response;
                bool accepted = false;
                bool first_gga = true;
                const auto handshake_start = std::chrono::steady_clock::now();
                auto last_valid_rtcm = handshake_start;
                auto last_gga = handshake_start - 1s;
                char data[8192];
                while (!stop_ && rclcpp::ok())
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (!accepted && now - handshake_start > 10s)
                        throw std::runtime_error("NTRIP response timeout");
                    if (accepted)
                    {
                        if (now - last_gga >= 1s)
                        {
                            const auto gga = latest_gga();
                            if (!gga.empty())
                            {
                                send_ntrip(curl, socket, gga);
                                if (first_gga)
                                    RCLCPP_INFO(get_logger(), "NTRIP GGA sent on connected socket; updating every 1 s");
                                else
                                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                                        "NTRIP GGA update sent");
                                first_gga = false;
                            }
                            else
                                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                                    "NTRIP has no fresh GGA to upload");
                            last_gga = now;
                        }
                        if (now - last_valid_rtcm > 60s)
                            throw std::runtime_error("NTRIP no valid RTCM3 for 60 s");
                    }
                    size_t count = 0;
                    const auto received = curl_easy_recv(curl, data, sizeof(data), &count);
                    if (received == CURLE_AGAIN)
                    {
                        pollfd descriptor{socket, POLLIN, 0};
                        if (poll(&descriptor, 1, 200) < 0 && errno != EINTR)
                            throw std::runtime_error("NTRIP socket poll failed");
                        continue;
                    }
                    if (received != CURLE_OK) throw std::runtime_error(curl_easy_strerror(received));
                    if (count == 0) throw std::runtime_error("NTRIP disconnected by server");
                    if (!accepted)
                    {
                        response.append(data, count);
                        const auto end_line = response.find("\r\n");
                        if (end_line == std::string::npos)
                        {
                            if (response.size() > 16384) throw std::runtime_error("NTRIP response too large");
                            continue;
                        }
                        std::istringstream status(response.substr(0, end_line));
                        std::string protocol;
                        int code = 0;
                        status >> protocol >> code;
                        if ((protocol != "ICY" && protocol.rfind("HTTP/", 0) != 0) || code != 200)
                            throw std::runtime_error("NTRIP rejected request: " + protocol + " " + std::to_string(code));
                        size_t body = end_line + 2;
                        if (protocol != "ICY")
                        {
                            const auto end_headers = response.find("\r\n\r\n");
                            if (end_headers == std::string::npos)
                            {
                                if (response.size() > 16384) throw std::runtime_error("NTRIP headers too large");
                                continue;
                            }
                            body = end_headers + 4;
                            // This is a v1 request; chunked v2 streams need a separate decoder.
                            std::string headers = response.substr(0, body);
                            for (auto& c : headers) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                            if (headers.find("transfer-encoding: chunked") != std::string::npos)
                                throw std::runtime_error("NTRIP chunked HTTP stream unsupported; use a v1 mountpoint");
                        }
                        accepted = true;
                        last_valid_rtcm = std::chrono::steady_clock::now();
                        RCLCPP_INFO(get_logger(), "NTRIP response: %s %d; connected", protocol.c_str(), code);
                        // A legacy ICY response ends after its first CRLF. Optional text
                        // headers/blank lines are skipped by the RTCM3 frame synchronizer.
                        const auto gga = latest_gga();
                        if (!gga.empty())
                        {
                            send_ntrip(curl, socket, gga);
                            first_gga = false;
                            last_gga = std::chrono::steady_clock::now();
                            RCLCPP_INFO(get_logger(), "NTRIP initial GGA sent; updating every 1 s");
                        }
                        if (response.size() > body &&
                            receive(response.data() + body, 1, response.size() - body, this) != response.size() - body)
                            throw std::runtime_error("RTCM serial forwarding failed");
                        response.clear();
                    }
                    else
                    {
                        const auto frames_before = received_frames_;
                        if (receive(data, 1, count, this) != count)
                            throw std::runtime_error("RTCM serial forwarding failed");
                        if (received_frames_ != frames_before) last_valid_rtcm = std::chrono::steady_clock::now();
                    }
                }
            }
            catch (const std::exception& e)
            {
                if (!stop_ && rclcpp::ok())
                    RCLCPP_WARN(get_logger(), "NTRIP: %s; received=%zu bytes/%zu valid frames, serial=%zu frames/%zu bytes; retry in 2 s",
                                e.what(), received_bytes_, received_frames_, written_frames_, written_bytes_);
            }
            connection.reset();
            for (int i = 0; i < 20 && !stop_ && rclcpp::ok(); ++i) std::this_thread::sleep_for(100ms);
        }
    }

    YAML::Node config_;
    std::unique_ptr<UnicoreMessages> unicore_;
    std::unique_ptr<RtcmWriter> rtcm_writer_;
    serial::Serial port_;
    std::mutex serial_mutex_, gga_mutex_;
    std::string frame_, buffer_, rtcm_, gga_, url_, authority_, mountpoint_, username_, password_;
    // NTRIP-thread-only counters, reset for each connection.
    size_t received_bytes_{0}, received_frames_{0}, written_frames_{0}, written_bytes_{0};
    uint64_t ntrip_session_{0};
    std::chrono::steady_clock::time_point last_rtcm_log_;
    std::chrono::steady_clock::time_point gga_time_;
    std::atomic<bool> stop_{false};
    std::thread ntrip_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gga_pub_, rmc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) return 1;
    int result = 0;
    try { rclcpp::spin(std::make_shared<Um982>()); }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("um982_driver"), "%s", e.what());
        result = 1;
    }
    curl_global_cleanup();
    rclcpp::shutdown();
    return result;
}
