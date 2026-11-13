#include "um982_driver/rtcm_writer.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

RtcmWriter::RtcmWriter(rclcpp::Node& node, const std::string& directory)
    : node_(node), directory_(std::filesystem::absolute(directory).string())
{
    std::filesystem::create_directories(directory_);
    RCLCPP_INFO(node_.get_logger(), "Async RTCM recording enabled: %s", directory_.c_str());
    worker_ = std::thread([this] { run(); });
}

RtcmWriter::~RtcmWriter()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
}

void RtcmWriter::enqueue(std::string frame, uint64_t session, unsigned message_type)
{
    const auto unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || failed_) return;
        // Never block the correction stream on disk throughput. Drop a complete
        // frame on overflow, with an explicit warning that the recording has gaps.
        constexpr size_t max_queue_bytes = 8 * 1024 * 1024;
        if (queued_bytes_ + frame.size() > max_queue_bytes)
        {
            RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                "RTCM recording queue full: dropping frames from file; recording is incomplete");
            return;
        }
        queued_bytes_ += frame.size();
        queue_.push_back({std::move(frame), session, message_type, unix_ns});
    }
    ready_.notify_one();
}

void RtcmWriter::run()
{
    size_t frames = 0, bytes = 0;
    try
    {
        std::ofstream output, index;
        uint64_t session = 0;
        size_t offset = 0;
        auto last_log = std::chrono::steady_clock::now();
        while (true)
        {
            std::deque<Record> batch;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (queue_.empty() && stopping_) break;
                batch.swap(queue_);
                queued_bytes_ = 0;
            }
            for (const auto& record : batch)
            {
                if (!output.is_open() || session != record.session)
                {
                    if (output.is_open())
                    {
                        output.close();
                        index.close();
                        if (!output || !index) throw std::runtime_error("RTCM recording close failed");
                    }
                    session = record.session;
                    offset = 0;
                    const auto stem = directory_ + "/ntrip_" + std::to_string(record.unix_ns) +
                        "_s" + std::to_string(session);
                    output.open(stem + ".rtcm3", std::ios::binary | std::ios::out);
                    index.open(stem + ".csv");
                    if (!output || !index) throw std::runtime_error("Cannot open RTCM recording files");
                    // Raw RTCM contains no logger headers; retain UTC receipt time
                    // separately to recover the week/date during offline conversion.
                    index << "received_unix_ns,offset_bytes,frame_bytes,message_type\n";
                    RCLCPP_INFO(node_.get_logger(), "RTCM recording file: %s.rtcm3", stem.c_str());
                }
                output.write(record.frame.data(), static_cast<std::streamsize>(record.frame.size()));
                index << record.unix_ns << ',' << offset << ',' << record.frame.size() << ','
                      << record.message_type << '\n';
                if (!output || !index) throw std::runtime_error("RTCM recording write failed");
                offset += record.frame.size();
                ++frames;
                bytes += record.frame.size();
            }
            output.flush();
            index.flush();
            if (!output || !index) throw std::runtime_error("RTCM recording flush failed");
            const auto now = std::chrono::steady_clock::now();
            if (now - last_log >= std::chrono::seconds(5))
            {
                RCLCPP_INFO(node_.get_logger(), "RTCM file write OK: %zu frames/%zu bytes total", frames, bytes);
                last_log = now;
            }
        }
        if (output.is_open())
        {
            output.close();
            index.close();
            if (!output || !index) throw std::runtime_error("RTCM recording close failed");
        }
        RCLCPP_INFO(node_.get_logger(), "RTCM recording stopped: %zu frames/%zu bytes written", frames, bytes);
    }
    catch (const std::exception& e)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            failed_ = true;
            queue_.clear();
            queued_bytes_ = 0;
        }
        RCLCPP_ERROR(node_.get_logger(), "RTCM recording disabled: %s; file may be incomplete", e.what());
    }
}
