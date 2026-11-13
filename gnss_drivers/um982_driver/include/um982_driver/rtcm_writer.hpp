#pragma once

#include <rclcpp/rclcpp.hpp>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

// One producer (NTRIP), one disk worker. Each connection gets a separate file.
class RtcmWriter
{
public:
    RtcmWriter(rclcpp::Node& node, const std::string& directory);
    ~RtcmWriter();
    void enqueue(std::string frame, uint64_t session, unsigned message_type);

private:
    struct Record
    {
        std::string frame;
        uint64_t session;
        unsigned message_type;
        int64_t unix_ns;
    };
    void run();
    rclcpp::Node& node_;
    std::string directory_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Record> queue_;
    size_t queued_bytes_{0};
    bool stopping_{false};
    bool failed_{false};
    std::thread worker_;
};
