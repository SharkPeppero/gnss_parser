/**
 * @brief 封装串口收发、消息读取和回调分发逻辑。
 */
#ifndef SERIAL_HANDLER_HPP_
#define SERIAL_HANDLER_HPP_

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ublox_driver/serial/data_callback.hpp"

/**
 * @brief 串口通信管理
 */
class SerialHandler {
public:
  SerialHandler(std::string serial_port, unsigned int baud_rate, unsigned int buf_size = 8192);
  ~SerialHandler();

  void registerDataCallback(DataCallback callback);
  void clearDataCallbacks();
  void registerUBLOXMessageParser(DataCallback callback);
  void registerCallback(DataCallback callback);
  void startRead();
  void stopRead();
  void stop_read();
  bool write(const std::string &str, uint32_t timeout_ms = 50);
  bool writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms = 50);
  void close();
  bool is_open() const;

private:
  void read_loop();
  bool read_exact(uint8_t *buffer, size_t len);
  bool wait_for_event(short events, int timeout_ms);
  bool write_all(const uint8_t *data, size_t len, uint32_t timeout_ms);
  void wake_reader();
  std::string serial_port_;
  std::unique_ptr<uint8_t[]> data_buf_;
  int serial_fd_;
  int wake_pipe_[2];
  std::thread read_thread_;
  std::atomic<bool> read_running_;
  std::atomic<bool> shutting_down_;
  std::mutex write_mutex_;

  uint32_t MSG_HEADER_LEN;

  std::vector<DataCallback> data_callbacks_;
};

#endif
