/**
 * @brief 通用 TCP 客户端基类，负责连接、线程化读写和原始数据回调分发。
 */
#ifndef UBLOX_DRIVER_TCP_CLIENT_HPP_
#define UBLOX_DRIVER_TCP_CLIENT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ublox_driver/data_callback.hpp"

class TcpClient {
public:
  TcpClient(std::string host, uint64_t port, unsigned int buf_size = 8192);
  TcpClient(const TcpClient &) = delete;
  TcpClient &operator=(const TcpClient &) = delete;
  virtual ~TcpClient();

  bool connect();
  void registerDataCallback(DataCallback callback);
  void clearDataCallbacks();
  void addCallback(DataCallback callback);
  void registerConnectionStateCallback(std::function<void(bool)> callback);
  void startRead();
  bool write(const std::string &str, uint32_t timeout_ms = 50);
  bool writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms = 50);
  void close();
  bool is_open() const;

protected:
  virtual bool onConnected();
  virtual void handleReadData(const uint8_t *data, size_t len);

  void dispatchDataCallbacks(const uint8_t *data, size_t len);
  bool waitForEvent(short events, int timeout_ms);
  bool writeAll(const uint8_t *data, size_t len, uint32_t timeout_ms);
  ssize_t recvSome(uint8_t *data, size_t len, int flags = 0);
  bool readUntil(std::string &buffer, const std::string &delimiter, size_t max_bytes, int timeout_ms);

  const std::string &host() const;
  uint64_t port() const;
  int socketFd() const;

private:
  bool openConnection();
  void notifyConnectionState(bool connected);
  void closeSocket();
  void readLoop();
  void wakeReader();
  bool sleepInterruptibly(int timeout_ms);

  std::string host_;
  uint64_t port_;
  uint32_t buf_size_;
  std::unique_ptr<uint8_t[]> data_buf_;
  int socket_fd_;
  int wake_pipe_[2];
  std::thread read_thread_;
  std::atomic<bool> read_running_;
  std::atomic<bool> shutting_down_;
  std::atomic<bool> connection_active_;
  std::mutex write_mutex_;
  std::vector<DataCallback> callbacks_;
  std::vector<std::function<void(bool)>> connection_state_callbacks_;
};

#endif
