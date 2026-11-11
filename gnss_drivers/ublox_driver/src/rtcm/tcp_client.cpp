/**
 * @brief 通用 TCP 客户端实现。
 */
#include "ublox_driver/rtcm/tcp_client.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <chrono>
#include <utility>

namespace {

constexpr int kReconnectDelayMs = 3000;

} // namespace

TcpClient::TcpClient(std::string host, uint64_t port, unsigned int buf_size)
    : host_(std::move(host)),
      port_(port),
      buf_size_(buf_size),
      data_buf_(new uint8_t[buf_size_]),
      socket_fd_(-1),
      wake_pipe_{-1, -1},
      read_running_(false),
      shutting_down_(false),
      connection_active_(false) {
  if (pipe(wake_pipe_) != 0) {
    std::cerr << "Cannot create wake pipe for TCP client " << host_ << ':' << port_
              << ": " << std::strerror(errno) << '\n';
    wake_pipe_[0] = -1;
    wake_pipe_[1] = -1;
    return;
  }

  (void)fcntl(wake_pipe_[0], F_SETFL, O_NONBLOCK);
  (void)fcntl(wake_pipe_[1], F_SETFL, O_NONBLOCK);
}

TcpClient::~TcpClient() {
  close();
}

bool TcpClient::openConnection() {
  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo *results = nullptr;
  const int resolve_rc = getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &results);
  if (resolve_rc != 0) {
    std::cerr << "Could not resolve " << host_ << ':' << port_ << ": "
              << gai_strerror(resolve_rc) << '\n';
    return false;
  }

  for (struct addrinfo *entry = results; entry != nullptr; entry = entry->ai_next) {
    socket_fd_ = socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
    if (socket_fd_ < 0) {
      continue;
    }

    if (::connect(socket_fd_, entry->ai_addr, entry->ai_addrlen) == 0) {
      break;
    }

    ::close(socket_fd_);
    socket_fd_ = -1;
  }

  freeaddrinfo(results);

  if (!is_open()) {
    std::cerr << "Could not connect to " << host_ << " at port " << port_ << '\n';
    return false;
  }

  return true;
}

bool TcpClient::connect() {
  if (is_open()) {
    return true;
  }

  if (!openConnection()) {
    return false;
  }

  if (!onConnected()) {
    closeSocket();
    return false;
  }

  notifyConnectionState(true);

  return true;
}

void TcpClient::registerDataCallback(DataCallback callback) {
  callbacks_.push_back(std::move(callback));
}

void TcpClient::clearDataCallbacks() {
  callbacks_.clear();
}

void TcpClient::addCallback(DataCallback callback) {
  registerDataCallback(std::move(callback));
}

void TcpClient::registerConnectionStateCallback(std::function<void(bool)> callback) {
  connection_state_callbacks_.push_back(std::move(callback));
}

bool TcpClient::is_open() const {
  return socket_fd_ >= 0;
}

bool TcpClient::onConnected() {
  return true;
}

void TcpClient::dispatchDataCallbacks(const uint8_t *data, size_t len) {
  for (auto &callback : callbacks_) {
    callback(data, len);
  }
}

void TcpClient::handleReadData(const uint8_t *data, size_t len) {
  dispatchDataCallbacks(data, len);
}

const std::string &TcpClient::host() const {
  return host_;
}

uint64_t TcpClient::port() const {
  return port_;
}

int TcpClient::socketFd() const {
  return socket_fd_;
}

void TcpClient::notifyConnectionState(bool connected) {
  if (connection_active_.exchange(connected) == connected) {
    return;
  }

  for (auto &callback : connection_state_callbacks_) {
    callback(connected);
  }
}

void TcpClient::wakeReader() {
  if (wake_pipe_[1] < 0) {
    return;
  }

  const uint8_t signal = 1;
  const ssize_t rc = ::write(wake_pipe_[1], &signal, sizeof(signal));
  if (rc < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    std::cerr << "Failed to wake TCP reader for " << host_ << ':' << port_
              << ": " << std::strerror(errno) << '\n';
  }
}

bool TcpClient::waitForEvent(short events, int timeout_ms) {
  if (!is_open()) {
    return false;
  }

  struct pollfd fds[2];
  fds[0].fd = socket_fd_;
  fds[0].events = events;
  fds[0].revents = 0;
  fds[1].fd = wake_pipe_[0];
  fds[1].events = POLLIN;
  fds[1].revents = 0;

  while (true) {
    const int rc = poll(fds, 2, timeout_ms);
    if (rc > 0) {
      if ((fds[1].revents & POLLIN) != 0) {
        uint8_t drain[16];
        while (::read(wake_pipe_[0], drain, sizeof(drain)) > 0) {
        }
        return false;
      }

      if ((fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        return false;
      }

      if ((fds[0].revents & events) != 0) {
        return true;
      }
      continue;
    }

    if (rc == 0) {
      return false;
    }

    if (errno != EINTR) {
      return false;
    }
  }
}

bool TcpClient::writeAll(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  size_t total = 0;

  while (total < len && is_open()) {
    if (!waitForEvent(POLLOUT, static_cast<int>(timeout_ms))) {
      return false;
    }

    const ssize_t bytes_written = send(socket_fd_, data + total, len - total, MSG_NOSIGNAL);
    if (bytes_written > 0) {
      total += static_cast<size_t>(bytes_written);
      continue;
    }

    if (bytes_written < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }

    return false;
  }

  return total == len;
}

ssize_t TcpClient::recvSome(uint8_t *data, size_t len, int flags) {
  if (!is_open()) {
    return -1;
  }
  return recv(socket_fd_, data, len, flags);
}

bool TcpClient::readUntil(std::string &buffer, const std::string &delimiter, size_t max_bytes, int timeout_ms) {
  while (buffer.find(delimiter) == std::string::npos) {
    if (buffer.size() >= max_bytes) {
      return false;
    }

    if (!waitForEvent(POLLIN, timeout_ms)) {
      return false;
    }

    uint8_t temp[1024];
    const ssize_t bytes_read = recvSome(temp, sizeof(temp), 0);
    if (bytes_read > 0) {
      buffer.append(reinterpret_cast<const char *>(temp), static_cast<size_t>(bytes_read));
      continue;
    }

    if (bytes_read == 0) {
      return false;
    }

    if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    }
  }

  return true;
}

void TcpClient::readLoop() {
  while (read_running_) {
    if (!is_open()) {
      if (!connect()) {
        if (!read_running_ || shutting_down_) {
          break;
        }

        std::cerr << "TCP client " << host_ << ':' << port_
                  << " connect failed, retrying in " << (kReconnectDelayMs / 1000)
                  << "s.\n";
        if (!sleepInterruptibly(kReconnectDelayMs)) {
          break;
        }
        continue;
      }
    }

    if (!waitForEvent(POLLIN, -1)) {
      if (!read_running_ || shutting_down_) {
        break;
      }

      if (is_open()) {
        std::cerr << "TCP client " << host_ << ':' << port_
                  << " connection lost, retrying in " << (kReconnectDelayMs / 1000)
                  << "s.\n";
      }
      closeSocket();
      if (!sleepInterruptibly(kReconnectDelayMs)) {
        break;
      }
      continue;
    }

    const ssize_t bytes_read = recvSome(data_buf_.get(), buf_size_, 0);
    if (bytes_read > 0) {
      handleReadData(data_buf_.get(), static_cast<size_t>(bytes_read));
      continue;
    }

    if (bytes_read == 0) {
      std::cerr << "TCP client " << host_ << ':' << port_
                << " remote peer closed the connection, retrying in "
                << (kReconnectDelayMs / 1000) << "s.\n";
      closeSocket();
      if (!sleepInterruptibly(kReconnectDelayMs)) {
        break;
      }
      continue;
    }

    if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "TCP client " << host_ << ':' << port_ << " recv failed: "
                << std::strerror(errno) << ", retrying in "
                << (kReconnectDelayMs / 1000) << "s.\n";
      closeSocket();
      if (!sleepInterruptibly(kReconnectDelayMs)) {
        break;
      }
    }
  }

  closeSocket();
}

void TcpClient::startRead() {
  if (read_running_) {
    return;
  }

  read_running_ = true;
  read_thread_ = std::thread(&TcpClient::readLoop, this);
}

bool TcpClient::write(const std::string &str, uint32_t timeout_ms) {
  return writeRaw(reinterpret_cast<const uint8_t *>(str.data()), str.size(), timeout_ms);
}

bool TcpClient::writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  if (!writeAll(data, len, timeout_ms)) {
    std::cerr << "TCP write failed or operation timeout: " << host_ << ':' << port_ << '\n';
    return false;
  }

  return true;
}

bool TcpClient::sleepInterruptibly(int timeout_ms) {
  if (wake_pipe_[0] < 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    return read_running_ && !shutting_down_;
  }

  struct pollfd fd {};
  fd.fd = wake_pipe_[0];
  fd.events = POLLIN;
  fd.revents = 0;

  while (true) {
    const int rc = poll(&fd, 1, timeout_ms);
    if (rc > 0) {
      if ((fd.revents & POLLIN) != 0) {
        uint8_t drain[16];
        while (::read(wake_pipe_[0], drain, sizeof(drain)) > 0) {
        }
      }
      return false;
    }

    if (rc == 0) {
      return read_running_ && !shutting_down_;
    }

    if (errno != EINTR) {
      return false;
    }
  }
}

void TcpClient::closeSocket() {
  if (socket_fd_ >= 0) {
    notifyConnectionState(false);
    shutdown(socket_fd_, SHUT_RDWR);
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void TcpClient::close() {
  if (shutting_down_) {
    return;
  }

  shutting_down_ = true;
  if (read_running_) {
    read_running_ = false;
    wakeReader();
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

  closeSocket();

  if (wake_pipe_[0] >= 0) {
    ::close(wake_pipe_[0]);
    wake_pipe_[0] = -1;
  }
  if (wake_pipe_[1] >= 0) {
    ::close(wake_pipe_[1]);
    wake_pipe_[1] = -1;
  }
}
