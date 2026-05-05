#include "ublox_driver/serial_handler.hpp"

#include "ublox_driver/params.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace {

speed_t get_baud_rate(unsigned int baud_rate) {
  switch (baud_rate) {
    case 9600:return B9600;
    case 19200:return B19200;
    case 38400:return B38400;
    case 57600:return B57600;
    case 115200:return B115200;
    case 230400:return B230400;
    case 460800:return B460800;
    case 921600:return B921600;
    default:return 0;
  }
}

bool configure_serial_port(int fd, unsigned int baud_rate) {
  struct termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }

  const speed_t speed = get_baud_rate(baud_rate);
  if (speed == 0) {
    errno = EINVAL;
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    return false;
  }

  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

}  // namespace

SerialHandler::SerialHandler(std::string serial_port,
                             unsigned int baud_rate,
                             unsigned int buf_size)
    : serial_port_(std::move(serial_port)),
      data_buf_(new uint8_t[buf_size]),
      serial_fd_(-1),
      wake_pipe_{-1, -1},
      read_running_(false),
      shutting_down_(false),
      MSG_HEADER_LEN(kUbxMsgHeaderLen) {
  //
  if (pipe(wake_pipe_) != 0) {
    std::cerr << "Cannot create wake pipe for serial " << serial_port_ << ": " << std::strerror(errno) << '\n';
    wake_pipe_[0] = -1;
    wake_pipe_[1] = -1;
    return;
  }
  (void) fcntl(wake_pipe_[0], F_SETFL, O_NONBLOCK);
  (void) fcntl(wake_pipe_[1], F_SETFL, O_NONBLOCK);

  //
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    std::cerr << "Cannot open serial port " << serial_port_ << ": " << std::strerror(errno) << ".\n";
    return;
  }

  if (!configure_serial_port(serial_fd_, baud_rate)) {
    std::cerr << "Cannot configure serial port " << serial_port_ << ": " << std::strerror(errno) << ".\n";
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

SerialHandler::~SerialHandler() {
  close();
}

void SerialHandler::registerDataCallback(DataCallback callback) {
  data_callbacks_.push_back(std::move(callback));
}

void SerialHandler::clearDataCallbacks() {
  data_callbacks_.clear();
}

void SerialHandler::registerUBLOXMessageParser(DataCallback callback) {
  registerDataCallback(std::move(callback));
}

void SerialHandler::registerCallback(DataCallback callback) {
  registerDataCallback(std::move(callback));
}

bool SerialHandler::is_open() const {
  return serial_fd_ >= 0;
}

void SerialHandler::close() {
  if (shutting_down_) {
    return;
  }

  shutting_down_ = true;
  stopRead();

  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  if (wake_pipe_[0] >= 0) {
    ::close(wake_pipe_[0]);
    wake_pipe_[0] = -1;
  }
  if (wake_pipe_[1] >= 0) {
    ::close(wake_pipe_[1]);
    wake_pipe_[1] = -1;
  }
}

void SerialHandler::startRead() {
  if (!is_open() || read_running_) {
    return;
  }

  read_running_ = true;
  read_thread_ = std::thread(&SerialHandler::read_loop, this);
}

void SerialHandler::stopRead() {
  if (!read_running_) {
    return;
  }

  read_running_ = false;
  wake_reader();
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void SerialHandler::stop_read() {
  stopRead();
}

/// ================================================================= ///
void SerialHandler::read_loop() {
  while (read_running_) {
    if (!read_exact(data_buf_.get(), MSG_HEADER_LEN)) {
      break;
    }

    uint32_t header_remains = MSG_HEADER_LEN;
    while (header_remains != 0 && read_running_) {
      uint32_t pream_idx = 0;
      for (; pream_idx < MSG_HEADER_LEN - 1; ++pream_idx) {
        if (data_buf_[pream_idx] == 0xB5 && data_buf_[pream_idx + 1] == 0x62) {
          break;
        }
      }

      header_remains = pream_idx;
      if (header_remains != 0) {
        memmove(data_buf_.get(), data_buf_.get() + pream_idx, MSG_HEADER_LEN - header_remains);
        if (!read_exact(data_buf_.get() + MSG_HEADER_LEN - pream_idx, header_remains)) {
          header_remains = 0;
          break;
        }
      }
    }

    if (!read_running_) {
      break;
    }

    uint16_t *msg_len_ptr = reinterpret_cast<uint16_t *>(data_buf_.get() + 4);
    if (!read_exact(data_buf_.get() + MSG_HEADER_LEN, *msg_len_ptr + 2)) {
      break;
    }

    for (auto &callback : data_callbacks_) {
      callback(data_buf_.get(), MSG_HEADER_LEN + (*msg_len_ptr) + 2);
    }
  }

  if (!shutting_down_ && is_open()) {
    std::cerr << "Serial " << serial_port_ << " read loop stopped.\n";
  }
}

bool SerialHandler::read_exact(uint8_t *buffer, size_t len) {
  size_t total = 0;
  while (total < len && read_running_) {
    if (!wait_for_event(POLLIN, -1)) {
      return false;
    }

    const ssize_t bytes_read = ::read(serial_fd_, buffer + total, len - total);
    if (bytes_read > 0) {
      total += static_cast<size_t>(bytes_read);
      continue;
    }

    if (bytes_read == 0) {
      return false;
    }

    if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    }
  }

  return total == len;
}

/// ================================================================= ///
bool SerialHandler::write(const std::string &str, uint32_t timeout_ms) {
  return writeRaw(reinterpret_cast<const uint8_t *>(str.c_str()), str.size(), timeout_ms);
}

bool SerialHandler::writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  if (!write_all(data, len, timeout_ms)) {
    std::cerr << "Serial write failed or operation timeout: " << serial_port_ << '\n';
    return false;
  }
  return true;
}

bool SerialHandler::write_all(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  size_t total = 0;

  while (total < len && is_open()) {
    if (!wait_for_event(POLLOUT, static_cast<int>(timeout_ms))) {
      return false;
    }

    const ssize_t bytes_written = ::write(serial_fd_, data + total, len - total);
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

/// ================================================================ ///
void SerialHandler::wake_reader() {
  if (wake_pipe_[1] < 0) {
    return;
  }
  const uint8_t signal = 1;
  const ssize_t rc = ::write(wake_pipe_[1], &signal, sizeof(signal));
  if (rc < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    std::cerr << "Failed to wake serial reader for " << serial_port_ << ": "
              << std::strerror(errno) << '\n';
  }
}

bool SerialHandler::wait_for_event(short events, int timeout_ms) {
  if (!is_open()) {
    return false;
  }

  struct pollfd fds[2];
  fds[0].fd = serial_fd_;
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
