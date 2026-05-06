/**
 * @brief 连接本机回环端口的 RTCM TCP 客户端。
 */
#ifndef UBLOX_DRIVER_LOOPBACK_TCP_CLIENT_HPP_
#define UBLOX_DRIVER_LOOPBACK_TCP_CLIENT_HPP_

#include "ublox_driver/rtcm/tcp_client.hpp"

class LoopbackTcpClient : public TcpClient {
public:
  explicit LoopbackTcpClient(uint64_t port, unsigned int buf_size = 8192);
};

#endif
