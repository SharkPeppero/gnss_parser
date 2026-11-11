/**
 * @brief NTRIP caster 客户端，实现挂载点请求与 RTCM 流接收。
 */
#ifndef UBLOX_DRIVER_NTRIP_CLIENT_HPP_
#define UBLOX_DRIVER_NTRIP_CLIENT_HPP_

#include <string>

#include "ublox_driver/rtcm/tcp_client.hpp"

class NtripClient : public TcpClient {
public:
  NtripClient(std::string host,
              uint64_t port,
              std::string mountpoint,
              std::string username,
              std::string password,
              unsigned int buf_size = 8192);

protected:
  bool onConnected() override;

private:
  std::string buildRequest() const;
  bool parseResponse(std::string response);

  std::string mountpoint_;
  std::string username_;
  std::string password_;
};

#endif
