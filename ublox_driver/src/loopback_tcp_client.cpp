#include "ublox_driver/loopback_tcp_client.hpp"

LoopbackTcpClient::LoopbackTcpClient(uint64_t port, unsigned int buf_size)
    : TcpClient("127.0.0.1", port, buf_size) {}
