/**
 * @brief 原始二进制数据回调类型定义。
 */
#ifndef UBLOX_DRIVER_DATA_CALLBACK_HPP_
#define UBLOX_DRIVER_DATA_CALLBACK_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>

using DataCallback = std::function<void(const uint8_t *, size_t)>;

#endif
