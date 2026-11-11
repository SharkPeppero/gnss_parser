#include "ublox_driver/gnss_message_parser/ublox_message_processor.hpp"

#include <cmath>
#include <cstring>

UbloxMessageProcessor::UbloxMessageProcessor(std::shared_ptr<ublox_driver::UbloxRosHandler> ros_handler)
    : curr_time{},                      //
      MSG_HEADER_LEN(kUbxMsgHeaderLen), //
      ros_handler_(std::move(ros_handler)) {
  std::memset(subfrm, 0, sizeof(subfrm));
  std::memset(bds_d1_subfrm_flags_, 0, sizeof(bds_d1_subfrm_flags_));
}

/** @brief Ublox模块处理串口数据 */
void UbloxMessageProcessor::process_data(const uint8_t *data, size_t len) {
  /// 验证消息合法性（前导字节、长度、校验和）
  if (!verifyMsg(data, len)) {
    return;
  }

  /// 确定当前消息的类型
  const uint16_t msg_type = (data[2] << 8) | data[3];
  if (msg_type == UBX_RXMRAWX_ID) {
    /// GNSS卫星原始数据观测
    ///     1、伪距
    ///     2、多普勒
    ///     3、载波
    ///     4、信号强度
    ///     5、周期半值
    ///     6、跳变
    ///     7、信号状态
    std::vector<ObsPtr> meas = parse_meas_msg(data, len);
    if (ros_handler_) {
      ros_handler_->publishObservations(meas);
    }
  } else if (msg_type == UBX_RXMSFRBX_ID) {
    /// GNSS原始导航子帧，星历、电离层信息
    std::vector<double> iono_params;
    EphemBasePtr ephem = parse_subframe(data, len, iono_params);

    // 发布星历信息（需要注意GLONASS星历以及其他的星历区别处理）
    if (ros_handler_) {
      ros_handler_->publishEphemeris(ephem);
    }

    // 发布电离层参数（）
    if (ros_handler_ && ephem) {
      ros_handler_->publishIonoParams(iono_params, ephem->ttr);
    }

  } else if (msg_type == UBX_TIM_TP_ID) {
    /// 发布时间脉冲信号发布
    TimePulseInfoPtr tp_info = parse_time_pulse(data, len);
    if (ros_handler_) {
      ros_handler_->publishTimePulse(tp_info);
    }
  } else if (msg_type == UBX_NAVPOS_ID) {
    /// 解析UBLOX的导航数据消息
    PVTSolutionPtr pvt_soln = parse_pvt(data, len);
    if (ros_handler_) {
      ros_handler_->publishPvt(pvt_soln);
    }
  }
}

/** @brief 串口数据Message的基础校验 */
bool UbloxMessageProcessor::verifyMsg(const uint8_t *data, const size_t len) {
  // ubx header length is 6, checksum length is 2
  if (len < 8) {
    LOG(ERROR) << "Invalid message length ";
    return false;
  }

  // 前导字节(0 1字节的格式要求)
  if (data[0] != UBX_SYNC_1 || data[1] != UBX_SYNC_2) {
    LOG(ERROR) << "Invalid ublox message preamble.";
    return false;
  }

  // 检查长度与校验和是否匹配
  uint16_t payload_len = *reinterpret_cast<const uint16_t *>(data + 4);
  if (len != payload_len + kUbxMsgHeaderLen + 2) {
    LOG(ERROR) << "Invalid message length.";
    return false;
  }

  // 检查长度与校验和是否匹配
  if (!check_checksum(data, len)) {
    LOG(ERROR) << "Invalid checksum.\n";
    return false;
  }

  return true;
}
