#ifndef GNSS_COMM_UTILITY_SYSTEM_HPP_
#define GNSS_COMM_UTILITY_SYSTEM_HPP_

#include <string>

// 系统命令辅助接口，不参与 GNSS 定位算法。
namespace gnss_comm {

/**
 * @brief 执行外部命令并收集标准输出；会产生系统副作用，调用方需保证命令可信。
 * @param cmd 交给系统命令解释器执行的命令。
 * @return 捕获的标准输出文本，不是退出码；标准错误不会自动合并。
 * @note 使用 popen/pclose；无法创建管道时抛出 std::runtime_error，当前实现不返回命令退出状态。
 */
std::string exec(const std::string &cmd);

} // namespace gnss_comm

#endif // GNSS_COMM_UTILITY_SYSTEM_HPP_
