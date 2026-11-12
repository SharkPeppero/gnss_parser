#ifndef GNSS_COMM_SOLVER_LAMBDA_HPP_
#define GNSS_COMM_SOLVER_LAMBDA_HPP_

#include <eigen3/Eigen/Dense>

namespace gnss_comm {

// 整数最小二乘模糊度搜索（MLAMBDA）：LD 分解 → 整数 Z 变换降相关 → 收缩椭球搜索。
// 参考 Chang X.-W., Yang X., Zhou T. (2005) MLAMBDA，以及 RTKLIB lambda.c（BSD-2）。
// a 为浮点模糊度（周，长度 n），Q 为其协方差（n×n，需正定）。
// 输出 m 组最优整数解 F（n×m，按二次型升序），s(i) = (a−F_i)ᵀ Q⁻¹ (a−F_i)；ratio 检验取 s(1)/s(0)。
// Q 非正定或搜索超出循环上限时返回 false。
bool lambda(const Eigen::VectorXd &a, const Eigen::MatrixXd &Q, int m, Eigen::MatrixXd &F, Eigen::VectorXd &s);

} // namespace gnss_comm

#endif // GNSS_COMM_SOLVER_LAMBDA_HPP_
