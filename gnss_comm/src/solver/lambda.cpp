#include "gnss_comm/solver/lambda.hpp"
#include <cmath>
#include <utility>

namespace gnss_comm {

namespace {

constexpr int LOOPMAX = 10000; // 搜索循环上限

double sgn(const double x) { return x <= 0.0 ? -1.0 : 1.0; }

// Q = Lᵀ·diag(D)·L，L 为单位下三角。
bool ld_factor(const Eigen::MatrixXd &Q, Eigen::MatrixXd &L, Eigen::VectorXd &D) {
  const int n = static_cast<int>(Q.rows());
  Eigen::MatrixXd A = Q;
  L = Eigen::MatrixXd::Zero(n, n);
  D = Eigen::VectorXd::Zero(n);
  for (int i = n - 1; i >= 0; --i) {
    D(i) = A(i, i);
    if (D(i) <= 0.0)
      return false;
    const double a = std::sqrt(D(i));
    for (int j = 0; j <= i; ++j)
      L(i, j) = A(i, j) / a;
    for (int j = 0; j <= i - 1; ++j)
      for (int k = 0; k <= j; ++k)
        A(j, k) -= L(i, k) * L(i, j);
    for (int j = 0; j <= i; ++j)
      L(i, j) /= L(i, i);
  }
  return true;
}

// 整数高斯变换：消去 L(i,j) 的整数部分，同步更新 Z。
void gauss(Eigen::MatrixXd &L, Eigen::MatrixXd &Z, const int i, const int j) {
  const int n = static_cast<int>(L.rows());
  const double mu = std::round(L(i, j));
  if (mu == 0.0)
    return;
  for (int k = i; k < n; ++k)
    L(k, j) -= mu * L(k, i);
  for (int k = 0; k < n; ++k)
    Z(k, j) -= mu * Z(k, i);
}

// 交换第 j、j+1 个模糊度，del 为交换后 D(j+1) 的新值。
void perm(Eigen::MatrixXd &L, Eigen::VectorXd &D, const int j, const double del, Eigen::MatrixXd &Z) {
  const int n = static_cast<int>(L.rows());
  const double eta = D(j) / del;
  const double lam = D(j + 1) * L(j + 1, j) / del;
  D(j) = eta * D(j + 1);
  D(j + 1) = del;
  for (int k = 0; k <= j - 1; ++k) {
    const double a0 = L(j, k), a1 = L(j + 1, k);
    L(j, k) = -L(j + 1, j) * a0 + a1;
    L(j + 1, k) = eta * a0 + lam * a1;
  }
  L(j + 1, j) = lam;
  for (int k = j + 2; k < n; ++k)
    std::swap(L(k, j), L(k, j + 1));
  for (int k = 0; k < n; ++k)
    std::swap(Z(k, j), Z(k, j + 1));
}

// 降相关：整数高斯变换 + 排序，使 D 尽量升序、L 非对角元素尽量小。
void reduction(Eigen::MatrixXd &L, Eigen::VectorXd &D, Eigen::MatrixXd &Z) {
  const int n = static_cast<int>(L.rows());
  int j = n - 2, k = n - 2;
  while (j >= 0) {
    if (j <= k)
      for (int i = j + 1; i < n; ++i)
        gauss(L, Z, i, j);
    const double del = D(j) + L(j + 1, j) * L(j + 1, j) * D(j + 1);
    if (del + 1e-6 < D(j + 1)) {
      perm(L, D, j, del, Z);
      k = j;
      j = n - 2;
    } else {
      --j;
    }
  }
}

// 在降相关空间内深度优先搜索 m 个最优整数向量。
bool search(const Eigen::MatrixXd &L, const Eigen::VectorXd &D, const Eigen::VectorXd &zs, const int m, Eigen::MatrixXd &zn, Eigen::VectorXd &s) {
  const int n = static_cast<int>(L.rows());
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd dist(n), zb(n), z(n), step(n);
  zn = Eigen::MatrixXd::Zero(n, m);
  s = Eigen::VectorXd::Zero(m);

  int nn = 0, imax = 0, k = n - 1, c = 0;
  double maxdist = 1e99;
  dist(k) = 0.0;
  zb(k) = zs(k);
  z(k) = std::round(zb(k));
  double y = zb(k) - z(k);
  step(k) = sgn(y);
  for (; c < LOOPMAX; ++c) {
    const double newdist = dist(k) + y * y / D(k);
    if (newdist < maxdist) {
      if (k != 0) {
        // 向下一层搜索
        dist(--k) = newdist;
        for (int i = 0; i <= k; ++i)
          S(k, i) = S(k + 1, i) + (z(k + 1) - zb(k + 1)) * L(k + 1, i);
        zb(k) = zs(k) + S(k, k);
        z(k) = std::round(zb(k));
        y = zb(k) - z(k);
        step(k) = sgn(y);
      } else {
        // 得到一个候选：前 m 个直接保存，之后替换当前最差者并收缩搜索半径
        if (nn < m) {
          if (nn == 0 || newdist > s(imax))
            imax = nn;
          zn.col(nn) = z;
          s(nn++) = newdist;
        } else {
          if (newdist < s(imax)) {
            zn.col(imax) = z;
            s(imax) = newdist;
            imax = 0;
            for (int i = 0; i < m; ++i)
              if (s(imax) < s(i))
                imax = i;
          }
          maxdist = s(imax);
        }
        z(0) += step(0);
        y = zb(0) - z(0);
        step(0) = -step(0) - sgn(step(0));
      }
    } else {
      // 回到上一层，按 zig-zag 顺序尝试下一个整数
      if (k == n - 1)
        break;
      ++k;
      z(k) += step(k);
      y = zb(k) - z(k);
      step(k) = -step(k) - sgn(step(k));
    }
  }
  for (int i = 0; i < m - 1; ++i) {
    for (int j = i + 1; j < m; ++j) {
      if (s(i) <= s(j))
        continue;
      std::swap(s(i), s(j));
      const Eigen::VectorXd tmp = zn.col(i);
      zn.col(i) = zn.col(j);
      zn.col(j) = tmp;
    }
  }
  return c < LOOPMAX && nn == m;
}

} // namespace

bool lambda(const Eigen::VectorXd &a, const Eigen::MatrixXd &Q, const int m, Eigen::MatrixXd &F, Eigen::VectorXd &s) {
  const int n = static_cast<int>(a.size());
  if (n <= 0 || m <= 0 || Q.rows() != n || Q.cols() != n)
    return false;
  Eigen::MatrixXd L, Z = Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd D;
  if (!ld_factor(Q, L, D))
    return false;
  reduction(L, D, Z);
  // z = Zᵀa 为降相关后的浮点模糊度；搜索结果再经 Z⁻ᵀ 变回原空间（Z 为幺模矩阵，逆仍为整数矩阵）。
  const Eigen::VectorXd z = Z.transpose() * a;
  Eigen::MatrixXd E;
  if (!search(L, D, z, m, E, s))
    return false;
  F = Z.transpose().fullPivLu().solve(E).array().round().matrix();
  return true;
}

} // namespace gnss_comm
