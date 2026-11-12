# SPP 单点定位：WLS 闭式求解推导

Nov 12, 2026 · @Haocheng Xu

SPP 用单历元 L1 类伪距做迭代加权最小二乘，每次迭代的增量有闭式解 dx = −(GᵀWG)⁻¹GᵀWb，下文按 gnss\_comm 的实现约定逐步推导。

## 1. 伪距观测方程

第 i 颗卫星（属于星座 s）在接收机时刻 t\_r 的 L1 类伪距建模为：

$$
P^i = \lVert \mathbf{r}^i(t^i_{tx}) - \mathbf{r}_r \rVert + \delta_{sag}^i + c\,\delta t_{r,s} - c\,\delta t^i + c\,T_{GD}^i + I^i + T^i + \varepsilon^i
$$

| 符号 | 含义 | 单位 | 来源 |
| --- | --- | --- | --- |
| r^i(t\_tx) | 卫星在信号发射时刻的 ECEF 位置 | m | 广播星历，t\_tx = t\_r − P/c − δt^i |
| r\_r | 接收机 ECEF 位置（待估） | m | 状态量 |
| δ\_sag | Sagnac 改正，ω\_e (x^i·y\_r − y^i·x\_r)/c | m | 地球自转 |
| c·δt\_{r,s} | 星座 s 的接收机钟差（待估），已乘光速 | m | 状态量 |
| δt^i | 卫星钟差，含相对论改正 | s | 广播星历 |
| T\_GD | 群延迟改正 | s | 广播星历 tgd\[0\] |
| I^i, T^i | 电离层（Klobuchar 8 参数）、对流层延迟 | m | 模型 |
| ε^i | 伪距噪声与多路径等残余误差 | m | — |

接收机钟差按星座分开估计，因为各星座的时间系统偏差与接收机内部通道延迟不同；QZSS 与 GPS 共用一个钟差。状态向量为 7 维：

$$
\mathbf{x} = \begin{bmatrix} x_r & y_r & z_r & c\,\delta t_{GPS} & c\,\delta t_{GLO} & c\,\delta t_{GAL} & c\,\delta t_{BDS} \end{bmatrix}^\mathsf{T}
$$

把已知改正合并后，记观测的预测函数为 h^i(x)，观测方程简写为 P^i = h^i(x) + ε^i。

## 2. 线性化

几何距离对位置非线性，在当前估计 xₖ 处一阶泰勒展开，忽略二阶项（卫星距离约 2×10⁷ m，定位误差百米级时二阶项仅毫米级）：

$$
h^i(\mathbf{x}_k + \delta\mathbf{x}) \approx h^i(\mathbf{x}_k) + \mathbf{g}_i^\mathsf{T}\,\delta\mathbf{x}, \qquad \mathbf{g}_i^\mathsf{T} = \left.\frac{\partial h^i}{\partial \mathbf{x}}\right|_{\mathbf{x}_k}
$$

记接收机指向卫星的单位向量为 eᵢ = (r^i − r\_r)/‖r^i − r\_r‖。距离对接收机位置的偏导为 −eᵢᵀ，对本星座钟差的偏导为 1，对其它钟差为 0（Sagnac 项对位置的导数约 10⁻⁶ 量级，忽略）：

$$
\mathbf{g}_i^\mathsf{T} = \begin{bmatrix} -\mathbf{e}_i^\mathsf{T} & \mathbf{u}_{s(i)}^\mathsf{T} \end{bmatrix}, \qquad \mathbf{u}_{s} \in \{0,1\}^4 \text{ 仅在星座 } s \text{ 处为 } 1
$$

### 2.1 残差对接收机 ECEF 位置的偏导推导

结论：∂bᵢ/∂r\_r = −eᵢᵀ。也就是说，接收机朝卫星方向移动 1 m，预测伪距减小 1 m。下面逐项推导。

**第一步：只有哪些项依赖 r\_r。** 残差 bᵢ = hᵢ(x) − Pᵢ，其中观测 Pᵢ 是常数，故 ∂bᵢ/∂r\_r = ∂hᵢ/∂r\_r。把 hᵢ 按是否依赖 r\_r 拆开：

$$
h_i(\mathbf{x}) = \underbrace{\rho_i(\mathbf{r}_r)}_{\text{几何距离}} + \underbrace{\delta^i_{sag}(\mathbf{r}_r)}_{\text{Sagnac}} + \underbrace{I_i(\mathbf{r}_r) + T_i(\mathbf{r}_r)}_{\text{经高度角/经纬度弱依赖}} + c\,\delta t_{r,s} \underbrace{- c\,\delta t^i + c\,T_{GD}^i}_{\text{与 } \mathbf{r}_r \text{ 无关}}
$$

卫星位置 r^i 也不依赖 r\_r。发射时刻 t\_tx = t\_r − Pᵢ/c − δt^i 用的是观测伪距 Pᵢ 而不是几何距离，所以在一次迭代内 r^i 是常数（`sat_states` 在迭代前只算一次）。

**第二步：几何距离的偏导（主项）。** 令差向量 d = r^i − r\_r = \[Δx, Δy, Δz\]ᵀ，则：

$$
\rho_i = \lVert\mathbf{d}\rVert = \sqrt{(x^i - x_r)^2 + (y^i - y_r)^2 + (z^i - z_r)^2}
$$

对 x\_r 求导。外层是平方根，内层是平方，最内层 ∂(x^i − x\_r)/∂x\_r = −1：

$$
\frac{\partial \rho_i}{\partial x_r} = \frac{1}{2\rho_i}\cdot 2\,(x^i - x_r)\cdot(-1) = -\frac{x^i - x_r}{\rho_i}
$$

y\_r、z\_r 同理。三个分量写成行向量，正是单位视线向量取负：

$$
\frac{\partial \rho_i}{\partial \mathbf{r}_r} = -\frac{1}{\rho_i}\begin{bmatrix} x^i - x_r & y^i - y_r & z^i - z_r \end{bmatrix} = -\mathbf{e}_i^\mathsf{T}, \qquad \mathbf{e}_i = \frac{\mathbf{r}^i - \mathbf{r}_r}{\lVert\mathbf{r}^i - \mathbf{r}_r\rVert}
$$

用矩阵链式法则可以一步得到同样结果：ρ = (dᵀd)^{1/2}，∂ρ/∂d = dᵀ/ρ，∂d/∂r\_r = −I₃。

$$
\frac{\partial \rho_i}{\partial \mathbf{r}_r} = \frac{\partial \rho_i}{\partial \mathbf{d}}\,\frac{\partial \mathbf{d}}{\partial \mathbf{r}_r} = \frac{\mathbf{d}^\mathsf{T}}{\rho_i}\,(-\mathbf{I}_3) = -\mathbf{e}_i^\mathsf{T}
$$

**第三步：Sagnac 项的偏导（可忽略）。** 代码中 δ\_sag = ω\_e (x^i·y\_r − y^i·x\_r)/c，对 r\_r 线性：

$$
\frac{\partial \delta^i_{sag}}{\partial \mathbf{r}_r} = \frac{\omega_e}{c}\begin{bmatrix} -y^i & x^i & 0 \end{bmatrix}, \qquad \left\lVert\frac{\partial \delta^i_{sag}}{\partial \mathbf{r}_r}\right\rVert \le \frac{\omega_e \lVert\mathbf{r}^i\rVert}{c} \approx \frac{7.29\times10^{-5}\times 2.66\times10^{7}}{3\times10^{8}} \approx 6.5\times10^{-6}
$$

它与主项 ‖eᵢ‖ = 1 相比小 5 个量级。定位偏差 100 m 时，忽略它带来的预测误差不到 1 mm，所以雅可比里不含此项；残差里仍完整保留 Sagnac（其本身可达十几米）。

**第四步：大气延迟的偏导（视为常数）。** Iᵢ、Tᵢ 只通过高度角、方位角和接收机经纬高依赖 r\_r。接收机移动 100 m，高度角变化约 100 / 2×10⁷ ≈ 5×10⁻⁶ rad（约 3×10⁻⁴ 度），延迟变化在亚毫米级。所以每次迭代用当前位置重新计算延迟，但不进入雅可比。

**合并。** 忽略第三、第四步的小项，再加上钟差列的 ∂hᵢ/∂(cδt\_{r,s}) = 1：

$$
\frac{\partial b_i}{\partial \mathbf{x}} = \begin{bmatrix} -e_{i,x} & -e_{i,y} & -e_{i,z} & \mathbf{u}_{s(i)}^\mathsf{T} \end{bmatrix} = \mathbf{g}_i^\mathsf{T}
$$

对应代码（`psr_res`）：`rv2sv = sv_pos - rcv_pos` 就是 d，`unit_rv2sv = rv2sv.normalized()` 就是 eᵢ，`J.block(i, 0, 1, 3) = -unit_rv2sv.transpose()` 就是 −eᵢᵀ。

**几何意义。** 把 eᵢ 转到接收机 ENU 坐标系（方位角 A、高度角 E）：

$$
\mathbf{e}_i^{enu} = \begin{bmatrix} \cos E_i \sin A_i & \cos E_i \cos A_i & \sin E_i \end{bmatrix}^\mathsf{T}
$$

低仰角卫星主要约束水平方向，天顶卫星只约束高程。所有卫星都在地平线以上（sin E > 0），高程分量只有单侧约束，而且与钟差列（全为 1）强相关。这就是 VDOP 通常大于 HDOP 的原因。

**线性化误差。** 几何距离的 Hessian 为 (I − eᵢeᵢᵀ)/ρᵢ。位置偏差 Δr 引起的二阶截断误差约为：

$$
\left|\rho_i(\mathbf{r}_r + \Delta\mathbf{r}) - \rho_i(\mathbf{r}_r) + \mathbf{e}_i^\mathsf{T}\Delta\mathbf{r}\right| \approx \frac{\lVert\Delta\mathbf{r}_\perp\rVert^2}{2\rho_i}
$$

其中 Δr\_⊥ 是 Δr 垂直于视线的分量。Δr = 100 m、ρ ≈ 2×10⁷ m 时约 0.25 mm。第一次迭代从地心出发时 Δr ≈ 6.4×10⁶ m，截断误差达百公里量级，所以必须迭代；每次迭代后 Δr 近似按平方缩小，几次即可收敛。

代码采用「预测值−观测值」的残差符号约定（`psr_res` 中 `res(i) = psr_estimated - psr`）：

$$
b_i = h^i(\mathbf{x}_k) - P^i
$$

代入线性化观测方程 P^i = h^i(xₖ) + gᵢᵀδx + ε^i，得到 n 颗星堆叠后的线性模型：

$$
\mathbf{G}\,\delta\mathbf{x} = -\mathbf{b} - \boldsymbol{\varepsilon}, \qquad \mathbf{G} = \begin{bmatrix} \mathbf{g}_1^\mathsf{T} \\ \vdots \\ \mathbf{g}_n^\mathsf{T} \end{bmatrix} \in \mathbb{R}^{n\times 7}
$$

注意负号来自残差定义。若改用「观测−预测」的新息 y = −b，则是常见写法 Gδx = y − ε，最终闭式解完全一致。

## 3. WLS 闭式解推导

每次迭代的增量由法方程直接给出：δx = −(GᵀWG)⁻¹GᵀWb。推导如下。

**代价函数。** 由 ε = −(Gδx + b)，加权残差平方和为：

$$
J(\delta\mathbf{x}) = (\mathbf{G}\delta\mathbf{x} + \mathbf{b})^\mathsf{T}\,\mathbf{W}\,(\mathbf{G}\delta\mathbf{x} + \mathbf{b})
= \delta\mathbf{x}^\mathsf{T}\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G}\,\delta\mathbf{x} + 2\,\mathbf{b}^\mathsf{T}\mathbf{W}\mathbf{G}\,\delta\mathbf{x} + \mathbf{b}^\mathsf{T}\mathbf{W}\mathbf{b}
$$

展开时用到 W 对称，故 δxᵀGᵀWb = bᵀWGδx（标量的转置等于自身）。

**一阶条件。** 利用 ∂(xᵀAx)/∂x = 2Ax（A 对称）与 ∂(aᵀx)/∂x = a：

$$
\frac{\partial J}{\partial \delta\mathbf{x}} = 2\,\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G}\,\delta\mathbf{x} + 2\,\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{b} = \mathbf{0}
$$

**法方程与闭式解。**

$$
\underbrace{\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G}}_{\mathbf{N}}\,\delta\mathbf{x} = -\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{b}
\quad\Longrightarrow\quad
\delta\mathbf{x} = -\left(\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G}\right)^{-1}\mathbf{G}^\mathsf{T}\mathbf{W}\,\mathbf{b}
$$

**极小值与存在条件** Hessian 为 2N。W 正定且 G 列满秩（rank = 7）时 N 正定，驻点是唯一极小值。列满秩至少需要 3 + （参与的星座数）颗星，并且卫星不能共面；缺失星座的钟差列全零，要靠第 4 节的约束补秩。**。**

**统计意义。** 若取 W = Σ⁻¹（Σ 为伪距噪声协方差），由 Gauss–Markov 定理，该解是线性无偏估计中方差最小的（BLUE）；噪声还服从高斯分布时，它等价于极大似然估计。代码中的 W 是经验权、不是严格的 Σ⁻¹，见第 5 节。

**Gauss–Newton 迭代。** 线性化只在 xₖ 附近成立，所以重复「线性化 → 求 δx → 更新」：

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \delta\mathbf{x}_k, \qquad \text{直到 } \lVert\delta\mathbf{x}_k\rVert < \epsilon \text{ 或 } k = k_{max}
$$

1. 初值 x₀ = 0（地心、钟差为 0）。首次迭代接收机位置为零，代码跳过大气改正，高度角取默认值 90°，故第一步所有卫星都参与。
2. 从第二次起按当前位置计算高度角、对流层和电离层延迟，并剔除低于截止角的卫星。通常 5–7 次收敛。
3. 收敛判据是状态增量范数，位置与钟差单位都是 m。alg\_cfg 中 `convergence_threshold: 1.0e-4`、`max_iterations: 30`；达到迭代上限时返回零向量表示失败。

数值上，代码对 N 直接求逆（`.inverse()`）。7×7 规模下影响不大，但几何较差时改用 `N.ldlt().solve(...)` 更稳定。

## 4. 多星座钟差的可观性

某个星座本历元没有观测时，它的钟差列在 G 中全为零，N 奇异。代码为每个缺失星座补一行伪观测「c·δt\_s = 0」，把它压在 0 附近：

$$
\mathbf{g}_c^\mathsf{T} = \begin{bmatrix} \mathbf{0}_{1\times3} & \mathbf{u}_s^\mathsf{T} \end{bmatrix}, \qquad b_c = 0, \qquad w_c = 1000
$$

增广后的法矩阵只在该钟差的对角元上加一项，相当于对它做强正则化：

$$
\mathbf{N}' = \mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G} + w_c\,\mathbf{e}_{3+s}\mathbf{e}_{3+s}^\mathsf{T}, \qquad \left(\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{b}\right)' = \mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{b}
$$

由于该列与其他状态无耦合，得到 δ(c·δt\_s) = 0。该钟差从初值 0 始终不动，其它 6 个状态的解与去掉这一列后的解完全相同。b\_c = 0 之所以成立，正是因为当前估计一直是 0。

有一个边界情况要注意：星座是否缺失（`sys_mask`）是按截止角筛选前的 `valid_obs` 判定的。若某星座的卫星全部低于截止角，它既没有观测行，也不会补约束行，N 仍然奇异。把掩码改为按 `good_idx` 统计即可避免。

## 5. 权阵设计

W 取对角阵（各星伪距噪声视为独立），每个权值由高度角、接收机给出的伪距标准差和星历 URA 三部分相乘得到：

$$
w_i = \sin^2 E_i \cdot \frac{0.16}{\sigma_{P,i}} \cdot \frac{1}{k_{s}(\mathrm{URA}_i)}
$$

| 因子 | 代码中的形式 | 说明 |
| --- | --- | --- |
| 高度角 | sin²E | 对应 σ² ∝ 1/sin²E，低仰角的大气与多路径误差大 |
| 伪距标准差 | ÷ (σ\_P / 0.16)，仅 σ\_P > 0 时 | 0.16 m 为参考标准差；RINEX 观测没有该字段（为 0），此项不生效 |
| GPS / BDS / QZSS | ÷ (URA − 1) | 星历 `ura` 字段 |
| Galileo | ÷ (URA − 2) | 星历 `ura` 字段 |
| GLONASS | ÷ 4 | 无 URA，取常数 |
| 钟差约束行 | 1000 | 见第 4 节 |

与理论最优权 W = Σ⁻¹ 相比，有两处不同：

- 伪距标准差按一次方进入权值，而 Σ⁻¹ 要求 1/σ\_P²。当前写法对大 σ\_P 的降权弱一些：σ\_P 翻倍时权只减半，而不是降到 1/4。
- URA 项是一个经验缩放，不是 URA²（米）；若星历里存的是 URA 索引而不是米，这个公式对索引没有物理意义。

**建议修改：严格的 Σ⁻¹ 权。** 把接收机端噪声（随高度角放大）和星历端误差（URA）当作独立误差，按方差相加后取倒数：

$$
\sigma_i^2 = \frac{\sigma_{P,i}^2}{\sin^2 E_i} + \sigma_{URA,i}^2, \qquad w_i = \frac{1}{\sigma_i^2}
$$

- σ\_P：接收机给出的伪距标准差，单位 m。缺失时（RINEX 观测为 0）取默认值，例如 0.3 m。
- σ\_URA：星历用户测距精度，单位 m。若 `ura` 字段是 URA 索引，先按 GPS ICD 表换算成米（索引 0–6 对应 2.4–13.65 m）；GLONASS 没有 URA，取常数（例如 5 m）代替现在的 ÷ 4。
- 钟差约束行的权 1000 对应 σ ≈ 3 cm，可以保留。

这样 W 就是 Σ⁻¹（σ₀² = 1），第 6 节的 (GᵀWG)⁻¹ 直接就是以 m² 为单位的协方差；σ̂₀² 远大于 1 时，说明有粗差或模型偏乐观，可以做 χ² 检验。对应 `psr_pos` 的改动：

```cpp
const double sin_el = sin(all_sv_azel[origin_idx].y());
const double psr_std = this_obs->psr_std[l1_idx] > 0 ? this_obs->psr_std[l1_idx] : 0.3; // m
const double ura_m = obs_sys == SYS_GLO ? 5.0 : ura_index_to_m(valid_ephems[origin_idx]->ura);
good_W(i, i) = 1.0 / (psr_std * psr_std / (sin_el * sin_el) + ura_m * ura_m);
```

`ura_index_to_m` 是需要新增的查表函数。Galileo 的 SISA 编码与 GPS 不同，要单独换算。

spp\_solver\_node 在调用 solver 前会先拒绝产生非正权的观测（URA − 1 ≤ 0 等）。由于 WLS 解只依赖权值之间的相对大小，权阵整体乘一个常数不改变 δx；但它会改变协方差的绝对量级，见第 6 节。

## 6. 精度评估

收敛后的状态协方差为 σ̂₀²(GᵀWG)⁻¹，当前代码没有输出它，NavSatFix 的协方差类型为 UNKNOWN。

**协方差传播。** δx 是 b 的线性函数 δx = −N⁻¹GᵀWb。设伪距噪声协方差为 Σ：

$$
\mathrm{Cov}(\hat{\mathbf{x}}) = \mathbf{N}^{-1}\mathbf{G}^\mathsf{T}\mathbf{W}\,\boldsymbol{\Sigma}\,\mathbf{W}\mathbf{G}\,\mathbf{N}^{-1}
\;\xrightarrow{\;\mathbf{W} = \sigma_0^{-2}\boldsymbol{\Sigma}^{-1}\;}\;
\sigma_0^2\,\left(\mathbf{G}^\mathsf{T}\mathbf{W}\mathbf{G}\right)^{-1}
$$

**单位权方差。** 第 5 节的权只定义了相对大小，σ₀² 用收敛时最后一次迭代的残差 v = b 估计，n 为参与的观测行数，m = 7（含约束行时两者同时计入）：

$$
\hat{\sigma}_0^2 = \frac{\mathbf{v}^\mathsf{T}\mathbf{W}\mathbf{v}}{n - m}
$$

n − m ≤ 0 时没有多余观测，无法估计 σ₀²，也无法做残差检验。

**DOP。** 取 W = I，只看几何：Q = (GᵀG)⁻¹。位置块是 ECEF。用 ENU→ECEF 旋转矩阵 R（代码中 `geo2rotation(lla)`）转到当地坐标系：

$$
\mathbf{Q}_{enu} = \mathbf{R}^\mathsf{T}\,\mathbf{Q}_{xyz}\,\mathbf{R}, \qquad
\mathrm{HDOP} = \sqrt{Q_{ee} + Q_{nn}},\quad \mathrm{VDOP} = \sqrt{Q_{uu}},\quad \mathrm{PDOP} = \sqrt{\mathrm{tr}\,\mathbf{Q}_{xyz}}
$$

多星座时，每个钟差都有自己的 TDOP\_s = √Q\_{3+s,3+s}。增加一个星座会多一个待估钟差，只有该星座至少贡献 2 颗卫星时，位置精度才会改善。

位置标准差约为 σ\_ENU ≈ σ\_UERE × DOP。例如 σ\_UERE = 3 m、HDOP = 1.2 时，水平精度约 3.6 m。

## 7. 与 gnss\_comm 代码的对应

公式与实现的对应关系如下，文件为 `gnss_comm/src/solver/spp.cpp`。

| 步骤 | 公式 | 代码位置 |
| --- | --- | --- |
| 筛选 L1 类观测与星历配对 | — | `filter_L1` |
| 发射时刻、卫星位置/钟差/TGD | t\_tx = t\_r − P/c − δt^i | `sat_states`（`eph2svdt` → `eph2pos`，GLONASS 用 `geph2*`） |
| 预测伪距 h(x) 与残差 b | 第 1–2 节 | `psr_res`：`psr_estimated`、`res(i)` |
| 雅可比行 gᵢᵀ | \[−eᵢᵀ, u\_sᵀ\] | `psr_res`：`J.block(i,0,1,3) = -unit_rv2sv`、`J(i,3+sys) = 1` |
| 高度角截止 | Eᵢ > E\_cut | `psr_pos`：`good_idx` |
| 权阵 W | 第 5 节 | `psr_pos`：`good_W(i,i) = weight` |
| 缺失星座钟差约束 | 第 4 节 | `psr_pos`：`sys_mask`、`good_W = 1000` |
| 闭式增量 | δx = −(GᵀWG)⁻¹GᵀWb | `psr_pos`：`dx = -(G'WG).inverse() G'W b` |
| 迭代与收敛 | ‖δx‖ < ε | `psr_pos`：`while` 循环，`max_iterations` / `convergence_threshold` |
| 输出 | \[r\_r, cδt\_GPS, cδt\_GLO, cδt\_GAL, cδt\_BDS\] | `psr_pos` 返回值；节点转成 LLA/ENU 发布并写 CSV |

多普勒测速 `dopp_vel` 的结构完全相同：状态是 \[v\_r, c·dδt\]，观测是 −λ·D，雅可比仍为 \[−eᵢᵀ, 1\]，闭式解同第 3 节。
