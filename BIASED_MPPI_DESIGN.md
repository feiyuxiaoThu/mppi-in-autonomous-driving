# Biased-MPPI 集成与黑箱 E2E 多模态轨迹接入设计

## 1. 目标
本项目的目标不是让端到端黑箱网络直接输出最终控制，而是将其输出的多模态轨迹作为 Bias-MPPI 的采样先验分布，从而在保持本地动力学模型、障碍物代价和约束一致性的前提下，提高 MPPI 在复杂场景中的搜索效率和多模态决策能力。

简化地说：

- 黑箱 E2E 网络负责给出“可能去哪”的多个候选模态。
- Bias-MPPI 负责在这些模态附近更密集地采样，并用本地模型和代价函数决定“真正执行哪条”。
- 最终输出仍然是 MPPI 优化出的控制序列，而不是直接采用黑箱轨迹。

## 2. 设计原则

### 2.1 黑箱网络的角色
黑箱网络只提供先验，不提供最终控制闭环。

它最适合输出：

- 多条候选状态轨迹
- 每条轨迹的相对置信度或排序

它不适合直接替代本地规划器的原因有三点：

- 它不保证满足当前车辆动力学和控制约束
- 它不保证与当前障碍物代价定义一致
- 它无法直接替代系统中现有的可解释安全约束链路

### 2.2 Bias-MPPI 的角色
Bias-MPPI 的本质是 informed sampling，而不是 hard selection。

它做的事情是：

- 保留 nominal/warm-start 分布作为基准
- 为每个 E2E 模态增加一个先验分布
- 从混合分布中采样 rollout
- 通过重要性采样修正维持优化目标的一致性
- 在所有 proposal 中选出经本地代价评估后的最优控制

### 2.3 当前仓库中的控制空间
当前系统的动力学模型定义了控制量为：

- `jerk`
- `steer_rate`

状态量为：

- `x`
- `y`
- `velocity`
- `yaw`
- `accel`
- `steer`

因此，任何来自黑箱网络的轨迹，如果想作为 Bias-MPPI 的 prior，必须最终映射到 `[jerk, steer_rate]` 控制空间，而不能直接把 `(x, y, yaw, v)` 当作 MPPI 控制输入。

## 3. 数学形式

### 3.1 采样分布
Bias-MPPI 中的实际采样分布是一个混合分布：

$$
q_s(V) = \sum_{m=0}^{M} \alpha_m \mathcal{N}(V \mid \mu_m, \Sigma)
$$

其中：

- `m = 0` 是 nominal 分布
- `m = 1..M` 是由黑箱网络多模态轨迹转换而来的先验分布
- `alpha_m` 是每个模态的混合系数
- `mu_m` 是对应模态的控制均值序列
- `Sigma` 是所有模态共享的采样协方差

### 3.2 代价修正
为了在混合 proposal 分布下仍然优化 nominal 目标，采样轨迹代价需要修正为：

$$
\tilde{S}(V) = S(V) + \lambda (\ln p(V) - \ln q_s(V))
$$

其中：

- `S(V)` 是环境代价、参考线代价、控制代价、碰撞代价等的总和
- `p(V)` 是 nominal distribution，也就是 Distribution 0 的密度
- `q_s(V)` 是混合采样分布的密度

关键点：

- 这里的 `p(V)` 必须是 nominal 分布，不是“该样本所属分量”的分布
- 如果实现成 `ln q_component(V) - ln q_mix(V)`，那就是另一种近似，不再是本文档定义的 Bias-MPPI

### 3.3 Log-Sum-Exp
混合密度的对数概率必须使用 Log-Sum-Exp 计算：

$$
\ln q_s(V) = \ln \left( \sum_{m=0}^{M} \alpha_m \exp(\ln \mathcal{N}(V \mid \mu_m, \Sigma)) \right)
$$

数值稳定形式：

$$
\ln \left( \sum_i e^{x_i} \right) = x_{max} + \ln \left( \sum_i e^{x_i - x_{max}} \right)
$$

## 4. 黑箱 E2E 多模态轨迹如何进入 Bias-MPPI

这是整个系统中最重要的一层。黑箱网络输出通常是状态空间轨迹，不是控制序列，因此必须经过中间处理链路。

### 4.1 输入定义
建议黑箱网络每帧输出：

- `M` 条候选轨迹
- 每条轨迹由一串时序点组成
- 每个时序点至少包含 `x, y, yaw, v`
- 可选包含 `t`
- 可选包含每条模态的 `score/logit/probability`

在当前仓库里，可统一为：

```cpp
std::vector<std::vector<PathPoint>>
```

其中 `PathPoint` 为：

- `x`
- `y`
- `yaw`
- `v`
- `t`

### 4.2 预处理
黑箱轨迹进入 Bias-MPPI 前，必须先做预处理：

1. 时间对齐
将网络输出重采样到 MPPI 固定 horizon 和固定 `dt`

2. 坐标对齐
确保轨迹位于与当前局部规划器一致的坐标系

3. 起点对齐
轨迹第一个点应与当前 ego 状态连续，否则 prior 会产生不合理的大控制跳变

4. 去噪和平滑
黑箱输出经常存在局部振荡，直接差分会放大成 jerk/steer_rate 尖峰

5. 可行性过滤
剔除明显越界、碰撞、反向或过于不连续的模态

### 4.3 从状态轨迹到控制先验
这是当前实现最需要加强的部分。

当前仓库里已有一个 `TrajectoryToControl` 工具，它通过有限差分把状态轨迹近似映射为：

- 加速度序列
- 转角序列
- 再进一步得到 `jerk` 和 `steer_rate`

这能作为初版验证，但对于黑箱网络轨迹还不够稳健。

更合理的生产方案应该分两层：

1. 几何/运动学反演

- 由 `yaw` 差分得到曲率或转向角
- 由 `v` 差分得到加速度
- 得到粗略的 `(accel, steer)` 参考轨迹

2. 约束一致的控制投影

- 结合车辆 wheelbase 和控制边界
- 使用一个轻量 tracking/inverse controller
- 将 `(x, y, yaw, v)` 投影为可执行的 `[jerk, steer_rate]`

推荐做法：

- 不把黑箱轨迹直接当最终控制
- 把黑箱轨迹变成“prior mean control sequence”
- 必要时加入限幅、平滑和可行性修复

### 4.4 混合系数 alpha 的来源
如果黑箱网络给出了每条模态的置信度，应直接用于 `alpha_m`。

推荐顺序：

1. 使用网络原始概率或 logits
2. 做 softmax 或归一化
3. 保留一部分权重给 nominal distribution

例如：

- `alpha_0 = 0.2`
- 剩余 `0.8` 按网络模态分数分配给 `alpha_1..alpha_M`

如果网络没有显式置信度，可退化为：

- 均匀分配
- 或按轨迹与当前状态连续性/参考线一致性做启发式分配

### 4.5 Nominal 分布与 E2E 分布的职责划分

Distribution 0:

- 上一帧 MPPI 优化出的最优控制序列
- 用于 warm-start
- 是重要性采样修正中的 nominal `p(V)`

Distribution 1..M:

- 由黑箱 E2E 模态轨迹转换而来的控制先验
- 每帧直接刷新
- 不承担闭环稳定职责，只承担搜索引导职责

## 5. 与外部 biased-mppi demo 的对应关系

外部 demo 路径：

- `/Users/feiyushaw/Desktop/workspace/bias-mppi-e2e/bias-mppi`

这个 demo 的核心启发不是“轨迹格式”，而是“prior 的角色”。

在 point robot 示例中：

- `prior` 以回调形式传给 planner
- planner 每个时刻根据当前状态调用 prior
- prior 返回多个辅助控制器输出
- MPPI 在这些 proposal 附近采样，而不是直接执行这些 proposal

对应文件：

- [README](/Users/feiyushaw/Desktop/workspace/bias-mppi-e2e/bias-mppi/README.md)
- [examples/point_robot/run.py](/Users/feiyushaw/Desktop/workspace/bias-mppi-e2e/bias-mppi/examples/point_robot/run.py)
- [examples/point_robot/priors.py](/Users/feiyushaw/Desktop/workspace/bias-mppi-e2e/bias-mppi/examples/point_robot/priors.py)

这个思路迁移到当前仓库时，应理解为：

- 黑箱网络输出的多模态轨迹，本质上就是多个 ancillary proposals
- 它们不是 optimizer 外部的 hard command
- 它们只决定 sampling bias，不决定最终 action

## 6. 当前仓库中的落地点

### 6.1 输入接口
当前 `StochasticOptimizer::plan_once(...)` 已经支持：

```cpp
const std::vector<std::vector<PathPoint>>& e2e_priors
```

这是合理的第一层输入接口。

建议后续补成一个更清晰的结构，例如：

```cpp
struct E2EPriorMode {
  std::vector<PathPoint> trajectory;
  float confidence;
};
```

再传：

```cpp
std::vector<E2EPriorMode>
```

这样可以同时承载轨迹和模态分数。

### 6.2 轨迹转换层
当前 `common/trajectory_utils.hpp` 提供了状态轨迹到控制轨迹的近似转换。

建议将其升级为两个阶段：

1. `ResampleAndSanitizeTrajectory`
2. `ProjectStateTrajectoryToControlPrior`

这样职责更清晰，也便于未来替换成更强的 tracking-based inversion。

### 6.3 Sampler 层
建议在 `BiasedMPPIController` 内部：

- 明确维护 `M+1` 个 control means
- `Distribution 0` 由 nominal control 更新
- `Distribution 1..M` 每帧由新 prior 覆盖
- `alpha_m` 支持外部传入置信度

### 6.4 CUDA 修正层
`BiasedWeightCorrectionKernel` 应满足：

1. 对每条 rollout 计算 `ln p_nominal(V)`
2. 对所有分布计算 `ln q_m(V)`
3. 用 LSE 得到 `ln q_mix(V)`
4. 执行：

```text
S_tilde = S + lambda * (log_p_nominal - log_q_mix)
```

不能使用“样本所属分布”的概率替代 nominal。

### 6.5 更新层
建议使用全体 rollout 的加权结果，仅更新 nominal control sequence。

也就是说：

- Distribution 0 是被优化后的输出分布
- Distribution 1..M 是输入先验，不做自更新累积

这样更符合“黑箱 proposal + MPPI refinement”的系统语义。

## 7. 推荐实现路线

### 阶段 1: 建立最小可运行链路

目标：

- 能把黑箱多模态轨迹以 prior 形式接入
- 能完成多分布采样
- 能完成 nominal-vs-mixture 修正

任务：

1. 保持 `PathPoint` 输入形式
2. 做统一重采样和基本平滑
3. 用简化反演得到 `jerk/steer_rate`
4. 支持每帧刷新多个 prior means
5. 正确实现 importance correction

### 阶段 2: 提升 prior 质量

目标：

- 让黑箱 prior 真正有助于搜索，而不是制造噪声

任务：

1. 加入模态置信度 `alpha_m`
2. 加入轨迹可行性检查
3. 将差分反演升级为 tracking-based inversion
4. 对先验做限幅、平滑和约束投影

### 阶段 3: 提升系统效果

目标：

- 让多模态先验在绕障和岔路场景里真正改变最终输出

任务：

1. 调整 cost，使不同模态 basin 都可能成为可接受解
2. 为高置信模态缩小采样方差
3. 评估 prior 对收敛速度和局部极小值逃逸能力的影响
4. 在 Foxglove 中可视化：
   - 黑箱轨迹
   - prior control rollout
   - nominal rollout
   - 最终最优轨迹

## 8. 当前实现与目标实现的差距

当前仓库已经具备：

- `PathPoint` 轨迹输入
- 轨迹到控制的初版转换
- `BiasedMPPIController` 的基本骨架
- demo 级别的 prior 注入

但距离完整的“黑箱 E2E 多模态轨迹 + Bias-MPPI”仍有几处关键差距：

1. 轨迹到控制的映射仍然偏近似，缺少真正的 tracking/inverse controller
2. `alpha_m` 尚未与黑箱模态置信度真正打通
3. 代价修正必须严格使用 nominal distribution
4. prior 轨迹的可行性过滤和投影仍不完整
5. 缺少针对多模态效果的系统级可视化与评估

## 9. 最终建议
如果要把“端到端黑箱网络的多模态轨迹”稳定地用在当前规划器里，正确路径不是直接替代 MPPI，而是：

1. 黑箱网络输出多模态状态轨迹
2. 对轨迹做重采样、过滤、平滑
3. 将状态轨迹投影到 MPPI 控制空间
4. 将这些控制序列作为 Bias-MPPI 的多个 proposal means
5. 用 nominal-vs-mixture 的重要性采样修正确保数学一致性
6. 仍由本地动力学和代价函数决定最终输出

这条链路兼顾了：

- 黑箱网络的多模态表达能力
- MPPI 的可解释优化能力
- 车辆动力学一致性
- 障碍物与安全约束的一致性

因此，这是当前仓库中接入黑箱 E2E 多模态轨迹的正确工程方向。
