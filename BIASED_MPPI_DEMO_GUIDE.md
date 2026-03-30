# Biased-MPPI 算法演示指南 (E2E 先验引导)

## 1. 演示目的
本演示程序 (`biased_mppi_demo`) 旨在展示如何通过外部提供的**多模态先验轨迹**（模拟端到端神经网络输出）来引导 MPPI 算法。在复杂的避障场景中，这种引导可以帮助 MPPI 克服局部最优解，更快地收敛到理想的避障路径。

## 2. 核心机制

### 2.1 模拟 E2E 输出
在 `biased_mppi_demo.cpp` 中，`GenerateDummyE2EPriors` 函数模拟了端到端模型输出的两条多模态轨迹：
- **Mode 1 (左绕行)**: 基于参考线向左偏移 2.0 米。
- **Mode 2 (右绕行)**: 基于参考线向右偏移 2.0 米。

这些轨迹被转换为 MPPI 的控制量序列 (`[jerk, steer_rate]`) 并作为 `Priors` 加载到采样器中。

### 2.2 采样分布分布
采样器此时维护 $M+1 = 3$ 个高斯分布：
- **Distribution 0**: 名义分布 (Nominal)，延续上一帧的最优解。
- **Distribution 1**: 对应“左绕行”先验。
- **Distribution 2**: 对应“右绕行”先验。

### 2.3 偏差修正 (Bias Correction)
CUDA 核函数 `BiasedWeightCorrectionKernel` 会实时计算每条采样轨迹相对于这三个分布的似然比，并通过 `Log-Sum-Exp` 技巧计算修正后的总代价 $\tilde{S}(V)$。

## 3. 构建与运行

### 3.1 编译
在 Ubuntu 容器或支持 CUDA 的环境中运行：
```bash
cd mppi-in-autonomous-driving
mkdir -p build && cd build
cmake .. -DCMAKE_CUDA_ARCHITECTURES=86
make biased_mppi_demo -j8
```

### 3.2 运行
启动演示节点：
```bash
./biased_mppi_demo -c ../config/standalone.yaml
```

## 4. 观察与验证

### 4.1 日志输出
观察终端日志：
- `priors_count=2`: 确认两条 E2E 先验已成功加载。
- `cost=X.XXms`: 观察加入偏差修正核函数后的计算耗时（通常增加极小，约 0.1-0.3ms）。

### 4.2 可视化 (Foxglove)
1. 启动 Foxglove Studio 并连接到服务器。
2. 加载 `assets/mppi_layout.json` 布局。
3. **观察重点**：
   - **采样轨迹分布**：你会发现采样轨迹不再仅仅集中在车辆当前轨迹周围，而是呈现出明显的“三股”趋势（中间、左偏、右偏）。
   - **避障行为**：当前方出现障碍物时，由于有了左右绕行的先验引导，MPPI 会迅速“塌陷”到其中一个更有利的模式中，而不会在障碍物正前方产生左右摆动的犹豫（Oscillation）。

## 5. 进阶实验
你可以通过修改 `modules/biased_mppi_demo.cpp` 中的 `offsets` 数组：
- 增加更多模式（例如 `{-3.0, 0.0, 3.0}`）。
- 调整先验的混合系数（在 `alphas_` 中设置非均匀权重），赋予某些模式更高的信任度。
