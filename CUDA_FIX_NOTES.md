# CUDA 编程与 Bias-MPPI 显存管理修复笔记

在 Bias-MPPI 的集成过程中，我们遇到了几个典型的 CUDA 运行时错误（特别是 `misaligned address`）。以下是这些问题的深度复盘、数学原理及规避策略。

---

## 1. 规约算法与 2 的幂次 (Power of 2) 对齐
### 现象
报错：`GPUassert: misaligned address`，位置通常在 `computeNormalizer` 或 `launchWeightedReductionKernel`。

### 原因
*   **Warp/Block Shuffle 优化**：MPPI 内部在计算 Baseline（基准代价）和 Normalizer（归一化因子）时，使用了 CUDA 的**规约（Reduction）**算法。为了追求极致性能，这些核函数通常使用 `__shfl_down_sync` 等指令在线程束（Warp）内进行数据交换。
*   **对齐要求**：这种高性能规约算法通常要求输入的元素总数必须是 **2 的幂次**（如 1024, 2048, 4096）。
*   **冲突点**：在 Bias-MPPI 中，我们引入了 $M$ 个先验分布。总样本数变为 $(M+1) \times NUM\_ROLLOUTS$。如果 $M=2$，总数 $3 \times 4096 = 12288$ **不是** 2 的幂次。规约核函数在处理边界处的树状求和时，会访问到未定义的对齐地址，触发崩溃。

### 修复策略
*   **采样与规约分离**：在全分布 $(M+1)$ 上进行轨迹 Rollout 采样，但在最后计算均值更新时，**只选取第一个分布（Distribution 0）的样本进行规约计算**（前提是该分布已通过权重修正包含了全局信息）。这确保了传入规约核函数的样本数始终是 $NUM\_ROLLOUTS$（2 的幂次）。

---

## 2. 显存指针偏移与共享显存 (Shared vs Individual)
### 现象
报错：`misaligned address` 或计算结果异常。

### 原因
*   **指针误判**：在处理多分布采样器（Sampler）时，我们容易习惯性地认为所有参数（Mean, Sigma）都是按分布数量 $M$ 排列的。
*   **MPPI 库实现细节**：在 `MPPI-Generic` 中，`control_means_d_` 是随分布数 $M$ 线性增长的，但 `std_dev_d_`（采样方差）通常是**所有分布共享一份**。
*   **错误操作**：如果我们使用 `std_dev_d[m * control_dim]` 尝试获取第 $m$ 个分布的方差，指针会指向未分配的显存区域。

### 修复策略
*   **防御性编程**：在访问三方库提供的设备指针时，必须核实其内存分配布局。如果方差是共享的，则所有分布计算 Log-likelihood 时都应指向同一个基地址 `std_dev_d`。

---

## 3. 动态状态下的显存重分配 (Lifecycle Management)
### 现象
现象：增加 E2E 先验模态后，规划器行为错乱或显存溢出。

### 原因
*   **显存延迟更新**：在 CUDA 程序中，Host 侧修改了变量（如分布数量），Device 侧对应的缓存（Buffer）不会自动扩容。
*   **采样器 Buffer**：采样器内部的控制量样本 Buffer 是在初始化时根据 `num_distributions` 分配的。如果后续调用 `setPriors` 增加了分布数，而没有重新触发 `allocate`，核函数就会向已越界的显存区域写入数据。

### 修复策略
*   **显式重分配序列**：在修改关键状态参数（如分布数、时间步长）后，必须执行：
    1.  `sampler_->freeCudaMem()` (采样分布释放)
    2.  更新分布数
    3.  `sampler_->allocateCUDAMemoryHelper()` (采样分布分配)
    4.  `this->deallocateCUDAMemory()` (控制器内部释放)
    5.  `this->allocateCUDAMemoryHelper(...)` (控制器内部重新分配)
*   **RAII 原则**：确保在析构函数中释放自定义的设备内存（如 `alphas_d_`），避免显存泄漏。

---

## 4. CUDA 模板类的构建冲突 (One Definition Rule)
### 现象
报错：`function ... has already been defined`。

### 原因
*   **模板实例化特性**：CUDA 模板类（如 `BiasedMPPIController`）通常在 `.cuh` 中声明并在末尾 `#include ".cu"` 实现。这允许其他源文件通过包含头文件来实例化模板。
*   **构建系统重复包含**：如果在 `CMakeLists.txt` 中将该 `.cu` 文件也列入源文件列表，编译器会尝试将其作为一个独立的编译单元（Object file）进行编译。这会导致在链接阶段出现两份相同的函数实现。

### 修复策略
*   **CMake 过滤**：在 `CMakeLists.txt` 中使用 `list(REMOVE_ITEM)` 将那些作为模板实现的 `.cu` 文件从编译源文件中排除。它们应仅作为“头文件”被其他文件包含。

---

## 5. 常用调试与性能工具
*   **编译选项**：`-expt-relaxed-constexpr` 可以减少 Eigen 等数学库在 CUDA 下的编译告警。
*   **错误捕获**：使用 `HANDLE_ERROR()` 宏包装每一个 CUDA API 调用，能第一时间定位错误位置。
*   **数值兜底**：在核函数中进行除法或 `log` 运算前，使用 `fmaxf(val, epsilon)` 确保分母不为 0，防止产生 `NaN`。
