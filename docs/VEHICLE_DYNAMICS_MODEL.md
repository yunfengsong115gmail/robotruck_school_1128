# 三轴矿用卡车动力学模型

> 基于项目 PPT《无人驾驶矿车动力学建模及控制方案》提取  
> 版本：2026-06-24

---

## 一、模型概述

### 1.1 车辆构型

三轴矿用卡车（6轮），前轴转向，中后轴驱动。

```
         L (总轴距)
    |<──────────────────>|
    
    ◎──────────────────◎──────────────────◎
    │←── a ──→│←─ b ──→│←─── c ────→│
    │          │         │              │
   前轴 (F)   中轴 (M)   后轴 (R)
   转向轴     驱动轴     驱动轴
```

### 1.2 简化假设

| 假设 | 说明 |
|------|------|
| 自行车模型 | 左右轮合并为单轨，侧向动力学简化为2轮 |
| 小角度近似 | 转向角、侧滑角均 ≤ 10° |
| 线性轮胎 | 侧向力与侧滑角成正比：Fyj = Cj × αj |
| 恒定车速 | 在侧向动力学分析中 vx 视为准静态参数 |
| 平坦路面 | 忽略坡度、侧倾对侧向动力学的影响 |

### 1.3 符号表

| 符号 | 含义 | 单位 |
|------|------|------|
| m | 整车质量 | kg |
| Iz | 绕 Z 轴转动惯量 | kg·m² |
| a | 质心到前轴距离 | m |
| b | 质心到中轴距离 | m |
| c | 质心到后轴距离 | m |
| L | 总轴距 = a+b+c | m |
| vx | 纵向速度 | m/s |
| vy | 侧向速度 | m/s |
| β | 质心侧偏角 = vy/vx | rad |
| γ | 横摆角速度 | rad/s |
| δ | 前轮转角 | rad |
| Cf, Cm, Cr | 前/中/后轴侧偏刚度 | N/rad |
| Cxf, Cxm, Cxr | 前/中/后轴纵向滑移刚度 | N |
| μ_fl … μ_rr | 6轮路面附着系数 | — |
| αf, αm, αr | 前/中/后轮胎侧偏角 | rad |
| Fyf, Fym, Fyr | 前/中/后轴侧向力 | N |

---

## 二、3-DOF 自行车模型

### 2.1 轮胎侧偏角

各轴轮胎侧偏角（小角度近似）：

$$\\alpha_f = \\beta + \\frac{a \\gamma}{v_x} - \\delta$$

$$\\alpha_m = \\beta - \\frac{b \\gamma}{v_x}$$

$$\\alpha_r = \\beta - \\frac{c \\gamma}{v_x}$$

### 2.2 线性轮胎模型

侧向力与侧偏角成正比（线性区）：

$$F_{yf} = C_f \\cdot \\alpha_f$$

$$F_{ym} = C_m \\cdot \\alpha_m$$

$$F_{yr} = C_r \\cdot \\alpha_r$$

### 2.3 运动方程

**侧向运动：**

$$m(\\dot{v}_y + v_x \\gamma) = F_{yf} + F_{ym} + F_{yr}$$

代入侧偏角表达式，整理得：

$$\\dot{\\beta} = \\frac{C_f + C_m + C_r}{m v_x} \\beta + \\left(\\frac{a C_f - b C_m - c C_r}{m v_x^2} - 1\\right) \\gamma - \\frac{C_f}{m v_x} \\delta$$

**横摆运动：**

$$I_z \\dot{\\gamma} = a F_{yf} - b F_{ym} - c F_{yr}$$

整理得：

$$\\dot{\\gamma} = \\frac{a C_f - b C_m - c C_r}{I_z} \\beta + \\frac{a^2 C_f + b^2 C_m + c^2 C_r}{I_z v_x} \\gamma - \\frac{a C_f}{I_z} \\delta$$

### 2.4 状态空间形式

定义：

$$\\sigma = C_f + C_m + C_r$$
$$\\rho = a C_f - b C_m - c C_r$$
$$\\kappa = a^2 C_f + b^2 C_m + c^2 C_r$$

状态向量 $x = [\\beta, \\gamma]^T$，控制输入 $u = \\delta$：

$$\\begin{bmatrix} \\dot{\\beta} \\\\ \\dot{\\gamma} \\end{bmatrix} = \\begin{bmatrix} -\\frac{\\sigma}{m v_x} & -1 - \\frac{\\rho}{m v_x^2} \\\\ -\\frac{\\rho}{I_z} & -\\frac{\\kappa}{I_z v_x} \\end{bmatrix} \\begin{bmatrix} \\beta \\\\ \\gamma \\end{bmatrix} + \\begin{bmatrix} \\frac{C_f}{m v_x} \\\\ \\frac{a C_f}{I_z} \\end{bmatrix} \\delta$$

观测方程：

$$y = \\begin{bmatrix} a_y \\\\ \\dot{\\gamma} \\end{bmatrix} = \\begin{bmatrix} -\\frac{\\sigma}{m} & -\\frac{\\rho}{m v_x} \\\\ 0 & 1 \\end{bmatrix} \\begin{bmatrix} \\beta \\\\ \\gamma \\end{bmatrix} + \\begin{bmatrix} \\frac{C_f}{m} \\\\ 0 \\end{bmatrix} \\delta$$

其中侧向加速度 $a_y = \\dot{v}_y + v_x \\gamma$。

---

## 三、Dugoff 轮胎模型（非线性）

线性轮胎模型仅在侧向力 < 峰值附着力的 50% 时准确。Dugoff 模型引入了附着系数的非线性。

### 3.1 单轮胎力

轮胎纵向滑移率 $s_j$、侧偏角 $\\alpha_j$：

轮胎刚度缩减因子：

$$\\lambda_j = \\frac{\\mu_j F_{zj} (1 - s_j)}{2 \\sqrt{(C_{xj} s_j)^2 + (C_j \\tan \\alpha_j)^2}}$$

非线性修正函数：

$$f(\\lambda_j) = \\begin{cases} (2 - \\lambda_j) \\lambda_j, & \\lambda_j < 1 \\\\ 1, & \\lambda_j \\ge 1 \\end{cases}$$

纵向力与侧向力：

$$F_{xj} = C_{xj} \\cdot \\frac{s_j}{1 - s_j} \\cdot f(\\lambda_j)$$

$$F_{yj} = C_j \\cdot \\frac{\\tan \\alpha_j}{1 - s_j} \\cdot f(\\lambda_j)$$

### 3.2 垂向载荷（含动态转移）

6 轮独立垂向载荷（考虑纵/侧向加速度的载荷转移）：

**静态载荷**（前轴2轮、中轴2轮、后轴2轮）：

$$F_{zf\\_static} = \\frac{mg(b+c)}{2L}, \\quad F_{zm\\_static} = \\frac{mg(a+c)}{2L}, \\quad F_{zr\\_static} = \\frac{mg(a+b)}{2L}$$

**动态修正**（加减速和转向时的载荷转移）：

$$F_{z,fl} = F_{zf\\_static} - \\Delta F_{x} - \\Delta F_{y,f}$$
$$F_{z,fr} = F_{zf\\_static} - \\Delta F_{x} + \\Delta F_{y,f}$$
$$F_{z,ml} = F_{zm\\_static} + \\Delta F_{x} - \\Delta F_{y,m}$$
$$F_{z,mr} = F_{zm\\_static} + \\Delta F_{x} + \\Delta F_{y,m}$$
$$F_{z,rl} = F_{zr\\_static} + \\Delta F_{x} - \\Delta F_{y,r}$$
$$F_{z,rr} = F_{zr\\_static} + \\Delta F_{x} + \\Delta F_{y,r}$$

其中：

$$\\Delta F_x = \\frac{m a_x h}{2L} \\quad \\text{（纵向载荷转移）}$$

$$\\Delta F_{y,f} = \\frac{m a_y h}{B_f} \\cdot \\frac{b+c}{L} \\quad \\text{（前轴侧向载荷转移）}$$

$$\\Delta F_{y,m} = \\frac{m a_y h}{B_m} \\cdot \\frac{a}{L} \\quad \\text{（中轴侧向载荷转移）}$$

$$\\Delta F_{y,r} = \\frac{m a_y h}{B_r} \\cdot \\frac{a}{L} \\quad \\text{（后轴侧向载荷转移）}$$

$h$ = 质心高度，$B_f, B_m, B_r$ = 各轴轮距

### 3.3 完整 6 轮动力学

将各轮纵向力和侧向力投影到车体坐标系：

**纵向合力：**

$$\\sum F_x = \\sum_{j} (F_{xj} \\cos \\delta_j - F_{yj} \\sin \\delta_j)$$

其中 $\\delta_f = \\delta$，$\\delta_m = \\delta_r = 0$。

**侧向合力：**

$$\\sum F_y = \\sum_{j} (F_{xj} \\sin \\delta_j + F_{yj} \\cos \\delta_j)$$

**横摆力矩：**

$$M_z = a(F_{x,fl} \\sin \\delta + F_{y,fl} \\cos \\delta + F_{x,fr} \\sin \\delta + F_{y,fr} \\cos \\delta)$$
$$\\quad - b(F_{y,ml} + F_{y,mr}) - c(F_{y,rl} + F_{y,rr})$$
$$\\quad + \\frac{B_f}{2}(F_{x,fr} \\cos \\delta - F_{y,fr} \\sin \\delta - F_{x,fl} \\cos \\delta + F_{y,fl} \\sin \\delta)$$
$$\\quad + \\frac{B_m}{2}(F_{x,mr} - F_{x,ml}) + \\frac{B_r}{2}(F_{x,rr} - F_{x,rl})$$

---

## 四、RAUKF 双阶段参数估计

### 4.1 总体框架

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────┐
│  传感器数据   │────→│  阶段一：动力学参数  │────→│  阶段二：路面  │
│  vx,ay,γ,δ  │     │  辨识 (RAUKF)      │     │  附着系数估计  │
│  轮速,加速度  │     │  输出: m,Iz,Cj,Cxj │     │  输出: μ×6    │
└─────────────┘     └──────────────────┘     └─────────────┘
```

### 4.2 阶段一：动力学参数辨识

**状态向量**（待辨识参数）：

$$x_{stage1} = [m, I_z, C_f, C_m, C_r, C_{xf}, C_{xm}, C_{xr}]^T$$

**观测向量**：

$$z = [a_y, \\dot{\\gamma}]^T$$

**过程模型**：参数随机游走

$$x_{k+1} = x_k + w_k, \\quad w_k \\sim \\mathcal{N}(0, Q)$$

**观测模型**：状态空间输出方程（见 2.4 节）

$$z_k = h(x_k, u_k) + v_k, \\quad v_k \\sim \\mathcal{N}(0, R)$$

### 4.3 阶段二：路面附着系数估计

**状态向量**：

$$x_{stage2} = [\\mu_{fl}, \\mu_{fr}, \\mu_{ml}, \\mu_{mr}, \\mu_{rl}, \\mu_{rr}]^T$$

**约束**：$\\mu_j \\in [0.05, 0.95]$

**观测向量**：

$$z = [a_x, a_y, \\dot{\\gamma}]^T$$

**观测模型**：完整 6 轮 Dugoff 轮胎 + 动力学方程（见 3.3 节）

### 4.4 RAUKF 算法流程

```
1. 初始化状态 x₀ + 协方差 P₀
2. For each timestep k:
   a. Sigma 点生成: X_{k-1} = [x̂, x̂±√((n+λ)P)]
   b. Sigma 点传播: X̂_k = f(X_{k-1}, u_k)
   c. 先验估计: x̂_k¯ = Σ w_i X̂_{k,i}
   d. 先验协方差: P_k¯ = Σ w_i (X̂_{k,i} - x̂_k¯)(...)' + Q_{adaptive}
   e. 观测预测: Ẑ_k = h(X̂_k)
   f. 计算残差: r_k = z_k - Σ w_i Ẑ_{k,i}
   g. 自适应调整 Q: Q_{adaptive} = f(r_k, innovation_cov)
   h. 卡尔曼增益: K_k = P_{xz} P_{zz}⁻¹
   i. 状态更新: x̂_k = x̂_k¯ + K_k r_k
   j. 协方差更新: P_k = P_k¯ - K_k P_{zz} K_kᵀ
```

**残差自适应机制**：

当模型失配导致残差异常增大时，自动放大过程噪声协方差 Q，使滤波器更依赖当前测量而非模型预测：

$$Q_{adaptive} = Q_0 \\cdot \\max\\left(1, \\frac{r_k^T r_k}{\\text{tr}(H P_k¯ H^T + R)}\\right)$$

---

## 五、模型参数表

### 5.1 几何参数

| 参数 | 符号 | 空载 (61t) | 满载 (152t) | 单位 |
|------|------|-----------|------------|------|
| 质心-前轴 | a | 2.2 | 3.884 | m |
| 质心-中轴 | b | 2.0 | 0.316 | m |
| 质心-后轴 | c | 3.9 | 2.216 | m |
| 总轴距 | L | 6.1 | 6.1 | m |
| 质心高度 | h | 1.7 | 2.2 | m |
| 前轮距 | Bf | 3.184 | — | m |
| 中/后轮距 | Bm, Br | 3.324 | — | m |
| 轮胎半径 | Rw | 0.845 | — | m |

### 5.2 轮胎参数（初始值）

| 参数 | 符号 | 空载 | 满载 | 单位 |
|------|------|------|------|------|
| 前轴侧偏刚度 | Cf | 1.4×10⁵ | 2.5×10⁵ | N/rad |
| 中轴侧偏刚度 | Cm | 1.2×10⁵ | 1.2×10⁵ | N/rad |
| 后轴侧偏刚度 | Cr | 1.2×10⁵ | 1.2×10⁵ | N/rad |
| 前轴纵滑刚度 | Cxf | — | — | N |
| 中轴纵滑刚度 | Cxm | — | — | N |
| 后轴纵滑刚度 | Cxr | — | — | N |

### 5.3 惯量参数

| 参数 | 符号 | 空载 | 满载 | 单位 |
|------|------|------|------|------|
| 整车质量 | m | 61,000 | 152,000 | kg |
| Z轴转动惯量 | Iz | 6.5×10⁵ | 2.0×10⁶ | kg·m² |

---

## 六、与项目代码的对应关系

| 模型组件 | 对应代码 | 说明 |
|----------|---------|------|
| 自行车状态空间 | `MPC_controller.py` 内部 A/B 矩阵 | MPC 预测模型 |
| 线性轮胎模型 | `veh_param_est.py` | EKF 参数辨识的观测模型 |
| Dugoff 轮胎模型 | `adhesion_est.py` | UKF 附着系数估计 |
| 质量/坡度 EKF | `mass_slope_est.py` | 辅助参数估计 |
| 运动学模拟 | `mock_truck.py` 第295-307行 | 简版自行车运动学 |
| 完整动力学解算 | TruckSim S-Function (.slx) | 商业软件，真实多体动力学 |

---

## 七、使用建议

### 7.1 控制应用（MPC）

使用 **2.4 节状态空间模型**，配合实时估计的 Cf/Cm/Cr/Iz 参数更新 A/B 矩阵。

### 7.2 参数离线辨识

使用 **4.2 节** 模型，固定动力学结构，用 Optuna + PyTorch 优化参数使 MSE 最小。

### 7.3 PINN 残差补偿

```
物理计算值 = 自行车模型(δ, vx; Cf, Cm, Cr, Iz)
NN 补偿项 = MLP(vx, m, δ, ax, ay, γ, λ)
最终预测 = 物理计算值 + NN 补偿项
```

物理模型始终在线保证稳定性，神经网络补偿非线性未建模动态。

---

*文档版本 1.0 — 2026-06-24*
*数据来源：《矿车项目交流7.21.pptx》+ 项目源码分析*
