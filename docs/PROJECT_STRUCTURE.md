# RoboTruck 项目结构说明

> 三轴矿用卡车无人驾驶车辆动力学仿真系统  
> 版本：2026-06-24 | 总大小：~437MB（含编译产物）

---

## 一、项目概览

本项目是伯镭科技三轴矿用卡车（6轮）的无人驾驶仿真系统，包含两个子系统：
- **win_sim/** — 纯仿真模式（TruckSim + Simulink 8-DOF 动力学）
- **win_ros_sim/** — ROS 联合仿真模式（TruckSim + Simulink + ROS 实时参数估计 + MPC 控制）

同时还配有 Dev Container 开发环境配置和项目文档。

---

## 二、顶层目录树

```
robotruck_school_1128/
├── .devcontainer/              # Dev Container 环境配置
│   ├── Dockerfile               #   容器构建文件
│   └── devcontainer.json        #   VS Code 容器连接配置
│
├── .gitignore                   # Git 忽略规则
├── readme.md                    # 项目 README
├── README.md                    # README（英文）
├── architecture.html            # 系统架构可视化
│
├── docs/                        # 文档
│   ├── ENV_SETUP_GUIDE.md       #   环境搭建手册（577行）
│   └── 参数辨识与模型优化方案.md  #   参数辨识技术方案
│
├── offline_opt/                 # 离线优化算法（预留）
│   └── data/                    #   实车数据存放处（.gitignore）
│
├── win_sim/                     # 纯仿真子系统
│   ├── truck_mpc.cpar           #   TruckSim 配置
│   └── truck_mpc_8dof_2020a.slx #   Simulink 8-DOF 动力学模型
│
└── win_ros_sim/                 # ROS 联合仿真子系统
    ├── truck_model.cpar          #   TruckSim 配置
    ├── truckt_ROS_mpc_8dof_2020a.slx  # Simulink 模型（含 ROS 接口）
    └── ws/                       #   ROS catkin workspace
        ├── src/                  #     源代码
        │   ├── common/
        │   │   ├── robominer_msgs/   # 伯镭自定义 ROS 消息
        │   │   └── simulation/
        │   │       └── mock_truck/   # MPC 控制器 + 车辆模拟
        │   └── estimation/           # 参数估计（3个算法）
        ├── build/                    # CMake 编译产物（自动生成）
        └── devel/                    # 编译输出（自动生成）
```

---

## 三、文件清单与说明

### 3.1 环境配置 (.devcontainer/)

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `Dockerfile` | 0.7K | 24 | 容器镜像定义：ROS Noetic + PyTorch + Optuna |
| `devcontainer.json` | 0.6K | 22 | VS Code 容器连接配置（含 GPU 直通、工作区挂载） |

### 3.2 项目根目录

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `.gitignore` | 0.3K | 33 | Git 忽略规则（排除数据文件、编译产物、缓存） |
| `readme.md` | 2.2K | 55 | 项目简介 |
| `README.md` | 6.0K | 93 | 项目 README（英文） |
| `architecture.html` | 18.8K | 304 | 系统架构交互式文档 |

### 3.3 文档 (docs/)

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `ENV_SETUP_GUIDE.md` | 14.8K | 581 | 环境搭建手册：从零到可用（含 GPU、WSL、Docker） |
| `参数辨识与模型优化方案.md` | 7.3K | 198 | 参数辨识技术方案：EKF/UKF/PINN 架构设计 |

### 3.4 纯仿真子系统 (win_sim/)

纯 Simulink 仿真，不依赖 ROS，适合快速验证动力学模型。

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `truck_mpc.cpar` | 2.2M | 17,778 | TruckSim 整车参数配置（空载/满载、悬架、轮胎） |
| `truck_mpc_8dof_2020a.slx` | 100.4K | — | Simulink 8自由度车辆动力学模型（MATLAB 2020a） |

### 3.5 ROS 联合仿真子系统 (win_ros_sim/)

TruckSim ⇄ Simulink ⇄ ROS 三端联合仿真，包含完整参数估计和控制算法。

#### 3.5.1 顶层

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `truck_model.cpar` | 3.1M | 24,114 | TruckSim 配置（含 ROS 联合仿真参数） |
| `truckt_ROS_mpc_8dof_2020a.slx` | 3.0M | — | Simulink 模型（含 ROS Publish/Subscribe 接口） |

#### 3.5.2 ROS 工作区 (ws/)

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `.catkin_workspace` | 0.1K | 1 | catkin 工作区标记文件 |
| `readme.md` | 2.2K | 55 | 工作区说明 |
| `src/CMakeLists.txt` | 0.05K | 1 | 顶层 CMake |

#### 3.5.3 参数估计功能包 (estimation/)

三个实时参数估计算法，均为 30Hz ROS 节点。

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `CMakeLists.txt` | 1.0K | 58 | 编译配置 |
| `package.xml` | 1.3K | 38 | ROS 包元信息 |

**自定义消息：**

| 文件 | 行数 | 说明 |
|------|------|------|
| `msg/MassSlopeEst.msg` | 5 | 质量/坡度估计输出消息 |
| `msg/VehParamEst.msg` | 8 | 车辆参数估计输出消息 |
| `msg/AdhesionEst.msg` | 10 | 附着系数估计输出消息 |

**算法源码：**

| 文件 | 大小 | 行数 | 算法 | 估计目标 |
|------|------|------|------|----------|
| `src/mass_slope_est.py` | 5.5K | 171 | **3态 EKF** | 整车质量 m + 坡度 tan(β) |
| `src/veh_param_est.py` | 17.6K | 433 | **UKF→EKF 级联** | Cf/Cm/Cr/Cxf/Cxm/Cxr/Iz |
| `src/adhesion_est.py` | 22.1K | 526 | **6态 UKF + Dugoff 轮胎模型** | 6个轮胎附着系数 μ₁~μ₆ |

**启动配置：**

| 文件 | 行数 | 说明 |
|------|------|------|
| `launch/estimation_launch.launch` | 14 | 一键启动三个估计节点 |

#### 3.5.4 控制器 + 车辆模拟 (mock_truck/)

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `CMakeLists.txt` | 7.5K | 230 | 编译配置 |
| `package.xml` | 3.0K | 71 | ROS 包元信息 |

**核心算法：**

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `src/mock_truck.py` | 25.0K | 610 | **车辆模拟主程序**：接收控制指令，模拟车辆运动，发布仿真传感器数据 |
| `src/MPC_controller.py` | 10.3K | 309 | **模型预测控制器**：4态自行车模型，N=50预测时域，30Hz |
| `src/simple_controller.py` | 1.5K | 57 | **简版 PID 控制器**：测试用 |

**启动与可视化：**

| 文件 | 大小 | 行数 | 说明 |
|------|------|------|------|
| `launch/mock_truck_simple.launch` | 2.5K | 55 | 一键启动模拟 + 控制器 |
| `rviz/simu.rviz` | 13.5K | 498 | RViz 可视化配置 |

#### 3.5.5 伯镭自定义消息 (robominer_msgs/)

伯镭科技自研的 ROS 消息定义包，包含 80+ 种自定义消息类型，是全部系统的通信协议基础。

| 文件 | 行数 | 说明 |
|------|------|------|
| `CMakeLists.txt` | 9.9K/358 | 消息编译配置 |
| `package.xml` | 3.2K/78 | 包元信息 |

**消息分类：**

**底盘与控制（VCU / Chassis）：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `VcuData.msg` | 229 | **VCU 整车数据**（最大最核心的消息）：车速、扭矩、档位、制动、转向、故障码等 |
| `ControlCmd.msg` | 11 | 控制指令：转向角、速度、制动 |
| `ControlCmdAux.msg` | 35 | 辅助控制指令 |
| `ChassisEnableCtrl.msg` | 9 | 底盘使能控制 |
| `ChassisError.msg` | 7 | 底盘故障码 |
| `VcuErrCode.msg` | 8 | VCU 错误码 |
| `VcuSysErrLevel.msg` | 12 | VCU 系统错误等级 |
| `AutoControllerInfo.msg` | 24 | 自动驾驶控制器信息 |
| `AutoDriveStatus.msg` | 13 | 自动驾驶状态 |
| `VehicleData.msg` | 112 | 整车数据（传感器汇总） |
| `VehicleError.msg` | 34 | 整车故障 |
| `VehicleStatus.msg` | 28 | 整车状态 |

**定位与感知（Localization / Perception）：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `LocalizationData.msg` | 36 | **定位数据**：ENU坐标、姿态、速度 |
| `LocalizationDataAux.msg` | 25 | 定位辅助数据 |
| `LocalizationStatus.msg` | 10 | 定位状态（RTK/DR/融合） |
| `LocalizationIfdiag.msg` | 14 | 定位诊断 |
| `DetectedObject.msg` | 61 | 检测目标 |
| `DetectedObjectArray.msg` | 4 | 检测目标列表 |
| `CloudCluster.msg` | 36 | 点云聚类 |
| `CloudClusterArray.msg` | 2 | 点云聚类列表 |
| `CloudObstacle.msg` | 4 | 点云障碍物 |
| `CloudObstacles.msg` | 2 | 点云障碍物列表 |
| `RadarObjectData.msg` | 14 | 毫米波雷达目标 |
| `RadarObjectDataArray.msg` | 3 | 雷达目标列表 |
| `PerceptionSensorStatus.msg` | 13 | 感知传感器状态 |

**规划与路径（Planning）：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `GlobalPlan.msg` | 2 | 全局路径 |
| `GlobalPlanRlt.msg` | 20 | 全局路径结果 |
| `LocalPlan.msg` | 11 | 局部路径 |
| `PlanGoal.msg` | 26 | 规划目标 |
| `PlanSrvResult.msg` | 11 | 规划服务结果 |
| `PlannerStatus.msg` | 139 | **规划器状态**（大消息，含避障区详细数据） |
| `PlannerFrame.msg` | 8 | 规划帧 |
| `PlannerCommArea.msg` | 74 | 规划通信区 |
| `PlannerObstStatus.msg` | 21 | 规划障碍物状态 |
| `PlannerObstacleSL.msg` | 8 | 规划障碍物 SL 坐标 |
| `Waypoint.msg` | 19 | 路点 |
| `Waypoints.msg` | 1 | 路点列表 |
| `WaypointType.msg` | 8 | 路点类型 |
| `GuideLine.msg` | 6 | 引导线 |
| `LaneLine.msg` | 9 | 车道线 |
| `TrafficExchangePath.msg` | 12 | 交通交换路径 |
| `RightOfWay.msg` | 9 | 路权 |
| `FreespacePlannerStatus.msg` | 8 | 自由空间规划状态 |
| `PlanningStatus.msg` | 16 | 规划状态 |
| `PredictedObject.msg` | 15 | 预测目标 |

**地图与矿区（Mine/Map）：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `BoonrayArea.msg` | 19 | 伯镭区域定义 |
| `BoonrayMap.msg` | 8 | 伯镭地图 |
| `BoonrayPoint.msg` | 4 | 伯镭坐标点 |
| `BoonrayBorder.msg` | 5 | 边界 |
| `BoonrayLane.msg` | 10 | 车道 |
| `BoonrayObstArea.msg` | 7 | 障碍区 |
| `BoonrayRutArea.msg` | 8 | 车辙区 |
| `Brim.msg` | 5 | 边缘 |
| `BrimPosition.msg` | 6 | 边缘位置 |
| `MultiDump.msg` | 9 | 多点卸料 |
| `MultiDumpCell.msg` | 1 | 卸料单元格 |
| `MultiDumpPoint.msg` | 4 | 卸料点 |
| `EdgePoint.msg` | 14 | 边缘点 |
| `EdgeRectangle.msg` | 4 | 边缘矩形 |
| `EsdfCell.msg` | 4 | ESDF 单元格 |
| `EsdfMap.msg` | 5 | ESDF 地图 |
| `EsdfOccupy.msg` | 3 | ESDF 占据 |

**V2X 通信：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `V2VStatus.msg` | 3 | V2V 状态 |
| `V2VStatusItem.msg` | 9 | V2V 状态条目 |
| `V2VDistribution.msg` | 8 | V2V 分发 |
| `V2VProcess.msg` | 7 | V2V 处理 |
| `V2XCommonDevice.msg` | 16 | V2X 通用设备 |
| `V2XCommonDeviceType.msg` | 10 | V2X 设备类型 |
| `V2XCommonDevices.msg` | 2 | V2X 设备列表 |

**执行器与辅助系统：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `SpeedCmd.msg` | 4 | 速度指令 |
| `Gear.msg` | 8 | 档位 |
| `GearData.msg` | 17 | 档位数据 |
| `Epb.msg` | 4 | 电子驻车 |
| `Pto.msg` | 5 | 取力器 |
| `PtoState.msg` | 11 | 取力器状态 |
| `RatarderState.msg` | 8 | 缓速器状态 |
| `KeyPowerState.msg` | 6 | 钥匙电源状态 |
| `EpowerState.msg` | 7 | 电子功率状态 |
| `DriveMode.msg` | 6 | 驾驶模式 |
| `DriveStatus.msg` | 9 | 驾驶状态 |
| `DiggerStatus.msg` | 5 | 挖掘机状态 |
| `LiftAngle.msg` | 6 | 举升角度 |
| `TyreState.msg` | 13 | 轮胎状态 |
| `TpmsBcan.msg` | 12 | 胎压监测 |

**诊断与记录：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `FaultCode.msg` | 19 | 故障码 |
| `FaultCodes.msg` | 10 | 故障码列表 |
| `FaultLevel.msg` | 6 | 故障等级 |
| `FaultModule.msg` | 10 | 故障模块 |
| `RecordStatus.msg` | 24 | 数据记录状态 |
| `IpcInfo.msg` | 9 | IPC 信息 |

**其他：**

| 文件 | 行数 | 内容 |
|------|------|------|
| `VehStateEst.msg` | 6 | 车辆状态估计 |
| `VciCanObj.msg` | 21 | VCI CAN 对象 |
| `ProtobufMsg.msg` | 2 | Protobuf 消息容器 |
| `NavGridOfDoubles.msg` | 5 | 导航网格 |
| `ProjectSettings.msg` | 23 | 项目设置 |
| `InsdCtrlCmd.msg` | 4 | 内部控制指令 |
| `Centroids.msg` | 2 | 质心 |
| `LineSegment.msg` | 3 | 线段 |
| `BoundingBox.msg` | 17 | 包围盒 |
| `BoundingBoxes.msg` | 2 | 包围盒列表 |
| `Boundary.msg` | 2 | 边界 |
| `GroundPlane.msg` | 19 | 地平面 |
| `GroundPlanes.msg` | 3 | 地平面列表 |
| `PixelPoint3D.msg` | 3 | 3D 像素点 |
| `CurrentVoxel.msg` | 4 | 当前体素 |
| `DeviceBodyCoordinates.msg` | 16 | 设备体坐标系 |
| `DeviceType.msg` | 12 | 设备类型 |
| `OtherDevice.msg` | 15 | 其他设备 |
| `OtherDevices.msg` | 2 | 其他设备列表 |
| `SoftDtu.msg` | 6 | 软 DTU |
| `SoftDtuStatus.msg` | 8 | 软 DTU 状态 |
| `TaskType.msg` | 11 | 任务类型 |
| `SrvRlt.msg` | 4 | 服务结果 |

**服务定义：**

| 文件 | 行数 | 说明 |
|------|------|------|
| `srv/StartPlan.srv` | 13 | 启动规划服务 |
| `srv/StopPlan.srv` | 4 | 停止规划服务 |

---

## 四、系统数据流

```
┌─────────────┐
│  TruckSim    │  车辆动力学仿真（cpar 配置）
└──────┬──────┘
       ↓
┌─────────────┐
│  Simulink    │  8-DOF 模型 + ROS 发布/订阅（slx 文件）
└──────┬──────┘
       ↓ /vcu/vcu_data + /localization/localization_data
┌──────────────────────────────────────────────┐
│              ROS 节点（30Hz 循环）              │
│                                               │
│  ┌─────────────────┐  → /estimation/          │
│  │ mass_slope_est   │     mass_slope_est       │
│  │ (EKF 质量+坡度)   │                          │
│  └─────────────────┘                          │
│                                               │
│  ┌─────────────────┐  → /estimation/          │
│  │ veh_param_est    │     veh_params           │
│  │ (UKF+EKF 参数)   │                          │
│  └─────────────────┘                          │
│                                               │
│  ┌─────────────────┐  → /estimation/          │
│  │ adhesion_est     │     adhesion_est          │
│  │ (UKF 附着系数)    │                          │
│  └─────────────────┘                          │
│                                               │
│  ┌─────────────────┐  → /guardian/            │
│  │ MPC_controller   │     control_cmd          │
│  │ (线性MPC N=50)    │                          │
│  └─────────────────┘                          │
└──────────────────────────────────────────────┘
```

---

## 五、关键技术参数

| 参数 | 值 |
|------|-----|
| 车型 | 三轴矿用卡车（6轮） |
| 车辆总重 | 空载 61t / 满载 152t |
| 轴距 | 前-中 2.2m / 中-后 2.0m（空载），满载时质心后移 |
| 轮胎半径 | 0.845m |
| MPC 预测时域 | N=50 |
| 控制频率 | 30Hz |
| 估计频率 | 30Hz（全部节点） |
| Simulink 模型 | 8-DOF |
| MATLAB 版本 | 2020a |

---

## 六、编译产物说明

以下目录为 ROS catkin 编译自动生成，**不应手动修改，也不纳入 Git 版本控制**：

| 目录 | 大小 | 内容 |
|------|------|------|
| `ws/build/` | ~400MB | CMake 编译中间文件 |
| `ws/devel/` | ~30MB | 编译输出（库、头文件、消息定义） |

---

*文档版本 1.0 — 2026-06-24*
