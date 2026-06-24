# RoboTruck 动力学仿真开发环境搭建手册

> 适用：Windows 11 全新电脑 → 完整可用的 ROS + PyTorch 开发环境
> 耗时：约 30-45 分钟（含下载）
> 版本：2026-06-24

\---

## 概述

本手册从一台全新的 Windows 11 电脑开始，逐步搭建一套基于 **WSL + Docker + VS Code Dev Containers** 的自动驾驶算法开发环境。

**最终效果**：在 Windows 的 VS Code 图形界面中编写代码，代码实际运行在一个包含 ROS Noetic + PyTorch + Optuna 的 Linux 容器中。

```
Windows 11
  └─ WSL 2 (Ubuntu)
       └─ Docker 容器
            ├─ ROS Noetic (机器人操作系统)
            ├─ PyTorch 2.4 (深度学习)
            ├─ Optuna (超参数优化)
            └─ NumPy / SciPy / Pandas (数据处理)
```

\---

## 第一步：安装 WSL 2 + Ubuntu

Windows Subsystem for Linux 让你在 Windows 里原生运行 Linux，比传统虚拟机快得多。

### 1.1 启用 WSL

右键开始菜单 → **Windows PowerShell (管理员)**，执行：

```powershell
wsl --install
```

这条命令会自动：

* 启用 WSL 2 虚拟化平台
* 安装 Ubuntu 发行版
* 将 WSL 2 设为默认版本

安装完成后**重启电脑**。

### 1.2 初始化 Ubuntu

重启后，开始菜单会出现 **Ubuntu**。点击打开，首次启动会提示创建用户名和密码。

```
Enter new UNIX username: bolei
New password: \*\*\*\*\*\*
Retype new password: \*\*\*\*\*\*
```

> 建议用户名设为 `bolei`，与后续路径保持一致。

### 1.3 验证

```bash
wsl --version
```

应看到类似输出：

```
WSL 版本： 2.x.x
内核版本： 6.x.x
```

\---

## 第二步：安装 Docker Desktop

Docker 负责创建和管理"容器"——隔离的、可复现的运行环境。

### 2.1 下载安装

1. 访问 https://www.docker.com/products/docker-desktop/
2. 下载 **Docker Desktop for Windows**
3. 运行安装程序，全部默认选项即可
4. 安装完成后**重启电脑**

### 2.2 配置 WSL 集成

1. 启动 Docker Desktop
2. 点击右上角齿轮图标 ⚙️ → **Settings**
3. **General**：确保勾选 **"Use the WSL 2 based engine"**
4. **Resources → WSL Integration**：

   * 确保你的 Ubuntu 发行版开关处于**开启**状态
   * 点击 **Apply \& Restart**

### 2.3 验证

打开 Ubuntu 终端，输入：

```bash
docker --version
docker ps
```

看到版本号和不报错即成功。

\---

## 第三步：安装 Visual Studio Code

### 3.1 下载安装

1. 访问 https://code.visualstudio.com/
2. 下载 **Windows x64** 版本
3. 运行安装程序，建议勾选：

   * "将 Code 添加到 PATH"
   * "将 Code 注册为受支持的编辑器"

### 3.2 安装核心插件

打开 VS Code，点击左侧 **扩展** 图标（或按 `Ctrl+Shift+X`），搜索并安装：

|插件|发布者|用途|
|-|-|-|
|**WSL**|Microsoft|让 VS Code 连接 WSL 里的 Ubuntu|
|**Dev Containers**|Microsoft|在 Docker 容器内开发|

安装完成后，左下角会出现一个绿色图标 `><`，表示远程开发功能就绪。

\---

## 第四步：创建工程目录

在 Ubuntu 终端中执行（所有工程代码放在 WSL 的 Linux 文件系统中，不要放在 `/mnt/c/` 下）：

```bash
# 进入 Linux 用户主目录
cd \~

# 创建项目目录结构
mkdir -p robotruck\_school\_1128/offline\_opt/data
mkdir -p robotruck\_school\_1128/.devcontainer

# 进入项目目录
cd robotruck\_school\_1128
```

目录说明：

```
robotruck\_school\_1128/
  ├── .devcontainer/          # 容器配置文件（施工图纸）
  │   ├── Dockerfile           #   定义容器里装什么软件
  │   └── devcontainer.json    #   定义 VS Code 如何连接容器
  └── offline\_opt/             # 离线优化算法目录
      └── data/                #   实车数据存放处
```

\---

## 第五步：编写容器配置文件

### 5.1 Dockerfile

在 Ubuntu 终端中：

```bash
nano .devcontainer/Dockerfile
```

粘贴以下内容（`Ctrl+Shift+V`），然后 `Ctrl+O` 保存、`Enter` 确认、`Ctrl+X` 退出：

```dockerfile
# 使用 ROS Noetic 作为底座
FROM osrf/ros:noetic-desktop

# 安装基础编译工具
RUN apt-get update \&\& apt-get install -y \\
    python3-pip \\
    git \\
    build-essential \\
    nano \\
    \&\& rm -rf /var/lib/apt/lists/\*

# pip 用清华源加速安装算法库
RUN pip3 install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \\
    catkin-tools \\
    numpy pandas scipy matplotlib optuna \\
    torch torchvision \\
    rospkg bagpy

# 设置默认工作目录
WORKDIR /workspace

# 自动 source ROS 环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> \~/.bashrc
```

**每行解释：**

|行|作用|
|-|-|
|`FROM osrf/ros:noetic-desktop`|基于官方 ROS Noetic 完整镜像|
|`apt-get install python3-pip git build-essential nano`|装 Python 包管理器、Git、C 编译器、文本编辑器|
|`pip3 install catkin-tools`|ROS 编译工具|
|`pip3 install numpy pandas scipy matplotlib optuna`|数据处理 + 可视化 + 超参数优化|
|`pip3 install torch torchvision`|PyTorch 深度学习框架|
|`pip3 install rospkg bagpy`|ROS 包管理 + 数据回放|
|`WORKDIR /workspace`|默认进入 /workspace 目录|
|`source /opt/ros/noetic/setup.bash`|自动加载 ROS 环境变量|

### 5.2 devcontainer.json

```bash
nano .devcontainer/devcontainer.json
```

粘贴：

```json
{
    "name": "RoboTruck-Dynamics-Env",
    "build": {
        "dockerfile": "Dockerfile"
    },
    "workspaceFolder": "/workspace",
    "workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind",
    "customizations": {
        "vscode": {
            "extensions": \[
                "ms-python.python",
                "ms-vscode.cpptools",
                "ms-iot.vscode-ros"
            ],
            "settings": {
                "python.defaultInterpreterPath": "/usr/bin/python3"
            }
        }
    },
    "remoteUser": "root"
}
```

**每项解释：**

|配置项|作用|
|-|-|
|`build.dockerfile`|告诉 VS Code 用哪个 Dockerfile 构建容器|
|`workspaceFolder`|容器内的工作目录|
|`workspaceMount`|将本地项目目录映射到容器内的 `/workspace`|
|`extensions`|容器内自动安装的 VS Code 插件|
|`remoteUser`|以 root 身份运行（开发环境，方便装包）|

\---

## 第六步：构建并进入开发环境

### 6.1 打开项目

在 Ubuntu 终端（确保当前在 `\~/robotruck\_school\_1128` 目录）：

```bash
code .
```

这会在 VS Code 中打开当前目录。

### 6.2 构建容器

VS Code 打开后，右下角会弹出提示：

> \*\*"Folder contains a Dev Container configuration file. Reopen in a container."\*\*

点击 **Reopen in Container**。

> 如果没有弹窗：按 `F1` → 输入 `Dev Containers: Reopen in Container` → 回车。

**首次构建需要 10-20 分钟**。VS Code 在后台：

1. 拉取 ROS Noetic 基础镜像（\~2GB）
2. 安装 PyTorch + 依赖（\~800MB）
3. 将项目目录挂载到容器内

右下角会有进度条，可以点开看详细日志。

### 6.3 验证环境

构建完成后，打开 VS Code 终端（`Terminal → New Terminal`），输入：

```bash
python3 -c "import torch, optuna, scipy, rospy; print('环境部署成功，所有依赖就绪！')"
```

如果输出 **"环境部署成功，所有依赖就绪！"**，恭喜，开发环境已就绪。

\---

## 第七步：后续使用

### 日常开发

1. 打开 VS Code
2. 左下角点 `><` → 选择 **Connect to WSL**
3. 打开 `\~/robotruck\_school\_1128` 目录
4. VS Code 会自动检测 devcontainer 配置，提示 **Reopen in Container**
5. 进入容器后即可开始编码

### 容器内终端

VS Code 的终端（`Ctrl+``）就是容器内的 Shell。可以直接用：

```bash
# 运行 Python 脚本
python3 offline\_opt/train.py

# 查看 ROS 话题
rostopic list

# 安装额外包
pip3 install <package\_name>
```

### 文件位置

|位置|路径|
|-|-|
|Windows 中的项目|`D:\\Project\_Dynamic\_Model\\robotruck\_school\_1128`|
|WSL 中的项目|`\~/robotruck\_school\_1128`|
|容器内的项目|`/workspace`|

> 三个位置通过 Docker bind mount 实时同步，修改任一处都会反映到其他位置。

\---

## 第八步：环境迁移与团队共享

Docker 的核心价值之一是**"一次构建，到处运行"**。当需要换电脑、或者新同事加入项目时，不需要从头再装一遍依赖，直接把环境"搬"过去。

### 8.1 方案对比

| 方案 | 适合场景 | 速度 | 体积 |
|------|---------|------|------|
| **导出镜像文件** | 换电脑、离线环境 | 快 | ~7GB 压缩包 |
| **共享 Dockerfile** | 团队协作（推荐） | 需重建 15min | 几 KB |
| **推送到镜像仓库** | 团队规模大、CI/CD | 中等 | 取决于网络 |

### 8.2 方案一：导出镜像文件（换电脑 / 给同事）

#### 导出

在已配置好环境的电脑上：

```bash
# 将镜像打包成 tar 文件
docker save -o robotruck-dynamics-env.tar robotruck-dynamics-env:latest

# 压缩一下（7GB → 约 3GB）
gzip robotruck-dynamics-env.tar
```

得到 `robotruck-dynamics-env.tar.gz`（约 3GB），用 U 盘拷贝或网络传输给目标电脑。

#### 导入

在目标电脑上（已装好 Docker Desktop 即可）：

```bash
# 解压并导入镜像
gunzip robotruck-dynamics-env.tar.gz
docker load -i robotruck-dynamics-env.tar

# 验证
docker images robotruck-dynamics-env
```

然后把项目文件夹（含 `.devcontainer/`）拷贝过去，VS Code 打开 → Reopen in Container，**秒进，不用重新构建**。

### 8.3 方案二：共享 Dockerfile（推荐团队使用）

其实**不需要传整个 7GB 镜像**。只要把 `.devcontainer/` 目录（两个文件，几 KB）连同项目代码一起放到 Git 仓库里：

```
robotruck_school_1128/
  ├── .devcontainer/
  │   ├── Dockerfile           # ← 这就是环境的"DNA"
  │   └── devcontainer.json
  ├── docs/
  ├── offline_opt/
  └── ...
```

新同事 clone 代码后：

```bash
cd robotruck_school_1128
code .
```

VS Code 检测到 `.devcontainer/` → 点 **Reopen in Container** → 自动构建（15 分钟），和你的环境**完全一致**。

> 这就是"基础设施即代码"（Infrastructure as Code）——环境配置不再靠口口相传的 Word 文档，而是靠可以版本控制的代码文件。

### 8.4 方案三：推送到镜像仓库（团队规模较大时）

如果公司有 Docker Registry（如 Harbor、阿里云容器镜像服务）：

```bash
# 打标签
docker tag robotruck-dynamics-env:latest your-registry.com/robotruck/ros-noetic-ml:latest

# 推送
docker push your-registry.com/robotruck/ros-noetic-ml:latest
```

团队成员直接：

```bash
docker pull your-registry.com/robotruck/ros-noetic-ml:latest
```

然后修改 `.devcontainer/devcontainer.json`，把 `build` 替换为 `image`：

```json
{
    "image": "your-registry.com/robotruck/ros-noetic-ml:latest",
    ...
}
```

这样 VS Code 直接拉取预构建镜像，省去本地构建时间。

### 8.5 迁移检查清单

换新电脑时，确保以下内容全部迁移：

| 项目 | 位置 | 大小 | 方式 |
|------|------|------|------|
| 项目代码 | `~/robotruck_school_1128/` | ~440MB | Git 或拷贝 |
| Docker 镜像 | `docker save` 导出 | ~7GB | U盘/网络 |
| VS Code 插件 | WSL + Dev Containers | — | 重装（见第三步） |
| Docker Desktop | — | — | 重装（见第二步） |

> **最精简的方式**：项目代码放 Git + `.devcontainer/` 目录即可。新电脑只需装 Docker Desktop + VS Code + 两个插件，然后 clone → Reopen in Container，全自动。

---

## 常见问题

### Q: Docker Desktop 启动报错 "WSL 2 installation is incomplete"

在 Windows PowerShell (管理员) 中执行：

```powershell
wsl --update
wsl --shutdown
```

然后重启 Docker Desktop。

### Q: docker pull 或 apt-get 很慢

中国用户建议配置镜像加速器。在 Docker Desktop → Settings → Docker Engine 中添加：

```json
{
  "registry-mirrors": \[
    "https://mirror.ccs.tencentyun.com"
  ]
}
```

然后 Apply \& Restart。

### Q: pytorch 安装失败

ROS Noetic 使用 Python 3.8，PyTorch 2.x 不支持。如果遇到版本问题，在 Dockerfile 中指定版本：

```dockerfile
RUN pip3 install torch==1.13.1 torchvision==0.14.1 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q: 容器里改了文件但 Windows 里看不到

确认文件是保存在 `/workspace` 下（不是容器内的临时目录 `/tmp`）。只有 `/workspace` 是映射到宿主机的。

\---

## 环境清单

|组件|版本|用途|
|-|-|-|
|ROS|Noetic|机器人操作系统|
|Python|3.8|脚本语言|
|PyTorch|2.4.1|深度学习框架|
|torchvision|0.19.1|视觉模型库|
|NumPy|1.x|数值计算|
|SciPy|1.10.x|科学计算|
|Pandas|2.0.x|数据处理|
|Matplotlib|3.x|数据可视化|
|Optuna|4.5.x|超参数优化|
|catkin-tools|latest|ROS 构建工具|
|bagpy|latest|ROS bag 解析|
|rospkg|latest|ROS 包管理|

\---

*文档版本 1.0 — 2026-06-24*

