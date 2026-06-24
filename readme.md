
# 伯镭无人驾驶项目ROS目录
.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── common
│   ├── robominer_msgs
│   └── simulation
│       └── mock_truck
└── estimation
    ├── CMakeLists.txt
    ├── launch
    │   └── estimation_launch.launch
    ├── msg
    │   ├── AdhesionEst.msg
    │   ├── MassSlopeEst.msg
    │   └── VehParamEst.msg
    ├── package.xml
    └── src
        ├── adhesion_est.py
        ├── mass_slope_est.py
        └── veh_param_est.py

其中，
1. /common/mock_truck/src目录下包含MPC控制脚本 MPC_controller.py
2. estimation package包含自定义消息msg，以及3个节点脚本：路面附着系数估计、质量+坡度估计、车辆动力学参数估计 


# requirements
1. Ubuntu20.04
2. ROS noetic
3. matlab_2020a or later versions
4. trucksim_2019.0

# Trucksim--Matlab/Simulink 启动流程
1. 将文件夹win_sim下truck_mpc.cpar配置导入trucksim_2019.0中
2. trucksim配置truck_mpc_8dof_2020a.slx文件路径后send to simulink
3. simulink可自行设计参考路径并运行仿真

# Matlab/Simulink--ROS 双机通信配置流程
1. 通过网线连接两台设备对应PC Ubuntu 与 PC Windows处于同一局域网，可互相ping通
2. PC Ubuntu下 sudo gedit /etc/hosts 添加PC Win IP+host
3. PC Ubuntu下 sudo gedit ~/.bashrc 添加ROS IP
4. PC Windows下 matlab rosmsggen工具导入自定义msg type
5. PC Windows下 matlab rosshutdown; setenv('ROS_MASTER_URI', 'http://PCU_IP:11311')
6. PC Ubuntu下 source ~/.bashrc; roscore
7. PC Windows下 rosinit

# Trucksim--Matlab/Simulink--ROS 启动流程
1. PC Ubuntu cd ./win_ros_sim/ws后 catkin_make
2. PC Windows下 配置win_ros_sim文件夹下的.cpar以及.slx，建立trucksim-simulink联合仿真
3. PC Windows下 setenv后配合PCu的roscore进行rosinit, 启动simulink仿真程序
4. PC Ubuntu下 rostopic echo /vcu/vcu_data; rostopic echo /localization/localization_data 查看simulink发布的topic
5. PC Ubuntu下 rosrun estimation veh_param_est.py 启动车辆动力学参数估计节点
6. PC Ubuntu下 rosrun mock_truck MPC_controller.py 启动MPC控制节点

