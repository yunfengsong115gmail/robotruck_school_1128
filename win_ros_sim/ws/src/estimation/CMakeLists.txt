cmake_minimum_required(VERSION 3.0.2)
project(estimation)

## 依赖包
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  message_filters
  robominer_msgs
)

## 生成自定义消息
add_message_files(
  FILES
  AdhesionEst.msg
  MassSlopeEst.msg
  VehParamEst.msg
)

## 生成消息依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Catkin配置
catkin_package(
  INCLUDE_DIRS
  LIBRARIES estimation
  CATKIN_DEPENDS rospy std_msgs message_runtime message_filters robominer_msgs
  DEPENDS
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Python
catkin_install_python(PROGRAMS
  src/adhesion_est.py
  src/mass_slope_est.py 
  src/veh_param_est.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装launch文件
install(DIRECTORY launch
  /launch/estimation_launch.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装msg文件
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
