std_msgs/Header header                  # 消息构建时间戳
robominer_msgs/DetectedObjectArray objs # 感知发布的障碍物李列表
robominer_msgs/GroundPlanes grounds     # 感知发布的地面方程列表
bool ground_replace                     # 使用pose替换地面方程
robominer_msgs/OtherDevices otherDevices# 其他设备列表
robominer_msgs/LocalizationData pose    # 车辆实时位置
robominer_msgs/NavGridOfDoubles gridmap # 地面方程与障碍物计算的2.5D珊格图
uint16[] relations                      # 高度与障碍物列表索引对应表