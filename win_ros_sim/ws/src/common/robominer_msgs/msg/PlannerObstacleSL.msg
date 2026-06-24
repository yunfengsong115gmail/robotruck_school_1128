std_msgs/Header header                  # 消息构建时间戳
int32 idm                               # 碰撞位置局部路径索引
robominer_msgs/Waypoint wp              # 碰撞位置的路径点信息
geometry_msgs/Polygon robot             # 车辆外形，外框，还是内框
robominer_msgs/DetectedObject object    # 障碍物
robominer_msgs/PlannerCommArea area     # 功能区域
geometry_msgs/Point pos                 # 第一个碰撞点
float64 pos_height                      # 第一个碰撞点对地高度
