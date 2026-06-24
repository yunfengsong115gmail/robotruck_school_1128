std_msgs/Header header
string goal_id  #目标点唯一标识，坐标变化时id同步变化

string start_area_id  #起点区域资源id 2024年09月11日废弃
string goal_area_id #目标点区域资源id 2024年09月11日废弃
string goal_point_id #目标点资源id 2024年09月11日废弃
string path_filename #主干道轨迹文件的完整路径与文件名 2024年09月11日废弃

float64 goal_x #目标点坐标. ENU
float64 goal_y
float64 goal_z
float64 goal_yaw  #ENU, degrees
# goal_type 2024年08月16日废弃，替换为任务类型
robominer_msgs/WaypointType goal_type
robominer_msgs/TaskType task_type

# 2024年9月11日添加 goal_gear 目标点正进倒进标识
robominer_msgs/Gear goal_gear

# 2024年9月11日添加 front_fs back_fs
# 目标点停靠准确程度
# front_fs --- goal --- back_fs
# [front_fs, back_fs]范围内执行堵转停车或者直接停车，goal为目标点
# 自身携带正负负号，目标点朝向的前方为正，后方为负
float64 goal_front_fs
float64 goal_back_fs