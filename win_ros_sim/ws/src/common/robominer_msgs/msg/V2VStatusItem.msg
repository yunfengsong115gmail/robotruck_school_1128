string device_sn
std_msgs/Time stamp
bool high_priority                         # 是否高优先级
robominer_msgs/V2VProcess proc             # V2V状态推测
robominer_msgs/Waypoint near               # 进入交点
robominer_msgs/V2VDistribution near_at     # 近处交点所处位置
robominer_msgs/Waypoint far                # 退出交点
robominer_msgs/V2VDistribution far_at      # 近处交点所处位置
robominer_msgs/PlanningStatus plan_status  # 规划给出的状态