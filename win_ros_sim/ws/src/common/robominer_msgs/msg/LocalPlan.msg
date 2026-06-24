std_msgs/Header header
robominer_msgs/TaskType task_type
robominer_msgs/Waypoints waypoints
robominer_msgs/PlanningStatus status
int32 nearest_point_index #离当前最近的点
float64 fs
float64 fd #左正右负
float64 total_fs #全局轨迹的总fs
int32 valid_point_num #局部轨迹中有效的点数
float64 ref_yaw
float64 yaw_deviation