int64   aSelectPoint
int64   bSelectPoint
string id
string name
string version
string[] next_ids # 后序路径列表 基于空间相近进行组织
string[] prev_ids # 前序路径列表
string[] next_joints # 后序拼接路径 基于逻辑进行组织，处理拼接路径
string[] prev_joints # 前序拼接路径
robominer_msgs/Waypoint[] waypoints