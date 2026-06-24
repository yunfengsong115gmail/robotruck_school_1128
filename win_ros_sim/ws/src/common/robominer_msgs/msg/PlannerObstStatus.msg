#obstacle state
uint8 OE_STATUS_IDLE=0
uint8 OE_OBSTACLING =1
uint8 OE_OBSTACLE_KEEP=2
uint8 OE_OBSTACLED  =3
uint8 OE_STATUS_MAX =4

uint8 obstacleStatus
std_msgs/Time obstacleUptime
#global path waypoint index
# 障碍触碰路径点信息索引
uint32 globalIndex
# 障碍停止线路径点信息
robominer_msgs/Waypoint stop_waypoint
float64 fs
robominer_msgs/Gear gear
uint32 pixelNumber
robominer_msgs/PixelPoint3D[] pixel

# 停止线类型编号
int32 id