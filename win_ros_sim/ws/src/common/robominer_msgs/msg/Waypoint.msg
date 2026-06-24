float64 x
float64 y
float64 z
float64 latitude
float64 longitude
float64 altitude #录制轨迹时必须包含以上字段

# 以下字段可在录制时保存，也可自行计算
float64 yaw
float64 pitch
float64 speed #始终为正值
float64 steering #左正右负
float64 fs #accumulated distance from beginning of the path
float64 kappa #curvature，转弯半径的倒数 注意：左正右负
float64 speed_limit #录制轨迹时记录的速度存在此处. 程序内部发布global path时会根据限速、曲率、坡度降低该值
Gear gear
GearData gear_data #实际档位值，控制根据该信息进行锁档,default=0表示不锁档
WaypointType waypoint_type
uint32 index