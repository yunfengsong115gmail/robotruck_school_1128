float64 minHeight
float64 maxHeight
float64 before_fs
float64 fs_limit        #区域纵差限制
float64 fd_limit        #区域横差限制
float64 speed_limit     #速度限制，区域速度小于该值
float64 ignore_height   #忽略小于该值的障碍物
float64 stop_height     #存在高于该值障碍物则区域无法通行
bool dump_flag          #卸料过程使能
bool load_flag          #装料过程使能
bool replan             #本区域是否允许绕障，根据type类型决定是否生效
GearData gear_data      #锁档限制
bool limit_up           #是否上坡限速
bool limit_down         #是否下坡限速
string name
string type
string id
string version
robominer_msgs/Waypoint[] gpsPoints