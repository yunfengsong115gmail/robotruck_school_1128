# 三轴矿卡路面附着系数估计结果
std_msgs/Header header                  # 时间戳与坐标系

float64 mu_fl                           # 左前轮附着系数 [0.0, 1.0]
float64 mu_fr                           # 右前轮附着系数 [0.0, 1.0]
float64 mu_ml                           # 左中轮附着系数 [0.0, 1.0]
float64 mu_mr                           # 右中轮附着系数 [0.0, 1.0]
float64 mu_rl                           # 左后轮附着系数 [0.0, 1.0]
float64 mu_rr                           # 右后轮附着系数 [0.0, 1.0]
float64 speed_mps                       # 矿车当前速度（m/s）
