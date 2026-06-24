std_msgs/Header header

bool is_valid       #可用标志

float64 latitude        #经纬高
float64 longitude
float64 altitude

float64 x #in ENU, meters     东北天
float64 y
float64 z

float64 speed #m/s   

float64 vx #in ENU , m/s   东北天速度
float64 vy
float64 vz

float64 ax #in ENU, m/s^2    加速度 
float64 ay
float64 az

float64 roll #in ENU, degrees  姿态角
float64 pitch
float64 yaw

float64 wx #in ENU, degrees/s   角速度
float64 wy
float64 wz

uint8 solnsvs         #搜星数
uint8 solnsvs2         #搜星数

uint8 age
uint8 gga_age
LocalizationStatus rtk_status
