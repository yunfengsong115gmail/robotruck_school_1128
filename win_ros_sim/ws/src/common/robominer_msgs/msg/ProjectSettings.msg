std_msgs/Header header

float64 origin_longitude #中心点经度
float64 origin_latitude #中心点纬度
float64 origin_altitude #中心点高度
float64 forward_speed #前进速度 0.0表示车端设置
float64 recede_speed #倒车速度 0.0表示车端设置

uint8 main_road_obstacles #主干道是否绕障 0车端设置 1绕障 2不绕障
uint8 ld_area_obstacles #装卸料区是否绕障 0车端设置 1绕障 2不绕障

uint8 foggy_mode #大雪大雾模式 0关闭 1打开

uint8 drywetroad_state #

#v1.0 2022/10/24
#定义项目设置的参数中心点经纬高、前进速度及后退速度
#v1.1 2022/10/26
#增加主干道是否绕障、装卸区是否绕障
#v1.2 2023/02/20
#增加大雪/大雾模式 0: 非大雪/大雾模式  1: 大雪/大雾模式
#v1.3 2023/04/27
#增加干路/湿路模式
