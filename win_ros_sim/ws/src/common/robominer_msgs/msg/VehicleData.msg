# 对不同车，vcu反馈的转向、油门、刹车、档位可能各不相同，在这里全部转为我们的定义
std_msgs/Header header

#车辆状态
VehicleStatus vehicle_status
#工作状态 0循环作业 1单次作业
uint8 work_state

#自动驾驶状态
uint8 autodrive_state
#铲运调度状态
uint8 shovel_task_state
#目的地
string goal

#坐标
float64 longitude
float64 latitude
float64 altitude
#东北天坐标
float64 enu_x
float64 enu_y
float64 enu_z
#计算车辆四个角的东北天坐标
float64[] enu_lf
float64[] enu_lr
float64[] enu_rf
float64[] enu_rr
#加速度
float64 acceleration_x
float64 acceleration_y
float64 acceleration_z
#速度 由定位模块给出的
float64 speed
#姿态角
float64 yaw
float64 pitch
float64 roll
# 定位数据是否有效
bool location_valid

#感知是否有效
bool perception_valid

#规划状态
uint8 planner_status


#驾驶模式
uint8 drive_mode
#方向盘
float64 steering
#油门值
float64 throttle
#刹车值
float64 brake
#档位
uint8 gear
#手刹
Epb epb_state
#车厢角度
float64 lift_angle
#横向偏差
string cte


###增加目标相关数据
float64 target_speed
Gear target_gear
float64 target_steering
Epb target_epb
Pto target_pto
#目标缓行器状态
uint8 retarder_state




#发动机转速
float64 engine_speed
#
float64 engine_torque
float64 steering_wheel_torque
float64 steering_wheel_speed
float64 steering_motor_torque
uint8 brake_work_mode
uint8 brake_over_heat
uint8 brake_emergency_state
float64 deceleration
float64 pressure_fa
float64 pressure_ra
float64 wheel_speed_rr
float64 wheel_speed_rl
float64 wheel_speed_fr
float64 wheel_speed_fl
float64 break_pressure_r2r
float64 break_pressure_r2l
float64 break_pressure_r1r
float64 break_pressure_r1l
float64 break_pressure_fr
float64 break_pressure_fl
float64 odometer
float64 oil
float64 engine_oil_temperature
float64 engine_fuel_temperature
float64 engine_cool_water_temperature
float64 total_fuel_consumption

#2021-12-14 本版本减少修改，增加vcu的完整数据
VcuData vcu_data
#2023-01-10 增加完整的localization的完整数据
LocalizationData localization_data
