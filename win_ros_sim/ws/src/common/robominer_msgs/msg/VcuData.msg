#信号值*精度值 + 偏移量 = 物理值

std_msgs/Header header

DriveStatus drive_status

float64 fuel_level #剩余油量，[0.0,100.0]
float64 speed #车速，m/s

float64 totalWorkingTime #运行总时间, h
float64 odograph #总里程，km
float64 trip_dist #小计里程，km
float64 total_fuel_consumption #总油耗，L
float64 trip_fuel_consumption #小计油耗，L
float64 instant_fuel_consumption #瞬时油耗，km/L
float64 average_fuel_consumption #平均油耗，km/L
float64 fuel_rate #燃油率 L/h
float64 engine_oil_temperature #机油温度°
float64 engine_fuel_temperature #燃油温度°
float64 engine_cool_water_temperature #冷却水温度°
float64 engine_oil_pressure #机油压力kPa --------------
float64 engine_oil_level #机油液位，[0.0,100.0] -----------
float64 wheel_speed_fr #右前轮速，m/s，原数据km/h
float64 wheel_speed_fl #左前轮速，m/s，原数据km/h
float64 wheel_speed_rr #右后轮速，m/s，原数据km/h
float64 wheel_speed_rl #左后轮速，m/s，原数据km/h

#油门
float64 throttle_pos #油门踏板开度，[0.0,100.0]
float64 engine_torque  #发动机扭矩 V3.5从%变为Nm
float64 engine_speed #发动机转速，rpm
uint8 engine_fault #0表示无故障   1表示二级故障  4表示一级故障  5一二级故障共存

#制动
Epb epb
uint8 epb_fault #0表示无故障
float64 brake_pos #刹车踏板开度，[0.0,100.0]
uint8 brake_fault #0表示无故障
float64 deceleration #制动减速度, m/s^2
float64 latrl_aclrtn #横向加速度, m/s^2
uint8 brake_work_mode #制动系统工作模式，参考潍柴协议
uint8 brake_over_heat #制动系统高温报警，0表示无报警
uint8 brake_emergency_state #行车紧急制动状态
uint8 epb_emergency_state #驻车紧急制动状态
float64 break_pressure_fr #右前制动气压 kPa
float64 break_pressure_fl #左前制动气压 kPa
float64 break_pressure_r1r #后1轴右制动气压 kPa
float64 break_pressure_r1l #后1轴左制动气压 kPa
float64 break_pressure_r2r #后2轴右制动气压 kPa
float64 break_pressure_r2l #后2轴左制动气压 kPa
float64 pressure_fa #前桥气压 kPa
float64 pressure_ra #后桥气压 kPa
float64 epb_pressure #epb气压 kPa

#转向
float64 steering_angle #前轮转角(°)，左正右负(原定义左负右正，转为左正右负发出)，范围：[-50.0, 50.0]
float64 steering_wheel_torque #方向盘扭矩，Nm
float64 steering_wheel_speed  #方向盘角速度，°/s
uint8 steering_fault  #0表示无故障   1表示二级故障  4表示一级故障  5一二级故障共存
float64 steering_motor_torque 
float64 steering_angle_fr #右前轮转角(°)，左正右负

#换挡
Gear gear
uint8 transmission_fault  #0表示无故障

#举升
float64 lift_angle #货箱相对于车体的举升角度，°
uint8 lift_fault  #0表示无故障
float64 lift_angle_original  #货箱传感器的原始角度值，即bwm425_angle

#附件系统
KeyPowerState key_power_state #钥匙状态，0：OFF; 1:ACC; 2:ON; 3:START
uint8 acc_state
uint8 on_state
uint8 start_state
PtoState pto_state         #取力器状态。0：未挂上；1：挂上；2：error
VehicleError vehicle_error     # 0:无故障；1：轻微故障；2：一般故障；3：严重故障；4：致命故障
VcuSysErrLevel vcu_sys_errLevel #VCU系统故障等级
VcuErrCode vcu_err_code #VCU故障码
TyreState[] tyre_states #轮胎状态
TpmsBcan[] tpms_bcans
uint8 emrg_btn_state     #急停按钮状态。0：未按下；1：按下
uint8 lsd_state         #差速器/轴差电磁阀状态。0：关闭；1：开启
uint8 retarder_state    #缓速器状态
uint8 left_light_state
uint8 right_light_state
uint8 low_beam_state       
uint8 high_beam_state      
uint8 int_light_state
uint8 pos_light_state      
uint8 back_light_state     
uint8 brake_light_state    
uint8 front_fog_light_state     
uint8 lift_warn_state       
uint8 horn_state       
uint8 ac_switch_state     #压缩机工作状态
uint8 airconditioner_state    #空调工作状态 
uint8 smokealarmsw_sta        #烟雾报警开关信号
uint8 oiltemperature_brake     #回油温度
uint8 eps_presesure            #转向蓄能器压力 bar 0-255，正常范围80-150
uint8 steer_pressure           #转向工作压力 bar 0-255
uint8 oil_filter_presesur     #液压油滤压力信号 bar
uint8 oil_level             #液压油箱液位信号
uint8 drywetroad_state   #干湿路开关信号
float64 transmission_oil_temperature  #变速箱油温 -40-300℃
uint8 loader_weight       #货物重量
uint8 xbr_state     #XBR系统状态
uint8 lift_pressure_up   #举升上升压力 bar
uint8 lift_pressure_down   #举升下降压力 bar 
uint8 wiper_slow_state    #雨刮快档
uint8 wiper_fast_state    #雨刮慢档
uint8 wiper_clean_state   #雨刮洗涤
uint8 aeb_state
# 电车专属
EpowerState epower_state  #0:预留；1：低压上电；2：附件高压；3：驱动高压；4：ready 
float64 soc #剩余电量，[0.0,100.0]
float64 high_battery_current #高压电池电流
float64 high_battery_voltage #高压电池电压
float64 battery_current #蓄电池电流
float64 battery_voltage #蓄电池电压 24v电池 小电池
float64 motor_speed #电机转速

# 宏威电车
Brim brim_state  #0:停止； 1：下降； 2：上升； 3：故障
BrimPosition brim_position #0:中间位置； 1：下极限位置； 2：上极限位置； 3：故障
# 徐工电车
float64 methanol_level #甲醇液位

#####

GearData gear_data #当前的具体档位，可为空
uint8 reverse_inhibition  #倒挡抑制
uint8 main_neg_relay_st  #BMS主负继电器
uint8 tcu_mode  #变速箱换挡模式
uint8 brake_demand_vcu  #VCU主动制动请求
uint8 emergency_steering_ctl  #应急转向控制

float64 motor_temperature #电机温度

float64 motor1_motspeed #电机1转速
float64 motor1_torque #电机1扭矩
float64 motor1_mottemp #电机1温度
float64 motor1_invtemp #电机控制器1温度
float64 motor1_current #电机1母线电流
float64 motor1_hvdcvolt #电机1母线电压

float64 motor2_motspeed #电机2转速
float64 motor2_torque #电机2扭矩
float64 motor2_mottemp #电机2温度
float64 motor2_invtemp #电机控制器2温度
float64 motor2_current #电机2母线电流
float64 motor2_hvdcvolt #电机2母线电压

float64 motor3_motspeed #电机3转速
float64 motor3_torque #电机3扭矩
float64 motor3_mottemp #电机3温度
float64 motor3_invtemp #电机控制器3温度
float64 motor3_current #电机3母线电流
float64 motor3_hvdcvolt #电机3母线电压

float64 motor4_motspeed #电机4转速
float64 motor4_torque #电机4扭矩
float64 motor4_mottemp #电机4温度
float64 motor4_invtemp #电机控制器4温度
float64 motor4_current #电机4母线电流
float64 motor4_hvdcvolt #电机4母线电压

float64 soh #电池SOH

float64 tot_chgenergy #累计充电电量
float64 single_chgenergy #单次充电电量
float64 total_charge_consumption #累计放电电量


uint8 chrg_status #充电状态
uint8 high_battery_max_temperature #高压电池最高温度
uint8 high_battery_min_temperature #高压电池最低温度
uint8 high_battery_ave_temperature #高压电池平均温度


uint8 pdc_error_level #PDC故障等级
uint8 vehicle_error_level #整车故障类型
uint8 tms_error_level #变速箱故障等级
uint8 tcu_error_level #TCU故障等级
uint8 bms_error_level #BMS故障等级
uint8 bty_error_level #电池故障等级
uint8 mt1_error_level #电机1故障等级
uint8 mt2_error_level #电机2故障等级
uint8 mt3_error_level #电机3故障等级
uint8 mt4_error_level #电机4故障等级
uint8 acs_error_level #电附件故障等级
uint8 bss_error_level #换电系统故障等级
uint8 els_error_level #其他故障等级

uint8 pdc_error_code #PDC故障码
uint8 vehicle_error_code #整车故障码
uint8 tms_error_code #变速箱故障码
uint8 tcu_error_code#TCU故障码   
uint8 bms_error_code #BMS故障码
uint8 bty_error_code #电池故障码
uint8 mt1_error_code #电机1故障码
uint8 mt2_error_code #电机2故障码
uint8 mt3_error_code #电机3故障码
uint8 mt4_error_code #电机4故障码
uint8 acs_error_code #电附件故障码
uint8 bss_error_code #换电系统故障码
uint8 els_error_code #其他故障码


float64 left_front_wheel_distance #左前轮偏移距离
float64 right_front_wheel_distance #右前轮偏移距离

uint8 up_shiftpermissionst #是否允许升档
uint8 down_shiftpermissionst #是否允许降档

uint8 brake_pad_warning_fl #左前刹车片磨损报警
uint8 brake_pad_warning_fr #右前刹车片磨损报警
uint8 brake_pad_warning_ml #左中刹车片磨损报警
uint8 brake_pad_warning_mr #右中刹车片磨损报警
uint8 brake_pad_warning_rl #左后刹车片磨损报警
uint8 brake_pad_warning_rr #右后刹车片磨损报警
uint8 side_wiper_stop_pstn #侧雨刮停位
uint8 filter_clogged_st #油滤堵塞状态

#软件版本
uint8 prjct_num
uint8 cal_ver
uint8 sw_ver
