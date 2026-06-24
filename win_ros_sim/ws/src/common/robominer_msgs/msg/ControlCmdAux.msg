std_msgs/Header header

Pto target_pto
uint8 target_pto_warn
uint8 target_right_light
uint8 target_left_light
uint8 target_inside_light
uint8 target_pos_light
uint8 target_low_beam
uint8 target_high_beam
uint8 target_back_light
uint8 target_brake_light
uint8 target_horn
uint8 target_wiper_clean
uint8 target_wiper_slow
uint8 target_wiper_fast
uint8 target_airconditioner#开空调
uint8 target_emergency_flashers #双闪

Epb target_epb

uint8 target_engine #发动机状态 0熄火1点火

#考虑增加 排气制动、缓行器、差速锁
uint8 target_lsd #差速锁/轴差电磁阀 0：关闭；1：开启

#2023.2.16 增加干湿路控制使能  0:未使能   1:使能
uint8 target_drywetroadctl


Brim target_brim 
#BrimState target_brim  #0:停止； 1：下降； 2：上升；

# 缓行器控制 
RatarderState target_ratarder_state