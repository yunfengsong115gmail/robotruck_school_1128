std_msgs/Header header

DriveMode target_drive_mode
Gear target_gear
GearData target_gear_data #默认不填充，需要手动锁档时填，表示希望变速箱不要超过的最高档位
float64 target_throttle #[0.0,100.0]
float64 target_brake #[0.0,100.0]
float64 target_steering #[-50.0,50.0]度，左正右负

#float64 target_speed #always positive,m/s
#bool speed_mode
