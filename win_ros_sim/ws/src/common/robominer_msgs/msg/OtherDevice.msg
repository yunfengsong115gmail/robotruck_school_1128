std_msgs/Header header
#其他设备的位置和朝向
string device_id     # mqtt设备号
string device_sn     # 设备序列号
float64 x            # 设备坐标
float64 y            # 设备坐标
float64 z            # 设备坐标
float64 yaw          # 设备朝向
float64 speed        # 设备速度
DeviceType device_type
DeviceBodyCoordinates body_coordinates
#增加挖机的状态
DiggerStatus status
LocalPlan local_plan    # 局部轨迹
uint8 drive_mode
